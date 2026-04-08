// SafeBoxForest v5 — Benchmark: Full Pipeline (Endpoint → Link Envelope)
// Compares volume, voxel count, and speed across pipeline combinations.
// Cold = fresh compute (no prior state).
// Hot  = IFK incremental (reuse FKState), CritSample hull-accumulated.
// Multiple interval widths, deterministic seed.

#include <sbf/core/types.h>
#include <sbf/core/robot.h>
#include <sbf/core/fk_state.h>
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/ifk_source.h>
#include <sbf/envelope/crit_source.h>
#include <sbf/envelope/link_iaabb.h>
#include <sbf/envelope/link_grid.h>
#include <sbf/envelope/envelope_type.h>
#include <sbf/voxel/voxel_grid.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <random>
#include <string>
#include <vector>

using namespace sbf;
using Clock = std::chrono::high_resolution_clock;

// ═══════════════════════════════════════════════════════════════════════════
//  Helpers
// ═══════════════════════════════════════════════════════════════════════════

static double sum_link_volume(const float* link_aabbs, int n_act) {
    double vol = 0.0;
    for (int i = 0; i < n_act; ++i) {
        const float* a = link_aabbs + i * 6;
        double dx = std::max(0.0, double(a[3] - a[0]));
        double dy = std::max(0.0, double(a[4] - a[1]));
        double dz = std::max(0.0, double(a[5] - a[2]));
        vol += dx * dy * dz;
    }
    return vol;
}

static std::vector<Interval> random_intervals(
    const Robot& robot, std::mt19937& rng, double half_width)
{
    int n = robot.n_joints();
    const auto& lim = robot.joint_limits().limits;
    std::vector<Interval> ivs(n);
    for (int j = 0; j < n; ++j) {
        double lo = lim[j].lo + half_width;
        double hi = lim[j].hi - half_width;
        if (lo > hi) lo = hi = (lim[j].lo + lim[j].hi) * 0.5;
        std::uniform_real_distribution<double> dist(lo, hi);
        double c = dist(rng);
        ivs[j] = Interval(c - half_width, c + half_width);
    }
    return ivs;
}

// Perturb one dimension slightly to simulate incremental change for hot mode
static std::vector<Interval> perturb_intervals(
    const std::vector<Interval>& ivs,
    const Robot& robot,
    std::mt19937& rng,
    int& changed_dim)
{
    int n = robot.n_joints();
    std::uniform_int_distribution<int> dim_dist(0, n - 1);
    changed_dim = dim_dist(rng);

    std::vector<Interval> out = ivs;
    const auto& lim = robot.joint_limits().limits;
    double hw = ivs[changed_dim].width() * 0.5;
    double shift = std::uniform_real_distribution<double>(-hw * 0.1, hw * 0.1)(rng);
    double new_c = ivs[changed_dim].center() + shift;
    double new_lo = std::max(lim[changed_dim].lo, new_c - hw);
    double new_hi = std::min(lim[changed_dim].hi, new_c + hw);
    out[changed_dim] = Interval(new_lo, new_hi);
    return out;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Pipeline configuration
// ═══════════════════════════════════════════════════════════════════════════

struct PipelineConfig {
    const char* name;
    EndpointSource ep_source;
    EnvelopeType   env_type;
    int            n_subdivisions;
    double         voxel_delta;     // 0 = not applicable
    int            n_crit_samples;  // only for CritSample
};

// ═══════════════════════════════════════════════════════════════════════════
//  Pipeline runner result
// ═══════════════════════════════════════════════════════════════════════════

struct PipelineResult {
    double ep_time_us   = 0.0;   // endpoint source time
    double env_time_us  = 0.0;   // link envelope time
    double total_time_us = 0.0;  // total
    double link_volume  = 0.0;   // sum of link AABB volumes
    int    n_voxels     = 0;     // grid voxel count (0 if no grid)
    int    n_act        = 0;
    bool   is_safe      = false;
};

// ═══════════════════════════════════════════════════════════════════════════
//  Cold pipeline run
// ═══════════════════════════════════════════════════════════════════════════

static PipelineResult run_cold(
    const Robot& robot,
    const std::vector<Interval>& ivs,
    const PipelineConfig& pipe,
    uint64_t seed)
{
    PipelineResult res;
    int n_act = robot.n_active_links();
    res.n_act = n_act;

    // ── Endpoint source ──
    auto ep_t0 = Clock::now();
    EndpointIAABBResult ep_res;
    if (pipe.ep_source == EndpointSource::IFK) {
        ep_res = compute_endpoint_iaabb_ifk(robot, ivs);
    } else {
        ep_res = compute_endpoint_iaabb_crit(robot, ivs, pipe.n_crit_samples, seed);
    }
    auto ep_t1 = Clock::now();
    res.ep_time_us = std::chrono::duration<double, std::micro>(ep_t1 - ep_t0).count();
    res.is_safe = ep_res.is_safe;

    // ── Link envelope ──
    EnvelopeTypeConfig env_cfg;
    env_cfg.type = pipe.env_type;
    env_cfg.n_subdivisions = pipe.n_subdivisions;
    env_cfg.grid_config.voxel_delta = pipe.voxel_delta > 0 ? pipe.voxel_delta : 0.05;

    auto env_t0 = Clock::now();
    LinkEnvelope env = compute_link_envelope(
        ep_res.endpoint_iaabbs.data(),
        ep_res.n_active_links,
        robot.active_link_radii(),
        env_cfg);
    auto env_t1 = Clock::now();
    res.env_time_us = std::chrono::duration<double, std::micro>(env_t1 - env_t0).count();

    res.total_time_us = res.ep_time_us + res.env_time_us;
    res.link_volume = sum_link_volume(env.link_iaabbs.data(), n_act);

    if (env.sparse_grid)
        res.n_voxels = env.sparse_grid->count_occupied();

    return res;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Hot pipeline run  (IFK incremental / CritSample hull-accumulate)
// ═══════════════════════════════════════════════════════════════════════════

static PipelineResult run_hot(
    const Robot& robot,
    const std::vector<Interval>& ivs_base,
    const PipelineConfig& pipe,
    uint64_t seed,
    int n_hot_rounds,
    std::mt19937& rng)
{
    PipelineResult res;
    int n_act = robot.n_active_links();
    int ep_len = n_act * 2 * 6;
    res.n_act = n_act;

    double ep_time_sum = 0.0;
    double env_time_sum = 0.0;

    if (pipe.ep_source == EndpointSource::IFK) {
        // Hot IFK: first round full, subsequent rounds incremental
        FKState fk;
        std::vector<float> ep_hull(ep_len);
        // Init to inverted extremes
        for (int i = 0; i < ep_len; i += 6) {
            ep_hull[i]   = ep_hull[i+1] = ep_hull[i+2] =  std::numeric_limits<float>::max();
            ep_hull[i+3] = ep_hull[i+4] = ep_hull[i+5] = -std::numeric_limits<float>::max();
        }

        auto ivs = ivs_base;
        for (int round = 0; round < n_hot_rounds; ++round) {
            int changed_dim = -1;
            if (round > 0) {
                ivs = perturb_intervals(ivs, robot, rng, changed_dim);
            }

            auto t0 = Clock::now();
            EndpointIAABBResult ep_res;
            if (round == 0) {
                ep_res = compute_endpoint_iaabb_ifk(robot, ivs);
            } else {
                ep_res = compute_endpoint_iaabb_ifk(robot, ivs, &fk, changed_dim);
            }
            auto t1 = Clock::now();
            ep_time_sum += std::chrono::duration<double, std::micro>(t1 - t0).count();

            fk = ep_res.fk_state;
            hull_endpoint_iaabbs(ep_hull.data(), ep_res.endpoint_iaabbs.data(), n_act * 2);
            res.is_safe = ep_res.is_safe; // IFK always safe
        }

        // Final envelope from hulled endpoints
        EnvelopeTypeConfig env_cfg;
        env_cfg.type = pipe.env_type;
        env_cfg.n_subdivisions = pipe.n_subdivisions;
        env_cfg.grid_config.voxel_delta = pipe.voxel_delta > 0 ? pipe.voxel_delta : 0.05;

        auto et0 = Clock::now();
        LinkEnvelope env = compute_link_envelope(
            ep_hull.data(), n_act, robot.active_link_radii(), env_cfg);
        auto et1 = Clock::now();
        env_time_sum = std::chrono::duration<double, std::micro>(et1 - et0).count();

        res.link_volume = sum_link_volume(env.link_iaabbs.data(), n_act);
        if (env.sparse_grid) res.n_voxels = env.sparse_grid->count_occupied();

    } else {
        // Hot CritSample: hull-accumulate multiple rounds
        std::vector<float> ep_hull(ep_len);
        for (int i = 0; i < ep_len; i += 6) {
            ep_hull[i]   = ep_hull[i+1] = ep_hull[i+2] =  std::numeric_limits<float>::max();
            ep_hull[i+3] = ep_hull[i+4] = ep_hull[i+5] = -std::numeric_limits<float>::max();
        }

        for (int round = 0; round < n_hot_rounds; ++round) {
            auto t0 = Clock::now();
            auto ep_res = compute_endpoint_iaabb_crit(
                robot, ivs_base, pipe.n_crit_samples, seed + round);
            auto t1 = Clock::now();
            ep_time_sum += std::chrono::duration<double, std::micro>(t1 - t0).count();

            hull_endpoint_iaabbs(ep_hull.data(), ep_res.endpoint_iaabbs.data(), n_act * 2);
            res.is_safe = false;
        }

        EnvelopeTypeConfig env_cfg;
        env_cfg.type = pipe.env_type;
        env_cfg.n_subdivisions = pipe.n_subdivisions;
        env_cfg.grid_config.voxel_delta = pipe.voxel_delta > 0 ? pipe.voxel_delta : 0.05;

        auto et0 = Clock::now();
        LinkEnvelope env = compute_link_envelope(
            ep_hull.data(), n_act, robot.active_link_radii(), env_cfg);
        auto et1 = Clock::now();
        env_time_sum = std::chrono::duration<double, std::micro>(et1 - et0).count();

        res.link_volume = sum_link_volume(env.link_iaabbs.data(), n_act);
        if (env.sparse_grid) res.n_voxels = env.sparse_grid->count_occupied();
    }

    res.ep_time_us  = ep_time_sum / n_hot_rounds;
    res.env_time_us = env_time_sum;
    res.total_time_us = res.ep_time_us + res.env_time_us;

    return res;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Table printing
// ═══════════════════════════════════════════════════════════════════════════

static void print_header() {
    std::printf("  %-28s %-5s %9s %9s %9s %11s %8s %4s\n",
                "Pipeline", "Mode", "EP(us)", "Env(us)", "Tot(us)",
                "LinkVol", "Voxels", "Safe");
    std::printf("  %-28s %-5s %9s %9s %9s %11s %8s %4s\n",
                "----------------------------", "-----",
                "---------", "---------", "---------",
                "-----------", "--------", "----");
}

static void print_row(const PipelineConfig& pipe, const char* mode,
                      const PipelineResult& r) {
    std::printf("  %-28s %-5s %9.1f %9.1f %9.1f %11.6f %8d %4s\n",
                pipe.name, mode,
                r.ep_time_us, r.env_time_us, r.total_time_us,
                r.link_volume, r.n_voxels,
                r.is_safe ? "Y" : "N");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Benchmark one interval configuration
// ═══════════════════════════════════════════════════════════════════════════

static void bench_interval(
    const Robot& robot,
    const std::vector<Interval>& ivs,
    const PipelineConfig* pipelines, int n_pipes,
    const char* title,
    uint64_t seed,
    int n_cold_repeats,
    int n_hot_rounds)
{
    std::printf("\n=== %s ===\n", title);
    print_header();

    for (int pi = 0; pi < n_pipes; ++pi) {
        auto& pipe = pipelines[pi];

        // ── Cold: average over repeats ──
        double cold_ep_sum = 0, cold_env_sum = 0, cold_tot_sum = 0;
        PipelineResult cold_last;
        for (int r = 0; r < n_cold_repeats; ++r) {
            auto res = run_cold(robot, ivs, pipe, seed);
            cold_ep_sum  += res.ep_time_us;
            cold_env_sum += res.env_time_us;
            cold_tot_sum += res.total_time_us;
            cold_last = res;
        }
        PipelineResult cold_avg = cold_last;
        cold_avg.ep_time_us  = cold_ep_sum / n_cold_repeats;
        cold_avg.env_time_us = cold_env_sum / n_cold_repeats;
        cold_avg.total_time_us = cold_tot_sum / n_cold_repeats;
        print_row(pipe, "cold", cold_avg);

        // ── Hot ──
        std::mt19937 hot_rng(seed);
        auto hot_res = run_hot(robot, ivs, pipe, seed, n_hot_rounds, hot_rng);
        print_row(pipe, "hot", hot_res);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Main
// ═══════════════════════════════════════════════════════════════════════════

int main() {
    std::printf("SafeBoxForest v5 — Full Pipeline Benchmark (cold/hot)\n");
    std::printf("=====================================================\n");

    constexpr uint64_t SEED           = 42;
    constexpr int      N_COLD_REPEATS = 30;
    constexpr int      N_HOT_ROUNDS   = 10;
    constexpr int      N_RANDOM       = 3;  // random intervals per width

    // ── Pipeline definitions ─────────────────────────────────────────────
    // Default endpoint = CritSample (1000 samples); also test IFK for comparison.
    // voxel_delta = 0.02 for grid pipelines.

    const PipelineConfig PIPES[] = {
        // name                        ep_src                env_type                 sub  delta  n_crit
        {"IFK->AABB(sub=1)",           EndpointSource::IFK,        EnvelopeType::LinkIAABB,      1, 0.0,  0},
        {"IFK->AABB(sub=4)",           EndpointSource::IFK,        EnvelopeType::LinkIAABB,      4, 0.0,  0},
        {"IFK->Grid(flat)",            EndpointSource::IFK,        EnvelopeType::LinkIAABB_Grid,  1, 0.02, 0},
        {"IFK->Hull16(sparse)",        EndpointSource::IFK,        EnvelopeType::Hull16_Grid,     1, 0.02, 0},
        {"Crit1k->AABB(sub=1)",        EndpointSource::CritSample, EnvelopeType::LinkIAABB,      1, 0.0,  1000},
        {"Crit1k->AABB(sub=4)",        EndpointSource::CritSample, EnvelopeType::LinkIAABB,      4, 0.0,  1000},
        {"Crit1k->Grid(flat)",         EndpointSource::CritSample, EnvelopeType::LinkIAABB_Grid,  1, 0.02, 1000},
        {"Crit1k->Hull16(sparse)",     EndpointSource::CritSample, EnvelopeType::Hull16_Grid,     1, 0.02, 1000},
        {"Crit5k->AABB(sub=1)",        EndpointSource::CritSample, EnvelopeType::LinkIAABB,      1, 0.0,  5000},
        {"Crit5k->Hull16(sparse)",     EndpointSource::CritSample, EnvelopeType::Hull16_Grid,     1, 0.02, 5000},
    };
    constexpr int N_PIPES = sizeof(PIPES) / sizeof(PIPES[0]);

    // Half-widths to test (rad) — controls interval width
    const double HALF_WIDTHS[] = {0.05, 0.15, 0.3, 0.5, 1.0};
    constexpr int N_HW = sizeof(HALF_WIDTHS) / sizeof(HALF_WIDTHS[0]);

    // ── 2DOF Robot ──────────────────────────────────────────────────────
    {
        auto robot = Robot::from_json("data/2dof_planar.json");
        std::printf("\n\n>>> Robot: 2dof_planar  (n_joints=%d, n_active=%d)\n",
                    robot.n_joints(), robot.n_active_links());

        for (int hw = 0; hw < N_HW; ++hw) {
            double half = HALF_WIDTHS[hw];
            std::mt19937 rng(SEED);
            for (int s = 0; s < N_RANDOM; ++s) {
                auto ivs = random_intervals(robot, rng, half);
                char title[256];
                std::snprintf(title, sizeof(title),
                    "2DOF  width=%.2frad  #%d  [%.3f,%.3f] x [%.3f,%.3f]  "
                    "(cold=%d, hot=%d)",
                    half * 2, s + 1,
                    ivs[0].lo, ivs[0].hi, ivs[1].lo, ivs[1].hi,
                    N_COLD_REPEATS, N_HOT_ROUNDS);
                bench_interval(robot, ivs, PIPES, N_PIPES, title,
                               SEED, N_COLD_REPEATS, N_HOT_ROUNDS);
            }
        }
    }

    // ── 7DOF Panda ──────────────────────────────────────────────────────
    {
        auto robot = Robot::from_json("data/panda.json");
        std::printf("\n\n>>> Robot: panda_7dof  (n_joints=%d, n_active=%d, tool=%d)\n",
                    robot.n_joints(), robot.n_active_links(),
                    robot.has_tool() ? 1 : 0);

        // Panda has tighter limits, cap half-widths
        const double PANDA_HW[] = {0.05, 0.15, 0.3, 0.5};
        constexpr int N_PHW = sizeof(PANDA_HW) / sizeof(PANDA_HW[0]);

        for (int hw = 0; hw < N_PHW; ++hw) {
            double half = PANDA_HW[hw];
            std::mt19937 rng(SEED);
            for (int s = 0; s < N_RANDOM; ++s) {
                auto ivs = random_intervals(robot, rng, half);
                char title[256];
                std::snprintf(title, sizeof(title),
                    "Panda  width=%.2frad  #%d  (cold=%d, hot=%d)",
                    half * 2, s + 1,
                    N_COLD_REPEATS, N_HOT_ROUNDS);
                bench_interval(robot, ivs, PIPES, N_PIPES, title,
                               SEED, N_COLD_REPEATS, N_HOT_ROUNDS);
            }
        }
    }

    std::printf("\n\nDone.\n");
    return 0;
}

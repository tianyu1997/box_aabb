// SafeBoxForest v5 — Benchmark: Link Envelope Pipelines
// Fixed robot: KUKA iiwa14
// Fixed EP source: CritSample (1000 samples)
// Compares link envelope variants: AABB(sub=1), AABB(sub=N), Grid(flat), Hull16(sparse)
// Across multiple interval widths, cold vs hot, deterministic seed.

#include <sbf/core/types.h>
#include <sbf/core/robot.h>
#include <sbf/core/fk_state.h>
#include <sbf/envelope/endpoint_source.h>
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

static double sum_ep_volume(const float* ep, int n_act) {
    double vol = 0.0;
    int n_ep = n_act * 2;
    for (int i = 0; i < n_ep; ++i) {
        const float* a = ep + i * 6;
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

// ═══════════════════════════════════════════════════════════════════════════
//  Link envelope pipeline definition
// ═══════════════════════════════════════════════════════════════════════════

struct EnvPipeline {
    const char* name;
    EnvelopeType type;
    int  n_sub;         // subdivision count (only for LinkIAABB)
    double voxel_delta; // for grid types
};

// ═══════════════════════════════════════════════════════════════════════════
//  Result struct
// ═══════════════════════════════════════════════════════════════════════════

struct EnvResult {
    double env_time_us  = 0.0;   // link envelope time
    double link_volume  = 0.0;   // sum of link AABB volumes
    int    n_voxels     = 0;     // grid voxel count (0 if no grid)
    double grid_volume  = 0.0;   // occupied voxel volume (m^3)
};

// ═══════════════════════════════════════════════════════════════════════════
//  Run one link envelope pipeline
// ═══════════════════════════════════════════════════════════════════════════

static EnvResult run_envelope(
    const float* ep_iaabbs, int n_act, const double* link_radii,
    const EnvPipeline& pipe)
{
    EnvResult res;

    EnvelopeTypeConfig cfg;
    cfg.type = pipe.type;
    cfg.n_subdivisions = pipe.n_sub;
    cfg.grid_config.voxel_delta = pipe.voxel_delta;

    auto t0 = Clock::now();
    LinkEnvelope env = compute_link_envelope(
        ep_iaabbs, n_act, link_radii, cfg);
    auto t1 = Clock::now();

    res.env_time_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    int n_boxes = n_act * env.n_subdivisions;
    res.link_volume = sum_link_volume(env.link_iaabbs.data(), n_boxes);

    if (env.sparse_grid) {
        res.n_voxels = env.sparse_grid->count_occupied();
        res.grid_volume = env.sparse_grid->occupied_volume();
    } else {
        // AABB-only: compute union volume via temp grid (inflated by r)
        voxel::SparseVoxelGrid temp_sg(pipe.voxel_delta);
        int n_sub = env.n_subdivisions;
        for (int i = 0; i < n_boxes; ++i) {
            const float* src = env.link_iaabbs.data() + i * 6;
            int ci = i / n_sub;
            float r = link_radii ? static_cast<float>(link_radii[ci]) : 0.0f;
            float inflated[6] = {
                src[0] - r, src[1] - r, src[2] - r,
                src[3] + r, src[4] + r, src[5] + r
            };
            temp_sg.fill_aabb(inflated);
        }
        res.n_voxels = temp_sg.count_occupied();
        res.grid_volume = temp_sg.occupied_volume();
    }

    return res;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Table printing
// ═══════════════════════════════════════════════════════════════════════════

static void print_header() {
    std::printf("  %-26s %-5s %9s %11s %8s %11s\n",
                "LinkEnvelope", "Mode", "Env(us)", "LinkVol",
                "Voxels", "GridVol");
    std::printf("  %-26s %-5s %9s %11s %8s %11s\n",
                "--------------------------", "-----",
                "---------", "-----------",
                "--------", "-----------");
}

static void print_row(const EnvPipeline& pipe, const char* mode,
                      const EnvResult& r) {
    std::printf("  %-26s %-5s %9.1f %11.6f %8d %11.6f\n",
                pipe.name, mode,
                r.env_time_us, r.link_volume, r.n_voxels, r.grid_volume);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Benchmark one interval width
// ═══════════════════════════════════════════════════════════════════════════

static void bench_width(
    const Robot& robot,
    double half_width,
    const EnvPipeline* pipes, int n_pipes,
    uint64_t seed,
    int n_crit_samples,
    int n_cold_repeats,
    int n_hot_rounds,
    int n_random_intervals)
{
    int n_act = robot.n_active_links();
    int ep_len = n_act * 2 * 6;

    std::printf("\n━━━ width = %.2f rad  (half = %.3f) ━━━\n",
                half_width * 2, half_width);

    std::mt19937 rng(seed);

    for (int si = 0; si < n_random_intervals; ++si) {
        auto ivs = random_intervals(robot, rng, half_width);

        // ── Compute EP iAABBs (CritSample, cold) ──
        auto ep_cold = compute_endpoint_iaabb_crit(
            robot, ivs, n_crit_samples, seed);

        // ── Compute EP iAABBs (CritSample, hot = hull of multiple rounds) ──
        std::vector<float> ep_hot(ep_len);
        for (int i = 0; i < ep_len; i += 6) {
            ep_hot[i]   = ep_hot[i+1] = ep_hot[i+2] =  std::numeric_limits<float>::max();
            ep_hot[i+3] = ep_hot[i+4] = ep_hot[i+5] = -std::numeric_limits<float>::max();
        }
        for (int round = 0; round < n_hot_rounds; ++round) {
            auto ep_r = compute_endpoint_iaabb_crit(
                robot, ivs, n_crit_samples, seed + round);
            hull_endpoint_iaabbs(ep_hot.data(), ep_r.endpoint_iaabbs.data(),
                                 n_act * 2);
        }

        double ep_cold_vol = sum_ep_volume(
            ep_cold.endpoint_iaabbs.data(), n_act);
        double ep_hot_vol = sum_ep_volume(ep_hot.data(), n_act);

        std::printf("\n  interval #%d  |  EP_cold_vol=%.6f  EP_hot_vol=%.6f"
                    "  (hot_rounds=%d)\n",
                    si + 1, ep_cold_vol, ep_hot_vol, n_hot_rounds);
        print_header();

        for (int pi = 0; pi < n_pipes; ++pi) {
            auto& pipe = pipes[pi];

            // ── Cold: envelope from single CritSample run ──
            double cold_time_sum = 0.0;
            EnvResult cold_last;
            for (int r = 0; r < n_cold_repeats; ++r) {
                auto res = run_envelope(
                    ep_cold.endpoint_iaabbs.data(), n_act,
                    robot.active_link_radii(), pipe);
                cold_time_sum += res.env_time_us;
                cold_last = res;
            }
            cold_last.env_time_us = cold_time_sum / n_cold_repeats;
            print_row(pipe, "cold", cold_last);

            // ── Hot: envelope from hull-accumulated CritSample ──
            double hot_time_sum = 0.0;
            EnvResult hot_last;
            for (int r = 0; r < n_cold_repeats; ++r) {
                auto res = run_envelope(
                    ep_hot.data(), n_act,
                    robot.active_link_radii(), pipe);
                hot_time_sum += res.env_time_us;
                hot_last = res;
            }
            hot_last.env_time_us = hot_time_sum / n_cold_repeats;
            print_row(pipe, "hot", hot_last);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Main
// ═══════════════════════════════════════════════════════════════════════════

int main() {
    std::printf("SafeBoxForest v5 — Link Envelope Pipeline Benchmark\n");
    std::printf("Robot: KUKA iiwa14 R820 | EP: CritSample\n");
    std::printf("====================================================\n");

    constexpr uint64_t SEED           = 42;
    constexpr int      N_CRIT_SAMPLES = 1000;
    constexpr int      N_COLD_REPEATS = 50;
    constexpr int      N_HOT_ROUNDS   = 10;
    constexpr int      N_RANDOM       = 3;

    // ── Pipeline definitions ─────────────────────────────────────────────
    const EnvPipeline PIPES[] = {
        // name                      type                     sub  delta
        {"AABB(sub=1)",              EnvelopeType::LinkIAABB,      1, 0.02},
        {"AABB(sub=2)",              EnvelopeType::LinkIAABB,      2, 0.02},
        {"AABB(sub=4)",              EnvelopeType::LinkIAABB,      4, 0.02},
        {"AABB(sub=8)",              EnvelopeType::LinkIAABB,      8, 0.02},
        {"Grid(d=0.05)",             EnvelopeType::LinkIAABB_Grid, 1, 0.05},
        {"Grid(d=0.02)",             EnvelopeType::LinkIAABB_Grid, 1, 0.02},
        {"Grid(d=0.01)",             EnvelopeType::LinkIAABB_Grid, 1, 0.01},
        {"Hull16(d=0.05)",           EnvelopeType::Hull16_Grid,    1, 0.05},
        {"Hull16(d=0.02)",           EnvelopeType::Hull16_Grid,    1, 0.02},
        {"Hull16(d=0.01)",           EnvelopeType::Hull16_Grid,    1, 0.01},
    };
    constexpr int N_PIPES = sizeof(PIPES) / sizeof(PIPES[0]);

    // Half-widths (rad)
    const double HALF_WIDTHS[] = {0.05, 0.1, 0.2, 0.3, 0.5, 0.8};
    constexpr int N_HW = sizeof(HALF_WIDTHS) / sizeof(HALF_WIDTHS[0]);

    // ── Load robot ──────────────────────────────────────────────────────
    auto robot = Robot::from_json("data/iiwa14.json");
    if (robot.n_joints() == 0) {
        std::fprintf(stderr, "ERROR: failed to load data/iiwa14.json\n");
        return 1;
    }

    std::printf("\nRobot: %s  (n_joints=%d, n_active=%d, tool=%d)\n",
                robot.name().c_str(),
                robot.n_joints(), robot.n_active_links(),
                robot.has_tool() ? 1 : 0);

    std::printf("Active links:");
    for (int i = 0; i < robot.n_active_links(); ++i)
        std::printf(" %d", robot.active_link_map()[i]);
    std::printf("\n");

    if (robot.has_link_radii()) {
        std::printf("Active radii:");
        for (int i = 0; i < robot.n_active_links(); ++i)
            std::printf(" %.3f", robot.active_link_radii()[i]);
        std::printf("\n");
    }

    // ── Run benchmarks per width ────────────────────────────────────────
    for (int hw = 0; hw < N_HW; ++hw) {
        bench_width(robot, HALF_WIDTHS[hw], PIPES, N_PIPES,
                    SEED, N_CRIT_SAMPLES, N_COLD_REPEATS, N_HOT_ROUNDS,
                    N_RANDOM);
    }

    std::printf("\n\nDone.\n");
    return 0;
}

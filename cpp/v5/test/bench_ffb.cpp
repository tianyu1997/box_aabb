// SafeBoxForest v5 — Benchmark: FFB performance across pipeline configs
//
// 4 EndpointSource × 3 EnvelopeType × 2 cache modes = 24 comparison groups
// Multiple interval half-widths × N_SEEDS deterministic seeds per width.
//
// Cold = fresh LECT per FFB call (no envelope cache reuse).
// Hot  = shared LECT across sequential FFB calls (later calls reuse
//        envelopes computed by earlier calls on the same tree).
//
// Outputs CSV to stdout for easy post-processing.

#include <sbf/core/types.h>
#include <sbf/core/robot.h>
#include <sbf/core/fk_state.h>
#include <sbf/lect/lect.h>
#include <sbf/ffb/ffb.h>
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/envelope_type.h>
#include <sbf/envelope/gcpc_source.h>

#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <vector>

using namespace sbf;
using Clock = std::chrono::high_resolution_clock;

// ═══════════════════════════════════════════════════════════════════════════
//  Configuration
// ═══════════════════════════════════════════════════════════════════════════

// Deterministic master seed
static constexpr uint64_t MASTER_SEED  = 42;

// Number of FFB calls per (config, half-width) combination
static constexpr int N_SEEDS = 8;

// FFB parameters
static constexpr int    FFB_MAX_DEPTH = 10;
static constexpr double FFB_MIN_EDGE  = 0.01;

// Obstacles — moderate difficulty (some collide, some don't)
static constexpr int N_OBS = 5;

// ═══════════════════════════════════════════════════════════════════════════
//  Pipeline grid
// ═══════════════════════════════════════════════════════════════════════════

static const char* ep_name(EndpointSource s) {
    switch (s) {
        case EndpointSource::IFK:        return "IFK";
        case EndpointSource::CritSample: return "Crit";
        case EndpointSource::Analytical: return "Anly";
        case EndpointSource::GCPC:       return "GCPC";
    }
    return "?";
}

static const char* env_name(EnvelopeType t) {
    switch (t) {
        case EnvelopeType::LinkIAABB:      return "AABB";
        case EnvelopeType::LinkIAABB_Grid: return "Grid";
        case EnvelopeType::Hull16_Grid:    return "H16G";
    }
    return "?";
}

struct PipelineSpec {
    EndpointSource ep;
    EnvelopeType   env;
};

static const EndpointSource ALL_EP[] = {
    EndpointSource::IFK,
    EndpointSource::CritSample,
    EndpointSource::Analytical,
    EndpointSource::GCPC,
};
static constexpr int N_EP = 4;

static const EnvelopeType ALL_ENV[] = {
    EnvelopeType::LinkIAABB,
    EnvelopeType::LinkIAABB_Grid,
    EnvelopeType::Hull16_Grid,
};
static constexpr int N_ENV = 3;

// ═══════════════════════════════════════════════════════════════════════════
//  Helpers
// ═══════════════════════════════════════════════════════════════════════════

static std::vector<Interval> make_root_intervals(
    const Robot& robot, double half_width, std::mt19937& rng)
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

static Eigen::VectorXd random_seed_in(
    const std::vector<Interval>& ivs, std::mt19937& rng)
{
    int n = static_cast<int>(ivs.size());
    Eigen::VectorXd s(n);
    for (int j = 0; j < n; ++j) {
        std::uniform_real_distribution<double> d(ivs[j].lo, ivs[j].hi);
        s[j] = d(rng);
    }
    return s;
}

static std::vector<Obstacle> make_obstacles(
    const Robot& robot, const std::vector<Interval>& ivs,
    int n_obs, std::mt19937& rng)
{
    // Compute FK at interval midpoint to find where links actually are,
    // then place obstacles overlapping those link AABBs.
    int n = robot.n_joints();
    int n_act = robot.n_active_links();
    std::vector<Interval> mid_ivs(n);
    for (int j = 0; j < n; ++j) {
        double c = (ivs[j].lo + ivs[j].hi) * 0.5;
        mid_ivs[j] = Interval(c, c); // zero-width = single config
    }
    FKState fk = compute_fk_full(robot, mid_ivs);
    std::vector<float> link_boxes(n_act * 6);
    extract_link_aabbs(fk, robot.active_link_map(), n_act,
                       link_boxes.data(), robot.active_link_radii());

    // For each obstacle, pick a random active link and offset
    // Shift range large enough that ~50% of obstacles miss the envelope
    std::uniform_int_distribution<int> link_dist(0, n_act - 1);
    std::uniform_real_distribution<float> shift(-0.5f, 0.5f);
    std::uniform_real_distribution<float> sz(0.05f, 0.15f);

    std::vector<Obstacle> obs(n_obs);
    for (int i = 0; i < n_obs; ++i) {
        int li = link_dist(rng);
        const float* lb = link_boxes.data() + li * 6;
        float cx = (lb[0] + lb[3]) * 0.5f + shift(rng);
        float cy = (lb[1] + lb[4]) * 0.5f + shift(rng);
        float cz = (lb[2] + lb[5]) * 0.5f + shift(rng);
        float hw = sz(rng);
        obs[i] = Obstacle(cx - hw, cy - hw, cz - hw,
                          cx + hw, cy + hw, cz + hw);
    }
    return obs;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Per-call result
// ═══════════════════════════════════════════════════════════════════════════

struct FFBBenchRow {
    // Config identifiers
    const char* robot_name;
    const char* ep_str;
    const char* env_str;
    const char* cache_mode; // "cold" or "hot"
    double      half_width;
    int         seed_idx;

    // FFB result
    int    success;      // 0 or 1
    int    fail_code;
    int    depth;        // path length - 1
    int    n_fk_calls;
    int    n_new_nodes;
    double time_us;      // wall-clock microseconds

    double box_volume;   // product of (hi-lo) over joints

    // LECT state
    int    lect_n_nodes;
};

// ═══════════════════════════════════════════════════════════════════════════
//  Benchmark runner
// ═══════════════════════════════════════════════════════════════════════════

static void run_ffb_bench(
    const Robot& robot,
    const GcpcCache& gcpc_cache,
    const char* robot_name,
    const double* half_widths, int n_hw,
    std::vector<FFBBenchRow>& results)
{
    FFBConfig ffb_cfg;
    ffb_cfg.max_depth = FFB_MAX_DEPTH;
    ffb_cfg.min_edge  = FFB_MIN_EDGE;

    for (int hw_i = 0; hw_i < n_hw; ++hw_i) {
        double hw = half_widths[hw_i];

        // Deterministic RNG for intervals + obstacles + seeds
        std::mt19937 rng_scene(MASTER_SEED + hw_i * 1000);
        auto root_ivs = make_root_intervals(robot, hw, rng_scene);
        auto obs = make_obstacles(robot, root_ivs, N_OBS, rng_scene);

        // Pre-generate seeds (shared across all 24 configs)
        std::mt19937 rng_seeds(MASTER_SEED + hw_i * 2000);
        std::vector<Eigen::VectorXd> seeds(N_SEEDS);
        for (int s = 0; s < N_SEEDS; ++s)
            seeds[s] = random_seed_in(root_ivs, rng_seeds);

        // Iterate over 4 EP × 3 ENV × 2 cache modes = 24 groups
        for (int ei = 0; ei < N_EP; ++ei) {
            for (int vi = 0; vi < N_ENV; ++vi) {
                EndpointSourceConfig ep_cfg;
                ep_cfg.source = ALL_EP[ei];
                ep_cfg.n_samples_crit = 1000;
                ep_cfg.max_phase_analytical = 3;
                ep_cfg.gcpc_cache = &gcpc_cache;

                EnvelopeTypeConfig env_cfg;
                env_cfg.type = ALL_ENV[vi];
                env_cfg.n_subdivisions = 1;
                env_cfg.grid_config.voxel_delta = 0.02;

                const char* ep_str  = ep_name(ALL_EP[ei]);
                const char* env_str = env_name(ALL_ENV[vi]);

                // ── Cold mode: fresh LECT per FFB call ──────────────
                for (int s = 0; s < N_SEEDS; ++s) {
                    LECT lect(robot, root_ivs, ep_cfg, env_cfg);

                    auto t0 = Clock::now();
                    FFBResult ffb = find_free_box(
                        lect, seeds[s], obs.data(), N_OBS, ffb_cfg);
                    auto t1 = Clock::now();

                    FFBBenchRow row;
                    row.robot_name  = robot_name;
                    row.ep_str      = ep_str;
                    row.env_str     = env_str;
                    row.cache_mode  = "cold";
                    row.half_width  = hw;
                    row.seed_idx    = s;
                    row.success     = ffb.success() ? 1 : 0;
                    row.fail_code   = ffb.fail_code;
                    row.depth       = static_cast<int>(ffb.path.size()) - 1;
                    row.n_fk_calls  = ffb.n_fk_calls;
                    row.n_new_nodes = ffb.n_new_nodes;
                    row.time_us     = std::chrono::duration<double, std::micro>(
                                          t1 - t0).count();
                    if (ffb.success()) {
                        auto biv = lect.node_intervals(ffb.node_idx);
                        double vol = 1.0;
                        for (auto& iv : biv) vol *= (iv.hi - iv.lo);
                        row.box_volume = vol;
                    } else {
                        row.box_volume = 0.0;
                    }
                    row.lect_n_nodes = lect.n_nodes();
                    results.push_back(row);
                }

                // ── Hot mode: shared LECT across all seeds ──────────
                {
                    LECT lect(robot, root_ivs, ep_cfg, env_cfg);
                    for (int s = 0; s < N_SEEDS; ++s) {
                        auto t0 = Clock::now();
                        FFBResult ffb = find_free_box(
                            lect, seeds[s], obs.data(), N_OBS, ffb_cfg);
                        auto t1 = Clock::now();

                        FFBBenchRow row;
                        row.robot_name  = robot_name;
                        row.ep_str      = ep_str;
                        row.env_str     = env_str;
                        row.cache_mode  = "hot";
                        row.half_width  = hw;
                        row.seed_idx    = s;
                        row.success     = ffb.success() ? 1 : 0;
                        row.fail_code   = ffb.fail_code;
                        row.depth       = static_cast<int>(ffb.path.size()) - 1;
                        row.n_fk_calls  = ffb.n_fk_calls;
                        row.n_new_nodes = ffb.n_new_nodes;
                        row.time_us     = std::chrono::duration<double, std::micro>(
                                              t1 - t0).count();
                        if (ffb.success()) {
                            auto biv = lect.node_intervals(ffb.node_idx);
                            double vol = 1.0;
                            for (auto& iv : biv) vol *= (iv.hi - iv.lo);
                            row.box_volume = vol;
                        } else {
                            row.box_volume = 0.0;
                        }
                        row.lect_n_nodes = lect.n_nodes();
                        results.push_back(row);
                    }
                }
            }
        }

        std::fprintf(stderr, "  [%s] hw=%.3f done (%d rows so far)\n",
                     robot_name, hw, static_cast<int>(results.size()));
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Summary table (stderr) — aggregated per group
// ═══════════════════════════════════════════════════════════════════════════

struct GroupStats {
    const char* ep;
    const char* env;
    const char* cache;
    double hw;

    int    count       = 0;
    int    n_success   = 0;
    double time_sum    = 0.0;
    double time_min    = 1e18;
    double time_max    = 0.0;
    int    fk_sum      = 0;
    int    new_sum     = 0;
    int    depth_sum   = 0;
    double vol_sum     = 0.0;
    int    vol_count   = 0;
};

static void print_summary(const std::vector<FFBBenchRow>& rows) {
    // Group by (robot, ep, env, cache, hw)
    struct Key {
        const char* robot; const char* ep; const char* env; const char* cache; double hw;
        bool operator==(const Key& o) const {
            return std::strcmp(robot, o.robot) == 0 &&
                   std::strcmp(ep, o.ep) == 0 &&
                   std::strcmp(env, o.env) == 0 &&
                   std::strcmp(cache, o.cache) == 0 &&
                   hw == o.hw;
        }
    };

    std::vector<Key> keys;
    std::vector<GroupStats> stats;

    for (auto& r : rows) {
        Key k{r.robot_name, r.ep_str, r.env_str, r.cache_mode, r.half_width};
        int idx = -1;
        for (int i = 0; i < static_cast<int>(keys.size()); ++i) {
            if (keys[i] == k) { idx = i; break; }
        }
        if (idx < 0) {
            idx = static_cast<int>(keys.size());
            keys.push_back(k);
            GroupStats gs;
            gs.ep = r.ep_str; gs.env = r.env_str; gs.cache = r.cache_mode; gs.hw = r.half_width;
            stats.push_back(gs);
        }
        auto& g = stats[idx];
        g.count++;
        g.n_success += r.success;
        g.time_sum  += r.time_us;
        if (r.time_us < g.time_min) g.time_min = r.time_us;
        if (r.time_us > g.time_max) g.time_max = r.time_us;
        g.fk_sum    += r.n_fk_calls;
        g.new_sum   += r.n_new_nodes;
        g.depth_sum += r.depth;
        if (r.success) {
            g.vol_sum += r.box_volume;
            g.vol_count++;
        }
    }

    std::fprintf(stderr, "\n");
    std::fprintf(stderr, "%-6s %-4s %-4s %-5s %6s %5s %10s %10s %10s %7s %7s %12s %6s\n",
                 "Robot", "EP", "Env", "Cache", "HW", "N",
                 "Avg(us)", "Min(us)", "Max(us)",
                 "AvgFK", "AvgNew", "AvgVol", "Succ%");
    std::fprintf(stderr, "%-6s %-4s %-4s %-5s %6s %5s %10s %10s %10s %7s %7s %12s %6s\n",
                 "------", "----", "----", "-----", "------", "-----",
                 "----------", "----------", "----------",
                 "-------", "-------", "------------", "------");

    const char* last_robot = "";
    double last_hw = -1;
    for (int i = 0; i < static_cast<int>(stats.size()); ++i) {
        auto& g = stats[i];
        auto& k = keys[i];
        if (std::strcmp(k.robot, last_robot) != 0 || k.hw != last_hw) {
            if (i > 0) std::fprintf(stderr, "\n");
            last_robot = k.robot;
            last_hw = k.hw;
        }
        double avg_t  = g.time_sum / g.count;
        double avg_fk  = double(g.fk_sum) / g.count;
        double avg_new = double(g.new_sum) / g.count;
        double succ_pct = 100.0 * g.n_success / g.count;
        double avg_vol = g.vol_count > 0 ? g.vol_sum / g.vol_count : 0.0;
        std::fprintf(stderr, "%-6s %-4s %-4s %-5s %6.3f %5d %10.1f %10.1f %10.1f %7.1f %7.1f %12.6g %5.1f%%\n",
                     k.robot, g.ep, g.env, g.cache, g.hw,
                     g.count, avg_t, g.time_min, g.time_max,
                     avg_fk, avg_new, avg_vol, succ_pct);
    }
    std::fprintf(stderr, "\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Main
// ═══════════════════════════════════════════════════════════════════════════

int main() {
    std::fprintf(stderr, "SafeBoxForest v5 — FFB Pipeline Benchmark\n");
    std::fprintf(stderr, "4 EP x 3 Env x 2 cache = 24 groups per half-width\n");
    std::fprintf(stderr, "N_SEEDS=%d  max_depth=%d  min_edge=%.0e  N_OBS=%d\n\n",
                 N_SEEDS, FFB_MAX_DEPTH, FFB_MIN_EDGE, N_OBS);

    // CSV header
    std::printf("robot,ep,env,cache,half_width,seed_idx,"
                "success,fail_code,depth,n_fk,n_new,time_us,box_volume,lect_nodes\n");

    std::vector<FFBBenchRow> results;
    results.reserve(50000);

    // Half-widths: small → large
    const double HW_2DOF[] = {0.05, 0.1, 0.2, 0.4, 0.8, 1.2};
    constexpr int N_HW_2DOF = sizeof(HW_2DOF) / sizeof(HW_2DOF[0]);

    const double HW_7DOF[] = {0.05, 0.1, 0.2};
    constexpr int N_HW_7DOF = sizeof(HW_7DOF) / sizeof(HW_7DOF[0]);

    // ── 2DOF planar ─────────────────────────────────────────────────────
    {
        std::fprintf(stderr, "Loading 2dof_planar ...\n");
        auto robot = Robot::from_json("data/2dof_planar.json");
        auto gcpc  = GcpcCache::load("data/2dof_500.gcpc");
        std::fprintf(stderr, "  n_joints=%d  n_active=%d  gcpc=%d points\n",
                     robot.n_joints(), robot.n_active_links(), gcpc.n_points());

        run_ffb_bench(robot, gcpc, "2dof", HW_2DOF, N_HW_2DOF, results);
    }

    // ── 7DOF Panda ──────────────────────────────────────────────────────
    {
        std::fprintf(stderr, "Loading panda ...\n");
        auto robot = Robot::from_json("data/panda.json");
        auto gcpc  = GcpcCache::load("data/panda_5000.gcpc");
        std::fprintf(stderr, "  n_joints=%d  n_active=%d  gcpc=%d points\n",
                     robot.n_joints(), robot.n_active_links(), gcpc.n_points());

        run_ffb_bench(robot, gcpc, "panda", HW_7DOF, N_HW_7DOF, results);
    }

    // ── Flush CSV rows ──────────────────────────────────────────────────
    for (auto& r : results) {
        std::printf("%s,%s,%s,%s,%.4f,%d,%d,%d,%d,%d,%d,%.1f,%.8g,%d\n",
                    r.robot_name, r.ep_str, r.env_str, r.cache_mode,
                    r.half_width, r.seed_idx,
                    r.success, r.fail_code, r.depth,
                    r.n_fk_calls, r.n_new_nodes, r.time_us,
                    r.box_volume, r.lect_n_nodes);
    }

    // ── Summary table to stderr ─────────────────────────────────────────
    print_summary(results);

    int total = static_cast<int>(results.size());
    int succ  = 0;
    for (auto& r : results) succ += r.success;
    std::fprintf(stderr, "Total rows: %d  (%d success, %d fail)\n",
                 total, succ, total - succ);

    return 0;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Fully-Warm Benchmark — same seeds, cold vs warm
//
//  Methodology:
//    1. Pre-generate N seeds (common across cold & warm)
//    2. Cold pass: fresh tree, run all N seeds through FFB
//    3. Clear occupation (tree + all cached envelopes/hulls stay)
//    4. Warm pass: SAME N seeds through the SAME tree → pure descent
//    5. Compare: time, boxes, volume
//
//  This is the fairest cold-vs-warm comparison: identical workload,
//  the only difference is whether envelopes/hulls are already cached.
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/forest/sbf.h"
#include "sbf/robot/robot.h"
#include "sbf/common/types.h"

#include <chrono>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

using namespace sbf;
using namespace sbf::forest;
using namespace sbf::envelope;

static const std::string ROBOT_PATH =
    "C:/Users/TIAN/OneDrive - The Chinese University of Hong Kong"
    "/Desktop/code/box_aabb/safeboxforest/v1/configs/iiwa14.json";

static std::vector<Obstacle> make_obstacles() {
    std::vector<Obstacle> obs;
    obs.emplace_back(Eigen::Vector3d(0.5, 0.0, 0.3),
                     Eigen::Vector3d(0.1, 0.1, 0.1), "box1");
    obs.emplace_back(Eigen::Vector3d(-0.3, 0.4, 0.5),
                     Eigen::Vector3d(0.05, 0.05, 0.05), "box2");
    obs.emplace_back(Eigen::Vector3d(0.0, -0.5, 0.1),
                     Eigen::Vector3d(0.15, 0.08, 0.2), "box3");
    return obs;
}

// ─── Pre-generate seeds ─────────────────────────────────────────────────────
static std::vector<Eigen::VectorXd> generate_seeds(
    const Robot& robot, int n_seeds, uint64_t rng_seed)
{
    const auto& limits = robot.joint_limits().limits;
    int n_dims = static_cast<int>(limits.size());
    std::mt19937_64 rng(rng_seed);
    std::uniform_real_distribution<double> udist(0.0, 1.0);

    std::vector<Eigen::VectorXd> seeds(n_seeds);
    for (int s = 0; s < n_seeds; ++s) {
        seeds[s].resize(n_dims);
        for (int d = 0; d < n_dims; ++d)
            seeds[s][d] = limits[d].lo + udist(rng) * limits[d].width();
    }
    return seeds;
}

// ─── Run one pass (cold or warm) with pre-generated seeds ───────────────────
struct PassResult {
    int    n_success  = 0;
    int    n_fail     = 0;
    int    n_nodes    = 0;
    double total_ms   = 0.0;
    double total_vol  = 0.0;
};

static PassResult run_pass(
    LECT& lect,
    const std::vector<Eigen::VectorXd>& seeds,
    const Obstacle* obstacles, int n_obs,
    double min_edge, int max_depth)
{
    PassResult pr;
    int n = static_cast<int>(seeds.size());

    auto t0 = std::chrono::high_resolution_clock::now();

    for (int s = 0; s < n; ++s) {
        FFBResult ffb = lect.find_free_box(
            seeds[s], obstacles, n_obs, min_edge, max_depth);

        if (ffb.success()) {
            lect.mark_occupied(ffb.node_idx, s);
            pr.n_success++;
            // Compute box volume
            auto ivs = lect.node_intervals(ffb.node_idx);
            double vol = 1.0;
            for (auto& iv : ivs) vol *= iv.width();
            pr.total_vol += vol;
        } else {
            pr.n_fail++;
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    pr.total_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    pr.n_nodes  = lect.n_nodes();
    return pr;
}

// ═══════════════════════════════════════════════════════════════════════════
struct BenchRow {
    const char* name;
    int    n_seeds;
    // Cold
    int    cold_boxes;
    int    cold_nodes;
    double cold_ms;
    double cold_vol;
    // Warm
    int    warm_boxes;
    int    warm_nodes;
    double warm_ms;
    double warm_vol;
};

int main() {
    std::printf("╔══════════════════════════════════════════════════════════════════════╗\n");
    std::printf("║   Fully-Warm Benchmark — same seeds, cold vs warm                  ║\n");
    std::printf("╚══════════════════════════════════════════════════════════════════════╝\n\n");

    Robot robot = Robot::from_json(ROBOT_PATH);
    auto obs = make_obstacles();
    const Obstacle* obs_ptr = obs.data();
    int n_obs = static_cast<int>(obs.size());

    constexpr int    N_SEEDS  = 100;
    constexpr double MIN_EDGE = 0.05;
    constexpr int    MAX_DEPTH = 30;

    // Pre-generate one set of seeds for all pipelines
    auto seeds = generate_seeds(robot, N_SEEDS, 42);

    struct PipelineCase {
        const char* name;
        SBFConfig config;
    };

    PipelineCase cases[] = {
        {"IFK+SubAABB",         SBFConfig::ifk_sub_aabb(N_SEEDS, MIN_EDGE)},
        {"IFK+SubAABB_Grid",    SBFConfig::ifk_sub_aabb_grid(N_SEEDS, MIN_EDGE)},
        {"IFK+Hull16_Grid",     SBFConfig::ifk_hull16_grid(N_SEEDS, MIN_EDGE)},
        {"Crit+SubAABB",        SBFConfig::crit_sub_aabb(N_SEEDS, MIN_EDGE)},
        {"Crit+SubAABB_Grid",   SBFConfig::crit_sub_aabb_grid(N_SEEDS, MIN_EDGE)},
        {"Crit+Hull16_Grid",    SBFConfig::crit_hull16_grid(N_SEEDS, MIN_EDGE)},
        {"Analyt+SubAABB",      SBFConfig::analytical_sub_aabb(N_SEEDS, MIN_EDGE)},
        {"Analyt+SubAABB_Grid", SBFConfig::analytical_sub_aabb_grid(N_SEEDS, MIN_EDGE)},
        {"Analyt+Hull16_Grid",  SBFConfig::analytical_hull16_grid(N_SEEDS, MIN_EDGE)},
    };

    std::vector<BenchRow> rows;

    for (auto& tc : cases) {
        std::printf("── %s ──\n", tc.name);

        // ── Construct fresh LECT ────────────────────────────────────────
        LECT lect(robot, tc.config.pipeline);

        // Pre-rasterize scene for grid-based pipelines
        bool is_grid =
            tc.config.pipeline.envelope.type == EnvelopeType::SubAABB_Grid ||
            tc.config.pipeline.envelope.type == EnvelopeType::Hull16_Grid;
        if (is_grid)
            lect.set_scene(obs_ptr, n_obs);

        // ── Cold pass ───────────────────────────────────────────────────
        PassResult cold = run_pass(lect, seeds, obs_ptr, n_obs, MIN_EDGE, MAX_DEPTH);
        std::printf("   cold: %d seeds → %d boxes, %d nodes, %.3f ms, vol=%.6f\n",
                    N_SEEDS, cold.n_success, cold.n_nodes, cold.total_ms, cold.total_vol);

        // ── Clear occupation (keep tree + all cached envelopes/hulls) ───
        lect.clear_all_occupation();

        // ── Warm pass — SAME seeds ──────────────────────────────────────
        PassResult warm = run_pass(lect, seeds, obs_ptr, n_obs, MIN_EDGE, MAX_DEPTH);
        std::printf("   warm: %d seeds → %d boxes, %d nodes, %.3f ms, vol=%.6f\n",
                    N_SEEDS, warm.n_success, warm.n_nodes, warm.total_ms, warm.total_vol);

        double speedup = (warm.total_ms > 0) ? cold.total_ms / warm.total_ms : 0;
        std::printf("   speedup: %.2fx   vol_match: %s\n\n",
                    speedup,
                    (std::abs(cold.total_vol - warm.total_vol) < 1e-9) ? "YES" : "NO");

        BenchRow row{};
        row.name       = tc.name;
        row.n_seeds    = N_SEEDS;
        row.cold_boxes = cold.n_success;
        row.cold_nodes = cold.n_nodes;
        row.cold_ms    = cold.total_ms;
        row.cold_vol   = cold.total_vol;
        row.warm_boxes = warm.n_success;
        row.warm_nodes = warm.n_nodes;
        row.warm_ms    = warm.total_ms;
        row.warm_vol   = warm.total_vol;
        rows.push_back(row);
    }

    // ── Summary table ───────────────────────────────────────────────────
    std::printf("══════════════════════════════════════════════════════════════════════════════════════════════════════\n");
    std::printf("  SUMMARY (%d seeds, same seeds cold vs warm, min_edge=%.3f)\n", N_SEEDS, MIN_EDGE);
    std::printf("══════════════════════════════════════════════════════════════════════════════════════════════════════\n");
    std::printf("  %-24s %9s %9s %10s %10s %9s %9s %10s %8s\n",
                "Pipeline", "Cold ms", "Warm ms", "Cold us/s", "Warm us/s",
                "Boxes(C)", "Boxes(W)", "Vol match", "Speedup");
    std::printf("  %-24s %9s %9s %10s %10s %9s %9s %10s %8s\n",
                "------------------------", "---------", "---------",
                "----------", "----------", "---------", "---------",
                "----------", "--------");
    for (auto& r : rows) {
        double cold_us = (r.cold_ms * 1000.0) / r.n_seeds;
        double warm_us = (r.warm_ms * 1000.0) / r.n_seeds;
        double speedup = (r.warm_ms > 0) ? r.cold_ms / r.warm_ms : 0;
        bool vol_match = std::abs(r.cold_vol - r.warm_vol) < 1e-9;
        std::printf("  %-24s %9.1f %9.3f %10.1f %10.1f %9d %9d %10s %7.2fx\n",
                    r.name, r.cold_ms, r.warm_ms,
                    cold_us, warm_us,
                    r.cold_boxes, r.warm_boxes,
                    vol_match ? "YES" : "NO",
                    speedup);
    }
    std::printf("══════════════════════════════════════════════════════════════════════════════════════════════════════\n");

    return 0;
}

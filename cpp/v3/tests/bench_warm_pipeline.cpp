// ═══════════════════════════════════════════════════════════════════════════
//  Warm-cache benchmark — all 9 pipeline combinations
//
//  Methodology:
//    1. Cold pass: build with N_COLD seeds → fills LECT tree + envelopes
//    2. Clear occupation markers (tree structure + envelopes stay cached)
//    3. Pre-rasterize scene grid (cached for hull collision)
//    4. Warm pass: build with N_WARM seeds → pure descent + collision check
//    5. Report per-seed warm FFB time for each pipeline
//
//  This isolates the collision-checking cost from envelope computation cost.
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/forest/sbf.h"
#include "sbf/robot/robot.h"
#include "sbf/common/types.h"

#include <chrono>
#include <cstdio>
#include <iostream>
#include <random>
#include <string>
#include <vector>

using namespace sbf;
using namespace sbf::forest;
using namespace sbf::envelope;

static const std::string ROBOT_PATH =
    "C:/Users/TIAN/OneDrive - The Chinese University of Hong Kong"
    "/Desktop/code/box_aabb/safeboxforest/v1/configs/iiwa14.json";

// ─── Obstacle set ───────────────────────────────────────────────────────────
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

// ═════════════════════════════════════════════════════════════════════════════
//  Run one pipeline benchmark (cold + warm)
// ═════════════════════════════════════════════════════════════════════════════
struct BenchResult {
    const char* name;
    int cold_seeds, cold_boxes, cold_nodes;
    double cold_ms;
    int warm_seeds, warm_boxes;
    double warm_ms;
    double warm_per_seed_us;
    double total_vol;
};

static BenchResult bench_pipeline(
    const char* name,
    const Robot& robot,
    const SBFConfig& base_cfg,
    const Obstacle* obstacles, int n_obs,
    int n_cold, int n_warm)
{
    BenchResult br{};
    br.name = name;

    // ── Cold pass ───────────────────────────────────────────────────────
    SBFConfig cold_cfg = base_cfg;
    cold_cfg.max_seeds = n_cold;
    cold_cfg.rng_seed  = 42;

    SafeBoxForest sbf(robot, cold_cfg);
    SBFResult cold_result = sbf.build(obstacles, n_obs);

    br.cold_seeds = cold_result.n_seeds_tried;
    br.cold_boxes = static_cast<int>(cold_result.boxes.size());
    br.cold_nodes = cold_result.n_tree_nodes;
    br.cold_ms    = cold_result.build_time_ms;

    // ── Clear occupation (keep tree + cached envelopes) ─────────────────
    sbf.lect_mut().clear_all_occupation();

    // ── Pre-rasterize scene grid (for grid-based pipelines) ─────────────
    bool is_grid = sbf.is_grid_envelope();
    if (is_grid && !sbf.lect().has_scene_grid()) {
        sbf.lect_mut().set_scene(obstacles, n_obs);
    }

    // ── Warm pass: manually sample + FFB to measure per-seed time ───────
    std::mt19937_64 rng(12345);  // different seed from cold pass
    const auto& limits = robot.joint_limits().limits;
    int n_dims = static_cast<int>(limits.size());
    std::uniform_real_distribution<double> udist(0.0, 1.0);

    int warm_success = 0;
    int warm_fail    = 0;

    auto t0 = std::chrono::high_resolution_clock::now();

    for (int s = 0; s < n_warm; ++s) {
        // Sample random config
        Eigen::VectorXd seed(n_dims);
        for (int d = 0; d < n_dims; ++d)
            seed[d] = limits[d].lo + udist(rng) * limits[d].width();

        FFBResult ffb = sbf.lect_mut().find_free_box(
            seed, obstacles, n_obs,
            base_cfg.min_edge, base_cfg.max_depth);

        if (ffb.success()) {
            sbf.lect_mut().mark_occupied(ffb.node_idx, 10000 + s);
            warm_success++;
        } else {
            warm_fail++;
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    double warm_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    br.warm_seeds = n_warm;
    br.warm_boxes = warm_success;
    br.warm_ms    = warm_ms;
    br.warm_per_seed_us = (warm_ms * 1000.0) / n_warm;

    return br;
}

// ═════════════════════════════════════════════════════════════════════════════
int main() {
    std::printf("╔══════════════════════════════════════════════════════════════════════╗\n");
    std::printf("║   SafeBoxForest — Warm Cache Benchmark (all 9 pipelines)           ║\n");
    std::printf("╚══════════════════════════════════════════════════════════════════════╝\n\n");

    Robot robot = Robot::from_json(ROBOT_PATH);
    auto obs = make_obstacles();
    const Obstacle* obs_ptr = obs.data();
    int n_obs = static_cast<int>(obs.size());

    constexpr int N_COLD = 50;
    constexpr int N_WARM = 100;
    constexpr double MIN_EDGE = 0.05;

    struct PipelineCase {
        const char* name;
        SBFConfig config;
    };

    PipelineCase cases[] = {
        {"IFK+SubAABB",         SBFConfig::ifk_sub_aabb(N_COLD, MIN_EDGE)},
        {"IFK+SubAABB_Grid",    SBFConfig::ifk_sub_aabb_grid(N_COLD, MIN_EDGE)},
        {"IFK+Hull16_Grid",     SBFConfig::ifk_hull16_grid(N_COLD, MIN_EDGE)},
        {"Crit+SubAABB",        SBFConfig::crit_sub_aabb(N_COLD, MIN_EDGE)},
        {"Crit+SubAABB_Grid",   SBFConfig::crit_sub_aabb_grid(N_COLD, MIN_EDGE)},
        {"Crit+Hull16_Grid",    SBFConfig::crit_hull16_grid(N_COLD, MIN_EDGE)},
        {"Analyt+SubAABB",      SBFConfig::analytical_sub_aabb(N_COLD, MIN_EDGE)},
        {"Analyt+SubAABB_Grid", SBFConfig::analytical_sub_aabb_grid(N_COLD, MIN_EDGE)},
        {"Analyt+Hull16_Grid",  SBFConfig::analytical_hull16_grid(N_COLD, MIN_EDGE)},
    };

    std::vector<BenchResult> results;

    for (auto& tc : cases) {
        std::printf("── %s ──\n", tc.name);
        BenchResult br = bench_pipeline(
            tc.name, robot, tc.config, obs_ptr, n_obs, N_COLD, N_WARM);
        results.push_back(br);
        std::printf("   cold: %d seeds → %d boxes, %d nodes, %.1f ms\n",
                    br.cold_seeds, br.cold_boxes, br.cold_nodes, br.cold_ms);
        std::printf("   warm: %d seeds → %d boxes, %.3f ms total, %.1f μs/seed\n\n",
                    br.warm_seeds, br.warm_boxes, br.warm_ms, br.warm_per_seed_us);
    }

    // ── Summary table ───────────────────────────────────────────────────
    std::printf("══════════════════════════════════════════════════════════════════════\n");
    std::printf("  SUMMARY (cold=%d seeds, warm=%d seeds, min_edge=%.3f)\n", N_COLD, N_WARM, MIN_EDGE);
    std::printf("══════════════════════════════════════════════════════════════════════\n");
    std::printf("  %-24s %8s %8s %10s %10s %10s\n",
                "Pipeline", "Cold ms", "Warm ms", "us/seed(W)", "Boxes(C)", "Boxes(W)");
    std::printf("  %-24s %8s %8s %10s %10s %10s\n",
                "------------------------", "--------", "--------", "----------", "----------", "----------");
    for (auto& r : results) {
        std::printf("  %-24s %8.1f %8.3f %10.1f %10d %10d\n",
                    r.name, r.cold_ms, r.warm_ms,
                    r.warm_per_seed_us, r.cold_boxes, r.warm_boxes);
    }
    std::printf("══════════════════════════════════════════════════════════════════════\n");

    return 0;
}

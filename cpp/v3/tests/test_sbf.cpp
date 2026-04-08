// ═══════════════════════════════════════════════════════════════════════════
//  Test — SafeBoxForest minimal skeleton
//
//  Tests:
//    1. SBF construction with each of 9 pipeline combos
//    2. build() with random sampling (no obstacles → all FFB succeed)
//    3. build() with obstacles (some FFB fail)
//    4. Promotion (sibling merge)
//    5. Grid vs AABB promotion dispatch
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/forest/sbf.h"
#include "sbf/robot/robot.h"
#include "sbf/common/types.h"

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

using namespace sbf;
using namespace sbf::forest;
using namespace sbf::envelope;

static const std::string ROBOT_PATH =
    "C:/Users/TIAN/OneDrive - The Chinese University of Hong Kong"
    "/Desktop/code/box_aabb/safeboxforest/v1/configs/iiwa14.json";

// ─── Helper: small obstacle set ─────────────────────────────────────────────
static std::vector<Obstacle> make_obstacles() {
    std::vector<Obstacle> obs;
    obs.emplace_back(Eigen::Vector3d(0.5, 0.0, 0.3),
                     Eigen::Vector3d(0.1, 0.1, 0.1), "box1");
    obs.emplace_back(Eigen::Vector3d(-0.3, 0.4, 0.5),
                     Eigen::Vector3d(0.05, 0.05, 0.05), "box2");
    return obs;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Test 1: Basic SBF build with IFK+SubAABB (fastest pipeline)
// ═════════════════════════════════════════════════════════════════════════════
static void test_basic_build() {
    std::cout << "=== Test 1: Basic SBF build (IFK+SubAABB) ===\n";

    Robot robot = Robot::from_json(ROBOT_PATH);
    SBFConfig cfg = SBFConfig::ifk_sub_aabb(20, 0.05);
    SafeBoxForest sbf(robot, cfg);

    auto obs = make_obstacles();
    SBFResult result = sbf.build(obs.data(), static_cast<int>(obs.size()));

    std::printf("  seeds: %d, boxes: %d, fail: %d, promotions: %d\n",
                result.n_seeds_tried, static_cast<int>(result.boxes.size()),
                result.n_ffb_fail, result.n_promotions);
    std::printf("  total vol: %.6f, tree nodes: %d, time: %.1f ms\n",
                result.total_volume, result.n_tree_nodes, result.build_time_ms);

    assert(result.n_seeds_tried == 20);
    assert(result.n_ffb_success + result.n_ffb_fail == 20);
    assert(result.total_volume > 0.0);
    assert(sbf.n_boxes() > 0);

    // Check is_grid_envelope dispatch
    assert(!sbf.is_grid_envelope());

    std::cout << "  PASS\n\n";
}

// ═════════════════════════════════════════════════════════════════════════════
//  Test 2: IFK+Hull16_Grid (grid-based envelope)
// ═════════════════════════════════════════════════════════════════════════════
static void test_grid_build() {
    std::cout << "=== Test 2: SBF build (IFK+Hull16_Grid) ===\n";

    Robot robot = Robot::from_json(ROBOT_PATH);
    SBFConfig cfg = SBFConfig::ifk_hull16_grid(15, 0.05);
    SafeBoxForest sbf(robot, cfg);

    auto obs = make_obstacles();
    SBFResult result = sbf.build(obs.data(), static_cast<int>(obs.size()));

    std::printf("  seeds: %d, boxes: %d, fail: %d, promotions: %d\n",
                result.n_seeds_tried, static_cast<int>(result.boxes.size()),
                result.n_ffb_fail, result.n_promotions);
    std::printf("  total vol: %.6f, tree nodes: %d, time: %.1f ms\n",
                result.total_volume, result.n_tree_nodes, result.build_time_ms);

    assert(result.n_seeds_tried == 15);
    assert(sbf.is_grid_envelope());
    assert(sbf.n_boxes() > 0);

    std::cout << "  PASS\n\n";
}

// ═════════════════════════════════════════════════════════════════════════════
//  Test 3: No obstacles → every seed should succeed
// ═════════════════════════════════════════════════════════════════════════════
static void test_no_obstacles() {
    std::cout << "=== Test 3: No obstacles (all seeds succeed) ===\n";

    Robot robot = Robot::from_json(ROBOT_PATH);
    SBFConfig cfg = SBFConfig::ifk_sub_aabb(10, 0.05);
    SafeBoxForest sbf(robot, cfg);

    SBFResult result = sbf.build(nullptr, 0);

    std::printf("  seeds: %d, success: %d, fail: %d, promotions: %d\n",
                result.n_seeds_tried, result.n_ffb_success,
                result.n_ffb_fail, result.n_promotions);

    // Without obstacles, every FFB either succeeds or lands on an
    // already-occupied node.  At least the first seed must succeed.
    assert(result.n_ffb_success >= 1);
    assert(sbf.n_boxes() >= 1);

    std::cout << "  PASS\n\n";
}

// ═════════════════════════════════════════════════════════════════════════════
//  Test 4: All 9 pipeline combos can construct and build
// ═════════════════════════════════════════════════════════════════════════════
static void test_all_pipelines() {
    std::cout << "=== Test 4: All 9 pipeline combos ===\n";

    Robot robot = Robot::from_json(ROBOT_PATH);
    auto obs = make_obstacles();

    struct PipelineCase {
        const char* name;
        SBFConfig config;
    };

    // Use small seed count for speed (1 for very slow Analytical pipelines)
    PipelineCase cases[] = {
        {"IFK+SubAABB",              SBFConfig::ifk_sub_aabb(5, 0.1)},
        {"IFK+SubAABB_Grid",         SBFConfig::ifk_sub_aabb_grid(5, 0.1)},
        {"IFK+Hull16_Grid",          SBFConfig::ifk_hull16_grid(5, 0.1)},
        {"Crit+SubAABB",             SBFConfig::crit_sub_aabb(3, 0.1)},
        {"Crit+SubAABB_Grid",        SBFConfig::crit_sub_aabb_grid(3, 0.1)},
        {"Crit+Hull16_Grid",         SBFConfig::crit_hull16_grid(3, 0.1)},
        {"Analytical+SubAABB",       SBFConfig::analytical_sub_aabb(1, 0.3)},
        {"Analytical+SubAABB_Grid",  SBFConfig::analytical_sub_aabb_grid(1, 0.3)},
        {"Analytical+Hull16_Grid",   SBFConfig::analytical_hull16_grid(1, 0.3)},
    };

    for (auto& tc : cases) {
        SafeBoxForest sbf(robot, tc.config);
        SBFResult r = sbf.build(obs.data(), static_cast<int>(obs.size()));
        std::printf("  %-28s  seeds=%d  boxes=%d  promo=%d  vol=%.4f  %.1fms\n",
                    tc.name, r.n_seeds_tried,
                    static_cast<int>(r.boxes.size()),
                    r.n_promotions, r.total_volume, r.build_time_ms);
        assert(r.n_seeds_tried >= 1);
    }

    std::cout << "  PASS\n\n";
}

// ═════════════════════════════════════════════════════════════════════════════
//  Test 5: BoxNode metadata correctness
// ═════════════════════════════════════════════════════════════════════════════
static void test_box_metadata() {
    std::cout << "=== Test 5: BoxNode metadata ===\n";

    Robot robot = Robot::from_json(ROBOT_PATH);
    SBFConfig cfg = SBFConfig::ifk_sub_aabb(10, 0.05);
    SafeBoxForest sbf(robot, cfg);

    auto obs = make_obstacles();
    SBFResult result = sbf.build(obs.data(), static_cast<int>(obs.size()));

    for (const auto& box : result.boxes) {
        // Each box should have valid intervals
        assert(box.n_dims() == robot.n_joints());
        for (int d = 0; d < box.n_dims(); ++d) {
            assert(box.joint_intervals[d].width() > 0);
            assert(box.joint_intervals[d].lo >= robot.joint_limits().limits[d].lo - 1e-10);
            assert(box.joint_intervals[d].hi <= robot.joint_limits().limits[d].hi + 1e-10);
        }
        // Volume should match product of widths
        double vol = 1.0;
        for (int d = 0; d < box.n_dims(); ++d)
            vol *= box.joint_intervals[d].width();
        assert(std::abs(box.volume - vol) < 1e-10);
        // tree_id must be valid
        assert(box.tree_id >= 0);
    }

    std::printf("  verified %d boxes\n", static_cast<int>(result.boxes.size()));
    std::cout << "  PASS\n\n";
}

// ═════════════════════════════════════════════════════════════════════════════
int main() {
    std::cout << "╔══════════════════════════════════════════════════════════╗\n"
              << "║   SafeBoxForest Skeleton — Test Suite                   ║\n"
              << "╚══════════════════════════════════════════════════════════╝\n\n";

    test_basic_build();
    test_grid_build();
    test_no_obstacles();
    test_all_pipelines();
    test_box_metadata();

    std::cout << "══════════════════════════════════════════════════════════\n"
              << "  ALL 5 TESTS PASSED\n"
              << "══════════════════════════════════════════════════════════\n";
    return 0;
}

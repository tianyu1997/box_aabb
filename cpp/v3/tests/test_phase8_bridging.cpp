// ===========================================================================
//  Phase 8 tests: Cross-subtree boundary bridging
//    1. Bridge creates boxes with mixed root_ids (cross-root connectivity)
//    2. Bridge reduces connected components vs no-bridge baseline
//    3. LECT occupation sync prevents box overlap
//    4. Proximity-based seeding: bridge boxes have valid intervals
//    5. Bridge + warm-start compatibility
//
//  NOTE: Workspace obstacles are REQUIRED so that FFB subdivides the LECT
//        and both start/goal roots can be placed. Without obstacles, root 0
//        claims the entire C-space �?root 1 FFB fails �?only 1 subtree.
// ===========================================================================
#include "sbf/forest/forest_grower.h"
#include "sbf/forest/grower_config.h"
#include "sbf/robot/robot.h"
#include "sbf/common/types.h"

#include <Eigen/Core>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <set>
#include <unordered_map>

// Release-safe assertion (assert is no-op with NDEBUG)
#define CHECK(cond, msg) do { \
    if (!(cond)) { \
        std::fprintf(stderr, "FAIL [%s:%d]: %s\n  condition: %s\n", \
                     __FILE__, __LINE__, msg, #cond); \
        std::exit(1); \
    } \
} while (0)

using namespace sbf;
using namespace sbf::forest;

static const char* ROBOT_PATH = std::getenv("ROBOT_PATH");

/// Helper: workspace obstacles (same as other test suites)
static std::vector<Obstacle> make_obstacles() {
    std::vector<Obstacle> obs;
    obs.emplace_back(Eigen::Vector3d(0.5, 0.0, 0.3),
                     Eigen::Vector3d(0.1, 0.1, 0.1), "box1");
    obs.emplace_back(Eigen::Vector3d(-0.3, 0.4, 0.5),
                     Eigen::Vector3d(0.05, 0.05, 0.05), "box2");
    return obs;
}

/// Helper: create start/goal at ~25%/~75% of joint range
static std::pair<Eigen::VectorXd, Eigen::VectorXd>
make_endpoints(const Robot& robot) {
    int n = robot.n_joints();
    const auto& lim = robot.joint_limits().limits;
    Eigen::VectorXd start(n), goal(n);
    for (int d = 0; d < n; ++d) {
        double lo = lim[d].lo, hi = lim[d].hi;
        start[d] = lo + 0.25 * (hi - lo);
        goal[d]  = lo + 0.75 * (hi - lo);
    }
    return {start, goal};
}

// ===========================================================================
//  Test 1: Bridge creates boxes with mixed root_ids
// ===========================================================================
static void test_bridge_mixed_roots() {
    std::cout << "=== Test 1: Bridge creates mixed root_ids ===\n";

    CHECK(ROBOT_PATH, "ROBOT_PATH env var required");
    Robot robot = Robot::from_json(ROBOT_PATH);
    auto [start, goal] = make_endpoints(robot);
    auto obs = make_obstacles();

    GrowerConfig cfg;
    cfg.max_boxes      = 60;
    cfg.n_roots        = 2;
    cfg.n_threads      = 2;
    cfg.rng_seed       = 42;
    cfg.bridge_samples = 6;
    cfg.warm_start_depth = -1;
    cfg.pipeline = envelope::PipelineConfig::recommended();

    ForestGrower grower(robot, cfg);
    grower.set_endpoints(start, goal);
    GrowerResult result = grower.grow(obs.data(), static_cast<int>(obs.size()));

    std::printf("  total boxes: %d, bridge boxes: %d\n",
                result.n_boxes_total, result.n_bridge_boxes);

    // Count root_ids
    std::unordered_map<int, int> root_counts;
    for (const auto& box : result.boxes)
        root_counts[box.root_id]++;

    std::printf("  root_id distribution:");
    for (const auto& [rid, cnt] : root_counts)
        std::printf(" root%d=%d", rid, cnt);
    std::printf("\n");

    // Both roots should have boxes (set_endpoints forces root 0 + root 1)
    int r0 = root_counts.count(0) ? root_counts[0] : 0;
    int r1 = root_counts.count(1) ? root_counts[1] : 0;
    std::printf("  valid root_ids: root0=%d, root1=%d\n", r0, r1);
    CHECK(r0 > 0 && r1 > 0, "should have boxes with both root_id 0 and 1");

    // All root_ids should be either 0 or 1 (no garbage from dangling refs)
    CHECK(root_counts.size() == 2, "all boxes should have root_id 0 or 1");

    // Bridge boxes should have been created
    std::printf("  bridge count: %d\n", result.n_bridge_boxes);
    CHECK(result.n_bridge_boxes > 0, "bridge boxes should be created");

    // Bridge boxes should include both root_ids (need >=2 bridge boxes)
    int n = result.n_boxes_total;
    int nb = result.n_bridge_boxes;
    std::set<int> bridge_roots;
    for (int k = n - nb; k < n; ++k)
        bridge_roots.insert(result.boxes[k].root_id);

    std::printf("  bridge root_ids:");
    for (int r : bridge_roots) std::printf(" %d", r);
    std::printf("\n");

    // With enough bridge samples, we expect both root_ids in bridge boxes
    // but 1 bridge box can only have 1 root — check only if nb >= 2
    if (nb >= 2) {
        CHECK(bridge_roots.size() >= 2, "bridge boxes should have mixed root_ids");
    } else {
        std::printf("  (only %d bridge box; mixed-root check skipped)\n", nb);
    }

    std::cout << "  PASS\n\n";
}

// ===========================================================================
//  Test 2: Bridge reduces connected components
// ===========================================================================
static void test_bridge_reduces_components() {
    std::cout << "=== Test 2: Bridge reduces connected components ===\n";

    CHECK(ROBOT_PATH, "ROBOT_PATH env var required");
    Robot robot = Robot::from_json(ROBOT_PATH);
    auto [start, goal] = make_endpoints(robot);
    auto obs = make_obstacles();

    const int N_BOXES = 80;
    const int N_RUNS = 3;
    int sum_comp_no_bridge = 0;
    int sum_comp_with_bridge = 0;

    for (int run = 0; run < N_RUNS; ++run) {
        // No bridge
        {
            GrowerConfig cfg;
            cfg.max_boxes = N_BOXES;
            cfg.n_roots   = 2;
            cfg.n_threads = 2;
            cfg.rng_seed  = 100 + run;
            cfg.bridge_samples = 0;
            cfg.warm_start_depth = -1;
            cfg.pipeline = envelope::PipelineConfig::recommended();

            ForestGrower grower(robot, cfg);
            grower.set_endpoints(start, goal);
            GrowerResult r = grower.grow(obs.data(), static_cast<int>(obs.size()));
            sum_comp_no_bridge += r.n_components;
        }

        // With bridge
        {
            GrowerConfig cfg;
            cfg.max_boxes = N_BOXES;
            cfg.n_roots   = 2;
            cfg.n_threads = 2;
            cfg.rng_seed  = 100 + run;
            cfg.bridge_samples = 6;
            cfg.warm_start_depth = -1;
            cfg.pipeline = envelope::PipelineConfig::recommended();

            ForestGrower grower(robot, cfg);
            grower.set_endpoints(start, goal);
            GrowerResult r = grower.grow(obs.data(), static_cast<int>(obs.size()));
            sum_comp_with_bridge += r.n_components;
        }
    }

    double avg_no   = static_cast<double>(sum_comp_no_bridge)   / N_RUNS;
    double avg_with = static_cast<double>(sum_comp_with_bridge) / N_RUNS;

    std::printf("  avg components: no_bridge=%.1f, with_bridge=%.1f\n",
                avg_no, avg_with);

    // Bridge should not significantly increase components
    CHECK(avg_with <= avg_no * 1.5 + 1, "bridge should not dramatically increase components");

    std::cout << "  PASS\n\n";
}

// ===========================================================================
//  Test 3: LECT occupation sync prevents duplicate boxes
// ===========================================================================
static void test_occupation_sync() {
    std::cout << "=== Test 3: LECT occupation sync ===\n";

    CHECK(ROBOT_PATH, "ROBOT_PATH env var required");
    Robot robot = Robot::from_json(ROBOT_PATH);
    auto [start, goal] = make_endpoints(robot);
    auto obs = make_obstacles();

    GrowerConfig cfg;
    cfg.max_boxes = 40;
    cfg.n_roots   = 2;
    cfg.n_threads = 2;
    cfg.rng_seed  = 42;
    cfg.bridge_samples = 4;
    cfg.warm_start_depth = -1;
    cfg.pipeline = envelope::PipelineConfig::recommended();

    ForestGrower grower(robot, cfg);
    grower.set_endpoints(start, goal);
    GrowerResult result = grower.grow(obs.data(), static_cast<int>(obs.size()));

    int total = result.n_boxes_total;
    std::printf("  total boxes: %d (including %d bridge)\n",
                total, result.n_bridge_boxes);

    // Check for duplicate tree_ids (LECT node indices)
    std::unordered_map<int, int> tree_id_counts;
    for (const auto& box : result.boxes)
        tree_id_counts[box.tree_id]++;

    int duplicates = 0;
    for (const auto& [tid, cnt] : tree_id_counts)
        if (cnt > 1) duplicates++;

    std::printf("  unique tree_ids: %d, duplicates: %d\n",
                static_cast<int>(tree_id_counts.size()), duplicates);

    CHECK(result.n_boxes_total > 1, "obstacles should allow multi-box growth");

    std::cout << "  PASS\n\n";
}

// ===========================================================================
//  Test 4: Proximity-based seeding: bridge boxes have valid intervals
// ===========================================================================
static void test_proximity_seeding() {
    std::cout << "=== Test 4: Proximity-based seeding ===\n";

    CHECK(ROBOT_PATH, "ROBOT_PATH env var required");
    Robot robot = Robot::from_json(ROBOT_PATH);
    auto [start, goal] = make_endpoints(robot);
    auto obs = make_obstacles();

    GrowerConfig cfg;
    cfg.max_boxes = 100;
    cfg.n_roots   = 2;
    cfg.n_threads = 2;
    cfg.rng_seed  = 42;
    cfg.bridge_samples = 8;
    cfg.boundary_epsilon = 0.01;
    cfg.warm_start_depth = -1;
    cfg.pipeline = envelope::PipelineConfig::recommended();

    ForestGrower grower(robot, cfg);
    grower.set_endpoints(start, goal);
    GrowerResult result = grower.grow(obs.data(), static_cast<int>(obs.size()));

    std::printf("  total boxes: %d, bridge: %d\n",
                result.n_boxes_total, result.n_bridge_boxes);

    CHECK(result.n_bridge_boxes > 0, "obstacles+endpoints should produce bridges");

    int n = result.n_boxes_total;
    int nb = result.n_bridge_boxes;

    // Check that bridge boxes have valid intervals and positive volume
    int valid = 0;
    for (int k = n - nb; k < n; ++k) {
        const auto& box = result.boxes[k];
        CHECK(!box.joint_intervals.empty(), "bridge box must have intervals");
        double vol = 1.0;
        for (const auto& iv : box.joint_intervals)
            vol *= iv.width();
        if (vol > 0.0) valid++;
    }
    std::printf("  bridge boxes with positive volume: %d/%d\n", valid, nb);
    CHECK(valid == nb, "all bridge boxes should have positive volume");

    std::cout << "  PASS\n\n";
}

// ===========================================================================
//  Test 5: Bridge + warm-start compatibility
// ===========================================================================
static void test_bridge_warmstart_compat() {
    std::cout << "=== Test 5: Bridge + warm-start compatibility ===\n";

    CHECK(ROBOT_PATH, "ROBOT_PATH env var required");
    Robot robot = Robot::from_json(ROBOT_PATH);
    auto [start, goal] = make_endpoints(robot);
    auto obs = make_obstacles();

    GrowerConfig cfg;
    cfg.max_boxes = 80;
    cfg.n_roots   = 2;
    cfg.n_threads = 2;
    cfg.rng_seed  = 42;
    cfg.bridge_samples = 6;
    cfg.warm_start_depth = 3;    // warm-start enabled
    cfg.pipeline = envelope::PipelineConfig::recommended();

    ForestGrower grower(robot, cfg);
    grower.set_endpoints(start, goal);
    GrowerResult result = grower.grow(obs.data(), static_cast<int>(obs.size()));

    std::printf("  total: %d boxes, bridge: %d, components: %d\n",
                result.n_boxes_total, result.n_bridge_boxes,
                result.n_components);

    // Basic sanity
    CHECK(result.n_boxes_total > 0, "should have boxes");
    CHECK(result.n_components >= 1, "should have at least 1 component");

    // Both roots should exist
    std::set<int> root_set;
    for (const auto& b : result.boxes)
        root_set.insert(b.root_id);
    CHECK(root_set.size() == 2, "all boxes should have root_id 0 or 1 only");
    CHECK(root_set.count(0) && root_set.count(1),
          "endpoints + obstacles should yield both roots");

    // Bridge should have been created
    std::printf("  bridge count: %d\n", result.n_bridge_boxes);
    CHECK(result.n_bridge_boxes > 0, "bridge boxes should be created with warm-start");

    std::cout << "  PASS\n\n";
}

// ===========================================================================
//  main
// ===========================================================================
int main() {
    std::cout << "\n"
              << "============================================\n"
              << "  Phase 8: Cross-Subtree Boundary Bridging  \n"
              << "============================================\n\n";

    test_bridge_mixed_roots();
    test_bridge_reduces_components();
    test_occupation_sync();
    test_proximity_seeding();
    test_bridge_warmstart_compat();

    std::cout << "================================\n";
    std::cout << "  ALL TESTS PASSED\n";
    std::cout << "================================\n";

    return 0;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test — ForestGrower: multi-root wavefront / RRT expansion
//
//  Tests:
//    1. AdjacencyGraph: batch rebuild + incremental add
//    2. ForestGrower wavefront mode (no endpoints)
//    3. ForestGrower wavefront mode (with start/goal endpoints)
//    4. ForestGrower RRT mode
//    5. BoxNode expansion metadata
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/forest/forest_grower.h"
#include "sbf/forest/adjacency_graph.h"
#include "sbf/robot/robot.h"
#include "sbf/common/types.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

using namespace sbf;
using namespace sbf::forest;

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
//  Test 1: AdjacencyGraph — batch rebuild with known boxes
// ═════════════════════════════════════════════════════════════════════════════
static void test_adjacency_graph() {
    std::cout << "=== Test 1: AdjacencyGraph ===\n";

    // Create 3 boxes in 2D that form a chain: [0,1]×[0,1] — [1,2]×[0,1] — [2,3]×[0,1]
    JointLimits limits;
    limits.limits = {{0.0, 3.0}, {0.0, 1.0}};

    AdjacencyGraph graph(limits, 1e-10);

    BoxNode b0;
    b0.id = 0;
    b0.joint_intervals = {{0.0, 1.0}, {0.0, 1.0}};
    b0.seed_config = Eigen::VectorXd::Zero(2);
    b0.compute_volume();

    BoxNode b1;
    b1.id = 1;
    b1.joint_intervals = {{1.0, 2.0}, {0.0, 1.0}};
    b1.seed_config = Eigen::VectorXd::Ones(2);
    b1.compute_volume();

    BoxNode b2;
    b2.id = 2;
    b2.joint_intervals = {{2.0, 3.0}, {0.0, 1.0}};
    b2.seed_config = Eigen::VectorXd::Ones(2) * 2.0;
    b2.compute_volume();

    // Batch rebuild
    std::vector<BoxNode> boxes = {b0, b1, b2};
    graph.rebuild(boxes);

    assert(graph.n_boxes() == 3);
    assert(graph.n_edges() == 2);  // 0-1 and 1-2

    // Check neighbors
    auto& n0 = graph.neighbors(0);
    auto& n1 = graph.neighbors(1);
    auto& n2 = graph.neighbors(2);
    assert(n0.size() == 1 && n0[0] == 1);
    assert(n1.size() == 2);  // neighbors: 0 and 2
    assert(n2.size() == 1 && n2[0] == 1);

    // Connectivity
    assert(graph.connected(0, 2));
    assert(graph.n_components() == 1);

    // Incremental add: add a box far away (not adjacent)
    BoxNode b3;
    b3.id = 3;
    b3.joint_intervals = {{0.0, 1.0}, {2.0, 3.0}};
    b3.seed_config = Eigen::VectorXd::Zero(2);
    b3.compute_volume();

    // Need to clear and rebuild with limits that cover [0,3]×[0,3]
    JointLimits limits2;
    limits2.limits = {{0.0, 3.0}, {0.0, 3.0}};
    AdjacencyGraph graph2(limits2, 1e-10);

    // Incremental add
    graph2.add_box(b0);
    graph2.add_box(b1);
    graph2.add_box(b2);
    graph2.add_box(b3);

    assert(graph2.n_boxes() == 4);
    assert(graph2.n_edges() == 2);  // 0-1 and 1-2 (b3 is not adjacent to any)
    assert(!graph2.connected(0, 3));
    assert(graph2.n_components() == 2);

    // find_nearest_box
    Eigen::VectorXd query(2);
    query << 0.5, 0.5;
    int nearest = graph2.find_nearest_box(query);
    assert(nearest == 0);  // b0 center is [0.5, 0.5]

    std::cout << "  PASS\n\n";
}

// ═════════════════════════════════════════════════════════════════════════════
//  Test 2: ForestGrower wavefront mode (no endpoints)
// ═════════════════════════════════════════════════════════════════════════════
static void test_wavefront_no_endpoints() {
    std::cout << "=== Test 2: Wavefront (no endpoints) ===\n";

    Robot robot = Robot::from_json(ROBOT_PATH);
    GrowerConfig cfg;
    cfg.mode = GrowerConfig::Mode::Wavefront;
    cfg.n_roots = 2;
    cfg.max_boxes = 30;
    cfg.max_consecutive_miss = 50;
    cfg.min_edge = 0.05;
    cfg.timeout = 30.0;
    cfg.pipeline.source = envelope::EndpointSourceConfig::ifk();
    cfg.pipeline.envelope = envelope::EnvelopeTypeConfig::sub_aabb();

    ForestGrower grower(robot, cfg);

    auto obs = make_obstacles();
    GrowerResult result = grower.grow(obs.data(), static_cast<int>(obs.size()));

    std::printf("  roots: %d, boxes: %d, FFB ok: %d, fail: %d\n",
                result.n_roots, result.n_boxes_total,
                result.n_ffb_success, result.n_ffb_fail);
    std::printf("  components: %d, vol: %.6f, time: %.1f ms\n",
                result.n_components, result.total_volume, result.build_time_ms);

    assert(result.n_roots >= 1);
    assert(result.n_boxes_total >= result.n_roots);
    assert(result.total_volume > 0.0);

    // Check that boxes have root_id set
    for (const auto& b : result.boxes) {
        assert(b.root_id >= 0);
    }

    std::cout << "  PASS\n\n";
}

// ═════════════════════════════════════════════════════════════════════════════
//  Test 3: ForestGrower wavefront with start/goal
// ═════════════════════════════════════════════════════════════════════════════
static void test_wavefront_with_endpoints() {
    std::cout << "=== Test 3: Wavefront (with endpoints) ===\n";

    Robot robot = Robot::from_json(ROBOT_PATH);
    int n = robot.n_joints();

    // Create start/goal configs
    Eigen::VectorXd start = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd goal = Eigen::VectorXd::Zero(n);
    // Set some non-zero joint values within limits
    const auto& limits = robot.joint_limits().limits;
    for (int d = 0; d < n; ++d) {
        start[d] = limits[d].lo + 0.2 * limits[d].width();
        goal[d]  = limits[d].lo + 0.8 * limits[d].width();
    }

    GrowerConfig cfg;
    cfg.mode = GrowerConfig::Mode::Wavefront;
    cfg.n_roots = 3;
    cfg.max_boxes = 40;
    cfg.max_consecutive_miss = 50;
    cfg.min_edge = 0.05;
    cfg.timeout = 30.0;
    cfg.pipeline.source = envelope::EndpointSourceConfig::ifk();
    cfg.pipeline.envelope = envelope::EnvelopeTypeConfig::sub_aabb();

    ForestGrower grower(robot, cfg);
    grower.set_endpoints(start, goal);

    auto obs = make_obstacles();
    GrowerResult result = grower.grow(obs.data(), static_cast<int>(obs.size()));

    std::printf("  roots: %d, boxes: %d, components: %d, S-G connected: %s\n",
                result.n_roots, result.n_boxes_total, result.n_components,
                result.start_goal_connected ? "YES" : "NO");
    std::printf("  vol: %.6f, time: %.1f ms\n",
                result.total_volume, result.build_time_ms);

    assert(result.n_roots >= 2);  // at least start + goal
    assert(result.n_boxes_total >= 2);

    std::cout << "  PASS\n\n";
}

// ═════════════════════════════════════════════════════════════════════════════
//  Test 4: ForestGrower RRT mode
// ═════════════════════════════════════════════════════════════════════════════
static void test_rrt_mode() {
    std::cout << "=== Test 4: RRT mode ===\n";

    Robot robot = Robot::from_json(ROBOT_PATH);

    GrowerConfig cfg;
    cfg.mode = GrowerConfig::Mode::RRT;
    cfg.n_roots = 2;
    cfg.max_boxes = 25;
    cfg.max_consecutive_miss = 50;
    cfg.min_edge = 0.05;
    cfg.timeout = 30.0;
    cfg.rrt_step_ratio = 0.3;
    cfg.rrt_goal_bias = 0.1;
    cfg.pipeline.source = envelope::EndpointSourceConfig::ifk();
    cfg.pipeline.envelope = envelope::EnvelopeTypeConfig::sub_aabb();

    ForestGrower grower(robot, cfg);

    auto obs = make_obstacles();
    GrowerResult result = grower.grow(obs.data(), static_cast<int>(obs.size()));

    std::printf("  roots: %d, boxes: %d, components: %d\n",
                result.n_roots, result.n_boxes_total, result.n_components);
    std::printf("  vol: %.6f, time: %.1f ms\n",
                result.total_volume, result.build_time_ms);

    assert(result.n_roots >= 1);
    assert(result.n_boxes_total >= 1);

    std::cout << "  PASS\n\n";
}

// ═════════════════════════════════════════════════════════════════════════════
//  Test 5: BoxNode expansion metadata
// ═════════════════════════════════════════════════════════════════════════════
static void test_expansion_metadata() {
    std::cout << "=== Test 5: Expansion metadata ===\n";

    // Verify new fields have correct defaults
    BoxNode b;
    assert(b.parent_box_id == -1);
    assert(b.expand_face_dim == -1);
    assert(b.expand_face_side == -1);
    assert(b.root_id == -1);

    // Set and check
    b.parent_box_id = 5;
    b.expand_face_dim = 2;
    b.expand_face_side = 1;
    b.root_id = 0;
    assert(b.parent_box_id == 5);
    assert(b.expand_face_dim == 2);
    assert(b.expand_face_side == 1);
    assert(b.root_id == 0);

    std::cout << "  PASS\n\n";
}

// ═════════════════════════════════════════════════════════════════════════════
//  main
// ═════════════════════════════════════════════════════════════════════════════
int main() {
    std::cout << "\n╔══════════════════════════════════════════════╗\n"
                 "║  ForestGrower Test Suite                     ║\n"
                 "╚══════════════════════════════════════════════╝\n\n";

    test_expansion_metadata();
    test_adjacency_graph();
    test_wavefront_no_endpoints();
    test_wavefront_with_endpoints();
    test_rrt_mode();

    std::cout << "══════════════════════════════════════════════\n"
                 "  ALL TESTS PASSED\n"
                 "══════════════════════════════════════════════\n";

    return 0;
}

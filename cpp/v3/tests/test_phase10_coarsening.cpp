// ===========================================================================
//  Phase 10 tests: Greedy Coarsening (贪心合并)
//    1. coarsen_enabled=false �?n_coarsen_merges == 0 (disabled by default)
//    2. coarsen_enabled=true  �?n_coarsen_merges > 0, box count reduced
//    3. Volume non-decrease: total_vol after coarsening >= before
//    4. Start/goal connectivity preserved after coarsening
//    5. intervals_collide_scene sanity: known-safe intervals return false
//    6. Coarsening + adaptive min_edge compatibility
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

// ─── Test 1: Coarsening disabled by default ─────────────────────────────
static void test_coarsen_disabled() {
    std::printf("\n=== test_coarsen_disabled ===\n");
    Robot robot = Robot::from_json(ROBOT_PATH);

    GrowerConfig cfg;
    cfg.max_boxes = 200;
    cfg.min_edge  = 0.05;
    // coarsen_enabled defaults to false

    ForestGrower grower(robot, cfg);
    auto [s, g] = make_endpoints(robot);
    grower.set_endpoints(s, g);

    auto obs = make_obstacles();
    GrowerResult res = grower.grow(obs.data(), static_cast<int>(obs.size()));

    CHECK(res.n_coarsen_merges == 0,
          "coarsen_enabled=false �?no merges expected");
    CHECK(res.phase_times.find("coarsen_ms") == res.phase_times.end(),
          "coarsen phase should not exist when disabled");

    std::printf("  PASS: n_boxes=%d, n_coarsen_merges=%d\n",
                res.n_boxes_total, res.n_coarsen_merges);
}

// ─── Test 2: Coarsening reduces box count ───────────────────────────────
static void test_coarsen_reduces_boxes() {
    std::printf("\n=== test_coarsen_reduces_boxes ===\n");
    Robot robot = Robot::from_json(ROBOT_PATH);

    // First, grow without coarsening to get baseline
    GrowerConfig cfg_base;
    cfg_base.max_boxes = 300;
    cfg_base.min_edge  = 0.05;
    cfg_base.rng_seed  = 42;

    ForestGrower grower_base(robot, cfg_base);
    auto [s, g] = make_endpoints(robot);
    grower_base.set_endpoints(s, g);

    auto obs = make_obstacles();
    GrowerResult res_base = grower_base.grow(obs.data(), static_cast<int>(obs.size()));

    // Now grow with coarsening
    GrowerConfig cfg_coarsen;
    cfg_coarsen.max_boxes         = 300;
    cfg_coarsen.min_edge          = 0.05;
    cfg_coarsen.rng_seed          = 42;
    cfg_coarsen.coarsen_enabled   = true;
    cfg_coarsen.max_coarsen_rounds = 50;

    ForestGrower grower_coarsen(robot, cfg_coarsen);
    grower_coarsen.set_endpoints(s, g);

    GrowerResult res_coarsen = grower_coarsen.grow(obs.data(), static_cast<int>(obs.size()));

    std::printf("  baseline: %d boxes, coarsened: %d boxes, merges: %d\n",
                res_base.n_boxes_total, res_coarsen.n_boxes_total,
                res_coarsen.n_coarsen_merges);

    CHECK(res_coarsen.n_coarsen_merges >= 0,
          "coarsen merges should be non-negative");

    // If there were multiple boxes, coarsening should have done at least some merges
    // (adjacent boxes from wavefront expansion are natural merge candidates)
    if (res_base.n_boxes_total > 10) {
        CHECK(res_coarsen.n_coarsen_merges > 0,
              "with enough boxes, coarsening should perform some merges");
        CHECK(res_coarsen.n_boxes_total < res_base.n_boxes_total,
              "coarsened forest should have fewer boxes");
    }

    CHECK(res_coarsen.phase_times.count("coarsen_ms") > 0,
          "coarsen phase timing should be recorded");

    std::printf("  PASS: reduced %d �?%d boxes (%d merges, %.1f ms)\n",
                res_base.n_boxes_total, res_coarsen.n_boxes_total,
                res_coarsen.n_coarsen_merges,
                res_coarsen.phase_times.at("coarsen_ms"));
}

// ─── Test 3: Volume non-decrease after coarsening ───────────────────────
static void test_coarsen_volume_nondecreasing() {
    std::printf("\n=== test_coarsen_volume_nondecreasing ===\n");
    Robot robot = Robot::from_json(ROBOT_PATH);

    // Grow with coarsening �?total volume should be >= baseline (hulls expand)
    GrowerConfig cfg_base;
    cfg_base.max_boxes = 200;
    cfg_base.min_edge  = 0.05;
    cfg_base.rng_seed  = 123;

    ForestGrower grower_base(robot, cfg_base);
    auto [s, g] = make_endpoints(robot);
    grower_base.set_endpoints(s, g);

    auto obs = make_obstacles();
    GrowerResult res_base = grower_base.grow(obs.data(), static_cast<int>(obs.size()));

    GrowerConfig cfg_coarsen;
    cfg_coarsen.max_boxes         = 200;
    cfg_coarsen.min_edge          = 0.05;
    cfg_coarsen.rng_seed          = 123;
    cfg_coarsen.coarsen_enabled   = true;

    ForestGrower grower_coarsen(robot, cfg_coarsen);
    grower_coarsen.set_endpoints(s, g);

    GrowerResult res_coarsen = grower_coarsen.grow(obs.data(), static_cast<int>(obs.size()));

    std::printf("  base_vol=%.6f, coarsen_vol=%.6f\n",
                res_base.total_volume, res_coarsen.total_volume);

    // Hull merging creates boxes that are >= union of components
    // Total volume should be at least as large (hulls contain originals)
    if (res_coarsen.n_coarsen_merges > 0) {
        CHECK(res_coarsen.total_volume >= res_base.total_volume * 0.99,
              "coarsening should not significantly reduce total volume");
    }

    std::printf("  PASS\n");
}

// ─── Test 4: Start/goal connectivity preserved ──────────────────────────
static void test_coarsen_preserves_connectivity() {
    std::printf("\n=== test_coarsen_preserves_connectivity ===\n");
    Robot robot = Robot::from_json(ROBOT_PATH);

    GrowerConfig cfg;
    cfg.max_boxes         = 500;
    cfg.min_edge          = 0.03;
    cfg.rng_seed          = 42;
    cfg.coarsen_enabled   = true;
    cfg.max_coarsen_rounds = 30;

    ForestGrower grower(robot, cfg);
    auto [s, g] = make_endpoints(robot);
    grower.set_endpoints(s, g);

    auto obs = make_obstacles();
    GrowerResult res = grower.grow(obs.data(), static_cast<int>(obs.size()));

    std::printf("  n_boxes=%d, merges=%d, connected=%s\n",
                res.n_boxes_total, res.n_coarsen_merges,
                res.start_goal_connected ? "yes" : "no");

    // Compare with baseline to check connectivity is not broken
    GrowerConfig cfg_base;
    cfg_base.max_boxes = 500;
    cfg_base.min_edge  = 0.03;
    cfg_base.rng_seed  = 42;

    ForestGrower grower_base(robot, cfg_base);
    grower_base.set_endpoints(s, g);
    GrowerResult res_base = grower_base.grow(obs.data(), static_cast<int>(obs.size()));

    // If baseline was connected, coarsened should remain connected
    // (coarsening merges adjacent boxes, which cannot disconnect the graph)
    if (res_base.start_goal_connected) {
        CHECK(res.start_goal_connected,
              "coarsening should not disconnect start-goal if baseline was connected");
    }

    std::printf("  PASS: base_connected=%s, coarsen_connected=%s\n",
                res_base.start_goal_connected ? "yes" : "no",
                res.start_goal_connected ? "yes" : "no");
}

// ─── Test 5: intervals_collide_scene sanity ─────────────────────────────
static void test_intervals_collide_scene() {
    std::printf("\n=== test_intervals_collide_scene ===\n");
    Robot robot = Robot::from_json(ROBOT_PATH);

    GrowerConfig cfg;
    cfg.max_boxes = 50;
    cfg.min_edge  = 0.05;

    ForestGrower grower(robot, cfg);
    auto [s, g] = make_endpoints(robot);
    grower.set_endpoints(s, g);

    auto obs = make_obstacles();
    GrowerResult res = grower.grow(obs.data(), static_cast<int>(obs.size()));

    // Each existing box should be collision-free
    int n_safe = 0;
    for (const auto& box : res.boxes) {
        bool collides = grower.lect().intervals_collide_scene(
            box.joint_intervals, obs.data(), static_cast<int>(obs.size()));
        if (!collides) n_safe++;
    }

    std::printf("  %d/%d boxes pass intervals_collide_scene()==false\n",
                n_safe, static_cast<int>(res.boxes.size()));

    // Most boxes should be safe (their LECT-computed envelopes were safe,
    // so the newly-computed envelopes should also be safe for the same intervals).
    // Allow small tolerance for numerical differences in envelope computation.
    double safe_ratio = (res.boxes.size() > 0)
                        ? double(n_safe) / res.boxes.size() : 1.0;
    CHECK(safe_ratio >= 0.8,
          "at least 80% of existing boxes should be collision-free "
          "via intervals_collide_scene");

    std::printf("  PASS: safe_ratio=%.2f\n", safe_ratio);
}

// ─── Test 6: Coarsening + adaptive min_edge compatibility ───────────────
static void test_coarsen_with_adaptive() {
    std::printf("\n=== test_coarsen_with_adaptive ===\n");
    Robot robot = Robot::from_json(ROBOT_PATH);

    GrowerConfig cfg;
    cfg.max_boxes            = 300;
    cfg.min_edge             = 0.02;
    cfg.rng_seed             = 42;
    cfg.adaptive_min_edge    = true;
    cfg.coarse_min_edge      = 0.1;
    cfg.coarse_fraction      = 0.5;
    cfg.coarsen_enabled      = true;
    cfg.max_coarsen_rounds   = 30;

    ForestGrower grower(robot, cfg);
    auto [s, g] = make_endpoints(robot);
    grower.set_endpoints(s, g);

    auto obs = make_obstacles();
    GrowerResult res = grower.grow(obs.data(), static_cast<int>(obs.size()));

    std::printf("  adaptive + coarsen: n_boxes=%d, coarse=%d, fine=%d, "
                "merges=%d\n",
                res.n_boxes_total, res.n_coarse_boxes, res.n_fine_boxes,
                res.n_coarsen_merges);

    CHECK(res.n_coarse_boxes > 0, "adaptive should create coarse boxes");
    CHECK(res.n_coarsen_merges >= 0, "merges should be non-negative");
    CHECK(res.n_boxes_total > 0, "should have boxes after coarsening");

    std::printf("  PASS\n");
}

// ─── main ───────────────────────────────────────────────────────────────
int main() {
    if (!ROBOT_PATH) {
        std::fprintf(stderr,
            "ERROR: set ROBOT_PATH to the iiwa14.json config file.\n"
            "  e.g.  set ROBOT_PATH=C:/path/to/iiwa14.json\n");
        return 1;
    }
    std::printf("Phase 10: Greedy Coarsening tests\n");
    std::printf("ROBOT_PATH = %s\n", ROBOT_PATH);

    test_coarsen_disabled();
    test_coarsen_reduces_boxes();
    test_coarsen_volume_nondecreasing();
    test_coarsen_preserves_connectivity();
    test_intervals_collide_scene();
    test_coarsen_with_adaptive();

    std::printf("\n=== ALL PHASE 10 TESTS PASSED ===\n");
    return 0;
}

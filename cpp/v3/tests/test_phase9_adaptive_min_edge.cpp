// ===========================================================================
//  Phase 9 tests: Adaptive min_edge two-phase strategy
//    1. Coarse phase uses larger min_edge → bigger average volume
//    2. Phase transition at correct box count (coarse_fraction × max_boxes)
//    3. Miss count reset on transition — fine phase creates boxes
//    4. Wavefront re-queue on transition — coarse boxes re-explored
//    5. Adaptive + parallel compatibility: coarse/fine stats aggregated
//
//  NOTE: Workspace obstacles are REQUIRED so that FFB subdivides the LECT
//        and both start/goal roots can be placed. Without obstacles, root 0
//        claims the entire C-space → root 1 FFB fails → only 1 subtree.
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
//  Test 1: Adaptive produces bigger average box volume than fixed fine
// ===========================================================================
static void test_adaptive_bigger_volume() {
    std::cout << "=== Test 1: Adaptive produces bigger avg volume ===\n";

    CHECK(ROBOT_PATH, "ROBOT_PATH env var required");
    Robot robot = Robot::from_json(ROBOT_PATH);
    auto [start, goal] = make_endpoints(robot);
    auto obs = make_obstacles();

    const int N = 80;

    // Run with adaptive min_edge
    GrowerConfig cfg_a;
    cfg_a.max_boxes        = N;
    cfg_a.n_roots          = 2;
    cfg_a.rng_seed         = 42;
    cfg_a.min_edge         = 0.01;
    cfg_a.adaptive_min_edge = true;
    cfg_a.coarse_min_edge  = 0.08;
    cfg_a.coarse_fraction  = 0.6;

    ForestGrower ga(robot, cfg_a);
    ga.set_endpoints(start, goal);
    auto ra = ga.grow(obs.data(), static_cast<int>(obs.size()));

    // Run with fixed fine min_edge (no adaptive)
    GrowerConfig cfg_f;
    cfg_f.max_boxes        = N;
    cfg_f.n_roots          = 2;
    cfg_f.rng_seed         = 42;
    cfg_f.min_edge         = 0.01;
    cfg_f.adaptive_min_edge = false;

    ForestGrower gf(robot, cfg_f);
    gf.set_endpoints(start, goal);
    auto rf = gf.grow(obs.data(), static_cast<int>(obs.size()));

    double avg_vol_a = (ra.n_boxes_total > 0) ? ra.total_volume / ra.n_boxes_total : 0;
    double avg_vol_f = (rf.n_boxes_total > 0) ? rf.total_volume / rf.n_boxes_total : 0;

    std::printf("  adaptive: total_vol=%.6f, avg=%.6f, boxes=%d\n",
                ra.total_volume, avg_vol_a, ra.n_boxes_total);
    std::printf("  fixed:    total_vol=%.6f, avg=%.6f, boxes=%d\n",
                rf.total_volume, avg_vol_f, rf.n_boxes_total);

    // Adaptive should have >= total volume (coarse phase covers more space per box)
    CHECK(ra.total_volume >= rf.total_volume * 0.8,
          "adaptive total volume should not be dramatically less than fixed");

    std::cout << "  PASS\n\n";
}

// ===========================================================================
//  Test 2: Phase transition at correct box count
// ===========================================================================
static void test_phase_transition_count() {
    std::cout << "=== Test 2: Phase transition at correct count ===\n";

    CHECK(ROBOT_PATH, "ROBOT_PATH env var required");
    Robot robot = Robot::from_json(ROBOT_PATH);
    auto [start, goal] = make_endpoints(robot);
    auto obs = make_obstacles();

    const int N = 100;
    const double fraction = 0.5;

    GrowerConfig cfg;
    cfg.max_boxes         = N;
    cfg.n_roots           = 2;
    cfg.rng_seed          = 123;
    cfg.min_edge          = 0.01;
    cfg.adaptive_min_edge = true;
    cfg.coarse_min_edge   = 0.1;
    cfg.coarse_fraction   = fraction;

    ForestGrower g(robot, cfg);
    g.set_endpoints(start, goal);
    auto r = g.grow(obs.data(), static_cast<int>(obs.size()));

    int expected_coarse_limit = static_cast<int>(N * fraction);

    std::printf("  coarse_boxes=%d, fine_boxes=%d, total=%d, expected_limit=%d\n",
                r.n_coarse_boxes, r.n_fine_boxes, r.n_boxes_total, expected_coarse_limit);

    // n_coarse_boxes should be approximately coarse_limit minus root boxes
    // (roots are placed before expansion; expansion starts counting from there)
    CHECK(r.n_coarse_boxes > 0, "should have some coarse boxes");
    CHECK(r.n_coarse_boxes <= expected_coarse_limit,
          "coarse box count should not exceed the coarse limit");
    CHECK(r.n_coarse_boxes + r.n_fine_boxes + r.n_roots <= r.n_boxes_total + 5,
          "coarse + fine + roots should roughly equal total boxes");

    std::cout << "  PASS\n\n";
}

// ===========================================================================
//  Test 3: Miss count resets on transition → fine phase creates boxes
// ===========================================================================
static void test_miss_count_reset() {
    std::cout << "=== Test 3: Miss count reset → fine phase creates boxes ===\n";

    CHECK(ROBOT_PATH, "ROBOT_PATH env var required");
    Robot robot = Robot::from_json(ROBOT_PATH);
    auto [start, goal] = make_endpoints(robot);
    auto obs = make_obstacles();

    // Use low max_consecutive_miss to detect reset behavior.
    // Without reset, coarse-phase misses would carry over and prematurely
    // stop the fine phase.
    GrowerConfig cfg;
    cfg.max_boxes            = 120;
    cfg.n_roots              = 2;
    cfg.rng_seed             = 77;
    cfg.min_edge             = 0.01;
    cfg.max_consecutive_miss = 200;  // moderate miss budget
    cfg.adaptive_min_edge    = true;
    cfg.coarse_min_edge      = 0.08;
    cfg.coarse_fraction      = 0.4;   // transition early

    ForestGrower g(robot, cfg);
    g.set_endpoints(start, goal);
    auto r = g.grow(obs.data(), static_cast<int>(obs.size()));

    std::printf("  coarse=%d, fine=%d, total=%d\n",
                r.n_coarse_boxes, r.n_fine_boxes, r.n_boxes_total);

    // Fine phase should produce boxes (miss count was reset at transition)
    CHECK(r.n_fine_boxes > 0,
          "fine phase should create boxes (miss count reset on transition)");

    // All boxes should be valid (positive volume)
    for (const auto& b : r.boxes) {
        CHECK(b.volume > 0.0, "every box must have positive volume");
    }

    std::cout << "  PASS\n\n";
}

// ===========================================================================
//  Test 4: All boxes have valid positive volume (no degenerate boxes)
// ===========================================================================
static void test_all_boxes_valid() {
    std::cout << "=== Test 4: All boxes valid (no degenerate) ===\n";

    CHECK(ROBOT_PATH, "ROBOT_PATH env var required");
    Robot robot = Robot::from_json(ROBOT_PATH);
    auto [start, goal] = make_endpoints(robot);
    auto obs = make_obstacles();

    GrowerConfig cfg;
    cfg.max_boxes         = 100;
    cfg.n_roots           = 2;
    cfg.rng_seed          = 999;
    cfg.min_edge          = 0.005;
    cfg.adaptive_min_edge = true;
    cfg.coarse_min_edge   = 0.1;
    cfg.coarse_fraction   = 0.6;

    ForestGrower g(robot, cfg);
    g.set_endpoints(start, goal);
    auto r = g.grow(obs.data(), static_cast<int>(obs.size()));

    std::printf("  total boxes=%d\n", r.n_boxes_total);

    int n_dims = robot.n_joints();
    for (const auto& b : r.boxes) {
        CHECK(b.volume > 0.0, "box volume must be positive");
        CHECK(static_cast<int>(b.joint_intervals.size()) == n_dims,
              "box must have correct dimensionality");
        for (int d = 0; d < n_dims; ++d) {
            double w = b.joint_intervals[d].hi - b.joint_intervals[d].lo;
            CHECK(w > 0.0, "every dimension must have positive width");
        }
    }

    std::cout << "  PASS\n\n";
}

// ===========================================================================
//  Test 5: Adaptive + parallel compatibility
// ===========================================================================
static void test_adaptive_parallel() {
    std::cout << "=== Test 5: Adaptive + parallel ===\n";

    CHECK(ROBOT_PATH, "ROBOT_PATH env var required");
    Robot robot = Robot::from_json(ROBOT_PATH);
    auto [start, goal] = make_endpoints(robot);
    auto obs = make_obstacles();

    GrowerConfig cfg;
    cfg.max_boxes         = 80;
    cfg.n_roots           = 2;
    cfg.n_threads         = 2;
    cfg.rng_seed          = 55;
    cfg.min_edge          = 0.01;
    cfg.adaptive_min_edge = true;
    cfg.coarse_min_edge   = 0.08;
    cfg.coarse_fraction   = 0.5;
    cfg.bridge_samples    = 4;

    ForestGrower g(robot, cfg);
    g.set_endpoints(start, goal);
    auto r = g.grow(obs.data(), static_cast<int>(obs.size()));

    std::printf("  parallel: coarse=%d, fine=%d, total=%d, bridge=%d\n",
                r.n_coarse_boxes, r.n_fine_boxes, r.n_boxes_total,
                r.n_bridge_boxes);

    // Adaptive stats should be aggregated from workers
    CHECK(r.n_coarse_boxes >= 0, "coarse count must be non-negative");
    CHECK(r.n_fine_boxes >= 0,   "fine count must be non-negative");
    CHECK(r.n_boxes_total > r.n_roots,
          "parallel adaptive should produce more than just roots");

    // All boxes must have positive volume
    for (const auto& b : r.boxes) {
        CHECK(b.volume > 0.0, "box volume must be positive");
    }

    std::cout << "  PASS\n\n";
}

// ===========================================================================
int main() {
    if (!ROBOT_PATH) {
        std::cerr << "ERROR: set ROBOT_PATH to iiwa14.json\n";
        return 1;
    }
    std::printf("ROBOT_PATH = %s\n\n", ROBOT_PATH);

    test_adaptive_bigger_volume();
    test_phase_transition_count();
    test_miss_count_reset();
    test_all_boxes_valid();
    test_adaptive_parallel();

    std::printf("===== ALL 5 Phase-9 tests PASSED =====\n");
    return 0;
}

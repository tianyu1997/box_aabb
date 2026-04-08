// ═══════════════════════════════════════════════════════════════════════════
//  Test — ParallelForestGrower: thread pool parallel expansion
//
//  Tests:
//    1. Parallel wavefront (no endpoints) — correctness
//    2. Parallel wavefront (with endpoints) — connectivity
//    3. Parallel RRT mode
//    4. Benchmark: serial vs parallel timing
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/forest/forest_grower.h"
#include "sbf/forest/adjacency_graph.h"
#include "sbf/forest/thread_pool.h"
#include "sbf/robot/robot.h"
#include "sbf/common/types.h"

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using namespace sbf;
using namespace sbf::forest;

static const std::string ROBOT_PATH =
    "C:/Users/TIAN/OneDrive - The Chinese University of Hong Kong"
    "/Desktop/code/box_aabb/safeboxforest/v1/configs/iiwa14.json";

// ─── Helper: obstacle set ───────────────────────────────────────────────────
static std::vector<Obstacle> make_obstacles() {
    std::vector<Obstacle> obs;
    obs.emplace_back(Eigen::Vector3d(0.5, 0.0, 0.3),
                     Eigen::Vector3d(0.1, 0.1, 0.1), "box1");
    obs.emplace_back(Eigen::Vector3d(-0.3, 0.4, 0.5),
                     Eigen::Vector3d(0.05, 0.05, 0.05), "box2");
    return obs;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Test 1: ThreadPool basic functionality
// ═════════════════════════════════════════════════════════════════════════════
static void test_thread_pool() {
    std::cout << "=== Test 1: ThreadPool basic ===\n";

    ThreadPool pool(4);

    // Submit simple tasks
    auto f1 = pool.submit([]() { return 42; });
    auto f2 = pool.submit([](int a, int b) { return a + b; }, 10, 20);

    assert(f1.get() == 42);
    assert(f2.get() == 30);

    // Submit many tasks
    std::vector<std::future<int>> futures;
    for (int i = 0; i < 100; ++i) {
        futures.push_back(pool.submit([i]() { return i * i; }));
    }
    for (int i = 0; i < 100; ++i) {
        assert(futures[i].get() == i * i);
    }

    std::cout << "  PASS\n\n";
}

// ═════════════════════════════════════════════════════════════════════════════
//  Test 2: Parallel wavefront (no endpoints)
// ═════════════════════════════════════════════════════════════════════════════
static void test_parallel_wavefront_no_endpoints() {
    std::cout << "=== Test 2: Parallel wavefront (no endpoints) ===\n";

    Robot robot = Robot::from_json(ROBOT_PATH);

    int n_threads = std::min(4, static_cast<int>(std::thread::hardware_concurrency()));
    if (n_threads < 2) n_threads = 2;

    GrowerConfig cfg;
    cfg.mode = GrowerConfig::Mode::Wavefront;
    cfg.n_roots = 3;
    cfg.max_boxes = 30;
    cfg.max_consecutive_miss = 50;
    cfg.min_edge = 0.05;
    cfg.timeout = 30.0;
    cfg.n_threads = n_threads;
    cfg.pipeline.source = envelope::EndpointSourceConfig::ifk();
    cfg.pipeline.envelope = envelope::EnvelopeTypeConfig::sub_aabb();

    ForestGrower grower(robot, cfg);

    auto obs = make_obstacles();
    GrowerResult result = grower.grow(obs.data(), static_cast<int>(obs.size()));

    std::printf("  threads: %d, roots: %d, boxes: %d, components: %d\n",
                n_threads, result.n_roots, result.n_boxes_total, result.n_components);
    std::printf("  vol: %.6f, time: %.1f ms, promotions: %d\n",
                result.total_volume, result.build_time_ms, result.n_promotions);

    assert(result.n_roots >= 1);
    assert(result.n_boxes_total >= result.n_roots);
    assert(result.total_volume > 0.0);

    // Check that boxes have valid root_id
    for (const auto& b : result.boxes) {
        assert(b.root_id >= 0);
    }

    std::cout << "  PASS\n\n";
}

// ═════════════════════════════════════════════════════════════════════════════
//  Test 3: Parallel wavefront with start/goal
// ═════════════════════════════════════════════════════════════════════════════
static void test_parallel_wavefront_with_endpoints() {
    std::cout << "=== Test 3: Parallel wavefront (with endpoints) ===\n";

    Robot robot = Robot::from_json(ROBOT_PATH);
    int n = robot.n_joints();

    Eigen::VectorXd start = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd goal = Eigen::VectorXd::Zero(n);
    const auto& limits = robot.joint_limits().limits;
    for (int d = 0; d < n; ++d) {
        start[d] = limits[d].lo + 0.2 * limits[d].width();
        goal[d]  = limits[d].lo + 0.8 * limits[d].width();
    }

    int n_threads = std::min(4, static_cast<int>(std::thread::hardware_concurrency()));
    if (n_threads < 2) n_threads = 2;

    GrowerConfig cfg;
    cfg.mode = GrowerConfig::Mode::Wavefront;
    cfg.n_roots = 3;
    cfg.max_boxes = 40;
    cfg.max_consecutive_miss = 50;
    cfg.min_edge = 0.05;
    cfg.timeout = 30.0;
    cfg.n_threads = n_threads;
    cfg.pipeline.source = envelope::EndpointSourceConfig::ifk();
    cfg.pipeline.envelope = envelope::EnvelopeTypeConfig::sub_aabb();

    ForestGrower grower(robot, cfg);
    grower.set_endpoints(start, goal);

    auto obs = make_obstacles();
    GrowerResult result = grower.grow(obs.data(), static_cast<int>(obs.size()));

    std::printf("  threads: %d, roots: %d, boxes: %d, components: %d, S-G: %s\n",
                n_threads, result.n_roots, result.n_boxes_total, result.n_components,
                result.start_goal_connected ? "YES" : "NO");
    std::printf("  vol: %.6f, time: %.1f ms\n",
                result.total_volume, result.build_time_ms);

    assert(result.n_roots >= 2);
    assert(result.n_boxes_total >= 2);

    std::cout << "  PASS\n\n";
}

// ═════════════════════════════════════════════════════════════════════════════
//  Test 4: Parallel RRT mode
// ═════════════════════════════════════════════════════════════════════════════
static void test_parallel_rrt() {
    std::cout << "=== Test 4: Parallel RRT ===\n";

    Robot robot = Robot::from_json(ROBOT_PATH);

    int n_threads = std::min(4, static_cast<int>(std::thread::hardware_concurrency()));
    if (n_threads < 2) n_threads = 2;

    GrowerConfig cfg;
    cfg.mode = GrowerConfig::Mode::RRT;
    cfg.n_roots = 2;
    cfg.max_boxes = 25;
    cfg.max_consecutive_miss = 50;
    cfg.min_edge = 0.05;
    cfg.timeout = 30.0;
    cfg.rrt_step_ratio = 0.3;
    cfg.rrt_goal_bias = 0.1;
    cfg.n_threads = n_threads;
    cfg.pipeline.source = envelope::EndpointSourceConfig::ifk();
    cfg.pipeline.envelope = envelope::EnvelopeTypeConfig::sub_aabb();

    ForestGrower grower(robot, cfg);

    auto obs = make_obstacles();
    GrowerResult result = grower.grow(obs.data(), static_cast<int>(obs.size()));

    std::printf("  threads: %d, roots: %d, boxes: %d, components: %d\n",
                n_threads, result.n_roots, result.n_boxes_total, result.n_components);
    std::printf("  vol: %.6f, time: %.1f ms\n",
                result.total_volume, result.build_time_ms);

    assert(result.n_roots >= 1);
    assert(result.n_boxes_total >= 1);

    std::cout << "  PASS\n\n";
}

// ═════════════════════════════════════════════════════════════════════════════
//  Test 5: Benchmark — serial vs parallel
// ═════════════════════════════════════════════════════════════════════════════
static void bench_serial_vs_parallel() {
    std::cout << "=== Test 5: Benchmark — serial vs parallel ===\n";

    Robot robot = Robot::from_json(ROBOT_PATH);
    int n = robot.n_joints();

    Eigen::VectorXd start = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd goal = Eigen::VectorXd::Zero(n);
    const auto& limits = robot.joint_limits().limits;
    for (int d = 0; d < n; ++d) {
        start[d] = limits[d].lo + 0.2 * limits[d].width();
        goal[d]  = limits[d].lo + 0.8 * limits[d].width();
    }

    auto obs = make_obstacles();

    int hw_threads = static_cast<int>(std::thread::hardware_concurrency());
    int n_threads = std::min(4, hw_threads);
    if (n_threads < 2) n_threads = 2;

    int max_boxes = 200;
    int n_roots = 4;

    // ── Serial run ──
    GrowerConfig serial_cfg;
    serial_cfg.mode = GrowerConfig::Mode::Wavefront;
    serial_cfg.n_roots = n_roots;
    serial_cfg.max_boxes = max_boxes;
    serial_cfg.max_consecutive_miss = 100;
    serial_cfg.min_edge = 0.05;
    serial_cfg.timeout = 60.0;
    serial_cfg.n_threads = 1;
    serial_cfg.pipeline.source = envelope::EndpointSourceConfig::ifk();
    serial_cfg.pipeline.envelope = envelope::EnvelopeTypeConfig::sub_aabb();

    ForestGrower serial_grower(robot, serial_cfg);
    serial_grower.set_endpoints(start, goal);

    auto t0 = std::chrono::high_resolution_clock::now();
    GrowerResult serial_result = serial_grower.grow(obs.data(), static_cast<int>(obs.size()));
    auto t1 = std::chrono::high_resolution_clock::now();
    double serial_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // ── Parallel run ──
    GrowerConfig par_cfg = serial_cfg;
    par_cfg.n_threads = n_threads;

    ForestGrower par_grower(robot, par_cfg);
    par_grower.set_endpoints(start, goal);

    auto t2 = std::chrono::high_resolution_clock::now();
    GrowerResult par_result = par_grower.grow(obs.data(), static_cast<int>(obs.size()));
    auto t3 = std::chrono::high_resolution_clock::now();
    double par_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();

    double speedup = serial_ms / par_ms;

    std::printf("\n  ┌─────────────────────────────────────────────┐\n");
    std::printf("  │  Benchmark: %d roots, %d max_boxes          │\n", n_roots, max_boxes);
    std::printf("  │  HW threads: %d, using: %d                  │\n", hw_threads, n_threads);
    std::printf("  ├─────────────────────┬───────────────────────┤\n");
    std::printf("  │            Serial   │   Parallel (%d thr)   │\n", n_threads);
    std::printf("  ├─────────────────────┼───────────────────────┤\n");
    std::printf("  │  boxes   %6d     │   %6d              │\n",
                serial_result.n_boxes_total, par_result.n_boxes_total);
    std::printf("  │  comps   %6d     │   %6d              │\n",
                serial_result.n_components, par_result.n_components);
    std::printf("  │  time   %7.1f ms  │  %7.1f ms           │\n",
                serial_ms, par_ms);
    std::printf("  │  speedup            │   %.2fx               │\n", speedup);
    std::printf("  └─────────────────────┴───────────────────────┘\n\n");

    // Parallel should produce boxes
    assert(par_result.n_boxes_total >= n_roots);

    std::cout << "  PASS\n\n";
}

// ═════════════════════════════════════════════════════════════════════════════
//  main
// ═════════════════════════════════════════════════════════════════════════════
int main() {
    std::cout << "\n╔══════════════════════════════════════════════╗\n"
                 "║  Parallel ForestGrower Test Suite             ║\n"
                 "╚══════════════════════════════════════════════╝\n\n";

    test_thread_pool();
    test_parallel_wavefront_no_endpoints();
    test_parallel_wavefront_with_endpoints();
    test_parallel_rrt();
    bench_serial_vs_parallel();

    std::cout << "══════════════════════════════════════════════\n"
                 "  ALL TESTS PASSED\n"
                 "══════════════════════════════════════════════\n";

    return 0;
}

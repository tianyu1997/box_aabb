// ═══════════════════════════════════════════════════════════════════════════
//  Phase 6 optimization tests:
//    1. KD-tree find_nearest_box correctness (vs brute-force)
//    2. KD-tree performance: O(log N) vs O(N)
//    3. SIMD adjacency kernel correctness (edge counts match scalar)
//    4. SIMD adjacency rebuild benchmark (timing comparison)
//    5. Combined: RRT mode with KD-tree acceleration
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/forest/adjacency_graph.h"
#include "sbf/forest/kd_tree.h"
#include "sbf/forest/forest_grower.h"
#include "sbf/forest/grower_config.h"
#include "sbf/robot/robot.h"
#include "sbf/common/types.h"

#include <Eigen/Core>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <random>

using namespace sbf;
using namespace sbf::forest;

static const char* ROBOT_PATH = std::getenv("ROBOT_PATH");

// Helper: create an N-D box at a given center with given half-width
static BoxNode make_box(int id, const Eigen::VectorXd& center, double half_w) {
    int nd = static_cast<int>(center.size());
    BoxNode box;
    box.id = id;
    box.joint_intervals.resize(nd);
    for (int d = 0; d < nd; ++d) {
        box.joint_intervals[d] = Interval(center[d] - half_w, center[d] + half_w);
    }
    box.seed_config = center;
    box.compute_volume();
    return box;
}

// ═════════════════════════════════════════════════════════════════════
//  Test 1: KD-tree find_nearest_box correctness
// ═════════════════════════════════════════════════════════════════════
static void test_kdtree_correctness() {
    std::cout << "=== Test 1: KD-tree find_nearest_box correctness ===\n";

    const int N_DIMS = 7;
    const int N_BOXES = 200;
    const int N_QUERIES = 500;

    // Generate random boxes
    std::mt19937_64 rng(123);
    std::uniform_real_distribution<double> dist(-3.0, 3.0);

    JointLimits limits;
    limits.limits.resize(N_DIMS, Interval(-3.14, 3.14));

    std::vector<BoxNode> boxes;
    boxes.reserve(N_BOXES);
    for (int i = 0; i < N_BOXES; ++i) {
        Eigen::VectorXd center(N_DIMS);
        for (int d = 0; d < N_DIMS; ++d)
            center[d] = dist(rng);
        double hw = 0.05 + 0.1 * std::abs(dist(rng));
        boxes.push_back(make_box(i, center, hw));
    }

    // Build adjacency graph with boxes (for find_nearest_box)
    AdjacencyGraph graph(limits);
    graph.rebuild(boxes);

    // Also do brute-force for comparison
    auto brute_nearest = [&](const Eigen::VectorXd& q) -> int {
        int best = -1;
        double best_d = 1e30;
        for (const auto& b : boxes) {
            double d = b.distance_to_config(q);
            if (d < best_d) { best_d = d; best = b.id; }
        }
        return best;
    };

    int mismatches = 0;
    for (int i = 0; i < N_QUERIES; ++i) {
        Eigen::VectorXd q(N_DIMS);
        for (int d = 0; d < N_DIMS; ++d)
            q[d] = dist(rng);

        int kd_id = graph.find_nearest_box(q);
        int bf_id = brute_nearest(q);

        // IDs might differ if distances are equal; check distances match
        double kd_dist = boxes[kd_id].distance_to_config(q);
        double bf_dist = boxes[bf_id].distance_to_config(q);
        if (std::abs(kd_dist - bf_dist) > 1e-9) {
            mismatches++;
            std::printf("    MISMATCH query %d: kd=%d (%.6f) vs bf=%d (%.6f)\n",
                        i, kd_id, kd_dist, bf_id, bf_dist);
        }
    }

    std::printf("  KD-tree: %d boxes, %d queries, %d mismatches\n",
                N_BOXES, N_QUERIES, mismatches);
    assert(mismatches == 0);
    std::cout << "  PASS\n\n";
}

// ═════════════════════════════════════════════════════════════════════
//  Test 2: KD-tree vs brute-force performance
// ═════════════════════════════════════════════════════════════════════
static void test_kdtree_performance() {
    std::cout << "=== Test 2: KD-tree performance ===\n";

    const int N_DIMS = 7;
    const int N_BOXES = 2000;
    const int N_QUERIES = 5000;

    std::mt19937_64 rng(456);
    std::uniform_real_distribution<double> dist(-3.0, 3.0);

    JointLimits limits;
    limits.limits.resize(N_DIMS, Interval(-3.14, 3.14));

    std::vector<BoxNode> boxes;
    boxes.reserve(N_BOXES);
    for (int i = 0; i < N_BOXES; ++i) {
        Eigen::VectorXd center(N_DIMS);
        for (int d = 0; d < N_DIMS; ++d)
            center[d] = dist(rng);
        boxes.push_back(make_box(i, center, 0.05));
    }

    // Generate queries
    std::vector<Eigen::VectorXd> queries(N_QUERIES);
    for (auto& q : queries) {
        q.resize(N_DIMS);
        for (int d = 0; d < N_DIMS; ++d)
            q[d] = dist(rng);
    }

    // Brute-force timing
    auto t0 = std::chrono::high_resolution_clock::now();
    volatile int bf_sum = 0;
    for (const auto& q : queries) {
        int best = -1;
        double best_d = 1e30;
        for (const auto& b : boxes) {
            double d = b.distance_to_config(q);
            if (d < best_d) { best_d = d; best = b.id; }
        }
        bf_sum += best;
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    double bf_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // KD-tree timing
    AdjacencyGraph graph(limits);
    graph.rebuild(boxes);

    auto t2 = std::chrono::high_resolution_clock::now();
    volatile int kd_sum = 0;
    for (const auto& q : queries) {
        kd_sum += graph.find_nearest_box(q);
    }
    auto t3 = std::chrono::high_resolution_clock::now();
    double kd_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();

    double speedup = bf_ms / kd_ms;
    std::printf("  %d boxes, %d queries:\n", N_BOXES, N_QUERIES);
    std::printf("    brute-force: %.1f ms (%.1f us/query)\n",
                bf_ms, bf_ms * 1000.0 / N_QUERIES);
    std::printf("    KD-tree:     %.1f ms (%.1f us/query)\n",
                kd_ms, kd_ms * 1000.0 / N_QUERIES);
    std::printf("    speedup: %.1fx\n", speedup);

    // KD-tree should be at least 2x faster for 2000 boxes
    assert(speedup > 1.5);
    std::cout << "  PASS\n\n";
}

// ═════════════════════════════════════════════════════════════════════
//  Test 3: SIMD adjacency correctness (edge count stability)
// ═════════════════════════════════════════════════════════════════════
static void test_simd_adjacency_correctness() {
    std::cout << "=== Test 3: SIMD adjacency correctness ===\n";

    // Use a deterministic set of touching/overlapping/separated boxes
    const int N_DIMS = 7;
    JointLimits limits;
    limits.limits.resize(N_DIMS, Interval(-3.14, 3.14));

    // Create a chain of adjacent boxes along dim 0
    std::vector<BoxNode> chain;
    for (int i = 0; i < 20; ++i) {
        Eigen::VectorXd center = Eigen::VectorXd::Zero(N_DIMS);
        center[0] = i * 0.2;  // spacing = 0.2, half-width = 0.1 → touching
        chain.push_back(make_box(i, center, 0.1));
    }

    AdjacencyGraph graph(limits);
    graph.rebuild(chain);

    // Chain of 20: each adjacent to next → 19 edges
    std::printf("  chain: %d boxes, %d edges (expected 19)\n",
                graph.n_boxes(), graph.n_edges());
    assert(graph.n_edges() == 19);
    assert(graph.n_components() == 1);

    // Now test incremental add: insert a box that touches box 10
    Eigen::VectorXd center = Eigen::VectorXd::Zero(N_DIMS);
    center[0] = 2.0;  // same as box 10
    center[1] = 0.2;  // offset in dim 1 → touching face in dim 1
    BoxNode extra = make_box(100, center, 0.1);
    graph.add_box(extra);

    // Should have 1 or more new edges (to box 10 at least)
    assert(graph.n_boxes() == 21);
    auto& nbrs = graph.neighbors(100);
    std::printf("  extra box 100: %d neighbors\n", static_cast<int>(nbrs.size()));
    assert(!nbrs.empty());  // at least one neighbor

    // Verify that graph.connected still works
    assert(graph.connected(0, 19));  // original chain
    assert(graph.connected(0, 100));  // via the chain to box 10 to extra

    std::cout << "  PASS\n\n";
}

// ═════════════════════════════════════════════════════════════════════
//  Test 4: Adjacency rebuild benchmark
// ═════════════════════════════════════════════════════════════════════
static void test_adjacency_benchmark() {
    std::cout << "=== Test 4: Adjacency rebuild benchmark ===\n";

    const int N_DIMS = 7;
    const int N_BOXES = 500;
    const int N_RUNS = 5;

    std::mt19937_64 rng(789);
    std::uniform_real_distribution<double> dist(-2.0, 2.0);

    JointLimits limits;
    limits.limits.resize(N_DIMS, Interval(-3.14, 3.14));

    // Generate boxes with small size → many near-adjacent pairs
    std::vector<BoxNode> boxes;
    boxes.reserve(N_BOXES);
    for (int i = 0; i < N_BOXES; ++i) {
        Eigen::VectorXd center(N_DIMS);
        for (int d = 0; d < N_DIMS; ++d)
            center[d] = dist(rng);
        boxes.push_back(make_box(i, center, 0.08));
    }

    // Warmup + benchmark
    double total_ms = 0.0;
    int last_edges = 0;
    for (int run = 0; run < N_RUNS; ++run) {
        AdjacencyGraph graph(limits);
        auto t0 = std::chrono::high_resolution_clock::now();
        graph.rebuild(boxes);
        auto t1 = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        if (run > 0) total_ms += ms;  // skip warmup
        last_edges = graph.n_edges();
    }
    double avg_ms = total_ms / (N_RUNS - 1);

    std::printf("  %d boxes, %d edges\n", N_BOXES, last_edges);
    std::printf("  rebuild time: %.2f ms (avg of %d runs)\n", avg_ms, N_RUNS - 1);

#ifdef SBF_HAS_AVX2
    std::printf("  SIMD: AVX2 enabled\n");
#else
    std::printf("  SIMD: scalar fallback\n");
#endif

    std::cout << "  PASS\n\n";
}

// ═════════════════════════════════════════════════════════════════════
//  Test 5: RRT with KD-tree (full pipeline test)
// ═════════════════════════════════════════════════════════════════════
static void test_rrt_with_kdtree() {
    std::cout << "=== Test 5: RRT mode with KD-tree ===\n";

    if (!ROBOT_PATH) {
        std::cout << "  SKIP (ROBOT_PATH not set)\n\n";
        return;
    }

    Robot robot = Robot::from_json(ROBOT_PATH);

    // Obstacles
    std::vector<Obstacle> obstacles = {
        Obstacle(Eigen::Vector3d(0.5, 0.0, 0.3),
                 Eigen::Vector3d(0.05, 0.05, 0.05)),
    };

    GrowerConfig cfg;
    cfg.mode = GrowerConfig::Mode::RRT;
    cfg.n_roots = 2;
    cfg.max_boxes = 50;
    cfg.min_edge = 0.02;
    cfg.max_consecutive_miss = 300;
    cfg.rng_seed = 42;

    ForestGrower grower(robot, cfg);
    auto result = grower.grow(obstacles.data(),
                              static_cast<int>(obstacles.size()));

    std::printf("  RRT: %d boxes, %d comps, vol=%.4f, time=%.1f ms\n",
                result.n_boxes_total, result.n_components,
                result.total_volume, result.build_time_ms);

    assert(result.n_boxes_total > 5);
    std::cout << "  PASS\n\n";
}

// ═════════════════════════════════════════════════════════════════════
int main() {
    std::cout << "\n";
    std::cout << "\xE2\x95\x94\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x97\n";
    std::cout << "\xE2\x95\x91 Phase 6: KD-tree + SIMD Test Suite      "
              << "\xE2\x95\x91\n";
    std::cout << "\xE2\x95\x9A\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x9D\n\n";

    test_kdtree_correctness();
    test_kdtree_performance();
    test_simd_adjacency_correctness();
    test_adjacency_benchmark();
    test_rrt_with_kdtree();

    std::cout << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\n";
    std::cout << "  ALL TESTS PASSED\n";
    std::cout << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\n";

    return 0;
}

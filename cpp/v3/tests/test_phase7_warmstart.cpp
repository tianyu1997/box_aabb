// ══════════════════════════════════════════════════════════════════════════�?
//  Phase 7 tests: LECT Warm-Start / Pre-expansion
//    1. NodeStore::snapshot correctness (deep copy, cleared occupation)
//    2. LECT::snapshot correctness (tree + envelopes preserved, occ cleared)
//    3. LECT::pre_expand correctness (all leaves up to depth N are split)
//    4. Warm-start parallel grow: same-quality output as cold-start
//    5. Warm-start performance benchmark vs cold-start
// ══════════════════════════════════════════════════════════════════════════�?
#include "sbf/forest/forest_grower.h"
#include "sbf/forest/grower_config.h"
#include "sbf/forest/lect.h"
#include "sbf/forest/node_store.h"
#include "sbf/robot/robot.h"
#include "sbf/common/types.h"

#include <Eigen/Core>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>

using namespace sbf;
using namespace sbf::forest;

static const char* ROBOT_PATH = std::getenv("ROBOT_PATH");

// ════════════════════════════════════════════════════════════════════�?
//  Test 1: NodeStore::snapshot
// ════════════════════════════════════════════════════════════════════�?
static void test_nodestore_snapshot() {
    std::cout << "=== Test 1: NodeStore snapshot ===\n";

    assert(ROBOT_PATH && "ROBOT_PATH env var required");
    Robot robot = Robot::from_json(ROBOT_PATH);

    envelope::PipelineConfig pipeline = envelope::PipelineConfig::recommended();
    LECT lect(robot, pipeline);

    // The LECT has been initialized with one root node (idx=0)
    assert(lect.n_nodes() == 1);

    // Do a find_free_box to add more nodes
    Eigen::VectorXd seed = Eigen::VectorXd::Zero(robot.n_joints());
    FFBResult ffb = lect.find_free_box(seed, nullptr, 0, 0.1, 10);
    assert(ffb.success());
    int n_before = lect.n_nodes();
    assert(n_before > 1);

    // Mark a node as occupied
    lect.mark_occupied(ffb.node_idx, 42);
    assert(lect.is_occupied(ffb.node_idx));

    // Snapshot the underlying store
    // Access store via: we'll test indirectly through LECT snapshot
    LECT snap = lect.snapshot();

    // Snapshot should have same node count
    assert(snap.n_nodes() == n_before);
    std::printf("  original nodes: %d, snapshot nodes: %d\n", n_before, snap.n_nodes());

    // Snapshot should have cleared occupation
    assert(!snap.is_occupied(ffb.node_idx));

    // Original should still be occupied
    assert(lect.is_occupied(ffb.node_idx));

    // Both should have same tree structure
    assert(snap.left(0) == lect.left(0));
    assert(snap.right(0) == lect.right(0));
    assert(snap.depth(ffb.node_idx) == lect.depth(ffb.node_idx));

    // Both should have AABBs for node 0
    assert(snap.has_aabb(0));
    assert(lect.has_aabb(0));

    // Snapshot should be independent: marking in snapshot doesn't affect original
    snap.mark_occupied(1, 99);
    assert(snap.is_occupied(1));
    assert(!lect.is_occupied(1));

    std::cout << "  PASS\n\n";
}

// ════════════════════════════════════════════════════════════════════�?
//  Test 2: LECT::snapshot preserves envelopes
// ════════════════════════════════════════════════════════════════════�?
static void test_lect_snapshot_envelopes() {
    std::cout << "=== Test 2: LECT snapshot preserves envelopes ===\n";

    assert(ROBOT_PATH);
    Robot robot = Robot::from_json(ROBOT_PATH);

    envelope::PipelineConfig pipeline = envelope::PipelineConfig::recommended();
    LECT lect(robot, pipeline);

    // Expand a bit to have more nodes with envelopes
    Eigen::VectorXd seed = Eigen::VectorXd::Zero(robot.n_joints());
    lect.find_free_box(seed, nullptr, 0, 0.05, 12);
    int n = lect.n_nodes();
    std::printf("  LECT has %d nodes\n", n);

    // Count nodes with AABBs
    int aabb_count = 0;
    for (int i = 0; i < n; ++i) {
        if (lect.has_aabb(i)) aabb_count++;
    }
    std::printf("  %d nodes have AABBs\n", aabb_count);
    assert(aabb_count > 1);

    // Snapshot
    LECT snap = lect.snapshot();
    assert(snap.n_nodes() == n);

    // Verify all AABB nodes are preserved in snapshot
    int snap_aabb_count = 0;
    for (int i = 0; i < n; ++i) {
        if (snap.has_aabb(i)) snap_aabb_count++;
    }
    assert(snap_aabb_count == aabb_count);

    // Verify AABB values match (spot check node 0)
    const float* orig_aabb = lect.get_link_aabbs(0);
    const float* snap_aabb = snap.get_link_aabbs(0);
    assert(orig_aabb && snap_aabb);
    int n_floats = robot.n_active_links() * 6;
    for (int i = 0; i < n_floats; ++i) {
        assert(std::abs(orig_aabb[i] - snap_aabb[i]) < 1e-10f);
    }

    // Verify endpoint AABBs are preserved
    if (lect.has_endpoint_aabbs(0) && snap.has_endpoint_aabbs(0)) {
        const float* orig_ep = lect.get_endpoint_aabbs(0);
        const float* snap_ep = snap.get_endpoint_aabbs(0);
        // Just check they're non-null (deep comparison would need knowing size)
        assert(orig_ep && snap_ep);
    }

    // Snapshot's occupation should be independent
    lect.mark_occupied(0, 1);
    assert(lect.is_occupied(0));
    assert(!snap.is_occupied(0));

    std::printf("  snapshot: %d nodes, %d with AABBs  [OK]\n", snap.n_nodes(), snap_aabb_count);
    std::cout << "  PASS\n\n";
}

// ════════════════════════════════════════════════════════════════════�?
//  Test 3: LECT::pre_expand
// ════════════════════════════════════════════════════════════════════�?
static void test_pre_expand() {
    std::cout << "=== Test 3: LECT pre_expand ===\n";

    assert(ROBOT_PATH);
    Robot robot = Robot::from_json(ROBOT_PATH);

    envelope::PipelineConfig pipeline = envelope::PipelineConfig::recommended();
    LECT lect(robot, pipeline);

    // Initially: 1 root node (depth 0, leaf)
    assert(lect.n_nodes() == 1);
    assert(lect.is_leaf(0));

    // Pre-expand to depth 2 �?root (d=0) splits to 2 children (d=1),
    // each child splits to 2 (d=2). Total new: 2 + 4 = 6 nodes.
    // Total nodes = 1 + 6 = 7.
    int new_nodes = lect.pre_expand(2);
    std::printf("  pre_expand(2): %d new nodes, total %d\n", new_nodes, lect.n_nodes());
    assert(new_nodes == 6);
    assert(lect.n_nodes() == 7);

    // Root (d=0) should NOT be a leaf anymore
    assert(!lect.is_leaf(0));

    // All depth-2 nodes should be leaves
    int leaf_count = 0;
    for (int i = 0; i < lect.n_nodes(); ++i) {
        if (lect.is_leaf(i)) leaf_count++;
        // All nodes should have AABBs
        assert(lect.has_aabb(i));
    }
    // 4 leaves at depth 2
    assert(leaf_count == 4);
    std::printf("  tree: %d nodes, %d leaves (expected 4), all have AABBs\n",
                lect.n_nodes(), leaf_count);

    // Pre-expand further to depth 3 �?4 leaves become 4 internal + 8 leaves = 8 new
    int new_nodes2 = lect.pre_expand(3);
    std::printf("  pre_expand(3): %d new nodes, total %d\n", new_nodes2, lect.n_nodes());
    assert(new_nodes2 == 8);
    assert(lect.n_nodes() == 15);

    // 8 leaves at depth 3
    int leaf_count3 = 0;
    for (int i = 0; i < lect.n_nodes(); ++i) {
        if (lect.is_leaf(i)) leaf_count3++;
    }
    assert(leaf_count3 == 8);
    std::printf("  tree: %d nodes, %d leaves (expected 8)\n",
                lect.n_nodes(), leaf_count3);

    // Calling pre_expand again with same depth should create 0 new nodes
    int new_nodes3 = lect.pre_expand(3);
    assert(new_nodes3 == 0);
    assert(lect.n_nodes() == 15);
    std::printf("  pre_expand(3) again: %d new nodes (expected 0)\n", new_nodes3);

    std::cout << "  PASS\n\n";
}

// ════════════════════════════════════════════════════════════════════�?
//  Test 4: Warm-start parallel grow quality
//    Compare warm_start_depth=3 vs warm_start_depth=-1 (disabled)
//    Both should produce comparable box counts and volume.
// ════════════════════════════════════════════════════════════════════�?
static void test_warmstart_parallel_quality() {
    std::cout << "=== Test 4: Warm-start parallel grow quality ===\n";

    assert(ROBOT_PATH);
    Robot robot = Robot::from_json(ROBOT_PATH);

    const int N_BOXES = 100;
    const int N_THREADS = 2;

    // ── Cold start (warm_start_depth = -1) ───────────────────────────
    GrowerResult cold_result;
    {
        GrowerConfig cold_cfg;
        cold_cfg.max_boxes = N_BOXES;
        cold_cfg.n_roots = 2;
        cold_cfg.n_threads = N_THREADS;
        cold_cfg.rng_seed = 42;
        cold_cfg.warm_start_depth = -1;     // disabled
        cold_cfg.pipeline = envelope::PipelineConfig::recommended();

        ForestGrower grower(robot, cold_cfg);
        cold_result = grower.grow(nullptr, 0);
    }

    // ── Warm start (warm_start_depth = 3) ────────────────────────────
    GrowerResult warm_result;
    {
        GrowerConfig warm_cfg;
        warm_cfg.max_boxes = N_BOXES;
        warm_cfg.n_roots = 2;
        warm_cfg.n_threads = N_THREADS;
        warm_cfg.rng_seed = 42;
        warm_cfg.warm_start_depth = 3;      // pre-expand 3 levels
        warm_cfg.pipeline = envelope::PipelineConfig::recommended();

        ForestGrower grower(robot, warm_cfg);
        warm_result = grower.grow(nullptr, 0);
    }

    std::printf("  cold: %d boxes, vol=%.4f, %.1f ms\n",
                cold_result.n_boxes_total, cold_result.total_volume,
                cold_result.build_time_ms);
    std::printf("  warm: %d boxes, vol=%.4f, %.1f ms",
                warm_result.n_boxes_total, warm_result.total_volume,
                warm_result.build_time_ms);
    if (warm_result.phase_times.count("warm_start_ms")) {
        std::printf("  (warm_start: %.1f ms)", warm_result.phase_times.at("warm_start_ms"));
    }
    std::printf("\n");

    // Both should produce boxes (sanity)
    assert(cold_result.n_boxes_total > 0);
    assert(warm_result.n_boxes_total > 0);

    // Warm start should produce at least 50% as many boxes as cold
    // (determinism may differ slightly due to different LECT states)
    double ratio = static_cast<double>(warm_result.n_boxes_total) /
                   std::max(1, cold_result.n_boxes_total);
    std::printf("  warm/cold box ratio: %.2f\n", ratio);
    assert(ratio > 0.5 && "warm-start should produce comparable box count");

    std::cout << "  PASS\n\n";
}

// ════════════════════════════════════════════════════════════════════�?
//  Test 5: Warm-start performance benchmark
//    Measure time savings from pre-expansion
// ════════════════════════════════════════════════════════════════════�?
static void test_warmstart_benchmark() {
    std::cout << "=== Test 5: Warm-start performance benchmark ===\n";

    assert(ROBOT_PATH);
    Robot robot = Robot::from_json(ROBOT_PATH);

    const int N_BOXES = 200;
    const int N_THREADS = 4;
    const int N_RUNS = 3;

    double cold_total = 0, warm_total = 0;

    for (int run = 0; run < N_RUNS; ++run) {
        // Cold start
        {
            GrowerConfig cfg;
            cfg.max_boxes = N_BOXES;
            cfg.n_roots = 4;
            cfg.n_threads = N_THREADS;
            cfg.rng_seed = 100 + run;
            cfg.warm_start_depth = -1;
            cfg.pipeline = envelope::PipelineConfig::recommended();

            ForestGrower grower(robot, cfg);
            auto r = grower.grow(nullptr, 0);
            cold_total += r.build_time_ms;
        }

        // Warm start
        {
            GrowerConfig cfg;
            cfg.max_boxes = N_BOXES;
            cfg.n_roots = 4;
            cfg.n_threads = N_THREADS;
            cfg.rng_seed = 100 + run;
            cfg.warm_start_depth = 3;
            cfg.pipeline = envelope::PipelineConfig::recommended();

            ForestGrower grower(robot, cfg);
            auto r = grower.grow(nullptr, 0);
            warm_total += r.build_time_ms;
        }
    }

    double cold_avg = cold_total / N_RUNS;
    double warm_avg = warm_total / N_RUNS;
    double speedup = cold_avg / std::max(warm_avg, 0.01);

    std::printf("  cold avg: %.1f ms\n", cold_avg);
    std::printf("  warm avg: %.1f ms\n", warm_avg);
    std::printf("  speedup:  %.2fx\n", speedup);

    // Warm should not be significantly slower than cold
    assert(warm_avg < cold_avg * 2.0 && "warm-start should not be much slower");

    std::cout << "  PASS\n\n";
}

// ════════════════════════════════════════════════════════════════════�?
//  main
// ════════════════════════════════════════════════════════════════════�?
int main() {
    std::cout << "\n"
              << "\xE2\x95\x94\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x97\n"
              << "\xE2\x95\x91 Phase 7: LECT Warm-Start Test Suite          \xE2\x95\x91\n"
              << "\xE2\x95\x9A\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90\xE2\x95\x90"
              << "\xE2\x95\x90\xE2\x95\x90\xE2\x95\x9D\n\n";

    test_nodestore_snapshot();
    test_lect_snapshot_envelopes();
    test_pre_expand();
    test_warmstart_parallel_quality();
    test_warmstart_benchmark();

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

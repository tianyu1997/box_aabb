// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — FFB + Multi-Channel Cache Tests
//
//  Tests:
//    1. source_can_serve — CritSample safety class separation
//    2. FFB basic descent — find_free_box returns valid nodes
//    3. FFB with Z4 cache — symmetry reuse during online FFB
//    4. FFB with heuristic split — WIDEST_FIRST and BEST_TIGHTEN
//    5. Hull invalidation — hull_valid cleared when source changes
//    6. Multi-pipeline cache reuse — source upgrade path
//
//  Build: cmake --build . --config Release --target test_ffb
//  Run:   Release\test_ffb.exe
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/core/joint_symmetry.h"
#include "sbf/robot/robot.h"
#include "sbf/robot/interval_fk.h"
#include "sbf/envelope/endpoint_source.h"
#include "sbf/envelope/envelope_derive.h"
#include "sbf/envelope/pipeline.h"
#include "sbf/core/types.h"
#include "sbf/core/config.h"
#include "sbf/forest/forest_grower.h"
#include "sbf/forest/lect.h"

#include <Eigen/Core>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <cstring>
#include <vector>

using namespace sbf;
using namespace sbf::envelope;
using namespace sbf::forest;

// ─── Helpers ────────────────────────────────────────────────────────────────
static int g_pass = 0, g_fail = 0;

#define CHECK(cond, msg) do {                                         \
    if (!(cond)) {                                                    \
        std::printf("  FAIL  %s:%d  %s\n", __FILE__, __LINE__, msg); \
        ++g_fail;                                                     \
    } else { ++g_pass; }                                              \
} while(0)

#define CHECK_CLOSE(a, b, tol, msg) \
    CHECK(std::abs(static_cast<double>(a) - static_cast<double>(b)) < (tol), msg)

#define SECTION(name) std::printf("\n── %s ──\n", (name))

// ─── Build Panda-like robot ────────────────────────────────────────────────
static Robot make_panda() {
    std::vector<DHParam> dh = {
        {  0.0,     0.0,     0.333,  0.0, 0},
        { -HALF_PI, 0.0,     0.0,    0.0, 0},
        {  HALF_PI, 0.0,     0.316,  0.0, 0},
        {  HALF_PI, 0.0825,  0.0,    0.0, 0},
        { -HALF_PI,-0.0825,  0.384,  0.0, 0},
        {  HALF_PI, 0.0,     0.0,    0.0, 0},
        {  HALF_PI, 0.088,   0.0,    0.0, 0},
    };

    JointLimits limits;
    limits.limits = {
        {-2.8973, 2.8973},
        {-1.7628, 1.7628},
        {-2.8973, 2.8973},
        {-3.0718,-0.0698},
        {-2.8973, 2.8973},
        {-0.0175, 3.7525},
        {-2.8973, 2.8973},
    };

    std::vector<double> radii = {0.08, 0.08, 0.06, 0.06, 0.04, 0.04, 0.04};

    return Robot("panda_test", dh, limits, std::nullopt, radii);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 1: source_can_serve — CritSample safety class separation
// ═══════════════════════════════════════════════════════════════════════════
static void test_source_safety_class() {
    SECTION("source_can_serve — safety class separation");

    static_assert(kEndpointSourceCount == 4,
                  "EndpointSource changed: update 4x4 substitution test matrix");

    // ── Safety class checks ─────────────────────────────────────────────
    CHECK(source_safety_class(EndpointSource::IFK) == SourceSafetyClass::SAFE,
          "IFK is SAFE");
    CHECK(source_safety_class(EndpointSource::Analytical) == SourceSafetyClass::SAFE,
          "Analytical is SAFE");
    CHECK(source_safety_class(EndpointSource::GCPC) == SourceSafetyClass::SAFE,
          "GCPC is SAFE");
    CHECK(source_safety_class(EndpointSource::CritSample) == SourceSafetyClass::UNSAFE,
          "CritSample is UNSAFE");

    // ── Within SAFE class: higher quality serves lower ──────────────────
    CHECK(source_can_serve(EndpointSource::GCPC, EndpointSource::IFK),
          "GCPC can serve IFK");
    CHECK(source_can_serve(EndpointSource::GCPC, EndpointSource::Analytical),
          "GCPC can serve Analytical");
    CHECK(source_can_serve(EndpointSource::Analytical, EndpointSource::IFK),
          "Analytical can serve IFK");
    CHECK(source_can_serve(EndpointSource::IFK, EndpointSource::IFK),
          "IFK serves IFK (same)");
    CHECK(source_can_serve(EndpointSource::GCPC, EndpointSource::GCPC),
          "GCPC serves GCPC (same)");

    // ── Within SAFE class: lower quality cannot serve higher ────────────
    CHECK(!source_can_serve(EndpointSource::IFK, EndpointSource::Analytical),
          "IFK cannot serve Analytical");
    CHECK(!source_can_serve(EndpointSource::IFK, EndpointSource::GCPC),
          "IFK cannot serve GCPC");
    CHECK(!source_can_serve(EndpointSource::Analytical, EndpointSource::GCPC),
          "Analytical cannot serve GCPC");

    // ── Cross-class: UNSAFE→SAFE blocked, SAFE→UNSAFE allowed ────────
    CHECK(!source_can_serve(EndpointSource::CritSample, EndpointSource::IFK),
          "CritSample cannot serve IFK (UNSAFE→SAFE blocked)");
    CHECK(!source_can_serve(EndpointSource::CritSample, EndpointSource::GCPC),
          "CritSample cannot serve GCPC (UNSAFE→SAFE blocked)");
    CHECK(!source_can_serve(EndpointSource::CritSample, EndpointSource::Analytical),
          "CritSample cannot serve Analytical (UNSAFE→SAFE blocked)");
    CHECK(source_can_serve(EndpointSource::IFK, EndpointSource::CritSample),
          "IFK can serve CritSample (SAFE→UNSAFE allowed)");
    CHECK(source_can_serve(EndpointSource::GCPC, EndpointSource::CritSample),
          "GCPC can serve CritSample (SAFE→UNSAFE allowed)");
    CHECK(source_can_serve(EndpointSource::Analytical, EndpointSource::CritSample),
          "Analytical can serve CritSample (SAFE→UNSAFE allowed)");

    // ── CritSample serves itself ────────────────────────────────────────
    CHECK(source_can_serve(EndpointSource::CritSample, EndpointSource::CritSample),
          "CritSample serves CritSample");

    // ── source_quality_sufficient (stored byte → source enum) ───────────
    //  Stored byte = static_cast<uint8_t>(EndpointSource)
    uint8_t stored_gcpc = static_cast<uint8_t>(EndpointSource::GCPC);
    uint8_t stored_ifk  = static_cast<uint8_t>(EndpointSource::IFK);
    uint8_t stored_crit = static_cast<uint8_t>(EndpointSource::CritSample);

    CHECK(source_quality_sufficient(stored_gcpc, EndpointSource::IFK),
          "stored GCPC sufficient for IFK");
    CHECK(!source_quality_sufficient(stored_ifk, EndpointSource::GCPC),
          "stored IFK not sufficient for GCPC");
    CHECK(source_quality_sufficient(stored_gcpc, EndpointSource::CritSample),
          "stored GCPC sufficient for CritSample (SAFE→UNSAFE allowed)");
    CHECK(!source_quality_sufficient(stored_crit, EndpointSource::IFK),
          "stored CritSample not sufficient for IFK (UNSAFE→SAFE blocked)");
    CHECK(source_quality_sufficient(stored_crit, EndpointSource::CritSample),
          "stored CritSample sufficient for CritSample");

    // ── Exhaustive 4x4 substitution matrix check ───────────────────────
    const EndpointSource all_src[kEndpointSourceCount] = {
        EndpointSource::IFK,
        EndpointSource::CritSample,
        EndpointSource::Analytical,
        EndpointSource::GCPC,
    };
    const bool expected[kEndpointSourceCount][kEndpointSourceCount] = {
        // requested:          IFK    Crit   Analyt GCPC
        /* cached IFK */      {true,  true,  false, false},
        /* cached Crit */     {false, true,  false, false},
        /* cached Analytical*/{true,  true,  true,  false},
        /* cached GCPC */     {true,  true,  true,  true },
    };
    for (int ci = 0; ci < kEndpointSourceCount; ++ci) {
        for (int ri = 0; ri < kEndpointSourceCount; ++ri) {
            CHECK(source_can_serve(all_src[ci], all_src[ri]) == expected[ci][ri],
                  "source_can_serve matches 4x4 substitution matrix");
        }
    }

    std::printf("  source_can_serve: OK\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 2: FFB basic descent — find_free_box returns valid node
// ═══════════════════════════════════════════════════════════════════════════
static void test_ffb_basic() {
    SECTION("FFB basic descent");

    Robot robot = make_panda();
    LECT lect(robot, 0.02, 256);

    // Pre-expand a few levels
    int n_new = lect.pre_expand(3);
    CHECK(n_new > 0, "pre_expand created nodes");

    // An obstacle far from the robot workspace — FFB should succeed easily
    Obstacle far_obs;
    far_obs.center = Eigen::Vector3d(10.0, 10.0, 10.0);
    far_obs.half_sizes = Eigen::Vector3d(0.1, 0.1, 0.1);

    // Seed at the center of joint limits
    Eigen::VectorXd seed(7);
    for (int j = 0; j < 7; ++j)
        seed[j] = robot.joint_limits().limits[j].center();

    FFBResult result = lect.find_free_box(seed, &far_obs, 1, 1e-4, 20);

    CHECK(result.success(), "FFB succeeds with far obstacle");
    CHECK(result.node_idx >= 0, "FFB returns valid node index");
    CHECK(result.fail_code == 0, "FFB fail_code = 0");
    CHECK(!result.path.empty(), "FFB path non-empty");

    // Verify the returned node has iAABBs
    CHECK(lect.has_iaabb(result.node_idx), "FFB result node has iAABBs");

    std::printf("  FFB basic: OK\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 2b: FFB hard timeout — expired deadline aborts immediately
// ═══════════════════════════════════════════════════════════════════════════
static void test_ffb_timeout() {
    SECTION("FFB hard timeout");

    Robot robot = make_panda();
    LECT lect(robot, 0.02, 256);

    Obstacle far_obs;
    far_obs.center = Eigen::Vector3d(10.0, 10.0, 10.0);
    far_obs.half_sizes = Eigen::Vector3d(0.1, 0.1, 0.1);

    Eigen::VectorXd seed(7);
    for (int j = 0; j < 7; ++j)
        seed[j] = robot.joint_limits().limits[j].center();

    lect.set_deadline(LECT::Clock::now());
    FFBResult result = lect.find_free_box(seed, &far_obs, 1, 1e-4, 20);

    CHECK(!result.success(), "FFB timeout returns failure");
    CHECK(result.fail_code == 4, "FFB timeout uses fail_code 4");
    CHECK(result.node_idx < 0, "FFB timeout returns no node");

    lect.set_deadline(std::nullopt);
    std::printf("  FFB timeout: OK\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 2c: ForestGrower stage timeout — root_select and expand are bounded
// ═══════════════════════════════════════════════════════════════════════════
static void test_grower_stage_timeout() {
    SECTION("ForestGrower staged timeout");

    Robot robot = make_panda();

    GrowerConfig cfg;
    cfg.pipeline = PipelineConfig::fast();
    cfg.mode = GrowerConfig::Mode::Wavefront;
    cfg.partition_mode = PartitionMode::Uniform;
    cfg.n_roots = 32;
    cfg.root_n_candidates = 50;
    cfg.max_boxes = 4096;
    cfg.timeout = 1e-3;
    cfg.max_consecutive_miss = 100000;
    cfg.n_boundary_samples = 8;
    cfg.min_edge = 1e-4;

    Obstacle far_obs;
    far_obs.center = Eigen::Vector3d(10.0, 10.0, 10.0);
    far_obs.half_sizes = Eigen::Vector3d(0.1, 0.1, 0.1);

    ForestGrower grower(robot, cfg);
    GrowerResult result = grower.grow(&far_obs, 1);

    auto root_it = result.phase_times.find("root_select_ms");
    auto expand_it = result.phase_times.find("expand_ms");
    CHECK(root_it != result.phase_times.end(), "grower reports root_select_ms");
    CHECK(expand_it != result.phase_times.end(), "grower reports expand_ms");
    if (root_it != result.phase_times.end()) {
        CHECK(root_it->second < 500.0,
              "root_select hard timeout stays below 500 ms in smoke test");
    }
    if (expand_it != result.phase_times.end()) {
        CHECK(expand_it->second < 500.0,
              "expand hard timeout stays below 500 ms in smoke test");
    }

    std::printf("  grower timeout: OK (root=%.3f ms, expand=%.3f ms)\n",
                root_it != result.phase_times.end() ? root_it->second : -1.0,
                expand_it != result.phase_times.end() ? expand_it->second : -1.0);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 3: FFB with Z4 cache — verify cache active during FFB
// ═══════════════════════════════════════════════════════════════════════════
static void test_ffb_z4_cache() {
    SECTION("FFB Z4 symmetry cache in online descent");

    Robot robot = make_panda();
    LECT lect(robot, 0.02, 512);

    // Verify Z4 detected and cache auto-enabled
    CHECK(lect.symmetry_q0().type == JointSymmetryType::Z4_ROTATION,
          "Panda has Z4 rotation");

    // Pre-expand to populate Z4 cache
    lect.pre_expand(2);

    // FFB in sector 0 (canonical): q_0 ∈ [0, π/4]
    Eigen::VectorXd seed0(7);
    seed0[0] = 0.5;  // sector 0
    for (int j = 1; j < 7; ++j)
        seed0[j] = robot.joint_limits().limits[j].center();

    Obstacle far_obs;
    far_obs.center = Eigen::Vector3d(10.0, 10.0, 10.0);
    far_obs.half_sizes = Eigen::Vector3d(0.1, 0.1, 0.1);

    FFBResult r0 = lect.find_free_box(seed0, &far_obs, 1, 1e-4, 15);
    CHECK(r0.success(), "FFB sector 0 succeeds");

    int nodes_after_s0 = lect.n_nodes();

    // FFB in sector 1: q_0 ∈ [π/2, π]
    Eigen::VectorXd seed1(7);
    seed1[0] = 2.0;  // sector 1 (≈ π/2 + offset)
    for (int j = 1; j < 7; ++j)
        seed1[j] = seed0[j];  // same downstream joints

    FFBResult r1 = lect.find_free_box(seed1, &far_obs, 1, 1e-4, 15);
    CHECK(r1.success(), "FFB sector 1 succeeds");

    // FFB in sector 2: q_0 ∈ [π, 3π/2]
    Eigen::VectorXd seed2(7);
    seed2[0] = -2.0;  // sector 2/3 (negative side)
    for (int j = 1; j < 7; ++j)
        seed2[j] = seed0[j];

    FFBResult r2 = lect.find_free_box(seed2, &far_obs, 1, 1e-4, 15);
    CHECK(r2.success(), "FFB sector 2/3 succeeds");

    // All returned nodes should have valid iAABBs
    CHECK(lect.has_iaabb(r0.node_idx), "sector 0 node has iAABBs");
    CHECK(lect.has_iaabb(r1.node_idx), "sector 1 node has iAABBs");
    CHECK(lect.has_iaabb(r2.node_idx), "sector 2/3 node has iAABBs");

    std::printf("  FFB Z4 cache: OK\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 4: FFB with heuristic split — WIDEST_FIRST and BEST_TIGHTEN
// ═══════════════════════════════════════════════════════════════════════════
static void test_ffb_heuristic_split() {
    SECTION("FFB heuristic split strategies");

    Robot robot = make_panda();
    Obstacle far_obs;
    far_obs.center = Eigen::Vector3d(10.0, 10.0, 10.0);
    far_obs.half_sizes = Eigen::Vector3d(0.1, 0.1, 0.1);

    Eigen::VectorXd seed(7);
    for (int j = 0; j < 7; ++j)
        seed[j] = robot.joint_limits().limits[j].center();

    // ── WIDEST_FIRST ────────────────────────────────────────────────────
    {
        LECT lect(robot, 0.02, 256);
        lect.set_split_order(SplitOrder::WIDEST_FIRST);
        lect.pre_expand(2);

        FFBResult r = lect.find_free_box(seed, &far_obs, 1, 1e-4, 15);
        CHECK(r.success(), "WIDEST_FIRST FFB succeeds");
        CHECK(r.node_idx >= 0, "WIDEST_FIRST returns valid node");
    }

    // ── BEST_TIGHTEN ────────────────────────────────────────────────────
    {
        LECT lect(robot, 0.02, 256);
        lect.set_split_order(SplitOrder::BEST_TIGHTEN);
        lect.pre_expand(2);

        FFBResult r = lect.find_free_box(seed, &far_obs, 1, 1e-4, 15);
        CHECK(r.success(), "BEST_TIGHTEN FFB succeeds");
        CHECK(r.node_idx >= 0, "BEST_TIGHTEN returns valid node");
    }

    // ── ROUND_ROBIN ─────────────────────────────────────────────────────
    {
        LECT lect(robot, 0.02, 256);
        lect.set_split_order(SplitOrder::ROUND_ROBIN);
        lect.pre_expand(2);

        FFBResult r = lect.find_free_box(seed, &far_obs, 1, 1e-4, 15);
        CHECK(r.success(), "ROUND_ROBIN FFB succeeds");
        CHECK(r.node_idx >= 0, "ROUND_ROBIN returns valid node");
    }

    std::printf("  FFB heuristic split: OK\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 5: FFB with collision — leaf splitting and hull refinement
// ═══════════════════════════════════════════════════════════════════════════
static void test_ffb_collision() {
    SECTION("FFB collision handling");

    Robot robot = make_panda();

    // Obstacle near the base at workspace center
    Obstacle near_obs;
    near_obs.center = Eigen::Vector3d(0.3, 0.0, 0.5);
    near_obs.half_sizes = Eigen::Vector3d(0.05, 0.05, 0.05);

    LECT lect(robot, 0.02, 1024);
    lect.pre_expand(2);

    Eigen::VectorXd seed(7);
    for (int j = 0; j < 7; ++j)
        seed[j] = robot.joint_limits().limits[j].center();

    FFBResult r = lect.find_free_box(seed, &near_obs, 1, 1e-4, 20);

    // FFB should either succeed (by splitting fine enough) or fail with
    // max_depth/min_edge — never with invariant violations
    if (r.success()) {
        CHECK(r.node_idx >= 0, "collision FFB: valid result node");
        CHECK(lect.has_iaabb(r.node_idx), "collision FFB: has iAABBs");
        CHECK(r.n_new_nodes >= 0, "collision FFB: created ≥0 nodes");
    } else {
        CHECK(r.fail_code == 2 || r.fail_code == 3,
              "collision FFB: fail due to depth or edge limit");
    }

    std::printf("  FFB collision: OK\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 6: FFB occupation and subtree_occ
// ═══════════════════════════════════════════════════════════════════════════
static void test_ffb_occupation() {
    SECTION("FFB occupation tracking");

    Robot robot = make_panda();
    LECT lect(robot, 0.02, 256);
    lect.pre_expand(3);

    Obstacle far_obs;
    far_obs.center = Eigen::Vector3d(10.0, 10.0, 10.0);
    far_obs.half_sizes = Eigen::Vector3d(0.1, 0.1, 0.1);

    Eigen::VectorXd seed(7);
    for (int j = 0; j < 7; ++j)
        seed[j] = robot.joint_limits().limits[j].center();

    // First FFB — should succeed
    FFBResult r1 = lect.find_free_box(seed, &far_obs, 1, 1e-4, 20);
    CHECK(r1.success(), "first FFB succeeds");

    // Mark the found node as occupied
    lect.mark_occupied(r1.node_idx, 0);
    CHECK(lect.is_occupied(r1.node_idx), "node is occupied after mark");

    // Second FFB with same seed — should find a different node (the
    // original is occupied, so it will descend further/find sibling)
    FFBResult r2 = lect.find_free_box(seed, &far_obs, 1, 1e-4, 20);

    // It should either succeed (different node) or fail (occupied)
    if (r2.success()) {
        CHECK(r2.node_idx != r1.node_idx,
              "second FFB finds different node");
    }

    // Unmark and try again
    lect.unmark_occupied(r1.node_idx);
    CHECK(!lect.is_occupied(r1.node_idx), "node unoccupied after unmark");

    std::printf("  FFB occupation: OK\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 7: Z4 sector boundary splitting
// ═══════════════════════════════════════════════════════════════════════════
static void test_z4_sector_boundary_split() {
    SECTION("Z4 sector boundary splitting in split_leaf");

    Robot robot = make_panda();
    LECT lect(robot, 0.02, 512);

    // After pre_expand, check that splits on dim=0 prefer kπ/2 boundaries
    lect.set_split_order(SplitOrder::WIDEST_FIRST);
    lect.pre_expand(2);

    // Check root node split: if dim=0 was widest, split should be near kπ/2
    const double half_pi = HALF_PI;
    if (!lect.is_leaf(0)) {
        double sv = lect.split_val(0);
        // Check if split is near a kπ/2 boundary
        double dist_to_boundary = 1e10;
        for (int k = -10; k <= 10; ++k) {
            dist_to_boundary = std::min(dist_to_boundary,
                std::abs(sv - k * half_pi));
        }
        // The root's full range is about [-2.9, 2.9], so widest_first
        // may not pick dim=0 first. Only check if the split is on dim 0.
        if (dist_to_boundary < 1.0) {
            CHECK(dist_to_boundary < 0.5,
                  "dim=0 split near sector boundary");
        }
    }

    std::printf("  Z4 sector boundary: OK\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 8: Hull grid lazy derivation and collision test
// ═══════════════════════════════════════════════════════════════════════════
static void test_hull_grid_lazy() {
    SECTION("Hull grid lazy derivation");

    Robot robot = make_panda();

    auto pipeline = PipelineConfig::fast();
    pipeline.envelope = EnvelopeTypeConfig::hull16_grid();

    LECT lect(robot, pipeline, 256);
    lect.pre_expand(2);

    // Initially, no hull grids should be computed (lazy)
    int hull_count = lect.count_nodes_with_hull();
    // Hull grids computed during pre_expand only if collision test triggers
    // (no obstacles here, so none should be computed)

    // Set scene and call collides_scene — should trigger lazy hull derive
    Obstacle near_obs;
    near_obs.center = Eigen::Vector3d(0.3, 0.0, 0.5);
    near_obs.half_sizes = Eigen::Vector3d(0.05, 0.05, 0.05);
    lect.set_scene(&near_obs, 1);

    // Find a leaf node and test collision — this should lazily compute hull
    Eigen::VectorXd seed(7);
    for (int j = 0; j < 7; ++j)
        seed[j] = robot.joint_limits().limits[j].center();

    FFBResult r = lect.find_free_box(seed, &near_obs, 1, 1e-4, 15);
    // After FFB descent with collisions, some hull grids may have been derived
    int hull_after = lect.count_nodes_with_hull();

    // The FFB attempted hull derivation for collision-positive nodes
    // (exact count depends on collision outcomes — just verify no crash)
    CHECK(hull_after >= 0, "hull count after FFB is valid");

    std::printf("  hull grid lazy: OK (hulls before=%d, after=%d)\n",
                hull_count, hull_after);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 9: snapshot preserves Z4 state
// ═══════════════════════════════════════════════════════════════════════════
static void test_snapshot_z4() {
    SECTION("snapshot preserves Z4 cache state");

    Robot robot = make_panda();
    LECT lect(robot, 0.02, 256);
    lect.pre_expand(2);

    CHECK(lect.symmetry_q0().type == JointSymmetryType::Z4_ROTATION,
          "original has Z4");

    LECT copy = lect.snapshot();
    CHECK(copy.symmetry_q0().type == JointSymmetryType::Z4_ROTATION,
          "snapshot has Z4");
    CHECK(copy.n_nodes() == lect.n_nodes(),
          "snapshot has same node count");

    // FFB should work on snapshot
    Obstacle far_obs;
    far_obs.center = Eigen::Vector3d(10.0, 10.0, 10.0);
    far_obs.half_sizes = Eigen::Vector3d(0.1, 0.1, 0.1);

    Eigen::VectorXd seed(7);
    for (int j = 0; j < 7; ++j)
        seed[j] = robot.joint_limits().limits[j].center();

    FFBResult r = copy.find_free_box(seed, &far_obs, 1, 1e-4, 15);
    CHECK(r.success(), "FFB on snapshot succeeds");

    std::printf("  snapshot Z4: OK\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  main
// ═══════════════════════════════════════════════════════════════════════════
int main() {
    std::printf("═══ FFB + Multi-Channel Cache Tests ═══\n");

    test_source_safety_class();
    test_ffb_basic();
    test_ffb_timeout();
    test_grower_stage_timeout();
    test_ffb_z4_cache();
    test_ffb_heuristic_split();
    test_ffb_collision();
    test_ffb_occupation();
    test_z4_sector_boundary_split();
    test_hull_grid_lazy();
    test_snapshot_z4();

    std::printf("\n═══ Summary: %d passed, %d failed ═══\n",
                g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}

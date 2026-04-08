// ═══════════════════════════════════════════════════════════════════════════
//  Test: Modular Pipeline Integration
//
//  Verifies the modular endpoint_aabb / envelope_type / cache modules
//  produce correct results across all 4×4 = 16 pipeline combinations.
//
//  Also tests the new LECT constructor with PipelineConfig.
//
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/robot/robot.h"
#include "sbf/envelope/frame_source.h"
#include "sbf/envelope/envelope_type.h"
#include "sbf/envelope/envelope_cache.h"
#include "sbf/forest/lect.h"

#include <cassert>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace sbf;
using namespace sbf::envelope;
using namespace sbf::forest;

static const char* ROBOT_PATH =
    "C:/Users/TIAN/OneDrive - The Chinese University of Hong Kong"
    "/Desktop/code/box_aabb/safeboxforest/v1/configs/iiwa14.json";

static bool approx_eq(double a, double b, double tol = 1e-6) {
    return std::abs(a - b) < tol;
}

// ─── Test 1: Endpoint Source Module ────────────────────────────────────────
static bool test_endpoint_source(const Robot& robot) {
    std::cout << "\n=== Test 1: Endpoint Source Module ===\n";

    // Create a medium-width C-space box
    auto lim = robot.joint_limits().limits;
    std::vector<Interval> ivs(robot.n_joints());
    for (int j = 0; j < robot.n_joints(); ++j) {
        double c = (lim[j].lo + lim[j].hi) * 0.5;
        double hw = (lim[j].hi - lim[j].lo) * 0.15;
        ivs[j] = {c - hw, c + hw};
    }

    // Test IFK source
    {
        auto cfg = EndpointSourceConfig::ifk();
        auto res = compute_endpoint_aabb(cfg, robot, ivs);
        assert(!res.endpoint_aabbs.empty());
        assert(res.has_fk_state());
        std::cout << "  IFK:            " << res.endpoint_aabbs.size() / 6
                  << " endpoint_aabbs, fk_valid=" << res.fk_state.valid << "\n";
    }

    // Test CritSample source
    {
        auto cfg = EndpointSourceConfig::crit_sampling();
        auto res = compute_endpoint_aabb(cfg, robot, ivs);
        assert(!res.endpoint_aabbs.empty());
        assert(res.has_fk_state());
        std::cout << "  CritSample:     " << res.endpoint_aabbs.size() / 6
                  << " endpoint_aabbs, n_eval=" << res.n_evaluations << "\n";
    }

    // Test AnalyticalCritical source
    {
        auto cfg = EndpointSourceConfig::analytical();
        auto res = compute_endpoint_aabb(cfg, robot, ivs);
        assert(!res.endpoint_aabbs.empty());
        assert(res.has_fk_state());
        int n_endpoints = robot.n_joints() + (robot.has_tool() ? 1 : 0);
        int expected = n_endpoints * 6;  // always n_endpoints × 6
        assert(static_cast<int>(res.endpoint_aabbs.size()) == expected);
        std::cout << "  AnalyticalCrit: " << res.endpoint_aabbs.size() / 6
                  << " endpoint_aabbs (endpoints), n_eval=" << res.n_evaluations << "\n";
    }

    // Test incremental source (IFK)
    {
        auto cfg = EndpointSourceConfig::ifk();
        auto full = compute_endpoint_aabb(cfg, robot, ivs);

        // Modify one dimension
        auto ivs2 = ivs;
        ivs2[2].hi = ivs2[2].lo + (ivs2[2].hi - ivs2[2].lo) * 0.5;
        auto incr = compute_endpoint_aabb_incremental(
            cfg, full.fk_state, robot, ivs2, 2);
        assert(!incr.endpoint_aabbs.empty());
        assert(incr.has_fk_state());
        std::cout << "  IFK incremental: " << incr.endpoint_aabbs.size() / 6
                  << " endpoint_aabbs OK\n";
    }

    std::cout << "  PASS\n";
    return true;
}

// ─── Test 2: Envelope Type Module ───────────────────────────────────────────
static bool test_envelope_type(const Robot& robot) {
    std::cout << "\n=== Test 2: Envelope Type Module ===\n";

    auto lim = robot.joint_limits().limits;
    std::vector<Interval> ivs(robot.n_joints());
    for (int j = 0; j < robot.n_joints(); ++j) {
        double c = (lim[j].lo + lim[j].hi) * 0.5;
        double hw = (lim[j].hi - lim[j].lo) * 0.15;
        ivs[j] = {c - hw, c + hw};
    }

    // Get endpoint AABBs from IFK
    auto src_cfg = EndpointSourceConfig::ifk();
    auto src_res = compute_endpoint_aabb(src_cfg, robot, ivs);

    // Test SubAABB (n_sub=1, equivalent to old AABB)
    {
        auto env_cfg = EnvelopeTypeConfig::sub_aabb();
        auto env_res = compute_link_envelope(env_cfg, robot, src_res);
        assert(env_res.valid);
        assert(!env_res.sub_aabbs.empty());
        assert(env_res.volume > 0);
        std::cout << "  SubAABB(n=1):  vol=" << std::fixed
                  << std::setprecision(4) << env_res.volume
                  << " m^3, n=" << env_res.n_voxels << "\n";
    }

    // Verify backward-compat alias aabb() produces same result
    {
        auto env_cfg = EnvelopeTypeConfig::aabb();
        assert(env_cfg.type == EnvelopeType::SubAABB);
        assert(env_cfg.n_sub == 1);
    }

    // Test SubAABB_Grid
    {
        auto env_cfg = EnvelopeTypeConfig::sub_aabb_grid();
        auto env_res = compute_link_envelope(env_cfg, robot, src_res);
        assert(env_res.valid);
        assert(!env_res.grid.empty());
        assert(env_res.n_voxels > 0);
        std::cout << "  SubAABB_Grid:  vol=" << std::fixed
                  << std::setprecision(4) << env_res.volume
                  << " m^3, voxels=" << env_res.n_voxels << "\n";
    }

    // Test Hull16_Grid
    {
        auto env_cfg = EnvelopeTypeConfig::hull16_grid();
        auto env_res = compute_link_envelope(env_cfg, robot, src_res);
        assert(env_res.valid);
        assert(env_res.n_voxels > 0);
        assert(env_res.n_bricks > 0);
        std::cout << "  Hull16_Grid:   vol=" << std::fixed
                  << std::setprecision(4) << env_res.volume
                  << " m^3, voxels=" << env_res.n_voxels
                  << ", bricks=" << env_res.n_bricks << "\n";
    }

    // Test default_envelope_config returns valid configs
    for (int s = 0; s < 4; ++s) {
        for (int e = 0; e < 3; ++e) {
            auto src = static_cast<EndpointSource>(s);
            auto env = static_cast<EnvelopeType>(e);
            auto cfg = default_envelope_config(src, env);
            assert(cfg.type == env);
            assert(cfg.n_sub >= 1);
        }
    }
    std::cout << "  default_envelope_config: 12 combos OK\n";

    std::cout << "  PASS\n";
    return true;
}

// ─── Test 3: Cache Module ───────────────────────────────────────────────────
static bool test_cache(const Robot& robot) {
    std::cout << "\n=== Test 3: Cache Module ===\n";

    // Test robot hash
    auto hash1 = EnvelopeCache::compute_robot_hash(robot);
    auto hash2 = EnvelopeCache::compute_robot_hash(robot);
    assert(hash1 == hash2);
    assert(hash1 != 0);
    std::cout << "  Robot hash: 0x" << std::hex << hash1 << std::dec << "\n";

    // Test metadata save/load
    std::string test_dir = "test_cache_temp";
    fs::create_directories(test_dir);

    CacheMetadata meta;
    meta.source_method = EndpointSource::CritSample;
    meta.envelope_type = EnvelopeType::Hull16_Grid;
    meta.n_sub = 4;
    meta.delta = 0.01;
    meta.grid_R = 64;
    meta.n_nodes = 100;
    meta.robot_hash = hash1;

    EnvelopeCache::save_metadata(test_dir, meta);

    CacheMetadata loaded;
    bool ok = EnvelopeCache::load_metadata(test_dir, loaded);
    assert(ok);
    assert(loaded.source_method == EndpointSource::CritSample);
    assert(loaded.envelope_type == EnvelopeType::Hull16_Grid);
    assert(loaded.n_sub == 4);
    assert(loaded.robot_hash == hash1);
    std::cout << "  Metadata save/load: OK\n";

    // Test is_valid
    auto pipeline = PipelineConfig::recommended();
    bool valid = EnvelopeCache::is_valid(test_dir, pipeline, robot);
    assert(valid);  // CritSample + Hull16_Grid matches recommended()
    std::cout << "  is_valid (matching): OK\n";

    auto pipeline2 = PipelineConfig::fast();
    bool valid2 = EnvelopeCache::is_valid(test_dir, pipeline2, robot);
    assert(!valid2);  // IFK + SubAABB doesn't match
    std::cout << "  is_valid (mismatch): OK\n";

    // Cleanup
    fs::remove_all(test_dir);

    std::cout << "  PASS\n";
    return true;
}

// ─── Test 4: LECT with PipelineConfig ───────────────────────────────────────
static bool test_lect_pipeline(const Robot& robot) {
    std::cout << "\n=== Test 4: LECT with PipelineConfig ===\n";

    // Test default constructor (backward compatible)
    {
        LECT lect(robot, 0.02, 64);
        assert(lect.source_method() == EndpointSource::IFK);
        assert(lect.envelope_type() == EnvelopeType::Hull16_Grid);
        assert(lect.n_nodes() == 1);
        assert(lect.has_hull_grid(0));
        std::cout << "  Default LECT:    source="
                  << frame_source_name(lect.source_method())
                  << ", envelope="
                  << envelope_type_name(lect.envelope_type()) << "\n";
    }

    // Test PipelineConfig constructor (recommended)
    {
        auto pipeline = PipelineConfig::recommended();
        LECT lect(robot, pipeline, 64);
        assert(lect.source_method() == EndpointSource::CritSample);
        assert(lect.envelope_type() == EnvelopeType::Hull16_Grid);
        assert(lect.n_nodes() == 1);
        assert(lect.has_hull_grid(0));
        std::cout << "  Recommended LECT: source="
                  << frame_source_name(lect.source_method())
                  << ", envelope="
                  << envelope_type_name(lect.envelope_type()) << "\n";
    }

    // Test PipelineConfig constructor (fast)
    {
        auto pipeline = PipelineConfig::fast();
        LECT lect(robot, pipeline, 64);
        assert(lect.source_method() == EndpointSource::IFK);
        assert(lect.envelope_type() == EnvelopeType::SubAABB);
        assert(lect.n_nodes() == 1);
        std::cout << "  Fast LECT:       source="
                  << frame_source_name(lect.source_method())
                  << ", envelope="
                  << envelope_type_name(lect.envelope_type()) << "\n";
    }

    // Test PipelineConfig constructor (production) + FFB
    {
        auto pipeline = PipelineConfig::production();
        LECT lect(robot, pipeline, 256);

        Eigen::VectorXd seed(robot.n_joints());
        auto lim = robot.joint_limits().limits;
        for (int j = 0; j < robot.n_joints(); ++j)
            seed[j] = (lim[j].lo + lim[j].hi) * 0.5;

        // No obstacles → should succeed
        auto result = lect.find_free_box(seed, nullptr, 0, 0.05, 10);
        assert(result.fail_code == 0);
        assert(result.node_idx >= 0);
        std::cout << "  Production FFB:  node=" << result.node_idx
                  << ", n_new=" << result.n_new_nodes
                  << ", n_fk=" << result.n_fk_calls << "\n";
    }

    std::cout << "  PASS\n";
    return true;
}

// ─── Test 5: Full Pipeline 4×4 Sweep ────────────────────────────────────────
static bool test_full_sweep(const Robot& robot) {
    std::cout << "\n=== Test 5: Full Pipeline 4x4 Sweep ===\n";

    auto lim = robot.joint_limits().limits;
    std::vector<Interval> ivs(robot.n_joints());
    for (int j = 0; j < robot.n_joints(); ++j) {
        double c = (lim[j].lo + lim[j].hi) * 0.5;
        double hw = (lim[j].hi - lim[j].lo) * 0.1;
        ivs[j] = {c - hw, c + hw};
    }

    const char* src_names[] = {"IFK", "CritSample", "AnalyticalCrit", "GCPC"};
    const char* env_names[] = {"SubAABB", "SubAABB_Grid", "Hull16_Grid"};

    for (int s = 0; s < 4; ++s) {
        auto src_method = static_cast<EndpointSource>(s);
        EndpointSourceConfig src_cfg;
        src_cfg.method = src_method;
        if (s == 0) src_cfg = EndpointSourceConfig::ifk();
        if (s == 1) src_cfg = EndpointSourceConfig::crit_sampling();
        if (s == 2) src_cfg = EndpointSourceConfig::analytical();
        if (s == 3) continue;  // GCPC needs cache, skip in basic sweep

        for (int e = 0; e < 3; ++e) {
            auto env_type = static_cast<EnvelopeType>(e);
            auto env_cfg = default_envelope_config(src_method, env_type);

            // Compute endpoint AABBs
            auto src_res = compute_endpoint_aabb(
                src_cfg, robot, ivs);

            // Compute link envelope
            auto env_res = compute_link_envelope(env_cfg, robot, src_res);

            assert(env_res.valid);
            assert(env_res.volume > 0);

            std::cout << "  " << src_names[s] << " + " << env_names[e]
                      << ": vol=" << std::fixed << std::setprecision(4)
                      << env_res.volume << " m^3";
            if (env_res.n_bricks > 0)
                std::cout << ", bricks=" << env_res.n_bricks;
            std::cout << "\n";
        }
    }

    std::cout << "  PASS\n";
    return true;
}

// ═════════════════════════════════════════════════════════════════════════════
int main() {
    std::cout << "╔══════════════════════════════════════════════════════╗\n";
    std::cout << "║ Modular Pipeline Integration Tests                  ║\n";
    std::cout << "╚══════════════════════════════════════════════════════╝\n";

    // Load robot
    Robot robot = Robot::from_json(ROBOT_PATH);
    std::cout << "Robot: " << robot.n_joints() << " joints, "
              << robot.n_active_links() << " active links\n";

    int pass = 0, fail = 0;

    if (test_endpoint_source(robot))    ++pass; else ++fail;
    if (test_envelope_type(robot))   ++pass; else ++fail;
    if (test_cache(robot))           ++pass; else ++fail;
    if (test_lect_pipeline(robot))   ++pass; else ++fail;
    if (test_full_sweep(robot))      ++pass; else ++fail;

    std::cout << "\n══════════════════════════════════════════════════════\n";
    if (fail == 0)
        std::cout << "  ALL " << pass << " TESTS PASSED\n";
    else
        std::cout << "  " << fail << " of " << (pass + fail) << " FAILED\n";
    std::cout << "══════════════════════════════════════════════════════\n";

    return fail;
}

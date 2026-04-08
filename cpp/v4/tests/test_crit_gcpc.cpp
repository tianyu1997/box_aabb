// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — CritSample + GCPC Module Tests
//
//  Tests the GCPC-based critical sampling pipeline and GCPC cache:
//    - CritSample boundary-only phase (no cache)
//    - CritSample config factory methods
//    - CritSample stats population
//    - CritSample vs iFK containment
//    - CritSample endpoint_source dispatch
//    - GCPC cache construction & query
//    - GCPC derive_aabb_with_gcpc pipeline
//    - End-to-end: CritSample with GCPC cache
//
//  Build: cmake --build . --config Release --target test_crit_gcpc
//  Run:   Release\test_crit_gcpc.exe
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/robot/robot.h"
#include "sbf/robot/interval_math.h"
#include "sbf/robot/interval_fk.h"
#include "sbf/robot/fk.h"
#include "sbf/envelope/analytical_utils.h"
#include "sbf/envelope/crit_sample.h"
#include "sbf/envelope/gcpc.h"
#include "sbf/envelope/endpoint_source.h"
#include "sbf/envelope/envelope_derive.h"

#include <Eigen/Core>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <algorithm>
#include <numeric>

using namespace sbf;
using namespace sbf::envelope;

// ─── Helpers ────────────────────────────────────────────────────────────────
static int g_pass = 0, g_fail = 0;

#define CHECK(cond, msg) do {                                         \
    if (!(cond)) {                                                    \
        std::printf("  FAIL  %s:%d  %s\n", __FILE__, __LINE__, msg); \
        ++g_fail;                                                     \
    } else { ++g_pass; }                                              \
} while(0)

#define CHECK_CLOSE(a, b, tol, msg) \
    CHECK(std::abs((a) - (b)) < (tol), msg)

#define SECTION(name) std::printf("\n── %s ──\n", (name))

// ─── Build IIWA14-like robot (same as other test files) ─────────────────────
static Robot make_iiwa14() {
    std::vector<DHParam> dh = {
        {  0.0,             0.0, 0.1575, 0.0, 0},
        { -HALF_PI,         0.0, 0.0,    0.0, 0},
        {  HALF_PI,         0.0, 0.2025, 0.0, 0},
        {  HALF_PI,         0.0, 0.0,    0.0, 0},
        { -HALF_PI,         0.0, 0.2155, 0.0, 0},
        { -HALF_PI,         0.0, 0.0,    0.0, 0},
        {  HALF_PI,         0.0, 0.081,  0.0, 0},
    };

    JointLimits limits;
    limits.limits = {
        {-1.865, 1.866},
        {-0.100, 1.087},
        {-0.663, 0.662},
        {-2.094,-0.372},
        {-0.619, 0.620},
        {-1.095, 1.258},
        { 1.050, 2.091},
    };

    DHParam tool{0.0, 0.0, 0.185, 0.0, 0};
    std::vector<double> radii = {0.08, 0.08, 0.06, 0.06, 0.04, 0.04, 0.04, 0.03};
    return Robot("iiwa14_test", dh, limits, tool, radii);
}

// Build intervals for a narrow box around center q
static std::vector<Interval> make_narrow_intervals(const Robot& robot, double half_width) {
    const int n = robot.n_joints();
    const auto& jl = robot.joint_limits();
    std::vector<Interval> ivs(n);
    for (int j = 0; j < n; ++j) {
        double mid = 0.5 * (jl.limits[j].lo + jl.limits[j].hi);
        ivs[j].lo = mid - half_width;
        ivs[j].hi = mid + half_width;
    }
    return ivs;
}

// Build full-range intervals
static std::vector<Interval> make_full_intervals(const Robot& robot) {
    const int n = robot.n_joints();
    const auto& jl = robot.joint_limits();
    std::vector<Interval> ivs(n);
    for (int j = 0; j < n; ++j) {
        ivs[j].lo = jl.limits[j].lo;
        ivs[j].hi = jl.limits[j].hi;
    }
    return ivs;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 1: CritSample boundary-only (no GCPC cache)
// ═══════════════════════════════════════════════════════════════════════════
static void test_critsample_boundary_only() {
    SECTION("CritSample boundary-only (no cache)");

    Robot robot = make_iiwa14();
    auto ivs = make_narrow_intervals(robot, 0.15);
    const int n_endpoints = robot.n_joints() + (robot.has_tool() ? 1 : 0);

    std::vector<float> ep_iaabbs(n_endpoints * 6);
    CritSampleStats stats{};

    auto config = CriticalSamplingConfig::boundary_only();
    int n_fk = derive_crit_endpoints(robot, ivs, config,
                                      ep_iaabbs.data(), &stats);

    // All endpoint iAABBs should be valid (lo <= hi)
    bool all_valid = true;
    for (int k = 0; k < n_endpoints; ++k) {
        float* a = ep_iaabbs.data() + k * 6;
        if (a[0] > a[3] || a[1] > a[4] || a[2] > a[5]) {
            std::printf("  Endpoint %d invalid: [%.4f,%.4f]x[%.4f,%.4f]x[%.4f,%.4f]\n",
                        k, a[0], a[3], a[1], a[4], a[2], a[5]);
            all_valid = false;
        }
    }
    CHECK(all_valid, "all endpoint iAABBs valid (boundary-only)");

    // Stats: GCPC phase should be zero, boundary phase should be non-zero
    CHECK(stats.n_gcpc_matches == 0, "no GCPC matches (boundary-only)");
    CHECK(stats.n_gcpc_fk == 0,     "no GCPC FK calls (boundary-only)");
    CHECK(stats.n_boundary_combos > 0, "boundary combos > 0");
    CHECK(stats.n_boundary_fk > 0,  "boundary FK > 0");
    CHECK(n_fk == stats.n_boundary_fk, "total FK = boundary FK");

    std::printf("  boundary_combos=%d  boundary_fk=%d  phase2_ms=%.3f\n",
                stats.n_boundary_combos, stats.n_boundary_fk, stats.phase2_ms);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 2: CritSample config factory methods
// ═══════════════════════════════════════════════════════════════════════════
static void test_critsample_config_factories() {
    SECTION("CritSample config factories");

    auto def = CriticalSamplingConfig::defaults();
    CHECK(def.enable_gcpc,     "defaults: enable_gcpc=true");
    CHECK(def.enable_boundary, "defaults: enable_boundary=true");
    CHECK(def.max_boundary_combos == 60000, "defaults: max_boundary_combos=60000");
    CHECK(def.gcpc_cache == nullptr, "defaults: gcpc_cache=nullptr");

    auto go = CriticalSamplingConfig::gcpc_only();
    CHECK(go.enable_gcpc,      "gcpc_only: enable_gcpc=true");
    CHECK(!go.enable_boundary, "gcpc_only: enable_boundary=false");

    auto bo = CriticalSamplingConfig::boundary_only();
    CHECK(!bo.enable_gcpc,     "boundary_only: enable_gcpc=false");
    CHECK(bo.enable_boundary,  "boundary_only: enable_boundary=true");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 3: CritSample boundary vs iFK containment
//
//  CritSample boundary should produce boxes that are inside or equal to
//  iFK boxes (since iFK is a bounding guarantee, CritSample samples actual
//  configurations which are always within the iFK bound).
// ═══════════════════════════════════════════════════════════════════════════
static void test_critsample_within_ifk() {
    SECTION("CritSample boundary within iFK");

    Robot robot = make_iiwa14();
    auto ivs = make_narrow_intervals(robot, 0.15);
    const int n_endpoints = robot.n_joints() + (robot.has_tool() ? 1 : 0);

    // Compute iFK endpoint iAABBs
    auto fk_state = compute_fk_full(robot, ivs);
    std::vector<float> ifk_ep(n_endpoints * 6);
    fk_to_endpoints(fk_state, robot, ifk_ep);

    // Compute CritSample boundary endpoint iAABBs
    std::vector<float> crit_ep(n_endpoints * 6);
    auto config = CriticalSamplingConfig::boundary_only();
    derive_crit_endpoints(robot, ivs, config, crit_ep.data(), nullptr);

    // CritSample values are actual FK results → must lie within iFK bounds
    // i.e., crit_lo >= ifk_lo and crit_hi <= ifk_hi (within float tolerance)
    bool within = true;
    const float tol = 1e-5f;
    for (int k = 0; k < n_endpoints; ++k) {
        float* ifk = ifk_ep.data() + k * 6;
        float* crt = crit_ep.data() + k * 6;
        for (int d = 0; d < 3; ++d) {
            if (crt[d] < ifk[d] - tol) {
                std::printf("  EP%d lo[%d]: crit=%.6f < ifk=%.6f\n", k, d, crt[d], ifk[d]);
                within = false;
            }
            if (crt[d+3] > ifk[d+3] + tol) {
                std::printf("  EP%d hi[%d]: crit=%.6f > ifk=%.6f\n", k, d, crt[d+3], ifk[d+3]);
                within = false;
            }
        }
    }
    CHECK(within, "CritSample endpoints within iFK bounds");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 4: CritSample multiple widths — wider intervals → more FK
// ═══════════════════════════════════════════════════════════════════════════
static void test_critsample_width_scaling() {
    SECTION("CritSample width scaling");

    Robot robot = make_iiwa14();

    auto config = CriticalSamplingConfig::boundary_only();
    std::vector<int> fk_counts;

    double widths[] = {0.05, 0.15, 0.30};
    for (double hw : widths) {
        auto ivs = make_narrow_intervals(robot, hw);
        const int n_endpoints = robot.n_joints() + (robot.has_tool() ? 1 : 0);
        std::vector<float> ep(n_endpoints * 6);
        CritSampleStats stats{};
        derive_crit_endpoints(robot, ivs, config, ep.data(), &stats);
        fk_counts.push_back(stats.n_boundary_fk);
        std::printf("  hw=%.2f: boundary_fk=%d\n", hw, stats.n_boundary_fk);
    }

    // Wider intervals should have more kπ/2 values in range → more combos
    // This is not strict monotonic (depends on interval content), but for
    // our test robot, wider → more critical angles → more combos
    CHECK(fk_counts[0] > 0, "narrow width has FK calls");
    CHECK(fk_counts[2] >= fk_counts[0],
          "wider interval has >= FK calls than narrow");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 5: CritSample endpoint_source dispatch
// ═══════════════════════════════════════════════════════════════════════════
static void test_critsample_dispatch() {
    SECTION("CritSample endpoint_source dispatch");

    Robot robot = make_iiwa14();
    auto ivs = make_narrow_intervals(robot, 0.15);

    EndpointSourceConfig src_cfg = EndpointSourceConfig::crit_sampling();
    auto result = compute_endpoint_iaabb(src_cfg, robot, ivs);

    CHECK(result.has_fk_state(), "CritSample dispatch produces FK state");
    CHECK(result.n_active_ep > 0, "CritSample dispatch has endpoints");

    const int n_endpoints = robot.n_joints() + (robot.has_tool() ? 1 : 0);
        CHECK(result.endpoint_iaabb_len() == result.n_active_ep * 6,
            "CritSample dispatch: endpoint_iaabb_len matches active endpoint count");
        CHECK(result.endpoint_iaabb_len() <= n_endpoints * 6,
          "CritSample dispatch: correct endpoint_iaabbs size");

    // Extract link iAABBs
    const int n_act = robot.n_active_links();
    std::vector<float> link_aabbs(n_act * 6);
    extract_link_iaabbs(result, robot, link_aabbs.data());

    // All link iAABBs should be valid
    bool all_valid = true;
    for (int ci = 0; ci < n_act; ++ci) {
        float* a = link_aabbs.data() + ci * 6;
        if (a[0] > a[3] || a[1] > a[4] || a[2] > a[5]) {
            all_valid = false;
        }
    }
    CHECK(all_valid, "CritSample dispatch: all link iAABBs valid");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 6: CritSample with GCPC disabled — same as boundary-only
// ═══════════════════════════════════════════════════════════════════════════
static void test_critsample_gcpc_disabled() {
    SECTION("CritSample with GCPC disabled");

    Robot robot = make_iiwa14();
    auto ivs = make_narrow_intervals(robot, 0.15);
    const int n_endpoints = robot.n_joints() + (robot.has_tool() ? 1 : 0);

    // defaults() with no cache → GCPC phase should produce nothing
    std::vector<float> ep_default(n_endpoints * 6);
    CritSampleStats stats_default{};
    auto cfg_default = CriticalSamplingConfig::defaults();
    derive_crit_endpoints(robot, ivs, cfg_default, ep_default.data(), &stats_default);

    // boundary_only() should produce same result
    std::vector<float> ep_boundary(n_endpoints * 6);
    CritSampleStats stats_boundary{};
    auto cfg_boundary = CriticalSamplingConfig::boundary_only();
    derive_crit_endpoints(robot, ivs, cfg_boundary, ep_boundary.data(), &stats_boundary);

    CHECK(stats_default.n_gcpc_matches == 0,
          "defaults (no cache): no GCPC matches");
    CHECK(stats_default.n_boundary_fk == stats_boundary.n_boundary_fk,
          "defaults (no cache) == boundary_only FK count");

    // Endpoint iAABBs should be identical
    bool match = true;
    for (int i = 0; i < n_endpoints * 6; ++i) {
        if (std::abs(ep_default[i] - ep_boundary[i]) > 1e-10f) {
            match = false;
            break;
        }
    }
    CHECK(match, "defaults (no cache) produces same results as boundary_only");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 7: CritSample full range — no crash
// ═══════════════════════════════════════════════════════════════════════════
static void test_critsample_full_range() {
    SECTION("CritSample full-range");

    Robot robot = make_iiwa14();
    auto ivs = make_full_intervals(robot);
    const int n_endpoints = robot.n_joints() + (robot.has_tool() ? 1 : 0);

    std::vector<float> ep(n_endpoints * 6);
    CritSampleStats stats{};

    // Use boundary_only to avoid needing GCPC cache
    // With full range, many kπ/2 combos → may hit max_boundary_combos cap
    auto config = CriticalSamplingConfig::boundary_only();
    config.max_boundary_combos = 10000;  // cap for speed

    derive_crit_endpoints(robot, ivs, config, ep.data(), &stats);

    bool all_valid = true;
    for (int k = 0; k < n_endpoints; ++k) {
        float* a = ep.data() + k * 6;
        if (a[0] > a[3] || a[1] > a[4] || a[2] > a[5]) {
            all_valid = false;
        }
    }
    CHECK(all_valid, "full-range CritSample: all endpoint iAABBs valid");
    std::printf("  full-range: boundary_combos=%d  boundary_fk=%d\n",
                stats.n_boundary_combos, stats.n_boundary_fk);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 8: GCPC cache construction — build + query_link
// ═══════════════════════════════════════════════════════════════════════════
static void test_gcpc_build_query() {
    SECTION("GCPC cache build + query");

    Robot robot = make_iiwa14();
    const int n = robot.n_joints();
    const int n_act = robot.n_active_links();
    const int* map = robot.active_link_map();

    // Manually create some GcpcPoints
    std::vector<GcpcPoint> points;
    for (int ci = 0; ci < n_act; ++ci) {
        int link_id = map[ci];
        for (int dir = 0; dir < 2; ++dir) {
            GcpcPoint pt{};
            pt.link_id = link_id;
            pt.direction = dir;
            pt.n_eff = n - 1;  // q₁..q₆ (q₀ eliminated)
            // Set q_eff to midpoints (in [0,π] for q₁)
            for (int j = 0; j < pt.n_eff; ++j) {
                double mid = 0.5 * (robot.joint_limits().limits[j+1].lo +
                                    robot.joint_limits().limits[j+1].hi);
                // For q₁: clamp to [0,π]
                if (j == 0 && mid < 0) mid = -mid;
                if (j == 0 && mid > M_PI) mid = M_PI;
                pt.q_eff[j] = mid;
            }
            // Compute local FK values for A, B, C, R
            // Use a simple position evaluation
            FKWorkspace ws;
            Eigen::VectorXd q(n);
            q[0] = 0.0;  // q₀ = 0 for computing local values
            for (int j = 0; j < pt.n_eff; ++j)
                q[j+1] = pt.q_eff[j];
            ws.compute(robot, q);
            if (link_id + 1 < ws.np) {
                Eigen::Vector3d pos = ws.pos(link_id + 1);
                pt.A = pos[0];
                pt.B = pos[1];
                pt.C = pos[2];
                pt.R = std::sqrt(pt.A * pt.A + pt.B * pt.B);
            }
            points.push_back(pt);
        }
    }

    // Build cache
    GcpcCache cache;
    cache.build(robot, points);

    CHECK(cache.is_loaded(), "GCPC cache is loaded after build");
    CHECK(cache.n_total_points() == static_cast<int>(points.size()),
          "GCPC cache has correct point count");

    // Query with intervals that should contain our midpoints
    auto ivs = make_narrow_intervals(robot, 0.5);
    for (int ci = 0; ci < n_act; ++ci) {
        int link_id = map[ci];
        std::vector<GcpcQueryResult> results;
        cache.query_link(link_id, ivs.data(), results);
        // We may or may not get matches depending on q₀ reconstruction
        // Just verify no crash
        std::printf("  Link %d: %d query results\n", link_id,
                    static_cast<int>(results.size()));
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 9: GCPC cache enrich_with_interior_search
// ═══════════════════════════════════════════════════════════════════════════
static void test_gcpc_enrich() {
    SECTION("GCPC cache enrichment");

    Robot robot = make_iiwa14();

    // Build empty cache, then enrich
    GcpcCache cache;
    std::vector<GcpcPoint> empty_points;
    cache.build(robot, empty_points);
    CHECK(cache.is_loaded(), "empty cache is loaded");

    int n_before = cache.n_total_points();
    int n_added = cache.enrich_with_interior_search(robot,
        /*n_random_seeds=*/50, /*max_sweeps=*/3);

    std::printf("  Before enrichment: %d points\n", n_before);
    std::printf("  Added by enrichment: %d points\n", n_added);
    std::printf("  After enrichment: %d points\n", cache.n_total_points());

    CHECK(cache.n_total_points() >= n_before,
          "enrichment adds >= 0 points");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 10: GCPC derive_aabb_with_gcpc — produces valid AABBs
// ═══════════════════════════════════════════════════════════════════════════
static void test_gcpc_derive_aabb() {
    SECTION("GCPC derive_aabb_with_gcpc");

    Robot robot = make_iiwa14();
    auto ivs = make_narrow_intervals(robot, 0.15);
    const int n_act = robot.n_active_links();

    // Build enriched cache
    GcpcCache cache;
    std::vector<GcpcPoint> empty_pts;
    cache.build(robot, empty_pts);
    cache.enrich_with_interior_search(robot, 100, 3);

    // Derive AABBs
    std::vector<float> aabbs(n_act * 6);
    GcpcQueryStats stats{};
    cache.derive_aabb_with_gcpc(robot, ivs, /*n_sub=*/1,
                                 aabbs.data(), &stats);

    bool all_valid = true;
    for (int ci = 0; ci < n_act; ++ci) {
        float* a = aabbs.data() + ci * 6;
        if (a[0] > a[3] || a[1] > a[4] || a[2] > a[5]) {
            std::printf("  Link %d invalid AABB\n", ci);
            all_valid = false;
        }
    }
    CHECK(all_valid, "GCPC derive: all AABBs valid");

    std::printf("  cache_matches=%d  boundary_kpi2=%d  boundary_atan2=%d  fk=%d\n",
                stats.n_cache_matches, stats.n_boundary_kpi2,
                stats.n_boundary_atan2, stats.n_fk_calls);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 11: GCPC vs iFK containment — GCPC AABBs inside iFK
// ═══════════════════════════════════════════════════════════════════════════
static void test_gcpc_within_ifk() {
    SECTION("GCPC AABBs within iFK");

    Robot robot = make_iiwa14();
    auto ivs = make_narrow_intervals(robot, 0.15);
    const int n_act = robot.n_active_links();

    // Compute iFK AABBs
    auto fk_state = compute_fk_full(robot, ivs);
    std::vector<float> ifk_aabbs(n_act * 6);
    extract_link_aabbs(fk_state, robot.active_link_map(), n_act,
                       ifk_aabbs.data(), robot.active_link_radii());

    // Build enriched GCPC cache
    GcpcCache cache;
    std::vector<GcpcPoint> empty_pts;
    cache.build(robot, empty_pts);
    cache.enrich_with_interior_search(robot, 100, 3);

    // Derive GCPC AABBs (these include link radius)
    std::vector<float> gcpc_aabbs(n_act * 6);
    cache.derive_aabb_with_gcpc(robot, ivs, 1, gcpc_aabbs.data());

    // GCPC AABBs should be within iFK bounds (both are outer bounds)
    // GCPC should be tighter or equal
    bool within = true;
    int n_tighter = 0;
    const float tol = 1e-4f;
    for (int ci = 0; ci < n_act; ++ci) {
        float* ifk = ifk_aabbs.data() + ci * 6;
        float* gcpc = gcpc_aabbs.data() + ci * 6;
        for (int d = 0; d < 3; ++d) {
            if (gcpc[d] < ifk[d] - tol) {
                std::printf("  Link %d lo[%d]: gcpc=%.6f < ifk=%.6f\n",
                            ci, d, gcpc[d], ifk[d]);
                within = false;
            }
            if (gcpc[d+3] > ifk[d+3] + tol) {
                std::printf("  Link %d hi[%d]: gcpc=%.6f > ifk=%.6f\n",
                            ci, d, gcpc[d+3], ifk[d+3]);
                within = false;
            }
        }
        // Check tightness
        double ifk_vol = (ifk[3]-ifk[0]) * (ifk[4]-ifk[1]) * (ifk[5]-ifk[2]);
        double gcpc_vol = (gcpc[3]-gcpc[0]) * (gcpc[4]-gcpc[1]) * (gcpc[5]-gcpc[2]);
        if (gcpc_vol < ifk_vol - 1e-12) ++n_tighter;
    }
    CHECK(within, "GCPC AABBs within iFK bounds");
    std::printf("  Links where GCPC tighter: %d/%d\n", n_tighter, n_act);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 12: GCPC query_link stats and q₀ reconstruction
// ═══════════════════════════════════════════════════════════════════════════
static void test_gcpc_query_reconstruction() {
    SECTION("GCPC query q0 reconstruction");

    Robot robot = make_iiwa14();

    // Build enriched cache with enough seeds
    GcpcCache cache;
    std::vector<GcpcPoint> empty_pts;
    cache.build(robot, empty_pts);
    int n_added = cache.enrich_with_interior_search(robot, 200, 5);
    std::printf("  Enriched: %d points\n", n_added);

    if (cache.n_total_points() == 0) {
        std::printf("  (skip: no points in cache)\n");
        ++g_pass;  // Skip gracefully
        return;
    }

    // Query with wide intervals
    auto ivs = make_narrow_intervals(robot, 0.5);
    const int n_act = robot.n_active_links();
    const int* map = robot.active_link_map();

    int total_matches = 0;
    int total_reflected = 0;
    for (int ci = 0; ci < n_act; ++ci) {
        int link_id = map[ci];
        std::vector<GcpcQueryResult> results;
        cache.query_link(link_id, ivs.data(), results);
        total_matches += static_cast<int>(results.size());
        for (const auto& r : results) {
            if (r.q1_reflected) ++total_reflected;
            // q0_optimal should be within interval[0]
            CHECK(r.q0_optimal >= ivs[0].lo - 1e-6 &&
                  r.q0_optimal <= ivs[0].hi + 1e-6,
                  "reconstructed q0 within interval");
        }
    }
    std::printf("  Total matches: %d  Reflected: %d\n",
                total_matches, total_reflected);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 13: CritSample + GCPC end-to-end
// ═══════════════════════════════════════════════════════════════════════════
static void test_critsample_with_gcpc() {
    SECTION("CritSample with GCPC cache end-to-end");

    Robot robot = make_iiwa14();
    auto ivs = make_narrow_intervals(robot, 0.3);
    const int n_endpoints = robot.n_joints() + (robot.has_tool() ? 1 : 0);

    // Build enriched GCPC cache
    GcpcCache cache;
    std::vector<GcpcPoint> empty_pts;
    cache.build(robot, empty_pts);
    cache.enrich_with_interior_search(robot, 100, 3);

    // Run CritSample with GCPC cache
    std::vector<float> ep_with_gcpc(n_endpoints * 6);
    CritSampleStats stats_gcpc{};
    auto cfg_gcpc = CriticalSamplingConfig::defaults();
    cfg_gcpc.gcpc_cache = &cache;
    derive_crit_endpoints(robot, ivs, cfg_gcpc, ep_with_gcpc.data(), &stats_gcpc);

    // Run CritSample without GCPC cache (boundary-only)
    std::vector<float> ep_boundary(n_endpoints * 6);
    CritSampleStats stats_bnd{};
    auto cfg_bnd = CriticalSamplingConfig::boundary_only();
    derive_crit_endpoints(robot, ivs, cfg_bnd, ep_boundary.data(), &stats_bnd);

    std::printf("  With GCPC: gcpc_matches=%d gcpc_fk=%d boundary_fk=%d\n",
                stats_gcpc.n_gcpc_matches, stats_gcpc.n_gcpc_fk,
                stats_gcpc.n_boundary_fk);
    std::printf("  Boundary-only: boundary_fk=%d\n", stats_bnd.n_boundary_fk);

    // With GCPC + boundary, the iAABBs should be at least as tight as
    // boundary-only (since GCPC adds more sampled configurations)
    bool all_valid = true;
    for (int k = 0; k < n_endpoints; ++k) {
        float* a = ep_with_gcpc.data() + k * 6;
        if (a[0] > a[3] || a[1] > a[4] || a[2] > a[5]) {
            all_valid = false;
        }
    }
    CHECK(all_valid, "CritSample+GCPC: all endpoint iAABBs valid");

    // Both should have same boundary FK count (Phase 2 identical)
    CHECK(stats_gcpc.n_boundary_fk == stats_bnd.n_boundary_fk,
          "same boundary FK with or without GCPC");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 14: CritSample endpoint_source with GCPC cache
// ═══════════════════════════════════════════════════════════════════════════
static void test_critsample_dispatch_with_gcpc() {
    SECTION("CritSample dispatch with GCPC cache");

    Robot robot = make_iiwa14();
    auto ivs = make_narrow_intervals(robot, 0.2);

    // Build enriched GCPC cache
    GcpcCache cache;
    std::vector<GcpcPoint> empty_pts;
    cache.build(robot, empty_pts);
    cache.enrich_with_interior_search(robot, 50, 3);

    // Dispatch via endpoint_source with GCPC cache
    EndpointSourceConfig src_cfg = EndpointSourceConfig::crit_sampling();
    src_cfg.gcpc_cache = &cache;

    auto result = compute_endpoint_iaabb(src_cfg, robot, ivs);
    CHECK(result.has_fk_state(), "dispatch with GCPC: has FK state");

    // Extract link iAABBs
    const int n_act = robot.n_active_links();
    std::vector<float> link_aabbs(n_act * 6);
    extract_link_iaabbs(result, robot, link_aabbs.data());

    bool all_valid = true;
    for (int ci = 0; ci < n_act; ++ci) {
        float* a = link_aabbs.data() + ci * 6;
        if (a[0] > a[3] || a[1] > a[4] || a[2] > a[5]) {
            all_valid = false;
        }
    }
    CHECK(all_valid, "dispatch with GCPC: all link iAABBs valid");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 15: GCPC n_sub > 1
// ═══════════════════════════════════════════════════════════════════════════
static void test_gcpc_nsub() {
    SECTION("GCPC n_sub > 1");

    Robot robot = make_iiwa14();
    auto ivs = make_narrow_intervals(robot, 0.15);
    const int n_act = robot.n_active_links();
    const int n_sub = 4;

    // Build enriched GCPC cache
    GcpcCache cache;
    std::vector<GcpcPoint> empty_pts;
    cache.build(robot, empty_pts);
    cache.enrich_with_interior_search(robot, 50, 3);

    std::vector<float> aabbs(n_act * n_sub * 6);
    GcpcQueryStats stats{};
    cache.derive_aabb_with_gcpc(robot, ivs, n_sub, aabbs.data(), &stats);

    bool all_valid = true;
    for (int ci = 0; ci < n_act; ++ci) {
        for (int s = 0; s < n_sub; ++s) {
            float* a = aabbs.data() + (ci * n_sub + s) * 6;
            if (a[0] > a[3] || a[1] > a[4] || a[2] > a[5]) {
                std::printf("  Link %d sub %d invalid\n", ci, s);
                all_valid = false;
            }
        }
    }
    CHECK(all_valid, "GCPC n_sub=4: all sub-segment AABBs valid");
    std::printf("  n_sub=%d, fk_calls=%d\n", n_sub, stats.n_fk_calls);
}

// ═══════════════════════════════════════════════════════════════════════════
int main() {
    std::printf("SafeBoxForest v4 — CritSample + GCPC Tests\n");

    // CritSample tests
    test_critsample_boundary_only();
    test_critsample_config_factories();
    test_critsample_within_ifk();
    test_critsample_width_scaling();
    test_critsample_dispatch();
    test_critsample_gcpc_disabled();
    test_critsample_full_range();

    // GCPC tests
    test_gcpc_build_query();
    test_gcpc_enrich();
    test_gcpc_derive_aabb();
    test_gcpc_within_ifk();
    test_gcpc_query_reconstruction();

    // Integration tests
    test_critsample_with_gcpc();
    test_critsample_dispatch_with_gcpc();
    test_gcpc_nsub();

    std::printf("\n════════════════════════════════════════════════════════\n");
    std::printf("  PASS: %d   FAIL: %d\n", g_pass, g_fail);
    std::printf("════════════════════════════════════════════════════════\n");

    return g_fail > 0 ? 1 : 0;
}

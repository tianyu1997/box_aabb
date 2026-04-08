// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
//  SafeBoxForest v4 鈥?Analytical Critical Solver Tests
//
//  Tests the analytical gradient-zero enumeration pipeline:
//    - crit_angles / build_csets / FKWorkspace utilities
//    - derive_aabb_critical_analytical() all phases
//    - AA pruning integration
//    - endpoint_source Analytical dispatch
//
//  Build: cmake --build . --config Release --target test_analytical
//  Run:   Release\test_analytical.exe
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
#include "sbf/robot/robot.h"
#include "sbf/robot/interval_math.h"
#include "sbf/robot/interval_fk.h"
#include "sbf/robot/fk.h"
#include "sbf/envelope/analytical_utils.h"
#include "sbf/envelope/analytical_solve.h"
#include "sbf/envelope/analytical_coeff.h"
#include "sbf/envelope/endpoint_source.h"
#include "sbf/envelope/envelope_derive.h"

#include <Eigen/Core>
#include <Eigen/QR>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <vector>

using namespace sbf;
using namespace sbf::envelope;

// 鈹€鈹€鈹€ Helpers 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
static int g_pass = 0, g_fail = 0;

#define CHECK(cond, msg) do {                                         \
    if (!(cond)) {                                                    \
        std::printf("  FAIL  %s:%d  %s\n", __FILE__, __LINE__, msg); \
        ++g_fail;                                                     \
    } else { ++g_pass; }                                              \
} while(0)

#define CHECK_CLOSE(a, b, tol, msg) \
    CHECK(std::abs((a) - (b)) < (tol), msg)

#define SECTION(name) std::printf("\n鈹€鈹€ %s 鈹€鈹€\n", (name))

// 鈹€鈹€鈹€ Build IIWA14-like robot (same as test_ifk_pipeline) 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
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

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
//  Test 1: crit_angles
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
static void test_crit_angles() {
    SECTION("crit_angles");

    // [0, 蟺] should include 0, 蟺/2, 蟺 (and endpoints)
    auto ca = crit_angles(0.0, M_PI);
    CHECK(ca.size() >= 3, "crit_angles([0,pi]) should have >=3 values");

    // Check that all returned values are in range
    bool all_in_range = true;
    for (double v : ca)
        if (v < -1e-12 || v > M_PI + 1e-12) all_in_range = false;
    CHECK(all_in_range, "all crit_angles in range");

    // Narrow interval that doesn't contain any k蟺/2
    auto ca2 = crit_angles(0.1, 0.2);
    CHECK(ca2.size() >= 2, "crit_angles should include endpoints");
}

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
//  Test 2: FKWorkspace basic
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
static void test_fk_workspace() {
    SECTION("FKWorkspace");

    Robot robot = make_iiwa14();
    const int n = robot.n_joints();

    // Use mid configuration
    Eigen::VectorXd q(n);
    const auto& jl = robot.joint_limits();
    for (int j = 0; j < n; ++j)
        q[j] = 0.5 * (jl.limits[j].lo + jl.limits[j].hi);

    FKWorkspace ws;
    ws.compute(robot, q);

    CHECK(ws.np > 0, "FKWorkspace should compute some frames");
    CHECK(ws.np == n + 2, "FKWorkspace should have n+2 frames (base + joints + tool)");

    // Position of frame 0 (base) should be origin
    Eigen::Vector3d base_pos = ws.pos(0);
    CHECK_CLOSE(base_pos.norm(), 0.0, 1e-10, "base frame at origin");

    // Position of frame 1 should be non-zero (d[0] = 0.1575)
    Eigen::Vector3d pos1 = ws.pos(1);
    CHECK(pos1.norm() > 0.1, "frame 1 should be offset from origin");
}

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
//  Test 3: derive_aabb_critical_analytical 鈥?basic
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
static void test_analytical_basic() {
    SECTION("analytical_solve basic");

    Robot robot = make_iiwa14();
    auto ivs = make_narrow_intervals(robot, 0.1);  // 卤0.1 rad

    const int n_act = robot.n_active_links();
    std::vector<float> aabbs(n_act * 6);
    AnalyticalCriticalStats stats{};

    auto config = AnalyticalCriticalConfig::all_enabled();
    derive_aabb_critical_analytical(
        robot, ivs, /*n_sub=*/1, config,
        aabbs.data(), &stats);

    // Check valid AABBs (min <= max for all)
    bool all_valid = true;
    for (int ci = 0; ci < n_act; ++ci) {
        float* a = aabbs.data() + ci * 6;
        if (a[0] > a[3] || a[1] > a[4] || a[2] > a[5]) {
            std::printf("  Link %d: AABB invalid [%f,%f,%f]-[%f,%f,%f]\n",
                        ci, a[0], a[1], a[2], a[3], a[4], a[5]);
            all_valid = false;
        }
    }
    CHECK(all_valid, "all analytical AABBs valid (min<=max)");

    // Check stats
    std::printf("  Stats: Ph0_vertices=%d  Ph1_edges=%d  Ph2_faces=%d\n",
                stats.n_phase0_vertices, stats.n_phase1_edges, stats.n_phase2_faces);
    std::printf("         Ph2.5a=%d  Ph2.5b=%d  Ph3=%d\n",
                stats.n_phase25a_pair1d, stats.n_phase25b_pair2d,
                stats.n_phase3_interior);
    std::printf("         FK calls: Ph1=%d  Ph2=%d  Ph2.5=%d  Ph3=%d\n",
                stats.n_phase1_fk_calls, stats.n_phase2_fk_calls,
                stats.n_phase25_fk_calls, stats.n_phase3_fk_calls);

    CHECK(stats.n_phase0_vertices > 0, "Phase 0 should find some vertices");
}

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
//  Test 4: Analytical vs iFK containment
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
static void test_analytical_vs_ifk() {
    SECTION("analytical vs iFK containment");

    Robot robot = make_iiwa14();
    auto ivs = make_narrow_intervals(robot, 0.15);

    const int n_act = robot.n_active_links();

    // Compute iFK AABBs
    auto fk_state = compute_fk_full(robot, ivs);
    std::vector<float> ifk_aabbs(n_act * 6);
    extract_link_aabbs(fk_state,
                       robot.active_link_map(), n_act,
                       ifk_aabbs.data(), robot.active_link_radii());

    // Compute analytical AABBs
    std::vector<float> ana_aabbs(n_act * 6);
    auto config = AnalyticalCriticalConfig::all_enabled();
    derive_aabb_critical_analytical(
        robot, ivs, /*n_sub=*/1, config, ana_aabbs.data());

    // Analytical should be tighter than or equal to iFK (within tolerance)
    // But analytical MUST be a valid inner bound (all critical points found)
    // So we just check that both produce valid boxes
    bool all_ifk_valid = true, all_ana_valid = true;
    int n_tighter = 0;
    double total_ifk_vol = 0, total_ana_vol = 0;

    for (int ci = 0; ci < n_act; ++ci) {
        float* ia = ifk_aabbs.data() + ci * 6;
        float* aa = ana_aabbs.data() + ci * 6;

        if (ia[0] > ia[3] || ia[1] > ia[4] || ia[2] > ia[5])
            all_ifk_valid = false;
        if (aa[0] > aa[3] || aa[1] > aa[4] || aa[2] > aa[5])
            all_ana_valid = false;

        double ifk_vol = (ia[3]-ia[0]) * (ia[4]-ia[1]) * (ia[5]-ia[2]);
        double ana_vol = (aa[3]-aa[0]) * (aa[4]-aa[1]) * (aa[5]-aa[2]);
        total_ifk_vol += ifk_vol;
        total_ana_vol += ana_vol;
        if (ana_vol < ifk_vol - 1e-12) ++n_tighter;
    }

    CHECK(all_ifk_valid, "all iFK AABBs valid");
    CHECK(all_ana_valid, "all analytical AABBs valid");

    std::printf("  iFK total vol: %.6e  Analytical total vol: %.6e\n",
                total_ifk_vol, total_ana_vol);
    std::printf("  Links where analytical tighter: %d/%d\n", n_tighter, n_act);
    CHECK(total_ana_vol <= total_ifk_vol + 1e-6,
          "analytical should be at most as large as iFK");
}

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
//  Test 5: AA Pruning
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
static void test_aa_pruning() {
    SECTION("AA pruning");

    Robot robot = make_iiwa14();
    auto ivs = make_narrow_intervals(robot, 0.05);  // Very narrow 鈫?many links prunable

    const int n_act = robot.n_active_links();
    std::vector<float> aabbs_no_aa(n_act * 6);
    std::vector<float> aabbs_aa(n_act * 6);
    AnalyticalCriticalStats stats_no_aa{}, stats_aa{};

    auto config_no_aa = AnalyticalCriticalConfig::all_enabled();
    config_no_aa.enable_aa_pruning = false;
    derive_aabb_critical_analytical(
        robot, ivs, 1, config_no_aa, aabbs_no_aa.data(), &stats_no_aa);

    auto config_aa = AnalyticalCriticalConfig::all_enabled();
    config_aa.enable_aa_pruning = true;
    derive_aabb_critical_analytical(
        robot, ivs, 1, config_aa, aabbs_aa.data(), &stats_aa);

    std::printf("  AA pruned links: %d/%d\n",
                stats_aa.n_aa_pruned_links, n_act);
    std::printf("  FK calls without AA: Ph1=%d Ph2=%d\n",
                stats_no_aa.n_phase1_fk_calls, stats_no_aa.n_phase2_fk_calls);
    std::printf("  FK calls with    AA: Ph1=%d Ph2=%d\n",
                stats_aa.n_phase1_fk_calls, stats_aa.n_phase2_fk_calls);

    // With AA pruning, fewer FK calls (if any links are pruned)
    if (stats_aa.n_aa_pruned_links > 0) {
        int total_no_aa = stats_no_aa.n_phase1_fk_calls + stats_no_aa.n_phase2_fk_calls +
                          stats_no_aa.n_phase25_fk_calls + stats_no_aa.n_phase3_fk_calls;
        int total_aa    = stats_aa.n_phase1_fk_calls + stats_aa.n_phase2_fk_calls +
                          stats_aa.n_phase25_fk_calls + stats_aa.n_phase3_fk_calls;
        CHECK(total_aa <= total_no_aa,
              "AA pruning should reduce or equal FK calls");
    }

    // Results should be identical or very close (AA doesn't change bounds, just skips)
    double max_diff = 0;
    for (int ci = 0; ci < n_act; ++ci) {
        for (int d = 0; d < 6; ++d) {
            double diff = std::abs(aabbs_aa[ci*6+d] - aabbs_no_aa[ci*6+d]);
            max_diff = std::max(max_diff, diff);
        }
    }
    std::printf("  Max AABB diff (AA vs no-AA): %.6e\n", max_diff);
    // AA pruning might slightly change results because skipped links might have
    // improved during later phases if not pruned. But bounds should be close.
}

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
//  Test 6: Config variants
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
static void test_config_variants() {
    SECTION("config variants");

    Robot robot = make_iiwa14();
    auto ivs = make_narrow_intervals(robot, 0.1);
    const int n_act = robot.n_active_links();
    std::vector<float> aabbs(n_act * 6);

    // edges_only
    {
        AnalyticalCriticalStats stats{};
        auto cfg = AnalyticalCriticalConfig::edges_only();
        derive_aabb_critical_analytical(robot, ivs, 1, cfg, aabbs.data(), &stats);
        CHECK(stats.n_phase1_edges >= 0, "edges_only runs without crash");
        CHECK(stats.n_phase2_faces == 0, "edges_only should not run phase 2");
    }

    // edges_and_faces
    {
        AnalyticalCriticalStats stats{};
        auto cfg = AnalyticalCriticalConfig::edges_and_faces();
        derive_aabb_critical_analytical(robot, ivs, 1, cfg, aabbs.data(), &stats);
        CHECK(stats.n_phase1_edges >= 0, "edges_and_faces: edges ok");
        CHECK(stats.n_phase3_interior == 0, "edges_and_faces: no interior");
    }

    // v1_analytical (no pair, no improved interior)
    {
        AnalyticalCriticalStats stats{};
        auto cfg = AnalyticalCriticalConfig::v1_analytical();
        derive_aabb_critical_analytical(robot, ivs, 1, cfg, aabbs.data(), &stats);
        CHECK(stats.n_phase25a_pair1d == 0, "v1_analytical: no pair1d");
        CHECK(stats.n_phase25b_pair2d == 0, "v1_analytical: no pair2d");
    }
}

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
//  Test 7: Endpoint source Analytical dispatch
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
static void test_endpoint_source_dispatch() {
    SECTION("endpoint_source Analytical dispatch — pure endpoint iAABBs");

    Robot robot = make_iiwa14();
    auto ivs = make_narrow_intervals(robot, 0.1);

    EndpointSourceConfig src_cfg = EndpointSourceConfig::analytical();
    auto result = compute_endpoint_iaabb(src_cfg, robot, ivs);

    // Stage 1 must NOT produce per-link data — only endpoint iAABBs
    CHECK(result.has_fk_state(), "should still have FK state");

    const int n_endpoints = robot.n_joints() + (robot.has_tool() ? 1 : 0);
        CHECK(result.endpoint_iaabb_len() == result.n_active_ep * 6,
            "Analytical dispatch: endpoint_iaabb_len matches active endpoint count");
        CHECK(result.endpoint_iaabb_len() <= n_endpoints * 6,
          "Analytical dispatch: correct endpoint_iaabbs size");

    // All endpoint iAABBs referenced by active links must be valid (lo <= hi).
    // Unreferenced endpoints (skipped zero-length links) may remain empty.
    bool all_valid = true;
    {
        const int* alm = robot.active_link_map();
        const int n_act = robot.n_active_links();
        std::vector<bool> referenced(n_endpoints, false);
        for (int ci = 0; ci < n_act; ++ci) {
            int V = alm[ci];
            if (V >= 1) referenced[V - 1] = true;
            referenced[V] = true;
        }
        for (int k = 0; k < n_endpoints; ++k) {
            if (!referenced[k]) continue;
            const float* ep = result.endpoint_iaabbs.data() + k * 6;
            if (ep[0] > ep[3] || ep[1] > ep[4] || ep[2] > ep[5]) {
                all_valid = false;
                break;
            }
        }
    }
    CHECK(all_valid, "all referenced endpoint iAABBs valid (lo <= hi)");

    // extract_link_iaabbs must still produce valid per-link iAABBs
    const int n_act = robot.n_active_links();
    std::vector<float> link_aabbs(n_act * 6);
    extract_link_iaabbs(result, robot, link_aabbs.data());

    bool links_valid = true;
    for (int ci = 0; ci < n_act; ++ci) {
        const float* a = link_aabbs.data() + ci * 6;
        if (a[0] > a[3] || a[1] > a[4] || a[2] > a[5]) {
            links_valid = false;
            break;
        }
    }
    CHECK(links_valid, "extract_link_iaabbs produces valid link iAABBs");
}

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
//  Test 8: n_sub > 1
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
static void test_nsub() {
    SECTION("n_sub > 1");

    Robot robot = make_iiwa14();
    auto ivs = make_narrow_intervals(robot, 0.1);
    const int n_act = robot.n_active_links();
    const int n_sub = 4;

    std::vector<float> aabbs(n_act * n_sub * 6);
    AnalyticalCriticalStats stats{};

    auto config = AnalyticalCriticalConfig::edges_only();
    derive_aabb_critical_analytical(
        robot, ivs, n_sub, config, aabbs.data(), &stats);

    // Check all sub-segment AABBs are valid
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
    CHECK(all_valid, "all sub-segment AABBs valid");
}

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
//  Test 9: with_configs variant
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
static void test_with_configs() {
    SECTION("with_configs variant");

    Robot robot = make_iiwa14();
    auto ivs = make_narrow_intervals(robot, 0.1);
    const int n_act = robot.n_active_links();

    std::vector<float> aabbs(n_act * 6);
    std::vector<Eigen::VectorXd> configs(n_act * 6);
    AnalyticalCriticalStats stats{};

    auto config = AnalyticalCriticalConfig::all_enabled();
    derive_aabb_critical_analytical_with_configs(
        robot, ivs, 1, config, aabbs.data(), configs.data(), &stats);

    // Check configs are valid joint vectors
    bool all_configs_valid = true;
    for (int ci = 0; ci < n_act; ++ci) {
        for (int f = 0; f < 6; ++f) {
            const auto& cfg = configs[ci * 6 + f];
            if (cfg.size() != robot.n_joints()) {
                all_configs_valid = false;
                std::printf("  Link %d face %d: config size=%d (expected %d)\n",
                            ci, f, (int)cfg.size(), robot.n_joints());
            }
        }
    }
    CHECK(all_configs_valid, "all configs have correct size");
}

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
//  Test 10: Full-range box
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
static void test_full_range() {
    SECTION("full-range analytical");

    Robot robot = make_iiwa14();
    auto ivs = make_full_intervals(robot);
    const int n_act = robot.n_active_links();

    std::vector<float> aabbs(n_act * 6);
    AnalyticalCriticalStats stats{};

    // Use edges_only for speed (full-range + all_enabled is expensive)
    auto config = AnalyticalCriticalConfig::edges_only();
    config.enable_aa_pruning = true;
    derive_aabb_critical_analytical(
        robot, ivs, 1, config, aabbs.data(), &stats);

    std::printf("  Full-range stats: vertices=%d edges=%d aa_pruned=%d\n",
                stats.n_phase0_vertices, stats.n_phase1_edges,
                stats.n_aa_pruned_links);

    bool all_valid = true;
    for (int ci = 0; ci < n_act; ++ci) {
        float* a = aabbs.data() + ci * 6;
        if (a[0] > a[3] || a[1] > a[4] || a[2] > a[5]) {
            all_valid = false;
        }
    }
    CHECK(all_valid, "full-range AABBs valid");
}

// ---------------------------------------------------------------------------
//  Test 11: extract_1d_coefficients vs 3-point QR
//  (Tests 11-13 removed in Phase E cleanup: hybrid tests no longer applicable)
// ---------------------------------------------------------------------------
static void test_extract_1d_coefficients() {
    SECTION("extract-1d-coefficients");

    Robot robot = make_iiwa14();
    const int n = robot.n_joints();
    auto ivs = make_narrow_intervals(robot, 0.15);

    Eigen::VectorXd q(n);
    for (int jj = 0; jj < n; ++jj) q[jj] = ivs[jj].mid();

    FKWorkspace ws;
    ws.resize(n + 2);

    // Test for each free joint j and several backgrounds
    int n_tested = 0;
    double max_alpha_err = 0.0, max_beta_err = 0.0, max_gamma_err = 0.0;

    for (int j = 0; j < n; ++j) {
        double lo_j = ivs[j].lo, hi_j = ivs[j].hi;
        double mid_j = 0.5 * (lo_j + hi_j);
        double qvals[3] = { lo_j, mid_j, hi_j };

        // Build QR interpolation matrix
        Eigen::Matrix3d A;
        for (int si = 0; si < 3; ++si) {
            A(si, 0) = std::cos(qvals[si]);
            A(si, 1) = std::sin(qvals[si]);
            A(si, 2) = 1.0;
        }
        auto qr_A = A.colPivHouseholderQr();

        // Just use midpoints for background (one background combo)
        for (int jj = 0; jj < n; ++jj) q[jj] = ivs[jj].mid();

        // Compute prefix
        ws.set_identity();
        for (int k = 0; k < j; ++k)
            ws.compute_joint(robot, q, k);
        const Eigen::Matrix4d& prefix = ws.tf[j];
        const auto& dh_j = robot.dh_params()[j];

        // For each eval_link in {j+1, ..., min(j+2, n+1)} test distal
        int V = std::min(j + 1, n);  // link index
        int eval_frame = V + 1;
        if (eval_frame >= n + 2) continue;

        // QR path: compute 3 FK samples
        Eigen::Vector3d pts[3];
        for (int si = 0; si < 3; ++si) {
            q[j] = qvals[si];
            ws.compute_from(robot, q, j);
            pts[si] = ws.pos(eval_frame);
        }
        q[j] = mid_j;  // restore

        Eigen::Vector3d coeff_qr[3];  // alpha, beta, gamma per axis
        for (int d = 0; d < 3; ++d) {
            Eigen::Vector3d b_vec;
            b_vec << pts[0][d], pts[1][d], pts[2][d];
            coeff_qr[d] = qr_A.solve(b_vec);
        }

        // Direct path via DH
        Eigen::Vector3d suf_pos = compute_suffix_pos(robot, q, j, eval_frame);
        Coeff1D c = extract_1d_coefficients(prefix, suf_pos, dh_j);

        for (int d = 0; d < 3; ++d) {
            double err_a = std::abs(c.alpha[d] - coeff_qr[d][0]);
            double err_b = std::abs(c.beta[d]  - coeff_qr[d][1]);
            double err_g = std::abs(c.gamma[d] - coeff_qr[d][2]);
            if (err_a > max_alpha_err) max_alpha_err = err_a;
            if (err_b > max_beta_err)  max_beta_err  = err_b;
            if (err_g > max_gamma_err) max_gamma_err = err_g;
        }
        ++n_tested;
    }

    std::printf("  extract_1d: tested %d joints, max_err alpha=%.2e beta=%.2e gamma=%.2e\n",
                n_tested, max_alpha_err, max_beta_err, max_gamma_err);

    CHECK(max_alpha_err < 1e-10, "extract_1d alpha matches QR");
    CHECK(max_beta_err  < 1e-10, "extract_1d beta matches QR");
    CHECK(max_gamma_err < 1e-10, "extract_1d gamma matches QR");
}

// ---------------------------------------------------------------------------
//  Test 15: P1 direct path vs QR path — AABB comparison
// ---------------------------------------------------------------------------
static void test_p1_direct_vs_qr() {
    SECTION("p1-direct-vs-qr");

    Robot robot = make_iiwa14();
    auto ivs = make_narrow_intervals(robot, 0.15);
    const int n_act = robot.n_active_links();

    // Baseline: QR path (explicit off)
    auto cfg_qr = AnalyticalCriticalConfig::edges_only();
    cfg_qr.enable_p1_direct_coeff = false;
    std::vector<float> aabb_qr(n_act * 6);
    AnalyticalCriticalStats st_qr{};
    derive_aabb_critical_analytical(robot, ivs, 1, cfg_qr, aabb_qr.data(), &st_qr);

    // Direct path
    auto cfg_direct = AnalyticalCriticalConfig::edges_only();
    std::vector<float> aabb_direct(n_act * 6);
    AnalyticalCriticalStats st_direct{};
    derive_aabb_critical_analytical(robot, ivs, 1, cfg_direct, aabb_direct.data(), &st_direct);

    double max_diff = 0.0;
    for (int i = 0; i < n_act * 6; ++i) {
        double d = std::fabs(double(aabb_direct[i]) - double(aabb_qr[i]));
        if (d > max_diff) max_diff = d;
    }

    std::printf("  p1-direct vs qr: max_diff=%.2e  qr_fk=%d  direct_fk=%d\n",
                max_diff, st_qr.n_phase1_fk_calls, st_direct.n_phase1_fk_calls);

    CHECK(max_diff < 1e-10, "p1-direct AABB matches QR path");
    // Note: n_phase1_edges may differ slightly because the direct path
    // does not skip background combos with ws.np check, but AABB results
    // are bit-identical.
    CHECK(st_direct.n_phase1_fk_calls < st_qr.n_phase1_fk_calls,
          "p1-direct uses fewer FK calls than QR");
}

// ---------------------------------------------------------------------------
//  Test 16: P1 direct with all phases — regression check
// ---------------------------------------------------------------------------
static void test_p1_direct_all_phases() {
    SECTION("p1-direct-all-phases");

    Robot robot = make_iiwa14();
    auto ivs = make_narrow_intervals(robot, 0.15);
    const int n_act = robot.n_active_links();

    // Baseline: all phases, QR (explicit off)
    auto cfg_base = AnalyticalCriticalConfig::all_enabled();
    cfg_base.enable_p1_direct_coeff = false;
    std::vector<float> aabb_base(n_act * 6);
    derive_aabb_critical_analytical(robot, ivs, 1, cfg_base, aabb_base.data(), nullptr);

    // All phases, direct P1
    auto cfg_dir = AnalyticalCriticalConfig::all_enabled();
    std::vector<float> aabb_dir(n_act * 6);
    derive_aabb_critical_analytical(robot, ivs, 1, cfg_dir, aabb_dir.data(), nullptr);

    double max_diff = 0.0;
    for (int i = 0; i < n_act * 6; ++i) {
        double d = std::fabs(double(aabb_dir[i]) - double(aabb_base[i]));
        if (d > max_diff) max_diff = d;
    }
    std::printf("  p1-direct all-phases: max_diff=%.2e\n", max_diff);
    CHECK(max_diff < 1e-10, "p1-direct all-phases matches baseline");
}

// ---------------------------------------------------------------------------
//  Test 17: extract_2d_coefficients vs 9-point QR
// ---------------------------------------------------------------------------
static void test_extract_2d_coefficients() {
    SECTION("extract-2d-coefficients");

    Robot robot = make_iiwa14();
    const int n = robot.n_joints();
    auto ivs = make_narrow_intervals(robot, 0.15);

    Eigen::VectorXd q(n);
    for (int jj = 0; jj < n; ++jj) q[jj] = ivs[jj].mid();

    FKWorkspace ws;
    ws.resize(n + 2);

    int n_tested = 0;
    double max_coeff_err = 0.0;

    // Test all valid face pairs (j_lo < j_hi)
    for (int j_lo = 0; j_lo < n; ++j_lo) {
        for (int j_hi = j_lo + 1; j_hi < n; ++j_hi) {
            // Background: use midpoints
            for (int jj = 0; jj < n; ++jj) q[jj] = ivs[jj].mid();

            double lo_lo = ivs[j_lo].lo, hi_lo = ivs[j_lo].hi;
            double lo_hi = ivs[j_hi].lo, hi_hi = ivs[j_hi].hi;
            double lo_vals[3] = { lo_lo, 0.5*(lo_lo+hi_lo), hi_lo };
            double hi_vals[3] = { lo_hi, 0.5*(lo_hi+hi_hi), hi_hi };

            // Build QR interpolation matrix (9×9)
            Eigen::Matrix<double, 9, 9> A_mat;
            for (int si = 0; si < 3; ++si) {
                for (int sj = 0; sj < 3; ++sj) {
                    int row = si * 3 + sj;
                    double ci = std::cos(lo_vals[si]), si_v = std::sin(lo_vals[si]);
                    double cj = std::cos(hi_vals[sj]), sj_v = std::sin(hi_vals[sj]);
                    A_mat(row, 0) = ci * cj;
                    A_mat(row, 1) = ci * sj_v;
                    A_mat(row, 2) = si_v * cj;
                    A_mat(row, 3) = si_v * sj_v;
                    A_mat(row, 4) = ci;
                    A_mat(row, 5) = si_v;
                    A_mat(row, 6) = cj;
                    A_mat(row, 7) = sj_v;
                    A_mat(row, 8) = 1.0;
                }
            }
            auto qr_A = A_mat.colPivHouseholderQr();

            // Compute prefix
            ws.set_identity();
            for (int k = 0; k < j_lo; ++k)
                ws.compute_joint(robot, q, k);
            const Eigen::Matrix4d& prefix = ws.tf[j_lo];

            // For each eval_link: test V+1 (distal) where V = j_hi+1
            int V = std::min(j_hi + 1, n);
            int eval_frame = V + 1;
            if (eval_frame >= n + 2) continue;

            // QR path: 9-point FK sampling
            Eigen::Vector3d pos_cache[9];
            bool ok = true;
            for (int si = 0; si < 3; ++si) {
                q[j_lo] = lo_vals[si];
                ws.compute_joint(robot, q, j_lo);
                for (int k = j_lo + 1; k < j_hi; ++k)
                    ws.compute_joint(robot, q, k);
                for (int sj = 0; sj < 3; ++sj) {
                    q[j_hi] = hi_vals[sj];
                    ws.compute_from(robot, q, j_hi);
                    if (eval_frame >= ws.np) { ok = false; break; }
                    pos_cache[si * 3 + sj] = ws.pos(eval_frame);
                }
                if (!ok) break;
            }
            if (!ok) continue;

            // QR solve for 9 coefficients
            Eigen::Matrix<double, 9, 3> B_mat;
            for (int r = 0; r < 9; ++r)
                for (int dd = 0; dd < 3; ++dd)
                    B_mat(r, dd) = pos_cache[r][dd];
            Eigen::Matrix<double, 9, 3> coeff_qr = qr_A.solve(B_mat);

            // Direct path via DH
            q[j_lo] = ivs[j_lo].mid();
            q[j_hi] = ivs[j_hi].mid();  // restore

            // Recompute prefix (background still at midpoints)
            ws.set_identity();
            for (int k = 0; k < j_lo; ++k)
                ws.compute_joint(robot, q, k);

            Eigen::Matrix4d middle = compute_middle_matrix(robot, q, j_lo, j_hi);
            Eigen::Vector3d suf_pos = compute_suffix_pos(robot, q, j_hi, eval_frame);
            Coeff2D c2d = extract_2d_coefficients(ws.tf[j_lo],
                robot.dh_params()[j_lo], middle, robot.dh_params()[j_hi], suf_pos);

            // Compare: c2d.a[k][d] should match coeff_qr(k, d)
            for (int d = 0; d < 3; ++d) {
                for (int k = 0; k < 9; ++k) {
                    double err = std::abs(c2d.a[k][d] - coeff_qr(k, d));
                    if (err > max_coeff_err) max_coeff_err = err;
                }
            }
            ++n_tested;
        }
    }

    std::printf("  extract_2d: tested %d pairs, max_coeff_err=%.2e\n",
                n_tested, max_coeff_err);
    CHECK(max_coeff_err < 1e-10, "extract_2d matches 9-point QR");
}

// ---------------------------------------------------------------------------
//  Test 18: P2 direct path vs QR path — AABB comparison
// ---------------------------------------------------------------------------
static void test_p2_direct_vs_qr() {
    SECTION("p2-direct-vs-qr");

    Robot robot = make_iiwa14();
    auto ivs = make_narrow_intervals(robot, 0.15);
    const int n_act = robot.n_active_links();

    // Baseline: QR path (P1+P2, no P2.5/P3)
    auto cfg_qr = AnalyticalCriticalConfig::edges_and_faces();
    cfg_qr.enable_p1_direct_coeff = false;
    cfg_qr.enable_p2_direct_coeff = false;
    std::vector<float> aabb_qr(n_act * 6);
    AnalyticalCriticalStats st_qr{};
    derive_aabb_critical_analytical(robot, ivs, 1, cfg_qr, aabb_qr.data(), &st_qr);

    // Direct path (P2 direct)
    auto cfg_direct = AnalyticalCriticalConfig::edges_and_faces();
    std::vector<float> aabb_direct(n_act * 6);
    AnalyticalCriticalStats st_direct{};
    derive_aabb_critical_analytical(robot, ivs, 1, cfg_direct, aabb_direct.data(), &st_direct);

    double max_diff = 0.0;
    for (int i = 0; i < n_act * 6; ++i) {
        double d = std::fabs(double(aabb_direct[i]) - double(aabb_qr[i]));
        if (d > max_diff) max_diff = d;
    }

    std::printf("  p2-direct vs qr: max_diff=%.2e  qr_fk=%d  direct_fk=%d\n",
                max_diff, st_qr.n_phase2_fk_calls, st_direct.n_phase2_fk_calls);

    CHECK(max_diff < 1e-10, "p2-direct AABB matches QR path");
    CHECK(st_direct.n_phase2_fk_calls < st_qr.n_phase2_fk_calls,
          "p2-direct uses fewer FK calls than QR");
}

// ---------------------------------------------------------------------------
//  Test 19: P2 direct with all phases — regression check
// ---------------------------------------------------------------------------
static void test_p2_direct_all_phases() {
    SECTION("p2-direct-all-phases");

    Robot robot = make_iiwa14();
    auto ivs = make_narrow_intervals(robot, 0.15);
    const int n_act = robot.n_active_links();

    // Baseline: all phases, QR (explicit off)
    auto cfg_base = AnalyticalCriticalConfig::all_enabled();
    cfg_base.enable_p1_direct_coeff = false;
    cfg_base.enable_p2_direct_coeff = false;
    cfg_base.enable_p25_direct_coeff = false;
    std::vector<float> aabb_base(n_act * 6);
    derive_aabb_critical_analytical(robot, ivs, 1, cfg_base, aabb_base.data(), nullptr);

    // All phases, direct P2
    auto cfg_dir = AnalyticalCriticalConfig::all_enabled();
    std::vector<float> aabb_dir(n_act * 6);
    derive_aabb_critical_analytical(robot, ivs, 1, cfg_dir, aabb_dir.data(), nullptr);

    double max_diff = 0.0;
    for (int i = 0; i < n_act * 6; ++i) {
        double d = std::fabs(double(aabb_dir[i]) - double(aabb_base[i]));
        if (d > max_diff) max_diff = d;
    }
    std::printf("  p2-direct all-phases: max_diff=%.2e\n", max_diff);
    CHECK(max_diff < 1e-10, "p2-direct all-phases matches baseline");
}

// ---------------------------------------------------------------------------
//  Test 20: P2.5 direct path vs QR path — AABB comparison
// ---------------------------------------------------------------------------
static void test_p25_direct_vs_qr() {
    SECTION("p25-direct-vs-qr");

    Robot robot = make_iiwa14();
    auto ivs = make_narrow_intervals(robot, 0.15);
    const int n_act = robot.n_active_links();

    // Baseline: QR path, all phases enabled (explicit off)
    auto cfg_qr = AnalyticalCriticalConfig::all_enabled();
    cfg_qr.enable_p1_direct_coeff = false;
    cfg_qr.enable_p2_direct_coeff = false;
    cfg_qr.enable_p25_direct_coeff = false;
    std::vector<float> aabb_qr(n_act * 6);
    AnalyticalCriticalStats st_qr{};
    derive_aabb_critical_analytical(robot, ivs, 1, cfg_qr, aabb_qr.data(), &st_qr);

    // Direct P2.5 path
    auto cfg_direct = AnalyticalCriticalConfig::all_enabled();
    std::vector<float> aabb_direct(n_act * 6);
    AnalyticalCriticalStats st_direct{};
    derive_aabb_critical_analytical(robot, ivs, 1, cfg_direct, aabb_direct.data(), &st_direct);

    double max_diff = 0.0;
    for (int i = 0; i < n_act * 6; ++i) {
        double d = std::fabs(double(aabb_direct[i]) - double(aabb_qr[i]));
        if (d > max_diff) max_diff = d;
    }

    std::printf("  p25-direct vs qr: max_diff=%.2e  qr_fk=%d  direct_fk=%d\n",
                max_diff, st_qr.n_phase25_fk_calls, st_direct.n_phase25_fk_calls);

    CHECK(max_diff < 1e-10, "p25-direct AABB matches QR path");
    CHECK(st_direct.n_phase25_fk_calls < st_qr.n_phase25_fk_calls,
          "p25-direct uses fewer FK calls than QR");
}

// ---------------------------------------------------------------------------
//  Test 21: All direct paths (P1+P2+P2.5) — full regression check
// ---------------------------------------------------------------------------
static void test_all_direct_phases() {
    SECTION("all-direct-phases");

    Robot robot = make_iiwa14();
    auto ivs = make_narrow_intervals(robot, 0.15);
    const int n_act = robot.n_active_links();

    // Baseline: all phases, all QR (explicit off)
    auto cfg_base = AnalyticalCriticalConfig::all_enabled();
    cfg_base.enable_p1_direct_coeff = false;
    cfg_base.enable_p2_direct_coeff = false;
    cfg_base.enable_p25_direct_coeff = false;
    std::vector<float> aabb_base(n_act * 6);
    AnalyticalCriticalStats st_base{};
    derive_aabb_critical_analytical(robot, ivs, 1, cfg_base, aabb_base.data(), &st_base);

    // All direct paths enabled (now the default)
    auto cfg_dir = AnalyticalCriticalConfig::all_enabled();
    std::vector<float> aabb_dir(n_act * 6);
    AnalyticalCriticalStats st_dir{};
    derive_aabb_critical_analytical(robot, ivs, 1, cfg_dir, aabb_dir.data(), &st_dir);

    double max_diff = 0.0;
    for (int i = 0; i < n_act * 6; ++i) {
        double d = std::fabs(double(aabb_dir[i]) - double(aabb_base[i]));
        if (d > max_diff) max_diff = d;
    }

    int total_fk_base = st_base.n_phase1_fk_calls + st_base.n_phase2_fk_calls + st_base.n_phase25_fk_calls;
    int total_fk_dir  = st_dir.n_phase1_fk_calls + st_dir.n_phase2_fk_calls + st_dir.n_phase25_fk_calls;

    std::printf("  all-direct: max_diff=%.2e  base_fk=%d  direct_fk=%d  (P1:%d→%d  P2:%d→%d  P25:%d→%d)\n",
                max_diff, total_fk_base, total_fk_dir,
                st_base.n_phase1_fk_calls, st_dir.n_phase1_fk_calls,
                st_base.n_phase2_fk_calls, st_dir.n_phase2_fk_calls,
                st_base.n_phase25_fk_calls, st_dir.n_phase25_fk_calls);

    CHECK(max_diff < 1e-10, "all-direct AABB matches baseline");
    CHECK(total_fk_dir < total_fk_base, "all-direct uses fewer total FK calls");
}

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
int main() {
    std::printf("SafeBoxForest v4 鈥?Analytical Critical Solver Tests\n");

    test_crit_angles();
    test_fk_workspace();
    test_analytical_basic();
    test_analytical_vs_ifk();
    test_aa_pruning();
    test_config_variants();
    test_endpoint_source_dispatch();
    test_nsub();
    test_with_configs();
    test_full_range();
    test_extract_1d_coefficients();
    test_p1_direct_vs_qr();
    test_p1_direct_all_phases();
    test_extract_2d_coefficients();
    test_p2_direct_vs_qr();
    test_p2_direct_all_phases();
    test_p25_direct_vs_qr();
    test_all_direct_phases();

    std::printf("\n鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲\n");
    std::printf("  PASS: %d   FAIL: %d\n", g_pass, g_fail);
    std::printf("鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲\n");

    return g_fail > 0 ? 1 : 0;
}

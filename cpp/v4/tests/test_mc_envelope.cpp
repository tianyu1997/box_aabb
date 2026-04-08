// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — compute_mc_link_iaabb tests
//
//  Tests for the Monte Carlo link iAABB reference baseline.
//
//  Build: cmake --build build --config Release --target test_mc_envelope
//  Run:   build\Release\test_mc_envelope.exe
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/robot/robot.h"
#include "sbf/envelope/mc_envelope.h"
#include "sbf/envelope/endpoint_source.h"
#include "sbf/robot/fk.h"
#include "sbf/core/types.h"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

using namespace sbf;
using namespace sbf::envelope;

// ─── Test harness ────────────────────────────────────────────────────────────

static int g_pass = 0, g_fail = 0;

#define CHECK(cond, msg) do {                                               \
    if (!(cond)) {                                                          \
        std::printf("  FAIL  %s:%d  %s\n", __FILE__, __LINE__, msg);       \
        ++g_fail;                                                           \
    } else { ++g_pass; }                                                    \
} while(0)

#define CHECK_CLOSE(a, b, tol, msg) \
    CHECK(std::abs((a) - (b)) < (tol), msg)

#define SECTION(name) std::printf("\n── %s ──\n", (name))

// ─── IIWA14 factory ──────────────────────────────────────────────────────────

static Robot make_iiwa14() {
    std::vector<DHParam> dh = {
        {  0.0,     0.0, 0.1575, 0.0, 0},
        { -HALF_PI, 0.0, 0.0,    0.0, 0},
        {  HALF_PI, 0.0, 0.2025, 0.0, 0},
        {  HALF_PI, 0.0, 0.0,    0.0, 0},
        { -HALF_PI, 0.0, 0.2155, 0.0, 0},
        { -HALF_PI, 0.0, 0.0,    0.0, 0},
        {  HALF_PI, 0.0, 0.081,  0.0, 0},
    };
    JointLimits limits;
    limits.limits = {
        {-1.865, 1.866}, {-0.100, 1.087}, {-0.663, 0.662},
        {-2.094,-0.372}, {-0.619, 0.620}, {-1.095, 1.258},
        { 1.050, 2.091},
    };
    DHParam tool{0.0, 0.0, 0.185, 0.0, 0};
    std::vector<double> radii = {0.08, 0.08, 0.06, 0.06, 0.04, 0.04, 0.04, 0.03};
    return Robot("iiwa14", dh, limits, tool, radii);
}

// Build full-range intervals for a robot
static std::vector<Interval> full_range(const Robot& robot) {
    const auto& lim = robot.joint_limits().limits;
    std::vector<Interval> ivs(robot.n_joints());
    for (int j = 0; j < robot.n_joints(); ++j)
        ivs[j] = {lim[j].lo, lim[j].hi};
    return ivs;
}

// Build tight single-config intervals
static std::vector<Interval> point_intervals(const Robot& robot,
                                             const Eigen::VectorXd& q)
{
    std::vector<Interval> ivs(robot.n_joints());
    for (int j = 0; j < robot.n_joints(); ++j)
        ivs[j] = {q[j], q[j]};
    return ivs;
}

// ─── Tests ───────────────────────────────────────────────────────────────────

static void test_output_validity()
{
    SECTION("Output validity (no NaN/Inf, lo <= hi)");

    Robot robot = make_iiwa14();
    const int n_act = robot.n_active_links();
    auto intervals = full_range(robot);

    std::vector<float> out(n_act * 6);
    compute_mc_link_iaabb(robot, intervals, 1000, 42, out.data());

    bool all_valid = true;
    bool all_ordered = true;
    for (int a = 0; a < n_act; ++a) {
        const float* p = out.data() + a * 6;
        for (int d = 0; d < 6; ++d) {
            if (!std::isfinite(p[d])) { all_valid = false; break; }
        }
        for (int d = 0; d < 3; ++d) {
            if (p[d] > p[d+3]) { all_ordered = false; break; }
        }
    }
    CHECK(all_valid,   "All output values are finite");
    CHECK(all_ordered, "lo <= hi for every axis of every active link");
}

static void test_point_interval()
{
    SECTION("Point interval: AABB must contain the FK position");

    Robot robot = make_iiwa14();
    const int n_act  = robot.n_active_links();
    const int* alm   = robot.active_link_map();

    // Zero configuration
    Eigen::VectorXd q = Eigen::VectorXd::Zero(robot.n_joints());
    auto ivs = point_intervals(robot, q);

    std::vector<float> out(n_act * 6);
    compute_mc_link_iaabb(robot, ivs, 1, 0, out.data());

    auto positions = fk_link_positions(robot, q);

    bool all_contained = true;
    for (int a = 0; a < n_act; ++a) {
        const float* p = out.data() + a * 6;
        double r = robot.link_radius(alm[a]);
        for (int ep = 0; ep < 2; ++ep) {
            const auto& pt = positions[alm[a] + ep];
            for (int d = 0; d < 3; ++d) {
                float lo = p[d], hi = p[d + 3];
                float v = static_cast<float>(pt[d]);
                if (v - (float)r < lo - 1e-4f || v + (float)r > hi + 1e-4f)
                    all_contained = false;
            }
        }
    }
    CHECK(all_contained,
          "FK endpoints (+ radius) are contained in MC AABB for point interval");
}

static void test_conservative_vs_analytical()
{
    // The Analytical endpoint source is conservative; the MC reference should
    // generally be TIGHTER (smaller bounds) than Analytical for sufficient n_mc.
    // This is a statistical test — with 100000 samples it should hold for most
    // links and axes, so we only require ≥ 80% of bounds to satisfy this.
    SECTION("MC is not looser than Analytical (statistical, 100k samples)");

    Robot robot = make_iiwa14();
    const int n_act = robot.n_active_links();
    auto intervals = full_range(robot);

    std::vector<float> mc_out(n_act * 6);
    compute_mc_link_iaabb(robot, intervals, 100000, 7, mc_out.data());

    EndpointSourceConfig cfg = EndpointSourceConfig::analytical();
    auto ep_result = compute_endpoint_iaabb(cfg, robot, intervals);
    std::vector<float> anal_out(n_act * 6);
    extract_link_iaabbs(ep_result, robot, anal_out.data());

    int total = 0, mc_leq_anal = 0;
    for (int a = 0; a < n_act; ++a) {
        for (int d = 0; d < 3; ++d) {
            // lo: Analytical lo <= MC lo (anal encloses mc)
            if (anal_out[a*6+d] <= mc_out[a*6+d] + 1e-3f) ++mc_leq_anal;
            // hi: Analytical hi >= MC hi
            if (anal_out[a*6+3+d] >= mc_out[a*6+3+d] - 1e-3f) ++mc_leq_anal;
            total += 2;
        }
    }
    double ratio = (total > 0) ? (double)mc_leq_anal / total : 0.0;
    std::printf("  Analytical encloses MC: %.1f%% of bounds (threshold 80%%)\n",
                100.0 * ratio);
    CHECK(ratio >= 0.80,
          "Analytical encloses MC for >= 80% of bounds (conservative property)");
}

static void test_determinism()
{
    SECTION("Same seed → identical output");

    Robot robot = make_iiwa14();
    const int n_act = robot.n_active_links();
    auto intervals = full_range(robot);

    std::vector<float> out1(n_act * 6), out2(n_act * 6);
    compute_mc_link_iaabb(robot, intervals, 5000, 123, out1.data());
    compute_mc_link_iaabb(robot, intervals, 5000, 123, out2.data());

    bool identical = (out1 == out2);
    CHECK(identical, "Identical seed produces identical output");
}

static void test_more_samples_tighter()
{
    SECTION("More samples → bounds do not expand (monotone convergence)");

    Robot robot = make_iiwa14();
    const int n_act = robot.n_active_links();
    auto intervals = full_range(robot);

    // 10k samples
    std::vector<float> out_small(n_act * 6);
    compute_mc_link_iaabb(robot, intervals, 10000, 42, out_small.data());

    // 100k samples (same seed, more draws → wider or equal bounds)
    std::vector<float> out_large(n_act * 6);
    compute_mc_link_iaabb(robot, intervals, 100000, 42, out_large.data());

    // Larger sample should produce an equal or larger bounding box
    // (large ≥ small in the sense: lo_large ≤ lo_small, hi_large ≥ hi_small)
    bool monotone = true;
    for (int a = 0; a < n_act; ++a) {
        for (int d = 0; d < 3; ++d) {
            if (out_large[a*6+d]     > out_small[a*6+d]     + 1e-4f) monotone = false;
            if (out_large[a*6+3+d]   < out_small[a*6+3+d]   - 1e-4f) monotone = false;
        }
    }
    CHECK(monotone, "100k-sample AABB encloses 10k-sample AABB (monotone convergence)");
}

// ─── Main ─────────────────────────────────────────────────────────────────────

int main()
{
    test_output_validity();
    test_point_interval();
    test_determinism();
    test_more_samples_tighter();
    test_conservative_vs_analytical();

    std::printf("\n═══ Results: %d passed, %d failed ═══\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}

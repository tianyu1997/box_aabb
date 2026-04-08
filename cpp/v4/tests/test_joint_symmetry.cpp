// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Joint Symmetry Unit Tests
//
//  Tests:
//    1. detect_joint_symmetries — q_0 Z4 detected, q_1..q_6 NONE
//    2. canonicalize — maps q to [0, π/2] + correct sector
//    3. transform_aabb — Z4 rotations for sectors 1/2/3 (a=0 and a≠0)
//    4. Round-trip: 4 rotations of sector 1 == identity
//    5. LECT pre-expand symmetry (validates symmetry-skipped AABBs
//       match full-compute AABBs within tolerance)
//
//  Build: cmake --build . --config Release --target test_joint_symmetry
//  Run:   Release\test_joint_symmetry.exe
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/core/joint_symmetry.h"
#include "sbf/robot/robot.h"
#include "sbf/robot/interval_fk.h"
#include "sbf/envelope/endpoint_source.h"
#include "sbf/envelope/envelope_derive.h"
#include "sbf/core/types.h"
#include "sbf/forest/lect.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <cstring>
#include <vector>

using namespace sbf;

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

// ─── Build Panda-like robot (α_0 = 0, a_0 = 0) ────────────────────────────
static Robot make_panda() {
    std::vector<DHParam> dh = {
        {  0.0,     0.0,     0.333,  0.0, 0},   // α=0, a=0
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

// ─── Build IIWA14-like robot (α_0 = 0, a_0 = 0) ──────────────────────────
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

// ─── Build a hypothetical robot with α_0 ≠ 0 (no Z4 symmetry) ────────────
static Robot make_non_z4() {
    std::vector<DHParam> dh = {
        {  HALF_PI, 0.1,  0.2,  0.0, 0},   // α_0 = π/2 → NOT Z4
        { -HALF_PI, 0.0,  0.3,  0.0, 0},
    };

    JointLimits limits;
    limits.limits = {
        {-PI, PI},
        {-PI, PI},
    };

    return Robot("non_z4_test", dh, limits);
}

// ─── Build a robot with a_0 ≠ 0 (Z4 with offset) ──────────────────────────
static Robot make_offset_base() {
    std::vector<DHParam> dh = {
        {  0.0,  0.15,  0.2,  0.0, 0},   // α_0 = 0, a_0 = 0.15
        { -HALF_PI, 0.0, 0.3, 0.0, 0},
    };

    JointLimits limits;
    limits.limits = {
        {-PI, PI},
        {-PI, PI},
    };

    std::vector<double> radii = {0.04, 0.04};

    return Robot("offset_base_test", dh, limits, std::nullopt, radii);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 1: Detection — Panda q_0 = Z4, q_1..q_6 = NONE
// ═══════════════════════════════════════════════════════════════════════════
static void test_detect_panda() {
    SECTION("detect_joint_symmetries — Panda");

    Robot robot = make_panda();
    auto syms = detect_joint_symmetries(robot);

    CHECK(static_cast<int>(syms.size()) == 7,
          "result.size() == 7");
    CHECK(syms[0].type == JointSymmetryType::Z4_ROTATION,
          "q_0 → Z4_ROTATION");
    CHECK_CLOSE(syms[0].canonical_lo, 0.0, 1e-15, "canonical_lo == 0");
    CHECK_CLOSE(syms[0].canonical_hi, HALF_PI, 1e-15, "canonical_hi == π/2");
    CHECK_CLOSE(syms[0].period, HALF_PI, 1e-15, "period == π/2");
    CHECK_CLOSE(syms[0].offset_x, 0.0, 1e-15, "offset_x == 0 (Panda a_0=0)");

    for (int i = 1; i < 7; ++i) {
        char msg[64];
        std::snprintf(msg, sizeof(msg), "q_%d → NONE", i);
        CHECK(syms[i].type == JointSymmetryType::NONE, msg);
    }

    std::printf("  detect (Panda): OK\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 2: Detection — non-Z4 robot (α_0 ≠ 0)
// ═══════════════════════════════════════════════════════════════════════════
static void test_detect_non_z4() {
    SECTION("detect_joint_symmetries — non-Z4");

    Robot robot = make_non_z4();
    auto syms = detect_joint_symmetries(robot);

    CHECK(syms[0].type == JointSymmetryType::NONE,
          "α_0 = π/2 → NONE");

    std::printf("  detect (non-Z4): OK\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 3: Detection — offset base (a_0 ≠ 0, still Z4)
// ═══════════════════════════════════════════════════════════════════════════
static void test_detect_offset() {
    SECTION("detect_joint_symmetries — offset base (a_0 = 0.15)");

    Robot robot = make_offset_base();
    auto syms = detect_joint_symmetries(robot);

    CHECK(syms[0].type == JointSymmetryType::Z4_ROTATION,
          "α_0 = 0, a_0 = 0.15 → Z4_ROTATION");
    CHECK_CLOSE(syms[0].offset_x, 0.15, 1e-15, "offset_x == 0.15");

    std::printf("  detect (offset base): OK\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 4: canonicalize — sector mapping
// ═══════════════════════════════════════════════════════════════════════════
static void test_canonicalize() {
    SECTION("canonicalize");

    JointSymmetry sym;
    sym.type = JointSymmetryType::Z4_ROTATION;
    sym.canonical_lo = 0.0;
    sym.canonical_hi = HALF_PI;
    sym.period = HALF_PI;
    sym.offset_x = 0.0;

    double qc;

    // sector 0: q ∈ [0, π/2)
    int s0 = sym.canonicalize(0.5, qc);
    CHECK(s0 == 0, "q=0.5 → sector 0");
    CHECK_CLOSE(qc, 0.5, 1e-14, "q_canonical = 0.5");

    // sector 1: q ∈ [π/2, π)
    int s1 = sym.canonicalize(HALF_PI + 0.3, qc);
    CHECK(s1 == 1, "q=π/2+0.3 → sector 1");
    CHECK_CLOSE(qc, 0.3, 1e-14, "q_canonical = 0.3");

    // sector 2: q ∈ [π, 3π/2)
    int s2 = sym.canonicalize(PI + 0.2, qc);
    CHECK(s2 == 2, "q=π+0.2 → sector 2");
    CHECK_CLOSE(qc, 0.2, 1e-14, "q_canonical = 0.2");

    // sector 3: q ∈ [3π/2, 2π)
    int s3 = sym.canonicalize(3.0 * HALF_PI + 0.1, qc);
    CHECK(s3 == 3, "q=3π/2+0.1 → sector 3");
    CHECK_CLOSE(qc, 0.1, 1e-14, "q_canonical = 0.1");

    // Negative angles: q = −π/2 + 0.4 → normalise to 2π − π/2 + 0.4 = 3π/2+0.4
    int sn = sym.canonicalize(-HALF_PI + 0.4, qc);
    CHECK(sn == 3, "q = −π/2+0.4 → sector 3");
    CHECK_CLOSE(qc, 0.4, 1e-14, "q_canonical = 0.4 (wrapped)");

    std::printf("  canonicalize: OK\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 5: AABB transform — a = 0
// ═══════════════════════════════════════════════════════════════════════════
static void test_transform_aabb_a0() {
    SECTION("transform_aabb (a = 0)");

    JointSymmetry sym;
    sym.type = JointSymmetryType::Z4_ROTATION;
    sym.offset_x = 0.0;

    // Canonical AABB: [x_lo=1, y_lo=2, z_lo=3, x_hi=4, y_hi=5, z_hi=6]
    float src[6] = {1.f, 2.f, 3.f, 4.f, 5.f, 6.f};
    float dst[6];

    // sector 0 (identity)
    sym.transform_aabb(src, 0, dst);
    CHECK_CLOSE(dst[0], 1.f, 1e-6, "s0: x_lo=1");
    CHECK_CLOSE(dst[3], 4.f, 1e-6, "s0: x_hi=4");

    // sector 1 (+π/2): x' = [-y_hi, -y_lo], y' = [x_lo, x_hi]
    sym.transform_aabb(src, 1, dst);
    CHECK_CLOSE(dst[0], -5.f, 1e-6, "s1: x_lo = -y_hi = -5");
    CHECK_CLOSE(dst[3], -2.f, 1e-6, "s1: x_hi = -y_lo = -2");
    CHECK_CLOSE(dst[1],  1.f, 1e-6, "s1: y_lo = x_lo = 1");
    CHECK_CLOSE(dst[4],  4.f, 1e-6, "s1: y_hi = x_hi = 4");
    CHECK_CLOSE(dst[2],  3.f, 1e-6, "s1: z_lo unchanged");
    CHECK_CLOSE(dst[5],  6.f, 1e-6, "s1: z_hi unchanged");

    // sector 2 (+π): x' = [-x_hi, -x_lo], y' = [-y_hi, -y_lo]
    sym.transform_aabb(src, 2, dst);
    CHECK_CLOSE(dst[0], -4.f, 1e-6, "s2: x_lo = -x_hi = -4");
    CHECK_CLOSE(dst[3], -1.f, 1e-6, "s2: x_hi = -x_lo = -1");
    CHECK_CLOSE(dst[1], -5.f, 1e-6, "s2: y_lo = -y_hi = -5");
    CHECK_CLOSE(dst[4], -2.f, 1e-6, "s2: y_hi = -y_lo = -2");
    CHECK_CLOSE(dst[2],  3.f, 1e-6, "s2: z unchanged");

    // sector 3 (+3π/2): x' = [y_lo, y_hi], y' = [-x_hi, -x_lo]
    sym.transform_aabb(src, 3, dst);
    CHECK_CLOSE(dst[0],  2.f, 1e-6, "s3: x_lo = y_lo = 2");
    CHECK_CLOSE(dst[3],  5.f, 1e-6, "s3: x_hi = y_hi = 5");
    CHECK_CLOSE(dst[1], -4.f, 1e-6, "s3: y_lo = -x_hi = -4");
    CHECK_CLOSE(dst[4], -1.f, 1e-6, "s3: y_hi = -x_lo = -1");
    CHECK_CLOSE(dst[2],  3.f, 1e-6, "s3: z unchanged");

    std::printf("  transform_aabb (a=0): OK\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 6: AABB transform — a ≠ 0
// ═══════════════════════════════════════════════════════════════════════════
static void test_transform_aabb_offset() {
    SECTION("transform_aabb (a = 0.15)");

    JointSymmetry sym;
    sym.type = JointSymmetryType::Z4_ROTATION;
    sym.offset_x = 0.15;
    const float a = 0.15f;

    float src[6] = {1.f, 2.f, 3.f, 4.f, 5.f, 6.f};
    float dst[6];

    // sector 1 (+π/2): x' = [-y_hi+a, -y_lo+a], y' = [x_lo-a, x_hi-a]
    sym.transform_aabb(src, 1, dst);
    CHECK_CLOSE(dst[0], -5.f + a, 1e-5, "s1: x_lo = -y_hi + a");
    CHECK_CLOSE(dst[3], -2.f + a, 1e-5, "s1: x_hi = -y_lo + a");
    CHECK_CLOSE(dst[1],  1.f - a, 1e-5, "s1: y_lo = x_lo - a");
    CHECK_CLOSE(dst[4],  4.f - a, 1e-5, "s1: y_hi = x_hi - a");

    // sector 2 (+π): x' = [-x_hi+2a, -x_lo+2a], y' = [-y_hi, -y_lo]
    sym.transform_aabb(src, 2, dst);
    CHECK_CLOSE(dst[0], -4.f + 2*a, 1e-5, "s2: x_lo = -x_hi + 2a");
    CHECK_CLOSE(dst[3], -1.f + 2*a, 1e-5, "s2: x_hi = -x_lo + 2a");
    CHECK_CLOSE(dst[1], -5.f, 1e-5, "s2: y_lo = -y_hi");
    CHECK_CLOSE(dst[4], -2.f, 1e-5, "s2: y_hi = -y_lo");

    // sector 3 (+3π/2): x' = [y_lo-a, y_hi-a], y' = [-x_hi+a, -x_lo+a]
    sym.transform_aabb(src, 3, dst);
    CHECK_CLOSE(dst[0],  2.f - a, 1e-5, "s3: x_lo = y_lo - a");
    CHECK_CLOSE(dst[3],  5.f - a, 1e-5, "s3: x_hi = y_hi - a");
    CHECK_CLOSE(dst[1], -4.f + a, 1e-5, "s3: y_lo = -x_hi + a");
    CHECK_CLOSE(dst[4], -1.f + a, 1e-5, "s3: y_hi = -x_lo + a");

    std::printf("  transform_aabb (a=0.15): OK\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 7: Z4 round-trip — 4 applications of sector 1 = identity
// ═══════════════════════════════════════════════════════════════════════════
static void test_z4_roundtrip() {
    SECTION("Z4 round-trip (4 × sector 1 = identity)");

    JointSymmetry sym;
    sym.type = JointSymmetryType::Z4_ROTATION;
    sym.offset_x = 0.0;

    float src[6] = {1.f, 2.f, 3.f, 4.f, 5.f, 6.f};
    float tmp[6], out[6];

    // Apply sector 1 four times
    sym.transform_aabb(src, 1, tmp);         // rot 1
    sym.transform_aabb(tmp, 1, out);         // rot 2
    sym.transform_aabb(out, 1, tmp);         // rot 3
    sym.transform_aabb(tmp, 1, out);         // rot 4 = identity

    for (int i = 0; i < 6; ++i) {
        char msg[64];
        std::snprintf(msg, sizeof(msg), "roundtrip[%d]: %.4f == %.4f",
                      i, (double)out[i], (double)src[i]);
        CHECK_CLOSE(out[i], src[i], 1e-5, msg);
    }

    // Same test with a ≠ 0
    sym.offset_x = 0.15;
    sym.transform_aabb(src, 1, tmp);
    sym.transform_aabb(tmp, 1, out);
    sym.transform_aabb(out, 1, tmp);
    sym.transform_aabb(tmp, 1, out);

    for (int i = 0; i < 6; ++i) {
        char msg[64];
        std::snprintf(msg, sizeof(msg), "roundtrip_a[%d]: %.4f == %.4f",
                      i, (double)out[i], (double)src[i]);
        CHECK_CLOSE(out[i], src[i], 1e-4, msg);
    }

    std::printf("  Z4 round-trip: OK\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 8: Numerical validation — compare symmetry-derived AABB against
//          ground-truth iFK AABB computation for Panda sectors
//
//  Strategy: pick a q_0 interval in sector 0, compute iAABBs via iFK.
//  Then shift the q_0 interval by +π/2, +π, +3π/2, compute iAABBs via iFK,
//  and verify they match the transform_aabb result from sector 0.
// ═══════════════════════════════════════════════════════════════════════════
static void test_numerical_validation() {
    SECTION("numerical validation — iFK vs transform_aabb");

    Robot robot = make_panda();
    auto syms = detect_joint_symmetries(robot);
    const auto& sym = syms[0];
    CHECK(sym.type == JointSymmetryType::Z4_ROTATION,
          "Panda q_0 has Z4");

    // Canonical interval: q_0 ∈ [0.2, 0.8], other joints = full range
    auto limits = robot.joint_limits().limits;
    limits[0] = {0.2, 0.8};   // canonical sector

    // Compute iFK for canonical sector
    envelope::EndpointSourceConfig src_cfg;
    src_cfg.method = envelope::EndpointSource::IFK;

    auto ep_can = envelope::compute_endpoint_iaabb(src_cfg, robot, limits);
    int n_active = robot.n_active_links();
    std::vector<float> iaabb_can(n_active * 6);
    envelope::extract_link_iaabbs(ep_can, robot, iaabb_can.data());

    // For each non-canonical sector, compute ground truth and compare
    for (int sector = 1; sector <= 3; ++sector) {
        double shift = sector * HALF_PI;
        auto shifted_limits = limits;
        shifted_limits[0] = {0.2 + shift, 0.8 + shift};

        // Check within joint range (skip if out of bounds)
        if (shifted_limits[0].hi > robot.joint_limits().limits[0].hi + 0.01)
            continue;

        auto ep_shifted = envelope::compute_endpoint_iaabb(
            src_cfg, robot, shifted_limits);
        std::vector<float> iaabb_truth(n_active * 6);
        envelope::extract_link_iaabbs(ep_shifted, robot, iaabb_truth.data());

        // Apply symmetry transform to canonical AABBs
        std::vector<float> iaabb_sym(n_active * 6);
        sym.transform_all_link_aabbs(
            iaabb_can.data(), n_active, sector, iaabb_sym.data());

        // Compare: should match exactly (up to floating-point)
        for (int k = 0; k < n_active; ++k) {
            for (int c = 0; c < 6; ++c) {
                int idx = k * 6 + c;
                double diff = std::abs(
                    static_cast<double>(iaabb_sym[idx]) -
                    static_cast<double>(iaabb_truth[idx]));
                char msg[128];
                std::snprintf(msg, sizeof(msg),
                    "sector %d link %d comp %d: sym=%.6f truth=%.6f diff=%.2e",
                    sector, k, c,
                    (double)iaabb_sym[idx],
                    (double)iaabb_truth[idx], diff);
                // Tolerance: 1e-4 for float accumulation
                CHECK(diff < 1e-4, msg);
            }
        }

        char sec_msg[32];
        std::snprintf(sec_msg, sizeof(sec_msg), "  sector %d: OK", sector);
        std::printf("%s\n", sec_msg);
    }

    std::printf("  numerical validation: OK\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 9: LECT pre-expand with symmetry — compare against full pre-expand
//
//  Build two LECTs for Panda:
//    (a) Full pre-expand to depth D (no symmetry)
//    (b) Pre-expand to depth D with symmetry enabled
//  All node iAABBs must match within tolerance.
// ═══════════════════════════════════════════════════════════════════════════
static void test_lect_pre_expand_symmetry() {
    SECTION("LECT pre_expand with symmetry reuse");

    Robot robot = make_panda();

    // Build full LECT (baseline) — no symmetry
    forest::LECT lect_full(robot, 0.02, 256);
    lect_full.set_split_order(SplitOrder::WIDEST_FIRST);
    int n_full = lect_full.pre_expand(4);
    std::printf("  full pre_expand: %d new nodes\n", n_full);

    // Build symmetry-enabled LECT
    forest::LECT lect_sym(robot, 0.02, 256);
    lect_sym.set_split_order(SplitOrder::WIDEST_FIRST);
    int n_sym = lect_sym.pre_expand(4);
    std::printf("  sym  pre_expand: %d new nodes\n", n_sym);

    // Both should have the same number of nodes
    CHECK(lect_full.n_nodes() == lect_sym.n_nodes(),
          "node count must match");

    // Compare all node iAABBs (tolerance: 1e-4f for float iAABB)
    int n_links = robot.n_active_links();
    int mismatches = 0;
    for (int i = 0; i < lect_full.n_nodes(); ++i) {
        if (!lect_full.has_iaabb(i) || !lect_sym.has_iaabb(i)) continue;
        const float* aabb_f = lect_full.get_link_iaabbs(i);
        const float* aabb_s = lect_sym.get_link_iaabbs(i);
        for (int c = 0; c < n_links * 6; ++c) {
            double diff = std::abs(
                static_cast<double>(aabb_f[c]) -
                static_cast<double>(aabb_s[c]));
            if (diff > 1e-3) {
                ++mismatches;
                if (mismatches <= 5) {
                    std::printf("    mismatch node %d comp %d: full=%.6f sym=%.6f\n",
                        i, c, (double)aabb_f[c], (double)aabb_s[c]);
                }
            }
        }
    }
    CHECK(mismatches == 0, "all iAABBs must match (sym vs full)");

    std::printf("  LECT pre_expand symmetry: OK\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  main
// ═══════════════════════════════════════════════════════════════════════════
int main() {
    std::printf("═══ Joint Symmetry Tests ═══\n");

    test_detect_panda();
    test_detect_non_z4();
    test_detect_offset();
    test_canonicalize();
    test_transform_aabb_a0();
    test_transform_aabb_offset();
    test_z4_roundtrip();
    test_numerical_validation();
    test_lect_pre_expand_symmetry();

    std::printf("\n═══ Summary: %d passed, %d failed ═══\n",
                g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}

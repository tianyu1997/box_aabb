// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Joint Symmetry Detection & AABB Transform
//  Module: sbf::core
//
//  Implementation of joint_symmetry.h.
//  Mathematical derivation: doc/PLAN_JOINT_SYMMETRY_CACHE.md §5, §10
// ═══════════════════════════════════════════════════════════════════════════

#include "sbf/core/joint_symmetry.h"
#include "sbf/robot/robot.h"
#include "sbf/core/types.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace sbf {

// ═══════════════════════════════════════════════════════════════════════════
//  JointSymmetry::canonicalize
//
//  Map q into the canonical interval, return sector index (0-3 for Z4).
//  Algorithm:
//    1. Shift q into [canonical_lo, canonical_lo + 2π)
//    2. sector = floor((q_shifted − canonical_lo) / period)
//    3. q_canonical = q_shifted − sector * period
// ═══════════════════════════════════════════════════════════════════════════

int JointSymmetry::canonicalize(double q, double& q_canonical) const {
    if (type == JointSymmetryType::NONE) {
        q_canonical = q;
        return 0;
    }

    // Normalise q into [canonical_lo, canonical_lo + 2π)
    const double TWO_PI = 2.0 * PI;
    double q_norm = std::fmod(q - canonical_lo, TWO_PI);
    if (q_norm < 0.0) q_norm += TWO_PI;
    q_norm += canonical_lo;

    // Determine sector (for Z4: period = π/2, sectors 0..3)
    int sector = static_cast<int>(std::floor(
        (q_norm - canonical_lo) / period));
    sector = std::clamp(sector, 0, 3);

    q_canonical = q_norm - sector * period;
    return sector;
}

// ═══════════════════════════════════════════════════════════════════════════
//  JointSymmetry::map_interval
//
//  Given a canonical-sector interval [can_lo, can_hi] ⊆ [canonical_lo, canonical_hi],
//  produce the interval in the target sector:
//    [can_lo + sector * period, can_hi + sector * period]
// ═══════════════════════════════════════════════════════════════════════════

void JointSymmetry::map_interval(double can_lo, double can_hi,
                                 int sector,
                                 double& out_lo, double& out_hi) const {
    double shift = sector * period;
    out_lo = can_lo + shift;
    out_hi = can_hi + shift;
}

// ═══════════════════════════════════════════════════════════════════════════
//  JointSymmetry::transform_aabb
//
//  Z4 rotation AABB mapping (offset = a = offset_x):
//
//  sector 0 (identity):
//    dst = src
//  sector 1 (+π/2):
//    x' = [−y_max + a, −y_min + a]
//    y' = [ x_min − a,  x_max − a]
//    z' = z
//  sector 2 (+π):
//    x' = [−x_max + 2a, −x_min + 2a]
//    y' = [−y_max,       −y_min      ]
//    z' = z
//  sector 3 (+3π/2):
//    x' = [ y_min − a,   y_max − a ]
//    y' = [−x_max + a,  −x_min + a ]
//    z' = z
// ═══════════════════════════════════════════════════════════════════════════

void JointSymmetry::transform_aabb(const float* src, int sector,
                                   float* dst) const {
    if (type == JointSymmetryType::NONE || (sector % 4) == 0) {
        std::memcpy(dst, src, 6 * sizeof(float));
        return;
    }

    const float x_lo = src[0], y_lo = src[1], z_lo = src[2];
    const float x_hi = src[3], y_hi = src[4], z_hi = src[5];
    const float a = static_cast<float>(offset_x);

    switch (sector % 4) {
    case 1:  // +π/2
        dst[0] = -y_hi + a;   dst[3] = -y_lo + a;
        dst[1] =  x_lo - a;   dst[4] =  x_hi - a;
        dst[2] =  z_lo;       dst[5] =  z_hi;
        break;
    case 2:  // +π
        dst[0] = -x_hi + 2.f * a;  dst[3] = -x_lo + 2.f * a;
        dst[1] = -y_hi;            dst[4] = -y_lo;
        dst[2] =  z_lo;            dst[5] =  z_hi;
        break;
    case 3:  // +3π/2
        dst[0] =  y_lo - a;   dst[3] =  y_hi - a;
        dst[1] = -x_hi + a;   dst[4] = -x_lo + a;
        dst[2] =  z_lo;       dst[5] =  z_hi;
        break;
    default:
        std::memcpy(dst, src, 6 * sizeof(float));
        break;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  JointSymmetry::transform_all_link_aabbs
// ═══════════════════════════════════════════════════════════════════════════

void JointSymmetry::transform_all_link_aabbs(
    const float* src_aabbs, int n_links,
    int sector,
    float* dst_aabbs) const
{
    for (int i = 0; i < n_links; ++i) {
        transform_aabb(src_aabbs + i * 6, sector, dst_aabbs + i * 6);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  JointSymmetry::transform_all_endpoint_iaabbs
// ═══════════════════════════════════════════════════════════════════════════

void JointSymmetry::transform_all_endpoint_iaabbs(
    const float* src, int n_endpoints,
    int sector,
    float* dst) const
{
    for (int i = 0; i < n_endpoints; ++i) {
        transform_aabb(src + i * 6, sector, dst + i * 6);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  detect_joint_symmetries
//
//  For a general N-DOF serial chain:
//    - Joint 0: Z4_ROTATION if α_0 ≈ 0 (pure z rotation as first joint)
//    - Joint i > 0: NONE (prefix is interval matrix → no closed-form AABB map)
// ═══════════════════════════════════════════════════════════════════════════

std::vector<JointSymmetry> detect_joint_symmetries(const Robot& robot) {
    const int n = robot.n_joints();
    std::vector<JointSymmetry> result(n);

    for (int i = 0; i < n; ++i) {
        result[i].joint_index = i;
        result[i].type = JointSymmetryType::NONE;
    }

    if (n == 0) return result;

    // Check joint 0: α_0 must be ≈ 0 for Z4
    const auto& dh = robot.dh_params();
    const double alpha_0 = dh[0].alpha;
    constexpr double EPS = 1e-10;

    if (std::abs(alpha_0) < EPS &&
        dh[0].joint_type == 0 /* revolute */) {

        auto& s = result[0];
        s.type = JointSymmetryType::Z4_ROTATION;
        s.canonical_lo = 0.0;
        s.canonical_hi = HALF_PI;
        s.period = HALF_PI;
        s.offset_x = dh[0].a;
    }

    return result;
}

} // namespace sbf

// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Joint Symmetry Detection & AABB Transform
//  Module: sbf::core
//
//  Detects exploitable symmetries in serial-chain DH parameters for
//  reducing LECT interval-cache range.
//  Currently supports:
//    - Z4_ROTATION: 4-fold rotation symmetry for joint 0 when α_0 = 0.
//      Canonical interval [0, π/2], 4× cache reduction.
//
//  Mathematical basis: see doc/PLAN_JOINT_SYMMETRY_CACHE.md §5
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include <cstdint>
#include <cstring>
#include <vector>

namespace sbf {

// Forward declaration
class Robot;

// ─── Joint symmetry type ────────────────────────────────────────────────────
enum class JointSymmetryType : uint8_t {
    NONE         = 0,   // No exploitable symmetry
    Z4_ROTATION  = 1,   // 4-fold rotation about z (shift by kπ/2)
};

// ─── Single-joint symmetry descriptor ───────────────────────────────────────
struct JointSymmetry {
    int                joint_index = 0;
    JointSymmetryType  type = JointSymmetryType::NONE;
    double             canonical_lo = 0.0;  // canonical interval lower bound
    double             canonical_hi = 0.0;  // canonical interval upper bound
    double             period = 0.0;        // symmetry period (π/2 for Z4)
    double             offset_x = 0.0;      // DH a_i parameter (AABB offset)

    /// Map q to canonical interval, return sector number (0-3 for Z4).
    /// sector 0: canonical,  1: +π/2,  2: +π,  3: +3π/2
    int canonicalize(double q, double& q_canonical) const;

    /// Map a canonical-sector interval [lo,hi] to the target sector.
    /// Returns the transformed interval via out_lo / out_hi.
    void map_interval(double can_lo, double can_hi,
                      int sector,
                      double& out_lo, double& out_hi) const;

    /// Transform a single link's AABB from canonical sector to target sector.
    /// src/dst layout: [x_min, y_min, z_min, x_max, y_max, z_max]
    void transform_aabb(const float* src, int sector, float* dst) const;

    /// Batch-transform ALL active links' AABBs from canonical to target sector.
    void transform_all_link_aabbs(const float* src_aabbs, int n_links,
                                  int sector,
                                  float* dst_aabbs) const;

    /// Batch-transform ALL endpoint iAABBs from canonical to target sector.
    void transform_all_endpoint_iaabbs(const float* src, int n_endpoints,
                                       int sector,
                                       float* dst) const;
};

// ─── Detection ──────────────────────────────────────────────────────────────

/// Detect exploitable LECT-level symmetries for all joints.
/// Returns a vector of size robot.n_joints().
/// Currently only joint 0 may have Z4_ROTATION (when α_0 ≈ 0).
std::vector<JointSymmetry> detect_joint_symmetries(const Robot& robot);

} // namespace sbf

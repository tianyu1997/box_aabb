#pragma once
/// @file joint_symmetry.h
/// @brief Joint rotational symmetry detection and C-space canonicalization.
///
/// For joints with discrete rotational symmetry (e.g. Z4: 4-fold
/// rotation by kπ/2), the LECT tree only needs to expand one canonical
/// sector.  Envelopes in other sectors are obtained by transforming
/// the cached data.  This gives up to 4× speedup for symmetric joints.

#include <sbf/core/types.h>

#include <cstdint>
#include <vector>

namespace sbf {

class Robot;

/// @brief Type of discrete rotational symmetry for a joint.
enum class JointSymmetryType : uint8_t {
    NONE         = 0,   ///< No exploitable symmetry.
    Z4_ROTATION  = 1,   ///< 4-fold rotation about z-axis (shift by kπ/2).
};

/// @brief Symmetry descriptor for a single joint.
///
/// When `type == Z4_ROTATION`, the joint range [−π, π] is divided into
/// 4 sectors of width π/2.  `canonicalize()` maps any joint value q to
/// its canonical representative in [canonical_lo, canonical_hi] and
/// returns the sector index k.
struct JointSymmetry {
    int                joint_index = 0;     ///< Which joint this applies to.
    JointSymmetryType  type = JointSymmetryType::NONE;  ///< Symmetry type.
    double             canonical_lo = 0.0;  ///< Canonical sector lower bound.
    double             canonical_hi = 0.0;  ///< Canonical sector upper bound.
    double             period = 0.0;        ///< Rotation period (e.g. π/2 for Z4).
    double             offset_x = 0.0;      ///< DH `a` parameter (link length offset).

    /// Map joint value @p q to canonical sector; return sector index.
    int canonicalize(double q, double& q_canonical) const;

    /// Map canonical interval to the @p sector-th copy.
    void map_interval(double can_lo, double can_hi,
                      int sector,
                      double& out_lo, double& out_hi) const;

    /// Transform a single link AABB from canonical sector to @p sector.
    void transform_aabb(const float* src, int sector, float* dst) const;

    /// Transform all link AABBs (n_links × 6 floats) to @p sector.
    void transform_all_link_aabbs(const float* src_aabbs, int n_links,
                                  int sector, float* dst_aabbs) const;

    /// Transform all endpoint iAABBs (n_endpoints × 6 floats) to @p sector.
    void transform_all_endpoint_iaabbs(const float* src, int n_endpoints,
                                       int sector, float* dst) const;
};

/// Auto-detect joint symmetries from robot DH parameters.
std::vector<JointSymmetry> detect_joint_symmetries(const Robot& robot);

}  // namespace sbf

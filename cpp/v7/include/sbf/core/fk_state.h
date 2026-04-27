#pragma once
/// @file fk_state.h
/// @brief Interval forward kinematics state and computation.
///
/// Port of v6's `FKState`. Stores per-joint interval DH matrices and the
/// cumulative prefix chain T_0·…·T_i for incremental updates.
///
/// D1 enforcement: `extract_link_aabbs` and `extract_endpoint_iaabbs`
/// always return zero-radius AABBs — `link_radii` is NOT a parameter.

#include "sbf/core/interval_math.h"
#include "sbf/core/robot.h"

#include <vector>

namespace sbf::core {

/// Interval FK state: prefix chain + per-joint DH matrices.
struct FKState {
    double prefix_lo[MAX_TF][16];
    double prefix_hi[MAX_TF][16];
    double joints_lo[MAX_JOINTS][16];
    double joints_hi[MAX_JOINTS][16];
    int n_tf  = 0;
    int n_jm  = 0;
    bool valid = false;
};

/// Build the DH interval matrix for a single joint at the given joint range.
void build_joint_interval(const Robot& robot, int joint_idx,
                          const Interval& iv,
                          double A_lo[16], double A_hi[16]);

/// Compute full interval FK chain from scratch.
FKState compute_fk_full(const Robot& robot,
                        const std::vector<Interval>& intervals);

FKState compute_fk_full(const Robot& robot,
                        const Interval* intervals, int n_intervals);

/// Incremental FK: copy parent state and recompute from `changed_dim`.
FKState compute_fk_incremental(const FKState& parent,
                               const Robot& robot,
                               const std::vector<Interval>& intervals,
                               int changed_dim);

/// In-place incremental FK: mutate `state` directly.
void update_fk_inplace(FKState& state,
                       const Robot& robot,
                       const std::vector<Interval>& intervals,
                       int changed_dim);

/// Extract per-link zero-radius AABBs (D1).
/// Output layout: [n_active_links × 6] = [lo_xyz, hi_xyz] per link.
void extract_link_aabbs(const FKState& state,
                        const int* active_link_map, int n_active_links,
                        float* out_aabb);

/// Extract per-endpoint zero-radius iAABBs.
/// Output layout: [n_active × 2 × 6] — proximal then distal per active link.
void extract_endpoint_iaabbs(const FKState& state,
                             const int* active_link_map, int n_active_links,
                             float* out);

}  // namespace sbf::core

#pragma once
/// @file fk_state.h
/// @brief Interval forward kinematics (IFK) state and computation.
///
/// `FKState` stores the interval-valued prefix chain T_0 · T_1 ·…· T_i
/// and individual per-joint DH interval matrices.  Two computation modes:
///   - `compute_fk_full()` — recomputes the entire chain from scratch.
///   - `compute_fk_incremental()` — reuses parent chain, only recomputes
///     from the changed dimension onward (O(n−k) instead of O(n)).

#include <sbf/core/types.h>
#include <sbf/core/interval_math.h>
#include <sbf/core/robot.h>

#include <vector>

namespace sbf {

/// @brief Interval FK state: prefix transform chain + per-joint DH matrices.
///
/// After computing FK, `prefix_lo[i]/prefix_hi[i]` contain the interval
/// bounding the cumulative T_0·…·T_i transform, and `joints_lo[j]/joints_hi[j]`
/// contain the single-joint DH interval matrix for joint j.
struct FKState {
    double prefix_lo[MAX_TF][16];     ///< Lower bounds of prefix transforms.
    double prefix_hi[MAX_TF][16];     ///< Upper bounds of prefix transforms.
    double joints_lo[MAX_JOINTS][16]; ///< Lower bounds of per-joint DH matrices.
    double joints_hi[MAX_JOINTS][16]; ///< Upper bounds of per-joint DH matrices.
    int n_tf  = 0;   ///< Number of transform frames stored.
    int n_jm  = 0;   ///< Number of joint matrices stored.
    bool valid = false;  ///< True after successful FK computation.
};

/// Build the DH interval matrix for a single joint.
void build_joint_interval(const Robot& robot, int joint_idx,
                          const Interval& iv,
                          double A_lo[16], double A_hi[16]);

/// Compute full interval FK chain from scratch.
FKState compute_fk_full(const Robot& robot,
                        const std::vector<Interval>& intervals);

/// Compute full interval FK chain from a raw Interval pointer (avoids vector allocation).
FKState compute_fk_full(const Robot& robot,
                        const Interval* intervals, int n_intervals);

/// Incremental FK: reuse parent state, recomputing only from @p changed_dim.
FKState compute_fk_incremental(const FKState& parent,
                               const Robot& robot,
                               const std::vector<Interval>& intervals,
                               int changed_dim);

/// In-place incremental FK: update @p state directly, recomputing from @p changed_dim.
/// Avoids the ~17KB memcpy of compute_fk_incremental by modifying in place.
void update_fk_inplace(FKState& state,
                       const Robot& robot,
                       const std::vector<Interval>& intervals,
                       int changed_dim);

/// Extract per-link AABBs from FK state into flat float array.
/// Output layout: [n_active_links × 6] = [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z] per link.
void extract_link_aabbs(const FKState& state,
                        const int* active_link_map, int n_active_links,
                        float* out_aabb,
                        const double* link_radii = nullptr);

/// Extract per-endpoint iAABBs from FK state.
/// Output layout: [n_active × 2 × 6] — proximal + distal per active link.
void extract_endpoint_iaabbs(const FKState& state,
                             const int* active_link_map, int n_active_links,
                             float* out);

}  // namespace sbf

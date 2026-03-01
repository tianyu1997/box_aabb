// SafeBoxForest — Interval Forward Kinematics
// DH-based interval FK with full and incremental computation
#pragma once

#include "sbf/aabb/interval_math.h"
#include "sbf/core/robot.h"
#include "sbf/core/types.h"

namespace sbf {

// ─── FK State ───────────────────────────────────────────────────────────────
// Holds the interval prefix chain and per-joint DH interval matrices.
// Allocated on stack (~35KB for MAX_TF=34).
struct FKState {
    double prefix_lo[MAX_TF][16];   // prefix[i] = prod(joints[0..i-1])
    double prefix_hi[MAX_TF][16];
    double joints_lo[MAX_JOINTS][16];
    double joints_hi[MAX_JOINTS][16];
    int n_tf  = 0;   // number of valid prefix entries
    int n_jm  = 0;   // number of valid joint entries
    bool valid = false;
};

// ─── Build single joint interval matrix ─────────────────────────────────────
// Constructs the DH interval matrix for joint `joint_idx` given the Robot
// parameters and joint interval `iv`.
void build_joint_interval(const Robot& robot, int joint_idx,
                          const Interval& iv,
                          double A_lo[16], double A_hi[16]);

// ─── Full FK ────────────────────────────────────────────────────────────────
// Compute full interval FK chain from joint intervals.
// prefix[0] = I, prefix[i+1] = prefix[i] × joints[i]
// If robot has tool frame, appends one more.
FKState compute_fk_full(const Robot& robot,
                        const std::vector<Interval>& intervals);

// ─── Incremental FK ─────────────────────────────────────────────────────────
// Recompute FK after changing one dimension's interval.
// Copies parent state, then recomputes from `changed_dim` onward.
// Much cheaper than full FK when only one joint changes.
FKState compute_fk_incremental(const FKState& parent,
                               const Robot& robot,
                               const std::vector<Interval>& intervals,
                               int changed_dim);

// ─── Extract Link AABBs ─────────────────────────────────────────────────────
// Extract compact AABB array from FK prefix matrices.
// Each link AABB = bounding box of (prefix[link_start][:3,3], prefix[link_end][:3,3])
// Output: flat float array [n_active_links * 6] in format [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]
// If link_radii is non-null, inflate each link AABB by its radius (compact order).
void extract_link_aabbs(const FKState& state,
                        const int* active_link_map, int n_active_links,
                        float* out_aabb,
                        const double* link_radii = nullptr);

// ─── Convenience: compute AABB directly ─────────────────────────────────────
// Full pipeline: intervals → FK → extract AABBs
void compute_link_aabbs(const Robot& robot,
                        const std::vector<Interval>& intervals,
                        float* out_aabb);

} // namespace sbf

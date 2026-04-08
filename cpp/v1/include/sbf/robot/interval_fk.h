// SafeBoxForest — Interval Forward Kinematics
// Module: sbf::robot
// DH-based interval FK with full and incremental computation
#pragma once

#include "sbf/robot/interval_math.h"
#include "sbf/robot/robot.h"
#include "sbf/common/types.h"

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
void build_joint_interval(const Robot& robot, int joint_idx,
                          const Interval& iv,
                          double A_lo[16], double A_hi[16]);

// ─── Full FK ────────────────────────────────────────────────────────────────
// Compute full interval FK chain from joint intervals.
FKState compute_fk_full(const Robot& robot,
                        const std::vector<Interval>& intervals);

// ─── Incremental FK ─────────────────────────────────────────────────────────
// Recompute FK after changing one dimension's interval.
FKState compute_fk_incremental(const FKState& parent,
                               const Robot& robot,
                               const std::vector<Interval>& intervals,
                               int changed_dim);

// ─── Extract Link AABBs ─────────────────────────────────────────────────────
void extract_link_aabbs(const FKState& state,
                        const int* active_link_map, int n_active_links,
                        float* out_aabb,
                        const double* link_radii = nullptr);

// ─── Extract EE Sphere AABBs ────────────────────────────────────────────────
void extract_ee_sphere_aabbs(const FKState& state,
                             const EESphere* spheres, int n_spheres,
                             int frame_index,
                             float* out_aabb);

// ─── Extract EE Group AABBs ─────────────────────────────────────────────────
void extract_ee_group_aabbs(const FKState& state,
                            const EEGroup* groups, int n_groups,
                            int frame_index,
                            float* out_aabb);

// ─── Convenience: compute AABB directly ─────────────────────────────────────
void compute_link_aabbs(const Robot& robot,
                        const std::vector<Interval>& intervals,
                        float* out_aabb);

void compute_all_aabbs(const Robot& robot,
                       const std::vector<Interval>& intervals,
                       float* out_link_aabb,
                       float* out_ee_aabb);

} // namespace sbf

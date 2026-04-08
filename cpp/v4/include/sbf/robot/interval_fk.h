// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Interval Forward Kinematics (iFK)
//  Module: sbf::robot
//  DH-based interval FK with full and incremental computation
//  迁移自 v3 interval_fk.h
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/robot/interval_math.h"
#include "sbf/robot/robot.h"
#include "sbf/core/types.h"

#include <vector>

namespace sbf {

// ─── FK State ───────────────────────────────────────────────────────────
struct FKState {
    double prefix_lo[MAX_TF][16];     // cumulative transform chain lower
    double prefix_hi[MAX_TF][16];     // cumulative transform chain upper
    double joints_lo[MAX_JOINTS][16]; // per-joint DH matrix lower
    double joints_hi[MAX_JOINTS][16]; // per-joint DH matrix upper
    int n_tf  = 0;    // n_joints + 1 + has_tool
    int n_jm  = 0;    // n_joints + has_tool
    bool valid = false;
};

// ─── Build single joint interval matrix ───────────────────────────────────
void build_joint_interval(const Robot& robot, int joint_idx,
                          const Interval& iv,
                          double A_lo[16], double A_hi[16]);

// ─── Full FK ────────────────────────────────────────────────────────────
FKState compute_fk_full(const Robot& robot,
                        const std::vector<Interval>& intervals);

// ─── Incremental FK ─────────────────────────────────────────────────────
FKState compute_fk_incremental(const FKState& parent,
                               const Robot& robot,
                               const std::vector<Interval>& intervals,
                               int changed_dim);

// ─── Extract Link AABBs ─────────────────────────────────────────────────
void extract_link_aabbs(const FKState& state,
                        const int* active_link_map, int n_active_links,
                        float* out_aabb,
                        const double* link_radii = nullptr);

// ─── Extract EE Sphere AABBs ────────────────────────────────────────────
void extract_ee_sphere_aabbs(const FKState& state,
                             const EESphere* spheres, int n_spheres,
                             int frame_index,
                             float* out_aabb);

// ─── Extract EE Group AABBs ─────────────────────────────────────────────
void extract_ee_group_aabbs(const FKState& state,
                            const EEGroup* groups, int n_groups,
                            int frame_index,
                            float* out_aabb);

// ─── Convenience: compute AABB directly ───────────────────────────────────
void compute_link_aabbs(const Robot& robot,
                        const std::vector<Interval>& intervals,
                        float* out_aabb);

void compute_all_aabbs(const Robot& robot,
                       const std::vector<Interval>& intervals,
                       float* out_link_aabb,
                       float* out_ee_aabb);

} // namespace sbf

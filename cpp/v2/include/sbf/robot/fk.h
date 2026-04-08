// SafeBoxForest v2 — Scalar Forward Kinematics
// Module: sbf::robot
#pragma once

#include "sbf/robot/robot.h"
#include <Eigen/Core>
#include <vector>

namespace sbf {

// Compute scalar FK: single configuration -> link positions
std::vector<Eigen::Vector3d> fk_link_positions(const Robot& robot,
                                                const Eigen::VectorXd& q);

// Compute scalar FK: single configuration -> 4x4 transforms
std::vector<Eigen::Matrix4d> fk_transforms(const Robot& robot,
                                            const Eigen::VectorXd& q);

// Build single DH transform matrix (scalar)
Eigen::Matrix4d dh_transform(double alpha, double a, double d, double theta);

// ──── Optimised FK helpers for L-BFGS-B acceleration ─────────────────────

// In-place FK: write transforms into caller-owned buffer.
// buf must have size >= n_joints+1 (+1 for tool if has_tool).
// Returns number of frames written.
int fk_transforms_inplace(const Robot& robot,
                           const Eigen::VectorXd& q,
                           Eigen::Matrix4d* buf);

// Analytical position Jacobian for link `link_idx` (0-based frame index).
// Given pre-computed transforms buf[0..link_idx], computes
//   grad[i] = e_dim · (z_i × (p_link - o_i))   for i = 0..n_active_joints-1
// where n_active_joints = min(link_idx, n_joints).
// This replaces 2*n finite-difference FK calls with n cross-products.
// Returns the scalar positional value p_link[dim].
double fk_position_gradient(const Robot& robot,
                             const Eigen::Matrix4d* transforms,
                             int link_idx,       // which link position
                             int dim,            // 0=x, 1=y, 2=z
                             int n_active_joints, // joints [0..n_active_joints-1]
                             double* grad);       // output: [n_active_joints]

} // namespace sbf

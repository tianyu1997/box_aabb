// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Scalar FK (convenience header)
//  Module: sbf::robot
//  迁移自 v3 fk.h，函数声明在 sbf:: 命名空间
// ═══════════════════════════════════════════════════════════════════════════
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

// In-place FK: writes transforms to pre-allocated buffer, returns frame count
int fk_transforms_inplace(const Robot& robot,
                           const Eigen::VectorXd& q,
                           Eigen::Matrix4d* buf);

} // namespace sbf

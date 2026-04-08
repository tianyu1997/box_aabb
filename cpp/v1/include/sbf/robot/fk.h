// SafeBoxForest — Scalar Forward Kinematics
// Module: sbf::robot
#pragma once

#include "sbf/robot/robot.h"
#include <Eigen/Core>
#include <vector>

namespace sbf {

// Compute scalar FK: single configuration → link positions
// Returns (n_links+1) positions (3D)
std::vector<Eigen::Vector3d> fk_link_positions(const Robot& robot,
                                                const Eigen::VectorXd& q);

// Compute scalar FK: single configuration → 4×4 transforms
std::vector<Eigen::Matrix4d> fk_transforms(const Robot& robot,
                                            const Eigen::VectorXd& q);

// Build single DH transform matrix (scalar)
Eigen::Matrix4d dh_transform(double alpha, double a, double d, double theta);

} // namespace sbf

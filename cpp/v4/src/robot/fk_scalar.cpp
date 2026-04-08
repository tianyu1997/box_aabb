// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Scalar FK implementation
//  迁移自 v3 src/robot/fk_scalar.cpp
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/robot/fk.h"
#include <cmath>

namespace sbf {

Eigen::Matrix4d dh_transform(double alpha, double a, double d, double theta) {
    double ct = std::cos(theta);
    double st = std::sin(theta);
    double ca = std::cos(alpha);
    double sa = std::sin(alpha);

    Eigen::Matrix4d T;
    T << ct,      -st,     0,   a,
         st * ca,  ct * ca, -sa, -d * sa,
         st * sa,  ct * sa,  ca,  d * ca,
         0,        0,       0,   1;
    return T;
}

std::vector<Eigen::Vector3d> fk_link_positions(const Robot& robot,
                                                const Eigen::VectorXd& q) {
    auto transforms = fk_transforms(robot, q);
    std::vector<Eigen::Vector3d> positions;
    positions.reserve(transforms.size());
    for (auto& T : transforms)
        positions.push_back(T.block<3, 1>(0, 3));
    return positions;
}

std::vector<Eigen::Matrix4d> fk_transforms(const Robot& robot,
                                            const Eigen::VectorXd& q) {
    int n = robot.n_joints();
    std::vector<Eigen::Matrix4d> result;
    result.reserve(n + 2);

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    result.push_back(T);

    for (int i = 0; i < n; ++i) {
        const auto& dh = robot.dh_params()[i];
        double theta, d;
        if (dh.joint_type == 0) {
            theta = q[i] + dh.theta;
            d = dh.d;
        } else {
            theta = dh.theta;
            d = q[i] + dh.d;
        }
        T = T * dh_transform(dh.alpha, dh.a, d, theta);
        result.push_back(T);
    }

    if (robot.has_tool()) {
        const auto& tool = *robot.tool_frame();
        T = T * dh_transform(tool.alpha, tool.a, tool.d, tool.theta);
        result.push_back(T);
    }

    return result;
}

int fk_transforms_inplace(const Robot& robot,
                           const Eigen::VectorXd& q,
                           Eigen::Matrix4d* buf) {
    int n = robot.n_joints();
    int idx = 0;

    buf[idx] = Eigen::Matrix4d::Identity();
    ++idx;

    for (int i = 0; i < n; ++i) {
        const auto& dh = robot.dh_params()[i];
        double theta, d;
        if (dh.joint_type == 0) {
            theta = q[i] + dh.theta;
            d = dh.d;
        } else {
            theta = dh.theta;
            d = q[i] + dh.d;
        }
        buf[idx] = buf[idx - 1] * dh_transform(dh.alpha, dh.a, d, theta);
        ++idx;
    }

    if (robot.has_tool()) {
        const auto& tool = *robot.tool_frame();
        buf[idx] = buf[idx - 1] * dh_transform(tool.alpha, tool.a, tool.d, tool.theta);
        ++idx;
    }

    return idx;
}

} // namespace sbf

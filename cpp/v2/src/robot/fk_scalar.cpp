// SafeBoxForest v2 — Scalar FK implementation
#include "sbf/robot/fk.h"
#include <Eigen/Geometry>
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

// ──── In-place FK (avoids heap allocation) ───────────────────────────────

int fk_transforms_inplace(const Robot& robot,
                           const Eigen::VectorXd& q,
                           Eigen::Matrix4d* buf) {
    int n = robot.n_joints();
    int idx = 0;

    buf[idx] = Eigen::Matrix4d::Identity();
    ++idx;

    Eigen::Matrix4d T = buf[0];
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
        buf[idx] = T;
        ++idx;
    }

    if (robot.has_tool()) {
        const auto& tool = *robot.tool_frame();
        T = T * dh_transform(tool.alpha, tool.a, tool.d, tool.theta);
        buf[idx] = T;
        ++idx;
    }

    return idx;
}

// ──── Analytical position Jacobian ───────────────────────────────────────
//
// For revolute joint i, the position Jacobian column is:
//   J_i = z_i × (p_link - o_i)
// where z_i is the z-axis of frame i's transform, o_i is frame i's origin,
// and p_link is the target link position.
//
// We return the dot product with e_dim (unit vector along axis `dim`),
// giving ∂p_link[dim]/∂q_i for each joint.

double fk_position_gradient(const Robot& robot,
                             const Eigen::Matrix4d* transforms,
                             int link_idx,
                             int dim,
                             int n_active_joints,
                             double* grad) {
    // p_link = transforms[link_idx].col(3).head<3>()
    const Eigen::Vector3d p_link = transforms[link_idx].block<3,1>(0, 3);

    for (int i = 0; i < n_active_joints; ++i) {
        // Joint i's transform is transforms[i] (frame before joint rotation)
        // For revolute: z_i = transforms[i].col(2).head<3>()
        //               o_i = transforms[i].col(3).head<3>()
        // BUT transforms[i] here is frame i (after joint i-1).
        // Actually transforms[0] = base, transforms[i+1] = frame after joint i.
        // Joint i rotates about the z-axis of transforms[i] (the frame at the input of joint i).
        // Wait -- let me reconsider the DH convention used here.
        //
        // transforms[0] = Identity (base)
        // transforms[1] = T_0 (after joint 0)
        // transforms[i+1] = T_0 * T_1 * ... * T_i (after joint i)
        //
        // For DH standard, joint i rotates about z_{i-1} which is the z-axis
        // of the frame BEFORE joint i. In our indexing, that's transforms[i].
        // (transforms[i] = T_0 * ... * T_{i-1} has z-axis = z_{i-1})
        //
        // Wait, actually the DH convention in the code is:
        // T_i = dh_transform(alpha_i, a_i, d_i, theta_i + q_i)
        // transforms[0] = Identity
        // transforms[1] = T_0
        // transforms[i+1] = T_0 ... T_i
        //
        // Joint i parameter q_i appears in theta = q_i + dh.theta (for revolute).
        // The rotation is about z before the transform is applied.
        // So the rotation axis for joint i is the z-axis of transforms[i].
        const Eigen::Vector3d z_i = transforms[i].block<3,1>(0, 2);
        const Eigen::Vector3d o_i = transforms[i].block<3,1>(0, 3);

        // J_i = z_i × (p_link - o_i)
        const Eigen::Vector3d diff = p_link - o_i;
        const Eigen::Vector3d J_i = z_i.cross(diff);
        grad[i] = J_i[dim];
    }

    return p_link[dim];
}

} // namespace sbf

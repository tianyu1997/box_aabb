// SafeBoxForest v2 — Robot model (DH parameters)
// Module: sbf::robot
#pragma once

#include "sbf/common/types.h"
#include <Eigen/Core>
#include <string>
#include <utility>
#include <tuple>
#include <vector>

namespace sbf {

// DH parameter for one joint
struct DHParam {
    double alpha = 0.0;   // link twist
    double a     = 0.0;   // link length
    double d     = 0.0;   // link offset
    double theta = 0.0;   // joint angle offset
    int joint_type = 0;   // 0 = revolute, 1 = prismatic
};

// End-effector collision sphere (e.g., WSG gripper)
struct EESphere {
    double center[3] = {0, 0, 0};  // in frame prefix[frame_index]
    double radius = 0.0;
};

// Key sphere for interval-AABB group computation
struct EEGroupKey {
    double center[3] = {0, 0, 0};
    double coverage_radius = 0.0;  // covers all group members from this center
};

// Group of EE spheres sharing one AABB slot in the tree
struct EEGroup {
    static constexpr int MAX_KEYS = 8;
    EEGroupKey keys[MAX_KEYS];
    int n_keys = 0;
};

class Robot {
public:
    Robot() = default;

    // Factory: load from JSON config file (panda.json / iiwa14.json format)
    static Robot from_json(const std::string& path);

    // Factory: construct directly
    Robot(std::string name,
          std::vector<DHParam> dh_params,
          JointLimits limits,
          std::optional<DHParam> tool_frame = std::nullopt,
          std::vector<double> link_radii = {});

    // ── Accessors ────────────────────────────────────────────────────────
    const std::string& name() const { return name_; }
    int n_joints() const { return n_joints_; }
    int n_links()  const { return n_links_; }
    int n_transforms() const { return n_tf_; }

    const std::vector<DHParam>& dh_params() const { return dh_params_; }
    const JointLimits& joint_limits() const { return limits_; }
    const std::optional<DHParam>& tool_frame() const { return tool_frame_; }
    bool has_tool() const { return tool_frame_.has_value(); }
    const std::vector<double>& link_radii() const { return link_radii_; }

    // Pre-packed DH arrays for fast FK (contiguous memory)
    const double* dh_alpha() const { return dh_alpha_.data(); }
    const double* dh_a()     const { return dh_a_.data(); }
    const double* dh_d()     const { return dh_d_.data(); }
    const double* dh_theta() const { return dh_theta_.data(); }
    const int*    dh_joint_type() const { return dh_joint_type_.data(); }

    // Active link map: compact_idx -> original link index
    const int* active_link_map() const { return active_link_map_.data(); }
    int n_active_links() const { return n_active_links_; }

    // Link radii for active links (compact order, for AABB inflation)
    const double* active_link_radii() const {
        return active_link_radii_.empty() ? nullptr : active_link_radii_.data();
    }
    bool has_link_radii() const { return !link_radii_.empty(); }

    // Coupled joint constraints (for critical-point constraint enumeration)
    const std::vector<std::pair<int,int>>& coupled_pairs() const { return coupled_pairs_; }
    const std::vector<std::tuple<int,int,int>>& coupled_triples() const { return coupled_triples_; }

    // End-effector collision spheres
    bool has_ee_spheres() const { return !ee_spheres_.empty(); }
    int n_ee_spheres() const { return static_cast<int>(ee_spheres_.size()); }
    const std::vector<EESphere>& ee_spheres() const { return ee_spheres_; }
    int ee_spheres_frame() const { return ee_spheres_frame_; }

    // EE AABB groups
    bool has_ee_groups() const { return !ee_groups_.empty(); }
    int n_ee_groups() const { return static_cast<int>(ee_groups_.size()); }
    const std::vector<EEGroup>& ee_groups() const { return ee_groups_; }
    int n_ee_aabb_slots() const {
        if (!ee_groups_.empty()) return static_cast<int>(ee_groups_.size());
        return static_cast<int>(ee_spheres_.size());
    }

    // Scalar FK: compute link positions for a given configuration
    Eigen::MatrixXd fk_link_positions(const Eigen::VectorXd& q) const;

    // Compute full 4x4 transforms for each link
    std::vector<Eigen::Matrix4d> fk_transforms(const Eigen::VectorXd& q) const;

    // Fingerprint for cache compatibility
    std::string fingerprint() const;

private:
    std::string name_;
    std::vector<DHParam> dh_params_;
    JointLimits limits_;
    std::optional<DHParam> tool_frame_;
    std::vector<double> link_radii_;

    std::vector<std::pair<int,int>> coupled_pairs_;
    std::vector<std::tuple<int,int,int>> coupled_triples_;

    std::vector<EESphere> ee_spheres_;
    int ee_spheres_frame_ = 0;
    std::vector<EEGroup> ee_groups_;

    int n_joints_ = 0;
    int n_links_  = 0;
    int n_tf_     = 0;
    int n_active_links_ = 0;

    std::vector<double> dh_alpha_;
    std::vector<double> dh_a_;
    std::vector<double> dh_d_;
    std::vector<double> dh_theta_;
    std::vector<int>    dh_joint_type_;
    std::vector<int>    active_link_map_;
    std::vector<double> active_link_radii_;

    void pack_arrays();

    friend Robot;
};

} // namespace sbf

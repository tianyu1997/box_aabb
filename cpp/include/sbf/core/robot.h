// SafeBoxForest — Robot model (DH parameters)
#pragma once

#include "sbf/core/types.h"
#include <Eigen/Core>
#include <string>
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
    int n_transforms() const { return n_tf_; } // n_joints + 1 (+1 if tool)

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

    // Active link map: compact_idx → original link index
    // Excludes link0 (base link, fixed position) and zero-length links
    const int* active_link_map() const { return active_link_map_.data(); }
    int n_active_links() const { return n_active_links_; }

    // Link radii for active links (compact order, for AABB inflation)
    const double* active_link_radii() const {
        return active_link_radii_.empty() ? nullptr : active_link_radii_.data();
    }
    bool has_link_radii() const { return !link_radii_.empty(); }

    // Scalar FK: compute link positions for a given configuration
    // Returns (n_links+1) × 3 matrix of translation vectors
    Eigen::MatrixXd fk_link_positions(const Eigen::VectorXd& q) const;

    // Compute full 4×4 transforms for each link
    // Returns vector of n_transforms 4×4 matrices
    std::vector<Eigen::Matrix4d> fk_transforms(const Eigen::VectorXd& q) const;

    // Fingerprint for cache compatibility
    std::string fingerprint() const;

private:
    std::string name_;
    std::vector<DHParam> dh_params_;
    JointLimits limits_;
    std::optional<DHParam> tool_frame_;
    std::vector<double> link_radii_;

    int n_joints_ = 0;
    int n_links_  = 0;
    int n_tf_     = 0;          // number of transforms
    int n_active_links_ = 0;

    // Pre-packed arrays for fast access
    std::vector<double> dh_alpha_;
    std::vector<double> dh_a_;
    std::vector<double> dh_d_;
    std::vector<double> dh_theta_;
    std::vector<int>    dh_joint_type_;
    std::vector<int>    active_link_map_;
    std::vector<double> active_link_radii_;  // compact radii for active links

    void pack_arrays();
};

} // namespace sbf

#pragma once
/// @file robot.h
/// @brief DH-parameterised serial robot kinematic model.
///
/// Port of v6's `Robot`. Loaded from a JSON DH description. Holds
/// joint params, limits, optional tool frame, and `link_radii` (read-only
/// in P1; applied by the envelope/collision pipeline in P2).

#include "sbf/core/types.h"

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace sbf::core {

/// Standard Denavit–Hartenberg parameters for one link/joint.
struct DHParam {
    double alpha = 0.0;   ///< Link twist  (rad).
    double a     = 0.0;   ///< Link length (m).
    double d     = 0.0;   ///< Link offset (m).
    double theta = 0.0;   ///< Joint angle offset (rad).
    int joint_type = 0;   ///< 0 = revolute, 1 = prismatic.
};

/// DH-parameterised serial robot.
class Robot {
public:
    Robot() = default;

    /// Load robot from a JSON description file.
    /// Expected keys: "dh_params", "joint_limits", optional "tool_frame",
    /// "link_radii", "coupled_pairs".
    static Robot from_json(const std::string& path);

    Robot(std::string name,
          std::vector<DHParam> dh_params,
          JointLimits limits,
          std::optional<DHParam> tool_frame = std::nullopt,
          std::vector<double> link_radii = {});

    const std::string& name() const { return name_; }
    int n_joints() const { return n_joints_; }
    bool has_tool() const { return tool_frame_.has_value(); }

    const std::vector<DHParam>& dh_params() const { return dh_params_; }
    const JointLimits& joint_limits() const { return limits_; }
    const std::optional<DHParam>& tool_frame() const { return tool_frame_; }

    /// Cylindrical radius per link (m). Read-only in P1; applied in P2.
    const std::vector<double>& link_radii() const { return link_radii_; }
    bool has_link_radii() const { return !link_radii_.empty(); }

    const int* active_link_map() const { return active_link_map_.data(); }
    int n_active_links() const { return n_active_links_; }
    int n_active_endpoints() const { return n_active_links_ * 2; }
    int last_active_frame() const { return last_active_frame_; }

    /// Active-link radii (subset of `link_radii` for non-skipped links).
    const double* active_link_radii() const {
        return active_link_radii_.empty() ? nullptr : active_link_radii_.data();
    }

    const std::vector<std::pair<int,int>>& coupled_pairs() const {
        return coupled_pairs_;
    }

    /// FNV-1a 64-bit fingerprint over full kinematic identity.
    /// NOTE (D1): includes link_radii; P3 cache key must skip radii bytes.
    uint64_t fingerprint() const;

private:
    std::string name_;
    std::vector<DHParam> dh_params_;
    JointLimits limits_;
    std::optional<DHParam> tool_frame_;
    std::vector<double> link_radii_;
    std::vector<std::pair<int,int>> coupled_pairs_;

    int n_joints_ = 0;
    int n_active_links_ = 0;
    int last_active_frame_ = 0;

    std::vector<int> active_link_map_;
    std::vector<double> active_link_radii_;

    void pack_arrays();

    friend class RobotFriend;  // for from_json access in cpp
};

}  // namespace sbf::core

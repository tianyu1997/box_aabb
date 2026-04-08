#pragma once
/// @file robot.h
/// @brief DH-parameterised serial robot kinematic model.
///
/// `Robot` is the central kinematic description used by every SBF module.
/// It stores Denavit–Hartenberg (DH) parameters, joint limits, optional
/// tool frame, link radii (cylindrical swept-volume radius per link),
/// and derived arrays for active-link indexing.
///
/// Typical usage:
/// @code
///   Robot robot = Robot::from_json("data/panda.json");
///   int n = robot.n_joints();          // 7
///   const auto& lims = robot.joint_limits();
/// @endcode

#include <sbf/core/types.h>

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace sbf {

/// @brief Standard Denavit–Hartenberg parameters for one link/joint.
struct DHParam {
    double alpha = 0.0;   ///< Link twist  (rad)
    double a     = 0.0;   ///< Link length (m)
    double d     = 0.0;   ///< Link offset (m)
    double theta = 0.0;   ///< Joint angle offset (rad)
    int joint_type = 0;   ///< 0 = revolute, 1 = prismatic
};

/// @brief DH-parameterised serial robot.
///
/// Constructed from a JSON robot description (`from_json`) or
/// programmatically.  Internally computes active-link maps (links with
/// nonzero radius) that guide the envelope pipeline.
class Robot {
public:
    Robot() = default;

    /// Load robot from a JSON description file.
    /// Expected keys: "dh_params", "joint_limits", optional "tool_frame", "link_radii".
    static Robot from_json(const std::string& path);

    /// Construct robot directly from parameters.
    Robot(std::string name,
          std::vector<DHParam> dh_params,
          JointLimits limits,
          std::optional<DHParam> tool_frame = std::nullopt,
          std::vector<double> link_radii = {});

    const std::string& name() const { return name_; }          ///< Robot model name.
    int n_joints() const { return n_joints_; }                  ///< Number of joints (DOF).
    bool has_tool() const { return tool_frame_.has_value(); }   ///< Whether a tool frame is defined.

    const std::vector<DHParam>& dh_params() const { return dh_params_; }           ///< DH parameter list.
    const JointLimits& joint_limits() const { return limits_; }                    ///< Per-joint limits.
    const std::optional<DHParam>& tool_frame() const { return tool_frame_; }       ///< Optional tool frame DH.
    const std::vector<double>& link_radii() const { return link_radii_; }          ///< Cylindrical radius per link (m).

    const int* active_link_map() const { return active_link_map_.data(); }   ///< Indices of links with nonzero radius.
    int n_active_links() const { return n_active_links_; }                   ///< Number of active (nonzero-radius) links.
    int n_active_endpoints() const { return n_active_links_ * 2; }           ///< Active links × 2 (proximal + distal).
    int last_active_frame() const { return last_active_frame_; }             ///< Frame index of the last active link.

    /// Active-link radii (float cast, for envelope pipeline).
    const double* active_link_radii() const {
        return active_link_radii_.empty() ? nullptr : active_link_radii_.data();
    }
    bool has_link_radii() const { return !link_radii_.empty(); }  ///< Whether link radii are provided.

    /// Coupled joint pairs detected from robot structure.
    const std::vector<std::pair<int,int>>& coupled_pairs() const { return coupled_pairs_; }

    /// FNV-1a 64-bit fingerprint over full kinematic identity:
    /// DH params, joint limits, tool frame, radii, active link map, coupled pairs.
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

    friend Robot; // allow from_json to set coupled_pairs_
};

}  // namespace sbf

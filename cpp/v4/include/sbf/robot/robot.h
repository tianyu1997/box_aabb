// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Robot model (DH parameters)
//  Module: sbf::robot
//  迁移自 v3 robot.h，适配 v4 include 路径
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/core/types.h"
#include "sbf/robot/iaabb_config.h"

#include <Eigen/Core>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace sbf {

// ─── DH Parameter ───────────────────────────────────────────────────────────
struct DHParam {
    double alpha = 0.0;   // link twist
    double a     = 0.0;   // link length
    double d     = 0.0;   // link offset
    double theta = 0.0;   // joint angle offset
    int joint_type = 0;   // 0 = revolute, 1 = prismatic
};

// ─── End-effector collision sphere ──────────────────────────────────────────
struct EESphere {
    double center[3] = {0, 0, 0};  // in frame prefix[frame_index]
    double radius = 0.0;
};

// ─── Key sphere for interval-AABB group computation ────────────────────────
struct EEGroupKey {
    double center[3] = {0, 0, 0};
    double coverage_radius = 0.0;  // covers all group members from this center
};

// ─── Group of EE spheres sharing one AABB slot in the tree ─────────────────
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
    int n_endpoints() const { return n_joints_ + (has_tool() ? 1 : 0); }

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

    // Paired active endpoints: n_active * 2 (proximal + distal per link)
    int n_active_endpoints() const { return n_active_links_ * 2; }

    // Last FK frame needed for active-only endpoint extraction.
    // = max(active_link_map[ci]) + 1  (distal of last active link)
    int last_active_frame() const { return last_active_frame_; }

    // Per-link affecting joint count:
    // n_affecting_joints[ci] = active_link_map[ci] + 1 (for serial DH chain)
    const int* n_affecting_joints() const { return n_affecting_joints_.data(); }

    // Link radii for active links (compact order, for AABB inflation)
    const double* active_link_radii() const {
        return active_link_radii_.empty() ? nullptr : active_link_radii_.data();
    }
    bool has_link_radii() const { return !link_radii_.empty(); }

    // Link radius by original index
    double link_radius(int link_idx) const {
        if (link_idx < 0 || link_idx >= static_cast<int>(link_radii_.size()))
            return 0.0;
        return link_radii_[link_idx];
    }

    // ── iAABB preprocessed configuration ─────────────────────────────
    //  Pre-processed DH parameters optimized for interval AABB generation.
    //  Uses configurable zero-length threshold (default 0.1).
    //  Built automatically during Robot construction.
    const IAABBConfig& iaabb_config() const { return iaabb_config_; }

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
    int last_active_frame_ = 0;

    std::vector<double> dh_alpha_;
    std::vector<double> dh_a_;
    std::vector<double> dh_d_;
    std::vector<double> dh_theta_;
    std::vector<int>    dh_joint_type_;
    std::vector<int>    active_link_map_;
    std::vector<double> active_link_radii_;
    std::vector<int>    n_affecting_joints_;

    // iAABB preprocessed config (built in pack_arrays)
    IAABBConfig iaabb_config_;

    void pack_arrays();
};

} // namespace sbf

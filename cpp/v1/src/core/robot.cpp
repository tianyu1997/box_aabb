// SafeBoxForest — Robot model implementation
#include "sbf/robot/robot.h"
#include "sbf/robot/fk.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <functional>
#include <sstream>
#include <stdexcept>

namespace sbf {

Robot::Robot(std::string name,
             std::vector<DHParam> dh_params,
             JointLimits limits,
             std::optional<DHParam> tool_frame,
             std::vector<double> link_radii)
    : name_(std::move(name)),
      dh_params_(std::move(dh_params)),
      limits_(std::move(limits)),
      tool_frame_(std::move(tool_frame)),
      link_radii_(std::move(link_radii))
{
    n_joints_ = static_cast<int>(dh_params_.size());
    n_links_  = n_joints_;  // one link per joint (DH convention)
    n_tf_     = n_joints_ + 1 + (has_tool() ? 1 : 0);
    pack_arrays();
}

Robot Robot::from_json(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error("Cannot open robot config: " + path);

    nlohmann::json j;
    f >> j;

    std::string name = j.value("name", "unknown");

    // Parse DH parameters
    std::vector<DHParam> dh;
    for (auto& p : j["dh_params"]) {
        DHParam dp;
        dp.alpha = p.value("alpha", 0.0);
        dp.a     = p.value("a", 0.0);
        dp.d     = p.value("d", 0.0);
        dp.theta = p.value("theta", 0.0);
        std::string type = p.value("type", "revolute");
        dp.joint_type = (type == "prismatic") ? 1 : 0;
        dh.push_back(dp);
    }

    // Parse joint limits
    JointLimits lim;
    for (auto& l : j["joint_limits"]) {
        lim.limits.push_back({l[0].get<double>(), l[1].get<double>()});
    }

    // Parse tool frame (optional)
    std::optional<DHParam> tool;
    if (j.contains("tool_frame")) {
        DHParam tp;
        tp.alpha = j["tool_frame"].value("alpha", 0.0);
        tp.a     = j["tool_frame"].value("a", 0.0);
        tp.d     = j["tool_frame"].value("d", 0.0);
        tp.theta = j["tool_frame"].value("theta", 0.0);
        tp.joint_type = 0;
        tool = tp;
    }

    // Parse link radii (optional)
    std::vector<double> radii;
    if (j.contains("link_radii")) {
        for (auto& r : j["link_radii"])
            radii.push_back(r.get<double>());
    }

    Robot robot(std::move(name), std::move(dh), std::move(lim),
                std::move(tool), std::move(radii));

    // Parse end-effector collision spheres (optional)
    if (j.contains("end_effector_spheres")) {
        auto& ee = j["end_effector_spheres"];
        robot.ee_spheres_frame_ = ee.value("frame_index", 0);
        if (ee.contains("spheres")) {
            for (auto& s : ee["spheres"]) {
                EESphere sp;
                auto& c = s["center"];
                sp.center[0] = c[0].get<double>();
                sp.center[1] = c[1].get<double>();
                sp.center[2] = c[2].get<double>();
                sp.radius = s.value("radius", 0.0);
                robot.ee_spheres_.push_back(sp);
            }
        }

        // Parse AABB groups for efficient interval-AABB computation
        if (ee.contains("aabb_groups")) {
            for (auto& g : ee["aabb_groups"]) {
                EEGroup group;
                auto key_idx = g["key_indices"].get<std::vector<int>>();

                // Determine member indices (default = key_indices)
                std::vector<int> member_idx;
                if (g.contains("member_indices"))
                    member_idx = g["member_indices"].get<std::vector<int>>();
                else
                    member_idx = key_idx;

                group.n_keys = static_cast<int>(key_idx.size());
                for (int ki = 0; ki < group.n_keys && ki < EEGroup::MAX_KEYS; ++ki) {
                    int kidx = key_idx[ki];
                    const auto& ks = robot.ee_spheres_[kidx];
                    group.keys[ki].center[0] = ks.center[0];
                    group.keys[ki].center[1] = ks.center[1];
                    group.keys[ki].center[2] = ks.center[2];
                    group.keys[ki].coverage_radius = ks.radius;  // init to own radius
                }

                // Nearest-key assignment: each member assigned to closest key,
                // coverage_radius = max(|c_member - c_key| + r_member) over assigned members
                for (int mi : member_idx) {
                    const auto& ms = robot.ee_spheres_[mi];
                    // Find nearest key
                    int best_ki = 0;
                    double best_dist = 1e30;
                    for (int ki = 0; ki < group.n_keys && ki < EEGroup::MAX_KEYS; ++ki) {
                        double dx = ms.center[0] - group.keys[ki].center[0];
                        double dy = ms.center[1] - group.keys[ki].center[1];
                        double dz = ms.center[2] - group.keys[ki].center[2];
                        double d = std::sqrt(dx*dx + dy*dy + dz*dz);
                        if (d < best_dist) { best_dist = d; best_ki = ki; }
                    }
                    double cr = best_dist + ms.radius;
                    if (cr > group.keys[best_ki].coverage_radius)
                        group.keys[best_ki].coverage_radius = cr;
                }
                robot.ee_groups_.push_back(group);
            }
        }
    }

    return robot;
}

void Robot::pack_arrays() {
    dh_alpha_.resize(n_joints_);
    dh_a_.resize(n_joints_);
    dh_d_.resize(n_joints_);
    dh_theta_.resize(n_joints_);
    dh_joint_type_.resize(n_joints_);

    for (int i = 0; i < n_joints_; ++i) {
        dh_alpha_[i]      = dh_params_[i].alpha;
        dh_a_[i]          = dh_params_[i].a;
        dh_d_[i]          = dh_params_[i].d;
        dh_theta_[i]      = dh_params_[i].theta;
        dh_joint_type_[i] = dh_params_[i].joint_type;
    }

    // Total link segments: n_joints (one per DH row) + 1 if tool
    // Link i goes from transform prefix[i] to prefix[i+1]
    int n_total_links = n_joints_ + (has_tool() ? 1 : 0);
    n_links_ = n_total_links;

    // Determine which links to skip:
    //  - Link 0 (base link): position is fixed at (0,0,0)→(0,0,d),
    //    not affected by any joint angle, skip collision checking
    //  - Zero-length links: DH row with a≈0 and d≈0, degenerate
    std::vector<bool> skip(n_total_links, false);
    skip[0] = true;  // always skip link 0 (base link)
    for (int i = 1; i < n_joints_; ++i) {
        if (std::abs(dh_params_[i].a) < 1e-10 &&
            std::abs(dh_params_[i].d) < 1e-10) {
            skip[i] = true;  // zero-length link
        }
    }
    if (has_tool()) {
        int tool_idx = n_joints_;
        if (std::abs(tool_frame_->a) < 1e-10 &&
            std::abs(tool_frame_->d) < 1e-10) {
            skip[tool_idx] = true;  // zero-length tool
        }
    }

    // Build active link map (compact index → original link index)
    active_link_map_.clear();
    for (int i = 0; i < n_total_links; ++i) {
        if (!skip[i])
            active_link_map_.push_back(i);
    }
    n_active_links_ = static_cast<int>(active_link_map_.size());

    // Build compact link radii for active links
    active_link_radii_.clear();
    if (!link_radii_.empty()) {
        active_link_radii_.reserve(n_active_links_);
        for (int ci = 0; ci < n_active_links_; ++ci) {
            int orig = active_link_map_[ci];
            if (orig < static_cast<int>(link_radii_.size()))
                active_link_radii_.push_back(link_radii_[orig]);
            else
                active_link_radii_.push_back(0.0);
        }
    }
}

Eigen::MatrixXd Robot::fk_link_positions(const Eigen::VectorXd& q) const {
    auto positions = sbf::fk_link_positions(*this, q);
    Eigen::MatrixXd result(static_cast<int>(positions.size()), 3);
    for (int i = 0; i < static_cast<int>(positions.size()); ++i)
        result.row(i) = positions[i].transpose();
    return result;
}

std::vector<Eigen::Matrix4d> Robot::fk_transforms(const Eigen::VectorXd& q) const {
    return sbf::fk_transforms(*this, q);
}

std::string Robot::fingerprint() const {
    // Hash from DH parameters + limits + link_radii
    std::ostringstream ss;
    ss << name_ << "|" << n_joints_;
    for (int i = 0; i < n_joints_; ++i) {
        ss << "|" << dh_alpha_[i] << "," << dh_a_[i] << ","
           << dh_d_[i] << "," << dh_theta_[i];
    }
    for (auto& l : limits_.limits)
        ss << "|" << l.lo << "," << l.hi;
    ss << "|radii";
    for (auto& r : link_radii_)
        ss << "," << r;
    if (!ee_spheres_.empty()) {
        ss << "|ee_frame=" << ee_spheres_frame_;
        for (auto& sp : ee_spheres_) {
            ss << "|" << sp.center[0] << "," << sp.center[1] << ","
               << sp.center[2] << "," << sp.radius;
        }
        ss << "|ee_groups=" << ee_groups_.size();
        for (auto& g : ee_groups_) {
            ss << "|g" << g.n_keys;
            for (int k = 0; k < g.n_keys; ++k)
                ss << "," << g.keys[k].coverage_radius;
        }
    }
    std::string s = ss.str();
    // Simple hash
    size_t h = std::hash<std::string>{}(s);
    std::ostringstream hs;
    hs << std::hex << h;
    return hs.str();
}

} // namespace sbf

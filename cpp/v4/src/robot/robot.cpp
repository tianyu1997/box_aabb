// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Robot model implementation
//  迁移自 v3 robot.cpp，适配 v4 include 路径
// ═══════════════════════════════════════════════════════════════════════════
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
    n_links_  = n_joints_;
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

    // Parse coupled joint pairs (optional, for critical-point constraint enum)
    if (j.contains("coupled_pairs")) {
        for (auto& p : j["coupled_pairs"]) {
            robot.coupled_pairs_.emplace_back(p[0].get<int>(), p[1].get<int>());
        }
    }
    // Parse coupled joint triples (optional)
    if (j.contains("coupled_triples")) {
        for (auto& t : j["coupled_triples"]) {
            robot.coupled_triples_.emplace_back(
                t[0].get<int>(), t[1].get<int>(), t[2].get<int>());
        }
    }

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
                    group.keys[ki].coverage_radius = ks.radius;
                }

                for (int mi : member_idx) {
                    const auto& ms = robot.ee_spheres_[mi];
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

    int n_total_links = n_joints_ + (has_tool() ? 1 : 0);
    n_links_ = n_total_links;

    std::vector<bool> skip(n_total_links, false);
    skip[0] = true;
    for (int i = 1; i < n_joints_; ++i) {
        if (std::abs(dh_params_[i].a) < 1e-10 &&
            std::abs(dh_params_[i].d) < 1e-10) {
            skip[i] = true;
        }
    }
    if (has_tool()) {
        int tool_idx = n_joints_;
        if (std::abs(tool_frame_->a) < 1e-10 &&
            std::abs(tool_frame_->d) < 1e-10) {
            skip[tool_idx] = true;
        }
    }

    active_link_map_.clear();
    for (int i = 0; i < n_total_links; ++i) {
        if (!skip[i])
            active_link_map_.push_back(i);
    }
    n_active_links_ = static_cast<int>(active_link_map_.size());

    // Last FK frame needed: distal of last active link = max(alm) + 1
    last_active_frame_ = 0;
    if (n_active_links_ > 0)
        last_active_frame_ = active_link_map_.back() + 1;

    // Per-link affecting joint count: serial DH chain → alm[ci] + 1
    n_affecting_joints_.resize(n_active_links_);
    for (int ci = 0; ci < n_active_links_; ++ci)
        n_affecting_joints_[ci] = active_link_map_[ci] + 1;

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

    // ── Build iAABB-specific preprocessed config ────────────────────────
    //  General-purpose DH → iAABB conversion with 0.1 threshold.
    //  Uses sqrt(a² + d²) < 0.1 to filter short links (more aggressive
    //  than the 1e-10 threshold above, which is for backward compat).
    iaabb_config_ = IAABBConfig::from_robot(*this);
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
    size_t h = std::hash<std::string>{}(s);
    std::ostringstream hs;
    hs << std::hex << h;
    return hs.str();
}

} // namespace sbf

#include <sbf/core/robot.h>

#include <nlohmann/json.hpp>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <cmath>

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
    pack_arrays();
}

Robot Robot::from_json(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open())
        return Robot{};  // return empty robot on failure

    nlohmann::json j;
    f >> j;

    std::string name = j.value("name", "unknown");

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

    JointLimits lim;
    for (auto& l : j["joint_limits"]) {
        lim.limits.push_back({l[0].get<double>(), l[1].get<double>()});
    }

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

    std::vector<double> radii;
    if (j.contains("link_radii")) {
        for (auto& r : j["link_radii"])
            radii.push_back(r.get<double>());
    }

    Robot robot(std::move(name), std::move(dh), std::move(lim),
                std::move(tool), std::move(radii));

    if (j.contains("coupled_pairs")) {
        for (auto& p : j["coupled_pairs"]) {
            robot.coupled_pairs_.emplace_back(p[0].get<int>(), p[1].get<int>());
        }
    }

    return robot;
}

// ── FNV-1a 64-bit fingerprint ──────────────────────────────────────────────
uint64_t Robot::fingerprint() const {
    constexpr uint64_t FNV_OFFSET = 14695981039346656037ULL;
    constexpr uint64_t FNV_PRIME  = 1099511628211ULL;

    uint64_t h = FNV_OFFSET;

    auto hash_bytes = [&](const void* data, size_t n) {
        auto* p = static_cast<const uint8_t*>(data);
        for (size_t i = 0; i < n; ++i) {
            h ^= p[i];
            h *= FNV_PRIME;
        }
    };

    // Structure: n_joints, n_active_links
    hash_bytes(&n_joints_, sizeof(n_joints_));
    hash_bytes(&n_active_links_, sizeof(n_active_links_));

    // DH params (full kinematic identity — v4 missed this)
    for (int j = 0; j < n_joints_; ++j) {
        hash_bytes(&dh_params_[j].alpha, sizeof(double));
        hash_bytes(&dh_params_[j].a,     sizeof(double));
        hash_bytes(&dh_params_[j].d,     sizeof(double));
        hash_bytes(&dh_params_[j].theta, sizeof(double));
        hash_bytes(&dh_params_[j].joint_type, sizeof(int));
    }

    // Joint limits
    for (int j = 0; j < n_joints_; ++j) {
        hash_bytes(&limits_.limits[j].lo, sizeof(double));
        hash_bytes(&limits_.limits[j].hi, sizeof(double));
    }

    // Tool frame (if present)
    if (tool_frame_) {
        uint8_t has = 1;
        hash_bytes(&has, 1);
        hash_bytes(&tool_frame_->alpha, sizeof(double));
        hash_bytes(&tool_frame_->a,     sizeof(double));
        hash_bytes(&tool_frame_->d,     sizeof(double));
        hash_bytes(&tool_frame_->theta, sizeof(double));
    } else {
        uint8_t has = 0;
        hash_bytes(&has, 1);
    }

    // Active link radii
    if (!active_link_radii_.empty())
        hash_bytes(active_link_radii_.data(),
                   active_link_radii_.size() * sizeof(double));

    // Active link map
    if (!active_link_map_.empty())
        hash_bytes(active_link_map_.data(),
                   active_link_map_.size() * sizeof(int));

    // Coupled pairs
    for (auto& [a, b] : coupled_pairs_) {
        hash_bytes(&a, sizeof(int));
        hash_bytes(&b, sizeof(int));
    }

    return h;
}

void Robot::pack_arrays() {
    int n_total_links = n_joints_ + (has_tool() ? 1 : 0);

    // Determine zero-length links to skip
    std::vector<bool> skip(n_total_links, false);
    skip[0] = true;  // base link always skipped
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

    last_active_frame_ = 0;
    if (n_active_links_ > 0)
        last_active_frame_ = active_link_map_.back() + 1;

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

}  // namespace sbf

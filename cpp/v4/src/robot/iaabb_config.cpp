// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — IAABBConfig implementation
//
//  通用 DH → iAABB 预处理：from_robot() + JSON 序列化。
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/robot/iaabb_config.h"
#include "sbf/robot/robot.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace sbf {

// ═════════════════════════════════════════════════════════════════════════════
//  IAABBConfig::from_robot — 通用 DH 链预处理
//
//  适用于任意 N-DOF 串联 DH 链（Panda / IIWA14 / UR5 / 自定义等）。
//  不依赖特定机器人型号的硬编码逻辑。
// ═════════════════════════════════════════════════════════════════════════════

IAABBConfig IAABBConfig::from_robot(const Robot& robot, double threshold) {
    IAABBConfig cfg;
    cfg.zero_length_threshold = threshold;
    cfg.n_joints     = robot.n_joints();
    cfg.has_tool     = robot.has_tool();
    cfg.n_total_links = cfg.n_joints + (cfg.has_tool ? 1 : 0);

    const auto& dh = robot.dh_params();

    // ── Step 1: Compute link lengths for ALL links ──────────────────────
    //  link_length[i] = sqrt(a_i² + d_i²)
    //  This is the fixed geometric distance between frame i and frame i+1
    //  in the DH convention, independent of joint angle.
    cfg.all_link_lengths.resize(cfg.n_total_links);
    cfg.link_is_active.resize(cfg.n_total_links, false);

    for (int i = 0; i < cfg.n_joints; ++i) {
        double a = dh[i].a;
        double d = dh[i].d;
        cfg.all_link_lengths[i] = std::sqrt(a * a + d * d);
    }
    if (cfg.has_tool) {
        const auto& tf = *robot.tool_frame();
        cfg.all_link_lengths[cfg.n_joints] =
            std::sqrt(tf.a * tf.a + tf.d * tf.d);
    }

    // ── Step 2: Determine which links are active ────────────────────────
    //  Rule 1: link 0 is always skipped (base frame — no body to collide)
    //  Rule 2: links with length < threshold are treated as zero-length
    //          and skipped (negligible collision volume)
    for (int i = 0; i < cfg.n_total_links; ++i) {
        if (i == 0) continue;                           // Rule 1: base
        if (cfg.all_link_lengths[i] < threshold) continue;  // Rule 2: short
        cfg.link_is_active[i] = true;
    }

    // ── Step 3: Build active link map ───────────────────────────────────
    cfg.active_link_map.clear();
    for (int i = 0; i < cfg.n_total_links; ++i) {
        if (cfg.link_is_active[i])
            cfg.active_link_map.push_back(i);
    }
    cfg.n_active           = static_cast<int>(cfg.active_link_map.size());
    cfg.n_active_endpoints = cfg.n_active * 2;   // paired layout

    // ── Step 4: last_active_frame ───────────────────────────────────────
    //  The highest frame index referenced by any active link.
    //  Active link ci uses frames prefix[V] (proximal) and prefix[V+1]
    //  (distal), where V = active_link_map[ci].
    cfg.last_active_frame = 0;
    for (int ci = 0; ci < cfg.n_active; ++ci) {
        int distal_frame = cfg.active_link_map[ci] + 1;
        if (distal_frame > cfg.last_active_frame)
            cfg.last_active_frame = distal_frame;
    }

    // ── Step 5: Active link lengths ─────────────────────────────────────
    cfg.active_link_lengths.resize(cfg.n_active);
    for (int ci = 0; ci < cfg.n_active; ++ci)
        cfg.active_link_lengths[ci] =
            cfg.all_link_lengths[cfg.active_link_map[ci]];

    // ── Step 6: Active link radii ───────────────────────────────────────
    cfg.active_link_radii.resize(cfg.n_active, 0.0);
    const auto& radii = robot.link_radii();
    if (!radii.empty()) {
        for (int ci = 0; ci < cfg.n_active; ++ci) {
            int orig = cfg.active_link_map[ci];
            if (orig < static_cast<int>(radii.size()))
                cfg.active_link_radii[ci] = radii[orig];
        }
    }

    // ── Step 7: Per-link affecting joints count ─────────────────────────
    //  In a serial DH chain, the position of frame V+1 depends on joints
    //  0, 1, ..., V.  So the number of affecting joints = V + 1.
    //  For the tool frame (link index = n_joints), all n_joints joints
    //  affect it, but the tool has no additional joint.
    cfg.n_affecting_joints.resize(cfg.n_active);
    for (int ci = 0; ci < cfg.n_active; ++ci) {
        int V = cfg.active_link_map[ci];
        cfg.n_affecting_joints[ci] = std::min(V + 1, cfg.n_joints);
    }

    return cfg;
}

// ═════════════════════════════════════════════════════════════════════════════
//  JSON serialization
// ═════════════════════════════════════════════════════════════════════════════

void IAABBConfig::save_json(const std::string& path) const {
    nlohmann::json j;
    j["format"]  = "iaabb_config";
    j["version"] = 1;

    j["zero_length_threshold"] = zero_length_threshold;
    j["n_joints"]       = n_joints;
    j["has_tool"]        = has_tool;
    j["n_total_links"]   = n_total_links;

    j["n_active"]            = n_active;
    j["n_active_endpoints"]  = n_active_endpoints;
    j["last_active_frame"]   = last_active_frame;

    j["active_link_map"]     = active_link_map;
    j["active_link_radii"]   = active_link_radii;
    j["active_link_lengths"] = active_link_lengths;
    j["n_affecting_joints"]  = n_affecting_joints;

    j["all_link_lengths"] = all_link_lengths;

    // vector<bool> → int array (JSON doesn't handle vector<bool> directly)
    std::vector<int> flags(n_total_links);
    for (int i = 0; i < n_total_links; ++i)
        flags[i] = link_is_active[i] ? 1 : 0;
    j["link_is_active"] = flags;

    std::ofstream f(path);
    if (!f.is_open())
        throw std::runtime_error(
            "IAABBConfig::save_json: cannot open " + path);
    f << j.dump(2);
}

IAABBConfig IAABBConfig::load_json(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error(
            "IAABBConfig::load_json: cannot open " + path);

    nlohmann::json j;
    f >> j;

    // Validate format tag
    if (j.contains("format") && j["format"] != "iaabb_config")
        throw std::runtime_error(
            "IAABBConfig::load_json: unexpected format '" +
            j["format"].get<std::string>() + "'");

    IAABBConfig cfg;
    cfg.zero_length_threshold = j.value("zero_length_threshold", 0.1);
    cfg.n_joints       = j.value("n_joints", 0);
    cfg.has_tool        = j.value("has_tool", false);
    cfg.n_total_links   = j.value("n_total_links", 0);

    cfg.n_active            = j.value("n_active", 0);
    cfg.n_active_endpoints  = j.value("n_active_endpoints", 0);
    cfg.last_active_frame   = j.value("last_active_frame", 0);

    cfg.active_link_map     = j["active_link_map"].get<std::vector<int>>();
    cfg.active_link_radii   = j["active_link_radii"].get<std::vector<double>>();
    cfg.active_link_lengths = j["active_link_lengths"].get<std::vector<double>>();
    cfg.n_affecting_joints  = j["n_affecting_joints"].get<std::vector<int>>();

    cfg.all_link_lengths = j["all_link_lengths"].get<std::vector<double>>();

    auto flags = j["link_is_active"].get<std::vector<int>>();
    cfg.link_is_active.resize(flags.size());
    for (size_t i = 0; i < flags.size(); ++i)
        cfg.link_is_active[i] = (flags[i] != 0);

    return cfg;
}

// ═════════════════════════════════════════════════════════════════════════════
//  summary — human-readable debug string
// ═════════════════════════════════════════════════════════════════════════════

std::string IAABBConfig::summary() const {
    std::ostringstream ss;
    ss << "IAABBConfig: n_joints=" << n_joints
       << "  has_tool=" << (has_tool ? "Y" : "N")
       << "  n_total_links=" << n_total_links
       << "  threshold=" << zero_length_threshold << "\n";

    ss << "  All links (length -> status):\n";
    for (int i = 0; i < n_total_links; ++i) {
        ss << "    link " << i << ": len="
           << all_link_lengths[i];
        if (i == 0)
            ss << "  [SKIP: base]";
        else if (!link_is_active[i])
            ss << "  [SKIP: < " << zero_length_threshold << "]";
        else
            ss << "  [ACTIVE]";
        ss << "\n";
    }

    ss << "  Active links: [";
    for (int ci = 0; ci < n_active; ++ci) {
        if (ci > 0) ss << ", ";
        ss << active_link_map[ci];
    }
    ss << "]  (n_active=" << n_active << ")\n";

    ss << "  n_active_endpoints=" << n_active_endpoints
       << "  last_active_frame=" << last_active_frame << "\n";

    ss << "  Per-link metadata:\n";
    for (int ci = 0; ci < n_active; ++ci) {
        ss << "    [" << ci << "]"
           << "  frame=" << active_link_map[ci]
           << "  length=" << active_link_lengths[ci]
           << "  radius=" << active_link_radii[ci]
           << "  n_affecting=" << n_affecting_joints[ci]
           << "\n";
    }

    return ss.str();
}

} // namespace sbf

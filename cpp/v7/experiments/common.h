/// @file experiments/common.h
/// @brief Tiny helpers shared by experiment binaries (P6).
///
/// Keep this header-only and dependency-free (apart from nlohmann_json
/// already pulled in for `Robot::from_json`).
#pragma once

#include "sbf/core/endpoint_iaabb.h"
#include "sbf/core/robot.h"
#include "sbf/envelope/envelope_type.h"
#include "sbf/lect/lect.h"
#include "sbf/planner/sbf_planner.h"
#include "sbf/scene/scene_config.h"

#include <nlohmann/json.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace sbf::exp {

/// Resolve a robot id (e.g. "iiwa14") to an absolute json path.
inline std::string robot_json_path(const std::string& robot_id) {
    return std::string(SBF_DATA_DIR) + "/" + robot_id + ".json";
}

/// Build a LECT for a robot.  Defaults to LinkIAABB / n_sub=1 (cheap), but
/// callers can request tighter envelopes (LinkIAABB_Grid, n_sub>1) when the
/// scene needs them.
inline sbf::lect::LECT make_lect(const sbf::core::Robot& r,
                                 sbf::envelope::EnvelopeType type =
                                     sbf::envelope::EnvelopeType::LinkIAABB,
                                 int    n_subdivisions = 1,
                                 double voxel_delta    = 0.05,
                                 sbf::core::EndpointSourceKind endpoint_source =
                                     sbf::core::EndpointSourceKind::IFK) {
    std::vector<sbf::core::Interval> iv;
    for (const auto& l : r.joint_limits().limits) iv.push_back(l);
    sbf::envelope::EnvelopeTypeConfig ec;
    ec.type                    = type;
    ec.n_subdivisions          = n_subdivisions;
    ec.grid_config.voxel_delta = voxel_delta;
    sbf::lect::LECTConfig lc;
    lc.endpoint_source.kind = endpoint_source;
    return sbf::lect::LECT(r, iv, ec, lc);
}

inline sbf::lect::LECT load_or_make_lect(
    const sbf::core::Robot&        r,
    sbf::envelope::EnvelopeType    type,
    int                            n_subdivisions,
    double                         voxel_delta,
    const std::string&             cache_path,
    bool*                          loaded_existing = nullptr,
    sbf::core::EndpointSourceKind  endpoint_source =
        sbf::core::EndpointSourceKind::IFK) {
    if (loaded_existing) *loaded_existing = false;
    if (!cache_path.empty() && std::filesystem::exists(cache_path)) {
        auto lect = sbf::lect::LECT::load_binary(cache_path, r);
        const auto cached_source = lect.config().endpoint_source.kind;
        if (cached_source != endpoint_source) {
            throw std::runtime_error(
                std::string("LECT cache endpoint-source mismatch: cache=") +
                sbf::core::endpoint_source_name(cached_source) +
                " requested=" + sbf::core::endpoint_source_name(endpoint_source));
        }
        lect.clear_all_occupation();
        if (loaded_existing) *loaded_existing = true;
        std::cout << "[lect-cache] loaded " << cache_path
                  << " and cleared scene-specific occupation\n";
        return lect;
    }
    if (!cache_path.empty()) {
        std::cout << "[lect-cache] starting new cache at " << cache_path << "\n";
    }
    return make_lect(r, type, n_subdivisions, voxel_delta, endpoint_source);
}

inline void save_lect_geometry_cache(const sbf::lect::LECT& lect,
                                     const std::string&     cache_path) {
    if (cache_path.empty()) return;
    auto cache_copy = lect.snapshot();
    cache_copy.clear_all_occupation();
    cache_copy.save_binary(cache_path);
    std::cout << "[lect-cache] saved geometry cache to " << cache_path << "\n";
}

/// Convert a PlanResult to a JSON record (one trial).
inline nlohmann::json plan_to_json(const sbf::planner::PlanResult& r) {
    return {
        {"success",            r.success},
        {"final_state",        sbf::planner::to_string(r.final_state)},
        {"n_boxes",            r.n_boxes},
        {"n_islands",          r.n_islands},
        {"start_box",          r.start_box},
        {"goal_box",           r.goal_box},
        {"raw_length",         r.raw_length},
        {"opt_length",         r.opt_length},
        {"grow_time_ms",       r.grow_time_ms},
        {"path_find_time_ms",  r.path_find_time_ms},
        {"opt_time_ms",        r.opt_time_ms},
        {"total_time_ms",      r.total_time_ms},
        {"step_lengths",       r.step_lengths},
        {"path_size",          static_cast<int>(r.path.size())},
        {"fail_reason",        r.fail_reason},
        {"used_point_bridge",  r.used_point_bridge},
    };
}

/// Write `j` to `out_path` (mkdir parents). Pretty-printed with 2 spaces.
inline void write_json(const std::filesystem::path& out_path,
                       const nlohmann::json&         j) {
    std::filesystem::create_directories(out_path.parent_path());
    std::ofstream f(out_path);
    if (!f) throw std::runtime_error("cannot write " + out_path.string());
    f << j.dump(2) << "\n";
}

/// Parse a flag like "--quick" / "--full" / "--scene=path.json".
struct CliArgs {
    bool        quick          = false;
    int         seeds          = 3;          ///< default in --quick mode
    double      timeout_s      = 30.0;
    std::string scene_path;
    std::string query_scenes_csv;                ///< optional comma-separated scene list for multi-query OMPL baselines
    std::string out_path;
    int         n_threads      = 0;          ///< 0 → grower auto
    std::string env_type       = "link_iaabb";  ///< link_iaabb | link_iaabb_grid | hull16_grid
    int         n_subdivisions = 1;
    double      voxel_delta    = 0.05;
    int         ffb_max_depth  = -1;          ///< <0 → leave default
    int         max_boxes      = -1;          ///< <0 → quick/full default
    int         bridge_boxes   = -1;          ///< <0 → grower default
    bool        point_bridge   = false;
    double      point_bridge_timeout_ms = 3000.0;
    std::string planner        = "rrt_connect"; ///< OMPL planner id (baseline_ompl)
    double      cost_threshold = -1.0;          ///< optional optimization target for OMPL baselines
    bool        no_simplify    = false;         ///< skip post-solve simplify for fair anytime timing
    uint64_t    seed_base      = 42;            ///< OMPL global RNG seed base
    std::string lect_cache_path;                 ///< optional persisted LECT path
    std::string endpoint_source = "ifk";         ///< ifk | critsample

    sbf::envelope::EnvelopeType env_type_enum() const {
        if (env_type == "link_iaabb_grid")
            return sbf::envelope::EnvelopeType::LinkIAABB_Grid;
        if (env_type == "hull16_grid")
            return sbf::envelope::EnvelopeType::Hull16_Grid;
        return sbf::envelope::EnvelopeType::LinkIAABB;
    }

    sbf::core::EndpointSourceKind endpoint_source_kind() const {
        return sbf::core::parse_endpoint_source_kind(endpoint_source);
    }

    static CliArgs parse(int argc, char** argv) {
        CliArgs a;
        for (int i = 1; i < argc; ++i) {
            std::string s = argv[i];
            auto eat = [&](const char* k) -> std::string {
                std::string p = std::string("--") + k + "=";
                if (s.rfind(p, 0) == 0) return s.substr(p.size());
                return {};
            };
            if      (s == "--quick") { a.quick = true; a.seeds = 3;  a.timeout_s = 30.0; }
            else if (s == "--full")  { a.quick = false;a.seeds = 20; a.timeout_s = 120.0; }
            else if (auto v = eat("scene");        !v.empty()) a.scene_path     = v;
            else if (auto v = eat("query-scenes"); !v.empty()) a.query_scenes_csv = v;
            else if (auto v = eat("out");          !v.empty()) a.out_path       = v;
            else if (auto v = eat("seeds");        !v.empty()) a.seeds          = std::stoi(v);
            else if (auto v = eat("timeout");      !v.empty()) a.timeout_s      = std::stod(v);
            else if (auto v = eat("threads");      !v.empty()) a.n_threads      = std::stoi(v);
            else if (auto v = eat("env");          !v.empty()) a.env_type       = v;
            else if (auto v = eat("n-sub");        !v.empty()) a.n_subdivisions = std::stoi(v);
            else if (auto v = eat("voxel-delta");  !v.empty()) a.voxel_delta    = std::stod(v);
            else if (auto v = eat("ffb-depth");    !v.empty()) a.ffb_max_depth  = std::stoi(v);
            else if (auto v = eat("max-boxes");    !v.empty()) a.max_boxes      = std::stoi(v);
            else if (auto v = eat("bridge-boxes"); !v.empty()) a.bridge_boxes   = std::stoi(v);
            else if (auto v = eat("point-bridge-timeout-ms"); !v.empty()) a.point_bridge_timeout_ms = std::stod(v);
            else if (auto v = eat("planner");      !v.empty()) a.planner        = v;
            else if (auto v = eat("cost-threshold"); !v.empty()) a.cost_threshold = std::stod(v);
            else if (auto v = eat("seed-base");    !v.empty()) a.seed_base      = static_cast<uint64_t>(std::stoull(v));
            else if (auto v = eat("lect-cache");   !v.empty()) a.lect_cache_path = v;
            else if (auto v = eat("endpoint-source"); !v.empty()) a.endpoint_source = v;
            else if (auto v = eat("endpoint"); !v.empty()) a.endpoint_source = v;
            else if (s == "--point-bridge") a.point_bridge = true;
            else if (s == "--no-simplify") a.no_simplify = true;
            else {
                std::cerr << "unknown arg: " << s << "\n";
                std::exit(2);
            }
        }
        if (a.scene_path.empty()) {
            std::cerr << "usage: <bin> --scene=<path.json> --out=<path.json> "
                         "[--quick|--full] [--seeds=N] [--timeout=SEC] "
                         "[--threads=N] [--lect-cache=<path.bin>]\n";
            std::exit(2);
        }
        return a;
    }
};

/// Run a single trial with `seed` and return the PlanResult + timing.
inline sbf::planner::PlanResult run_trial(const sbf::scene::SceneConfig& sc,
                                          uint64_t                       seed,
                                          int                            n_threads,
                                          double                         timeout_s,
                                          bool                           quick,
                                          const CliArgs&                 args = CliArgs{}) {
    auto robot = sbf::core::Robot::from_json(robot_json_path(sc.robot));
    auto lect  = load_or_make_lect(robot,
                                   args.env_type_enum(),
                                   args.n_subdivisions,
                                   args.voxel_delta,
                                   args.lect_cache_path,
                                   nullptr,
                                   args.endpoint_source_kind());

    sbf::planner::PlannerConfig cfg;
    cfg.grower.rng_seed   = seed;
    cfg.grower.n_threads  = n_threads;
    cfg.grower.max_boxes  = args.max_boxes > 0
                            ? args.max_boxes
                            : (quick ? 200 : 800);
    cfg.grower.timeout_ms = static_cast<double>(timeout_s) * 1000.0;
    // Low-DoF (≤3) scenes need shallower FFB or boxes degenerate.
    if (robot.n_joints() <= 3) cfg.grower.ffb.max_depth = 8;
    if (args.ffb_max_depth > 0) cfg.grower.ffb.max_depth = args.ffb_max_depth;
    if (args.bridge_boxes > 0) cfg.grower.endpoint_bridge_max_boxes = args.bridge_boxes;
    cfg.point_bridge_fallback = args.point_bridge;
    cfg.point_bridge_timeout_ms = args.point_bridge_timeout_ms;
    cfg.timeout_s         = static_cast<double>(timeout_s);
    cfg.quick_mode        = quick;

    sbf::planner::SbfPlanner planner(robot, lect, cfg);
    auto packed = sc.packed_obstacles();
    auto result = planner.plan(sc.q_start, sc.q_goal,
                               packed.empty() ? nullptr : packed.data(),
                               static_cast<int>(sc.obstacles.size()));
    save_lect_geometry_cache(lect, args.lect_cache_path);
    return result;
}

}  // namespace sbf::exp

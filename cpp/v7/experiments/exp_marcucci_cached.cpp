/// @file exp_marcucci_cached.cpp
/// @brief Cached-query Marcucci runner for the v7 paper.
///
/// Build protocol:
///   1. Build one shared multi-root coverage forest from the five canonical
///      Marcucci postures (AS, TS, CS, LB, RB).
///   2. Freeze that shared forest and measure per-query path retrieval and
///      local refinement on the already built box graph.
///
/// The emitted JSON matches the historical marcucci.json schema consumed by
/// build_paper_tables.py and build_paper_figures.py.
#include "experiments/common.h"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace fs = std::filesystem;

namespace {

using Clock = std::chrono::steady_clock;

struct Args {
    bool        quick          = false;
    int         seeds          = 10;
    int         timeout_s      = 120;
    int         n_threads      = 1;
    int         n_subdivisions = 4;
    double      voxel_delta    = 0.05;
    int         ffb_max_depth  = 55;
    int         max_boxes      = 2500;
    int         bridge_boxes   = 2000;
    int         post_connect_extra_boxes = 4000;
    std::string env_type       = "link_iaabb_grid";
    std::string endpoint_source = "ifk";
    std::string lect_cache_path;
    int         seed_offset    = 0;
    fs::path    scene_dir;
    fs::path    out_path;

    sbf::envelope::EnvelopeType env_type_enum() const {
        if (env_type == "link_iaabb_grid") {
            return sbf::envelope::EnvelopeType::LinkIAABB_Grid;
        }
        if (env_type == "hull16_grid") {
            return sbf::envelope::EnvelopeType::Hull16_Grid;
        }
        return sbf::envelope::EnvelopeType::LinkIAABB;
    }

    sbf::core::EndpointSourceKind endpoint_source_kind() const {
        return sbf::core::parse_endpoint_source_kind(endpoint_source);
    }
};

std::string eat_value(const std::string& s, const char* key) {
    std::string p = std::string("--") + key + "=";
    if (s.rfind(p, 0) == 0) return s.substr(p.size());
    return {};
}

Args parse_args(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string s = argv[i];
        if (s == "--quick") {
            a.quick = true;
            a.seeds = 3;
            a.timeout_s = 30;
            a.max_boxes = 2500;
        } else if (s == "--full") {
            a.quick = false;
            a.seeds = 10;
            a.timeout_s = 120;
            a.max_boxes = 2500;
        } else if (auto v = eat_value(s, "scene-dir"); !v.empty()) {
            a.scene_dir = v;
        } else if (auto v = eat_value(s, "out"); !v.empty()) {
            a.out_path = v;
        } else if (auto v = eat_value(s, "seeds"); !v.empty()) {
            a.seeds = std::stoi(v);
        } else if (auto v = eat_value(s, "timeout"); !v.empty()) {
            a.timeout_s = std::stoi(v);
        } else if (auto v = eat_value(s, "threads"); !v.empty()) {
            a.n_threads = std::stoi(v);
        } else if (auto v = eat_value(s, "env"); !v.empty()) {
            a.env_type = v;
        } else if (auto v = eat_value(s, "endpoint-source"); !v.empty()) {
            a.endpoint_source = v;
        } else if (auto v = eat_value(s, "endpoint"); !v.empty()) {
            a.endpoint_source = v;
        } else if (auto v = eat_value(s, "n-sub"); !v.empty()) {
            a.n_subdivisions = std::stoi(v);
        } else if (auto v = eat_value(s, "voxel-delta"); !v.empty()) {
            a.voxel_delta = std::stod(v);
        } else if (auto v = eat_value(s, "ffb-depth"); !v.empty()) {
            a.ffb_max_depth = std::stoi(v);
        } else if (auto v = eat_value(s, "max-boxes"); !v.empty()) {
            a.max_boxes = std::stoi(v);
        } else if (auto v = eat_value(s, "bridge-boxes"); !v.empty()) {
            a.bridge_boxes = std::stoi(v);
        } else if (auto v = eat_value(s, "post-connect-extra-boxes"); !v.empty()) {
            a.post_connect_extra_boxes = std::stoi(v);
        } else if (auto v = eat_value(s, "lect-cache"); !v.empty()) {
            a.lect_cache_path = v;
        } else if (auto v = eat_value(s, "seed-offset"); !v.empty()) {
            a.seed_offset = std::stoi(v);
        } else {
            std::cerr << "unknown arg: " << s << "\n";
            std::exit(2);
        }
    }
    if (a.scene_dir.empty() || a.out_path.empty()) {
        std::cerr << "usage: exp_marcucci_cached --scene-dir=<configs/marcucci> "
                     "--out=<json> [--quick|--full] [--seeds=N] [--timeout=SEC] "
                     "[--threads=N] [--env=link_iaabb_grid] "
                     "[--endpoint-source=ifk|critsample] [--n-sub=4] "
                     "[--max-boxes=4000] [--bridge-boxes=2000] "
                     "[--post-connect-extra-boxes=4000] "
                     "[--lect-cache=<path.bin>] [--seed-offset=N]\n";
        std::exit(2);
    }
    return a;
}

double mean(const std::vector<double>& xs) {
    if (xs.empty()) return 0.0;
    return std::accumulate(xs.begin(), xs.end(), 0.0) /
           static_cast<double>(xs.size());
}

double median(std::vector<double> xs) {
    if (xs.empty()) return 0.0;
    std::sort(xs.begin(), xs.end());
    const std::size_t n = xs.size();
    return (n % 2) ? xs[n / 2] : 0.5 * (xs[n / 2 - 1] + xs[n / 2]);
}

struct QuerySpec {
    const char* label;
    const char* from;
    const char* to;
    const char* file;
};

std::string query_name(const QuerySpec& spec) {
    return spec.label;
}

}  // namespace

int main(int argc, char** argv) {
    const Args a = parse_args(argc, argv);
    const std::vector<QuerySpec> query_specs = {
        {"AS->TS", "AS", "TS", "AS_TS.json"},
        {"TS->CS", "TS", "CS", "TS_CS.json"},
        {"CS->LB", "CS", "LB", "CS_LB.json"},
        {"LB->RB", "LB", "RB", "LB_RB.json"},
        {"RB->AS", "RB", "AS", "RB_AS.json"},
    };

    std::vector<sbf::scene::SceneConfig> scenes;
    scenes.reserve(query_specs.size());
    for (const auto& spec : query_specs) {
        scenes.push_back(sbf::scene::load_scene_json((a.scene_dir / spec.file).string()));
    }
    if (scenes.empty()) {
        std::cerr << "no Marcucci scene files loaded\n";
        return 2;
    }

    const auto robot = sbf::core::Robot::from_json(
        sbf::exp::robot_json_path(scenes.front().robot));
    const auto packed = scenes.front().packed_obstacles();
    const float* obs = packed.empty() ? nullptr : packed.data();
    const int n_obs = static_cast<int>(scenes.front().obstacles.size());

    nlohmann::json out = {
        {"experiment", "marcucci"},
        {"robot", scenes.front().robot},
        {"scene", "marcucci_combined"},
        {"source_protocol", "v7_live_build_coverage_query"},
        {"env", a.env_type},
        {"endpoint_source", sbf::core::endpoint_source_name(a.endpoint_source_kind())},
        {"n_subdivisions", a.n_subdivisions},
        {"voxel_delta", a.voxel_delta},
        {"v7_live_config", {
            {"raw_max_boxes", std::max(200000, a.max_boxes)},
            {"post_connect_extra_boxes", a.post_connect_extra_boxes},
            {"ffb_depth", std::max(300, a.ffb_max_depth)},
            {"threads", a.n_threads},
            {"point_bridge_fallback", true},
        }},
        {"trials", nlohmann::json::array()},
        {"queries", nlohmann::json::array()},
        {"build", {{"median_s", 0.0}, {"mean_s", 0.0}}},
        {"seeds", a.seeds},
    };

    std::vector<double> build_times_s;
    std::vector<double> planner_build_total_ms_samples;
    std::vector<double> build_grow_ms_samples;
    std::vector<double> build_adjacency_ms_samples;
    std::vector<double> build_overhead_ms_samples;
    std::vector<double> n_boxes_samples;
    std::vector<double> unique_box_count_samples;
    std::vector<double> duplicate_box_count_samples;
    std::vector<double> box_volume_sum_samples;
    std::vector<double> unique_box_volume_sum_samples;
    std::vector<double> duplicate_box_volume_sum_samples;
    std::vector<std::vector<double>> query_times_s(scenes.size());
    std::vector<std::vector<double>> query_lengths(scenes.size());
    std::vector<int> query_successes(scenes.size(), 0);

    std::vector<Eigen::VectorXd> coverage_roots;
    std::vector<std::string> coverage_labels;
    auto add_coverage_root = [&](const char* label, const Eigen::VectorXd& q) {
        if (std::find(coverage_labels.begin(), coverage_labels.end(), label) !=
            coverage_labels.end()) {
            return;
        }
        coverage_labels.emplace_back(label);
        coverage_roots.push_back(q);
    };
    for (std::size_t qi = 0; qi < scenes.size(); ++qi) {
        add_coverage_root(query_specs[qi].from, scenes[qi].q_start);
        add_coverage_root(query_specs[qi].to, scenes[qi].q_goal);
    }

    for (int seed_i = 0; seed_i < a.seeds; ++seed_i) {
        const int seed = a.seed_offset + seed_i;
        bool loaded_existing = false;
        auto cache_lect = sbf::exp::load_or_make_lect(
            robot, a.env_type_enum(), a.n_subdivisions, a.voxel_delta,
            a.lect_cache_path, &loaded_existing, a.endpoint_source_kind());
        auto t_build0 = Clock::now();
        cache_lect.clear_all_occupation();

        sbf::planner::PlannerConfig pcfg;
        pcfg.grower.rng_seed = static_cast<std::uint64_t>(seed * 1000 + 42);
        pcfg.grower.n_threads = a.n_threads;
        pcfg.grower.max_boxes = std::max(200000, a.max_boxes);
        pcfg.grower.timeout_ms = static_cast<double>(a.timeout_s) * 1000.0;
        pcfg.grower.connect_mode = true;
        pcfg.grower.stop_after_connect = false;
        pcfg.grower.post_connect_extra_boxes = a.post_connect_extra_boxes;
        pcfg.grower.endpoint_auto_bridge = false;
        pcfg.grower.rrt_goal_bias = 0.1;
        pcfg.grower.rrt_step_ratio = 0.05;
        pcfg.grower.ffb.max_depth = std::max(300, a.ffb_max_depth);
        pcfg.quick_mode = a.quick;
        pcfg.timeout_s = static_cast<double>(a.timeout_s);
        pcfg.point_bridge_fallback = true;
        pcfg.point_bridge_timeout_ms = 5000.0;
        pcfg.point_bridge_max_iters = 200000;
        pcfg.point_bridge_step = 0.10;
        pcfg.point_bridge_goal_bias = 0.20;

        sbf::planner::SbfPlanner planner(robot, cache_lect, pcfg);
        auto build_result = planner.build_coverage(coverage_roots, obs, n_obs);

        const double build_s = std::chrono::duration<double>(Clock::now() - t_build0).count();
        build_times_s.push_back(build_s);
        planner_build_total_ms_samples.push_back(build_result.total_time_ms);
        build_grow_ms_samples.push_back(build_result.grow_time_ms);
        build_adjacency_ms_samples.push_back(build_result.adjacency_time_ms);
        build_overhead_ms_samples.push_back(
            std::max(0.0, build_s * 1000.0 - build_result.total_time_ms));
        n_boxes_samples.push_back(build_result.n_boxes);
        unique_box_count_samples.push_back(build_result.unique_box_count);
        duplicate_box_count_samples.push_back(build_result.duplicate_box_count);
        box_volume_sum_samples.push_back(build_result.box_volume_sum);
        unique_box_volume_sum_samples.push_back(build_result.unique_box_volume_sum);
        duplicate_box_volume_sum_samples.push_back(build_result.duplicate_box_volume_sum);

        nlohmann::json trial_json = {
            {"seed", seed},
            {"seed_index", seed_i},
            {"loaded_lect_cache", loaded_existing},
            {"build_s", build_s},
            {"planner_build_total_ms", build_result.total_time_ms},
            {"build_grow_time_ms", build_result.grow_time_ms},
            {"build_adjacency_time_ms", build_result.adjacency_time_ms},
            {"build_overhead_ms", std::max(0.0, build_s * 1000.0 - build_result.total_time_ms)},
            {"n_boxes", build_result.n_boxes},
            {"unique_box_count", build_result.unique_box_count},
            {"duplicate_box_count", build_result.duplicate_box_count},
            {"box_volume_sum", build_result.box_volume_sum},
            {"unique_box_volume_sum", build_result.unique_box_volume_sum},
            {"dedup_box_volume_sum", build_result.unique_box_volume_sum},
            {"duplicate_box_volume_sum", build_result.duplicate_box_volume_sum},
            {"n_islands", build_result.n_islands},
            {"build_success", build_result.success},
            {"build_fail_reason", build_result.fail_reason},
            {"queries", nlohmann::json::array()},
        };

        std::cout << "[marcucci-cached] seed=" << seed
                  << " build=" << build_s << "s"
                  << " boxes=" << build_result.n_boxes
                  << " islands=" << build_result.n_islands
                  << (build_result.success ? "" : " BUILD_FAIL") << "\n";

        for (std::size_t qi = 0; qi < scenes.size(); ++qi) {
            const auto& sc = scenes[qi];
            const auto& spec = query_specs[qi];
            const auto t0 = Clock::now();

            bool ok = false;
            double length = 0.0;
            int n_pts = 0;
            std::string fail_reason;
            bool used_point_bridge = false;

            sbf::planner::PlanResult qr;
            if (build_result.success) {
                qr = planner.query(sc.q_start, sc.q_goal, obs, n_obs);
                ok = qr.success;
                length = qr.opt_length;
                n_pts = static_cast<int>(qr.path.size());
                fail_reason = qr.fail_reason;
                used_point_bridge = qr.used_point_bridge;
            } else {
                fail_reason = build_result.fail_reason;
            }

            const double query_s = build_result.success
                ? qr.total_time_ms / 1000.0
                : std::chrono::duration<double>(Clock::now() - t0).count();
            trial_json["queries"].push_back(nlohmann::json{
                {"from", spec.from},
                {"to", spec.to},
                {"t_s", query_s},
                {"ok", ok},
                {"length", length},
                {"n_pts", n_pts},
                {"used_point_bridge", used_point_bridge},
                {"fail_reason", fail_reason},
            });

            if (ok) {
                query_times_s[qi].push_back(query_s);
                query_lengths[qi].push_back(length);
                ++query_successes[qi];
            }

            std::cout << "  " << query_name(spec)
                      << " q=" << query_s << "s"
                      << " " << (ok ? "OK" : "FAIL")
                      << " len=" << length << "\n";
        }

        out["trials"].push_back(trial_json);
        sbf::exp::save_lect_geometry_cache(cache_lect, a.lect_cache_path);
    }

    for (std::size_t qi = 0; qi < scenes.size(); ++qi) {
        out["queries"].push_back({
            {"name", query_name(query_specs[qi])},
            {"sr", a.seeds > 0 ? static_cast<double>(query_successes[qi]) / a.seeds : 0.0},
            {"t_med_s", median(query_times_s[qi])},
            {"len_med", median(query_lengths[qi])},
        });
    }
    out["build"] = {
        {"median_s", median(build_times_s)},
        {"mean_s", mean(build_times_s)},
        {"median_planner_total_ms", median(planner_build_total_ms_samples)},
        {"mean_planner_total_ms", mean(planner_build_total_ms_samples)},
        {"median_grow_ms", median(build_grow_ms_samples)},
        {"mean_grow_ms", mean(build_grow_ms_samples)},
        {"median_adjacency_ms", median(build_adjacency_ms_samples)},
        {"mean_adjacency_ms", mean(build_adjacency_ms_samples)},
        {"median_overhead_ms", median(build_overhead_ms_samples)},
        {"mean_overhead_ms", mean(build_overhead_ms_samples)},
        {"median_n_boxes", median(n_boxes_samples)},
        {"mean_n_boxes", mean(n_boxes_samples)},
        {"median_unique_box_count", median(unique_box_count_samples)},
        {"mean_unique_box_count", mean(unique_box_count_samples)},
        {"median_duplicate_box_count", median(duplicate_box_count_samples)},
        {"mean_duplicate_box_count", mean(duplicate_box_count_samples)},
        {"median_box_volume_sum", median(box_volume_sum_samples)},
        {"mean_box_volume_sum", mean(box_volume_sum_samples)},
        {"median_unique_box_volume_sum", median(unique_box_volume_sum_samples)},
        {"mean_unique_box_volume_sum", mean(unique_box_volume_sum_samples)},
        {"median_dedup_box_volume_sum", median(unique_box_volume_sum_samples)},
        {"mean_dedup_box_volume_sum", mean(unique_box_volume_sum_samples)},
        {"median_duplicate_box_volume_sum", median(duplicate_box_volume_sum_samples)},
        {"mean_duplicate_box_volume_sum", mean(duplicate_box_volume_sum_samples)},
    };

    sbf::exp::write_json(a.out_path, out);
    std::cout << "[marcucci-cached] wrote " << a.out_path << "\n";
    return 0;
}
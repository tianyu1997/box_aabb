/// @file exp_marcucci_combined.cpp
/// @brief Marcucci combined-scene workload runner for the v7 paper.
///
/// This runner keeps one LECT alive across the five canonical Marcucci
/// shelf/bin/table queries of the same seed, so the experiment measures the
/// lifelong envelope-cache reuse that the paper claims.  The box forest itself
/// is rebuilt per query by the current synchronous SbfPlanner API; envelopes
/// materialised by earlier queries are reused by later queries through the
/// shared LECT.
#include "experiments/common.h"

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

struct Args {
    bool        quick          = false;
    int         seeds          = 3;
    int         timeout_s      = 30;
    int         n_threads      = 0;
    int         n_subdivisions = 4;
    double      voxel_delta    = 0.05;
    int         ffb_max_depth  = -1;
    int         max_boxes      = 2500;
    int         bridge_boxes   = 2000;
    bool        point_bridge   = true;
    double      point_bridge_timeout_ms = 20000.0;
    std::string env_type       = "link_iaabb_grid";
    std::string lect_cache_path;
    fs::path    scene_dir;
    fs::path    out_path;

    sbf::envelope::EnvelopeType env_type_enum() const {
        if (env_type == "link_iaabb_grid")
            return sbf::envelope::EnvelopeType::LinkIAABB_Grid;
        if (env_type == "hull16_grid")
            return sbf::envelope::EnvelopeType::Hull16_Grid;
        return sbf::envelope::EnvelopeType::LinkIAABB;
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
        if (s == "--quick") { a.quick = true; a.seeds = 3; a.timeout_s = 30; }
        else if (s == "--full") { a.quick = false; a.seeds = 20; a.timeout_s = 120; }
        else if (auto v = eat_value(s, "scene-dir"); !v.empty()) a.scene_dir = v;
        else if (auto v = eat_value(s, "out"); !v.empty()) a.out_path = v;
        else if (auto v = eat_value(s, "seeds"); !v.empty()) a.seeds = std::stoi(v);
        else if (auto v = eat_value(s, "timeout"); !v.empty()) a.timeout_s = std::stoi(v);
        else if (auto v = eat_value(s, "threads"); !v.empty()) a.n_threads = std::stoi(v);
        else if (auto v = eat_value(s, "env"); !v.empty()) a.env_type = v;
        else if (auto v = eat_value(s, "n-sub"); !v.empty()) a.n_subdivisions = std::stoi(v);
        else if (auto v = eat_value(s, "voxel-delta"); !v.empty()) a.voxel_delta = std::stod(v);
        else if (auto v = eat_value(s, "ffb-depth"); !v.empty()) a.ffb_max_depth = std::stoi(v);
        else if (auto v = eat_value(s, "max-boxes"); !v.empty()) a.max_boxes = std::stoi(v);
        else if (auto v = eat_value(s, "bridge-boxes"); !v.empty()) a.bridge_boxes = std::stoi(v);
        else if (auto v = eat_value(s, "point-bridge-timeout-ms"); !v.empty()) a.point_bridge_timeout_ms = std::stod(v);
        else if (auto v = eat_value(s, "lect-cache"); !v.empty()) a.lect_cache_path = v;
        else if (s == "--point-bridge") a.point_bridge = true;
        else if (s == "--no-point-bridge") a.point_bridge = false;
        else {
            std::cerr << "unknown arg: " << s << "\n";
            std::exit(2);
        }
    }
    if (a.scene_dir.empty() || a.out_path.empty()) {
        std::cerr << "usage: exp_marcucci_combined --scene-dir=<configs/marcucci> "
                     "--out=<json> [--quick|--full] [--seeds=N] [--timeout=SEC] "
                     "[--threads=N] [--env=link_iaabb_grid] [--n-sub=4] "
                     "[--max-boxes=2500] [--bridge-boxes=2000] "
                     "[--point-bridge|--no-point-bridge] "
                     "[--lect-cache=<path.bin>]\n";
        std::exit(2);
    }
    return a;
}

double mean(const std::vector<double>& xs) {
    if (xs.empty()) return 0.0;
    return std::accumulate(xs.begin(), xs.end(), 0.0) / static_cast<double>(xs.size());
}

double median(std::vector<double> xs) {
    if (xs.empty()) return 0.0;
    std::sort(xs.begin(), xs.end());
    const std::size_t n = xs.size();
    return (n % 2) ? xs[n / 2] : 0.5 * (xs[n / 2 - 1] + xs[n / 2]);
}

}  // namespace

int main(int argc, char** argv) {
    const Args a = parse_args(argc, argv);
    const std::vector<std::pair<std::string, std::string>> query_files = {
        {"AS->TS", "AS_TS.json"},
        {"TS->CS", "TS_CS.json"},
        {"CS->LB", "CS_LB.json"},
        {"LB->RB", "LB_RB.json"},
        {"RB->AS", "RB_AS.json"},
    };

    std::vector<sbf::scene::SceneConfig> scenes;
    scenes.reserve(query_files.size());
    for (const auto& [name, file] : query_files) {
        (void)name;
        scenes.push_back(sbf::scene::load_scene_json((a.scene_dir / file).string()));
    }
    if (scenes.empty()) {
        std::cerr << "no Marcucci scene files loaded\n";
        return 2;
    }

    auto robot = sbf::core::Robot::from_json(sbf::exp::robot_json_path(scenes.front().robot));

    nlohmann::json out = {
        {"experiment", "marcucci_combined"},
        {"scene", "iiwa14_marcucci_combined"},
        {"robot", scenes.front().robot},
        {"quick", a.quick},
        {"seeds", a.seeds},
        {"env", a.env_type},
        {"n_subdivisions", a.n_subdivisions},
        {"max_boxes", a.max_boxes},
        {"bridge_boxes", a.bridge_boxes},
        {"point_bridge", a.point_bridge},
        {"lect_cache", a.lect_cache_path.empty()
                            ? nlohmann::json(nullptr)
                            : nlohmann::json(a.lect_cache_path)},
        {"queries", nlohmann::json::array()},
        {"seed_trials", nlohmann::json::array()},
    };
    for (const auto& [name, file] : query_files) {
        out["queries"].push_back({{"name", name}, {"file", file}});
    }

    std::vector<double> all_total_ms;
    std::vector<double> all_grow_ms;
    std::vector<double> all_opt_len;
    std::vector<double> seed_total_ms;
    int n_success = 0;
    int n_total = 0;

    for (int seed = 0; seed < a.seeds; ++seed) {
        bool loaded_existing = false;
        auto lect = sbf::exp::load_or_make_lect(robot, a.env_type_enum(),
                                                a.n_subdivisions, a.voxel_delta,
                                                a.lect_cache_path,
                                                &loaded_existing);
        nlohmann::json seed_json = {
            {"seed", seed},
            {"loaded_lect_cache", loaded_existing},
            {"queries", nlohmann::json::array()},
        };
        double seed_ms = 0.0;
        int seed_success = 0;

        for (std::size_t qi = 0; qi < scenes.size(); ++qi) {
            // Reuse materialised envelopes across the combined workload, but
            // rebuild the forest per query.  Occupancy belongs to one query's
            // forest; leaving it set makes the next query's endpoint FFB fail
            // whenever it lands in a previous start/goal box.
            lect.clear_all_occupation();

            sbf::planner::PlannerConfig cfg;
            cfg.grower.rng_seed   = static_cast<std::uint64_t>(1000 * seed + qi);
            cfg.grower.n_threads  = a.n_threads;
            cfg.grower.max_boxes  = a.max_boxes;
            cfg.grower.endpoint_bridge_max_boxes = a.bridge_boxes;
            cfg.grower.timeout_ms = static_cast<double>(a.timeout_s) * 1000.0;
            cfg.timeout_s         = static_cast<double>(a.timeout_s);
            cfg.quick_mode        = a.quick;
            cfg.point_bridge_fallback = a.point_bridge;
            cfg.point_bridge_timeout_ms = a.point_bridge_timeout_ms;
            if (a.ffb_max_depth > 0) cfg.grower.ffb.max_depth = a.ffb_max_depth;

            sbf::planner::SbfPlanner planner(robot, lect, cfg);
            auto packed = scenes[qi].packed_obstacles();
            auto res = planner.plan(scenes[qi].q_start, scenes[qi].q_goal,
                                    packed.empty() ? nullptr : packed.data(),
                                    static_cast<int>(scenes[qi].obstacles.size()));
            nlohmann::json qj = sbf::exp::plan_to_json(res);
            qj["query"] = query_files[qi].first;
            seed_json["queries"].push_back(qj);

            ++n_total;
            seed_ms += res.total_time_ms;
            if (res.success) {
                ++n_success;
                ++seed_success;
                all_total_ms.push_back(res.total_time_ms);
                all_grow_ms.push_back(res.grow_time_ms);
                all_opt_len.push_back(res.opt_length);
            }
            std::cout << "[marcucci] seed=" << seed
                      << " query=" << query_files[qi].first
                      << " success=" << res.success
                      << " boxes=" << res.n_boxes
                      << " len=" << res.opt_length
                      << " t_ms=" << res.total_time_ms
                      << "\n";
        }
        seed_json["n_success"] = seed_success;
        seed_json["total_time_ms"] = seed_ms;
        out["seed_trials"].push_back(seed_json);
        seed_total_ms.push_back(seed_ms);
        sbf::exp::save_lect_geometry_cache(lect, a.lect_cache_path);
    }

    out["summary"] = {
        {"n_total", n_total},
        {"n_success", n_success},
        {"success_rate", n_total ? static_cast<double>(n_success) / n_total : 0.0},
        {"avg_query_total_ms_success", mean(all_total_ms)},
        {"median_query_total_ms_success", median(all_total_ms)},
        {"avg_query_grow_ms_success", mean(all_grow_ms)},
        {"median_query_grow_ms_success", median(all_grow_ms)},
        {"avg_opt_length_success", mean(all_opt_len)},
        {"median_opt_length_success", median(all_opt_len)},
        {"median_workload_time_ms", median(seed_total_ms)},
    };

    sbf::exp::write_json(a.out_path, out);
    std::cout << "[marcucci] SR=" << out["summary"]["success_rate"]
              << " (" << n_success << "/" << n_total << ") wrote "
              << a.out_path << "\n";
    return 0;
}

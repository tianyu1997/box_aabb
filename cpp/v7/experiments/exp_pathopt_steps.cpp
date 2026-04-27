/// @file exp_pathopt_steps.cpp
/// @brief Ablation of the 5-step path optimisation pipeline.
///
/// For each step combo, runs N trials and reports
/// avg_opt_length / avg_opt_time_ms vs raw Dijkstra length.
///
/// Combos exercised:
///   - "raw"            : only Dijkstra (no opt)
///   - "greedy"         : GREEDY_SHORTCUT
///   - "greedy+final"   : GREEDY_SHORTCUT + SHORTCUT_FINAL  (== quick_mode)
///   - "full"           : default 5-step pipeline
#include "experiments/common.h"

#include "sbf/planner/path_opt_pipeline.h"

namespace {

using sbf::planner::PathOptStep;

std::vector<PathOptStep> combo_for(const std::string& name) {
    if (name == "raw")          return {};
    if (name == "greedy")       return {PathOptStep::GREEDY_SHORTCUT};
    if (name == "greedy+final") return {PathOptStep::GREEDY_SHORTCUT,
                                        PathOptStep::SHORTCUT_FINAL};
    /* full */                  return {PathOptStep::GREEDY_SHORTCUT,
                                        PathOptStep::SHORTCUT,
                                        PathOptStep::DENSIFY,
                                        PathOptStep::ELASTIC_BAND,
                                        PathOptStep::SHORTCUT_FINAL};
}

sbf::planner::PlanResult run_with_combo(const sbf::scene::SceneConfig& sc,
                                        uint64_t                       seed,
                                        int                            n_threads,
                                        int                            timeout_s,
                                        bool                           quick,
                                        const std::string&             combo_name,
                                        const sbf::exp::CliArgs&       args) {
    auto robot = sbf::core::Robot::from_json(sbf::exp::robot_json_path(sc.robot));
    auto lect  = sbf::exp::load_or_make_lect(robot,
                                             args.env_type_enum(),
                                             args.n_subdivisions,
                                             args.voxel_delta,
                                             args.lect_cache_path);

    sbf::planner::PlannerConfig cfg;
    cfg.grower.rng_seed   = seed;
    cfg.grower.n_threads  = n_threads;
    cfg.grower.max_boxes  = quick ? 200 : 800;
    cfg.grower.timeout_ms = static_cast<double>(timeout_s) * 1000.0;
    if (robot.n_joints() <= 3) cfg.grower.ffb.max_depth = 8;
    cfg.timeout_s         = static_cast<double>(timeout_s);
    cfg.quick_mode        = false;            // override pipeline manually
    cfg.path_opt.steps          = combo_for(combo_name);
    cfg.path_opt.shortcut_iters = quick ? 30 : 100;
    cfg.path_opt.elastic_iters  = quick ? 10 : 20;

    sbf::planner::SbfPlanner planner(robot, lect, cfg);
    auto packed = sc.packed_obstacles();
    auto result = planner.plan(sc.q_start, sc.q_goal,
                               packed.empty() ? nullptr : packed.data(),
                               static_cast<int>(sc.obstacles.size()));
    sbf::exp::save_lect_geometry_cache(lect, args.lect_cache_path);
    return result;
}

}  // namespace

int main(int argc, char** argv) {
    auto a  = sbf::exp::CliArgs::parse(argc, argv);
    auto sc = sbf::scene::load_scene_json(a.scene_path);

    std::vector<std::string> combos = {"raw", "greedy", "greedy+final", "full"};

    nlohmann::json out = {
        {"experiment", "pathopt_steps"},
        {"scene",      sc.name},
        {"robot",      sc.robot},
        {"quick",      a.quick},
        {"seeds",      a.seeds},
        {"lect_cache", a.lect_cache_path.empty()
                            ? nlohmann::json(nullptr)
                            : nlohmann::json(a.lect_cache_path)},
        {"results",    nlohmann::json::array()},
    };

    for (const auto& combo : combos) {
        int n_success = 0;
        double sum_opt = 0.0, sum_raw = 0.0, sum_opt_ms = 0.0;
        for (int s = 0; s < a.seeds; ++s) {
            auto r = run_with_combo(sc, static_cast<uint64_t>(42 + s),
                                    a.n_threads, a.timeout_s, a.quick, combo, a);
            if (r.success) {
                ++n_success;
                sum_opt    += r.opt_length;
                sum_raw    += r.raw_length;
                sum_opt_ms += r.opt_time_ms;
            }
        }
        out["results"].push_back({
            {"combo",            combo},
            {"n_success",        n_success},
            {"avg_opt_length",   n_success ? sum_opt    / n_success : 0.0},
            {"avg_raw_length",   n_success ? sum_raw    / n_success : 0.0},
            {"avg_opt_time_ms",  n_success ? sum_opt_ms / n_success : 0.0},
            {"length_ratio",     n_success && sum_raw > 0.0
                                     ? sum_opt / sum_raw : 1.0},
        });
        std::cout << "[pathopt] combo=" << combo
                  << " sr=" << n_success << "/" << a.seeds
                  << " avg_opt_len=" << (n_success ? sum_opt / n_success : 0.0)
                  << "\n";
    }
    if (!a.out_path.empty()) sbf::exp::write_json(a.out_path, out);
    return 0;
}

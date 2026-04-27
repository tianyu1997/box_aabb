/// @file exp_main.cpp
/// @brief Main success-rate / path-quality experiment.
///
/// Runs N trials (seeds 0..N-1) of SbfPlanner on a single scene and
/// emits a JSON summary suitable for `build_tables.py`.
#include "experiments/common.h"

int main(int argc, char** argv) {
    auto a  = sbf::exp::CliArgs::parse(argc, argv);
    auto sc = sbf::scene::load_scene_json(a.scene_path);

    nlohmann::json out = {
        {"experiment", "main"},
        {"scene",      sc.name},
        {"robot",      sc.robot},
        {"quick",      a.quick},
        {"seeds",      a.seeds},
        {"lect_cache", a.lect_cache_path.empty()
                            ? nlohmann::json(nullptr)
                            : nlohmann::json(a.lect_cache_path)},
        {"trials",     nlohmann::json::array()},
    };

    int    n_success = 0;
    double sum_total = 0.0, sum_opt = 0.0, sum_raw = 0.0;
    for (int s = 0; s < a.seeds; ++s) {
        auto r = sbf::exp::run_trial(sc, static_cast<uint64_t>(42 + s),
                                     a.n_threads, a.timeout_s, a.quick, a);
        out["trials"].push_back(sbf::exp::plan_to_json(r));
        if (r.success) {
            ++n_success;
            sum_total += r.total_time_ms;
            sum_opt   += r.opt_length;
            sum_raw   += r.raw_length;
        }
        std::cout << "[main] seed=" << s
                  << " success=" << r.success
                  << " boxes=" << r.n_boxes
                  << " opt_len=" << r.opt_length
                  << " t_ms=" << r.total_time_ms << "\n";
    }
    double sr = static_cast<double>(n_success) / std::max(1, a.seeds);
    out["summary"] = {
        {"success_rate",     sr},
        {"n_success",        n_success},
        {"avg_total_time_ms", n_success ? sum_total / n_success : 0.0},
        {"avg_opt_length",    n_success ? sum_opt   / n_success : 0.0},
        {"avg_raw_length",    n_success ? sum_raw   / n_success : 0.0},
    };
    if (!a.out_path.empty()) sbf::exp::write_json(a.out_path, out);
    std::cout << "[main] SR=" << sr << " (" << n_success << "/" << a.seeds
              << ")\n";
    return 0;
}

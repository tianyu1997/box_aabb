/// @file exp_threads.cpp
/// @brief Thread-scaling experiment for the parallel grower.
///
/// Sweeps n_threads ∈ {1,2,4,8} and reports avg total_time_ms + speedup
/// against the 1-thread baseline (per scene).
#include "experiments/common.h"

int main(int argc, char** argv) {
    auto a  = sbf::exp::CliArgs::parse(argc, argv);
    auto sc = sbf::scene::load_scene_json(a.scene_path);

    std::vector<int> threads_sweep =
        a.quick ? std::vector<int>{1, 2, 4} : std::vector<int>{1, 2, 4, 8};

    nlohmann::json out = {
        {"experiment", "threads"},
        {"scene",      sc.name},
        {"robot",      sc.robot},
        {"quick",      a.quick},
        {"seeds",      a.seeds},
        {"lect_cache", a.lect_cache_path.empty()
                            ? nlohmann::json(nullptr)
                            : nlohmann::json(a.lect_cache_path)},
        {"results",    nlohmann::json::array()},
    };

    double baseline_avg = 0.0;
    for (int nt : threads_sweep) {
        int    n_success = 0;
        double sum_total = 0.0;
        nlohmann::json trials = nlohmann::json::array();
        for (int s = 0; s < a.seeds; ++s) {
            auto r = sbf::exp::run_trial(sc, static_cast<uint64_t>(42 + s),
                                         nt, a.timeout_s, a.quick, a);
            trials.push_back(sbf::exp::plan_to_json(r));
            if (r.success) { ++n_success; sum_total += r.total_time_ms; }
        }
        double avg = n_success ? sum_total / n_success : 0.0;
        if (nt == 1) baseline_avg = avg;
        double speedup = (avg > 0.0 && baseline_avg > 0.0) ? baseline_avg / avg : 0.0;
        out["results"].push_back({
            {"n_threads",         nt},
            {"n_success",         n_success},
            {"avg_total_time_ms", avg},
            {"speedup_vs_1t",     speedup},
            {"trials",            trials},
        });
        std::cout << "[threads] nt=" << nt
                  << " sr=" << n_success << "/" << a.seeds
                  << " avg_ms=" << avg
                  << " speedup=" << speedup << "\n";
    }
    if (!a.out_path.empty()) sbf::exp::write_json(a.out_path, out);
    return 0;
}

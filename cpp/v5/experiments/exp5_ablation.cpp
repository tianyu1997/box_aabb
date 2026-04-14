/**
 * exp5_ablation.cpp — 实验 5: Optimisation Ablation Study
 *
 * 对应论文 Tab 4 (Optimisation Effectiveness).
 *
 * 系统性地启用/禁用以下优化, 测量 build_time, query_time, path_length:
 *   P0: Multi-trial RRT competition     (query phase)
 *   P2: Parallel bridge                  (build phase)
 *   P4: Connect-stop                     (build phase)
 *
 * 基线: 所有优化关闭 (顺序桥接, 无 connect-stop, 单次 RRT)
 *
 * 用法:
 *   ./exp5_ablation [--seeds N] [--threads N] [--quick]
 */

#include <sbf/planner/sbf_planner.h>
#include <sbf/scene/collision_checker.h>
#include <sbf/core/robot.h>
#include "marcucci_scenes.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

using namespace sbf;

// ═══════════════════════════════════════════════════════════════════════════

struct Stats { double median, mean, q25, q75; };

Stats compute_stats(std::vector<double>& d) {
    Stats s{};
    if (d.empty()) return s;
    std::sort(d.begin(), d.end());
    int n = static_cast<int>(d.size());
    s.median = d[n / 2]; s.q25 = d[n / 4]; s.q75 = d[3 * n / 4];
    s.mean = std::accumulate(d.begin(), d.end(), 0.0) / n;
    return s;
}

double path_length(const std::vector<Eigen::VectorXd>& path) {
    double len = 0;
    for (size_t i = 1; i < path.size(); ++i)
        len += (path[i] - path[i - 1]).norm();
    return len;
}

// ═══════════════════════════════════════════════════════════════════════════
// Ablation configuration
// ═══════════════════════════════════════════════════════════════════════════

struct AblationConfig {
    std::string name;
    bool P0_multi_trial_rrt;    // Multi-trial RRT competition in query
    bool P2_parallel_bridge;    // Multi-threaded bridging
    bool P4_connect_stop;       // Connect-mode growth termination
};

SBFPlannerConfig make_planner_config(const AblationConfig& ab, int n_threads,
                                      uint64_t seed) {
    SBFPlannerConfig cfg;
    cfg.z4_enabled = true;
    cfg.split_order = SplitOrder::BEST_TIGHTEN;
    cfg.lect_no_cache = true;  // Disable LECT disk cache to avoid 10+ second lect_save

    cfg.grower.mode = GrowerConfig::Mode::RRT;
    cfg.grower.max_boxes = 200000;
    cfg.grower.n_threads = 5;
    cfg.grower.rng_seed = seed;
    cfg.grower.max_consecutive_miss = 2000;
    cfg.grower.rrt_goal_bias = 0.8;
    cfg.grower.rrt_step_ratio = 0.05;
    cfg.grower.enable_promotion = true;
    cfg.grower.ffb_config.max_depth = 300;

    // P4: Connect-stop
    if (ab.P4_connect_stop) {
        cfg.grower.connect_mode = true;
        cfg.grower.post_connect_extra_boxes = 4000;
        cfg.grower.timeout_ms = 60000.0;
    } else {
        cfg.grower.connect_mode = false;
        cfg.grower.post_connect_extra_boxes = 0;
        cfg.grower.max_boxes = 3000;      // Cap box count without connect-stop
        cfg.grower.timeout_ms = 3000.0;   // 3s budget
    }

    // P2: Parallel bridge
    if (ab.P2_parallel_bridge) {
        cfg.grower.bridge_n_threads = n_threads;
    } else {
        cfg.grower.bridge_n_threads = 1;
    }

    cfg.coarsen.target_boxes = 300;
    cfg.coarsen.max_rounds = 100;
    cfg.coarsen.max_lect_fk_per_round = 10000;
    cfg.coarsen.score_threshold = 500.0;

    // P0: Multi-trial RRT — controlled by shortcut iterations
    // With P0=OFF, limit smoother to minimal processing (single-pass, no competition)
    cfg.smoother.shortcut_max_iters = ab.P0_multi_trial_rrt ? 100 : 0;
    cfg.smoother.smooth_window = 3;
    cfg.smoother.smooth_iters = ab.P0_multi_trial_rrt ? 5 : 0;

    return cfg;
}

// ═══════════════════════════════════════════════════════════════════════════
// Main
// ═══════════════════════════════════════════════════════════════════════════

int main(int argc, char** argv) {
    std::string robot_path = std::string(SBF_DATA_DIR) + "/iiwa14.json";
    int n_seeds = 5;
    int n_threads = static_cast<int>(std::thread::hardware_concurrency());
    bool quick = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--seeds" && i + 1 < argc) n_seeds = std::atoi(argv[++i]);
        else if (a == "--threads" && i + 1 < argc) n_threads = std::atoi(argv[++i]);
        else if (a == "--quick") quick = true;
        else if (a[0] != '-') robot_path = a;
    }

    if (quick) { n_seeds = 2; }
    if (n_threads < 1) n_threads = 1;

    Robot robot = Robot::from_json(robot_path);
    auto obstacles = make_combined_obstacles();
    auto queries = make_combined_queries();
    int n_obs = static_cast<int>(obstacles.size());
    int n_pairs = static_cast<int>(queries.size());

    std::cout << "Robot: " << robot.name() << "  DOF=" << robot.n_joints() << "\n"
              << "Scene: combined (" << n_obs << " obs, " << n_pairs << " queries)\n"
              << "Seeds=" << n_seeds << "  Threads=" << n_threads << "\n\n";

    // Seed points from query endpoints
    std::vector<Eigen::VectorXd> seed_points;
    {
        auto approx_eq = [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
            return (a - b).squaredNorm() < 1e-8;
        };
        auto maybe_add = [&](const Eigen::VectorXd& q) {
            for (const auto& s : seed_points)
                if (approx_eq(s, q)) return;
            seed_points.push_back(q);
        };
        for (const auto& qp : queries) {
            maybe_add(qp.start);
            maybe_add(qp.goal);
        }
    }

    // Ablation configurations
    std::vector<AblationConfig> configs = {
        {"Baseline (all OFF)",  false, false, false},
        {"+P0 (multi-RRT)",    true,  false, false},
        {"+P2 (parallel brg)", false, true,  false},
        {"+P4 (connect-stop)", false, false, true},
        {"+P0+P2",             true,  true,  false},
        {"+P2+P4",             false, true,  true},
        {"ALL ON (P0+P2+P4)",  true,  true,  true},
    };

    std::cout << std::string(120, '=') << "\n"
              << "  Optimisation Ablation Study (Paper Tab 4)\n"
              << std::string(120, '=') << "\n\n";

    // Header
    std::cout << std::left << std::setw(25) << "Config"
              << std::right
              << std::setw(10) << "P0" << std::setw(10) << "P2" << std::setw(10) << "P4"
              << std::setw(12) << "Build(s)"
              << std::setw(10) << "Boxes"
              << std::setw(12) << "Query(s)"
              << std::setw(10) << "Len"
              << std::setw(8)  << "SR%"
              << "\n" << std::string(120, '-') << "\n";

    for (auto& ab : configs) {
        std::vector<double> build_times;
        std::vector<double> query_times, path_lens;
        int n_success = 0, n_total = 0;
        std::vector<int> box_counts;

        for (int seed = 0; seed < n_seeds; ++seed) {
            auto cfg = make_planner_config(ab, n_threads, static_cast<uint64_t>(seed));
            SBFPlanner planner(robot, cfg);

            auto t_build0 = std::chrono::steady_clock::now();
            planner.build_coverage(obstacles.data(), n_obs, cfg.grower.timeout_ms,
                                   seed_points);
            double build_t = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t_build0).count();
            build_times.push_back(build_t);
            box_counts.push_back(planner.n_boxes());

            for (int pi = 0; pi < n_pairs; ++pi) {
                auto& qp = queries[pi];
                auto t0 = std::chrono::steady_clock::now();
                auto res = planner.query(qp.start, qp.goal, obstacles.data(), n_obs);
                double qt = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - t0).count();

                n_total++;
                if (res.success) {
                    n_success++;
                    query_times.push_back(qt);
                    path_lens.push_back(res.path_length);
                }
            }
        }

        auto sb = compute_stats(build_times);
        auto sq = compute_stats(query_times);
        auto sl = compute_stats(path_lens);
        double sr = n_total > 0 ? 100.0 * n_success / n_total : 0.0;
        std::sort(box_counts.begin(), box_counts.end());
        int med_boxes = box_counts.empty() ? 0 : box_counts[box_counts.size() / 2];

        std::cout << std::left << std::setw(25) << ab.name
                  << std::right
                  << std::setw(10) << (ab.P0_multi_trial_rrt ? "ON" : "off")
                  << std::setw(10) << (ab.P2_parallel_bridge ? "ON" : "off")
                  << std::setw(10) << (ab.P4_connect_stop ? "ON" : "off")
                  << std::fixed
                  << std::setw(12) << std::setprecision(2) << sb.median
                  << std::setw(10) << med_boxes
                  << std::setw(12) << std::setprecision(3) << sq.median
                  << std::setw(10) << std::setprecision(3) << sl.median
                  << std::setw(8) << std::setprecision(0) << sr
                  << "\n";
    }

    std::cout << "\n" << std::string(120, '=') << "\n"
              << "  P0=Multi-trial RRT competition, P2=Parallel bridging, P4=Connect-stop\n"
              << std::string(120, '=') << "\n";

    std::cout << "\n  Exp 5 complete.\n";
    return 0;
}

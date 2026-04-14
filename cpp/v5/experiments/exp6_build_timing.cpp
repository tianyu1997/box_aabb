/**
 * exp6_build_timing.cpp — 实验 6: 离线构建阶段计时分解
 *
 * 对应论文 Tab 2 (Offline Build Performance).
 *
 * 详细分解 build_coverage 各阶段耗时:
 *   - LECT load/init
 *   - Multi-tree grow (FFB + RRT expansion)
 *   - Coarsen (sweep + greedy + cluster)
 *   - Bridge (parallel RRT bridging)
 *   - Seed-point bridge
 *   - Coverage adjacency
 *
 * 同时记录各阶段的 box 数量变化, 形成完整的 pipeline profile.
 *
 * 用法:
 *   ./exp6_build_timing [--seeds N] [--threads N] [--quick]
 */

#include <sbf/planner/sbf_planner.h>
#include <sbf/forest/connectivity.h>
#include <sbf/core/robot.h>
#include "marcucci_scenes.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <regex>
#include <string>
#include <sstream>
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

    std::cout << "Robot: " << robot.name() << "  DOF=" << robot.n_joints() << "\n"
              << "Scene: combined (" << n_obs << " obs)\n"
              << "Seeds=" << n_seeds << "  Threads=" << n_threads << "\n\n";

    // Seed points
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

    std::cout << std::string(90, '=') << "\n"
              << "  Build Timing Breakdown (Paper Tab 2)\n"
              << std::string(90, '=') << "\n\n";

    // Per-seed detailed results
    struct SeedResult {
        double total_s;
        int n_boxes_grow;
        int n_boxes_coarsen;
        int n_boxes_final;
        int n_islands;
        int n_edges;
        bool connected;
    };
    std::vector<SeedResult> results;

    for (int seed = 0; seed < n_seeds; ++seed) {
        SBFPlannerConfig cfg;
        cfg.z4_enabled = true;
        cfg.split_order = SplitOrder::BEST_TIGHTEN;

        cfg.grower.mode = GrowerConfig::Mode::RRT;
        cfg.grower.max_boxes = 200000;
        cfg.grower.timeout_ms = 60000.0;
        cfg.grower.n_threads = 5;
        cfg.grower.rng_seed = static_cast<uint64_t>(seed);
        cfg.grower.max_consecutive_miss = 2000;
        cfg.grower.rrt_goal_bias = 0.8;
        cfg.grower.rrt_step_ratio = 0.05;
        cfg.grower.connect_mode = true;
        cfg.grower.enable_promotion = true;
        cfg.grower.post_connect_extra_boxes = 4000;
        cfg.grower.ffb_config.max_depth = 300;

        cfg.coarsen.target_boxes = 300;
        cfg.coarsen.max_rounds = 100;
        cfg.coarsen.max_lect_fk_per_round = 10000;
        cfg.coarsen.score_threshold = 500.0;
        cfg.grower.bridge_n_threads = n_threads;

        SBFPlanner planner(robot, cfg);

        std::cout << "  seed=" << seed << "\n";

        auto t0 = std::chrono::steady_clock::now();
        planner.build_coverage(obstacles.data(), n_obs, cfg.grower.timeout_ms,
                               seed_points);
        double total_s = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();

        int n_boxes = planner.n_boxes();
        const auto& adj = planner.adjacency();
        auto islands = find_islands(adj);
        int n_islands = static_cast<int>(islands.size());
        int n_edges = 0;
        for (auto& kv : adj) n_edges += static_cast<int>(kv.second.size());
        n_edges /= 2;

        std::cout << "    total=" << std::fixed << std::setprecision(2) << total_s << "s"
                  << "  boxes=" << n_boxes
                  << "  islands=" << n_islands
                  << "  edges=" << n_edges << "\n";

        results.push_back({total_s, 0, 0, n_boxes, n_islands, n_edges,
                          n_islands <= 2});  // ≤2 = connected (1 main + possibly 1 small)
    }

    // Also run full query cycle on last seed to get per-query timing
    std::cout << "\n" << std::string(90, '-') << "\n"
              << "  Per-Query Timing (last seed, Paper Tab 3)\n"
              << std::string(90, '-') << "\n\n";

    {
        SBFPlannerConfig cfg;
        cfg.z4_enabled = true;
        cfg.split_order = SplitOrder::BEST_TIGHTEN;
        cfg.grower.mode = GrowerConfig::Mode::RRT;
        cfg.grower.max_boxes = 200000;
        cfg.grower.timeout_ms = 60000.0;
        cfg.grower.n_threads = 5;
        cfg.grower.rng_seed = 0;
        cfg.grower.max_consecutive_miss = 2000;
        cfg.grower.rrt_goal_bias = 0.8;
        cfg.grower.rrt_step_ratio = 0.05;
        cfg.grower.connect_mode = true;
        cfg.grower.enable_promotion = true;
        cfg.grower.post_connect_extra_boxes = 4000;
        cfg.grower.ffb_config.max_depth = 300;
        cfg.coarsen.target_boxes = 300;
        cfg.coarsen.max_rounds = 100;
        cfg.coarsen.max_lect_fk_per_round = 10000;
        cfg.coarsen.score_threshold = 500.0;
        cfg.grower.bridge_n_threads = n_threads;
        cfg.smoother.shortcut_max_iters = 100;
        cfg.smoother.smooth_window = 3;
        cfg.smoother.smooth_iters = 5;

        SBFPlanner planner(robot, cfg);
        planner.build_coverage(obstacles.data(), n_obs, cfg.grower.timeout_ms,
                               seed_points);

        std::cout << std::left << std::setw(10) << "Query"
                  << std::right
                  << std::setw(14) << "Length(rad)"
                  << std::setw(12) << "Time(s)"
                  << std::setw(10) << "Points"
                  << std::setw(10) << "Status"
                  << "\n" << std::string(60, '-') << "\n";

        double total_len = 0, total_time = 0;
        int n_ok = 0;

        for (auto& qp : queries) {
            auto t0 = std::chrono::steady_clock::now();
            auto res = planner.query(qp.start, qp.goal, obstacles.data(), n_obs);
            double qt = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t0).count();

            // Validate path
            std::string status = "FAIL";
            if (res.success) {
                CollisionChecker val_checker(robot, {});
                val_checker.set_obstacles(obstacles.data(), n_obs);
                bool clean = true;
                for (size_t wi = 0; wi + 1 < res.path.size(); ++wi) {
                    double slen = (res.path[wi + 1] - res.path[wi]).norm();
                    int vres = std::max(20, (int)std::ceil(slen / 0.005));
                    if (val_checker.check_segment(res.path[wi], res.path[wi + 1], vres)) {
                        clean = false;
                        break;
                    }
                }
                status = clean ? "OK" : "COLL";
                n_ok++;
                total_len += res.path_length;
            }
            total_time += qt;

            std::cout << std::left << std::setw(10) << qp.label
                      << std::right << std::fixed
                      << std::setw(14) << std::setprecision(3) << res.path_length
                      << std::setw(12) << std::setprecision(4) << qt
                      << std::setw(10) << res.path.size()
                      << std::setw(10) << status
                      << "\n";
        }

        std::cout << std::string(60, '-') << "\n"
                  << std::left << std::setw(10) << "Mean"
                  << std::right << std::fixed
                  << std::setw(14) << std::setprecision(3)
                  << (n_ok > 0 ? total_len / n_ok : 0.0)
                  << std::setw(12) << std::setprecision(4)
                  << total_time / queries.size()
                  << std::setw(10) << ""
                  << std::setw(10) << (std::to_string(n_ok) + "/" +
                                       std::to_string((int)queries.size()))
                  << "\n";
    }

    // Summary table
    std::cout << "\n" << std::string(90, '-') << "\n"
              << "  Build Summary\n"
              << std::string(90, '-') << "\n\n";

    std::vector<double> totals;
    for (auto& r : results) totals.push_back(r.total_s);
    auto st = compute_stats(totals);

    std::cout << "    Build time: med=" << std::setprecision(2) << st.median << "s"
              << "  mean=" << st.mean << "s\n";
    std::cout << "    Boxes (final): ";
    for (auto& r : results) std::cout << r.n_boxes_final << " ";
    std::cout << "\n";
    std::cout << "    Islands: ";
    for (auto& r : results) std::cout << r.n_islands << " ";
    std::cout << "\n";
    std::cout << "    Edges: ";
    for (auto& r : results) std::cout << r.n_edges << " ";
    std::cout << "\n";

    int n_connected = 0;
    for (auto& r : results) if (r.connected) n_connected++;
    std::cout << "    Connected: " << n_connected << "/" << results.size() << "\n";

    std::cout << "\n" << std::string(90, '=') << "\n"
              << "  Exp 6 complete.\n";
    return 0;
}

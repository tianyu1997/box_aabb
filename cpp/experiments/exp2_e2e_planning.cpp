/**
 * exp2_e2e_planning.cpp — 实验 2: 端到端 GCS 规划性能
 *
 * 设计:
 *   1 个合并场景 (16 obstacles)
 *   5 个 s-t pairs: AS→TS, TS→CS, CS→LB, LB→RB, RB→AS
 *   每个 seed: build_multi 一次 → 对每个 pair 做 query → 统计
 *   测量: build_time, query_time, path_length, success_rate
 *   GCS 求解在 Python 侧 (LinearGCS + MosekSolver), C++ 仅 SBF query
 *
 * 用法:
 *   ./exp2_e2e_planning [robot.json] [--seeds N] [--max-boxes N] [--quick]
 */

#include "sbf/planner/sbf_planner.h"
#include "sbf/planner/pipeline.h"
#include "sbf/io/json_io.h"
#include "marcucci_scenes.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

using namespace sbf;

// ═══════════════════════════════════════════════════════════════════════════
// Path quality
// ═══════════════════════════════════════════════════════════════════════════

double path_length(const Eigen::MatrixXd& path) {
    double len = 0;
    for (int i = 1; i < path.rows(); ++i)
        len += (path.row(i) - path.row(i - 1)).norm();
    return len;
}

double path_smoothness(const Eigen::MatrixXd& path) {
    if (path.rows() < 3) return 0;
    double cost = 0;
    for (int i = 1; i < path.rows() - 1; ++i) {
        auto a = path.row(i + 1) - 2.0 * path.row(i) + path.row(i - 1);
        cost += a.norm();
    }
    return cost;
}

// ═══════════════════════════════════════════════════════════════════════════
// Statistics
// ═══════════════════════════════════════════════════════════════════════════

struct Stats { double median, q25, q75, mean; };

Stats compute_stats(std::vector<double>& d) {
    Stats s{};
    if (d.empty()) return s;
    std::sort(d.begin(), d.end());
    int n = (int)d.size();
    s.median = d[n/2]; s.q25 = d[n/4]; s.q75 = d[3*n/4];
    s.mean = std::accumulate(d.begin(), d.end(), 0.0) / n;
    return s;
}

// ═══════════════════════════════════════════════════════════════════════════
// Main
// ═══════════════════════════════════════════════════════════════════════════

int main(int argc, char** argv) {
    std::string robot_path = "configs/iiwa14.json";
    int n_seeds = 10;
    int max_boxes = 30000;   // R1: 5×5000 + R2: +5×5000 (if needed) + 5000 random
    int n_random = 5000;
    double timeout = 120.0;
    bool quick = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--seeds" && i+1 < argc) n_seeds = std::atoi(argv[++i]);
        else if (a == "--timeout" && i+1 < argc) timeout = std::atof(argv[++i]);
        else if (a == "--max-boxes" && i+1 < argc) max_boxes = std::atoi(argv[++i]);
        else if (a == "--random" && i+1 < argc) n_random = std::atoi(argv[++i]);
        else if (a == "--quick") quick = true;
        else if (a[0] != '-') robot_path = a;
    }

    if (quick) { n_seeds = 2; max_boxes = 5000; n_random = 1000; timeout = 60.0; }

    Robot robot = Robot::from_json(robot_path);
    std::cout << "Robot: " << robot.name() << "  DOF=" << robot.n_joints()
              << "  active_links=" << robot.n_active_links() << "\n\n";

    auto obstacles = make_combined_obstacles();
    auto queries = make_combined_queries();

    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> pairs;
    for (auto& qp : queries)
        pairs.push_back({qp.start, qp.goal});

    std::cout << "Scene: combined  (" << obstacles.size() << " obstacles, "
              << pairs.size() << " pairs)\n"
              << "  max_boxes=" << max_boxes
              << "  random=" << n_random
              << "  seeds=" << n_seeds << "\n"
              << std::string(72, '=') << "\n";

    std::cout << std::fixed;

    std::vector<double> all_build;
    std::vector<std::vector<double>> per_pair_query(queries.size());
    std::vector<std::vector<double>> per_pair_len(queries.size());
    std::vector<int> per_pair_success(queries.size(), 0);

    for (int seed = 0; seed < n_seeds; ++seed) {
        // Build forest once for all pairs
        SBFConfig cfg = make_panda_config(seed);
        cfg.max_boxes = max_boxes;
        cfg.max_consecutive_miss = 500;

        auto t_build0 = std::chrono::steady_clock::now();
        SBFPlanner planner(robot, obstacles, cfg);
        planner.build_multi(pairs, n_random, 300.0);
        double build_t = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t_build0).count();
        all_build.push_back(build_t);

        std::cout << "\n  seed=" << seed
                  << "  build=" << std::setprecision(2) << build_t << "s"
                  << "  boxes=" << planner.forest().n_boxes() << "\n";

        // Query each pair
        for (int pi = 0; pi < static_cast<int>(queries.size()); ++pi) {
            auto& qp = queries[pi];

            auto t0 = std::chrono::steady_clock::now();
            auto res = planner.query(qp.start, qp.goal, timeout);
            double qt = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t0).count();

            double plen = res.success ? path_length(res.path) : 0;

            if (res.success) {
                per_pair_query[pi].push_back(qt);
                per_pair_len[pi].push_back(plen);
                per_pair_success[pi]++;
            }

            std::cout << "    " << qp.label
                      << "  q=" << std::setprecision(4) << qt << "s"
                      << "  " << (res.success ? "OK  " : "FAIL")
                      << "  len=" << std::setprecision(3) << plen << "\n";
        }
    }

    // Summary
    std::cout << "\n" << std::string(72, '=')
              << "\n  Summary (SBF Dijkstra query)\n"
              << std::string(72, '=') << "\n";

    auto sb = compute_stats(all_build);
    std::cout << "  Build:   med=" << std::setprecision(3) << sb.median << "s"
              << "  mean=" << sb.mean << "s\n\n";

    for (int pi = 0; pi < static_cast<int>(queries.size()); ++pi) {
        double sr = n_seeds > 0 ? 100.0 * per_pair_success[pi] / n_seeds : 0;
        std::cout << "  " << queries[pi].label
                  << "  SR=" << std::setprecision(1) << sr << "%";
        if (!per_pair_query[pi].empty()) {
            auto sq = compute_stats(per_pair_query[pi]);
            auto sl = compute_stats(per_pair_len[pi]);
            std::cout << "  query_med=" << std::setprecision(4) << sq.median << "s"
                      << "  len_med=" << std::setprecision(3) << sl.median;
        }
        std::cout << "\n";
    }

    std::cout << "\n  Exp 2 complete.\n";
    return 0;
}

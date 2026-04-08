/**
 * exp3_incremental.cpp — 实验 3: 增量重建时间
 *
 * 设计:
 *   合并场景 (shelves + bins + table = 16 obstacles)
 *   初始 build_multi → ±2cm 扰动 (移动 bin/shelves 位置)
 *   仅统计增量重建时间:
 *     invalidate_against_obstacle + 重新 grow → rebuild adjacency
 *   不做全量 rebuild 对比, 不做规划
 *
 * 扰动方式: 对每个 obstacle 的 center 施加 ±2cm 随机偏移
 *
 * 用法:
 *   ./exp3_incremental [robot.json] [--seeds N] [--trials N] [--quick]
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
#include <random>
#include <string>
#include <vector>

using namespace sbf;

// ═══════════════════════════════════════════════════════════════════════════
// Perturbation: ±2cm offsets to obstacle centers
// ═══════════════════════════════════════════════════════════════════════════

std::vector<Obstacle> perturb_obstacles(
    const std::vector<Obstacle>& base,
    double delta, std::mt19937& rng)
{
    std::uniform_real_distribution<double> ud(-delta, delta);
    auto perturbed = base;
    for (auto& o : perturbed) {
        o.center[0] += ud(rng);
        o.center[1] += ud(rng);
        o.center[2] += ud(rng);
    }
    return perturbed;
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
    int n_trials = 5;
    int max_boxes = 30000;   // R1: 5×5000 + R2: +5×5000 (if needed) + 5000 random
    int n_random = 5000;
    double delta = 0.02;    // ±2cm
    bool quick = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--seeds" && i+1 < argc) n_seeds = std::atoi(argv[++i]);
        else if (a == "--trials" && i+1 < argc) n_trials = std::atoi(argv[++i]);
        else if (a == "--max-boxes" && i+1 < argc) max_boxes = std::atoi(argv[++i]);
        else if (a == "--random" && i+1 < argc) n_random = std::atoi(argv[++i]);
        else if (a == "--delta" && i+1 < argc) delta = std::atof(argv[++i]);
        else if (a == "--quick") quick = true;
        else if (a[0] != '-') robot_path = a;
    }

    if (quick) { n_seeds = 3; n_trials = 2; max_boxes = 5000; n_random = 1000; }

    Robot robot = Robot::from_json(robot_path);
    std::cout << "Robot: " << robot.name() << "  DOF=" << robot.n_joints()
              << "  active=" << robot.n_active_links() << "\n";

    auto base_obs = make_combined_obstacles();
    auto queries = make_combined_queries();

    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> pairs;
    for (auto& qp : queries)
        pairs.push_back({qp.start, qp.goal});

    int total = n_seeds * n_trials;
    std::cout << "\n" << std::string(72, '=') << "\n"
              << "  Exp 3: Incremental Rebuild  (combined scene, ±"
              << delta * 100 << "cm perturbation)\n"
              << "  " << n_seeds << " seeds × " << n_trials << " trials"
              << "  max_boxes=" << max_boxes
              << "  random=" << n_random << "\n"
              << std::string(72, '=') << "\n";
    std::cout << std::fixed;

    std::vector<double> all_build, all_incr;
    int done = 0;

    for (int seed = 0; seed < n_seeds; ++seed) {
        // ── Initial build_multi ──
        SBFConfig cfg = make_panda_config(seed);
        cfg.max_boxes = max_boxes;
        cfg.max_consecutive_miss = 500;

        auto t0 = std::chrono::steady_clock::now();
        SBFPlanner planner(robot, base_obs, cfg);
        planner.build_multi(pairs, n_random, 300.0);
        double build_t = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
        all_build.push_back(build_t);

        int n_initial = planner.forest().n_boxes();
        std::cout << "\n  seed=" << seed
                  << "  build=" << std::setprecision(2) << build_t << "s"
                  << "  boxes=" << n_initial << "\n";

        for (int trial = 0; trial < n_trials; ++trial) {
            // Generate perturbed obstacles
            std::mt19937 pert_rng(seed * 10000 + trial);
            auto new_obs = perturb_obstacles(base_obs, delta, pert_rng);

            // ── Incremental update: remove old obstacles, add new ones ──
            // We simulate by: invalidate all old → add all new
            auto ti0 = std::chrono::steady_clock::now();

            // Remove all old obstacles and add perturbed ones
            for (auto& o : base_obs)
                planner.remove_obstacle(o.name);
            for (auto& o : new_obs)
                planner.add_obstacle(o);

            double incr_t = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - ti0).count();
            all_incr.push_back(incr_t);

            int n_remaining = planner.forest().n_boxes();
            done++;

            std::cout << "    trial=" << trial
                      << "  incr=" << std::setprecision(4) << incr_t << "s"
                      << "  boxes=" << n_remaining
                      << "/" << n_initial
                      << "  [" << done << "/" << total << "]\n";

            // Restore original obstacles for next trial
            for (auto& o : new_obs)
                planner.remove_obstacle(o.name);
            for (auto& o : base_obs)
                planner.add_obstacle(o);
        }
    }

    // Summary
    auto sb = compute_stats(all_build);
    auto si = compute_stats(all_incr);

    std::cout << "\n" << std::string(72, '=') << "\n"
              << "  Summary  (combined, ±" << delta * 100 << "cm perturbation)\n"
              << std::string(72, '-') << "\n"
              << "    Build:   med=" << std::setprecision(3) << sb.median << "s"
              << "  mean=" << sb.mean << "s\n"
              << "    Incr:    med=" << std::setprecision(4) << si.median << "s"
              << "  mean=" << si.mean << "s\n"
              << "    Speedup: " << std::setprecision(1)
              << (si.median > 1e-6 ? sb.median / si.median : 0) << "x (median)\n"
              << std::string(72, '=') << "\n";

    return 0;
}

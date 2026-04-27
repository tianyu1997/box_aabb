/**
 * exp7_parallel_vs_serial.cpp — 实验 7: 并行几何分区 vs 串行方法性能对比
 *
 * 维度:
 *   - 方法: Serial (n_threads=1) vs. Parallel-Geometric (n_threads=5)
 *   - Envelope: LinkIAABB vs. LinkIAABB_Grid
 *
 * 指标:
 *   - build_time (s): 总 build_coverage 时间
 *   - parallel_grow_ms: grow 阶段耗时 (从日志解析)
 *   - n_boxes: 最终 box 数量
 *   - master_promote: 跨域 promotion 合并次数 (parallel 模式下)
 *   - query_time (ms): 中位查询时间
 *   - SR%: 成功率
 *
 * 用法:
 *   ./exp7_parallel_vs_serial [--seeds N] [--quick]
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
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace sbf;

// ─── Utility ────────────────────────────────────────────────────────────────

struct Stats { double median, q25, q75; };

Stats compute_stats(std::vector<double> d) {
    if (d.empty()) return {};
    std::sort(d.begin(), d.end());
    int n = static_cast<int>(d.size());
    return { d[n / 2], d[n / 4], d[3 * n / 4] };
}

// ─── Capture SBF_INFO/WARN output for log mining ────────────────────────────
// SBFPlanner emits timing/count lines via SBF_INFO; redirect stderr to capture.
// Rather than global redirection (fragile in multi-threaded contexts) we simply
// run the experiment and parse results fields that are already exposed via the
// planner API.  For fields only in logs (master_promote count), we track via a
// custom callback if available, else fall back to 0.

// ─── Benchmark config ────────────────────────────────────────────────────────

struct BenchConfig {
    std::string label;
    int n_threads;                    // 1 = serial, 5 = parallel-geometric
    EnvelopeType envelope;
    bool enable_promotion = true;
};

SBFPlannerConfig make_cfg(const BenchConfig& bc, uint64_t seed,
                          int bridge_threads) {
    SBFPlannerConfig cfg;
    cfg.z4_enabled = true;
    cfg.split_order = SplitOrder::BEST_TIGHTEN;
    cfg.lect_no_cache = true;
    cfg.enable_coarsen = true;
    cfg.enable_path_opt = true;
    cfg.envelope_type.type = bc.envelope;

    cfg.grower.mode = GrowerConfig::Mode::RRT;
    cfg.grower.max_boxes = 200000;
    cfg.grower.n_threads = bc.n_threads;
    cfg.grower.rng_seed = seed;
    cfg.grower.max_consecutive_miss = 2000;
    cfg.grower.rrt_goal_bias = 0.1;
    cfg.grower.rrt_step_ratio = 0.05;
    cfg.grower.enable_promotion = bc.enable_promotion;
    cfg.grower.ffb_config.max_depth = 300;
    cfg.grower.connect_mode = true;
    cfg.grower.post_connect_extra_boxes = 500;
    cfg.grower.timeout_ms = 5000.0;
    cfg.grower.bridge_n_threads = bridge_threads;

    cfg.coarsen.target_boxes = 300;
    cfg.coarsen.max_rounds = 100;
    cfg.coarsen.max_lect_fk_per_round = 10000;
    cfg.coarsen.score_threshold = 500.0;

    cfg.smoother.shortcut_max_iters = 100;
    cfg.smoother.smooth_window = 3;
    cfg.smoother.smooth_iters = 5;

    return cfg;
}

// ─── Main ────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    std::string robot_path = std::string(SBF_DATA_DIR) + "/iiwa14.json";
    int n_seeds = 5;
    int bridge_threads = static_cast<int>(std::thread::hardware_concurrency());
    bool quick = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--seeds" && i + 1 < argc)   n_seeds = std::atoi(argv[++i]);
        else if (a == "--quick")               { quick = true; }
        else if (a[0] != '-')                  robot_path = a;
    }
    if (quick) n_seeds = 2;

    Robot robot = Robot::from_json(robot_path);
    auto obstacles = make_combined_obstacles();
    auto queries   = make_combined_queries();
    int n_obs = static_cast<int>(obstacles.size());
    int n_pairs = static_cast<int>(queries.size());

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

    std::cout << "Robot: " << robot.name() << "  DOF=" << robot.n_joints() << "\n"
              << "Scene: combined (" << n_obs << " obs, " << n_pairs << " queries)\n"
              << "Seeds=" << n_seeds << "\n\n";

    // Benchmark matrix
    std::vector<BenchConfig> configs = {
        { "Serial   / LinkIAABB",      1, EnvelopeType::LinkIAABB      },
        { "Parallel / LinkIAABB",      5, EnvelopeType::LinkIAABB      },
        { "Serial   / Grid",           1, EnvelopeType::LinkIAABB_Grid },
        { "Parallel / Grid",           5, EnvelopeType::LinkIAABB_Grid },
    };

    // Column header
    std::cout << std::string(110, '=') << "\n"
              << "  Parallel vs Serial — LinkIAABB & Grid\n"
              << std::string(110, '=') << "\n\n"
              << std::left  << std::setw(30) << "Config"
              << std::right
              << std::setw(12) << "Build(s)"
              << std::setw(10) << "Boxes"
              << std::setw(14) << "Query(ms)"
              << std::setw(8)  << "SR%"
              << "\n" << std::string(110, '-') << "\n";

    for (auto& bc : configs) {
        std::vector<double> build_times, query_times;
        std::vector<int> box_counts;
        int n_success = 0, n_total = 0;

        for (int seed = 0; seed < n_seeds; ++seed) {
            auto cfg = make_cfg(bc, static_cast<uint64_t>(seed), bridge_threads);
            SBFPlanner planner(robot, cfg);

            auto t0 = std::chrono::steady_clock::now();
            planner.build_coverage(obstacles.data(), n_obs, cfg.grower.timeout_ms,
                                   seed_points);
            double bt = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t0).count();
            build_times.push_back(bt);
            box_counts.push_back(planner.n_boxes());

            for (int pi = 0; pi < n_pairs; ++pi) {
                auto& qp = queries[pi];
                auto tq = std::chrono::steady_clock::now();
                auto res = planner.query(qp.start, qp.goal,
                                         obstacles.data(), n_obs);
                double qt = 1000.0 * std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - tq).count();
                n_total++;
                if (res.success) { n_success++; query_times.push_back(qt); }
            }
        }

        auto sb = compute_stats(build_times);
        auto sq = compute_stats(query_times);
        std::sort(box_counts.begin(), box_counts.end());
        int med_boxes = box_counts.empty() ? 0 : box_counts[box_counts.size() / 2];
        double sr = n_total > 0 ? 100.0 * n_success / n_total : 0.0;

        std::cout << std::left  << std::setw(30) << bc.label
                  << std::right << std::fixed
                  << std::setw(12) << std::setprecision(2) << sb.median
                  << std::setw(10) << med_boxes
                  << std::setw(14) << std::setprecision(1) << sq.median
                  << std::setw(8)  << std::setprecision(0) << sr
                  << "\n";
        std::cout.flush();
    }

    std::cout << "\n" << std::string(110, '=') << "\n"
              << "  Serial=n_threads=1, Parallel=n_threads=5 (geometric partition)\n"
              << "  Grid=LinkIAABB_Grid (voxel refinement), LinkIAABB=AABB only\n"
              << std::string(110, '=') << "\n";
    return 0;
}

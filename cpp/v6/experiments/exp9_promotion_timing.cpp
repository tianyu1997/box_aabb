/**
 * exp9_promotion_timing.cpp — 实验 9: Promotion 机制效果对比
 *
 * Promotion 是"孩子 envelope 并集合并"的自底向上粗化操作:
 *   - 若两个兄弟叶子均被占用, 将父节点的 envelope 设为孩子 envelope ∪
 *     (per-link min/max 或 SparseVoxelGrid::merge),
 *     再进行碰撞检测; 若无碰撞则两个孩子被合并为一个更大的父 box.
 *   - 并行模式只扫描各 worker domain root 的祖先链 (boundary_candidates),
 *     比串行的全节点扫描快 O(n_workers × depth) vs O(n_nodes).
 *
 * 对比配置:
 *   (1) No Promotion       : enable_promotion=false, n_threads=1 (串行)
 *   (2) No Promotion (Par) : enable_promotion=false, n_threads=5 (并行)
 *   (3) Promotion ON (Ser) : enable_promotion=true,  n_threads=1 (串行, 全扫描)
 *   (4) Promotion ON (Par) : enable_promotion=true,  n_threads=5 (并行, boundary-candidate 扫描)
 *
 * 指标: Build(s), Boxes, Promotions(median), Query(ms), SR%
 *
 * 用法:
 *   ./exp9_promotion_timing [--seeds N] [--quick]
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
#include <vector>

using namespace sbf;

struct Stats { double median, q25, q75; };
Stats compute_stats(std::vector<double> d) {
    if (d.empty()) return {};
    std::sort(d.begin(), d.end());
    int n = static_cast<int>(d.size());
    return { d[n / 2], d[n / 4], d[3 * n / 4] };
}

struct BenchConfig {
    std::string label;
    bool enable_promotion;
    int  n_threads;
};

SBFPlannerConfig make_cfg(const BenchConfig& bc, uint64_t seed, int bridge_threads) {
    SBFPlannerConfig cfg;
    cfg.z4_enabled = true;
    cfg.split_order = SplitOrder::BEST_TIGHTEN;
    cfg.lect_no_cache = true;
    cfg.enable_coarsen = true;
    cfg.enable_path_opt = true;
    cfg.envelope_type.type = EnvelopeType::LinkIAABB;

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

int main(int argc, char** argv) {
    std::string robot_path = std::string(SBF_DATA_DIR) + "/iiwa14.json";
    int n_seeds = 5;
    int bridge_threads = static_cast<int>(std::thread::hardware_concurrency());
    bool quick = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--seeds" && i + 1 < argc)   n_seeds = std::atoi(argv[++i]);
        else if (a == "--quick")               quick = true;
        else if (a[0] != '-')                  robot_path = a;
    }
    if (quick) n_seeds = 2;

    Robot robot = Robot::from_json(robot_path);
    auto obstacles = make_combined_obstacles();
    auto queries   = make_combined_queries();
    int n_obs = static_cast<int>(obstacles.size());
    int n_pairs = static_cast<int>(queries.size());

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
              << "Seeds=" << n_seeds << "  Envelope=LinkIAABB\n\n";

    std::vector<BenchConfig> configs = {
        { "No Promotion  (parallel, t=5)",  false, 5 },
        { "Promotion ON  (parallel, t=5)",  true,  5 },
    };

    constexpr int W = 105;
    std::cout << std::string(W, '=') << "\n"
              << "  Promotion Timing — Build & Query Comparison\n"
              << std::string(W, '=') << "\n\n"
              << std::left  << std::setw(34) << "Config"
              << std::right
              << std::setw(12) << "Build(s)"
              << std::setw(10) << "Boxes"
              << std::setw(12) << "Promos"
              << std::setw(14) << "Query(ms)"
              << std::setw(8)  << "SR%"
              << "\n" << std::string(W, '-') << "\n";

    for (auto& bc : configs) {
        std::vector<double> build_times, query_times;
        std::vector<int> box_counts, promo_counts;
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
            promo_counts.push_back(planner.build_timing().n_promotions);

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
        std::sort(promo_counts.begin(), promo_counts.end());
        int med_boxes  = box_counts.empty()  ? 0 : box_counts[box_counts.size() / 2];
        int med_promos = promo_counts.empty() ? 0 : promo_counts[promo_counts.size() / 2];
        double sr = n_total > 0 ? 100.0 * n_success / n_total : 0.0;

        std::cout << std::left  << std::setw(34) << bc.label
                  << std::right << std::fixed
                  << std::setw(12) << std::setprecision(2) << sb.median
                  << std::setw(10) << med_boxes
                  << std::setw(12) << med_promos
                  << std::setw(14) << std::setprecision(1) << sq.median
                  << std::setw(7)  << std::setprecision(1) << sr << "%"
                  << "\n";
    }

    std::cout << std::string(W, '=') << "\n";
    return 0;
}

/**
 * exp5_ablation.cpp — 实验 5: Optimisation Ablation Study (v2)
 *
 * 对应论文 Tab 4 (Optimisation Effectiveness).
 *
 * 系统性地启用/禁用以下算法组件, 测量 build_time, query_time, path_length:
 *   - LECT Split Strategy  (BT / RR / WF)
 *   - Coordinated Growth   (connect_mode ON/OFF)
 *   - Promotion            (enable_promotion ON/OFF)
 *   - Coarsening           (enable_coarsen ON/OFF)
 *   - Path Optimization    (enable_path_opt ON/OFF)
 *
 * 基线: 全部组件开启, BEST_TIGHTEN 分裂策略
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
#include <fstream>
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

const char* split_order_name(SplitOrder s) {
    switch (s) {
        case SplitOrder::ROUND_ROBIN:     return "RR";
        case SplitOrder::WIDEST_FIRST:    return "WF";
        case SplitOrder::BEST_TIGHTEN:    return "BT";
        case SplitOrder::BEST_TIGHTEN_V2: return "BT2";
    }
    return "?";
}

struct AblationConfig {
    std::string name;
    SplitOrder split_order = SplitOrder::BEST_TIGHTEN;
    bool connect_mode     = true;   // Coordinated multi-tree growth
    bool enable_promotion = true;   // Sibling leaf promotion
    bool enable_coarsen   = true;   // Multi-level coarsening
    bool enable_path_opt  = true;   // 5-step path optimization
};

SBFPlannerConfig make_planner_config(const AblationConfig& ab, int n_threads,
                                      uint64_t seed) {
    SBFPlannerConfig cfg;
    cfg.z4_enabled = true;
    cfg.split_order = ab.split_order;
    cfg.lect_no_cache = true;  // Disable LECT disk cache to avoid 10+ second lect_save
    cfg.enable_coarsen = ab.enable_coarsen;
    cfg.enable_path_opt = ab.enable_path_opt;

    cfg.grower.mode = GrowerConfig::Mode::RRT;
    cfg.grower.max_boxes = 200000;
    cfg.grower.n_threads = 5;
    cfg.grower.rng_seed = seed;
    cfg.grower.max_consecutive_miss = 2000;
    cfg.grower.rrt_goal_bias = 0.1;
    cfg.grower.rrt_step_ratio = 0.05;
    cfg.grower.enable_promotion = ab.enable_promotion;
    cfg.grower.ffb_config.max_depth = 300;

    // Coordinated growth (connect-mode)
    if (ab.connect_mode) {
        cfg.grower.connect_mode = true;
        cfg.grower.post_connect_extra_boxes = 4000;
        cfg.grower.timeout_ms = 60000.0;
    } else {
        cfg.grower.connect_mode = false;
        cfg.grower.post_connect_extra_boxes = 0;
        cfg.grower.max_boxes = 3000;      // Cap box count without connect-stop
        cfg.grower.timeout_ms = 3000.0;   // 3s budget
    }

    cfg.grower.bridge_n_threads = n_threads;

    cfg.coarsen.target_boxes = 300;
    cfg.coarsen.max_rounds = 100;
    cfg.coarsen.max_lect_fk_per_round = 10000;
    cfg.coarsen.score_threshold = 500.0;

    cfg.smoother.shortcut_max_iters = 100;
    cfg.smoother.smooth_window = 3;
    cfg.smoother.smooth_iters = 5;

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
    std::string scene_name = "combined";
    std::string json_out;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--seeds" && i + 1 < argc) n_seeds = std::atoi(argv[++i]);
        else if (a == "--threads" && i + 1 < argc) n_threads = std::atoi(argv[++i]);
        else if (a == "--scene" && i + 1 < argc) scene_name = argv[++i];
        else if (a == "--json" && i + 1 < argc) json_out = argv[++i];
        else if (a == "--quick") quick = true;
        else if (a[0] != '-') robot_path = a;
    }

    if (quick) { n_seeds = 2; }
    if (n_threads < 1) n_threads = 1;

    Robot robot = Robot::from_json(robot_path);

    // Scene selection
    std::vector<Obstacle> obstacles;
    std::vector<QueryPair> queries;
    if (scene_name == "shelves") {
        obstacles = make_shelves_obstacles();
        queries   = make_shelves_queries();
    } else if (scene_name == "bins") {
        obstacles = make_bins_obstacles();
        queries   = make_bins_queries();
    } else if (scene_name == "table") {
        obstacles = make_table_obstacles();
        queries   = make_table_queries();
    } else {
        obstacles = make_combined_obstacles();
        queries   = make_combined_queries();
    }
    int n_obs = static_cast<int>(obstacles.size());
    int n_pairs = static_cast<int>(queries.size());

    std::cout << "Robot: " << robot.name() << "  DOF=" << robot.n_joints() << "\n"
              << "Scene: " << scene_name << " (" << n_obs << " obs, " << n_pairs << " queries)\n"
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
        // ── Baseline: all components ON, BEST_TIGHTEN split ──
        {"Full",                SplitOrder::BEST_TIGHTEN, true, true, true, true},
        // ── Group 1: LECT Split Strategy ──
        {"Split: RoundRobin",   SplitOrder::ROUND_ROBIN,  true, true, true, true},
        {"Split: WidestFirst",  SplitOrder::WIDEST_FIRST,  true, true, true, true},
        // ── Group 2: Coordinated Growth ──
        {"w/o ConnectMode",     SplitOrder::BEST_TIGHTEN, false, true, true, true},
        // ── Group 3: Promotion ──
        {"w/o Promotion",       SplitOrder::BEST_TIGHTEN, true, false, true, true},
        // ── Group 4: Coarsening ──
        {"w/o Coarsen",         SplitOrder::BEST_TIGHTEN, true, true, false, true},
        // ── Group 5: Path Optimization ──
        {"w/o PathOpt",         SplitOrder::BEST_TIGHTEN, true, true, true, false},
    };

    std::cout << std::string(120, '=') << "\n"
              << "  Optimisation Ablation Study v2 (Paper Tab 4)\n"
              << std::string(120, '=') << "\n\n";

    // Header
    std::cout << std::left << std::setw(22) << "Config"
              << std::right
              << std::setw(6) << "Split"
              << std::setw(6) << "CM"
              << std::setw(6) << "Promo"
              << std::setw(6) << "Coar"
              << std::setw(6) << "PaOpt"
              << std::setw(12) << "Build(s)"
              << std::setw(10) << "Boxes"
              << std::setw(12) << "Query(s)"
              << std::setw(10) << "Len"
              << std::setw(8)  << "SR%"
              << "\n" << std::string(120, '-') << "\n";

    // JSON accumulator
    std::string json_rows;

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

        std::cout << std::left << std::setw(22) << ab.name
                  << std::right
                  << std::setw(6) << split_order_name(ab.split_order)
                  << std::setw(6) << (ab.connect_mode ? "ON" : "off")
                  << std::setw(6) << (ab.enable_promotion ? "ON" : "off")
                  << std::setw(6) << (ab.enable_coarsen ? "ON" : "off")
                  << std::setw(6) << (ab.enable_path_opt ? "ON" : "off")
                  << std::fixed
                  << std::setw(12) << std::setprecision(2) << sb.median
                  << std::setw(10) << med_boxes
                  << std::setw(12) << std::setprecision(3) << sq.median
                  << std::setw(10) << std::setprecision(3) << sl.median
                  << std::setw(8) << std::setprecision(0) << sr
                  << "\n";

        // Accumulate JSON row
        char jbuf[512];
        std::snprintf(jbuf, sizeof(jbuf),
            "%s{\"name\":\"%s\",\"split\":\"%s\","
            "\"connect_mode\":%s,\"promotion\":%s,\"coarsen\":%s,\"path_opt\":%s,"
            "\"build_median\":%.4f,\"build_mean\":%.4f,\"build_q25\":%.4f,\"build_q75\":%.4f,"
            "\"boxes\":%d,"
            "\"query_median\":%.6f,\"query_mean\":%.6f,\"query_q25\":%.6f,\"query_q75\":%.6f,"
            "\"len_median\":%.4f,\"sr\":%.1f}",
            json_rows.empty() ? "" : ",\n",
            ab.name.c_str(),
            split_order_name(ab.split_order),
            ab.connect_mode ? "true" : "false",
            ab.enable_promotion ? "true" : "false",
            ab.enable_coarsen ? "true" : "false",
            ab.enable_path_opt ? "true" : "false",
            sb.median, sb.mean, sb.q25, sb.q75,
            med_boxes,
            sq.median, sq.mean, sq.q25, sq.q75,
            sl.median, sr);
        json_rows += jbuf;
    }

    std::cout << "\n" << std::string(120, '=') << "\n"
              << "  Split=LECT split strategy (BT/RR/WF)\n"
              << "  CM=Coordinated multi-tree growth, Promo=Promotion\n"
              << "  Coar=Multi-level coarsening, PaOpt=5-step path optimization\n"
              << std::string(120, '=') << "\n";

    // Write JSON output if requested
    if (!json_out.empty()) {
        std::ofstream ofs(json_out);
        if (ofs.is_open()) {
            ofs << "{\"scene\":\"" << scene_name << "\","
                << "\"robot\":\"" << robot.name() << "\","
                << "\"n_seeds\":" << n_seeds << ","
                << "\"n_threads\":" << n_threads << ","
                << "\"results\":[\n" << json_rows << "\n]}\n";
            ofs.close();
            std::cout << "\n  JSON written to: " << json_out << "\n";
        }
    }

    std::cout << "\n  Exp 5 complete.\n";
    return 0;
}

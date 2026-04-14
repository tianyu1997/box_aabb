/**
 * exp3_incremental.cpp — 实验 3: 冷启动 vs 热重建 (LECT cache)
 *
 * v5 没有 v1 的 add/remove_obstacle 增量 API;
 * 改为对比: cold build (无 LECT 缓存) vs warm build (有 LECT 缓存)
 * 来量化 LECT persistent cache 的加速效果.
 *
 * 附加测试: 环境扰动 (±2cm 障碍物偏移) 后重建,
 *   验证 warm LECT cache 对 邻近场景 仍有加速效果.
 *
 * 可选: --gcs 启用 GCS 求解器, --query 在每次 build 后执行 5 对查询.
 *
 * 设计:
 *   合并场景 (16 obstacles), 多 seed
 *   1) cold build: lect_no_cache = true (同 seed)
 *   2) warm build: lect_no_cache = false (先预热, 再计时, 同 seed)
 *   3) perturbed build: warm cache + ±2cm 障碍物扰动
 *
 * 注意: cold 与 warm 使用相同 RNG seed 以隔离缓存效果。
 *
 * 用法:
 *   ./exp3_incremental [--seeds N] [--threads N] [--delta D] [--quick]
 *                      [--gcs] [--query]
 */

#include <sbf/planner/sbf_planner.h>
#include <sbf/scene/collision_checker.h>
#include <sbf/core/robot.h>
#include "marcucci_scenes.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <string>
#include <thread>
#include <vector>

using namespace sbf;

// ═══════════════════════════════════════════════════════════════════════════
// Obstacle perturbation: ±delta offset to each obstacle center
// ═══════════════════════════════════════════════════════════════════════════

std::vector<Obstacle> perturb_obstacles(
    const std::vector<Obstacle>& base, double delta, std::mt19937& rng)
{
    std::uniform_real_distribution<float> ud(
        static_cast<float>(-delta), static_cast<float>(delta));
    auto perturbed = base;
    for (auto& o : perturbed) {
        float dx = ud(rng), dy = ud(rng), dz = ud(rng);
        o.bounds[0] += dx; o.bounds[3] += dx;  // lo_x, hi_x
        o.bounds[1] += dy; o.bounds[4] += dy;  // lo_y, hi_y
        o.bounds[2] += dz; o.bounds[5] += dz;  // lo_z, hi_z
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
    int n_seeds = 10;
    int n_threads = static_cast<int>(std::thread::hardware_concurrency());
    int n_perturb = 5;    // perturbation trials per seed
    double delta = 0.02;  // ±2cm
    bool quick = false;
    bool use_gcs = false;
    bool run_query = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--seeds" && i+1 < argc) n_seeds = std::atoi(argv[++i]);
        else if (a == "--threads" && i+1 < argc) n_threads = std::atoi(argv[++i]);
        else if (a == "--perturb" && i+1 < argc) n_perturb = std::atoi(argv[++i]);
        else if (a == "--delta" && i+1 < argc) delta = std::atof(argv[++i]);
        else if (a == "--quick") quick = true;
        else if (a == "--gcs") use_gcs = true;
        else if (a == "--query") run_query = true;
        else if (a[0] != '-') robot_path = a;
    }

    if (quick) { n_seeds = 3; n_perturb = 2; }
    if (n_threads < 1) n_threads = 1;

    Robot robot = Robot::from_json(robot_path);
    std::cout << "Robot: " << robot.name() << "  DOF=" << robot.n_joints()
              << "  active_links=" << robot.n_active_links() << "\n";

    auto obstacles = make_combined_obstacles();
    auto queries = make_combined_queries();
    int n_obs = static_cast<int>(obstacles.size());
    int n_pairs = static_cast<int>(queries.size());

    std::cout << "\n" << std::string(72, '=') << "\n"
              << "  Exp 3: Cold vs Warm Rebuild (LECT cache)\n"
              << "  combined scene (" << n_obs << " obs), "
              << n_seeds << " seeds, "
              << n_perturb << " perturbation trials, "
              << "±" << delta * 100 << "cm"
              << (use_gcs ? "  solver=GCS" : "  solver=Dijkstra")
              << (run_query ? "  +query" : "") << "\n"
              << "  threads=" << n_threads << "\n"
              << std::string(72, '=') << "\n";
    std::cout << std::fixed;

    // Collect unique query endpoints as seed points for multi-goal RRT
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

    // Shared config template (matches exp2 settings)
    auto make_cfg = [&](uint64_t seed, bool lect_cold) {
        SBFPlannerConfig cfg;
        cfg.z4_enabled = true;
        cfg.split_order = SplitOrder::BEST_TIGHTEN;
        cfg.lect_no_cache = lect_cold;

        cfg.grower.mode = GrowerConfig::Mode::RRT;
        cfg.grower.max_boxes = 200000;
        cfg.grower.timeout_ms = 5000.0;
        cfg.grower.n_threads = 5;                // one thread per tree
        cfg.grower.rng_seed = seed;
        cfg.grower.max_consecutive_miss = 2000;
        cfg.grower.rrt_goal_bias = 0.8;
        cfg.grower.rrt_step_ratio = 0.05;
        cfg.grower.connect_mode = true;
        cfg.grower.enable_promotion = true;
        cfg.grower.ffb_config.max_depth = 300;

        cfg.coarsen.target_boxes = 300;
        cfg.coarsen.max_rounds = 100;
        cfg.coarsen.max_lect_fk_per_round = 10000;
        cfg.coarsen.score_threshold = 500.0;

        cfg.smoother.shortcut_max_iters = 100;
        cfg.smoother.smooth_window = 3;
        cfg.smoother.smooth_iters = 5;

        cfg.use_gcs = use_gcs;

        return cfg;
    };

    std::vector<double> cold_times, warm_times, perturb_times;
    std::vector<double> cold_query_len, warm_query_len, perturb_query_len;
    std::vector<int>    cold_query_ok, warm_query_ok, perturb_query_ok;
    int cold_query_total = 0, warm_query_total = 0, perturb_query_total = 0;
    std::vector<int>    cold_boxes, warm_boxes, perturb_boxes;
    // GCS collision tracking
    int gcs_total_segs = 0, gcs_coll_segs = 0;

    // Helper: run queries on a built planner, report results
    auto run_queries = [&](SBFPlanner& planner, const Obstacle* obs, int nobs,
                           const std::string& tag,
                           std::vector<double>& len_acc,
                           std::vector<int>& ok_acc, int& total_acc) {
        if (!run_query) return;
        int ok = 0;
        for (int pi = 0; pi < n_pairs; ++pi) {
            auto& qp = queries[pi];
            auto res = planner.query(qp.start, qp.goal, obs, nobs);
            total_acc++;
            if (res.success) {
                ok++;
                len_acc.push_back(res.path_length);
                ok_acc.push_back(1);

                // Collision validation
                CollisionChecker val_checker(robot, {});
                val_checker.set_obstacles(obs, nobs);
                int n_col = 0;
                for (size_t wi = 0; wi + 1 < res.path.size(); ++wi) {
                    double slen = (res.path[wi+1] - res.path[wi]).norm();
                    int vres = std::max(20, static_cast<int>(std::ceil(slen / 0.005)));
                    if (val_checker.check_segment(res.path[wi], res.path[wi+1], vres))
                        n_col++;
                }
                if (n_col > 0)
                    fprintf(stderr, "[VAL] %s %s: %d/%d segs collide\n",
                            tag.c_str(), qp.label.c_str(),
                            n_col, (int)(res.path.size()-1));
            } else {
                ok_acc.push_back(0);
            }
        }
        std::cout << "      " << tag << " query: " << ok << "/" << n_pairs << " OK\n";
    };

    for (int seed = 0; seed < n_seeds; ++seed) {
        std::cout << "\n  ── seed=" << seed << " ──\n";

        // ── 1) Cold build ──
        {
            auto cfg = make_cfg(static_cast<uint64_t>(seed), /*lect_cold=*/true);
            SBFPlanner planner(robot, cfg);

            auto t0 = std::chrono::steady_clock::now();
            planner.build_coverage(obstacles.data(), n_obs, 5000.0, seed_points);
            double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t0).count();
            cold_times.push_back(elapsed);

            cold_boxes.push_back(planner.n_boxes());
            std::cout << "    cold:    t=" << std::setprecision(3) << elapsed << "s"
                      << "  boxes=" << planner.n_boxes() << "\n";
            run_queries(planner, obstacles.data(), n_obs, "cold",
                        cold_query_len, cold_query_ok, cold_query_total);
        }

        // ── 2) Warm build (pre-heat LECT cache, then timed run) ──
        {
            // Pre-heat: run once to populate LECT cache
            auto preheat_cfg = make_cfg(static_cast<uint64_t>(seed), /*lect_cold=*/false);
            {
                SBFPlanner preheat(robot, preheat_cfg);
                preheat.build_coverage(obstacles.data(), n_obs, 5000.0, seed_points);
            }

            // Timed warm run (should load LECT from disk cache)
            // 使用与 cold 相同的 seed 以隔离缓存效果
            auto cfg = make_cfg(static_cast<uint64_t>(seed), /*lect_cold=*/false);
            SBFPlanner planner(robot, cfg);

            auto t0 = std::chrono::steady_clock::now();
            planner.build_coverage(obstacles.data(), n_obs, 5000.0, seed_points);
            double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t0).count();
            warm_times.push_back(elapsed);

            warm_boxes.push_back(planner.n_boxes());
            std::cout << "    warm:    t=" << std::setprecision(3) << elapsed << "s"
                      << "  boxes=" << planner.n_boxes() << "\n";
            run_queries(planner, obstacles.data(), n_obs, "warm",
                        warm_query_len, warm_query_ok, warm_query_total);
        }

        // ── 3) Perturbed builds (warm cache + shifted obstacles) ──
        for (int trial = 0; trial < n_perturb; ++trial) {
            std::mt19937 pert_rng(static_cast<uint32_t>(seed * 10000 + trial));
            auto pert_obs = perturb_obstacles(obstacles, delta, pert_rng);
            int n_pert = static_cast<int>(pert_obs.size());

            auto cfg = make_cfg(static_cast<uint64_t>(seed * 100 + trial), /*lect_cold=*/false);
            SBFPlanner planner(robot, cfg);

            auto t0 = std::chrono::steady_clock::now();
            planner.build_coverage(pert_obs.data(), n_pert, 5000.0, seed_points);
            double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t0).count();
            perturb_times.push_back(elapsed);

            perturb_boxes.push_back(planner.n_boxes());
            std::cout << "    pert[" << trial << "]: t=" << std::setprecision(3)
                      << elapsed << "s"
                      << "  boxes=" << planner.n_boxes() << "\n";
            run_queries(planner, pert_obs.data(), n_pert, "pert",
                        perturb_query_len, perturb_query_ok, perturb_query_total);
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Summary
    // ═══════════════════════════════════════════════════════════════════════
    auto sc = compute_stats(cold_times);
    auto sw = compute_stats(warm_times);
    auto sp = compute_stats(perturb_times);

    // Box count stats
    auto compute_istat = [](std::vector<int>& d) -> std::pair<int,double> {
        if (d.empty()) return {0, 0};
        std::sort(d.begin(), d.end());
        int med = d[d.size()/2];
        double mean = 0; for (int v : d) mean += v; mean /= d.size();
        return {med, mean};
    };
    auto [cb_med, cb_mean] = compute_istat(cold_boxes);
    auto [wb_med, wb_mean] = compute_istat(warm_boxes);
    auto [pb_med, pb_mean] = compute_istat(perturb_boxes);

    std::cout << "\n" << std::string(72, '=') << "\n"
              << "  Summary\n"
              << std::string(72, '-') << "\n"
              << "    Cold:    med=" << std::setprecision(3) << sc.median << "s"
              << "  mean=" << sc.mean << "s"
              << "  boxes_med=" << cb_med << "\n"
              << "    Warm:    med=" << std::setprecision(3) << sw.median << "s"
              << "  mean=" << sw.mean << "s"
              << "  boxes_med=" << wb_med << "\n"
              << "    Perturb: med=" << std::setprecision(3) << sp.median << "s"
              << "  mean=" << sp.mean << "s"
              << "  boxes_med=" << pb_med << "\n"
              << "\n"
              << "    Cold/Warm speedup: " << std::setprecision(2)
              << (sw.median > 1e-6 ? sc.median / sw.median : 0) << "x (median)\n"
              << "    Cold/Pert speedup: " << std::setprecision(2)
              << (sp.median > 1e-6 ? sc.median / sp.median : 0) << "x (median)\n";

    if (run_query) {
        auto compute_sr = [](const std::vector<int>& ok, int total) -> double {
            if (total == 0) return 0;
            int sum = 0; for (int v : ok) sum += v;
            return 100.0 * sum / total;
        };
        std::cout << "\n  Query results"
                  << (use_gcs ? " (GCS)" : " (Dijkstra)") << ":\n"
                  << "    Cold:    SR=" << std::setprecision(0)
                  << compute_sr(cold_query_ok, cold_query_total) << "%";
        if (!cold_query_len.empty()) {
            auto sl = compute_stats(cold_query_len);
            std::cout << "  len_med=" << std::setprecision(3) << sl.median;
        }
        std::cout << "\n    Warm:    SR=" << std::setprecision(0)
                  << compute_sr(warm_query_ok, warm_query_total) << "%";
        if (!warm_query_len.empty()) {
            auto sl = compute_stats(warm_query_len);
            std::cout << "  len_med=" << std::setprecision(3) << sl.median;
        }
        std::cout << "\n    Perturb: SR=" << std::setprecision(0)
                  << compute_sr(perturb_query_ok, perturb_query_total) << "%";
        if (!perturb_query_len.empty()) {
            auto sl = compute_stats(perturb_query_len);
            std::cout << "  len_med=" << std::setprecision(3) << sl.median;
        }
        std::cout << "\n";
    }

    std::cout << std::string(72, '=') << "\n";

    std::cout << "\n  Exp 3 complete.\n";
    return 0;
}

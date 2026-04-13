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
 * 设计:
 *   合并场景 (16 obstacles), 多 seed
 *   1) cold build: lect_no_cache = true
 *   2) warm build: lect_no_cache = false (先预热, 再计时)
 *   3) perturbed build: warm cache + ±2cm 障碍物扰动
 *
 * 用法:
 *   ./exp3_incremental [--seeds N] [--threads N] [--delta D] [--quick]
 */

#include <sbf/planner/sbf_planner.h>
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

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--seeds" && i+1 < argc) n_seeds = std::atoi(argv[++i]);
        else if (a == "--threads" && i+1 < argc) n_threads = std::atoi(argv[++i]);
        else if (a == "--perturb" && i+1 < argc) n_perturb = std::atoi(argv[++i]);
        else if (a == "--delta" && i+1 < argc) delta = std::atof(argv[++i]);
        else if (a == "--quick") quick = true;
        else if (a[0] != '-') robot_path = a;
    }

    if (quick) { n_seeds = 3; n_perturb = 2; }
    if (n_threads < 1) n_threads = 1;

    Robot robot = Robot::from_json(robot_path);
    std::cout << "Robot: " << robot.name() << "  DOF=" << robot.n_joints()
              << "  active_links=" << robot.n_active_links() << "\n";

    auto obstacles = make_combined_obstacles();
    int n_obs = static_cast<int>(obstacles.size());

    std::cout << "\n" << std::string(72, '=') << "\n"
              << "  Exp 3: Cold vs Warm Rebuild (LECT cache)\n"
              << "  combined scene (" << n_obs << " obs), "
              << n_seeds << " seeds, "
              << n_perturb << " perturbation trials, "
              << "±" << delta * 100 << "cm\n"
              << "  threads=" << n_threads << "\n"
              << std::string(72, '=') << "\n";
    std::cout << std::fixed;

    // Shared config template
    auto make_cfg = [&](uint64_t seed, bool lect_cold) {
        SBFPlannerConfig cfg;
        cfg.z4_enabled = true;
        cfg.split_order = SplitOrder::BEST_TIGHTEN;
        cfg.lect_no_cache = lect_cold;

        cfg.grower.mode = GrowerConfig::Mode::WAVEFRONT;
        cfg.grower.max_boxes = 2000;
        cfg.grower.n_threads = n_threads;
        cfg.grower.rng_seed = seed;
        cfg.grower.wavefront_stages = {
            {100}, {300}, {800}, {1500}, {2000}
        };
        cfg.grower.max_consecutive_miss = 100;

        cfg.coarsen.target_boxes = 500;
        cfg.coarsen.max_rounds = 100;

        return cfg;
    };

    std::vector<double> cold_times, warm_times, perturb_times;

    for (int seed = 0; seed < n_seeds; ++seed) {
        std::cout << "\n  ── seed=" << seed << " ──\n";

        // ── 1) Cold build ──
        {
            auto cfg = make_cfg(static_cast<uint64_t>(seed), /*lect_cold=*/true);
            SBFPlanner planner(robot, cfg);

            auto t0 = std::chrono::steady_clock::now();
            planner.build_coverage(obstacles.data(), n_obs, 60000.0);
            double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t0).count();
            cold_times.push_back(elapsed);

            std::cout << "    cold:    t=" << std::setprecision(3) << elapsed << "s"
                      << "  boxes=" << planner.n_boxes() << "\n";
        }

        // ── 2) Warm build (pre-heat LECT cache, then timed run) ──
        {
            // Pre-heat: run once to populate LECT cache
            auto preheat_cfg = make_cfg(static_cast<uint64_t>(seed), /*lect_cold=*/false);
            {
                SBFPlanner preheat(robot, preheat_cfg);
                preheat.build_coverage(obstacles.data(), n_obs, 60000.0);
            }

            // Timed warm run (should load LECT from disk cache)
            auto cfg = make_cfg(static_cast<uint64_t>(seed + 100), /*lect_cold=*/false);
            SBFPlanner planner(robot, cfg);

            auto t0 = std::chrono::steady_clock::now();
            planner.build_coverage(obstacles.data(), n_obs, 60000.0);
            double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t0).count();
            warm_times.push_back(elapsed);

            std::cout << "    warm:    t=" << std::setprecision(3) << elapsed << "s"
                      << "  boxes=" << planner.n_boxes() << "\n";
        }

        // ── 3) Perturbed builds (warm cache + shifted obstacles) ──
        for (int trial = 0; trial < n_perturb; ++trial) {
            std::mt19937 pert_rng(static_cast<uint32_t>(seed * 10000 + trial));
            auto pert_obs = perturb_obstacles(obstacles, delta, pert_rng);
            int n_pert = static_cast<int>(pert_obs.size());

            auto cfg = make_cfg(static_cast<uint64_t>(seed * 100 + trial), /*lect_cold=*/false);
            SBFPlanner planner(robot, cfg);

            auto t0 = std::chrono::steady_clock::now();
            planner.build_coverage(pert_obs.data(), n_pert, 60000.0);
            double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t0).count();
            perturb_times.push_back(elapsed);

            std::cout << "    pert[" << trial << "]: t=" << std::setprecision(3)
                      << elapsed << "s"
                      << "  boxes=" << planner.n_boxes() << "\n";
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Summary
    // ═══════════════════════════════════════════════════════════════════════
    auto sc = compute_stats(cold_times);
    auto sw = compute_stats(warm_times);
    auto sp = compute_stats(perturb_times);

    std::cout << "\n" << std::string(72, '=') << "\n"
              << "  Summary\n"
              << std::string(72, '-') << "\n"
              << "    Cold:    med=" << std::setprecision(3) << sc.median << "s"
              << "  mean=" << sc.mean << "s\n"
              << "    Warm:    med=" << std::setprecision(3) << sw.median << "s"
              << "  mean=" << sw.mean << "s\n"
              << "    Perturb: med=" << std::setprecision(3) << sp.median << "s"
              << "  mean=" << sp.mean << "s\n"
              << "\n"
              << "    Cold/Warm speedup: " << std::setprecision(1)
              << (sw.median > 1e-6 ? sc.median / sw.median : 0) << "x (median)\n"
              << "    Cold/Pert speedup: " << std::setprecision(1)
              << (sp.median > 1e-6 ? sc.median / sp.median : 0) << "x (median)\n"
              << std::string(72, '=') << "\n";

    std::cout << "\n  Exp 3 complete.\n";
    return 0;
}

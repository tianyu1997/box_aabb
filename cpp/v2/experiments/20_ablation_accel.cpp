// ═══════════════════════════════════════════════════════════════════════════
// Experiment 20 — Ablation + Acceleration: n_sub=1 only
// ═══════════════════════════════════════════════════════════════════════════
//
// Compares the ORIGINAL slow pipeline (FD gradient + full-joint optimisation
// + O(N) seed screening) against the ACCELERATED pipeline:
//
//   Acceleration 1: Analytical FK Jacobian  (14× cheaper gradient)
//   Acceleration 2: Per-link joint scope    (only joints [0..V] for link V)
//   Acceleration 3: Smart seed selection    (use tracked extremes, no pre-screening)
//   Acceleration 4: Reduced iterations      (30→15 iter, 2→1 seed)
//
// Variants (all at n_sub=1):
//   A: Baseline        — no enhancements
//   B: Full (slow)     — all 3 stages, finite-difference L-BFGS-B
//   C: Full (fast)     — all 3 stages, analytical Jacobian + smart seeds
//   D: Full (fast+)    — fast + reduced iterations (15 iter, 1 seed)
//   E: Coup+Manif only — stages 1+2 without L-BFGS-B (cheapest enhancement)
//   F: Fast, 1 seed    — all 3 stages, analytical Jacobian, 1 seed only
//
// Build:
//   cmake .. -DSBF_BUILD_EXPERIMENTS=ON
//   cmake --build . --target exp_20_ablation_accel --config Release
//
// Run:
//   ./exp_20_ablation_accel [--trials 30] [--repeats 5]
//
// ═══════════════════════════════════════════════════════════════════════════

#include "sbf/robot/robot.h"
#include "sbf/robot/fk.h"
#include "sbf/envelope/envelope_derive_critical.h"
#include "sbf/common/config.h"

#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace sbf;
using namespace sbf::envelope;
using Clock = std::chrono::high_resolution_clock;

// ─── CLI config ──────────────────────────────────────────────────────────

struct ExpConfig {
    int n_trials  = 30;
    int n_repeats = 5;
    std::string output_dir;
};

static ExpConfig parse_args(int argc, char** argv) {
    ExpConfig cfg;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--trials" && i + 1 < argc) cfg.n_trials = std::stoi(argv[++i]);
        if (a == "--repeats" && i + 1 < argc) cfg.n_repeats = std::stoi(argv[++i]);
        if (a == "--output" && i + 1 < argc) cfg.output_dir = argv[++i];
    }
    return cfg;
}

// ─── Width regimes ───────────────────────────────────────────────────────

struct WidthRegime {
    const char* name;
    double lo, hi;
};

static const WidthRegime WIDTHS[] = {
    {"small",  0.05, 0.15},
    {"medium", 0.15, 0.50},
    {"large",  0.50, 1.50},
};

// ─── Ablation + acceleration variants ────────────────────────────────────

struct Variant {
    const char* name;
    const char* short_name;
    CriticalSamplingConfig config;
};

static std::vector<Variant> build_variants() {
    std::vector<Variant> vars;

    // A: Baseline (no enhancements)
    {
        auto c = CriticalSamplingConfig::baseline();
        vars.push_back({"Baseline", "base", c});
    }

    // B: Full pipeline — SLOW (finite-difference, all-joint, O(N) seed screening)
    {
        auto c = CriticalSamplingConfig::all_enabled();
        c.use_analytical_jacobian = false;
        c.smart_seed_selection    = false;
        c.restrict_joint_scope    = false;
        c.lbfgs_max_iter = 30;
        c.lbfgs_n_seeds  = 2;
        vars.push_back({"Full(slow)", "full_slow", c});
    }

    // C: Full pipeline — FAST (analytical Jacobian + smart seeds)
    {
        auto c = CriticalSamplingConfig::all_enabled();
        c.use_analytical_jacobian = true;
        c.smart_seed_selection    = true;
        c.restrict_joint_scope    = true;
        c.lbfgs_max_iter = 30;
        c.lbfgs_n_seeds  = 2;
        vars.push_back({"Full(fast)", "full_fast", c});
    }

    // D: Full pipeline — FAST+ (analytical Jacobian + smart seeds + reduced iter)
    {
        auto c = CriticalSamplingConfig::all_enabled();
        c.use_analytical_jacobian = true;
        c.smart_seed_selection    = true;
        c.restrict_joint_scope    = true;
        c.lbfgs_max_iter = 15;
        c.lbfgs_n_seeds  = 1;
        vars.push_back({"Full(fast+)", "full_fastp", c});
    }

    // E: Coupled + Manifold only (no L-BFGS-B — cheapest useful enhancement)
    {
        auto c = CriticalSamplingConfig::baseline();
        c.enable_coupled_constraints = true;
        c.enable_manifold_sampling   = true;
        vars.push_back({"Coup+Manif", "coup_manif", c});
    }

    // F: Fast, 1 seed only
    {
        auto c = CriticalSamplingConfig::all_enabled();
        c.use_analytical_jacobian = true;
        c.smart_seed_selection    = true;
        c.restrict_joint_scope    = true;
        c.lbfgs_max_iter = 30;
        c.lbfgs_n_seeds  = 1;
        vars.push_back({"Fast(1seed)", "fast_1seed", c});
    }

    return vars;
}

// ─── Robot configs ───────────────────────────────────────────────────────

struct RobotSetup {
    std::string name;
    std::string json_path;
};

static std::vector<RobotSetup> find_robots() {
    std::vector<RobotSetup> robots;
    std::vector<std::string> search_paths = {
        "../../../safeboxforest/v1/configs",
        "../../safeboxforest/v1/configs",
        "../safeboxforest/v1/configs",
        "../../../../safeboxforest/v1/configs",
    };
    for (auto& base : search_paths) {
        std::string panda_path = base + "/panda.json";
        std::string iiwa_path  = base + "/iiwa14.json";
        if (fs::exists(panda_path)) {
            robots.push_back({"panda", panda_path});
            if (fs::exists(iiwa_path))
                robots.push_back({"iiwa14", iiwa_path});
            break;
        }
    }
    if (robots.empty())
        std::cerr << "Warning: No robot configs found.\n";
    return robots;
}

// ─── Volume helper ───────────────────────────────────────────────────────

static double aabb_volume(const float* aabbs, int n_slots) {
    double total = 0.0;
    for (int i = 0; i < n_slots; ++i) {
        const float* a = aabbs + i * 6;
        double dx = std::max(0.0, double(a[3] - a[0]));
        double dy = std::max(0.0, double(a[4] - a[1]));
        double dz = std::max(0.0, double(a[5] - a[2]));
        total += dx * dy * dz;
    }
    return total;
}

// ─── Result record ───────────────────────────────────────────────────────

struct TrialResult {
    std::string robot;
    std::string width_regime;
    std::string variant;
    double volume_m3;
    double derive_us;
    int n_stage1_base;
    int n_stage1_coupled;
    int n_stage2_manifold;
    int n_stage3_lbfgs;
};

// ─── Main ────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    auto exp_cfg = parse_args(argc, argv);
    auto robots = find_robots();
    auto variants = build_variants();

    if (robots.empty()) return 1;

    constexpr int N_SUB = 1;  // Fixed at n_sub=1

    std::vector<TrialResult> all_results;
    std::mt19937 rng(12345);

    std::cout << std::string(70, '=') << "\n"
              << " Experiment 20: Ablation + Acceleration (n_sub=1)\n"
              << std::string(70, '=') << "\n"
              << " Trials: " << exp_cfg.n_trials
              << "  Repeats: " << exp_cfg.n_repeats
              << "  Robots: " << robots.size()
              << "  Variants: " << variants.size()
              << "\n\n";

    for (auto& rs : robots) {
        Robot robot = Robot::from_json(rs.json_path);
        const int n = robot.n_joints();
        const int n_act = robot.n_active_links();

        std::cout << "== Robot: " << rs.name
                  << " (" << n << "J, " << n_act << " links"
                  << ", " << robot.coupled_pairs().size() << " pairs"
                  << ", " << robot.coupled_triples().size() << " triples"
                  << ") ==\n\n";

        for (auto& wr : WIDTHS) {
            const int total_slots = n_act * N_SUB;

            // Per-variant accumulators
            struct Acc {
                std::vector<double> vols, times;
                CriticalStats last_stats{};
            };
            std::map<std::string, Acc> acc;

            for (int trial = 0; trial < exp_cfg.n_trials; ++trial) {
                // Generate random intervals
                std::vector<Interval> ivls(n);
                for (int j = 0; j < n; ++j) {
                    double range = robot.joint_limits().limits[j].hi
                                 - robot.joint_limits().limits[j].lo;
                    double width = std::uniform_real_distribution<double>(
                        wr.lo, wr.hi)(rng) * range;
                    double center = std::uniform_real_distribution<double>(
                        robot.joint_limits().limits[j].lo + width / 2,
                        robot.joint_limits().limits[j].hi - width / 2)(rng);
                    ivls[j].lo = center - width / 2;
                    ivls[j].hi = center + width / 2;
                }

                for (auto& var : variants) {
                    std::vector<float> aabb(total_slots * 6);

                    // Warm-up
                    CriticalStats stats{};
                    derive_aabb_critical_enhanced(
                        robot, ivls, N_SUB, var.config,
                        aabb.data(), &stats);

                    // Timed runs — take best
                    double best_us = 1e18;
                    for (int rep = 0; rep < exp_cfg.n_repeats; ++rep) {
                        auto t0 = Clock::now();
                        derive_aabb_critical_enhanced(
                            robot, ivls, N_SUB, var.config,
                            aabb.data(), nullptr);
                        auto t1 = Clock::now();
                        double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
                        best_us = std::min(best_us, us);
                    }

                    double vol = aabb_volume(aabb.data(), total_slots);

                    acc[var.short_name].vols.push_back(vol);
                    acc[var.short_name].times.push_back(best_us);
                    acc[var.short_name].last_stats = stats;

                    TrialResult tr;
                    tr.robot = rs.name;
                    tr.width_regime = wr.name;
                    tr.variant = var.short_name;
                    tr.volume_m3 = vol;
                    tr.derive_us = best_us;
                    tr.n_stage1_base = stats.n_stage1_base;
                    tr.n_stage1_coupled = stats.n_stage1_coupled;
                    tr.n_stage2_manifold = stats.n_stage2_manifold;
                    tr.n_stage3_lbfgs = stats.n_stage3_lbfgs;
                    all_results.push_back(tr);
                }
            }

            // Print summary
            std::cout << "  " << wr.name << "\n";
            std::cout << "  " << std::string(100, '-') << "\n";
            std::cout << "  " << std::left << std::setw(16) << "Variant"
                      << std::right
                      << std::setw(12) << "Vol(m3)"
                      << std::setw(12) << "Time(us)"
                      << std::setw(10) << "Speedup"
                      << std::setw(10) << "VolRatio"
                      << std::setw(10) << "S1base"
                      << std::setw(10) << "S1coup"
                      << std::setw(10) << "S2manif"
                      << std::setw(10) << "S3lbfgs"
                      << "\n";

            // Compute baseline and full_slow means
            double base_vol = 0, base_time = 0;
            double slow_time = 0;
            if (!acc["base"].vols.empty()) {
                for (double v : acc["base"].vols) base_vol += v;
                base_vol /= acc["base"].vols.size();
                for (double t : acc["base"].times) base_time += t;
                base_time /= acc["base"].times.size();
            }
            if (!acc["full_slow"].times.empty()) {
                for (double t : acc["full_slow"].times) slow_time += t;
                slow_time /= acc["full_slow"].times.size();
            }

            for (auto& var : variants) {
                auto& a = acc[var.short_name];
                if (a.vols.empty()) continue;

                double vol_mean = 0, time_mean = 0;
                for (double v : a.vols) vol_mean += v;
                for (double t : a.times) time_mean += t;
                vol_mean /= a.vols.size();
                time_mean /= a.times.size();

                double vol_ratio = (base_vol > 0) ? vol_mean / base_vol : 1.0;
                // Speedup: how many × faster than full_slow
                // For baseline, show slowdown ratio vs baseline
                double speedup = (slow_time > 0) ? slow_time / time_mean : 0.0;

                auto& st = a.last_stats;

                std::cout << "  " << std::left << std::setw(16) << var.name
                          << std::right << std::fixed << std::setprecision(6)
                          << std::setw(12) << vol_mean
                          << std::setprecision(1)
                          << std::setw(12) << time_mean
                          << std::setprecision(2)
                          << std::setw(10) << speedup << "x"
                          << std::setprecision(4)
                          << std::setw(9) << vol_ratio
                          << std::setw(10) << st.n_stage1_base
                          << std::setw(10) << st.n_stage1_coupled
                          << std::setw(10) << st.n_stage2_manifold
                          << std::setw(10) << st.n_stage3_lbfgs
                          << "\n";
            }
            std::cout << "\n";
        }
    }

    // ─── Write CSV ───────────────────────────────────────────────────

    std::string dir_path;
    if (!exp_cfg.output_dir.empty()) {
        dir_path = exp_cfg.output_dir;
    } else {
        auto now = std::chrono::system_clock::now();
        auto t = std::chrono::system_clock::to_time_t(now);
        std::tm tm_buf{};
#ifdef _WIN32
        localtime_s(&tm_buf, &t);
#else
        localtime_r(&t, &tm_buf);
#endif
        std::ostringstream oss;
        oss << "../results/exp20_accel_"
            << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
        dir_path = oss.str();
    }
    fs::create_directories(dir_path);
    std::string csv_path = dir_path + "/exp20_accel.csv";

    std::ofstream csv(csv_path);
    if (csv.is_open()) {
        csv << "robot,width,variant,volume_m3,derive_us,"
            << "n_stage1_base,n_stage1_coupled,n_stage2_manifold,n_stage3_lbfgs\n";
        for (auto& r : all_results) {
            csv << r.robot << "," << r.width_regime << ","
                << r.variant << "," << std::scientific << std::setprecision(8)
                << r.volume_m3 << "," << std::fixed << std::setprecision(1)
                << r.derive_us << ","
                << r.n_stage1_base << ","
                << r.n_stage1_coupled << ","
                << r.n_stage2_manifold << ","
                << r.n_stage3_lbfgs << "\n";
        }
        csv.close();
        std::cout << "CSV written to: " << csv_path << "\n";
    }

    std::cout << "\nDone. " << all_results.size() << " total trial results.\n";
    return 0;
}

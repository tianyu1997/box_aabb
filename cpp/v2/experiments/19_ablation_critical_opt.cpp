// ═══════════════════════════════════════════════════════════════════════════
// Experiment 19 — Ablation study: enhanced critical sampling optimisations
// ═══════════════════════════════════════════════════════════════════════════
//
// Tests The contribution of each optimisation stage ported from Python v4:
//
//   Variant A: Baseline        — original crit enumeration only
//   Variant B: +Coupled        — A + coupled pair/triple constraint enum
//   Variant C: +Manifold       — A + constraint manifold random sampling
//   Variant D: +L-BFGS-B       — A + L-BFGS-B local optimisation
//   Variant E: +Coupled+Manif  — A + B + C
//   Variant F: Full pipeline   — A + B + C + D  (all stages)
//
// For each variant, measures:
//   • AABB volume  (smaller = tighter envelope = better)
//   • Derive time  (wall-clock microseconds)
//   • Sample counts per stage
//
// Swept variables:
//   • Robot:     panda, iiwa14
//   • Width:     small (0.05-0.15), medium (0.15-0.50), large (0.50-1.50)
//   • n_sub:     1, 2, 4
//
// Build:
//   cmake .. -DSBF_BUILD_EXPERIMENTS=ON
//   cmake --build . --target exp_19_ablation_crit_opt --config Release
//
// Run:
//   ./exp_19_ablation_crit_opt [--trials 30] [--repeats 5]
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

// ─── Ablation variants ──────────────────────────────────────────────────

struct Variant {
    const char* name;
    const char* short_name;
    CriticalSamplingConfig config;
};

static std::vector<Variant> build_variants() {
    std::vector<Variant> vars;

    // A: Baseline (no enhancements — equivalent to original v2)
    {
        auto c = CriticalSamplingConfig::baseline();
        vars.push_back({"Baseline", "base", c});
    }
    // B: +Coupled constraints only
    {
        auto c = CriticalSamplingConfig::baseline();
        c.enable_coupled_constraints = true;
        vars.push_back({"+Coupled", "coupled", c});
    }
    // C: +Manifold sampling only
    {
        auto c = CriticalSamplingConfig::baseline();
        c.enable_manifold_sampling = true;
        vars.push_back({"+Manifold", "manifold", c});
    }
    // D: +L-BFGS-B only (from baseline seeds)
    {
        auto c = CriticalSamplingConfig::baseline();
        c.enable_lbfgs_optimization = true;
        vars.push_back({"+LBFGSB", "lbfgsb", c});
    }
    // E: +Coupled + Manifold (no optimisation)
    {
        auto c = CriticalSamplingConfig::baseline();
        c.enable_coupled_constraints = true;
        c.enable_manifold_sampling = true;
        vars.push_back({"+Coupled+Manifold", "coup_manif", c});
    }
    // F: Full pipeline (all three stages)
    {
        auto c = CriticalSamplingConfig::all_enabled();
        vars.push_back({"Full", "full", c});
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
    // Try common paths relative to build dir
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
    if (robots.empty()) {
        std::cerr << "Warning: No robot configs found. "
                  << "Please run from build/ dir.\n";
    }
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
    int n_sub;
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

    if (robots.empty()) {
        std::cerr << "No robots found. Aborting.\n";
        return 1;
    }

    std::vector<int> n_subs = {1, 2, 4};
    std::vector<TrialResult> all_results;

    std::mt19937 rng(12345);

    std::cout << "═══════════════════════════════════════════════════════════════\n"
              << " Experiment 19: Ablation — Enhanced Critical Sampling\n"
              << "═══════════════════════════════════════════════════════════════\n"
              << " Trials: " << exp_cfg.n_trials
              << "  Repeats: " << exp_cfg.n_repeats
              << "  Robots: " << robots.size()
              << "  Variants: " << variants.size()
              << "\n\n";

    for (auto& rs : robots) {
        Robot robot = Robot::from_json(rs.json_path);
        const int n = robot.n_joints();
        const int n_act = robot.n_active_links();

        std::cout << "── Robot: " << rs.name
                  << " (" << n << " joints, " << n_act << " active links"
                  << ", " << robot.coupled_pairs().size() << " coupled pairs"
                  << ", " << robot.coupled_triples().size() << " coupled triples"
                  << ") ──\n\n";

        for (auto& wr : WIDTHS) {
            for (int n_sub : n_subs) {
                const int total_slots = n_act * n_sub;

                // Per-variant accumulators
                std::map<std::string, std::vector<double>> vol_acc, time_acc;
                std::map<std::string, CriticalStats> stats_acc;

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

                        // Warm-up run
                        CriticalStats stats{};
                        derive_aabb_critical_enhanced(
                            robot, ivls, n_sub, var.config,
                            aabb.data(), &stats);

                        // Timed runs
                        double best_us = 1e18;
                        for (int rep = 0; rep < exp_cfg.n_repeats; ++rep) {
                            auto t0 = Clock::now();
                            derive_aabb_critical_enhanced(
                                robot, ivls, n_sub, var.config,
                                aabb.data(), nullptr);
                            auto t1 = Clock::now();
                            double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
                            best_us = std::min(best_us, us);
                        }

                        double vol = aabb_volume(aabb.data(), total_slots);

                        vol_acc[var.short_name].push_back(vol);
                        time_acc[var.short_name].push_back(best_us);
                        stats_acc[var.short_name] = stats;  // last trial stats

                        TrialResult tr;
                        tr.robot = rs.name;
                        tr.width_regime = wr.name;
                        tr.n_sub = n_sub;
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

                // Print summary for this (robot, width, n_sub) group
                std::cout << "  " << wr.name << " | n_sub=" << n_sub << "\n";
                std::cout << "  " << std::string(90, '-') << "\n";
                std::cout << "  " << std::left << std::setw(20) << "Variant"
                          << std::right
                          << std::setw(12) << "Vol(m³)"
                          << std::setw(12) << "Time(µs)"
                          << std::setw(10) << "S1base"
                          << std::setw(10) << "S1coup"
                          << std::setw(10) << "S2manif"
                          << std::setw(10) << "S3lbfgs"
                          << "\n";

                // Baseline volume for ratio calculation
                double base_vol_mean = 0;
                if (!vol_acc["base"].empty()) {
                    for (double v : vol_acc["base"]) base_vol_mean += v;
                    base_vol_mean /= vol_acc["base"].size();
                }

                for (auto& var : variants) {
                    auto& vols = vol_acc[var.short_name];
                    auto& times = time_acc[var.short_name];
                    if (vols.empty()) continue;

                    double vol_mean = 0, time_mean = 0;
                    for (double v : vols) vol_mean += v;
                    for (double t : times) time_mean += t;
                    vol_mean /= vols.size();
                    time_mean /= times.size();

                    double vol_ratio = (base_vol_mean > 0)
                        ? vol_mean / base_vol_mean : 1.0;

                    auto& st = stats_acc[var.short_name];

                    std::cout << "  " << std::left << std::setw(20) << var.name
                              << std::right << std::fixed << std::setprecision(6)
                              << std::setw(12) << vol_mean
                              << std::setprecision(1)
                              << std::setw(12) << time_mean
                              << std::setw(10) << st.n_stage1_base
                              << std::setw(10) << st.n_stage1_coupled
                              << std::setw(10) << st.n_stage2_manifold
                              << std::setw(10) << st.n_stage3_lbfgs
                              << "  (vol ratio: " << std::setprecision(4)
                              << vol_ratio << ")"
                              << "\n";
                }
                std::cout << "\n";
            }
        }
    }

    // ─── Write CSV ───────────────────────────────────────────────────

    // Determine output path
    std::string csv_path;
    if (!exp_cfg.output_dir.empty()) {
        fs::create_directories(exp_cfg.output_dir);
        csv_path = exp_cfg.output_dir + "/exp19_ablation.csv";
    } else {
        // Auto-create timestamped output dir
        auto now = std::chrono::system_clock::now();
        auto t = std::chrono::system_clock::to_time_t(now);
        std::tm tm_buf{};
#ifdef _WIN32
        localtime_s(&tm_buf, &t);
#else
        localtime_r(&t, &tm_buf);
#endif
        std::ostringstream oss;
        oss << "../results/exp19_ablation_"
            << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
        std::string dir = oss.str();
        fs::create_directories(dir);
        csv_path = dir + "/exp19_ablation.csv";
    }

    std::ofstream csv(csv_path);
    if (csv.is_open()) {
        csv << "robot,width,n_sub,variant,volume_m3,derive_us,"
            << "n_stage1_base,n_stage1_coupled,n_stage2_manifold,n_stage3_lbfgs\n";
        for (auto& r : all_results) {
            csv << r.robot << "," << r.width_regime << "," << r.n_sub << ","
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

    // ─── Write Markdown report ───────────────────────────────────────

    std::string md_path = csv_path.substr(0, csv_path.size() - 4) + "_report.md";
    std::ofstream md(md_path);
    if (md.is_open()) {
        md << "# Experiment 19 — Ablation: Enhanced Critical Sampling\n\n";
        md << "**Date**: " << __DATE__ << "  \n";
        md << "**Trials**: " << exp_cfg.n_trials
           << " | **Repeats**: " << exp_cfg.n_repeats << "\n\n";

        md << "## Ablation Variants\n\n";
        md << "| Variant | Coupled | Manifold | L-BFGS-B |\n";
        md << "|---------|---------|----------|----------|\n";
        md << "| A: Baseline        | ✗ | ✗ | ✗ |\n";
        md << "| B: +Coupled        | ✓ | ✗ | ✗ |\n";
        md << "| C: +Manifold       | ✗ | ✓ | ✗ |\n";
        md << "| D: +L-BFGS-B       | ✗ | ✗ | ✓ |\n";
        md << "| E: +Coupled+Manif  | ✓ | ✓ | ✗ |\n";
        md << "| F: Full pipeline   | ✓ | ✓ | ✓ |\n";
        md << "\n";

        md << "## Summary\n\n";
        md << "The full results are in the companion CSV file.\n\n";

        // Aggregate summary table
        md << "## Aggregate Results (mean over all trials)\n\n";

        // Group by (robot, variant)
        struct Agg { double vol_sum = 0, time_sum = 0; int count = 0; };
        std::map<std::string, Agg> agg;
        for (auto& r : all_results) {
            std::string key = r.robot + " | " + r.variant;
            agg[key].vol_sum += r.volume_m3;
            agg[key].time_sum += r.derive_us;
            agg[key].count++;
        }

        md << "| Robot | Variant | Mean Vol (m³) | Mean Time (µs) |\n";
        md << "|-------|---------|---------------|----------------|\n";
        for (auto& [key, a] : agg) {
            md << "| " << key << " | "
               << std::scientific << std::setprecision(4)
               << a.vol_sum / a.count << " | "
               << std::fixed << std::setprecision(1)
               << a.time_sum / a.count << " |\n";
        }
        md << "\n";

        md.close();
        std::cout << "Report written to: " << md_path << "\n";
    }

    std::cout << "\nDone. " << all_results.size() << " total trial results.\n";
    return 0;
}

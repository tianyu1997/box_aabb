// ═══════════════════════════════════════════════════════════════════════════
// Experiment 22 — Analytical Critical-Point Enumeration vs. Optimisation
// ═══════════════════════════════════════════════════════════════════════════
//
// Compares the ANALYTICAL gradient-zero enumeration approach (no iterative
// optimisation) against the OPTIMISATION-based pipeline from exp 20.
//
// Analytical phases:
//   Phase 0: kπ/2 baseline (boundary vertices)
//   Phase 1: 1D edge solve (atan2)       — exact, 2 candidates per edge-dim
//   Phase 2: 2D face solve (companion)   — exact, degree-8 polynomial
//   Phase 3: 3D+ interior  (coord sweep) — iterative 1D solves, convergent
//
// Variants:
//   A: Baseline (kπ/2 only)              ← same as exp20 baseline
//   B: +Edge (Phase 0 + Phase 1)
//   C: +Edge+Face (Phase 0..2)
//   D: Full Analytical (Phase 0..3)
//   E: Full Analytical (all pairs)       ← face_all_pairs=true
//   F: Full(fast) L-BFGS-B              ← exp20 best optimiser for reference
//   G: Coup+Manif only                  ← no optimiser, no analytical
//
// Build:
//   cmake --build . --target exp_22_analytical_crit --config Release
//
// Run:
//   ./exp_22_analytical_crit [--trials 30] [--repeats 5]
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

enum class MethodKind { ANALYTICAL, ENHANCED };

struct Variant {
    const char* name;
    const char* short_name;
    MethodKind kind;
    // One of these is used depending on kind
    AnalyticalCriticalConfig ana_config;
    CriticalSamplingConfig   enh_config;
};

static std::vector<Variant> build_variants() {
    std::vector<Variant> vars;

    // A: Baseline (kπ/2 only — via analytical with everything off)
    {
        AnalyticalCriticalConfig c;
        c.keep_kpi2_baseline = true;
        c.enable_edge_solve = false;
        c.enable_face_solve = false;
        c.enable_interior_solve = false;
        vars.push_back({"Baseline(kpi2)", "base", MethodKind::ANALYTICAL, c, {}});
    }

    // B: Phase 0 + Phase 1 (edges only)
    {
        auto c = AnalyticalCriticalConfig::edges_only();
        vars.push_back({"+Edge", "edge", MethodKind::ANALYTICAL, c, {}});
    }

    // C: Phase 0 + Phase 1 + Phase 2 (edges + faces, coupled pairs)
    {
        auto c = AnalyticalCriticalConfig::edges_and_faces();
        vars.push_back({"+Edge+Face", "edge_face", MethodKind::ANALYTICAL, c, {}});
    }

    // D: Full Analytical (all phases, coupled pairs only for faces)
    {
        auto c = AnalyticalCriticalConfig::all_enabled();
        vars.push_back({"FullAnalytical", "full_ana", MethodKind::ANALYTICAL, c, {}});
    }

    // E: Full Analytical with face_all_pairs=true
    {
        auto c = AnalyticalCriticalConfig::all_enabled();
        c.face_all_pairs = true;
        vars.push_back({"FullAna(allp)", "full_ana_ap", MethodKind::ANALYTICAL, c, {}});
    }

    // F: Full(fast) L-BFGS-B — reference from exp20
    {
        auto c = CriticalSamplingConfig::all_enabled();
        c.use_analytical_jacobian = true;
        c.smart_seed_selection    = true;
        c.restrict_joint_scope    = true;
        c.lbfgs_max_iter = 30;
        c.lbfgs_n_seeds  = 2;
        vars.push_back({"Full(fast)LBFGS", "lbfgs_fast", MethodKind::ENHANCED, {}, c});
    }

    // G: Coupled + Manifold only (cheapest enhancement)
    {
        auto c = CriticalSamplingConfig::baseline();
        c.enable_coupled_constraints = true;
        c.enable_manifold_sampling   = true;
        vars.push_back({"Coup+Manif", "coup_manif", MethodKind::ENHANCED, {}, c});
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
    // Analytical stats (filled for analytical variants)
    int n_p0_verts;
    int n_p1_edges;
    int n_p2_faces;
    int n_p3_interior;
    int fk_p1, fk_p2, fk_p3;
    // Enhanced stats (filled for optimisation variants)
    int n_s1_base, n_s1_coup, n_s2_manif, n_s3_lbfgs;
};

// ─── Main ────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    auto exp_cfg = parse_args(argc, argv);
    auto robots = find_robots();
    auto variants = build_variants();

    if (robots.empty()) return 1;

    constexpr int N_SUB = 1;

    std::vector<TrialResult> all_results;
    std::mt19937 rng(12345);

    std::cout << std::string(78, '=') << "\n"
              << " Experiment 22: Analytical Critical-Point Enumeration vs. Optimisation\n"
              << std::string(78, '=') << "\n"
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

            struct Acc {
                std::vector<double> vols, times;
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

                    TrialResult tr{};
                    tr.robot = rs.name;
                    tr.width_regime = wr.name;
                    tr.variant = var.short_name;

                    if (var.kind == MethodKind::ANALYTICAL) {
                        // Warm-up + stats
                        AnalyticalCriticalStats ast{};
                        derive_aabb_critical_analytical(
                            robot, ivls, N_SUB, var.ana_config,
                            aabb.data(), &ast);

                        // Timed runs
                        double best_us = 1e18;
                        for (int rep = 0; rep < exp_cfg.n_repeats; ++rep) {
                            auto t0 = Clock::now();
                            derive_aabb_critical_analytical(
                                robot, ivls, N_SUB, var.ana_config,
                                aabb.data(), nullptr);
                            auto t1 = Clock::now();
                            double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
                            best_us = std::min(best_us, us);
                        }

                        tr.volume_m3 = aabb_volume(aabb.data(), total_slots);
                        tr.derive_us = best_us;
                        tr.n_p0_verts = ast.n_phase0_vertices;
                        tr.n_p1_edges = ast.n_phase1_edges;
                        tr.n_p2_faces = ast.n_phase2_faces;
                        tr.n_p3_interior = ast.n_phase3_interior;
                        tr.fk_p1 = ast.n_phase1_fk_calls;
                        tr.fk_p2 = ast.n_phase2_fk_calls;
                        tr.fk_p3 = ast.n_phase3_fk_calls;

                    } else {
                        // Enhanced (optimisation) pipeline
                        CriticalStats cst{};
                        derive_aabb_critical_enhanced(
                            robot, ivls, N_SUB, var.enh_config,
                            aabb.data(), &cst);

                        double best_us = 1e18;
                        for (int rep = 0; rep < exp_cfg.n_repeats; ++rep) {
                            auto t0 = Clock::now();
                            derive_aabb_critical_enhanced(
                                robot, ivls, N_SUB, var.enh_config,
                                aabb.data(), nullptr);
                            auto t1 = Clock::now();
                            double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
                            best_us = std::min(best_us, us);
                        }

                        tr.volume_m3 = aabb_volume(aabb.data(), total_slots);
                        tr.derive_us = best_us;
                        tr.n_s1_base = cst.n_stage1_base;
                        tr.n_s1_coup = cst.n_stage1_coupled;
                        tr.n_s2_manif = cst.n_stage2_manifold;
                        tr.n_s3_lbfgs = cst.n_stage3_lbfgs;
                    }

                    acc[var.short_name].vols.push_back(tr.volume_m3);
                    acc[var.short_name].times.push_back(tr.derive_us);
                    all_results.push_back(tr);
                }
            }

            // ──── Print summary table ──────────────────────────
            std::cout << "  " << wr.name << "\n";
            std::cout << "  " << std::string(110, '-') << "\n";
            std::cout << "  " << std::left << std::setw(18) << "Variant"
                      << std::right
                      << std::setw(12) << "Vol(m3)"
                      << std::setw(12) << "Time(us)"
                      << std::setw(10) << "VolRatio"
                      << std::setw(10) << "Speedup"
                      << "\n";

            double base_vol = 0, base_time = 0;
            if (!acc["base"].vols.empty()) {
                for (double v : acc["base"].vols) base_vol += v;
                base_vol /= acc["base"].vols.size();
                for (double t : acc["base"].times) base_time += t;
                base_time /= acc["base"].times.size();
            }
            double lbfgs_time = 0;
            if (!acc["lbfgs_fast"].times.empty()) {
                for (double t : acc["lbfgs_fast"].times) lbfgs_time += t;
                lbfgs_time /= acc["lbfgs_fast"].times.size();
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
                double speedup = (lbfgs_time > 0) ? lbfgs_time / time_mean : 0.0;

                std::cout << "  " << std::left << std::setw(18) << var.name
                          << std::right << std::fixed << std::setprecision(6)
                          << std::setw(12) << vol_mean
                          << std::setprecision(1)
                          << std::setw(12) << time_mean
                          << std::setprecision(4)
                          << std::setw(10) << vol_ratio
                          << std::setprecision(2)
                          << std::setw(9) << speedup << "x"
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
        oss << "../results/exp22_analytical_"
            << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
        dir_path = oss.str();
    }
    fs::create_directories(dir_path);
    std::string csv_path = dir_path + "/exp22_analytical.csv";

    std::ofstream csv(csv_path);
    if (csv.is_open()) {
        csv << "robot,width,variant,volume_m3,derive_us,"
            << "n_p0_verts,n_p1_edges,n_p2_faces,n_p3_interior,"
            << "fk_p1,fk_p2,fk_p3,"
            << "n_s1_base,n_s1_coup,n_s2_manif,n_s3_lbfgs\n";
        for (auto& r : all_results) {
            csv << r.robot << "," << r.width_regime << ","
                << r.variant << ","
                << std::scientific << std::setprecision(8) << r.volume_m3 << ","
                << std::fixed << std::setprecision(1) << r.derive_us << ","
                << r.n_p0_verts << ","
                << r.n_p1_edges << ","
                << r.n_p2_faces << ","
                << r.n_p3_interior << ","
                << r.fk_p1 << ","
                << r.fk_p2 << ","
                << r.fk_p3 << ","
                << r.n_s1_base << ","
                << r.n_s1_coup << ","
                << r.n_s2_manif << ","
                << r.n_s3_lbfgs << "\n";
        }
        csv.close();
        std::cout << "CSV written to: " << csv_path << "\n";
    }

    std::cout << "\nDone. " << all_results.size() << " total trial results.\n";
    return 0;
}

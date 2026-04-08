// ═══════════════════════════════════════════════════════════════════════════
// Experiment 25 — 改进的关键采样策略 (Pair-Constrained Analytical)
// ═══════════════════════════════════════════════════════════════════════════
//
// 基于 Exp-24 的分析结果，测试新的 Phase 2.5（pair-constrained 求解器）
// 和 improved Phase 3（multi-start + pair-coupled sweep）相比原策略的提升。
//
// 对比组：
//   A: v1_analytical  — Phase 0+1+2+3 (original)
//   B: all_enabled    — Phase 0+1+2+2.5+3 (new, with improved interior)
//   C: pair_1d_only   — Phase 0+1+2+2.5a+3 (only pair-constrained 1D)
//   D: pair_2d_only   — Phase 0+1+2+2.5b+3 (only pair-constrained 2D)
//   E: dense_sample   — 100k random dense-sample baseline (ground truth approx)
//
// 指标：per-face 极值偏差、AABB 体积误差、FK 调用次数、运行时间
//
// Build:
//   cmake --build . --target exp_25_improved_analytical --config Release
//
// ═══════════════════════════════════════════════════════════════════════════

#define _USE_MATH_DEFINES

#include "sbf/robot/robot.h"
#include "sbf/robot/fk.h"
#include "sbf/envelope/envelope_derive_critical.h"

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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace sbf;
using namespace sbf::envelope;
using Clock = std::chrono::high_resolution_clock;

// ═══════════════════════════════════════════════════════════════════════════
//  Dense random sampling — approximate ground truth
// ═══════════════════════════════════════════════════════════════════════════
static void dense_sample_aabb(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub, int n_samples,
    float* out_aabb)
{
    const int n = robot.n_joints();
    const int n_act = robot.n_active_links();
    const int* map = robot.active_link_map();
    const double* rad = robot.active_link_radii();
    const float inv_n = 1.0f / (float)n_sub;

    // Init to empty
    for (int ci = 0; ci < n_act; ++ci) {
        for (int s = 0; s < n_sub; ++s) {
            float* a = out_aabb + (ci * n_sub + s) * 6;
            a[0] = a[1] = a[2] =  1e30f;
            a[3] = a[4] = a[5] = -1e30f;
        }
    }

    std::mt19937 rng(42);
    Eigen::VectorXd q(n);

    for (int i = 0; i < n_samples; ++i) {
        for (int j = 0; j < n; ++j) {
            std::uniform_real_distribution<double> dist(intervals[j].lo, intervals[j].hi);
            q[j] = dist(rng);
        }
        auto pos = fk_link_positions(robot, q);
        const int np = (int)pos.size();
        for (int ci = 0; ci < n_act; ++ci) {
            int V = map[ci];
            if (V + 1 >= np) continue;
            const auto& p_prox = pos[V];
            const auto& p_dist = pos[V + 1];
            for (int s = 0; s < n_sub; ++s) {
                float t0 = s * inv_n, t1 = (s + 1) * inv_n;
                Eigen::Vector3d s0 = p_prox + (double)t0 * (p_dist - p_prox);
                Eigen::Vector3d s1 = p_prox + (double)t1 * (p_dist - p_prox);
                float* a = out_aabb + (ci * n_sub + s) * 6;
                for (int d = 0; d < 3; ++d) {
                    float vmin = (float)std::min(s0[d], s1[d]);
                    float vmax = (float)std::max(s0[d], s1[d]);
                    if (vmin < a[d])     a[d]     = vmin;
                    if (vmax > a[d + 3]) a[d + 3] = vmax;
                }
            }
        }
    }

    // Inflate by link_radii
    for (int ci = 0; ci < n_act; ++ci) {
        float r = rad ? (float)rad[ci] : 0.f;
        for (int s = 0; s < n_sub; ++s) {
            float* a = out_aabb + (ci * n_sub + s) * 6;
            a[0] -= r; a[1] -= r; a[2] -= r;
            a[3] += r; a[4] += r; a[5] += r;
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Per-face error metrics
// ═══════════════════════════════════════════════════════════════════════════
struct FaceErrors {
    int total_faces = 0;
    int n_misses = 0;          // test is STRICTLY inside ground-truth (missed extreme)
    double sum_gap = 0.0;      // sum of |test - gt| for misses
    double max_gap = 0.0;
    double sum_vol_ratio = 0.0;
};

static FaceErrors compute_face_errors(
    const float* test_aabb,
    const float* gt_aabb,
    int n_slots)
{
    FaceErrors e;
    for (int i = 0; i < n_slots; ++i) {
        const float* t = test_aabb + i * 6;
        const float* g = gt_aabb + i * 6;

        // Check each of 6 faces: lo-faces (0,1,2) and hi-faces (3,4,5)
        for (int f = 0; f < 6; ++f) {
            ++e.total_faces;
            double gap;
            if (f < 3) {
                // lo face: test should be <= gt  (test more conservative)
                gap = (double)t[f] - (double)g[f];  // positive → miss
            } else {
                // hi face: test should be >= gt
                gap = (double)g[f] - (double)t[f];  // positive → miss
            }
            if (gap > 1e-8) {
                ++e.n_misses;
                e.sum_gap += gap;
                e.max_gap = std::max(e.max_gap, gap);
            }
        }

        // Volume ratio
        double vol_t = 1.0, vol_g = 1.0;
        for (int d = 0; d < 3; ++d) {
            vol_t *= std::max(0.0, (double)(t[d+3] - t[d]));
            vol_g *= std::max(0.0, (double)(g[d+3] - g[d]));
        }
        if (vol_g > 1e-20)
            e.sum_vol_ratio += vol_t / vol_g;
    }
    return e;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Main
// ═══════════════════════════════════════════════════════════════════════════
int main(int argc, char** argv) {
    int n_trials = 30;
    int dense_samples = 200000;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--trials" && i+1 < argc)
            n_trials = std::atoi(argv[++i]);
        if (arg == "--dense" && i+1 < argc)
            dense_samples = std::atoi(argv[++i]);
    }

    // ── Robot configs ────────────────────────────────────────────────
    struct RobotCase {
        std::string name;
        Robot robot;
    };
    std::vector<RobotCase> robots;

    // Find robot JSON configs
    std::vector<std::string> search_paths = {
        "../../../safeboxforest/v1/configs",
        "../../safeboxforest/v1/configs",
        "../safeboxforest/v1/configs",
        "../../../../safeboxforest/v1/configs",
    };
    for (auto& base : search_paths) {
        namespace fs = std::filesystem;
        std::string panda_path = base + "/panda.json";
        std::string iiwa_path  = base + "/iiwa14.json";
        if (fs::exists(panda_path)) {
            robots.push_back({"panda", Robot::from_json(panda_path)});
            if (fs::exists(iiwa_path))
                robots.push_back({"iiwa14", Robot::from_json(iiwa_path)});
            break;
        }
    }
    if (robots.empty()) {
        std::cerr << "No robot configs loaded!\n";
        return 1;
    }

    std::vector<double> widths = {0.4, 0.6, 1.0};
    const int n_sub = 1;

    // ── Method configs ───────────────────────────────────────────────
    struct Method {
        std::string name;
        AnalyticalCriticalConfig cfg;
    };

    // A: Original (v1 analytical)
    Method methodA;
    methodA.name = "v1_analytical";
    methodA.cfg = AnalyticalCriticalConfig::v1_analytical();

    // B: Full improved (all new phases)
    Method methodB;
    methodB.name = "v2_full";
    methodB.cfg = AnalyticalCriticalConfig::all_enabled();

    // C: Only pair-constrained 1D (no 2D, no improved interior)
    Method methodC;
    methodC.name = "v2_pair1d_only";
    methodC.cfg = AnalyticalCriticalConfig::all_enabled();
    methodC.cfg.enable_pair_2d = false;
    methodC.cfg.improved_interior = false;

    // D: Phase 0+1+2 + pair 1D+2D (no interior at all)
    Method methodD;
    methodD.name = "v2_no_interior";
    methodD.cfg = AnalyticalCriticalConfig::all_enabled();
    methodD.cfg.enable_interior_solve = false;

    // E: v2 full with kpi2 backgrounds disabled
    Method methodE;
    methodE.name = "v2_no_kpi2_bg";
    methodE.cfg = AnalyticalCriticalConfig::all_enabled();
    methodE.cfg.pair_kpi2_backgrounds = false;

    std::vector<Method> methods = {methodA, methodB, methodC, methodD, methodE};

    // ── Results storage ──────────────────────────────────────────────
    struct Result {
        std::string robot, method;
        double width;
        int trial;
        int n_misses;
        double max_gap, avg_gap;
        double vol_ratio;
        double time_ms;
        int fk_total;
        AnalyticalCriticalStats stats;
    };
    std::vector<Result> results;

    std::mt19937 rng(12345);

    std::cout << "═══════════════════════════════════════════════════════════\n";
    std::cout << " Experiment 25 — Improved Analytical Critical Sampling\n";
    std::cout << "═══════════════════════════════════════════════════════════\n";
    std::cout << "Trials: " << n_trials << ", Dense samples: " << dense_samples << "\n";
    std::cout << "Methods: " << methods.size() << "\n\n";

    for (auto& rc : robots) {
        const Robot& robot = rc.robot;
        const int n = robot.n_joints();
        const int n_act = robot.n_active_links();
        const int total_slots = n_act * n_sub;

        for (double width : widths) {
            std::cout << "─── " << rc.name << " width=" << width << " ───\n";

            for (int trial = 0; trial < n_trials; ++trial) {
                // Random intervals
                std::vector<Interval> intervals(n);
                for (int j = 0; j < n; ++j) {
                    std::uniform_real_distribution<double> dist(-M_PI, M_PI - width);
                    double lo = dist(rng);
                    intervals[j] = {lo, lo + width};
                }

                // Ground truth: dense sampling
                std::vector<float> gt_aabb(total_slots * 6);
                dense_sample_aabb(robot, intervals, n_sub, dense_samples, gt_aabb.data());

                // Run each method
                for (auto& method : methods) {
                    std::vector<float> test_aabb(total_slots * 6);
                    AnalyticalCriticalStats stats{};

                    auto t0 = Clock::now();
                    derive_aabb_critical_analytical(
                        robot, intervals, n_sub, method.cfg,
                        test_aabb.data(), &stats);
                    auto t1 = Clock::now();
                    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

                    auto err = compute_face_errors(test_aabb.data(), gt_aabb.data(), total_slots);

                    int fk_total = stats.n_phase0_vertices
                                 + stats.n_phase1_fk_calls
                                 + stats.n_phase2_fk_calls
                                 + stats.n_phase25_fk_calls
                                 + stats.n_phase3_fk_calls;

                    Result r;
                    r.robot = rc.name;
                    r.method = method.name;
                    r.width = width;
                    r.trial = trial;
                    r.n_misses = err.n_misses;
                    r.max_gap = err.max_gap;
                    r.avg_gap = (err.n_misses > 0) ? err.sum_gap / err.n_misses : 0.0;
                    r.vol_ratio = (total_slots > 0) ? err.sum_vol_ratio / total_slots : 0.0;
                    r.time_ms = ms;
                    r.fk_total = fk_total;
                    r.stats = stats;
                    results.push_back(r);
                }
            }

            // Print per-method summary for this robot+width
            std::cout << std::fixed << std::setprecision(4);
            std::cout << "  Method              | Misses  AvgGap   MaxGap   VolRatio  Time(ms)   "
                      << "P0      P1     P2    P2.5a  P2.5b  P3     FKtot\n";
            std::cout << "  ─────────────────── + ─────── ──────── ──────── ──────── ────────── "
                      << "─────── ────── ────── ────── ────── ────── ───────\n";

            for (auto& method : methods) {
                int cnt = 0;
                double sum_miss = 0, sum_avg_gap = 0, sum_max_gap = 0;
                double sum_vol = 0, sum_time = 0;
                double sum_p0 = 0, sum_p1 = 0, sum_p2 = 0;
                double sum_p25a = 0, sum_p25b = 0, sum_p3 = 0;
                double sum_fk = 0;

                for (auto& r : results) {
                    if (r.robot == rc.name && r.method == method.name &&
                        std::abs(r.width - width) < 1e-6) {
                        ++cnt;
                        sum_miss += r.n_misses;
                        sum_avg_gap += r.avg_gap;
                        sum_max_gap += r.max_gap;
                        sum_vol += r.vol_ratio;
                        sum_time += r.time_ms;
                        sum_p0 += r.stats.n_phase0_vertices;
                        sum_p1 += r.stats.n_phase1_edges;
                        sum_p2 += r.stats.n_phase2_faces;
                        sum_p25a += r.stats.n_phase25a_pair1d;
                        sum_p25b += r.stats.n_phase25b_pair2d;
                        sum_p3 += r.stats.n_phase3_interior;
                        sum_fk += r.fk_total;
                    }
                }
                if (cnt == 0) continue;
                double inv = 1.0 / cnt;
                std::cout << "  " << std::left << std::setw(20) << method.name
                          << std::right
                          << std::setw(6) << (sum_miss * inv)
                          << std::setw(9) << std::setprecision(6) << (sum_avg_gap * inv)
                          << std::setw(9) << std::setprecision(6) << (sum_max_gap * inv)
                          << std::setw(9) << std::setprecision(4) << (sum_vol * inv)
                          << std::setw(10) << std::setprecision(2) << (sum_time * inv)
                          << std::setw(8) << (int)(sum_p0 * inv)
                          << std::setw(7) << (int)(sum_p1 * inv)
                          << std::setw(7) << (int)(sum_p2 * inv)
                          << std::setw(7) << (int)(sum_p25a * inv)
                          << std::setw(7) << (int)(sum_p25b * inv)
                          << std::setw(7) << (int)(sum_p3 * inv)
                          << std::setw(8) << (int)(sum_fk * inv)
                          << "\n";
            }
            std::cout << "\n";
        }
    }

    // ── Write CSV ────────────────────────────────────────────────────
    namespace fs = std::filesystem;
    auto now = std::chrono::system_clock::now();
    auto tt = std::chrono::system_clock::to_time_t(now);
    std::tm ltm;
#if defined(_MSC_VER)
    localtime_s(&ltm, &tt);
#else
    localtime_r(&tt, &ltm);
#endif
    char ts[64];
    std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &ltm);

    std::string dir = std::string("results/exp25_improved_") + ts;
    fs::create_directories(dir);
    std::string csv_path = dir + "/exp25_results.csv";

    std::ofstream csv(csv_path);
    csv << "robot,method,width,trial,n_misses,avg_gap,max_gap,vol_ratio,time_ms,"
        << "n_p0,n_p1,n_p2,n_p25a,n_p25b,n_p3,fk_total,fk_p1,fk_p2,fk_p25,fk_p3\n";
    for (auto& r : results) {
        csv << r.robot << "," << r.method << "," << r.width << "," << r.trial
            << "," << r.n_misses << "," << r.avg_gap << "," << r.max_gap
            << "," << r.vol_ratio << "," << r.time_ms
            << "," << r.stats.n_phase0_vertices
            << "," << r.stats.n_phase1_edges
            << "," << r.stats.n_phase2_faces
            << "," << r.stats.n_phase25a_pair1d
            << "," << r.stats.n_phase25b_pair2d
            << "," << r.stats.n_phase3_interior
            << "," << r.fk_total
            << "," << r.stats.n_phase1_fk_calls
            << "," << r.stats.n_phase2_fk_calls
            << "," << r.stats.n_phase25_fk_calls
            << "," << r.stats.n_phase3_fk_calls
            << "\n";
    }
    csv.close();

    std::cout << "\n═══════════════════════════════════════════════════════════\n";
    std::cout << " CSV saved to: " << csv_path << "\n";
    std::cout << "═══════════════════════════════════════════════════════════\n";

    return 0;
}

// ═══════════════════════════════════════════════════════════════════════════
// Experiment 27 — v1 Analytical Phase Ablation Study
// ═══════════════════════════════════════════════════════════════════════════
//
// 对 v1_analytical 的 4 个阶段 (Phase 0/1/2/3) 进行消融实验。
// 通过逐步增加阶段和逐个去除阶段两种方式，量化每个阶段的贡献。
//
// 增量组（逐步添加）：
//   P0_only        — Phase 0 (kπ/2 baseline only)
//   P0_P1          — Phase 0 + Phase 1 (+ edge solve)
//   P0_P1_P2       — Phase 0 + Phase 1 + Phase 2 (+ face solve)
//   P0_P1_P2_P3    — Phase 0 + Phase 1 + Phase 2 + Phase 3 (full v1)
//
// 消融组（逐个去除）：
//   no_P0          — Phase 1 + 2 + 3 (去除 kπ/2 baseline)
//   no_P1          — Phase 0 + 2 + 3 (去除 edge solve)
//   no_P2          — Phase 0 + 1 + 3 (去除 face solve)
//   no_P3          — Phase 0 + 1 + 2 (去除 interior sweep)
//
// 指标：miss count, max gap, avg gap, vol ratio, 运行时间, FK 调用数
//
// Build:
//   cmake --build . --target exp_27_v1_phase_ablation --config Release
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
//  Dense random sampling — ground truth
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

    for (int ci = 0; ci < n_act; ++ci)
        for (int s = 0; s < n_sub; ++s) {
            float* a = out_aabb + (ci * n_sub + s) * 6;
            a[0] = a[1] = a[2] =  1e30f;
            a[3] = a[4] = a[5] = -1e30f;
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
    int n_misses = 0;
    double sum_gap = 0.0;
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
        const float* g = gt_aabb   + i * 6;

        for (int f = 0; f < 6; ++f) {
            ++e.total_faces;
            double gap;
            if (f < 3)
                gap = (double)t[f] - (double)g[f];   // lo: positive → miss
            else
                gap = (double)g[f] - (double)t[f];   // hi: positive → miss
            if (gap > 1e-8) {
                ++e.n_misses;
                e.sum_gap += gap;
                e.max_gap = std::max(e.max_gap, gap);
            }
        }

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
//  Method configuration builder
// ═══════════════════════════════════════════════════════════════════════════
struct Method {
    std::string name;
    AnalyticalCriticalConfig cfg;
};

static AnalyticalCriticalConfig make_v1_base() {
    // Start with everything disabled (no pair-constrained, no improved interior)
    AnalyticalCriticalConfig c;
    c.enable_pair_1d = false;
    c.enable_pair_2d = false;
    c.improved_interior = false;
    // Disable all 4 phases by default — caller turns on what's needed
    c.keep_kpi2_baseline    = false;
    c.enable_edge_solve     = false;
    c.enable_face_solve     = false;
    c.enable_interior_solve = false;
    return c;
}

static std::vector<Method> build_methods() {
    std::vector<Method> methods;

    // ── Incremental group: accumulate phases ──
    {
        auto c = make_v1_base();
        c.keep_kpi2_baseline = true;
        methods.push_back({"P0_only", c});
    }
    {
        auto c = make_v1_base();
        c.keep_kpi2_baseline = true;
        c.enable_edge_solve  = true;
        methods.push_back({"P0_P1", c});
    }
    {
        auto c = make_v1_base();
        c.keep_kpi2_baseline = true;
        c.enable_edge_solve  = true;
        c.enable_face_solve  = true;
        methods.push_back({"P0_P1_P2", c});
    }
    {
        // Full v1: all 4 phases
        auto c = AnalyticalCriticalConfig::v1_analytical();
        methods.push_back({"P0_P1_P2_P3", c});
    }

    // ── Selective combination: P0+P3 (skip edges and faces) ──
    {
        auto c = make_v1_base();
        c.keep_kpi2_baseline    = true;
        c.enable_interior_solve = true;
        methods.push_back({"P0_P3", c});
    }

    // ── Ablation group: remove one phase ──
    {
        auto c = AnalyticalCriticalConfig::v1_analytical();
        c.keep_kpi2_baseline = false;
        methods.push_back({"no_P0", c});
    }
    {
        auto c = AnalyticalCriticalConfig::v1_analytical();
        c.enable_edge_solve = false;
        methods.push_back({"no_P1", c});
    }
    {
        auto c = AnalyticalCriticalConfig::v1_analytical();
        c.enable_face_solve = false;
        methods.push_back({"no_P2", c});
    }
    {
        auto c = AnalyticalCriticalConfig::v1_analytical();
        c.enable_interior_solve = false;
        methods.push_back({"no_P3", c});
    }

    return methods;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Main
// ═══════════════════════════════════════════════════════════════════════════
int main(int argc, char** argv) {
    int n_trials = 30;
    int dense_samples = 300000;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--trials" && i+1 < argc)
            n_trials = std::atoi(argv[++i]);
        if (arg == "--dense" && i+1 < argc)
            dense_samples = std::atoi(argv[++i]);
    }

    // ── Robot configs ────────────────────────────────────────────────
    struct RobotCase { std::string name; Robot robot; };
    std::vector<RobotCase> robots;

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
        std::cerr << "No robot configs found!\n";
        return 1;
    }

    std::vector<double> widths = {0.4, 0.6, 1.0};
    const int n_sub = 1;

    auto methods = build_methods();

    // ── Results storage ──────────────────────────────────────────────
    struct Result {
        std::string robot, method;
        double width;
        int trial;
        int n_misses;
        double max_gap, avg_gap;
        double vol_ratio;
        double time_ms;
        AnalyticalCriticalStats stats;
    };
    std::vector<Result> results;

    std::mt19937 rng(12345);

    std::cout << "=============================================================\n";
    std::cout << " Experiment 27 - v1 Analytical Phase Ablation\n";
    std::cout << "=============================================================\n";
    std::cout << "Trials: " << n_trials
              << ", Dense samples: " << dense_samples << "\n";
    std::cout << "Methods (" << methods.size() << "): ";
    for (auto& m : methods) std::cout << m.name << " ";
    std::cout << "\n\n";

    for (auto& rc : robots) {
        const Robot& robot = rc.robot;
        const int n = robot.n_joints();
        const int n_act = robot.n_active_links();
        const int total_slots = n_act * n_sub;

        for (double width : widths) {
            std::cout << "--- " << rc.name << " width="
                      << std::fixed << std::setprecision(1) << width << " ---\n";

            for (int trial = 0; trial < n_trials; ++trial) {
                // Random intervals
                std::vector<Interval> intervals(n);
                for (int j = 0; j < n; ++j) {
                    std::uniform_real_distribution<double> dist(-M_PI, M_PI - width);
                    double lo = dist(rng);
                    intervals[j] = {lo, lo + width};
                }

                // Ground truth
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
                    r.stats = stats;
                    results.push_back(r);
                }
            }

            // ── Print per-method summary ─────────────────────────────
            std::cout << std::fixed;
            std::cout << "  Method         | Misses  AvgGap     MaxGap     VolRatio  "
                      << "Time(ms)  P0     P1     P2     P3     FK_tot\n";
            std::cout << "  --------------- + ------ ---------- ---------- --------- "
                      << "--------- ------ ------ ------ ------ --------\n";

            for (auto& method : methods) {
                int cnt = 0;
                double sum_miss = 0, sum_avg = 0, sum_max = 0;
                double sum_vol = 0, sum_time = 0;
                double sum_p0 = 0, sum_p1 = 0, sum_p2 = 0, sum_p3 = 0;
                double sum_fk = 0;

                for (auto& r : results) {
                    if (r.robot != rc.name || r.method != method.name ||
                        std::abs(r.width - width) > 1e-6)
                        continue;
                    ++cnt;
                    sum_miss += r.n_misses;
                    sum_avg  += r.avg_gap;
                    sum_max  += r.max_gap;
                    sum_vol  += r.vol_ratio;
                    sum_time += r.time_ms;
                    sum_p0   += r.stats.n_phase0_vertices;
                    sum_p1   += r.stats.n_phase1_edges;
                    sum_p2   += r.stats.n_phase2_faces;
                    sum_p3   += r.stats.n_phase3_interior;
                    sum_fk   += r.stats.n_phase0_vertices
                              + r.stats.n_phase1_fk_calls
                              + r.stats.n_phase2_fk_calls
                              + r.stats.n_phase3_fk_calls;
                }
                if (cnt == 0) continue;
                double inv = 1.0 / cnt;
                std::cout << "  " << std::left << std::setw(16) << method.name
                          << std::right
                          << std::setw(7) << std::setprecision(3) << (sum_miss * inv)
                          << std::setw(11) << std::setprecision(7) << (sum_avg * inv)
                          << std::setw(11) << std::setprecision(7) << (sum_max * inv)
                          << std::setw(10) << std::setprecision(5) << (sum_vol * inv)
                          << std::setw(10) << std::setprecision(2) << (sum_time * inv)
                          << std::setw(7) << (int)(sum_p0 * inv)
                          << std::setw(7) << (int)(sum_p1 * inv)
                          << std::setw(7) << (int)(sum_p2 * inv)
                          << std::setw(7) << (int)(sum_p3 * inv)
                          << std::setw(9) << (int)(sum_fk * inv)
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

    std::string dir = std::string("results/exp27_v1_ablation_") + ts;
    fs::create_directories(dir);
    std::string csv_path = dir + "/exp27_results.csv";

    std::ofstream csv(csv_path);
    csv << "robot,method,width,trial,"
        << "n_misses,avg_gap,max_gap,vol_ratio,time_ms,"
        << "n_p0,n_p1,n_p2,n_p3,"
        << "fk_p0,fk_p1,fk_p2,fk_p3,fk_total\n";
    for (auto& r : results) {
        int fk_total = r.stats.n_phase0_vertices
                     + r.stats.n_phase1_fk_calls
                     + r.stats.n_phase2_fk_calls
                     + r.stats.n_phase3_fk_calls;
        csv << r.robot << "," << r.method << ","
            << r.width << "," << r.trial << ","
            << r.n_misses << "," << r.avg_gap << "," << r.max_gap << ","
            << r.vol_ratio << "," << r.time_ms << ","
            << r.stats.n_phase0_vertices << ","
            << r.stats.n_phase1_edges << ","
            << r.stats.n_phase2_faces << ","
            << r.stats.n_phase3_interior << ","
            << r.stats.n_phase0_vertices << ","
            << r.stats.n_phase1_fk_calls << ","
            << r.stats.n_phase2_fk_calls << ","
            << r.stats.n_phase3_fk_calls << ","
            << fk_total << "\n";
    }
    csv.close();

    std::cout << "\n=============================================================\n";
    std::cout << " CSV saved to: " << csv_path << "\n";
    std::cout << "=============================================================\n";

    return 0;
}

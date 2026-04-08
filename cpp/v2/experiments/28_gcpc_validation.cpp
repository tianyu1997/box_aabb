// ═══════════════════════════════════════════════════════════════════════════
// Experiment 28 — GCPC (Global Critical Point Cache) Validation
// ═══════════════════════════════════════════════════════════════════════════
//
// 验证 GCPC 缓存查询方法与现有方法的精度/速度对比。
//
// GCPC 方法：
//   gcpc_full    — Cache 查询 + 边界 kπ/2 + 边界 atan2
//   gcpc_noatan  — Cache 查询 + 边界 kπ/2 (无 Phase C)
//
// 对比基线：
//   v1_full      — v1 全 4 阶段 (Phase 0+1+2+3)
//   no_P2        — v1 去除 Phase 2 (Phase 0+1+3)
//   P0_only      — 仅 Phase 0 (kπ/2 baseline)
//
// 指标：miss count, max gap, avg gap, vol ratio, 运行时间, FK 调用数
//
// 分为两种模式：
//   1. 从 JSON 加载 GCPC 缓存 (需要先运行 precompute_gcpc.jl)
//   2. 备用模式：用 v1 Phase 0+1 生成伪缓存 (无需 Julia)
//
// Build:
//   cmake --build . --target exp_28_gcpc_validation --config Release
//
// Usage:
//   exp_28_gcpc_validation [--cache path/to/robot_gcpc.json]
//
// ═══════════════════════════════════════════════════════════════════════════

#define _USE_MATH_DEFINES

#include "sbf/robot/robot.h"
#include "sbf/robot/fk.h"
#include "sbf/envelope/envelope_derive_critical.h"
#include "sbf/envelope/gcpc_cache.h"

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
                gap = (double)t[f] - (double)g[f];
            else
                gap = (double)g[f] - (double)t[f];
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
//  Build synthetic GCPC from dense random sampling (fallback when no Julia)
// ═══════════════════════════════════════════════════════════════════════════
static GcpcCache build_synthetic_gcpc(const Robot& robot) {
    std::cout << "  Building synthetic GCPC from dense sampling..." << std::endl;

    const int n = robot.n_joints();
    const int n_act = robot.n_active_links();
    const int* map = robot.active_link_map();
    double d0 = robot.dh_params()[0].d;

    std::vector<GcpcPoint> points;
    std::mt19937 rng(12345);

    // For each active link, sample random configs and find approximate critical points
    // via coordinate-wise sweep
    for (int ci = 0; ci < n_act; ++ci) {
        int link_id = map[ci];
        int nj = std::min(link_id, n);
        int n_eff = nj - 1;  // exclude q₀

        if (n_eff <= 0) continue;

        // Check q₆ skip
        bool skip_q6 = false;
        if (n_eff >= 6 && link_id <= n) {
            const auto& dh6 = robot.dh_params()[6];
            if (std::abs(dh6.d) < 1e-10 && std::abs(dh6.a) < 1e-10) {
                skip_q6 = true;
                n_eff = std::min(n_eff, 5);
            }
        }

        // Generate random starting configs and sweep to approximate critical points
        int n_starts = 200;
        for (int start = 0; start < n_starts; ++start) {
            Eigen::VectorXd q(n);
            // q₀ = 0 (will be analytically determined)
            q[0] = 0.0;
            // q₁ in [0, π] (half range)
            std::uniform_real_distribution<double> dist_q1(0.0, M_PI);
            q[1] = dist_q1(rng);
            for (int j = 2; j <= n_eff; ++j) {
                std::uniform_real_distribution<double> dist_full(-M_PI, M_PI);
                q[j] = dist_full(rng);
            }
            for (int j = n_eff + 1; j < n; ++j) q[j] = 0.0;

            // Coordinate sweep for R² and z
            for (int iter = 0; iter < 5; ++iter) {
                for (int jj = 1; jj <= n_eff; ++jj) {
                    auto pos = fk_link_positions(robot, q);
                    if (link_id + 1 >= (int)pos.size()) break;

                    double best_val = -1e30;
                    double best_q = q[jj];

                    // Sweep this joint
                    double lo = (jj == 1) ? 0.0 : -M_PI;
                    double hi = (jj == 1) ? M_PI : M_PI;
                    for (int k = 0; k <= 20; ++k) {
                        q[jj] = lo + (hi - lo) * k / 20.0;
                        auto p2 = fk_link_positions(robot, q);
                        if (link_id + 1 >= (int)p2.size()) continue;
                        double A = p2[link_id + 1][0];
                        double B = p2[link_id + 1][1];
                        double R2 = A*A + B*B;
                        if (R2 > best_val) {
                            best_val = R2;
                            best_q = q[jj];
                        }
                    }
                    q[jj] = best_q;
                }
            }

            // Record this approximate critical point
            auto pos = fk_link_positions(robot, q);
            if (link_id + 1 < (int)pos.size()) {
                GcpcPoint pt{};
                pt.n_eff = n_eff;
                for (int d = 0; d < n_eff; ++d)
                    pt.q_eff[d] = q[d + 1];
                pt.link_id = link_id;

                pt.A = pos[link_id + 1][0];
                pt.B = pos[link_id + 1][1];
                pt.C = pos[link_id + 1][2] - d0;
                pt.R = std::sqrt(pt.A * pt.A + pt.B * pt.B);

                // xy direction
                pt.direction = 0;
                points.push_back(pt);

                // z direction
                pt.direction = 1;
                points.push_back(pt);
            }
        }
    }

    std::cout << "  Synthetic GCPC: " << points.size() << " points" << std::endl;

    GcpcCache cache;
    cache.build(robot, points);
    return cache;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Method definitions
// ═══════════════════════════════════════════════════════════════════════════
struct Method {
    std::string name;
    enum Type { ANALYTICAL, GCPC_CACHE, GCPC_COMPLETE, GCPC_MERGE } type;
    AnalyticalCriticalConfig analytical_cfg;
    std::vector<AnalyticalCriticalConfig> merge_passes;  // for GCPC_MERGE
};

static AnalyticalCriticalConfig make_v1_base() {
    AnalyticalCriticalConfig c;
    c.enable_pair_1d = false;
    c.enable_pair_2d = false;
    c.improved_interior = false;
    c.keep_kpi2_baseline    = false;
    c.enable_edge_solve     = false;
    c.enable_face_solve     = false;
    c.enable_interior_solve = false;
    return c;
}

// ─── GCPC_MERGE: GCPC + N 轮 analytical 合并 ───────────────────────────
// configs[] 指定每轮analytical的配置；与GCPC结果取min(lo)/max(hi)合并
struct MergeSpec {
    std::string name;
    std::vector<AnalyticalCriticalConfig> passes;
};

static AnalyticalCriticalConfig make_noP2() {
    auto c = make_v1_base();
    c.keep_kpi2_baseline = true;
    c.enable_edge_solve = true;
    c.enable_interior_solve = true;
    return c;
}
static AnalyticalCriticalConfig make_v1_full() {
    auto c = make_v1_base();
    c.keep_kpi2_baseline = true;
    c.enable_edge_solve = true;
    c.enable_face_solve = true;
    c.enable_interior_solve = true;
    return c;
}

static AnalyticalCriticalConfig make_p25_lite(int max_bg = 64, bool p1d = true, bool p2d = true) {
    auto c = make_v1_base();
    c.keep_kpi2_baseline = true;   // Phase 0
    c.enable_edge_solve  = true;   // Phase 1
    c.enable_pair_1d     = p1d;
    c.enable_pair_2d     = p2d;
    c.pair_max_bg        = max_bg;
    c.enable_interior_solve = false;
    return c;
}

// ─── Fused config: Phase 0+1+2+3×2+2.5a in a single call ──────────────
static AnalyticalCriticalConfig make_fused(bool p1d = true, bool p2d = false) {
    auto c = make_v1_base();
    c.keep_kpi2_baseline = true;   // Phase 0
    c.enable_edge_solve  = true;   // Phase 1
    c.enable_face_solve  = true;   // Phase 2
    c.enable_pair_1d     = p1d;    // Phase 2.5a
    c.enable_pair_2d     = p2d;    // Phase 2.5b
    c.enable_interior_solve = true; // Phase 3 (original)
    c.improved_interior  = false;
    c.dual_phase3        = true;   // Run Phase 3 twice: with/without Phase 2
    return c;
}

static std::vector<Method> build_methods() {
    std::vector<Method> methods;

    // ── 基线 ──
    methods.push_back({"gcpc_full", Method::GCPC_CACHE, {}});
    methods.push_back({"gc+v1+noP2", Method::GCPC_MERGE, {}, {make_v1_full(), make_noP2()}});

    // ── 参考：上一轮最快零遗漏 (3-pass) ──
    methods.push_back({"25L_1dOnly", Method::GCPC_MERGE, {},
        {make_v1_full(), make_noP2(), make_p25_lite(64, true, false)}});

    // ── 融合方案：GCPC + single fused call (1-pass) ──
    // fused_1d: Phase 0+1+2+dual_P3+pair_1d → eliminates 2× Phase 0+1 overhead
    methods.push_back({"fused_1d", Method::GCPC_MERGE, {}, {make_fused(true, false)}});

    // ── 不带 Phase 2.5a 的 fused (测试 pair_1d 是否仍必需) ──
    {
        auto c = make_fused(false, false);
        c.enable_pair_1d = false;
        methods.push_back({"fused_noP25", Method::GCPC_MERGE, {}, {c}});
    }

    // ── fused + 跳过 Phase 0 (GCPC 已覆盖 kπ/2 点) ──
    {
        auto c = make_fused(true, false);
        c.keep_kpi2_baseline = false;
        methods.push_back({"fused_noP0", Method::GCPC_MERGE, {}, {c}});
    }

    return methods;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Main experiment runner
// ═══════════════════════════════════════════════════════════════════════════

int main(int argc, char* argv[]) {
    std::cout << "═══ Experiment 28: GCPC Validation ═══\n" << std::endl;

    // Parse command line
    std::string cache_path_panda, cache_path_iiwa14;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--cache-panda" && i + 1 < argc)
            cache_path_panda = argv[++i];
        else if (arg == "--cache-iiwa14" && i + 1 < argc)
            cache_path_iiwa14 = argv[++i];
    }

    // Robot configs
    std::string configs_base = "../../v1/configs";
    if (!std::filesystem::exists(configs_base))
        configs_base = "../../../v1/configs";
    if (!std::filesystem::exists(configs_base))
        configs_base = "../../safeboxforest/v1/configs";

    struct RobotEntry {
        std::string name;
        std::string config_path;
        std::string cache_path;
    };

    std::vector<RobotEntry> robots;
    {
        std::string panda_cfg = configs_base + "/panda.json";
        std::string iiwa_cfg  = configs_base + "/iiwa14.json";
        if (std::filesystem::exists(panda_cfg))
            robots.push_back({"panda", panda_cfg, cache_path_panda});
        if (std::filesystem::exists(iiwa_cfg))
            robots.push_back({"iiwa14", iiwa_cfg, cache_path_iiwa14});

        if (robots.empty()) {
            std::cerr << "ERROR: Cannot find robot configs in " << configs_base << std::endl;
            return 1;
        }
    }

    auto methods = build_methods();
    std::vector<double> widths = { 0.4, 0.6, 1.0 };
    int n_trials = 30;
    int n_sub = 1;
    int n_gt_samples = 300'000;

    // Results CSV
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    struct tm tm_buf;
    localtime_s(&tm_buf, &t);
    char ts[64];
    std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tm_buf);

    std::string results_dir = "results/exp28_gcpc_" + std::string(ts);
    std::filesystem::create_directories(results_dir);
    std::string csv_path = results_dir + "/exp28_results.csv";

    std::ofstream csv(csv_path);
    csv << "robot,width,method,trial,"
        << "n_misses,max_gap,avg_gap,vol_ratio,time_ms,fk_total,"
        << "cache_matches,q1_reflected,q0_valid,boundary_kpi2,boundary_atan2,"
        << "ifk_prune_checks,ifk_pruned,ifk_not_pruned\n";

    std::cout << "Robots: " << robots.size()
              << " | Methods: " << methods.size()
              << " | Widths: " << widths.size()
              << " | Trials: " << n_trials
              << "\nTotal runs: " << robots.size() * methods.size() * widths.size() * n_trials
              << std::endl;

    for (const auto& robot_entry : robots) {
        std::cout << "\n━━━ Robot: " << robot_entry.name << " ━━━" << std::endl;

        Robot robot = Robot::from_json(robot_entry.config_path);

        const int n = robot.n_joints();
        const int n_act = robot.n_active_links();
        int n_aabb = n_act * n_sub;

        // Load or build GCPC cache
        GcpcCache gcpc;
        if (!robot_entry.cache_path.empty() &&
            std::filesystem::exists(robot_entry.cache_path)) {
            std::cout << "  Loading GCPC from: " << robot_entry.cache_path << std::endl;
            // Check extension
            auto& cp = robot_entry.cache_path;
            if (cp.size() >= 5 && cp.substr(cp.size() - 5) == ".json") {
                gcpc.load_json(robot_entry.cache_path, robot);
            } else {
                gcpc.load(robot_entry.cache_path);
            }
        } else {
            // Build synthetic cache (fallback)
            gcpc = build_synthetic_gcpc(robot);
        }

        std::cout << "  GCPC loaded: " << gcpc.n_total_points() << " total points, "
                  << gcpc.n_links() << " link sections" << std::endl;

        for (double w : widths) {
            std::cout << "\n  Width: " << w << std::endl;

            for (int trial = 0; trial < n_trials; ++trial) {
                // Random interval center
                std::mt19937 rng(trial * 1000 + (int)(w * 100));
                std::vector<Interval> intervals(n);
                const auto& jlimits = robot.joint_limits().limits;
                for (int j = 0; j < n; ++j) {
                    double jlo = jlimits[j].lo;
                    double jhi = jlimits[j].hi;
                    double range = jhi - jlo;
                    double half_w = w * 0.5;
                    double center_lo = jlo + half_w;
                    double center_hi = jhi - half_w;
                    if (center_lo > center_hi) {
                        intervals[j] = {jlo, jhi};
                    } else {
                        std::uniform_real_distribution<double> dist(center_lo, center_hi);
                        double c = dist(rng);
                        intervals[j] = {c - half_w, c + half_w};
                    }
                }

                // Ground truth
                std::vector<float> gt_aabb(n_aabb * 6);
                dense_sample_aabb(robot, intervals, n_sub, n_gt_samples, gt_aabb.data());

                // Run each method
                for (const auto& method : methods) {
                    std::vector<float> test_aabb(n_aabb * 6);
                    double time_ms = 0;
                    int fk_total = 0;
                    GcpcQueryStats gcpc_stats{};

                    if (method.type == Method::GCPC_CACHE) {
                        auto t0 = Clock::now();
                        gcpc.derive_aabb_with_gcpc(robot, intervals, n_sub,
                                                   test_aabb.data(), &gcpc_stats);
                        auto t1 = Clock::now();
                        time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
                        fk_total = gcpc_stats.n_fk_calls;
                    } else if (method.type == Method::GCPC_MERGE) {
                        // GCPC + N 轮 analytical 合并
                        auto t0 = Clock::now();

                        // Step 1: GCPC
                        gcpc.derive_aabb_with_gcpc(robot, intervals, n_sub,
                                                   test_aabb.data(), &gcpc_stats);
                        fk_total = gcpc_stats.n_fk_calls;

                        // Step 2..N: 每轮 analytical，合并到 test_aabb
                        for (const auto& pass_cfg : method.merge_passes) {
                            std::vector<float> aabb_pass(n_aabb * 6);
                            AnalyticalCriticalStats astats{};
                            derive_aabb_critical_analytical(
                                robot, intervals, n_sub,
                                pass_cfg, aabb_pass.data(), &astats);

                            // 合并: min(lo), max(hi)
                            for (int i = 0; i < n_aabb; ++i) {
                                float* a = test_aabb.data() + i * 6;
                                const float* b = aabb_pass.data() + i * 6;
                                for (int d = 0; d < 3; ++d) {
                                    a[d]     = std::min(a[d], b[d]);
                                    a[d + 3] = std::max(a[d + 3], b[d + 3]);
                                }
                            }
                            fk_total += astats.n_phase0_vertices + astats.n_phase1_fk_calls
                                      + astats.n_phase2_fk_calls + astats.n_phase25_fk_calls
                                      + astats.n_phase3_fk_calls;
                        }

                        auto t1 = Clock::now();
                        time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
                    } else if (method.type == Method::GCPC_COMPLETE) {
                        // 零遗漏模式: GCPC + 3 轮 analytical (improved P3 / original P3+all / no_P2)
                        // 根因: Phase 2 会改变 Phase 3 的起始点，导致收敛到不同局部最优
                        //   → no_P2 (Phase 1 起始) 有时优于 all_enabled (Phase 2 起始)
                        //   → 因此需要 BOTH 来保证全覆盖
                        auto t0 = Clock::now();

                        // Step 1: GCPC Phase A+B+C
                        gcpc.derive_aabb_with_gcpc(robot, intervals, n_sub,
                                                   test_aabb.data(), &gcpc_stats);

                        // Step 2a: Analytical with IMPROVED Phase 3 + all phases
                        std::vector<float> aabb_improved(n_aabb * 6);
                        AnalyticalCriticalStats astats_imp{};
                        derive_aabb_critical_analytical(
                            robot, intervals, n_sub,
                            method.analytical_cfg,
                            aabb_improved.data(), &astats_imp);

                        // Step 2b: Analytical with ORIGINAL Phase 3 + all phases
                        auto cfg_orig = method.analytical_cfg;
                        cfg_orig.improved_interior = false;
                        std::vector<float> aabb_original(n_aabb * 6);
                        AnalyticalCriticalStats astats_orig{};
                        derive_aabb_critical_analytical(
                            robot, intervals, n_sub,
                            cfg_orig,
                            aabb_original.data(), &astats_orig);

                        // Step 2c: no_P2 config (P0+P1+P3 only — Phase 1 provides
                        //          starting point for Phase 3, avoiding Phase 2 basin shift)
                        auto cfg_noP2 = make_v1_base();
                        cfg_noP2.keep_kpi2_baseline = true;
                        cfg_noP2.enable_edge_solve = true;
                        cfg_noP2.enable_interior_solve = true;
                        cfg_noP2.interior_max_sweeps = method.analytical_cfg.interior_max_sweeps;
                        std::vector<float> aabb_noP2(n_aabb * 6);
                        AnalyticalCriticalStats astats_noP2{};
                        derive_aabb_critical_analytical(
                            robot, intervals, n_sub,
                            cfg_noP2,
                            aabb_noP2.data(), &astats_noP2);

                        // Step 3: Merge all four — take loosest bounds
                        for (int i = 0; i < n_aabb; ++i) {
                            float* a = test_aabb.data() + i * 6;
                            const float* b = aabb_improved.data() + i * 6;
                            const float* c = aabb_original.data() + i * 6;
                            const float* d2 = aabb_noP2.data() + i * 6;
                            for (int d = 0; d < 3; ++d) {
                                a[d]     = std::min({a[d], b[d], c[d], d2[d]});
                                a[d + 3] = std::max({a[d + 3], b[d + 3], c[d + 3], d2[d + 3]});
                            }
                        }

                        auto t1 = Clock::now();
                        time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
                        fk_total = gcpc_stats.n_fk_calls
                                 + astats_imp.n_phase1_fk_calls + astats_imp.n_phase2_fk_calls
                                 + astats_imp.n_phase25_fk_calls + astats_imp.n_phase3_fk_calls
                                 + astats_imp.n_phase0_vertices
                                 + astats_orig.n_phase1_fk_calls + astats_orig.n_phase2_fk_calls
                                 + astats_orig.n_phase25_fk_calls + astats_orig.n_phase3_fk_calls
                                 + astats_orig.n_phase0_vertices
                                 + astats_noP2.n_phase1_fk_calls + astats_noP2.n_phase3_fk_calls
                                 + astats_noP2.n_phase0_vertices;
                    } else {
                        AnalyticalCriticalStats astats{};
                        auto t0 = Clock::now();
                        derive_aabb_critical_analytical(
                            robot, intervals, n_sub,
                            method.analytical_cfg,
                            test_aabb.data(), &astats);
                        auto t1 = Clock::now();
                        time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
                        fk_total = astats.n_phase1_fk_calls + astats.n_phase2_fk_calls
                                 + astats.n_phase25_fk_calls + astats.n_phase3_fk_calls
                                 + astats.n_phase0_vertices;
                    }

                    auto err = compute_face_errors(test_aabb.data(), gt_aabb.data(), n_aabb);
                    double avg_gap = (err.n_misses > 0) ? err.sum_gap / err.n_misses : 0.0;
                    double vol_ratio = err.sum_vol_ratio / std::max(n_aabb, 1);

                    csv << robot_entry.name << ","
                        << w << ","
                        << method.name << ","
                        << trial << ","
                        << err.n_misses << ","
                        << err.max_gap << ","
                        << avg_gap << ","
                        << vol_ratio << ","
                        << time_ms << ","
                        << fk_total << ","
                        << gcpc_stats.n_cache_matches << ","
                        << gcpc_stats.n_q1_reflected << ","
                        << gcpc_stats.n_q0_valid << ","
                        << gcpc_stats.n_boundary_kpi2 << ","
                        << gcpc_stats.n_boundary_atan2 << ","
                        << gcpc_stats.n_ifk_prune_checks << ","
                        << gcpc_stats.n_ifk_pruned << ","
                        << gcpc_stats.n_ifk_not_pruned << "\n";
                }

                if (trial % 10 == 0)
                    std::cout << "    Trial " << trial << "/" << n_trials << "\r" << std::flush;
            }
            csv.flush();
        }
    }

    csv.close();
    std::cout << "\n\n═══ Results saved to: " << csv_path << " ═══" << std::endl;

    // Print summary
    std::cout << "\n─── Summary ───" << std::endl;
    std::ifstream csv_in(csv_path);
    std::string header_line;
    std::getline(csv_in, header_line);

    struct Summary {
        double sum_misses = 0, sum_max_gap = 0, sum_vol = 0, sum_time = 0;
        int sum_fk = 0, count = 0;
    };
    std::map<std::string, Summary> summaries;

    std::string line;
    while (std::getline(csv_in, line)) {
        std::istringstream iss(line);
        std::string robot_name, method_name;
        double w, n_misses, max_gap, avg_gap, vol_ratio, time_ms;
        int trial, fk_total;
        char comma;

        std::getline(iss, robot_name, ',');
        iss >> w >> comma;
        std::getline(iss, method_name, ',');
        iss >> trial >> comma >> n_misses >> comma >> max_gap >> comma
            >> avg_gap >> comma >> vol_ratio >> comma >> time_ms >> comma >> fk_total;

        auto& s = summaries[method_name];
        s.sum_misses += n_misses;
        s.sum_max_gap += max_gap;
        s.sum_vol += vol_ratio;
        s.sum_time += time_ms;
        s.sum_fk += fk_total;
        s.count++;
    }

    std::cout << std::fixed << std::setprecision(3);
    std::cout << std::setw(15) << "Method"
              << std::setw(10) << "Miss"
              << std::setw(12) << "MaxGap×1e4"
              << std::setw(10) << "Vol"
              << std::setw(10) << "Time(ms)"
              << std::setw(10) << "FK" << std::endl;

    for (auto& [name, s] : summaries) {
        double c = std::max(s.count, 1);
        std::cout << std::setw(15) << name
                  << std::setw(10) << s.sum_misses / c
                  << std::setw(12) << s.sum_max_gap / c * 1e4
                  << std::setw(10) << s.sum_vol / c
                  << std::setw(10) << s.sum_time / c
                  << std::setw(10) << (int)(s.sum_fk / c) << std::endl;
    }

    return 0;
}

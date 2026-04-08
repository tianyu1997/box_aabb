// ═══════════════════════════════════════════════════════════════════════════
//  GCPC vs Analytical Per-Phase Timing Comparison
//
//  Runs both pipelines on N random C-space boxes and reports:
//    - GCPC per-phase timing (A–G) from built-in stats
//    - Analytical total timing
//    - Per-width breakdown
//
//  Build: cmake --build build --config Release --target exp_phase_compare
//  Run:   build\Release\exp_phase_compare.exe [n_trials] [seed]
// ═══════════════════════════════════════════════════════════════════════════
#define _USE_MATH_DEFINES
#include <cmath>

#include "sbf/robot/robot.h"
#include "sbf/robot/interval_math.h"
#include "sbf/envelope/analytical_solve.h"
#include "sbf/envelope/gcpc.h"
#include "sbf/robot/fk.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <random>
#include <vector>

using namespace sbf;
using namespace sbf::envelope;

// ─── IIWA14 factory ─────────────────────────────────────────────────────────

static Robot make_iiwa14() {
    std::vector<DHParam> dh = {
        {  0.0,     0.0, 0.1575, 0.0, 0},
        { -HALF_PI, 0.0, 0.0,    0.0, 0},
        {  HALF_PI, 0.0, 0.2025, 0.0, 0},
        {  HALF_PI, 0.0, 0.0,    0.0, 0},
        { -HALF_PI, 0.0, 0.2155, 0.0, 0},
        { -HALF_PI, 0.0, 0.0,    0.0, 0},
        {  HALF_PI, 0.0, 0.081,  0.0, 0},
    };

    JointLimits limits;
    limits.limits = {
        {-1.865, 1.866}, {-0.100, 1.087}, {-0.663, 0.662},
        {-2.094,-0.372}, {-0.619, 0.620}, {-1.095, 1.258},
        { 1.050, 2.091},
    };

    DHParam tool{0.0, 0.0, 0.185, 0.0, 0};
    std::vector<double> radii = {0.08, 0.08, 0.06, 0.06, 0.04, 0.04, 0.04, 0.03};
    return Robot("iiwa14", dh, limits, tool, radii);
}

// ─── GCPC cache builder ─────────────────────────────────────────────────────

static GcpcCache build_gcpc_cache(const Robot& robot) {
    const int n = robot.n_joints();
    const int n_act = robot.n_active_links();
    const int* link_map = robot.active_link_map();
    auto limits = robot.joint_limits();

    std::vector<std::vector<double>> per_joint(n);
    for (int j = 0; j < n; ++j) {
        double lo = limits.limits[j].lo, hi = limits.limits[j].hi;
        per_joint[j].push_back(lo);
        per_joint[j].push_back(hi);
        per_joint[j].push_back(0.5 * (lo + hi));
        for (int k = -20; k <= 20; ++k) {
            double a = k * HALF_PI;
            if (a > lo + 1e-10 && a < hi - 1e-10)
                per_joint[j].push_back(a);
        }
        std::sort(per_joint[j].begin(), per_joint[j].end());
        per_joint[j].erase(
            std::unique(per_joint[j].begin(), per_joint[j].end()),
            per_joint[j].end());
    }

    std::vector<double> q1_half;
    for (double v : per_joint[1]) {
        if (v >= -1e-10 && v <= M_PI + 1e-10)
            q1_half.push_back(std::max(0.0, std::min(M_PI, v)));
    }
    if (q1_half.empty()) q1_half.push_back(0.5);

    std::vector<GcpcPoint> cache_points;

    for (int ci = 0; ci < n_act; ++ci) {
        int link_id = link_map[ci];
        int nj = std::min(link_id + 1, n);
        int n_eff = nj - 1;
        if (n_eff <= 0) continue;

        std::vector<std::vector<double>> joint_vals(n_eff);
        joint_vals[0] = q1_half;
        for (int d = 1; d < n_eff; ++d)
            joint_vals[d] = per_joint[d + 1];

        long long product = 1;
        for (int d = 0; d < n_eff; ++d) {
            product *= (long long)joint_vals[d].size();
            if (product > 50000) break;
        }

        auto eval_config = [&](const std::vector<double>& config) {
            GcpcPoint pt{};
            pt.link_id = link_id;
            pt.n_eff = n_eff;
            for (int i = 0; i < n_eff; ++i)
                pt.q_eff[i] = config[i];

            Eigen::VectorXd q_full(n);
            q_full.setZero();
            for (int i = 0; i < n_eff && i + 1 < n; ++i)
                q_full[i + 1] = pt.q_eff[i];

            auto pos = fk_link_positions(robot, q_full);
            if (link_id + 1 < (int)pos.size()) {
                const auto& p = pos[link_id + 1];
                pt.direction = 0;
                pt.A = p[0]; pt.B = p[1]; pt.C = p[2];
                pt.R = std::sqrt(pt.A * pt.A + pt.B * pt.B);
                cache_points.push_back(pt);
                GcpcPoint ptz = pt; ptz.direction = 1;
                cache_points.push_back(ptz);
            }
        };

        if (product > 50000) {
            std::mt19937 rng(12345 + ci);
            for (int s = 0; s < 5000; ++s) {
                std::vector<double> config(n_eff);
                for (int d = 0; d < n_eff; ++d)
                    config[d] = joint_vals[d][
                        std::uniform_int_distribution<int>(
                            0, (int)joint_vals[d].size() - 1)(rng)];
                eval_config(config);
            }
        } else {
            std::vector<double> config(n_eff);
            std::function<void(int)> enumerate;
            enumerate = [&](int d) {
                if (d >= n_eff) { eval_config(config); return; }
                for (double v : joint_vals[d]) {
                    config[d] = v;
                    enumerate(d + 1);
                }
            };
            enumerate(0);
        }
    }

    GcpcCache cache;
    cache.build(robot, cache_points);
    cache.enrich_with_interior_search(robot, 500, 5);
    return cache;
}

// ─── Box generator ──────────────────────────────────────────────────────────

struct BoxGenerator {
    const Robot& robot;
    std::mt19937& rng;

    std::vector<Interval> generate(double frac) {
        const auto& lim = robot.joint_limits().limits;
        int n = robot.n_joints();
        std::vector<Interval> ivs(n);
        for (int j = 0; j < n; ++j) {
            double lo = lim[j].lo, hi = lim[j].hi, range = hi - lo;
            std::uniform_real_distribution<double> d(lo, hi);
            double c = d(rng);
            double hw = range * frac * 0.5;
            ivs[j].lo = std::max(lo, c - hw);
            ivs[j].hi = std::min(hi, c + hw);
        }
        return ivs;
    }
};

// ─── Main ───────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    const int n_trials = (argc > 1) ? std::atoi(argv[1]) : 100;
    const int seed     = (argc > 2) ? std::atoi(argv[2]) : 42;

    Robot robot = make_iiwa14();
    const int n_act = robot.n_active_links();

    std::fprintf(stderr, "[phase_compare] Building GCPC cache...\n");
    GcpcCache gcpc_cache = build_gcpc_cache(robot);
    std::fprintf(stderr, "[phase_compare] GCPC cache built.\n");

    AnalyticalCriticalConfig anal_cfg = AnalyticalCriticalConfig::all_enabled();
    anal_cfg.dual_phase3 = true;

    // width distribution matching exp_v3v4_epiAABB
    const double widths_narrow[]  = {0.05, 0.10, 0.15};
    const double widths_medium[]  = {0.20, 0.30, 0.40};
    const double widths_wide[]    = {0.50, 0.70, 1.00};

    // CSV header
    std::printf("trial,width,"
                "gcpc_total_ms,gcpc_A_ms,gcpc_B_ms,gcpc_C_ms,gcpc_D_ms,gcpc_E_ms,gcpc_F_ms,gcpc_G_ms,"
                "gcpc_n_cache,gcpc_n_kpi2,gcpc_n_atan2,gcpc_n_d_faces,gcpc_n_e_pair1d,gcpc_n_f_pair2d,gcpc_n_g_interior,"
                "gcpc_n_fk,gcpc_n_aa_pruned,gcpc_n_aa_checks,"
                "anal_total_ms,"
                "anal_n_p0,anal_n_p1,anal_n_p2,anal_n_p25a,anal_n_p25b,anal_n_p3,"
                "anal_n_p1_fk,anal_n_p2_fk,anal_n_p25_fk,anal_n_p3_fk\n");

    std::mt19937 rng(seed);
    BoxGenerator gen{robot, rng};

    for (int trial = 0; trial < n_trials; ++trial) {
        double frac;
        if (trial < n_trials * 4 / 10) {
            frac = widths_narrow[trial % 3];
        } else if (trial < n_trials * 8 / 10) {
            frac = widths_medium[(trial - n_trials * 4 / 10) % 3];
        } else {
            frac = widths_wide[(trial - n_trials * 8 / 10) % 3];
        }

        auto intervals = gen.generate(frac);

        // ── GCPC ────────────────────────────────────────────────────────
        std::vector<float> gcpc_aabb(n_act * 6);
        GcpcQueryStats gstats{};

        auto t0g = std::chrono::high_resolution_clock::now();
        gcpc_cache.derive_aabb_with_gcpc(
            robot, intervals, 1, gcpc_aabb.data(), &gstats);
        auto t1g = std::chrono::high_resolution_clock::now();
        double gcpc_total = std::chrono::duration<double, std::milli>(t1g - t0g).count();

        // ── Analytical ──────────────────────────────────────────────────
        std::vector<float> anal_aabb(n_act * 6);
        AnalyticalCriticalStats astats{};

        auto t0a = std::chrono::high_resolution_clock::now();
        derive_aabb_critical_analytical(
            robot, intervals, 1, anal_cfg, anal_aabb.data(), &astats);
        auto t1a = std::chrono::high_resolution_clock::now();
        double anal_total = std::chrono::duration<double, std::milli>(t1a - t0a).count();

        // ── Output ──────────────────────────────────────────────────────
        std::printf("%d,%.3f,"
                    "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                    "%d,%d,%d,%d,%d,%d,%d,"
                    "%d,%d,%d,"
                    "%.4f,"
                    "%d,%d,%d,%d,%d,%d,"
                    "%d,%d,%d,%d\n",
                    trial, frac,
                    gcpc_total,
                    gstats.phase_a_ms, gstats.phase_b_ms,
                    gstats.phase_c_ms, gstats.phase_d_ms,
                    gstats.phase_e_ms, gstats.phase_f_ms,
                    gstats.phase_g_ms,
                    gstats.n_cache_matches, gstats.n_boundary_kpi2,
                    gstats.n_boundary_atan2, gstats.n_phase_d_faces,
                    gstats.n_phase_e_pair1d, gstats.n_phase_f_pair2d,
                    gstats.n_phase_g_interior,
                    gstats.n_fk_calls, gstats.n_aa_pruned, gstats.n_aa_prune_checks,
                    anal_total,
                    astats.n_phase0_vertices, astats.n_phase1_edges,
                    astats.n_phase2_faces,
                    astats.n_phase25a_pair1d, astats.n_phase25b_pair2d,
                    astats.n_phase3_interior,
                    astats.n_phase1_fk_calls, astats.n_phase2_fk_calls,
                    astats.n_phase25_fk_calls, astats.n_phase3_fk_calls);

        if ((trial + 1) % 20 == 0)
            std::fprintf(stderr, "[phase_compare] %d/%d done.\n", trial + 1, n_trials);
    }

    std::fprintf(stderr, "[phase_compare] All %d trials completed.\n", n_trials);
    return 0;
}

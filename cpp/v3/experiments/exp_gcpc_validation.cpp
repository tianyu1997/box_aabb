// ═══════════════════════════════════════════════════════════════════════════
//  Experiment: GCPC Validation  —  GCPC+AA vs Analytical vs Dense Sampling
//
//  Validates the ported GCPC cache by comparing three methods:
//    1. GCPC (Global Critical Point Cache + two-level AA pruning)
//    2. AnalyticalCritical (exact gradient-zero enumeration)
//    3. MC  (Monte-Carlo dense sampling ground truth, 300K samples)
//
//  Reports: per-link volume, tightness ratio vs MC, containment, timing,
//           AA pruning statistics.
//
//  Usage: exp_gcpc_validation [panda|iiwa14] [width]
// ═══════════════════════════════════════════════════════════════════════════
#define _USE_MATH_DEFINES
#include <cmath>

#include "sbf/robot/robot.h"
#include "sbf/robot/fk.h"
#include "sbf/robot/interval_fk.h"
#include "sbf/robot/affine_fk.h"
#include "sbf/envelope/gcpc_cache.h"
#include "sbf/envelope/envelope_derive_critical.h"
#include "sbf/common/types.h"

#include <Eigen/Core>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>
#include <algorithm>

using namespace sbf;
using namespace sbf::envelope;

static const std::string ROBOT_DIR =
    "C:/Users/TIAN/OneDrive - The Chinese University of Hong Kong"
    "/Desktop/code/box_aabb/safeboxforest/v1/configs/";

// ─── Helpers ────────────────────────────────────────────────────────────────

static double aabb_volume_6(const float* aabb) {
    double dx = aabb[3] - aabb[0];
    double dy = aabb[4] - aabb[1];
    double dz = aabb[5] - aabb[2];
    if (dx <= 0 || dy <= 0 || dz <= 0) return 0.0;
    return dx * dy * dz;
}

static double total_volume(const float* aabbs, int n_links) {
    double vol = 0.0;
    for (int i = 0; i < n_links; ++i)
        vol += aabb_volume_6(aabbs + i * 6);
    return vol;
}

// Check containment: does `outer` contain `inner` per-link?
static bool contains_all(const float* outer, const float* inner, int n_links,
                          double tol = 1e-6)
{
    for (int i = 0; i < n_links; ++i) {
        const float* o = outer + i * 6;
        const float* m = inner + i * 6;
        for (int d = 0; d < 3; ++d) {
            if (o[d] > m[d] + tol) return false;        // lo too high
            if (o[d + 3] < m[d + 3] - tol) return false; // hi too low
        }
    }
    return true;
}

// Dense MC ground truth: sample 300K configs, evaluate FK, build tight AABB
static void dense_mc(const Robot& robot,
                     const std::vector<Interval>& intervals,
                     int n_samples,
                     float* out_aabb)
{
    const int n = robot.n_joints();
    const int n_act = robot.n_active_links();
    const int* map = robot.active_link_map();
    const double* rad = robot.active_link_radii();

    // Init to inverted
    for (int ci = 0; ci < n_act; ++ci) {
        float* a = out_aabb + ci * 6;
        a[0] = a[1] = a[2] =  1e30f;
        a[3] = a[4] = a[5] = -1e30f;
    }

    std::mt19937 rng(42);
    Eigen::VectorXd q(n);

    for (int s = 0; s < n_samples; ++s) {
        for (int j = 0; j < n; ++j) {
            std::uniform_real_distribution<double> dist(
                intervals[j].lo, intervals[j].hi);
            q[j] = dist(rng);
        }
        auto pos = fk_link_positions(robot, q);
        int np = static_cast<int>(pos.size());

        for (int ci = 0; ci < n_act; ++ci) {
            int V = map[ci];
            if (V + 1 >= np) continue;
            // Use distal frame position
            const auto& p = pos[V + 1];
            float* a = out_aabb + ci * 6;
            for (int d = 0; d < 3; ++d) {
                float v = static_cast<float>(p[d]);
                if (v < a[d])     a[d]     = v;
                if (v > a[d + 3]) a[d + 3] = v;
            }
            // Also proximal frame
            const auto& pp = pos[V];
            for (int d = 0; d < 3; ++d) {
                float v = static_cast<float>(pp[d]);
                if (v < a[d])     a[d]     = v;
                if (v > a[d + 3]) a[d + 3] = v;
            }
        }
    }

    // Inflate by link radii
    for (int ci = 0; ci < n_act; ++ci) {
        float r = rad ? static_cast<float>(rad[ci]) : 0.f;
        float* a = out_aabb + ci * 6;
        a[0] -= r;  a[1] -= r;  a[2] -= r;
        a[3] += r;  a[4] += r;  a[5] += r;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Main
// ═══════════════════════════════════════════════════════════════════════════
int main(int argc, char** argv) {
    std::string robot_name = "panda";
    double width = 0.1;

    if (argc >= 2) robot_name = argv[1];
    if (argc >= 3) width = std::atof(argv[2]);

    printf("═══════════════════════════════════════════════════════════\n");
    printf("  GCPC Validation Experiment\n");
    printf("  Robot: %s   Width: %.4f\n", robot_name.c_str(), width);
    printf("═══════════════════════════════════════════════════════════\n\n");

    // Load robot
    auto robot = Robot::from_json(ROBOT_DIR + robot_name + ".json");
    const int n = robot.n_joints();
    const int n_act = robot.n_active_links();
    printf("Robot: %s  n_joints=%d  n_active_links=%d\n\n",
           robot.name().c_str(), n, n_act);

    // Build intervals centered at midpoint of joint limits
    std::vector<Interval> intervals(n);
    auto limits = robot.joint_limits();
    for (int j = 0; j < n; ++j) {
        double mid = 0.5 * (limits.limits[j].lo + limits.limits[j].hi);
        intervals[j].lo = mid - width * 0.5;
        intervals[j].hi = mid + width * 0.5;
    }

    printf("Intervals (centered at joint midpoints, width=%.4f):\n", width);
    for (int j = 0; j < n; ++j)
        printf("  q%d: [%.4f, %.4f]\n", j, intervals[j].lo, intervals[j].hi);
    printf("\n");

    // ── 1. Dense MC ground truth ────────────────────────────────────────
    printf("Computing MC ground truth (300K samples)...\n");
    std::vector<float> mc_aabb(n_act * 6);
    auto t0 = std::chrono::high_resolution_clock::now();
    dense_mc(robot, intervals, 300000, mc_aabb.data());
    auto t1 = std::chrono::high_resolution_clock::now();
    double mc_time = std::chrono::duration<double, std::milli>(t1 - t0).count();
    double mc_vol = total_volume(mc_aabb.data(), n_act);
    printf("  MC: total_volume=%.6e  time=%.1f ms\n\n", mc_vol, mc_time);

    // ── 2. AnalyticalCritical ───────────────────────────────────────────
    printf("Computing AnalyticalCritical...\n");
    std::vector<float> ac_aabb(n_act * 6);
    AnalyticalCriticalConfig ac_config = AnalyticalCriticalConfig::all_enabled();
    ac_config.dual_phase3 = true;
    AnalyticalCriticalStats ac_stats{};

    t0 = std::chrono::high_resolution_clock::now();
    derive_aabb_critical_analytical(
        robot, intervals, 1, ac_config, ac_aabb.data(), &ac_stats);
    t1 = std::chrono::high_resolution_clock::now();
    double ac_time = std::chrono::duration<double, std::milli>(t1 - t0).count();
    double ac_vol = total_volume(ac_aabb.data(), n_act);
    bool ac_ok = contains_all(ac_aabb.data(), mc_aabb.data(), n_act);
    printf("  Analytical: total_volume=%.6e  ratio=%.4f  contains_MC=%s  time=%.3f ms\n",
           ac_vol, ac_vol / mc_vol, ac_ok ? "YES" : "NO", ac_time);
    printf("  Stats: V=%d  E=%d  F=%d  P1D=%d  P2D=%d  I=%d\n\n",
           ac_stats.n_phase0_vertices, ac_stats.n_phase1_edges,
           ac_stats.n_phase2_faces, ac_stats.n_phase25a_pair1d,
           ac_stats.n_phase25b_pair2d, ac_stats.n_phase3_interior);

    // ── 3. GCPC (build from analytical critical points) ─────────────────
    //
    // Since we don't have a pre-built GCPC JSON cache, we construct one
    // on-the-fly: sample many critical points across the full joint range,
    // build the GcpcCache, then query it for our interval.

    printf("Building GCPC cache from full-range critical sampling...\n");

    // Generate critical points across the full joint range
    std::vector<GcpcPoint> cache_points;
    {
        // Use a grid of critical configs across the full range
        // For each active link, sample critical configs at kπ/2 multiples
        // within the half-range q₁ ∈ [0, π]
        const int* link_map = robot.active_link_map();
        const double* dh_d = nullptr;

        // Gather kπ/2 values per joint (within full joint limits)
        std::vector<std::vector<double>> per_joint(n);
        for (int j = 0; j < n; ++j) {
            double lo = limits.limits[j].lo;
            double hi = limits.limits[j].hi;
            per_joint[j].push_back(lo);
            per_joint[j].push_back(hi);
            per_joint[j].push_back(0.5 * (lo + hi));
            for (int k = -20; k <= 20; ++k) {
                double a = k * M_PI * 0.5;
                if (a > lo + 1e-10 && a < hi - 1e-10)
                    per_joint[j].push_back(a);
            }
            std::sort(per_joint[j].begin(), per_joint[j].end());
            per_joint[j].erase(
                std::unique(per_joint[j].begin(), per_joint[j].end()),
                per_joint[j].end());
        }

        // For q₁, only keep [0, π] range (symmetry reduction)
        std::vector<double> q1_half;
        for (double v : per_joint[1]) {
            if (v >= -1e-10 && v <= M_PI + 1e-10)
                q1_half.push_back(std::max(0.0, std::min(M_PI, v)));
        }
        if (q1_half.empty()) q1_half.push_back(0.5);

        // Enumerate configs for each link
        // To keep it tractable: for each link, enumerate up to its effective DOF
        for (int ci = 0; ci < n_act; ++ci) {
            int link_id = link_map[ci];
            int nj = std::min(link_id + 1, n);
            int n_eff = nj - 1;  // excluding q₀

            if (n_eff <= 0) continue;

            // Build configs recursively (capped at 50K per link)
            std::vector<std::vector<double>> joint_vals(n_eff);
            joint_vals[0] = q1_half;  // q₁ in [0, π]
            for (int d = 1; d < n_eff; ++d)
                joint_vals[d] = per_joint[d + 1];  // q₂, q₃, ...

            long long product = 1;
            for (int d = 0; d < n_eff; ++d) {
                product *= static_cast<long long>(joint_vals[d].size());
                if (product > 50000) break;
            }

            if (product > 50000) {
                // Too many — subsample with random selection
                std::mt19937 rng(12345 + ci);
                for (int s = 0; s < 5000; ++s) {
                    GcpcPoint pt{};
                    pt.link_id = link_id;
                    pt.n_eff = n_eff;
                    for (int d = 0; d < n_eff; ++d) {
                        int idx = std::uniform_int_distribution<int>(
                            0, static_cast<int>(joint_vals[d].size()) - 1)(rng);
                        pt.q_eff[d] = joint_vals[d][idx];
                    }

                    // Evaluate FK with q₀=0 to get A, B, C
                    Eigen::VectorXd q_full(n);
                    q_full.setZero();
                    for (int d = 0; d < n_eff && d + 1 < n; ++d)
                        q_full[d + 1] = pt.q_eff[d];

                    auto pos = fk_link_positions(robot, q_full);
                    int np = static_cast<int>(pos.size());
                    if (link_id + 1 < np) {
                        const auto& p = pos[link_id + 1];
                        // xy direction
                        pt.direction = 0;
                        pt.A = p[0];
                        pt.B = p[1];
                        pt.C = p[2];
                        pt.R = std::sqrt(pt.A * pt.A + pt.B * pt.B);
                        cache_points.push_back(pt);

                        // z direction
                        GcpcPoint ptz = pt;
                        ptz.direction = 1;
                        cache_points.push_back(ptz);
                    }
                }
            } else {
                // Full enumeration
                std::vector<double> config(n_eff);
                std::function<void(int)> enumerate;
                enumerate = [&](int d) {
                    if (d >= n_eff) {
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
                        int np = static_cast<int>(pos.size());
                        if (link_id + 1 < np) {
                            const auto& p = pos[link_id + 1];
                            pt.direction = 0;
                            pt.A = p[0];
                            pt.B = p[1];
                            pt.C = p[2];
                            pt.R = std::sqrt(pt.A * pt.A + pt.B * pt.B);
                            cache_points.push_back(pt);

                            GcpcPoint ptz = pt;
                            ptz.direction = 1;
                            cache_points.push_back(ptz);
                        }
                        return;
                    }
                    for (double v : joint_vals[d]) {
                        config[d] = v;
                        enumerate(d + 1);
                    }
                };
                enumerate(0);
            }
        }
    }

    printf("  Generated %d cache points\n", static_cast<int>(cache_points.size()));

    GcpcCache cache;
    cache.build(robot, cache_points);
    printf("  Cache built: %d links, %d total points\n",
           cache.n_links(), cache.n_total_points());

    // Enrich with interior critical points via coordinate descent
    printf("  Enriching with interior critical points...\n");
    auto t_enrich_0 = std::chrono::high_resolution_clock::now();
    int n_added = cache.enrich_with_interior_search(robot, 500, 5);
    auto t_enrich_1 = std::chrono::high_resolution_clock::now();
    double enrich_ms = std::chrono::duration<double, std::milli>(t_enrich_1 - t_enrich_0).count();
    printf("  Enrichment: +%d interior points (%.1f ms), total=%d\n",
           n_added, enrich_ms, cache.n_total_points());

    // ── Run GCPC derive_aabb_with_gcpc ──────────────────────────────────
    printf("\nComputing GCPC + AA pruning...\n");
    std::vector<float> gc_aabb(n_act * 6);
    GcpcQueryStats gc_stats{};

    t0 = std::chrono::high_resolution_clock::now();
    cache.derive_aabb_with_gcpc(robot, intervals, 1, gc_aabb.data(), &gc_stats);
    t1 = std::chrono::high_resolution_clock::now();
    double gc_time = std::chrono::duration<double, std::milli>(t1 - t0).count();
    double gc_vol = total_volume(gc_aabb.data(), n_act);
    bool gc_ok = contains_all(gc_aabb.data(), mc_aabb.data(), n_act);
    printf("  GCPC: total_volume=%.6e  ratio=%.4f  contains_MC=%s  time=%.3f ms\n",
           gc_vol, gc_vol / mc_vol, gc_ok ? "YES" : "NO", gc_time);
    printf("  Stats: cache_matches=%d  q1_reflected=%d  q0_valid=%d\n",
           gc_stats.n_cache_matches, gc_stats.n_q1_reflected, gc_stats.n_q0_valid);
    printf("         boundary_kpi2=%d  boundary_atan2=%d  fk_calls=%d\n",
           gc_stats.n_boundary_kpi2, gc_stats.n_boundary_atan2, gc_stats.n_fk_calls);
    printf("         AA_prune_checks=%d  AA_pruned=%d  AA_not_pruned=%d\n",
           gc_stats.n_aa_prune_checks, gc_stats.n_aa_pruned, gc_stats.n_aa_not_pruned);
    printf("         D_faces=%d  D_fk=%d  D_pruned=%d\n",
           gc_stats.n_phase_d_faces, gc_stats.n_phase_d_fk, gc_stats.n_phase_d_pruned);
    printf("         E_pair1d=%d  E_fk=%d  E_pruned=%d\n",
           gc_stats.n_phase_e_pair1d, gc_stats.n_phase_e_fk, gc_stats.n_phase_e_pruned);
    printf("         F_pair2d=%d  F_fk=%d  F_pruned=%d\n",
           gc_stats.n_phase_f_pair2d, gc_stats.n_phase_f_fk, gc_stats.n_phase_f_pruned);
    printf("         G_interior=%d  G_fk=%d  G_pruned=%d\n",
           gc_stats.n_phase_g_interior, gc_stats.n_phase_g_fk, gc_stats.n_phase_g_pruned);
    printf("  Phase timing (ms): A=%.2f B=%.2f C=%.2f D=%.2f E=%.2f F=%.2f G=%.2f\n",
           gc_stats.phase_a_ms, gc_stats.phase_b_ms, gc_stats.phase_c_ms,
           gc_stats.phase_d_ms, gc_stats.phase_e_ms, gc_stats.phase_f_ms,
           gc_stats.phase_g_ms);

    // ── Summary ─────────────────────────────────────────────────────────
    printf("\n═══════════════════════════════════════════════════════════\n");
    printf("  SUMMARY (width=%.4f)\n", width);
    printf("───────────────────────────────────────────────────────────\n");
    printf("  %-20s %12s %10s %10s %10s\n",
           "Method", "Volume", "Ratio", "Contains?", "Time(ms)");
    printf("  %-20s %12.6e %10s %10s %10.1f\n",
           "MC (ground truth)", mc_vol, "1.0000", "---", mc_time);
    printf("  %-20s %12.6e %10.4f %10s %10.3f\n",
           "AnalyticalCritical", ac_vol, ac_vol / mc_vol,
           ac_ok ? "YES" : "NO", ac_time);
    printf("  %-20s %12.6e %10.4f %10s %10.3f\n",
           "GCPC+AA", gc_vol, gc_vol / mc_vol,
           gc_ok ? "YES" : "NO", gc_time);
    printf("═══════════════════════════════════════════════════════════\n");

    // ── Per-link breakdown ──────────────────────────────────────────────
    printf("\nPer-link volumes:\n");
    printf("  %-6s %12s %12s %12s\n", "Link", "MC", "Analytical", "GCPC");
    for (int ci = 0; ci < n_act; ++ci) {
        double v_mc = aabb_volume_6(mc_aabb.data() + ci * 6);
        double v_ac = aabb_volume_6(ac_aabb.data() + ci * 6);
        double v_gc = aabb_volume_6(gc_aabb.data() + ci * 6);
        printf("  %-6d %12.6e %12.6e %12.6e\n", ci, v_mc, v_ac, v_gc);
    }

    // ── Test save/load binary ───────────────────────────────────────────
    printf("\nTesting binary save/load...\n");
    bool saved = cache.save("test_gcpc_cache.bin");
    printf("  save: %s\n", saved ? "OK" : "FAIL");

    GcpcCache cache2;
    bool loaded = cache2.load("test_gcpc_cache.bin");
    printf("  load: %s  n_points=%d\n",
           loaded ? "OK" : "FAIL", cache2.n_total_points());

    // Verify same result after reload
    if (loaded) {
        std::vector<float> gc2_aabb(n_act * 6);
        cache2.derive_aabb_with_gcpc(robot, intervals, 1, gc2_aabb.data());
        double gc2_vol = total_volume(gc2_aabb.data(), n_act);
        double diff = std::abs(gc2_vol - gc_vol);
        printf("  reload vol diff: %.2e  %s\n",
               diff, diff < 1e-10 ? "MATCH" : "MISMATCH");
    }

    printf("\nDone.\n");
    return 0;
}

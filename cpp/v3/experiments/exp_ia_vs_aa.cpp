// ═══════════════════════════════════════════════════════════════════════════
//  Experiment: IA vs AA  —  AABB Volume & Timing Comparison (v2)
//
//  Compares three methods for each link AABB:
//    1. IA (standard interval arithmetic)
//    2. AA (affine arithmetic)
//    3. MC (Monte-Carlo ground truth via dense FK sampling)
//
//  Reports: volume, tightness ratio vs MC, timing, containment of MC AABB.
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/robot/robot.h"
#include "sbf/robot/interval_fk.h"
#include "sbf/robot/affine_fk.h"
#include "sbf/robot/fk.h"
#include "sbf/common/types.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>
#include <algorithm>
#include <Eigen/Core>

using namespace sbf;

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

// Check if method AABB contains the ground-truth (MC) AABB per-link.
// Returns true if for ALL links: method_lo <= mc_lo and method_hi >= mc_hi
static bool contains_mc(const float* method_aabb, const float* mc_aabb,
                        int n_links, float tol = 1e-4f) {
    for (int i = 0; i < n_links; ++i) {
        const float* m = method_aabb + i * 6;
        const float* g = mc_aabb + i * 6;
        for (int d = 0; d < 3; ++d) {
            if (m[d]   > g[d]   + tol) return false;  // method lo > mc lo
            if (m[3+d] < g[3+d] - tol) return false;  // method hi < mc hi
        }
    }
    return true;
}

// Generate random box intervals
static std::vector<Interval> random_box(const Robot& robot, double width,
                                        std::mt19937& rng) {
    int n = robot.n_joints();
    const auto& lim = robot.joint_limits();
    std::vector<Interval> ivs(n);

    for (int i = 0; i < n; ++i) {
        double lo = lim.limits[i].lo;
        double hi = lim.limits[i].hi;
        double range = hi - lo;
        double half_w = width * range * 0.5;

        std::uniform_real_distribution<double> dist(lo + half_w, hi - half_w);
        double center = dist(rng);
        ivs[i] = Interval(center - half_w, center + half_w);
    }
    return ivs;
}

// Compute ground-truth per-link AABB via Monte Carlo FK sampling
static void mc_link_aabbs(const Robot& robot,
                          const std::vector<Interval>& intervals,
                          int n_samples,
                          float* out_aabb,
                          std::mt19937& rng) {
    int n = robot.n_joints();
    int n_active = robot.n_active_links();
    const int* alm = robot.active_link_map();

    // Init to extreme
    for (int i = 0; i < n_active; ++i) {
        float* o = out_aabb + i * 6;
        o[0] = o[1] = o[2] =  1e30f;
        o[3] = o[4] = o[5] = -1e30f;
    }

    Eigen::VectorXd q(n);
    Eigen::Matrix4d buf[MAX_TF];

    for (int s = 0; s < n_samples; ++s) {
        // Random config within box
        for (int j = 0; j < n; ++j) {
            std::uniform_real_distribution<double> dist(intervals[j].lo, intervals[j].hi);
            q[j] = dist(rng);
        }

        // Compute FK
        int n_tf = fk_transforms_inplace(robot, q, buf);
        (void)n_tf;

        // Update per-active-link AABBs
        for (int i = 0; i < n_active; ++i) {
            int li = alm[i];
            // Link AABB: envelope of proximal (li) and distal (li+1) frame origins
            for (int d = 0; d < 3; ++d) {
                float p0 = static_cast<float>(buf[li](d, 3));
                float p1 = static_cast<float>(buf[li + 1](d, 3));
                float lo = std::min(p0, p1);
                float hi = std::max(p0, p1);
                out_aabb[i*6 + d]     = std::min(out_aabb[i*6 + d],     lo);
                out_aabb[i*6 + 3 + d] = std::max(out_aabb[i*6 + 3 + d], hi);
            }
        }
    }
}

// ─── Timing helper with warmup ──────────────────────────────────────────────

template<typename Func>
double bench_min_us(Func f, int warmup = 3, int iters = 20) {
    for (int i = 0; i < warmup; ++i) f();

    double best = 1e30;
    for (int i = 0; i < iters; ++i) {
        auto t0 = std::chrono::high_resolution_clock::now();
        f();
        auto t1 = std::chrono::high_resolution_clock::now();
        double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
        if (us < best) best = us;
    }
    return best;
}

// ─── Main ───────────────────────────────────────────────────────────────────
int main() {
    std::vector<std::string> robot_names = {"panda", "iiwa14"};
    std::vector<double> widths = {0.05, 0.1, 0.2, 0.4, 0.6, 0.8, 1.0};
    int n_trials = 100;
    int mc_samples = 10000;

    // CSV header
    std::printf("robot,n_joints,n_links,width,trial,"
                "ia_vol,aa_vol,mc_vol,"
                "ia_mc_ratio,aa_mc_ratio,aa_ia_ratio,"
                "ia_contains_mc,aa_contains_mc,"
                "ia_min_us,aa_min_us\n");

    for (const auto& rname : robot_names) {
        Robot robot = Robot::from_json(ROBOT_DIR + rname + ".json");
        int n = robot.n_joints();
        int n_active = robot.n_active_links();

        std::vector<float> ia_aabbs(n_active * 6);
        std::vector<float> aa_aabbs(n_active * 6);
        std::vector<float> mc_aabbs(n_active * 6);

        for (double w : widths) {
            std::mt19937 rng(42 + static_cast<int>(w * 1000));

            for (int trial = 0; trial < n_trials; ++trial) {
                auto ivs = random_box(robot, w, rng);

                // ── MC ground truth ──
                std::mt19937 mc_rng(12345 + trial);
                mc_link_aabbs(robot, ivs, mc_samples, mc_aabbs.data(), mc_rng);

                // ── IA timing (min of 20 runs) ──
                double ia_us = bench_min_us([&]{ compute_link_aabbs(robot, ivs, ia_aabbs.data()); });

                // ── AA timing (min of 20 runs) ──
                double aa_us = bench_min_us([&]{ aa_compute_link_aabbs(robot, ivs, aa_aabbs.data()); });

                // Final compute for correct values
                compute_link_aabbs(robot, ivs, ia_aabbs.data());
                aa_compute_link_aabbs(robot, ivs, aa_aabbs.data());

                double ia_vol = total_volume(ia_aabbs.data(), n_active);
                double aa_vol = total_volume(aa_aabbs.data(), n_active);
                double mc_vol = total_volume(mc_aabbs.data(), n_active);

                double ia_mc = (mc_vol > 1e-30) ? (ia_vol / mc_vol) : 999.0;
                double aa_mc = (mc_vol > 1e-30) ? (aa_vol / mc_vol) : 999.0;
                double aa_ia = (ia_vol > 1e-30) ? (aa_vol / ia_vol) : 999.0;

                bool ia_ok = contains_mc(ia_aabbs.data(), mc_aabbs.data(), n_active);
                bool aa_ok = contains_mc(aa_aabbs.data(), mc_aabbs.data(), n_active);

                std::printf("%s,%d,%d,%.2f,%d,"
                            "%.8e,%.8e,%.8e,"
                            "%.4f,%.4f,%.4f,"
                            "%d,%d,"
                            "%.2f,%.2f\n",
                            rname.c_str(), n, n_active, w, trial,
                            ia_vol, aa_vol, mc_vol,
                            ia_mc, aa_mc, aa_ia,
                            ia_ok ? 1 : 0, aa_ok ? 1 : 0,
                            ia_us, aa_us);
            }
        }
    }

    // ── Summary to stderr ──
    std::fprintf(stderr, "\n%6s  %4s | %8s %8s %8s | %5s %5s | %7s %7s\n",
                 "robot", "w", "IA/MC", "AA/MC", "AA/IA",
                 "IA_ok", "AA_ok", "IA_us", "AA_us");
    std::fprintf(stderr, "------  ---- | -------- -------- -------- | ----- ----- | ------- -------\n");

    for (const auto& rname : robot_names) {
        Robot robot = Robot::from_json(ROBOT_DIR + rname + ".json");
        int n_active = robot.n_active_links();

        std::vector<float> ia_aabbs(n_active * 6);
        std::vector<float> aa_aabbs(n_active * 6);
        std::vector<float> mc_aabbs(n_active * 6);

        for (double w : widths) {
            std::mt19937 rng(42 + static_cast<int>(w * 1000));
            double sum_ia_mc = 0, sum_aa_mc = 0, sum_aa_ia = 0;
            double sum_ia_us = 0, sum_aa_us = 0;
            int n_ia_ok = 0, n_aa_ok = 0;
            int N = 100;

            for (int trial = 0; trial < N; ++trial) {
                auto ivs = random_box(robot, w, rng);

                std::mt19937 mc_rng(12345 + trial);
                mc_link_aabbs(robot, ivs, mc_samples, mc_aabbs.data(), mc_rng);

                double ia_us = bench_min_us([&]{ compute_link_aabbs(robot, ivs, ia_aabbs.data()); });
                double aa_us = bench_min_us([&]{ aa_compute_link_aabbs(robot, ivs, aa_aabbs.data()); });

                compute_link_aabbs(robot, ivs, ia_aabbs.data());
                aa_compute_link_aabbs(robot, ivs, aa_aabbs.data());

                double ia_vol = total_volume(ia_aabbs.data(), n_active);
                double aa_vol = total_volume(aa_aabbs.data(), n_active);
                double mc_vol = total_volume(mc_aabbs.data(), n_active);

                sum_ia_mc += (mc_vol > 1e-30) ? (ia_vol / mc_vol) : 999.0;
                sum_aa_mc += (mc_vol > 1e-30) ? (aa_vol / mc_vol) : 999.0;
                sum_aa_ia += (ia_vol > 1e-30) ? (aa_vol / ia_vol) : 999.0;
                sum_ia_us += ia_us;
                sum_aa_us += aa_us;
                if (contains_mc(ia_aabbs.data(), mc_aabbs.data(), n_active)) ++n_ia_ok;
                if (contains_mc(aa_aabbs.data(), mc_aabbs.data(), n_active)) ++n_aa_ok;
            }

            std::fprintf(stderr, "%6s  %.2f | %8.2f %8.2f %8.2f | %3d/%d %3d/%d | %7.2f %7.2f\n",
                         rname.c_str(), w,
                         sum_ia_mc / N, sum_aa_mc / N, sum_aa_ia / N,
                         n_ia_ok, N, n_aa_ok, N,
                         sum_ia_us / N, sum_aa_us / N);
        }
    }

    return 0;
}
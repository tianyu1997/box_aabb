// ═══════════════════════════════════════════════════════════════════════════
//  v3 vs v4 Endpoint iAABB Pipeline Benchmark  (v4 side)
//
//  Sweeps all 4 endpoint sources (IFK, CritSample, Analytical, GCPC) on
//  N random C-space boxes of varying widths, measuring:
//    - Total per-link body AABB volume (includes radius)
//    - Cold-start time (ms)
//
//  Volume metric:
//    - IFK / CritSample: endpoint pipeline → extract_link_iaabbs (union + radius)
//    - Analytical / GCPC: direct solver call → raw per-link body AABB
//
//  NOTE: CritSample with GCPC cache (derive_crit_endpoints) is a fast
//        approximation but NOT the full GCPC method. GCPC must use
//        derive_aabb_with_gcpc() (phases A–G) to guarantee soundness.
//
//  Output CSV columns:
//    version, source, trial, width, n_active, volume, time_ms, n_eval
//
//  Build: cmake --build build --config Release --target exp_v3v4_epiAABB
//  Run:   build\Release\exp_v3v4_epiAABB.exe [n_trials] [seed]
// ═══════════════════════════════════════════════════════════════════════════
#define _USE_MATH_DEFINES
#include <cmath>

#include "sbf/robot/robot.h"
#include "sbf/robot/interval_math.h"
#include "sbf/envelope/endpoint_source.h"
#include "sbf/envelope/crit_sample.h"
#include "sbf/envelope/analytical_solve.h"
#include "sbf/envelope/gcpc.h"
#include "sbf/robot/fk.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <vector>

using namespace sbf;
using namespace sbf::envelope;

// ─── IIWA14 factory (identical to v3 benchmark) ─────────────────────────────

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

// ─── Random box generator (deterministic with seed) ─────────────────────────

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

// ─── Compute per-link iAABB volume from EndpointIAABBResult ─────────────
//  For IFK / CritSample: calls extract_link_iaabbs() to derive per-link body
//  AABBs (union of parent+child endpoint iAABBs + link_radius inflation),
//  matching v3's compute_link_volume metric.

static double compute_link_volume(const EndpointIAABBResult& result,
                                  const Robot& robot) {
    const int n_act = robot.n_active_links();
    std::vector<float> link_aabb(n_act * 6);
    extract_link_iaabbs(result, robot, link_aabb.data());

    double vol = 0.0;
    for (int i = 0; i < n_act; ++i) {
        const float* a = link_aabb.data() + i * 6;
        double dx = std::max(0.0, double(a[3] - a[0]));
        double dy = std::max(0.0, double(a[4] - a[1]));
        double dz = std::max(0.0, double(a[5] - a[2]));
        vol += dx * dy * dz;
    }
    return vol;
}

// ─── Compute raw per-link AABB volume from float[n_act × 6] ────────────
//  For Analytical / GCPC: directly sums solver-produced per-link body AABBs,
//  matching v3's compute_raw_link_volume metric (bypasses endpoint pipeline).

static double compute_raw_link_volume(const float* link_aabb, int n_act) {
    double vol = 0.0;
    for (int i = 0; i < n_act; ++i) {
        const float* a = link_aabb + i * 6;
        double dx = std::max(0.0, double(a[3] - a[0]));
        double dy = std::max(0.0, double(a[4] - a[1]));
        double dz = std::max(0.0, double(a[5] - a[2]));
        vol += dx * dy * dz;
    }
    return vol;
}

// ─── GCPC cache builder (same enumeration as v3) ────────────────────────────

static GcpcCache build_gcpc_cache(const Robot& robot) {
    const int n = robot.n_joints();
    const int n_act = robot.n_active_links();
    const int* link_map = robot.active_link_map();
    auto limits = robot.joint_limits();

    // kπ/2 values per joint
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

    // q₁: only [0, π]
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

// ─── Main ───────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    const int n_trials = (argc > 1) ? std::atoi(argv[1]) : 50;
    const int seed     = (argc > 2) ? std::atoi(argv[2]) : 42;

    Robot robot = make_iiwa14();
    const int n_act = robot.n_active_links();

    std::fprintf(stderr, "[v4] Building GCPC cache...\n");
    GcpcCache gcpc_cache = build_gcpc_cache(robot);
    std::fprintf(stderr, "[v4] GCPC cache built: %d links.\n",
                 gcpc_cache.n_links());

    // Configs for each source (dual_phase3=true is now the default)
    AnalyticalCriticalConfig anal_cfg = AnalyticalCriticalConfig::all_enabled();

    EndpointSourceConfig cfg_ifk  = EndpointSourceConfig::ifk();

    // CritSample: boundary kπ/2 only (no GCPC cache) — fast standalone method
    EndpointSourceConfig cfg_crit;
    cfg_crit.method = EndpointSource::CritSample;

    EndpointSourceConfig cfg_anal;
    cfg_anal.method = EndpointSource::Analytical;
    cfg_anal.analytical_config_ptr = &anal_cfg;
    cfg_anal.gcpc_cache = &gcpc_cache;  // also tighten endpoints via GCPC cache

    EndpointSourceConfig cfg_gcpc = EndpointSourceConfig::gcpc(&gcpc_cache);

    struct SourceEntry {
        const char* name;
        EndpointSourceConfig config;
    };
    SourceEntry sources[] = {
        {"IFK",        cfg_ifk},
        {"CritSample", cfg_crit},
        {"Analytical", cfg_anal},
        {"GCPC",       cfg_gcpc},
    };

    // Width distribution: 40% narrow, 40% medium, 20% wide
    const double widths_narrow[]  = {0.05, 0.10, 0.15};
    const double widths_medium[]  = {0.20, 0.30, 0.40};
    const double widths_wide[]    = {0.50, 0.70, 1.00};

    // Header
    std::printf("version,source,trial,width,n_active,volume,time_ms,n_eval\n");

    std::mt19937 rng(seed);
    BoxGenerator gen{robot, rng};

    for (int trial = 0; trial < n_trials; ++trial) {
        // Pick width category
        double frac;
        if (trial < n_trials * 4 / 10) {
            frac = widths_narrow[trial % 3];
        } else if (trial < n_trials * 8 / 10) {
            frac = widths_medium[(trial - n_trials * 4 / 10) % 3];
        } else {
            frac = widths_wide[(trial - n_trials * 8 / 10) % 3];
        }

        auto intervals = gen.generate(frac);

        // Compute max width for diagnostics
        double max_w = 0.0;
        for (const auto& iv : intervals)
            max_w = std::max(max_w, iv.hi - iv.lo);

        for (auto& src : sources) {
            constexpr int N_REPEATS = 3;
            double times[N_REPEATS];
            double vol = 0.0;
            int    n_eval = 0;

            const bool is_analytical = (std::strcmp(src.name, "Analytical") == 0);
            const bool is_gcpc       = (std::strcmp(src.name, "GCPC") == 0);

            if (is_analytical) {
                // ── Analytical: directly call solver → per-link body AABB ────
                // Matches v3 benchmark: bypass endpoint pipeline, use raw solver
                // output for per-link body AABBs (includes radius).
                std::vector<float> link_aabbs(n_act * 6);
                AnalyticalCriticalStats stats{};

                // Warmup
                derive_aabb_critical_analytical(
                    robot, intervals, 1, anal_cfg, link_aabbs.data(), &stats);

                // Timed runs
                for (int r = 0; r < N_REPEATS; ++r) {
                    stats = {};
                    auto t0 = std::chrono::high_resolution_clock::now();
                    derive_aabb_critical_analytical(
                        robot, intervals, 1, anal_cfg, link_aabbs.data(), &stats);
                    auto t1 = std::chrono::high_resolution_clock::now();
                    times[r] = std::chrono::duration<double, std::milli>(
                        t1 - t0).count();
                }

                vol = compute_raw_link_volume(link_aabbs.data(), n_act);
                n_eval = stats.n_phase0_vertices + stats.n_phase1_edges
                       + stats.n_phase2_faces
                       + stats.n_phase25a_pair1d + stats.n_phase25b_pair2d
                       + stats.n_phase3_interior;

            } else if (is_gcpc) {
                // ── GCPC: full solver (phases A–G) → per-link body AABB ──────
                // derive_aabb_with_gcpc = cache lookup + boundary kπ/2 + atan2
                // + face analysis. Guarantees soundness (no omissions).
                // CritSample with GCPC cache is NOT the same — it only does
                // cache lookup + kπ/2, which may miss critical points.
                std::vector<float> link_aabbs(n_act * 6);
                GcpcQueryStats gstats{};

                // Warmup
                gcpc_cache.derive_aabb_with_gcpc(
                    robot, intervals, 1, link_aabbs.data(), &gstats);

                // Timed runs
                for (int r = 0; r < N_REPEATS; ++r) {
                    gstats = {};
                    auto t0 = std::chrono::high_resolution_clock::now();
                    gcpc_cache.derive_aabb_with_gcpc(
                        robot, intervals, 1, link_aabbs.data(), &gstats);
                    auto t1 = std::chrono::high_resolution_clock::now();
                    times[r] = std::chrono::duration<double, std::milli>(
                        t1 - t0).count();
                }

                vol = compute_raw_link_volume(link_aabbs.data(), n_act);
                n_eval = gstats.n_cache_matches + gstats.n_boundary_kpi2
                       + gstats.n_boundary_atan2;

            } else {
                // ── IFK / CritSample: endpoint pipeline ─────────────────────
                // Warmup
                {
                    auto _ = compute_endpoint_iaabb(src.config, robot, intervals);
                    (void)_;
                }

                // Timed runs
                EndpointIAABBResult last_result;
                for (int r = 0; r < N_REPEATS; ++r) {
                    auto t0 = std::chrono::high_resolution_clock::now();
                    last_result = compute_endpoint_iaabb(
                        src.config, robot, intervals);
                    auto t1 = std::chrono::high_resolution_clock::now();
                    times[r] = std::chrono::duration<double, std::milli>(
                        t1 - t0).count();
                }

                vol = compute_link_volume(last_result, robot);
                n_eval = last_result.n_evaluations;
            }

            // Median time
            std::sort(times, times + N_REPEATS);
            double median_ms = times[N_REPEATS / 2];

            std::printf("v4,%s,%d,%.3f,%d,%.10e,%.6f,%d\n",
                        src.name, trial, frac, n_act,
                        vol, median_ms, n_eval);
        }

        if ((trial + 1) % 10 == 0) {
            std::fprintf(stderr, "[v4] %d/%d trials done.\n",
                         trial + 1, n_trials);
        }
    }

    std::fprintf(stderr, "[v4] All %d trials completed.\n", n_trials);
    return 0;
}

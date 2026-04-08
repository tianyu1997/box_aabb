// ═══════════════════════════════════════════════════════════════════════════
// Experiment 17 — Critical-Sample Envelope Hyperparameter Sweep
// ═══════════════════════════════════════════════════════════════════════════
//
// Goal: Find optimal hyperparameter settings for AABB_CRIT and OBB_CRIT
// envelope methods on 2D (planar), Panda (7-DOF), and IIWA14 (7-DOF) robots.
//
// Optimization objective:
//   MAXIMIZE envelope tightness (smaller volume = tighter)
//   MINIMIZE derivation time (cold_us)
//
// Hyperparameters swept:
//   1. CRIT_MAX_COMBOS:  {1000, 5000, 10000, 30000, 60000, 120000}
//   2. n_sub (subdivision): {1, 2, 4, 8, 16}
//   3. Envelope method:   {AABB_CRIT, OBB_CRIT}
//   4. Interval width:    {small, medium, large}
//   5. Fallback strategy: {lo_mid_hi, lo_hi_only, lo_quartiles_hi}
//
// Metrics per (robot, method, max_combos, n_sub, width, fallback, trial):
//   - derive_us:  cold derivation time (critical enum + FK + derive)
//   - volume_m3:  envelope bounding volume
//   - n_combos:   actual number of critical configurations evaluated
//   - collision:  bool (for reference)
//
// Robots:
//   - planar_2dof: synthetic 2-joint planar arm (DH constructed in-code)
//   - panda:       7-DOF Franka Emika Panda (from JSON)
//   - iiwa14:      7-DOF KUKA LBR iiwa14 (from JSON)
//
// Build:
//   cmake .. -DSBF_BUILD_EXPERIMENTS=ON
//   cmake --build . --target exp_17_crit_hyperparam --config Release
//
// Run:
//   ./exp_17_crit_hyperparam [--trials N] [--repeats N]
//
// ═══════════════════════════════════════════════════════════════════════════

#include "sbf/robot/robot.h"
#include "sbf/robot/fk.h"
#include "sbf/robot/interval_fk.h"
#include "sbf/scene/scene.h"
#include "sbf/scene/aabb_collision_checker.h"
#include "sbf/envelope/envelope_derive_critical.h"
#include "sbf/envelope/frame_store.h"
#include "sbf/envelope/envelope_derive.h"
#include "sbf/common/types.h"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
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

// ═══════════════════════════════════════════════════════════════════════════
//  Configurable CRIT_MAX_COMBOS — local reimplementation
// ═══════════════════════════════════════════════════════════════════════════
//
// We reimplement the critical-angle enumeration locally so we can vary
// CRIT_MAX_COMBOS as a parameter rather than using the compile-time constant
// from envelope_derive_critical.cpp.

// Build candidate angles for one joint interval
static std::vector<double> local_crit_angles(double lo, double hi) {
    std::vector<double> v = { lo, hi };
    for (int k = -20; k <= 20; ++k) {
        double a = k * HALF_PI;
        if (a > lo && a < hi)
            v.push_back(a);
    }
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end()), v.end());
    return v;
}

// Fallback strategies when combo count exceeds CRIT_MAX_COMBOS
enum class FallbackStrategy : int {
    LO_MID_HI       = 0,   // {lo, mid, hi} per joint (default in current code)
    LO_HI_ONLY      = 1,   // {lo, hi} per joint (minimal)
    LO_QUARTILES_HI = 2,   // {lo, q25, mid, q75, hi} per joint (finer)
};

static const char* fallback_name(FallbackStrategy s) {
    switch (s) {
        case FallbackStrategy::LO_MID_HI:       return "lo_mid_hi";
        case FallbackStrategy::LO_HI_ONLY:      return "lo_hi";
        case FallbackStrategy::LO_QUARTILES_HI: return "lo_q25_mid_q75_hi";
        default: return "unknown";
    }
}

static std::vector<double> fallback_angles(double lo, double hi, FallbackStrategy s) {
    double mid = 0.5 * (lo + hi);
    switch (s) {
        case FallbackStrategy::LO_HI_ONLY:
            return { lo, hi };
        case FallbackStrategy::LO_QUARTILES_HI:
            return { lo, 0.25*lo + 0.75*hi, mid, 0.75*lo + 0.25*hi, hi };
        case FallbackStrategy::LO_MID_HI:
        default:
            return { lo, mid, hi };
    }
}

// Build Cartesian-product sets for joints [0..n_joints-1] with configurable max_combos
static std::vector<std::vector<double>> local_build_csets(
    const std::vector<std::vector<double>>& per_joint,
    const std::vector<Interval>& intervals,
    int n_joints,
    long long max_combos,
    FallbackStrategy fallback)
{
    std::vector<std::vector<double>> csets(n_joints);
    long long total = 1;
    for (int j = 0; j < n_joints; ++j) {
        csets[j] = per_joint[j];
        total *= static_cast<long long>(csets[j].size());
        if (total > max_combos) {
            for (int j2 = 0; j2 < n_joints; ++j2)
                csets[j2] = fallback_angles(intervals[j2].lo, intervals[j2].hi, fallback);
            break;
        }
    }
    return csets;
}

// Recursive Cartesian-product enumeration
static void local_crit_enum(
    const std::vector<std::vector<double>>& csets,
    Eigen::VectorXd& q, int depth,
    const std::function<void()>& cb)
{
    if (depth == static_cast<int>(csets.size())) { cb(); return; }
    for (double v : csets[depth]) {
        q[depth] = v;
        local_crit_enum(csets, q, depth + 1, cb);
    }
}

// ── Local AABB critical derive with configurable max_combos ──────────────

struct CritResult {
    std::vector<float> aabbs;   // [n_active * n_sub * 6]
    int n_combos = 0;
    int n_slots  = 0;
};

static CritResult local_derive_aabb_critical(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub,
    long long max_combos,
    FallbackStrategy fallback)
{
    const int n       = robot.n_joints();
    const int n_act   = robot.n_active_links();
    const int* map    = robot.active_link_map();
    const double* rad = robot.active_link_radii();
    const int total_slots = n_act * n_sub;

    CritResult res;
    res.aabbs.resize(total_slots * 6);
    res.n_slots = total_slots;

    // Initialise all AABBs to empty
    for (int k = 0; k < total_slots; ++k) {
        res.aabbs[k*6+0] = res.aabbs[k*6+1] = res.aabbs[k*6+2] =  1e30f;
        res.aabbs[k*6+3] = res.aabbs[k*6+4] = res.aabbs[k*6+5] = -1e30f;
    }

    std::vector<std::vector<double>> per_joint(n);
    for (int j = 0; j < n; ++j)
        per_joint[j] = local_crit_angles(intervals[j].lo, intervals[j].hi);

    int max_V = 0;
    for (int ci = 0; ci < n_act; ++ci)
        max_V = std::max(max_V, map[ci]);
    int n_joints = std::min(max_V + 1, n);

    auto csets = local_build_csets(per_joint, intervals, n_joints, max_combos, fallback);

    Eigen::VectorXd q(n);
    for (int j = 0; j < n; ++j) q[j] = intervals[j].mid();

    const float inv_n = 1.0f / static_cast<float>(n_sub);
    int n_combos = 0;

    local_crit_enum(csets, q, 0, [&]() {
        ++n_combos;
        auto pos = fk_link_positions(robot, q);
        const int np = static_cast<int>(pos.size());

        for (int ci = 0; ci < n_act; ++ci) {
            int V = map[ci];
            if (V + 1 >= np) continue;
            const Eigen::Vector3d& p_prox = pos[V];
            const Eigen::Vector3d& p_dist = pos[V + 1];

            for (int s = 0; s < n_sub; ++s) {
                float t0 = s * inv_n;
                float t1 = (s + 1) * inv_n;
                Eigen::Vector3d s0 = p_prox + double(t0) * (p_dist - p_prox);
                Eigen::Vector3d s1 = p_prox + double(t1) * (p_dist - p_prox);

                float* a = res.aabbs.data() + (ci * n_sub + s) * 6;
                for (int k = 0; k < 3; ++k) {
                    float vs = static_cast<float>(s0[k]);
                    float ve = static_cast<float>(s1[k]);
                    a[k]     = std::min(a[k],     std::min(vs, ve));
                    a[k + 3] = std::max(a[k + 3], std::max(vs, ve));
                }
            }
        }
    });

    // Inflate by link radius
    for (int ci = 0; ci < n_act; ++ci) {
        float r = (rad) ? static_cast<float>(rad[ci]) : 0.f;
        for (int s = 0; s < n_sub; ++s) {
            float* a = res.aabbs.data() + (ci * n_sub + s) * 6;
            a[0] -= r; a[1] -= r; a[2] -= r;
            a[3] += r; a[4] += r; a[5] += r;
        }
    }

    res.n_combos = n_combos;
    return res;
}

// ── Local OBB critical derive ────────────────────────────────────────────

static constexpr int OBB_F = 15;  // floats per OBB slot

static void local_pca_obb(
    const std::vector<Eigen::Vector3d>& pts,
    float* out, double radius)
{
    if (pts.size() < 2) {
        std::fill(out, out + OBB_F, 0.0f);
        out[6] = 1.0f; out[10] = 1.0f; out[14] = 1.0f;
        return;
    }
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (auto& p : pts) mean += p;
    mean /= static_cast<double>(pts.size());

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (auto& p : pts) {
        Eigen::Vector3d d = p - mean;
        cov += d * d.transpose();
    }
    cov /= static_cast<double>(pts.size());
    cov.diagonal().array() += 1e-12;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    const Eigen::Vector3d ax = solver.eigenvectors().col(2).normalized();
    const Eigen::Vector3d ay = solver.eigenvectors().col(1).normalized();
    const Eigen::Vector3d az = (ax.cross(ay)).normalized();

    double lo[3] = {  1e30,  1e30,  1e30 };
    double hi[3] = { -1e30, -1e30, -1e30 };
    const Eigen::Vector3d* axes[3] = { &ax, &ay, &az };

    for (auto& p : pts) {
        for (int k = 0; k < 3; ++k) {
            double proj = axes[k]->dot(p);
            if (proj < lo[k]) lo[k] = proj;
            if (proj > hi[k]) hi[k] = proj;
        }
    }
    for (int k = 0; k < 3; ++k) { lo[k] -= radius; hi[k] += radius; }

    Eigen::Vector3d c_world = 0.5*(lo[0]+hi[0])*ax + 0.5*(lo[1]+hi[1])*ay + 0.5*(lo[2]+hi[2])*az;

    out[0]  = (float)c_world[0]; out[1]  = (float)c_world[1]; out[2]  = (float)c_world[2];
    out[3]  = (float)(0.5*(hi[0]-lo[0])); out[4]  = (float)(0.5*(hi[1]-lo[1])); out[5]  = (float)(0.5*(hi[2]-lo[2]));
    out[6]  = (float)ax[0]; out[7]  = (float)ax[1]; out[8]  = (float)ax[2];
    out[9]  = (float)ay[0]; out[10] = (float)ay[1]; out[11] = (float)ay[2];
    out[12] = (float)az[0]; out[13] = (float)az[1]; out[14] = (float)az[2];
}

struct ObbCritResult {
    std::vector<float> obbs;    // [n_active * n_sub * OBB_F]
    int n_combos = 0;
    int n_slots  = 0;
};

static ObbCritResult local_derive_obb_critical(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub,
    long long max_combos,
    FallbackStrategy fallback)
{
    const int n       = robot.n_joints();
    const int n_act   = robot.n_active_links();
    const int* map    = robot.active_link_map();
    const double* rad = robot.active_link_radii();
    const int total_slots = n_act * n_sub;

    ObbCritResult res;
    res.obbs.resize(total_slots * OBB_F, 0.f);
    res.n_slots = total_slots;

    std::vector<std::vector<double>> per_joint(n);
    for (int j = 0; j < n; ++j)
        per_joint[j] = local_crit_angles(intervals[j].lo, intervals[j].hi);

    int max_V = 0;
    for (int ci = 0; ci < n_act; ++ci)
        max_V = std::max(max_V, map[ci]);
    int n_joints = std::min(max_V + 1, n);

    auto csets = local_build_csets(per_joint, intervals, n_joints, max_combos, fallback);

    Eigen::VectorXd q(n);
    for (int j = 0; j < n; ++j) q[j] = intervals[j].mid();

    const float inv_n = 1.0f / static_cast<float>(n_sub);
    std::vector<std::vector<Eigen::Vector3d>> clouds(total_slots);
    int n_combos = 0;

    local_crit_enum(csets, q, 0, [&]() {
        ++n_combos;
        auto pos = fk_link_positions(robot, q);
        const int np = static_cast<int>(pos.size());

        for (int ci = 0; ci < n_act; ++ci) {
            int V = map[ci];
            if (V + 1 >= np) continue;
            const Eigen::Vector3d& p_prox = pos[V];
            const Eigen::Vector3d& p_dist = pos[V + 1];

            for (int s = 0; s < n_sub; ++s) {
                float t0 = s * inv_n;
                float t1 = (s + 1) * inv_n;
                Eigen::Vector3d s0 = p_prox + double(t0) * (p_dist - p_prox);
                Eigen::Vector3d s1 = p_prox + double(t1) * (p_dist - p_prox);
                auto& cloud = clouds[ci * n_sub + s];
                cloud.push_back(s0);
                cloud.push_back(s1);
            }
        }
    });

    for (int ci = 0; ci < n_act; ++ci) {
        double r = (rad) ? rad[ci] : 0.0;
        for (int s = 0; s < n_sub; ++s) {
            int idx = ci * n_sub + s;
            local_pca_obb(clouds[idx], res.obbs.data() + idx * OBB_F, r);
        }
    }

    res.n_combos = n_combos;
    return res;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Volume helpers
// ═══════════════════════════════════════════════════════════════════════════

static double volume_aabb(const float* aabb, int n) {
    double vol = 0;
    for (int i = 0; i < n; ++i) {
        const float* b = aabb + i * 6;
        vol += std::max(0., double(b[3]-b[0]))
             * std::max(0., double(b[4]-b[1]))
             * std::max(0., double(b[5]-b[2]));
    }
    return vol;
}

static double volume_obb(const float* obbs, int n) {
    double total = 0.0;
    for (int i = 0; i < n; ++i) {
        const float* p = obbs + i * OBB_F;
        double hx = p[3], hy = p[4], hz = p[5];
        if (hx > 0 && hy > 0 && hz > 0)
            total += 8.0 * double(hx) * double(hy) * double(hz);
    }
    return total;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Robot factories
// ═══════════════════════════════════════════════════════════════════════════

static Robot make_planar_2dof() {
    // 2-DOF planar arm: L1 = 1.0, L2 = 0.8
    // DH convention: alpha=0, a=L, d=0, theta=0 (revolute)
    std::vector<DHParam> dh = {
        {0.0, 1.0, 0.0, 0.0, 0},   // joint 0: L1 = 1.0
        {0.0, 0.8, 0.0, 0.0, 0},   // joint 1: L2 = 0.8
    };
    JointLimits lim;
    lim.limits.push_back({-PI, PI});
    lim.limits.push_back({-PI, PI});

    std::vector<double> radii = {0.0, 0.0};  // no capsule radius for 2D
    return Robot("planar_2dof", dh, lim, std::nullopt, radii);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Helpers
// ═══════════════════════════════════════════════════════════════════════════

static std::vector<Interval> random_intervals(
    const Robot& robot, std::mt19937& rng,
    double width_lo, double width_hi)
{
    std::uniform_real_distribution<double> wdist(width_lo, width_hi);
    int n = robot.n_joints();
    const auto& lim = robot.joint_limits();
    std::vector<Interval> ivs(n);
    for (int i = 0; i < n; ++i) {
        double lo_lim = lim.limits[i].lo;
        double hi_lim = lim.limits[i].hi;
        double w = wdist(rng);
        double range = hi_lim - lo_lim;
        if (w > range) w = range;
        std::uniform_real_distribution<double> cdist(lo_lim + w/2, hi_lim - w/2);
        double c = cdist(rng);
        ivs[i] = {c - w/2, c + w/2};
    }
    return ivs;
}

template<typename Fn>
static double median_us(Fn fn, int n_repeats) {
    std::vector<double> times;
    times.reserve(n_repeats);
    for (int r = 0; r < n_repeats; ++r) {
        auto t0 = Clock::now();
        fn();
        auto t1 = Clock::now();
        times.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
    }
    std::sort(times.begin(), times.end());
    return times[n_repeats / 2];
}

struct Stats {
    double mean, stddev, p50, p5, p95;
};
static Stats stats_of(const std::vector<double>& vals) {
    Stats s = {};
    if (vals.empty()) return s;
    int n = (int)vals.size();
    s.mean = std::accumulate(vals.begin(), vals.end(), 0.0) / n;
    double var = 0;
    for (double v : vals) var += (v - s.mean) * (v - s.mean);
    s.stddev = std::sqrt(var / std::max(n - 1, 1));
    auto sorted = vals;
    std::sort(sorted.begin(), sorted.end());
    s.p50 = sorted[n / 2];
    s.p5  = sorted[std::max(0, n * 5 / 100)];
    s.p95 = sorted[std::min(n - 1, n * 95 / 100)];
    return s;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Config
// ═══════════════════════════════════════════════════════════════════════════

struct ExpConfig {
    int n_trials  = 30;
    int n_repeats = 10;
};

// ─── Sweep dimensions ────────────────────────────────────────────────────

struct WidthRegime {
    const char* name;
    double lo, hi;
};
static const WidthRegime WIDTHS[] = {
    {"small",  0.05, 0.15},
    {"medium", 0.25, 0.50},
    {"large",  0.70, 1.40},
};
static constexpr int N_WIDTHS = 3;

static const long long MAX_COMBOS_SWEEP[] = {
    500, 1000, 5000, 10000, 30000, 60000, 120000
};
static constexpr int N_MAX_COMBOS = 7;

static const int SUBDIV_SWEEP[] = {1, 2, 4, 8, 16};
static constexpr int N_SUBDIV = 5;

static const FallbackStrategy FALLBACK_SWEEP[] = {
    FallbackStrategy::LO_MID_HI,
    FallbackStrategy::LO_HI_ONLY,
    FallbackStrategy::LO_QUARTILES_HI,
};
static constexpr int N_FALLBACK = 3;

enum Method { AABB_CRIT = 0, OBB_CRIT = 1 };
static const char* METHOD_NAMES[] = { "AABB_CRIT", "OBB_CRIT" };
static constexpr int N_METHODS = 2;

// ─── Trial row ───────────────────────────────────────────────────────────

struct TrialRow {
    int trial;
    const char* robot_name;
    const char* width;
    const char* method;
    long long max_combos;
    int subdiv_n;
    const char* fallback;
    double derive_us;
    double volume;
    int actual_combos;
};

// ═══════════════════════════════════════════════════════════════════════════
//  Run sweep for one robot
// ═══════════════════════════════════════════════════════════════════════════

static void run_robot_sweep(
    const Robot& robot,
    const ExpConfig& cfg,
    std::vector<TrialRow>& rows,
    const std::string& output_dir)
{
    const char* rname = robot.name().c_str();
    int n_active = robot.n_active_links();

    std::cout << "\n════════════════════════════════════════════════\n"
              << "Robot: " << rname << " (" << robot.n_joints()
              << " joints, " << n_active << " active links)\n"
              << "════════════════════════════════════════════════\n";

    std::mt19937 rng(42);
    int total_runs = N_WIDTHS * N_MAX_COMBOS * N_SUBDIV * N_FALLBACK * N_METHODS * cfg.n_trials;
    int progress = 0;

    for (int wi = 0; wi < N_WIDTHS; ++wi) {
        const auto& wr = WIDTHS[wi];
        std::cout << "  Width: " << wr.name << " ..." << std::flush;

        for (int trial = 0; trial < cfg.n_trials; ++trial) {
            auto ivs = random_intervals(robot, rng, wr.lo, wr.hi);

            for (int mi = 0; mi < N_METHODS; ++mi) {
                for (int ci = 0; ci < N_MAX_COMBOS; ++ci) {
                    long long mc = MAX_COMBOS_SWEEP[ci];

                    for (int fi = 0; fi < N_FALLBACK; ++fi) {
                        auto fb = FALLBACK_SWEEP[fi];

                        for (int si = 0; si < N_SUBDIV; ++si) {
                            int sub_n = SUBDIV_SWEEP[si];
                            ++progress;

                            double derive_t = 0;
                            double vol = 0;
                            int actual_combos = 0;

                            if (mi == AABB_CRIT) {
                                CritResult cr;
                                derive_t = median_us([&](){
                                    cr = local_derive_aabb_critical(
                                        robot, ivs, sub_n, mc, fb);
                                }, cfg.n_repeats);
                                vol = volume_aabb(cr.aabbs.data(), cr.n_slots);
                                actual_combos = cr.n_combos;
                            } else {
                                ObbCritResult cr;
                                derive_t = median_us([&](){
                                    cr = local_derive_obb_critical(
                                        robot, ivs, sub_n, mc, fb);
                                }, cfg.n_repeats);
                                vol = volume_obb(cr.obbs.data(), cr.n_slots);
                                actual_combos = cr.n_combos;
                            }

                            rows.push_back({
                                trial, rname, wr.name, METHOD_NAMES[mi],
                                mc, sub_n, fallback_name(fb),
                                derive_t, vol, actual_combos
                            });
                        }
                    }
                }
            }

            if ((trial + 1) % 10 == 0)
                std::cout << " " << (trial+1) << "/" << cfg.n_trials << std::flush;
        }
        std::cout << " done\n";
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Write CSV
// ═══════════════════════════════════════════════════════════════════════════

static void write_csv(const std::string& path, const std::vector<TrialRow>& rows) {
    std::ofstream f(path);
    f << "trial,robot,width,method,max_combos,subdiv_n,fallback,"
      << "derive_us,volume_m3,actual_combos\n";
    for (auto& r : rows) {
        f << r.trial << "," << r.robot_name << "," << r.width << ","
          << r.method << "," << r.max_combos << "," << r.subdiv_n << ","
          << r.fallback << ","
          << std::fixed << std::setprecision(3) << r.derive_us << ","
          << std::scientific << std::setprecision(6) << r.volume << ","
          << r.actual_combos << "\n";
    }
    std::cout << "CSV written: " << path << "\n";
}

// ═══════════════════════════════════════════════════════════════════════════
//  Write Markdown report
// ═══════════════════════════════════════════════════════════════════════════

static void write_report(const std::string& path,
                         const std::vector<TrialRow>& rows,
                         const std::vector<std::string>& robot_names)
{
    std::ofstream md(path);
    md << "# Experiment 17 — Critical-Sample Hyperparameter Sweep\n\n";

    md << "## Goal\n\n"
       << "Find optimal hyperparameter settings for AABB_CRIT and OBB_CRIT "
       << "envelope methods across 2D, Panda, and IIWA14 robots.\n\n"
       << "**Optimization**: maximize tightness (smaller volume) while "
       << "minimizing derivation time.\n\n";

    md << "## Hyperparameters Swept\n\n"
       << "| Parameter | Values |\n|---|---|\n"
       << "| CRIT_MAX_COMBOS | 500, 1000, 5000, 10000, 30000, 60000, 120000 |\n"
       << "| n_sub | 1, 2, 4, 8, 16 |\n"
       << "| Method | AABB_CRIT, OBB_CRIT |\n"
       << "| Fallback | lo_mid_hi, lo_hi, lo_q25_mid_q75_hi |\n"
       << "| Width | small (0.05-0.15), medium (0.25-0.50), large (0.70-1.40) |\n\n";

    // Per-robot summary tables
    for (auto& rname : robot_names) {
        md << "---\n\n## Robot: " << rname << "\n\n";

        // Find best configs per method (Pareto: lowest volume, reasonable time)
        for (int mi = 0; mi < N_METHODS; ++mi) {
            md << "### " << METHOD_NAMES[mi] << "\n\n";

            md << "#### Derive Time vs Volume (aggregated over all widths)\n\n"
               << "| max_combos | n_sub | fallback | Derive p50 us | Volume p50 m3 | Combos p50 |\n"
               << "|---|---|---|---|---|---|\n";

            // Group by (max_combos, n_sub, fallback) → aggregate
            struct Key {
                long long mc; int sub; std::string fb;
                bool operator<(const Key& o) const {
                    if (mc != o.mc) return mc < o.mc;
                    if (sub != o.sub) return sub < o.sub;
                    return fb < o.fb;
                }
            };
            std::map<Key, std::vector<const TrialRow*>> groups;
            for (auto& r : rows) {
                if (std::string(r.robot_name) != rname) continue;
                if (std::string(r.method) != METHOD_NAMES[mi]) continue;
                groups[{r.max_combos, r.subdiv_n, r.fallback}].push_back(&r);
            }
            for (auto& [key, rrs] : groups) {
                std::vector<double> dv, vv;
                std::vector<double> cv;
                for (auto* r : rrs) {
                    dv.push_back(r->derive_us);
                    vv.push_back(r->volume);
                    cv.push_back(r->actual_combos);
                }
                auto ds = stats_of(dv), vs = stats_of(vv), cs = stats_of(cv);
                md << "| " << key.mc << " | " << key.sub
                   << " | " << key.fb
                   << " | " << std::fixed << std::setprecision(1) << ds.p50
                   << " | " << std::scientific << std::setprecision(3) << vs.p50
                   << " | " << std::fixed << std::setprecision(0) << cs.p50
                   << " |\n";
            }
            md << "\n";

            // Per-width breakdown — best config
            for (int wi = 0; wi < N_WIDTHS; ++wi) {
                md << "#### Width: " << WIDTHS[wi].name << "\n\n"
                   << "| max_combos | n_sub | fallback | Derive p50 us | Volume p50 m3 | Combos p50 |\n"
                   << "|---|---|---|---|---|---|\n";

                std::map<Key, std::vector<const TrialRow*>> wgroups;
                for (auto& r : rows) {
                    if (std::string(r.robot_name) != rname) continue;
                    if (std::string(r.method) != METHOD_NAMES[mi]) continue;
                    if (std::string(r.width) != WIDTHS[wi].name) continue;
                    wgroups[{r.max_combos, r.subdiv_n, r.fallback}].push_back(&r);
                }
                for (auto& [key, rrs] : wgroups) {
                    std::vector<double> dv, vv, cv;
                    for (auto* r : rrs) {
                        dv.push_back(r->derive_us);
                        vv.push_back(r->volume);
                        cv.push_back(r->actual_combos);
                    }
                    auto ds = stats_of(dv), vs = stats_of(vv), cs = stats_of(cv);
                    md << "| " << key.mc << " | " << key.sub
                       << " | " << key.fb
                       << " | " << std::fixed << std::setprecision(1) << ds.p50
                       << " | " << std::scientific << std::setprecision(3) << vs.p50
                       << " | " << std::fixed << std::setprecision(0) << cs.p50
                       << " |\n";
                }
                md << "\n";
            }
        }

        // ── Pareto-front analysis ────────────────────────────────────────
        md << "### Pareto-Optimal Configurations (volume vs time)\n\n"
           << "> Configurations where no other setting achieves both lower "
           << "volume AND lower derive time (all widths combined).\n\n"
           << "| Method | max_combos | n_sub | fallback | "
           << "Derive p50 us | Volume p50 m3 |\n"
           << "|---|---|---|---|---|---|\n";

        struct ParetoPoint {
            std::string method; long long mc; int sub; std::string fb;
            double time, vol;
        };
        std::vector<ParetoPoint> all_points;

        for (int mi = 0; mi < N_METHODS; ++mi) {
            struct Key {
                long long mc; int sub; std::string fb;
                bool operator<(const Key& o) const {
                    if (mc != o.mc) return mc < o.mc;
                    if (sub != o.sub) return sub < o.sub;
                    return fb < o.fb;
                }
            };
            std::map<Key, std::vector<const TrialRow*>> groups;
            for (auto& r : rows) {
                if (std::string(r.robot_name) != rname) continue;
                if (std::string(r.method) != METHOD_NAMES[mi]) continue;
                groups[{r.max_combos, r.subdiv_n, r.fallback}].push_back(&r);
            }
            for (auto& [key, rrs] : groups) {
                std::vector<double> dv, vv;
                for (auto* r : rrs) { dv.push_back(r->derive_us); vv.push_back(r->volume); }
                auto ds = stats_of(dv), vs = stats_of(vv);
                all_points.push_back({METHOD_NAMES[mi], key.mc, key.sub, key.fb, ds.p50, vs.p50});
            }
        }

        // Extract Pareto front (minimize both time and volume)
        std::vector<ParetoPoint> pareto;
        for (auto& p : all_points) {
            bool dominated = false;
            for (auto& q : all_points) {
                if (&p == &q) continue;
                if (q.time <= p.time && q.vol <= p.vol &&
                    (q.time < p.time || q.vol < p.vol)) {
                    dominated = true;
                    break;
                }
            }
            if (!dominated) pareto.push_back(p);
        }
        std::sort(pareto.begin(), pareto.end(),
            [](const ParetoPoint& a, const ParetoPoint& b) { return a.time < b.time; });

        for (auto& p : pareto) {
            md << "| " << p.method << " | " << p.mc << " | " << p.sub
               << " | " << p.fb
               << " | " << std::fixed << std::setprecision(1) << p.time
               << " | " << std::scientific << std::setprecision(3) << p.vol
               << " |\n";
        }
        md << "\n";
    }

    // ── Recommended settings ─────────────────────────────────────────────
    md << "---\n\n## Recommended Settings\n\n"
       << "Based on Pareto analysis, pick the configuration that gives the "
       << "best trade-off between tightness and speed for each robot.\n\n"
       << "| Robot | Method | CRIT_MAX_COMBOS | n_sub | Fallback | Rationale |\n"
       << "|---|---|---|---|---|---|\n"
       << "| (see Pareto tables above) | | | | | |\n\n";

    std::cout << "Report written: " << path << "\n";
}

// ═══════════════════════════════════════════════════════════════════════════
//  MAIN
// ═══════════════════════════════════════════════════════════════════════════

int main(int argc, char* argv[]) {
    ExpConfig cfg;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--trials"  && i+1 < argc) cfg.n_trials  = std::stoi(argv[++i]);
        else if (arg == "--repeats" && i+1 < argc) cfg.n_repeats = std::stoi(argv[++i]);
        else if (arg == "--help") {
            std::cout << "Usage: exp_17_crit_hyperparam [--trials N] [--repeats N]\n";
            return 0;
        }
    }

    // ── Create output directory ──────────────────────────────────────────
    auto now = std::chrono::system_clock::now();
    char ts[64];
    { auto t = std::chrono::system_clock::to_time_t(now);
      std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", std::localtime(&t)); }
    std::string output_dir = "results/exp17_" + std::string(ts);
    fs::create_directories(output_dir);

    std::cout << "═══════════════════════════════════════════════════════════\n"
              << " Experiment 17 — Critical-Sample Hyperparameter Sweep\n"
              << "═══════════════════════════════════════════════════════════\n"
              << "Trials:  " << cfg.n_trials << "\n"
              << "Repeats: " << cfg.n_repeats << "\n"
              << "Output:  " << output_dir << "\n\n";

    std::vector<TrialRow> all_rows;
    std::vector<std::string> robot_names;

    // ── 1. Planar 2-DOF ──────────────────────────────────────────────────
    {
        Robot planar = make_planar_2dof();
        robot_names.push_back(planar.name());
        run_robot_sweep(planar, cfg, all_rows, output_dir);
    }

    // ── 2. Panda 7-DOF ──────────────────────────────────────────────────
    {
        fs::path exe_dir = fs::path(argv[0]).parent_path();
        std::string robot_json;
        for (auto& c : {
            exe_dir / ".." / ".." / "v1" / "configs",
            exe_dir / ".." / "v1" / "configs",
            exe_dir / ".." / ".." / ".." / "safeboxforest" / "v1" / "configs",
            fs::path("../v1/configs"),
            fs::path("../../v1/configs"),
        }) {
            fs::path p = c / "panda.json";
            if (fs::exists(p)) { robot_json = p.string(); break; }
        }
        if (!robot_json.empty()) {
            Robot panda = Robot::from_json(robot_json);
            robot_names.push_back(panda.name());
            run_robot_sweep(panda, cfg, all_rows, output_dir);
        } else {
            std::cerr << "WARNING: panda.json not found, skipping Panda\n";
        }
    }

    // ── 3. IIWA14 7-DOF ─────────────────────────────────────────────────
    {
        fs::path exe_dir = fs::path(argv[0]).parent_path();
        std::string robot_json;
        for (auto& c : {
            exe_dir / ".." / ".." / "v1" / "configs",
            exe_dir / ".." / "v1" / "configs",
            exe_dir / ".." / ".." / ".." / "safeboxforest" / "v1" / "configs",
            fs::path("../v1/configs"),
            fs::path("../../v1/configs"),
        }) {
            fs::path p = c / "iiwa14.json";
            if (fs::exists(p)) { robot_json = p.string(); break; }
        }
        if (!robot_json.empty()) {
            Robot iiwa = Robot::from_json(robot_json);
            robot_names.push_back(iiwa.name());
            run_robot_sweep(iiwa, cfg, all_rows, output_dir);
        } else {
            std::cerr << "WARNING: iiwa14.json not found, skipping IIWA14\n";
        }
    }

    // ── Write outputs ────────────────────────────────────────────────────
    std::cout << "\nTotal rows: " << all_rows.size() << "\n\n";
    write_csv(output_dir + "/hyperparam_sweep_raw.csv", all_rows);
    write_report(output_dir + "/report.md", all_rows, robot_names);

    // ── Console summary: best per robot per method ───────────────────────
    std::cout << "\n═══ Quick Summary: Best Config per Robot ═══\n";
    for (auto& rname : robot_names) {
        std::cout << "\nRobot: " << rname << "\n";
        for (int mi = 0; mi < N_METHODS; ++mi) {
            // Find config with lowest volume among those with derive < 200us
            struct Cand { long long mc; int sub; std::string fb; double time; double vol; };
            std::map<std::string, std::vector<const TrialRow*>> groups;
            for (auto& r : all_rows) {
                if (std::string(r.robot_name) != rname) continue;
                if (std::string(r.method) != METHOD_NAMES[mi]) continue;
                std::string key = std::to_string(r.max_combos) + "_"
                    + std::to_string(r.subdiv_n) + "_" + r.fallback;
                groups[key].push_back(&r);
            }
            double best_vol = 1e30;
            Cand best = {};
            for (auto& [key, rrs] : groups) {
                std::vector<double> dv, vv;
                for (auto* r : rrs) { dv.push_back(r->derive_us); vv.push_back(r->volume); }
                auto ds = stats_of(dv), vs = stats_of(vv);
                // Prefer configs with derive < 500us for 7-DOF, < 50us for 2-DOF
                double time_limit = (rname == "planar_2dof") ? 50.0 : 500.0;
                if (ds.p50 <= time_limit && vs.p50 < best_vol) {
                    best_vol = vs.p50;
                    best = {rrs[0]->max_combos, rrs[0]->subdiv_n,
                            rrs[0]->fallback, ds.p50, vs.p50};
                }
            }
            if (best_vol < 1e29) {
                std::cout << "  " << METHOD_NAMES[mi]
                          << ": max_combos=" << best.mc
                          << " n_sub=" << best.sub
                          << " fallback=" << best.fb
                          << " → derive=" << std::fixed << std::setprecision(1)
                          << best.time << "us"
                          << " vol=" << std::scientific << std::setprecision(3)
                          << best.vol << " m3\n";
            }
        }
    }

    std::cout << "\nDone. Results in " << output_dir << "\n";
    return 0;
}

// Diagnostic: compare analytical solver volume with v4 optimizations on/off
// and compare with v3-equivalent path (IFK clamp effect).
//
// Build: cmake --build build --config Release --target diag_analytical_vol
// Run:   build\Release\diag_analytical_vol.exe [n_trials] [seed]
#define _USE_MATH_DEFINES
#include <cmath>
#include "sbf/robot/robot.h"
#include "sbf/envelope/endpoint_source.h"
#include "sbf/envelope/analytical_solve.h"
#include "sbf/envelope/envelope_derive.h"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>
#include <cstring>

using namespace sbf;
using namespace sbf::envelope;

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

static double compute_raw_volume(const float* aabb, int n_act) {
    double vol = 0.0;
    for (int i = 0; i < n_act; ++i) {
        const float* a = aabb + i * 6;
        double dx = std::max(0.0, (double)(a[3] - a[0]));
        double dy = std::max(0.0, (double)(a[4] - a[1]));
        double dz = std::max(0.0, (double)(a[5] - a[2]));
        vol += dx * dy * dz;
    }
    return vol;
}

int main(int argc, char* argv[]) {
    const int n_trials = (argc > 1) ? std::atoi(argv[1]) : 20;
    const int seed     = (argc > 2) ? std::atoi(argv[2]) : 42;
    Robot robot = make_iiwa14();
    const int n_act = robot.n_active_links();

    // v4 default config (all opts on)
    AnalyticalCriticalConfig cfg_v4 = AnalyticalCriticalConfig::all_enabled();
    cfg_v4.dual_phase3 = true;

    // v3-equivalent config (all v4 opts off)
    AnalyticalCriticalConfig cfg_v3eq = AnalyticalCriticalConfig::all_enabled();
    cfg_v3eq.dual_phase3 = true;
    cfg_v3eq.enable_aa_pruning = false;
    cfg_v3eq.enable_p1_direct_coeff = false;
    cfg_v3eq.enable_p2_direct_coeff = false;
    cfg_v3eq.enable_p25_direct_coeff = false;
    cfg_v3eq.enable_p3_direct_coeff = false;
    cfg_v3eq.enable_candidate_dedup = false;

    // Ablation configs: disable one phase of direct_coeff at a time
    AnalyticalCriticalConfig cfg_no_p1dc = cfg_v4;
    cfg_no_p1dc.enable_p1_direct_coeff = false;

    AnalyticalCriticalConfig cfg_no_p2dc = cfg_v4;
    cfg_no_p2dc.enable_p2_direct_coeff = false;

    AnalyticalCriticalConfig cfg_no_p25dc = cfg_v4;
    cfg_no_p25dc.enable_p25_direct_coeff = false;

    AnalyticalCriticalConfig cfg_no_p3dc = cfg_v4;
    cfg_no_p3dc.enable_p3_direct_coeff = false;

    struct ConfigEntry {
        const char* name;
        AnalyticalCriticalConfig* cfg;
    };
    ConfigEntry configs[] = {
        {"v4_default",       &cfg_v4},
        {"v3_equivalent",    &cfg_v3eq},
        {"no_p1_dc",         &cfg_no_p1dc},
        {"no_p2_dc",         &cfg_no_p2dc},
        {"no_p25_dc",        &cfg_no_p25dc},
        {"no_p3_dc",         &cfg_no_p3dc},
    };

    const double widths_narrow[]  = {0.05, 0.10, 0.15};
    const double widths_medium[]  = {0.20, 0.30, 0.40};
    const double widths_wide[]    = {0.50, 0.70, 1.00};

    std::printf("config,trial,width,volume,n_eval,time_ms\n");

    std::mt19937 rng(seed);

    for (int trial = 0; trial < n_trials; ++trial) {
        double frac;
        if (trial < n_trials * 4 / 10)
            frac = widths_narrow[trial % 3];
        else if (trial < n_trials * 8 / 10)
            frac = widths_medium[(trial - n_trials * 4 / 10) % 3];
        else
            frac = widths_wide[(trial - n_trials * 8 / 10) % 3];

        const auto& lim = robot.joint_limits().limits;
        int n = robot.n_joints();
        std::vector<Interval> intervals(n);
        for (int j = 0; j < n; ++j) {
            double lo = lim[j].lo, hi = lim[j].hi, range = hi - lo;
            std::uniform_real_distribution<double> d(lo, hi);
            double c = d(rng);
            double hw = range * frac * 0.5;
            intervals[j].lo = std::max(lo, c - hw);
            intervals[j].hi = std::min(hi, c + hw);
        }

        for (auto& ce : configs) {
            std::vector<float> link_aabbs(n_act * 6);
            AnalyticalCriticalStats stats{};

            auto t0 = std::chrono::high_resolution_clock::now();
            derive_aabb_critical_analytical(
                robot, intervals, 1, *ce.cfg, link_aabbs.data(), &stats);
            auto t1 = std::chrono::high_resolution_clock::now();
            double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

            double vol = compute_raw_volume(link_aabbs.data(), n_act);
            int n_eval = stats.n_phase0_vertices + stats.n_phase1_edges
                       + stats.n_phase2_faces
                       + stats.n_phase25a_pair1d + stats.n_phase25b_pair2d
                       + stats.n_phase3_interior;

            std::printf("%s,%d,%.3f,%.10e,%d,%.4f\n",
                        ce.name, trial, frac, vol, n_eval, ms);
        }
    }
    return 0;
}

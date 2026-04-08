// ═══════════════════════════════════════════════════════════════════════════
//  v3 Analytical Solver Baseline — for comparison with v4
//
//  Outputs CSV with columns:
//    method, width, n_sub, volume, fk_total, fk_p1, fk_p2, fk_p25, fk_p3,
//    time_ms, checksum
//
//  Build: cmake --build build --config Release --target exp_analytical_v3_baseline
//  Run:   Release\exp_analytical_v3_baseline.exe
// ═══════════════════════════════════════════════════════════════════════════
#define _USE_MATH_DEFINES
#include <cmath>

#include "sbf/robot/robot.h"
#include "sbf/common/types.h"
#include "sbf/envelope/envelope_derive_critical.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace sbf;
using namespace sbf::envelope;

static Robot make_iiwa14() {
    std::vector<DHParam> dh = {
        {  0.0,     0.0, 0.1575, 0.0, 0},
        { -M_PI/2,  0.0, 0.0,    0.0, 0},
        {  M_PI/2,  0.0, 0.2025, 0.0, 0},
        {  M_PI/2,  0.0, 0.0,    0.0, 0},
        { -M_PI/2,  0.0, 0.2155, 0.0, 0},
        { -M_PI/2,  0.0, 0.0,    0.0, 0},
        {  M_PI/2,  0.0, 0.081,  0.0, 0},
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

static std::vector<Interval> make_centered_intervals(const Robot& robot, double width) {
    const int n = robot.n_joints();
    std::vector<Interval> ivs(n);
    for (int j = 0; j < n; ++j) {
        const auto& lim = robot.joint_limits().limits[j];
        const double center = 0.5 * (lim.lo + lim.hi);
        double lo = std::max(lim.lo, center - 0.5 * width);
        double hi = std::min(lim.hi, center + 0.5 * width);
        if (hi - lo < 1e-9) {
            const double eps = std::min(1e-3, 0.5 * (lim.hi - lim.lo));
            lo = std::max(lim.lo, center - eps);
            hi = std::min(lim.hi, center + eps);
        }
        ivs[j] = Interval(lo, hi);
    }
    return ivs;
}

int main() {
    Robot robot = make_iiwa14();
    const int n_act = robot.n_active_links();

    const std::vector<double> widths = {0.05, 0.10, 0.15, 0.20, 0.30, 0.50, 0.70, 1.00};
    const int warmup = 3;
    const int repeats = 10;
    const int n_sub = 1;

    std::printf("method,width,n_sub,volume,fk_total,fk_p1,fk_p2,fk_p25,fk_p3,"
                "vertices,edges,faces,pair1d,pair2d,interior,"
                "time_ms,checksum\n");

    for (double width : widths) {
        const auto ivs = make_centered_intervals(robot, width);

        // Warmup
        {
            auto cfg = AnalyticalCriticalConfig::all_enabled();
            std::vector<float> aabb(n_act * n_sub * 6);
            for (int r = 0; r < warmup; ++r)
                derive_aabb_critical_analytical(robot, ivs, n_sub, cfg, aabb.data(), nullptr);
        }

        // Timed runs
        auto cfg = AnalyticalCriticalConfig::all_enabled();
        double total_ms = 0.0;
        AnalyticalCriticalStats last_stats{};
        std::vector<float> aabb(n_act * n_sub * 6);

        for (int r = 0; r < repeats; ++r) {
            AnalyticalCriticalStats stats{};
            auto t0 = std::chrono::high_resolution_clock::now();
            derive_aabb_critical_analytical(robot, ivs, n_sub, cfg, aabb.data(), &stats);
            auto t1 = std::chrono::high_resolution_clock::now();
            total_ms += std::chrono::duration<double, std::milli>(t1 - t0).count();
            last_stats = stats;
        }

        // Compute volume + checksum
        double total_vol = 0.0;
        double checksum = 0.0;
        for (int ci = 0; ci < n_act * n_sub; ++ci) {
            const float* a = aabb.data() + ci * 6;
            double dx = std::max(0.0, double(a[3] - a[0]));
            double dy = std::max(0.0, double(a[4] - a[1]));
            double dz = std::max(0.0, double(a[5] - a[2]));
            total_vol += dx * dy * dz;
            for (int k = 0; k < 6; ++k) checksum += double(a[k]);
        }

        const double mean_ms = total_ms / repeats;
        const int fk_total = last_stats.n_phase1_fk_calls + last_stats.n_phase2_fk_calls
                           + last_stats.n_phase25_fk_calls + last_stats.n_phase3_fk_calls;

        std::printf("v3_analytical,%.2f,%d,%.10e,%d,%d,%d,%d,%d,"
                    "%d,%d,%d,%d,%d,%d,"
                    "%.6f,%.10e\n",
                    width, n_sub, total_vol, fk_total,
                    last_stats.n_phase1_fk_calls,
                    last_stats.n_phase2_fk_calls,
                    last_stats.n_phase25_fk_calls,
                    last_stats.n_phase3_fk_calls,
                    last_stats.n_phase0_vertices,
                    last_stats.n_phase1_edges,
                    last_stats.n_phase2_faces,
                    last_stats.n_phase25a_pair1d,
                    last_stats.n_phase25b_pair2d,
                    last_stats.n_phase3_interior,
                    mean_ms, checksum);

        std::fprintf(stderr, "  [v3 width=%.2f] vol=%.6e  fk=%d  time=%.2fms\n",
                     width, total_vol, fk_total, mean_ms);
    }

    return 0;
}

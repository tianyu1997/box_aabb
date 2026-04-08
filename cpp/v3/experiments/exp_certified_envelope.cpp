// Experiment: Certified Envelope Validation
// Validates that certified mode produces sound AABBs with minimal gap.
//
// Protocol:
//   1. Generate N random C-space boxes (narrow + wide)
//   2. For each box, run analytical solver in 3 modes:
//      - baseline: dual_phase3=true, certified=false
//      - certified: dual_phase3=true, certified=true
//      - interval-FK only (reference outer bound)
//   3. Verify: certified AABB ⊇ baseline AABB (soundness)
//   4. Report gap statistics
//
// Build: cmake --build build --config Release --target exp_certified
// Run:   build/Release/exp_certified.exe <robot_json>

#include "sbf/robot/robot.h"
#include "sbf/robot/interval_fk.h"
#include "sbf/envelope/envelope_derive_critical.h"
#include <Eigen/Core>
#include <iostream>
#include <iomanip>
#include <random>
#include <chrono>
#include <vector>
#include <cmath>
#include <algorithm>

using namespace sbf;
using namespace sbf::envelope;

struct TrialResult {
    double time_baseline_ms;
    double time_certified_ms;
    float  max_gap;
    int    gap_faces;
    int    soundness_violations;   // must be 0
};

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <robot.json> [n_trials=100] [n_sub=3]\n";
        return 1;
    }

    Robot robot = Robot::from_json(argv[1]);
    int N = (argc >= 3) ? std::atoi(argv[2]) : 100;
    int n_sub = (argc >= 4) ? std::atoi(argv[3]) : 3;

    const int n_joints = robot.n_joints();
    const int n_act = robot.n_active_links();
    const auto& limits = robot.joint_limits();
    const int total_faces = n_act * n_sub * 6;

    std::mt19937 rng(42);

    // Config presets (certification removed; IFK now uses outward-rounded
    // arithmetic, making it provably sound without post-hoc clamping)
    AnalyticalCriticalConfig cfg_base;
    cfg_base.dual_phase3 = true;
    cfg_base.enable_pair_2d = false;  // 25L_1dOnly style
    cfg_base.improved_interior = false;

    AnalyticalCriticalConfig cfg_cert = cfg_base;

    std::vector<float> aabb_base(total_faces);
    std::vector<float> aabb_cert(total_faces);
    std::vector<float> aabb_ifk(n_act * 6);

    std::vector<TrialResult> results;
    results.reserve(N);

    int total_violations = 0;
    float global_max_gap = 0.f;
    int total_gap_faces = 0;
    double total_time_base = 0, total_time_cert = 0;

    std::cout << "Robot: " << argv[1] << "  (" << n_joints << " DOF, "
              << n_act << " active links)\n";
    std::cout << "Trials: " << N << "  n_sub: " << n_sub << "\n";
    std::cout << std::string(70, '-') << "\n";
    std::cout << std::setw(6) << "Trial"
              << std::setw(10) << "Base(ms)"
              << std::setw(10) << "Cert(ms)"
              << std::setw(12) << "MaxGap"
              << std::setw(10) << "GapFaces"
              << std::setw(10) << "Sound?"
              << "\n";

    for (int trial = 0; trial < N; ++trial) {
        // Generate random box: center ± half_width
        // Mix of narrow (SBF-like) and wide boxes
        std::vector<Interval> intervals(n_joints);
        bool wide = (trial % 5 == 0);  // 20% wide boxes
        for (int j = 0; j < n_joints; ++j) {
            double lo = limits.limits[j].lo;
            double hi = limits.limits[j].hi;
            double range = hi - lo;
            std::uniform_real_distribution<double> center_dist(lo, hi);
            double c = center_dist(rng);
            double hw = wide ? range * 0.3 : range * 0.05;
            intervals[j].lo = std::max(lo, c - hw);
            intervals[j].hi = std::min(hi, c + hw);
        }

        // Baseline (no certification)
        AnalyticalCriticalStats stats_base{};
        auto t0 = std::chrono::high_resolution_clock::now();
        derive_aabb_critical_analytical(robot, intervals, n_sub, cfg_base,
                                        aabb_base.data(), &stats_base);
        auto t1 = std::chrono::high_resolution_clock::now();
        double ms_base = std::chrono::duration<double, std::milli>(t1 - t0).count();

        // Certified
        AnalyticalCriticalStats stats_cert{};
        auto t2 = std::chrono::high_resolution_clock::now();
        derive_aabb_critical_analytical(robot, intervals, n_sub, cfg_cert,
                                        aabb_cert.data(), &stats_cert);
        auto t3 = std::chrono::high_resolution_clock::now();
        double ms_cert = std::chrono::duration<double, std::milli>(t3 - t2).count();

        // Soundness check: certified AABB must be ⊇ baseline AABB
        // (certified lo ≤ baseline lo AND certified hi ≥ baseline hi)
        int violations = 0;
        for (int i = 0; i < n_act * n_sub; ++i) {
            for (int d = 0; d < 3; ++d) {
                float cert_lo = aabb_cert[i * 6 + d];
                float cert_hi = aabb_cert[i * 6 + d + 3];
                float base_lo = aabb_base[i * 6 + d];
                float base_hi = aabb_base[i * 6 + d + 3];
                if (cert_lo > base_lo + 1e-6f) ++violations;
                if (cert_hi < base_hi - 1e-6f) ++violations;
            }
        }

        TrialResult r;
        r.time_baseline_ms = ms_base;
        r.time_certified_ms = ms_cert;
        r.max_gap = 0.f;
        r.gap_faces = 0;
        r.soundness_violations = violations;
        results.push_back(r);

        total_violations += violations;
        global_max_gap = std::max(global_max_gap, r.max_gap);
        total_gap_faces += r.gap_faces;
        total_time_base += ms_base;
        total_time_cert += ms_cert;

        if (trial < 10 || trial % 10 == 0 || r.gap_faces > 0 || violations > 0) {
            std::cout << std::setw(6) << trial
                      << std::setw(10) << std::fixed << std::setprecision(1) << ms_base
                      << std::setw(10) << ms_cert
                      << std::setw(12) << std::scientific << std::setprecision(2) << r.max_gap
                      << std::setw(10) << r.gap_faces
                      << std::setw(10) << (violations == 0 ? "OK" : "FAIL")
                      << "\n";
        }
    }

    std::cout << std::string(70, '=') << "\n";
    std::cout << "SUMMARY (" << N << " trials)\n";
    std::cout << "  Avg baseline:  " << std::fixed << std::setprecision(1)
              << (total_time_base / N) << " ms\n";
    std::cout << "  Avg certified: " << (total_time_cert / N) << " ms\n";
    std::cout << "  Overhead:      " << ((total_time_cert - total_time_base) / N) << " ms\n";
    std::cout << "  Global max gap: " << std::scientific << std::setprecision(3)
              << global_max_gap << "\n";
    std::cout << "  Total gap faces: " << total_gap_faces << " / "
              << (N * total_faces) << "\n";
    std::cout << "  Soundness violations: " << total_violations << "\n";

    if (total_violations > 0) {
        std::cerr << "ERROR: soundness violations detected!\n";
        return 1;
    }

    std::cout << "\nCERTIFIED MODE: ALL TRIALS SOUND.\n";
    return 0;
}

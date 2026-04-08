// ═══════════════════════════════════════════════════════════════════════════
//  v3 vs v4 Link iAABB Pipeline Benchmark  (v3 side)
//
//  Fixes CritSample as the endpoint source (Stage 1), then sweeps all
//  link iAABB pipeline variants (Stage 2) on N random C-space boxes:
//
//    1. SubAABB(n_sub=1)        — single AABB per link
//    2. SubAABB(n_sub=4)        — 4 sub-segments per link
//    3. SubAABB(n_sub=16)       — 16 sub-segments per link
//    4. SubAABB_Grid(n_sub=16, R=64)  — rasterised byte grid
//    5. Hull16_Grid(n_sub=1, δ=0.01)  — sparse VoxelGrid
//
//  Measures:
//    - Stage 1 time (endpoint AABB generation via CritSample)
//    - Stage 2 time (link envelope construction, each variant)
//    - Volume (m³)
//
//  Uses identical random seed and box generation as the v4 counterpart
//  for direct comparison.
//
//  Output CSV columns:
//    version,envelope,trial,width,n_active,volume,ep_time_ms,env_time_ms,total_ms
//
//  Build: cmake --build build --config Release --target exp_v3v4_linkIAABB
//  Run:   build\Release\exp_v3v4_linkIAABB.exe [n_trials] [seed]
// ═══════════════════════════════════════════════════════════════════════════
#define _USE_MATH_DEFINES
#include <cmath>

#include "sbf/robot/robot.h"
#include "sbf/envelope/frame_source.h"
#include "sbf/envelope/envelope_type.h"
#include "sbf/common/types.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

using namespace sbf;
using namespace sbf::envelope;

// ─── IIWA14 factory (identical to v4 benchmark) ─────────────────────────────

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

// ─── Random box generator (deterministic with seed — identical to v4) ───────

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
    const int n_trials = (argc > 1) ? std::atoi(argv[1]) : 50;
    const int seed     = (argc > 2) ? std::atoi(argv[2]) : 42;

    Robot robot = make_iiwa14();
    const int n_act = robot.n_active_links();

    // CritSample endpoint source config
    EndpointSourceConfig src_cfg = EndpointSourceConfig::crit_sampling();

    // Envelope configs to benchmark
    struct EnvEntry {
        const char* name;
        EnvelopeTypeConfig config;
    };

    EnvelopeTypeConfig cfg_sub1;
    cfg_sub1.type = EnvelopeType::SubAABB;
    cfg_sub1.n_sub = 1;

    EnvelopeTypeConfig cfg_sub4;
    cfg_sub4.type = EnvelopeType::SubAABB;
    cfg_sub4.n_sub = 4;

    EnvelopeTypeConfig cfg_sub16;
    cfg_sub16.type = EnvelopeType::SubAABB;
    cfg_sub16.n_sub = 16;

    EnvelopeTypeConfig cfg_grid = EnvelopeTypeConfig::sub_aabb_grid();  // n_sub=16, R=64

    EnvelopeTypeConfig cfg_hull = EnvelopeTypeConfig::hull16_grid();    // n_sub=1, δ=0.01

    EnvEntry envs[] = {
        {"SubAABB_s1",    cfg_sub1},
        {"SubAABB_s4",    cfg_sub4},
        {"SubAABB_s16",   cfg_sub16},
        {"SubAABB_Grid",  cfg_grid},
        {"Hull16_Grid",   cfg_hull},
    };
    const int n_envs = sizeof(envs) / sizeof(envs[0]);

    // Width distribution: 40% narrow, 40% medium, 20% wide
    const double widths_narrow[]  = {0.05, 0.10, 0.15};
    const double widths_medium[]  = {0.20, 0.30, 0.40};
    const double widths_wide[]    = {0.50, 0.70, 1.00};

    // Header
    std::printf("version,envelope,trial,width,n_active,volume,ep_time_ms,env_time_ms,total_ms\n");

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

        // ── Stage 1: compute endpoint AABBs (CritSample) ────────────────
        // Warmup
        {
            auto _ = compute_endpoint_aabb(src_cfg, robot, intervals);
            (void)_;
        }

        constexpr int N_REPEATS = 5;

        // Timed run for endpoint source
        double ep_times[N_REPEATS];
        EndpointAABBResult ep_result;
        for (int r = 0; r < N_REPEATS; ++r) {
            auto t0 = std::chrono::high_resolution_clock::now();
            ep_result = compute_endpoint_aabb(src_cfg, robot, intervals);
            auto t1 = std::chrono::high_resolution_clock::now();
            ep_times[r] = std::chrono::duration<double, std::milli>(t1 - t0).count();
        }
        std::sort(ep_times, ep_times + N_REPEATS);
        double median_ep_ms = ep_times[N_REPEATS / 2];

        // ── Stage 2: compute link envelope (each variant) ───────────────
        for (int ei = 0; ei < n_envs; ++ei) {
            // Warmup
            {
                auto _ = compute_link_envelope(envs[ei].config, robot, ep_result);
                (void)_;
            }

            double env_times[N_REPEATS];
            EnvelopeResult env_result;
            for (int r = 0; r < N_REPEATS; ++r) {
                auto t0 = std::chrono::high_resolution_clock::now();
                env_result = compute_link_envelope(envs[ei].config, robot, ep_result);
                auto t1 = std::chrono::high_resolution_clock::now();
                env_times[r] = std::chrono::duration<double, std::milli>(t1 - t0).count();
            }
            std::sort(env_times, env_times + N_REPEATS);
            double median_env_ms = env_times[N_REPEATS / 2];

            std::printf("v3,%s,%d,%.3f,%d,%.10e,%.6f,%.6f,%.6f\n",
                        envs[ei].name,
                        trial, frac, n_act,
                        env_result.volume,
                        median_ep_ms,
                        median_env_ms,
                        median_ep_ms + median_env_ms);
        }

        if ((trial + 1) % 10 == 0) {
            std::fprintf(stderr, "[v3] %d/%d trials done.\n",
                         trial + 1, n_trials);
        }
    }

    std::fprintf(stderr, "[v3] All %d trials completed.\n", n_trials);
    return 0;
}

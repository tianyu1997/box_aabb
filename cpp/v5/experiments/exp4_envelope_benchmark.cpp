/**
 * exp4_envelope_benchmark.cpp — 实验 4: Envelope Pipeline Benchmark
 *
 * 对应论文 Tab 1 (Envelope Tightness) + Tab 2 相关数据.
 *
 * 比较 4 种 endpoint-iAABB 来源 × 3 种 envelope 表示的:
 *   - 平均 link-envelope volume (tightness)
 *   - 平均计算时间 (cold / warm)
 *
 * 设计:
 *   随机采样 N 个 box (interval set) 在关节限内,
 *   对每个 box 分别调用 compute_endpoint_iaabb + compute_link_envelope,
 *   测量 volume 和 timing.
 *
 * 用法:
 *   ./exp4_envelope_benchmark [--n-boxes N] [--quick] [--threads N]
 */

#include <sbf/core/robot.h>
#include <sbf/core/types.h>
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/envelope_type.h>
#include <sbf/scene/collision_checker.h>
#include "marcucci_scenes.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <string>
#include <vector>

using namespace sbf;

// ═══════════════════════════════════════════════════════════════════════════
// Random box generation
// ═══════════════════════════════════════════════════════════════════════════

struct RandomBox {
    std::vector<Interval> intervals;
};

std::vector<RandomBox> sample_boxes(const Robot& robot, int n,
                                     double width_lo, double width_hi,
                                     uint64_t seed) {
    std::mt19937_64 rng(seed);
    std::uniform_real_distribution<double> unif(0.0, 1.0);
    const auto& jl = robot.joint_limits();
    int ndim = robot.n_joints();

    std::vector<RandomBox> boxes;
    boxes.reserve(n);
    for (int i = 0; i < n; ++i) {
        RandomBox rb;
        rb.intervals.resize(ndim);
        for (int d = 0; d < ndim; ++d) {
            double w = width_lo + unif(rng) * (width_hi - width_lo);
            double range = jl.limits[d].hi - jl.limits[d].lo;
            if (w > range) w = range;
            double lo = jl.limits[d].lo + unif(rng) * (range - w);
            rb.intervals[d] = Interval{lo, lo + w};
        }
        boxes.push_back(std::move(rb));
    }
    return boxes;
}

// Compute volume of link envelope (sum of per-link AABB volumes)
double envelope_volume(const float* link_iaabbs, int n_active, int n_sub) {
    double vol = 0.0;
    int n_entries = n_active * n_sub;
    for (int i = 0; i < n_entries; ++i) {
        double dx = link_iaabbs[i * 6 + 3] - link_iaabbs[i * 6 + 0];
        double dy = link_iaabbs[i * 6 + 4] - link_iaabbs[i * 6 + 1];
        double dz = link_iaabbs[i * 6 + 5] - link_iaabbs[i * 6 + 2];
        vol += std::max(0.0, dx) * std::max(0.0, dy) * std::max(0.0, dz);
    }
    return vol;
}

// ═══════════════════════════════════════════════════════════════════════════
// Statistics
// ═══════════════════════════════════════════════════════════════════════════

struct Stats { double mean, std_dev, median; };

Stats compute_stats(std::vector<double>& d) {
    Stats s{};
    if (d.empty()) return s;
    std::sort(d.begin(), d.end());
    int n = static_cast<int>(d.size());
    s.median = d[n / 2];
    s.mean = std::accumulate(d.begin(), d.end(), 0.0) / n;
    double var = 0;
    for (double v : d) var += (v - s.mean) * (v - s.mean);
    s.std_dev = n > 1 ? std::sqrt(var / (n - 1)) : 0.0;
    return s;
}

// ═══════════════════════════════════════════════════════════════════════════
// Main
// ═══════════════════════════════════════════════════════════════════════════

int main(int argc, char** argv) {
    std::string robot_path = std::string(SBF_DATA_DIR) + "/iiwa14.json";
    int n_boxes = 500;
    int n_timing_repeats = 3;
    bool quick = false;
    double width_lo = 0.1, width_hi = 0.5;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--n-boxes" && i + 1 < argc) n_boxes = std::atoi(argv[++i]);
        else if (a == "--repeats" && i + 1 < argc) n_timing_repeats = std::atoi(argv[++i]);
        else if (a == "--quick") quick = true;
        else if (a[0] != '-') robot_path = a;
    }

    if (quick) { n_boxes = 50; n_timing_repeats = 1; }

    Robot robot = Robot::from_json(robot_path);

    // Use v4 planning limits (narrowed for Marcucci scene)
    {
        auto& lim = const_cast<JointLimits&>(robot.joint_limits());
        const double planning_limits[7][2] = {
            {-1.865488,  1.865691},
            {-0.100000,  1.086648},
            {-0.662656,  0.662338},
            {-2.094400, -0.371673},
            {-0.619251,  0.619534},
            {-1.095222,  1.257951},
            { 1.050209,  2.091190},
        };
        for (int j = 0; j < 7; ++j) {
            lim.limits[j].lo = planning_limits[j][0];
            lim.limits[j].hi = planning_limits[j][1];
        }
    }

    std::cout << "Robot: " << robot.name() << "  DOF=" << robot.n_joints()
              << "  active=" << robot.n_active_links() << "\n";
    std::cout << "n_boxes=" << n_boxes << "  width=[" << width_lo << "," << width_hi << "]\n";
    std::cout << "timing_repeats=" << n_timing_repeats << "\n\n";

    // Sample boxes
    auto boxes = sample_boxes(robot, n_boxes, width_lo, width_hi, 42);

    // Endpoint sources
    const char* ep_names[] = {"IFK", "CritSample", "Analytical"};
    EndpointSource ep_sources[] = {
        EndpointSource::IFK,
        EndpointSource::CritSample,
        EndpointSource::Analytical,
    };
    int n_ep = 3;  // GCPC omitted: requires pre-built GcpcCache

    // Envelope types
    const char* env_names[] = {"LinkIAABB", "LinkIAABB_Grid", "Hull16_Grid"};
    EnvelopeType env_types[] = {
        EnvelopeType::LinkIAABB,
        EnvelopeType::LinkIAABB_Grid,
        EnvelopeType::Hull16_Grid,
    };
    int n_env = 3;

    // Table header
    std::cout << std::string(100, '=') << "\n"
              << "  Envelope Pipeline Benchmark (Paper Tab 1)\n"
              << std::string(100, '=') << "\n\n";

    // Reference volume for ratio (IFK-LinkIAABB)
    double ref_volume = 0.0;

    std::cout << std::left << std::setw(14) << "Source"
              << std::setw(16) << "Envelope"
              << std::right
              << std::setw(12) << "Vol(m³)"
              << std::setw(12) << "Vol_std"
              << std::setw(12) << "Ratio"
              << std::setw(14) << "EP(μs)"
              << std::setw(14) << "Env(μs)"
              << std::setw(14) << "Total(μs)"
              << "\n" << std::string(100, '-') << "\n";

    for (int ei = 0; ei < n_ep; ++ei) {
        EndpointSourceConfig ep_cfg;
        ep_cfg.source = ep_sources[ei];
        if (ep_cfg.source == EndpointSource::CritSample)
            ep_cfg.n_samples_crit = 64;

        for (int vi = 0; vi < n_env; ++vi) {
            EnvelopeTypeConfig env_cfg;
            env_cfg.type = env_types[vi];
            env_cfg.n_subdivisions = 1;
            if (env_cfg.type != EnvelopeType::LinkIAABB) {
                env_cfg.grid_config.voxel_delta = 0.04f;
            }

            std::vector<double> volumes;
            std::vector<double> ep_times_us, env_times_us, total_times_us;

            for (int bi = 0; bi < n_boxes; ++bi) {
                // Warm-up (first call may populate caches)
                if (bi == 0) {
                    compute_endpoint_iaabb(robot, boxes[bi].intervals, ep_cfg);
                }

                for (int rep = 0; rep < n_timing_repeats; ++rep) {
                    // Endpoint
                    auto t0 = std::chrono::steady_clock::now();
                    auto ep_res = compute_endpoint_iaabb(robot, boxes[bi].intervals, ep_cfg);
                    auto t1 = std::chrono::steady_clock::now();

                    // Envelope
                    auto env_res = compute_link_envelope(
                        ep_res.endpoint_iaabbs.data(),
                        ep_res.n_active_links,
                        robot.active_link_radii(),
                        env_cfg);
                    auto t2 = std::chrono::steady_clock::now();

                    double ep_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
                    double env_us = std::chrono::duration<double, std::micro>(t2 - t1).count();

                    ep_times_us.push_back(ep_us);
                    env_times_us.push_back(env_us);
                    total_times_us.push_back(ep_us + env_us);

                    if (rep == 0) {
                        double vol = envelope_volume(
                            env_res.link_iaabbs.data(),
                            env_res.n_active_links,
                            env_res.n_subdivisions);
                        volumes.push_back(vol);
                    }
                }
            }

            auto sv = compute_stats(volumes);
            auto st = compute_stats(total_times_us);
            auto sep = compute_stats(ep_times_us);
            auto senv = compute_stats(env_times_us);

            if (ei == 0 && vi == 0) ref_volume = sv.mean;

            double ratio = ref_volume > 0 ? sv.mean / ref_volume : 0.0;

            std::cout << std::left << std::setw(14) << ep_names[ei]
                      << std::setw(16) << env_names[vi]
                      << std::right << std::fixed
                      << std::setw(12) << std::setprecision(6) << sv.mean
                      << std::setw(12) << std::setprecision(6) << sv.std_dev
                      << std::setw(12) << std::setprecision(3) << ratio
                      << std::setw(14) << std::setprecision(1) << sep.mean
                      << std::setw(14) << std::setprecision(1) << senv.mean
                      << std::setw(14) << std::setprecision(1) << st.mean
                      << "\n";
        }
        std::cout << "\n";
    }

    std::cout << std::string(100, '=') << "\n"
              << "  Reference (IFK-LinkIAABB): " << std::scientific << ref_volume << " m³\n"
              << "  Lower ratio = tighter = better\n"
              << std::string(100, '=') << "\n\n";

    std::cout << "  Exp 4 complete.\n";
    return 0;
}

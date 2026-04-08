// ═══════════════════════════════════════════════════════════════════════════
// Experiment 26 — AABB / Grid 全流程耗时对比
// ═══════════════════════════════════════════════════════════════════════════
//
// 三部分度量:
//   Part A — AABB 推导耗时 (IFK / CRIT 帧源 × n_sub∈{1,2,4,8})
//   Part B — Grid 推导耗时 (CRIT 帧源, n_sub∈{1,2,4,8})
//   Part C — 纯碰撞检测耗时 (full-scan): AABB_check vs Grid_scan vs Grid_AND
//
// Build:
//   cmake --build . --target exp_26_aabb_grid_timing --config Release
//
// Run:
//   ./exp_26_aabb_grid_timing [--trials 100] [--repeats 5]
//
// ═══════════════════════════════════════════════════════════════════════════

#include "sbf/robot/robot.h"
#include "sbf/robot/fk.h"
#include "sbf/robot/interval_fk.h"
#include "sbf/envelope/envelope_derive.h"
#include "sbf/envelope/envelope_derive_critical.h"
#include "sbf/envelope/grid_store.h"
#include "sbf/envelope/frame_store.h"
#include "sbf/envelope/collision_policy.h"
#include "sbf/common/config.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#ifdef _MSC_VER
#include <intrin.h>
static inline int popcnt64(uint64_t v) { return (int)__popcnt64(v); }
#else
static inline int popcnt64(uint64_t v) { return __builtin_popcountll(v); }
#endif

namespace fs = std::filesystem;
using namespace sbf;
using namespace sbf::envelope;
using Clock = std::chrono::high_resolution_clock;

// ─── World bounds ────────────────────────────────────────────────────────
static const float WORLD_BOUNDS[6] = {
    -0.8f, -1.2f, 0.0f,
     1.8f,  1.2f, 1.4f
};

// ─── CLI config ──────────────────────────────────────────────────────────
struct ExpConfig {
    int n_trials  = 100;
    int n_repeats = 5;
};

static ExpConfig parse_args(int argc, char** argv) {
    ExpConfig cfg;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--trials"  && i + 1 < argc) cfg.n_trials  = std::stoi(argv[++i]);
        if (a == "--repeats" && i + 1 < argc) cfg.n_repeats = std::stoi(argv[++i]);
    }
    return cfg;
}

// ─── Robot config finder ─────────────────────────────────────────────────
static std::string find_robot_config(const std::string& name) {
    std::vector<std::string> bases = {
        "../../../safeboxforest/v1/configs",
        "../../safeboxforest/v1/configs",
        "../safeboxforest/v1/configs",
        "../../../../safeboxforest/v1/configs",
    };
    for (auto& b : bases) {
        std::string p = b + "/" + name + ".json";
        if (fs::exists(p)) return p;
    }
    return "";
}

// ─── Width regimes ───────────────────────────────────────────────────────
struct WidthRegime {
    const char* name;
    double lo, hi;
};
static const WidthRegime WIDTHS[] = {
    {"small",  0.05, 0.15},
    {"medium", 0.15, 0.50},
    {"large",  0.50, 1.50},
};
static constexpr int N_WIDTHS = 3;

// ─── Random interval generator ──────────────────────────────────────────
static std::vector<Interval> random_intervals(
    const Robot& robot, const WidthRegime& wr, std::mt19937& rng)
{
    const int n = robot.n_joints();
    std::vector<Interval> ivls(n);
    for (int j = 0; j < n; ++j) {
        double lo_j = robot.joint_limits().limits[j].lo;
        double hi_j = robot.joint_limits().limits[j].hi;
        double range = hi_j - lo_j;
        double width = std::uniform_real_distribution<double>(
            wr.lo, wr.hi)(rng) * range;
        double center = std::uniform_real_distribution<double>(
            lo_j + width / 2.0, hi_j - width / 2.0)(rng);
        ivls[j].lo = center - width / 2.0;
        ivls[j].hi = center + width / 2.0;
    }
    return ivls;
}

// ─── Random obstacle generator ──────────────────────────────────────────
// Generates n_obs random obstacle AABBs inside WORLD_BOUNDS.
// Format: [lo_x, hi_x, lo_y, hi_y, lo_z, hi_z] per obstacle (interleaved).
static std::vector<float> random_obstacles(int n_obs, std::mt19937& rng) {
    std::vector<float> obs(n_obs * 6);
    for (int i = 0; i < n_obs; ++i) {
        float* o = obs.data() + i * 6;
        for (int c = 0; c < 3; ++c) {
            float lo = WORLD_BOUNDS[c];
            float hi = WORLD_BOUNDS[3 + c];
            float span = hi - lo;
            // obstacle size: 2-8% of workspace span per axis
            float size = std::uniform_real_distribution<float>(
                0.02f * span, 0.08f * span)(rng);
            float center = std::uniform_real_distribution<float>(
                lo + size / 2.0f, hi - size / 2.0f)(rng);
            o[c * 2 + 0] = center - size / 2.0f;  // lo
            o[c * 2 + 1] = center + size / 2.0f;  // hi
        }
    }
    return obs;
}

// ─── Statistics helper ───────────────────────────────────────────────────
struct Stats {
    double mean = 0, stddev = 0, min_val = 0, max_val = 0, median = 0;
};

static Stats compute_stats(std::vector<double>& vals) {
    Stats s;
    if (vals.empty()) return s;
    std::sort(vals.begin(), vals.end());
    s.min_val = vals.front();
    s.max_val = vals.back();
    s.median  = vals[vals.size() / 2];
    double sum = 0;
    for (double v : vals) sum += v;
    s.mean = sum / vals.size();
    double var_sum = 0;
    for (double v : vals) var_sum += (v - s.mean) * (v - s.mean);
    s.stddev = std::sqrt(var_sum / vals.size());
    return s;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Main
// ═════════════════════════════════════════════════════════════════════════════

int main(int argc, char** argv)
{
    auto cfg = parse_args(argc, argv);

    std::string panda_path = find_robot_config("panda");
    if (panda_path.empty()) {
        std::cerr << "ERROR: Cannot find panda.json. Run from build/ dir.\n";
        return 1;
    }

    Robot robot = Robot::from_json(panda_path);
    const int n       = robot.n_joints();
    const int n_act   = robot.n_active_links();
    const int n_frames = n + (robot.has_tool() ? 1 : 0);

    // Link radii
    std::vector<float> radii_f(n_act, 0.f);
    if (robot.has_link_radii()) {
        const double* rd = robot.active_link_radii();
        for (int i = 0; i < n_act; ++i)
            radii_f[i] = static_cast<float>(rd[i]);
    }
    float base_pos[3] = {0.f, 0.f, 0.f};

    std::mt19937 rng(42);

    const std::vector<int> N_SUBS = {1, 2, 4, 8};
    constexpr int MAX_SUB = 8;

    std::cout << "================================================================\n"
              << " Experiment 26: AABB / Grid 全流程耗时对比\n"
              << "================================================================\n"
              << " Robot: panda (" << n << " joints, " << n_act << " active links)\n"
              << " Trials: " << cfg.n_trials
              << "  Repeats: " << cfg.n_repeats
              << "  Grid R: " << GRID_R << "\n\n";

    // CSV output
    auto now = std::chrono::system_clock::now();
    auto t   = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf{};
#ifdef _WIN32
    localtime_s(&tm_buf, &t);
#else
    localtime_r(&t, &tm_buf);
#endif
    std::ostringstream dir_oss;
    dir_oss << "../results/exp26_aabb_grid_timing_"
            << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
    std::string out_dir = dir_oss.str();
    fs::create_directories(out_dir);

    // ═════════════════════════════════════════════════════════════════════
    //  Part A: AABB 推导耗时 (IFK / CRIT × n_sub)
    // ═════════════════════════════════════════════════════════════════════

    std::cout << "────────────────────────────────────────────────────────────\n"
              << " Part A: AABB 推导耗时\n"
              << "────────────────────────────────────────────────────────────\n\n";

    {
        std::ofstream csv(out_dir + "/part_a_aabb_derive.csv");
        csv << "width,frame_src,n_sub,trial,derive_us\n";

        // Header
        std::cout << std::left << std::setw(8) << "Width"
                  << std::setw(8) << "Frame"
                  << std::setw(6) << "nSub"
                  << std::right
                  << std::setw(12) << "Mean(us)"
                  << std::setw(12) << "Median(us)"
                  << std::setw(12) << "Std(us)"
                  << std::setw(12) << "Min(us)"
                  << std::setw(12) << "Max(us)"
                  << "\n"
                  << std::string(82, '-') << "\n";

        // Frame buffers
        std::vector<float> ifk_frames(n_frames * 6);
        std::vector<float> crit_frames(n_frames * 6);

        // AABB buffer
        std::vector<float> aabb_buf(n_act * MAX_SUB * 6);

        for (auto& wr : WIDTHS) {
            for (const char* frame_src : {"IFK", "CRIT"}) {
                for (int n_sub : N_SUBS) {
                    std::vector<double> times;
                    times.reserve(cfg.n_trials);

                    for (int trial = 0; trial < cfg.n_trials; ++trial) {
                        auto ivls = random_intervals(robot, wr, rng);

                        // Derive frames
                        if (std::string(frame_src) == "IFK") {
                            FKState fk = compute_fk_full(robot, ivls);
                            for (int k = 0; k < n_frames; ++k) {
                                const double* plo = fk.prefix_lo[k + 1];
                                const double* phi = fk.prefix_hi[k + 1];
                                float* f = ifk_frames.data() + k * 6;
                                f[0] = static_cast<float>(plo[3]);
                                f[1] = static_cast<float>(plo[7]);
                                f[2] = static_cast<float>(plo[11]);
                                f[3] = static_cast<float>(phi[3]);
                                f[4] = static_cast<float>(phi[7]);
                                f[5] = static_cast<float>(phi[11]);
                            }
                        } else {
                            derive_crit_frames(robot, ivls,
                                               crit_frames.data(), nullptr);
                        }

                        const float* frames = (std::string(frame_src) == "IFK")
                                              ? ifk_frames.data()
                                              : crit_frames.data();

                        // Warmup
                        for (int i = 0; i < n_act; ++i) {
                            int frame_idx  = robot.active_link_map()[i];
                            int parent_idx = frame_idx - 1;
                            derive_aabb_subdivided(
                                frames, n_frames,
                                parent_idx, frame_idx,
                                n_sub, radii_f[i], base_pos,
                                aabb_buf.data() + i * n_sub * 6);
                        }

                        // Timed — take best of repeats
                        double best_us = 1e18;
                        for (int rep = 0; rep < cfg.n_repeats; ++rep) {
                            auto t0 = Clock::now();
                            for (int i = 0; i < n_act; ++i) {
                                int frame_idx  = robot.active_link_map()[i];
                                int parent_idx = frame_idx - 1;
                                derive_aabb_subdivided(
                                    frames, n_frames,
                                    parent_idx, frame_idx,
                                    n_sub, radii_f[i], base_pos,
                                    aabb_buf.data() + i * n_sub * 6);
                            }
                            auto t1 = Clock::now();
                            double us = std::chrono::duration<double,
                                        std::micro>(t1 - t0).count();
                            best_us = std::min(best_us, us);
                        }

                        times.push_back(best_us);
                        csv << wr.name << "," << frame_src << ","
                            << n_sub << "," << trial << ","
                            << std::fixed << std::setprecision(2)
                            << best_us << "\n";
                    }

                    auto st = compute_stats(times);
                    std::cout << std::left << std::setw(8) << wr.name
                              << std::setw(8) << frame_src
                              << std::setw(6) << n_sub
                              << std::right << std::fixed << std::setprecision(2)
                              << std::setw(12) << st.mean
                              << std::setw(12) << st.median
                              << std::setw(12) << st.stddev
                              << std::setw(12) << st.min_val
                              << std::setw(12) << st.max_val
                              << "\n";
                }
            }
        }
        std::cout << "\n";
    }

    // ═════════════════════════════════════════════════════════════════════
    //  Part B: Grid 推导耗时 (CRIT 帧源, n_sub)
    // ═════════════════════════════════════════════════════════════════════

    std::cout << "────────────────────────────────────────────────────────────\n"
              << " Part B: Grid 推导耗时 (CRIT frames)\n"
              << "────────────────────────────────────────────────────────────\n\n";

    {
        std::ofstream csv(out_dir + "/part_b_grid_derive.csv");
        csv << "width,n_sub,trial,derive_us,occupied\n";

        std::cout << std::left << std::setw(8) << "Width"
                  << std::setw(6) << "nSub"
                  << std::right
                  << std::setw(12) << "Mean(us)"
                  << std::setw(12) << "Median(us)"
                  << std::setw(12) << "Std(us)"
                  << std::setw(12) << "Min(us)"
                  << std::setw(12) << "Max(us)"
                  << std::setw(10) << "MeanOcc"
                  << "\n"
                  << std::string(84, '-') << "\n";

        std::vector<float> crit_frames(n_frames * 6);

        // GridStore for derivation
        GridStore gs(n_frames, n_act,
                     robot.active_link_map(), radii_f.data(),
                     base_pos, WORLD_BOUNDS, /*initial_cap=*/4);

        for (auto& wr : WIDTHS) {
            for (int n_sub : N_SUBS) {
                std::vector<double> times;
                std::vector<double> occ_vals;
                times.reserve(cfg.n_trials);
                occ_vals.reserve(cfg.n_trials);

                for (int trial = 0; trial < cfg.n_trials; ++trial) {
                    auto ivls = random_intervals(robot, wr, rng);
                    derive_crit_frames(robot, ivls,
                                       crit_frames.data(), nullptr);

                    // Warmup
                    gs.derive_from_frames(0, crit_frames.data(), n_sub);

                    // Timed — best of repeats
                    double best_us = 1e18;
                    for (int rep = 0; rep < cfg.n_repeats; ++rep) {
                        auto t0 = Clock::now();
                        gs.derive_from_frames(0, crit_frames.data(), n_sub);
                        auto t1 = Clock::now();
                        double us = std::chrono::duration<double,
                                    std::micro>(t1 - t0).count();
                        best_us = std::min(best_us, us);
                    }

                    times.push_back(best_us);
                    occ_vals.push_back(gs.occupied_count(0));

                    csv << wr.name << "," << n_sub << ","
                        << trial << ","
                        << std::fixed << std::setprecision(2) << best_us << ","
                        << gs.occupied_count(0) << "\n";
                }

                auto st = compute_stats(times);
                double occ_mean = 0;
                for (double v : occ_vals) occ_mean += v;
                occ_mean /= occ_vals.size();

                std::cout << std::left << std::setw(8) << wr.name
                          << std::setw(6) << n_sub
                          << std::right << std::fixed << std::setprecision(2)
                          << std::setw(12) << st.mean
                          << std::setw(12) << st.median
                          << std::setw(12) << st.stddev
                          << std::setw(12) << st.min_val
                          << std::setw(12) << st.max_val
                          << std::setprecision(0)
                          << std::setw(10) << occ_mean
                          << "\n";
            }
        }
        std::cout << "\n";
    }

    // ═════════════════════════════════════════════════════════════════════
    //  Part C: 纯碰撞检测耗时 — Full-scan (无 early-exit)
    //
    //  所有方法均预先推导好 envelope，然后只计时碰撞检测部分。
    //  全扫描：遍历所有 (box×obs) 或 (obs×voxels)，统计碰撞次数。
    //
    //  三种方法:
    //    AABB_check  — 预推导 AABB[n_act*n_sub], 逐box逐obs做SAT
    //                  复杂度 O(n_act * n_sub * n_obs)
    //    Grid_scan   — 预推导 Grid bitfield, 逐obs体素化并探测位
    //                  复杂度 O(n_obs * voxels_per_obs)
    //    Grid_AND    — 预推导 robot Grid + obstacle Grid, 512-word AND
    //                  复杂度 O(512) ≈ 常数，与 n_obs 无关
    // ═════════════════════════════════════════════════════════════════════

    std::cout << "────────────────────────────────────────────────────────────\n"
              << " Part C: 碰撞检测耗时 (full-scan, 无 early-exit)\n"
              << "────────────────────────────────────────────────────────────\n\n";

    {
        std::ofstream csv(out_dir + "/part_c_collision.csv");
        csv << "width,n_sub,n_obs,method,trial,check_us,n_hits\n";

        const std::vector<int> N_OBS_LIST = {1, 5, 10, 50, 100};

        std::cout << std::left << std::setw(8) << "Width"
                  << std::setw(6) << "nSub"
                  << std::setw(7) << "nObs"
                  << std::setw(12) << "Method"
                  << std::right
                  << std::setw(12) << "Mean(us)"
                  << std::setw(12) << "Median(us)"
                  << std::setw(12) << "Std(us)"
                  << std::setw(12) << "MeanHits"
                  << "\n"
                  << std::string(81, '-') << "\n";

        std::vector<float> crit_frames(n_frames * 6);

        // GridStore for pre-derived grid
        GridStore gs(n_frames, n_act,
                     robot.active_link_map(), radii_f.data(),
                     base_pos, WORLD_BOUNDS, /*initial_cap=*/4);

        // Pre-computed inv_cell for voxelization
        float inv_cell[3];
        for (int c = 0; c < 3; ++c)
            inv_cell[c] = float(GRID_R) / (WORLD_BOUNDS[3 + c] - WORLD_BOUNDS[c]);

        // AABB buffer for pre-derived AABBs
        std::vector<float> aabb_buf(n_act * MAX_SUB * 6);

        // Obstacle grid buffer (512 × uint64_t)
        std::vector<uint64_t> obs_grid(WORDS_PER_NODE, 0);

        for (auto& wr : WIDTHS) {
            for (int n_sub : {1, 4, 8}) {
                for (int n_obs : N_OBS_LIST) {

                    // ─── Method names ────────────────────────────────────
                    const char* method_names[] = {
                        "AABB_check", "Grid_scan", "Grid_AND"
                    };

                    for (int mi = 0; mi < 3; ++mi) {
                        std::vector<double> times;
                        std::vector<double> hit_counts;
                        times.reserve(cfg.n_trials);
                        hit_counts.reserve(cfg.n_trials);

                        for (int trial = 0; trial < cfg.n_trials; ++trial) {
                            auto ivls = random_intervals(robot, wr, rng);
                            derive_crit_frames(robot, ivls,
                                               crit_frames.data(), nullptr);
                            auto obs = random_obstacles(n_obs, rng);

                            // ── Pre-derive AABB (for AABB_check) ─────────
                            const int total_boxes = n_act * n_sub;
                            if (mi == 0) {
                                for (int i = 0; i < n_act; ++i) {
                                    int frame_idx  = robot.active_link_map()[i];
                                    int parent_idx = frame_idx - 1;
                                    derive_aabb_subdivided(
                                        crit_frames.data(), n_frames,
                                        parent_idx, frame_idx,
                                        n_sub, radii_f[i], base_pos,
                                        aabb_buf.data() + i * n_sub * 6);
                                }
                            }

                            // ── Pre-derive Grid (for Grid_scan / Grid_AND)
                            if (mi == 1 || mi == 2) {
                                gs.derive_from_frames(0,
                                    crit_frames.data(), n_sub);
                            }

                            // ── Pre-build obstacle grid (for Grid_AND) ───
                            if (mi == 2) {
                                std::memset(obs_grid.data(), 0,
                                            WORDS_PER_NODE * sizeof(uint64_t));
                                for (int oi = 0; oi < n_obs; ++oi) {
                                    const float* ob = obs.data() + oi * 6;
                                    int ix0 = std::max(0, int(std::floor(
                                        (ob[0] - WORLD_BOUNDS[0]) * inv_cell[0])));
                                    int iy0 = std::max(0, int(std::floor(
                                        (ob[2] - WORLD_BOUNDS[1]) * inv_cell[1])));
                                    int iz0 = std::max(0, int(std::floor(
                                        (ob[4] - WORLD_BOUNDS[2]) * inv_cell[2])));
                                    int ix1 = std::min(GRID_R, int(std::ceil(
                                        (ob[1] - WORLD_BOUNDS[0]) * inv_cell[0])));
                                    int iy1 = std::min(GRID_R, int(std::ceil(
                                        (ob[3] - WORLD_BOUNDS[1]) * inv_cell[1])));
                                    int iz1 = std::min(GRID_R, int(std::ceil(
                                        (ob[5] - WORLD_BOUNDS[2]) * inv_cell[2])));
                                    for (int x = ix0; x < ix1; ++x)
                                        for (int y = iy0; y < iy1; ++y)
                                            for (int z = iz0; z < iz1; ++z) {
                                                int lin = x * GRID_R * GRID_R
                                                        + y * GRID_R + z;
                                                obs_grid[lin >> 6] |=
                                                    uint64_t(1) << (lin & 63);
                                            }
                                }
                            }

                            // ── Warmup ───────────────────────────────────
                            volatile int warmup_hits = 0;
                            if (mi == 0) {
                                // AABB full-scan
                                for (int b = 0; b < total_boxes; ++b) {
                                    const float* a = aabb_buf.data() + b * 6;
                                    for (int oi = 0; oi < n_obs; ++oi) {
                                        const float* o = obs.data() + oi * 6;
                                        if (a[3] >= o[0] && a[0] <= o[1] &&
                                            a[4] >= o[2] && a[1] <= o[3] &&
                                            a[5] >= o[4] && a[2] <= o[5])
                                            warmup_hits++;
                                    }
                                }
                            } else if (mi == 1) {
                                // Grid per-obs scan
                                const uint64_t* grid = gs.get_grid(0);
                                for (int oi = 0; oi < n_obs; ++oi) {
                                    const float* ob = obs.data() + oi * 6;
                                    int ix0 = std::max(0, int(std::floor(
                                        (ob[0] - WORLD_BOUNDS[0]) * inv_cell[0])));
                                    int iy0 = std::max(0, int(std::floor(
                                        (ob[2] - WORLD_BOUNDS[1]) * inv_cell[1])));
                                    int iz0 = std::max(0, int(std::floor(
                                        (ob[4] - WORLD_BOUNDS[2]) * inv_cell[2])));
                                    int ix1 = std::min(GRID_R, int(std::ceil(
                                        (ob[1] - WORLD_BOUNDS[0]) * inv_cell[0])));
                                    int iy1 = std::min(GRID_R, int(std::ceil(
                                        (ob[3] - WORLD_BOUNDS[1]) * inv_cell[1])));
                                    int iz1 = std::min(GRID_R, int(std::ceil(
                                        (ob[5] - WORLD_BOUNDS[2]) * inv_cell[2])));
                                    for (int x = ix0; x < ix1; ++x)
                                        for (int y = iy0; y < iy1; ++y)
                                            for (int z = iz0; z < iz1; ++z) {
                                                int lin = x*GRID_R*GRID_R
                                                        + y*GRID_R + z;
                                                if (grid[lin>>6] &
                                                    (uint64_t(1)<<(lin&63)))
                                                    warmup_hits++;
                                            }
                                }
                            } else {
                                // Grid AND (512-word)
                                const uint64_t* grid = gs.get_grid(0);
                                for (int w = 0; w < WORDS_PER_NODE; ++w)
                                    if (grid[w] & obs_grid[w])
                                        warmup_hits++;
                            }

                            // ── Timed — best of repeats ──────────────────
                            double best_us = 1e18;
                            int hits = 0;

                            for (int rep = 0; rep < cfg.n_repeats; ++rep) {
                                int h = 0;
                                auto t0 = Clock::now();

                                if (mi == 0) {
                                    // AABB full-scan: every box × every obs
                                    for (int b = 0; b < total_boxes; ++b) {
                                        const float* a =
                                            aabb_buf.data() + b * 6;
                                        for (int oi = 0; oi < n_obs; ++oi) {
                                            const float* o =
                                                obs.data() + oi * 6;
                                            // AABB [lo_x,lo_y,lo_z,hi_x,hi_y,hi_z]
                                            // obs  [lo_x,hi_x,lo_y,hi_y,lo_z,hi_z]
                                            if (a[3] >= o[0] && a[0] <= o[1] &&
                                                a[4] >= o[2] && a[1] <= o[3] &&
                                                a[5] >= o[4] && a[2] <= o[5])
                                                h++;
                                        }
                                    }
                                } else if (mi == 1) {
                                    // Grid per-obs scan (no early exit)
                                    const uint64_t* grid = gs.get_grid(0);
                                    for (int oi = 0; oi < n_obs; ++oi) {
                                        const float* ob =
                                            obs.data() + oi * 6;
                                        int ix0 = std::max(0, int(std::floor(
                                            (ob[0]-WORLD_BOUNDS[0])*inv_cell[0])));
                                        int iy0 = std::max(0, int(std::floor(
                                            (ob[2]-WORLD_BOUNDS[1])*inv_cell[1])));
                                        int iz0 = std::max(0, int(std::floor(
                                            (ob[4]-WORLD_BOUNDS[2])*inv_cell[2])));
                                        int ix1 = std::min(GRID_R, int(std::ceil(
                                            (ob[1]-WORLD_BOUNDS[0])*inv_cell[0])));
                                        int iy1 = std::min(GRID_R, int(std::ceil(
                                            (ob[3]-WORLD_BOUNDS[1])*inv_cell[1])));
                                        int iz1 = std::min(GRID_R, int(std::ceil(
                                            (ob[5]-WORLD_BOUNDS[2])*inv_cell[2])));
                                        for (int x = ix0; x < ix1; ++x)
                                            for (int y = iy0; y < iy1; ++y)
                                                for (int z = iz0; z < iz1; ++z) {
                                                    int lin = x*GRID_R*GRID_R
                                                            + y*GRID_R + z;
                                                    if (grid[lin>>6] &
                                                        (uint64_t(1)<<(lin&63)))
                                                        h++;
                                                }
                                    }
                                } else {
                                    // Grid AND: 512-word bitwise AND
                                    const uint64_t* grid = gs.get_grid(0);
                                    for (int w = 0; w < WORDS_PER_NODE; ++w) {
                                        uint64_t overlap = grid[w] & obs_grid[w];
                                        // popcount = number of overlapping voxels
                                        h += popcnt64(overlap);
                                    }
                                }

                                auto t1 = Clock::now();
                                double us = std::chrono::duration<double,
                                            std::micro>(t1 - t0).count();
                                if (us < best_us) {
                                    best_us = us;
                                    hits = h;
                                }
                            }

                            times.push_back(best_us);
                            hit_counts.push_back(hits);

                            csv << wr.name << "," << n_sub << ","
                                << n_obs << "," << method_names[mi] << ","
                                << trial << ","
                                << std::fixed << std::setprecision(3)
                                << best_us << ","
                                << hits << "\n";
                        }

                        auto st = compute_stats(times);
                        double hits_mean = 0;
                        for (double v : hit_counts) hits_mean += v;
                        hits_mean /= hit_counts.size();

                        std::cout << std::left << std::setw(8) << wr.name
                                  << std::setw(6) << n_sub
                                  << std::setw(7) << n_obs
                                  << std::setw(12) << method_names[mi]
                                  << std::right << std::fixed
                                  << std::setprecision(2)
                                  << std::setw(12) << st.mean
                                  << std::setw(12) << st.median
                                  << std::setw(12) << st.stddev
                                  << std::setprecision(1)
                                  << std::setw(12) << hits_mean
                                  << "\n";
                    }
                    std::cout << "\n";
                }
            }
        }
    }

    // ═════════════════════════════════════════════════════════════════════
    //  Summary
    // ═════════════════════════════════════════════════════════════════════

    std::cout << "────────────────────────────────────────────────────────────\n"
              << " Results written to: " << out_dir << "/\n"
              << "   part_a_aabb_derive.csv\n"
              << "   part_b_grid_derive.csv\n"
              << "   part_c_collision.csv\n"
              << "────────────────────────────────────────────────────────────\n"
              << "Done.\n";

    return 0;
}

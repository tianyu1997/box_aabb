// ═══════════════════════════════════════════════════════════════════════════
// Experiment 25 — AABB / Grid 全流程耗时对比
// ═══════════════════════════════════════════════════════════════════════════
//
// 三部分度量:
//   Part A — AABB 推导耗时 (IFK / CRIT 帧源 × n_sub∈{1,2,4,8})
//   Part B — Grid 推导耗时 (CRIT 帧源, n_sub∈{1,2,4,8})
//   Part C — 碰撞检测耗时: AABB vs Grid, 障碍物数量∈{1,5,10,50,100}
//
// Build:
//   cmake --build . --target exp_25_aabb_grid_timing --config Release
//
// Run:
//   ./exp_25_aabb_grid_timing [--trials 100] [--repeats 5]
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
              << " Experiment 25: AABB / Grid 全流程耗时对比\n"
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
    dir_oss << "../results/exp25_aabb_grid_timing_"
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
    //  Part C: 碰撞检测耗时 — AABB vs Grid × 障碍物数量
    // ═════════════════════════════════════════════════════════════════════

    std::cout << "────────────────────────────────────────────────────────────\n"
              << " Part C: 碰撞检测耗时 (AABB vs Grid × 障碍物数量)\n"
              << "────────────────────────────────────────────────────────────\n\n";

    {
        std::ofstream csv(out_dir + "/part_c_collision.csv");
        csv << "width,n_sub,n_obs,method,trial,check_us,collided\n";

        const std::vector<int> N_OBS_LIST = {1, 5, 10, 50, 100};
        constexpr int COLL_N_SUB = 4;  // fixed subdivision for collision test

        std::cout << std::left << std::setw(8) << "Width"
                  << std::setw(6) << "nSub"
                  << std::setw(7) << "nObs"
                  << std::setw(12) << "Method"
                  << std::right
                  << std::setw(12) << "Mean(us)"
                  << std::setw(12) << "Median(us)"
                  << std::setw(12) << "Std(us)"
                  << std::setw(10) << "CollRate"
                  << "\n"
                  << std::string(91, '-') << "\n";

        std::vector<float> crit_frames(n_frames * 6);

        // Scratch buffers for collision
        std::vector<float> buf_aabb(n_act * 6);
        std::vector<float> buf_sub(MAX_SUB * 6);
        std::vector<uint8_t> buf_grid(GRID_R * GRID_R * GRID_R);

        // GridStore for pre-derived grid collision
        GridStore gs(n_frames, n_act,
                     robot.active_link_map(), radii_f.data(),
                     base_pos, WORLD_BOUNDS, /*initial_cap=*/4);

        // AABB buffer for pre-derived AABB collision
        std::vector<float> aabb_buf(n_act * MAX_SUB * 6);

        for (auto& wr : WIDTHS) {
            for (int n_sub : {1, 4, 8}) {  // representative subdivisions
                for (int n_obs : N_OBS_LIST) {
                    // 4 methods:
                    // (1) collide_aabb         — on-the-fly AABB derive + SAT
                    // (2) collide_aabb_subdiv  — on-the-fly AABB + subdivide
                    // (3) collide_grid         — on-the-fly grid derive + check
                    // (4) GridStore::check_collision — pre-derived grid check

                    struct Method {
                        const char* name;
                        int id;
                    };
                    Method methods[] = {
                        {"AABB",         1},
                        {"AABB_SUB",     2},
                        {"Grid_OTF",     3},
                        {"Grid_Pre",     4},
                    };

                    for (auto& meth : methods) {
                        std::vector<double> times;
                        int n_collided = 0;
                        times.reserve(cfg.n_trials);

                        for (int trial = 0; trial < cfg.n_trials; ++trial) {
                            auto ivls = random_intervals(robot, wr, rng);
                            derive_crit_frames(robot, ivls,
                                               crit_frames.data(), nullptr);

                            auto obs = random_obstacles(n_obs, rng);

                            // Pre-derive for Grid_Pre method
                            if (meth.id == 4) {
                                gs.derive_from_frames(0,
                                    crit_frames.data(), n_sub);
                            }

                            // Pre-derive AABB for legacy check (used as warmup)
                            // (Not timed; we time the full pipeline for methods 1-3)

                            // Warmup run
                            bool warmup_result = false;
                            switch (meth.id) {
                            case 1:
                                warmup_result = collide_aabb(
                                    crit_frames.data(), n_frames,
                                    robot.active_link_map(), n_act,
                                    radii_f.data(), base_pos,
                                    obs.data(), n_obs,
                                    buf_aabb.data());
                                break;
                            case 2:
                                warmup_result = collide_aabb_subdiv(
                                    crit_frames.data(), n_frames,
                                    robot.active_link_map(), n_act,
                                    radii_f.data(), base_pos,
                                    obs.data(), n_obs,
                                    n_sub, MAX_SUB,
                                    buf_aabb.data(), buf_sub.data());
                                break;
                            case 3:
                                warmup_result = collide_grid(
                                    crit_frames.data(), n_frames,
                                    robot.active_link_map(), n_act,
                                    radii_f.data(), base_pos,
                                    obs.data(), n_obs,
                                    GRID_R, WORLD_BOUNDS,
                                    n_sub, MAX_SUB,
                                    buf_grid.data());
                                break;
                            case 4:
                                warmup_result = gs.check_collision(
                                    0, obs.data(), n_obs);
                                break;
                            }
                            (void)warmup_result;

                            // Timed — best of repeats
                            double best_us = 1e18;
                            bool result = false;
                            for (int rep = 0; rep < cfg.n_repeats; ++rep) {
                                auto t0 = Clock::now();
                                switch (meth.id) {
                                case 1:
                                    result = collide_aabb(
                                        crit_frames.data(), n_frames,
                                        robot.active_link_map(), n_act,
                                        radii_f.data(), base_pos,
                                        obs.data(), n_obs,
                                        buf_aabb.data());
                                    break;
                                case 2:
                                    result = collide_aabb_subdiv(
                                        crit_frames.data(), n_frames,
                                        robot.active_link_map(), n_act,
                                        radii_f.data(), base_pos,
                                        obs.data(), n_obs,
                                        n_sub, MAX_SUB,
                                        buf_aabb.data(),
                                        buf_sub.data());
                                    break;
                                case 3:
                                    result = collide_grid(
                                        crit_frames.data(), n_frames,
                                        robot.active_link_map(), n_act,
                                        radii_f.data(), base_pos,
                                        obs.data(), n_obs,
                                        GRID_R, WORLD_BOUNDS,
                                        n_sub, MAX_SUB,
                                        buf_grid.data());
                                    break;
                                case 4:
                                    result = gs.check_collision(
                                        0, obs.data(), n_obs);
                                    break;
                                }
                                auto t1 = Clock::now();
                                double us = std::chrono::duration<double,
                                            std::micro>(t1 - t0).count();
                                best_us = std::min(best_us, us);
                            }

                            if (result) n_collided++;
                            times.push_back(best_us);

                            csv << wr.name << "," << n_sub << ","
                                << n_obs << "," << meth.name << ","
                                << trial << ","
                                << std::fixed << std::setprecision(3)
                                << best_us << ","
                                << (result ? 1 : 0) << "\n";
                        }

                        auto st = compute_stats(times);
                        double coll_rate = 100.0 * n_collided / cfg.n_trials;

                        std::cout << std::left << std::setw(8) << wr.name
                                  << std::setw(6) << n_sub
                                  << std::setw(7) << n_obs
                                  << std::setw(12) << meth.name
                                  << std::right << std::fixed
                                  << std::setprecision(2)
                                  << std::setw(12) << st.mean
                                  << std::setw(12) << st.median
                                  << std::setw(12) << st.stddev
                                  << std::setprecision(1)
                                  << std::setw(9) << coll_rate << "%"
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

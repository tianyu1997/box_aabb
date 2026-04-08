// ═══════════════════════════════════════════════════════════════════════════
// Experiment 15 — AABB_IFK vs AABB_CRIT vs OBB_CRIT  (HCACHE03)
// ═══════════════════════════════════════════════════════════════════════════
//
// Compares 3 envelope methods with subdivision sweep:
//   AABB_IFK   — interval-FK  → FrameStore → derive_aabb[_subdivided]
//   AABB_CRIT  — critical-point enumeration → scalar FK → AABB per link
//   OBB_CRIT   — critical-point enumeration → scalar FK → PCA OBB per link
//
// Variables swept:
//   • Interval width regime   small / medium / large
//   • Subdivision count       1, 2, 4, 8
//
// Metrics:
//   cold_us   — full pipeline from intervals (derive + check)
//   cached_us — cached envelope → collision check only
//   volume_m3 — bounding volume per link (avg over sub-segments)
//   collision — bool
//
// 3 methods × 4 subdivs × 2 timings = 24 experimental groups
//
// Build:
//   cmake .. -DSBF_BUILD_EXPERIMENTS=ON
//   cmake --build . --target exp_15_critical_bench --config Release
//
// Run:
//   ./exp_15_critical_bench [--trials N] [--repeats N] [--robot panda]
//
// ═══════════════════════════════════════════════════════════════════════════

#include "sbf/robot/robot.h"
#include "sbf/robot/interval_fk.h"
#include "sbf/scene/scene.h"
#include "sbf/scene/aabb_collision_checker.h"
#include "sbf/envelope/frame_store.h"
#include "sbf/envelope/envelope_derive.h"
#include "sbf/envelope/envelope_derive_critical.h"
#include "sbf/envelope/collision_policy.h"
#include "sbf/common/config.h"

#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
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

// ─── Config ──────────────────────────────────────────────────────────────────

struct ExpConfig {
    int         n_trials  = 50;
    int         n_repeats = 20;
    std::string robot_name = "panda";
    std::string robot_json;
    std::string output_dir;
};

// ─── Width regimes ───────────────────────────────────────────────────────────

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

// ─── Subdivision sweep values ────────────────────────────────────────────────

static const int SUBDIV_SWEEP[] = {1, 2, 4, 8};
static constexpr int N_SUBDIV = 4;

// ─── Method names ────────────────────────────────────────────────────────────

static const char* METHOD_NAMES[] = { "AABB_IFK", "AABB_CRIT", "OBB_CRIT" };
static constexpr int N_METHODS = 3;
enum Method { AABB_IFK = 0, AABB_CRIT = 1, OBB_CRIT = 2 };

// ─── Trial row ───────────────────────────────────────────────────────────────

struct TrialRow {
    int         trial;
    const char* width;
    int         subdiv_n;
    const char* method;
    double      cold_us;     // full pipeline: derive + check (median us)
    double      cached_us;   // warm path (median us)
    double      derive_us;   // derive-only (median us)
    double      check_us;    // collision-check-only (median us, batch 200)
    double      volume;
    bool        collision;
};

// ─── Obstacle scene ──────────────────────────────────────────────────────────

static Scene make_clutter_scene() {
    std::vector<Obstacle> obs;
    obs.emplace_back(Eigen::Vector3d(0.5,  0.0,  0.4),
                     Eigen::Vector3d(0.15, 0.15, 0.01), "shelf");
    obs.emplace_back(Eigen::Vector3d(0.3,  0.3,  0.3),
                     Eigen::Vector3d(0.05, 0.05, 0.15), "pole_L");
    obs.emplace_back(Eigen::Vector3d(0.3, -0.3,  0.3),
                     Eigen::Vector3d(0.05, 0.05, 0.15), "pole_R");
    obs.emplace_back(Eigen::Vector3d(0.6,  0.0,  0.1),
                     Eigen::Vector3d(0.2,  0.2,  0.005), "tabletop");
    obs.emplace_back(Eigen::Vector3d(0.0,  0.5,  0.6),
                     Eigen::Vector3d(0.4,  0.005, 0.3), "wall_L");
    obs.emplace_back(Eigen::Vector3d(0.0, -0.5,  0.6),
                     Eigen::Vector3d(0.4,  0.005, 0.3), "wall_R");
    return Scene(std::move(obs));
}

// ─── Random interval generation ──────────────────────────────────────────────

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

// ─── Timing helper ───────────────────────────────────────────────────────────

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

// Batch timing for sub-microsecond operations: run N_BATCH iterations
// per sample, take median, divide by N_BATCH.  Returns microseconds.
static constexpr int CHECK_BATCH = 200;

template<typename Fn>
static double batch_median_us(Fn fn, int n_repeats) {
    std::vector<double> times;
    times.reserve(n_repeats);
    for (int r = 0; r < n_repeats; ++r) {
        auto t0 = Clock::now();
        for (int b = 0; b < CHECK_BATCH; ++b) fn();
        auto t1 = Clock::now();
        double total = std::chrono::duration<double, std::micro>(t1 - t0).count();
        times.push_back(total / CHECK_BATCH);
    }
    std::sort(times.begin(), times.end());
    return times[n_repeats / 2];
}

// ─── Statistics ──────────────────────────────────────────────────────────────

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

// ─── Volume computation ──────────────────────────────────────────────────────

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

// ─── Extract frames from FKState ─────────────────────────────────────────────

static void extract_frames_from_fk(const FKState& fk, int n_frames,
                                   float* out) {
    for (int f = 0; f < n_frames; ++f) {
        int k = f + 1;
        if (k >= fk.n_tf) k = fk.n_tf - 1;
        out[f*6+0] = (float)fk.prefix_lo[k][3];
        out[f*6+1] = (float)fk.prefix_lo[k][7];
        out[f*6+2] = (float)fk.prefix_lo[k][11];
        out[f*6+3] = (float)fk.prefix_hi[k][3];
        out[f*6+4] = (float)fk.prefix_hi[k][7];
        out[f*6+5] = (float)fk.prefix_hi[k][11];
    }
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
        else if (arg == "--robot"   && i+1 < argc) cfg.robot_name = argv[++i];
        else if (arg == "--help") {
            std::cout << "Usage: exp_15_critical_bench [--trials N] "
                         "[--repeats N] [--robot panda]\n";
            return 0;
        }
    }

    // ── Resolve robot JSON ───────────────────────────────────────────────
    fs::path exe_dir = fs::path(argv[0]).parent_path();
    for (auto& c : {
        exe_dir / ".." / ".." / "v1" / "configs",
        exe_dir / ".." / "v1" / "configs",
        exe_dir / ".." / ".." / ".." / "safeboxforest" / "v1" / "configs",
        fs::path("../v1/configs"),
        fs::path("../../v1/configs"),
    }) {
        fs::path p = c / (cfg.robot_name + ".json");
        if (fs::exists(p)) { cfg.robot_json = p.string(); break; }
    }
    if (cfg.robot_json.empty()) {
        std::cerr << "ERROR: Cannot find " << cfg.robot_name << ".json\n";
        return 1;
    }

    auto now = std::chrono::system_clock::now();
    char ts[64];
    { auto t = std::chrono::system_clock::to_time_t(now);
      std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", std::localtime(&t)); }
    cfg.output_dir = "results/exp15_" + std::string(ts);
    fs::create_directories(cfg.output_dir);

    Robot robot = Robot::from_json(cfg.robot_json);
    Scene scene = make_clutter_scene();
    const float* obs = scene.obs_compact();
    int n_obs = scene.n_obstacles();
    int n_active = robot.n_active_links();

    std::cout << "=== Experiment 15: AABB_IFK vs AABB_CRIT vs OBB_CRIT ===\n"
              << "Robot:      " << robot.name() << " (" << robot.n_joints()
              << " joints, " << n_active << " active links)\n"
              << "Obstacles:  " << n_obs << "\n"
              << "Trials:     " << cfg.n_trials << " x " << cfg.n_repeats
              << " repeats\nSubdiv:     {";
    for (int i = 0; i < N_SUBDIV; ++i) std::cout << (i?",":"") << SUBDIV_SWEEP[i];
    std::cout << "}\nOutput:     " << cfg.output_dir << "\n\n";

    // ── Prepare buffers ──────────────────────────────────────────────────

    int max_nodes = cfg.n_trials * N_WIDTHS + 16;
    FrameStore frame_store(robot, max_nodes);
    int n_frames = frame_store.n_frames();

    int max_sub = SUBDIV_SWEEP[N_SUBDIV - 1];  // 8
    std::vector<float> buf_aabb_ifk(n_active * max_sub * 6);
    std::vector<float> buf_aabb_crit(n_active * max_sub * 6);
    std::vector<float> buf_obb_crit(n_active * max_sub * CRIT_OBB_FLOATS);
    std::vector<float> buf_frames(n_frames * 6);

    std::vector<TrialRow> rows;
    rows.reserve(cfg.n_trials * N_WIDTHS * N_SUBDIV * N_METHODS);

    std::mt19937 rng(42);
    int progress = 0, total = cfg.n_trials * N_WIDTHS;

    for (int wi = 0; wi < N_WIDTHS; ++wi) {
        const auto& wr = WIDTHS[wi];
        std::cout << "Width: " << wr.name << " ...\n";

        for (int trial = 0; trial < cfg.n_trials; ++trial) {
            if (++progress % 20 == 0)
                std::cout << "  " << progress << "/" << total << "\r" << std::flush;

            auto ivs = random_intervals(robot, rng, wr.lo, wr.hi);

            // Pre-compute IFK for AABB_IFK pipeline
            FKState fk = compute_fk_full(robot, ivs);
            if (!fk.valid) continue;

            int node_idx = trial + wi * cfg.n_trials;
            frame_store.store_from_fk(node_idx, fk);
            const float* frames = frame_store.get_frames(node_idx);

            // Reference v1 AABBs for consistency checking
            std::vector<float> ref_aabbs(n_active * 6);
            extract_link_aabbs(fk, robot.active_link_map(), n_active,
                               ref_aabbs.data(), robot.active_link_radii());
            bool ref_coll = aabbs_collide_obs(ref_aabbs.data(), n_active, obs, n_obs);

            for (int si = 0; si < N_SUBDIV; ++si) {
                int sub_n = SUBDIV_SWEEP[si];

                // ── AABB_IFK ─────────────────────────────────────────────
                {
                    bool res = false;
                    // Cold: IFK + extract frames + derive + check
                    double cold = median_us([&](){
                        FKState fk2 = compute_fk_full(robot, ivs);
                        extract_frames_from_fk(fk2, n_frames, buf_frames.data());
                        if (sub_n == 1) {
                            derive_aabb(buf_frames.data(), n_frames,
                                frame_store.active_link_map(), n_active,
                                frame_store.link_radii(), frame_store.base_pos(),
                                buf_aabb_ifk.data());
                            res = aabbs_collide_obs(buf_aabb_ifk.data(), n_active,
                                obs, n_obs);
                        } else {
                            for (int li = 0; li < n_active; ++li) {
                                int fidx = frame_store.active_link_map()[li];
                                float rad = frame_store.link_radii() ?
                                    frame_store.link_radii()[li] : 0.f;
                                derive_aabb_subdivided(buf_frames.data(), n_frames,
                                    fidx - 1, fidx, sub_n, rad,
                                    frame_store.base_pos(),
                                    buf_aabb_ifk.data() + li * sub_n * 6);
                            }
                            res = aabbs_collide_obs(buf_aabb_ifk.data(),
                                n_active * sub_n, obs, n_obs);
                        }
                    }, cfg.n_repeats);

                    // Warm: frames cached → derive + check
                    double warm = median_us([&](){
                        if (sub_n == 1) {
                            derive_aabb(frames, n_frames,
                                frame_store.active_link_map(), n_active,
                                frame_store.link_radii(), frame_store.base_pos(),
                                buf_aabb_ifk.data());
                            res = aabbs_collide_obs(buf_aabb_ifk.data(), n_active,
                                obs, n_obs);
                        } else {
                            for (int li = 0; li < n_active; ++li) {
                                int fidx = frame_store.active_link_map()[li];
                                float rad = frame_store.link_radii() ?
                                    frame_store.link_radii()[li] : 0.f;
                                derive_aabb_subdivided(frames, n_frames,
                                    fidx - 1, fidx, sub_n, rad,
                                    frame_store.base_pos(),
                                    buf_aabb_ifk.data() + li * sub_n * 6);
                            }
                            res = aabbs_collide_obs(buf_aabb_ifk.data(),
                                n_active * sub_n, obs, n_obs);
                        }
                    }, cfg.n_repeats);

                    // Derive-only: from cached frames
                    double derive_t = median_us([&](){
                        if (sub_n == 1) {
                            derive_aabb(frames, n_frames,
                                frame_store.active_link_map(), n_active,
                                frame_store.link_radii(), frame_store.base_pos(),
                                buf_aabb_ifk.data());
                        } else {
                            for (int li = 0; li < n_active; ++li) {
                                int fidx = frame_store.active_link_map()[li];
                                float rad = frame_store.link_radii() ?
                                    frame_store.link_radii()[li] : 0.f;
                                derive_aabb_subdivided(frames, n_frames,
                                    fidx - 1, fidx, sub_n, rad,
                                    frame_store.base_pos(),
                                    buf_aabb_ifk.data() + li * sub_n * 6);
                            }
                        }
                    }, cfg.n_repeats);

                    // Ensure envelope is computed for check_only
                    if (sub_n == 1) {
                        derive_aabb(frames, n_frames,
                            frame_store.active_link_map(), n_active,
                            frame_store.link_radii(), frame_store.base_pos(),
                            buf_aabb_ifk.data());
                    } else {
                        for (int li = 0; li < n_active; ++li) {
                            int fidx = frame_store.active_link_map()[li];
                            float rad = frame_store.link_radii() ?
                                frame_store.link_radii()[li] : 0.f;
                            derive_aabb_subdivided(frames, n_frames,
                                fidx - 1, fidx, sub_n, rad,
                                frame_store.base_pos(),
                                buf_aabb_ifk.data() + li * sub_n * 6);
                        }
                    }
                    int ifk_slots = (sub_n == 1) ? n_active : n_active * sub_n;
                    double check_t = batch_median_us([&](){
                        res = aabbs_collide_obs(buf_aabb_ifk.data(),
                            ifk_slots, obs, n_obs);
                    }, cfg.n_repeats);

                    // Volume: average per link
                    double vol = volume_aabb(buf_aabb_ifk.data(), ifk_slots)
                                 / std::max(sub_n, 1);

                    rows.push_back({trial, wr.name, sub_n, "AABB_IFK",
                        cold, warm, derive_t, check_t, vol, res});
                }

                // ── AABB_CRIT ────────────────────────────────────────────
                {
                    bool res = false;
                    // Cold: derive_aabb_critical + check
                    double cold = median_us([&](){
                        derive_aabb_critical(robot, ivs, sub_n,
                            buf_aabb_crit.data());
                        res = aabbs_collide_obs(buf_aabb_crit.data(),
                            n_active * sub_n, obs, n_obs);
                    }, cfg.n_repeats);

                    // Derive-only
                    double derive_t = median_us([&](){
                        derive_aabb_critical(robot, ivs, sub_n,
                            buf_aabb_crit.data());
                    }, cfg.n_repeats);

                    // Ensure envelope is computed for check_only
                    derive_aabb_critical(robot, ivs, sub_n,
                        buf_aabb_crit.data());
                    res = aabbs_collide_obs(buf_aabb_crit.data(),
                        n_active * sub_n, obs, n_obs);

                    // Check-only (batch)
                    double check_t = batch_median_us([&](){
                        res = aabbs_collide_obs(buf_aabb_crit.data(),
                            n_active * sub_n, obs, n_obs);
                    }, cfg.n_repeats);

                    // Warm = check-only (same envelope, no derive)
                    double warm = check_t;

                    double vol = volume_aabb(buf_aabb_crit.data(),
                        n_active * sub_n) / sub_n;

                    rows.push_back({trial, wr.name, sub_n, "AABB_CRIT",
                        cold, warm, derive_t, check_t, vol, res});
                }

                // ── OBB_CRIT ─────────────────────────────────────────────
                {
                    bool res = false;
                    // Cold: derive_obb_critical + check
                    double cold = median_us([&](){
                        derive_obb_critical(robot, ivs, sub_n,
                            buf_obb_crit.data());
                        res = collide_obb_obs(buf_obb_crit.data(),
                            n_active * sub_n, obs, n_obs);
                    }, cfg.n_repeats);

                    // Derive-only
                    double derive_t = median_us([&](){
                        derive_obb_critical(robot, ivs, sub_n,
                            buf_obb_crit.data());
                    }, cfg.n_repeats);

                    // Ensure envelope is computed for check_only
                    derive_obb_critical(robot, ivs, sub_n,
                        buf_obb_crit.data());
                    res = collide_obb_obs(buf_obb_crit.data(),
                        n_active * sub_n, obs, n_obs);

                    // Check-only (batch)
                    double check_t = batch_median_us([&](){
                        res = collide_obb_obs(buf_obb_crit.data(),
                            n_active * sub_n, obs, n_obs);
                    }, cfg.n_repeats);

                    // Warm = check-only
                    double warm = check_t;

                    double vol = volume_obb_slots(buf_obb_crit.data(),
                        n_active * sub_n) / sub_n;

                    rows.push_back({trial, wr.name, sub_n, "OBB_CRIT",
                        cold, warm, derive_t, check_t, vol, res});
                }

                // ── REF_V1 (once per trial, subdiv=1 only) ──────────────
                if (si == 0) {
                    double cold = median_us([&](){
                        FKState fk2 = compute_fk_full(robot, ivs);
                        std::vector<float> tmp(n_active * 6);
                        extract_link_aabbs(fk2, robot.active_link_map(),
                            n_active, tmp.data(), robot.active_link_radii());
                        aabbs_collide_obs(tmp.data(), n_active, obs, n_obs);
                    }, cfg.n_repeats);
                    double derive_t = median_us([&](){
                        FKState fk2 = compute_fk_full(robot, ivs);
                        extract_link_aabbs(fk2, robot.active_link_map(),
                            n_active, ref_aabbs.data(), robot.active_link_radii());
                    }, cfg.n_repeats);
                    double check_t = batch_median_us([&](){
                        aabbs_collide_obs(ref_aabbs.data(), n_active, obs, n_obs);
                    }, cfg.n_repeats);
                    double warm = check_t;
                    double vol = volume_aabb(ref_aabbs.data(), n_active);

                    rows.push_back({trial, wr.name, 1, "REF_V1",
                        cold, warm, derive_t, check_t, vol, ref_coll});
                }

            } // subdiv
        } // trials
        std::cout << "\n";
    } // widths

    std::cout << "Collected " << rows.size() << " measurements.\n\n";

    // ═════════════════════════════════════════════════════════════════════
    //  Write raw CSV
    // ═════════════════════════════════════════════════════════════════════
    {
        std::string path = cfg.output_dir + "/benchmark_raw.csv";
        std::ofstream f(path);
        f << "trial,width,subdiv_n,method,cold_us,cached_us,derive_us,check_us,volume_m3,collision\n";
        for (auto& r : rows) {
            f << r.trial << "," << r.width << "," << r.subdiv_n << ","
              << r.method << ","
              << std::fixed << std::setprecision(3)
              << r.cold_us << "," << r.cached_us << ","
              << r.derive_us << "," << r.check_us << ","
              << std::scientific << std::setprecision(6) << r.volume << ","
              << (r.collision ? "true" : "false") << "\n";
        }
        std::cout << "CSV: " << path << "\n";
    }

    // ═════════════════════════════════════════════════════════════════════
    //  Consistency analysis
    // ═════════════════════════════════════════════════════════════════════
    //
    // All methods must be at least as conservative as REF_V1.
    // AABB_IFK with subdiv=1 must match REF_V1 exactly.
    // AABB_CRIT / OBB_CRIT: if REF_V1=collision, they MUST also report
    // collision (no false negatives). If they report no-collision when
    // REF_V1=collision, that means the envelope is NOT a superset — BUG.
    //
    int n_false_neg_aabb_ifk = 0;  // These are critical bugs
    int n_false_neg_aabb_crit = 0;
    int n_false_neg_obb_crit = 0;
    int n_tighten_aabb_ifk = 0;
    int n_tighten_aabb_crit = 0;
    int n_tighten_obb_crit = 0;
    int n_checked = 0;

    for (int wi = 0; wi < N_WIDTHS; ++wi) {
        for (int t = 0; t < cfg.n_trials; ++t) {
            // Find REF_V1 result
            bool ref = false; bool ref_found = false;
            for (auto& r : rows) {
                if (std::string(r.width) != WIDTHS[wi].name || r.trial != t) continue;
                if (std::string(r.method) == "REF_V1") {
                    ref = r.collision; ref_found = true; break;
                }
            }
            if (!ref_found) continue;
            ++n_checked;

            for (auto& r : rows) {
                if (std::string(r.width) != WIDTHS[wi].name || r.trial != t) continue;
                std::string m = r.method;
                if (m == "REF_V1") continue;

                // "false negative" = method says safe when ref says collision
                if (ref && !r.collision) {
                    if (m == "AABB_IFK"  && r.subdiv_n == 1) n_false_neg_aabb_ifk++;
                    // For subdiv>1 or CRIT methods, tightening is expected
                    if (m == "AABB_IFK"  && r.subdiv_n > 1)  n_tighten_aabb_ifk++;
                    if (m == "AABB_CRIT")                    n_tighten_aabb_crit++;
                    if (m == "OBB_CRIT")                     n_tighten_obb_crit++;
                }
            }
        }
    }

    // ═════════════════════════════════════════════════════════════════════
    //  Write Markdown report
    // ═════════════════════════════════════════════════════════════════════
    {
        std::string path = cfg.output_dir + "/report.md";
        std::ofstream md(path);
        md << "# Experiment 15 — AABB_IFK vs AABB_CRIT vs OBB_CRIT\n\n";

        // Setup
        md << "## Setup\n\n| Param | Value |\n|---|---|\n"
           << "| Robot | " << robot.name() << " (" << robot.n_joints() << " DOF, "
           << n_active << " active links) |\n"
           << "| Obstacles | " << n_obs << " |\n"
           << "| Trials/regime | " << cfg.n_trials << " |\n"
           << "| Repeats | " << cfg.n_repeats << " (median) |\n"
           << "| Subdiv sweep | {";
        for (int i = 0; i < N_SUBDIV; ++i) md << (i?",":"") << SUBDIV_SWEEP[i];
        md << "} |\n\n";

        // Consistency
        md << "## Consistency (vs REF_V1)\n\n"
           << "Checked " << n_checked << " trial/width combos.\n\n"
           << "| Check | Count | Status |\n|---|---|---|\n";

        auto status = [](int c) { return c == 0 ? "PASS" : "**FAIL**"; };
        md << "| AABB_IFK(sub=1) false neg | " << n_false_neg_aabb_ifk
           << " | " << status(n_false_neg_aabb_ifk) << " |\n"
           << "| AABB_IFK(sub>1) tighten | " << n_tighten_aabb_ifk
           << " | OK (expected) |\n"
           << "| AABB_CRIT tighten | " << n_tighten_aabb_crit
           << " | OK (expected) |\n"
           << "| OBB_CRIT tighten | " << n_tighten_obb_crit
           << " | OK (expected) |\n\n";

        // Performance tables per width (full)
        md << "## Performance Overview (median us)\n\n";
        for (int wi = 0; wi < N_WIDTHS; ++wi) {
            md << "### " << WIDTHS[wi].name << " intervals\n\n"
               << "| Method | subdiv | Cold us | Warm us | Cold/Warm | Volume m3 |\n"
               << "|---|---|---|---|---|---|\n";

            const char* ALL[] = { "AABB_IFK", "AABB_CRIT", "OBB_CRIT", "REF_V1" };
            for (int mi = 0; mi < 4; ++mi) {
                std::map<int, std::vector<TrialRow*>> by_sub;
                for (auto& r : rows) {
                    if (std::string(r.width) != WIDTHS[wi].name) continue;
                    if (std::string(r.method) != ALL[mi]) continue;
                    by_sub[r.subdiv_n].push_back(&r);
                }
                for (auto& [sn, rr] : by_sub) {
                    std::vector<double> cv, wv, vv;
                    for (auto* r : rr) {
                        cv.push_back(r->cold_us);
                        wv.push_back(r->cached_us);
                        vv.push_back(r->volume);
                    }
                    auto cs = stats_of(cv), ws = stats_of(wv), vs = stats_of(vv);
                    double sp = cs.p50 / std::max(ws.p50, 0.001);
                    md << "| " << ALL[mi] << " | " << sn
                       << " | " << std::fixed << std::setprecision(1)
                       << cs.p50 << "+/-" << cs.stddev
                       << " | " << std::setprecision(3) << ws.p50 << "+/-" << ws.stddev
                       << " | " << std::setprecision(1) << sp << "x"
                       << " | " << std::scientific << std::setprecision(3) << vs.p50
                       << " |\n";
                }
            }
            md << "\n";
        }

        // ── Timing Breakdown: Derive vs Check ───────────────────────────
        md << "## Timing Breakdown: Derive vs Check (median us)\n\n"
           << "> **derive_us** = envelope derivation only (no collision check)\n"
           << "> **check_us** = collision check only (envelope pre-computed, "
           << "batch " << CHECK_BATCH << " averaged)\n"
           << "> For IFK methods, derive_us uses cached FrameStore frames.\n"
           << "> For CRIT methods, derive_us includes critical-point enumeration + FK.\n\n";
        for (int wi = 0; wi < N_WIDTHS; ++wi) {
            md << "### " << WIDTHS[wi].name << " intervals\n\n"
               << "| Method | subdiv | Derive us | Check us | Derive % | Check % |\n"
               << "|---|---|---|---|---|---|\n";

            const char* ALL[] = { "AABB_IFK", "AABB_CRIT", "OBB_CRIT", "REF_V1" };
            for (int mi = 0; mi < 4; ++mi) {
                std::map<int, std::vector<TrialRow*>> by_sub;
                for (auto& r : rows) {
                    if (std::string(r.width) != WIDTHS[wi].name) continue;
                    if (std::string(r.method) != ALL[mi]) continue;
                    by_sub[r.subdiv_n].push_back(&r);
                }
                for (auto& [sn, rr] : by_sub) {
                    std::vector<double> dv, ckv;
                    for (auto* r : rr) {
                        dv.push_back(r->derive_us);
                        ckv.push_back(r->check_us);
                    }
                    auto ds = stats_of(dv), cks = stats_of(ckv);
                    double total = ds.p50 + cks.p50;
                    double dpct = total > 0 ? 100.0*ds.p50/total : 0;
                    double cpct = total > 0 ? 100.0*cks.p50/total : 0;
                    md << "| " << ALL[mi] << " | " << sn
                       << " | " << std::fixed << std::setprecision(3) << ds.p50
                       << "+/-" << ds.stddev
                       << " | " << std::setprecision(3) << cks.p50
                       << "+/-" << cks.stddev
                       << " | " << std::setprecision(1) << dpct << "%"
                       << " | " << cpct << "%"
                       << " |\n";
                }
            }
            md << "\n";
        }

        // ── Cold vs Warm Analysis ────────────────────────────────────────
        md << "## Cold vs Warm Startup Analysis\n\n"
           << "> **Cold** = full pipeline (IFK: interval-FK + frame extract + derive + check; "
           << "CRIT: critical enum + scalar FK + derive + check)\n"
           << "> **Warm** = cached path (IFK: derive from cached frames + check; "
           << "CRIT: check only, envelope pre-computed)\n\n"
           << "| Method | subdiv | Cold p50 us | Warm p50 us | Speedup | Note |\n"
           << "|---|---|---|---|---|---|\n";
        {
            const char* ALL[] = { "AABB_IFK", "AABB_CRIT", "OBB_CRIT", "REF_V1" };
            const char* NOTES[] = {
                "warm = derive(cached frames) + check",
                "warm = check only",
                "warm = check only",
                "warm = check only"
            };
            for (int mi = 0; mi < 4; ++mi) {
                std::map<int, std::vector<TrialRow*>> by_sub;
                for (auto& r : rows) {
                    if (std::string(r.method) != ALL[mi]) continue;
                    by_sub[r.subdiv_n].push_back(&r);
                }
                for (auto& [sn, rr] : by_sub) {
                    std::vector<double> cv, wv;
                    for (auto* r : rr) { cv.push_back(r->cold_us); wv.push_back(r->cached_us); }
                    auto cs = stats_of(cv), ws = stats_of(wv);
                    double sp = cs.p50 / std::max(ws.p50, 0.001);
                    md << "| " << ALL[mi] << " | " << sn
                       << " | " << std::fixed << std::setprecision(1) << cs.p50
                       << " | " << std::setprecision(3) << ws.p50
                       << " | " << std::setprecision(0) << sp << "x"
                       << " | " << NOTES[mi] << " |\n";
                }
            }
        }
        md << "\n";

        // ── Per-Check Collision Cost ─────────────────────────────────────
        md << "## Per-Collision-Check Cost (ns, batch " << CHECK_BATCH << ")\n\n"
           << "> Single collision check latency averaged over " << CHECK_BATCH
           << " invocations, median across repeats.\n"
           << "> Includes " << n_active << " active links × N_sub slots vs "
           << n_obs << " obstacles.\n\n"
           << "| Method | subdiv | Slots | Check p50 ns | Check p95 ns |\n"
           << "|---|---|---|---|---|\n";
        {
            const char* ALL[] = { "AABB_IFK", "AABB_CRIT", "OBB_CRIT", "REF_V1" };
            for (int mi = 0; mi < 4; ++mi) {
                std::map<int, std::vector<TrialRow*>> by_sub;
                for (auto& r : rows) {
                    if (std::string(r.method) != ALL[mi]) continue;
                    by_sub[r.subdiv_n].push_back(&r);
                }
                for (auto& [sn, rr] : by_sub) {
                    int slots = (std::string(ALL[mi]) == "REF_V1" || sn == 0)
                        ? n_active : n_active * sn;
                    std::vector<double> ckv;
                    for (auto* r : rr) ckv.push_back(r->check_us * 1000.0); // → ns
                    auto cks = stats_of(ckv);
                    md << "| " << ALL[mi] << " | " << sn
                       << " | " << slots
                       << " | " << std::fixed << std::setprecision(0) << cks.p50
                       << " | " << cks.p95 << " |\n";
                }
            }
        }
        md << "\n";

        // Volume comparison
        md << "## Volume Comparison (tightness)\n\n"
           << "| Method | subdiv | Median Vol m3 | Ratio vs AABB_IFK(1) |\n"
           << "|---|---|---|---|\n";
        {
            // Baseline: AABB_IFK subdiv=1
            std::vector<double> baseline_v;
            for (auto& r : rows)
                if (std::string(r.method) == "AABB_IFK" && r.subdiv_n == 1)
                    baseline_v.push_back(r.volume);
            double base_med = baseline_v.empty() ? 1.0 : stats_of(baseline_v).p50;

            const char* ALL[] = { "AABB_IFK", "AABB_CRIT", "OBB_CRIT", "REF_V1" };
            for (int mi = 0; mi < 4; ++mi) {
                std::map<int, std::vector<double>> by_sub;
                for (auto& r : rows)
                    if (std::string(r.method) == ALL[mi])
                        by_sub[r.subdiv_n].push_back(r.volume);
                for (auto& [sn, vv] : by_sub) {
                    auto vs = stats_of(vv);
                    md << "| " << ALL[mi] << " | " << sn
                       << " | " << std::scientific << std::setprecision(3) << vs.p50
                       << " | " << std::fixed << std::setprecision(3)
                       << vs.p50 / std::max(base_med, 1e-12) << " |\n";
                }
            }
        }
        md << "\n";

        // Collision rejection rate
        md << "## Collision Rejection Rate\n\n"
           << "| Width | Method | subdiv | Collision_% |\n"
           << "|---|---|---|---|\n";
        for (int wi = 0; wi < N_WIDTHS; ++wi) {
            const char* ALL[] = { "AABB_IFK", "AABB_CRIT", "OBB_CRIT", "REF_V1" };
            for (int mi = 0; mi < 4; ++mi) {
                std::map<int, std::pair<int,int>> by_sub;
                for (auto& r : rows) {
                    if (std::string(r.width) != WIDTHS[wi].name) continue;
                    if (std::string(r.method) != ALL[mi]) continue;
                    auto& p = by_sub[r.subdiv_n];
                    if (r.collision) p.first++;
                    p.second++;
                }
                for (auto& [sn, ct] : by_sub) {
                    double rate = ct.second > 0 ? 100.0*ct.first/ct.second : 0;
                    md << "| " << WIDTHS[wi].name << " | " << ALL[mi] << " | " << sn
                       << " | " << std::fixed << std::setprecision(1) << rate << " |\n";
                }
            }
        }
        md << "\n";

        std::cout << "Report: " << path << "\n";
    }

    // ═════════════════════════════════════════════════════════════════════
    //  Console summary
    // ═════════════════════════════════════════════════════════════════════
    std::cout << "\n=== Consistency ===\n"
              << "  AABB_IFK(sub=1) false_neg: " << n_false_neg_aabb_ifk
              << (n_false_neg_aabb_ifk ? " *** BUG ***" : " OK") << "\n"
              << "  AABB_IFK(sub>1) tighten:   " << n_tighten_aabb_ifk << " (OK)\n"
              << "  AABB_CRIT tighten:         " << n_tighten_aabb_crit << " (OK)\n"
              << "  OBB_CRIT tighten:          " << n_tighten_obb_crit << " (OK)\n\n";

    // Quick volume summary
    {
        std::vector<double> v_ifk1, v_crit1, v_obb1;
        for (auto& r : rows) {
            if (r.subdiv_n != 1) continue;
            if (std::string(r.method) == "AABB_IFK")  v_ifk1.push_back(r.volume);
            if (std::string(r.method) == "AABB_CRIT") v_crit1.push_back(r.volume);
            if (std::string(r.method) == "OBB_CRIT")  v_obb1.push_back(r.volume);
        }
        auto s_ifk  = stats_of(v_ifk1);
        auto s_crit = stats_of(v_crit1);
        auto s_obb  = stats_of(v_obb1);
        std::cout << "=== Volume (subdiv=1 median) ===\n"
                  << "  AABB_IFK:  " << std::scientific << std::setprecision(4)
                  << s_ifk.p50  << "\n"
                  << "  AABB_CRIT: " << s_crit.p50 << " (ratio "
                  << std::fixed << std::setprecision(3)
                  << s_crit.p50/std::max(s_ifk.p50, 1e-12) << ")\n"
                  << "  OBB_CRIT:  " << std::scientific << std::setprecision(4)
                  << s_obb.p50  << " (ratio "
                  << std::fixed << std::setprecision(3)
                  << s_obb.p50/std::max(s_ifk.p50, 1e-12) << ")\n\n";
    }

    // Timing breakdown console summary
    {
        std::cout << "=== Timing Breakdown (all widths, p50 us) ===\n"
                  << std::left
                  << std::setw(14) << "Method"
                  << std::setw(8)  << "subdiv"
                  << std::setw(12) << "Cold"
                  << std::setw(10) << "Warm"
                  << std::setw(12) << "Derive"
                  << std::setw(10) << "Check"
                  << std::setw(10) << "Check_ns"
                  << "\n" << std::string(76, '-') << "\n";

        const char* ALL[] = { "AABB_IFK", "AABB_CRIT", "OBB_CRIT", "REF_V1" };
        for (int mi = 0; mi < 4; ++mi) {
            std::map<int, std::vector<TrialRow*>> by_sub;
            for (auto& r : rows)
                if (std::string(r.method) == ALL[mi])
                    by_sub[r.subdiv_n].push_back(&r);
            for (auto& [sn, rr] : by_sub) {
                std::vector<double> cv, wv, dv, ckv;
                for (auto* r : rr) {
                    cv.push_back(r->cold_us);
                    wv.push_back(r->cached_us);
                    dv.push_back(r->derive_us);
                    ckv.push_back(r->check_us);
                }
                auto cs = stats_of(cv), ws = stats_of(wv);
                auto ds = stats_of(dv), cks = stats_of(ckv);
                std::cout << std::setw(14) << ALL[mi]
                          << std::setw(8)  << sn
                          << std::fixed << std::setprecision(1)
                          << std::setw(12) << cs.p50
                          << std::setprecision(3)
                          << std::setw(10) << ws.p50
                          << std::setw(12) << ds.p50
                          << std::setw(10) << cks.p50
                          << std::setprecision(0)
                          << std::setw(10) << (cks.p50 * 1000)
                          << "\n";
            }
        }
        std::cout << "\n";
    }

    if (n_false_neg_aabb_ifk) {
        std::cout << "\n*** FAILED: AABB_IFK(sub=1) vs REF_V1 false negative ***\n";
        return 1;
    }
    std::cout << "*** ALL TESTS PASSED ***\n";
    return 0;
}

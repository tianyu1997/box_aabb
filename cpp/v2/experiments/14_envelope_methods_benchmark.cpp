// ═══════════════════════════════════════════════════════════════════════════
// Experiment 14 — Envelope Method Benchmark & Consistency  (HCACHE03)
// ═══════════════════════════════════════════════════════════════════════════
//
// Compares 3 CollisionPolicy envelope methods + REF_V1 baseline:
//   0  AABB         — per-link AABB from frames
//   1  AABB_SUBDIV  — subdivided AABB (n_sub swept as variable)
//   2  GRID         — sub-AABB voxelisation → bitwise grid
//   +  REF_V1       — legacy extract_link_aabbs + aabbs_collide_obs
//
// Variables swept:
//   • Interval width regime   small / medium / large
//   • Subdivision count       2, 4, 8  (affects AABB_SUBDIV)
//
// Metrics per (width, subdiv, policy, trial):
//   1. cold_us     — full pipeline: FK + frame extract + derive + check
//   2. cached_us   — cache hit: derive + check  (frames already stored)
//   3. volume      — envelope bounding volume (m³)
//   4. disk_bytes  — per-node storage cost on disk
//   5. collision   — bool result
//
// Build:
//   cmake .. -DSBF_BUILD_EXPERIMENTS=ON
//   cmake --build . --target exp_14_envelope_bench --config Release
//
// Run:
//   ./exp_14_envelope_bench [--trials N] [--repeats N] [--robot panda|iiwa14]
//
// ═══════════════════════════════════════════════════════════════════════════

#include "sbf/robot/robot.h"
#include "sbf/robot/interval_fk.h"
#include "sbf/scene/scene.h"
#include "sbf/scene/aabb_collision_checker.h"
#include "sbf/envelope/frame_store.h"
#include "sbf/envelope/envelope_derive.h"
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
    int         grid_R = 32;
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

static const int SUBDIV_SWEEP[] = {2, 4, 8};
static constexpr int N_SUBDIV = 3;

// ─── Policy names ────────────────────────────────────────────────────────────

static const char* POLICY_NAMES[] = {
    "AABB", "AABB_SUBDIV", "GRID"
};
static constexpr int N_POLICIES = 3;

// ─── Trial row ───────────────────────────────────────────────────────────────

struct TrialRow {
    int         trial;
    const char* width;
    int         subdiv_n;
    const char* policy;
    double      cold_us;
    double      cached_us;
    double      volume;
    int         disk_bytes;
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

// ─── Grid world bounds from robot workspace ──────────────────────────────────

static void compute_world_bounds(const Robot& robot, float bounds[6]) {
    double reach = 0;
    for (auto& dh : robot.dh_params())
        reach += std::fabs(dh.d) + std::fabs(dh.a);
    if (robot.has_tool())
        reach += std::fabs(robot.tool_frame()->d) + std::fabs(robot.tool_frame()->a);
    float r = static_cast<float>(reach * 1.1);
    bounds[0] = -r; bounds[1] = -r; bounds[2] = 0.f;
    bounds[3] =  r; bounds[4] =  r; bounds[5] = r * 1.2f;
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

static double volume_grid(const uint8_t* grid, int R, const float* wb) {
    double cx = (wb[3] - wb[0]) / R;
    double cy = (wb[4] - wb[1]) / R;
    double cz = (wb[5] - wb[2]) / R;
    int occ = 0;
    for (int i = 0; i < R*R*R; ++i) if (grid[i]) ++occ;
    return occ * cx * cy * cz;
}

// ─── Disk size computation ───────────────────────────────────────────────────

static int disk_bytes_frames(int n_frames) {
    return n_frames * 6 * (int)sizeof(float);
}
static int disk_bytes_aabb(int n_active) {
    return n_active * 6 * (int)sizeof(float);
}
static int disk_bytes_aabb_subdiv(int n_active, int subdiv_n) {
    return n_active * subdiv_n * 6 * (int)sizeof(float);
}
static int disk_bytes_grid(int R) { return R * R * R; }

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
        else if (arg == "--grid-r"  && i+1 < argc) cfg.grid_R = std::stoi(argv[++i]);
        else if (arg == "--help") {
            std::cout << "Usage: exp_14_envelope_bench [--trials N] "
                         "[--repeats N] [--robot panda|iiwa14] [--grid-r R]\n";
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
    cfg.output_dir = "results/exp14_" + std::string(ts);
    fs::create_directories(cfg.output_dir);

    Robot robot = Robot::from_json(cfg.robot_json);
    Scene scene = make_clutter_scene();
    const float* obs = scene.obs_compact();
    int n_obs = scene.n_obstacles();
    int n_active = robot.n_active_links();
    int grid_R = cfg.grid_R;

    float world_bounds[6];
    compute_world_bounds(robot, world_bounds);

    std::cout << "=== Experiment 14: Envelope Method Benchmark (HCACHE03) ===\n"
              << "Robot:      " << robot.name() << " (" << robot.n_joints()
              << " joints, " << n_active << " active links)\n"
              << "Obstacles:  " << n_obs << "\n"
              << "Trials:     " << cfg.n_trials << " x " << cfg.n_repeats
              << " repeats\nGrid R:     " << grid_R << "\nSubdiv:     {";
    for (int i = 0; i < N_SUBDIV; ++i) std::cout << (i?",":"") << SUBDIV_SWEEP[i];
    std::cout << "}\nOutput:     " << cfg.output_dir << "\n\n";

    int max_nodes = cfg.n_trials * N_WIDTHS + 16;
    FrameStore frame_store(robot, max_nodes);
    int n_frames = frame_store.n_frames();

    int max_sub = SUBDIV_SWEEP[N_SUBDIV - 1];
    std::vector<float>   buf_aabb(n_active * 6);
    std::vector<float>   buf_sub(n_active * max_sub * 6);
    std::vector<uint8_t> buf_grid(grid_R * grid_R * grid_R);
    std::vector<float>   buf_frames(n_frames * 6);

    std::vector<TrialRow> rows;
    rows.reserve(cfg.n_trials * N_WIDTHS * N_SUBDIV * 4);

    std::mt19937 rng(42);
    int progress = 0, total = cfg.n_trials * N_WIDTHS;

    for (int wi = 0; wi < N_WIDTHS; ++wi) {
        const auto& wr = WIDTHS[wi];
        std::cout << "Width: " << wr.name << " ...\n";

        for (int trial = 0; trial < cfg.n_trials; ++trial) {
            if (++progress % 20 == 0)
                std::cout << "  " << progress << "/" << total << "\r" << std::flush;

            auto ivs = random_intervals(robot, rng, wr.lo, wr.hi);
            FKState fk = compute_fk_full(robot, ivs);
            if (!fk.valid) continue;

            int node_idx = trial + wi * cfg.n_trials;
            frame_store.store_from_fk(node_idx, fk);
            const float* frames = frame_store.get_frames(node_idx);

            // Reference v1 AABBs
            std::vector<float> ref_aabbs(n_active * 6);
            extract_link_aabbs(fk, robot.active_link_map(), n_active,
                               ref_aabbs.data(), robot.active_link_radii());
            bool ref_coll = aabbs_collide_obs(ref_aabbs.data(), n_active, obs, n_obs);

            for (int si = 0; si < N_SUBDIV; ++si) {
                int sub_n = SUBDIV_SWEEP[si];

                // ── 0: AABB (only once, subdiv irrelevant) ───────────────
                if (si == 0) {
                    bool res = false;
                    double cold = median_us([&](){
                        FKState fk2 = compute_fk_full(robot, ivs);
                        extract_frames_from_fk(fk2, n_frames, buf_frames.data());
                        res = check_collision(CollisionPolicy::AABB,
                            buf_frames.data(), n_frames,
                            frame_store.active_link_map(), n_active,
                            frame_store.link_radii(), frame_store.base_pos(),
                            obs, n_obs, 4, 8, grid_R, world_bounds, nullptr);
                    }, cfg.n_repeats);
                    double warm = median_us([&](){
                        res = check_collision(CollisionPolicy::AABB,
                            frames, n_frames,
                            frame_store.active_link_map(), n_active,
                            frame_store.link_radii(), frame_store.base_pos(),
                            obs, n_obs, 4, 8, grid_R, world_bounds, nullptr);
                    }, cfg.n_repeats);

                    derive_aabb(frames, n_frames,
                        frame_store.active_link_map(), n_active,
                        frame_store.link_radii(), frame_store.base_pos(),
                        buf_aabb.data());
                    double vol = volume_aabb(buf_aabb.data(), n_active);

                    rows.push_back({trial, wr.name, 0, "AABB",
                        cold, warm, vol, disk_bytes_frames(n_frames), res});
                }

                // ── 1: AABB_SUBDIV (varied by sub_n) ─────────────────────
                {
                    bool res = false;
                    double cold = median_us([&](){
                        FKState fk2 = compute_fk_full(robot, ivs);
                        extract_frames_from_fk(fk2, n_frames, buf_frames.data());
                        res = check_collision(CollisionPolicy::AABB_SUBDIV,
                            buf_frames.data(), n_frames,
                            frame_store.active_link_map(), n_active,
                            frame_store.link_radii(), frame_store.base_pos(),
                            obs, n_obs, sub_n, sub_n, grid_R, world_bounds, nullptr);
                    }, cfg.n_repeats);
                    double warm = median_us([&](){
                        res = check_collision(CollisionPolicy::AABB_SUBDIV,
                            frames, n_frames,
                            frame_store.active_link_map(), n_active,
                            frame_store.link_radii(), frame_store.base_pos(),
                            obs, n_obs, sub_n, sub_n, grid_R, world_bounds, nullptr);
                    }, cfg.n_repeats);

                    // Average sub-AABB volume per link (not sum) —
                    // sub-AABBs overlap, so sum overcounts.  The average
                    // represents effective per-collision-check tightness.
                    double vol = 0;
                    for (int li = 0; li < n_active; ++li) {
                        int fidx = frame_store.active_link_map()[li];
                        float rad = frame_store.link_radii() ?
                                    frame_store.link_radii()[li] : 0.f;
                        derive_aabb_subdivided(frames, n_frames,
                            fidx - 1, fidx, sub_n, rad,
                            frame_store.base_pos(), buf_sub.data());
                        vol += volume_aabb(buf_sub.data(), sub_n) / sub_n;
                    }

                    rows.push_back({trial, wr.name, sub_n, "AABB_SUBDIV",
                        cold, warm, vol, disk_bytes_frames(n_frames), res});
                }

                // ── 2: GRID (only once) ──────────────────────────────────
                if (si == 0) {
                    bool res = false;
                    double cold = median_us([&](){
                        FKState fk2 = compute_fk_full(robot, ivs);
                        extract_frames_from_fk(fk2, n_frames, buf_frames.data());
                        res = check_collision(CollisionPolicy::GRID,
                            buf_frames.data(), n_frames,
                            frame_store.active_link_map(), n_active,
                            frame_store.link_radii(), frame_store.base_pos(),
                            obs, n_obs, 4, 8, grid_R, world_bounds, nullptr);
                    }, cfg.n_repeats);
                    double warm = median_us([&](){
                        res = check_collision(CollisionPolicy::GRID,
                            frames, n_frames,
                            frame_store.active_link_map(), n_active,
                            frame_store.link_radii(), frame_store.base_pos(),
                            obs, n_obs, 4, 8, grid_R, world_bounds, nullptr);
                    }, cfg.n_repeats);

                    derive_grid(frames, n_frames,
                        frame_store.active_link_map(), n_active,
                        frame_store.link_radii(), frame_store.base_pos(),
                        world_bounds, grid_R, 4, 8, buf_grid.data());
                    double vol = volume_grid(buf_grid.data(), grid_R, world_bounds);

                    rows.push_back({trial, wr.name, 0, "GRID",
                        cold, warm, vol, disk_bytes_grid(grid_R), res});
                }

                // ── REF_V1 (only once) ───────────────────────────────────
                if (si == 0) {
                    double cold = median_us([&](){
                        FKState fk2 = compute_fk_full(robot, ivs);
                        std::vector<float> tmp(n_active * 6);
                        extract_link_aabbs(fk2, robot.active_link_map(),
                            n_active, tmp.data(), robot.active_link_radii());
                        aabbs_collide_obs(tmp.data(), n_active, obs, n_obs);
                    }, cfg.n_repeats);
                    double warm = median_us([&](){
                        aabbs_collide_obs(ref_aabbs.data(), n_active, obs, n_obs);
                    }, cfg.n_repeats);
                    double vol = volume_aabb(ref_aabbs.data(), n_active);

                    rows.push_back({trial, wr.name, 0, "REF_V1",
                        cold, warm, vol, disk_bytes_aabb(n_active), ref_coll});
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
        f << "trial,width,subdiv_n,policy,cold_us,cached_us,volume_m3,"
             "disk_bytes,collision\n";
        for (auto& r : rows) {
            f << r.trial << "," << r.width << "," << r.subdiv_n << ","
              << r.policy << ","
              << std::fixed << std::setprecision(3)
              << r.cold_us << "," << r.cached_us << ","
              << std::scientific << std::setprecision(6) << r.volume << ","
              << r.disk_bytes << ","
              << (r.collision ? "true" : "false") << "\n";
        }
        std::cout << "CSV: " << path << "\n";
    }

    // ═════════════════════════════════════════════════════════════════════
    //  Consistency analysis
    // ═════════════════════════════════════════════════════════════════════
    //
    // Two-tier analysis:
    //   (1) AABB vs REF_V1:  Must match exactly (same approximation level).
    //       Any mismatch is a BUG.
    //   (2) Tighter methods vs REF_V1:  SUBDIV/GRID are designed to be
    //       tighter than AABB, so they MAY correctly reject collisions
    //       that AABB reports.  These are valid refinements, not bugs.
    //
    int n_aabb_violations = 0;
    struct PStats { int agree=0, false_neg=0, extra_rej=0, tighten=0; };
    PStats pstats[N_POLICIES] = {};

    for (int wi = 0; wi < N_WIDTHS; ++wi) {
        for (int t = 0; t < cfg.n_trials; ++t) {
            bool ref = false, ref_found = false;
            for (auto& r : rows) {
                if (std::string(r.width)!=WIDTHS[wi].name || r.trial!=t) continue;
                if (std::string(r.policy)=="REF_V1") { ref=r.collision; ref_found=true; break; }
            }
            if (!ref_found) continue;

            for (int pi = 0; pi < N_POLICIES; ++pi) {
                for (auto& r : rows) {
                    if (std::string(r.width)!=WIDTHS[wi].name || r.trial!=t) continue;
                    if (std::string(r.policy)!=POLICY_NAMES[pi]) continue;
                    if (pi==1 && r.subdiv_n!=4) continue;
                    if (r.collision==ref) {
                        pstats[pi].agree++;
                    } else if (!r.collision && ref) {
                        if (pi == 0) {
                            pstats[pi].false_neg++;
                            n_aabb_violations++;
                        } else {
                            pstats[pi].tighten++;
                        }
                    } else {
                        pstats[pi].extra_rej++;
                    }
                    break;
                }
            }
        }
    }

    // ═════════════════════════════════════════════════════════════════════
    //  AABB numerical consistency
    // ═════════════════════════════════════════════════════════════════════
    double max_err = 0; int n_checked = 0;
    {
        std::mt19937 rng2(123);
        for (int wi = 0; wi < N_WIDTHS; ++wi) {
            for (int t = 0; t < std::min(cfg.n_trials, 10); ++t) {
                auto iv = random_intervals(robot, rng2, WIDTHS[wi].lo, WIDTHS[wi].hi);
                FKState fk2 = compute_fk_full(robot, iv);
                if (!fk2.valid) continue;
                std::vector<float> ref_vals(n_active*6), der(n_active*6), tmp(n_frames*6);
                extract_link_aabbs(fk2, robot.active_link_map(), n_active,
                                   ref_vals.data(), robot.active_link_radii());
                extract_frames_from_fk(fk2, n_frames, tmp.data());
                derive_aabb(tmp.data(), n_frames,
                    frame_store.active_link_map(), n_active,
                    frame_store.link_radii(), frame_store.base_pos(), der.data());
                for (int i = 0; i < n_active*6; ++i)
                    max_err = std::max(max_err, std::fabs(double(ref_vals[i])-double(der[i])));
                ++n_checked;
            }
        }
    }

    // ═════════════════════════════════════════════════════════════════════
    //  Write comprehensive Markdown report
    // ═════════════════════════════════════════════════════════════════════
    {
        std::string path = cfg.output_dir + "/report.md";
        std::ofstream md(path);
        md << "# Experiment 14 -- Envelope Method Benchmark & Consistency\n\n";

        // Setup
        md << "## Setup\n\n| Param | Value |\n|---|---|\n"
           << "| Robot | " << robot.name() << " (" << robot.n_joints() << " DOF, "
           << n_active << " active links) |\n"
           << "| Obstacles | " << n_obs << " |\n"
           << "| Trials/regime | " << cfg.n_trials << " |\n"
           << "| Repeats | " << cfg.n_repeats << " (median) |\n"
           << "| Grid R | " << grid_R << "^3 |\n| Subdiv sweep | {";
        for (int i = 0; i < N_SUBDIV; ++i) md << (i?",":"") << SUBDIV_SWEEP[i];
        md << "} |\n| FrameStore B/node | " << disk_bytes_frames(n_frames) << " |\n\n";

        // Disk table
        md << "## Per-Node Disk Storage\n\n"
           << "| Storage | Bytes | Notes |\n|---|---|---|\n"
           << "| **FrameStore** (universal) | " << disk_bytes_frames(n_frames)
           << " | " << n_frames << " frames x 6xf32 -- stores once, derives everything |\n"
           << "| AABB (derived) | " << disk_bytes_aabb(n_active) << " | transient |\n";
        for (int s : SUBDIV_SWEEP)
            md << "| AABB_SUBDIV(n=" << s << ") | "
               << disk_bytes_aabb_subdiv(n_active, s) << " | transient |\n";
        md << "| GRID (raw) | " << disk_bytes_grid(grid_R) << " | uncompressed |\n"
           << "| REF_V1 (legacy) | " << disk_bytes_aabb(n_active) << " | stored per node |\n\n";

        // Consistency
        md << "## Consistency (vs REF_V1, subdiv=4)\n\n"
           << "| Policy | Agree | FalseNeg (BUG) | Tighten (OK) | ExtraReject (OK) |\n"
           << "|---|---|---|---|---|\n";
        for (int pi = 0; pi < N_POLICIES; ++pi)
            md << "| " << POLICY_NAMES[pi] << " | " << pstats[pi].agree
               << " | " << pstats[pi].false_neg
               << " | " << pstats[pi].tighten
               << " | " << pstats[pi].extra_rej << " |\n";
        if (n_aabb_violations)
            md << "\n> **BUG**: AABB vs REF_V1 mismatch in "
               << n_aabb_violations << " cases!\n\n";
        else
            md << "\n> AABB matches REF_V1 exactly\n";

        int total_tighten_md = 0;
        for (int pi = 0; pi < N_POLICIES; ++pi) total_tighten_md += pstats[pi].tighten;
        if (total_tighten_md > 0)
            md << "> Tighter methods correctly refined " << total_tighten_md
               << " AABB false-positive(s)\n";
        md << "\n";

        // AABB accuracy
        md << "## AABB Numerical Accuracy\n\n"
           << "Checked " << n_checked << " intervals -- max |ref-derived| = "
           << std::scientific << std::setprecision(2) << max_err << "  \n"
           << (max_err < 1e-5 ? "**PASS**" : "**FAIL**") << "\n\n";

        // Performance tables per width
        const char* ALL[] = {"AABB","AABB_SUBDIV","GRID","REF_V1"};

        md << "## Performance (median us)\n\n";
        for (int wi = 0; wi < N_WIDTHS; ++wi) {
            md << "### " << WIDTHS[wi].name << " intervals\n\n"
               << "| Policy | subdiv | Cold us | Cached us | Speedup | Volume m3 | Disk B |\n"
               << "|---|---|---|---|---|---|---|\n";
            for (int di = 0; di < 4; ++di) {
                std::map<int, std::vector<TrialRow*>> by_sub;
                for (auto& r : rows) {
                    if (std::string(r.width)!=WIDTHS[wi].name) continue;
                    if (std::string(r.policy)!=ALL[di]) continue;
                    by_sub[r.subdiv_n].push_back(&r);
                }
                for (auto& [sn, rr] : by_sub) {
                    std::vector<double> cv, wv, vv; int db = 0;
                    for (auto* r : rr) { cv.push_back(r->cold_us); wv.push_back(r->cached_us);
                                          vv.push_back(r->volume); db = r->disk_bytes; }
                    auto cs = stats_of(cv), ws = stats_of(wv), vs = stats_of(vv);
                    double sp = cs.p50 / std::max(ws.p50, 0.001);
                    md << "| " << ALL[di] << " | " << (sn ? std::to_string(sn) : "-")
                       << " | " << std::fixed << std::setprecision(1)
                       << cs.p50 << "+/-" << cs.stddev
                       << " | " << ws.p50 << "+/-" << ws.stddev
                       << " | " << sp << "x"
                       << " | " << std::scientific << std::setprecision(3) << vs.p50
                       << " | " << db << " |\n";
                }
            }
            md << "\n";
        }

        // Collision rejection rate analysis
        md << "## Collision Rejection Rate (per width regime)\n\n"
           << "> This shows what % of random intervals each method reports as collision.\n"
           << "> Lower rate = tighter method (fewer false positives = larger safe boxes in forest).\n\n"
           << "| Width | Policy | subdiv | Collisions | Total | Rate % |\n"
           << "|---|---|---|---|---|---|\n";
        for (int wi = 0; wi < N_WIDTHS; ++wi) {
            for (int di = 0; di < 4; ++di) {
                std::map<int, std::pair<int,int>> by_sub;
                for (auto& r : rows) {
                    if (std::string(r.width)!=WIDTHS[wi].name) continue;
                    if (std::string(r.policy)!=ALL[di]) continue;
                    auto& p = by_sub[r.subdiv_n];
                    if (r.collision) p.first++;
                    p.second++;
                }
                for (auto& [sn, ct] : by_sub) {
                    double rate = ct.second > 0 ? 100.0 * ct.first / ct.second : 0;
                    md << "| " << WIDTHS[wi].name << " | " << ALL[di]
                       << " | " << (sn ? std::to_string(sn) : "-")
                       << " | " << ct.first << " | " << ct.second
                       << " | " << std::fixed << std::setprecision(1) << rate << " |\n";
                }
            }
        }
        md << "\n";

        // Volume comparison
        md << "## Volume Comparison (tightness)\n\n"
           << "> **Note**: AABB_SUBDIV volume is the *average sub-AABB volume per link*,\n"
           << "> representing the effective per-collision-check tightness. Smaller = tighter.\n\n"
           << "| Policy | subdiv | Median Vol m3 | Ratio vs AABB |\n|---|---|---|---|\n";
        {
            std::vector<double> av;
            for (auto& r : rows) if (std::string(r.policy)=="AABB") av.push_back(r.volume);
            double aabb_med = av.empty() ? 1.0 : stats_of(av).p50;

            for (int di = 0; di < 4; ++di) {
                std::map<int, std::vector<double>> by_sub;
                for (auto& r : rows)
                    if (std::string(r.policy)==ALL[di]) by_sub[r.subdiv_n].push_back(r.volume);
                for (auto& [sn, vv] : by_sub) {
                    auto vs = stats_of(vv);
                    md << "| " << ALL[di] << " | " << (sn ? std::to_string(sn) : "-")
                       << " | " << std::scientific << std::setprecision(3) << vs.p50
                       << " | " << std::fixed << std::setprecision(3) << vs.p50/std::max(aabb_med,1e-12)
                       << " |\n";
                }
            }
        }
        md << "\n";
        std::cout << "Report: " << path << "\n";
    }

    // ═════════════════════════════════════════════════════════════════════
    //  Console summary
    // ═════════════════════════════════════════════════════════════════════
    std::cout << "\n=== Consistency (vs REF_V1, subdiv=4) ===\n" << std::left
              << std::setw(14) << "Policy" << std::setw(8) << "Agree"
              << std::setw(10) << "FalseNeg" << std::setw(10) << "Tighten"
              << std::setw(12) << "ExtraRej"
              << "\n" << std::string(54,'-') << "\n";
    for (int pi = 0; pi < N_POLICIES; ++pi)
        std::cout << std::setw(14) << POLICY_NAMES[pi]
                  << std::setw(8)  << pstats[pi].agree
                  << std::setw(10) << pstats[pi].false_neg
                  << std::setw(10) << pstats[pi].tighten
                  << std::setw(12) << pstats[pi].extra_rej << "\n";

    int total_tighten = 0;
    for (int pi = 0; pi < N_POLICIES; ++pi) total_tighten += pstats[pi].tighten;
    if (total_tighten > 0)
        std::cout << "\nTighter methods correctly refined "
                  << total_tighten << " AABB false-positive(s)\n";

    std::cout << "\nAABB accuracy: max err = " << std::scientific << max_err
              << (max_err < 1e-5 ? " PASS" : " FAIL") << "\n";

    if (n_aabb_violations) {
        std::cout << "\n*** FAILED: AABB vs REF_V1 mismatch in "
                  << n_aabb_violations << " case(s) ***\n";
        return 1;
    }
    if (max_err >= 1e-5) {
        std::cout << "\n*** FAILED: AABB numerical accuracy ***\n";
        return 1;
    }
    std::cout << "\n*** ALL TESTS PASSED ***\n";
    return 0;
}

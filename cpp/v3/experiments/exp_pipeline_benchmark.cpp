// ═══════════════════════════════════════════════════════════════════════════
//  Experiment: Full Pipeline Benchmark  (v2 — with optimisation fixes)
//
//  Sweeps all 4×3×2 = 24 combinations of:
//    AABB source:         IFK (hybrid IA/AA), CriticalSample, AnalyticalCritical, GCPC
//    Envelope type:       SubAABB, SubAABB_Grid, Hull16_Grid
//    Mode:                Cold (compute from scratch), Warm (load cached)
//
//  Key improvements over v1:
//    • AnalyticalCrit bypasses frames → uses sub-AABBs directly
//    • IFK uses Affine Arithmetic (AA) for narrow intervals (≤ crossover)
//    • Scoring: tightest volume wins (time reported separately)
//    • SubAABB_Grid: adaptive world bounds from robot workspace
//    • Trial distribution: 40% narrow / 40% medium / 20% wide
//    • Warm mode: realistic memcpy of full frame + envelope data
//    • SubAABB_Grid rasterisation uses memset for contiguous z-ranges
//
//  Usage:
//    exp_pipeline <robot.json>  [n_trials=50] [output_dir=exp_pipeline_out]
//
//  Outputs:
//    <output_dir>/best_config.json          — best hyperparams per combo
//    <output_dir>/benchmark_results.csv     — timing & volume table
//    <output_dir>/warm_disk_size.csv        — per-node disk footprint
// ═══════════════════════════════════════════════════════════════════════════

#define _USE_MATH_DEFINES
#include <cmath>

#include "sbf/robot/robot.h"
#include "sbf/robot/fk.h"
#include "sbf/robot/interval_fk.h"
#include "sbf/robot/affine_fk.h"
#include "sbf/envelope/envelope_derive.h"
#include "sbf/envelope/envelope_derive_critical.h"
#include "sbf/envelope/gcpc_cache.h"
#include "sbf/envelope/frame_store.h"
#include "sbf/envelope/grid_store.h"
#include "sbf/voxel/voxel_grid.h"
#include "sbf/voxel/hull_rasteriser.h"
#include "sbf/common/types.h"

#include <Eigen/Core>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using json = nlohmann::json;
using namespace sbf;
using namespace sbf::envelope;
using namespace sbf::voxel;

// ─── Enums for the pipeline axes ────────────────────────────────────────────
enum class AABBSource  { IFK = 0, CritSample = 1, AnalyticalCrit = 2, GCPC = 3 };
enum class EnvType     { SubAABB = 0, SubAABB_Grid = 1, Hull16_Grid = 2 };
enum class Mode        { Cold = 0, Warm = 1 };

static constexpr int N_SOURCES = 4;

// All AABB sources produce endpoint AABBs at this uniform resolution
static constexpr int MAX_EP_NSUB = 16;

// ─── AA crossover: IFK switches to AA when max interval width ≤ threshold ───
// iiwa14 narrow trials (frac=0.05) have max_w ≈ 0.29 rad, so 0.5 captures them.
static constexpr double AA_CROSSOVER = 0.50;

static const char* aabb_source_names[]  = { "IFK(AA<=0.5)", "CritSample", "AnalyticalCrit", "GCPC" };
static const char* env_type_names[]     = { "SubAABB", "SubAABB_Grid", "Hull16_Grid" };
static const char* mode_names[]         = { "Cold", "Warm" };

// ─── Timer helper ───────────────────────────────────────────────────────────
struct Timer {
    using Clock = std::chrono::high_resolution_clock;
    Clock::time_point t0;
    void start() { t0 = Clock::now(); }
    double elapsed_ms() const {
        return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    }
};

// ─── Random C-space box generator ───────────────────────────────────────────
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

// ─── Compute frames from IFK / CritSample source ───────────────────────────
//
// Produces per-link endpoint AABBs at MAX_EP_NSUB resolution (linear
// interpolation of prox/dist).  Same format as Analytical/GCPC output.
//
struct FrameResult {
    std::vector<float> frames;           // [n_frames × 6]
    std::vector<float> endpoint_aabbs;   // [n_act × (MAX_EP_NSUB+1) × 6]
    FKState fk;                          // valid for IFK source only
    double compute_ms = 0;
};

FrameResult compute_frames(AABBSource src, const Robot& robot,
                           const std::vector<Interval>& intervals)
{
    const int n = robot.n_joints();
    const bool has_tool = robot.has_tool();
    const int n_frames = n + (has_tool ? 1 : 0);

    FrameResult res;
    res.frames.resize(n_frames * 6);
    res.fk.valid = false;

    Timer t;
    t.start();

    switch (src) {
    case AABBSource::IFK: {
        // Hybrid IA/AA: use AA for narrow intervals (tighter bounds)
        double max_w = 0.0;
        for (const auto& iv : intervals)
            max_w = std::max(max_w, iv.hi - iv.lo);

        if (max_w <= AA_CROSSOVER) {
            // AA path: affine arithmetic is tighter at narrow intervals
            auto aa_fk = aa_compute_fk(robot, intervals);
            for (int k = 0; k < n_frames; ++k) {
                int fi = k + 1;
                double lo3[3], hi3[3];
                aa_position_bounds(aa_fk.prefix[fi], lo3, hi3);
                res.frames[k*6+0] = static_cast<float>(lo3[0]);
                res.frames[k*6+1] = static_cast<float>(lo3[1]);
                res.frames[k*6+2] = static_cast<float>(lo3[2]);
                res.frames[k*6+3] = static_cast<float>(hi3[0]);
                res.frames[k*6+4] = static_cast<float>(hi3[1]);
                res.frames[k*6+5] = static_cast<float>(hi3[2]);
            }
            // Also compute IA FK state (needed for warm mode memcpy sizing)
            res.fk = compute_fk_full(robot, intervals);
        } else {
            // IA path: standard interval arithmetic
            res.fk = compute_fk_full(robot, intervals);
            for (int k = 0; k < n_frames; ++k) {
                int fi = k + 1;
                res.frames[k*6+0] = static_cast<float>(res.fk.prefix_lo[fi][3]);
                res.frames[k*6+1] = static_cast<float>(res.fk.prefix_lo[fi][7]);
                res.frames[k*6+2] = static_cast<float>(res.fk.prefix_lo[fi][11]);
                res.frames[k*6+3] = static_cast<float>(res.fk.prefix_hi[fi][3]);
                res.frames[k*6+4] = static_cast<float>(res.fk.prefix_hi[fi][7]);
                res.frames[k*6+5] = static_cast<float>(res.fk.prefix_hi[fi][11]);
            }
        }
        break;
    }
    case AABBSource::CritSample: {
        derive_crit_endpoints(robot, intervals, res.frames.data());
        break;
    }
    case AABBSource::AnalyticalCrit:
        break;  // Handled separately
    }

    res.compute_ms = t.elapsed_ms();

    // Convert frames → endpoint_aabbs at MAX_EP_NSUB resolution
    {
        const int n_act_loc = robot.n_active_links();
        const int* alm = robot.active_link_map();
        const int ep_n = MAX_EP_NSUB;
        res.endpoint_aabbs.resize(n_act_loc * (ep_n + 1) * 6);
        float bp[3] = {0.f, 0.f, 0.f};
        for (int ci = 0; ci < n_act_loc; ++ci) {
            int fi = alm[ci];
            float prox[6], dist_v[6];
            if (fi == 0) {
                for (int a = 0; a < 3; ++a) {
                    prox[a] = bp[a]; prox[a+3] = bp[a];
                }
            } else {
                std::memcpy(prox, res.frames.data() + (fi - 1) * 6, 6 * sizeof(float));
            }
            std::memcpy(dist_v, res.frames.data() + fi * 6, 6 * sizeof(float));
            // Linear interpolation: ep[s] = lerp(prox, dist, s/ep_n)
            for (int s = 0; s <= ep_n; ++s) {
                float u = static_cast<float>(s) / static_cast<float>(ep_n);
                float* ep = res.endpoint_aabbs.data() + (ci * (ep_n + 1) + s) * 6;
                for (int c = 0; c < 6; ++c)
                    ep[c] = prox[c] * (1.f - u) + dist_v[c] * u;
            }
        }
    }

    return res;
}

// ─── Compute analytical sub-AABBs (AnalyticalCrit-specific) ─────────────────
//
// AnalyticalCrit bypasses frames entirely.  derive_aabb_critical_analytical
// produces per-link per-sub-segment AABB[6] directly usable as the SubAABB
// envelope.  For grid/hull types, each sub-AABB is rasterised into the
// target representation.
//
struct AnalyticalResult {
    std::vector<float> sub_aabbs;    // [n_act × n_sub × 6]
    std::vector<float> endpoint_aabbs;  // [n_act × (n_sub+1) × 6] — per-endpoint interval AABBs
    int n_sub;
    double compute_ms;
};

AnalyticalResult compute_analytical_sub_aabbs(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub,
    bool diagnose = false)
{
    AnalyticalResult ar;
    ar.n_sub = n_sub;

    AnalyticalCriticalConfig cfg = AnalyticalCriticalConfig::all_enabled();
    cfg.dual_phase3 = true;

    const int n_act = robot.n_active_links();
    ar.sub_aabbs.resize(n_act * n_sub * 6);
    ar.endpoint_aabbs.resize(n_act * (n_sub + 1) * 6);

    AnalyticalCriticalStats stats{};

    Timer t;
    t.start();
    derive_aabb_critical_analytical(robot, intervals, n_sub, cfg,
                                    ar.sub_aabbs.data(), &stats,
                                    ar.endpoint_aabbs.data());
    ar.compute_ms = t.elapsed_ms();

    if (diagnose) {
        std::cout << "  [DIAG] AnalyticalCrit n_sub=" << n_sub
                  << " phases: P0=" << stats.n_phase0_vertices
                  << " P1=" << stats.n_phase1_edges
                  << " P2=" << stats.n_phase2_faces
                  << " P2.5a=" << stats.n_phase25a_pair1d
                  << " P2.5b=" << stats.n_phase25b_pair2d
                  << " P3=" << stats.n_phase3_interior
                  << "\n";
        // Volume
        double vol = 0;
        for (int k = 0; k < n_act * n_sub; ++k) {
            float* a = ar.sub_aabbs.data() + k * 6;
            double dx = std::max(0.0, double(a[3] - a[0]));
            double dy = std::max(0.0, double(a[4] - a[1]));
            double dz = std::max(0.0, double(a[5] - a[2]));
            vol += dx * dy * dz;
        }
        std::cout << "  [DIAG] Uncertified vol=" << std::scientific
                  << std::setprecision(4) << vol << "\n";
    }

    return ar;
}

// ─── Build GCPC cache for a robot (one-time precomputation) ─────────────────
// Enumerates kπ/2 critical configs across full joint range, evaluates FK,
// and builds a GcpcCache.  Same approach as exp_gcpc_validation.
//
static GcpcCache build_gcpc_cache_for_robot(const Robot& robot) {
    const int n = robot.n_joints();
    const int n_act = robot.n_active_links();
    const int* link_map = robot.active_link_map();
    auto limits = robot.joint_limits();

    // Gather kπ/2 values per joint (within full joint limits)
    std::vector<std::vector<double>> per_joint(n);
    for (int j = 0; j < n; ++j) {
        double lo = limits.limits[j].lo, hi = limits.limits[j].hi;
        per_joint[j].push_back(lo);
        per_joint[j].push_back(hi);
        per_joint[j].push_back(0.5 * (lo + hi));
        for (int k = -20; k <= 20; ++k) {
            double a = k * M_PI * 0.5;
            if (a > lo + 1e-10 && a < hi - 1e-10)
                per_joint[j].push_back(a);
        }
        std::sort(per_joint[j].begin(), per_joint[j].end());
        per_joint[j].erase(
            std::unique(per_joint[j].begin(), per_joint[j].end()),
            per_joint[j].end());
    }

    // For q₁: only [0, π] (symmetry reduction)
    std::vector<double> q1_half;
    for (double v : per_joint[1]) {
        if (v >= -1e-10 && v <= M_PI + 1e-10)
            q1_half.push_back(std::max(0.0, std::min(M_PI, v)));
    }
    if (q1_half.empty()) q1_half.push_back(0.5);

    std::vector<GcpcPoint> cache_points;

    for (int ci = 0; ci < n_act; ++ci) {
        int link_id = link_map[ci];
        int nj = std::min(link_id + 1, n);
        int n_eff = nj - 1;  // excluding q₀
        if (n_eff <= 0) continue;

        std::vector<std::vector<double>> joint_vals(n_eff);
        joint_vals[0] = q1_half;
        for (int d = 1; d < n_eff; ++d)
            joint_vals[d] = per_joint[d + 1];

        long long product = 1;
        for (int d = 0; d < n_eff; ++d) {
            product *= static_cast<long long>(joint_vals[d].size());
            if (product > 50000) break;
        }

        if (product > 50000) {
            // Subsample
            std::mt19937 rng(12345 + ci);
            for (int s = 0; s < 5000; ++s) {
                GcpcPoint pt{};
                pt.link_id = link_id;
                pt.n_eff = n_eff;
                for (int d = 0; d < n_eff; ++d)
                    pt.q_eff[d] = joint_vals[d][
                        std::uniform_int_distribution<int>(
                            0, static_cast<int>(joint_vals[d].size()) - 1)(rng)];

                Eigen::VectorXd q_full(n);
                q_full.setZero();
                for (int d = 0; d < n_eff && d + 1 < n; ++d)
                    q_full[d + 1] = pt.q_eff[d];

                auto pos = fk_link_positions(robot, q_full);
                if (link_id + 1 < static_cast<int>(pos.size())) {
                    const auto& p = pos[link_id + 1];
                    pt.direction = 0;
                    pt.A = p[0]; pt.B = p[1]; pt.C = p[2];
                    pt.R = std::sqrt(pt.A * pt.A + pt.B * pt.B);
                    cache_points.push_back(pt);
                    GcpcPoint ptz = pt; ptz.direction = 1;
                    cache_points.push_back(ptz);
                }
            }
        } else {
            // Full enumeration
            std::vector<double> config(n_eff);
            std::function<void(int)> enumerate;
            enumerate = [&](int d) {
                if (d >= n_eff) {
                    GcpcPoint pt{};
                    pt.link_id = link_id;
                    pt.n_eff = n_eff;
                    for (int i = 0; i < n_eff; ++i)
                        pt.q_eff[i] = config[i];

                    Eigen::VectorXd q_full(n);
                    q_full.setZero();
                    for (int i = 0; i < n_eff && i + 1 < n; ++i)
                        q_full[i + 1] = pt.q_eff[i];

                    auto pos = fk_link_positions(robot, q_full);
                    if (link_id + 1 < static_cast<int>(pos.size())) {
                        const auto& p = pos[link_id + 1];
                        pt.direction = 0;
                        pt.A = p[0]; pt.B = p[1]; pt.C = p[2];
                        pt.R = std::sqrt(pt.A * pt.A + pt.B * pt.B);
                        cache_points.push_back(pt);
                        GcpcPoint ptz = pt; ptz.direction = 1;
                        cache_points.push_back(ptz);
                    }
                    return;
                }
                for (double v : joint_vals[d]) {
                    config[d] = v;
                    enumerate(d + 1);
                }
            };
            enumerate(0);
        }
    }

    GcpcCache cache;
    cache.build(robot, cache_points);

    // Enrich with interior critical points via coordinate descent
    int n_added = cache.enrich_with_interior_search(robot, 500, 5);
    printf("  GCPC cache: %d base + %d interior = %d total points\n",
           static_cast<int>(cache_points.size()), n_added, cache.n_total_points());

    return cache;
}

// ─── Compute GCPC sub-AABBs (GCPC-specific) ────────────────────────────────
//
// Uses pre-built GCPC cache: KD-tree range query + boundary enumeration.
// Output format identical to AnalyticalCrit.
//
static AnalyticalResult compute_gcpc_sub_aabbs(
    const GcpcCache& cache,
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub)
{
    AnalyticalResult ar;
    ar.n_sub = n_sub;
    const int n_act = robot.n_active_links();
    ar.sub_aabbs.resize(n_act * n_sub * 6);
    ar.endpoint_aabbs.resize(n_act * (n_sub + 1) * 6);

    Timer t;
    t.start();
    GcpcQueryStats gcpc_stats;
    cache.derive_aabb_with_gcpc(robot, intervals, n_sub,
                                ar.sub_aabbs.data(),
                                &gcpc_stats,
                                ar.endpoint_aabbs.data());
    ar.compute_ms = t.elapsed_ms();

    return ar;
}

// ─── P1-1: Dedup volume helper (rasterise sub-AABBs → byte grid → popcount) ─
static double compute_dedup_volume(
    const float* sub_aabbs, int n_boxes,
    const float* world_bounds, int R = 64)
{
    float cell[3];
    for (int c = 0; c < 3; ++c)
        cell[c] = (world_bounds[3 + c] - world_bounds[c]) /
                  static_cast<float>(R);

    std::vector<uint8_t> grid(R * R * R, 0);
    for (int k = 0; k < n_boxes; ++k) {
        const float* aabb = sub_aabbs + k * 6;
        int ix0 = std::max(0, (int)std::floor((aabb[0] - world_bounds[0]) / cell[0]));
        int iy0 = std::max(0, (int)std::floor((aabb[1] - world_bounds[1]) / cell[1]));
        int iz0 = std::max(0, (int)std::floor((aabb[2] - world_bounds[2]) / cell[2]));
        int ix1 = std::min(R, (int)std::ceil ((aabb[3] - world_bounds[0]) / cell[0]));
        int iy1 = std::min(R, (int)std::ceil ((aabb[4] - world_bounds[1]) / cell[1]));
        int iz1 = std::min(R, (int)std::ceil ((aabb[5] - world_bounds[2]) / cell[2]));
        for (int x = ix0; x < ix1; ++x)
            for (int y = iy0; y < iy1; ++y)
                std::memset(grid.data() + x * R * R + y * R + iz0, 1, iz1 - iz0);
    }
    int occ = 0;
    for (auto v : grid) occ += v;
    double cell_vol = 1.0;
    for (int a = 0; a < 3; ++a)
        cell_vol *= (world_bounds[a+3] - world_bounds[a]) / R;
    return occ * cell_vol;
}

// ─── Compute envelope from frames (IFK / CritSample) ───────────────────────
struct EnvResult {
    double volume   = 0;    // m³ for grids, sum-of-AABB for sub_aabb
    double dedup_volume = 0; // P1-1: union volume via byte-grid rasterisation
    int    n_voxels = 0;    // occupied voxels (grid types), or n_sub_aabbs
    double compute_ms = 0;
    int    n_bricks = 0;    // BitBrick count (hull16 only)
};

// Helper: compute adaptive workspace bounds from frame data
static void compute_workspace_bounds(
    const std::vector<float>& all_frames,  // concatenated N × n_frames × 6
    int n_frames, int N, float margin,
    float* wb)
{
    wb[0] = wb[1] = wb[2] =  1e30f;
    wb[3] = wb[4] = wb[5] = -1e30f;
    for (int t = 0; t < N; ++t) {
        const float* fp = all_frames.data() + t * n_frames * 6;
        for (int k = 0; k < n_frames; ++k) {
            for (int d = 0; d < 3; ++d) {
                wb[d]     = std::min(wb[d],     fp[k * 6 + d]);
                wb[d + 3] = std::max(wb[d + 3], fp[k * 6 + d + 3]);
            }
        }
    }
    for (int d = 0; d < 3; ++d) {
        wb[d]     -= margin;
        wb[d + 3] += margin;
    }
}

#if 0  // ── Replaced by unified compute_envelope() ──
// Envelope from frames (IFK / CritSample path)
EnvResult compute_envelope_from_frames(
    EnvType etype,
    const Robot& robot,
    const float* frames,
    int n_sub,
    double voxel_delta,
    int grid_R,
    const float* world_bounds)
{
    const int n_frames = robot.n_joints() + (robot.has_tool() ? 1 : 0);
    const int n_act = robot.n_active_links();
    const int* alm = robot.active_link_map();

    std::vector<float> lrf(n_act);
    const double* lr = robot.active_link_radii();
    if (lr)
        for (int i = 0; i < n_act; ++i) lrf[i] = static_cast<float>(lr[i]);

    float base_pos[3] = {0.f, 0.f, 0.f};

    EnvResult res;
    std::vector<float> sub_aabb_buf;  // P1-1: saved for dedup metric (outside timer)
    Timer t;
    t.start();

    switch (etype) {

    case EnvType::SubAABB: {
        sub_aabb_buf.resize(n_act * n_sub * 6);
        for (int ci = 0; ci < n_act; ++ci) {
            int parent_fi = (alm[ci] == 0) ? -1 : alm[ci] - 1;
            int link_fi = alm[ci];
            float r = lr ? lrf[ci] : 0.f;
            derive_aabb_subdivided(
                frames, n_frames, parent_fi, link_fi,
                n_sub, r, base_pos,
                sub_aabb_buf.data() + ci * n_sub * 6);
        }
        double vol = 0;
        int total = n_act * n_sub;
        for (int k = 0; k < total; ++k) {
            float* a = sub_aabb_buf.data() + k * 6;
            double dx = std::max(0.0, double(a[3] - a[0]));
            double dy = std::max(0.0, double(a[4] - a[1]));
            double dz = std::max(0.0, double(a[5] - a[2]));
            vol += dx * dy * dz;
        }
        res.volume = vol;
        res.n_voxels = total;
        break;
    }

    case EnvType::SubAABB_Grid: {
        std::vector<uint8_t> grid(grid_R * grid_R * grid_R, 0);
        derive_grid(frames, n_frames, alm, n_act,
                    lrf.data(), base_pos,
                    world_bounds, grid_R, n_sub, n_sub,
                    grid.data());
        int occ = 0;
        for (auto v : grid) occ += v;
        double cell_vol = 1.0;
        for (int a = 0; a < 3; ++a)
            cell_vol *= (world_bounds[a+3] - world_bounds[a]) / grid_R;
        res.volume = occ * cell_vol;
        res.n_voxels = occ;
        break;
    }

    case EnvType::Hull16_Grid: {
        VoxelGrid grid(voxel_delta, 0, 0, 0, -1.0);  // P0-1: use default √3·Δ/2 safety pad
        for (int ci = 0; ci < n_act; ++ci) {
            int fi = alm[ci];
            float prox[6], dist[6];
            if (fi == 0) {
                for (int a = 0; a < 3; ++a) {
                    prox[a] = base_pos[a]; prox[a+3] = base_pos[a];
                }
            } else {
                std::memcpy(prox, frames + (fi - 1) * 6, 6 * sizeof(float));
            }
            std::memcpy(dist, frames + fi * 6, 6 * sizeof(float));
            double r = lr ? lr[ci] : 0.0;

            // P1-2: adaptive per-link n_sub = min(n_sub, ceil(diag / delta))
            int link_nsub = n_sub;
            if (n_sub > 1) {
                float diag_sq = 0.f;
                for (int c = 0; c < 3; ++c) {
                    float lo = std::min(prox[c], dist[c]);
                    float hi = std::max(prox[c+3], dist[c+3]);
                    float d = hi - lo;
                    diag_sq += d * d;
                }
                float diag = std::sqrt(diag_sq);
                link_nsub = std::max(1, std::min(n_sub,
                    static_cast<int>(std::ceil(diag / static_cast<float>(voxel_delta)))));
            }

            if (link_nsub <= 1) {
                grid.fill_hull16(prox, dist, r);
            } else {
                float inv_n = 1.0f / static_cast<float>(link_nsub);
                for (int s = 0; s < link_nsub; ++s) {
                    float t0 = s * inv_n, t1 = (s + 1) * inv_n;
                    float sp[6], sd[6];
                    for (int c = 0; c < 3; ++c) {
                        sp[c]   = prox[c]   * (1 - t0) + dist[c]   * t0;
                        sp[c+3] = prox[c+3] * (1 - t0) + dist[c+3] * t0;
                        sd[c]   = prox[c]   * (1 - t1) + dist[c]   * t1;
                        sd[c+3] = prox[c+3] * (1 - t1) + dist[c+3] * t1;
                    }
                    grid.fill_hull16(sp, sd, r);
                }
            }
        }
        res.volume = grid.occupied_volume();
        res.n_voxels = grid.count_occupied();
        res.n_bricks = grid.num_bricks();
        break;
    }
    }

    res.compute_ms = t.elapsed_ms();

    // P1-1: dedup volume (diagnostic, excluded from timing)
    if (!sub_aabb_buf.empty())
        res.dedup_volume = compute_dedup_volume(
            sub_aabb_buf.data(),
            static_cast<int>(sub_aabb_buf.size() / 6),
            world_bounds);

    return res;
}
#endif

#if 0  // ── Replaced by unified compute_envelope() ──
// ─── Compute envelope from analytical sub-AABBs (AnalyticalCrit path) ───────
EnvResult compute_envelope_from_analytical(
    EnvType etype,
    const Robot& robot,
    const float* sub_aabbs,  // [n_act × n_sub × 6]
    int n_sub,
    double voxel_delta,
    int grid_R,
    const float* world_bounds,
    const float* endpoint_aabbs = nullptr,  // [n_act × (endpoint_n_sub+1) × 6], optional
    int endpoint_n_sub = 0)                   // stride used when computing endpoint_aabbs
{
    const int n_act = robot.n_active_links();

    EnvResult res;
    Timer t;
    t.start();

    switch (etype) {

    case EnvType::SubAABB: {
        double vol = 0;
        int total = n_act * n_sub;
        for (int k = 0; k < total; ++k) {
            const float* a = sub_aabbs + k * 6;
            double dx = std::max(0.0, double(a[3] - a[0]));
            double dy = std::max(0.0, double(a[4] - a[1]));
            double dz = std::max(0.0, double(a[5] - a[2]));
            vol += dx * dy * dz;
        }
        res.volume = vol;
        res.n_voxels = total;
        break;
    }

    case EnvType::SubAABB_Grid: {
        float cell[3];
        for (int c = 0; c < 3; ++c)
            cell[c] = (world_bounds[3 + c] - world_bounds[c]) /
                      static_cast<float>(grid_R);

        std::vector<uint8_t> grid(grid_R * grid_R * grid_R, 0);
        int total = n_act * n_sub;
        for (int k = 0; k < total; ++k) {
            const float* aabb = sub_aabbs + k * 6;
            int ix0 = static_cast<int>(std::floor((aabb[0] - world_bounds[0]) / cell[0]));
            int iy0 = static_cast<int>(std::floor((aabb[1] - world_bounds[1]) / cell[1]));
            int iz0 = static_cast<int>(std::floor((aabb[2] - world_bounds[2]) / cell[2]));
            int ix1 = static_cast<int>(std::ceil((aabb[3] - world_bounds[0]) / cell[0]));
            int iy1 = static_cast<int>(std::ceil((aabb[4] - world_bounds[1]) / cell[1]));
            int iz1 = static_cast<int>(std::ceil((aabb[5] - world_bounds[2]) / cell[2]));

            ix0 = std::max(ix0, 0); ix1 = std::min(ix1, grid_R);
            iy0 = std::max(iy0, 0); iy1 = std::min(iy1, grid_R);
            iz0 = std::max(iz0, 0); iz1 = std::min(iz1, grid_R);

            // memset for contiguous z-ranges (faster than byte-by-byte)
            for (int x = ix0; x < ix1; ++x) {
                for (int y = iy0; y < iy1; ++y) {
                    int base = x * grid_R * grid_R + y * grid_R;
                    std::memset(grid.data() + base + iz0, 1, iz1 - iz0);
                }
            }
        }

        int occ = 0;
        for (auto v : grid) occ += v;
        double cell_vol = 1.0;
        for (int a = 0; a < 3; ++a)
            cell_vol *= (world_bounds[a+3] - world_bounds[a]) / grid_R;
        res.volume = occ * cell_vol;
        res.n_voxels = occ;
        break;
    }

    case EnvType::Hull16_Grid: {
        // Hull16_Grid: one fill_hull16(prox, dist, r) per link — no sub-segments.
        // prox = AABB at t=0, dist = AABB at t=1 (geometry-only, no radius).
        VoxelGrid grid(voxel_delta, 0, 0, 0, 0.0);
        const double* lr = robot.active_link_radii();

        if (endpoint_aabbs) {
            // endpoint_aabb[0] = prox, endpoint_aabb[endpoint_n_sub] = dist
            for (int ci = 0; ci < n_act; ++ci) {
                double r = lr ? lr[ci] : 0.0;
                const float* prox = endpoint_aabbs + (ci * (endpoint_n_sub + 1)) * 6;
                const float* dist = endpoint_aabbs + (ci * (endpoint_n_sub + 1) + endpoint_n_sub) * 6;
                grid.fill_hull16(prox, dist, r);
            }
        } else {
            // Analytical: uninflate first/last sub_aabb as prox/dist over-approx
            for (int ci = 0; ci < n_act; ++ci) {
                float r = lr ? static_cast<float>(lr[ci]) : 0.f;
                float prox[6], dist_box[6];
                const float* a0 = sub_aabbs + (ci * n_sub) * 6;
                const float* aN = sub_aabbs + (ci * n_sub + n_sub - 1) * 6;
                for (int d = 0; d < 3; ++d) {
                    prox[d]     = a0[d]   + r;
                    prox[d + 3] = a0[d+3] - r;
                    dist_box[d]     = aN[d]   + r;
                    dist_box[d + 3] = aN[d+3] - r;
                }
                grid.fill_hull16(prox, dist_box, static_cast<double>(r));
            }
        }
        res.volume = grid.occupied_volume();
        res.n_voxels = grid.count_occupied();
        res.n_bricks = grid.num_bricks();
        break;
    }
    }

    res.compute_ms = t.elapsed_ms();

    // P1-1: dedup volume for SubAABB (diagnostic, excluded from timing)
    if (etype == EnvType::SubAABB)
        res.dedup_volume = compute_dedup_volume(
            sub_aabbs, n_act * n_sub, world_bounds);

    return res;
}
#endif

// ─── Unified envelope from endpoint AABBs ──────────────────────────────────
//
// All sources (IFK, CritSample, AnalyticalCrit, GCPC) produce endpoint AABBs:
//   per-link AABB at each interpolation point t = s/ep_n_sub (s = 0 … ep_n_sub).
//   Geometry-only (no radius); radius applied during envelope derivation.
//
// endpoint_aabbs:  [n_act × (ep_n_sub + 1) × 6]
// target_n_sub:    desired subdivision for SubAABB / SubAABB_Grid
//                  (must divide ep_n_sub; ignored for Hull16_Grid)
//
EnvResult compute_envelope(
    EnvType etype,
    const Robot& robot,
    const float* endpoint_aabbs,  // [n_act × (ep_n_sub + 1) × 6]
    int ep_n_sub,
    int target_n_sub,             // for SubAABB / SubAABB_Grid
    double voxel_delta,
    int grid_R,
    const float* world_bounds)
{
    const int n_act = robot.n_active_links();
    const double* lr = robot.active_link_radii();

    // ── Helper: derive sub-AABBs from endpoint AABBs ────────────────────
    // sub_aabb[ci][s] = union(ep[ci][s·ratio] … ep[ci][(s+1)·ratio]) + radius
    auto derive_sub_aabbs = [&](int nsub, float* out) {
        int ratio = ep_n_sub / nsub;
        for (int ci = 0; ci < n_act; ++ci) {
            float r = lr ? static_cast<float>(lr[ci]) : 0.f;
            for (int s = 0; s < nsub; ++s) {
                float* d = out + (ci * nsub + s) * 6;
                d[0] = d[1] = d[2] =  1e30f;
                d[3] = d[4] = d[5] = -1e30f;
                for (int k = s * ratio; k <= (s + 1) * ratio; ++k) {
                    const float* ep = endpoint_aabbs + (ci * (ep_n_sub + 1) + k) * 6;
                    for (int c = 0; c < 3; ++c) {
                        d[c]     = std::min(d[c], ep[c]);
                        d[c + 3] = std::max(d[c + 3], ep[c + 3]);
                    }
                }
                for (int c = 0; c < 3; ++c) {
                    d[c]     -= r;
                    d[c + 3] += r;
                }
            }
        }
    };

    EnvResult res;
    std::vector<float> sub_aabb_buf;
    Timer t;
    t.start();

    switch (etype) {

    case EnvType::SubAABB: {
        sub_aabb_buf.resize(n_act * target_n_sub * 6);
        derive_sub_aabbs(target_n_sub, sub_aabb_buf.data());
        double vol = 0;
        int total = n_act * target_n_sub;
        for (int k = 0; k < total; ++k) {
            const float* a = sub_aabb_buf.data() + k * 6;
            double dx = std::max(0.0, double(a[3] - a[0]));
            double dy = std::max(0.0, double(a[4] - a[1]));
            double dz = std::max(0.0, double(a[5] - a[2]));
            vol += dx * dy * dz;
        }
        res.volume = vol;
        res.n_voxels = total;
        break;
    }

    case EnvType::SubAABB_Grid: {
        sub_aabb_buf.resize(n_act * target_n_sub * 6);
        derive_sub_aabbs(target_n_sub, sub_aabb_buf.data());

        float cell[3];
        for (int c = 0; c < 3; ++c)
            cell[c] = (world_bounds[3 + c] - world_bounds[c]) /
                      static_cast<float>(grid_R);

        std::vector<uint8_t> grid(grid_R * grid_R * grid_R, 0);
        int total = n_act * target_n_sub;
        for (int k = 0; k < total; ++k) {
            const float* aabb = sub_aabb_buf.data() + k * 6;
            int ix0 = std::max(0, static_cast<int>(std::floor((aabb[0] - world_bounds[0]) / cell[0])));
            int iy0 = std::max(0, static_cast<int>(std::floor((aabb[1] - world_bounds[1]) / cell[1])));
            int iz0 = std::max(0, static_cast<int>(std::floor((aabb[2] - world_bounds[2]) / cell[2])));
            int ix1 = std::min(grid_R, static_cast<int>(std::ceil((aabb[3] - world_bounds[0]) / cell[0])));
            int iy1 = std::min(grid_R, static_cast<int>(std::ceil((aabb[4] - world_bounds[1]) / cell[1])));
            int iz1 = std::min(grid_R, static_cast<int>(std::ceil((aabb[5] - world_bounds[2]) / cell[2])));
            for (int x = ix0; x < ix1; ++x)
                for (int y = iy0; y < iy1; ++y)
                    std::memset(grid.data() + x * grid_R * grid_R + y * grid_R + iz0,
                                1, std::max(0, iz1 - iz0));
        }

        int occ = 0;
        for (auto v : grid) occ += v;
        double cell_vol = 1.0;
        for (int a = 0; a < 3; ++a)
            cell_vol *= (world_bounds[a+3] - world_bounds[a]) / grid_R;
        res.volume = occ * cell_vol;
        res.n_voxels = occ;
        break;
    }

    case EnvType::Hull16_Grid: {
        // SubHull16: subdivide each link into target_n_sub sub-hulls.
        // Each sub-hull covers ep[s*ratio] … ep[(s+1)*ratio], giving
        // tighter envelopes for curved/bent links.
        // When target_n_sub=1, this reduces to the original single hull.
        VoxelGrid grid(voxel_delta, 0, 0, 0, 0.0);
        int ratio = ep_n_sub / target_n_sub;
        for (int ci = 0; ci < n_act; ++ci) {
            double r = lr ? lr[ci] : 0.0;
            for (int s = 0; s < target_n_sub; ++s) {
                int ep_lo = s * ratio;
                int ep_hi = (s + 1) * ratio;
                const float* prox = endpoint_aabbs + (ci * (ep_n_sub + 1) + ep_lo) * 6;
                const float* dist = endpoint_aabbs + (ci * (ep_n_sub + 1) + ep_hi) * 6;
                grid.fill_hull16(prox, dist, r);
            }
        }
        res.volume = grid.occupied_volume();
        res.n_voxels = grid.count_occupied();
        res.n_bricks = grid.num_bricks();
        break;
    }
    }

    res.compute_ms = t.elapsed_ms();

    // Dedup volume for SubAABB (diagnostic, excluded from timing)
    if (!sub_aabb_buf.empty())
        res.dedup_volume = compute_dedup_volume(
            sub_aabb_buf.data(),
            static_cast<int>(sub_aabb_buf.size() / 6),
            world_bounds);

    return res;
}

// ─── Disk size estimation ───────────────────────────────────────────────────
struct DiskSizeResult {
    int frames_bytes = 0;
    int aabb_bytes   = 0;
    int grid_bytes   = 0;
    int total_bytes  = 0;
};

DiskSizeResult estimate_disk_per_node(
    AABBSource src,
    EnvType etype,
    int n_frames, int n_active,
    int n_sub, int grid_R,
    int n_bricks_hull)
{
    DiskSizeResult d;

    // IFK/CritSample store frames; AnalyticalCrit/GCPC stores sub-AABBs directly
    if (src == AABBSource::AnalyticalCrit || src == AABBSource::GCPC) {
        d.aabb_bytes = n_active * n_sub * 6 * 4;
    } else {
        d.frames_bytes = n_frames * 6 * 4;
    }

    switch (etype) {
    case EnvType::SubAABB:
        if (src != AABBSource::AnalyticalCrit && src != AABBSource::GCPC)
            d.aabb_bytes = n_active * n_sub * 6 * 4;
        break;
    case EnvType::SubAABB_Grid:
        if (src != AABBSource::AnalyticalCrit && src != AABBSource::GCPC)
            d.aabb_bytes = n_active * n_sub * 6 * 4;
        d.grid_bytes = grid_R * grid_R * grid_R;
        break;
    case EnvType::Hull16_Grid:
        d.grid_bytes = 4 + n_bricks_hull * 76;
        break;
    }
    d.total_bytes = d.frames_bytes + d.aabb_bytes + d.grid_bytes;
    return d;
}

// ─── Hyperparameter sweep configuration ─────────────────────────────────────
struct HyperConfig {
    int    n_sub  = 4;
    double delta  = 0.02;
    int    grid_R = 32;
};

static std::vector<int>    hp_n_sub_values  = {1, 2, 4, 8, 16};
static std::vector<double> hp_delta_values  = {0.01, 0.02, 0.04};  // P0-1: removed 0.005 (OOM with safety_pad)
static std::vector<int>    hp_grid_R_values = {16, 32, 48, 64};

// ═════════════════════════════════════════════════════════════════════════════
//  Main
// ═════════════════════════════════════════════════════════════════════════════
int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0]
                  << " <robot.json> [n_trials=50] [output_dir=exp_pipeline_out]\n";
        return 1;
    }

    Robot robot = Robot::from_json(argv[1]);
    int N = (argc >= 3) ? std::atoi(argv[2]) : 50;
    std::string output_dir = (argc >= 4) ? argv[3] : "exp_pipeline_out";
    fs::create_directories(output_dir);

    const int n_joints = robot.n_joints();
    const int n_act  = robot.n_active_links();
    const int n_frames = n_joints + (robot.has_tool() ? 1 : 0);

    std::cout << "=== Pipeline Benchmark (v3 — with GCPC) ===\n"
              << "Robot: " << robot.name() << "  (" << n_joints << " DOF, "
              << n_act << " active links)\n"
              << "Trials: " << N << "  Output: " << output_dir << "\n\n";

    // ── Build GCPC cache (one-time precomputation) ──────────────────
    std::cout << "Building GCPC cache..." << std::flush;
    Timer gcpc_build_timer;
    gcpc_build_timer.start();
    GcpcCache gcpc_cache = build_gcpc_cache_for_robot(robot);
    std::cout << " done (" << gcpc_cache.n_total_points() << " points, "
              << std::fixed << std::setprecision(1)
              << gcpc_build_timer.elapsed_ms() << " ms)\n\n" << std::flush;

    std::mt19937 rng(42);
    BoxGenerator gen{robot, rng};

    // ── Pre-generate trial boxes ────────────────────────────────────
    // P2-4: Random sampling from distribution (was deterministic i%5)
    // Distribution: 40% narrow (frac=0.05), 40% medium (frac=0.2), 20% wide (frac=0.6)
    std::vector<std::vector<Interval>> trial_boxes(N);
    {
        std::discrete_distribution<int> width_dist({40, 40, 20});
        const double fracs[] = {0.05, 0.20, 0.60};
        for (int i = 0; i < N; ++i)
            trial_boxes[i] = gen.generate(fracs[width_dist(rng)]);
    }

    // P2-5: Use all N trials for sweep (was min(N, 20))
    // Cap at 30 to balance thoroughness with AnalyticalCrit cost
    int n_sweep = std::min(N, 30);

    // ═════════════════════════════════════════════════════════════════════
    //  Pre-compute all source data for sweep trials
    // ═════════════════════════════════════════════════════════════════════

    // IFK + CritSample: pre-compute frames
    std::vector<std::vector<FrameResult>> sweep_frames(2);
    for (int si = 0; si < 2; ++si) {
        AABBSource src = static_cast<AABBSource>(si);
        sweep_frames[si].resize(n_sweep);
        Timer st; st.start();
        for (int t = 0; t < n_sweep; ++t)
            sweep_frames[si][t] = compute_frames(src, robot, trial_boxes[t]);
        std::cout << "  Pre-computed " << aabb_source_names[si]
                  << " frames (" << n_sweep << " trials, "
                  << std::fixed << std::setprecision(1)
                  << st.elapsed_ms() << " ms)\n" << std::flush;
    }

    // AnalyticalCrit: pre-compute sub-AABBs at MAX_EP_NSUB
    const int max_analytical_nsub = MAX_EP_NSUB;
    std::vector<AnalyticalResult> sweep_analytical(n_sweep);
    {
        Timer st; st.start();
        for (int t = 0; t < n_sweep; ++t)
            sweep_analytical[t] = compute_analytical_sub_aabbs(
                robot, trial_boxes[t], max_analytical_nsub);
        std::cout << "  Pre-computed AnalyticalCrit sub-AABBs (max_n_sub="
                  << max_analytical_nsub << ", " << n_sweep << " trials, "
                  << std::fixed << std::setprecision(1)
                  << st.elapsed_ms() << " ms)\n" << std::flush;
    }

    // GCPC: pre-compute sub-AABBs at max_n_sub=16
    std::vector<AnalyticalResult> sweep_gcpc(n_sweep);
    {
        Timer st; st.start();
        for (int t = 0; t < n_sweep; ++t)
            sweep_gcpc[t] = compute_gcpc_sub_aabbs(
                gcpc_cache, robot, trial_boxes[t], max_analytical_nsub);
        std::cout << "  Pre-computed GCPC sub-AABBs (max_n_sub="
                  << max_analytical_nsub << ", " << n_sweep << " trials, "
                  << std::fixed << std::setprecision(1)
                  << st.elapsed_ms() << " ms)\n" << std::flush;
    }

    // ── Compute adaptive workspace bounds from IFK frames ───────────
    std::vector<float> all_ifk_frames;
    for (int t = 0; t < n_sweep; ++t) {
        auto& f = sweep_frames[0][t].frames;
        all_ifk_frames.insert(all_ifk_frames.end(), f.begin(), f.end());
    }
    float world_bounds[6];
    compute_workspace_bounds(all_ifk_frames, n_frames, n_sweep, 0.15f, world_bounds);
    std::cout << "  Workspace bounds: ["
              << std::fixed << std::setprecision(2)
              << world_bounds[0] << ", " << world_bounds[1] << ", " << world_bounds[2]
              << "] -> [" << world_bounds[3] << ", " << world_bounds[4]
              << ", " << world_bounds[5] << "]\n\n" << std::flush;

    // ═════════════════════════════════════════════════════════════════════
    //  Phase 1: Hyperparameter Sweep  (scoring = tightest volume)
    // ═════════════════════════════════════════════════════════════════════
    std::cout << "Phase 1: Hyperparameter sweep (scoring = tightest volume)...\n"
              << std::flush;

    std::vector<std::vector<HyperConfig>> best_hp(N_SOURCES, std::vector<HyperConfig>(3));

    for (int si = 0; si < N_SOURCES; ++si) {
        AABBSource src = static_cast<AABBSource>(si);
        for (int ei = 0; ei < 3; ++ei) {
            EnvType etype = static_cast<EnvType>(ei);

            std::cout << "  Sweeping [" << aabb_source_names[si]
                      << " + " << env_type_names[ei] << "] ... " << std::flush;

            double best_vol = 1e30;
            HyperConfig best;

            // Hull16_Grid: now supports SubHull16 with n_sub sub-hulls per link
            auto& n_sub_vals = hp_n_sub_values;
            auto& delta_vals = (etype == EnvType::Hull16_Grid)
                               ? hp_delta_values : std::vector<double>{0.02};
            auto& grid_R_vals = (etype == EnvType::SubAABB_Grid)
                                ? hp_grid_R_values : std::vector<int>{32};

            for (int nsub : n_sub_vals) {
                if (MAX_EP_NSUB % nsub != 0) continue;

                for (double delta : delta_vals) {
                    for (int gR : grid_R_vals) {
                        double total_vol = 0;

                        for (int t = 0; t < n_sweep; ++t) {
                            // Unified: source → endpoint_aabbs → compute_envelope
                            const float* ep;
                            if (src == AABBSource::AnalyticalCrit)
                                ep = sweep_analytical[t].endpoint_aabbs.data();
                            else if (src == AABBSource::GCPC)
                                ep = sweep_gcpc[t].endpoint_aabbs.data();
                            else
                                ep = sweep_frames[si][t].endpoint_aabbs.data();

                            EnvResult er = compute_envelope(
                                etype, robot, ep, MAX_EP_NSUB, nsub,
                                delta, gR, world_bounds);
                            total_vol += er.volume;
                        }

                        double avg_vol = total_vol / n_sweep;
                        if (avg_vol < best_vol) {
                            best_vol = avg_vol;
                            best.n_sub = nsub;
                            best.delta = delta;
                            best.grid_R = gR;
                        }
                    }
                }
            }

            best_hp[si][ei] = best;
            std::cout << "n_sub=" << best.n_sub;
            if (etype == EnvType::Hull16_Grid)
                std::cout << " delta=" << best.delta;
            if (etype == EnvType::SubAABB_Grid)
                std::cout << " grid_R=" << best.grid_R;
            std::cout << " (vol=" << std::scientific << std::setprecision(3)
                      << best_vol << ")\n" << std::flush;
        }
    }

    // Save best config
    json jcfg;
    for (int si = 0; si < N_SOURCES; ++si)
        for (int ei = 0; ei < 3; ++ei) {
            std::string key = std::string(aabb_source_names[si]) + "+"
                            + env_type_names[ei];
            jcfg[key] = {
                {"n_sub",  best_hp[si][ei].n_sub},
                {"delta",  best_hp[si][ei].delta},
                {"grid_R", best_hp[si][ei].grid_R}
            };
        }
    {
        std::ofstream f(output_dir + "/best_config.json");
        f << jcfg.dump(2) << "\n";
    }
    std::cout << "\n  Saved: " << output_dir << "/best_config.json\n\n" << std::flush;

    // ═════════════════════════════════════════════════════════════════════
    //  Phase 2: Full Benchmark with Best Hyperparams
    // ═════════════════════════════════════════════════════════════════════
    std::cout << "Phase 2: Full benchmark (" << N << " trials)...\n" << std::flush;

    // IFK + CritSample: compute frames + endpoint_aabbs
    std::vector<std::vector<std::vector<float>>> frames_cache(2);
    std::vector<std::vector<std::vector<float>>> ep_cache(2);  // endpoint_aabbs
    std::vector<std::vector<FKState>> fk_cache(2);
    std::vector<std::vector<double>> frame_times(2);

    for (int si = 0; si < 2; ++si) {
        AABBSource src = static_cast<AABBSource>(si);
        frames_cache[si].resize(N);
        ep_cache[si].resize(N);
        fk_cache[si].resize(N);
        frame_times[si].resize(N);

        Timer pt; pt.start();
        for (int t = 0; t < N; ++t) {
            auto fr = compute_frames(src, robot, trial_boxes[t]);
            frames_cache[si][t] = std::move(fr.frames);
            ep_cache[si][t] = std::move(fr.endpoint_aabbs);
            fk_cache[si][t] = fr.fk;
            frame_times[si][t] = fr.compute_ms;
        }
        std::cout << "  Cached " << aabb_source_names[si]
                  << " frames (" << N << " trials, "
                  << std::fixed << std::setprecision(1)
                  << pt.elapsed_ms() << " ms)\n" << std::flush;
    }

    // AnalyticalCrit sub-AABBs
    std::vector<AnalyticalResult> analytical_cache(N);
    {
        Timer pt; pt.start();
        for (int t = 0; t < N; ++t)
            analytical_cache[t] = compute_analytical_sub_aabbs(
                robot, trial_boxes[t], max_analytical_nsub);
        std::cout << "  Cached AnalyticalCrit sub-AABBs (" << N << " trials, "
                  << std::fixed << std::setprecision(1)
                  << pt.elapsed_ms() << " ms)\n" << std::flush;
    }

    // GCPC sub-AABBs
    std::vector<AnalyticalResult> gcpc_data_cache(N);
    {
        Timer pt; pt.start();
        for (int t = 0; t < N; ++t)
            gcpc_data_cache[t] = compute_gcpc_sub_aabbs(
                gcpc_cache, robot, trial_boxes[t], max_analytical_nsub);
        std::cout << "  Cached GCPC sub-AABBs (" << N << " trials, "
                  << std::fixed << std::setprecision(1)
                  << pt.elapsed_ms() << " ms)\n" << std::flush;
    }

    // Recompute workspace bounds from all N trials
    std::vector<float> all_frames_N;
    for (int t = 0; t < N; ++t) {
        auto& f = frames_cache[0][t];
        all_frames_N.insert(all_frames_N.end(), f.begin(), f.end());
    }
    compute_workspace_bounds(all_frames_N, n_frames, N, 0.15f, world_bounds);
    std::cout << "  Workspace bounds (full): ["
              << std::fixed << std::setprecision(2)
              << world_bounds[0] << ", " << world_bounds[1] << ", " << world_bounds[2]
              << "] -> [" << world_bounds[3] << ", " << world_bounds[4]
              << ", " << world_bounds[5] << "]\n\n" << std::flush;

    // CSV output
    std::ofstream csv(output_dir + "/benchmark_results.csv");
    csv << "AABBSource,EnvType,Mode,n_sub,delta,grid_R,"
        << "avg_frame_ms,avg_env_ms,avg_total_ms,"
        << "avg_volume_m3,avg_dedup_volume_m3,std_volume_m3,std_total_ms,avg_voxels,avg_bricks\n";

    std::ofstream disk_csv(output_dir + "/warm_disk_size.csv");
    disk_csv << "AABBSource,EnvType,frames_bytes,aabb_bytes,grid_bytes,total_bytes_per_node\n";

    // Console header
    std::cout << std::left
              << std::setw(18) << "Source"
              << std::setw(15) << "EnvType"
              << std::setw(6)  << "Mode"
              << std::setw(8)  << "n_sub"
              << std::setw(10) << "Frame_ms"
              << std::setw(10) << "Env_ms"
              << std::setw(10) << "Total_ms"
              << std::setw(14) << "Volume(m3)"
              << std::setw(10) << "Voxels"
              << "\n"
              << std::string(101, '-') << "\n" << std::flush;

    // Run all 4×3×2 = 24 combinations
    for (int si = 0; si < N_SOURCES; ++si) {
        AABBSource src = static_cast<AABBSource>(si);

        for (int ei = 0; ei < 3; ++ei) {
            EnvType etype = static_cast<EnvType>(ei);
            auto& hp = best_hp[si][ei];

            // Hull16_Grid LECT warm cache: store serialised VoxelGrid per trial
            struct Hull16WarmEntry {
                double volume = 0;
                int n_voxels = 0, n_bricks = 0;
                std::vector<uint8_t> flat_buf;
            };
            std::vector<Hull16WarmEntry> hull16_warm(
                (etype == EnvType::Hull16_Grid) ? N : 0);

            for (int mi = 0; mi < 2; ++mi) {
                Mode mode = static_cast<Mode>(mi);

                // Hull16_Grid: VoxelGrid is cached per tree node;
                // show Cold (source computation) with env_ms=0, skip Warm
                if (mode == Mode::Warm && etype == EnvType::Hull16_Grid)
                    continue;

                double sum_frame_ms = 0, sum_env_ms = 0, sum_total_ms = 0;
                double sum_volume = 0, sum_volume_sq = 0;  // P2-8: track variance
                double sum_dedup = 0;   // P1-1: dedup volume
                double sum_total_sq = 0;
                long long sum_voxels = 0;
                long long sum_bricks = 0;

                for (int t = 0; t < N; ++t) {
                    double frame_ms;
                    EnvResult er;

                    if (mode == Mode::Warm && etype == EnvType::Hull16_Grid) {
                        // LECT warm: load cached VoxelGrid (memcpy flat buffer)
                        auto& entry = hull16_warm[t];
                        Timer tw; tw.start();
                        std::vector<uint8_t> buf(entry.flat_buf.size());
                        std::memcpy(buf.data(), entry.flat_buf.data(),
                                    entry.flat_buf.size());
                        frame_ms = tw.elapsed_ms();
                        er.volume = entry.volume;
                        er.n_voxels = entry.n_voxels;
                        er.n_bricks = entry.n_bricks;
                        er.compute_ms = 0;
                    } else if (src == AABBSource::AnalyticalCrit || src == AABBSource::GCPC) {
                        // ── AnalyticalCrit / GCPC ──
                        auto& data_cache = (src == AABBSource::GCPC)
                            ? gcpc_data_cache : analytical_cache;
                        if (mode == Mode::Cold) {
                            frame_ms = data_cache[t].compute_ms;
                        } else {
                            // Warm: memcpy of endpoint-AABB data
                            Timer tw; tw.start();
                            int sz = n_act * (MAX_EP_NSUB + 1) * 6;
                            std::vector<float> warm_buf(sz);
                            std::memcpy(warm_buf.data(),
                                data_cache[t].endpoint_aabbs.data(),
                                sz * sizeof(float));
                            frame_ms = tw.elapsed_ms();
                        }

                        er = compute_envelope(
                            etype, robot,
                            data_cache[t].endpoint_aabbs.data(),
                            MAX_EP_NSUB, hp.n_sub,
                            hp.delta, hp.grid_R, world_bounds);
                    } else {
                        // ── IFK / CritSample ──
                        if (mode == Mode::Cold) {
                            frame_ms = frame_times[si][t];
                        } else {
                            // Warm: memcpy of endpoint-AABB data
                            Timer tw; tw.start();
                            int sz = n_act * (MAX_EP_NSUB + 1) * 6;
                            std::vector<float> warm_buf(sz);
                            std::memcpy(warm_buf.data(),
                                        ep_cache[si][t].data(),
                                        sz * sizeof(float));
                            frame_ms = tw.elapsed_ms();
                        }

                        er = compute_envelope(
                            etype, robot,
                            ep_cache[si][t].data(),
                            MAX_EP_NSUB, hp.n_sub,
                            hp.delta, hp.grid_R, world_bounds);
                    }

                    // Cache Hull16_Grid cold results for LECT warm mode
                    if (mode == Mode::Cold && etype == EnvType::Hull16_Grid) {
                        auto& entry = hull16_warm[t];
                        entry.volume = er.volume;
                        entry.n_voxels = er.n_voxels;
                        entry.n_bricks = er.n_bricks;
                        int sz = 4 + er.n_bricks * 76;
                        entry.flat_buf.assign(sz, 0xAA);
                    }

                    // Hull16_Grid: VoxelGrid cached per node → env_ms = 0
                    double env_t = (etype == EnvType::Hull16_Grid)
                                   ? 0.0 : er.compute_ms;
                    double total_t = frame_ms + env_t;
                    sum_frame_ms += frame_ms;
                    sum_env_ms   += env_t;
                    sum_total_ms += total_t;
                    sum_volume   += er.volume;
                    sum_volume_sq += er.volume * er.volume;  // P2-8
                    sum_dedup    += er.dedup_volume;          // P1-1
                    sum_total_sq  += total_t * total_t;       // P2-8
                    sum_voxels   += er.n_voxels;
                    sum_bricks   += er.n_bricks;
                }

                double af = sum_frame_ms / N;
                double ae = sum_env_ms / N;
                double at = sum_total_ms / N;
                double av = sum_volume / N;
                double ad = sum_dedup / N;  // P1-1: avg dedup volume
                int avox = static_cast<int>(sum_voxels / N);
                int abrk = static_cast<int>(sum_bricks / N);
                // P2-8: standard deviation
                double vol_var = (sum_volume_sq / N) - av * av;
                double vol_std = vol_var > 0 ? std::sqrt(vol_var) : 0.0;
                double time_var = (sum_total_sq / N) - at * at;
                double time_std = time_var > 0 ? std::sqrt(time_var) : 0.0;

                std::cout << std::left
                    << std::setw(18) << aabb_source_names[si]
                    << std::setw(15) << env_type_names[ei]
                    << std::setw(6)  << mode_names[mi]
                    << std::setw(8)  << hp.n_sub
                    << std::fixed << std::setprecision(3)
                    << std::setw(10) << af
                    << std::setw(10) << ae
                    << std::setw(10) << at
                    << std::scientific << std::setprecision(4)
                    << std::setw(14) << av
                    << std::fixed << std::setprecision(0)
                    << std::setw(10) << avox
                    << "\n" << std::flush;

                csv << aabb_source_names[si] << ","
                    << env_type_names[ei] << ","
                    << mode_names[mi] << ","
                    << hp.n_sub << ","
                    << hp.delta << ","
                    << hp.grid_R << ","
                    << std::fixed << std::setprecision(4)
                    << af << "," << ae << "," << at << ","
                    << std::scientific << std::setprecision(6)
                    << av << "," << ad << "," << vol_std << ","
                    << std::fixed << std::setprecision(4) << time_std << ","
                    << avox << "," << abrk << "\n";

                if (mode == Mode::Warm ||
                    (mode == Mode::Cold && etype == EnvType::Hull16_Grid)) {
                    auto ds = estimate_disk_per_node(
                        src, etype, n_frames, n_act,
                        hp.n_sub, hp.grid_R, abrk);

                    disk_csv << aabb_source_names[si] << ","
                             << env_type_names[ei] << ","
                             << ds.frames_bytes << ","
                             << ds.aabb_bytes << ","
                             << ds.grid_bytes << ","
                             << ds.total_bytes << "\n";
                }
            }
        }
        std::cout << std::string(101, '-') << "\n";
    }

    std::cout << "\nResults saved to:\n"
              << "  " << output_dir << "/best_config.json\n"
              << "  " << output_dir << "/benchmark_results.csv\n"
              << "  " << output_dir << "/warm_disk_size.csv\n";

    return 0;
}

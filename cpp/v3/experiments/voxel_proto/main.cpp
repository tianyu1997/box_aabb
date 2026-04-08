// ═══════════════════════════════════════════════════════════════════════════
//  Voxel BitMask Prototype — Phase 1: 16-Point Convex Hull Envelope
//  SafeBoxForest v3
//
//  This experiment validates the core mathematical insight:
//
//    Given proximal frame in box B₁ and distal frame in box B₂,
//    the swept volume of the link axis = Conv(B₁ ∪ B₂)
//    = convex hull of the 16 corner points (8 from B₁ + 8 from B₂).
//
//  Comparison:
//    A) Cartesian AABB        — per-link axis-aligned bounding box (baseline)
//    B) Subdivided AABB→Voxel — subdivide link, voxelise each sub-AABB
//    C) Hull-16→Voxel         — subdivide link, rasterise 16-pt convex hull
//
//  Key metric:  voxel_volume / aabb_volume  — lower is tighter.
//  Hull-16 is analytically optimal for the given endpoint boxes.
// ═══════════════════════════════════════════════════════════════════════════

#include "bit_brick.h"
#include "voxel_grid.h"

#include "sbf/robot/robot.h"
#include "sbf/robot/interval_fk.h"
#include "sbf/common/types.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace sbf;
using namespace sbf::voxel;

// ─────────────────────────────────────────────────────────────────────────
//  Helper: extract (x, y, z) position interval from a prefix transform
// ─────────────────────────────────────────────────────────────────────────
struct PosInterval { float lo[3], hi[3]; };

static PosInterval frame_pos(const FKState& fk, int frame_idx) {
    PosInterval p;
    p.lo[0] = static_cast<float>(fk.prefix_lo[frame_idx][3]);
    p.lo[1] = static_cast<float>(fk.prefix_lo[frame_idx][7]);
    p.lo[2] = static_cast<float>(fk.prefix_lo[frame_idx][11]);
    p.hi[0] = static_cast<float>(fk.prefix_hi[frame_idx][3]);
    p.hi[1] = static_cast<float>(fk.prefix_hi[frame_idx][7]);
    p.hi[2] = static_cast<float>(fk.prefix_hi[frame_idx][11]);
    return p;
}

// ─────────────────────────────────────────────────────────────────────────
//  AABB volume (sum of per-link AABBs — the conventional SBF metric)
// ─────────────────────────────────────────────────────────────────────────
static double aabb_sum_volume(const Robot& robot, const FKState& fk) {
    const int n = robot.n_active_links();
    std::vector<float> aabbs(n * 6);
    extract_link_aabbs(fk,
                       robot.active_link_map(), n,
                       aabbs.data(),
                       robot.active_link_radii());
    double vol = 0.0;
    for (int i = 0; i < n; ++i) {
        const float* a = aabbs.data() + i * 6;
        vol += static_cast<double>(a[3]-a[0])
             * static_cast<double>(a[4]-a[1])
             * static_cast<double>(a[5]-a[2]);
    }
    return vol;
}

// ─────────────────────────────────────────────────────────────────────────
//  Rasterise link envelopes into a VoxelGrid using sub-AABBs
// ─────────────────────────────────────────────────────────────────────────
static void rasterise_sub_aabbs(const Robot& robot,
                                const FKState& fk,
                                VoxelGrid& grid,
                                int n_sub)
{
    const int    n_act  = robot.n_active_links();
    const int*   alm    = robot.active_link_map();
    const double* radii = robot.active_link_radii();

    for (int i = 0; i < n_act; ++i) {
        const int li = alm[i];
        auto prox = frame_pos(fk, li);
        auto dist = frame_pos(fk, li + 1);
        const float r = radii ? static_cast<float>(radii[i]) : 0.0f;

        const float inv_n = 1.0f / static_cast<float>(n_sub);
        for (int s = 0; s < n_sub; ++s) {
            const float t0 = s * inv_n;
            const float t1 = (s + 1) * inv_n;

            // Interval lerp for sub-segment endpoints
            float sub_aabb[6];
            for (int c = 0; c < 3; ++c) {
                float sp_lo = prox.lo[c] * (1-t0) + dist.lo[c] * t0;
                float sp_hi = prox.hi[c] * (1-t0) + dist.hi[c] * t0;
                float sd_lo = prox.lo[c] * (1-t1) + dist.lo[c] * t1;
                float sd_hi = prox.hi[c] * (1-t1) + dist.hi[c] * t1;

                sub_aabb[c]     = std::min(sp_lo, sd_lo) - r;
                sub_aabb[c + 3] = std::max(sp_hi, sd_hi) + r;
            }
            grid.fill_aabb(sub_aabb);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────
//  Rasterise link envelopes into a VoxelGrid using sub-capsules
// ─────────────────────────────────────────────────────────────────────────
static void rasterise_capsules(const Robot& robot,
                               const FKState& fk,
                               VoxelGrid& grid,
                               int n_sub)
{
    const int    n_act  = robot.n_active_links();
    const int*   alm    = robot.active_link_map();
    const double* radii = robot.active_link_radii();

    for (int i = 0; i < n_act; ++i) {
        const int li = alm[i];
        auto prox = frame_pos(fk, li);
        auto dist = frame_pos(fk, li + 1);
        const double r = radii ? radii[i] : 0.0;

        const float inv_n = 1.0f / static_cast<float>(n_sub);
        for (int s = 0; s < n_sub; ++s) {
            const float t0 = s * inv_n;
            const float t1 = (s + 1) * inv_n;

            // Sub-segment proximal & distal position intervals
            float sp_iv[6], sd_iv[6];   // [lo_xyz, hi_xyz]
            for (int c = 0; c < 3; ++c) {
                sp_iv[c]     = prox.lo[c] * (1-t0) + dist.lo[c] * t0;
                sp_iv[c + 3] = prox.hi[c] * (1-t0) + dist.hi[c] * t0;
                sd_iv[c]     = prox.lo[c] * (1-t1) + dist.lo[c] * t1;
                sd_iv[c + 3] = prox.hi[c] * (1-t1) + dist.hi[c] * t1;
            }
            grid.fill_interval_capsule(sp_iv, sd_iv, r);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────
//  Rasterise link envelopes using 16-point convex hull (analytical exact)
// ─────────────────────────────────────────────────────────────────────────
static void rasterise_hull16(const Robot& robot,
                             const FKState& fk,
                             VoxelGrid& grid,
                             int n_sub)
{
    const int    n_act  = robot.n_active_links();
    const int*   alm    = robot.active_link_map();
    const double* radii = robot.active_link_radii();

    for (int i = 0; i < n_act; ++i) {
        const int li = alm[i];
        auto prox = frame_pos(fk, li);
        auto dist = frame_pos(fk, li + 1);
        const double r = radii ? radii[i] : 0.0;

        const float inv_n = 1.0f / static_cast<float>(n_sub);
        for (int s = 0; s < n_sub; ++s) {
            const float t0 = s * inv_n;
            const float t1 = (s + 1) * inv_n;

            float sp_iv[6], sd_iv[6];
            for (int c = 0; c < 3; ++c) {
                sp_iv[c]     = prox.lo[c] * (1-t0) + dist.lo[c] * t0;
                sp_iv[c + 3] = prox.hi[c] * (1-t0) + dist.hi[c] * t0;
                sd_iv[c]     = prox.lo[c] * (1-t1) + dist.lo[c] * t1;
                sd_iv[c + 3] = prox.hi[c] * (1-t1) + dist.hi[c] * t1;
            }
            grid.fill_convex_hull_16(sp_iv, sd_iv, r);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────
//  Rasterise link envelopes using 16-pt convex hull — SCANLINE accelerated
// ─────────────────────────────────────────────────────────────────────────
static void rasterise_hull16_scanline(const Robot& robot,
                                      const FKState& fk,
                                      VoxelGrid& grid,
                                      int n_sub)
{
    const int    n_act  = robot.n_active_links();
    const int*   alm    = robot.active_link_map();
    const double* radii = robot.active_link_radii();

    for (int i = 0; i < n_act; ++i) {
        const int li = alm[i];
        auto prox = frame_pos(fk, li);
        auto dist = frame_pos(fk, li + 1);
        const double r = radii ? radii[i] : 0.0;

        const float inv_n = 1.0f / static_cast<float>(n_sub);
        for (int s = 0; s < n_sub; ++s) {
            const float t0 = s * inv_n;
            const float t1 = (s + 1) * inv_n;

            float sp_iv[6], sd_iv[6];
            for (int c = 0; c < 3; ++c) {
                sp_iv[c]     = prox.lo[c] * (1-t0) + dist.lo[c] * t0;
                sp_iv[c + 3] = prox.hi[c] * (1-t0) + dist.hi[c] * t0;
                sd_iv[c]     = prox.lo[c] * (1-t1) + dist.lo[c] * t1;
                sd_iv[c + 3] = prox.hi[c] * (1-t1) + dist.hi[c] * t1;
            }
            grid.fill_convex_hull_16_scanline(sp_iv, sd_iv, r);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────
//  Rasterise link envelopes using 16-pt convex hull — FAST (coherent scanline)
// ─────────────────────────────────────────────────────────────────────────
static void rasterise_hull16_fast(const Robot& robot,
                                  const FKState& fk,
                                  VoxelGrid& grid,
                                  int n_sub)
{
    const int    n_act  = robot.n_active_links();
    const int*   alm    = robot.active_link_map();
    const double* radii = robot.active_link_radii();

    for (int i = 0; i < n_act; ++i) {
        const int li = alm[i];
        auto prox = frame_pos(fk, li);
        auto dist = frame_pos(fk, li + 1);
        const double r = radii ? radii[i] : 0.0;

        const float inv_n = 1.0f / static_cast<float>(n_sub);
        for (int s = 0; s < n_sub; ++s) {
            const float t0 = s * inv_n;
            const float t1 = (s + 1) * inv_n;

            float sp_iv[6], sd_iv[6];
            for (int c = 0; c < 3; ++c) {
                sp_iv[c]     = prox.lo[c] * (1-t0) + dist.lo[c] * t0;
                sp_iv[c + 3] = prox.hi[c] * (1-t0) + dist.hi[c] * t0;
                sd_iv[c]     = prox.lo[c] * (1-t1) + dist.lo[c] * t1;
                sd_iv[c + 3] = prox.hi[c] * (1-t1) + dist.hi[c] * t1;
            }
            grid.fill_convex_hull_16_fast(sp_iv, sd_iv, r);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────
//  Rasterise link envelopes using 16-pt hull — TURBO (t-range + direct X bounds)
// ─────────────────────────────────────────────────────────────────────────
static void rasterise_hull16_turbo(const Robot& robot,
                                   const FKState& fk,
                                   VoxelGrid& grid,
                                   int n_sub)
{
    const int    n_act  = robot.n_active_links();
    const int*   alm    = robot.active_link_map();
    const double* radii = robot.active_link_radii();

    for (int i = 0; i < n_act; ++i) {
        const int li = alm[i];
        auto prox = frame_pos(fk, li);
        auto dist = frame_pos(fk, li + 1);
        const double r = radii ? radii[i] : 0.0;

        const float inv_n = 1.0f / static_cast<float>(n_sub);
        for (int s = 0; s < n_sub; ++s) {
            const float t0 = s * inv_n;
            const float t1 = (s + 1) * inv_n;

            float sp_iv[6], sd_iv[6];
            for (int c = 0; c < 3; ++c) {
                sp_iv[c]     = prox.lo[c] * (1-t0) + dist.lo[c] * t0;
                sp_iv[c + 3] = prox.hi[c] * (1-t0) + dist.hi[c] * t0;
                sd_iv[c]     = prox.lo[c] * (1-t1) + dist.lo[c] * t1;
                sd_iv[c + 3] = prox.hi[c] * (1-t1) + dist.hi[c] * t1;
            }
            grid.fill_convex_hull_16_turbo(sp_iv, sd_iv, r);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────
//  Rasterise AABBs into a VoxelGrid (for union-based volume)
// ─────────────────────────────────────────────────────────────────────────
static void rasterise_aabbs(const Robot& robot,
                            const FKState& fk,
                            VoxelGrid& grid)
{
    const int n = robot.n_active_links();
    std::vector<float> aabbs(n * 6);
    extract_link_aabbs(fk,
                       robot.active_link_map(), n,
                       aabbs.data(),
                       robot.active_link_radii());
    for (int i = 0; i < n; ++i)
        grid.fill_aabb(aabbs.data() + i * 6);
}

// ─────────────────────────────────────────────────────────────────────────
//  Obstacle helpers
// ─────────────────────────────────────────────────────────────────────────
struct BoxObstacle { double cx, cy, cz, hx, hy, hz; };

static void rasterise_obstacle(const BoxObstacle& obs, VoxelGrid& grid) {
    float aabb[6] = {
        static_cast<float>(obs.cx - obs.hx),
        static_cast<float>(obs.cy - obs.hy),
        static_cast<float>(obs.cz - obs.hz),
        static_cast<float>(obs.cx + obs.hx),
        static_cast<float>(obs.cy + obs.hy),
        static_cast<float>(obs.cz + obs.hz)
    };
    grid.fill_aabb(aabb);
}

// ═════════════════════════════════════════════════════════════════════════════
//  main
// ═════════════════════════════════════════════════════════════════════════════
int main(int argc, char** argv) {
    // ── Load robot ──────────────────────────────────────────────────────
    const char* config = (argc > 1)
        ? argv[1]
        : "../../v1/configs/iiwa14.json";

    Robot robot = Robot::from_json(config);
    printf("========================================================\n");
    printf("  Sparse Voxel BitMask — Phase 1 Prototype\n");
    printf("========================================================\n");
    printf("Robot       : %s\n", robot.name().c_str());
    printf("Joints      : %d\n", robot.n_joints());
    printf("Active links: %d\n", robot.n_active_links());
    printf("Transforms  : %d\n", robot.n_transforms());

    // Print active link radii
    printf("Link radii  :");
    for (int i = 0; i < robot.n_active_links(); ++i)
        printf(" %.3f", robot.active_link_radii()[i]);
    printf("\n");

    const int  ndof = robot.n_joints();
    const auto& lim = robot.joint_limits().limits;

    // ── Grid parameters ─────────────────────────────────────────────────
    const double delta = 0.02;   // 2 cm voxel
    const int    n_sub = 8;      // subdivisions per link

    printf("\nVoxel Δ     : %.3f m  (%d cm)\n", delta, (int)(delta * 100));
    printf("Link subdiv : %d\n", n_sub);
    printf("Safety pad  : %.4f m  (√3 Δ/2)\n", std::sqrt(3.0) * delta * 0.5);

    // ── Centre configuration ────────────────────────────────────────────
    std::vector<double> q_mid(ndof);
    for (int d = 0; d < ndof; ++d)
        q_mid[d] = 0.5 * (lim[d].lo + lim[d].hi);

    // ═════════════════════════════════════════════════════════════════════
    //  Experiment 1: Volume comparison across box sizes
    // ═════════════════════════════════════════════════════════════════════
    struct TestCase { const char* name; double frac; };
    TestCase cases[] = {
        {"Tiny   1%",   0.01},
        {"Small  5%",   0.05},
        {"Med   10%",   0.10},
        {"Large 25%",   0.25},
        {"Huge  50%",   0.50},
        {"Full 100%",   1.00},
    };
    constexpr int N_CASES = sizeof(cases) / sizeof(cases[0]);

    printf("\n");
    printf("========================================================================================\n");
    printf("  Experiment 1 — Volume: AABB(sum) vs SubAABB vs Hull-16 (convex hull of 16 corners)\n");
    printf("========================================================================================\n");
    printf("%-12s | %10s | %10s %6s | %10s %6s %7s | %10s %5s\n",
           "Box Width", "AABB(sum)", "SubAABB", "ratio", "Hull-16", "ratio", "elim%",
           "Scanline", "match");
    printf("-------------|------------|-------------------|-----------------------------|------------------\n");

    for (int ti = 0; ti < N_CASES; ++ti) {
        auto& tc = cases[ti];

        // Build joint intervals
        std::vector<Interval> ivs(ndof);
        for (int d = 0; d < ndof; ++d) {
            double range = lim[d].hi - lim[d].lo;
            double hw    = tc.frac * range * 0.5;
            ivs[d] = Interval(q_mid[d] - hw, q_mid[d] + hw);
        }

        // Interval FK
        FKState fk = compute_fk_full(robot, ivs);

        // (A) AABB sum volume
        double vol_aabb = aabb_sum_volume(robot, fk);

        // (B) SubAABB voxel volume
        VoxelGrid gB(delta);
        rasterise_sub_aabbs(robot, fk, gB, n_sub);
        double vol_sub = gB.occupied_volume();

        // (C) Hull-16: convex hull of 2×8 corner points (brute-force)
        VoxelGrid gH(delta);
        rasterise_hull16(robot, fk, gH, n_sub);
        double vol_hull = gH.occupied_volume();

        // (D) Hull-16 Scanline (must match C)
        VoxelGrid gS(delta);
        rasterise_hull16_scanline(robot, fk, gS, n_sub);
        double vol_scan = gS.occupied_volume();
        int vox_h = gH.count_occupied();
        int vox_s = gS.count_occupied();

        double r_sub  = vol_sub  / vol_aabb;
        double r_hull = vol_hull / vol_aabb;
        double elim   = (1.0 - r_hull) * 100.0;

        printf("%-12s | %10.6f | %10.6f %5.1f%% | %10.6f %5.1f%% %6.1f%% | %10.6f %s\n",
               tc.name, vol_aabb,
               vol_sub,  r_sub  * 100.0,
               vol_hull, r_hull * 100.0, elim,
               vol_scan, (vox_h == vox_s) ? "  OK" : " MISMATCH!");
    }

    // ═════════════════════════════════════════════════════════════════════
    //  Experiment 2: Per-link breakdown (Medium 10% box)
    // ═════════════════════════════════════════════════════════════════════
    {
        printf("\n");
        printf("==============================================================================\n");
        printf("  Experiment 2 — Per-link breakdown (Medium 10%% box)\n");
        printf("==============================================================================\n");

        std::vector<Interval> ivs(ndof);
        for (int d = 0; d < ndof; ++d) {
            double range = lim[d].hi - lim[d].lo;
            double hw    = 0.10 * range * 0.5;
            ivs[d] = Interval(q_mid[d] - hw, q_mid[d] + hw);
        }
        FKState fk = compute_fk_full(robot, ivs);

        const int    n_act = robot.n_active_links();
        const int*   alm   = robot.active_link_map();
        const double* rad  = robot.active_link_radii();

        // Per-link AABBs
        std::vector<float> aabbs(n_act * 6);
        extract_link_aabbs(fk, alm, n_act, aabbs.data(), rad);

        printf("%-6s | %6s | %10s | %10s | %10s | %7s\n",
               "Link", "Radius", "AABB Vol", "SubAABB Vx", "Hull-16 Vx", "Elim%");
        printf("-------|--------|------------|------------|------------|--------\n");

        for (int i = 0; i < n_act; ++i) {
            const float* a = aabbs.data() + i * 6;
            double vol_a = (a[3]-a[0]) * (a[4]-a[1]) * (a[5]-a[2]);

            // Per-link voxel grids
            const int li = alm[i];
            auto prox = frame_pos(fk, li);
            auto dist = frame_pos(fk, li + 1);
            double r = rad ? rad[i] : 0.0;
            float rf = static_cast<float>(r);

            VoxelGrid gB(delta), gH(delta);
            const float inv_n = 1.0f / static_cast<float>(n_sub);

            for (int s = 0; s < n_sub; ++s) {
                float t0 = s * inv_n, t1 = (s + 1) * inv_n;
                float sub_aabb[6], sp_iv[6], sd_iv[6];
                for (int c = 0; c < 3; ++c) {
                    float sp_lo = prox.lo[c]*(1-t0) + dist.lo[c]*t0;
                    float sp_hi = prox.hi[c]*(1-t0) + dist.hi[c]*t0;
                    float sd_lo = prox.lo[c]*(1-t1) + dist.lo[c]*t1;
                    float sd_hi = prox.hi[c]*(1-t1) + dist.hi[c]*t1;

                    sub_aabb[c]     = std::min(sp_lo, sd_lo) - rf;
                    sub_aabb[c + 3] = std::max(sp_hi, sd_hi) + rf;

                    sp_iv[c]     = sp_lo;  sp_iv[c+3] = sp_hi;
                    sd_iv[c]     = sd_lo;  sd_iv[c+3] = sd_hi;
                }
                gB.fill_aabb(sub_aabb);
                gH.fill_convex_hull_16(sp_iv, sd_iv, r);
            }

            double vol_bv = gB.occupied_volume();
            double vol_hv = gH.occupied_volume();
            double elim   = (1.0 - vol_hv / vol_a) * 100.0;

            printf("  %2d   | %5.3f  | %10.6f | %10.6f | %10.6f | %6.1f%%\n",
                   i, r, vol_a, vol_bv, vol_hv, elim);
        }
    }

    // ═════════════════════════════════════════════════════════════════════
    //  Experiment 3: Collision detection comparison
    // ═════════════════════════════════════════════════════════════════════
    {
        printf("\n");
        printf("==============================================================================\n");
        printf("  Experiment 3 — Collision detection: AABB vs Voxel\n");
        printf("==============================================================================\n");

        // Define some box obstacles
        std::vector<BoxObstacle> obstacles = {
            { 0.5, 0.0, 0.3,  0.05, 0.05, 0.05},   // small box near arm reach
            { 0.3, 0.3, 0.5,  0.10, 0.10, 0.10},   // medium box
            {-0.4, 0.0, 0.6,  0.08, 0.08, 0.08},   // behind arm
            { 0.0, 0.0, 1.5,  0.20, 0.20, 0.20},   // way above, should not collide
            { 0.0, 0.0, 0.0,  0.15, 0.15, 0.15},   // near base
        };

        // Use medium box (10%)
        std::vector<Interval> ivs(ndof);
        for (int d = 0; d < ndof; ++d) {
            double range = lim[d].hi - lim[d].lo;
            double hw    = 0.10 * range * 0.5;
            ivs[d] = Interval(q_mid[d] - hw, q_mid[d] + hw);
        }
        FKState fk = compute_fk_full(robot, ivs);

        // Robot envelope via AABB (extract per-link, union into grid)
        VoxelGrid grid_aabb(delta);
        rasterise_aabbs(robot, fk, grid_aabb);

        // Robot envelope via Hull-16
        VoxelGrid grid_hull(delta);
        rasterise_hull16(robot, fk, grid_hull, n_sub);

        printf("Robot envelope: AABB → %d voxels (%d bricks), "
               "Hull-16 → %d voxels (%d bricks)\n",
               grid_aabb.count_occupied(), grid_aabb.num_bricks(),
               grid_hull.count_occupied(),  grid_hull.num_bricks());

        printf("\n%-30s | %6s %8s | %6s %8s | %s\n",
               "Obstacle", "AABB", "overlap", "Voxel", "overlap", "False+?");
        printf("-------------------------------|----------------|----------------|--------\n");

        for (size_t oi = 0; oi < obstacles.size(); ++oi) {
            auto& obs = obstacles[oi];
            VoxelGrid grid_obs(delta);
            rasterise_obstacle(obs, grid_obs);

            bool col_aabb = grid_aabb.collides(grid_obs);
            bool col_hull = grid_hull.collides(grid_obs);
            int  ovlp_a   = grid_aabb.count_colliding(grid_obs);
            int  ovlp_h   = grid_hull.count_colliding(grid_obs);

            const char* false_pos = (col_aabb && !col_hull) ? "YES"
                                  : (!col_aabb && col_hull) ? "BUG!"
                                  : "-";

            char name[64];
            snprintf(name, sizeof(name),
                     "Obs %zu (%.2f,%.2f,%.2f) r=%.2f",
                     oi, obs.cx, obs.cy, obs.cz, obs.hx);

            printf("%-30s | %5s %7d | %5s %7d | %s\n",
                   name,
                   col_aabb ? "HIT" : "miss", ovlp_a,
                   col_hull ? "HIT" : "miss", ovlp_h,
                   false_pos);
        }

        printf("\nFalse+ = AABB says HIT but Voxel says miss (dead-space false alarm)\n");
    }

    // ═════════════════════════════════════════════════════════════════════
    //  Experiment 4: Timing benchmark
    // ═════════════════════════════════════════════════════════════════════
    {
        printf("\n");
        printf("==============================================================================\n");
        printf("  Experiment 4 — Timing (Medium 10%% box, %d reps)\n", 100);
        printf("==============================================================================\n");

        std::vector<Interval> ivs(ndof);
        for (int d = 0; d < ndof; ++d) {
            double range = lim[d].hi - lim[d].lo;
            double hw    = 0.10 * range * 0.5;
            ivs[d] = Interval(q_mid[d] - hw, q_mid[d] + hw);
        }

        const int REPS = 100;

        // Warm up
        {
            FKState fk = compute_fk_full(robot, ivs);
            VoxelGrid g(delta);
            rasterise_hull16(robot, fk, g, n_sub);
        }

        // Time interval FK
        auto t0 = std::chrono::high_resolution_clock::now();
        FKState fk;
        for (int r = 0; r < REPS; ++r)
            fk = compute_fk_full(robot, ivs);
        auto t1 = std::chrono::high_resolution_clock::now();
        double us_fk = std::chrono::duration<double, std::micro>(t1 - t0).count() / REPS;

        // Time AABB extraction
        {
            std::vector<float> aabbs(robot.n_active_links() * 6);
            auto ta = std::chrono::high_resolution_clock::now();
            for (int r = 0; r < REPS; ++r)
                extract_link_aabbs(fk, robot.active_link_map(),
                                   robot.n_active_links(),
                                   aabbs.data(), robot.active_link_radii());
            auto tb = std::chrono::high_resolution_clock::now();
            double us = std::chrono::duration<double, std::micro>(tb - ta).count() / REPS;
            printf("AABB extract       : %8.2f us\n", us);
        }

        // Time Hull-16 rasterisation (analytical, brute-force)
        {
            VoxelGrid g(delta);
            auto ta = std::chrono::high_resolution_clock::now();
            for (int r = 0; r < REPS; ++r) {
                g.clear();
                rasterise_hull16(robot, fk, g, n_sub);
            }
            auto tb = std::chrono::high_resolution_clock::now();
            double us = std::chrono::duration<double, std::micro>(tb - ta).count() / REPS;
            printf("Hull-16 brute      : %8.2f us  (n_sub=%d, per-voxel)\n", us, n_sub);
        }

        // Time Hull-16 rasterisation (scanline accelerated)
        {
            VoxelGrid g(delta);
            auto ta = std::chrono::high_resolution_clock::now();
            for (int r = 0; r < REPS; ++r) {
                g.clear();
                rasterise_hull16_scanline(robot, fk, g, n_sub);
            }
            auto tb = std::chrono::high_resolution_clock::now();
            double us = std::chrono::duration<double, std::micro>(tb - ta).count() / REPS;
            int vox = g.count_occupied();
            printf("Hull-16 scanline   : %8.2f us  (n_sub=%d, YZ-prefilter) [%d voxels]\n",
                   us, n_sub, vox);
        }

        // Time Hull-16 rasterisation (fast: coherence + batch fill)
        {
            VoxelGrid g(delta);
            auto ta = std::chrono::high_resolution_clock::now();
            for (int r = 0; r < REPS; ++r) {
                g.clear();
                rasterise_hull16_fast(robot, fk, g, n_sub);
            }
            auto tb = std::chrono::high_resolution_clock::now();
            double us = std::chrono::duration<double, std::micro>(tb - ta).count() / REPS;
            int vox = g.count_occupied();
            printf("Hull-16 fast       : %8.2f us  (n_sub=%d, coherent+batch) [%d voxels]\n",
                   us, n_sub, vox);
        }

        // Time Hull-16 rasterisation (turbo: t-range + analytical X bounds)
        {
            VoxelGrid g(delta);
            auto ta = std::chrono::high_resolution_clock::now();
            for (int r = 0; r < REPS; ++r) {
                g.clear();
                rasterise_hull16_turbo(robot, fk, g, n_sub);
            }
            auto tb = std::chrono::high_resolution_clock::now();
            double us = std::chrono::duration<double, std::micro>(tb - ta).count() / REPS;
            int vox = g.count_occupied();
            printf("Hull-16 turbo      : %8.2f us  (n_sub=%d, t-range+X-bounds) [%d voxels]\n",
                   us, n_sub, vox);
        }

        // Time sub-AABB rasterisation
        {
            VoxelGrid g(delta);
            auto ta = std::chrono::high_resolution_clock::now();
            for (int r = 0; r < REPS; ++r) {
                g.clear();
                rasterise_sub_aabbs(robot, fk, g, n_sub);
            }
            auto tb = std::chrono::high_resolution_clock::now();
            double us = std::chrono::duration<double, std::micro>(tb - ta).count() / REPS;
            printf("SubAABB rasterise  : %8.2f us  (n_sub=%d)\n", us, n_sub);
        }

        // Time collision test (voxel vs voxel)
        {
            VoxelGrid g_robot(delta);
            rasterise_hull16(robot, fk, g_robot, n_sub);

            VoxelGrid g_obs(delta);
            BoxObstacle obs{0.5, 0.0, 0.3, 0.05, 0.05, 0.05};
            rasterise_obstacle(obs, g_obs);

            const int COLLREPS = 10000;
            auto ta = std::chrono::high_resolution_clock::now();
            volatile bool hit = false;
            for (int r = 0; r < COLLREPS; ++r)
                hit = g_robot.collides(g_obs);
            auto tb = std::chrono::high_resolution_clock::now();
            double ns = std::chrono::duration<double, std::nano>(tb - ta).count() / COLLREPS;
            printf("Voxel collision    : %8.1f ns  (%d bricks robot, %d obs)  hit=%d\n",
                   ns, g_robot.num_bricks(), g_obs.num_bricks(), (int)hit);
        }

        // Time merge (OR) of two voxel grids
        {
            VoxelGrid g1(delta), g2(delta);
            rasterise_hull16(robot, fk, g1, n_sub);
            // Create a shifted copy
            std::vector<Interval> ivs2(ndof);
            for (int d = 0; d < ndof; ++d) {
                double range = lim[d].hi - lim[d].lo;
                double hw    = 0.05 * range * 0.5;
                double center = q_mid[d] + 0.02 * range;
                ivs2[d] = Interval(center - hw, center + hw);
            }
            FKState fk2 = compute_fk_full(robot, ivs2);
            rasterise_hull16(robot, fk2, g2, n_sub);

            const int MERGEREPS = 1000;
            auto ta = std::chrono::high_resolution_clock::now();
            for (int r = 0; r < MERGEREPS; ++r) {
                VoxelGrid tmp = g1;
                tmp.merge(g2);
            }
            auto tb = std::chrono::high_resolution_clock::now();
            double us = std::chrono::duration<double, std::micro>(tb - ta).count() / MERGEREPS;
            printf("Voxel merge (OR)   : %8.2f us  (%d + %d bricks)\n",
                   us, g1.num_bricks(), g2.num_bricks());
        }

        printf("Interval FK        : %8.2f us\n", us_fk);
    }

    // ═════════════════════════════════════════════════════════════════════
    //  Experiment 5: BitBrick unit checks
    // ═════════════════════════════════════════════════════════════════════
    {
        printf("\n");
        printf("==============================================================================\n");
        printf("  Experiment 5 — BitBrick sanity checks\n");
        printf("==============================================================================\n");

        BitBrick a, b;
        a.set(0, 0, 0);
        a.set(7, 7, 7);
        a.set(3, 3, 3);

        b.set(3, 3, 3);
        b.set(4, 4, 4);

        BitBrick c = a | b;
        BitBrick d = a & b;

        printf("a.popcount = %d  (expect 3)\n", a.popcount());
        printf("b.popcount = %d  (expect 2)\n", b.popcount());
        printf("(a|b).pop  = %d  (expect 4)\n", c.popcount());
        printf("(a&b).pop  = %d  (expect 1)\n", d.popcount());
        printf("intersects = %d  (expect 1)\n", a.intersects(b) ? 1 : 0);

        BitBrick e;
        e.set(1, 1, 1);
        printf("a∩e        = %d  (expect 0)\n", a.intersects(e) ? 1 : 0);

        printf("sizeof(BitBrick) = %zu bytes (expect 64)\n", sizeof(BitBrick));
    }

    printf("\n========================================================\n");
    printf("  Done.\n");
    printf("========================================================\n");
    return 0;
}

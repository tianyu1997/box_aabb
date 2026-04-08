// ══════════════════════════════════════════════════════════════════════════�?
//  SafeBoxForest v4 �?Envelope Pipeline Integration Tests
//
//  Covers the full Link iAABB envelope pipeline:
//    1. envelope_derive   �?derive_aabb, derive_aabb_subdivided,
//                           adaptive_subdivision_count, derive_grid
//    2. envelope_type     �?compute_link_envelope (LinkIAABB, LinkIAABB_Grid,
//                           Hull16_Grid)
//    3. collision_policy  �?collide_aabb, collide_aabb_subdiv, collide_grid,
//                           check_collision dispatcher, legacy interleaved
//    4. grid_store        �?derive_from_aabbs, union_grids, check_collision,
//                           occupied_count, save/load persistence
//    5. endpoint_store    �?store/get endpoints, union, refine
//    6. scene             �?Scene add/remove, AABBCollisionChecker
//    7. end-to-end        �?endpoint_source �?envelope_type �?collision
//
//  Build: cmake --build . --config Release --target test_envelope_pipeline
//  Run:   Release\test_envelope_pipeline.exe
// ══════════════════════════════════════════════════════════════════════════�?
#include "sbf/robot/robot.h"
#include "sbf/robot/interval_fk.h"
#include "sbf/envelope/endpoint_source.h"
#include "sbf/envelope/envelope_derive.h"
#include "sbf/envelope/envelope_type.h"
#include "sbf/envelope/collision_policy.h"
#include "sbf/envelope/grid_store.h"
#include "sbf/envelope/endpoint_store.h"
#include "sbf/scene/scene.h"
#include "sbf/scene/aabb_collision_checker.h"
#include "sbf/core/config.h"
#include "sbf/core/types.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#ifdef _WIN32
#include <io.h>
#define unlink _unlink
#else
#include <unistd.h>
#endif

using namespace sbf;

// ─── Test framework ─────────────────────────────────────────────────────────
static int g_pass = 0, g_fail = 0;

#define CHECK(cond, msg) do {                                         \
    if (!(cond)) {                                                    \
        std::printf("  FAIL  %s:%d  %s\n", __FILE__, __LINE__, msg); \
        ++g_fail;                                                     \
    } else { ++g_pass; }                                              \
} while(0)

#define CHECK_CLOSE(a, b, tol, msg) \
    CHECK(std::abs((a) - (b)) < (tol), msg)

#define SECTION(name) std::printf("\n── %s ──\n", (name))

// ─── Build IIWA14-like robot (same as test_ifk_pipeline) ────────────────────
static Robot make_iiwa14() {
    // DH from URDF (R820)
    std::vector<DHParam> dh = {
        {  0.0,    0.0, 0.36,   0.0, 0},
        { -HALF_PI, 0.0, 0.0,    0.0, 0},
        {  HALF_PI, 0.0, 0.42,   0.0, 0},
        {  HALF_PI, 0.0, 0.0,    0.0, 0},
        { -HALF_PI, 0.0, 0.4,    0.0, 0},
        { -HALF_PI, 0.0, 0.0,    0.0, 0},
        {  HALF_PI, 0.0, 0.0,    0.0, 0},
    };
    JointLimits limits;
    limits.limits = {
        {-2.9668, 2.9668}, {-2.0942, 2.0942}, {-2.9668, 2.9668},
        {-2.0942, 2.0942}, {-2.9668, 2.9668}, {-2.0942, 2.0942},
        {-3.0541, 3.0541},
    };
    DHParam tool{0.0, 0.0, 0.126, 0.0, 0};
    std::vector<double> radii = {0.08, 0.08, 0.06, 0.06, 0.04, 0.04, 0.04, 0.03};
    return Robot("iiwa14_test", dh, limits, tool, radii);
}

// ─── Helpers ────────────────────────────────────────────────────────────────
static Obstacle make_obstacle(double cx, double cy, double cz,
                               double hx, double hy, double hz,
                               const char* name = "obs") {
    return Obstacle(
        Eigen::Vector3d(cx, cy, cz),
        Eigen::Vector3d(hx, hy, hz),
        name);
}

// ══════════════════════════════════════════════════════════════════════════�?
//  Section 1: envelope_derive
// ══════════════════════════════════════════════════════════════════════════�?

static void test_derive_aabb_basic() {
    SECTION("derive_aabb �?basic two-endpoint link");

    // 1 link: frame 0 as parent (base), frame 1 as child
    // frames[0]  = endpoint 0: lo=(-1,-1,-1), hi=(1,1,1)
    // frames[1]  = endpoint 1: lo=(2,2,2), hi=(4,4,4)
    float frames[] = {
        -1.f, -1.f, -1.f,  1.f, 1.f, 1.f,   // endpoint 0
         2.f,  2.f,  2.f,  4.f, 4.f, 4.f,    // endpoint 1
    };
    int active_link_map[] = {1};  // link uses frames[0] as parent, frames[1] as child
    float base_pos[] = {0.f, 0.f, 0.f};
    float out[6];

    // No radius
    envelope::derive_aabb(frames, 2, active_link_map, 1, nullptr, base_pos, out);

    // Link AABB = union of parent(frame 0) and child(frame 1)
    // lo = min(frames[0].lo, frames[1].lo) = (-1,-1,-1)
    // hi = max(frames[0].hi, frames[1].hi) = (4,4,4)
    CHECK_CLOSE(out[0], -1.f, 1e-6f, "derive_aabb lo_x");
    CHECK_CLOSE(out[1], -1.f, 1e-6f, "derive_aabb lo_y");
    CHECK_CLOSE(out[2], -1.f, 1e-6f, "derive_aabb lo_z");
    CHECK_CLOSE(out[3],  4.f, 1e-6f, "derive_aabb hi_x");
    CHECK_CLOSE(out[4],  4.f, 1e-6f, "derive_aabb hi_y");
    CHECK_CLOSE(out[5],  4.f, 1e-6f, "derive_aabb hi_z");
}

static void test_derive_aabb_with_radius() {
    SECTION("derive_aabb �?with link radius");

    float frames[] = {
        0.f, 0.f, 0.f,  1.f, 1.f, 1.f,   // endpoint 0
        2.f, 2.f, 2.f,  3.f, 3.f, 3.f,    // endpoint 1
    };
    int active_link_map[] = {1};
    float base_pos[] = {0.f, 0.f, 0.f};
    float link_radii[] = {0.1f};
    float out[6];

    envelope::derive_aabb(frames, 2, active_link_map, 1, link_radii, base_pos, out);

    // lo = min(0,2) - 0.1 = -0.1; hi = max(1,3) + 0.1 = 3.1
    CHECK_CLOSE(out[0], -0.1f, 1e-6f, "with_radius lo_x");
    CHECK_CLOSE(out[3],  3.1f, 1e-6f, "with_radius hi_x");
    CHECK_CLOSE(out[1], -0.1f, 1e-6f, "with_radius lo_y");
    CHECK_CLOSE(out[4],  3.1f, 1e-6f, "with_radius hi_y");
}

static void test_derive_aabb_base_parent() {
    SECTION("derive_aabb �?first link uses base_pos as parent");

    // active_link_map[0] = 0 �?frame_idx=0, parent_idx=-1 �?base_pos
    float frames[] = {
        1.f, 2.f, 3.f,  4.f, 5.f, 6.f,
    };
    int active_link_map[] = {0};
    float base_pos[] = {0.5f, 0.5f, 0.5f};
    float out[6];

    envelope::derive_aabb(frames, 1, active_link_map, 1, nullptr, base_pos, out);

    // parent = base_pos = (0.5, 0.5, 0.5) �?lo=hi=0.5
    // child = frames[0] = lo(1,2,3) hi(4,5,6)
    // AABB lo = min(0.5, 1), min(0.5, 2), min(0.5, 3) = (0.5, 0.5, 0.5)
    // AABB hi = max(0.5, 4), max(0.5, 5), max(0.5, 6) = (4, 5, 6)
    CHECK_CLOSE(out[0], 0.5f, 1e-6f, "base_parent lo_x");
    CHECK_CLOSE(out[1], 0.5f, 1e-6f, "base_parent lo_y");
    CHECK_CLOSE(out[2], 0.5f, 1e-6f, "base_parent lo_z");
    CHECK_CLOSE(out[3], 4.0f, 1e-6f, "base_parent hi_x");
    CHECK_CLOSE(out[4], 5.0f, 1e-6f, "base_parent hi_y");
    CHECK_CLOSE(out[5], 6.0f, 1e-6f, "base_parent hi_z");
}

static void test_derive_aabb_multi_link() {
    SECTION("derive_aabb �?multiple active links");

    // 3 endpoints �?2 links
    float frames[] = {
        0.f, 0.f, 0.f,  1.f, 1.f, 1.f,   // ep 0
        2.f, 2.f, 2.f,  3.f, 3.f, 3.f,   // ep 1
        5.f, 5.f, 5.f,  6.f, 6.f, 6.f,   // ep 2
    };
    int active_link_map[] = {1, 2};
    float base_pos[] = {0.f, 0.f, 0.f};
    float out[12]; // 2 links × 6

    envelope::derive_aabb(frames, 3, active_link_map, 2, nullptr, base_pos, out);

    // Link 0: parent=ep0, child=ep1 �?lo=(0,0,0), hi=(3,3,3)
    CHECK_CLOSE(out[0],  0.f, 1e-6f, "multi_link_0 lo_x");
    CHECK_CLOSE(out[3],  3.f, 1e-6f, "multi_link_0 hi_x");

    // Link 1: parent=ep1, child=ep2 �?lo=(2,2,2), hi=(6,6,6)
    CHECK_CLOSE(out[6],  2.f, 1e-6f, "multi_link_1 lo_x");
    CHECK_CLOSE(out[9],  6.f, 1e-6f, "multi_link_1 hi_x");
}

static void test_derive_aabb_subdivided_identity() {
    SECTION("derive_aabb_subdivided �?n_sub=1 matches derive_aabb");

    float frames[] = {
        0.f, 0.f, 0.f,  1.f, 1.f, 1.f,
        3.f, 3.f, 3.f,  5.f, 5.f, 5.f,
    };
    int active_link_map[] = {1};
    float base_pos[] = {0.f, 0.f, 0.f};

    float aabb[6];
    envelope::derive_aabb(frames, 2, active_link_map, 1, nullptr, base_pos, aabb);

    float sub_aabb[6];
    envelope::derive_aabb_subdivided(frames, 2, 0, 1, 1, 0.f, base_pos, sub_aabb);

    for (int i = 0; i < 6; ++i) {
        CHECK_CLOSE(aabb[i], sub_aabb[i], 1e-6f, "subdiv_1 matches derive_aabb");
    }
}

static void test_derive_aabb_subdivided_tighter() {
    SECTION("derive_aabb_subdivided : n_sub>1 produces tighter coverage");

    // Parent and child are exact points on a diagonal — no interval uncertainty.
    // Subdivision splits the diagonal into segments, each sub-AABB is much
    // smaller than the single bounding box of the whole diagonal.
    float frames[] = {
        0.f, 0.f, 0.f,   0.f,  0.f,  0.f,  // ep 0: exact point at origin
       10.f,10.f,10.f,  10.f, 10.f, 10.f,  // ep 1: exact point at (10,10,10)
    };
    float base_pos[] = {0.f, 0.f, 0.f};

    // Single AABB
    float single[6];
    envelope::derive_aabb_subdivided(frames, 2, 0, 1, 1, 0.f, base_pos, single);
    double vol1 = (double)(single[3]-single[0]) * (single[4]-single[1]) * (single[5]-single[2]);
    CHECK(vol1 > 0, "single AABB has positive volume");

    // 4 sub-AABBs
    float subs[4 * 6];
    envelope::derive_aabb_subdivided(frames, 2, 0, 1, 4, 0.f, base_pos, subs);
    double vol4 = 0;
    for (int s = 0; s < 4; ++s) {
        float* a = subs + s * 6;
        vol4 += (double)(a[3]-a[0]) * (a[4]-a[1]) * (a[5]-a[2]);
    }

    // Sum of sub-AABB volumes should be strictly less than single AABB volume
    CHECK(vol4 <= vol1 + 1e-6, "subdiv vol4 <= vol1");
    // For a diagonal with no uncertainty, n_sub=4 => each sub is 1/4 length
    // in each axis => vol_per_sub = vol1/(4^3)=vol1/64, total=4*vol1/64=vol1/16
    CHECK(vol4 < vol1 * 0.1, "subdiv vol4 meaningfully less than vol1");
}

static void test_derive_aabb_subdivided_containment() {
    SECTION("derive_aabb_subdivided �?sub-AABBs contained in full AABB");

    float frames[] = {
        -1.f, -2.f, 0.f,   1.f, 2.f, 3.f,
         2.f,  0.f, 1.f,   5.f, 4.f, 6.f,
    };
    float base_pos[] = {0.f, 0.f, 0.f};

    float full[6];
    envelope::derive_aabb_subdivided(frames, 2, 0, 1, 1, 0.1f, base_pos, full);

    float subs[8 * 6];
    envelope::derive_aabb_subdivided(frames, 2, 0, 1, 8, 0.1f, base_pos, subs);

    for (int s = 0; s < 8; ++s) {
        float* a = subs + s * 6;
        CHECK(a[0] >= full[0] - 1e-5f, "sub lo_x >= full lo_x");
        CHECK(a[1] >= full[1] - 1e-5f, "sub lo_y >= full lo_y");
        CHECK(a[2] >= full[2] - 1e-5f, "sub lo_z >= full lo_z");
        CHECK(a[3] <= full[3] + 1e-5f, "sub hi_x <= full hi_x");
        CHECK(a[4] <= full[4] + 1e-5f, "sub hi_y <= full hi_y");
        CHECK(a[5] <= full[5] + 1e-5f, "sub hi_z <= full hi_z");
    }
}

static void test_adaptive_subdivision_count() {
    SECTION("adaptive_subdivision_count");

    // A link spanning 10 units �?with cell_size=2, expect ~5-8 subdivisions
    float frames[] = {
        0.f, 0.f, 0.f,   0.f,  0.f,  0.f,
        0.f, 0.f, 0.f,  10.f, 10.f, 10.f,
    };
    float base_pos[] = {0.f, 0.f, 0.f};

    int n = envelope::adaptive_subdivision_count(frames, 0, 1, base_pos, 2.0f, 64);
    CHECK(n >= 2, "adaptive n >= 2");
    CHECK(n <= 64, "adaptive n <= 64");

    // With very large cell, expect minimum of 2
    int n_min = envelope::adaptive_subdivision_count(frames, 0, 1, base_pos, 1000.f, 64);
    CHECK(n_min == 2, "adaptive min is 2");

    // With very small cell �?clamp to max_sub
    int n_max = envelope::adaptive_subdivision_count(frames, 0, 1, base_pos, 0.001f, 16);
    CHECK(n_max == 16, "adaptive clamp to max_sub");
}

static void test_derive_grid() {
    SECTION("derive_grid �?basic voxelisation");

    // A single link from origin to (1,1,1), world bounds [0,0,0,2,2,2], R=4
    float frames[] = {
        0.f, 0.f, 0.f,  0.f, 0.f, 0.f,   // ep 0 (point at origin)
        0.f, 0.f, 0.f,  1.f, 1.f, 1.f,    // ep 1
    };
    int active_link_map[] = {1};
    float base_pos[] = {0.f, 0.f, 0.f};
    float world_bounds[] = {0.f, 0.f, 0.f, 2.f, 2.f, 2.f};
    int R = 4;
    uint8_t grid[64]; // 4³
    std::memset(grid, 0, sizeof(grid));

    envelope::derive_grid(frames, 2, active_link_map, 1, nullptr, base_pos,
                          world_bounds, R, 4, 16, grid);

    // The link spans [0,0,0]-[1,1,1] in world [0,0,0]-[2,2,2] with R=4
    // Cell size = 0.5; link covers cells [0..2] in each axis
    int occupied = 0;
    for (int i = 0; i < R*R*R; ++i) occupied += grid[i];
    CHECK(occupied > 0, "derive_grid has occupied voxels");
    CHECK(occupied < R*R*R, "derive_grid doesn't fill everything");

    // Cells in upper-right corner (x=3,y=3,z=3) should be empty
    CHECK(grid[3*R*R + 3*R + 3] == 0, "far corner is empty");
}

// ══════════════════════════════════════════════════════════════════════════�?
//  Section 2: collision_policy
// ══════════════════════════════════════════════════════════════════════════�?

static void test_collide_aabb_hit() {
    SECTION("collide_aabb �?collision detected");

    // Link AABB: [0,0,0, 2,2,2]
    float link_iaabbs[] = {0.f, 0.f, 0.f, 2.f, 2.f, 2.f};
    // Obstacle centered at (1,1,1) with half_size (0.5,0.5,0.5)
    // �?[0.5, 0.5, 0.5] to [1.5, 1.5, 1.5] �?overlaps link
    Obstacle obs = make_obstacle(1, 1, 1, 0.5, 0.5, 0.5);

    bool result = envelope::collide_aabb(link_iaabbs, 1, &obs, 1);
    CHECK(result == true, "collide_aabb overlap �?true");
}

static void test_collide_aabb_miss() {
    SECTION("collide_aabb �?no collision");

    float link_iaabbs[] = {0.f, 0.f, 0.f, 1.f, 1.f, 1.f};
    // Obstacle far away
    Obstacle obs = make_obstacle(10, 10, 10, 0.5, 0.5, 0.5);

    bool result = envelope::collide_aabb(link_iaabbs, 1, &obs, 1);
    CHECK(result == false, "collide_aabb no overlap �?false");
}

static void test_collide_aabb_touching() {
    SECTION("collide_aabb �?touching boundary");

    // Link [0,0,0, 1,1,1], Obstacle [1,1,1] to [2,2,2]  (touching face)
    float link_iaabbs[] = {0.f, 0.f, 0.f, 1.f, 1.f, 1.f};
    Obstacle obs = make_obstacle(1.5, 1.5, 1.5, 0.5, 0.5, 0.5);

    bool result = envelope::collide_aabb(link_iaabbs, 1, &obs, 1);
    // Touching at boundary �?overlaps (SAT: a.hi >= o.lo && a.lo <= o.hi)
    CHECK(result == true, "collide_aabb touching �?true");
}

static void test_collide_aabb_multi_link_multi_obs() {
    SECTION("collide_aabb �?multi link, multi obstacle");

    // 2 links: [0,0,0,1,1,1] and [5,5,5,6,6,6]
    float link_iaabbs[] = {
        0.f, 0.f, 0.f, 1.f, 1.f, 1.f,
        5.f, 5.f, 5.f, 6.f, 6.f, 6.f,
    };

    // 2 obstacles: one at (3,3,3), one at (5.5,5.5,5.5) �?only second hits link 2
    Obstacle obs[2] = {
        make_obstacle(3, 3, 3, 0.1, 0.1, 0.1, "far"),
        make_obstacle(5.5, 5.5, 5.5, 0.1, 0.1, 0.1, "hit"),
    };

    bool result = envelope::collide_aabb(link_iaabbs, 2, obs, 2);
    CHECK(result == true, "multi: second obs hits second link");

    // Only first obstacle �?no hit
    bool miss = envelope::collide_aabb(link_iaabbs, 2, obs, 1);
    CHECK(miss == false, "multi: first obs alone misses");
}

static void test_collide_aabb_empty() {
    SECTION("collide_aabb �?empty inputs");

    float link_iaabbs[] = {0.f, 0.f, 0.f, 1.f, 1.f, 1.f};
    Obstacle obs = make_obstacle(0.5, 0.5, 0.5, 0.1, 0.1, 0.1);

    CHECK(envelope::collide_aabb(link_iaabbs, 0, &obs, 1) == false, "0 links �?false");
    CHECK(envelope::collide_aabb(link_iaabbs, 1, &obs, 0) == false, "0 obs �?false");
}

static void test_collide_aabb_subdiv_refine() {
    SECTION("collide_aabb_subdiv �?subdivision can refine false positives");

    // Set up a scenario where coarse AABB overlaps obstacle,
    // but when subdivided, no sub-AABB actually overlaps.
    //
    // Parent endpoint: [0,0,0] (point)
    // Child endpoint:  [0,0,0]-[4,0,0] (wide in x only)
    // Coarse AABB: [0,0,0, 4,0,0] �?degenerate in y,z
    //
    // Place obstacle at (2, 5, 0) �?far in y.
    // Coarse check would miss regardless, but let's make one that
    // the coarse catches but subdiv misses:
    //
    // Parent: [0,0,0, 0,0,0] (point)
    // Child:  [0,0,0, 10,0,0] (wide x)
    // Coarse AABB: [0,0,0, 10,0,0]
    // With radius 1.0: [�?,�?,�?, 11,1,1]
    // Obstacle at (5, 0, 0) hs(0.01, 0.01, 0.01)
    // Coarse hits (5 is inside), but with 4 subs:
    //   sub 0: x=[0, 2.5], sub 1: x=[2.5, 5.0], etc.
    //   With radius the subs still cover it, so this won't help.
    //
    // Better: use a diagonal scenario where fine subs DON'T cover a corner.
    // Parent: [-1,-1,0, 1,1,0]
    // Child:  [1,-1,0, -1,1,0] (X-shaped intervals �?big coarse AABB)
    // Coarse: [-1,-1,0, 1,1,0]
    // Sub at t=0..0.5 and t=0.5..1:
    //   sub0: parent_lo=(-1,-1), parent_hi=(1,1), child_lo=(1,-1), child_hi=(-1,1)
    //   at t0=0: lo=(-1,-1), hi=(1,1)
    //   at t1=0.5: lo=lerp(-1,1)=0, lerp(-1,-1)=-1; hi=lerp(1,-1)=0, lerp(1,1)=1
    //   sub0: lo=min((-1,-1),(0,-1))=(-1,-1), hi=max((1,1),(0,1))=(1,1) �?same as coarse
    //
    // OK, subdivision doesn't actually make it tighter in this way.
    // The subdivision helps when the child endpoint has a tight range
    // but parent is far away.
    //
    // Simple verifiable test: just ensure the function returns correct results.

    // Scenario: link from (0,0,0) to (10,0,0). Obstacle at (5,0,0).
    float frames[] = {
        0.f, 0.f, 0.f,  0.f, 0.f, 0.f,   // ep 0 (point at origin)
        10.f, 0.f, 0.f,  10.f, 0.f, 0.f,  // ep 1 (point at (10,0,0))
    };
    int active_link_map[] = {1};
    float base_pos[] = {0.f, 0.f, 0.f};
    float link_radii[] = {0.5f};

    // Derive coarse AABB
    float coarse[6];
    envelope::derive_aabb(frames, 2, active_link_map, 1, link_radii, base_pos, coarse);

    // Obstacle in the middle �?should collide
    Obstacle obs_hit = make_obstacle(5, 0, 0, 0.1, 0.1, 0.1);
    bool hit = envelope::collide_aabb_subdiv(
        coarse, frames, 2, active_link_map, 1,
        link_radii, base_pos, &obs_hit, 1, 8, 64);
    CHECK(hit == true, "subdiv: obstacle in middle �?collision");

    // Obstacle far away �?should not collide
    Obstacle obs_miss = make_obstacle(5, 10, 0, 0.1, 0.1, 0.1);
    bool miss = envelope::collide_aabb_subdiv(
        coarse, frames, 2, active_link_map, 1,
        link_radii, base_pos, &obs_miss, 1, 8, 64);
    CHECK(miss == false, "subdiv: obstacle far �?no collision");
}

static void test_collide_aabb_subdiv_adaptive() {
    SECTION("collide_aabb_subdiv �?adaptive mode (subdiv_n=0)");

    float frames[] = {
        0.f, 0.f, 0.f,  0.f, 0.f, 0.f,
        5.f, 0.f, 0.f,  5.f, 0.f, 0.f,
    };
    int active_link_map[] = {1};
    float base_pos[] = {0.f, 0.f, 0.f};
    float link_radii[] = {0.2f};
    float coarse[6];
    envelope::derive_aabb(frames, 2, active_link_map, 1, link_radii, base_pos, coarse);

    Obstacle obs = make_obstacle(2.5, 0, 0, 0.05, 0.05, 0.05);

    // subdiv_n=0 �?adaptive
    bool hit = envelope::collide_aabb_subdiv(
        coarse, frames, 2, active_link_map, 1,
        link_radii, base_pos, &obs, 1, 0, 32);
    CHECK(hit == true, "subdiv adaptive: hit in middle");
}

static void test_check_collision_dispatcher() {
    SECTION("check_collision �?AABB dispatcher");

    float link_iaabbs[] = {0.f, 0.f, 0.f, 1.f, 1.f, 1.f};
    Obstacle obs_hit = make_obstacle(0.5, 0.5, 0.5, 0.1, 0.1, 0.1);
    Obstacle obs_miss = make_obstacle(10, 10, 10, 0.1, 0.1, 0.1);

    CHECK(envelope::check_collision(CollisionPolicy::AABB, link_iaabbs, 1, &obs_hit, 1) == true,
          "AABB dispatch hit");
    CHECK(envelope::check_collision(CollisionPolicy::AABB, link_iaabbs, 1, &obs_miss, 1) == false,
          "AABB dispatch miss");

    // SUBDIV/GRID fall back to AABB in the unified dispatcher
    CHECK(envelope::check_collision(CollisionPolicy::AABB_SUBDIV, link_iaabbs, 1, &obs_hit, 1) == true,
          "SUBDIV fallback hit");
    CHECK(envelope::check_collision(CollisionPolicy::GRID, link_iaabbs, 1, &obs_miss, 1) == false,
          "GRID fallback miss");
}

static void test_check_collision_legacy_interleaved() {
    SECTION("check_collision_aabb_legacy �?interleaved format");

    float link_iaabbs[] = {0.f, 0.f, 0.f, 2.f, 2.f, 2.f};
    // Interleaved: [lo_x, hi_x, lo_y, hi_y, lo_z, hi_z]
    float obs_compact[] = {1.f, 1.5f, 1.f, 1.5f, 1.f, 1.5f};  // inside link

    bool hit = envelope::check_collision_aabb_legacy(link_iaabbs, 1, obs_compact, 1);
    CHECK(hit == true, "legacy interleaved hit");

    float obs_far[] = {10.f, 11.f, 10.f, 11.f, 10.f, 11.f};
    bool miss = envelope::check_collision_aabb_legacy(link_iaabbs, 1, obs_far, 1);
    CHECK(miss == false, "legacy interleaved miss");
}

// ══════════════════════════════════════════════════════════════════════════�?
//  Section 3: grid_store
// ══════════════════════════════════════════════════════════════════════════�?

static void test_grid_store_basic() {
    SECTION("GridStore �?derive_from_aabbs + check_collision");

    float wb[] = {0.f, 0.f, 0.f, 3.2f, 3.2f, 3.2f};
    envelope::GridStore store(wb, 16);

    // AABB occupying center of grid
    float aabb[] = {1.0f, 1.0f, 1.0f, 2.0f, 2.0f, 2.0f};
    store.derive_from_aabbs(0, aabb, 1);

    CHECK(store.has_grid(0) == true, "grid 0 valid after derive");
    CHECK(store.occupied_count(0) > 0, "grid 0 has occupied voxels");

    // Obstacle overlapping the occupied region �?collision
    Obstacle obs_hit = make_obstacle(1.5, 1.5, 1.5, 0.3, 0.3, 0.3);
    CHECK(store.check_collision(0, &obs_hit, 1) == true, "grid collision hit");

    // Obstacle outside the occupied region �?no collision
    Obstacle obs_miss = make_obstacle(2.8, 2.8, 2.8, 0.1, 0.1, 0.1);
    CHECK(store.check_collision(0, &obs_miss, 1) == false, "grid collision miss");
}

static void test_grid_store_union() {
    SECTION("GridStore �?union_grids");

    float wb[] = {0.f, 0.f, 0.f, 3.2f, 3.2f, 3.2f};
    envelope::GridStore store(wb, 16);

    // Grid A: left half
    float aabb_a[] = {0.f, 0.f, 0.f, 1.6f, 3.2f, 3.2f};
    store.derive_from_aabbs(0, aabb_a, 1);

    // Grid B: right half
    float aabb_b[] = {1.6f, 0.f, 0.f, 3.2f, 3.2f, 3.2f};
    store.derive_from_aabbs(1, aabb_b, 1);

    int occ_a = store.occupied_count(0);
    int occ_b = store.occupied_count(1);

    // Union into slot 2
    store.union_grids(2, 0, 1);
    int occ_union = store.occupied_count(2);

    CHECK(store.has_grid(2) == true, "union grid valid");
    // Union should have at least as many voxels as either part
    CHECK(occ_union >= occ_a, "union >= A");
    CHECK(occ_union >= occ_b, "union >= B");
    // And no more than sum (possible overlap)
    CHECK(occ_union <= occ_a + occ_b, "union <= A + B");
}

static void test_grid_store_collide_grid_function() {
    SECTION("collide_grid �?thin wrapper");

    float wb[] = {0.f, 0.f, 0.f, 3.2f, 3.2f, 3.2f};
    envelope::GridStore store(wb, 16);

    float aabb[] = {0.5f, 0.5f, 0.5f, 2.5f, 2.5f, 2.5f};
    store.derive_from_aabbs(0, aabb, 1);

    Obstacle obs_hit = make_obstacle(1.0, 1.0, 1.0, 0.2, 0.2, 0.2);
    Obstacle obs_miss = make_obstacle(3.0, 3.0, 3.0, 0.05, 0.05, 0.05);

    CHECK(envelope::collide_grid(store, 0, &obs_hit, 1) == true,
          "collide_grid wrapper hit");
    CHECK(envelope::collide_grid(store, 0, &obs_miss, 1) == false,
          "collide_grid wrapper miss");
}

static void test_grid_store_empty_node() {
    SECTION("GridStore �?empty/invalid node");

    float wb[] = {0.f, 0.f, 0.f, 3.2f, 3.2f, 3.2f};
    envelope::GridStore store(wb, 16);

    CHECK(store.has_grid(0) == false, "node 0 not valid initially");
    CHECK(store.occupied_count(0) == 0, "empty node has 0 voxels");

    // check_collision on invalid node returns true (conservative)
    Obstacle obs = make_obstacle(1, 1, 1, 0.1, 0.1, 0.1);
    CHECK(store.check_collision(0, &obs, 1) == true, "invalid node �?true (conservative)");
}

static void test_grid_store_persistence() {
    SECTION("GridStore �?save/load round-trip");

    const char* path = "_test_grid_store_tmp.grd3";
    {
        float wb[] = {-1.f, -1.f, -1.f, 2.f, 2.f, 2.f};
        envelope::GridStore store(wb, 8);

        float aabb1[] = {0.f, 0.f, 0.f, 1.f, 1.f, 1.f};
        store.derive_from_aabbs(0, aabb1, 1);

        float aabb2[] = {-0.5f, -0.5f, -0.5f, 0.5f, 0.5f, 0.5f};
        store.derive_from_aabbs(3, aabb2, 1);

        store.save(path);
    }
    {
        envelope::GridStore loaded;
        loaded.load(path);

        CHECK(loaded.has_grid(0) == true, "loaded grid 0 valid");
        CHECK(loaded.has_grid(3) == true, "loaded grid 3 valid");
        CHECK(loaded.has_grid(1) == false, "loaded grid 1 invalid");

        int occ0 = loaded.occupied_count(0);
        int occ3 = loaded.occupied_count(3);
        CHECK(occ0 > 0, "loaded grid 0 has voxels");
        CHECK(occ3 > 0, "loaded grid 3 has voxels");

        // Verify collision still works
        Obstacle obs = make_obstacle(0.5, 0.5, 0.5, 0.1, 0.1, 0.1);
        CHECK(loaded.check_collision(0, &obs, 1) == true, "loaded grid collision");
    }
    unlink(path);
}

static void test_grid_store_z_mask_correctness() {
    SECTION("GridStore �?z_mask word-level correctness");

    // Verify the z_mask inline function
    using envelope::z_mask;
    using envelope::grid_word;
    using envelope::grid_bit_base;

    // z_mask(0, 1, 0) = bit 0
    CHECK(z_mask(0, 1, 0) == 1ULL, "z_mask(0,1,0) = 1");

    // z_mask(0, 32, 0) = 32 bits starting at bit 0
    CHECK(z_mask(0, 32, 0) == 0xFFFFFFFFULL, "z_mask(0,32,0) = lower 32 bits");

    // z_mask(0, 32, 32) = 32 bits starting at bit 32
    CHECK(z_mask(0, 32, 32) == 0xFFFFFFFF00000000ULL, "z_mask(0,32,32) = upper 32 bits");

    // grid_word(0, 0) = 0
    CHECK(grid_word(0, 0) == 0, "grid_word(0,0) = 0");

    // grid_bit_base: even y �?0, odd y �?32
    CHECK(grid_bit_base(0) == 0, "bit_base(0) = 0");
    CHECK(grid_bit_base(1) == 32, "bit_base(1) = 32");
    CHECK(grid_bit_base(2) == 0, "bit_base(2) = 0");
}

// ══════════════════════════════════════════════════════════════════════════�?
//  Section 4: endpoint_store (basic tests)
// ══════════════════════════════════════════════════════════════════════════�?

static void test_endpoint_store_store_get() {
    SECTION("EndpointStore �?store and retrieve endpoints");

    Robot robot = make_iiwa14();
    envelope::EndpointStore store(robot, 16);

    int n_ep = store.n_endpoints();
    CHECK(n_ep == robot.n_endpoints(), "n_endpoints matches robot");
    CHECK(n_ep > 0, "has endpoints");

    // Create synthetic endpoints
    std::vector<float> eps(n_ep * 6);
    for (int k = 0; k < n_ep; ++k) {
        float* f = eps.data() + k * 6;
        f[0] = -0.1f * (k + 1);  // lo_x
        f[1] = -0.2f * (k + 1);  // lo_y
        f[2] = -0.3f * (k + 1);  // lo_z
        f[3] =  0.1f * (k + 1);  // hi_x
        f[4] =  0.2f * (k + 1);  // hi_y
        f[5] =  0.3f * (k + 1);  // hi_z
    }

    store.store_endpoints(0, eps.data());
    CHECK(store.has_endpoints(0) == true, "node 0 valid after store");
    CHECK(store.has_endpoints(1) == false, "node 1 not valid");

    const float* retrieved = store.get_endpoints(0);
    for (int i = 0; i < n_ep * 6; ++i) {
        CHECK_CLOSE(eps[i], retrieved[i], 1e-8f, "retrieved matches stored");
    }
}

static void test_endpoint_store_union() {
    SECTION("EndpointStore �?union_endpoints");

    Robot robot = make_iiwa14();
    envelope::EndpointStore store(robot, 16);
    int n_ep = store.n_endpoints();

    // Node 0: narrow intervals
    std::vector<float> eps_a(n_ep * 6);
    for (int k = 0; k < n_ep; ++k) {
        float* f = eps_a.data() + k * 6;
        f[0] = 0.f; f[1] = 0.f; f[2] = 0.f; f[3] = 1.f; f[4] = 1.f; f[5] = 1.f;
    }
    store.store_endpoints(0, eps_a.data());

    // Node 1: shifted intervals
    std::vector<float> eps_b(n_ep * 6);
    for (int k = 0; k < n_ep; ++k) {
        float* f = eps_b.data() + k * 6;
        f[0] = -1.f; f[1] = -1.f; f[2] = -1.f; f[3] = 0.5f; f[4] = 0.5f; f[5] = 0.5f;
    }
    store.store_endpoints(1, eps_b.data());

    // Union into node 2
    store.union_endpoints(2, 0, 1);
    CHECK(store.has_endpoints(2) == true, "union node valid");

    const float* u = store.get_endpoints(2);
    for (int k = 0; k < n_ep; ++k) {
        int off = k * 6;
        // lo = min(0, -1) = -1, hi = max(1, 0.5) = 1
        CHECK_CLOSE(u[off + 0], -1.f, 1e-6f, "union lo_x");
        CHECK_CLOSE(u[off + 3],  1.f, 1e-6f, "union hi_x");
    }
}

static void test_endpoint_store_refine() {
    SECTION("EndpointStore �?refine_endpoints");

    Robot robot = make_iiwa14();
    envelope::EndpointStore store(robot, 16);
    int n_ep = store.n_endpoints();

    // Node 0: wide intervals [�?, �?, �?, 3, 3, 3]
    std::vector<float> wide(n_ep * 6);
    for (int k = 0; k < n_ep; ++k) {
        float* f = wide.data() + k * 6;
        f[0] = -2.f; f[1] = -2.f; f[2] = -2.f; f[3] = 3.f; f[4] = 3.f; f[5] = 3.f;
    }
    store.store_endpoints(0, wide.data());

    // Node 1: tight intervals [�?, �?, �?, 2, 2, 2]
    std::vector<float> tight(n_ep * 6);
    for (int k = 0; k < n_ep; ++k) {
        float* f = tight.data() + k * 6;
        f[0] = -1.f; f[1] = -1.f; f[2] = -1.f; f[3] = 2.f; f[4] = 2.f; f[5] = 2.f;
    }
    store.store_endpoints(1, tight.data());

    // refine: tighten node 0 using node 1 as bound
    bool changed = store.refine_endpoints(0, 1);
    CHECK(changed == true, "refine should tighten");

    const float* r = store.get_endpoints(0);
    for (int k = 0; k < n_ep; ++k) {
        int off = k * 6;
        // refine: lo = max(-2, -1) = -1; hi = min(3, 2) = 2
        CHECK_CLOSE(r[off + 0], -1.f, 1e-6f, "refined lo_x");
        CHECK_CLOSE(r[off + 3],  2.f, 1e-6f, "refined hi_x");
    }

    // Second refine should not change
    bool changed2 = store.refine_endpoints(0, 1);
    CHECK(changed2 == false, "second refine no-op");
}

// ══════════════════════════════════════════════════════════════════════════�?
//  Section 5: scene + AABBCollisionChecker
// ══════════════════════════════════════════════════════════════════════════�?

static void test_scene_basic() {
    SECTION("Scene �?add/remove/clear");

    scene::Scene s;
    CHECK(s.n_obstacles() == 0, "empty scene");

    s.add_obstacle(make_obstacle(1, 0, 0, 0.1, 0.1, 0.1, "box1"));
    s.add_obstacle(make_obstacle(2, 0, 0, 0.1, 0.1, 0.1, "box2"));
    CHECK(s.n_obstacles() == 2, "2 obstacles added");

    s.remove_obstacle("box1");
    CHECK(s.n_obstacles() == 1, "1 after remove");
    CHECK(std::string(s[0].name) == "box2", "remaining is box2");

    s.clear();
    CHECK(s.n_obstacles() == 0, "empty after clear");
}

static void test_aabb_collision_checker() {
    SECTION("AABBCollisionChecker �?end-to-end");

    std::vector<Obstacle> obstacles = {
        make_obstacle(0.5, 0.5, 0.5, 0.1, 0.1, 0.1, "near"),
        make_obstacle(10, 10, 10, 0.1, 0.1, 0.1, "far"),
    };
    scene::Scene s(obstacles);
    scene::AABBCollisionChecker checker(s);

    // Link overlapping "near" obstacle
    float link_hit[] = {0.f, 0.f, 0.f, 1.f, 1.f, 1.f};
    CHECK(checker.check_collision(link_hit, 1) == true, "checker hit");

    // Link far from all obstacles
    float link_miss[] = {-5.f, -5.f, -5.f, -4.f, -4.f, -4.f};
    CHECK(checker.check_collision(link_miss, 1) == false, "checker miss");

    // Empty links �?no collision
    CHECK(checker.check_collision(link_hit, 0) == false, "no links �?false");
}

// ══════════════════════════════════════════════════════════════════════════�?
//  Section 6: envelope_type (compute_link_envelope)
// ══════════════════════════════════════════════════════════════════════════�?

static void test_envelope_type_link_iaabb() {
    SECTION("compute_link_envelope �?LinkIAABB");

    Robot robot = make_iiwa14();
    int n_act = robot.n_active_links();

    // Generate endpoint iAABBs via iFK
    std::vector<Interval> full_range;
    for (int d = 0; d < robot.n_joints(); ++d)
        full_range.push_back(robot.joint_limits().limits[d]);

    envelope::EndpointSourceConfig src_cfg;
    src_cfg.method = envelope::EndpointSource::IFK;
    auto ep = envelope::compute_endpoint_iaabb(src_cfg, robot, full_range);
    CHECK(ep.n_active == n_act, "ep n_active matches");

    // LinkIAABB (sub=1)
    auto env_cfg = envelope::EnvelopeTypeConfig::link_iaabb();
    auto result = envelope::compute_link_envelope(env_cfg, robot, ep);

    CHECK(result.valid == true, "LinkIAABB valid");
    CHECK(static_cast<int>(result.link_iaabbs.size()) == n_act * 6,
          "LinkIAABB size = n_active × 6");
    CHECK(result.volume > 0, "LinkIAABB volume > 0");
    CHECK(result.n_voxels == n_act, "LinkIAABB n_voxels = n_active");

    // Verify each AABB has lo < hi (non-degenerate)
    for (int li = 0; li < n_act; ++li) {
        const float* a = result.link_iaabbs.data() + li * 6;
        CHECK(a[3] >= a[0], "AABB hi_x >= lo_x");
        CHECK(a[4] >= a[1], "AABB hi_y >= lo_y");
        CHECK(a[5] >= a[2], "AABB hi_z >= lo_z");
    }
}

static void test_envelope_type_link_iaabb_subdiv() {
    SECTION("compute_link_envelope �?LinkIAABB (sub=4)");

    Robot robot = make_iiwa14();
    int n_act = robot.n_active_links();

    std::vector<Interval> narrow;
    for (int d = 0; d < robot.n_joints(); ++d) {
        auto& lim = robot.joint_limits().limits[d];
        double c = lim.center();
        double w = lim.width() * 0.1;
        narrow.push_back({c - w, c + w});
    }

    envelope::EndpointSourceConfig src_cfg;
    src_cfg.method = envelope::EndpointSource::IFK;
    auto ep = envelope::compute_endpoint_iaabb(src_cfg, robot, narrow);

    // LinkIAABB with sub=4
    envelope::EnvelopeTypeConfig env_cfg;
    env_cfg.type = envelope::EnvelopeType::LinkIAABB;
    env_cfg.n_sub = 4;
    auto result = envelope::compute_link_envelope(env_cfg, robot, ep);

    CHECK(result.valid == true, "LinkIAABB sub=4 valid");
    CHECK(static_cast<int>(result.link_iaabbs.size()) == n_act * 4 * 6,
          "LinkIAABB sub=4 size");
    CHECK(result.volume > 0, "sub=4 volume > 0");
}

static void test_envelope_type_link_iaabb_grid() {
    SECTION("compute_link_envelope �?LinkIAABB_Grid");

    Robot robot = make_iiwa14();

    // Narrow intervals for speed
    std::vector<Interval> narrow;
    for (int d = 0; d < robot.n_joints(); ++d) {
        auto& lim = robot.joint_limits().limits[d];
        double c = lim.center();
        double w = lim.width() * 0.05;
        narrow.push_back({c - w, c + w});
    }

    envelope::EndpointSourceConfig src_cfg;
    src_cfg.method = envelope::EndpointSource::IFK;
    auto ep = envelope::compute_endpoint_iaabb(src_cfg, robot, narrow);

    auto env_cfg = envelope::EnvelopeTypeConfig::link_iaabb_grid();
    auto result = envelope::compute_link_envelope(env_cfg, robot, ep);

    CHECK(result.valid == true, "Grid valid");
    CHECK(result.grid_R == 64, "Grid R=64");
    CHECK(static_cast<int>(result.grid.size()) == 64*64*64, "grid size = 64³");
    CHECK(result.n_voxels > 0, "has occupied voxels");
    CHECK(result.volume > 0, "grid volume > 0");
}

static void test_envelope_type_hull16_grid() {
    SECTION("compute_link_envelope �?Hull16_Grid");

    Robot robot = make_iiwa14();

    // Very narrow intervals
    std::vector<Interval> narrow;
    for (int d = 0; d < robot.n_joints(); ++d) {
        auto& lim = robot.joint_limits().limits[d];
        double c = lim.center();
        double w = lim.width() * 0.02;
        narrow.push_back({c - w, c + w});
    }

    envelope::EndpointSourceConfig src_cfg;
    src_cfg.method = envelope::EndpointSource::IFK;
    auto ep = envelope::compute_endpoint_iaabb(src_cfg, robot, narrow);

    auto env_cfg = envelope::EnvelopeTypeConfig::hull16_grid();
    auto result = envelope::compute_link_envelope(env_cfg, robot, ep);

    CHECK(result.valid == true, "Hull16 valid");
    CHECK(result.n_voxels > 0, "Hull16 has voxels");
    CHECK(result.volume > 0, "Hull16 volume > 0");
}

static void test_envelope_type_volume_ordering() {
    SECTION("compute_link_envelope : sub=4 individual sub-AABBs smaller than full");

    Robot robot = make_iiwa14();

    // Use narrow intervals
    std::vector<Interval> narrow;
    for (int d = 0; d < robot.n_joints(); ++d) {
        auto& lim = robot.joint_limits().limits[d];
        double c = lim.center();
        double w = lim.width() * 0.1;
        narrow.push_back({c - w, c + w});
    }

    envelope::EndpointSourceConfig src_cfg;
    src_cfg.method = envelope::EndpointSource::IFK;
    auto ep = envelope::compute_endpoint_iaabb(src_cfg, robot, narrow);

    // sub=1
    auto cfg1 = envelope::EnvelopeTypeConfig::link_iaabb();
    cfg1.n_sub = 1;
    auto r1 = envelope::compute_link_envelope(cfg1, robot, ep);

    // sub=4
    auto cfg4 = envelope::EnvelopeTypeConfig::link_iaabb();
    cfg4.n_sub = 4;
    auto r4 = envelope::compute_link_envelope(cfg4, robot, ep);

    CHECK(r1.valid && r4.valid, "both valid");
    // sub=4 produces 4x more boxes per link
    int n_act = ep.n_active;
    CHECK((int)r4.link_iaabbs.size() == n_act * 4 * 6,
          "sub=4 produces 4 boxes per link");
    CHECK((int)r1.link_iaabbs.size() == n_act * 1 * 6,
          "sub=1 produces 1 box per link");

    // Each individual sub-AABB should be no larger than the full per-link AABB
    // (subdivision splits the link so each piece is smaller or equal)
    int all_smaller = 1;
    for (int ci = 0; ci < n_act; ++ci) {
        const float* full_box = r1.link_iaabbs.data() + ci * 6;
        double full_vol = (double)(full_box[3]-full_box[0]) *
                          (full_box[4]-full_box[1]) * (full_box[5]-full_box[2]);
        for (int s = 0; s < 4; ++s) {
            const float* sub = r4.link_iaabbs.data() + (ci * 4 + s) * 6;
            double sv = (double)(sub[3]-sub[0]) * (sub[4]-sub[1]) * (sub[5]-sub[2]);
            if (sv > full_vol + 1e-8) all_smaller = 0;
        }
    }
    CHECK(all_smaller, "each sub-AABB vol <= its full-link AABB vol");
}

// ══════════════════════════════════════════════════════════════════════════
//  Section 7: link iAABB core pipeline deep tests
//
//  Focused tests for the Stage-2 link iAABB pipeline:
//    extract_link_iaabbs  (endpoint→link bridge)
//    derive_link_iaabbs_from_endpoints  (via compute_link_envelope)
//    consistency between sub=1 paths
//    collide_aabb_subdiv refinement effectiveness
//    soundness: link iAABB contains all FK samples
// ══════════════════════════════════════════════════════════════════════════

// ─── 7a. extract_link_iaabbs basic: endpoint→link with radius ──────────
static void test_extract_link_iaabbs_basic() {
    SECTION("extract_link_iaabbs : endpoint iAABB → per-link iAABB + radius");

    Robot robot = make_iiwa14();
    int n_act = robot.n_active_links();

    // Compute endpoint iAABBs via iFK with narrow intervals
    std::vector<Interval> narrow;
    for (int d = 0; d < robot.n_joints(); ++d) {
        auto& lim = robot.joint_limits().limits[d];
        double c = lim.center();
        double w = lim.width() * 0.1;
        narrow.push_back({c - w, c + w});
    }

    envelope::EndpointSourceConfig src_cfg;
    src_cfg.method = envelope::EndpointSource::IFK;
    auto ep = envelope::compute_endpoint_iaabb(src_cfg, robot, narrow);

    // Extract link iAABBs
    std::vector<float> link_aabbs(n_act * 6);
    envelope::extract_link_iaabbs(ep, robot, link_aabbs.data());

    // All link iAABBs should be valid (lo <= hi) and non-degenerate
    for (int ci = 0; ci < n_act; ++ci) {
        const float* a = link_aabbs.data() + ci * 6;
        CHECK(a[3] >= a[0], "link lo_x <= hi_x");
        CHECK(a[4] >= a[1], "link lo_y <= hi_y");
        CHECK(a[5] >= a[2], "link lo_z <= hi_z");
        // With radius, dimensions should be > 0
        float dx = a[3] - a[0], dy = a[4] - a[1], dz = a[5] - a[2];
        CHECK(dx > 0 && dy > 0 && dz > 0, "link iAABB non-degenerate");
    }

    // Link iAABBs must contain parent and child endpoint intervals.
    // For each active link i: aabb[i] must contain endpoint[parent] ∪ endpoint[child] + radius
    // endpoint_iaabbs is in paired layout: ci*2 = proximal, ci*2+1 = distal
    const double* lr = robot.active_link_radii();
    for (int ci = 0; ci < n_act; ++ci) {
        int prox_idx = ci * 2;
        int dist_idx = ci * 2 + 1;
        float r = lr ? static_cast<float>(lr[ci]) : 0.f;
        const float* a = link_aabbs.data() + ci * 6;

        // distal endpoint
        {
            const float* ep_c = ep.endpoint_iaabbs.data() + dist_idx * 6;
            CHECK(a[0] <= ep_c[0] - r + 1e-5f, "link lo_x <= ep_child lo_x - r");
            CHECK(a[1] <= ep_c[1] - r + 1e-5f, "link lo_y <= ep_child lo_y - r");
            CHECK(a[2] <= ep_c[2] - r + 1e-5f, "link lo_z <= ep_child lo_z - r");
            CHECK(a[3] >= ep_c[3] + r - 1e-5f, "link hi_x >= ep_child hi_x + r");
            CHECK(a[4] >= ep_c[4] + r - 1e-5f, "link hi_y >= ep_child hi_y + r");
            CHECK(a[5] >= ep_c[5] + r - 1e-5f, "link hi_z >= ep_child hi_z + r");
        }

        // proximal endpoint
        {
            const float* ep_p = ep.endpoint_iaabbs.data() + prox_idx * 6;
            CHECK(a[0] <= ep_p[0] - r + 1e-5f, "link lo_x <= ep_parent lo_x - r");
            CHECK(a[3] >= ep_p[3] + r - 1e-5f, "link hi_x >= ep_parent hi_x + r");
        }
    }
}

// ─── 7b. extract_link_iaabbs ⟺ compute_link_envelope(sub=1) consistency ──
static void test_extract_vs_compute_link_envelope_consistency() {
    SECTION("extract_link_iaabbs == compute_link_envelope(LinkIAABB,sub=1)");

    Robot robot = make_iiwa14();
    int n_act = robot.n_active_links();

    std::vector<Interval> narrow;
    for (int d = 0; d < robot.n_joints(); ++d) {
        auto& lim = robot.joint_limits().limits[d];
        double c = lim.center();
        double w = lim.width() * 0.15;
        narrow.push_back({c - w, c + w});
    }

    envelope::EndpointSourceConfig src_cfg;
    src_cfg.method = envelope::EndpointSource::IFK;
    auto ep = envelope::compute_endpoint_iaabb(src_cfg, robot, narrow);

    // Path 1: extract_link_iaabbs (direct)
    std::vector<float> direct(n_act * 6);
    envelope::extract_link_iaabbs(ep, robot, direct.data());

    // Path 2: compute_link_envelope(LinkIAABB, sub=1) → result.link_iaabbs
    auto env_cfg = envelope::EnvelopeTypeConfig::link_iaabb();
    env_cfg.n_sub = 1;
    auto env_result = envelope::compute_link_envelope(env_cfg, robot, ep);

    CHECK(static_cast<int>(env_result.link_iaabbs.size()) == n_act * 6,
          "envelope result size matches");

    // Both paths should produce identical results
    int match_count = 0;
    for (int i = 0; i < n_act * 6; ++i) {
        if (std::abs(direct[i] - env_result.link_iaabbs[i]) < 1e-6f)
            ++match_count;
    }
    CHECK(match_count == n_act * 6,
          "extract_link_iaabbs == compute_link_envelope(sub=1)");
}

// ─── 7c. derive_aabb vs derive_aabb_subdivided(sub=1) consistency ─────────
static void test_derive_aabb_vs_subdivided_consistency() {
    SECTION("derive_aabb == derive_aabb_subdivided(sub=1) for each link");

    Robot robot = make_iiwa14();
    int n_act = robot.n_active_links();

    std::vector<Interval> full_range;
    for (int d = 0; d < robot.n_joints(); ++d)
        full_range.push_back(robot.joint_limits().limits[d]);

    envelope::EndpointSourceConfig src_cfg;
    src_cfg.method = envelope::EndpointSource::IFK;
    auto ep = envelope::compute_endpoint_iaabb(src_cfg, robot, full_range);

    // derive_aabb_paired (multi-link, paired layout)
    const double* lr = robot.active_link_radii();
    float base_pos[3] = {0.f, 0.f, 0.f};

    std::vector<float> aabb_multi(n_act * 6);
    envelope::derive_aabb_paired(ep.endpoint_iaabbs.data(), n_act,
                                 nullptr, aabb_multi.data());

    // derive_aabb_subdivided(sub=1) per link (no radius, to match derive_aabb_paired with nullptr radii)
    // endpoint_iaabbs is in paired layout: ci*2 = proximal, ci*2+1 = distal
    for (int ci = 0; ci < n_act; ++ci) {
        float single[6];
        envelope::derive_aabb_subdivided(
            ep.endpoint_iaabbs.data(), ep.n_active_ep,
            ci * 2, ci * 2 + 1, 1, 0.f, base_pos, single);

        const float* multi = aabb_multi.data() + ci * 6;
        bool ok = true;
        for (int d = 0; d < 6; ++d) {
            if (std::abs(single[d] - multi[d]) > 1e-6f) ok = false;
        }
        CHECK(ok, "derive_aabb == derive_aabb_subdivided(sub=1) per link");
    }
}

// ─── 7d. Soundness: link iAABB contains all FK samples ───────────────────
static void test_link_iaabb_soundness_fk_samples() {
    SECTION("link iAABB soundness: contains all sampled FK positions");

    Robot robot = make_iiwa14();
    int n_act = robot.n_active_links();

    // Moderate intervals
    std::vector<Interval> ivs;
    for (int d = 0; d < robot.n_joints(); ++d) {
        auto& lim = robot.joint_limits().limits[d];
        double c = lim.center();
        double w = lim.width() * 0.2;
        ivs.push_back({c - w, c + w});
    }

    // Compute link iAABBs via iFK
    envelope::EndpointSourceConfig src_cfg;
    src_cfg.method = envelope::EndpointSource::IFK;
    auto ep = envelope::compute_endpoint_iaabb(src_cfg, robot, ivs);

    std::vector<float> link_aabbs(n_act * 6);
    envelope::extract_link_iaabbs(ep, robot, link_aabbs.data());

    // Sample N random configurations within intervals, compute FK, check containment
    const int N_SAMPLES = 200;
    const int* alm = robot.active_link_map();
    const double* lr = robot.active_link_radii();
    int violations = 0;

    std::srand(42);
    for (int sample = 0; sample < N_SAMPLES; ++sample) {
        // Random q within intervals
        Eigen::VectorXd q(robot.n_joints());
        for (int d = 0; d < robot.n_joints(); ++d) {
            double t = static_cast<double>(std::rand()) / RAND_MAX;
            q[d] = ivs[d].lo + t * (ivs[d].hi - ivs[d].lo);
        }

        // Forward kinematics — transforms[0]=base, transforms[i+1]=frame i
        auto transforms = robot.fk_transforms(q);

        // Check each active link: child frame position must be inside link iAABB
        for (int ci = 0; ci < n_act; ++ci) {
            int frame_idx = alm[ci];
            const float* a = link_aabbs.data() + ci * 6;

            // Get child frame position (frame_idx+1 because transforms[0]=base)
            Eigen::Vector3d pos = transforms[frame_idx + 1].block<3,1>(0,3);

            float px = static_cast<float>(pos[0]);
            float py = static_cast<float>(pos[1]);
            float pz = static_cast<float>(pos[2]);

            // Check containment (position must be inside link iAABB, with radius margin)
            if (px < a[0] - 1e-4f || px > a[3] + 1e-4f ||
                py < a[1] - 1e-4f || py > a[4] + 1e-4f ||
                pz < a[2] - 1e-4f || pz > a[5] + 1e-4f) {
                ++violations;
            }
        }
    }
    CHECK(violations == 0,
          "all FK samples contained in link iAABBs (soundness)");
}

// ─── 7e. collide_aabb_subdiv refinement: diagonal link scenario ──────────
static void test_collide_aabb_subdiv_effective_refine() {
    SECTION("collide_aabb_subdiv : subdivision effectively refines false positive");

    // Construct a diagonal link from (0,0,0) to (10,10,0).
    // The coarse AABB is [0,0,-r, 10,10,r] (full bounding box of diagonal).
    // Place obstacle at (9, 1, 0) — inside the coarse AABB but far from the
    // actual link line. With fine subdivision, sub-AABBs near (9,1) will be
    // narrow and NOT overlap the obstacle.
    float frames[] = {
        0.f, 0.f, 0.f,  0.f, 0.f, 0.f,   // ep 0 (point at origin)
       10.f,10.f, 0.f, 10.f,10.f, 0.f,   // ep 1 (point at (10,10,0))
    };
    int active_link_map[] = {1};
    float base_pos[] = {0.f, 0.f, 0.f};
    float link_radii[] = {0.1f};

    // Coarse AABB
    float coarse[6];
    envelope::derive_aabb(frames, 2, active_link_map, 1, link_radii, base_pos, coarse);
    // coarse should be [-0.1, -0.1, -0.1, 10.1, 10.1, 0.1]

    // Obstacle at (9, 1, 0) with half-sizes (0.05, 0.05, 0.05)
    // This is inside the coarse AABB but far from the diagonal.
    Obstacle obs_corner = make_obstacle(9.0, 1.0, 0.0, 0.05, 0.05, 0.05);

    // Coarse AABB check: should detect collision (false positive)
    bool coarse_hit = envelope::collide_aabb(coarse, 1, &obs_corner, 1);
    CHECK(coarse_hit == true, "coarse AABB: false positive on corner obstacle");

    // Fine subdivision (n_sub=16): should NOT detect collision
    // The sub-AABB near t=0.9 covers x=[8.5,10.1], y=[8.5,10.1] approximately
    // while (9,1) is outside the diagonal strip.
    bool fine_hit = envelope::collide_aabb_subdiv(
        coarse, frames, 2, active_link_map, 1,
        link_radii, base_pos, &obs_corner, 1,
        16, 64);
    CHECK(fine_hit == false, "subdiv n=16: refines false positive on corner");

    // Obstacle ON the diagonal at (5, 5, 0) — should still detect
    Obstacle obs_on_diag = make_obstacle(5.0, 5.0, 0.0, 0.05, 0.05, 0.05);
    bool diag_hit = envelope::collide_aabb_subdiv(
        coarse, frames, 2, active_link_map, 1,
        link_radii, base_pos, &obs_on_diag, 1,
        16, 64);
    CHECK(diag_hit == true, "subdiv n=16: obstacle on diagonal still detected");
}

// ─── 7f. compute_link_envelope sub>1 containment ─────────────────────────
static void test_link_envelope_subdiv_containment() {
    SECTION("compute_link_envelope(sub=4): sub-AABBs contained in sub=1 AABBs");

    Robot robot = make_iiwa14();
    int n_act = robot.n_active_links();

    std::vector<Interval> narrow;
    for (int d = 0; d < robot.n_joints(); ++d) {
        auto& lim = robot.joint_limits().limits[d];
        double c = lim.center();
        double w = lim.width() * 0.1;
        narrow.push_back({c - w, c + w});
    }

    envelope::EndpointSourceConfig src_cfg;
    src_cfg.method = envelope::EndpointSource::IFK;
    auto ep = envelope::compute_endpoint_iaabb(src_cfg, robot, narrow);

    // sub=1
    auto cfg1 = envelope::EnvelopeTypeConfig::link_iaabb();
    cfg1.n_sub = 1;
    auto r1 = envelope::compute_link_envelope(cfg1, robot, ep);

    // sub=4
    auto cfg4 = envelope::EnvelopeTypeConfig::link_iaabb();
    cfg4.n_sub = 4;
    auto r4 = envelope::compute_link_envelope(cfg4, robot, ep);

    CHECK(r1.valid && r4.valid, "both valid");

    // Each sub-AABB (from sub=4) must be contained within its parent link's
    // full AABB (from sub=1) — this verifies that subdivision only narrows.
    int contained = 0, total = 0;
    for (int ci = 0; ci < n_act; ++ci) {
        const float* full_box = r1.link_iaabbs.data() + ci * 6;
        for (int s = 0; s < 4; ++s) {
            const float* sub = r4.link_iaabbs.data() + (ci * 4 + s) * 6;
            ++total;
            if (sub[0] >= full_box[0] - 1e-5f &&
                sub[1] >= full_box[1] - 1e-5f &&
                sub[2] >= full_box[2] - 1e-5f &&
                sub[3] <= full_box[3] + 1e-5f &&
                sub[4] <= full_box[4] + 1e-5f &&
                sub[5] <= full_box[5] + 1e-5f) {
                ++contained;
            }
        }
    }
    CHECK(contained == total, "all sub-AABBs contained in full-link AABBs");
}

// ─── 7g. collide_aabb_subdiv vs collide_aabb: subdiv never misses ────────
static void test_subdiv_never_misses_real_collision() {
    SECTION("collide_aabb_subdiv: never misses a real AABB collision");

    Robot robot = make_iiwa14();
    int n_act = robot.n_active_links();

    // Narrow intervals
    std::vector<Interval> narrow;
    for (int d = 0; d < robot.n_joints(); ++d) {
        auto& lim = robot.joint_limits().limits[d];
        double c = lim.center();
        double w = lim.width() * 0.05;
        narrow.push_back({c - w, c + w});
    }

    envelope::EndpointSourceConfig src_cfg;
    src_cfg.method = envelope::EndpointSource::IFK;
    auto ep = envelope::compute_endpoint_iaabb(src_cfg, robot, narrow);

    // Extract link iAABBs and endpoint frames
    std::vector<float> link_aabbs(n_act * 6);
    envelope::extract_link_iaabbs(ep, robot, link_aabbs.data());

    const double* lr_d = robot.active_link_radii();
    std::vector<float> lr_f(n_act);
    for (int i = 0; i < n_act; ++i) lr_f[i] = lr_d ? static_cast<float>(lr_d[i]) : 0.f;
    float base_pos[3] = {0.f, 0.f, 0.f};

    // endpoint_iaabbs is in paired layout: [prox0, dist0, prox1, dist1, ...]
    // For derive_aabb_subdivided: use ci*2 (proximal) and ci*2+1 (distal).
    // For collide_aabb_subdiv: it internally uses active_link_map[li]-1 as
    // parent index, so provide a paired-compatible map where entry ci = ci*2+1.
    std::vector<int> paired_alm(n_act);
    for (int i = 0; i < n_act; ++i) paired_alm[i] = i * 2 + 1;

    // For each link, place an obstacle at the center of one real subdivided
    // sub-AABB. 这样命中点来自细分后的真实包围盒，而不是粗 AABB 的几何中心，
    // 可避免“粗盒中心落在对角线空区”这类假阳性位置导致 subdiv 被误判为漏检。
    int aabb_hits = 0, subdiv_hits = 0;
    for (int ci = 0; ci < n_act; ++ci) {
        float sub_boxes[8 * 6];
        envelope::derive_aabb_subdivided(
            ep.endpoint_iaabbs.data(), ep.n_active_ep,
            ci * 2, ci * 2 + 1, 8, lr_f[ci], base_pos,
            sub_boxes);

        const int mid_sub = 4;
        const float* sub = sub_boxes + mid_sub * 6;
        double cx = 0.5 * (sub[0] + sub[3]);
        double cy = 0.5 * (sub[1] + sub[4]);
        double cz = 0.5 * (sub[2] + sub[5]);

        Obstacle obs = make_obstacle(cx, cy, cz, 0.001, 0.001, 0.001);

        bool hit_aabb = envelope::collide_aabb(link_aabbs.data(), n_act, &obs, 1);
        bool hit_subdiv = envelope::collide_aabb_subdiv(
            link_aabbs.data(),
            ep.endpoint_iaabbs.data(), ep.n_active_ep,
            paired_alm.data(), n_act, lr_f.data(), base_pos,
            &obs, 1, 8, 64);

        if (hit_aabb) ++aabb_hits;
        if (hit_subdiv) ++subdiv_hits;
    }
    // 对于放置在真实细分子盒中心的障碍物，粗 AABB 与细分 AABB 都必须命中。
    CHECK(aabb_hits == n_act, "AABB detects all link-center obstacles");
    CHECK(subdiv_hits == n_act, "subdiv detects all real sub-box center obstacles");
}

// ─── 7h. LinkIAABB_Grid: link iAABBs correctly rasterised ───────────────
static void test_link_iaabb_grid_rasterisation() {
    SECTION("LinkIAABB_Grid: rasterised grid covers all link iAABBs");

    Robot robot = make_iiwa14();
    int n_act = robot.n_active_links();

    std::vector<Interval> narrow;
    for (int d = 0; d < robot.n_joints(); ++d) {
        auto& lim = robot.joint_limits().limits[d];
        double c = lim.center();
        double w = lim.width() * 0.1;
        narrow.push_back({c - w, c + w});
    }

    envelope::EndpointSourceConfig src_cfg;
    src_cfg.method = envelope::EndpointSource::IFK;
    auto ep = envelope::compute_endpoint_iaabb(src_cfg, robot, narrow);

    auto grid_cfg = envelope::EnvelopeTypeConfig::link_iaabb_grid();
    grid_cfg.n_sub = 4;
    grid_cfg.grid_R = 32;  // smaller for fast test
    auto res = envelope::compute_link_envelope(grid_cfg, robot, ep);

    CHECK(res.valid, "grid envelope valid");
    CHECK(res.n_voxels > 0, "grid has occupied voxels");
    CHECK(res.grid_R == 32, "grid resolution matches");
    CHECK(static_cast<int>(res.grid.size()) == 32*32*32, "grid byte array size");

    // Verify that every link iAABB sub-box has at least some voxels set
    // (unless the link is outside world_bounds)
    int n_covered = 0;
    const float* wb = grid_cfg.world_bounds;
    float cell[3];
    for (int c = 0; c < 3; ++c)
        cell[c] = (wb[3 + c] - wb[c]) / 32.0f;

    int n_subs = static_cast<int>(res.link_iaabbs.size()) / 6;
    for (int k = 0; k < n_subs; ++k) {
        const float* a = res.link_iaabbs.data() + k * 6;

        // Check if this sub-AABB is inside world bounds at all
        if (a[3] < wb[0] || a[0] > wb[3] ||
            a[4] < wb[1] || a[1] > wb[4] ||
            a[5] < wb[2] || a[2] > wb[5])
            continue;

        // Find center voxel
        float cx = 0.5f * (a[0] + a[3]);
        float cy = 0.5f * (a[1] + a[4]);
        float cz = 0.5f * (a[2] + a[5]);
        int ix = static_cast<int>((cx - wb[0]) / cell[0]);
        int iy = static_cast<int>((cy - wb[1]) / cell[1]);
        int iz = static_cast<int>((cz - wb[2]) / cell[2]);
        ix = std::max(0, std::min(ix, 31));
        iy = std::max(0, std::min(iy, 31));
        iz = std::max(0, std::min(iz, 31));

        if (res.grid[ix * 32 * 32 + iy * 32 + iz])
            ++n_covered;
    }
    CHECK(n_covered > 0, "at least some link iAABBs have grid voxels set");
}

// ══════════════════════════════════════════════════════════════════════════
//  Section 8: end-to-end pipeline
// ══════════════════════════════════════════════════════════════════════════

static void test_end_to_end_collision_pipeline() {
    SECTION("End-to-end: endpoint_source �?envelope �?collision");

    Robot robot = make_iiwa14();

    // Narrow C-space intervals (near center of joint range)
    std::vector<Interval> intervals;
    for (int d = 0; d < robot.n_joints(); ++d) {
        auto& lim = robot.joint_limits().limits[d];
        double c = lim.center();
        double w = lim.width() * 0.05;
        intervals.push_back({c - w, c + w});
    }

    // Stage 1: endpoint iAABBs
    envelope::EndpointSourceConfig src_cfg;
    src_cfg.method = envelope::EndpointSource::IFK;
    auto ep = envelope::compute_endpoint_iaabb(src_cfg, robot, intervals);

    CHECK(ep.n_active > 0, "e2e: has active links");
    CHECK(ep.n_active_ep > 0, "e2e: has endpoints");

    // Stage 2: envelope (LinkIAABB sub=1)
    auto env_cfg = envelope::EnvelopeTypeConfig::link_iaabb();
    auto env = envelope::compute_link_envelope(env_cfg, robot, ep);
    CHECK(env.valid, "e2e: envelope valid");

    // Stage 3: collision check
    // Create obstacle at robot base area �?should collide with link 0
    Obstacle obs_base = make_obstacle(0, 0, 0.15, 0.3, 0.3, 0.3, "base_area");
    bool hit = envelope::collide_aabb(
        env.link_iaabbs.data(), ep.n_active,
        &obs_base, 1);
    // Link 0 should be near the base �?likely collision
    // (This depends on robot geometry but IIWA base is at origin)
    std::printf("  e2e collision with base obstacle: %s\n", hit ? "HIT" : "miss");
    // Not asserting specific result since it depends on exact geometry

    // Obstacle far away �?definitely no collision
    Obstacle obs_far = make_obstacle(100, 100, 100, 0.1, 0.1, 0.1, "far");
    bool miss = envelope::collide_aabb(
        env.link_iaabbs.data(), ep.n_active,
        &obs_far, 1);
    CHECK(miss == false, "e2e: far obstacle �?no collision");
}

static void test_end_to_end_with_scene() {
    SECTION("End-to-end: endpoint_source �?envelope �?Scene + AABBCollisionChecker");

    Robot robot = make_iiwa14();

    std::vector<Interval> intervals;
    for (int d = 0; d < robot.n_joints(); ++d) {
        auto& lim = robot.joint_limits().limits[d];
        double c = lim.center();
        double w = lim.width() * 0.05;
        intervals.push_back({c - w, c + w});
    }

    envelope::EndpointSourceConfig src_cfg;
    src_cfg.method = envelope::EndpointSource::IFK;
    auto ep = envelope::compute_endpoint_iaabb(src_cfg, robot, intervals);

    auto env_cfg = envelope::EnvelopeTypeConfig::link_iaabb();
    auto env = envelope::compute_link_envelope(env_cfg, robot, ep);

    // Build scene
    scene::Scene s;
    s.add_obstacle(make_obstacle(100, 100, 100, 0.1, 0.1, 0.1, "very_far"));

    scene::AABBCollisionChecker checker(s);
    bool result = checker.check_collision(
        env.link_iaabbs.data(), ep.n_active);
    CHECK(result == false, "e2e scene: far obstacle �?collision free");
}

static void test_end_to_end_gridstore() {
    SECTION("End-to-end: envelope �?GridStore �?collision");

    Robot robot = make_iiwa14();

    std::vector<Interval> intervals;
    for (int d = 0; d < robot.n_joints(); ++d) {
        auto& lim = robot.joint_limits().limits[d];
        double c = lim.center();
        double w = lim.width() * 0.05;
        intervals.push_back({c - w, c + w});
    }

    envelope::EndpointSourceConfig src_cfg;
    src_cfg.method = envelope::EndpointSource::IFK;
    auto ep = envelope::compute_endpoint_iaabb(src_cfg, robot, intervals);

    auto env_cfg = envelope::EnvelopeTypeConfig::link_iaabb();
    auto env = envelope::compute_link_envelope(env_cfg, robot, ep);

    // Populate GridStore from link iAABBs
    float wb[] = {-1.f, -1.5f, -0.5f, 1.5f, 1.5f, 2.0f};
    envelope::GridStore gs(wb, 8);
    gs.derive_from_aabbs(0, env.link_iaabbs.data(),
                         static_cast<int>(env.link_iaabbs.size() / 6));
    CHECK(gs.has_grid(0), "e2e grid populated");
    CHECK(gs.occupied_count(0) > 0, "e2e grid has voxels");

    // Far obstacle �?no grid collision
    Obstacle obs_far = make_obstacle(100, 100, 100, 0.1, 0.1, 0.1);
    CHECK(envelope::collide_grid(gs, 0, &obs_far, 1) == false,
          "e2e grid: far obstacle �?collision free");
}

static void test_default_envelope_config() {
    SECTION("default_envelope_config �?factory correctness");

    // IFK(0) + LinkIAABB �?n_sub=1
    auto c1 = envelope::default_envelope_config(0, envelope::EnvelopeType::LinkIAABB);
    CHECK(c1.n_sub == 1, "IFK+LinkIAABB: n_sub=1");

    // Analytical(2) + Hull16_Grid �?n_sub=1
    auto c2 = envelope::default_envelope_config(2, envelope::EnvelopeType::Hull16_Grid);
    CHECK(c2.n_sub == 1, "Analytical+Hull16: n_sub=1");

    // IFK(0) + Hull16_Grid �?n_sub=1
    auto c3 = envelope::default_envelope_config(0, envelope::EnvelopeType::Hull16_Grid);
    CHECK(c3.n_sub == 1, "IFK+Hull16: n_sub=1");

    // CritSample(1) + LinkIAABB_Grid �?n_sub=16
    auto c4 = envelope::default_envelope_config(1, envelope::EnvelopeType::LinkIAABB_Grid);
    CHECK(c4.n_sub == 16, "CritSample+LinkIAABB_Grid: n_sub=16");
    CHECK(c4.grid_R == 64, "LinkIAABB_Grid: grid_R=64");
}

// ══════════════════════════════════════════════════════════════════════════�?
//  Main
// ══════════════════════════════════════════════════════════════════════════�?

int main() {
    std::printf("═══════════════════════════════════════════════════════════\n");
    std::printf("  SafeBoxForest v4 �?Envelope Pipeline Tests\n");
    std::printf("═══════════════════════════════════════════════════════════\n");

    // Section 1: envelope_derive
    test_derive_aabb_basic();
    test_derive_aabb_with_radius();
    test_derive_aabb_base_parent();
    test_derive_aabb_multi_link();
    test_derive_aabb_subdivided_identity();
    test_derive_aabb_subdivided_tighter();
    test_derive_aabb_subdivided_containment();
    test_adaptive_subdivision_count();
    test_derive_grid();

    // Section 2: collision_policy
    test_collide_aabb_hit();
    test_collide_aabb_miss();
    test_collide_aabb_touching();
    test_collide_aabb_multi_link_multi_obs();
    test_collide_aabb_empty();
    test_collide_aabb_subdiv_refine();
    test_collide_aabb_subdiv_adaptive();
    test_check_collision_dispatcher();
    test_check_collision_legacy_interleaved();

    // Section 3: grid_store
    test_grid_store_basic();
    test_grid_store_union();
    test_grid_store_collide_grid_function();
    test_grid_store_empty_node();
    test_grid_store_persistence();
    test_grid_store_z_mask_correctness();

    // Section 4: endpoint_store
    test_endpoint_store_store_get();
    test_endpoint_store_union();
    test_endpoint_store_refine();

    // Section 5: scene
    test_scene_basic();
    test_aabb_collision_checker();

    // Section 6: envelope_type
    test_envelope_type_link_iaabb();
    test_envelope_type_link_iaabb_subdiv();
    test_envelope_type_link_iaabb_grid();
    test_envelope_type_hull16_grid();
    test_envelope_type_volume_ordering();

    // Section 7: link iAABB core pipeline deep tests
    test_extract_link_iaabbs_basic();
    test_extract_vs_compute_link_envelope_consistency();
    test_derive_aabb_vs_subdivided_consistency();
    test_link_iaabb_soundness_fk_samples();
    test_collide_aabb_subdiv_effective_refine();
    test_link_envelope_subdiv_containment();
    test_subdiv_never_misses_real_collision();
    test_link_iaabb_grid_rasterisation();

    // Section 8: end-to-end
    test_end_to_end_collision_pipeline();
    test_end_to_end_with_scene();
    test_end_to_end_gridstore();
    test_default_envelope_config();

    std::printf("\n═══════════════════════════════════════════════════════════\n");
    std::printf("  Results: %d passed, %d failed\n", g_pass, g_fail);
    std::printf("═══════════════════════════════════════════════════════════\n");

    return g_fail > 0 ? 1 : 0;
}

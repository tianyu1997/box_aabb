// SafeBoxForest v7 — Smoke tests for P2 (link envelope, voxel, scene SAT).
#include "sbf/core/endpoint_iaabb.h"
#include "sbf/core/robot.h"
#include "sbf/envelope/envelope_type.h"
#include "sbf/envelope/link_iaabb.h"
#include "sbf/scene/collision.h"
#include "sbf/scene/obstacle.h"
#include "sbf/voxel/voxel_grid.h"

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

namespace {

// Build a synthetic endpoint_iaabbs buffer for n_active links.
// Each link i has prox=[i, 0, 0]→[i+1, 1, 1], dist=[i+0.5, 0.2, 0]→[i+1.5, 1.2, 1].
std::vector<float> make_synthetic_endpoints(int n_active) {
    std::vector<float> buf(n_active * 2 * 6);
    for (int i = 0; i < n_active; ++i) {
        float* p = buf.data() + i * 12;
        p[0] = static_cast<float>(i);
        p[1] = 0.0f;
        p[2] = 0.0f;
        p[3] = static_cast<float>(i) + 1.0f;
        p[4] = 1.0f;
        p[5] = 1.0f;
        p[6]  = static_cast<float>(i) + 0.5f;
        p[7]  = 0.2f;
        p[8]  = 0.0f;
        p[9]  = static_cast<float>(i) + 1.5f;
        p[10] = 1.2f;
        p[11] = 1.0f;
    }
    return buf;
}

}  // namespace

// ─── 1. Paired derive: zero-radius hull ──────────────────────────────────────
TEST(P2, DerivePairedZeroNoInflation) {
    auto eps = make_synthetic_endpoints(1);
    float out[6] = {};
    sbf::envelope::derive_link_iaabb_paired_zero(eps.data(), 1, out);

    EXPECT_FLOAT_EQ(out[0], 0.0f);
    EXPECT_FLOAT_EQ(out[1], 0.0f);
    EXPECT_FLOAT_EQ(out[2], 0.0f);
    EXPECT_FLOAT_EQ(out[3], 1.5f);
    EXPECT_FLOAT_EQ(out[4], 1.2f);
    EXPECT_FLOAT_EQ(out[5], 1.0f);
}

// ─── 2. Subdivided derive: t-lerp gives expected mid-point box ───────────────
TEST(P2, DeriveSubdividedMidpoint) {
    // Single link: prox=[0,0,0]→[1,1,1], dist=[2,0,0]→[3,1,1]
    float ep[12] = {
        0, 0, 0, 1, 1, 1,
        2, 0, 0, 3, 1, 1,
    };
    float sub[2 * 6] = {};
    sbf::envelope::derive_link_iaabb_subdivided_zero(ep, 1, 2, sub);

    // s=0: t in [0, 0.5]  → x_lo in [0, 1], x_hi in [1, 2]
    EXPECT_FLOAT_EQ(sub[0], 0.0f);
    EXPECT_FLOAT_EQ(sub[3], 2.0f);
    // s=1: t in [0.5, 1]  → x_lo in [1, 2], x_hi in [2, 3]
    EXPECT_FLOAT_EQ(sub[6], 1.0f);
    EXPECT_FLOAT_EQ(sub[9], 3.0f);
}

// ─── 3. SAT: free-space miss ─────────────────────────────────────────────────
TEST(P2, SatNoCollisionInFreeSpace) {
    const float aabb[6] = {0, 0, 0, 1, 1, 1};
    sbf::scene::Obstacle obs(5, 5, 5, 6, 6, 6);
    float compact[6];
    sbf::scene::pack_obstacles(&obs, 1, compact);

    EXPECT_FALSE(sbf::scene::aabbs_collide_obs(aabb, 1, compact, 1));
}

// ─── 4. SAT: positive hit ────────────────────────────────────────────────────
TEST(P2, SatDetectsCollision) {
    const float aabb[6] = {0, 0, 0, 1, 1, 1};
    sbf::scene::Obstacle obs(0.5f, 0.5f, 0.5f, 2.0f, 2.0f, 2.0f);
    float compact[6];
    sbf::scene::pack_obstacles(&obs, 1, compact);

    EXPECT_TRUE(sbf::scene::aabbs_collide_obs(aabb, 1, compact, 1));
}

// ─── 5. Inflated SAT: D1 path ────────────────────────────────────────────────
TEST(P2, InflatedCollisionDetected) {
    // Zero-radius link: [0,0,0,1,1,1]; obstacle just outside at x=[1.3,2].
    const float zero_aabb[6] = {0, 0, 0, 1, 1, 1};
    sbf::scene::Obstacle obs(1.3f, 1.3f, 1.3f, 2.0f, 2.0f, 2.0f);
    float compact[6];
    sbf::scene::pack_obstacles(&obs, 1, compact);

    // Without inflation: miss.
    EXPECT_FALSE(sbf::scene::aabbs_collide_obs(zero_aabb, 1, compact, 1));

    // With r=0.6 inflation: aabb becomes [-0.6,..1.6] → hit.
    const float r[1] = {0.6f};
    EXPECT_TRUE(sbf::scene::aabbs_collide_obs_inflated(
        zero_aabb, 1, r, compact, 1));

    // With r=0.2 inflation: aabb becomes [-0.2,..1.2] → still miss.
    const float r_small[1] = {0.2f};
    EXPECT_FALSE(sbf::scene::aabbs_collide_obs_inflated(
        zero_aabb, 1, r_small, compact, 1));
}

// ─── 6. Voxel grid: fill + collide ───────────────────────────────────────────
TEST(P2, VoxelGridFillAndCollide) {
    const float a[6] = {0, 0, 0, 1, 1, 1};
    const float b[6] = {0.5f, 0.5f, 0.5f, 1.5f, 1.5f, 1.5f};
    const float c[6] = {3, 3, 3, 4, 4, 4};

    sbf::voxel::SparseVoxelGrid g_a(0.1);
    sbf::voxel::SparseVoxelGrid g_b(0.1);
    sbf::voxel::SparseVoxelGrid g_c(0.1);
    g_a.fill_aabb(a);
    g_b.fill_aabb(b);
    g_c.fill_aabb(c);

    EXPECT_TRUE(g_a.collides(g_b));
    EXPECT_FALSE(g_a.collides(g_c));
    EXPECT_GT(g_a.count_occupied(), 0);
}

// ─── 7. End-to-end: P1 IFK → P2 derive → P2 SAT (IIWA at zero config) ───────
TEST(P2, PipelineEndToEndIIWA) {
    auto robot = sbf::core::Robot::from_json(
        std::string(SBF_DATA_DIR) + "/iiwa14.json");
    ASSERT_GT(robot.n_joints(), 0);

    const int n_dof = robot.n_joints();
    std::vector<sbf::core::Interval> q(n_dof);
    for (int i = 0; i < n_dof; ++i) q[i] = sbf::core::Interval(0.0, 0.0);

    auto res = sbf::core::compute_endpoint_iaabb_ifk(robot, q);
    ASSERT_GT(res.n_active_links, 0);

    sbf::envelope::EnvelopeTypeConfig cfg;
    cfg.type = sbf::envelope::EnvelopeType::LinkIAABB;
    auto env = sbf::envelope::compute_link_envelope(
        res.endpoint_iaabbs.data(), res.n_active_links,
        robot.active_link_radii(), cfg);
    ASSERT_EQ(env.n_active_links, res.n_active_links);
    ASSERT_EQ(static_cast<int>(env.link_iaabbs.size()),
              res.n_active_links * 6);

    // Far obstacle: no collision (even with full radii).
    sbf::scene::Obstacle far_obs(10, 10, 10, 11, 11, 11);
    float far_compact[6];
    sbf::scene::pack_obstacles(&far_obs, 1, far_compact);

    std::vector<float> radii_per_slot(env.n_active_links);
    sbf::scene::expand_radii_to_slots(robot.active_link_radii(),
                                      env.n_active_links, 1,
                                      radii_per_slot.data());

    EXPECT_FALSE(sbf::scene::aabbs_collide_obs_inflated(
        env.link_iaabbs.data(), env.n_active_links,
        radii_per_slot.data(), far_compact, 1));

    // Near obstacle straddling the end-effector at z≈1.4: collision.
    sbf::scene::Obstacle near_obs(-0.3f, -0.3f, 1.3f, 0.3f, 0.3f, 1.5f);
    float near_compact[6];
    sbf::scene::pack_obstacles(&near_obs, 1, near_compact);
    EXPECT_TRUE(sbf::scene::aabbs_collide_obs_inflated(
        env.link_iaabbs.data(), env.n_active_links,
        radii_per_slot.data(), near_compact, 1));
}

// Tests for collision checking
#include <gtest/gtest.h>
#include "sbf/forest/collision.h"
#include "sbf/core/types.h"

using namespace sbf;

TEST(CollisionTest, AABB3DOverlap) {
    float min1[] = {0, 0, 0}, max1[] = {1, 1, 1};
    float min2[] = {0.5f, 0.5f, 0.5f}, max2[] = {1.5f, 1.5f, 1.5f};
    EXPECT_TRUE(aabb_overlap_3d(min1, max1, min2, max2));

    float min3[] = {2, 2, 2}, max3[] = {3, 3, 3};
    EXPECT_FALSE(aabb_overlap_3d(min1, max1, min3, max3));
}

TEST(CollisionTest, AABB3DTouching) {
    float min1[] = {0, 0, 0}, max1[] = {1, 1, 1};
    float min2[] = {1, 0, 0}, max2[] = {2, 1, 1};
    EXPECT_TRUE(aabb_overlap_3d(min1, max1, min2, max2));
}

TEST(CollisionTest, AABBsCollideObsNoCollision) {
    // Single link AABB far from obstacle, stored as [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]
    float link_aabbs[6] = {10, 10, 10, 11, 11, 11};
    // obs_compact: [lo_x, hi_x, lo_y, hi_y, lo_z, hi_z]
    float obs_compact[6] = {-0.5f, 0.5f, -0.5f, 0.5f, -0.5f, 0.5f};
    EXPECT_FALSE(aabbs_collide_obs(link_aabbs, 1, obs_compact, 1));
}

TEST(CollisionTest, AABBsCollideObs) {
    // Link AABB overlapping with obstacle
    float link_aabbs[6] = {-0.1f, -0.1f, -0.1f, 0.1f, 0.1f, 0.1f};
    // obs_compact: [lo_x, hi_x, lo_y, hi_y, lo_z, hi_z]
    float obs_compact[6] = {-0.5f, 0.5f, -0.5f, 0.5f, -0.5f, 0.5f};
    EXPECT_TRUE(aabbs_collide_obs(link_aabbs, 1, obs_compact, 1));
}

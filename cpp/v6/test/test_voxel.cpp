#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <sbf/voxel/bit_brick.h>
#include <sbf/voxel/voxel_grid.h>
#include <sbf/voxel/hull_rasteriser.h>
#include <sbf/envelope/envelope_type.h>
#include <sbf/core/robot.h>
#include <sbf/core/fk_state.h>

#include <cmath>
#include <cstring>
#include <string>

using namespace sbf::voxel;

// ═══════════════════════════════════════════════════════════════════════════
TEST_SUITE("BitBrick") {

TEST_CASE("set and test single voxel") {
    BitBrick b;
    CHECK(b.is_empty());
    b.set(3, 4, 5);
    CHECK(b.test(3, 4, 5));
    CHECK_FALSE(b.test(0, 0, 0));
    CHECK_FALSE(b.test(3, 4, 6));
    CHECK_FALSE(b.is_empty());
}

TEST_CASE("popcount with known fills") {
    BitBrick b;
    for (int i = 0; i < 10; ++i)
        b.set(i % 8, (i / 8) % 8, 0);
    CHECK(b.popcount() == 10);
}

TEST_CASE("clear_voxel") {
    BitBrick b;
    b.set(1, 2, 3);
    CHECK(b.test(1, 2, 3));
    b.clear_voxel(1, 2, 3);
    CHECK_FALSE(b.test(1, 2, 3));
    CHECK(b.is_empty());
}

TEST_CASE("union operator") {
    BitBrick a, b;
    a.set(0, 0, 0);
    b.set(7, 7, 7);
    auto c = a | b;
    CHECK(c.test(0, 0, 0));
    CHECK(c.test(7, 7, 7));
    CHECK(c.popcount() == 2);
}

TEST_CASE("intersects: overlapping") {
    BitBrick a, b;
    a.set(3, 3, 3);
    b.set(3, 3, 3);
    CHECK(a.intersects(b));
}

TEST_CASE("intersects: non-overlapping") {
    BitBrick a, b;
    a.set(0, 0, 0);
    b.set(7, 7, 7);
    CHECK_FALSE(a.intersects(b));
}

TEST_CASE("AND operator") {
    BitBrick a, b;
    a.set(2, 2, 2);
    a.set(3, 3, 3);
    b.set(3, 3, 3);
    b.set(4, 4, 4);
    auto c = a & b;
    CHECK(c.popcount() == 1);
    CHECK(c.test(3, 3, 3));
}

TEST_CASE("full brick popcount == 512") {
    BitBrick b;
    for (int z = 0; z < 8; ++z)
        b.words[z] = ~uint64_t(0);
    CHECK(b.popcount() == 512);
}

}  // TEST_SUITE BitBrick

// ═══════════════════════════════════════════════════════════════════════════
TEST_SUITE("FlatBrickMap") {

TEST_CASE("insert and find") {
    FlatBrickMap map;
    BrickCoord c{1, 2, 3};
    map[c].set(0, 0, 0);
    CHECK(map.size() == 1);

    CHECK(map.contains(c));
    const auto& cmap = map;
    auto it = cmap.find(c);
    CHECK(it.second().test(0, 0, 0));

    CHECK_FALSE(map.contains({99, 99, 99}));
}

TEST_CASE("rehash under high load") {
    FlatBrickMap map;
    // Insert many entries to trigger rehash
    for (int i = 0; i < 200; ++i) {
        BrickCoord c{i, 0, 0};
        map[c].set(0, 0, 0);
    }
    CHECK(map.size() == 200);

    // Verify all entries survive rehash
    for (int i = 0; i < 200; ++i) {
        CHECK(map.contains({i, 0, 0}));
    }
}

TEST_CASE("reserve prevents rehash") {
    FlatBrickMap map;
    map.reserve(500);
    for (int i = 0; i < 300; ++i) {
        map[{i, i, i}].set(1, 1, 1);
    }
    CHECK(map.size() == 300);
}

TEST_CASE("iteration") {
    FlatBrickMap map;
    map[{0, 0, 0}].set(0, 0, 0);
    map[{1, 1, 1}].set(1, 1, 1);
    int count = 0;
    for (auto e : map) {
        (void)e;
        ++count;
    }
    CHECK(count == 2);
}

}  // TEST_SUITE FlatBrickMap

// ═══════════════════════════════════════════════════════════════════════════
TEST_SUITE("SparseVoxelGrid") {

TEST_CASE("fill_aabb: small box") {
    SparseVoxelGrid grid(0.1, 0.0, 0.0, 0.0, 0.0);  // safety_pad = 0
    float aabb[6] = {0.0f, 0.0f, 0.0f, 0.25f, 0.25f, 0.25f};
    grid.fill_aabb(aabb);

    int occ = grid.count_occupied();
    // 0.25/0.1 = 2.5 → cells [0..2] per axis → 3×3×3 = 27
    CHECK(occ == 27);
}

TEST_CASE("fill_aabb: count in expected range") {
    SparseVoxelGrid grid(0.05, 0.0, 0.0, 0.0, 0.0);
    float aabb[6] = {-0.1f, -0.1f, -0.1f, 0.1f, 0.1f, 0.1f};
    grid.fill_aabb(aabb);

    int occ = grid.count_occupied();
    // (0.2/0.05 + 1)³ = 5³ = 125 approximately
    CHECK(occ > 0);
    CHECK(occ <= 216);  // generous upper bound
}

TEST_CASE("merge: union of two grids") {
    SparseVoxelGrid a(0.1, 0.0, 0.0, 0.0, 0.0);
    SparseVoxelGrid b(0.1, 0.0, 0.0, 0.0, 0.0);

    float aabb_a[6] = {0.0f, 0.0f, 0.0f, 0.1f, 0.1f, 0.1f};
    float aabb_b[6] = {1.0f, 1.0f, 1.0f, 1.1f, 1.1f, 1.1f};
    a.fill_aabb(aabb_a);
    b.fill_aabb(aabb_b);

    int na = a.count_occupied();
    int nb = b.count_occupied();

    a.merge(b);
    CHECK(a.count_occupied() >= std::max(na, nb));
    CHECK(a.count_occupied() == na + nb);  // disjoint
}

TEST_CASE("collides: overlapping AABBs") {
    SparseVoxelGrid a(0.1, 0.0, 0.0, 0.0, 0.0);
    SparseVoxelGrid b(0.1, 0.0, 0.0, 0.0, 0.0);

    float aabb_a[6] = {0.0f, 0.0f, 0.0f, 0.5f, 0.5f, 0.5f};
    float aabb_b[6] = {0.3f, 0.3f, 0.3f, 0.8f, 0.8f, 0.8f};
    a.fill_aabb(aabb_a);
    b.fill_aabb(aabb_b);

    CHECK(a.collides(b));
    CHECK(b.collides(a));
}

TEST_CASE("collides: separated AABBs") {
    SparseVoxelGrid a(0.1, 0.0, 0.0, 0.0, 0.0);
    SparseVoxelGrid b(0.1, 0.0, 0.0, 0.0, 0.0);

    float aabb_a[6] = {0.0f, 0.0f, 0.0f, 0.1f, 0.1f, 0.1f};
    float aabb_b[6] = {5.0f, 5.0f, 5.0f, 5.1f, 5.1f, 5.1f};
    a.fill_aabb(aabb_a);
    b.fill_aabb(aabb_b);

    CHECK_FALSE(a.collides(b));
}

TEST_CASE("hull16 occupied <= aabb occupied (tighter)") {
    SparseVoxelGrid hull_grid(0.05, 0.0, 0.0, 0.0, 0.0);
    SparseVoxelGrid aabb_grid(0.05, 0.0, 0.0, 0.0, 0.0);

    // Two slightly different boxes (simulating link motion)
    float prox[6] = {0.0f, 0.0f, 0.0f, 0.1f, 0.1f, 0.1f};
    float dist[6] = {0.3f, 0.0f, 0.0f, 0.4f, 0.1f, 0.1f};
    double radius = 0.02;

    hull_grid.fill_hull16(prox, dist, radius);

    // Compare with full AABB (union of prox, dist, padded by radius + safety)
    double sp = hull_grid.safety_pad();
    float full_aabb[6] = {
        std::min(prox[0], dist[0]) - static_cast<float>(radius + sp),
        std::min(prox[1], dist[1]) - static_cast<float>(radius + sp),
        std::min(prox[2], dist[2]) - static_cast<float>(radius + sp),
        std::max(prox[3], dist[3]) + static_cast<float>(radius + sp),
        std::max(prox[4], dist[4]) + static_cast<float>(radius + sp),
        std::max(prox[5], dist[5]) + static_cast<float>(radius + sp)
    };
    aabb_grid.fill_aabb(full_aabb);

    int hull_occ = hull_grid.count_occupied();
    int aabb_occ = aabb_grid.count_occupied();

    CHECK(hull_occ > 0);
    CHECK(aabb_occ > 0);
    CHECK(hull_occ <= aabb_occ);
}

}  // TEST_SUITE SparseVoxelGrid

// ═══════════════════════════════════════════════════════════════════════════
TEST_SUITE("HullRasteriser") {

TEST_CASE("rasterise_box_obstacle") {
    SparseVoxelGrid grid(0.1, 0.0, 0.0, 0.0, 0.0);
    sbf::Obstacle obs(0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f);
    rasterise_box_obstacle(obs, grid);
    CHECK(grid.count_occupied() > 0);
}

TEST_CASE("rasterise_robot_2dof") {
    // Load 2DOF robot and compute FK to test rasterisation
    std::string path = std::string(SBF_DATA_DIR) + "/2dof_planar.json";
    auto robot = sbf::Robot::from_json(path);
    CHECK(robot.n_joints() == 2);

    // Full FK over a small box
    std::vector<sbf::Interval> intervals = {
        {0.0, 0.3},
        {0.0, 0.3}
    };
    auto fk = sbf::compute_fk_full(robot, intervals);
    CHECK(fk.valid);

    SparseVoxelGrid grid(0.05);
    rasterise_robot_hull16(robot, fk, grid, 4);
    CHECK(grid.count_occupied() > 0);
    CHECK(grid.num_bricks() > 0);
}

TEST_CASE("rasterise_robot_sub_aabbs vs hull16: hull tighter") {
    std::string path = std::string(SBF_DATA_DIR) + "/2dof_planar.json";
    auto robot = sbf::Robot::from_json(path);

    std::vector<sbf::Interval> intervals = {
        {0.0, 0.5},
        {0.0, 0.5}
    };
    auto fk = sbf::compute_fk_full(robot, intervals);

    SparseVoxelGrid hull_grid(0.05);
    SparseVoxelGrid aabb_grid(0.05);

    rasterise_robot_hull16(robot, fk, hull_grid, 8);
    rasterise_robot_sub_aabbs(robot, fk, aabb_grid, 8);

    // Hull-16 should always be <= sub-AABB (tighter or equal)
    CHECK(hull_grid.count_occupied() <= aabb_grid.count_occupied());
}

}  // TEST_SUITE HullRasteriser

// ═══════════════════════════════════════════════════════════════════════════
TEST_SUITE("EnvelopeType_Hull16") {

TEST_CASE("Hull16_Grid compute_link_envelope") {
    // Create synthetic endpoint_iaabbs: 1 link, prox + dist
    float ep[12] = {
        // proximal: [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]
        0.0f, 0.0f, 0.0f, 0.1f, 0.1f, 0.1f,
        // distal:
        0.3f, 0.0f, 0.0f, 0.4f, 0.1f, 0.1f
    };
    double radii[1] = {0.01};

    sbf::EnvelopeTypeConfig config;
    config.type = sbf::EnvelopeType::Hull16_Grid;
    config.grid_config.voxel_delta = 0.02;

    auto env = sbf::compute_link_envelope(ep, 1, radii, config);
    CHECK(env.type == sbf::EnvelopeType::Hull16_Grid);
    CHECK(env.n_active_links == 1);
    CHECK(env.sparse_grid != nullptr);
    CHECK(env.sparse_grid->count_occupied() > 0);

    // link_iaabbs should still be computed
    CHECK(env.link_iaabbs.size() == 6);
}

}  // TEST_SUITE EnvelopeType_Hull16

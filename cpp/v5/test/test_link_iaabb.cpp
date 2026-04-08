// SafeBoxForest v5 — test_link_iaabb (Phase C verification)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <sbf/envelope/link_iaabb.h>
#include <sbf/envelope/link_grid.h>
#include <sbf/envelope/envelope_type.h>
#include <sbf/core/robot.h>
#include <sbf/core/fk_state.h>

#include <cmath>
#include <vector>

using namespace sbf;

// ─── Helper: build 2DOF endpoint iAABBs ────────────────────────────────────
static std::vector<float> make_2dof_endpoint_iaabbs(
    const Robot& robot,
    const std::vector<Interval>& intervals)
{
    FKState state = compute_fk_full(robot, intervals);
    int n_active = robot.n_active_links();
    std::vector<float> ep(n_active * 2 * 6);
    extract_endpoint_iaabbs(state, robot.active_link_map(), n_active, ep.data());
    return ep;
}

// ═══════════════════════════════════════════════════════════════════════════
TEST_SUITE("LinkIAABB_Paired") {

TEST_CASE("2DOF sub=1: output dimension") {
    Robot robot = Robot::from_json("data/2dof_planar.json");
    int n_active = robot.n_active_links();
    std::vector<Interval> ivs = {{-0.3, 0.3}, {-0.2, 0.2}};
    auto ep = make_2dof_endpoint_iaabbs(robot, ivs);

    std::vector<float> link(n_active * 6);
    derive_link_iaabb_paired(ep.data(), n_active, robot.active_link_radii(), link.data());

    CHECK(link.size() == static_cast<size_t>(n_active * 6));
    // lo < hi for each dimension
    for (int ci = 0; ci < n_active; ++ci) {
        for (int d = 0; d < 3; ++d) {
            CHECK(link[ci * 6 + d] < link[ci * 6 + 3 + d]);
        }
    }
}

TEST_CASE("2DOF sub=1: AABB contains all endpoint iAABBs") {
    Robot robot = Robot::from_json("data/2dof_planar.json");
    int n_active = robot.n_active_links();
    std::vector<Interval> ivs = {{-0.5, 0.5}, {-0.5, 0.5}};
    auto ep = make_2dof_endpoint_iaabbs(robot, ivs);

    std::vector<float> link(n_active * 6);
    derive_link_iaabb_paired(ep.data(), n_active, nullptr, link.data());

    for (int ci = 0; ci < n_active; ++ci) {
        const float* prox = ep.data() + (ci * 2) * 6;
        const float* dist = ep.data() + (ci * 2 + 1) * 6;
        for (int d = 0; d < 3; ++d) {
            CHECK(link[ci * 6 + d] <= prox[d] + 1e-6f);
            CHECK(link[ci * 6 + d] <= dist[d] + 1e-6f);
            CHECK(link[ci * 6 + 3 + d] >= prox[3 + d] - 1e-6f);
            CHECK(link[ci * 6 + 3 + d] >= dist[3 + d] - 1e-6f);
        }
    }
}

TEST_CASE("radius inflation correct") {
    Robot robot = Robot::from_json("data/2dof_planar.json");
    int n_active = robot.n_active_links();
    std::vector<Interval> ivs = {{0.0, 0.1}, {0.0, 0.1}};
    auto ep = make_2dof_endpoint_iaabbs(robot, ivs);

    // Without radius
    std::vector<float> no_r(n_active * 6);
    derive_link_iaabb_paired(ep.data(), n_active, nullptr, no_r.data());

    // With radius
    std::vector<float> with_r(n_active * 6);
    derive_link_iaabb_paired(ep.data(), n_active, robot.active_link_radii(), with_r.data());

    const double* radii = robot.active_link_radii();
    for (int ci = 0; ci < n_active; ++ci) {
        float r = static_cast<float>(radii[ci]);
        for (int d = 0; d < 3; ++d) {
            CHECK(with_r[ci * 6 + d] == doctest::Approx(no_r[ci * 6 + d] - r).epsilon(1e-6));
            CHECK(with_r[ci * 6 + 3 + d] == doctest::Approx(no_r[ci * 6 + 3 + d] + r).epsilon(1e-6));
        }
    }
}

}  // TEST_SUITE("LinkIAABB_Paired")

// ═══════════════════════════════════════════════════════════════════════════
TEST_SUITE("LinkIAABB_Subdivided") {

TEST_CASE("2DOF sub=4: output dimension") {
    Robot robot = Robot::from_json("data/2dof_planar.json");
    int n_active = robot.n_active_links();
    int n_sub = 4;
    std::vector<Interval> ivs = {{-0.5, 0.5}, {-0.5, 0.5}};
    auto ep = make_2dof_endpoint_iaabbs(robot, ivs);

    std::vector<float> sub_aabbs(n_active * n_sub * 6);
    derive_link_iaabb_subdivided(ep.data(), n_active, robot.active_link_radii(),
                                 n_sub, sub_aabbs.data());

    CHECK(sub_aabbs.size() == static_cast<size_t>(n_active * n_sub * 6));

    // Each sub-AABB: lo < hi
    for (int i = 0; i < n_active * n_sub; ++i)
        for (int d = 0; d < 3; ++d)
            CHECK(sub_aabbs[i * 6 + d] < sub_aabbs[i * 6 + 3 + d]);
}

TEST_CASE("containment: sub=1 ⊇ union(sub=4)") {
    Robot robot = Robot::from_json("data/2dof_planar.json");
    int n_active = robot.n_active_links();
    int n_sub = 4;
    std::vector<Interval> ivs = {{-0.5, 0.5}, {-0.5, 0.5}};
    auto ep = make_2dof_endpoint_iaabbs(robot, ivs);

    // sub=1 (whole link)
    std::vector<float> whole(n_active * 6);
    derive_link_iaabb_paired(ep.data(), n_active, robot.active_link_radii(), whole.data());

    // sub=4
    std::vector<float> subs(n_active * n_sub * 6);
    derive_link_iaabb_subdivided(ep.data(), n_active, robot.active_link_radii(),
                                 n_sub, subs.data());

    // Union of sub-segments should be contained by sub=1
    for (int ci = 0; ci < n_active; ++ci) {
        for (int d = 0; d < 3; ++d) {
            float union_lo = subs[(ci * n_sub) * 6 + d];
            float union_hi = subs[(ci * n_sub) * 6 + 3 + d];
            for (int s = 1; s < n_sub; ++s) {
                union_lo = std::min(union_lo, subs[(ci * n_sub + s) * 6 + d]);
                union_hi = std::max(union_hi, subs[(ci * n_sub + s) * 6 + 3 + d]);
            }
            CHECK(whole[ci * 6 + d] <= union_lo + 1e-5f);
            CHECK(whole[ci * 6 + 3 + d] >= union_hi - 1e-5f);
        }
    }
}

TEST_CASE("sub=1 via subdivided matches paired") {
    Robot robot = Robot::from_json("data/2dof_planar.json");
    int n_active = robot.n_active_links();
    std::vector<Interval> ivs = {{-0.3, 0.3}, {-0.2, 0.2}};
    auto ep = make_2dof_endpoint_iaabbs(robot, ivs);

    std::vector<float> paired(n_active * 6);
    derive_link_iaabb_paired(ep.data(), n_active, robot.active_link_radii(), paired.data());

    std::vector<float> sub1(n_active * 6);
    derive_link_iaabb_subdivided(ep.data(), n_active, robot.active_link_radii(),
                                 1, sub1.data());

    for (int i = 0; i < n_active * 6; ++i)
        CHECK(sub1[i] == doctest::Approx(paired[i]).epsilon(1e-5));
}

}  // TEST_SUITE("LinkIAABB_Subdivided")

// ═══════════════════════════════════════════════════════════════════════════
TEST_SUITE("VoxelGrid") {

TEST_CASE("rasterize: all link AABBs covered") {
    // Synthetic link iAABBs
    float link_iaabbs[] = {
        0.0f, 0.0f, 0.0f, 0.1f, 0.1f, 0.1f,   // link 0
        0.2f, 0.2f, 0.2f, 0.3f, 0.3f, 0.3f    // link 1
    };
    GridConfig cfg;
    cfg.voxel_delta = 0.05;

    VoxelGrid grid = rasterize_link_iaabbs(link_iaabbs, 2, cfg);

    CHECK(grid.n_occupied() > 0);
    CHECK(grid.delta == doctest::Approx(0.05));

    // Check that voxels at link 0 center are occupied
    double cx = 0.05, cy = 0.05, cz = 0.05;
    int ix = static_cast<int>((cx - grid.origin[0]) / grid.delta);
    int iy = static_cast<int>((cy - grid.origin[1]) / grid.delta);
    int iz = static_cast<int>((cz - grid.origin[2]) / grid.delta);
    CHECK(grid.occupied(ix, iy, iz));
}

TEST_CASE("empty scene: 0 occupied voxels") {
    GridConfig cfg;
    cfg.voxel_delta = 0.05;
    VoxelGrid grid = rasterize_link_iaabbs(nullptr, 0, cfg);
    CHECK(grid.n_occupied() == 0);
}

TEST_CASE("n_occupied monotonic with n_active") {
    float link1[] = { 0.0f, 0.0f, 0.0f, 0.1f, 0.1f, 0.1f };
    float link2[] = { 0.0f, 0.0f, 0.0f, 0.1f, 0.1f, 0.1f,
                      0.5f, 0.5f, 0.5f, 0.6f, 0.6f, 0.6f };
    GridConfig cfg;
    cfg.voxel_delta = 0.05;

    VoxelGrid g1 = rasterize_link_iaabbs(link1, 1, cfg);
    VoxelGrid g2 = rasterize_link_iaabbs(link2, 2, cfg);

    CHECK(g2.n_occupied() >= g1.n_occupied());
}

}  // TEST_SUITE("VoxelGrid")

// ═══════════════════════════════════════════════════════════════════════════
TEST_SUITE("Unified_LinkEnvelope") {

TEST_CASE("LinkIAABB: grid == nullopt") {
    Robot robot = Robot::from_json("data/2dof_planar.json");
    int n_active = robot.n_active_links();
    std::vector<Interval> ivs = {{-0.3, 0.3}, {-0.2, 0.2}};
    auto ep = make_2dof_endpoint_iaabbs(robot, ivs);

    EnvelopeTypeConfig cfg;
    cfg.type = EnvelopeType::LinkIAABB;
    cfg.n_subdivisions = 1;

    LinkEnvelope env = compute_link_envelope(
        ep.data(), n_active, robot.active_link_radii(), cfg);

    CHECK(env.type == EnvelopeType::LinkIAABB);
    CHECK(env.n_active_links == n_active);
    CHECK(env.link_iaabbs.size() == static_cast<size_t>(n_active * 6));
    CHECK_FALSE(env.has_grid());
}

TEST_CASE("LinkIAABB_Grid: grid != nullopt") {
    Robot robot = Robot::from_json("data/2dof_planar.json");
    int n_active = robot.n_active_links();
    std::vector<Interval> ivs = {{-0.3, 0.3}, {-0.2, 0.2}};
    auto ep = make_2dof_endpoint_iaabbs(robot, ivs);

    EnvelopeTypeConfig cfg;
    cfg.type = EnvelopeType::LinkIAABB_Grid;
    cfg.grid_config.voxel_delta = 0.05;

    LinkEnvelope env = compute_link_envelope(
        ep.data(), n_active, robot.active_link_radii(), cfg);

    CHECK(env.type == EnvelopeType::LinkIAABB_Grid);
    CHECK(env.link_iaabbs.size() == static_cast<size_t>(n_active * 6));
    CHECK(env.has_grid());
    if (env.has_grid()) {
        CHECK(env.sparse_grid->count_occupied() > 0);
    }
}

}  // TEST_SUITE("Unified_LinkEnvelope")

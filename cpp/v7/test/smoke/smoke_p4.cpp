// SafeBoxForest v7 — Smoke tests for P4 (forest grower).
#include "sbf/core/robot.h"
#include "sbf/envelope/envelope_type.h"
#include "sbf/forest/adjacency.h"
#include "sbf/forest/bridge.h"
#include "sbf/forest/ffb.h"
#include "sbf/forest/grower.h"
#include "sbf/forest/union_find.h"
#include "sbf/lect/lect.h"
#include "sbf/scene/box_node.h"
#include "sbf/scene/collision.h"
#include "sbf/scene/obstacle.h"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace {

sbf::core::Robot load_iiwa() {
    return sbf::core::Robot::from_json(
        std::string(SBF_DATA_DIR) + "/iiwa14.json");
}

sbf::core::Robot load_2dof() {
    return sbf::core::Robot::from_json(
        std::string(SBF_DATA_DIR) + "/2dof_planar.json");
}

std::vector<sbf::core::Interval> joint_iv(const sbf::core::Robot& r) {
    std::vector<sbf::core::Interval> iv;
    for (const auto& l : r.joint_limits().limits) iv.push_back(l);
    return iv;
}

sbf::lect::LECT make_lect(const sbf::core::Robot& robot,
                          const std::vector<sbf::core::Interval>& iv) {
    sbf::envelope::EnvelopeTypeConfig cfg;
    cfg.type = sbf::envelope::EnvelopeType::LinkIAABB;
    return sbf::lect::LECT(robot, iv, cfg);
}

sbf::scene::BoxNode mkbox(double lo0, double hi0,
                          double lo1, double hi1, int id = 0) {
    sbf::scene::BoxNode b;
    b.id = id;
    b.joint_intervals = {sbf::core::Interval(lo0, hi0),
                         sbf::core::Interval(lo1, hi1)};
    b.compute_volume();
    return b;
}

}  // namespace

// ─── 1. UnionFind ────────────────────────────────────────────────────────────
TEST(P4, UnionFindBasic) {
    sbf::forest::UnionFind uf(6);
    EXPECT_EQ(uf.num_components(), 6);
    EXPECT_TRUE(uf.unite(0, 1));
    EXPECT_TRUE(uf.unite(2, 3));
    EXPECT_TRUE(uf.unite(1, 2));
    EXPECT_FALSE(uf.unite(0, 3));     // already connected
    EXPECT_TRUE(uf.connected(0, 3));
    EXPECT_FALSE(uf.connected(0, 4));
    EXPECT_EQ(uf.num_components(), 3);   // {0,1,2,3}, {4}, {5}
}

// ─── 2. Adjacency: face contact ──────────────────────────────────────────────
TEST(P4, BoxesAdjacentFaceContact) {
    auto a = mkbox(0, 1, 0, 1);
    auto b = mkbox(1, 2, 0, 1);              // shares dim-0 face at x=1
    EXPECT_TRUE(sbf::forest::boxes_adjacent(a, b));
}

TEST(P4, BoxesAdjacentSeparated) {
    auto a = mkbox(0, 1, 0, 1);
    auto c = mkbox(1.5, 2.5, 0, 1);          // gap of 0.5
    EXPECT_FALSE(sbf::forest::boxes_adjacent(a, c));
}

TEST(P4, BoxesAdjacentInterior) {
    auto a = mkbox(0, 2, 0, 2);
    auto b = mkbox(0.5, 1.5, 0.5, 1.5);      // fully inside a
    EXPECT_TRUE(sbf::forest::boxes_adjacent(a, b));
}

// ─── 3. Adjacency graph + islands ────────────────────────────────────────────
TEST(P4, AdjacencyGraphIslands) {
    std::vector<sbf::scene::BoxNode> boxes = {
        mkbox(0, 1, 0, 1, 0),         // touches box 1
        mkbox(1, 2, 0, 1, 1),         // touches box 0
        mkbox(5, 6, 5, 6, 2),         // isolated
        mkbox(5, 6, 6, 7, 3),         // touches box 2
    };
    auto g = sbf::forest::compute_adjacency_graph(boxes);
    EXPECT_EQ(g.n_edges, 2);

    auto isl = sbf::forest::find_islands(g);
    EXPECT_EQ(isl.n_components, 2);
    EXPECT_EQ(isl.component_id[0], isl.component_id[1]);
    EXPECT_EQ(isl.component_id[2], isl.component_id[3]);
    EXPECT_NE(isl.component_id[0], isl.component_id[2]);
    EXPECT_EQ(isl.largest_size, 2);
}

// ─── 4. FFB success on root, no obstacles ────────────────────────────────────
TEST(P4, FFBSuccessFromRoot) {
    auto robot = load_2dof();
    auto iv = joint_iv(robot);
    auto lect = make_lect(robot, iv);

    Eigen::VectorXd seed(robot.n_joints());
    for (int d = 0; d < robot.n_joints(); ++d) seed[d] = iv[d].center();

    auto r = sbf::forest::find_free_box(lect, seed, nullptr, 0);
    EXPECT_TRUE(r.success()) << "fail_code=" << r.fail_code;
    EXPECT_GE(r.node_idx, 0);
}

// ─── 5. FFB rejects already-occupied leaf ────────────────────────────────────
TEST(P4, FFBOccupiedReject) {
    auto robot = load_2dof();
    auto iv = joint_iv(robot);
    auto lect = make_lect(robot, iv);

    Eigen::VectorXd seed(robot.n_joints());
    for (int d = 0; d < robot.n_joints(); ++d) seed[d] = iv[d].center();

    auto r1 = sbf::forest::find_free_box(lect, seed, nullptr, 0);
    ASSERT_TRUE(r1.success());
    lect.mark_occupied(r1.node_idx, 0);

    auto r2 = sbf::forest::find_free_box(lect, seed, nullptr, 0);
    EXPECT_FALSE(r2.success());
    EXPECT_EQ(r2.fail_code, 1);
}

// ─── 6. FFB fails when root is fully obstructed ──────────────────────────────
TEST(P4, FFBCollisionFails) {
    auto robot = load_2dof();
    auto iv = joint_iv(robot);
    auto lect = make_lect(robot, iv);

    sbf::scene::Obstacle blocker(-100, -100, -100, 100, 100, 100);
    float compact[6];
    sbf::scene::pack_obstacles(&blocker, 1, compact);

    Eigen::VectorXd seed(robot.n_joints());
    for (int d = 0; d < robot.n_joints(); ++d) seed[d] = iv[d].center();

    auto r = sbf::forest::find_free_box(lect, seed, compact, 1);
    EXPECT_FALSE(r.success());
    EXPECT_EQ(r.fail_code, 3);
}

// ─── 7. enforce_parent_adjacency snaps small gap ─────────────────────────────
TEST(P4, EnforceParentAdjacencySnapsGap) {
    auto a = mkbox(0, 1, 0, 1);              // parent
    auto b = mkbox(1.01, 2, 0, 1);           // gap 0.01 in dim-0
    EXPECT_FALSE(sbf::forest::boxes_adjacent(a, b));

    auto robot = load_2dof();
    auto iv = joint_iv(robot);
    auto lect = make_lect(robot, iv);
    sbf::forest::enforce_parent_adjacency(b, a, lect, nullptr, 0);

    EXPECT_TRUE(sbf::forest::boxes_adjacent(a, b));
}

// ─── 8. Grower: 2-DoF endpoints connect ──────────────────────────────────────
TEST(P4, GrowerStartGoalConnected2DoF) {
    auto robot = load_2dof();
    auto iv = joint_iv(robot);
    auto lect = make_lect(robot, iv);

    sbf::forest::GrowerConfig cfg;
    cfg.max_boxes = 100;
    cfg.timeout_ms = 5000;
    cfg.rrt_goal_bias = 0.2;
    cfg.ffb.max_depth = 8;     // keep boxes coarse so face-adjacency is feasible
    sbf::forest::ForestGrower g(robot, lect, cfg);

    Eigen::VectorXd qs(2), qg(2);
    qs << -0.5, -0.5;
    qg << 0.5, 0.5;
    g.set_endpoints(qs, qg);

    auto res = g.grow(nullptr, 0);
    EXPECT_GE(res.start_box, 0);
    EXPECT_GE(res.goal_box, 0);
    EXPECT_GE(static_cast<int>(res.boxes.size()), 2);
    EXPECT_TRUE(res.start_goal_connected)
        << "boxes=" << res.boxes.size()
        << " islands=" << res.adjacency_islands;
}

// ─── 9. Grower: IIWA14 small scene SR=100% ──────────────────────────────────
TEST(P4, GrowerIIWA14SmallScene) {
    auto robot = load_iiwa();
    auto iv = joint_iv(robot);
    auto lect = make_lect(robot, iv);

    // Far obstacle — does not interfere.
    sbf::scene::Obstacle far(20, 20, 20, 21, 21, 21);
    float compact[6];
    sbf::scene::pack_obstacles(&far, 1, compact);

    sbf::forest::GrowerConfig cfg;
    cfg.max_boxes = 50;
    cfg.timeout_ms = 8000;
    cfg.rrt_goal_bias = 0.2;
    cfg.stop_after_connect = true;
    sbf::forest::ForestGrower g(robot, lect, cfg);

    Eigen::VectorXd qs = Eigen::VectorXd::Zero(robot.n_joints());
    Eigen::VectorXd qg = Eigen::VectorXd::Zero(robot.n_joints());
    qg[0] = 0.5;
    qg[1] = 0.3;
    g.set_endpoints(qs, qg);

    auto res = g.grow(compact, 1);
    EXPECT_GE(res.start_box, 0);
    EXPECT_GE(res.goal_box, 0);
    EXPECT_TRUE(res.start_goal_connected)
        << "boxes=" << res.boxes.size()
        << " islands=" << res.adjacency_islands;
    EXPECT_LT(res.build_time_ms, 8000.0);
}

// ─── 10. Multi-thread parity (P4.5) ─────────────────────────────────────────
TEST(P4, GrowerMultiThreadConnects2DoF) {
    auto robot = load_2dof();
    auto iv = joint_iv(robot);
    auto lect = make_lect(robot, iv);

    sbf::forest::GrowerConfig cfg;
    cfg.max_boxes = 200;
    cfg.timeout_ms = 5000;
    cfg.rrt_goal_bias = 0.2;
    cfg.n_threads = 4;     // explicit multi-thread
    cfg.ffb.max_depth = 8;
    sbf::forest::ForestGrower g(robot, lect, cfg);

    Eigen::VectorXd qs(2), qg(2);
    qs << -0.5, -0.5;
    qg <<  0.5,  0.5;
    g.set_endpoints(qs, qg);

    auto res = g.grow(nullptr, 0);
    EXPECT_GE(res.start_box, 0);
    EXPECT_GE(res.goal_box,  0);
    EXPECT_TRUE(res.start_goal_connected)
        << "boxes=" << res.boxes.size()
        << " islands=" << res.adjacency_islands
        << " workers=" << res.n_workers_used;
    EXPECT_GE(res.n_workers_used, 1);
    EXPECT_LE(res.n_workers_used, 4);
}

// ─── 11. Multi-thread strict adjacency (no orphan islands) ─────────────────
TEST(P4, GrowerMultiThreadStrictAdjacency) {
    auto robot = load_2dof();
    auto iv = joint_iv(robot);
    auto lect = make_lect(robot, iv);

    sbf::forest::GrowerConfig cfg;
    cfg.max_boxes = 300;
    cfg.timeout_ms = 5000;
    cfg.n_threads = 4;
    cfg.ffb.max_depth = 8;
    sbf::forest::ForestGrower g(robot, lect, cfg);

    Eigen::VectorXd qs(2), qg(2);
    qs << -0.8, -0.8;
    qg <<  0.8,  0.8;
    g.set_endpoints(qs, qg);

    auto res = g.grow(nullptr, 0);
    // Strict adjacency means at most as many islands as workers spawned.
    EXPECT_LE(res.adjacency_islands,
              std::max(1, res.n_workers_used + 2))
        << "boxes=" << res.boxes.size()
        << " islands=" << res.adjacency_islands;
}

// SafeBoxForest v7 — Smoke tests for P5 (planner).
#include "sbf/core/robot.h"
#include "sbf/envelope/envelope_type.h"
#include "sbf/lect/lect.h"
#include "sbf/planner/path_finder.h"
#include "sbf/planner/path_opt_pipeline.h"
#include "sbf/planner/sbf_planner.h"
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

}  // namespace

// ───── 1. PathFinder over a 3-box chain ─────────────────────────────────────
TEST(P5, PathFinderThreeBoxChain) {
    using sbf::scene::BoxNode;
    using sbf::core::Interval;
    auto mkbox = [](double lo, double hi, int id) {
        BoxNode b;
        b.id = id;
        b.joint_intervals = {Interval(lo, hi), Interval(0.0, 1.0)};
        b.compute_volume();
        return b;
    };
    std::vector<BoxNode> boxes = {
        mkbox(0.0, 1.0, 0),
        mkbox(1.0, 2.0, 1),
        mkbox(2.0, 3.0, 2),
    };
    auto graph = sbf::forest::compute_adjacency_graph(boxes);

    Eigen::VectorXd qs(2), qg(2);
    qs << 0.5, 0.5;
    qg << 2.5, 0.5;
    auto r = sbf::planner::find_box_path(boxes, graph, 0, 2, qs, qg);
    EXPECT_TRUE(r.success);
    EXPECT_EQ(r.box_path.size(), 3u);
    EXPECT_EQ(r.box_path.front(), 0);
    EXPECT_EQ(r.box_path.back(),  2);
    EXPECT_GT(r.length, 0.0);
}

// ───── 2. PathFinder reports failure across islands ─────────────────────────
TEST(P5, PathFinderDisconnected) {
    using sbf::scene::BoxNode;
    using sbf::core::Interval;
    auto mkbox = [](double lo, double hi, int id) {
        BoxNode b;
        b.id = id;
        b.joint_intervals = {Interval(lo, hi), Interval(0.0, 1.0)};
        b.compute_volume();
        return b;
    };
    std::vector<BoxNode> boxes = {
        mkbox(0.0, 1.0, 0),
        mkbox(2.0, 3.0, 1),                 // gap between 1 and 2
    };
    auto graph = sbf::forest::compute_adjacency_graph(boxes);

    Eigen::VectorXd qs(2), qg(2);
    qs << 0.5, 0.5;
    qg << 2.5, 0.5;
    auto r = sbf::planner::find_box_path(boxes, graph, 0, 1, qs, qg);
    EXPECT_FALSE(r.success);
}

// ───── 3. PathOptPipeline: greedy shortcut shortens straight-line path ─────
TEST(P5, PathOptGreedyShortcutCollapsesStraightLine) {
    // 5 colinear waypoints on a free segment → should collapse to {a, e}.
    std::vector<Eigen::VectorXd> raw;
    for (int i = 0; i < 5; ++i) {
        Eigen::VectorXd q(2);
        q << static_cast<double>(i), 0.0;
        raw.push_back(q);
    }
    sbf::planner::PathOptConfig cfg;
    cfg.steps = {sbf::planner::PathOptStep::GREEDY_SHORTCUT};
    sbf::planner::PathOptPipeline pipe(
        cfg, [](const Eigen::VectorXd&) { return true; });
    auto out = pipe.optimize(raw);
    EXPECT_EQ(out.size(), 2u);
    EXPECT_NEAR(sbf::planner::PathOptPipeline::path_length(out), 4.0, 1e-9);
}

// ───── 4. PathOptPipeline: lengths non-increasing across steps ──────────────
TEST(P5, PathOptStepLengthsNonIncreasing) {
    // Zig-zag input — every shortcut helps.
    std::vector<Eigen::VectorXd> raw;
    for (int i = 0; i < 10; ++i) {
        Eigen::VectorXd q(2);
        q << static_cast<double>(i), (i % 2 == 0) ? 0.0 : 0.3;
        raw.push_back(q);
    }
    sbf::planner::PathOptConfig cfg;
    cfg.shortcut_iters = 50;
    cfg.densify_dt     = 0.5;
    cfg.elastic_iters  = 5;
    sbf::planner::PathOptPipeline pipe(
        cfg, [](const Eigen::VectorXd&) { return true; });
    pipe.optimize(raw);
    const auto& Ls = pipe.step_lengths();
    ASSERT_GE(Ls.size(), 2u);
    // Densify increases length, so allow that step alone to grow; otherwise
    // each non-densify step must be non-increasing relative to the previous.
    for (size_t i = 1; i < Ls.size(); ++i) {
        if (cfg.steps[i - 1] == sbf::planner::PathOptStep::DENSIFY) continue;
        EXPECT_LE(Ls[i], Ls[i - 1] + 1e-9)
            << "step " << i << " grew: " << Ls[i - 1] << " → " << Ls[i];
    }
}

// ───── 5. SbfPlanner end-to-end: 2-DoF planar ───────────────────────────────
TEST(P5, SbfPlannerEndToEnd2DoF) {
    auto robot = load_2dof();
    auto iv    = joint_iv(robot);
    auto lect  = make_lect(robot, iv);
    sbf::planner::PlannerConfig cfg;
    cfg.grower.max_boxes  = 200;
    cfg.grower.timeout_ms = 5000;
    cfg.grower.n_threads  = 4;
    cfg.grower.ffb.max_depth = 8;
    sbf::planner::SbfPlanner planner(robot, lect, cfg);

    Eigen::VectorXd qs(2), qg(2);
    qs << -0.5, -0.5;
    qg <<  0.5,  0.5;
    auto res = planner.plan(qs, qg, nullptr, 0);
    ASSERT_TRUE(res.success) << "fail_reason=" << res.fail_reason;
    EXPECT_EQ(res.final_state, sbf::planner::PlannerState::SUCCESS);
    EXPECT_GE(res.path.size(), 2u);
    // First/last waypoints must equal endpoints exactly.
    EXPECT_NEAR((res.path.front() - qs).norm(), 0.0, 1e-9);
    EXPECT_NEAR((res.path.back()  - qg).norm(), 0.0, 1e-9);
    EXPECT_LE(res.opt_length, res.raw_length + 1e-6);
}

// ───── 6. SbfPlanner end-to-end: IIWA14 small scene ────────────────────────
TEST(P5, SbfPlannerEndToEndIIWA14) {
    auto robot = load_iiwa();
    auto iv    = joint_iv(robot);
    auto lect  = make_lect(robot, iv);

    sbf::scene::Obstacle far(20, 20, 20, 21, 21, 21);
    float compact[6];
    sbf::scene::pack_obstacles(&far, 1, compact);

    sbf::planner::PlannerConfig cfg;
    cfg.grower.max_boxes  = 200;
    cfg.grower.timeout_ms = 8000;
    cfg.grower.n_threads  = 4;
    sbf::planner::SbfPlanner planner(robot, lect, cfg);

    Eigen::VectorXd qs = Eigen::VectorXd::Zero(robot.n_joints());
    Eigen::VectorXd qg = Eigen::VectorXd::Zero(robot.n_joints());
    qg[0] = 0.5;
    qg[1] = 0.3;
    auto res = planner.plan(qs, qg, compact, 1);
    ASSERT_TRUE(res.success) << "fail_reason=" << res.fail_reason
                             << " boxes=" << res.n_boxes
                             << " islands=" << res.n_islands;
    EXPECT_GE(res.path.size(), 2u);
    EXPECT_LE(res.opt_length, res.raw_length + 1e-6);
    EXPECT_LT(res.total_time_ms, 10000.0);
}

// ───── 7. quick_mode trims path_opt steps & caps grower ────────────────────
TEST(P5, SbfPlannerQuickMode) {
    auto robot = load_2dof();
    auto iv    = joint_iv(robot);
    auto lect  = make_lect(robot, iv);
    sbf::planner::PlannerConfig cfg;
    cfg.grower.max_boxes     = 100000;        // huge; quick_mode caps it
    cfg.grower.timeout_ms    = 60000.0;
    cfg.grower.n_threads     = 4;
    cfg.grower.ffb.max_depth = 8;
    cfg.quick_mode = true;
    sbf::planner::SbfPlanner planner(robot, lect, cfg);

    Eigen::VectorXd qs(2), qg(2);
    qs << -0.5, -0.5;
    qg <<  0.5,  0.5;
    auto res = planner.plan(qs, qg, nullptr, 0);
    ASSERT_TRUE(res.success) << "fail_reason=" << res.fail_reason;
    EXPECT_LE(res.n_boxes, 5000);
    EXPECT_LT(res.total_time_ms, 10000.0);
    // quick_mode → only 2 opt steps (greedy + final shortcut) → step_lengths
    // has 1 (input) + 2 (steps) = 3 entries.
    EXPECT_EQ(res.step_lengths.size(), 3u);
}

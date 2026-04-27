// SafeBoxForest v7 — Smoke tests for P7 (baselines).
//
// Verifies the baseline_collision helper (zero-width interval FK + LinkIAABB)
// agrees with the SBF planner's own free-config criterion. The OMPL and
// Drake baselines themselves are exercised by run_compare_all.sh — this
// smoke layer just guards the shared C++ helper used by baseline_ompl.
#include "experiments/baseline_collision.h"
#include "experiments/common.h"

#include "sbf/scene/scene_config.h"

#include <gtest/gtest.h>

#include <string>

namespace {

std::string cfg_path(const std::string& fname) {
    return std::string(SBF_V7_ROOT) + "/experiments/configs/" + fname;
}

}  // namespace

// ───── 1. QFreeChecker: q_start of iiwa14_far is free ─────────────────────
TEST(P7, QFreeChecker_IIWA14Far_StartFree) {
    auto sc = sbf::scene::load_scene_json(cfg_path("iiwa14_far.json"));
    auto robot = sbf::core::Robot::from_json(
        sbf::exp::robot_json_path(sc.robot));
    auto packed = sc.packed_obstacles();
    sbf::exp::QFreeChecker chk(robot, packed.data(),
                               static_cast<int>(sc.obstacles.size()));
    EXPECT_TRUE(chk.is_free(sc.q_start));
    EXPECT_TRUE(chk.is_free(sc.q_goal));
}

// ───── 2. QFreeChecker: a config inside the obstacle is reported colliding ─
TEST(P7, QFreeChecker_ConfigInsideObstacleColliding) {
    // Use a 2DoF planar robot so we can construct an obstacle right at the
    // end-effector position for q=0.
    auto sc = sbf::scene::load_scene_json(cfg_path("2dof_box.json"));
    auto robot = sbf::core::Robot::from_json(
        sbf::exp::robot_json_path(sc.robot));
    // Obstacle that swallows the end-effector at q=(0, 0): a giant box at the
    // origin. The 2-DoF arm has unit links → end-effector at q=0 is at (~2, 0).
    sbf::scene::Obstacle big(-3.0f, -3.0f, -1.0f, 3.0f, 3.0f, 1.0f);
    float compact[6];
    sbf::scene::pack_obstacles(&big, 1, compact);
    sbf::exp::QFreeChecker chk(robot, compact, 1);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(robot.n_joints());
    EXPECT_FALSE(chk.is_free(q));
}

// ───── 3. QFreeChecker::segment_free agrees with endpoint check ────────────
TEST(P7, QFreeChecker_SegmentFreeIIWA14Far) {
    auto sc = sbf::scene::load_scene_json(cfg_path("iiwa14_far.json"));
    auto robot = sbf::core::Robot::from_json(
        sbf::exp::robot_json_path(sc.robot));
    auto packed = sc.packed_obstacles();
    sbf::exp::QFreeChecker chk(robot, packed.data(),
                               static_cast<int>(sc.obstacles.size()));
    EXPECT_TRUE(chk.segment_free(sc.q_start, sc.q_goal, /*dt=*/0.05));
}

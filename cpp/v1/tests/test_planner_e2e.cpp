// End-to-end planner test with a simple 2D robot
#include <gtest/gtest.h>
#include "sbf/planner/sbf_planner.h"
#include "sbf/core/types.h"
#include "sbf/core/config.h"
#include <cmath>

using namespace sbf;

class PlannerE2ETest : public ::testing::Test {
protected:
    void SetUp() override {
        // Simple 2-DOF planar robot
        std::vector<DHParam> dhs = {{0.0, 0.5, 0.0, 0.0, 0}, {0.0, 0.5, 0.0, 0.0, 0}};
        JointLimits jl;
        jl.limits = {{-2.0, 2.0}, {-2.0, 2.0}};
        robot_ = Robot("test_2dof", dhs, jl);

        // No obstacles: trivial environment
        obstacles_.clear();

        cfg_ = SBFConfig();
        cfg_.max_boxes = 100;
        cfg_.ffb_max_depth = 20;
        cfg_.ffb_min_edge = 0.1;
        cfg_.max_consecutive_miss = 50;
        cfg_.seed = 42;
    }

    Robot robot_;
    std::vector<Obstacle> obstacles_;
    SBFConfig cfg_;
};

TEST_F(PlannerE2ETest, DirectPath) {
    // Start and goal close together with no obstacles — should find direct
    Eigen::VectorXd start(2), goal(2);
    start << 0.0, 0.0;
    goal << 0.1, 0.1;

    SBFPlanner planner(robot_, obstacles_, cfg_);
    auto result = planner.plan(start, goal, 5.0);

    EXPECT_TRUE(result.success);
    EXPECT_GT(result.path.rows(), 0);
}

TEST_F(PlannerE2ETest, PlanWithForest) {
    // Start and goal far apart
    Eigen::VectorXd start(2), goal(2);
    start << -1.5, -1.5;
    goal << 1.5, 1.5;

    SBFPlanner planner(robot_, obstacles_, cfg_);
    auto result = planner.plan(start, goal, 10.0);

    EXPECT_TRUE(result.success);
    EXPECT_GT(result.path.rows(), 1);
    EXPECT_GT(result.cost, 0.0);
}

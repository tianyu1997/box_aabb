// Tests for interval FK
#include <gtest/gtest.h>
#include "sbf/core/types.h"
#include "sbf/core/robot.h"
#include "sbf/aabb/interval_fk.h"
#include "sbf/aabb/fk_scalar.h"
#include <cmath>

using namespace sbf;

class IntervalFKTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Build a simple 2-DOF planar robot for testing
        std::vector<DHParam> dhs = {{0.0, 1.0, 0.0, 0.0, 0}, {0.0, 1.0, 0.0, 0.0, 0}};
        JointLimits jl;
        jl.limits = {{-M_PI, M_PI}, {-M_PI, M_PI}};
        robot_2dof_ = Robot("test_2dof", dhs, jl);
    }

    Robot robot_2dof_;
};

TEST_F(IntervalFKTest, PointInterval) {
    // With point intervals (lo == hi), interval FK should produce finite AABBs.
    // Note: Link AABBs bound the rigid body from frame start to frame end,
    // so even with point intervals the AABB has nonzero size.
    Eigen::VectorXd q(2);
    q << 0.3, 0.5;

    std::vector<Interval> ivs(2);
    ivs[0] = {q[0], q[0]};
    ivs[1] = {q[1], q[1]};

    FKState fk_state = compute_fk_full(robot_2dof_, ivs);

    int n_active = robot_2dof_.n_active_links();
    std::vector<float> aabbs(n_active * 6);
    extract_link_aabbs(fk_state, robot_2dof_.active_link_map(),
                        n_active, aabbs.data());

    // For point intervals, AABBs should be finite and reasonable
    for (int link = 0; link < n_active; ++link) {
        float x_lo = aabbs[link * 6 + 0];
        float x_hi = aabbs[link * 6 + 3];
        float y_lo = aabbs[link * 6 + 1];
        float y_hi = aabbs[link * 6 + 4];
        // AABB should have lo <= hi
        EXPECT_LE(x_lo, x_hi) << "Link " << link << " x_lo > x_hi";
        EXPECT_LE(y_lo, y_hi) << "Link " << link << " y_lo > y_hi";
        // For a robot with a=1 links, max AABB extent is ~2 (a link length)
        EXPECT_LE(x_hi - x_lo, 3.0) << "Link " << link << " x extent too large";
        EXPECT_LE(y_hi - y_lo, 3.0) << "Link " << link << " y extent too large";
    }
}

TEST_F(IntervalFKTest, MonotonicGrowth) {
    // Wider joint intervals should produce wider AABBs
    std::vector<Interval> narrow(2);
    narrow[0] = {0.0, 0.1};
    narrow[1] = {0.0, 0.1};

    std::vector<Interval> wide(2);
    wide[0] = {-0.5, 0.5};
    wide[1] = {-0.5, 0.5};

    FKState fk_narrow = compute_fk_full(robot_2dof_, narrow);
    FKState fk_wide = compute_fk_full(robot_2dof_, wide);

    int n_active = robot_2dof_.n_active_links();
    std::vector<float> aabb_n(n_active * 6), aabb_w(n_active * 6);
    extract_link_aabbs(fk_narrow, robot_2dof_.active_link_map(),
                        n_active, aabb_n.data());
    extract_link_aabbs(fk_wide, robot_2dof_.active_link_map(),
                        n_active, aabb_w.data());

    // Wide AABB should encompass narrow
    for (int i = 0; i < n_active; ++i) {
        EXPECT_LE(aabb_w[i * 6 + 0], aabb_n[i * 6 + 0] + 1e-6);  // lo_x
        EXPECT_GE(aabb_w[i * 6 + 3], aabb_n[i * 6 + 3] - 1e-6);  // hi_x
    }
}

// Tests for SafeBoxForest
#include <gtest/gtest.h>
#include <algorithm>
#include "sbf/forest/safe_box_forest.h"
#include "sbf/core/types.h"

using namespace sbf;

class SafeBoxForestTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 2-DOF joint limits [-pi, pi]
        jl_.limits.resize(2);
        jl_.limits[0] = {-3.14, 3.14};
        jl_.limits[1] = {-3.14, 3.14};
        forest_ = SafeBoxForest(2, jl_);
    }

    JointLimits jl_;
    SafeBoxForest forest_;
};

TEST_F(SafeBoxForestTest, AddBox) {
    std::vector<Interval> ivs = {{0.0, 1.0}, {0.0, 1.0}};
    Eigen::VectorXd seed(2);
    seed << 0.5, 0.5;
    BoxNode box(0, ivs, seed);
    forest_.add_box_no_adjacency(box);
    EXPECT_EQ(forest_.n_boxes(), 1);
}

TEST_F(SafeBoxForestTest, FindContaining) {
    std::vector<Interval> ivs = {{0.0, 1.0}, {0.0, 1.0}};
    Eigen::VectorXd seed(2);
    seed << 0.5, 0.5;
    BoxNode box(0, ivs, seed);
    forest_.add_box_no_adjacency(box);

    Eigen::VectorXd q_in(2), q_out(2);
    q_in << 0.5, 0.5;
    q_out << 2.0, 2.0;

    auto found_in = forest_.find_containing(q_in);
    auto found_out = forest_.find_containing(q_out);
    EXPECT_TRUE(found_in != nullptr);
    EXPECT_TRUE(found_out == nullptr);
}

TEST_F(SafeBoxForestTest, Adjacency) {
    // Two touching boxes
    std::vector<Interval> ivs1 = {{0.0, 1.0}, {0.0, 1.0}};
    std::vector<Interval> ivs2 = {{1.0, 2.0}, {0.0, 1.0}};  // touching at dim 0
    Eigen::VectorXd s1(2), s2(2);
    s1 << 0.5, 0.5;
    s2 << 1.5, 0.5;

    BoxNode b1(0, ivs1, s1);
    BoxNode b2(1, ivs2, s2);
    forest_.add_box_no_adjacency(b1);
    forest_.add_box_no_adjacency(b2);
    forest_.rebuild_adjacency(1e-8);

    auto& adj = forest_.adjacency();
    // They should be adjacent
    bool adjacent = false;
    if (adj.count(0)) {
        const auto& v = adj.at(0);
        if (std::find(v.begin(), v.end(), 1) != v.end()) adjacent = true;
    }
    EXPECT_TRUE(adjacent);
}

TEST_F(SafeBoxForestTest, RemoveBox) {
    std::vector<Interval> ivs = {{0.0, 1.0}, {0.0, 1.0}};
    Eigen::VectorXd seed(2);
    seed << 0.5, 0.5;
    BoxNode box(0, ivs, seed);
    forest_.add_box_no_adjacency(box);
    EXPECT_EQ(forest_.n_boxes(), 1);

    forest_.remove_boxes({0});
    EXPECT_EQ(forest_.n_boxes(), 0);
}

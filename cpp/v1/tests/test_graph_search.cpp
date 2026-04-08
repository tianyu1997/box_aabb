// Tests for graph search (Dijkstra)
#include <gtest/gtest.h>
#include "sbf/planner/graph_search.h"
#include "sbf/core/types.h"

using namespace sbf;

TEST(GraphSearchTest, SimpleChain) {
    // 3 boxes in a chain: 0 -- 1 -- 2
    std::unordered_map<int, std::vector<int>> adj;
    adj[0] = {1};
    adj[1] = {0, 2};
    adj[2] = {1};

    std::unordered_map<int, BoxNode> boxes;
    Eigen::VectorXd s0(2), s1(2), s2(2);
    s0 << 0.5, 0.5; s1 << 1.5, 0.5; s2 << 2.5, 0.5;
    boxes[0] = BoxNode(0, {{0, 1}, {0, 1}}, s0);
    boxes[1] = BoxNode(1, {{1, 2}, {0, 1}}, s1);
    boxes[2] = BoxNode(2, {{2, 3}, {0, 1}}, s2);

    std::unordered_set<int> starts{0}, goals{2};
    auto result = dijkstra_center_distance(adj, boxes, starts, goals);

    EXPECT_TRUE(result.found);
    EXPECT_EQ(result.path.size(), 3u);
    EXPECT_EQ(result.path.front(), 0);
    EXPECT_EQ(result.path.back(), 2);
}

TEST(GraphSearchTest, NoPath) {
    // Disconnected: 0, 1 with no edges
    std::unordered_map<int, std::vector<int>> adj;
    adj[0] = {};
    adj[1] = {};

    std::unordered_map<int, BoxNode> boxes;
    Eigen::VectorXd s(2);
    s << 0.5, 0.5;
    boxes[0] = BoxNode(0, {{0, 1}, {0, 1}}, s);
    boxes[1] = BoxNode(1, {{5, 6}, {5, 6}}, s);

    std::unordered_set<int> starts{0}, goals{1};
    auto result = dijkstra_center_distance(adj, boxes, starts, goals);
    EXPECT_FALSE(result.found);
}

TEST(GraphSearchTest, ExtractWaypoints) {
    std::unordered_map<int, BoxNode> boxes;
    Eigen::VectorXd s0(2), s1(2);
    s0 << 0.5, 0.5; s1 << 1.5, 0.5;
    boxes[0] = BoxNode(0, {{0, 1}, {0, 1}}, s0);
    boxes[1] = BoxNode(1, {{1, 2}, {0, 1}}, s1);

    Eigen::VectorXd start(2), goal(2);
    start << 0.2, 0.5;
    goal << 1.8, 0.5;

    auto wps = extract_waypoints({0, 1}, boxes, start, goal);
    EXPECT_GE(wps.size(), 2u);
    EXPECT_DOUBLE_EQ(wps.front()[0], start[0]);
    EXPECT_DOUBLE_EQ(wps.back()[0], goal[0]);
}

/// @file path_finder.cpp
#include "sbf/planner/path_finder.h"

#include <algorithm>
#include <limits>
#include <queue>

namespace sbf::planner {

namespace {
inline double dist2(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
    return (a - b).squaredNorm();
}
}  // namespace

PathFinderResult find_box_path(
    const std::vector<sbf::scene::BoxNode>& boxes,
    const sbf::forest::AdjacencyGraph&      graph,
    int                                     start_box,
    int                                     goal_box,
    const Eigen::VectorXd&                  q_start,
    const Eigen::VectorXd&                  q_goal) {

    PathFinderResult res;
    const int N = static_cast<int>(boxes.size());
    if (start_box < 0 || goal_box < 0 || start_box >= N || goal_box >= N)
        return res;
    if (graph.n_nodes() != N) return res;

    // Build undirected neighbour lists from the lower-triangular nbrs[].
    std::vector<std::vector<int>> adj(N);
    for (int i = 0; i < N; ++i)
        for (int j : graph.nbrs[i]) {
            adj[i].push_back(j);
            adj[j].push_back(i);
        }

    // Dijkstra.
    constexpr double INF = std::numeric_limits<double>::infinity();
    std::vector<double> dist(N, INF);
    std::vector<int>    prev(N, -1);
    using QE = std::pair<double, int>;
    std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq;
    dist[start_box] = 0.0;
    pq.emplace(0.0, start_box);
    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (d > dist[u]) continue;
        if (u == goal_box) break;
        const Eigen::VectorXd cu = boxes[u].center();
        for (int v : adj[u]) {
            double w = std::sqrt(dist2(cu, boxes[v].center()));
            double nd = d + w;
            if (nd < dist[v]) {
                dist[v] = nd;
                prev[v] = u;
                pq.emplace(nd, v);
            }
        }
    }

    if (dist[goal_box] == INF) return res;

    // Reconstruct box path.
    std::vector<int> path;
    for (int u = goal_box; u != -1; u = prev[u]) path.push_back(u);
    std::reverse(path.begin(), path.end());
    res.box_path = path;

    // Waypoints: q_start, box centers (skip start_box and goal_box centers
    // when q_start/q_goal are inside them), q_goal.
    res.waypoints.push_back(q_start);
    for (int idx : path) {
        Eigen::VectorXd c = boxes[idx].center();
        // Skip center if it's redundant with the previous waypoint.
        if ((c - res.waypoints.back()).norm() > 1e-9)
            res.waypoints.push_back(c);
    }
    if ((q_goal - res.waypoints.back()).norm() > 1e-9)
        res.waypoints.push_back(q_goal);

    // Length.
    double L = 0.0;
    for (size_t k = 1; k < res.waypoints.size(); ++k)
        L += (res.waypoints[k] - res.waypoints[k - 1]).norm();
    res.length = L;
    res.success = true;
    return res;
}

}  // namespace sbf::planner

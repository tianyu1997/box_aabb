// SafeBoxForest v2 — Graph search implementation
// Module: sbf (planner)
#include "sbf/planner/graph_search.h"
#include <limits>
#include <queue>
#include <unordered_map>

namespace sbf {

DijkstraResult dijkstra(
    const std::unordered_map<int, std::vector<int>>& adjacency,
    const std::unordered_set<int>& start_ids,
    const std::unordered_set<int>& goal_ids,
    std::function<double(int, int)> weight_fn) {

    DijkstraResult result;

    using PQItem = std::pair<double, int>;
    std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> pq;
    std::unordered_map<int, double> dist;
    std::unordered_map<int, int> parent;

    for (int s : start_ids) {
        dist[s] = 0.0;
        parent[s] = -1;
        pq.push({0.0, s});
    }

    int found_goal = -1;

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();

        if (dist.count(u) && d > dist[u] + 1e-12)
            continue;

        if (goal_ids.count(u)) {
            found_goal = u;
            break;
        }

        auto adj_it = adjacency.find(u);
        if (adj_it == adjacency.end()) continue;

        for (int v : adj_it->second) {
            double w = weight_fn(u, v);
            double new_dist = d + w;
            if (!dist.count(v) || new_dist < dist[v]) {
                dist[v] = new_dist;
                parent[v] = u;
                pq.push({new_dist, v});
            }
        }
    }

    if (found_goal < 0) {
        result.found = false;
        return result;
    }

    // Reconstruct path
    result.found = true;
    result.total_cost = dist[found_goal];
    int node = found_goal;
    while (node >= 0) {
        result.path.push_back(node);
        node = parent[node];
    }
    std::reverse(result.path.begin(), result.path.end());
    return result;
}

DijkstraResult dijkstra_center_distance(
    const std::unordered_map<int, std::vector<int>>& adjacency,
    const std::unordered_map<int, BoxNode>& boxes,
    const std::unordered_set<int>& start_ids,
    const std::unordered_set<int>& goal_ids) {

    std::unordered_map<int, Eigen::VectorXd> centers;
    for (auto& [id, box] : boxes)
        centers[id] = box.center();

    auto weight_fn = [&centers](int u, int v) -> double {
        auto it_u = centers.find(u);
        auto it_v = centers.find(v);
        if (it_u == centers.end() || it_v == centers.end())
            return std::numeric_limits<double>::max();
        return (it_u->second - it_v->second).norm();
    };

    return dijkstra(adjacency, start_ids, goal_ids, weight_fn);
}

std::vector<Eigen::VectorXd>
extract_waypoints(const std::vector<int>& box_sequence,
                  const std::unordered_map<int, BoxNode>& boxes,
                  const Eigen::VectorXd& start,
                  const Eigen::VectorXd& goal) {
    std::vector<Eigen::VectorXd> waypoints;
    if (box_sequence.empty()) return waypoints;

    waypoints.push_back(start);

    for (size_t i = 0; i + 1 < box_sequence.size(); ++i) {
        auto it_a = boxes.find(box_sequence[i]);
        auto it_b = boxes.find(box_sequence[i + 1]);
        if (it_a == boxes.end() || it_b == boxes.end()) continue;

        Eigen::VectorXd center = it_a->second.shared_face_center(it_b->second);
        waypoints.push_back(center);
    }

    waypoints.push_back(goal);
    return waypoints;
}

} // namespace sbf

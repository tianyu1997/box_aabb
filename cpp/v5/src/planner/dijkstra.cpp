// SafeBoxForest v5 — Dijkstra (Phase H1)
#include <sbf/planner/dijkstra.h>

#include <algorithm>
#include <limits>
#include <queue>
#include <unordered_map>

namespace sbf {

DijkstraResult dijkstra_search(
    const AdjacencyGraph& adj,
    const std::vector<BoxNode>& boxes,
    int start_box_id,
    int goal_box_id)
{
    DijkstraResult result;
    if (start_box_id == goal_box_id) {
        result.found = true;
        result.box_sequence = {start_box_id};
        result.total_cost = 0.0;
        return result;
    }

    // Cache box centers by id
    std::unordered_map<int, Eigen::VectorXd> centers;
    for (const auto& b : boxes)
        centers[b.id] = b.center();

    // Standard heap Dijkstra
    using PQItem = std::pair<double, int>;
    std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> pq;
    std::unordered_map<int, double> dist;
    std::unordered_map<int, int> prev;

    dist[start_box_id] = 0.0;
    pq.push({0.0, start_box_id});

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();

        if (dist.count(u) && d > dist[u] + 1e-12)
            continue;  // stale entry

        if (u == goal_box_id) {
            // Reconstruct path
            result.found = true;
            result.total_cost = d;
            int node = goal_box_id;
            while (node != start_box_id) {
                result.box_sequence.push_back(node);
                node = prev[node];
            }
            result.box_sequence.push_back(start_box_id);
            std::reverse(result.box_sequence.begin(), result.box_sequence.end());
            return result;
        }

        auto adj_it = adj.find(u);
        if (adj_it == adj.end()) continue;

        for (int v : adj_it->second) {
            auto cu = centers.find(u);
            auto cv = centers.find(v);
            if (cu == centers.end() || cv == centers.end()) continue;

            double edge_cost = (cu->second - cv->second).norm();
            double new_dist = d + edge_cost;

            if (!dist.count(v) || new_dist < dist[v]) {
                dist[v] = new_dist;
                prev[v] = u;
                pq.push({new_dist, v});
            }
        }
    }

    result.found = false;
    return result;
}

}  // namespace sbf

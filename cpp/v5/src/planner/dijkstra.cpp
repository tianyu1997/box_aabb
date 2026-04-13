// SafeBoxForest v5 — Dijkstra (Phase H1)
#include <sbf/planner/dijkstra.h>

#include <algorithm>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace sbf {

// Compute center point of a shared face between two boxes
static Eigen::VectorXd face_center(const BoxNode& a, const BoxNode& b) {
    auto face = shared_face(a, b);
    if (!face.has_value()) {
        // Fallback: midpoint of centers
        return (a.center() + b.center()) * 0.5;
    }
    const int nd = a.n_dims();
    Eigen::VectorXd fc(nd);
    int face_idx = 0;
    for (int d = 0; d < nd; ++d) {
        if (d == face->dim) {
            fc[d] = face->value;
        } else {
            fc[d] = face->face_ivs[face_idx].center();
            ++face_idx;
        }
    }
    return fc;
}

DijkstraResult dijkstra_search(
    const AdjacencyGraph& adj,
    const std::vector<BoxNode>& boxes,
    int start_box_id,
    int goal_box_id,
    const Eigen::VectorXd& goal_point)
{
    DijkstraResult result;
    if (start_box_id == goal_box_id) {
        result.found = true;
        result.box_sequence = {start_box_id};
        result.total_cost = 0.0;
        return result;
    }

    // Build id → BoxNode lookup
    std::unordered_map<int, const BoxNode*> box_map;
    for (const auto& b : boxes)
        box_map[b.id] = &b;

    // A* heuristic: Euclidean distance to goal center
    // Falls back to goal box center if goal_point is empty
    Eigen::VectorXd goal_target;
    if (goal_point.size() > 0) {
        goal_target = goal_point;
    } else {
        auto git = box_map.find(goal_box_id);
        if (git != box_map.end())
            goal_target = git->second->center();
    }
    const bool use_heuristic = (goal_target.size() > 0);

    auto heuristic = [&](int box_id) -> double {
        if (!use_heuristic) return 0.0;
        auto it = box_map.find(box_id);
        if (it == box_map.end()) return 0.0;
        return (it->second->center() - goal_target).norm();
    };

    // Representative point for each node: the face center through which we
    // entered it (or its center if it is the start node).
    std::unordered_map<int, Eigen::VectorXd> repr;
    repr[start_box_id] = box_map[start_box_id]->center();

    // A* search: priority = g + h
    using PQItem = std::pair<double, int>;  // (f = g + h, node_id)
    std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> pq;
    std::unordered_map<int, double> g_cost;  // best known g-cost
    std::unordered_map<int, int> prev;

    g_cost[start_box_id] = 0.0;
    pq.push({heuristic(start_box_id), start_box_id});

    while (!pq.empty()) {
        auto [f, u] = pq.top();
        pq.pop();

        double g_u = g_cost.count(u) ? g_cost[u] : 1e30;
        // Skip stale entries: f should be g_u + h(u)
        if (f > g_u + heuristic(u) + 1e-12)
            continue;

        if (u == goal_box_id) {
            result.found = true;
            result.total_cost = g_u;
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

        auto bu = box_map.find(u);
        if (bu == box_map.end()) continue;

        const Eigen::VectorXd& u_repr = repr[u];

        for (int v : adj_it->second) {
            auto bv = box_map.find(v);
            if (bv == box_map.end()) continue;

            // Edge cost = distance from u's representative to the shared face center
            Eigen::VectorXd fc = face_center(*bu->second, *bv->second);
            double edge_cost = (u_repr - fc).norm();
            edge_cost += 0.02;  // hop penalty: prefer fewer, larger boxes

            double new_g = g_u + edge_cost;
            if (!g_cost.count(v) || new_g < g_cost[v]) {
                g_cost[v] = new_g;
                prev[v] = u;
                repr[v] = fc;
                double f_v = new_g + heuristic(v);
                pq.push({f_v, v});
            }
        }
    }

    result.found = false;
    return result;
}

std::vector<int> shortcut_box_sequence(
    const std::vector<int>& box_seq,
    const AdjacencyGraph& adj)
{
    if (box_seq.size() <= 2) return box_seq;

    // Build adjacency sets for O(1) lookup (for all boxes in the sequence)
    std::unordered_map<int, std::unordered_set<int>> adj_set;
    for (int id : box_seq) {
        auto it = adj.find(id);
        if (it != adj.end())
            adj_set[id] = std::unordered_set<int>(it->second.begin(), it->second.end());
    }

    std::vector<int> result;
    result.reserve(box_seq.size());
    result.push_back(box_seq[0]);

    size_t i = 0;
    while (i < box_seq.size() - 1) {
        size_t best = i + 1;
        auto it_i = adj_set.find(box_seq[i]);

        for (size_t j = box_seq.size() - 1; j > i + 1; --j) {
            // 1-hop: box_seq[i] directly adjacent to box_seq[j]
            if (it_i != adj_set.end() && it_i->second.count(box_seq[j])) {
                best = j;
                break;
            }

            // 2-hop: ∃ intermediate N that is neighbor of both box_seq[i] and box_seq[j]
            if (it_i != adj_set.end()) {
                auto it_j = adj_set.find(box_seq[j]);
                if (it_j != adj_set.end()) {
                    bool found = false;
                    for (int nbr : it_i->second) {
                        if (it_j->second.count(nbr)) {
                            found = true;
                            break;
                        }
                    }
                    if (found) {
                        best = j;
                        break;
                    }
                }
            }
        }
        result.push_back(box_seq[best]);
        i = best;
    }

    return result;
}

}  // namespace sbf

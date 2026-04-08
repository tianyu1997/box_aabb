// SafeBoxForest v5 — GCS Planner (Phase H3 + Phase L)
#include <sbf/planner/gcs_planner.h>
#include <sbf/planner/dijkstra.h>
#include <sbf/planner/path_extract.h>

#include <algorithm>

#ifdef SBF_HAS_DRAKE
#include <drake/geometry/optimization/graph_of_convex_sets.h>
#include <drake/geometry/optimization/hpolyhedron.h>
#include <drake/geometry/optimization/point.h>
#include <drake/solvers/solve.h>
#endif

namespace sbf {

// ─── Corridor expansion ─────────────────────────────────────────────────────

std::unordered_set<int> expand_corridor(
    const AdjacencyGraph& adj,
    const std::vector<int>& path_boxes,
    int hops)
{
    std::unordered_set<int> corridor(path_boxes.begin(), path_boxes.end());
    std::unordered_set<int> frontier = corridor;

    for (int h = 0; h < hops; ++h) {
        std::unordered_set<int> next_frontier;
        for (int box_id : frontier) {
            auto it = adj.find(box_id);
            if (it == adj.end()) continue;
            for (int nbr : it->second) {
                if (corridor.insert(nbr).second) {
                    next_frontier.insert(nbr);
                }
            }
        }
        frontier = std::move(next_frontier);
    }
    return corridor;
}

// ─── Drake GCS implementation ────────────────────────────────────────────────

#ifdef SBF_HAS_DRAKE

using drake::geometry::optimization::GraphOfConvexSets;
using drake::geometry::optimization::HPolyhedron;
using drake::geometry::optimization::Point;

GCSResult gcs_plan(
    const AdjacencyGraph& adj,
    const std::vector<BoxNode>& boxes,
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& goal,
    const GCSConfig& config)
{
    const int n = static_cast<int>(start.size());

    // 1. Find containing boxes
    int start_box = -1, goal_box = -1;
    for (const auto& b : boxes) {
        if (start_box < 0 && b.contains(start)) start_box = b.id;
        if (goal_box < 0 && b.contains(goal))   goal_box = b.id;
        if (start_box >= 0 && goal_box >= 0) break;
    }
    if (start_box < 0 || goal_box < 0)
        return gcs_plan_fallback(adj, boxes, start, goal);

    // 2. Dijkstra baseline
    auto dij = dijkstra_search(adj, boxes, start_box, goal_box);
    if (!dij.found)
        return {false, {}, 0.0};

    // 3. Expand corridor
    auto corridor = expand_corridor(adj, dij.box_sequence, config.corridor_hops);

    // Build box lookup by id
    std::unordered_map<int, const BoxNode*> box_map;
    for (const auto& b : boxes)
        box_map[b.id] = &b;

    // 4. Build GCS
    GraphOfConvexSets gcs;
    std::unordered_map<int, GraphOfConvexSets::Vertex*> verts;

    // 4a. Box -> HPolyhedron vertices
    for (int box_id : corridor) {
        auto bit = box_map.find(box_id);
        if (bit == box_map.end()) continue;
        const BoxNode& box = *bit->second;

        Eigen::VectorXd lb(n), ub(n);
        for (int d = 0; d < n; ++d) {
            lb[d] = box.joint_intervals[d].lo;
            ub[d] = box.joint_intervals[d].hi;
        }
        auto hpoly = HPolyhedron::MakeBox(lb, ub);
        verts[box_id] = gcs.AddVertex(hpoly, "box_" + std::to_string(box_id));
    }

    // 4b. Start/Goal point vertices
    auto* v_start = gcs.AddVertex(Point(start), "start");
    auto* v_goal  = gcs.AddVertex(Point(goal),  "goal");

    // 4c. Start -> containing box, containing box -> Goal
    gcs.AddEdge(v_start, verts[start_box]);
    gcs.AddEdge(verts[goal_box], v_goal);

    // 4d. Adjacency edges (bidirectional, deduplicated)
    for (int u : corridor) {
        auto adj_it = adj.find(u);
        if (adj_it == adj.end()) continue;
        for (int v : adj_it->second) {
            if (corridor.count(v) && u < v) {
                gcs.AddEdge(verts[u], verts[v]);
                gcs.AddEdge(verts[v], verts[u]);
            }
        }
    }

    // 4e. Edge cost: ||x_u - x_v||₂²
    for (auto* edge : gcs.Edges()) {
        auto xu = edge->xu();
        auto xv = edge->xv();
        edge->AddCost((xu - xv).squaredNorm() * config.cost_weight_length);
    }

    // 5. Solve
    auto result = gcs.SolveShortestPath(
        *v_start, *v_goal,
        config.convex_relaxation);

    if (!result.is_success()) {
        return gcs_plan_fallback(adj, boxes, start, goal);
    }

    // 6. Extract path along Dijkstra box sequence (ordered by original path)
    std::vector<Eigen::VectorXd> path;
    path.push_back(start);

    for (int box_id : dij.box_sequence) {
        auto vit = verts.find(box_id);
        if (vit != verts.end()) {
            Eigen::VectorXd wp = result.GetSolution(vit->second->x());
            path.push_back(wp);
        }
    }

    path.push_back(goal);

    double cost = result.get_optimal_cost();
    return {true, std::move(path), cost};
}

#endif  // SBF_HAS_DRAKE

// ─── Fallback (no Drake) ────────────────────────────────────────────────────

GCSResult gcs_plan_fallback(
    const AdjacencyGraph& adj,
    const std::vector<BoxNode>& boxes,
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& goal)
{
    GCSResult result;

    // Find start/goal containing boxes
    int start_id = -1, goal_id = -1;
    for (const auto& b : boxes) {
        if (start_id < 0 && b.contains(start)) start_id = b.id;
        if (goal_id < 0 && b.contains(goal))   goal_id = b.id;
        if (start_id >= 0 && goal_id >= 0) break;
    }
    if (start_id < 0 || goal_id < 0) return result;

    auto dij = dijkstra_search(adj, boxes, start_id, goal_id);
    if (!dij.found) return result;

    result.found = true;
    result.path = extract_waypoints(dij.box_sequence, boxes, start, goal);
    result.cost = dij.total_cost;
    return result;
}

}  // namespace sbf

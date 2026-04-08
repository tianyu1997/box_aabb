// SafeBoxForest v2 — Drake GCS adapter implementation
// Module: sbf (bridge)
#include "sbf/adapters/drake_gcs.h"
#include "sbf/planner/graph_search.h"
#include <chrono>

#ifdef SBF_WITH_DRAKE
#include <drake/geometry/optimization/graph_of_convex_sets.h>
#include <drake/geometry/optimization/hpolyhedron.h>
#include <drake/solvers/solve.h>
#endif

namespace sbf {

GCSResult GCSOptimizer::solve_dijkstra_fallback(
    const SafeBoxForest& forest,
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& goal) {
    GCSResult result;

    // Find start/goal boxes
    std::unordered_set<int> start_set, goal_set;
    for (auto& [id, box] : forest.boxes()) {
        if (box.contains(start)) start_set.insert(id);
        if (box.contains(goal)) goal_set.insert(id);
    }
    if (start_set.empty() || goal_set.empty()) return result;

    auto dijk = dijkstra_center_distance(
        forest.adjacency(), forest.boxes(), start_set, goal_set);
    if (!dijk.found) return result;

    auto waypoints = extract_waypoints(dijk.path, forest.boxes(), start, goal);
    result.success = true;
    result.waypoints = waypoints;
    result.active_box_ids = dijk.path;
    result.cost = dijk.total_cost;
    return result;
}

#ifdef SBF_WITH_DRAKE

using drake::geometry::optimization::GraphOfConvexSets;
using drake::geometry::optimization::HPolyhedron;

GCSResult GCSOptimizer::solve_gcs(
    const SafeBoxForest& forest,
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& goal,
    const std::vector<int>& corridor_box_ids)
{
    GCSResult result;
    int n = start.size();

    GraphOfConvexSets gcs;
    std::unordered_map<int, GraphOfConvexSets::Vertex*> vtx_map;
    std::unordered_set<int> relevant_ids(corridor_box_ids.begin(), corridor_box_ids.end());

    for (int id : relevant_ids) {
        auto it = forest.boxes().find(id);
        if (it == forest.boxes().end()) continue;
        const BoxNode& box = it->second;

        Eigen::MatrixXd A(2 * n, n);
        A.topRows(n) = Eigen::MatrixXd::Identity(n, n);
        A.bottomRows(n) = -Eigen::MatrixXd::Identity(n, n);
        Eigen::VectorXd b(2 * n);
        for (int d = 0; d < n; ++d) {
            b[d] = box.joint_intervals[d].hi;
            b[n + d] = -box.joint_intervals[d].lo;
        }

        HPolyhedron region(A, b);
        auto* vtx = gcs.AddVertex(region, "box_" + std::to_string(id));
        vtx_map[id] = vtx;
    }

    for (int id : relevant_ids) {
        auto adj_it = forest.adjacency().find(id);
        if (adj_it == forest.adjacency().end()) continue;
        for (int neighbor : adj_it->second) {
            if (relevant_ids.count(neighbor) && id < neighbor) {
                auto* va = vtx_map[id];
                auto* vb = vtx_map[neighbor];
                gcs.AddEdge(va, vb);
                gcs.AddEdge(vb, va);
            }
        }
    }

    if (!corridor_box_ids.empty()) {
        auto* source = vtx_map[corridor_box_ids.front()];
        auto* target = vtx_map[corridor_box_ids.back()];
        auto math_result = gcs.SolveShortestPath(*source, *target);
        if (math_result.is_success()) {
            result.success = true;
            result.cost = math_result.get_optimal_cost();
            result.waypoints.push_back(start);
            for (int id : corridor_box_ids) {
                auto vit = vtx_map.find(id);
                if (vit != vtx_map.end()) {
                    result.waypoints.push_back(
                        math_result.GetSolution(vit->second->x()));
                }
            }
            result.waypoints.push_back(goal);
            result.active_box_ids = corridor_box_ids;
        }
    }
    return result;
}

GCSResult GCSOptimizer::optimize(const SafeBoxForest& forest,
                                  const Eigen::VectorXd& start,
                                  const Eigen::VectorXd& goal,
                                  int corridor_hops) {
    auto t0 = std::chrono::steady_clock::now();

    // First get Dijkstra path
    auto fallback = solve_dijkstra_fallback(forest, start, goal);
    if (!fallback.success) return fallback;

    // Expand corridor around Dijkstra path
    std::unordered_set<int> corridor(fallback.active_box_ids.begin(),
                                      fallback.active_box_ids.end());
    for (int hop = 0; hop < corridor_hops; ++hop) {
        std::unordered_set<int> expanded = corridor;
        for (int id : corridor) {
            auto adj_it = forest.adjacency().find(id);
            if (adj_it != forest.adjacency().end()) {
                for (int nb : adj_it->second)
                    expanded.insert(nb);
            }
        }
        corridor = expanded;
    }

    std::vector<int> corridor_ids(corridor.begin(), corridor.end());
    auto result = solve_gcs(forest, start, goal, corridor_ids);
    result.solve_time = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();

    if (!result.success) return fallback;
    return result;
}

bool GCSOptimizer::drake_available() { return true; }

#else  // !SBF_WITH_DRAKE

GCSResult GCSOptimizer::optimize(const SafeBoxForest& forest,
                                  const Eigen::VectorXd& start,
                                  const Eigen::VectorXd& goal,
                                  int /*corridor_hops*/) {
    return solve_dijkstra_fallback(forest, start, goal);
}

bool GCSOptimizer::drake_available() { return false; }

#endif  // SBF_WITH_DRAKE

// Free function
bool drake_available() { return GCSOptimizer::drake_available(); }

} // namespace sbf

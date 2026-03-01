// SafeBoxForest — Graph search (Dijkstra / A*)
#pragma once

#include "sbf/core/types.h"
#include <Eigen/Core>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace sbf {

// ─── Dijkstra on box adjacency graph ────────────────────────────────────────
// Returns ordered box ID sequence from start to goal.
// weight_fn: (from_id, to_id) → double
struct DijkstraResult {
    std::vector<int> path;    // box IDs in order
    double total_cost = 0.0;
    bool found = false;
};

DijkstraResult dijkstra(
    const std::unordered_map<int, std::vector<int>>& adjacency,
    const std::unordered_set<int>& start_ids,
    const std::unordered_set<int>& goal_ids,
    std::function<double(int, int)> weight_fn);

// Default weight: L2 distance between box centers
DijkstraResult dijkstra_center_distance(
    const std::unordered_map<int, std::vector<int>>& adjacency,
    const std::unordered_map<int, BoxNode>& boxes,
    const std::unordered_set<int>& start_ids,
    const std::unordered_set<int>& goal_ids);

// ─── Waypoint extraction from box sequence ──────────────────────────────────
// For each consecutive pair of boxes, compute the shared face center
std::vector<Eigen::VectorXd>
extract_waypoints(const std::vector<int>& box_sequence,
                  const std::unordered_map<int, BoxNode>& boxes,
                  const Eigen::VectorXd& start,
                  const Eigen::VectorXd& goal);

} // namespace sbf

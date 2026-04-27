#pragma once
/// @file path_finder.h
/// @brief Dijkstra over the box adjacency graph.
///
/// v7 P5 baseline path-finder: replaces v6's Drake GCS solver with a
/// straight Dijkstra search on the box-level adjacency graph. Yields
/// a sequence of box ids and the corresponding center-to-center
/// waypoints, suitable as input to `PathOptPipeline`.
///
/// Strict adjacency (F1) is already enforced upstream by
/// `compute_adjacency_graph()` (face-touch tol = 1e-11), so no extra
/// validation is performed here.

#include "sbf/forest/adjacency.h"
#include "sbf/scene/box_node.h"

#include <Eigen/Core>
#include <vector>

namespace sbf::planner {

struct PathFinderResult {
    bool success = false;
    /// Box id sequence start_box → ... → goal_box.
    std::vector<int>             box_path;
    /// Joint-space waypoints; first = q_start, last = q_goal,
    /// intermediates = box centers.
    std::vector<Eigen::VectorXd> waypoints;
    /// Total path length in joint space (Euclidean).
    double                       length = 0.0;
};

/// Find shortest path from start_box to goal_box on the adjacency graph
/// using Dijkstra with center-to-center Euclidean edge weights. Returns
/// `success=false` if the two boxes are in different islands.
PathFinderResult find_box_path(
    const std::vector<sbf::scene::BoxNode>& boxes,
    const sbf::forest::AdjacencyGraph&      graph,
    int                                     start_box,
    int                                     goal_box,
    const Eigen::VectorXd&                  q_start,
    const Eigen::VectorXd&                  q_goal);

}  // namespace sbf::planner

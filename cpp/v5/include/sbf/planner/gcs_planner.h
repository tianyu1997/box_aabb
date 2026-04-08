#pragma once
/// @file gcs_planner.h
/// @brief Graph of Convex Sets (GCS) motion planner.
///
/// When Drake is available (`SBF_HAS_DRAKE`), solves the GCS shortest-path
/// problem with Bézier-curve edges through a corridor of adjacent boxes.
/// Otherwise, uses `gcs_plan_fallback()` which extracts waypoints via
/// Dijkstra + face-midpoint interpolation.

#include <sbf/core/types.h>
#include <sbf/forest/adjacency.h>

#include <Eigen/Dense>
#include <unordered_set>
#include <vector>

namespace sbf {

/// @brief Configuration for GCS-based planning.
struct GCSConfig {
    int bezier_degree = 3;             ///< Bézier spline degree for edge trajectories.
    double time_limit_sec = 30.0;      ///< GCS solver time limit.
    int corridor_hops = 2;             ///< Corridor expansion layers around Dijkstra path.
    bool convex_relaxation = true;     ///< Use convex relaxation in GCS solve.
    double cost_weight_length = 1.0;   ///< Path-length cost weight.
};

/// @brief Result of a GCS (or fallback) planning query.
struct GCSResult {
    bool found = false;                         ///< Whether a path was found.
    std::vector<Eigen::VectorXd> path;          ///< Continuous C-space waypoints.
    double cost = 0.0;                          ///< Total path cost.
};

/// Expand Dijkstra path boxes by ±hops layers of neighbors.
std::unordered_set<int> expand_corridor(
    const AdjacencyGraph& adj,
    const std::vector<int>& path_boxes,
    int hops);

#ifdef SBF_HAS_DRAKE
GCSResult gcs_plan(
    const AdjacencyGraph& adj,
    const std::vector<BoxNode>& boxes,
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& goal,
    const GCSConfig& config = {});
#endif

// Fallback: Dijkstra + waypoint extraction (no Drake needed)
GCSResult gcs_plan_fallback(
    const AdjacencyGraph& adj,
    const std::vector<BoxNode>& boxes,
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& goal);

}  // namespace sbf

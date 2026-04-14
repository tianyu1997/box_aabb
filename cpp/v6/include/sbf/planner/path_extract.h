#pragma once
/// @file path_extract.h
/// @brief Waypoint extraction from a Dijkstra box sequence.
///
/// Converts a sequence of box IDs into a continuous C-space path
/// by placing waypoints at shared-face midpoints between adjacent boxes.

#include <sbf/core/types.h>
#include <sbf/forest/adjacency.h>

#include <Eigen/Dense>
#include <vector>

namespace sbf {

/// @brief Extract C-space waypoints from a box sequence.
///
/// For each consecutive box pair, places a waypoint at the centroid
/// of their shared face.  Prepends `start` and appends `goal`.
std::vector<Eigen::VectorXd> extract_waypoints(
    const std::vector<int>& box_sequence,
    const std::vector<BoxNode>& boxes,
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& goal);

}  // namespace sbf

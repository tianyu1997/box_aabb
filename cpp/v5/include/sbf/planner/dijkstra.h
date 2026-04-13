#pragma once
/// @file dijkstra.h
/// @brief Dijkstra shortest-path search over box adjacency graph.
///
/// Finds the cheapest box-to-box path (by centroid distance) between
/// a start-containing box and a goal-containing box.

#include <sbf/core/types.h>
#include <sbf/forest/adjacency.h>

#include <vector>

namespace sbf {

/// @brief Result of a Dijkstra box-graph search.
struct DijkstraResult {
    bool found = false;                  ///< Whether a path was found.
    std::vector<int> box_sequence;       ///< Sequence of box IDs from start to goal.
    double total_cost = 0.0;             ///< Cumulative centroid distance.
};

/// @brief Run A* search on the adjacency graph to find cheapest box sequence.
/// Uses Euclidean distance to goal center as admissible heuristic.
DijkstraResult dijkstra_search(
    const AdjacencyGraph& adj,
    const std::vector<BoxNode>& boxes,
    int start_box_id,
    int goal_box_id,
    const Eigen::VectorXd& goal_point = Eigen::VectorXd());

/// @brief Greedy forward-skip shortcut on a box sequence.
///
/// For each box in the sequence, attempts to jump as far forward as possible
/// while maintaining adjacency.  Dramatically reduces box_seq length.
/// @return Shortened box sequence preserving start and goal.
std::vector<int> shortcut_box_sequence(
    const std::vector<int>& box_seq,
    const AdjacencyGraph& adj);

}  // namespace sbf

#pragma once
/// @file adjacency.h
/// @brief Box-to-box adjacency graph construction.
///
/// Two boxes are **adjacent** if they share a face: identical value on
/// one dimension and overlapping intervals on all other dimensions
/// (within tolerance).  The adjacency graph is the foundation for
/// Dijkstra search, island detection, bridging, and coarsening.

#include <sbf/core/types.h>

#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace sbf {

/// Adjacency graph: box_id → list of adjacent box_ids.
using AdjacencyGraph = std::unordered_map<int, std::vector<int>>;

/// @brief Describes the shared face between two adjacent boxes.
struct SharedFace {
    int dim;                           ///< Dimension of the shared face (0..n_joints−1).
    double value;                      ///< Coordinate value at which the face lies.
    std::vector<Interval> face_ivs;    ///< Overlap interval on each *other* dimension.
};

/// Build the full adjacency graph from a set of boxes.
/// O(n²) pairwise comparison with early-exit.
AdjacencyGraph compute_adjacency(
    const std::vector<BoxNode>& boxes,
    double tol = 1e-6);

/// Return the shared face between two boxes, or std::nullopt if not adjacent.
std::optional<SharedFace> shared_face(
    const BoxNode& a,
    const BoxNode& b,
    double tol = 1e-6);

/// Find articulation points (bridge boxes) in the adjacency graph.
/// An articulation point is a box whose removal increases the number of
/// connected components. These boxes must be preserved during coarsening.
std::unordered_set<int> find_articulation_points(const AdjacencyGraph& adj);

}  // namespace sbf

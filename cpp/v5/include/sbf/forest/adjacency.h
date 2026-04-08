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
    double tol = 1e-10);

/// Return the shared face between two boxes, or std::nullopt if not adjacent.
std::optional<SharedFace> shared_face(
    const BoxNode& a,
    const BoxNode& b,
    double tol = 1e-10);

}  // namespace sbf

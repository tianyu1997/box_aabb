#pragma once
/// @file adjacency.h
/// @brief Box-level adjacency graph and connected components.

#include "sbf/scene/box_node.h"

#include <vector>

namespace sbf::forest {

/// Two BoxNodes are adjacent iff every joint dim either:
///   • genuinely overlaps (overlap >= tol), or
///   • face-touches (0 ≤ overlap < tol),
/// AND the boxes are not separated in any dim.
/// Volumetric interpenetration (all dims overlap >= tol) also counts.
bool boxes_adjacent(const sbf::scene::BoxNode& a,
                    const sbf::scene::BoxNode& b,
                    double tol = 1e-11);

struct AdjacencyGraph {
    /// `nbrs[i]` is the sorted list of indices j>i with i adjacent to j
    /// (lower-triangular adjacency; query `is_edge` for symmetric access).
    std::vector<std::vector<int>> nbrs;
    int n_edges = 0;

    int n_nodes() const { return static_cast<int>(nbrs.size()); }
};

AdjacencyGraph compute_adjacency_graph(
    const std::vector<sbf::scene::BoxNode>& boxes, double tol = 1e-11);

/// Connected component id per box (0..n_components-1).
struct IslandResult {
    std::vector<int> component_id;   ///< per-box component id
    int n_components = 0;
    int largest_size = 0;
};

IslandResult find_islands(const AdjacencyGraph& g);

}  // namespace sbf::forest

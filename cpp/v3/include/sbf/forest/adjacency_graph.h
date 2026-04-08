// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v3 — AdjacencyGraph: sweep-and-prune accelerated box graph
//  Module: sbf::forest
//
//  Maintains a graph of face-adjacent C-space boxes with two main APIs:
//    - rebuild():  full batch sweep-and-prune, O(N·K) where K ≪ N
//    - add_box():  incremental binary-search insert, O(log N + K)
//
//  Acceleration techniques (ported from v2 deoverlap.cpp):
//    1. Dimension ranking by fill ratio → best sweep/filter dims
//    2. Sweep-and-prune with sorted break on sweep dimension
//    3. Dual-dimension pre-filter before full-dim check
//    4. Row-major flat layout for cache-friendly access
//    5. Narrow→wide dimension ordering for early exit
//
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/common/types.h"
#include "sbf/forest/kd_tree.h"

#include <Eigen/Core>
#include <unordered_map>
#include <utility>
#include <vector>

namespace sbf {
namespace forest {

class AdjacencyGraph {
public:
    AdjacencyGraph() = default;

    /// Initialize with joint limits (needed for fill-ratio dimension ranking).
    explicit AdjacencyGraph(const JointLimits& limits, double tol = 1e-10);

    // ═════════════════════════════════════════════════════════════════════
    //  Incremental operations (sweep-accelerated)
    // ═════════════════════════════════════════════════════════════════════

    /// Add a single box and discover its adjacencies.
    /// O(log N + K) via binary-search on sweep order + local scan.
    void add_box(const BoxNode& box);

    /// Remove a box and all its adjacency edges.
    void remove_box(int box_id);

    /// Remove all boxes and edges.
    void clear();

    // ═════════════════════════════════════════════════════════════════════
    //  Batch rebuild (sweep-and-prune)
    // ═════════════════════════════════════════════════════════════════════

    /// Full batch rebuild from a list of boxes.
    /// O(N·K) where K = average sweep-direction overlap count.
    void rebuild(const std::vector<BoxNode>& boxes);

    // ═════════════════════════════════════════════════════════════════════
    //  Queries
    // ═════════════════════════════════════════════════════════════════════

    /// Neighbors of a box (empty if not present).
    const std::vector<int>& neighbors(int box_id) const;

    /// Check if two boxes are in the same connected component (BFS).
    bool connected(int a, int b) const;

    /// Number of connected components.
    int n_components() const;

    /// All connected components (each is a vector of box IDs).
    std::vector<std::vector<int>> components() const;

    /// Find the box whose closest point is nearest to a query config.
    /// O(log N) via KD-tree (falls back to O(N) linear scan if tree not built).
    int find_nearest_box(const Eigen::VectorXd& q) const;

    // ═════════════════════════════════════════════════════════════════════
    //  Statistics
    // ═════════════════════════════════════════════════════════════════════

    int n_boxes() const { return static_cast<int>(box_map_.size()); }
    int n_edges() const { return n_edges_; }

    /// Sweep dimension chosen for acceleration.
    int sweep_dim() const { return sweep_dim_; }
    /// Filter dimension (second-best).
    int filter_dim() const { return filter_dim_; }

private:
    // ── Configuration ───────────────────────────────────────────────────
    JointLimits limits_;
    double tol_ = 1e-10;
    int n_dims_ = 0;

    // ── Box storage ─────────────────────────────────────────────────────
    std::unordered_map<int, BoxNode>          box_map_;
    std::unordered_map<int, std::vector<int>> adj_;
    int n_edges_ = 0;
    static const std::vector<int> empty_neighbors_;

    // ── Sweep-and-prune acceleration ────────────────────────────────────
    int sweep_dim_  = 0;
    int filter_dim_ = 1;
    int padded_dims_ = 0;   // n_dims_ rounded up to multiple of 4 (for SIMD)
    std::vector<int> dim_order_;  // remaining dims, narrow→wide

    // Row-major flat arrays: flat_{lo,hi}[flat_idx * padded_dims_ + d]
    // Padding dims (n_dims_ .. padded_dims_-1) set to lo=0, hi=1 (neutral).
    std::vector<double> flat_lo_;
    std::vector<double> flat_hi_;
    std::vector<int>    flat_ids_;    // flat_idx → box_id
    std::unordered_map<int, int> id_to_flat_;   // box_id → flat_idx

    // Sorted by flat_lo_[flat_idx * n_dims_ + sweep_dim_]
    std::vector<int> sweep_order_;   // indices into flat_{lo,hi,ids}

    // ── KD-tree for nearest-box queries ──────────────────────────────────
    mutable KDTree kd_tree_;
    mutable bool   kd_dirty_ = true;   // rebuild on next find_nearest_box

    // ── Internal helpers ────────────────────────────────────────────────
    void rank_dimensions();
    void rebuild_flat_arrays();
    void sort_sweep_order();

    /// Full adjacency check between two flat indices.
    /// Checks all dims in pruning order: sweep, filter, then dim_order_.
    bool check_adjacent_flat(int fi, int fj) const;

    /// Return sweep-dim lo value for a flat index.
    double sweep_lo(int fi) const { return flat_lo_[fi * padded_dims_ + sweep_dim_]; }
    double sweep_hi(int fi) const { return flat_hi_[fi * padded_dims_ + sweep_dim_]; }

    /// Add an undirected edge (internal, no duplicate check).
    void add_edge(int a, int b);
    void remove_edge(int a, int b);
};

} // namespace forest
} // namespace sbf

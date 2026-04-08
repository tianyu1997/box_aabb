// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — AdjacencyGraph: sweep-and-prune accelerated box graph
//  Module: sbf::forest
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/core/types.h"
#include "sbf/forest/kd_tree.h"

#include <Eigen/Core>

#include <utility>
#include <unordered_map>
#include <vector>

namespace sbf {
namespace forest {

class AdjacencyGraph {
public:
    AdjacencyGraph() = default;
    explicit AdjacencyGraph(const JointLimits& limits, double tol = 1e-10);

    void add_box(const BoxNode& box);
    void remove_box(int box_id);
    void clear();

    /// Full batch rebuild from a list of boxes.
    void rebuild(const std::vector<BoxNode>& boxes);

    /// Find all boxes adjacent to the given box.
    const std::vector<int>& neighbors(int box_id) const;

    /// Check if two boxes are adjacent.
    bool are_adjacent(int box_a, int box_b) const;

    /// Check graph connectivity via BFS.
    bool connected(int a, int b) const;

    /// Count connected components.
    int n_components() const;

    /// Find nearest box under point-to-box metric.
    int find_nearest_box(const Eigen::VectorXd& q) const;

    int n_boxes() const { return static_cast<int>(box_map_.size()); }
    int n_edges() const;

    int sweep_dim() const { return sweep_dim_; }
    int filter_dim() const { return filter_dim_; }

private:
    void add_edge(int a, int b);
    void remove_edge(int a, int b);

    void rank_dimensions();
    void rebuild_flat_arrays();
    void sort_sweep_order();
    bool check_adjacent_flat(int fi, int fj) const;
    double sweep_lo(int fi) const { return flat_lo_[fi * padded_dims_ + sweep_dim_]; }
    double sweep_hi(int fi) const { return flat_hi_[fi * padded_dims_ + sweep_dim_]; }

    JointLimits limits_;
    double tol_ = 1e-10;
    int n_dims_ = 0;
    std::unordered_map<int, BoxNode> box_map_;
    std::unordered_map<int, std::vector<int>> adj_;
    int n_edges_ = 0;

    int sweep_dim_ = 0;
    int filter_dim_ = 1;
    int padded_dims_ = 0;
    std::vector<int> dim_order_;

    std::vector<double> flat_lo_;
    std::vector<double> flat_hi_;
    std::vector<int> flat_ids_;
    std::unordered_map<int, int> id_to_flat_;
    std::vector<int> sweep_order_;

    mutable KDTree kd_tree_;
    mutable bool kd_dirty_ = true;

    static const std::vector<int> empty_neighbors_;
};

} // namespace forest
} // namespace sbf

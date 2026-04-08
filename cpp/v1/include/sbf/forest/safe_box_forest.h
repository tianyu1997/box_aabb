// SafeBoxForest — Safe Box Forest: box collection + adjacency graph
#pragma once

#include "sbf/core/types.h"
#include "sbf/forest/collision.h"
#include <Eigen/Core>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace sbf {

class SafeBoxForest {
public:
    SafeBoxForest() = default;
    SafeBoxForest(int n_dims, const JointLimits& limits);

    // ── Box management ───────────────────────────────────────────────────
    int allocate_id();
    void add_box_direct(const BoxNode& box);
    void add_box_no_adjacency(const BoxNode& box);
    void clear();  // Remove all boxes and adjacency
    void remove_boxes(const std::unordered_set<int>& box_ids);
    void remove_boxes_no_adjacency(const std::unordered_set<int>& box_ids);

    // ── Adjacency ────────────────────────────────────────────────────────
    void rebuild_adjacency(double tol = 1e-10);
    const std::unordered_map<int, std::vector<int>>& adjacency() const {
        return adjacency_;
    }
    std::unordered_map<int, std::vector<int>>& adjacency() {
        return adjacency_;
    }

    // ── Queries ──────────────────────────────────────────────────────────
    const BoxNode* find_containing(const Eigen::VectorXd& q) const;
    const BoxNode* find_nearest(const Eigen::VectorXd& q) const;
    std::vector<Eigen::VectorXd> get_uncovered_seeds(int n, std::mt19937& rng) const;

    // ── Validation ───────────────────────────────────────────────────────
    std::unordered_set<int> validate_boxes(const CollisionChecker& checker);
    std::unordered_set<int> invalidate_against_obstacle(
        const Obstacle& obs, const Robot& robot, double safety_margin = 0.0);
    void validate_invariants(double tol = 1e-8) const;

    // ── Properties ───────────────────────────────────────────────────────
    int n_boxes() const { return static_cast<int>(boxes_.size()); }
    double total_volume() const;
    int n_dims() const { return n_dims_; }
    const JointLimits& joint_limits() const { return limits_; }

    const std::unordered_map<int, BoxNode>& boxes() const { return boxes_; }
    std::unordered_map<int, BoxNode>& boxes_mut() { return boxes_; }

    // ── Vectorized interval cache ────────────────────────────────────────
    // For fast batch adjacency computation
    void rebuild_interval_cache();
    const Eigen::MatrixXd& intervals_lo() const { return ivs_lo_; }
    const Eigen::MatrixXd& intervals_hi() const { return ivs_hi_; }
    const std::vector<int>& interval_ids() const { return ivs_ids_; }

private:
    int n_dims_ = 0;
    JointLimits limits_;
    int next_id_ = 0;

    std::unordered_map<int, BoxNode> boxes_;
    std::unordered_map<int, std::vector<int>> adjacency_;

    // Vectorized interval cache
    Eigen::MatrixXd ivs_lo_;   // (N, D)
    Eigen::MatrixXd ivs_hi_;   // (N, D)
    std::vector<int> ivs_ids_;

    bool check_adjacency(const BoxNode& a, const BoxNode& b, double tol) const;
    void add_adjacency_edge(int a, int b);
    void remove_adjacency_node(int id);
};

} // namespace sbf

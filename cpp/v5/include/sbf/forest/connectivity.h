#pragma once
/// @file connectivity.h
/// @brief Connected-component analysis and inter-island bridging.
///
/// Uses Union-Find for fast island detection, then attempts to
/// bridge disconnected components via RRT-Connect pathfinding
/// followed by FFB box paving along the path.

#include <sbf/core/types.h>
#include <sbf/core/robot.h>
#include <sbf/ffb/ffb.h>
#include <sbf/forest/adjacency.h>
#include <sbf/lect/lect.h>
#include <sbf/scene/collision_checker.h>

#include <vector>
#include <atomic>
#include <chrono>
#include <memory>
#include <Eigen/Dense>

namespace sbf {

// ─── RRT-Connect configuration ─────────────────────────────────────────────

/// @brief Parameters for bidirectional RRT-Connect bridge pathfinding.
struct RRTConnectConfig {
    int    max_iters          = 8000;   ///< Maximum RRT iterations per call.
    double step_size          = 0.2;    ///< Step size in C-space (radians).
    double goal_bias          = 0.15;   ///< Probability of sampling toward other tree root.
    double timeout_ms         = 500.0;  ///< Per-call timeout in milliseconds.
    int    segment_resolution = 10;     ///< Collision-check resolution for segments.
};

/// @brief Weighted Union-Find with path compression.
///
/// Used for O(α(n)) connected-component queries on the adjacency graph.
class UnionFind {
public:
    explicit UnionFind(int n = 0);

    int  find(int x);                  ///< Find root representative.
    void unite(int x, int y);          ///< Merge components of x and y.
    bool connected(int x, int y);      ///< Test same-component membership.
    int  size() const { return static_cast<int>(parent_.size()); }
    /// Grow the structure to accommodate indices [old_size .. new_size-1].
    void resize(int new_size);

private:
    std::vector<int> parent_;
    std::vector<int> rank_;
};

/// Decompose the adjacency graph into connected components ("islands").
/// Each inner vector contains the box IDs of one connected component.
std::vector<std::vector<int>> find_islands(const AdjacencyGraph& adj);

/// @brief Bidirectional RRT-Connect between two C-space configurations.
///
/// Returns a collision-free waypoint path from @p q_a to @p q_b,
/// or an empty vector if connection fails.
/// @param seed  RNG seed for reproducibility.
std::vector<Eigen::VectorXd> rrt_connect(
    const Eigen::VectorXd& q_a,
    const Eigen::VectorXd& q_b,
    const CollisionChecker& checker,
    const Robot& robot,
    const RRTConnectConfig& cfg = {},
    int seed = 42,
    std::shared_ptr<std::atomic<bool>> cancel = nullptr);

#ifdef SBF_HAS_OMPL
/// @brief OMPL BIT* bridge planner between two C-space configurations.
///
/// Uses OMPL's BIT* (Batch Informed Trees) algorithm for asymptotically
/// optimal path planning.  Supports cancel flag for early termination
/// when the island has already been merged by another parallel worker.
///
/// @param timeout_ms  Maximum planning time in milliseconds.
/// @param cancel      If set to true, planner terminates immediately.
/// @return Collision-free waypoint path, or empty vector on failure.
std::vector<Eigen::VectorXd> bitstar_bridge(
    const Eigen::VectorXd& q_a,
    const Eigen::VectorXd& q_b,
    const CollisionChecker& checker,
    const Robot& robot,
    double timeout_ms = 5000.0,
    int seed = 42,
    std::shared_ptr<std::atomic<bool>> cancel = nullptr);
#endif  // SBF_HAS_OMPL

/// @brief Chain-pave boxes along an RRT waypoint path.
///
/// Starting from @p anchor_box_id, iterates along the waypoints and
/// creates new boxes via snap-to-face + FFB so that each new box is
/// guaranteed to share a face with the previous one (chain adjacency).
/// Updates @p boxes, @p adj, and LECT occupancy in place.
///
/// @param max_chain  Maximum total boxes to create along the chain.
/// @param max_steps_per_wp  Maximum chain steps toward a single waypoint.
/// @return Number of boxes created.
int chain_pave_along_path(
    const std::vector<Eigen::VectorXd>& rrt_path,
    int anchor_box_id,
    std::vector<BoxNode>& boxes,
    LECT& lect,
    const Obstacle* obs, int n_obs,
    const FFBConfig& ffb_config,
    AdjacencyGraph& adj,
    int& next_box_id,
    const Robot& robot,
    int max_chain = 200,
    int max_steps_per_wp = 15);

/// @brief Bridge S-tree (root_id=0) and T-tree (root_id=1) using
/// best-first box-pair RRT-Connect + chain pave.
///
/// Enumerates cross-tree box pairs sorted by center distance (best-first).
/// For each pair, runs RRT-Connect with @p per_pair_timeout_ms, then
/// chain-paves boxes along the path.  Stops after @p max_pairs attempts
/// or when S and T trees are connected.
/// New boxes inherit root_id from their anchor box.
/// Returns number of bridge boxes added.
int bridge_s_t(
    int start_box_id,
    int goal_box_id,
    std::vector<BoxNode>& boxes,
    LECT& lect,
    const Obstacle* obs, int n_obs,
    AdjacencyGraph& adj,
    const FFBConfig& ffb_config,
    int& next_box_id,
    const Robot& robot,
    const CollisionChecker& checker,
    double per_pair_timeout_ms = 200.0,
    int max_pairs = 30,
    std::chrono::steady_clock::time_point deadline = std::chrono::steady_clock::time_point::max());

/// @brief Bridge ALL disconnected islands to the largest connected component.
///
/// Iterates over all islands smaller than the largest one, and for each
/// attempts RRT-Connect + chain-pave to merge it into the main component.
/// Repeats until a single connected component remains or budget is exhausted.
///
/// @param per_pair_timeout_ms  RRT timeout per candidate box pair.
/// @param max_pairs_per_gap    Max candidate pairs to try per island.
/// @param max_total_bridges    Global cap on total bridge boxes created.
/// @return Total number of bridge boxes added across all island merges.
int bridge_all_islands(
    std::vector<BoxNode>& boxes,
    LECT& lect,
    const Obstacle* obs, int n_obs,
    AdjacencyGraph& adj,
    const FFBConfig& ffb_config,
    int& next_box_id,
    const Robot& robot,
    const CollisionChecker& checker,
    double per_pair_timeout_ms = 300.0,
    int max_pairs_per_gap = 15,
    int max_total_bridges = 200,
    int n_threads = 0,
    std::chrono::steady_clock::time_point deadline = std::chrono::steady_clock::time_point::max());

/// @brief Repair geometric adjacency for bridge-forced edges.
///
/// After bridge_all_islands / bridge_s_t, adj may contain "forced" edges
/// (waypoint/seed containment jumps) between boxes that are NOT geometrically
/// adjacent.  When compute_adjacency() is called fresh (e.g. after coarsening),
/// these edges are lost, fragmenting the graph.
///
/// This function extends the SMALLER box in each non-adjacent pair so that
/// it overlaps with the larger box, ensuring compute_adjacency() can detect
/// the connection.  Only small gaps (< max_gap) are repaired.
///
/// @return Number of box pairs repaired.
int repair_bridge_adjacency(std::vector<BoxNode>& boxes,
                            const AdjacencyGraph& adj);

}  // namespace sbf

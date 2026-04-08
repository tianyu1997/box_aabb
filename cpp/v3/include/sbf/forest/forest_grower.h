// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v3 — ForestGrower: multi-root forest growing engine
//  Module: sbf::forest
//
//  Grows a collision-free C-space box forest from multiple roots using:
//    - Wavefront: BFS boundary expansion with goal-directed face bias
//    - RRT-like:  random sampling + nearest-box extension
//
//  Key features:
//    - Multi-root with FPS selection (or forced start/goal endpoints)
//    - LECT KD-tree subtree partitioning for future parallelism
//    - Sweep-and-prune accelerated adjacency graph
//    - Per-box expansion metadata (parent_box_id, expand_face)
//    - Promotion (leaf merging) reusing LECT collision checks
//
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/forest/adjacency_graph.h"
#include "sbf/forest/grower_config.h"
#include "sbf/forest/lect.h"
#include "sbf/robot/robot.h"
#include "sbf/common/types.h"

#include <Eigen/Core>
#include <atomic>
#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

namespace sbf {
namespace forest {

// ─── GrowerResult ───────────────────────────────────────────────────────────
struct GrowerResult {
    std::vector<BoxNode> boxes;
    int  n_roots              = 0;
    int  n_boxes_total        = 0;
    int  n_ffb_success        = 0;
    int  n_ffb_fail           = 0;
    int  n_promotions         = 0;
    int  n_bridge_boxes       = 0;
    int  n_coarse_boxes       = 0;     // boxes created during coarse phase (adaptive min_edge)
    int  n_fine_boxes         = 0;     // boxes created during fine phase
    int  n_coarsen_merges     = 0;     // greedy coarsening merges performed
    int  n_components         = 0;
    bool start_goal_connected = false;
    double total_volume       = 0.0;
    double build_time_ms      = 0.0;
    std::unordered_map<std::string, double> phase_times;
};

// ─── ParallelWorkerResult — carries both boxes and the worker's LECT ────────
struct ParallelWorkerResult {
    GrowerResult result;
    LECT         lect;   // worker's mutated LECT for transplant
};

// ═════════════════════════════════════════════════════════════════════════════
//  ForestGrower
// ═════════════════════════════════════════════════════════════════════════════
class ForestGrower {
public:
    ForestGrower() = default;

    /// Construct from robot model and grower configuration.
    ForestGrower(const Robot& robot, const GrowerConfig& config);

    /// Construct with a pre-built (warm-start) LECT.
    /// The LECT is moved in; the constructor does NOT call init_root().
    /// Used by parallel workers to skip redundant root FK + envelope computation.
    ForestGrower(const Robot& robot, const GrowerConfig& config, LECT warm_lect);

    // ── Endpoint setting (optional) ─────────────────────────────────────
    /// Set start/goal endpoints. Both will be forced as root 0 and root 1.
    void set_endpoints(const Eigen::VectorXd& start,
                       const Eigen::VectorXd& goal);

    // ── Main entry point ────────────────────────────────────────────────
    /// Grow the forest and return results.
    GrowerResult grow(const Obstacle* obstacles, int n_obs);

    /// Grow a single subtree from a given root seed within subtree_limits.
    /// Used internally by parallel grow; exposed for testing.
    /// If shared_box_count is provided, it is atomically incremented per box
    /// and the global max_boxes budget is enforced via the shared counter.
    GrowerResult grow_subtree(const Eigen::VectorXd& root_seed,
                              int root_id,
                              const std::vector<Interval>& subtree_limits,
                              const Obstacle* obs, int n_obs,
                              std::shared_ptr<std::atomic<int>> shared_box_count = nullptr);

    // ── Accessors ───────────────────────────────────────────────────────
    const LECT&           lect()   const { return lect_; }
    LECT&                 lect_mut()     { return lect_; }
    const AdjacencyGraph& graph()  const { return graph_; }
    const std::vector<BoxNode>& boxes() const { return boxes_; }
    const GrowerConfig&   config() const { return config_; }
    int n_boxes() const { return static_cast<int>(boxes_.size()); }

    /// Whether the envelope type uses voxel grids.
    bool is_grid_envelope() const;

private:
    const Robot* robot_ = nullptr;
    GrowerConfig config_;
    LECT         lect_;
    AdjacencyGraph graph_;
    std::vector<BoxNode> boxes_;
    int next_box_id_ = 0;

    // ── Endpoints ───────────────────────────────────────────────────────
    bool has_endpoints_ = false;
    Eigen::VectorXd start_config_;
    Eigen::VectorXd goal_config_;
    int start_box_id_ = -1;
    int goal_box_id_  = -1;

    // ── RNG ─────────────────────────────────────────────────────────────
    std::mt19937_64 rng_;

    // ── Fast box lookup ─────────────────────────────────────────────────
    std::unordered_map<int, int> box_id_to_idx_;  // box_id → index in boxes_

    // ── Work-stealing ───────────────────────────────────────────────────
    std::shared_ptr<std::atomic<int>> shared_box_count_;  // global atomic counter (nullptr in serial mode)

    // ── Adaptive min_edge phase counters ─────────────────────────────────
    int n_coarse_created_ = 0;  // boxes created during coarse phase
    int n_fine_created_   = 0;  // boxes created during fine phase

    // ── Subtree partitioning ────────────────────────────────────────────
    struct SubtreeInfo {
        int root_id = -1;                    // which root this subtree belongs to
        std::vector<Interval> limits;        // C-space sub-region for this root
        int lect_node_idx = -1;              // corresponding LECT KD-tree node index
    };
    std::vector<SubtreeInfo> subtrees_;       // one per root

    // ── Root selection ──────────────────────────────────────────────────
    void select_roots(const Obstacle* obs, int n_obs);
    void select_roots_no_endpoints(const Obstacle* obs, int n_obs);
    void select_roots_with_endpoints(const Obstacle* obs, int n_obs);

    // ── Subtree partitioning ────────────────────────────────────────────
    /// Partition C-space into per-root sub-regions by simulating KD-tree descent.
    void partition_subtrees();

    /// Partition C-space uniformly into n_roots cells by recursive
    /// widest-dimension bisection (independent of root positions).
    /// Used for partition-first parallel mode.
    void partition_uniform();

    /// Partition C-space aligned with LECT KD-tree splitting order.
    /// Uses dim = depth % n_dims and midpoint splitting (matching split_leaf).
    /// n_roots must be a power of 2.  Each cell records its LECT node index
    /// in SubtreeInfo::lect_node_idx for later transplant.
    /// Also pre-expands the LECT to partition depth.
    void partition_lect_aligned();

    /// Place one root box within each partition cell.
    /// Requires subtrees_ to be populated by partition_uniform/lect_aligned().
    void select_roots_in_partitions(const Obstacle* obs, int n_obs);

    // ── Expansion modes ─────────────────────────────────────────────────
    void grow_wavefront(const Obstacle* obs, int n_obs);
    void grow_rrt(const Obstacle* obs, int n_obs);

    /// Parallel expansion: dispatches per-subtree workers via ThreadPool.
    void grow_parallel(const Obstacle* obs, int n_obs, GrowerResult& result);

    /// Bridge gaps between adjacent subtree regions after parallel merge.
    /// Samples seeds on shared faces of subtree partitions and creates bridge boxes.
    /// Uses proximity-based seeding and alternating root_ids for cross-tree connectivity.
    void bridge_subtree_boundaries(const Obstacle* obs, int n_obs,
                                   GrowerResult& result);

    /// Sync coordinator LECT occupation from merged worker boxes.
    /// Walks the tree for each box to find the containing leaf, then marks it.
    /// Called before bridge creation to prevent overlapping boxes.
    void sync_lect_occupation();

    // ── Boundary sampling (for wavefront) ───────────────────────────────
    struct BoundarySeed {
        int dim;
        int side;  // 0 = lo face, 1 = hi face
        Eigen::VectorXd config;
    };
    std::vector<BoundarySeed> sample_boundary(
        const BoxNode& box,
        const Eigen::VectorXd* bias_target);

    // ── Per-tree goal bias ────────────────────────────────────────────
    /// Return the bias target for a given root_id in multi-tree growth.
    ///   root 0 (start tree) → goal_config
    ///   root 1 (goal tree)  → start_config
    ///   root ≥2 (random tree) → randomly start or goal (50/50)
    /// Returns nullptr when no endpoints are set.
    const Eigen::VectorXd* get_bias_target(int root_id);

    // ── FFB wrapper ─────────────────────────────────────────────────────
    /// Try to create a box from a seed. Returns box_id on success, -1 on failure.
    /// If min_edge_override > 0, use it instead of config_.min_edge.
    int try_create_box(const Eigen::VectorXd& seed,
                       const Obstacle* obs, int n_obs,
                       int parent_box_id = -1,
                       int face_dim = -1, int face_side = -1,
                       int root_id = -1,
                       double min_edge_override = -1.0);

    // ── Promotion (reuse SBF patterns) ──────────────────────────────────
    int promote_all(const Obstacle* obs, int n_obs);
    bool try_promote_node(int node_idx, const Obstacle* obs, int n_obs);

    // ── Coarsening (greedy merge) ─────────────────────────────────────
    int coarsen_greedy(const Obstacle* obs, int n_obs);
    void remove_box_by_id(int box_id);

    // ── Boundary-aware sampling (reduce isolated small boxes) ────────
    /// Sample a seed on a random face of a random existing box.
    /// If root_id >= 0, only consider boxes of that root. Returns zero vector on failure.
    Eigen::VectorXd sample_near_existing_boundary(int root_id = -1);

    /// RRT boundary-snap: find the best face of `nearest` box aligned with
    /// `direction`, place seed on that face (directional + random jitter).
    /// Returns (seed, face_dim, face_side). face_dim = -1 on fallback.
    struct SnapResult {
        Eigen::VectorXd seed;
        int face_dim  = -1;
        int face_side = -1;
    };
    SnapResult rrt_snap_to_face(const BoxNode& nearest,
                                const Eigen::VectorXd& direction,
                                double step);

    // ── Helpers ─────────────────────────────────────────────────────────
    Eigen::VectorXd sample_random();
    Eigen::VectorXd sample_random_in_subtree(int root_id);
    Eigen::VectorXd sample_random_in_region(const Eigen::VectorXd& center,
                                            double sigma);
};

} // namespace forest
} // namespace sbf

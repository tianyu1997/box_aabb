// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — ForestGrower: multi-root forest growing engine
//  Module: sbf::forest
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/forest/adjacency_graph.h"
#include "sbf/forest/grower_config.h"
#include "sbf/forest/lect.h"
#include "sbf/robot/robot.h"
#include "sbf/core/types.h"

#include <Eigen/Core>
#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
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
    int  n_coarse_boxes       = 0;
    int  n_fine_boxes         = 0;
    int  n_coarsen_merges     = 0;
    int  n_components         = 0;
    bool start_goal_connected = false;
    double total_volume       = 0.0;
    double build_time_ms      = 0.0;
    std::unordered_map<std::string, double> phase_times;
};

// ─── ParallelWorkerResult ───────────────────────────────────────────────────
struct ParallelWorkerResult {
    GrowerResult result;
    LECT         lect;
};

// ═════════════════════════════════════════════════════════════════════════════
//  ForestGrower
// ═════════════════════════════════════════════════════════════════════════════
class ForestGrower {
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    ForestGrower() = default;
    ForestGrower(const Robot& robot, const GrowerConfig& config);
    ForestGrower(const Robot& robot, const GrowerConfig& config, LECT warm_lect);

    // ── Endpoint setting ────────────────────────────────────────────────
    void set_endpoints(const Eigen::VectorXd& start,
                       const Eigen::VectorXd& goal);

    // ── Main entry point ────────────────────────────────────────────────
    GrowerResult grow(const Obstacle* obstacles, int n_obs);

    GrowerResult grow_subtree(const Eigen::VectorXd& root_seed,
                              int root_id,
                              const std::vector<Interval>& subtree_limits,
                              const Obstacle* obs, int n_obs,
                              std::shared_ptr<std::atomic<int>> shared_box_count = nullptr);

    // ── Accessors ───────────────────────────────────────────────────────
    const LECT&           lect()   const;
    LECT&                 lect_mut();
    const AdjacencyGraph& graph()  const;
    const std::vector<BoxNode>& boxes() const;
    const GrowerConfig&   config() const;
    int n_boxes() const;
    bool is_grid_envelope() const;

private:
    const Robot* robot_ = nullptr;
    GrowerConfig config_;
    LECT         lect_;
    AdjacencyGraph graph_;
    std::vector<BoxNode> boxes_;
    int next_box_id_ = 0;

    bool has_endpoints_ = false;
    Eigen::VectorXd start_config_;
    Eigen::VectorXd goal_config_;
    int start_box_id_ = -1;
    int goal_box_id_  = -1;

    std::mt19937_64 rng_;

    std::unordered_map<int, int> box_id_to_idx_;
    std::shared_ptr<std::atomic<int>> shared_box_count_;

    int n_coarse_created_ = 0;
    int n_fine_created_   = 0;
    bool lect_cache_loaded_ = false;
    std::optional<TimePoint> deadline_;

    struct SubtreeInfo {
        int root_id = -1;
        std::vector<Interval> limits;
        int lect_node_idx = -1;
    };
    std::vector<SubtreeInfo> subtrees_;

    struct BoundarySeed {
        int dim;
        int side;
        Eigen::VectorXd config;
    };

    struct SnapResult {
        Eigen::VectorXd seed;
        int face_dim  = -1;
        int face_side = -1;
    };

    void select_roots(const Obstacle* obs, int n_obs);
    void select_roots_no_endpoints(const Obstacle* obs, int n_obs);
    void select_roots_with_endpoints(const Obstacle* obs, int n_obs);
    void partition_subtrees();
    void partition_uniform();
    void partition_lect_aligned();
    void select_roots_in_partitions(const Obstacle* obs, int n_obs);

    void grow_wavefront(const Obstacle* obs, int n_obs);
    void grow_rrt(const Obstacle* obs, int n_obs);
    void grow_parallel(const Obstacle* obs, int n_obs, GrowerResult& result);

    int promote_all(const Obstacle* obs, int n_obs);
    bool try_promote_node(int node_idx, const Obstacle* obs, int n_obs);

    int coarsen_greedy(const Obstacle* obs, int n_obs);
    void remove_box_by_id(int box_id);

    void sync_lect_occupation();
    void bridge_subtree_boundaries(const Obstacle* obs, int n_obs,
                                   GrowerResult& result);

    std::vector<BoundarySeed> sample_boundary(
        const BoxNode& box,
        const Eigen::VectorXd* bias_target);

    const Eigen::VectorXd* get_bias_target(int root_id);

    int try_create_box(const Eigen::VectorXd& seed,
                       const Obstacle* obs, int n_obs,
                       int parent_box_id = -1,
                       int face_dim = -1, int face_side = -1,
                       int root_id = -1,
                       double min_edge_override = -1.0,
                       int max_depth_override = -1);

    Eigen::VectorXd sample_near_existing_boundary(int root_id = -1);
    SnapResult rrt_snap_to_face(const BoxNode& nearest,
                                const Eigen::VectorXd& direction,
                                double step);

    Eigen::VectorXd sample_random();
    Eigen::VectorXd sample_random_in_subtree(int root_id);
    Eigen::VectorXd sample_random_in_region(const Eigen::VectorXd& center,
                                            double sigma);

    void set_deadline(std::optional<TimePoint> deadline);
    bool deadline_reached() const;
};

} // namespace forest
} // namespace sbf

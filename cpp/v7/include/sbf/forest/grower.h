#pragma once
/// @file grower.h
/// @brief Forest grower over LECT — single- and multi-threaded.
///
/// v7 P4.5: multi-thread default-on. Each worker owns an independent
/// LECT snapshot restricted to a geometric domain (subtree). After
/// growth, workers' subtrees are transplanted back into the master LECT.

#include "sbf/core/robot.h"
#include "sbf/forest/adjacency.h"
#include "sbf/forest/ffb.h"
#include "sbf/lect/lect.h"
#include "sbf/scene/box_node.h"

#include <Eigen/Core>
#include <cstdint>
#include <thread>
#include <vector>

namespace sbf::forest {

struct GrowerConfig {
    int      max_boxes              = 500;
    double   timeout_ms             = 30000.0;
    int      max_consecutive_miss   = 2000;
    double   rrt_goal_bias          = 0.1;
    double   rrt_step_ratio         = 0.10;
    bool     connect_mode           = true;
    bool     stop_after_connect     = false;
    int      post_connect_extra_boxes = 0;
    bool     endpoint_auto_bridge   = true;
    int      endpoint_bridge_max_boxes = 0;   // 0 → clamp(max_boxes/20, 50, 500)
    uint64_t rng_seed               = 42;
    /// 0 / negative → auto-detect via std::thread::hardware_concurrency().
    int      n_threads              = 0;
    /// Cap how many extra LECT splits partition_for_seeds may use.
    int      partition_max_splits   = 256;
    FFBConfig ffb;
};

struct GrowerResult {
    std::vector<sbf::scene::BoxNode> boxes;
    int    n_ffb_success           = 0;
    int    n_ffb_fail              = 0;
    int    n_bridge_boxes          = 0;
    int    start_box               = -1;
    int    goal_box                = -1;
    bool   start_goal_connected    = false;
    bool   adjacency_all_connected = false;
    int    adjacency_islands       = 0;
    int    adjacency_largest_island = 0;
    int    n_workers_used          = 1;
    double build_time_ms           = 0.0;
    AdjacencyGraph adjacency;
};

class ForestGrower {
public:
    ForestGrower(const sbf::core::Robot& robot,
                 sbf::lect::LECT& lect,
                 GrowerConfig cfg = {});

    /// Set q_start and q_goal in joint space. Both will be turned into
    /// initial boxes (subject to FFB success).
    void set_endpoints(const Eigen::VectorXd& q_start,
                       const Eigen::VectorXd& q_goal);

    /// Seed multiple canonical roots and grow a shared coverage forest.
    void set_multi_goals(const std::vector<Eigen::VectorXd>& goals);

    /// Restrict this grower instance to a single geometric subtree.
    /// All sampled `q` are clamped inside the domain root's intervals,
    /// and FFB results outside the subtree are rejected. Used by
    /// grow_parallel workers.
    void set_domain_root(int master_node_idx) { domain_root_ = master_node_idx; }

    GrowerResult grow(const float* obs_compact, int n_obs);

private:
    GrowerResult grow_serial(const float* obs_compact, int n_obs);
    GrowerResult grow_parallel(const float* obs_compact, int n_obs);

    const sbf::core::Robot& robot_;
    sbf::lect::LECT&        lect_;
    GrowerConfig            cfg_;
    Eigen::VectorXd         q_start_;
    Eigen::VectorXd         q_goal_;
    std::vector<Eigen::VectorXd> multi_goals_;
    bool                    has_endpoints_ = false;
    bool                    has_multi_goals_ = false;
    int                     domain_root_   = -1;     // -1 = whole LECT
};

}  // namespace sbf::forest


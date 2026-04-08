// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — SBF-based motion planner
//  Module: sbf::planner
//
//  Uses SafeBoxForest + AdjacencyGraph for collision-free path search:
//    1. Build SBF forest (offline or on-the-fly)
//    2. Find boxes containing start and goal (FFB)
//    3. Search adjacency graph for path (BFS/A*)
//    4. Extract waypoints through box sequence
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/planner/i_planner.h"
#include "sbf/forest/sbf.h"
#include "sbf/forest/forest_grower.h"
#include "sbf/core/types.h"

#include <memory>

namespace sbf {
namespace planner {

/// SBF-based motion planner configuration.
struct SBFPlannerConfig {
    // Forest configuration
    forest::GrowerConfig grower_config;

    // Maximum planning time (seconds)
    double timeout = 60.0;

    // Path post-processing
    bool shortcut_enabled = true;
    int  max_shortcut_iters = 100;
};

/// SBF-based motion planner.
///
/// Builds a SafeBoxForest covering the free C-space, then searches
/// the box adjacency graph for a path from start to goal.
class SBFPlanner : public IPlanner {
public:
    SBFPlanner() = default;

    /// Construct with robot model and planner configuration.
    SBFPlanner(const Robot& robot,
               const SBFPlannerConfig& config);

    /// Set the scene (obstacles). Must be called before plan().
    void set_scene(const Obstacle* obstacles, int n_obs);

    /// Plan a path from start to goal.
    PathResult plan(const Eigen::VectorXd& start,
                    const Eigen::VectorXd& goal) override;

    // ── Accessors ───────────────────────────────────────────────────────
    const forest::ForestGrower& grower() const;
    const forest::AdjacencyGraph& graph() const;

private:
    const Robot* robot_ = nullptr;
    SBFPlannerConfig config_;

    // Scene
    std::vector<Obstacle> obstacles_;

    // Forest (built during plan())
    std::unique_ptr<forest::ForestGrower> grower_;

    // TODO: 实现路径搜索和后处理
};

} // namespace planner
} // namespace sbf

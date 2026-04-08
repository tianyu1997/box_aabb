// SafeBoxForest — Forest Grower: orchestrates box growth
// Module: sbf::forest
// Extracted from SBFPlanner::grow_boxes() to be a standalone component.
#pragma once

#include "sbf/common/types.h"
#include "sbf/common/config.h"
#include "sbf/robot/robot.h"
#include "sbf/scene/aabb_collision_checker.h"
#include "sbf/forest/safe_box_forest.h"
#include "sbf/forest/root_sampler.h"
#include "sbf/forest/boundary_sampler.h"
#include "sbf/forest/ffb.h"
#include "sbf/cache/hier_aabb_tree.h"
#include <Eigen/Core>
#include <functional>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace sbf {
namespace forest {

// ─── Forest Grower ──────────────────────────────────────────────────────────
// Orchestrates:
//   - RootSampler (uniform + goal-biased seed generation)
//   - BoundarySampler (directed face expansion)
//   - FFBEngine (find_free_box with phase management)
//   - Origin tracking (s/g/r tags for bidirectional BFS)
//   - Connection detection (overlap, containment, absorption bridge)
class ForestGrower {
public:
    ForestGrower() = default;
    ForestGrower(const Robot& robot,
                 const CollisionChecker& checker,
                 HierAABBTree& tree,
                 SafeBoxForest& forest,
                 const SBFConfig& config);

    // ── Single-pair growth ───────────────────────────────────────────────
    // Bidirectional BFS growth between start and goal.
    // Returns true if start↔goal became connected.
    bool grow(const Eigen::VectorXd& start,
              const Eigen::VectorXd& goal);

    // ── Multi-pair growth ────────────────────────────────────────────────
    // Phase 1: isolated per-pair growth
    // Phase 2: guided random BFS
    // Phase 3: bridge disconnected pairs
    void grow_multi(
        const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& pairs,
        int n_random_boxes = 5000,
        double timeout = 120.0);

    // ── Targeted regrowth ────────────────────────────────────────────────
    int regrow(int n_target, double timeout = 60.0);

    // ── Access ───────────────────────────────────────────────────────────
    const std::unordered_map<int,char>& box_origins() const { return box_origin_; }

private:
    const Robot* robot_ = nullptr;
    const CollisionChecker* checker_ = nullptr;
    HierAABBTree* tree_ = nullptr;
    SafeBoxForest* forest_ = nullptr;
    SBFConfig config_;

    RootSampler root_sampler_;
    BoundarySampler boundary_sampler_;
    FFBEngine ffb_engine_;

    // Origin tracking
    std::unordered_map<int,char> box_origin_;  // box_id → 's'/'g'/'r'
    std::unordered_set<int> start_box_ids_;
    std::unordered_set<int> goal_box_ids_;
    std::unordered_set<int> random_box_ids_;

    bool boxes_touch(const BoxNode& a, const BoxNode& b, double tol = 1e-8) const;
};

} // namespace forest
} // namespace sbf

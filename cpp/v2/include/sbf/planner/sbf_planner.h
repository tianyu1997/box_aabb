// SafeBoxForest v2 — Main SBF Planner
// Module: sbf (planner)
#pragma once

#include "sbf/common/config.h"
#include "sbf/robot/robot.h"
#include "sbf/common/types.h"
#include "sbf/scene/aabb_collision_checker.h"
#include "sbf/forest/hier_aabb_tree.h"
#include "sbf/forest/safe_box_forest.h"
#include "sbf/planner/connector.h"
#include "sbf/planner/path_smoother.h"
#include <Eigen/Core>
#include <deque>
#include <random>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace sbf {

// ── Boundary seed for directed edge expansion ────────────────────────────
struct BoundarySeed {
    int dim;
    int side;  // 0=lo, 1=hi
    Eigen::VectorXd config;
};

class SBFPlanner {
public:
    SBFPlanner() = default;
    SBFPlanner(const Robot& robot,
               const std::vector<Obstacle>& obstacles,
               const SBFConfig& config = SBFConfig());

    // ── Main planning interface ──────────────────────────────────────────
    PlanningResult plan(const Eigen::VectorXd& start,
                        const Eigen::VectorXd& goal,
                        double timeout = 30.0);

    // ── Build forest only (no direct-connection shortcut) ────────────────
    void build(const Eigen::VectorXd& start,
               const Eigen::VectorXd& goal,
               double timeout = 30.0);
    // ── Grow boxes in isolation (for sub-planners in build_multi) ─────────
    // Returns true if start↔goal became connected.
    bool grow_only(const Eigen::VectorXd& start, const Eigen::VectorXd& goal);
    // ── Build multi-pair forest ──────────────────────────────────────────
    // Phase 1: grow boxes_per_pair boxes for each (start, goal) pair
    // Phase 2: grow n_random_boxes random boxes (KD-tree free-space sampling)
    // Phase 3: bridge any disconnected s-t pair
    void build_multi(
        const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& pairs,
        int n_random_boxes = 5000,
        double timeout = 120.0);

    // ── Reusable state ───────────────────────────────────────────────────
    PlanningResult query(const Eigen::VectorXd& start,
                         const Eigen::VectorXd& goal,
                         double timeout = 10.0);

    // ── Incremental update ───────────────────────────────────────────────
    void add_obstacle(const Obstacle& obs);
    void remove_obstacle(const std::string& name);

    // ── Targeted regrowth ────────────────────────────────────────────────
    int regrow(int n_target, double timeout = 60.0);

    // ── Warm rebuild support ─────────────────────────────────────────────
    void clear_forest();

    // ── Access ───────────────────────────────────────────────────────────
    const SafeBoxForest& forest() const { return forest_; }
    const HierAABBTree& tree() const { return tree_; }
    const CollisionChecker& collision_checker() const { return checker_; }
    const TreeConnector& connector() const { return connector_; }
    const SBFConfig& config() const { return config_; }
    SBFConfig& config() { return config_; }
    bool forest_built() const { return forest_built_; }
    bool has_proxy_start() const { return has_proxy_start_; }
    bool has_proxy_goal()  const { return has_proxy_goal_; }
    const Eigen::VectorXd& proxy_start() const { return proxy_start_; }
    const Eigen::VectorXd& proxy_goal()  const { return proxy_goal_; }

private:
    Robot robot_;
    std::vector<Obstacle> obstacles_;
    SBFConfig config_;

    HierAABBTree tree_;
    SafeBoxForest forest_;
    CollisionChecker checker_;
    PathSmoother smoother_;
    TreeConnector connector_;

    bool forest_built_ = false;
    std::mt19937 rng_;

    // Proxy anchor state
    Eigen::VectorXd proxy_start_, proxy_goal_;
    bool has_proxy_start_ = false, has_proxy_goal_ = false;

    // ── Pipeline steps ───────────────────────────────────────────────────
    void build_forest(const Eigen::VectorXd& start,
                      const Eigen::VectorXd& goal);
    bool grow_boxes(const Eigen::VectorXd& start,
                    const Eigen::VectorXd& goal,
                    bool skip_final_adjacency = false);
    void expand_boundaries();
    void coarsen_and_connect();

    // ── Seed sampling (v4 logic) ─────────────────────────────────────────
    Eigen::VectorXd sample_seed(const Eigen::VectorXd& goal);
    Eigen::VectorXd sample_uniform();

    // ── Directed boundary expansion (v4 logic) ──────────────────────────
    std::vector<BoundarySeed> generate_boundary_seeds(
        const BoxNode& box,
        const std::set<std::pair<int,int>>& excluded_faces,
        const Eigen::VectorXd* goal_point,
        int n_samples,
        double eps_override = -1.0);

    // ── Proxy anchor for narrow-passage start/goal ────────────────────
    int try_create_proxy_anchor(
        const Eigen::VectorXd& q, char origin_tag,
        std::unordered_map<int,char>& box_origin,
        std::unordered_set<int>& start_box_ids,
        std::unordered_set<int>& goal_box_ids,
        std::unordered_set<int>& random_box_ids,
        std::unordered_map<int,int>& random_ancestor,
        Eigen::VectorXd& proxy_out, bool& used_proxy);

    // ── Origin tracking helpers ──────────────────────────────────────────
    bool boxes_touch(const BoxNode& a, const BoxNode& b, double tol = 1e-8) const;

    // ── Cache persistence ────────────────────────────────────────────────
    void save_tree_cache();
};

} // namespace sbf

// SafeBoxForest — Pipeline: top-level grow + plan functions
#pragma once

#include "sbf/core/config.h"
#include "sbf/core/robot.h"
#include "sbf/core/types.h"
#include "sbf/planner/sbf_planner.h"
#include <Eigen/Core>
#include <random>
#include <vector>

namespace sbf {

// ─── Scene generation ───────────────────────────────────────────────────────
struct SceneConfig {
    int n_obstacles = 6;
    double workspace_radius = 0.85;
    double workspace_z_min = 0.0;
    double workspace_z_max = 1.0;
    double obs_size_min = 0.08;
    double obs_size_max = 0.25;
};

// Generate random obstacle scene
std::vector<Obstacle> build_random_scene(const Robot& robot,
                                          const SceneConfig& scene_cfg,
                                          const Eigen::VectorXd& q_start,
                                          const Eigen::VectorXd& q_goal,
                                          std::mt19937& rng,
                                          int max_trials = 100);

// ─── Top-level pipeline ─────────────────────────────────────────────────────

// Build SBF planner config from default Panda setup
SBFConfig make_panda_config(int seed = 0);

// One-shot plan: build forest + plan path
PlanningResult plan_once(const Robot& robot,
                          const std::vector<Obstacle>& obstacles,
                          const Eigen::VectorXd& start,
                          const Eigen::VectorXd& goal,
                          const SBFConfig& config = SBFConfig());

} // namespace sbf

// SafeBoxForest v2 — pipeline convenience functions
// Module: sbf (planner)
#include "sbf/planner/pipeline.h"
#include <cmath>

namespace sbf {

std::vector<Obstacle> build_random_scene(const Robot& robot,
                                          const SceneConfig& scene_cfg,
                                          const Eigen::VectorXd& q_start,
                                          const Eigen::VectorXd& q_goal,
                                          std::mt19937& rng,
                                          int max_trials) {
    std::uniform_real_distribution<double> unif(0.0, 1.0);
    std::vector<Obstacle> obs;
    obs.reserve(scene_cfg.n_obstacles);

    // Compute start/goal link positions to avoid blocking them
    auto start_pos = robot.fk_link_positions(q_start);
    auto goal_pos  = robot.fk_link_positions(q_goal);

    for (int trial = 0; trial < max_trials && static_cast<int>(obs.size()) < scene_cfg.n_obstacles; ++trial) {
        double sx = scene_cfg.obs_size_min + unif(rng) * (scene_cfg.obs_size_max - scene_cfg.obs_size_min);
        double sy = scene_cfg.obs_size_min + unif(rng) * (scene_cfg.obs_size_max - scene_cfg.obs_size_min);
        double sz = scene_cfg.obs_size_min + unif(rng) * (scene_cfg.obs_size_max - scene_cfg.obs_size_min);

        Eigen::Vector3d half(sx * 0.5, sy * 0.5, sz * 0.5);

        double r = scene_cfg.workspace_radius;
        double angle = unif(rng) * 2.0 * PI;
        double dist  = std::sqrt(unif(rng)) * r;
        double cx = dist * std::cos(angle);
        double cy = dist * std::sin(angle);
        double cz = scene_cfg.workspace_z_min + half.z() +
                     unif(rng) * (scene_cfg.workspace_z_max - scene_cfg.workspace_z_min - 2 * half.z());

        Eigen::Vector3d center(cx, cy, cz);

        // Check that obstacle doesn't block start/goal positions too closely
        bool blocks = false;
        for (int i = 0; i < start_pos.rows() && !blocks; ++i) {
            Eigen::Vector3d p = start_pos.row(i);
            if ((p - center).cwiseAbs().maxCoeff() < half.maxCoeff() * 1.5)
                blocks = true;
        }
        for (int i = 0; i < goal_pos.rows() && !blocks; ++i) {
            Eigen::Vector3d p = goal_pos.row(i);
            if ((p - center).cwiseAbs().maxCoeff() < half.maxCoeff() * 1.5)
                blocks = true;
        }
        if (blocks) continue;

        Obstacle o;
        o.name = "obs_" + std::to_string(obs.size());
        o.center = center;
        o.half_sizes = half;
        obs.push_back(o);
    }
    return obs;
}

SBFConfig make_panda_config(int seed) {
    SBFConfig cfg;
    cfg.max_boxes = 500;
    cfg.ffb_max_depth = 100;
    cfg.ffb_min_edge = 0.01;
    cfg.ffb_min_edge_anchor = 0.001;
    cfg.ffb_min_edge_relaxed = 0.05;
    cfg.guided_sample_ratio = 0.60;
    cfg.max_consecutive_miss = 20;
    cfg.coarsen_max_rounds = 20;
    cfg.shortcut_max_iters = 300;
    cfg.adjacency_tol = 1e-8;
    cfg.segment_resolution = 0.05;
    cfg.n_edge_samples = 3;
    cfg.boundary_expand_epsilon = 0.01;
    cfg.bfs_phase_k     = {5.0, 2.0, 1.0, 0.5, 0.1};
    cfg.bfs_phase_budget = {500, 1000, 1000, 2000, 2000};
    cfg.seed = seed;
    return cfg;
}

PlanningResult plan_once(const Robot& robot,
                          const std::vector<Obstacle>& obstacles,
                          const Eigen::VectorXd& start,
                          const Eigen::VectorXd& goal,
                          const SBFConfig& config) {
    SBFPlanner planner(robot, obstacles, config);
    return planner.plan(start, goal, 30.0);
}

} // namespace sbf

#pragma once
/// @file scene_config.h
/// @brief JSON-driven experiment scene description.
///
/// A `SceneConfig` bundles everything needed to run a planning trial:
/// robot id, joint-space endpoints (q_start, q_goal), and a list of
/// workspace obstacles (axis-aligned boxes).

#include "sbf/scene/obstacle.h"

#include <Eigen/Core>
#include <string>
#include <vector>

namespace sbf::scene {

struct SceneConfig {
    std::string           name;          ///< human-readable id
    std::string           robot;         ///< "iiwa14" | "panda" | "2dof_planar"
    Eigen::VectorXd       q_start;
    Eigen::VectorXd       q_goal;
    std::vector<Obstacle> obstacles;

    /// Pack obstacles into a flat float array suitable for grower/planner.
    /// Returns vector of size 6 * obstacles.size().
    std::vector<float> packed_obstacles() const;
};

/// Parse a SceneConfig from a JSON file. Schema:
/// {
///   "name": "iiwa14_far",
///   "robot": "iiwa14",
///   "q_start": [0,0,0,0,0,0,0],
///   "q_goal":  [0.5,0.3,0,0,0,0,0],
///   "obstacles": [
///     {"lo": [20,20,20], "hi": [21,21,21]}
///   ]
/// }
SceneConfig load_scene_json(const std::string& path);

}  // namespace sbf::scene

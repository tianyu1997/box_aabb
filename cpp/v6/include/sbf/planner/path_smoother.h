#pragma once
/// @file path_smoother.h
/// @brief Path smoothing: random shortcutting + moving-average refinement.
///
/// Two smoothing passes applied sequentially:
///   1. **Shortcut**: randomly pick two path points; if the direct segment
///      is collision-free, remove intermediate waypoints.
///   2. **Moving average**: slide each waypoint toward its neighbours,
///      clamped to the containing box.

#include <sbf/core/types.h>
#include <sbf/scene/collision_checker.h>

#include <Eigen/Dense>
#include <cstdint>
#include <vector>

namespace sbf {

/// @brief Configuration for path smoothing.
struct SmootherConfig {
    int shortcut_max_iters = 100;       ///< Max random shortcut iterations.
    int smooth_window = 3;              ///< Moving-average window size.
    int smooth_iters = 5;               ///< Number of moving-average passes.
    int segment_resolution = 10;        ///< Samples per segment for collision check.
};

// Random shortcut: randomly pick two points, if direct connection is
// collision-free then remove intermediate waypoints.
std::vector<Eigen::VectorXd> shortcut(
    const std::vector<Eigen::VectorXd>& path,
    const CollisionChecker& checker,
    const SmootherConfig& config,
    uint64_t seed = 42);

// Moving-average smoothing with box clamping.
std::vector<Eigen::VectorXd> smooth_moving_average(
    const std::vector<Eigen::VectorXd>& path,
    const std::vector<BoxNode>& box_sequence,
    const SmootherConfig& config);

// Combined: shortcut → smooth.
std::vector<Eigen::VectorXd> smooth_path(
    const std::vector<Eigen::VectorXd>& path,
    const std::vector<BoxNode>& box_sequence,
    const CollisionChecker& checker,
    const SmootherConfig& config);

// Utility
double path_length(const std::vector<Eigen::VectorXd>& path);

}  // namespace sbf

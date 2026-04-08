// SafeBoxForest v2 — Path smoothing: shortcut + moving average
// Module: sbf (planner)
#pragma once

#include "sbf/common/types.h"
#include "sbf/scene/aabb_collision_checker.h"
#include "sbf/forest/safe_box_forest.h"
#include <Eigen/Core>
#include <random>
#include <vector>

namespace sbf {

class PathSmoother {
public:
    PathSmoother() = default;
    PathSmoother(const CollisionChecker& checker,
                 double segment_resolution = 0.05);

    // ── Random shortcutting ──────────────────────────────────────────────
    // Randomly pick two points on the path, try to connect them directly
    std::vector<Eigen::VectorXd>
    shortcut(const std::vector<Eigen::VectorXd>& path,
             int max_iters = 100,
             std::mt19937* rng = nullptr) const;

    // ── Box-aware shortcutting ───────────────────────────────────────────
    // Only shortcuts within/between adjacent boxes (guaranteed safe)
    std::vector<Eigen::VectorXd>
    box_aware_shortcut(const std::vector<Eigen::VectorXd>& path,
                       const SafeBoxForest& forest,
                       int max_iters = 100,
                       std::mt19937* rng = nullptr) const;

    // ── Moving average smoothing ─────────────────────────────────────────
    // Smooth path with box-aware moving average (stay within boxes)
    std::vector<Eigen::VectorXd>
    smooth_moving_average(const std::vector<Eigen::VectorXd>& path,
                          const SafeBoxForest& forest,
                          int window = 5,
                          int iterations = 3) const;

    // ── Resample path ────────────────────────────────────────────────────
    // Resample path at uniform spacing
    std::vector<Eigen::VectorXd>
    resample(const std::vector<Eigen::VectorXd>& path,
             double resolution = 0.05) const;

    // ── Compute path length ──────────────────────────────────────────────
    static double path_length(const std::vector<Eigen::VectorXd>& path);

private:
    const CollisionChecker* checker_ = nullptr;
    double segment_resolution_ = 0.05;
};

} // namespace sbf

#pragma once
/// @file i_planner.h
/// @brief Abstract planner interface for SBF and external planners.
///
/// All planners (SBFPlanner, OMPLPlannerAdapter, etc.) implement this
/// interface so they can be swapped transparently in benchmarks.

#include <sbf/core/types.h>

#include <Eigen/Dense>
#include <vector>

namespace sbf {

struct PlanResult;  // forward, defined in sbf_planner.h

/// @brief Abstract planner interface.
class IPlanner {
public:
    virtual ~IPlanner() = default;

    /// @brief Plan a collision-free path from start to goal.
    /// @param start      Start configuration.
    /// @param goal       Goal configuration.
    /// @param obs        Obstacle array.
    /// @param n_obs      Number of obstacles.
    /// @param timeout_ms Wall-clock timeout in milliseconds.
    /// @return PlanResult with success flag, path, and statistics.
    virtual PlanResult plan(
        const Eigen::VectorXd& start,
        const Eigen::VectorXd& goal,
        const Obstacle* obs, int n_obs,
        double timeout_ms = 30000.0) = 0;
};

}  // namespace sbf

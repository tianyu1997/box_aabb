// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Abstract planner interface
//  Module: sbf::planner
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/planner/path_result.h"

#include <Eigen/Core>

namespace sbf {
namespace planner {

/// Abstract motion planner interface.
///
/// All planner implementations must inherit from this class and
/// implement the plan() method.
class IPlanner {
public:
    virtual ~IPlanner() = default;

    /// Plan a path from start to goal configuration.
    ///
    /// @param start  Start configuration (n_dims)
    /// @param goal   Goal configuration (n_dims)
    /// @return PathResult with path on success, or failure info
    virtual PathResult plan(const Eigen::VectorXd& start,
                            const Eigen::VectorXd& goal) = 0;
};

} // namespace planner
} // namespace sbf

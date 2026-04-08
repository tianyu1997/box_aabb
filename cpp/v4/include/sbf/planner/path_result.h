// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Path result type
//  Module: sbf::planner
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include <Eigen/Core>
#include <string>
#include <unordered_map>
#include <vector>

namespace sbf {
namespace planner {

/// Planning result: path + cost + timing + success flag.
struct PathResult {
    bool success = false;

    /// Waypoint sequence: each row is a q-configuration.
    Eigen::MatrixXd path;

    /// Total path cost (e.g. sum of L2 segment lengths in C-space).
    double cost = 0.0;

    /// Wall-clock planning time (seconds).
    double planning_time = 0.0;

    /// Time to first feasible solution (seconds, 0 if none).
    double first_solution_time = 0.0;

    /// Number of collision checks performed.
    int collision_checks = 0;

    /// Number of graph/tree nodes explored.
    int nodes_explored = 0;

    /// Per-phase timing breakdown.
    std::unordered_map<std::string, double> phase_times;

    /// Additional metadata (key-value pairs).
    std::unordered_map<std::string, double> metadata;

    int n_waypoints() const { return static_cast<int>(path.rows()); }

    static PathResult failure(double time = 0.0) {
        PathResult r;
        r.planning_time = time;
        return r;
    }
};

} // namespace planner
} // namespace sbf

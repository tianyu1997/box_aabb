// SafeBoxForest — Drake GCS Adapter
// Converts SBF forest to Drake GraphOfConvexSets for optimal path computation
#pragma once

#include "sbf/core/types.h"
#include "sbf/forest/safe_box_forest.h"

#ifdef SBF_WITH_DRAKE
#include <drake/geometry/optimization/graph_of_convex_sets.h>
#include <drake/geometry/optimization/hpolyhedron.h>
#include <drake/geometry/optimization/point.h>
#endif

#include <Eigen/Core>
#include <vector>

namespace sbf {

struct GCSResult {
    bool success = false;
    std::vector<Eigen::VectorXd> waypoints;
    std::vector<int> active_box_ids;
    double cost = 0.0;
    double solve_time = 0.0;
};

class GCSOptimizer {
public:
    GCSOptimizer() = default;

    // Optimize path through forest using GCS (or Dijkstra fallback)
    // corridor_hops: how many hops around Dijkstra path to include in GCS
    GCSResult optimize(const SafeBoxForest& forest,
                       const Eigen::VectorXd& start,
                       const Eigen::VectorXd& goal,
                       int corridor_hops = 2);

    // Check if Drake is available at runtime
    static bool drake_available();

private:
#ifdef SBF_WITH_DRAKE
    GCSResult solve_gcs(const SafeBoxForest& forest,
                        const Eigen::VectorXd& start,
                        const Eigen::VectorXd& goal,
                        const std::vector<int>& corridor_box_ids);
#endif

    GCSResult solve_dijkstra_fallback(const SafeBoxForest& forest,
                                      const Eigen::VectorXd& start,
                                      const Eigen::VectorXd& goal);
};

// Free function for checking Drake availability
bool drake_available();

} // namespace sbf

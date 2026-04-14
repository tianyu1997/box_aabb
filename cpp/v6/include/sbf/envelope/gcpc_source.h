#pragma once
/// @file gcpc_source.h
/// @brief GCPC: Global Critical Point Cache — pre-computed interior critical points.
///
/// Single-tier flat storage of critical points discovered offline.
/// At query time, all cached points within the query interval are
/// evaluated via FK to tighten the endpoint AABB beyond what IFK provides.
///
/// Binary `.gcpc` file format for persistent caching.

#include <sbf/core/types.h>
#include <sbf/core/robot.h>
#include <sbf/envelope/endpoint_source.h>

#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

namespace sbf {

// Flat storage of pre-computed interior critical points.
// Single-tier (no smoke/quick/standard/full distinction).
class GcpcCache {
public:
    GcpcCache() = default;

    // Load from binary .gcpc file
    static GcpcCache load(const std::string& path);

    // Save to binary .gcpc file
    void save(const std::string& path) const;

    // Add a critical point (n_joints dimensional config)
    void add_point(const Eigen::VectorXd& q);

    // Query: return all cached points that fall within the given intervals
    std::vector<Eigen::VectorXd> lookup(const std::vector<Interval>& intervals) const;

    int n_points() const { return static_cast<int>(points_.size()); }
    int n_dims() const { return n_dims_; }
    bool empty() const { return points_.empty(); }

    void set_n_dims(int n) { n_dims_ = n; }
    const std::vector<Eigen::VectorXd>& points() const { return points_; }

private:
    int n_dims_ = 0;
    std::vector<Eigen::VectorXd> points_;
};

// Compute endpoint iAABBs via GCPC:
//   1. Analytical boundary (Phase 0-2)
//   2. Cache interior lookup → FK eval → expand AABB
//   3. Result is_safe = true
EndpointIAABBResult compute_endpoint_iaabb_gcpc(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    const GcpcCache& cache);

}  // namespace sbf

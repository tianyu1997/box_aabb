#pragma once
/// @file path_opt_pipeline.h
/// @brief Configurable 5-step path post-processor.
///
/// Each step rewrites the waypoint list. The collision predicate
/// `is_free(q)` is supplied by the caller; in v7 P5 it is a
/// box-corridor membership test (q lies in at least one of the
/// collision-free corridor boxes).

#include <Eigen/Core>
#include <functional>
#include <vector>

namespace sbf::planner {

enum class PathOptStep {
    GREEDY_SHORTCUT,   ///< delete waypoints whose neighbours connect freely
    SHORTCUT,          ///< randomised shortcut
    DENSIFY,           ///< insert points to keep |dq| <= densify_dt
    ELASTIC_BAND,      ///< Laplacian smoothing under free-segment constraint
    SHORTCUT_FINAL,    ///< final randomised shortcut
};

struct PathOptConfig {
    std::vector<PathOptStep> steps = {
        PathOptStep::GREEDY_SHORTCUT,
        PathOptStep::SHORTCUT,
        PathOptStep::DENSIFY,
        PathOptStep::ELASTIC_BAND,
        PathOptStep::SHORTCUT_FINAL,
    };
    int    shortcut_iters = 100;
    double densify_dt     = 0.05;     ///< rad
    double elastic_alpha  = 0.10;
    int    elastic_iters  = 20;
    /// Discretisation step (rad) used by the free-segment predicate.
    double seg_check_dt   = 0.02;
    uint64_t rng_seed     = 7;
};

/// `is_free` returns true iff q lies in the collision-free corridor.
using FreeFn = std::function<bool(const Eigen::VectorXd&)>;

class PathOptPipeline {
public:
    PathOptPipeline(PathOptConfig cfg, FreeFn is_free)
        : cfg_(std::move(cfg)), is_free_(std::move(is_free)) {}

    /// Apply the configured step sequence; returns the optimised path.
    /// The first and last waypoints are always preserved.
    std::vector<Eigen::VectorXd> optimize(
        const std::vector<Eigen::VectorXd>& raw);

    /// Length after each step (including step 0 = input length).
    const std::vector<double>& step_lengths() const { return step_lengths_; }

    /// Returns true iff every consecutive segment of `path` is free
    /// (sampled at `seg_check_dt`).
    bool is_path_free(const std::vector<Eigen::VectorXd>& path) const;

    /// Free-segment check between two waypoints.
    bool is_segment_free(const Eigen::VectorXd& a,
                         const Eigen::VectorXd& b) const;

    static double path_length(const std::vector<Eigen::VectorXd>& path);

private:
    PathOptConfig       cfg_;
    FreeFn              is_free_;
    std::vector<double> step_lengths_;

    std::vector<Eigen::VectorXd> step_greedy_shortcut(
        const std::vector<Eigen::VectorXd>& p);
    std::vector<Eigen::VectorXd> step_shortcut(
        const std::vector<Eigen::VectorXd>& p, int iters);
    std::vector<Eigen::VectorXd> step_densify(
        const std::vector<Eigen::VectorXd>& p);
    std::vector<Eigen::VectorXd> step_elastic_band(
        const std::vector<Eigen::VectorXd>& p);
};

}  // namespace sbf::planner

#pragma once
/// @file sbf_planner.h
/// @brief Top-level synchronous SBF planner — state machine over
///        forest growth + box-graph path search + path optimisation.
///
/// v7 P5 scope: synchronous `plan()` only. Async/cancel and Drake GCS
/// are deferred to a later phase.

#include "sbf/core/robot.h"
#include "sbf/forest/grower.h"
#include "sbf/lect/lect.h"
#include "sbf/planner/path_finder.h"
#include "sbf/planner/path_opt_pipeline.h"

#include <Eigen/Core>
#include <chrono>
#include <string>
#include <vector>

namespace sbf::planner {

enum class PlannerState {
    IDLE,
    GROWING,
    PATH_FINDING,
    OPTIMIZING_PATH,
    SUCCESS,
    FAILED
};

const char* to_string(PlannerState s);

struct PlannerConfig {
    sbf::forest::GrowerConfig grower;
    PathOptConfig             path_opt;
    double                    timeout_s  = 60.0;
    /// quick_mode: when true, override grower.max_boxes/timeout and
    /// trim path_opt to {GREEDY_SHORTCUT, SHORTCUT_FINAL} with 10 iters.
    bool                      quick_mode = false;
    /// Optional e2e fallback: if the certified box graph remains disconnected,
    /// run a point-level RRT-Connect bridge under the same per-configuration
    /// collision model.  This preserves a feasible path result while exposing
    /// that the axis-aligned certified corridor was not connected.
    bool                      point_bridge_fallback = false;
    double                    point_bridge_timeout_ms = 3000.0;
    int                       point_bridge_max_iters = 200000;
    double                    point_bridge_step = 0.30;
    double                    point_bridge_goal_bias = 0.20;
};

struct PlanResult {
    bool                          success            = false;
    PlannerState                  final_state        = PlannerState::IDLE;
    std::vector<Eigen::VectorXd>  path;              ///< optimised waypoints
    std::vector<Eigen::VectorXd>  raw_path;          ///< unoptimised (Dijkstra)
    int                           start_box          = -1;
    int                           goal_box           = -1;
    int                           n_boxes            = 0;
    int                           n_islands          = 0;
    double                        raw_length         = 0.0;
    double                        opt_length         = 0.0;
    double                        grow_time_ms       = 0.0;
    double                        path_find_time_ms  = 0.0;
    double                        opt_time_ms        = 0.0;
    double                        total_time_ms      = 0.0;
    std::vector<double>           step_lengths;       ///< per opt step
    std::string                   fail_reason;
    bool                          used_point_bridge  = false;
};

class SbfPlanner {
public:
    SbfPlanner(const sbf::core::Robot& robot,
               sbf::lect::LECT&        lect,
               PlannerConfig           cfg);

    /// Synchronous plan from q_start → q_goal under static obstacles.
    /// `obs_compact` = packed obstacle AABBs ([n_obs * 6]); may be null.
    PlanResult plan(const Eigen::VectorXd& q_start,
                    const Eigen::VectorXd& q_goal,
                    const float*           obs_compact,
                    int                    n_obs);

    PlannerState state() const { return state_; }

private:
    void apply_quick_mode();

    const sbf::core::Robot& robot_;
    sbf::lect::LECT&        lect_;
    PlannerConfig           cfg_;
    PlannerState            state_ = PlannerState::IDLE;
};

}  // namespace sbf::planner

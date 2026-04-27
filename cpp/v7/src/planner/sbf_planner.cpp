/// @file sbf_planner.cpp
#include "sbf/planner/sbf_planner.h"

#include "sbf/core/fk_state.h"
#include "sbf/forest/adjacency.h"
#include "sbf/scene/collision.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>

namespace sbf::planner {

using Clock = std::chrono::steady_clock;

const char* to_string(PlannerState s) {
    switch (s) {
        case PlannerState::IDLE:            return "IDLE";
        case PlannerState::GROWING:         return "GROWING";
        case PlannerState::PATH_FINDING:    return "PATH_FINDING";
        case PlannerState::OPTIMIZING_PATH: return "OPTIMIZING_PATH";
        case PlannerState::SUCCESS:         return "SUCCESS";
        case PlannerState::FAILED:          return "FAILED";
    }
    return "?";
}

SbfPlanner::SbfPlanner(const sbf::core::Robot& robot,
                       sbf::lect::LECT&        lect,
                       PlannerConfig           cfg)
    : robot_(robot), lect_(lect), cfg_(std::move(cfg)) {}

void SbfPlanner::apply_quick_mode() {
    if (!cfg_.quick_mode) return;
    // Experiment wrappers already pass explicit time and box budgets.
    // Quick mode should only trim the expensive optimiser stages; otherwise
    // reviewer-facing budget sweeps such as max_boxes=10000 get silently
    // truncated to 5000 and Marcucci runs time out at 10 s despite a larger
    // CLI timeout.
    cfg_.path_opt.steps = {
        PathOptStep::GREEDY_SHORTCUT,
        PathOptStep::SHORTCUT_FINAL,
    };
    cfg_.path_opt.shortcut_iters = std::min(cfg_.path_opt.shortcut_iters, 10);
    cfg_.path_opt.elastic_iters  = 0;
}

namespace {

/// Box-corridor membership: q is "free" iff it lies in any of the
/// supplied corridor boxes. Strict adjacency at growth time guarantees
/// this is a contiguous, collision-free region.
struct CorridorChecker {
    const std::vector<sbf::scene::BoxNode>* boxes;
    std::vector<int> corridor;        ///< box ids
    bool operator()(const Eigen::VectorXd& q) const {
        for (int i : corridor)
            if ((*boxes)[i].contains(q)) return true;
        return false;
    }
};

/// Per-configuration checker used by the optional point bridge.  It matches
/// the OMPL baseline's definition: exact FK at q, per-link radius inflation,
/// and AABB-vs-obstacle SAT.
class PointCollisionChecker {
public:
    PointCollisionChecker(const sbf::core::Robot& robot,
                          const float* obs_compact,
                          int n_obs)
        : robot_(robot), obs_(obs_compact), n_obs_(n_obs) {
        n_active_ = robot_.n_active_links();
        intervals_.reserve(robot_.n_joints());
        link_aabb_.assign(static_cast<std::size_t>(n_active_) * 6, 0.0f);
        radii_.assign(n_active_, 0.0f);
        if (const double* rr = robot_.active_link_radii()) {
            for (int i = 0; i < n_active_; ++i) radii_[i] = static_cast<float>(rr[i]);
        }
    }

    bool is_free(const Eigen::VectorXd& q) {
        intervals_.clear();
        for (int i = 0; i < q.size(); ++i) intervals_.emplace_back(q[i], q[i]);
        auto fk = sbf::core::compute_fk_full(robot_, intervals_);
        sbf::core::extract_link_aabbs(
            fk, robot_.active_link_map(), n_active_, link_aabb_.data());
        return !sbf::scene::aabbs_collide_obs_inflated(
            link_aabb_.data(), n_active_, radii_.data(), obs_, n_obs_);
    }

    bool segment_free(const Eigen::VectorXd& a,
                      const Eigen::VectorXd& b,
                      double dt = 0.05) {
        const double L = (b - a).norm();
        const int n = std::max(1, static_cast<int>(std::ceil(L / dt)));
        for (int k = 0; k <= n; ++k) {
            double t = static_cast<double>(k) / n;
            Eigen::VectorXd q = (1.0 - t) * a + t * b;
            if (!is_free(q)) return false;
        }
        return true;
    }

private:
    const sbf::core::Robot& robot_;
    const float* obs_ = nullptr;
    int n_obs_ = 0;
    int n_active_ = 0;
    std::vector<sbf::core::Interval> intervals_;
    std::vector<float> link_aabb_;
    std::vector<float> radii_;
};

struct PointBridgeConfig {
    double timeout_ms = 3000.0;
    int max_iters = 200000;
    double step = 0.30;
    double goal_bias = 0.20;
    uint64_t seed = 7;
};

int nearest_point_node(const std::vector<Eigen::VectorXd>& nodes,
                       const Eigen::VectorXd& q) {
    int best = -1;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
        double d2 = (nodes[i] - q).squaredNorm();
        if (d2 < best_d2) { best_d2 = d2; best = i; }
    }
    return best;
}

bool steer(const Eigen::VectorXd& from,
           const Eigen::VectorXd& to,
           double step,
           Eigen::VectorXd& out) {
    Eigen::VectorXd d = to - from;
    double L = d.norm();
    if (L < 1e-12) return false;
    out = (L <= step) ? to : from + (step / L) * d;
    return true;
}

int extend_point_tree(std::vector<Eigen::VectorXd>& nodes,
                      std::vector<int>& parent,
                      const Eigen::VectorXd& target,
                      const std::vector<sbf::core::Interval>& limits,
                      PointCollisionChecker& checker,
                      double step) {
    int near = nearest_point_node(nodes, target);
    if (near < 0) return -1;
    Eigen::VectorXd q_new;
    if (!steer(nodes[near], target, step, q_new)) return -1;
    for (int d = 0; d < q_new.size(); ++d)
        q_new[d] = std::clamp(q_new[d], limits[d].lo, limits[d].hi);
    if (!checker.segment_free(nodes[near], q_new)) return -1;
    int idx = static_cast<int>(nodes.size());
    nodes.push_back(q_new);
    parent.push_back(near);
    return idx;
}

int connect_point_tree(std::vector<Eigen::VectorXd>& nodes,
                       std::vector<int>& parent,
                       const Eigen::VectorXd& target,
                       const std::vector<sbf::core::Interval>& limits,
                       PointCollisionChecker& checker,
                       double step) {
    int cur = nearest_point_node(nodes, target);
    if (cur < 0) return -1;
    for (int k = 0; k < 80; ++k) {
        if ((nodes[cur] - target).norm() <= step * 0.5) {
            if (checker.segment_free(nodes[cur], target)) {
                int idx = static_cast<int>(nodes.size());
                nodes.push_back(target);
                parent.push_back(cur);
                return idx;
            }
            return -1;
        }
        Eigen::VectorXd q_new;
        if (!steer(nodes[cur], target, step, q_new)) return -1;
        for (int d = 0; d < q_new.size(); ++d)
            q_new[d] = std::clamp(q_new[d], limits[d].lo, limits[d].hi);
        if (!checker.segment_free(nodes[cur], q_new)) return -1;
        int idx = static_cast<int>(nodes.size());
        nodes.push_back(q_new);
        parent.push_back(cur);
        cur = idx;
    }
    return -1;
}

std::vector<Eigen::VectorXd> extract_point_path(
    const std::vector<Eigen::VectorXd>& nodes,
    const std::vector<int>& parent,
    int idx) {
    std::vector<Eigen::VectorXd> p;
    while (idx >= 0) {
        p.push_back(nodes[idx]);
        idx = parent[idx];
    }
    std::reverse(p.begin(), p.end());
    return p;
}

std::vector<Eigen::VectorXd> rrt_connect_point_bridge(
    const Eigen::VectorXd& q_start,
    const Eigen::VectorXd& q_goal,
    const sbf::core::Robot& robot,
    PointCollisionChecker& checker,
    const PointBridgeConfig& cfg) {
    if (!checker.is_free(q_start) || !checker.is_free(q_goal)) return {};
    const int nd = robot.n_joints();
    const auto& limits = robot.joint_limits().limits;
    std::mt19937_64 rng(cfg.seed);
    std::uniform_real_distribution<double> u01(0.0, 1.0);

    std::vector<Eigen::VectorXd> a_nodes{q_start}, b_nodes{q_goal};
    std::vector<int> a_parent{-1}, b_parent{-1};
    a_nodes.reserve(4096); b_nodes.reserve(4096);
    a_parent.reserve(4096); b_parent.reserve(4096);

    auto t0 = Clock::now();
    auto elapsed = [&]() {
        return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    };

    for (int it = 0; it < cfg.max_iters && elapsed() < cfg.timeout_ms; ++it) {
        bool forward = (it % 2 == 0);
        auto& x_nodes = forward ? a_nodes : b_nodes;
        auto& x_parent = forward ? a_parent : b_parent;
        auto& y_nodes = forward ? b_nodes : a_nodes;
        auto& y_parent = forward ? b_parent : a_parent;
        const Eigen::VectorXd& target_root = forward ? q_goal : q_start;

        Eigen::VectorXd q_rand(nd);
        if (u01(rng) < cfg.goal_bias) {
            q_rand = target_root;
        } else {
            for (int d = 0; d < nd; ++d)
                q_rand[d] = limits[d].lo + u01(rng) * limits[d].width();
        }

        int new_idx = extend_point_tree(x_nodes, x_parent, q_rand,
                                        limits, checker, cfg.step);
        if (new_idx < 0) continue;
        int conn_idx = connect_point_tree(y_nodes, y_parent, x_nodes[new_idx],
                                          limits, checker, cfg.step);
        if (conn_idx < 0) continue;

        std::vector<Eigen::VectorXd> pa, pb;
        if (forward) {
            pa = extract_point_path(a_nodes, a_parent, new_idx);
            pb = extract_point_path(b_nodes, b_parent, conn_idx);
        } else {
            pa = extract_point_path(a_nodes, a_parent, conn_idx);
            pb = extract_point_path(b_nodes, b_parent, new_idx);
        }
        std::reverse(pb.begin(), pb.end());
        for (std::size_t i = 1; i < pb.size(); ++i) pa.push_back(std::move(pb[i]));
        return pa;
    }
    return {};
}

}  // namespace

PlanResult SbfPlanner::plan(const Eigen::VectorXd& q_start,
                            const Eigen::VectorXd& q_goal,
                            const float*           obs_compact,
                            int                    n_obs) {
    PlanResult res;
    auto t0 = Clock::now();
    auto ms = [&]() {
        return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    };
    auto fail = [&](const char* why) {
        res.success      = false;
        res.final_state  = PlannerState::FAILED;
        res.fail_reason  = why;
        res.total_time_ms = ms();
        state_ = PlannerState::FAILED;
        return res;
    };

    apply_quick_mode();

    // ── State: GROWING ─────────────────────────────────────────────
    state_ = PlannerState::GROWING;
    sbf::forest::ForestGrower grower(robot_, lect_, cfg_.grower);
    grower.set_endpoints(q_start, q_goal);
    auto tg0 = Clock::now();
    sbf::forest::GrowerResult gr = grower.grow(obs_compact, n_obs);
    res.grow_time_ms = std::chrono::duration<double, std::milli>(
                           Clock::now() - tg0).count();
    res.n_boxes   = static_cast<int>(gr.boxes.size());
    res.n_islands = gr.adjacency_islands;
    res.start_box = gr.start_box;
    res.goal_box  = gr.goal_box;
    if (gr.start_box < 0 || gr.goal_box < 0)
        return fail("INFEASIBLE: q_start or q_goal in collision");
    if (!gr.start_goal_connected) {
        if (!cfg_.point_bridge_fallback)
            return fail("UNCONNECTED: start and goal in different islands");

        state_ = PlannerState::PATH_FINDING;
        auto tp0 = Clock::now();
        PointCollisionChecker checker(robot_, obs_compact, n_obs);
        PointBridgeConfig pcfg;
        pcfg.timeout_ms = cfg_.point_bridge_timeout_ms;
        pcfg.max_iters = cfg_.point_bridge_max_iters;
        pcfg.step = cfg_.point_bridge_step;
        pcfg.goal_bias = cfg_.point_bridge_goal_bias;
        pcfg.seed = cfg_.grower.rng_seed + 99173;
        auto raw = rrt_connect_point_bridge(q_start, q_goal, robot_, checker, pcfg);
        res.path_find_time_ms = std::chrono::duration<double, std::milli>(
                                    Clock::now() - tp0).count();
        if (raw.empty())
            return fail("UNCONNECTED: box graph disconnected and point bridge failed");
        res.used_point_bridge = true;
        res.raw_path = raw;
        res.raw_length = PathOptPipeline::path_length(raw);

        state_ = PlannerState::OPTIMIZING_PATH;
        auto to0 = Clock::now();
        PointCollisionChecker opt_checker(robot_, obs_compact, n_obs);
        PathOptConfig point_opt_cfg = cfg_.path_opt;
        point_opt_cfg.seg_check_dt = std::max(point_opt_cfg.seg_check_dt, 0.05);
        PathOptPipeline pipe(point_opt_cfg, FreeFn([&](const Eigen::VectorXd& q) {
            return opt_checker.is_free(q);
        }));
        res.path = pipe.optimize(res.raw_path);
        if (!pipe.is_path_free(res.path)) {
            res.path = res.raw_path;
            res.step_lengths = {res.raw_length};
        } else {
            res.step_lengths = pipe.step_lengths();
        }
        res.opt_length = PathOptPipeline::path_length(res.path);
        res.opt_time_ms = std::chrono::duration<double, std::milli>(
                              Clock::now() - to0).count();
        state_ = PlannerState::SUCCESS;
        res.success = true;
        res.final_state = PlannerState::SUCCESS;
        res.total_time_ms = ms();
        return res;
    }
    if (ms() / 1000.0 > cfg_.timeout_s)
        return fail("TIMEOUT during GROWING");

    // ── State: PATH_FINDING ────────────────────────────────────────
    state_ = PlannerState::PATH_FINDING;
    auto tp0 = Clock::now();
    PathFinderResult pf = find_box_path(
        gr.boxes, gr.adjacency, gr.start_box, gr.goal_box, q_start, q_goal);
    res.path_find_time_ms = std::chrono::duration<double, std::milli>(
                                Clock::now() - tp0).count();
    if (!pf.success)
        return fail("PATH_FIND failed despite connectivity flag");
    res.raw_path   = pf.waypoints;
    res.raw_length = pf.length;

    // ── State: OPTIMIZING_PATH ─────────────────────────────────────
    state_ = PlannerState::OPTIMIZING_PATH;
    auto to0 = Clock::now();
    CorridorChecker checker{&gr.boxes, pf.box_path};
    PathOptPipeline pipe(cfg_.path_opt, FreeFn(checker));
    res.path         = pipe.optimize(pf.waypoints);
    res.opt_length   = PathOptPipeline::path_length(res.path);
    res.step_lengths = pipe.step_lengths();
    res.opt_time_ms  = std::chrono::duration<double, std::milli>(
                           Clock::now() - to0).count();

    state_ = PlannerState::SUCCESS;
    res.success     = true;
    res.final_state = PlannerState::SUCCESS;
    res.total_time_ms = ms();
    return res;
}

}  // namespace sbf::planner

// SafeBoxForest v6 — SBF Planner (Phase H5)
#include <sbf/planner/sbf_planner.h>
#include <sbf/planner/path_extract.h>
#include <sbf/planner/path_smoother.h>
#include <sbf/forest/connectivity.h>
#include <sbf/core/ray_aabb.h>
#include <sbf/ffb/ffb.h>
#include <sbf/lect/lect_io.h>

#include <chrono>
#include <cinttypes>
#include <atomic>
#include <memory>
#include <filesystem>
#include <future>
#include <limits>
#include <random>
#include <unordered_set>
#include <sbf/core/log.h>

namespace sbf {

// ── Auto-derive cache path from robot fingerprint ──────────────────────────
std::string SBFPlanner::lect_auto_cache_path() const {
    uint64_t fp = robot_.fingerprint();
    char hex[17];
    snprintf(hex, sizeof(hex), "%016" PRIx64, fp);
    return config_.lect_cache_dir + "/" + robot_.name() + "_" + hex + ".lect";
}

SBFPlanner::SBFPlanner(const Robot& robot, const SBFPlannerConfig& config)
    : robot_(robot), config_(config) {}


PlanResult SBFPlanner::plan(
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& goal,
    const Obstacle* obs, int n_obs,
    double timeout_ms)
{
    PlanResult result;
    auto t0 = std::chrono::steady_clock::now();

    // 1-5. Build forest
    build(start, goal, obs, n_obs, timeout_ms);

    // Helper: set common fields on all return paths
    auto fill_result = [&]() {
        auto t1 = std::chrono::steady_clock::now();
        result.n_boxes = static_cast<int>(boxes_.size());
        result.planning_time_ms =
            std::chrono::duration<double, std::milli>(t1 - t0).count();
        result.build_time_ms = last_build_time_ms_;
        result.lect_time_ms = last_lect_time_ms_;
    };

    if (boxes_.empty()) { fill_result(); return result; }

    // Find containing boxes, with nearest-box fallback
    auto find_containing_or_nearest = [](const std::vector<BoxNode>& bxs,
                                         const Eigen::VectorXd& q) -> int {
        int best_id = -1;
        double best_dist = std::numeric_limits<double>::max();
        for (const auto& b : bxs) {
            if (b.contains(q)) return b.id;
            double d = (b.center() - q).squaredNorm();
            if (d < best_dist) { best_dist = d; best_id = b.id; }
        }
        return best_id;  // nearest box if none contains q
    };

    int start_id = find_containing_or_nearest(boxes_, start);
    int goal_id  = find_containing_or_nearest(boxes_, goal);

    if (start_id < 0 || goal_id < 0) {
        // Try raw_boxes_ as fallback
        for (const auto& rb : raw_boxes_) {
            if (start_id < 0 && rb.contains(start)) {
                boxes_.push_back(rb);
                start_id = rb.id;
            }
            if (goal_id < 0 && rb.contains(goal)) {
                boxes_.push_back(rb);
                goal_id = rb.id;
            }
            if (start_id >= 0 && goal_id >= 0) break;
        }
        if (start_id >= 0 && goal_id >= 0)
            adj_ = compute_adjacency(boxes_);
    }

    if (start_id < 0 || goal_id < 0) { fill_result(); return result; }

    SBF_INFO("[PLN] plan: start_id=%d goal_id=%d n_boxes=%d", start_id, goal_id, (int)boxes_.size());

    // Check adjacency for start/goal
    {
        auto it_s = adj_.find(start_id);
        auto it_g = adj_.find(goal_id);
        SBF_INFO("[PLN] adj: start_in=%d(%d) goal_in=%d(%d) total_entries=%d", it_s != adj_.end(), it_s != adj_.end() ? (int)it_s->second.size() : -1, it_g != adj_.end(), it_g != adj_.end() ? (int)it_g->second.size() : -1, (int)adj_.size());
    }

    // 6. Search
    std::vector<Eigen::VectorXd> path;
    std::vector<int> box_seq;

    if (config_.use_gcs) {
#ifdef SBF_HAS_DRAKE
        CollisionChecker gcs_checker(robot_, {});
        gcs_checker.set_obstacles(obs, n_obs);
        auto gcs_res = gcs_plan(adj_, boxes_, start, goal, config_.gcs, &gcs_checker, lect_.get());
#else
        auto gcs_res = gcs_plan_fallback(adj_, boxes_, start, goal);
#endif
        if (!gcs_res.found) { fill_result(); return result; }
        path = std::move(gcs_res.path);
    } else {
        auto dij = dijkstra_search(adj_, boxes_, start_id, goal_id, goal);
        if (!dij.found) { fill_result(); return result; }
        box_seq = shortcut_box_sequence(dij.box_sequence, adj_);
        path = extract_waypoints(box_seq, boxes_, start, goal);
    }

    // Build box sequence for smoother
    std::unordered_map<int, const BoxNode*> box_map;
    for (const auto& b : boxes_) box_map[b.id] = &b;

    std::vector<BoxNode> seq_boxes;
    for (int id : box_seq) {
        auto it = box_map.find(id);
        if (it != box_map.end()) seq_boxes.push_back(*it->second);
    }

    // 7. Smooth
    CollisionChecker checker(robot_, {});
    checker.set_obstacles(obs, n_obs);
    path = smooth_path(path, seq_boxes, checker, config_.smoother);

    // 8. Result
    auto t1 = std::chrono::steady_clock::now();
    result.success = true;
    result.path = std::move(path);
    result.box_sequence = std::move(box_seq);
    result.path_length = sbf::path_length(result.path);
    result.planning_time_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.n_boxes = static_cast<int>(boxes_.size());
    result.build_time_ms = last_build_time_ms_;
    result.lect_time_ms = last_lect_time_ms_;
    return result;
}

void SBFPlanner::clear_forest() {
    boxes_.clear();
    adj_.clear();
    lect_.reset();
    built_ = false;
}

}  // namespace sbf

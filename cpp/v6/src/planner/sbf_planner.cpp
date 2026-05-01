// SafeBoxForest v6 — SBF Planner (Phase H5)
#include <sbf/planner/sbf_planner.h>
#include <sbf/planner/path_extract.h>
#include <sbf/planner/path_smoother.h>
#include <sbf/forest/connectivity.h>
#include <sbf/core/ray_aabb.h>
#include <sbf/ffb/ffb.h>
#include <sbf/lect/lect_io.h>

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <memory>
#include <limits>
#include <stdexcept>
#include <sbf/core/log.h>

namespace sbf {

namespace {

float resolve_default_ffb_grid_margin(const EnvelopeTypeConfig& env_cfg,
                                      float requested_margin) {
    if (requested_margin >= 0.0f) return requested_margin;
    if (env_cfg.type == EnvelopeType::LinkIAABB) return 0.0f;
    const float voxel_delta = static_cast<float>(env_cfg.grid_config.voxel_delta);
    if (voxel_delta > 0.0f) return 0.5f * voxel_delta;
    return 0.02f;
}

}  // namespace

// ── Auto-derive cache path from robot fingerprint ──────────────────────────
std::string SBFPlanner::lect_auto_cache_path() const {
    uint64_t fp = robot_.fingerprint();
    char hex[17];
    snprintf(hex, sizeof(hex), "%016" PRIx64, fp);
    return config_.lect_cache_dir + "/" + robot_.name() + "_" + hex + ".lect";
}

SBFPlanner::SBFPlanner(const Robot& robot, const SBFPlannerConfig& config)
    : robot_(robot), config_(config) {
    config_.grower.ffb_config.grid_margin_threshold =
        resolve_default_ffb_grid_margin(
            config_.envelope_type,
            config_.grower.ffb_config.grid_margin_threshold);
}

RebuildResult SBFPlanner::add_obstacle_and_rebuild(const Obstacle& obstacle) {
    if (!built_ || !lect_) {
        throw std::runtime_error("SBFPlanner::add_obstacle_and_rebuild requires a built forest");
    }

    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();

    RebuildResult result;
    result.boxes_before = static_cast<int>(boxes_.size());
    result.raw_boxes_before = static_cast<int>(raw_boxes_.size());

    auto invalidates = [&](const BoxNode& box) {
        return lect_->intervals_collide_scene(box.joint_intervals, &obstacle, 1);
    };

    auto unmark_if_owned = [&](const BoxNode& box) {
        if (box.tree_id < 0 || box.tree_id >= lect_->n_nodes()) return;
        if (lect_->forest_id(box.tree_id) == box.id) {
            lect_->unmark_occupied(box.tree_id);
        }
    };

    const auto t_check0 = Clock::now();

    std::vector<BoxNode> kept_boxes;
    kept_boxes.reserve(boxes_.size());
    for (const auto& box : boxes_) {
        if (invalidates(box)) {
            ++result.boxes_removed;
            unmark_if_owned(box);
        } else {
            kept_boxes.push_back(box);
        }
    }

    std::vector<BoxNode> kept_raw_boxes;
    kept_raw_boxes.reserve(raw_boxes_.size());
    for (const auto& box : raw_boxes_) {
        if (invalidates(box)) {
            ++result.raw_boxes_removed;
            unmark_if_owned(box);
        } else {
            kept_raw_boxes.push_back(box);
        }
    }

    const auto t_check1 = Clock::now();
    result.collision_check_ms =
        std::chrono::duration<double, std::milli>(t_check1 - t_check0).count();

    boxes_ = std::move(kept_boxes);
    raw_boxes_ = std::move(kept_raw_boxes);
    stored_obs_.push_back(obstacle);

    const auto t_adj0 = Clock::now();
    adj_ = compute_adjacency(boxes_, config_.adjacency_tol, 0,
                             config_.adjacency_gap_tol);
    const auto islands = find_islands(adj_);
    const auto t_adj1 = Clock::now();

    int directed_edges = 0;
    for (const auto& kv : adj_) {
        directed_edges += static_cast<int>(kv.second.size());
    }

    result.boxes_after = static_cast<int>(boxes_.size());
    result.raw_boxes_after = static_cast<int>(raw_boxes_.size());
    result.adjacency_edges_after = directed_edges / 2;
    result.islands_after = static_cast<int>(islands.size());
    result.adjacency_ms =
        std::chrono::duration<double, std::milli>(t_adj1 - t_adj0).count();
    result.total_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

    SBF_INFO("[PLN] rebuild: removed=%d/%d raw_removed=%d/%d boxes=%d islands=%d adj_edges=%d total=%.2fms",
             result.boxes_removed, result.boxes_before,
             result.raw_boxes_removed, result.raw_boxes_before,
             result.boxes_after, result.islands_after,
             result.adjacency_edges_after, result.total_ms);

    return result;
}

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

int SBFPlanner::pre_bridge_pairs(
    const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& pairs,
    const Obstacle* obs, int n_obs,
    double per_pair_timeout_ms,
    int max_pairs_per_call)
{
    if (!built_ || boxes_.empty() || pairs.empty() || !lect_) return 0;
    if (!obs && !stored_obs_.empty()) {
        obs = stored_obs_.data();
        n_obs = static_cast<int>(stored_obs_.size());
    }

    CollisionChecker checker(robot_, {});
    if (obs && n_obs > 0) checker.set_obstacles(obs, n_obs);

    // Use a relaxed gap tolerance when re-computing adjacency in this
    // function, matching chain_pave's max_safe_gap.  This keeps the
    // chain_pave-created boxes connected in adj_ even when consecutive
    // FFB boxes have small (≤ 0.05 rad) C-space gaps that fall below
    // chain_pave's check_box-validated extension threshold.
    const double pb_gap_tol = std::max(config_.adjacency_gap_tol, 0.05);
    auto recompute_adj = [&]() {
        adj_ = compute_adjacency(boxes_, config_.adjacency_tol, 0, pb_gap_tol);
    };

    auto find_nearest = [&](const Eigen::VectorXd& q) -> int {
        int best = -1;
        double bd = std::numeric_limits<double>::max();
        for (const auto& b : boxes_) {
            if (b.contains(q)) return b.id;
            double d = (b.center() - q).squaredNorm();
            if (d < bd) { bd = d; best = b.id; }
        }
        return best;
    };

    int next_id = 0;
    for (const auto& b : boxes_) next_id = std::max(next_id, b.id + 1);

    auto ffb_cfg = config_.grower.ffb_config;

    // Ensure point q is inside SOME box: if not, FFB at q to create one,
    // then chain-pave from anchor (existing nearest box) toward q so the
    // new box is geometrically connected to the forest.
    auto ensure_containing_box = [&](const Eigen::VectorXd& q,
                                     int anchor_id) -> int {
        for (const auto& b : boxes_)
            if (b.contains(q)) return b.id;

        // Skip FFB when the seed configuration is itself in collision.
        if (checker.check_config(q))
            return anchor_id;

        FFBResult ffb = find_free_box(*lect_, q, obs, n_obs, ffb_cfg);
        if (!ffb.success() || lect_->is_occupied(ffb.node_idx))
            return anchor_id;

        BoxNode nb;
        nb.id = next_id++;
        nb.joint_intervals = lect_->node_intervals(ffb.node_idx);
        nb.seed_config = q;
        nb.tree_id = ffb.node_idx;
        nb.parent_box_id = anchor_id;
        nb.compute_volume();
        // Inherit root_id from anchor
        for (const auto& b : boxes_)
            if (b.id == anchor_id) { nb.root_id = b.root_id; break; }
        int new_id = nb.id;
        boxes_.push_back(std::move(nb));

        // Chain-pave from anchor toward q to glue the new box geometrically
        // to the forest.  Use a 2-waypoint path so chain_pave snaps face
        // boxes between them.
        std::vector<Eigen::VectorXd> link;
        for (const auto& b : boxes_)
            if (b.id == anchor_id) { link.push_back(b.center()); break; }
        link.push_back(q);
        chain_pave_along_path(link, anchor_id, boxes_, *lect_, obs, n_obs,
                              ffb_cfg, adj_, next_id, robot_,
                              /*max_chain=*/30, /*max_steps_per_wp=*/15,
                              /*checker=*/&checker, /*max_safe_gap=*/0.5);
        return new_id;
    };

    int total_added = 0;
    auto t0 = std::chrono::steady_clock::now();

    for (size_t pi = 0; pi < pairs.size(); ++pi) {
        const auto& [s, g] = pairs[pi];
        int s_id = find_nearest(s);
        int g_id = find_nearest(g);
        if (s_id < 0 || g_id < 0) continue;

        // Ensure both endpoints are actually inside a box that is part of
        // the forest graph.  This is the prerequisite for bridge_s_t to
        // produce a chain that compute_adjacency() will preserve.
        s_id = ensure_containing_box(s, s_id);
        g_id = ensure_containing_box(g, g_id);
        repair_bridge_adjacency(boxes_, adj_);
        recompute_adj();

        // Skip if already same island
        auto islands = find_islands(adj_);
        bool same = false;
        for (const auto& isl : islands) {
            bool hs=false, hg=false;
            for (int id : isl) { if (id==s_id) hs=true; if (id==g_id) hg=true; }
            if (hs && hg) { same = true; break; }
        }
        if (!same) {
            int created = bridge_s_t(
                s_id, g_id, boxes_, *lect_, obs, n_obs,
                adj_, ffb_cfg, next_id, robot_, checker,
                per_pair_timeout_ms, max_pairs_per_call,
                std::chrono::steady_clock::time_point::max());
            if (created > 0) {
                repair_bridge_adjacency(boxes_, adj_);
                recompute_adj();
                total_added += created;
                SBF_INFO("[PRE-BR] pair %zu (s=%d g=%d): bridge added %d boxes",
                         pi, s_id, g_id, created);
            }

            // Verify connection; if still not in same island, run direct
            // RRT-Connect(s,g) and chain-pave the resulting trajectory.
            // This mirrors the query-time proxy logic but happens at build
            // time so query() can skip it entirely.
            islands = find_islands(adj_);
            same = false;
            for (const auto& isl : islands) {
                bool hs=false, hg=false;
                for (int id : isl) { if (id==s_id) hs=true; if (id==g_id) hg=true; }
                if (hs && hg) { same = true; break; }
            }
            const bool enable_prebridge_direct_rrt_fallback = false;
            if (!same && enable_prebridge_direct_rrt_fallback) {
                RRTConnectConfig rrt_cfg;
                // Build-time prebridge must stay lightweight; keep fallback
                // direct RRT bounded to roughly the same per-pair budget.
                rrt_cfg.timeout_ms = per_pair_timeout_ms;
                rrt_cfg.max_iters = 6000;
                rrt_cfg.segment_resolution = 20;
                auto rrt = rrt_connect(s, g, checker, robot_, rrt_cfg);
                if (!rrt.empty()) {
                    int added = chain_pave_along_path(
                        rrt, s_id, boxes_, *lect_, obs, n_obs,
                        ffb_cfg, adj_, next_id, robot_,
                        static_cast<int>(rrt.size()) + 8, 20,
                        /*checker=*/&checker, /*max_safe_gap=*/0.5);
                    if (added > 0) {
                        repair_bridge_adjacency(boxes_, adj_);
                        recompute_adj();
                        total_added += added;
                        SBF_INFO("[PRE-BR] pair %zu: direct RRT chain-pave "
                                 "added %d boxes (rrt %d wp)",
                                 pi, added, (int)rrt.size());
                    }
                }
            }
        }

        // Now ensure start/goal points themselves sit inside a box: if
        // the nearest box does NOT contain them, run an RRT-Connect proxy
        // from the point to the nearest box center, then chain-pave it.
        auto box_by_id = [&](int id) -> const BoxNode* {
            for (const auto& b : boxes_)
                if (b.id == id) return &b;
            return nullptr;
        };
        for (int side = 0; side < 2; ++side) {
            const Eigen::VectorXd& q = (side == 0 ? s : g);
            int qid = (side == 0 ? s_id : g_id);
            const BoxNode* bx = box_by_id(qid);
            if (!bx || bx->contains(q)) continue;

            RRTConnectConfig rrt_cfg;
            rrt_cfg.timeout_ms = per_pair_timeout_ms;
            rrt_cfg.max_iters = 10000;
            rrt_cfg.segment_resolution = 20;
            auto rrt = rrt_connect(q, bx->center(), checker, robot_, rrt_cfg, qid);
            if (rrt.empty()) continue;

            // chain_pave from box anchor backward toward q
            std::vector<Eigen::VectorXd> rev(rrt.rbegin(), rrt.rend());
            int added = chain_pave_along_path(
                rev, qid, boxes_, *lect_, obs, n_obs,
                ffb_cfg, adj_, next_id, robot_,
                static_cast<int>(rev.size()) + 4, 15,
                /*checker=*/&checker, /*max_safe_gap=*/0.5);
            if (added > 0) {
                repair_bridge_adjacency(boxes_, adj_);
                recompute_adj();
                total_added += added;
                SBF_INFO("[PRE-BR] pair %zu side=%d: proxy-pave added %d",
                         pi, side, added);
            }
        }
    }

    // Final connectivity audit + soft-edge fallback.  For each input pair
    // still graph-disconnected after all bridge attempts, run one more
    // RRT-Connect, FFB a containing box at every waypoint, and add direct
    // adj edges between consecutive containing boxes — validated by
    // check_segment.  These soft edges do not guarantee box overlap, so
    // downstream GCS treats them as bridge_pairs (Point-vertex chains),
    // but they let Dijkstra return a multi-box corridor for the query.
    // We do NOT call recompute_adj after this so soft edges persist.
    auto find_containing_id = [&](const Eigen::VectorXd& q) -> int {
        for (const auto& b : boxes_) if (b.contains(q)) return b.id;
        return -1;
    };
    auto ffb_at = [&](const Eigen::VectorXd& q, int parent_id) -> int {
        int existing = find_containing_id(q);
        if (existing >= 0) return existing;
        FFBResult ffb = find_free_box(*lect_, q, obs, n_obs, ffb_cfg);
        if (!ffb.success() || lect_->is_occupied(ffb.node_idx)) return -1;
        BoxNode nb;
        nb.id = next_id++;
        nb.joint_intervals = lect_->node_intervals(ffb.node_idx);
        nb.seed_config = q;
        nb.tree_id = ffb.node_idx;
        nb.parent_box_id = parent_id;
        nb.compute_volume();
        for (const auto& b : boxes_)
            if (b.id == parent_id) { nb.root_id = b.root_id; break; }
        int new_id = nb.id;
        lect_->mark_occupied(ffb.node_idx, new_id);
        boxes_.push_back(std::move(nb));
        return new_id;
    };
    auto add_edge = [&](int a, int b) {
        if (a < 0 || b < 0 || a == b) return;
        auto& na = adj_[a];
        if (std::find(na.begin(), na.end(), b) == na.end()) na.push_back(b);
        auto& nb = adj_[b];
        if (std::find(nb.begin(), nb.end(), a) == nb.end()) nb.push_back(a);
    };
    const bool enable_soft_repair = false;
    if (enable_soft_repair) {
        auto islands_final = find_islands(adj_);
        std::unordered_map<int, int> id_to_isl;
        for (size_t ii = 0; ii < islands_final.size(); ++ii)
            for (int id : islands_final[ii]) id_to_isl[id] = static_cast<int>(ii);

        int n_connected = 0, n_disconnected = 0, n_soft_repaired = 0;
        int n_soft_edges = 0;
        for (size_t pi = 0; pi < pairs.size(); ++pi) {
            const auto& [s, g] = pairs[pi];
            int s_id = find_containing_id(s);
            int g_id = find_containing_id(g);
            auto its = id_to_isl.find(s_id);
            auto itg = id_to_isl.find(g_id);
            bool ok = (its != id_to_isl.end() && itg != id_to_isl.end()
                       && its->second == itg->second);
            if (ok) { ++n_connected; continue; }

            // Disconnected — attempt soft-edge repair.
            RRTConnectConfig rrt_cfg;
            rrt_cfg.timeout_ms = per_pair_timeout_ms * 2;
            rrt_cfg.max_iters = 20000;
            rrt_cfg.segment_resolution = 20;
            auto rrt = rrt_connect(s, g, checker, robot_, rrt_cfg);
            if (rrt.empty()) {
                ++n_disconnected;
                SBF_INFO("[PRE-BR] soft-repair pair %zu: RRT failed", pi);
                continue;
            }
            int prev_id = s_id;
            int edges_added_this_pair = 0;
            for (size_t wi = 0; wi < rrt.size(); ++wi) {
                int bid = ffb_at(rrt[wi], prev_id);
                if (bid < 0) continue;
                if (bid != prev_id && prev_id >= 0) {
                    // Adjacent waypoints in the RRT path are collision-free
                    // by construction; the connecting segment is guaranteed
                    // safe up to RRT's segment_resolution.  Add the edge.
                    add_edge(prev_id, bid);
                    ++edges_added_this_pair;
                }
                prev_id = bid;
            }
            // Connect the final waypoint's box to g_id explicitly.
            if (prev_id >= 0 && g_id >= 0 && prev_id != g_id) {
                add_edge(prev_id, g_id);
                ++edges_added_this_pair;
            }
            n_soft_edges += edges_added_this_pair;
            if (edges_added_this_pair > 0) ++n_soft_repaired;
            else ++n_disconnected;
            SBF_INFO("[PRE-BR] soft-repair pair %zu: +%d soft edges along "
                     "%d-wp RRT path",
                     pi, edges_added_this_pair, (int)rrt.size());
        }
        SBF_INFO("[PRE-BR] audit: connected=%d soft_repaired=%d "
                 "still_disconnected=%d islands=%zu soft_edges=%d",
                 n_connected, n_soft_repaired, n_disconnected,
                 islands_final.size(), n_soft_edges);
    }

    double elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
    SBF_INFO("[PRE-BR] total %d boxes added across %zu pairs in %.0fms",
             total_added, pairs.size(), elapsed);
    return total_added;
}

}  // namespace sbf

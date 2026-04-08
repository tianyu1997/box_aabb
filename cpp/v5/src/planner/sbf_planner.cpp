// SafeBoxForest v5 — SBF Planner (Phase H5)
#include <sbf/planner/sbf_planner.h>
#include <sbf/planner/path_extract.h>
#include <sbf/forest/connectivity.h>
#include <sbf/ffb/ffb.h>
#include <sbf/lect/lect_io.h>

#include <chrono>
#include <cinttypes>
#include <filesystem>

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

void SBFPlanner::build(const Eigen::VectorXd& start,
                        const Eigen::VectorXd& goal,
                        const Obstacle* obs, int n_obs,
                        double timeout_ms)
{
    auto t_build = std::chrono::steady_clock::now();

    // Overall build deadline (2× grow timeout to allow post-grow phases)
    using Clock = std::chrono::steady_clock;
    const double build_budget_ms = timeout_ms > 0.0 ? timeout_ms * 2.0 : 0.0;
    const bool has_build_deadline = build_budget_ms > 0.0;
    const auto build_deadline = t_build + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double, std::milli>(build_budget_ms));
    auto build_deadline_reached = [&]() {
        return has_build_deadline && Clock::now() >= build_deadline;
    };

    // 1. Construct LECT (using config pipeline settings)
    auto root_ivs = robot_.joint_limits().limits;
    auto t_lect = std::chrono::steady_clock::now();
    lect_ = std::make_unique<LECT>(robot_, root_ivs,
                                    config_.endpoint_source,
                                    config_.envelope_type);
    lect_->set_split_order(config_.split_order);
    if (!config_.z4_enabled)
        lect_->disable_z4();

    // Try loading cached LECT
    std::string cache_path;
    int loaded_n_nodes = 0;
    if (!config_.lect_no_cache) {
        cache_path = lect_auto_cache_path();
        if (lect_load_binary(*lect_, robot_, cache_path)) {
            loaded_n_nodes = lect_->n_nodes();
            lect_->clear_forest_state();
            lect_->set_ep_config(config_.endpoint_source);
            fprintf(stderr, "[PLN] lect: loaded %d cached nodes from %s\n",
                    loaded_n_nodes, cache_path.c_str());
        }
    }

    last_lect_time_ms_ = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_lect).count();
    fprintf(stderr, "[PLN] lect=%.0fms (nodes=%d)\n",
            last_lect_time_ms_, lect_->n_nodes());
    fflush(stderr);

    // 2. Grow forest
    config_.grower.timeout_ms = timeout_ms;
    ForestGrower grower(robot_, *lect_, config_.grower);
    grower.set_endpoints(start, goal);
    auto t_grow = std::chrono::steady_clock::now();
    auto gr = grower.grow(obs, n_obs);
    double grow_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_grow).count();
    fprintf(stderr, "[PLN] grow=%.0fms (boxes=%d)\n",
            grow_ms, (int)gr.boxes.size());
    fflush(stderr);
    boxes_ = std::move(gr.boxes);
    raw_boxes_ = boxes_;  // snapshot before coarsening
    int n0 = (int)boxes_.size();

    // Incremental save cached LECT
    if (!config_.lect_no_cache && !cache_path.empty()) {
        std::filesystem::create_directories(config_.lect_cache_dir);
        lect_save_incremental(*lect_, cache_path, loaded_n_nodes);
        fprintf(stderr, "[PLN] lect: saved %d nodes to %s\n",
                lect_->n_nodes(), cache_path.c_str());
        fflush(stderr);
    }

    // 2b. S→G interpolation connector (like V4)
    //     Create boxes along start→goal line with deep FFB.
    //     Skip if start and goal are already in the same island.
    {
        auto t_sg = std::chrono::steady_clock::now();
        // Check if S→G already connected
        auto pre_adj = compute_adjacency(boxes_);
        int start_box = -1, goal_box = -1;
        for (const auto& b : boxes_) {
            if (b.contains(start)) start_box = b.id;
            if (b.contains(goal)) goal_box = b.id;
        }
        bool already_connected = false;
        if (start_box >= 0 && goal_box >= 0) {
            auto pre_islands = find_islands(pre_adj);
            for (const auto& island : pre_islands) {
                bool has_s = false, has_g = false;
                for (int id : island) {
                    if (id == start_box) has_s = true;
                    if (id == goal_box) has_g = true;
                }
                if (has_s && has_g) { already_connected = true; break; }
            }
        }

        int connect_count = 0;
        if (!already_connected) {
            int next_id = 0;
            for (const auto& b : boxes_) next_id = std::max(next_id, b.id + 1);

            FFBConfig deep_ffb;
            deep_ffb.min_edge = 1e-4;
            deep_ffb.max_depth = config_.grower.ffb_config.max_depth;

            auto limits = robot_.joint_limits().limits;
            const int nd = (int)limits.size();

            constexpr int N_INTERP = 100;

            for (int k = 1; k <= N_INTERP; ++k) {
                if (build_deadline_reached()) break;
                double t = static_cast<double>(k) / (N_INTERP + 1);
                Eigen::VectorXd seed = (1.0 - t) * start + t * goal;
                for (int d = 0; d < nd; ++d)
                    seed[d] = std::clamp(seed[d], limits[d].lo, limits[d].hi);

                bool inside = false;
                for (const auto& b : boxes_) {
                    if (b.contains(seed)) { inside = true; break; }
                }
                if (inside) continue;

                FFBResult ffb = find_free_box(*lect_, seed, obs, n_obs, deep_ffb);
                if (!ffb.success() || lect_->is_occupied(ffb.node_idx))
                    continue;

                BoxNode new_box;
                new_box.id = next_id++;
                new_box.joint_intervals = lect_->node_intervals(ffb.node_idx);
                new_box.seed_config = seed;
                new_box.tree_id = ffb.node_idx;
                new_box.parent_box_id = -1;
                new_box.root_id = -1;
                new_box.compute_volume();

                lect_->mark_occupied(ffb.node_idx, new_box.id);
                boxes_.push_back(std::move(new_box));
                connect_count++;
            }
        }
        double sg_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_sg).count();
        fprintf(stderr, "[PLN] sg_connect=%.0fms added=%d total=%d\n",
                sg_ms, connect_count, (int)boxes_.size());
        fflush(stderr);
        n0 = (int)boxes_.size();
    }

    // Pre-coarsen connectivity check
    {
        auto adj_raw = compute_adjacency(boxes_);
        auto islands_raw = find_islands(adj_raw);
        int edges_raw = 0;
        for (auto& kv : adj_raw) edges_raw += (int)kv.second.size();
        // Count overlapping pairs (not touching, but volumetrically overlapping)
        int n_overlap = 0;
        for (int i = 0; i < n0; ++i) {
            for (int j = i+1; j < n0; ++j) {
                bool sep = false;
                bool all_overlap = true;
                for (int d = 0; d < boxes_[i].n_dims(); ++d) {
                    double lo_i = boxes_[i].joint_intervals[d].lo;
                    double hi_i = boxes_[i].joint_intervals[d].hi;
                    double lo_j = boxes_[j].joint_intervals[d].lo;
                    double hi_j = boxes_[j].joint_intervals[d].hi;
                    if (hi_i < lo_j + 1e-10 || hi_j < lo_i + 1e-10) { sep = true; break; }
                    double ov = std::min(hi_i, hi_j) - std::max(lo_i, lo_j);
                    if (ov < 1e-10) all_overlap = false;
                }
                if (!sep && !adj_raw[boxes_[i].id].empty()) {
                    // check if already adjacent
                    bool already = false;
                    for (int nb : adj_raw[boxes_[i].id])
                        if (nb == boxes_[j].id) { already = true; break; }
                    if (!already) n_overlap++;
                }
            }
        }
        fprintf(stderr, "[PLN] raw: boxes=%d islands=%d edges=%d overlap_nonadj=%d\n",
                n0, (int)islands_raw.size(), edges_raw/2, n_overlap);
        fflush(stderr);
    }

    // 3. Coarsen (sweep)
    auto t_sweep = std::chrono::steady_clock::now();
    CollisionChecker checker(robot_, {});
    checker.set_obstacles(obs, n_obs);
    if (!build_deadline_reached())
        coarsen_forest(boxes_, checker, 10);
    int n1 = (int)boxes_.size();
    double sweep_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_sweep).count();
    fprintf(stderr, "[PLN] sweep=%.0fms (%d->%d)\n", sweep_ms, n0, n1);
    fflush(stderr);

    // 4. Coarsen (greedy)
    auto t_greedy = std::chrono::steady_clock::now();
    if (!build_deadline_reached())
        coarsen_greedy(boxes_, checker, config_.coarsen, lect_.get());
    int n2 = (int)boxes_.size();
    double greedy_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_greedy).count();
    fprintf(stderr, "[PLN] greedy=%.0fms (%d->%d)\n", greedy_ms, n1, n2);
    fflush(stderr);

    // 4b. Remove coarsen overlaps (containment + partial trim)
    filter_coarsen_overlaps(boxes_);
    int n3 = (int)boxes_.size();
    fprintf(stderr, "[PLN] grow=%d sweep=%d greedy=%d filter=%d\n", n0, n1, n2, n3);
    fflush(stderr);

    // 5. Build adjacency
    adj_ = compute_adjacency(boxes_);

    // 6. Bridge disconnected islands (repeat until connected or no progress)
    {
        auto islands0 = find_islands(adj_);
        int edges0 = 0;
        for (auto& kv : adj_) edges0 += (int)kv.second.size();
        fprintf(stderr, "[PLN] pre-bridge: boxes=%d islands=%d edges=%d\n",
                (int)boxes_.size(), (int)islands0.size(), edges0/2);
        fflush(stderr);
    }
    auto t_bridge = std::chrono::steady_clock::now();
    int next_id = 0;
    for (const auto& b : boxes_) next_id = std::max(next_id, b.id + 1);
    for (int round = 0; round < 5; ++round) {
        if (build_deadline_reached()) break;
        int created = bridge_islands(boxes_, *lect_, obs, n_obs, adj_,
                                     config_.grower.ffb_config, next_id,
                                     has_build_deadline ? build_deadline
                                     : std::chrono::steady_clock::time_point::max());
        if (created == 0) break;
    }
    double bridge_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_bridge).count();
    fprintf(stderr, "[PLN] bridge=%.0fms\n", bridge_ms);
    fflush(stderr);
    {
        auto islands1 = find_islands(adj_);
        fprintf(stderr, "[PLN] post-bridge: boxes=%d islands=%d\n",
                (int)boxes_.size(), (int)islands1.size());
        fflush(stderr);
    }

    last_build_time_ms_ = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_build).count();

    built_ = true;
}

// ─── warmup_lect: obstacle-free LECT expansion ─────────────────────────────
int SBFPlanner::warmup_lect(int max_depth, int n_paths, int seed)
{
    // 1. Construct LECT
    auto root_ivs = robot_.joint_limits().limits;
    lect_ = std::make_unique<LECT>(robot_, root_ivs,
                                    config_.endpoint_source,
                                    config_.envelope_type);
    lect_->set_split_order(config_.split_order);
    if (!config_.z4_enabled)
        lect_->disable_z4();

    // 2. Load cached LECT (if available)
    std::string cache_path;
    int loaded_n_nodes = 0;
    if (!config_.lect_no_cache) {
        cache_path = lect_auto_cache_path();
        if (lect_load_binary(*lect_, robot_, cache_path)) {
            loaded_n_nodes = lect_->n_nodes();
            lect_->clear_forest_state();
            lect_->set_ep_config(config_.endpoint_source);
            fprintf(stderr, "[WRM] lect: loaded %d cached nodes from %s\n",
                    loaded_n_nodes, cache_path.c_str());
        }
    }

    // 3. Warmup: expand random paths to max_depth
    auto t0 = std::chrono::steady_clock::now();
    int new_nodes = lect_->warmup(max_depth, n_paths, seed);
    double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
    fprintf(stderr, "[WRM] warmup=%.0fms (new=%d total=%d depth=%d paths=%d)\n",
            ms, new_nodes, lect_->n_nodes(), max_depth, n_paths);
    fflush(stderr);

    // 4. Save cache
    if (!config_.lect_no_cache && !cache_path.empty()) {
        std::filesystem::create_directories(config_.lect_cache_dir);
        lect_save_incremental(*lect_, cache_path, loaded_n_nodes);
        fprintf(stderr, "[WRM] lect: saved %d nodes to %s\n",
                lect_->n_nodes(), cache_path.c_str());
        fflush(stderr);
    }

    return new_nodes;
}

// ─── build_coverage: wavefront only, no start/goal, no sg_connect/bridge ────
void SBFPlanner::build_coverage(const Obstacle* obs, int n_obs,
                                 double timeout_ms)
{
    auto t_build = std::chrono::steady_clock::now();

    // 1. Construct LECT
    auto root_ivs = robot_.joint_limits().limits;
    auto t_lect = std::chrono::steady_clock::now();
    lect_ = std::make_unique<LECT>(robot_, root_ivs,
                                    config_.endpoint_source,
                                    config_.envelope_type);
    lect_->set_split_order(config_.split_order);
    if (!config_.z4_enabled)
        lect_->disable_z4();

    // Try loading cached LECT
    std::string cache_path;
    int loaded_n_nodes = 0;
    if (!config_.lect_no_cache) {
        cache_path = lect_auto_cache_path();
        if (lect_load_binary(*lect_, robot_, cache_path)) {
            loaded_n_nodes = lect_->n_nodes();
            lect_->clear_forest_state();
            lect_->set_ep_config(config_.endpoint_source);
            fprintf(stderr, "[PLN] lect: loaded %d cached nodes from %s\n",
                    loaded_n_nodes, cache_path.c_str());
        }
    }

    last_lect_time_ms_ = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_lect).count();
    fprintf(stderr, "[PLN] lect=%.0fms (nodes=%d)\n",
            last_lect_time_ms_, lect_->n_nodes());
    fflush(stderr);

    // 2. Grow forest (wavefront, no endpoints)
    config_.grower.timeout_ms = timeout_ms;
    ForestGrower grower(robot_, *lect_, config_.grower);
    // No set_endpoints — grower uses diversity roots only
    auto t_grow = std::chrono::steady_clock::now();
    auto gr = grower.grow(obs, n_obs);
    double grow_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_grow).count();
    fprintf(stderr, "[PLN] grow=%.0fms (boxes=%d)\n",
            grow_ms, (int)gr.boxes.size());
    fflush(stderr);
    boxes_ = std::move(gr.boxes);
    raw_boxes_ = boxes_;
    int n0 = (int)boxes_.size();

    // Incremental save cached LECT
    if (!config_.lect_no_cache && !cache_path.empty()) {
        auto t_save = std::chrono::steady_clock::now();
        std::filesystem::create_directories(config_.lect_cache_dir);
        lect_save_incremental(*lect_, cache_path, loaded_n_nodes);
        double save_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_save).count();
        fprintf(stderr, "[PLN] lect_save=%.0fms (nodes=%d)\n",
                save_ms, lect_->n_nodes());
        fflush(stderr);
    }

    // 3. Coarsen (sweep)
    auto t_sweep = std::chrono::steady_clock::now();
    CollisionChecker checker(robot_, {});
    checker.set_obstacles(obs, n_obs);
    coarsen_forest(boxes_, checker, 10);
    int n1 = (int)boxes_.size();
    double sweep_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_sweep).count();
    fprintf(stderr, "[PLN] sweep=%.0fms (%d->%d)\n", sweep_ms, n0, n1);
    fflush(stderr);

    // 4. Coarsen (greedy)
    auto t_greedy = std::chrono::steady_clock::now();
    coarsen_greedy(boxes_, checker, config_.coarsen, lect_.get());
    int n2 = (int)boxes_.size();
    double greedy_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_greedy).count();
    fprintf(stderr, "[PLN] greedy=%.0fms (%d->%d)\n", greedy_ms, n1, n2);
    fflush(stderr);

    // 4b. Remove coarsen overlaps
    filter_coarsen_overlaps(boxes_);
    int n3 = (int)boxes_.size();
    fprintf(stderr, "[PLN] grow=%d sweep=%d greedy=%d filter=%d\n", n0, n1, n2, n3);
    fflush(stderr);

    // 5. Build adjacency
    adj_ = compute_adjacency(boxes_);

    {
        auto islands = find_islands(adj_);
        int edges = 0;
        for (auto& kv : adj_) edges += (int)kv.second.size();
        fprintf(stderr, "[PLN] coverage: boxes=%d islands=%d edges=%d\n",
                (int)boxes_.size(), (int)islands.size(), edges/2);
        fflush(stderr);
    }

    last_build_time_ms_ = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_build).count();

    built_ = true;
}

PlanResult SBFPlanner::query(const Eigen::VectorXd& start,
                              const Eigen::VectorXd& goal)
{
    PlanResult result;
    if (!built_ || boxes_.empty()) return result;

    auto t0 = std::chrono::steady_clock::now();

    // Find containing boxes
    int start_id = -1, goal_id = -1;
    for (const auto& b : boxes_) {
        if (start_id < 0 && b.contains(start)) start_id = b.id;
        if (goal_id < 0 && b.contains(goal))   goal_id = b.id;
        if (start_id >= 0 && goal_id >= 0) break;
    }
    if (start_id < 0 || goal_id < 0) return result;

    // Search
    std::vector<Eigen::VectorXd> path;
    std::vector<int> box_seq;

    if (config_.use_gcs) {
#ifdef SBF_HAS_DRAKE
        auto gcs_res = gcs_plan(adj_, boxes_, start, goal, config_.gcs);
#else
        auto gcs_res = gcs_plan_fallback(adj_, boxes_, start, goal);
#endif
        if (!gcs_res.found) return result;
        path = std::move(gcs_res.path);
    } else {
        auto dij = dijkstra_search(adj_, boxes_, start_id, goal_id);
        if (!dij.found) return result;
        box_seq = dij.box_sequence;
        path = extract_waypoints(box_seq, boxes_, start, goal);
    }

    // Build box_sequence for smoother (actual BoxNode objects)
    std::unordered_map<int, const BoxNode*> box_map;
    for (const auto& b : boxes_) box_map[b.id] = &b;

    std::vector<BoxNode> seq_boxes;
    for (int id : box_seq) {
        auto it = box_map.find(id);
        if (it != box_map.end()) seq_boxes.push_back(*it->second);
    }

    // Smooth (need a CollisionChecker — construct a lightweight one)
    // Note: we don't have the obstacles stored, so skip smoothing in query
    // if no obstacle info is available. For full smoothing, use plan().

    result.success = true;
    result.path = std::move(path);
    result.box_sequence = std::move(box_seq);
    result.path_length = sbf::path_length(result.path);
    result.n_boxes = static_cast<int>(boxes_.size());

    auto t1 = std::chrono::steady_clock::now();
    result.planning_time_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();

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

    // Find containing boxes (recover from raw_boxes_ if coarsening lost them)
    auto find_containing = [](const std::vector<BoxNode>& bxs,
                              const Eigen::VectorXd& q) -> int {
        for (const auto& b : bxs)
            if (b.contains(q)) return b.id;
        return -1;
    };

    int start_id = find_containing(boxes_, start);
    int goal_id  = find_containing(boxes_, goal);

    if (start_id < 0 || goal_id < 0) {
        // Re-add the raw box that contained start/goal
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
        // Rebuild adjacency with recovered boxes
        if (start_id >= 0 && goal_id >= 0)
            adj_ = compute_adjacency(boxes_);
    }

    if (start_id < 0 || goal_id < 0) { fill_result(); return result; }

    fprintf(stderr, "[PLN] plan: start_id=%d goal_id=%d n_boxes=%d\n",
            start_id, goal_id, (int)boxes_.size());

    // Check adjacency for start/goal
    {
        auto it_s = adj_.find(start_id);
        auto it_g = adj_.find(goal_id);
        fprintf(stderr, "[PLN] adj: start_in=%d(%d) goal_in=%d(%d) total_entries=%d\n",
                it_s != adj_.end(), it_s != adj_.end() ? (int)it_s->second.size() : -1,
                it_g != adj_.end(), it_g != adj_.end() ? (int)it_g->second.size() : -1,
                (int)adj_.size());
    }

    // 6. Search
    std::vector<Eigen::VectorXd> path;
    std::vector<int> box_seq;

    if (config_.use_gcs) {
#ifdef SBF_HAS_DRAKE
        auto gcs_res = gcs_plan(adj_, boxes_, start, goal, config_.gcs);
#else
        auto gcs_res = gcs_plan_fallback(adj_, boxes_, start, goal);
#endif
        if (!gcs_res.found) { fill_result(); return result; }
        path = std::move(gcs_res.path);
    } else {
        auto dij = dijkstra_search(adj_, boxes_, start_id, goal_id);
        if (!dij.found) { fill_result(); return result; }
        box_seq = dij.box_sequence;
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

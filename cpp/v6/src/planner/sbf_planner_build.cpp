// SafeBoxForest v6 — SBF Planner: build / warmup / build_coverage
#include <sbf/planner/sbf_planner.h>
#include <sbf/forest/connectivity.h>
#include <sbf/ffb/ffb.h>
#include <sbf/lect/lect_io.h>

#include <chrono>
#include <cinttypes>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <random>
#include <sbf/core/log.h>

namespace sbf {

// Connectivity contract (planner-level):
// UF all_connected is the canonical truth for connected/not-connected flow.
// Adjacency islands are logged/checked only as diagnostics.

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
            lect_->set_env_config(config_.envelope_type);
            SBF_INFO("[PLN] lect: loaded %d cached nodes from %s", loaded_n_nodes, cache_path.c_str());
        }
    }

    // Initialize V6 Z4-keyed persistent cache
    if (config_.use_v6_cache && !config_.lect_no_cache) {
        if (!cache_mgr_) cache_mgr_ = std::make_unique<LectCacheManager>();
        if (cache_mgr_->init(robot_.fingerprint(), robot_.name(),
                             lect_->n_active_links() * 2 * 6,
                             config_.lect_cache_dir)) {
            lect_->set_cache_manager(cache_mgr_.get());
            SBF_INFO("[PLN] V6 cache: EP safe=%d/%d unsafe=%d/%d, dir=%s", cache_mgr_->ep_cache(0).size(), cache_mgr_->ep_cache(0).capacity(), cache_mgr_->ep_cache(1).size(), cache_mgr_->ep_cache(1).capacity(), cache_mgr_->cache_dir().c_str());
        } else {
            SBF_WARN("[PLN] V6 cache init failed, falling back to V5");
            cache_mgr_.reset();
        }
    }

    last_lect_time_ms_ = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_lect).count();
    SBF_INFO("[PLN] lect=%.0fms (nodes=%d)", last_lect_time_ms_, lect_->n_nodes());
    // 2. Grow forest
    config_.grower.timeout_ms = timeout_ms;
    ForestGrower grower(robot_, *lect_, config_.grower);
    grower.set_endpoints(start, goal);
    auto t_grow = std::chrono::steady_clock::now();
    auto gr = grower.grow(obs, n_obs);
    double grow_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_grow).count();
    SBF_INFO("[PLN] grow=%.0fms (boxes=%d)", grow_ms, (int)gr.boxes.size());
    boxes_ = std::move(gr.boxes);
    raw_boxes_ = boxes_;  // snapshot before coarsening
    int n0 = (int)boxes_.size();

    auto log_stage_connectivity = [&](const char* stage) {
        auto stage_adj = compute_adjacency(boxes_);
        auto stage_islands = find_islands(stage_adj);
        int stage_edges = 0;
        int largest_island = 0;
        for (const auto& kv : stage_adj) stage_edges += (int)kv.second.size();
        for (const auto& isl : stage_islands)
            largest_island = std::max(largest_island, (int)isl.size());
        SBF_INFO("[PLN] stage=%s boxes=%d islands=%d largest=%d edges=%d",
                 stage, (int)boxes_.size(), (int)stage_islands.size(),
                 largest_island, stage_edges / 2);
    };
    log_stage_connectivity("post-grow");

    // Incremental save cached LECT
    if (!config_.lect_no_cache && !cache_path.empty()) {
        std::filesystem::create_directories(config_.lect_cache_dir);
        lect_save_incremental(*lect_, cache_path, loaded_n_nodes);
        SBF_INFO("[PLN] lect: saved %d nodes to %s", lect_->n_nodes(), cache_path.c_str());
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Pipeline: Grow → Coarsen1 → Bridge → Coarsen2 → Filter → Adjacency
    //   (coarsen first for larger boxes → better bridge, then coarsen again
    //    to merge bridge chain boxes)
    // ═══════════════════════════════════════════════════════════════════════

    // 2. First Coarsen pass (sweep + greedy on raw boxes)
    CollisionChecker checker(robot_, {});
    checker.set_obstacles(obs, n_obs);
    int n_sweep1 = n0;

    if (config_.enable_coarsen) {
        auto t_sweep1 = std::chrono::steady_clock::now();
        if (!build_deadline_reached())
            coarsen_forest(boxes_, checker, 10);
        n_sweep1 = (int)boxes_.size();
        double sweep1_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_sweep1).count();
        SBF_INFO("[PLN] sweep1=%.0fms (%d->%d)", sweep1_ms, n0, n_sweep1);
        {
            auto pre_adj_1 = compute_adjacency(boxes_);
            auto bridge_ids_1 = find_articulation_points(pre_adj_1);
            auto t_greedy1 = std::chrono::steady_clock::now();
            if (!build_deadline_reached())
                coarsen_greedy(boxes_, checker, config_.coarsen, lect_.get(), &bridge_ids_1, &pre_adj_1);
            int n_greedy1 = (int)boxes_.size();
            double greedy1_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t_greedy1).count();
            SBF_INFO("[PLN] greedy1=%.0fms (%d->%d)", greedy1_ms, n_sweep1, n_greedy1);
        }
        log_stage_connectivity("post-coarsen1");
    } else {
        SBF_INFO("[PLN] coarsen1 SKIPPED (enable_coarsen=false)");
    }
    log_stage_connectivity("pre-bridge");

    // 3. Bridge S↔T on coarsened boxes (larger boxes → better RRT success)
    {
        auto t_bridge = std::chrono::steady_clock::now();
        // Find start_box and goal_box (with nearest-box fallback)
        int start_box = -1, goal_box = -1;
        double best_s_dist = std::numeric_limits<double>::max();
        double best_g_dist = std::numeric_limits<double>::max();
        for (const auto& b : boxes_) {
            if (b.contains(start)) { start_box = b.id; best_s_dist = -1.0; }
            if (b.contains(goal))  { goal_box  = b.id; best_g_dist = -1.0; }
            if (best_s_dist >= 0.0) {
                double d = (b.center() - start).squaredNorm();
                if (d < best_s_dist) { best_s_dist = d; start_box = b.id; }
            }
            if (best_g_dist >= 0.0) {
                double d = (b.center() - goal).squaredNorm();
                if (d < best_g_dist) { best_g_dist = d; goal_box = b.id; }
            }
        }

        int bridge_count = 0;
        if (start_box >= 0 && goal_box >= 0 && !build_deadline_reached()) {
            adj_ = compute_adjacency(boxes_);
            for (const auto& b : boxes_) {
                if (adj_.find(b.id) == adj_.end())
                    adj_[b.id] = {};
            }
            int next_id = 0;
            for (const auto& b : boxes_) next_id = std::max(next_id, b.id + 1);

            bridge_count = bridge_s_t(
                start_box, goal_box, boxes_, *lect_, obs, n_obs,
                adj_, config_.grower.ffb_config, next_id,
                robot_, checker,
                /*per_pair_timeout_ms=*/200.0, /*max_pairs=*/30,
                has_build_deadline ? build_deadline
                : std::chrono::steady_clock::time_point::max());
        }
        double bridge_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_bridge).count();
        int n_after_bridge = (int)boxes_.size();
        {
            auto islands_post_bridge = find_islands(adj_);
            SBF_INFO("[PLN] bridge=%.0fms added=%d boxes=%d islands=%d", bridge_ms, bridge_count, n_after_bridge, (int)islands_post_bridge.size());
        }
        // Repair forced-adjacency edges so compute_adjacency() can detect them
        int repaired = repair_bridge_adjacency(boxes_, adj_);
        if (repaired > 0)
            SBF_INFO("[PLN] repaired %d non-geometric bridge edges", repaired);
    }

    // 4. Second Coarsen pass (sweep + greedy)
    int n_pre_coarsen2 = (int)boxes_.size();
    if (config_.enable_coarsen) {
        auto t_sweep2 = std::chrono::steady_clock::now();
        if (!build_deadline_reached())
            coarsen_forest(boxes_, checker, 10);
        int n_sweep2 = (int)boxes_.size();
        double sweep2_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_sweep2).count();
        SBF_INFO("[PLN] sweep2=%.0fms (%d->%d)", sweep2_ms, n_pre_coarsen2, n_sweep2);
        // Greedy2 with fresh articulation-point protection
        if (!build_deadline_reached()) {
            auto post_adj_s2 = compute_adjacency(boxes_);
            auto bridge_ids_s2 = find_articulation_points(post_adj_s2);
            auto t_greedy2 = std::chrono::steady_clock::now();
            coarsen_greedy(boxes_, checker, config_.coarsen, lect_.get(), &bridge_ids_s2, &post_adj_s2);
            int n_greedy2 = (int)boxes_.size();
            double greedy2_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t_greedy2).count();
            SBF_INFO("[PLN] greedy2=%.0fms (%d->%d)", greedy2_ms, n_sweep2, n_greedy2);
        }
    } else {
        SBF_INFO("[PLN] coarsen2 SKIPPED (enable_coarsen=false)");
    }

    // 4b. Remove coarsen overlaps — protect articulation points
    auto post_adj_g = compute_adjacency(boxes_);
    auto bridge_ids_g2 = find_articulation_points(post_adj_g);
    filter_coarsen_overlaps(boxes_, 1e-4, &bridge_ids_g2);
    int n3 = (int)boxes_.size();
    SBF_INFO("[PLN] grow=%d bridge=%d coarsen2=%d filter=%d", (int)raw_boxes_.size(), n_pre_coarsen2, (int)boxes_.size(), n3);
    // 5. Build final adjacency + connectivity check
    adj_ = compute_adjacency(boxes_);
    {
        auto islands_final = find_islands(adj_);
        int edges_final = 0;
        for (auto& kv : adj_) edges_final += (int)kv.second.size();
        SBF_INFO("[PLN] final: boxes=%d islands=%d edges=%d", (int)boxes_.size(), (int)islands_final.size(), edges_final/2);
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
            lect_->set_env_config(config_.envelope_type);
            SBF_INFO("[WRM] lect: loaded %d cached nodes from %s", loaded_n_nodes, cache_path.c_str());
        }
    }

    // Initialize V6 Z4-keyed persistent cache
    if (config_.use_v6_cache && !config_.lect_no_cache) {
        if (!cache_mgr_) cache_mgr_ = std::make_unique<LectCacheManager>();
        if (cache_mgr_->init(robot_.fingerprint(), robot_.name(),
                             lect_->n_active_links() * 2 * 6,
                             config_.lect_cache_dir)) {
            lect_->set_cache_manager(cache_mgr_.get());
            SBF_INFO("[WRM] V6 cache: EP safe=%d/%d unsafe=%d/%d, dir=%s", cache_mgr_->ep_cache(0).size(), cache_mgr_->ep_cache(0).capacity(), cache_mgr_->ep_cache(1).size(), cache_mgr_->ep_cache(1).capacity(), cache_mgr_->cache_dir().c_str());
        } else {
            SBF_WARN("[WRM] V6 cache init failed, falling back to V5");
            cache_mgr_.reset();
        }
    }

    // 3. Warmup: expand random paths to max_depth
    auto t0 = std::chrono::steady_clock::now();
    int new_nodes = lect_->warmup(max_depth, n_paths, seed);
    double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
    SBF_INFO("[WRM] warmup=%.0fms (new=%d total=%d depth=%d paths=%d)", ms, new_nodes, lect_->n_nodes(), max_depth, n_paths);
    // 4. Save cache
    if (!config_.lect_no_cache && !cache_path.empty()) {
        std::filesystem::create_directories(config_.lect_cache_dir);
        lect_save_incremental(*lect_, cache_path, loaded_n_nodes);
        SBF_INFO("[WRM] lect: saved %d nodes to %s", lect_->n_nodes(), cache_path.c_str());
    }

    return new_nodes;
}

// ─── build_coverage: wavefront only, no start/goal, no sg_connect/bridge ────
void SBFPlanner::build_coverage(const Obstacle* obs, int n_obs,
                                 double timeout_ms,
                                 const std::vector<Eigen::VectorXd>& seed_points)
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
            lect_->set_env_config(config_.envelope_type);
            SBF_INFO("[PLN] lect: loaded %d cached nodes from %s", loaded_n_nodes, cache_path.c_str());
        }
    }

    // Initialize V6 Z4-keyed persistent cache
    if (config_.use_v6_cache && !config_.lect_no_cache) {
        if (!cache_mgr_) cache_mgr_ = std::make_unique<LectCacheManager>();
        if (cache_mgr_->init(robot_.fingerprint(), robot_.name(),
                             lect_->n_active_links() * 2 * 6,
                             config_.lect_cache_dir)) {
            lect_->set_cache_manager(cache_mgr_.get());
            SBF_INFO("[PLN] V6 cache: EP safe=%d/%d unsafe=%d/%d, dir=%s", cache_mgr_->ep_cache(0).size(), cache_mgr_->ep_cache(0).capacity(), cache_mgr_->ep_cache(1).size(), cache_mgr_->ep_cache(1).capacity(), cache_mgr_->cache_dir().c_str());
        } else {
            SBF_WARN("[PLN] V6 cache init failed, falling back to V5");
            cache_mgr_.reset();
        }
    }

    last_lect_time_ms_ = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_lect).count();
    last_build_timing_.lect_ms = last_lect_time_ms_;
    SBF_INFO("[PLN] lect=%.0fms (nodes=%d)", last_lect_time_ms_, lect_->n_nodes());
    // 2. Grow forest (multi-goal RRT)
    config_.grower.mode = GrowerConfig::Mode::RRT;
    config_.grower.timeout_ms = timeout_ms;
    // Grow-stage optimization:
    // 1) cap overly deep FFB depth to reduce per-box expansion cost.
    // 2) cap post-connect coverage budget to avoid long tail after trees connect.
    config_.grower.ffb_config.max_depth = std::max(config_.grower.ffb_config.max_depth, 60);
    if (config_.grower.ffb_config.max_depth > 220) {
        SBF_INFO("[PLN] grow-opt: cap ffb max_depth %d -> 220",
                 config_.grower.ffb_config.max_depth);
        config_.grower.ffb_config.max_depth = 220;
    }

    // Use provided seed points (e.g. Marcucci poststone configs)
    std::vector<Eigen::VectorXd> seeds = seed_points;
    if (seeds.empty()) {
        // Fallback: single random seed in IAABB
        const auto& jlimits = robot_.joint_limits().limits;
        const int nd = robot_.n_joints();
        std::mt19937_64 seed_rng(config_.grower.rng_seed + 7777);
        std::uniform_real_distribution<double> u01(0.0, 1.0);
        Eigen::VectorXd q(nd);
        for (int d = 0; d < nd; d++)
            q[d] = jlimits[d].lo + u01(seed_rng) * jlimits[d].width();
        seeds.push_back(q);
    }

    const int nd = robot_.n_joints();
    if (config_.grower.connect_mode && seeds.size() > 1) {
        const int target_extra = std::max(3000, 1000 * (int)seeds.size());
        if (config_.grower.post_connect_extra_boxes <= 0 ||
            config_.grower.post_connect_extra_boxes > target_extra) {
            SBF_INFO("[PLN] grow-opt: post_connect_extra_boxes %d -> %d",
                     config_.grower.post_connect_extra_boxes, target_extra);
            config_.grower.post_connect_extra_boxes = target_extra;
        }
    }

    SBF_INFO("[PLN] multi-goal RRT: %d seed points", (int)seeds.size());
    for (int i = 0; i < static_cast<int>(seeds.size()); i++) {
        SBF_INFO("[PLN]   seed[%d] = (", i);
        for (int d = 0; d < nd; d++)
            SBF_INFO("%.3f%s", seeds[i][d], d < nd - 1 ? ", " : "");
        SBF_INFO(")");
    }
    ForestGrower grower(robot_, *lect_, config_.grower);
    grower.set_multi_goals(seeds);

    auto t_grow = std::chrono::steady_clock::now();
    auto gr = grower.grow(obs, n_obs);
    double grow_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_grow).count();
    SBF_INFO("[PLN] grow=%.0fms (boxes=%d)", grow_ms, (int)gr.boxes.size());
    boxes_ = std::move(gr.boxes);
    raw_boxes_ = boxes_;
    int n0 = (int)boxes_.size();
    bool grow_connected = gr.all_connected;
    SBF_INFO("[PLN] grow connectivity: uf_connected=%d | tree_uf_connected=%d tree_uf_t=%.0fms tree_uf_boxes=%d | adj_connected=%d islands=%d largest=%d adj_check=%.2fms",
             gr.all_connected ? 1 : 0,
             gr.tree_all_connected ? 1 : 0,
             gr.tree_connect_time_ms,
             gr.tree_connect_n_boxes,
             gr.adjacency_all_connected ? 1 : 0,
             gr.adjacency_islands,
             gr.adjacency_largest_island,
             gr.adjacency_check_ms);
    SBF_INFO("[PLN] canonical connectivity source: UF (adjacency islands are diagnostic only)");

    // OPT: diagnostic-only stage logging is gated behind SBF_STAGE_LOG env
    // var. Each call rebuilds compute_adjacency on 4000+ boxes (~50ms),
    // and is called 3× post-grow/post-coarsen1/pre-bridge ⇒ ~150ms wasted
    // when only logging is desired.
    static const bool kStageLogEnabled = [] {
        const char* e = std::getenv("SBF_STAGE_LOG");
        return e && e[0] && e[0] != '0';
    }();
    auto log_stage_connectivity = [&](const char* stage) {
        if (!kStageLogEnabled) return;
        auto stage_adj = compute_adjacency(boxes_, config_.adjacency_tol, 0,
                                           config_.adjacency_gap_tol);
        auto stage_islands = find_islands(stage_adj);
        int stage_edges = 0;
        int largest_island = 0;
        for (const auto& kv : stage_adj) stage_edges += (int)kv.second.size();
        for (const auto& isl : stage_islands)
            largest_island = std::max(largest_island, (int)isl.size());
        SBF_INFO("[PLN] stage=%s boxes=%d islands=%d largest=%d edges=%d (tol=%.1e gap_tol=%.1e)",
                 stage, (int)boxes_.size(), (int)stage_islands.size(),
                 largest_island, stage_edges / 2,
                 config_.adjacency_tol, config_.adjacency_gap_tol);
    };

    last_build_timing_.grow_ms = grow_ms;
    last_build_timing_.boxes_after_grow = n0;

    {
        SBF_INFO("[PLN] post-grow: boxes=%d", n0);
        // Per-tree box count
        std::unordered_map<int, int> tree_counts;
        for (const auto& b : boxes_) tree_counts[b.root_id]++;
        SBF_INFO("[PLN] tree sizes:");
        std::vector<std::pair<int,int>> sorted_trees(tree_counts.begin(), tree_counts.end());
        std::sort(sorted_trees.begin(), sorted_trees.end());
        for (auto& [rid, cnt] : sorted_trees)
            SBF_INFO(" root%d=%d", rid, cnt);
        SBF_INFO("");
    }
    log_stage_connectivity("post-grow");

    // Save cached LECT (synchronous — materialize_mmap is private,
    // so we let the save function handle it internally).
    if (!config_.lect_no_cache && !cache_path.empty()) {
        std::filesystem::create_directories(config_.lect_cache_dir);
        auto t_save = std::chrono::steady_clock::now();
        lect_save_incremental(*lect_, cache_path, loaded_n_nodes);
        double save_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_save).count();
        SBF_INFO("[PLN] lect_save=%.0fms (nodes=%d)", save_ms, lect_->n_nodes());
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Pipeline: Grow → Coarsen1 → Bridge → Coarsen2 → Filter
    //   (coarsen first for larger boxes → better bridge, then coarsen again
    //    to merge bridge chain boxes)
    // ═══════════════════════════════════════════════════════════════════════

    // Phase checkpoints for BuildTimingProfile
    auto t_coarsen1_start = std::chrono::steady_clock::now();

    // 2. First Coarsen pass (sweep + greedy on raw boxes)
    CollisionChecker checker(robot_, {});
    checker.set_obstacles(obs, n_obs);
    int n_sweep1 = n0;

    if (config_.enable_coarsen) {
    auto t_sweep1 = std::chrono::steady_clock::now();
    coarsen_forest(boxes_, checker, 20, 0.5);
    n_sweep1 = (int)boxes_.size();
    double sweep1_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_sweep1).count();
    SBF_INFO("[PLN] sweep1=%.0fms (%d->%d)", sweep1_ms, n0, n_sweep1);
    {
        auto t_rsweep1 = std::chrono::steady_clock::now();
        // OPT: cap max_rounds 20→4. Converges in ~3 rounds; extra rounds
        // yield <1% reduction for ~80ms of sorting work.
        coarsen_sweep_relaxed(boxes_, checker, lect_.get(),
                               4, 15.0, 10000, 0.5);
        int n_rsweep1 = (int)boxes_.size();
        double rsweep1_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_rsweep1).count();
        SBF_INFO("[PLN] relaxed_sweep1=%.0fms (%d->%d)", rsweep1_ms, n_sweep1, n_rsweep1);
    }

    {
        auto t_artic1 = std::chrono::steady_clock::now();
        auto pre_adj_1 = compute_adjacency(boxes_);
        auto bridge_ids_1 = find_articulation_points(pre_adj_1);
        double artic1_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_artic1).count();
        SBF_INFO("[PLN] articulation points: %d (adj+artic %.0fms)", (int)bridge_ids_1.size(), artic1_ms);
        auto t_greedy1 = std::chrono::steady_clock::now();
        int n_pre_greedy1 = (int)boxes_.size();
        // OPT: pass pre_adj_1 to greedy to avoid redundant compute_adjacency
        // rebuild in its first round (~50ms saved).
        // OPT: cap max_rounds & timeout for greedy1 — sweep+relaxed_sweep
        // already handled easy merges, greedy rarely benefits past round 5.
        GreedyCoarsenConfig greedy1_cfg = config_.coarsen;
        greedy1_cfg.max_rounds = 2;
        greedy1_cfg.timeout_ms = 250.0;
        coarsen_greedy(boxes_, checker, greedy1_cfg, lect_.get(),
                       &bridge_ids_1, &pre_adj_1);
        int n_greedy1 = (int)boxes_.size();
        double greedy1_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_greedy1).count();
        SBF_INFO("[PLN] greedy1=%.0fms (%d->%d)", greedy1_ms, n_pre_greedy1, n_greedy1);
        // OPT: cluster1 skipped — historical data shows it only merges ~20
        // clusters (< 1% reduction) while costing ~140ms.  The preceding
        // sweep + relaxed_sweep + greedy already captures the easy merges.
    }
    } else {
        SBF_INFO("[PLN] coarsen1 SKIPPED (enable_coarsen=false)");
    }

    auto t_coarsen1_end = std::chrono::steady_clock::now();
    last_build_timing_.coarsen1_ms = std::chrono::duration<double, std::milli>(t_coarsen1_end - t_coarsen1_start).count();
    last_build_timing_.boxes_after_coarsen1 = (int)boxes_.size();
    log_stage_connectivity("post-coarsen1");
    log_stage_connectivity("pre-bridge");

    // 3. Bridge all islands (skip if grow already connected all trees)
    auto t_bridge_start = std::chrono::steady_clock::now();
    if (!grow_connected) {
        auto t_adj_br = std::chrono::steady_clock::now();
        adj_ = compute_adjacency(boxes_);
        double adj_br_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_adj_br).count();
        {
            auto pre_islands = find_islands(adj_);
            SBF_INFO("[PLN] pre-bridge adj: %.0fms (boxes=%d, islands=%d)", adj_br_ms, (int)boxes_.size(), (int)pre_islands.size());
        }
        int next_id_br = 0;
        for (const auto& b : boxes_) next_id_br = std::max(next_id_br, b.id + 1);
        auto ffb_cfg_br = config_.grower.ffb_config;
        ffb_cfg_br.max_depth = std::max(ffb_cfg_br.max_depth, 60);

        auto t_brg = std::chrono::steady_clock::now();
        constexpr double bridge_per_pair_timeout_ms = 1000.0;
        constexpr int bridge_max_pairs_per_gap = 10;
        constexpr int bridge_max_total_boxes = 1500;
        int bridged = bridge_all_islands(
            boxes_, *lect_, obs, n_obs, adj_, ffb_cfg_br, next_id_br,
            robot_, checker,
            bridge_per_pair_timeout_ms,
            bridge_max_pairs_per_gap,
            bridge_max_total_boxes,
            config_.grower.bridge_n_threads > 0
                ? config_.grower.bridge_n_threads
                : config_.grower.n_threads);
        double brg_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_brg).count();
        {
            auto brg_islands = find_islands(adj_);
            SBF_INFO("[PLN] bridge_all: +%d boxes (%.0fms, total=%d, islands=%d)", bridged, brg_ms, (int)boxes_.size(), (int)brg_islands.size());
        }
        // Repair forced-adjacency edges so compute_adjacency() can detect them
        int repaired = repair_bridge_adjacency(boxes_, adj_);
        if (repaired > 0)
            SBF_INFO("[PLN] repaired %d non-geometric bridge edges", repaired);
    }

    auto t_bridge_end = std::chrono::steady_clock::now();
    last_build_timing_.bridge_ms = std::chrono::duration<double, std::milli>(t_bridge_end - t_bridge_start).count();
    last_build_timing_.boxes_after_bridge = (int)boxes_.size();

    // 4. Second Coarsen pass (skip if grow already connected — bridge boxes don't exist)
    int n_pre_coarsen2 = (int)boxes_.size();
    int n_greedy2 = n_pre_coarsen2;  // updated below
    if (!grow_connected && config_.enable_coarsen) {
        auto t_sweep2 = std::chrono::steady_clock::now();
        coarsen_forest(boxes_, checker, 15);
        int n_sweep2 = (int)boxes_.size();
        double sweep2_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_sweep2).count();
        SBF_INFO("[PLN] sweep2=%.0fms (%d->%d)", sweep2_ms, n_pre_coarsen2, n_sweep2);
        {
            auto t_rsweep2 = std::chrono::steady_clock::now();
            coarsen_sweep_relaxed(boxes_, checker, lect_.get(),
                                   10, 15.0, 5000);
            int n_rsweep2 = (int)boxes_.size();
            double rsweep2_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t_rsweep2).count();
            SBF_INFO("[PLN] relaxed_sweep2=%.0fms (%d->%d)", rsweep2_ms, n_sweep2, n_rsweep2);
        }

        // Greedy2 with fresh articulation-point protection (looser params post-bridge)
        auto post_adj_s2 = compute_adjacency(boxes_);
        auto bridge_ids_s2 = find_articulation_points(post_adj_s2);
        auto t_greedy2 = std::chrono::steady_clock::now();
        {
            GreedyCoarsenConfig coarsen2_cfg = config_.coarsen;
            coarsen2_cfg.score_threshold = 50.0;
            coarsen2_cfg.max_lect_fk_per_round = 30000;
            coarsen_greedy(boxes_, checker, coarsen2_cfg, lect_.get(), &bridge_ids_s2, &post_adj_s2);
        }
        n_greedy2 = (int)boxes_.size();
        double greedy2_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_greedy2).count();
        SBF_INFO("[PLN] greedy2=%.0fms (%d->%d)", greedy2_ms, n_sweep2, n_greedy2);
        // Cluster merge on coarsen2
        {
            auto t_cluster2 = std::chrono::steady_clock::now();
            ClusterCoarsenConfig cl2_cfg;
            cl2_cfg.max_cluster_size = 20;
            cl2_cfg.score_threshold = 50.0;
            cl2_cfg.max_lect_fk_per_round = 20000;
            auto cl2 = coarsen_cluster(boxes_, checker, cl2_cfg, lect_.get(),
                                       nullptr);  // no articulation protection: hull ⊇ original box
            double cl2_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t_cluster2).count();
            SBF_INFO("[PLN] cluster2=%.0fms (%d->%d, %d clusters)", cl2_ms, cl2.boxes_before, cl2.boxes_after, cl2.clusters_formed);
        }
    } else if (grow_connected) {
        SBF_INFO("[PLN] skip bridge+coarsen2 (grow already connected)");
    } else {
        SBF_INFO("[PLN] coarsen2 SKIPPED (enable_coarsen=false)");
    }

    last_build_timing_.coarsen2_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_bridge_end).count();
    last_build_timing_.boxes_after_coarsen2 = (int)boxes_.size();

    // 4b. Remove coarsen overlaps — protect articulation points
    auto t_filter = std::chrono::steady_clock::now();
    auto post_adj = compute_adjacency(boxes_);
    auto bridge_ids2 = find_articulation_points(post_adj);
    filter_coarsen_overlaps(boxes_, 1e-4, &bridge_ids2);
    int n3 = (int)boxes_.size();
    double filter_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_filter).count();

    SBF_INFO("[PLN] grow=%d coarsen1=%d bridge=%d coarsen2=%d filter=%d (%.0fms)", (int)raw_boxes_.size(), n_sweep1, n_pre_coarsen2, n_greedy2, n3, filter_ms);
    // 5. Build final adjacency  (方案a+c: use relaxed tolerances)
    double adjacency_ms_accum = 0.0;
    {
        auto t_adj_pre = std::chrono::steady_clock::now();
        adj_ = compute_adjacency(boxes_, config_.adjacency_tol, 0,
                                 config_.adjacency_gap_tol);
        last_build_timing_.adjacency_pre_seed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_adj_pre).count();
        adjacency_ms_accum += last_build_timing_.adjacency_pre_seed_ms;
    }

    // OP-1: Seed-point bridge guarantee — ensure query endpoints are
    // reachable from main island (post-coarsen to avoid fragmentation)
    last_build_timing_.seed_bridge_ms = 0.0;
    if (!seed_points.empty()) {
        auto t_sp_bridge = std::chrono::steady_clock::now();
        constexpr double sp_bridge_budget_ms = 800.0;  // total budget

        auto islands = find_islands(adj_);
        int largest_idx = 0;
        for (int i = 1; i < (int)islands.size(); ++i)
            if (islands[i].size() > islands[largest_idx].size())
                largest_idx = i;

        // OPT: Incremental connectivity tracking via UnionFind on box ids.
        // Instead of rebuilding compute_adjacency (O(n log n + n·k)) after
        // each seed-point bridge, maintain UF over all box ids seeded from
        // the current adj_ graph.  After each bridge_s_t, do a local
        // adjacency check for NEW boxes only (captures side-effect mergers
        // where chain boxes touch other small islands).  Defer full
        // compute_adjacency rebuild to after the loop.
        std::unordered_map<int, int> bid2uf;
        bid2uf.reserve((int)boxes_.size() * 2);
        for (const auto& b : boxes_) bid2uf[b.id] = (int)bid2uf.size();
        UnionFind sp_uf((int)boxes_.size() + (int)seed_points.size() * 1000);
        for (auto& kv : adj_) {
            auto itA = bid2uf.find(kv.first);
            if (itA == bid2uf.end()) continue;
            for (int nb_id : kv.second) {
                auto itB = bid2uf.find(nb_id);
                if (itB != bid2uf.end())
                    sp_uf.unite(itA->second, itB->second);
            }
        }
        int main_rep_uf = -1;
        if (!islands[largest_idx].empty())
            main_rep_uf = sp_uf.find(bid2uf[islands[largest_idx][0]]);

        auto ffb_cfg_sp = config_.grower.ffb_config;
        ffb_cfg_sp.max_depth = std::max(ffb_cfg_sp.max_depth, 60);
        int sp_bridged = 0;

        for (const auto& sp : seed_points) {
            // Total budget check
            double sp_elapsed = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t_sp_bridge).count();
            if (sp_elapsed >= sp_bridge_budget_ms) break;

            int sp_id = -1;
            double best_d = std::numeric_limits<double>::max();
            for (const auto& b : boxes_) {
                if (b.contains(sp)) { sp_id = b.id; break; }
                double d = (b.center() - sp).squaredNorm();
                if (d < best_d) { best_d = d; sp_id = b.id; }
            }
            if (sp_id < 0) continue;
            // Incremental main-set check via UF
            auto itSp = bid2uf.find(sp_id);
            if (itSp != bid2uf.end() && main_rep_uf >= 0 &&
                sp_uf.find(itSp->second) == main_rep_uf) continue;

            // Find nearest main-island box
            int tgt_id = -1;
            double best_tgt = std::numeric_limits<double>::max();
            Eigen::VectorXd sp_center;
            for (const auto& b : boxes_)
                if (b.id == sp_id) { sp_center = b.center(); break; }
            for (const auto& b : boxes_) {
                auto itB = bid2uf.find(b.id);
                if (itB == bid2uf.end()) continue;
                if (sp_uf.find(itB->second) != main_rep_uf) continue;
                double d = (b.center() - sp_center).squaredNorm();
                if (d < best_tgt) { best_tgt = d; tgt_id = b.id; }
            }
            if (tgt_id < 0) continue;

            // Distance-adaptive timeout and max_pairs
            double dist = std::sqrt(best_tgt);
            // OPT: cap per_pair timeout 1500→800ms and max_pairs 5→1.
            // bridge_s_t's internal s_t_connected() uses stale adj, so it
            // can't early-exit. The outer UF correctly captures connectivity
            // after chain_pave, so 1 pair is sufficient — a 2nd pair just
            // duplicates 500 paved boxes for no connectivity gain.
            double per_pair_timeout = std::min(800.0, 250.0 + 100.0 * dist);
            int max_pairs = 1;

            int next_id_sp = 0;
            for (const auto& b : boxes_) next_id_sp = std::max(next_id_sp, b.id + 1);

            CollisionChecker sp_checker(robot_, {});
            sp_checker.set_obstacles(obs, n_obs);
            int n_before = (int)boxes_.size();
            int created = bridge_s_t(
                sp_id, tgt_id, boxes_, *lect_, obs, n_obs,
                adj_, ffb_cfg_sp, next_id_sp,
                robot_, sp_checker, per_pair_timeout, max_pairs,
                std::chrono::steady_clock::time_point::max());
            sp_bridged += created;

            // Register new boxes in bid2uf and union with parent
            for (int bi = n_before; bi < (int)boxes_.size(); ++bi) {
                if (bid2uf.count(boxes_[bi].id)) continue;
                int slot = (int)bid2uf.size();
                bid2uf[boxes_[bi].id] = slot;
                auto itP = bid2uf.find(boxes_[bi].parent_box_id);
                if (itP != bid2uf.end())
                    sp_uf.unite(slot, itP->second);
            }

            if (created > 0) {
                // Primary guarantee: chain sp → tgt
                auto itA = bid2uf.find(sp_id);
                auto itB = bid2uf.find(tgt_id);
                if (itA != bid2uf.end() && itB != bid2uf.end())
                    sp_uf.unite(itA->second, itB->second);

                // Side-effect capture: chain boxes may touch OTHER small
                // islands.  Scan new boxes' adjacency against all existing
                // boxes (dim-0 sweep) to union.
                const int nb_total = (int)boxes_.size();
                std::vector<int> dim0_order(nb_total);
                std::iota(dim0_order.begin(), dim0_order.end(), 0);
                std::sort(dim0_order.begin(), dim0_order.end(),
                          [&](int a, int b) {
                    return boxes_[a].joint_intervals[0].lo
                         < boxes_[b].joint_intervals[0].lo;
                });
                std::vector<int> pos(nb_total);
                for (int i = 0; i < nb_total; ++i) pos[dim0_order[i]] = i;

                for (int bi = n_before; bi < nb_total; ++bi) {
                    double lo0 = boxes_[bi].joint_intervals[0].lo;
                    double hi0 = boxes_[bi].joint_intervals[0].hi;
                    // Find range of dim0_order whose lo <= hi0 and hi >= lo0
                    int idx = pos[bi];
                    // Scan forward
                    for (int k = idx + 1; k < nb_total; ++k) {
                        int bj = dim0_order[k];
                        if (boxes_[bj].joint_intervals[0].lo > hi0 + 1e-6) break;
                        if (boxes_adjacent(boxes_[bi], boxes_[bj])) {
                            auto itAA = bid2uf.find(boxes_[bi].id);
                            auto itBB = bid2uf.find(boxes_[bj].id);
                            if (itAA != bid2uf.end() && itBB != bid2uf.end())
                                sp_uf.unite(itAA->second, itBB->second);
                        }
                    }
                    // Scan backward
                    for (int k = idx - 1; k >= 0; --k) {
                        int bj = dim0_order[k];
                        if (boxes_[bj].joint_intervals[0].hi < lo0 - 1e-6) break;
                        if (boxes_adjacent(boxes_[bi], boxes_[bj])) {
                            auto itAA = bid2uf.find(boxes_[bi].id);
                            auto itBB = bid2uf.find(boxes_[bj].id);
                            if (itAA != bid2uf.end() && itBB != bid2uf.end())
                                sp_uf.unite(itAA->second, itBB->second);
                        }
                    }
                }
                if (itB != bid2uf.end() && main_rep_uf >= 0)
                    main_rep_uf = sp_uf.find(itB->second);
            }
        }

        // OPT: Defer full compute_adjacency to a single rebuild at end
        if (sp_bridged > 0) {
            SBF_INFO("[PLN] seed-point bridge: +%d boxes (%d seed_points)", sp_bridged, (int)seed_points.size());
            repair_bridge_adjacency(boxes_, adj_);
            auto t_adj_rebuild = std::chrono::steady_clock::now();
            adj_ = compute_adjacency(boxes_, config_.adjacency_tol, 0,
                                     config_.adjacency_gap_tol);
            adjacency_ms_accum += std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t_adj_rebuild).count();
        }
        last_build_timing_.seed_bridge_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_sp_bridge).count();
    }

    // Reliability fallback: if connectivity is still poor after the tuned
    // bridge budgets AND the forest is NOT tree-connected, run one stronger
    // rescue bridge pass.
    //
    // NOTE: gated on grow_connected=false. When the grower already got all
    // trees into a single tree-UF component, the remaining adjacency
    // islands are within-tree artifacts (thin coarsen-caused gaps that the
    // seed-point bridge handles in the query endpoints). Running a blanket
    // all-pair bridge here empirically adds +2000 boxes in ~1s with zero
    // islands-count reduction — pure waste.
    if (!grow_connected) {
        auto islands_now = find_islands(adj_);
        if ((int)islands_now.size() > 2) {
            SBF_INFO("[PLN] rescue bridge: islands=%d -> running strong fallback", (int)islands_now.size());

            int next_id_rb = 0;
            for (const auto& b : boxes_) next_id_rb = std::max(next_id_rb, b.id + 1);

            auto ffb_cfg_rb = config_.grower.ffb_config;
            ffb_cfg_rb.max_depth = std::max(ffb_cfg_rb.max_depth, 60);

            auto t_rb = std::chrono::steady_clock::now();
            int rescued = bridge_all_islands(
                boxes_, *lect_, obs, n_obs, adj_, ffb_cfg_rb, next_id_rb,
                robot_, checker,
                /*per_pair_timeout_ms=*/2000.0,
                /*max_pairs_per_gap=*/15,
                /*max_total_bridges=*/2000,
                config_.grower.bridge_n_threads > 0
                    ? config_.grower.bridge_n_threads
                    : config_.grower.n_threads);
            double rb_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t_rb).count();

            // Always rebuild: even rescued==0 may have mutated adj_ via
            // forced chain_pave edges.
            repair_bridge_adjacency(boxes_, adj_);
            {
                auto t_adj_rebuild = std::chrono::steady_clock::now();
                adj_ = compute_adjacency(boxes_, config_.adjacency_tol, 0,
                                         config_.adjacency_gap_tol);
                adjacency_ms_accum += std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - t_adj_rebuild).count();
            }

            auto islands_after = find_islands(adj_);
            SBF_INFO("[PLN] rescue bridge: +%d boxes (%.0fms) islands=%d", rescued, rb_ms, (int)islands_after.size());
        }
    }

    {
        // adj_ is already up-to-date here:
        // - no seed bridge: adj_ is the pre-seed adjacency
        // - with seed bridge: adj_ is rebuilt after each successful bridge
        // So a final full rebuild would be redundant.
        last_build_timing_.adjacency_final_ms = 0.0;
        auto islands = find_islands(adj_);
        int edges = 0;
        for (auto& kv : adj_) edges += (int)kv.second.size();
        SBF_INFO("[PLN] coverage: boxes=%d islands=%d edges=%d", (int)boxes_.size(), (int)islands.size(), edges/2);

        // Per-island sizes (sorted descending)
        std::vector<int> island_sizes;
        for (auto& isl : islands) island_sizes.push_back((int)isl.size());
        std::sort(island_sizes.rbegin(), island_sizes.rend());
        SBF_INFO("[PLN] island sizes:");
        for (int s : island_sizes) SBF_INFO(" %d", s);
        SBF_INFO("");
    }

    // Store obstacles for query-time use (proxy RRT, bridge, collision check)
    stored_obs_.assign(obs, obs + n_obs);

    last_build_time_ms_ = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_build).count();

    // Fill BuildTimingProfile remaining fields
    last_build_timing_.adjacency_ms = adjacency_ms_accum;
    last_build_timing_.boxes_final = (int)boxes_.size();
    last_build_timing_.total_ms = last_build_time_ms_;

    built_ = true;
}

}  // namespace sbf

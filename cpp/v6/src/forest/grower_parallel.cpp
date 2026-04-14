// SafeBoxForest v6 — Forest Grower: parallel / subtree
#include <sbf/forest/grower.h>
#include <sbf/forest/adjacency.h>
#include <sbf/forest/thread_pool.h>
#include <sbf/core/union_find.h>
#include <sbf/scene/collision_checker.h>

#include <algorithm>
#include <chrono>
#include <future>
#include <numeric>
#include <unordered_map>
#include <sbf/core/log.h>

namespace sbf {

GrowerResult ForestGrower::grow_subtree(const Eigen::VectorXd& root_seed,
                                        int root_id,
                                        const Obstacle* obs, int n_obs,
                                        std::shared_ptr<std::atomic<int>> shared_counter) {
    shared_box_count_ = std::move(shared_counter);
    boxes_.clear();
    next_box_id_ = 0;
    n_ffb_success_ = 0;
    n_ffb_fail_ = 0;
    ffb_total_ms_ = 0.0;
    ffb_envelope_ms_ = 0.0;
    ffb_collide_ms_ = 0.0;
    ffb_expand_ms_ = 0.0;
    ffb_intervals_ms_ = 0.0;
    ffb_cache_hits_ = 0;
    ffb_cache_misses_ = 0;
    ffb_collide_calls_ = 0;
    ffb_expand_calls_ = 0;
    ffb_total_steps_ = 0;
    ffb_total_calls_ = 0;

    // Create root box for this subtree
    FFBConfig saved_ffb = config_.ffb_config;

    int bid = try_create_box(root_seed, obs, n_obs, -1, -1, -1, root_id);
    config_.ffb_config = saved_ffb;

    if (bid < 0) {
        GrowerResult result;
        return result;
    }

    // Grow
    if (config_.mode == GrowerConfig::Mode::WAVEFRONT)
        grow_wavefront(obs, n_obs);
    else
        grow_rrt(obs, n_obs);

    // Promote
    int n_promotions = 0;
    if (config_.enable_promotion && !deadline_reached())
        n_promotions = promote_all(obs, n_obs);

    // Assemble result
    GrowerResult result;
    result.boxes = boxes_;
    result.n_roots = 1;
    result.n_ffb_success = n_ffb_success_;
    result.n_ffb_fail = n_ffb_fail_;
    result.n_promotions = n_promotions;
    for (const auto& b : boxes_)
        result.total_volume += b.volume;
    result.ffb_total_calls = ffb_total_calls_;
    result.ffb_total_ms = ffb_total_ms_;
    result.ffb_envelope_ms = ffb_envelope_ms_;
    result.ffb_collide_ms = ffb_collide_ms_;
    result.ffb_expand_ms = ffb_expand_ms_;
    result.ffb_intervals_ms = ffb_intervals_ms_;
    result.ffb_cache_hits = ffb_cache_hits_;
    result.ffb_cache_misses = ffb_cache_misses_;
    result.ffb_collide_calls = ffb_collide_calls_;
    result.ffb_expand_calls = ffb_expand_calls_;
    result.ffb_total_steps = ffb_total_steps_;
    result.lect_nodes_final = lect_.n_nodes();

    return result;
}

// ─── grow_parallel ──────────────────────────────────────────────────────────
void ForestGrower::grow_parallel(const Obstacle* obs, int n_obs,
                                 GrowerResult& result) {
    // Collect root boxes and their seeds
    struct RootInfo {
        int root_id;
        Eigen::VectorXd seed;
    };
    std::vector<RootInfo> roots;
    for (const auto& b : boxes_) {
        if (b.parent_box_id == -1)
            roots.push_back({b.root_id, b.seed_config});
    }

    int n_subtrees = static_cast<int>(roots.size());
    int n_workers = std::min(config_.n_threads, n_subtrees);

    if (n_workers <= 1) {
        // Fall back to serial
        if (config_.mode == GrowerConfig::Mode::WAVEFRONT)
            grow_wavefront(obs, n_obs);
        else
            grow_rrt(obs, n_obs);
        if (config_.enable_promotion && !deadline_reached())
            result.n_promotions = promote_all(obs, n_obs);
        return;
    }

    int n_root_boxes = static_cast<int>(boxes_.size());
    auto shared_counter = std::make_shared<std::atomic<int>>(n_root_boxes);
    int snapshot_base = lect_.n_nodes();

    const Robot* robot_ptr = &robot_;
    const auto worker_deadline = deadline_;

    // Submit workers to thread pool
    ThreadPool pool(n_workers);
    std::vector<std::future<ParallelWorkerResult>> futures;

    // For multi-goal: each tree gets an independent budget, no shared counter.
    // This eliminates the Matthew effect — small trees are no longer starved.
    bool per_tree_mode = has_multi_goals_;
    auto worker_counter = per_tree_mode
        ? std::shared_ptr<std::atomic<int>>(nullptr)
        : shared_counter;

    for (int i = 0; i < n_subtrees; ++i) {
        GrowerConfig worker_cfg = config_;
        if (per_tree_mode) {
            // Equal budget per tree
            worker_cfg.max_boxes = std::max(config_.max_boxes / n_subtrees, 50);
            // Trees in tight C-space regions need more miss tolerance
            worker_cfg.max_consecutive_miss =
                std::max(config_.max_consecutive_miss, 500);
        } else {
            worker_cfg.max_boxes = config_.max_boxes;
        }
        worker_cfg.rng_seed = config_.rng_seed +
                              static_cast<uint64_t>(i) * 12345ULL + 1;
        worker_cfg.n_threads = 1;  // no recursive parallelism

        auto warm_ptr = std::make_shared<LECT>(lect_.snapshot());
        Eigen::VectorXd seed = roots[i].seed;
        int rid = roots[i].root_id;
        bool has_ep = has_endpoints_;
        Eigen::VectorXd start_cfg = has_ep ? start_ : Eigen::VectorXd();
        Eigen::VectorXd goal_cfg = has_ep ? goal_ : Eigen::VectorXd();
        // Pass multi_goals to workers so RRT goal_bias drives toward other trees
        bool has_mg = has_multi_goals_;
        std::vector<Eigen::VectorXd> worker_multi_goals = multi_goals_;

        futures.push_back(pool.submit(
            [robot_ptr, worker_cfg, has_ep, start_cfg, goal_cfg,
             has_mg, worker_multi_goals,
             seed, rid, obs, n_obs, worker_counter, warm_ptr,
             worker_deadline]() -> ParallelWorkerResult {
                ForestGrower worker(*robot_ptr, std::move(*warm_ptr), worker_cfg);
                worker.set_deadline(worker_deadline);
                if (has_ep) worker.set_endpoints(start_cfg, goal_cfg);
                if (has_mg) worker.set_multi_goals(worker_multi_goals);
                ParallelWorkerResult pwr;
                pwr.result = worker.grow_subtree(seed, rid, obs, n_obs,
                                                 worker_counter);
                pwr.lect = std::move(worker.take_lect());
                return pwr;
            }
        ));
    }

    // Clear master boxes (we'll rebuild from worker results)
    boxes_.clear();
    next_box_id_ = 0;
    n_ffb_success_ = 0;
    n_ffb_fail_ = 0;
    ffb_total_ms_ = 0.0;
    ffb_envelope_ms_ = 0.0;
    ffb_collide_ms_ = 0.0;
    ffb_expand_ms_ = 0.0;
    ffb_intervals_ms_ = 0.0;
    ffb_cache_hits_ = 0;
    ffb_cache_misses_ = 0;
    ffb_collide_calls_ = 0;
    ffb_expand_calls_ = 0;
    ffb_total_steps_ = 0;
    ffb_total_calls_ = 0;
    int total_promotions = 0;
    int total_transplanted = 0;

    // Collect results and merge
    for (int fi = 0; fi < static_cast<int>(futures.size()); ++fi) {
        ParallelWorkerResult pwr = futures[fi].get();
        GrowerResult& wr = pwr.result;

        // Accumulate stats
        total_promotions += wr.n_promotions;
        n_ffb_success_ += wr.n_ffb_success;
        n_ffb_fail_ += wr.n_ffb_fail;
        ffb_total_ms_ += wr.ffb_total_ms;
        ffb_envelope_ms_ += wr.ffb_envelope_ms;
        ffb_collide_ms_ += wr.ffb_collide_ms;
        ffb_expand_ms_ += wr.ffb_expand_ms;
        ffb_intervals_ms_ += wr.ffb_intervals_ms;
        ffb_cache_hits_ += wr.ffb_cache_hits;
        ffb_cache_misses_ += wr.ffb_cache_misses;
        ffb_collide_calls_ += wr.ffb_collide_calls;
        ffb_expand_calls_ += wr.ffb_expand_calls;
        ffb_total_steps_ += wr.ffb_total_steps;
        ffb_total_calls_ += wr.ffb_total_calls;

        // Remap box IDs to global unique IDs
        std::unordered_map<int, int> id_map;
        for (auto& box : wr.boxes) {
            int new_id = next_box_id_++;
            id_map[box.id] = new_id;
            box.id = new_id;
        }
        for (auto& box : wr.boxes) {
            if (box.parent_box_id >= 0) {
                auto mit = id_map.find(box.parent_box_id);
                box.parent_box_id = (mit != id_map.end()) ? mit->second : -1;
            }
        }

        // Transplant LECT nodes expanded by this worker
        int n_tp = lect_.transplant_subtree(pwr.lect, snapshot_base, id_map);
        total_transplanted += n_tp;

        // Merge expand profiling
        lect_.expand_profile_.merge(pwr.lect.expand_profile_);

        // Merge boxes
        for (auto& box : wr.boxes)
            boxes_.push_back(std::move(box));
    }

    result.n_promotions = total_promotions;

    SBF_INFO("[GRW] parallel done: %d boxes merged, %d promotions, " "%d nodes transplanted, %d threads", static_cast<int>(boxes_.size()), total_promotions, total_transplanted, n_workers);

    // Log per-tree box counts (useful for diagnosing balance)
    {
        std::unordered_map<int, int> tree_sizes;
        for (const auto& b : boxes_) tree_sizes[b.root_id]++;
        SBF_INFO("[GRW] tree sizes:");
        for (const auto& kv : tree_sizes)
            SBF_INFO(" root%d=%d", kv.first, kv.second);
        SBF_INFO("");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// grow_coordinated — Master-worker architecture
//
// Master thread:  RRT sampling loop, nearest-box search, box acceptance,
//                 adjacency graph maintenance, connectivity tracking.
// Worker threads: FFB-only computation, each with its own LECT snapshot.
//
// Key guarantee: NO duplicate or overlapping boxes across trees because
// the master is the single point of truth for box list management.
// ═══════════════════════════════════════════════════════════════════════════


}  // namespace sbf

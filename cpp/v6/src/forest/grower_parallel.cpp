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
#include <unordered_set>
#include <sbf/core/log.h>

namespace sbf {

GrowerResult ForestGrower::grow_subtree(const Eigen::VectorXd& root_seed,
                                        int root_id,
                                        const Obstacle* obs, int n_obs,
                                        std::shared_ptr<std::atomic<int>> shared_counter,
                                        bool skip_promotion) {
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

    // Promote (skipped when invoked from grow_parallel — master promotes
    // post-merge so union-based envelope updates land in master LECT).
    int n_promotions = 0;
    if (!skip_promotion && config_.enable_promotion && !deadline_reached())
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

    // ── Geometric domain partitioning ──────────────────────────────────
    // Pre-split master LECT until each root seed lives in a distinct leaf.
    // Each leaf becomes a worker's exclusive subdomain. Workers cannot
    // create boxes outside their domain (try_create_box rejects them) so
    // there is no shared write or index race between workers.
    std::vector<Eigen::VectorXd> seed_vec;
    seed_vec.reserve(n_subtrees);
    for (const auto& r : roots) seed_vec.push_back(r.seed);
    std::vector<int> domain_for_worker = lect_.partition_for_seeds(seed_vec);
    SBF_INFO("[GRW] parallel partition: n_subtrees=%d, master n_nodes=%d (post-split)", n_subtrees, lect_.n_nodes());

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
        int worker_domain = domain_for_worker[i];
        bool has_ep = has_endpoints_;
        Eigen::VectorXd start_cfg = has_ep ? start_ : Eigen::VectorXd();
        Eigen::VectorXd goal_cfg = has_ep ? goal_ : Eigen::VectorXd();
        // Pass multi_goals to workers so RRT goal_bias drives toward other trees
        bool has_mg = has_multi_goals_;
        std::vector<Eigen::VectorXd> worker_multi_goals = multi_goals_;

        futures.push_back(pool.submit(
            [robot_ptr, worker_cfg, has_ep, start_cfg, goal_cfg,
             has_mg, worker_multi_goals, worker_domain,
             seed, rid, obs, n_obs, worker_counter, warm_ptr,
             worker_deadline]() -> ParallelWorkerResult {
                ForestGrower worker(*robot_ptr, std::move(*warm_ptr), worker_cfg);
                worker.set_deadline(worker_deadline);
                if (has_ep) worker.set_endpoints(start_cfg, goal_cfg);
                if (has_mg) worker.set_multi_goals(worker_multi_goals);
                // Geometric domain: worker only writes within this subtree.
                worker.set_domain_root(worker_domain);
                ParallelWorkerResult pwr;
                // Workers always run promotion locally — geometric isolation
                // guarantees writes stay within subtree(worker_domain), and
                // transplant_domain ships the entire subtree back so all
                // worker promotions land in the master LECT cache.
                pwr.result = worker.grow_subtree(
                    seed, rid, obs, n_obs, worker_counter,
                    /*skip_promotion=*/false);
                pwr.lect = std::move(worker.take_lect());
                pwr.domain_root = worker_domain;
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
    std::vector<int> domain_roots;  // master-side R_i indices (for cross-domain promotion)

    // Collect results and merge
    for (int fi = 0; fi < static_cast<int>(futures.size()); ++fi) {
        ParallelWorkerResult pwr = futures[fi].get();
        GrowerResult& wr = pwr.result;

        // Accumulate stats
        total_promotions += wr.n_promotions;
        SBF_INFO("[GRW] worker %d: %d boxes, %d promotions", fi, static_cast<int>(wr.boxes.size()), wr.n_promotions);
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

        // Geometric-domain transplant: ship the worker's entire subtree
        // rooted at its assigned domain leaf, remapping worker-allocated
        // descendant indices to fresh master indices. Box.tree_id values
        // get remapped accordingly so they continue to point at the right
        // master LECT node.
        std::unordered_map<int, int> node_remap;
        int n_tp = lect_.transplant_domain(pwr.lect, pwr.domain_root,
                                           id_map, node_remap);
        total_transplanted += n_tp;
        for (auto& box : wr.boxes) {
            auto it = node_remap.find(box.tree_id);
            if (it != node_remap.end()) box.tree_id = it->second;
        }

        // Track domain_root for cross-domain promotion (master node index is stable).
        if (pwr.domain_root >= 0)
            domain_roots.push_back(pwr.domain_root);

        // Merge expand profiling
        lect_.expand_profile_.merge(pwr.lect.expand_profile_);

        // Merge boxes
        for (auto& box : wr.boxes)
            boxes_.push_back(std::move(box));
    }

    result.n_promotions = total_promotions;

    // ── Master-side promotion (union-based) ───────────────────────────
    //   After all workers' boxes/LECT-nodes are merged into the master,
    //   replay occupation on master.lect_ and run promote_all once.
    //   try_promote_envelope_union writes the merged envelope back into
    //   master's own LECT cache (which subsequent queries read), and the
    //   recursive bubble-up may produce coarser boxes than any single
    //   worker could (cross-tree sibling pairs become reachable).
    if (config_.enable_promotion && !deadline_reached()) {
        auto t_promo_start = std::chrono::steady_clock::now();

        // A5: wipe stale occupation (workers wrote into snapshots; transplant
        //     copied per-worker subtree_occ_ but with index collisions across
        //     workers the master state may be inconsistent). Rebuild from
        //     boxes_ as the single source of truth.
        lect_.clear_all_occupation();

        // A6: validate b.tree_id, deduplicate against collisions. If two
        //     boxes claim the same tree_id (worker index collision survived
        //     transplant), only the first is marked; the rest are logged
        //     and skipped from promotion (they are still planning-valid as
        //     boxes; only the LECT-cache reuse for them is forfeited).
        const int n_master_nodes = lect_.n_nodes();
        int n_marked = 0;
        int n_skipped_oob = 0;
        int n_skipped_collide = 0;
        for (const auto& b : boxes_) {
            if (b.tree_id < 0 || b.tree_id >= n_master_nodes) {
                ++n_skipped_oob;
                continue;
            }
            if (lect_.is_occupied(b.tree_id)) {
                ++n_skipped_collide;
                continue;
            }
            lect_.mark_occupied(b.tree_id, b.id);
            ++n_marked;
        }
        if (n_skipped_oob > 0 || n_skipped_collide > 0) {
            SBF_WARN("[GRW] master mark: %d ok, %d oob, %d collide (of %d boxes)",
                     n_marked, n_skipped_oob, n_skipped_collide,
                     static_cast<int>(boxes_.size()));
        }

        // A7: build cross-domain boundary candidate nodes.
        // Only ancestors of each R_i (domain_root) can become newly promote-able
        // after the cross-worker merge — intra-domain promotions were already
        // done by each worker. Walk parent chains of all R_i up to the root,
        // deduplicate, and pass as start_nodes to promote_all for an O(n_workers×
        // depth) scan instead of O(n_nodes).
        std::vector<int> boundary_candidates;
        {
            std::unordered_set<int> seen;
            for (int r : domain_roots) {
                int cur = lect_.parent(r);
                while (cur >= 0 && seen.insert(cur).second) {
                    boundary_candidates.push_back(cur);
                    cur = lect_.parent(cur);
                }
            }
        }

        int master_promo = 0;
        if (!deadline_reached())
            master_promo = promote_all(obs, n_obs, boundary_candidates);
        total_promotions += master_promo;
        result.n_promotions = total_promotions;

        const double promo_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_promo_start).count();
        SBF_INFO("[GRW] master promote (post-merge): %d merges in %.2f ms (marked=%d)",
                 master_promo, promo_ms, n_marked);
    }

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

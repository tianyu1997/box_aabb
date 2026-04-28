// SafeBoxForest v6 — Forest Grower: parallel / subtree
#include <sbf/forest/grower.h>
#include <sbf/forest/adjacency.h>
#include <sbf/forest/thread_pool.h>
#include <sbf/core/union_find.h>
#include <sbf/scene/collision_checker.h>

#include <algorithm>
#include <chrono>
#include <future>
#include <limits>
#include <mutex>
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

GrowerResult ForestGrower::grow_existing_subtree(
        const std::vector<BoxNode>& initial_boxes,
        const Obstacle* obs, int n_obs,
        std::shared_ptr<std::atomic<int>> shared_counter,
        bool skip_promotion) {
    shared_box_count_ = std::move(shared_counter);
    boxes_ = initial_boxes;
    next_box_id_ = 0;
    for (const auto& b : boxes_)
        next_box_id_ = std::max(next_box_id_, b.id + 1);
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

    if (config_.mode == GrowerConfig::Mode::WAVEFRONT)
        grow_wavefront(obs, n_obs);
    else
        grow_rrt(obs, n_obs);

    int n_promotions = 0;
    if (!skip_promotion && config_.enable_promotion && !deadline_reached())
        n_promotions = promote_all(obs, n_obs);

    GrowerResult result;
    result.boxes = boxes_;
    result.n_roots = 0;
    for (const auto& b : initial_boxes)
        if (b.parent_box_id == -1) result.n_roots++;
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

// ─── grow_partitioned_shared ────────────────────────────────────────────────
void ForestGrower::grow_partitioned_shared(const Obstacle* obs, int n_obs,
                                           GrowerResult& result) {
    struct RootInfo {
        int root_id;
        Eigen::VectorXd seed;
        BoxNode box;
    };

    auto collect_roots = [&]() {
        std::vector<RootInfo> out;
        for (const auto& b : boxes_) {
            if (b.parent_box_id == -1)
                out.push_back({b.root_id, b.seed_config, b});
        }
        std::sort(out.begin(), out.end(),
                  [](const RootInfo& a, const RootInfo& b) {
                      return a.root_id < b.root_id;
                  });
        return out;
    };

    std::vector<RootInfo> roots = collect_roots();
    if (roots.empty()) return;

    auto contains_seed = [](const std::vector<Interval>& ivs,
                            const Eigen::VectorXd& q) {
        for (int d = 0; d < static_cast<int>(ivs.size()); ++d) {
            if (!ivs[d].contains(q[d], 1e-10)) return false;
        }
        return true;
    };

    auto domain_volume = [&](int node) {
        const auto ivs = lect_.node_intervals(node);
        double v = 1.0;
        for (const auto& iv : ivs) v *= std::max(0.0, iv.width());
        return v;
    };

    auto build_cover_domains = [&](int target) {
        std::vector<int> domains{0};
        while (static_cast<int>(domains.size()) < target && !deadline_reached()) {
            int best_pos = -1;
            double best_v = -1.0;
            for (int i = 0; i < static_cast<int>(domains.size()); ++i) {
                int node = domains[i];
                double v = domain_volume(node);
                if (v > best_v) {
                    best_v = v;
                    best_pos = i;
                }
            }
            if (best_pos < 0) break;
            int node = domains[best_pos];
            if (lect_.is_leaf(node)) {
                int before = lect_.n_nodes();
                lect_.expand_leaf(node);
                if (lect_.n_nodes() == before) break;
            }
            int l = lect_.left(node);
            int r = lect_.right(node);
            if (l < 0 || r < 0) break;
            domains[best_pos] = l;
            domains.push_back(r);
        }
        std::sort(domains.begin(), domains.end());
        return domains;
    };

    std::vector<int> domain_for_worker = build_cover_domains(config_.n_threads);
    const int n_workers = std::min(config_.n_threads, static_cast<int>(domain_for_worker.size()));
    if (n_workers <= 1) {
        if (config_.mode == GrowerConfig::Mode::WAVEFRONT)
            grow_wavefront(obs, n_obs);
        else
            grow_rrt(obs, n_obs);
        return;
    }

    auto assign_roots_to_domains = [&]() {
        std::vector<std::vector<RootInfo>> by_domain(domain_for_worker.size());
        std::vector<std::vector<Interval>> domain_ivs;
        domain_ivs.reserve(domain_for_worker.size());
        for (int d : domain_for_worker)
            domain_ivs.push_back(lect_.node_intervals(d));
        for (const auto& r : roots) {
            for (int i = 0; i < static_cast<int>(domain_ivs.size()); ++i) {
                if (contains_seed(domain_ivs[i], r.seed)) {
                    by_domain[i].push_back(r);
                    break;
                }
            }
        }
        return by_domain;
    };

    std::vector<std::vector<RootInfo>> roots_by_domain = assign_roots_to_domains();

    // Use all requested threads by synthesizing one ordinary free root inside
    // each empty cover domain. This preserves a full, gap-free LECT partition;
    // it is not a bridge stage.
    int next_root_id = 0;
    for (const auto& r : roots)
        next_root_id = std::max(next_root_id, r.root_id + 1);
    for (int i = 0; i < n_workers && !deadline_reached(); ++i) {
        if (!roots_by_domain[i].empty()) continue;
        int saved_domain = domain_root_;
        domain_root_ = domain_for_worker[i];
        int bid = -1;
        for (int attempt = 0; attempt < 16 && bid < 0; ++attempt) {
            Eigen::VectorXd q = (attempt == 0)
                ? [&]() {
                      const auto ivs = lect_.node_intervals(domain_for_worker[i]);
                      Eigen::VectorXd c(static_cast<int>(ivs.size()));
                      for (int d = 0; d < static_cast<int>(ivs.size()); ++d)
                          c[d] = ivs[d].center();
                      return c;
                  }()
                : sample_random();
            bid = try_create_box(q, obs, n_obs, -1, -1, -1, next_root_id);
        }
        domain_root_ = saved_domain;
        if (bid >= 0) next_root_id++;
    }

    roots = collect_roots();
    roots_by_domain = assign_roots_to_domains();

    const int root_count_for_budget = std::max(1, static_cast<int>(roots.size()));
    const int even_max_boxes_per_tree = std::max(1, config_.max_boxes / root_count_for_budget);
    int per_tree_box_budget = config_.partitioned_box_budget_per_tree;
    if (per_tree_box_budget <= 0) {
        if (config_.post_connect_extra_boxes > 0) {
            per_tree_box_budget = std::max(
                1,
                (config_.post_connect_extra_boxes + root_count_for_budget - 1) /
                    root_count_for_budget);
        } else {
            per_tree_box_budget = even_max_boxes_per_tree;
        }
    }
    per_tree_box_budget = std::clamp(per_tree_box_budget, 1, even_max_boxes_per_tree);

    int total_partition_box_budget = 0;
    for (const auto& domain_roots : roots_by_domain) {
        const int n_roots_here = std::max(1, static_cast<int>(domain_roots.size()));
        total_partition_box_budget += n_roots_here * per_tree_box_budget;
    }

    const int reserve_nodes = lect_.n_nodes() +
        std::max(4096, 2 * total_partition_box_budget +
                 4 * config_.max_consecutive_miss * n_workers);
    lect_.prepare_parallel_writes(reserve_nodes);
    SBF_INFO("[GRW] partitioned shared-LECT cover: roots=%d workers=%d per_tree_budget=%d total_budget=%d reserve_nodes=%d n_nodes=%d",
             static_cast<int>(roots.size()), n_workers, per_tree_box_budget,
             total_partition_box_budget, reserve_nodes, lect_.n_nodes());

    int max_box_id = -1;
    for (const auto& b : boxes_) max_box_id = std::max(max_box_id, b.id);
    auto next_id = std::make_shared<std::atomic<int>>(max_box_id + 1);
    auto stop_requested = std::make_shared<std::atomic<bool>>(false);

    std::unordered_map<int, int> root_to_uf;
    root_to_uf.reserve(roots.size());
    for (int i = 0; i < static_cast<int>(roots.size()); ++i)
        root_to_uf[roots[i].root_id] = i;
    UnionFind tree_uf(static_cast<int>(roots.size()));
    int n_components = static_cast<int>(roots.size());
    bool first_connected = false;
    auto t0 = Clock::now();
    std::mutex collect_mutex;

    auto try_union_roots = [&](const BoxNode& a, const BoxNode& b) {
        auto ia = root_to_uf.find(a.root_id);
        auto ib = root_to_uf.find(b.root_id);
        if (ia == root_to_uf.end() || ib == root_to_uf.end()) return;
        if (ia->second == ib->second) return;
        if (tree_uf.unite(ia->second, ib->second))
            n_components--;
    };

    for (int i = 0; i < static_cast<int>(boxes_.size()); ++i) {
        for (int j = i + 1; j < static_cast<int>(boxes_.size()); ++j) {
            if (boxes_[i].root_id != boxes_[j].root_id &&
                boxes_adjacent(boxes_[i], boxes_[j])) {
                try_union_roots(boxes_[i], boxes_[j]);
            }
        }
    }

    auto record_connect_if_ready = [&]() {
        if (!first_connected && n_components <= 1) {
            first_connected = true;
            wf_all_connected_ = true;
            wf_connect_boxes_ = static_cast<int>(boxes_.size());
            wf_connect_time_ms_ = std::chrono::duration<double, std::milli>(
                Clock::now() - t0).count();
            SBF_INFO("[GRW] partitioned shared-LECT connected: boxes=%d t=%.0fms",
                     wf_connect_boxes_, wf_connect_time_ms_);
        }
        if (first_connected) {
            const int extra = static_cast<int>(boxes_.size()) - wf_connect_boxes_;
            if (config_.stop_after_connect ||
                (config_.post_connect_extra_boxes > 0 &&
                 extra >= config_.post_connect_extra_boxes)) {
                stop_requested->store(true, std::memory_order_relaxed);
            }
        }
    };
    record_connect_if_ready();

    auto master_callback = [&](const BoxNode& box) {
        std::lock_guard<std::mutex> lock(collect_mutex);
        const int old_n = static_cast<int>(boxes_.size());
        boxes_.push_back(box);
        const BoxNode& added = boxes_.back();
        for (int j = 0; j < old_n; ++j) {
            if (added.root_id != boxes_[j].root_id &&
                boxes_adjacent(added, boxes_[j])) {
                try_union_roots(added, boxes_[j]);
            }
        }
        record_connect_if_ready();
    };

    ThreadPool pool(n_workers);
    std::vector<std::future<GrowerResult>> futures;
    const Robot* robot_ptr = &robot_;
    LECT* lect_ptr = &lect_;
    const auto worker_deadline = deadline_;
    bool has_ep = has_endpoints_;
    Eigen::VectorXd start_cfg = has_ep ? start_ : Eigen::VectorXd();
    Eigen::VectorXd goal_cfg = has_ep ? goal_ : Eigen::VectorXd();
    bool has_mg = has_multi_goals_;
    std::vector<Eigen::VectorXd> worker_multi_goals = multi_goals_;

    for (int i = 0; i < n_workers; ++i) {
        GrowerConfig worker_cfg = config_;
        worker_cfg.n_threads = 1;
        worker_cfg.enable_promotion = false;
        worker_cfg.enable_partitioned_lect_parallel = true;
        worker_cfg.max_consecutive_miss = std::max(config_.max_consecutive_miss, 1);
        worker_cfg.rng_seed = config_.rng_seed + static_cast<uint64_t>(i) * 12345ULL + 1;
        std::vector<BoxNode> initial_boxes;
        initial_boxes.reserve(roots_by_domain[i].size());
        for (const auto& r : roots_by_domain[i])
            initial_boxes.push_back(r.box);
        const int n_roots_here = std::max(1, static_cast<int>(initial_boxes.size()));
        worker_cfg.max_boxes = std::max(
            static_cast<int>(initial_boxes.size()),
            n_roots_here * per_tree_box_budget);
        int worker_domain = domain_for_worker[i];

        futures.push_back(pool.submit(
            [robot_ptr, lect_ptr, worker_cfg, has_ep, start_cfg, goal_cfg,
             has_mg, worker_multi_goals, worker_domain, initial_boxes,
             obs, n_obs, next_id, stop_requested, master_callback,
             worker_deadline, i]() mutable -> GrowerResult {
                ForestGrower worker(*robot_ptr, *lect_ptr, worker_cfg);
                worker.set_deadline(worker_deadline);
                worker.set_worker_tid(i);
                worker.set_domain_root(worker_domain);
                worker.set_shared_next_box_id(next_id);
                worker.set_stop_flag(stop_requested);
                worker.set_box_callback(master_callback);
                if (has_ep) worker.set_endpoints(start_cfg, goal_cfg);
                if (has_mg) worker.set_multi_goals(worker_multi_goals);
                return worker.grow_existing_subtree(initial_boxes, obs, n_obs,
                                                    nullptr,
                                                    /*skip_promotion=*/true);
            }
        ));
    }

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

    for (int i = 0; i < static_cast<int>(futures.size()); ++i) {
        GrowerResult wr = futures[i].get();
        SBF_INFO("[GRW] partition worker %d: local_boxes=%d ffb_ok=%d ffb_fail=%d",
                 i, static_cast<int>(wr.boxes.size()), wr.n_ffb_success, wr.n_ffb_fail);
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
    }

    next_box_id_ = next_id->load(std::memory_order_relaxed);
    result.n_promotions = 0;
    result.tree_all_connected = (n_components <= 1);
    result.tree_connect_time_ms = wf_connect_time_ms_;
    result.tree_connect_n_boxes = wf_connect_boxes_;
    SBF_INFO("[GRW] partitioned shared-LECT done: boxes=%d components=%d nodes=%d",
             static_cast<int>(boxes_.size()), n_components, lect_.n_nodes());
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

    // ── Phase C-2: secondary domain split ─────────────────────────────
    // When n_threads > n_subtrees AND we are in endpoint mode (no
    // multi-goal), synthesize extra "domain seeds" by sampling free
    // configurations far from existing roots. Each extra seed gives one
    // additional worker its own exclusive LECT subtree (via
    // partition_for_seeds) with a fresh root_id. Bridge pass at the end
    // of grow() will splice these orphan trees into the start/goal
    // component if needed.
    if (!has_multi_goals_ && n_subtrees > 0 &&
        n_subtrees < config_.n_threads) {
        const int extra = config_.n_threads - n_subtrees;
        constexpr int K_CANDIDATES = 16;
        const int next_id_base = n_subtrees;
        for (int e = 0; e < extra; ++e) {
            // Farthest-point sampling among K random candidates.
            Eigen::VectorXd best_q;
            double best_score = -1.0;
            for (int k = 0; k < K_CANDIDATES; ++k) {
                Eigen::VectorXd q = sample_random();
                double min_d2 = std::numeric_limits<double>::max();
                for (const auto& r : roots)
                    min_d2 = std::min(min_d2, (q - r.seed).squaredNorm());
                if (min_d2 > best_score) { best_score = min_d2; best_q = q; }
            }
            roots.push_back({next_id_base + e, best_q});
        }
        SBF_INFO("[GRW] parallel C-2: synthesized %d extra domain seeds (n_threads=%d > n_subtrees=%d)",
                 extra, config_.n_threads, n_subtrees);
        n_subtrees = static_cast<int>(roots.size());
    }

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
             seed, rid, obs, n_obs, worker_counter, warm_ptr, i,
             worker_deadline]() -> ParallelWorkerResult {
                ForestGrower worker(*robot_ptr, std::move(*warm_ptr), worker_cfg);
                worker.set_deadline(worker_deadline);
                            worker.set_worker_tid(i);
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

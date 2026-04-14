// SafeBoxForest v6 — Forest Grower: coordinated growth
#include <sbf/forest/grower.h>
#include <sbf/forest/adjacency.h>
#include <sbf/forest/thread_pool.h>
#include <sbf/core/union_find.h>
#include <sbf/scene/collision_checker.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <queue>
#include <unordered_map>
#include <sbf/core/log.h>

namespace sbf {

void ForestGrower::grow_coordinated(const Obstacle* obs, int n_obs) {
    const int n_workers = std::min(config_.n_threads,
                                   std::max(1, (int)std::thread::hardware_concurrency()));
    const int nd = robot_.n_joints();
    const auto& limits = robot_.joint_limits().limits;

    // ── Per-worker LECT snapshots for thread-safe FFB ───────────────────
    std::vector<std::unique_ptr<LECT>> worker_lects;
    worker_lects.reserve(n_workers);
    for (int i = 0; i < n_workers; ++i)
        worker_lects.push_back(std::make_unique<LECT>(lect_.snapshot()));

    ThreadPool pool(n_workers);

    // LECT refresh interval (every N batches, re-snapshot from master)
    const int lect_refresh_interval = 10;

    // ── Flat center cache for fast nearest-box search ───────────────────
    std::vector<double> center_cache;
    center_cache.reserve(config_.max_boxes * nd);
    for (const auto& b : boxes_)
        for (int d = 0; d < nd; ++d)
            center_cache.push_back(b.joint_intervals[d].center());

    // ── Connectivity tracking ───────────────────────────────────────────
    const bool cm = config_.connect_mode && has_multi_goals_;
    const int n_trees = has_multi_goals_ ? (int)multi_goals_.size() : 0;
    UnionFind tree_uf(std::max(n_trees, 1));
    int n_comp = n_trees;

    // Per-tree box count and index lists for fast tree-based lookups
    std::vector<int> tree_box_count(std::max(n_trees, 1), 0);
    std::vector<std::vector<int>> tree_box_indices(std::max(n_trees, 1));
    for (int i = 0; i < (int)boxes_.size(); ++i) {
        if (boxes_[i].root_id >= 0 && boxes_[i].root_id < n_trees) {
            tree_box_count[boxes_[i].root_id]++;
            tree_box_indices[boxes_[i].root_id].push_back(i);
        }
    }

    std::uniform_real_distribution<double> u01(0.0, 1.0);
    std::vector<double> q_buf(nd);

    int miss_count = 0;
    int total_batches = 0;

    // ── Task / result types ─────────────────────────────────────────────
    struct TaskInfo {
        Eigen::VectorXd seed;
        int parent_box_id;
        int face_dim, face_side;
        int root_id;
    };
    struct WorkerResult {
        bool success = false;
        BoxNode box;
        FFBResult ffb;
    };

    // Progress logging interval
    auto t_start = Clock::now();
    int last_log_boxes = (int)boxes_.size();

    // Stall detection: track when component count last changed
    int last_comp_change_boxes = (int)boxes_.size();
    int stall_threshold = 150;  // P19: lowered from 500 for faster bridge activation

    // Connectivity time tracking: record FIRST time n_comp reaches 1
    double first_connect_time_ms = -1.0;
    int first_connect_boxes = 0;

    // ── P12: Pre-compute & cache component sizes (update on component change) ──
    std::vector<int> comp_size(std::max(n_trees, 1), 0);
    if (n_trees > 0)
        for (int t = 0; t < n_trees; ++t)
            comp_size[tree_uf.find(t)]++;

    // ── P10: Adaptive step_ratio ──
    const double base_step_ratio = config_.rrt_step_ratio;

    // ── P17: Inline promotion tracking ──
    int n_inline_promotions = 0;
    int boxes_at_last_promote = 0;  // box count when last promotion ran
    const int promote_interval = 500;  // P23: lowered from 1000 for more frequent promotion

    // ── Timing accumulators for profiling ──
    double t_task_gen_ms = 0, t_ffb_wall_ms = 0, t_post_accept_ms = 0;
    double t_cross_tree_ms = 0, t_prefilter_ms = 0, t_nn_ms = 0;
    double t_promote_ms = 0;
    int n_prefilter_rejects = 0, n_postfilter_rejects = 0, n_isolated_rejects = 0;

    while ((int)boxes_.size() < config_.max_boxes &&
           !deadline_reached()) {

        // ── Termination logic ──
        // Before connectivity: keep growing (miss_count * 4 safety cap)
        // After connectivity:
        //   - stop_after_connect: break immediately
        //   - post_connect_extra_boxes > 0: grow N more boxes then stop
        //   - otherwise: use reduced miss threshold for natural termination
        bool connected_now = cm && n_comp <= 1;
        if (!connected_now && miss_count >= config_.max_consecutive_miss * 4) break;
        if (connected_now && config_.stop_after_connect) break;
        if (connected_now && config_.post_connect_extra_boxes > 0 &&
            (int)boxes_.size() >= first_connect_boxes + config_.post_connect_extra_boxes) break;
        if (connected_now && config_.post_connect_extra_boxes <= 0) {
            int post_connect_miss_limit = std::max(config_.max_consecutive_miss / 10, 100);
            if (miss_count >= post_connect_miss_limit) break;
        }

        // Detect stall (for logging and adaptive params)
        int boxes_since_comp_change = (int)boxes_.size() - last_comp_change_boxes;
        bool stalled = cm && n_comp > 1 &&
            (boxes_since_comp_change > stall_threshold);

        // P10: Adaptive step_ratio — decrease when stalled for finer exploration
        //   stall_level 0..3 → step_ratio = base, base/2, base/4, base/8
        int stall_level = stalled ? std::min(3, (boxes_since_comp_change - stall_threshold) / 3000) : 0;
        double cur_step_ratio = base_step_ratio / (1 << stall_level);

        // ── P15: Post-connectivity exploration mode ─────────────────────
        bool connected_phase = cm && n_comp <= 1;
        // After connectivity, use base step ratio (no coverage expansion)
        // so FFB failures accumulate faster and grow terminates naturally.
        double effective_step = connected_phase
            ? base_step_ratio : cur_step_ratio;

        // ── P20: Time-aware urgency — force bridge when running out of time ──
        double elapsed_frac = config_.timeout_ms > 0
            ? std::chrono::duration<double, std::milli>(Clock::now() - t_start).count() / config_.timeout_ms
            : 0.0;
        bool urgent = cm && n_comp > 1 && elapsed_frac > 0.4;

        // ── Generate a batch of FFB tasks (master, single-threaded) ─────
        auto t_gen0 = Clock::now();
        int effective_batch = (config_.batch_size > 0)
            ? std::min(config_.batch_size, n_workers) : n_workers;
        int batch_cap = std::min(effective_batch,
                                 config_.max_boxes - (int)boxes_.size());
        std::vector<TaskInfo> tasks;
        tasks.reserve(batch_cap);

        // Try up to 3x candidates to fill the batch (some get pre-filtered)
        for (int attempt = 0; attempt < batch_cap * 3 &&
                              (int)tasks.size() < batch_cap; ++attempt) {
            // Connection-driven scheduling: prefer trees in smaller components
            int tree_id = 0;
            if (n_trees > 0) {
                if (u01(rng_) < 0.7) {
                    // P12: Use cached comp_size (no recomputation per attempt)
                    int best_comp = comp_size[tree_uf.find(0)];
                    int best_cnt = tree_box_count[0];
                    for (int t = 1; t < n_trees; ++t) {
                        int cs = comp_size[tree_uf.find(t)];
                        if (cs < best_comp ||
                            (cs == best_comp && tree_box_count[t] < best_cnt)) {
                            best_comp = cs;
                            best_cnt = tree_box_count[t];
                            tree_id = t;
                        }
                    }
                } else {
                    tree_id = std::uniform_int_distribution<int>(
                        0, n_trees - 1)(rng_);
                }
            }

            // P11: Stall-aware sampling strategy
            // Normal: goal_bias toward unconnected trees
            // Stalled: midpoint bridging — sample along interpolation line
            //   between disconnected tree frontiers (much more effective than
            //   random frontier perturbation)
            Eigen::VectorXd q_rand;
            double gb = config_.rrt_goal_bias;

            // P20: urgent mode → 95% bridge; stalled → 90% bridge
            double bridge_prob = urgent ? 0.95 : 0.90;
            bool use_bridge = (stalled || urgent) && cm && n_comp > 1 && u01(rng_) < bridge_prob;
            if (use_bridge) {
                // P14: Midpoint bridging — sample along line between closest
                // boxes of two disconnected components
                std::vector<int> unconnected;
                for (int t = 0; t < n_trees; ++t)
                    if (t != tree_id && !tree_uf.connected(t, tree_id))
                        unconnected.push_back(t);
                if (!unconnected.empty()) {
                    int target_tree = unconnected[std::uniform_int_distribution<int>(
                        0, (int)unconnected.size() - 1)(rng_)];

                    // Find a frontier box from source tree (nearest to target)
                    Eigen::VectorXd target_center = multi_goals_[target_tree];
                    if (!tree_box_indices[target_tree].empty()) {
                        int tbi_rand = tree_box_indices[target_tree][
                            std::uniform_int_distribution<int>(
                                0, (int)tree_box_indices[target_tree].size() - 1)(rng_)];
                        target_center = boxes_[tbi_rand].center();
                    }

                    // Source: pick box from this tree closest to target
                    int src_idx = -1;
                    if (!tree_box_indices[tree_id].empty()) {
                        double bd = std::numeric_limits<double>::max();
                        const auto& stbi = tree_box_indices[tree_id];
                        int scan_n = std::min((int)stbi.size(), 64);
                        for (int si = 0; si < scan_n; ++si) {
                            int bi = (scan_n < (int)stbi.size())
                                ? stbi[std::uniform_int_distribution<int>(0, (int)stbi.size()-1)(rng_)]
                                : stbi[si];
                            double d2 = (boxes_[bi].center() - target_center).squaredNorm();
                            if (d2 < bd) { bd = d2; src_idx = bi; }
                        }
                    }

                    if (src_idx >= 0) {
                        Eigen::VectorXd src_center = boxes_[src_idx].center();
                        // Sample a random point along the line src→target
                        // Bias toward the middle and target end
                        double alpha = 0.3 + u01(rng_) * 0.7;  // [0.3, 1.0]
                        q_rand = src_center + alpha * (target_center - src_center);
                        // Add small random perturbation
                        for (int d = 0; d < nd; ++d)
                            q_rand[d] += std::normal_distribution<double>(0.0, 0.02)(rng_);
                        q_rand = clamp_to_limits(q_rand);
                    } else {
                        q_rand = sample_random();
                    }
                } else {
                    q_rand = sample_random();
                }
            // P21: After connectivity, use pure random for max coverage spread
            } else if (!connected_phase && has_multi_goals_ && u01(rng_) < gb) {
                // Collect trees not yet connected to tree_id
                std::vector<int> unconnected;
                for (int t = 0; t < n_trees; ++t)
                    if (t != tree_id && !tree_uf.connected(t, tree_id))
                        unconnected.push_back(t);
                if (!unconnected.empty()) {
                    int gi = unconnected[std::uniform_int_distribution<int>(
                        0, (int)unconnected.size() - 1)(rng_)];
                    // Sample a random box center from target tree (not just root)
                    const auto& tbi = tree_box_indices[gi];
                    if (!tbi.empty()) {
                        int bi = tbi[std::uniform_int_distribution<int>(
                            0, (int)tbi.size() - 1)(rng_)];
                        q_rand = boxes_[bi].center();
                    } else {
                        q_rand = multi_goals_[gi];
                    }
                } else {
                    q_rand = sample_random();  // all connected, fall back
                }
            } else {
                q_rand = sample_random();
            }

            // P13/P15: Find nearest box
            // Connected phase: search ALL boxes across all trees (better coverage)
            // Pre-connectivity: search within selected tree only
            for (int d = 0; d < nd; ++d) q_buf[d] = q_rand[d];
            double best_dist = std::numeric_limits<double>::max();
            int best_idx = -1;
            {
                const double* base_cc = center_cache.data();
                const double* qp = q_buf.data();
                constexpr int K_SUBSAMPLE = 64;
                constexpr int K_SUB_ALL = 128;

                if (connected_phase) {
                    // P15: Search all boxes for truly nearest
                    const int total = (int)boxes_.size();
                    if (total <= K_SUB_ALL) {
                        for (int bi = 0; bi < total; ++bi) {
                            const double* cc = base_cc + bi * nd;
                            double d2 = 0.0;
                            for (int k = 0; k < nd; ++k) {
                                double dk = cc[k] - qp[k];
                                d2 += dk * dk;
                            }
                            if (d2 < best_dist) { best_dist = d2; best_idx = bi; }
                        }
                    } else {
                        for (int s = 0; s < K_SUB_ALL; ++s) {
                            int bi = std::uniform_int_distribution<int>(0, total - 1)(rng_);
                            const double* cc = base_cc + bi * nd;
                            double d2 = 0.0;
                            for (int k = 0; k < nd; ++k) {
                                double dk = cc[k] - qp[k];
                                d2 += dk * dk;
                            }
                            if (d2 < best_dist) { best_dist = d2; best_idx = bi; }
                        }
                    }
                } else {
                    // Pre-connectivity: search within selected tree
                    const auto& tbi = tree_box_indices[tree_id];
                    const int tsize = (int)tbi.size();
                    if (tsize <= K_SUBSAMPLE) {
                        for (int bi : tbi) {
                            const double* cc = base_cc + bi * nd;
                            double d2 = 0.0;
                            for (int k = 0; k < nd; ++k) {
                                double dk = cc[k] - qp[k];
                                d2 += dk * dk;
                            }
                            if (d2 < best_dist) { best_dist = d2; best_idx = bi; }
                        }
                    } else {
                        for (int s = 0; s < K_SUBSAMPLE; ++s) {
                            int ri = std::uniform_int_distribution<int>(0, tsize - 1)(rng_);
                            int bi = tbi[ri];
                            const double* cc = base_cc + bi * nd;
                            double d2 = 0.0;
                            for (int k = 0; k < nd; ++k) {
                                double dk = cc[k] - qp[k];
                                d2 += dk * dk;
                            }
                            if (d2 < best_dist) { best_dist = d2; best_idx = bi; }
                        }
                    }
                }
            }
            if (best_idx < 0) continue;

            // P14: Bridge seeds bypass snap_to_face — use directly as FFB seed
            Eigen::VectorXd seed_for_ffb;
            int parent_id_for_task;
            int face_dim_for_task = -1, face_side_for_task = -1;

            if (use_bridge) {
                // Direct seed from interpolation line
                seed_for_ffb = q_rand;
                parent_id_for_task = boxes_[best_idx].id;
            } else {
                // Normal: snap to face of nearest box
                double saved_step_ratio = config_.rrt_step_ratio;
                config_.rrt_step_ratio = effective_step;

                Eigen::VectorXd dir = q_rand - boxes_[best_idx].center();
                double dir_norm = dir.norm();
                if (dir_norm < 1e-12) { config_.rrt_step_ratio = saved_step_ratio; continue; }
                dir /= dir_norm;
                auto snap = snap_to_face(boxes_[best_idx], dir);

                config_.rrt_step_ratio = saved_step_ratio;

                seed_for_ffb = snap.seed;
                parent_id_for_task = boxes_[best_idx].id;
                face_dim_for_task = snap.face_dim;
                face_side_for_task = snap.face_side;
            }

            // Pre-filter: reject if seed inside any existing box (O(n))
            {
                auto t_pf0 = Clock::now();
                bool inside = false;
                for (const auto& b : boxes_) {
                    if (b.contains(seed_for_ffb)) { inside = true; break; }
                }
                t_prefilter_ms += std::chrono::duration<double, std::milli>(Clock::now() - t_pf0).count();
                if (inside) { n_prefilter_rejects++; continue; }
            }

            tasks.push_back({seed_for_ffb, parent_id_for_task,
                             face_dim_for_task, face_side_for_task,
                             boxes_[best_idx].root_id});
        }

        if (tasks.empty()) {
            miss_count++;
            continue;
        }

        t_task_gen_ms += std::chrono::duration<double, std::milli>(Clock::now() - t_gen0).count();

        // ── Dispatch FFB to workers ─────────────────────────────────────
        // Task i → worker_lects[i].  batch ≤ n_workers, so each LECT is
        // accessed by at most one concurrent task (no data race).
        std::vector<std::future<WorkerResult>> futures;
        futures.reserve(tasks.size());

        for (int ti = 0; ti < (int)tasks.size(); ++ti) {
            LECT* lp = worker_lects[ti % n_workers].get();
            auto seed = tasks[ti].seed;
            int parent_id = tasks[ti].parent_box_id;
            int root_id = tasks[ti].root_id;
            FFBConfig fcfg = config_.ffb_config;
            // O3: Adaptive FFB depth — shallower after connectivity for speed
            if (connected_phase && fcfg.max_depth > 100)
                fcfg.max_depth = 100;

            futures.push_back(pool.submit(
                [lp, seed, parent_id, root_id,
                 obs, n_obs, fcfg]() -> WorkerResult {
                    WorkerResult wr;
                    wr.ffb = find_free_box(*lp, seed, obs, n_obs, fcfg);
                    if (!wr.ffb.success() || lp->is_occupied(wr.ffb.node_idx))
                        return wr;
                    wr.box.joint_intervals = lp->node_intervals(wr.ffb.node_idx);
                    wr.box.seed_config = seed;
                    wr.box.tree_id = wr.ffb.node_idx;
                    wr.box.parent_box_id = parent_id;
                    wr.box.root_id = root_id;
                    wr.box.compute_volume();
                    wr.success = true;
                    // Mark occupied in worker LECT to prevent self-reuse
                    lp->mark_occupied(wr.ffb.node_idx, 0);
                    return wr;
                }
            ));
        }

        // ── Collect results (master accepts/rejects) ────────────────────
        // First, wait for all futures to complete (FFB wall time)
        std::vector<WorkerResult> results(futures.size());
        {
            auto t_wait0 = Clock::now();
            for (int fi = 0; fi < (int)futures.size(); ++fi)
                results[fi] = futures[fi].get();
            t_ffb_wall_ms += std::chrono::duration<double, std::milli>(Clock::now() - t_wait0).count();
        }
        auto t_post0 = Clock::now();
        int batch_success = 0;
        int batch_start_idx = (int)boxes_.size();
        for (int fi = 0; fi < (int)results.size(); ++fi) {
            auto& wr = results[fi];

            // Accumulate FFB statistics
            ffb_total_calls_++;
            ffb_total_ms_      += wr.ffb.total_ms;
            ffb_envelope_ms_   += wr.ffb.envelope_ms;
            ffb_collide_ms_    += wr.ffb.collide_ms;
            ffb_expand_ms_     += wr.ffb.expand_ms;
            ffb_intervals_ms_  += wr.ffb.intervals_ms;
            ffb_cache_hits_    += wr.ffb.n_cache_hits;
            ffb_cache_misses_  += wr.ffb.n_cache_misses;
            ffb_collide_calls_ += wr.ffb.n_collide_calls;
            ffb_expand_calls_  += wr.ffb.n_expand_calls;
            ffb_total_steps_   += wr.ffb.n_steps;

            if (!wr.success) { n_ffb_fail_++; continue; }

            // Master-side validation: reject if seed inside any existing box (O(n))
            {
                bool reject = false;
                for (const auto& b : boxes_) {
                    if (b.contains(wr.box.seed_config)) { reject = true; break; }
                }
                if (reject) { n_ffb_fail_++; n_postfilter_rejects++; continue; }
            }

            // Accept box
            wr.box.id = next_box_id_++;
            boxes_.push_back(std::move(wr.box));
            n_ffb_success_++;
            batch_success++;

            // Update center cache
            const auto& nb = boxes_.back();
            for (int d = 0; d < nd; ++d)
                center_cache.push_back(nb.joint_intervals[d].center());

            // Update tree box count and index
            if (nb.root_id >= 0 && nb.root_id < n_trees) {
                tree_box_count[nb.root_id]++;
                tree_box_indices[nb.root_id].push_back((int)boxes_.size() - 1);
            }

            // Enforce parent adjacency (serial, master-side)
            bool adj_ok = enforce_parent_adjacency(tasks[fi].parent_box_id,
                                     tasks[fi].face_dim, tasks[fi].face_side,
                                     obs, n_obs);

            // Cross-tree adjacency for connect_mode
            // P9: Only check boxes from OTHER trees (skip same-tree)
            bool cross_tree_touch = false;
            if (cm && n_comp > 1) {
                auto t_ct0 = Clock::now();
                int old_comp = n_comp;
                int new_idx = (int)boxes_.size() - 1;
                int rj = boxes_[new_idx].root_id;
                if (rj >= 0 && rj < n_trees) {
                    for (int ti = 0; ti < n_trees && n_comp > 1; ++ti) {
                        if (ti == rj || tree_uf.connected(ti, rj)) continue;
                        for (int ei : tree_box_indices[ti]) {
                            if (boxes_adjacent(boxes_[ei], boxes_[new_idx])) {
                                tree_uf.unite(ti, rj);
                                n_comp--;
                                cross_tree_touch = true;
                                // P12: Update cached comp_size
                                std::fill(comp_size.begin(), comp_size.end(), 0);
                                for (int t = 0; t < n_trees; ++t)
                                    comp_size[tree_uf.find(t)]++;
                                break;
                            }
                        }
                    }
                }
                if (n_comp < old_comp) {
                    last_comp_change_boxes = (int)boxes_.size();
                    // Record first connectivity time
                    if (n_comp <= 1 && first_connect_time_ms < 0) {
                        first_connect_time_ms = std::chrono::duration<double>(
                            Clock::now() - t_start).count() * 1000.0;
                        first_connect_boxes = (int)boxes_.size();
                    }
                }
                t_cross_tree_ms += std::chrono::duration<double, std::milli>(Clock::now() - t_ct0).count();
            }

            // Reject isolated boxes: if enforce_parent_adjacency failed
            // AND the box didn't contribute to cross-tree connectivity,
            // check if it touches any existing box.  Isolated boxes create
            // orphan islands in the adjacency graph.
            if (!adj_ok && !cross_tree_touch) {
                int new_idx = (int)boxes_.size() - 1;
                bool touches_any = false;
                // Check parent directly
                for (int pi = new_idx - 1; pi >= 0; --pi) {
                    if (boxes_[pi].id == tasks[fi].parent_box_id) {
                        touches_any = boxes_adjacent(boxes_[pi], boxes_[new_idx]);
                        break;
                    }
                }
                // If not touching parent, scan nearby boxes (adaptive window)
                if (!touches_any) {
                    int window = std::min(new_idx / 5 + 100, 1000);
                    int scan_start = std::max(0, new_idx - window);
                    for (int si = new_idx - 1; si >= scan_start; --si) {
                        if (boxes_adjacent(boxes_[si], boxes_[new_idx])) {
                            touches_any = true;
                            break;
                        }
                    }
                }
                if (!touches_any) {
                    // Reject isolated box
                    n_isolated_rejects++;
                    boxes_.pop_back();
                    n_ffb_fail_++;
                    batch_success--;
                    // Remove from center cache
                    for (int d = 0; d < nd; ++d)
                        center_cache.pop_back();
                    // Remove from tree indices
                    int rj_rej = tasks[fi].root_id;
                    if (rj_rej >= 0 && rj_rej < n_trees) {
                        tree_box_count[rj_rej]--;
                        if (!tree_box_indices[rj_rej].empty())
                            tree_box_indices[rj_rej].pop_back();
                    }
                    continue;
                }
            }
        }
        t_post_accept_ms += std::chrono::duration<double, std::milli>(Clock::now() - t_post0).count();

        if (batch_success > 0) miss_count = 0;
        else miss_count += (int)tasks.size();

        total_batches++;

        // Periodic LECT refresh for workers
        if (total_batches % lect_refresh_interval == 0) {
            for (int i = 0; i < n_workers; ++i)
                *worker_lects[i] = lect_.snapshot();
        }

        // ── P17: Periodic inline promotion after connectivity ───────────
        //  Merge sibling LECT leaves into parent → fewer, larger boxes.
        //  Runs every promote_interval new boxes after first connectivity.
        if (connected_phase && config_.enable_promotion &&
            (int)boxes_.size() - boxes_at_last_promote >= promote_interval) {
            int before = (int)boxes_.size();
            int np = promote_all(obs, n_obs);
            n_inline_promotions += np;
            boxes_at_last_promote = (int)boxes_.size();

            if (np > 0) {
                // Rebuild center cache (promote modifies boxes_)
                center_cache.clear();
                center_cache.reserve(boxes_.size() * nd);
                for (const auto& b : boxes_)
                    for (int d = 0; d < nd; ++d)
                        center_cache.push_back(b.joint_intervals[d].center());

                // Rebuild tree indices
                for (auto& v : tree_box_indices) v.clear();
                std::fill(tree_box_count.begin(), tree_box_count.end(), 0);
                for (int i = 0; i < (int)boxes_.size(); ++i) {
                    if (boxes_[i].root_id >= 0 && boxes_[i].root_id < n_trees) {
                        tree_box_count[boxes_[i].root_id]++;
                        tree_box_indices[boxes_[i].root_id].push_back(i);
                    }
                }

                // Refresh worker LECTs
                for (int i = 0; i < n_workers; ++i)
                    *worker_lects[i] = lect_.snapshot();

                SBF_INFO("[GRW] inline promote: %d→%d boxes (%d merges)", before, (int)boxes_.size(), np);
            }
        }

        // Periodic progress log
        int cur_boxes = (int)boxes_.size();
        if (cur_boxes - last_log_boxes >= 500) {
            double elapsed = std::chrono::duration<double>(
                Clock::now() - t_start).count();
            SBF_INFO("[GRW] coordinated progress: %d boxes, %.1fs", cur_boxes, elapsed);
            if (cm) SBF_INFO(", %d components", n_comp);
            if (connected_phase) SBF_INFO(" [COV sr=%.4f]", effective_step);
            else if (stalled) SBF_INFO(" [STALL lv%d sr=%.4f]", stall_level, cur_step_ratio);
            SBF_INFO("");
            last_log_boxes = cur_boxes;
        }
    }

    // ── Merge worker LECT expand profiles ───────────────────────────────
    for (auto& wl : worker_lects)
        lect_.expand_profile_.merge(wl->expand_profile_);

    // ── Report results ──────────────────────────────────────────────────
    double elapsed = std::chrono::duration<double>(Clock::now() - t_start).count();
    SBF_INFO("[GRW] coordinated done: %d boxes, %d batches, %.1fs", (int)boxes_.size(), total_batches, elapsed);
    if (n_inline_promotions > 0)
        SBF_INFO(", %d inline promotions", n_inline_promotions);
    SBF_INFO("");
    // ── Profiling breakdown ─────────────────────────────────────────────
    SBF_INFO("[GRW] profile: task_gen=%.0fms ffb_wall=%.0fms post_accept=%.0fms", t_task_gen_ms, t_ffb_wall_ms, t_post_accept_ms);
    SBF_INFO("[GRW] profile: prefilter=%.0fms(rej=%d) cross_tree=%.0fms", t_prefilter_ms, n_prefilter_rejects, t_cross_tree_ms);
    SBF_INFO("[GRW] profile: post_rej=%d isolated_rej=%d", n_postfilter_rejects, n_isolated_rejects);

    if (cm) {
        wf_all_connected_ = (n_comp <= 1);
        // Use first connectivity time if achieved, otherwise total elapsed
        wf_connect_time_ms_ = (first_connect_time_ms >= 0)
            ? first_connect_time_ms : elapsed * 1000.0;
        wf_connect_boxes_ = first_connect_boxes;
        std::unordered_map<int, std::vector<int>> comp_trees;
        for (int t = 0; t < n_trees; ++t)
            comp_trees[tree_uf.find(t)].push_back(t);
        SBF_INFO("[GRW] connect: %d components%s", n_comp, wf_all_connected_ ? " — ALL CONNECTED" : "");
        if (first_connect_time_ms >= 0)
            SBF_INFO(" (first at %.0fms, %d boxes)", first_connect_time_ms, first_connect_boxes);
        SBF_INFO("");
        for (auto& [rep, members] : comp_trees) {
            SBF_INFO("  component[%d]:", rep);
            for (int m : members) SBF_INFO(" tree%d", m);
            SBF_INFO("");
        }
    }
    {
        std::unordered_map<int, int> tree_sizes;
        for (const auto& b : boxes_) tree_sizes[b.root_id]++;
        SBF_INFO("[GRW] tree sizes:");
        for (const auto& kv : tree_sizes)
            SBF_INFO(" root%d=%d", kv.first, kv.second);
        SBF_INFO("");
    }
}

}  // namespace sbf

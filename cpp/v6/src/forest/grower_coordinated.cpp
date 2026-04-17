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
#include <cstdlib>
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
        bool is_bridge;
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
    int n_enforce_ok = 0, n_enforce_fail = 0;
    int n_enforce_ok_bridge = 0, n_enforce_fail_bridge = 0;

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

            // ── Find nearest box to q_rand using point-to-box distance ────
            // For each candidate box, compute the squared distance from
            // q_rand to the L∞-closest point on the box surface.  This is
            // much better than center distance for elongated boxes.
            //
            // Pre-connectivity: search within selected tree only.
            // Connected phase / bridge: search all boxes globally.
            //
            // The search also records the closest point on the box surface
            // and the face through which q_rand "exits" the box — this is
            // used to build the FFB seed.
            int best_idx = -1;
            double best_dist2 = std::numeric_limits<double>::max();
            for (int d = 0; d < nd; ++d) q_buf[d] = q_rand[d];

            // Lambda: compute squared distance from point q to box and
            //         the clamped (nearest) point.
            auto box_point_dist2 = [&](int bi) -> double {
                const auto& bx = boxes_[bi];
                double d2 = 0.0;
                for (int d = 0; d < nd; ++d) {
                    double v = q_buf[d];
                    double lo = bx.joint_intervals[d].lo;
                    double hi = bx.joint_intervals[d].hi;
                    if (v < lo) { double g = lo - v; d2 += g * g; }
                    else if (v > hi) { double g = v - hi; d2 += g * g; }
                    // else inside interval, distance 0 in this dim
                }
                return d2;
            };

            // Subsample nearest-box search (full scan is too greedy,
            // reduces overlap).  Surface distance is still used per box.
            {
                constexpr int K_TREE = 128;
                constexpr int K_ALL  = 256;
                if (connected_phase || use_bridge) {
                    const int total = (int)boxes_.size();
                    if (total <= K_ALL) {
                        for (int bi = 0; bi < total; ++bi) {
                            double d2 = box_point_dist2(bi);
                            if (d2 < best_dist2) { best_dist2 = d2; best_idx = bi; }
                        }
                    } else {
                        for (int s = 0; s < K_ALL; ++s) {
                            int bi = std::uniform_int_distribution<int>(0, total - 1)(rng_);
                            double d2 = box_point_dist2(bi);
                            if (d2 < best_dist2) { best_dist2 = d2; best_idx = bi; }
                        }
                    }
                } else {
                    const auto& tbi = tree_box_indices[tree_id];
                    const int tsize = (int)tbi.size();
                    if (tsize <= K_TREE) {
                        for (int bi : tbi) {
                            double d2 = box_point_dist2(bi);
                            if (d2 < best_dist2) { best_dist2 = d2; best_idx = bi; }
                        }
                    } else {
                        for (int s = 0; s < K_TREE; ++s) {
                            int ri = std::uniform_int_distribution<int>(0, tsize - 1)(rng_);
                            double d2 = box_point_dist2(tbi[ri]);
                            if (d2 < best_dist2) { best_dist2 = d2; best_idx = tbi[ri]; }
                        }
                    }
                }
            }
            if (best_idx < 0) continue;

            // ── Compute nearest surface point and extension direction ────
            // Clamp q_rand into the box → nearest interior point.
            // Then find the face dimension where q_rand most strongly
            // "exits" the box, and place the seed just outside that face.
            const auto& parent_box = boxes_[best_idx];
            const double eps = config_.boundary_epsilon;

            Eigen::VectorXd seed_for_ffb(nd);
            int face_dim_for_task = -1;
            int face_side_for_task = -1;

            // If q_rand is OUTSIDE the box, find the dimension with the
            // largest signed exit distance and place seed on that face.
            // If q_rand is INSIDE (or on surface), use direction from center.
            {
                // Find exit face: the dimension where q_rand is furthest
                // beyond the box boundary
                double max_exit = -1e30;
                for (int d = 0; d < nd; ++d) {
                    double lo = parent_box.joint_intervals[d].lo;
                    double hi = parent_box.joint_intervals[d].hi;
                    // How far q_rand overshoots on the lo side
                    double exit_lo = lo - q_buf[d];  // positive if q < lo
                    // How far q_rand overshoots on the hi side
                    double exit_hi = q_buf[d] - hi;  // positive if q > hi
                    if (exit_lo > max_exit) {
                        max_exit = exit_lo;
                        face_dim_for_task = d;
                        face_side_for_task = 0;
                    }
                    if (exit_hi > max_exit) {
                        max_exit = exit_hi;
                        face_dim_for_task = d;
                        face_side_for_task = 1;
                    }
                }

                if (max_exit <= 0.0) {
                    // q_rand is inside the box → use direction from center
                    Eigen::VectorXd dir = q_rand - parent_box.center();
                    double dir_norm = dir.norm();
                    if (dir_norm < 1e-12) continue;
                    dir /= dir_norm;

                    // Pick the face most aligned with direction
                    double best_score = -1e30;
                    face_dim_for_task = -1;
                    for (int d = 0; d < nd; ++d) {
                        for (int side = 0; side < 2; ++side) {
                            double normal_sign = (side == 1) ? 1.0 : -1.0;
                            double score = dir[d] * normal_sign;
                            if (score <= 0.0) continue;
                            double face_val = (side == 0)
                                ? parent_box.joint_intervals[d].lo
                                : parent_box.joint_intervals[d].hi;
                            double limit_val = (side == 0)
                                ? limits[d].lo : limits[d].hi;
                            if (std::abs(face_val - limit_val) < eps) continue;
                            if (score > best_score) {
                                best_score = score;
                                face_dim_for_task = d;
                                face_side_for_task = side;
                            }
                        }
                    }
                    if (face_dim_for_task < 0) continue;
                }

                // Build the seed: place at face CENTER for maximum
                // overlap with parent in non-face dimensions.
                // Clamping q_rand to box corners creates poor overlap.
                for (int d = 0; d < nd; ++d)
                    seed_for_ffb[d] = parent_box.joint_intervals[d].center();
                // Offset just outside the chosen face
                if (face_side_for_task == 0) {
                    seed_for_ffb[face_dim_for_task] =
                        parent_box.joint_intervals[face_dim_for_task].lo - eps;
                } else {
                    seed_for_ffb[face_dim_for_task] =
                        parent_box.joint_intervals[face_dim_for_task].hi + eps;
                }
                seed_for_ffb = clamp_to_limits(seed_for_ffb);
            }

            int parent_id_for_task = parent_box.id;
            int task_root_id = use_bridge ? tree_id : parent_box.root_id;

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
                             task_root_id, use_bridge});
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
            if (adj_ok) { n_enforce_ok++; if (tasks[fi].is_bridge) n_enforce_ok_bridge++; }
            else        { n_enforce_fail++; if (tasks[fi].is_bridge) n_enforce_fail_bridge++; }

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

            // Reject boxes not adjacent to own tree: even if we found
            // a cross-tree touch (tree_uf already updated), the box must
            // still be reachable from its own tree root in the adjacency
            // graph.  Keeping it when it only touches other-tree boxes
            // creates orphan islands (the root cause of 5-island problem).
            // The tree_uf merge is kept — it's an optimistic signal that
            // trees are close enough; subsequent growth will naturally
            // produce geometrically-connected cross-tree boxes.
            if (!adj_ok) {
                int new_idx = (int)boxes_.size() - 1;
                bool touches_own = false;
                int own_root = tasks[fi].root_id;
                // Check parent directly (parent is same-tree by construction)
                for (int pi = new_idx - 1; pi >= 0; --pi) {
                    if (boxes_[pi].id == tasks[fi].parent_box_id) {
                        touches_own = boxes_adjacent(boxes_[pi], boxes_[new_idx]);
                        break;
                    }
                }
                // If not touching parent, scan same-tree boxes
                if (!touches_own && own_root >= 0 && own_root < n_trees) {
                    for (int si : tree_box_indices[own_root]) {
                        if (si == new_idx) continue;
                        if (boxes_adjacent(boxes_[si], boxes_[new_idx])) {
                            touches_own = true;
                            break;
                        }
                    }
                }
                if (!touches_own) {
                    // Reject: not adjacent to any same-tree box
                    n_isolated_rejects++;
                    boxes_.pop_back();
                    n_ffb_fail_++;
                    batch_success--;
                    for (int d = 0; d < nd; ++d)
                        center_cache.pop_back();
                    if (own_root >= 0 && own_root < n_trees) {
                        tree_box_count[own_root]--;
                        if (!tree_box_indices[own_root].empty())
                            tree_box_indices[own_root].pop_back();
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

    // ── Parent-link diagnostic ──────────────────────────────────────────
    {
        int n_broken = 0, n_checked = 0;
        std::unordered_map<int, int> id_to_idx;
        for (int i = 0; i < (int)boxes_.size(); ++i)
            id_to_idx[boxes_[i].id] = i;
        for (int i = 0; i < (int)boxes_.size(); ++i) {
            int pid = boxes_[i].parent_box_id;
            if (pid < 0) continue;
            auto it = id_to_idx.find(pid);
            if (it == id_to_idx.end()) { n_broken++; n_checked++; continue; }
            n_checked++;
            if (!boxes_adjacent(boxes_[i], boxes_[it->second]))
                n_broken++;
        }
        SBF_INFO("[GRW] parent-link check: %d/%d broken\n", n_broken, n_checked);

        // OPT: gate O(n²) cross-tree diagnostic behind SBF_STAGE_LOG env
        // var. For 5k boxes this is ~13M pair comparisons (≈100ms).
        static const bool kCrossAdjLog = [] {
            const char* e = std::getenv("SBF_STAGE_LOG");
            return e && e[0] && e[0] != '0';
        }();
        if (kCrossAdjLog) {
            int n_cross_adj = 0;
            for (int i = 0; i < (int)boxes_.size(); ++i) {
                int ri = boxes_[i].root_id;
                for (int j = i + 1; j < (int)boxes_.size(); ++j) {
                    if (boxes_[j].root_id == ri) continue;
                    if (boxes_adjacent(boxes_[i], boxes_[j]))
                        n_cross_adj++;
                }
            }
            SBF_INFO("[GRW] cross-tree adjacencies: %d\n", n_cross_adj);
        }
    }

    // ── Post-grow island repair: chain-grow to bridge gaps ────────────
    // Build adjacency UF, find islands, then grow a CHAIN of boxes
    // from each small island toward the main island.
    if (cm) {
        auto t_repair0 = Clock::now();
        const double eps = config_.boundary_epsilon;

        // Helper: build UF using parent-link + dim-0 sweep local scan.
        // O(n log n + n·k) instead of O(n²), where k = avg dim-0 overlap.
        auto build_box_uf = [&](int n) -> UnionFind {
            UnionFind uf(n);
            // Phase 1: parent links
            std::unordered_map<int, int> id_map;
            for (int i = 0; i < n; ++i) id_map[boxes_[i].id] = i;
            for (int i = 0; i < n; ++i) {
                auto it = id_map.find(boxes_[i].parent_box_id);
                if (it != id_map.end() && boxes_adjacent(boxes_[i], boxes_[it->second]))
                    uf.unite(i, it->second);
            }
            // Phase 2: dim-0 sweep for cross-component adjacency
            // Sort indices by dim-0 lo bound
            std::vector<int> order(n);
            std::iota(order.begin(), order.end(), 0);
            std::sort(order.begin(), order.end(), [&](int a, int b) {
                return boxes_[a].joint_intervals[0].lo
                     < boxes_[b].joint_intervals[0].lo;
            });
            for (int ii = 0; ii < n; ++ii) {
                int i = order[ii];
                double hi0 = boxes_[i].joint_intervals[0].hi;
                for (int jj = ii + 1; jj < n; ++jj) {
                    int j = order[jj];
                    if (boxes_[j].joint_intervals[0].lo > hi0 + 1e-6)
                        break;  // no more dim-0 overlap
                    if (uf.connected(i, j)) continue;
                    if (boxes_adjacent(boxes_[i], boxes_[j]))
                        uf.unite(i, j);
                }
            }
            return uf;
        };

        const int nb = (int)boxes_.size();
        UnionFind box_uf = build_box_uf(nb);

        std::unordered_map<int, std::vector<int>> comps;
        for (int i = 0; i < nb; ++i)
            comps[box_uf.find(i)].push_back(i);
        int n_islands = (int)comps.size();
        int main_rep = -1, main_sz = 0;
        for (auto& [r, v] : comps)
            if ((int)v.size() > main_sz) { main_sz = (int)v.size(); main_rep = r; }
        SBF_INFO("[GRW] repair: %d islands (main=%d)\n", n_islands, main_sz);

        // Chain-grow repair: bidirectional, multiple starting points
        int total_repair = 0;
        const double repair_budget_ms = 2000.0;

        // Chain-grow lambda: grow from start toward target, check
        // bridge against check_set (uses frontier of max 200 boxes).
        // Returns true if bridge found.
        auto chain_grow = [&](int start_idx, const Eigen::VectorXd& target,
                              const std::vector<int>& check_set,
                              int max_steps) -> bool {
            // Pre-select nearby frontier from check_set
            Eigen::VectorXd sc(nd);
            for (int d = 0; d < nd; ++d)
                sc[d] = boxes_[start_idx].joint_intervals[d].center();
            std::vector<std::pair<double, int>> cdist;
            for (int ci : check_set) {
                double d2 = 0.0;
                for (int d = 0; d < nd; ++d) {
                    double diff = boxes_[ci].joint_intervals[d].center() - sc[d];
                    d2 += diff * diff;
                }
                cdist.push_back({d2, ci});
            }
            std::sort(cdist.begin(), cdist.end());
            std::vector<int> frontier;
            for (int i = 0; i < std::min((int)cdist.size(), 200); ++i)
                frontier.push_back(cdist[i].second);

            int cur = start_idx;
            for (int step = 0; step < max_steps; ++step) {
                const auto cb = boxes_[cur];  // copy
                Eigen::VectorXd dir(nd);
                if (u01(rng_) < 0.7) {
                    for (int d = 0; d < nd; ++d)
                        dir[d] = target[d] - cb.joint_intervals[d].center();
                } else {
                    for (int d = 0; d < nd; ++d)
                        dir[d] = std::normal_distribution<double>(0.0, 1.0)(rng_);
                }
                double dn = dir.norm();
                if (dn < 1e-12) continue;
                dir /= dn;

                int fd = -1, fs = -1;
                double bs = -1e30;
                for (int d = 0; d < nd; ++d) {
                    for (int side = 0; side < 2; ++side) {
                        double ns = (side == 1) ? 1.0 : -1.0;
                        double sc2 = dir[d] * ns;
                        if (sc2 <= 0.0) continue;
                        double fv = (side == 0)
                            ? cb.joint_intervals[d].lo : cb.joint_intervals[d].hi;
                        double lv = (side == 0)
                            ? limits[d].lo : limits[d].hi;
                        if (std::abs(fv - lv) < eps) continue;
                        if (sc2 > bs) { bs = sc2; fd = d; fs = side; }
                    }
                }
                if (fd < 0) break;

                Eigen::VectorXd seed(nd);
                for (int d = 0; d < nd; ++d)
                    seed[d] = std::clamp(target[d],
                        cb.joint_intervals[d].lo,
                        cb.joint_intervals[d].hi);
                if (fs == 0) seed[fd] = cb.joint_intervals[fd].lo - eps;
                else         seed[fd] = cb.joint_intervals[fd].hi + eps;
                seed = clamp_to_limits(seed);

                // Advance through existing boxes
                bool ins = false;
                const int nc = (int)boxes_.size();
                for (int bi = 0; bi < nc; ++bi) {
                    if (boxes_[bi].contains(seed)) {
                        cur = bi;
                        for (int fi : frontier)
                            if (boxes_adjacent(boxes_[bi], boxes_[fi]))
                                return true;
                        ins = true;
                        break;
                    }
                }
                if (ins) continue;

                FFBConfig fc = config_.ffb_config;
                auto ffb = find_free_box(lect_, seed, obs, n_obs, fc);
                if (!ffb.success() || lect_.is_occupied(ffb.node_idx)) continue;

                BoxNode nb;
                nb.joint_intervals = lect_.node_intervals(ffb.node_idx);
                nb.seed_config = seed;
                nb.tree_id = ffb.node_idx;
                nb.parent_box_id = cb.id;
                nb.root_id = cb.root_id;
                nb.id = next_box_id_++;
                nb.compute_volume();
                lect_.mark_occupied(ffb.node_idx, 0);
                boxes_.push_back(std::move(nb));
                enforce_parent_adjacency(cb.id, fd, fs, obs, n_obs);

                int ni = (int)boxes_.size() - 1;
                for (int fi : frontier)
                    if (boxes_adjacent(boxes_[fi], boxes_[ni]))
                        return true;
                cur = ni;
            }
            return false;
        };

        for (int round = 0; round < 3 && n_islands > 1; ++round) {
            double elapsed_ms = std::chrono::duration<double, std::milli>(
                Clock::now() - t_repair0).count();
            if (elapsed_ms > repair_budget_ms) break;

            int round_ok = 0;
            for (auto& [rep, members] : comps) {
                if (rep == main_rep) continue;
                elapsed_ms = std::chrono::duration<double, std::milli>(
                    Clock::now() - t_repair0).count();
                if (elapsed_ms > repair_budget_ms) break;

                // Find top-3 diverse starting pairs (small→main)
                struct GapPair { double d2; int si, mi; };
                std::vector<GapPair> gap_pairs;
                // Subsample main island
                std::vector<int> main_sub;
                if (main_sz <= 300) main_sub = comps[main_rep];
                else {
                    main_sub.reserve(300);
                    for (int s = 0; s < 300; ++s)
                        main_sub.push_back(comps[main_rep][
                            std::uniform_int_distribution<int>(0, main_sz - 1)(rng_)]);
                }
                for (int si : members) {
                    for (int mi : main_sub) {
                        double d2 = 0.0;
                        for (int d = 0; d < nd; ++d) {
                            double gap = std::max(
                                boxes_[si].joint_intervals[d].lo - boxes_[mi].joint_intervals[d].hi,
                                boxes_[mi].joint_intervals[d].lo - boxes_[si].joint_intervals[d].hi);
                            if (gap > 0) d2 += gap * gap;
                        }
                        gap_pairs.push_back({d2, si, mi});
                    }
                }
                std::sort(gap_pairs.begin(), gap_pairs.end(),
                          [](const GapPair& a, const GapPair& b) { return a.d2 < b.d2; });
                std::vector<GapPair> top_pairs;
                std::unordered_set<int> used_si;
                for (auto& gp : gap_pairs) {
                    if (used_si.count(gp.si)) continue;
                    top_pairs.push_back(gp);
                    used_si.insert(gp.si);
                    // OPT: top 2 pairs (was 3) — seed_bridge later covers
                    // leftovers with a larger per-pair budget, so extra
                    // repair attempts mostly waste time here.
                    if ((int)top_pairs.size() >= 2) break;
                }

                double gap_dist = top_pairs.empty() ? 0.0 : std::sqrt(top_pairs[0].d2);
                SBF_INFO("[GRW] repair: island=%d gap=%.3f\n",
                         (int)members.size(), gap_dist);

                // Try each starting pair: forward then reverse
                bool bridged = false;
                // Strategy 1: Surface-extend near midpoint
                // Sample a point near the gap midpoint, find the nearest
                // existing box, compute the nearest surface point on that
                // box toward the sample, extend outward by eps to create
                // a surface-adjacent seed, then FFB from that seed.
                for (auto& gp : top_pairs) {
                    if (bridged) break;
                    Eigen::VectorXd mid(nd);
                    for (int d = 0; d < nd; ++d)
                        mid[d] = 0.5 * (boxes_[gp.si].joint_intervals[d].center() +
                                         boxes_[gp.mi].joint_intervals[d].center());
                    double g = std::sqrt(gp.d2);
                    for (int s = 0; s < 80 && !bridged; ++s) {
                        // Sample a query point near the midpoint
                        Eigen::VectorXd q(nd);
                        for (int d = 0; d < nd; ++d)
                            q[d] = mid[d] + std::normal_distribution<double>(0.0, g * 0.5)(rng_);
                        q = clamp_to_limits(q);

                        // Find nearest box (from both islands) using
                        // point-to-box distance
                        int best_bi = -1;
                        double best_d2 = std::numeric_limits<double>::max();
                        auto try_idx = [&](int bi) {
                            double d2 = 0.0;
                            for (int d = 0; d < nd; ++d) {
                                double v = q[d];
                                double lo = boxes_[bi].joint_intervals[d].lo;
                                double hi = boxes_[bi].joint_intervals[d].hi;
                                if (v < lo) { double gg = lo - v; d2 += gg * gg; }
                                else if (v > hi) { double gg = v - hi; d2 += gg * gg; }
                            }
                            if (d2 < best_d2) { best_d2 = d2; best_bi = bi; }
                        };
                        for (int si2 : members) try_idx(si2);
                        for (int mi2 : main_sub) try_idx(mi2);
                        if (best_bi < 0) continue;

                        // Compute exit face: dimension where q most
                        // strongly overshoots the box boundary
                        const auto& pb = boxes_[best_bi];
                        int fd = -1, fs = -1;
                        double max_exit = -1e30;
                        for (int d = 0; d < nd; ++d) {
                            double exit_lo = pb.joint_intervals[d].lo - q[d];
                            double exit_hi = q[d] - pb.joint_intervals[d].hi;
                            if (exit_lo > max_exit) {
                                max_exit = exit_lo; fd = d; fs = 0;
                            }
                            if (exit_hi > max_exit) {
                                max_exit = exit_hi; fd = d; fs = 1;
                            }
                        }
                        if (max_exit <= 0.0) {
                            // q inside box → pick face toward midpoint
                            Eigen::VectorXd dir = mid - pb.center();
                            double dn2 = dir.norm();
                            if (dn2 < 1e-12) continue;
                            dir /= dn2;
                            double bs2 = -1e30;
                            fd = -1;
                            for (int d = 0; d < nd; ++d) {
                                for (int side = 0; side < 2; ++side) {
                                    double ns = (side == 1) ? 1.0 : -1.0;
                                    double sc2 = dir[d] * ns;
                                    if (sc2 <= 0.0) continue;
                                    double fv = (side == 0)
                                        ? pb.joint_intervals[d].lo
                                        : pb.joint_intervals[d].hi;
                                    double lv = (side == 0)
                                        ? limits[d].lo : limits[d].hi;
                                    if (std::abs(fv - lv) < eps) continue;
                                    if (sc2 > bs2) { bs2 = sc2; fd = d; fs = side; }
                                }
                            }
                        }
                        if (fd < 0) continue;

                        // Build seed: clamp q to box, then offset outside
                        // the chosen face by eps
                        Eigen::VectorXd seed(nd);
                        for (int d = 0; d < nd; ++d) {
                            seed[d] = std::clamp(q[d],
                                pb.joint_intervals[d].lo,
                                pb.joint_intervals[d].hi);
                        }
                        if (fs == 0) seed[fd] = pb.joint_intervals[fd].lo - eps;
                        else         seed[fd] = pb.joint_intervals[fd].hi + eps;
                        seed = clamp_to_limits(seed);

                        // Skip if seed already inside some box
                        bool ins = false;
                        const int nc = (int)boxes_.size();
                        for (int bi = 0; bi < nc; ++bi)
                            if (boxes_[bi].contains(seed)) { ins = true; break; }
                        if (ins) continue;

                        // FFB from the surface-adjacent seed
                        FFBConfig fc = config_.ffb_config;
                        auto ffb = find_free_box(lect_, seed, obs, n_obs, fc);
                        if (!ffb.success() || lect_.is_occupied(ffb.node_idx))
                            continue;

                        BoxNode nb2;
                        nb2.joint_intervals = lect_.node_intervals(ffb.node_idx);
                        nb2.seed_config = seed;
                        nb2.tree_id = ffb.node_idx;
                        nb2.parent_box_id = pb.id;
                        nb2.root_id = pb.root_id;
                        nb2.id = next_box_id_++;
                        nb2.compute_volume();
                        lect_.mark_occupied(ffb.node_idx, 0);
                        boxes_.push_back(std::move(nb2));
                        enforce_parent_adjacency(pb.id, fd, fs, obs, n_obs);
                        int ni = (int)boxes_.size() - 1;
                        // Check if new box bridges both islands
                        bool touch_s = false, touch_m = false;
                        for (int si2 : members)
                            if (boxes_adjacent(boxes_[si2], boxes_[ni]))
                                { touch_s = true; break; }
                        for (int mi2 : main_sub)
                            if (boxes_adjacent(boxes_[mi2], boxes_[ni]))
                                { touch_m = true; break; }
                        if (touch_s && touch_m) bridged = true;
                    }
                }

                // Strategy 2: Chain-grow (each pair, forward + reverse)
                for (auto& gp : top_pairs) {
                    if (bridged) break;
                    elapsed_ms = std::chrono::duration<double, std::milli>(
                        Clock::now() - t_repair0).count();
                    if (elapsed_ms > repair_budget_ms) break;

                    // Forward: small → main
                    Eigen::VectorXd tgt(nd);
                    for (int d = 0; d < nd; ++d)
                        tgt[d] = boxes_[gp.mi].joint_intervals[d].center();
                    // OPT: max_boxes 150→60 — limits wasted expansion on
                    // gaps that can't be bridged (seed_bridge handles later).
                    if (chain_grow(gp.si, tgt, main_sub, 60)) {
                        bridged = true; break;
                    }
                    // Reverse: main → small (check against small island)
                    Eigen::VectorXd rtgt(nd);
                    for (int d = 0; d < nd; ++d)
                        rtgt[d] = boxes_[gp.si].joint_intervals[d].center();
                    if (chain_grow(gp.mi, rtgt, members, 60)) {
                        bridged = true; break;
                    }
                }

                if (bridged) {
                    round_ok++;
                    total_repair++;
                    // Merge small island members into main for next islands
                    comps[main_rep].insert(comps[main_rep].end(),
                                           members.begin(), members.end());
                    main_sz = (int)comps[main_rep].size();
                    SBF_INFO("[GRW] repair: bridged! (island=%d)\n",
                             (int)members.size());
                }
            } // end per-island

            if (round_ok == 0) break;

            // Rebuild UF for next round
            const int nb2 = (int)boxes_.size();
            box_uf = build_box_uf(nb2);
            comps.clear();
            for (int i = 0; i < nb2; ++i)
                comps[box_uf.find(i)].push_back(i);
            n_islands = (int)comps.size();
            main_rep = -1; main_sz = 0;
            for (auto& [r, v] : comps)
                if ((int)v.size() > main_sz) { main_sz = (int)v.size(); main_rep = r; }
            SBF_INFO("[GRW] repair round %d: %d islands (main=%d)\n",
                     round + 1, n_islands, main_sz);
        } // end rounds

        double repair_ms = std::chrono::duration<double, std::milli>(
            Clock::now() - t_repair0).count();
        SBF_INFO("[GRW] repair: bridged=%d final=%d islands (%.0fms)\n",
                 total_repair, n_islands, repair_ms);
    }

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
    SBF_INFO("[GRW] enforce: ok=%d(bridge=%d) fail=%d(bridge=%d)",
             n_enforce_ok, n_enforce_ok_bridge, n_enforce_fail, n_enforce_fail_bridge);

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

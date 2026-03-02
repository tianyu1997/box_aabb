// SafeBoxForest — SBFPlanner implementation (v4 logic: bidirectional tree
// growth with origin tracking, directed boundary expansion, connection
// detection, and random-tree absorption)
#include "sbf/planner/sbf_planner.h"
#include "sbf/forest/coarsen.h"
#include "sbf/forest/connectivity.h"
#include "sbf/planner/graph_search.h"
#include <chrono>
#include <algorithm>
#include <cmath>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <set>
#include <unordered_map>
#include <unordered_set>

namespace sbf {

SBFPlanner::SBFPlanner(const Robot& robot,
                        const std::vector<Obstacle>& obstacles,
                        const SBFConfig& config)
    : robot_(robot), obstacles_(obstacles), config_(config),
      rng_(config.seed)
{
    checker_ = CollisionChecker(robot_, obstacles_);
    smoother_ = PathSmoother(checker_, config_.segment_resolution);
    forest_ = SafeBoxForest(robot_.n_joints(), robot_.joint_limits());

    // Load KD-tree cache via mmap (lazy: OS loads pages on demand)
    if (config_.use_cache && !config_.cache_path.empty()) {
        bool file_exists = std::filesystem::exists(config_.cache_path);
        if (!file_exists) {
            // First run: create a minimal cache file (root only)
            auto parent = std::filesystem::path(config_.cache_path).parent_path();
            if (!parent.empty())
                std::filesystem::create_directories(parent);
            HierAABBTree fresh(robot_);
            fresh.save(config_.cache_path);
            file_exists = true;
        }
        try {
            tree_ = HierAABBTree::load_mmap(config_.cache_path, robot_);
            std::cout << "[SBFPlanner] mmap tree cache: "
                      << config_.cache_path
                      << " (" << tree_.store().next_idx() << " nodes, "
                      << tree_.total_fk_calls() << " fk_calls)\n";
        } catch (const std::exception& e) {
            std::cerr << "[SBFPlanner] mmap cache failed: " << e.what()
                      << ", starting fresh\n";
            tree_ = HierAABBTree(robot_);
        }
    } else {
        tree_ = HierAABBTree(robot_);
    }
}

PlanningResult SBFPlanner::plan(const Eigen::VectorXd& start,
                                 const Eigen::VectorXd& goal,
                                 double timeout) {
    auto t0 = std::chrono::steady_clock::now();
    PlanningResult result;

    auto elapsed = [&t0]() {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
    };

    // Step 1: Verify start/goal collision-free
    if (checker_.check_config(start)) {
        result.planning_time = elapsed();
        return result;
    }
    if (checker_.check_config(goal)) {
        result.planning_time = elapsed();
        return result;
    }

    // Step 2: Try direct connection
    if (!checker_.check_segment(start, goal, config_.segment_resolution)) {
        result.success = true;
        result.path.resize(2, start.size());
        result.path.row(0) = start.transpose();
        result.path.row(1) = goal.transpose();
        result.cost = (goal - start).norm();
        result.planning_time = elapsed();
        result.first_solution_time = elapsed();
        return result;
    }

    // Step 3: Build forest
    result.phase_times["direct_check"] = elapsed();
    build_forest(start, goal);
    forest_built_ = true;
    result.phase_times["forest_build"] = elapsed();

    if (elapsed() > timeout) {
        result.planning_time = elapsed();
        return result;
    }

    // Step 4: Coarsen
    coarsen_forest(forest_, checker_, config_.coarsen_max_rounds);
    forest_.rebuild_adjacency(config_.adjacency_tol);
    result.phase_times["coarsen"] = elapsed();

    // Step 5: Attach start/goal (use proxy if available)
    connector_ = TreeConnector(forest_, checker_);
    const Eigen::VectorXd& attach_start = has_proxy_start_ ? proxy_start_ : start;
    const Eigen::VectorXd& attach_goal  = has_proxy_goal_  ? proxy_goal_  : goal;
    auto start_attach = connector_.attach_config(attach_start);
    auto goal_attach  = connector_.attach_config(attach_goal);

    if (start_attach.box_id < 0 || goal_attach.box_id < 0) {
        result.planning_time = elapsed();
        return result;
    }

    // Step 6: Dijkstra search
    std::unordered_set<int> start_set{start_attach.box_id};
    std::unordered_set<int> goal_set{goal_attach.box_id};

    auto dijk = dijkstra_center_distance(
        forest_.adjacency(), forest_.boxes(), start_set, goal_set);
    result.phase_times["dijkstra"] = elapsed();

    if (!dijk.found) {
        // Try bridging as fallback
        std::vector<int> all_ids;
        for (auto& [id, _] : forest_.boxes()) all_ids.push_back(id);
        auto islands = find_islands(forest_.adjacency(), all_ids);

        if (islands.size() > 1) {
            auto bridge = bridge_islands(islands, forest_.boxes());
            for (auto& [a, b] : bridge.bridges) {
                forest_.adjacency()[a].push_back(b);
                forest_.adjacency()[b].push_back(a);
            }
            dijk = dijkstra_center_distance(
                forest_.adjacency(), forest_.boxes(), start_set, goal_set);
        }
    }

    if (!dijk.found) {
        result.planning_time = elapsed();
        return result;
    }

    // Step 7: Extract waypoints (using proxy endpoints for Dijkstra path)
    auto waypoints = extract_waypoints(dijk.path, forest_.boxes(),
                                        attach_start, attach_goal);
    result.phase_times["waypoints"] = elapsed();

    // Step 7b: Prepend/append real start/goal when proxy was used
    if (has_proxy_start_ && !waypoints.empty())
        waypoints.insert(waypoints.begin(), start);
    if (has_proxy_goal_ && !waypoints.empty())
        waypoints.push_back(goal);

    // Step 8: Shortcut + smooth
    auto smoothed = smoother_.box_aware_shortcut(waypoints, forest_,
                                                   config_.shortcut_max_iters, &rng_);
    smoothed = smoother_.smooth_moving_average(smoothed, forest_);

    // Step 9: Build result
    result.success = true;
    int n_wp = static_cast<int>(smoothed.size());
    int dof = start.size();
    result.path.resize(n_wp, dof);
    for (int i = 0; i < n_wp; ++i)
        result.path.row(i) = smoothed[i].transpose();

    result.cost = PathSmoother::path_length(smoothed);
    result.collision_checks = checker_.n_checks();
    result.nodes_explored = forest_.n_boxes();
    result.first_solution_time = elapsed();
    result.planning_time = elapsed();
    result.phase_times["total"] = elapsed();

    return result;
}

void SBFPlanner::build(const Eigen::VectorXd& start,
                        const Eigen::VectorXd& goal,
                        double timeout) {
    (void)timeout;
    build_forest(start, goal);
    coarsen_forest(forest_, checker_, config_.coarsen_max_rounds);
    forest_.rebuild_adjacency(config_.adjacency_tol);

    // Bridge disconnected islands (fallback)
    {
        std::vector<int> all_ids;
        for (auto& [id, _] : forest_.boxes()) all_ids.push_back(id);
        auto islands = find_islands(forest_.adjacency(), all_ids);
        if (islands.size() > 1) {
            auto bridge = bridge_islands(islands, forest_.boxes());
            for (auto& [a, b] : bridge.bridges) {
                forest_.adjacency()[a].push_back(b);
                forest_.adjacency()[b].push_back(a);
            }
        }
    }

    connector_ = TreeConnector(forest_, checker_);
    forest_built_ = true;

    // Save KD-tree cache
    save_tree_cache();
}

bool SBFPlanner::grow_only(const Eigen::VectorXd& start,
                            const Eigen::VectorXd& goal) {
    return grow_boxes(start, goal, /*skip_final_adjacency=*/true);
}

// ═══════════════════════════════════════════════════════════════════════════
// build_multi: multi-pair forest construction
//   Phase 1: per-pair directed growth (boxes_per_pair boxes each)
//   Phase 2: random boxes via uniform free-space sampling
//   Phase 3: bridge disconnected s-t pairs
// ═══════════════════════════════════════════════════════════════════════════
void SBFPlanner::build_multi(
    const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& pairs,
    int n_random_boxes,
    double timeout)
{
    auto t0 = std::chrono::steady_clock::now();
    auto elapsed = [&t0]() {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
    };

    int n_pairs = static_cast<int>(pairs.size());
    int total_quota = config_.max_boxes;
    int boxes_per_pair = (total_quota - n_random_boxes) / std::max(n_pairs, 1);
    if (boxes_per_pair < config_.min_boxes_per_pair) boxes_per_pair = config_.min_boxes_per_pair;

    std::cout << "[build_multi] " << n_pairs << " pairs, "
              << "up to " << config_.max_boxes_per_pair << " boxes/pair (isolated), "
              << n_random_boxes << " random, timeout=" << timeout << "s\n";

    // ══════════════════════════════════════════════════════════════════
    // Phase 1: per-pair isolated growth
    //   Each pair gets its OWN SBFPlanner (clean KD-tree).
    //   Boxes grown in clean isolation → no interference between pairs.
    //   Then all boxes are inserted into the shared main tree.
    // ══════════════════════════════════════════════════════════════════
    std::vector<bool> pair_connected(n_pairs, false);

    // Per-pair proxy info (captured from sub-planners)
    std::vector<Eigen::VectorXd> pair_proxy_start(n_pairs), pair_proxy_goal(n_pairs);
    std::vector<bool> pair_has_proxy_start(n_pairs, false), pair_has_proxy_goal(n_pairs, false);

    // Helper: insert a sub-planner's boxes into the main tree/forest
    auto merge_sub = [&](const SBFPlanner& sub) {
        for (const auto& [_, bx] : sub.forest().boxes()) {
            // Skip seeds that already landed inside an existing box
            if (tree_.is_occupied(bx.seed_config)) continue;

            auto ffb = tree_.find_free_box(bx.seed_config,
                                           checker_.obs_compact(), checker_.n_obs(),
                                           config_.ffb_max_depth, config_.ffb_min_edge);
            if (!ffb.success()) continue;

            auto pr = tree_.try_promote(ffb.path,
                                         checker_.obs_compact(), checker_.n_obs(), 2);
            auto ivs = tree_.get_node_intervals(pr.result_idx);

            int bid = forest_.allocate_id();
            BoxNode new_box(bid, ivs, bx.seed_config);
            forest_.add_box_no_adjacency(new_box);
            tree_.mark_occupied(pr.result_idx, bid);

            if (!pr.absorbed_box_ids.empty()) {
                std::unordered_set<int> abs(pr.absorbed_box_ids.begin(),
                                             pr.absorbed_box_ids.end());
                forest_.remove_boxes_no_adjacency(abs);
            }
        }
    };

    for (int pi = 0; pi < n_pairs; ++pi) {
        if (elapsed() > timeout) break;

        const auto& [s, g] = pairs[pi];

        // Budget: use max_boxes_per_pair so disconnected pairs keep growing until connected
        int pair_budget = (config_.max_boxes_per_pair > 0)
                          ? std::max(boxes_per_pair, config_.max_boxes_per_pair)
                          : boxes_per_pair;
        std::cout << "  [pair " << pi << "] growing up to " << pair_budget << " boxes ...";
        auto tg0 = std::chrono::steady_clock::now();

        // Isolated sub-planner: same robot + obstacles, fresh tree
        SBFConfig sub_cfg = config_;
        sub_cfg.max_boxes = pair_budget;
        sub_cfg.seed = config_.seed + pi;    // independent RNG
        sub_cfg.use_cache = false;           // sub-planners must NOT share main cache
        sub_cfg.cache_path.clear();
        SBFPlanner sub(robot_, obstacles_, sub_cfg);
        bool connected = sub.grow_only(s, g);

        // Capture proxy state from sub-planner
        if (sub.has_proxy_start()) {
            pair_proxy_start[pi] = sub.proxy_start();
            pair_has_proxy_start[pi] = true;
        }
        if (sub.has_proxy_goal()) {
            pair_proxy_goal[pi] = sub.proxy_goal();
            pair_has_proxy_goal[pi] = true;
        }

        int n_sub = sub.forest().n_boxes();
        double tg = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - tg0).count();
        std::cout << " done (" << n_sub << " isolated boxes, "
                  << std::fixed << std::setprecision(2) << tg << "s"
                  << (connected ? ", CONNECTED" : ", disconnected") << ")\n";

        // Merge into main tree
        auto tm0 = std::chrono::steady_clock::now();
        int before = forest_.n_boxes();
        merge_sub(sub);
        int after = forest_.n_boxes();
        double tm = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - tm0).count();
        std::cout << "           merged +" << (after - before)
                  << " boxes (total=" << after << ", "
                  << std::fixed << std::setprecision(2) << tm << "s)\n";

        pair_connected[pi] = connected;
    }

    // ══════════════════════════════════════════════════════════════════
    // Phase 2: guided random BFS growth with s-t bias
    //   - Seed sampling: guided (KD-tree unoccupied) + s-t biased
    //   - Each seed → create box → BFS expand from it (not just single box)
    //   - s-t bias: 30% of samples are biased toward disconnected pair endpoints
    // ══════════════════════════════════════════════════════════════════
    if (n_random_boxes > 0 && elapsed() < timeout) {
        std::cout << "  [random-bfs] growing " << n_random_boxes
                  << " boxes (guided + s-t bias + BFS) ...";
        auto tr0 = std::chrono::steady_clock::now();
        int before = forest_.n_boxes();
        int miss = 0;
        int max_miss_random = config_.max_consecutive_miss * 3;
        int n_dims = robot_.n_joints();
        const auto& limits = robot_.joint_limits().limits;
        double st_bias_ratio = 0.3;  // 30% of samples biased toward s-t endpoints
        std::uniform_real_distribution<double> unif01(0.0, 1.0);
        std::normal_distribution<double> gauss01(0.0, 1.0);

        // BFS expand queue for random growth phase
        struct RandExpandEntry {
            int box_id;
            std::set<std::pair<int,int>> excluded_faces;
            int depth;  // BFS depth from seed
        };
        std::deque<RandExpandEntry> rand_bfs_queue;
        int max_bfs_depth = 5;  // max BFS depth per random seed
        double rand_bfs_eps = config_.ffb_min_edge_relaxed * 0.5;  // boundary seed offset

        // Collect all s-t endpoints for biased sampling
        std::vector<Eigen::VectorXd> st_targets;
        for (const auto& [s, g] : pairs) {
            st_targets.push_back(s);
            st_targets.push_back(g);
        }

        while (miss < max_miss_random && elapsed() < timeout) {
            if (forest_.n_boxes() >= before + n_random_boxes) break;

            // ── Process BFS queue first ──
            if (!rand_bfs_queue.empty()) {
                auto entry = rand_bfs_queue.front();
                rand_bfs_queue.pop_front();

                if (forest_.n_boxes() >= before + n_random_boxes) break;
                auto box_it = forest_.boxes().find(entry.box_id);
                if (box_it == forest_.boxes().end()) continue;
                const BoxNode& parent_box = box_it->second;

                // Pick nearest s-t endpoint as goal for directed expansion
                Eigen::VectorXd box_center(n_dims);
                for (int d = 0; d < n_dims; ++d)
                    box_center[d] = 0.5 * (parent_box.joint_intervals[d].lo +
                                            parent_box.joint_intervals[d].hi);
                double best_dist = 1e18;
                const Eigen::VectorXd* goal_pt = nullptr;
                for (const auto& tgt : st_targets) {
                    double dd = (box_center - tgt).norm();
                    if (dd < best_dist) {
                        best_dist = dd;
                        goal_pt = &tgt;
                    }
                }

                auto seeds = generate_boundary_seeds(
                    parent_box, entry.excluded_faces,
                    goal_pt, config_.n_edge_samples, rand_bfs_eps);

                for (auto& [dim, side, q] : seeds) {
                    if (forest_.n_boxes() >= before + n_random_boxes) break;
                    if (tree_.is_occupied(q)) continue;

                    auto ffb = tree_.find_free_box(q, checker_.obs_compact(),
                                                    checker_.n_obs(),
                                                    config_.ffb_max_depth,
                                                    config_.ffb_min_edge_relaxed);
                    if (!ffb.success()) continue;

                    auto pr = tree_.try_promote(ffb.path,
                                                 checker_.obs_compact(), checker_.n_obs(), 2);
                    auto ivs = tree_.get_node_intervals(pr.result_idx);
                    int child_id = forest_.allocate_id();
                    BoxNode child_box(child_id, ivs, q);
                    forest_.add_box_no_adjacency(child_box);
                    tree_.mark_occupied(pr.result_idx, child_id);

                    if (!pr.absorbed_box_ids.empty()) {
                        std::unordered_set<int> abs_set(pr.absorbed_box_ids.begin(),
                                                         pr.absorbed_box_ids.end());
                        forest_.remove_boxes_no_adjacency(abs_set);
                    }

                    // Enqueue child for further BFS expansion
                    if (entry.depth + 1 < max_bfs_depth) {
                        std::set<std::pair<int,int>> new_excluded = entry.excluded_faces;
                        new_excluded.insert({dim, 1 - side});
                        rand_bfs_queue.push_back({child_id, new_excluded, entry.depth + 1});
                    }
                    miss = 0;
                }
                continue;
            }

            // ── Generate seed: guided + s-t bias ──
            Eigen::VectorXd q;
            if (unif01(rng_) < st_bias_ratio && !st_targets.empty()) {
                // s-t biased: pick a random s-t endpoint, sample in its vicinity
                int tgt_idx = static_cast<int>(unif01(rng_) * st_targets.size());
                tgt_idx = std::min(tgt_idx, static_cast<int>(st_targets.size()) - 1);
                const Eigen::VectorXd& tgt = st_targets[tgt_idx];
                double sigma = 0.3;
                q.resize(n_dims);
                for (int d = 0; d < n_dims; ++d) {
                    q[d] = tgt[d] + sigma * gauss01(rng_);
                    q[d] = std::clamp(q[d], limits[d].lo, limits[d].hi);
                }
            } else {
                // Guided (unoccupied KD-tree) or uniform
                q = sample_seed(Eigen::VectorXd());
            }

            if (tree_.is_occupied(q) || checker_.check_config(q)) {
                miss++;
                continue;
            }

            auto ffb = tree_.find_free_box(q, checker_.obs_compact(),
                                            checker_.n_obs(),
                                            config_.ffb_max_depth,
                                            config_.ffb_min_edge_relaxed);
            if (!ffb.success()) { miss++; continue; }

            auto pr = tree_.try_promote(ffb.path,
                                         checker_.obs_compact(), checker_.n_obs(), 2);
            auto ivs = tree_.get_node_intervals(pr.result_idx);

            int box_id = forest_.allocate_id();
            BoxNode box(box_id, ivs, q);
            forest_.add_box_no_adjacency(box);
            tree_.mark_occupied(pr.result_idx, box_id);

            if (!pr.absorbed_box_ids.empty()) {
                std::unordered_set<int> abs_set(pr.absorbed_box_ids.begin(),
                                                 pr.absorbed_box_ids.end());
                forest_.remove_boxes_no_adjacency(abs_set);
            }

            // Enqueue new box for BFS expansion
            rand_bfs_queue.push_back({box_id, {}, 0});
            miss = 0;
        }
        int after = forest_.n_boxes();
        double tr = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - tr0).count();
        std::cout << " done (+" << (after - before) << ", total="
                  << after << ", " << std::fixed << std::setprecision(2) << tr << "s)\n";
    }

    // ══════════════════════════════════════════════════════════════════
    // Phase 2.5: post-forest s-t attachment
    //   For each pair's s/g that is NOT yet contained by any forest box,
    //   find the nearest box and pave stepping-stone boxes to connect.
    //   This replaces the old proxy anchor approach — we wait until the
    //   forest is fully grown, then attach endpoints.
    // ══════════════════════════════════════════════════════════════════
    {
        auto tat0 = std::chrono::steady_clock::now();
        int n_attached = 0;
        int n_dims_att = robot_.n_joints();
        const auto& lim_att = robot_.joint_limits().limits;
        std::normal_distribution<double> att_noise(0.0, 0.05);

        for (int pi = 0; pi < n_pairs; ++pi) {
            for (int side = 0; side < 2; ++side) {
                const Eigen::VectorXd& q_target = (side == 0) ? pairs[pi].first : pairs[pi].second;
                const char* label = (side == 0) ? "s" : "g";

                // Check if q is already contained
                auto* contained = forest_.find_containing(q_target);
                if (contained) continue;

                // Also try proxy point from Phase 1
                const Eigen::VectorXd& q_proxy = (side == 0)
                    ? (pair_has_proxy_start[pi] ? pair_proxy_start[pi] : q_target)
                    : (pair_has_proxy_goal[pi]  ? pair_proxy_goal[pi]  : q_target);
                if (&q_proxy != &q_target) {
                    contained = forest_.find_containing(q_proxy);
                    if (contained) continue;
                }

                // Find nearest box to q_target
                const BoxNode* nearest = forest_.find_nearest(q_target);
                if (!nearest) continue;

                // Pave from nearest box center toward q_target
                Eigen::VectorXd near_center(n_dims_att);
                for (int d = 0; d < n_dims_att; ++d)
                    near_center[d] = 0.5 * (nearest->joint_intervals[d].lo +
                                             nearest->joint_intervals[d].hi);

                Eigen::VectorXd dir = q_target - near_center;
                double dist = dir.norm();
                if (dist < 1e-6) continue;
                dir /= dist;

                Eigen::VectorXd pos = near_center;
                int n_paved = 0;
                for (int step = 0; step < 300 && elapsed() < timeout; ++step) {
                    if ((q_target - pos).norm() < 0.03) break;

                    bool placed = false;
                    for (int att = 0; att < 15; ++att) {
                        Eigen::VectorXd q_try = pos;
                        if (att > 0)
                            for (int d = 0; d < n_dims_att; ++d)
                                q_try[d] += att_noise(rng_);
                        for (int d = 0; d < n_dims_att; ++d)
                            q_try[d] = std::clamp(q_try[d], lim_att[d].lo, lim_att[d].hi);

                        if (checker_.check_config(q_try)) continue;
                        if (tree_.is_occupied(q_try)) {
                            pos = pos + dir * 0.03;
                            placed = true;
                            break;
                        }

                        auto ffb = tree_.find_free_box(q_try, checker_.obs_compact(),
                                                        checker_.n_obs(),
                                                        config_.ffb_max_depth,
                                                        config_.ffb_min_edge);
                        if (!ffb.success()) continue;

                        auto pr = tree_.try_promote(ffb.path,
                                                     checker_.obs_compact(), checker_.n_obs(), 2);
                        auto ivs = tree_.get_node_intervals(pr.result_idx);
                        int bid = forest_.allocate_id();
                        BoxNode bx(bid, ivs, q_try);
                        forest_.add_box_direct(bx);
                        tree_.mark_occupied(pr.result_idx, bid);
                        if (!pr.absorbed_box_ids.empty()) {
                            std::unordered_set<int> abs_set(
                                pr.absorbed_box_ids.begin(),
                                pr.absorbed_box_ids.end());
                            forest_.remove_boxes(abs_set);
                        }
                        n_paved++;

                        const BoxNode& nb = forest_.boxes().at(bid);
                        double min_e = 1e18;
                        for (int d = 0; d < n_dims_att; ++d)
                            min_e = std::min(min_e,
                                              nb.joint_intervals[d].hi -
                                              nb.joint_intervals[d].lo);
                        pos = pos + dir * std::max(0.01, std::min(min_e * 0.4, 0.1));
                        placed = true;
                        break;
                    }
                    if (!placed) pos = pos + dir * 0.03;
                }

                // Also try to create a box directly at q_target (aggressive min_edge)
                if (!tree_.is_occupied(q_target) && !checker_.check_config(q_target)) {
                    auto ffb = tree_.find_free_box(q_target, checker_.obs_compact(),
                                                    checker_.n_obs(),
                                                    config_.ffb_max_depth,
                                                    config_.ffb_min_edge_anchor);
                    if (ffb.success()) {
                        auto pr = tree_.try_promote(ffb.path,
                                                     checker_.obs_compact(), checker_.n_obs(), 2);
                        auto ivs = tree_.get_node_intervals(pr.result_idx);
                        int bid = forest_.allocate_id();
                        BoxNode bx(bid, ivs, q_target);
                        forest_.add_box_direct(bx);
                        tree_.mark_occupied(pr.result_idx, bid);
                        if (!pr.absorbed_box_ids.empty()) {
                            std::unordered_set<int> abs_set(
                                pr.absorbed_box_ids.begin(),
                                pr.absorbed_box_ids.end());
                            forest_.remove_boxes(abs_set);
                        }
                        n_paved++;
                    }
                }

                if (n_paved > 0) {
                    n_attached++;
                    std::cout << "  [attach] pair " << pi << " " << label
                              << " paved " << n_paved << " boxes toward target\n";
                }
            }
        }
        double tat = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - tat0).count();
        if (n_attached > 0) {
            std::cout << "  [attach] " << n_attached << " endpoints connected, "
                      << forest_.n_boxes() << " total boxes, "
                      << std::fixed << std::setprecision(2) << tat << "s\n";
        }
    }

    // ══════════════════════════════════════════════════════════════════
    // Post-growth: coarsen + rebuild adjacency
    // ══════════════════════════════════════════════════════════════════
    {
        auto tc0 = std::chrono::steady_clock::now();
        coarsen_forest(forest_, checker_, config_.coarsen_max_rounds);
        double tc = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - tc0).count();
        std::cout << "  [coarsen] " << std::fixed << std::setprecision(2) << tc << "s\n";
    }

    // Greedy adjacency-based coarsen (if target set)
    if (config_.coarsen_target_boxes > 0) {
        std::cout << "  [greedy_coarsen] target=" << config_.coarsen_target_boxes
                  << " boxes, current=" << forest_.n_boxes() << "\n";
        auto gc = coarsen_greedy(forest_, checker_,
                                  config_.coarsen_target_boxes,
                                  config_.coarsen_greedy_rounds,
                                  config_.adjacency_tol,
                                  config_.coarsen_grid_check ? &tree_ : nullptr,
                                  config_.coarsen_split_depth,
                                  config_.coarsen_max_tree_fk);
        std::cout << "  [greedy_coarsen] done: " << gc.boxes_before
                  << " → " << gc.boxes_after << " boxes"
                  << " (" << gc.merges_performed << " merges, "
                  << gc.rounds << " rounds, "
                  << std::fixed << std::setprecision(2) << gc.elapsed_sec << "s)\n";
    }

    {
        auto ta0 = std::chrono::steady_clock::now();
        forest_.rebuild_adjacency(config_.adjacency_tol);
        double ta = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - ta0).count();
        std::cout << "  [adjacency] " << std::fixed << std::setprecision(2) << ta << "s\n";
    }

    // ══════════════════════════════════════════════════════════════════
    // Phase 3: bridge disconnected s-t pairs (only if needed)
    // ══════════════════════════════════════════════════════════════════
    bool any_disconnected = std::any_of(pair_connected.begin(),
                                         pair_connected.end(),
                                         [](bool c){ return !c; });
    if (any_disconnected) {
        auto tbp0 = std::chrono::steady_clock::now();
        // Find islands
        std::vector<int> all_ids;
        for (auto& [id, _] : forest_.boxes()) all_ids.push_back(id);
        auto islands = find_islands(forest_.adjacency(), all_ids);

        // Build box-id → island-index map
        std::unordered_map<int, int> box_island;
        for (int i = 0; i < static_cast<int>(islands.size()); ++i)
            for (int bid : islands[i])
                box_island[bid] = i;

        int n_bridged = 0;
        for (int pi = 0; pi < n_pairs; ++pi) {
            if (pair_connected[pi]) continue;
            const auto& [s, g] = pairs[pi];
            // Use proxy point when original s/g is not contained (narrow passage)
            const Eigen::VectorXd& s_lookup = pair_has_proxy_start[pi] ? pair_proxy_start[pi] : s;
            const Eigen::VectorXd& g_lookup = pair_has_proxy_goal[pi]  ? pair_proxy_goal[pi]  : g;
            auto* s_box = forest_.find_containing(s_lookup);
            auto* g_box = forest_.find_containing(g_lookup);
            if (!s_box) s_box = forest_.find_containing(s);  // fallback to original
            if (!g_box) g_box = forest_.find_containing(g);
            if (!s_box || !g_box) {
                std::cout << "  [bridge] pair " << pi
                          << ": cannot find containing boxes, skipping\n";
                continue;
            }

            int s_island = box_island.count(s_box->id) ? box_island[s_box->id] : -1;
            int g_island = box_island.count(g_box->id) ? box_island[g_box->id] : -1;
            if (s_island == g_island) continue;

            std::cout << "  [bridge] pair " << pi
                      << " disconnected, attempting bridge ...\n";

            Eigen::VectorXd s_center = s_box->seed_config;
            Eigen::VectorXd g_center = g_box->seed_config;
            Eigen::VectorXd dir = g_center - s_center;
            double dist = dir.norm();
            if (dist < 1e-6) continue;
            dir /= dist;

            int n_bridge = 0;
            Eigen::VectorXd pos = s_center;
            std::normal_distribution<double> noise(0.0, 0.05);
            int n_dims = robot_.n_joints();

            for (int step = 0; step < 500 && elapsed() < timeout; ++step) {
                if ((g_center - pos).norm() < 0.05) break;

                for (int att = 0; att < 15; ++att) {
                    Eigen::VectorXd q = pos;
                    if (att > 0)
                        for (int d = 0; d < n_dims; ++d)
                            q[d] += noise(rng_);
                    for (int d = 0; d < n_dims; ++d)
                        q[d] = std::clamp(q[d],
                                           robot_.joint_limits().limits[d].lo,
                                           robot_.joint_limits().limits[d].hi);

                    if (checker_.check_config(q)) continue;
                    if (tree_.is_occupied(q)) {
                        pos = pos + dir * 0.03;
                        break;
                    }

                    auto ffb = tree_.find_free_box(q, checker_.obs_compact(),
                                                    checker_.n_obs(),
                                                    config_.ffb_max_depth,
                                                    config_.ffb_min_edge);
                    if (!ffb.success()) continue;

                    auto pr = tree_.try_promote(ffb.path,
                                                 checker_.obs_compact(), checker_.n_obs(), 2);
                    auto ivs = tree_.get_node_intervals(pr.result_idx);
                    int bid = forest_.allocate_id();
                    BoxNode bx(bid, ivs, q);
                    forest_.add_box_direct(bx);  // incremental adjacency O(N)
                    tree_.mark_occupied(pr.result_idx, bid);
                    if (!pr.absorbed_box_ids.empty()) {
                        std::unordered_set<int> abs_set(
                            pr.absorbed_box_ids.begin(),
                            pr.absorbed_box_ids.end());
                        forest_.remove_boxes(abs_set);  // also cleans adjacency
                    }
                    n_bridge++;

                    const BoxNode& nb = forest_.boxes().at(bid);
                    double min_e = 1e18;
                    for (int d = 0; d < n_dims; ++d)
                        min_e = std::min(min_e,
                                          nb.joint_intervals[d].hi -
                                          nb.joint_intervals[d].lo);
                    pos = pos + dir * std::max(0.01, std::min(min_e * 0.4, 0.1));
                    break;
                }
            }
            n_bridged += n_bridge;
        }

        if (n_bridged > 0) {
            std::cout << "  [bridge] +" << n_bridged << " boxes, total="
                      << forest_.n_boxes() << "\n";
            // adjacency already maintained incrementally by add_box_direct
        }
        double tbp = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - tbp0).count();
        std::cout << "  [bridge paving] " << std::fixed << std::setprecision(2) << tbp << "s\n";
    }

    // Generic island bridging (skip if all pairs already connected — O(N²) cost)
    if (any_disconnected) {
        auto tb0 = std::chrono::steady_clock::now();
        std::vector<int> all_ids;
        for (auto& [id, _] : forest_.boxes()) all_ids.push_back(id);
        auto islands = find_islands(forest_.adjacency(), all_ids);
        if (islands.size() > 1) {
            auto bridge = bridge_islands(islands, forest_.boxes());
            for (auto& [a, b] : bridge.bridges) {
                forest_.adjacency()[a].push_back(b);
                forest_.adjacency()[b].push_back(a);
            }
        }
        double tb = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - tb0).count();
        std::cout << "  [island bridge] " << islands.size() << " islands, "
                  << std::fixed << std::setprecision(2) << tb << "s\n";
    }

    {
        auto tc0 = std::chrono::steady_clock::now();
        connector_ = TreeConnector(forest_, checker_);
        double tc = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - tc0).count();
        std::cout << "  [connector] " << std::fixed << std::setprecision(2) << tc << "s\n";
    }
    forest_built_ = true;
    {
        auto ti0 = std::chrono::steady_clock::now();
        forest_.rebuild_interval_cache();
        double ti = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - ti0).count();
        std::cout << "  [interval cache] " << std::fixed << std::setprecision(2) << ti << "s\n";
    }

    // Save KD-tree cache
    save_tree_cache();

    std::cout << "[build_multi] complete: " << forest_.n_boxes() << " boxes, "
              << std::fixed << std::setprecision(2) << elapsed() << "s\n";
}

PlanningResult SBFPlanner::query(const Eigen::VectorXd& start,
                                  const Eigen::VectorXd& goal,
                                  double timeout) {
    if (!forest_built_) return plan(start, goal, timeout);

    auto t0 = std::chrono::steady_clock::now();
    auto elapsed = [&t0]() {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
    };

    PlanningResult result;

    // Attach start/goal to nearest box (via find_containing or find_nearest)
    auto start_attach = connector_.attach_config(start);
    auto goal_attach  = connector_.attach_config(goal);

    if (start_attach.box_id < 0 || goal_attach.box_id < 0) {
        result.planning_time = elapsed();
        return result;
    }

    std::unordered_set<int> start_set{start_attach.box_id};
    std::unordered_set<int> goal_set{goal_attach.box_id};

    auto dijk = dijkstra_center_distance(
        forest_.adjacency(), forest_.boxes(), start_set, goal_set);

    if (!dijk.found) {
        result.planning_time = elapsed();
        return result;
    }

    auto waypoints = extract_waypoints(dijk.path, forest_.boxes(),
                                        start, goal);

    auto smoothed = smoother_.shortcut(waypoints, config_.shortcut_max_iters, &rng_);

    result.success = true;
    int n_wp = static_cast<int>(smoothed.size());
    result.path.resize(n_wp, start.size());
    for (int i = 0; i < n_wp; ++i)
        result.path.row(i) = smoothed[i].transpose();
    result.cost = PathSmoother::path_length(smoothed);
    result.planning_time = elapsed();
    return result;
}

void SBFPlanner::add_obstacle(const Obstacle& obs) {
    obstacles_.push_back(obs);

    // Only validate existing boxes against the NEWLY added obstacle.
    // Boxes safe w.r.t. unchanged obstacles remain valid.
    CollisionChecker single_checker(robot_, {obs});
    auto invalid = forest_.validate_boxes(single_checker);
    if (!invalid.empty()) {
        // Clear tree occupation marks before removing boxes from forest,
        // so that subsequent regrow() can fill the depleted areas.
        tree_.clear_boxes_occupation(invalid);
        forest_.remove_boxes(invalid);  // also cleans adjacency edges
    }

    // Rebuild full checker for subsequent operations (query, regrowth, etc.)
    checker_ = CollisionChecker(robot_, obstacles_);
}

void SBFPlanner::remove_obstacle(const std::string& name) {
    obstacles_.erase(
        std::remove_if(obstacles_.begin(), obstacles_.end(),
                       [&](const Obstacle& o) { return o.name == name; }),
        obstacles_.end());
    checker_ = CollisionChecker(robot_, obstacles_);
}

void SBFPlanner::clear_forest() {
    // Clear all tree occupation marks (keeps FK cache intact)
    tree_.clear_all_occupation();
    // Clear forest boxes and adjacency
    forest_.clear();
    forest_built_ = false;
}

int SBFPlanner::regrow(int n_target, double timeout) {
    auto t0 = std::chrono::steady_clock::now();
    auto elapsed = [&t0]() {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
    };

    int added = 0;
    int miss = 0;
    int max_miss = config_.max_consecutive_miss * 3;

    while (added < n_target && miss < max_miss && elapsed() < timeout) {
        // Tree-guided sampling: bias toward unoccupied (depleted) regions
        Eigen::VectorXd q;
        bool got_seed = tree_.sample_unoccupied_seed(rng_, q);
        if (!got_seed) {
            // Fallback to uniform sampling
            q = sample_uniform();
        }

        if (tree_.is_occupied(q) || checker_.check_config(q)) {
            miss++;
            continue;
        }

        auto ffb = tree_.find_free_box(q, checker_.obs_compact(),
                                        checker_.n_obs(),
                                        config_.ffb_max_depth,
                                        config_.ffb_min_edge_relaxed);
        if (!ffb.success()) { miss++; continue; }

        auto pr = tree_.try_promote(ffb.path,
                                     checker_.obs_compact(), checker_.n_obs(), 2);
        auto ivs = tree_.get_node_intervals(pr.result_idx);

        int box_id = forest_.allocate_id();
        BoxNode box(box_id, ivs, q);
        forest_.add_box_direct(box);  // incremental adjacency O(N)
        tree_.mark_occupied(pr.result_idx, box_id);

        if (!pr.absorbed_box_ids.empty()) {
            std::unordered_set<int> abs_set(pr.absorbed_box_ids.begin(),
                                             pr.absorbed_box_ids.end());
            forest_.remove_boxes(abs_set);  // also cleans adjacency
        }

        added++;
        miss = 0;
    }

    // Rebuild connector + interval cache for query readiness
    connector_ = TreeConnector(forest_, checker_);
    forest_.rebuild_interval_cache();

    return added;
}

// ─── Forest building ────────────────────────────────────────────────────────
void SBFPlanner::build_forest(const Eigen::VectorXd& start,
                               const Eigen::VectorXd& goal) {
    grow_boxes(start, goal);
    forest_built_ = true;
}

// ─── Helper: check if two boxes touch/overlap ───────────────────────────────
bool SBFPlanner::boxes_touch(const BoxNode& a, const BoxNode& b, double tol) const {
    int n = robot_.n_joints();
    for (int d = 0; d < n; ++d) {
        if (std::max(a.joint_intervals[d].lo, b.joint_intervals[d].lo)
            > std::min(a.joint_intervals[d].hi, b.joint_intervals[d].hi) + tol)
            return false;
    }
    return true;
}

// ─── Directed boundary seed generation (v4 logic) ───────────────────────────
std::vector<BoundarySeed> SBFPlanner::generate_boundary_seeds(
    const BoxNode& box,
    const std::set<std::pair<int,int>>& excluded_faces,
    const Eigen::VectorXd* goal_point,
    int n_samples,
    double eps_override)
{
    double eps = (eps_override > 0) ? eps_override : config_.boundary_expand_epsilon;
    int n_dims = robot_.n_joints();
    const auto& limits = robot_.joint_limits().limits;

    // 1. Enumerate valid faces (not excluded, within joint limits)
    std::vector<std::pair<int,int>> faces;  // (dim, side)
    for (int d = 0; d < n_dims; ++d) {
        double lo_d = box.joint_intervals[d].lo;
        double hi_d = box.joint_intervals[d].hi;
        if (excluded_faces.find({d, 0}) == excluded_faces.end()) {
            if (lo_d - eps >= limits[d].lo)
                faces.push_back({d, 0});
        }
        if (excluded_faces.find({d, 1}) == excluded_faces.end()) {
            if (hi_d + eps <= limits[d].hi)
                faces.push_back({d, 1});
        }
    }

    if (faces.empty()) return {};

    // 2. Face selection strategy
    std::vector<std::pair<int,int>> selected;
    if (n_samples <= 0 || n_samples >= static_cast<int>(faces.size())) {
        // All faces (BFS wavefront / post-connection random expansion)
        std::shuffle(faces.begin(), faces.end(), rng_);
        selected = faces;
    } else if (goal_point != nullptr) {
        // Goal-biased face selection
        Eigen::VectorXd box_center(n_dims);
        for (int d = 0; d < n_dims; ++d)
            box_center[d] = 0.5 * (box.joint_intervals[d].lo +
                                    box.joint_intervals[d].hi);
        Eigen::VectorXd direction = *goal_point - box_center;
        double dir_norm = direction.norm();
        if (dir_norm > 1e-12) direction /= dir_norm;

        // Score each face by dot product with direction
        struct ScoredFace {
            double score;
            int dim;
            int side;
        };
        std::vector<ScoredFace> scored;
        for (auto& [d, s] : faces) {
            double normal_sign = 2.0 * s - 1.0;
            double score = normal_sign * direction[d];
            scored.push_back({score, d, s});
        }
        std::sort(scored.begin(), scored.end(),
                  [](const ScoredFace& a, const ScoredFace& b) {
                      return a.score > b.score;
                  });

        // Best 1 goal-biased + (n_samples-1) random
        selected.push_back({scored[0].dim, scored[0].side});
        std::vector<std::pair<int,int>> rest;
        for (size_t i = 1; i < scored.size(); ++i)
            rest.push_back({scored[i].dim, scored[i].side});
        std::shuffle(rest.begin(), rest.end(), rng_);
        for (int i = 0; i < n_samples - 1 && i < static_cast<int>(rest.size()); ++i)
            selected.push_back(rest[i]);
    } else {
        // Pure random
        std::shuffle(faces.begin(), faces.end(), rng_);
        selected.assign(faces.begin(), faces.begin() + std::min(n_samples,
                         static_cast<int>(faces.size())));
    }

    // 3. Generate seed config for each selected face
    std::vector<BoundarySeed> results;
    for (auto& [dim, side] : selected) {
        double lo_d = box.joint_intervals[dim].lo;
        double hi_d = box.joint_intervals[dim].hi;

        double target_val;
        if (side == 0) {
            target_val = lo_d - eps;
            if (target_val < limits[dim].lo) continue;
        } else {
            target_val = hi_d + eps;
            if (target_val > limits[dim].hi) continue;
        }

        Eigen::VectorXd q(n_dims);
        for (int d = 0; d < n_dims; ++d) {
            if (d == dim) {
                q[d] = target_val;
            } else {
                // Face center point
                double b_lo = box.joint_intervals[d].lo;
                double b_hi = box.joint_intervals[d].hi;
                q[d] = (b_hi > b_lo) ? 0.5 * (b_lo + b_hi) : b_lo;
            }
        }

        // Clamp to joint limits
        for (int d = 0; d < n_dims; ++d)
            q[d] = std::clamp(q[d], limits[d].lo, limits[d].hi);

        results.push_back({dim, side, q});
    }
    return results;
}

// ═══════════════════════════════════════════════════════════════════════════
// try_create_proxy_anchor: create an anchor box for a start/goal config.
//   Step A: try FFB with normal ffb_min_edge at q.
//   Step B: if that fails, sample nearby configs reachable from q via
//           collision-free straight-line, and use the first one where FFB
//           with ffb_min_edge succeeds.  Record the proxy config.
//   Step C: if all proxy samples fail, fall back to ffb_min_edge_anchor
//           at q (the old aggressive behavior).
// Returns box_id (>=0) on success, -1 on total failure.
// ═══════════════════════════════════════════════════════════════════════════
int SBFPlanner::try_create_proxy_anchor(
    const Eigen::VectorXd& q, char origin_tag,
    std::unordered_map<int,char>& box_origin,
    std::unordered_set<int>& start_box_ids,
    std::unordered_set<int>& goal_box_ids,
    std::unordered_set<int>& random_box_ids,
    std::unordered_map<int,int>& random_ancestor,
    Eigen::VectorXd& proxy_out, bool& used_proxy)
{
    used_proxy = false;
    int n_dims = robot_.n_joints();
    const auto& limits = robot_.joint_limits().limits;

    // Helper lambda: create a box at seed, return box_id or -1.
    auto create_box_at = [&](const Eigen::VectorXd& seed,
                             double min_edge) -> int {
        if (tree_.is_occupied(seed)) return -2;  // occupied → not an error per se

        auto ffb = tree_.find_free_box(seed, checker_.obs_compact(), checker_.n_obs(),
                                       config_.ffb_max_depth, min_edge);
        if (!ffb.success()) return -1;

        auto pr = tree_.try_promote(ffb.path,
                                     checker_.obs_compact(), checker_.n_obs(), 2);
        auto ivs = tree_.get_node_intervals(pr.result_idx);
        int bid = forest_.allocate_id();
        BoxNode box(bid, ivs, seed);
        forest_.add_box_no_adjacency(box);
        tree_.mark_occupied(pr.result_idx, bid);

        if (!pr.absorbed_box_ids.empty()) {
            std::unordered_set<int> absorbed_set(pr.absorbed_box_ids.begin(),
                                                  pr.absorbed_box_ids.end());
            for (int aid : absorbed_set) {
                box_origin.erase(aid);
                start_box_ids.erase(aid);
                goal_box_ids.erase(aid);
                random_box_ids.erase(aid);
                random_ancestor.erase(aid);
            }
            forest_.remove_boxes_no_adjacency(absorbed_set);
        }
        return bid;
    };

    // Helper lambda: register box origin tracking.
    auto register_origin = [&](int bid) {
        box_origin[bid] = origin_tag;
        if (origin_tag == 's')
            start_box_ids.insert(bid);
        else
            goal_box_ids.insert(bid);
    };

    // ── Step A: Try normal FFB at q ──
    {
        int bid = create_box_at(q, config_.ffb_min_edge);
        if (bid == -2) {
            // Already occupied — register existing containing box
            auto* bx = forest_.find_containing(q);
            if (bx) { register_origin(bx->id); return bx->id; }
            return -1;
        }
        if (bid >= 0) {
            register_origin(bid);
            std::cout << "    [anchor] " << origin_tag
                      << " normal FFB ok (box " << bid << ")\n";
            return bid;
        }
    }

    // ── Step B: Proxy sampling — q is in a narrow passage ──
    std::cout << "    [anchor] " << origin_tag
              << " normal FFB failed, trying proxy sampling ...\n";

    // Layered radii: try close proxies first
    const double radii[] = {0.05, 0.1, 0.2, config_.proxy_anchor_radius};
    int samples_per_layer = config_.proxy_anchor_max_samples / 4;
    std::normal_distribution<double> norm01(0.0, 1.0);

    for (double radius : radii) {
        for (int s = 0; s < samples_per_layer; ++s) {
            // Gaussian sample around q, clamp to joint limits
            Eigen::VectorXd q_proxy(n_dims);
            double sigma = radius / 3.0;  // 3-sigma ≈ radius
            for (int d = 0; d < n_dims; ++d) {
                q_proxy[d] = q[d] + sigma * norm01(rng_);
                q_proxy[d] = std::clamp(q_proxy[d], limits[d].lo, limits[d].hi);
            }

            // Must be collision-free
            if (checker_.check_config(q_proxy)) continue;
            // Must be directly reachable from q
            if (checker_.check_segment(q, q_proxy, config_.segment_resolution))
                continue;
            // Must not already be occupied
            if (tree_.is_occupied(q_proxy)) continue;

            // Try normal FFB
            int bid = create_box_at(q_proxy, config_.ffb_min_edge);
            if (bid < 0) continue;

            // Success — record proxy
            proxy_out = q_proxy;
            used_proxy = true;
            register_origin(bid);
            std::cout << "    [anchor] " << origin_tag
                      << " PROXY found (box " << bid
                      << ", dist=" << std::fixed << std::setprecision(4)
                      << (q_proxy - q).norm()
                      << ", radius_layer=" << radius << ")\n";
            return bid;
        }
    }

    // ── Step C: Fallback — aggressive FFB at original q ──
    std::cout << "    [anchor] " << origin_tag
              << " proxy sampling failed, fallback to min_edge_anchor\n";
    {
        int bid = create_box_at(q, config_.ffb_min_edge_anchor);
        if (bid >= 0) {
            register_origin(bid);
            return bid;
        }
    }
    std::cout << "    [anchor] " << origin_tag << " ALL FAILED\n";
    return -1;
}

// ═══════════════════════════════════════════════════════════════════════════
// grow_boxes: v4 logic — bidirectional tree growth with origin tracking,
// directed boundary expansion, connection detection, absorption
// ═══════════════════════════════════════════════════════════════════════════
bool SBFPlanner::grow_boxes(const Eigen::VectorXd& start,
                             const Eigen::VectorXd& goal,
                             bool skip_final_adjacency) {
    int n_dims = robot_.n_joints();
    int max_boxes = config_.max_boxes;
    int max_miss = config_.max_consecutive_miss;
    int n_edge_samples = config_.n_edge_samples;
    double adj_tol = 1e-8;

    // ── Origin tracking ──
    std::unordered_map<int, char> box_origin;  // nid → 's'|'g'|'r'
    std::unordered_set<int> start_box_ids;
    std::unordered_set<int> goal_box_ids;
    std::unordered_set<int> random_box_ids;
    std::unordered_map<int, int> random_ancestor;  // nid → root random ancestor
    bool connected = false;

    // ── Lambda: check if new box connects trees ──
    auto check_box_connection = [&](const BoxNode& new_box, char origin) -> char {
        // Returns: 'C'=connected, 'S'=random touched start, 'G'=random touched goal, 0=none
        if (origin == 's') {
            for (int bid : goal_box_ids) {
                auto it = forest_.boxes().find(bid);
                if (it != forest_.boxes().end() && boxes_touch(new_box, it->second, adj_tol))
                    return 'C';
            }
        } else if (origin == 'g') {
            for (int bid : start_box_ids) {
                auto it = forest_.boxes().find(bid);
                if (it != forest_.boxes().end() && boxes_touch(new_box, it->second, adj_tol))
                    return 'C';
            }
        } else {
            for (int bid : start_box_ids) {
                auto it = forest_.boxes().find(bid);
                if (it != forest_.boxes().end() && boxes_touch(new_box, it->second, adj_tol))
                    return 'S';
            }
            for (int bid : goal_box_ids) {
                auto it = forest_.boxes().find(bid);
                if (it != forest_.boxes().end() && boxes_touch(new_box, it->second, adj_tol))
                    return 'G';
            }
        }
        return 0;
    };

    // ── Lambda: absorb random tree into start/goal ──
    auto absorb_random_tree = [&](int root_anc_id, char target_origin) {
        std::vector<int> to_flip;
        for (auto& [nid, anc] : random_ancestor) {
            if (anc == root_anc_id)
                to_flip.push_back(nid);
        }
        auto& target_set = (target_origin == 's') ? start_box_ids : goal_box_ids;
        for (int nid : to_flip) {
            box_origin[nid] = target_origin;
            target_set.insert(nid);
            random_box_ids.erase(nid);
            random_ancestor.erase(nid);
        }
        // Check if absorption bridges start↔goal
        if (!connected) {
            auto& opposite = (target_origin == 's') ? goal_box_ids : start_box_ids;
            for (int nid : to_flip) {
                auto it = forest_.boxes().find(nid);
                if (it == forest_.boxes().end()) continue;
                for (int obid : opposite) {
                    auto oit = forest_.boxes().find(obid);
                    if (oit != forest_.boxes().end() && boxes_touch(it->second, oit->second, adj_tol)) {
                        connected = true;
                        std::cout << "    [grow] CONNECTED (absorption bridge) at "
                                  << forest_.n_boxes() << " boxes\n";
                        return;
                    }
                }
            }
        }
    };

    // ── Lambda: check if seed is inside opposite tree ──
    auto seed_in_opposite_tree = [&](const Eigen::VectorXd& q, char parent_origin) -> bool {
        if (parent_origin != 's' && parent_origin != 'g') return false;
        auto& opposite = (parent_origin == 's') ? goal_box_ids : start_box_ids;
        for (int bid : opposite) {
            auto it = forest_.boxes().find(bid);
            if (it == forest_.boxes().end()) continue;
            const BoxNode& b = it->second;
            bool inside = true;
            for (int d = 0; d < n_dims; ++d) {
                if (q[d] < b.joint_intervals[d].lo - 1e-12 ||
                    q[d] > b.joint_intervals[d].hi + 1e-12) {
                    inside = false;
                    break;
                }
            }
            if (inside) return true;
        }
        return false;
    };

    // ── Lambda: create a box from config ──
    auto try_create_box = [&](const Eigen::VectorXd& q, double min_edge_override = -1.0) -> int {
        // Returns box_id or -1 on failure
        double me = (min_edge_override > 0) ? min_edge_override : config_.ffb_min_edge;
        auto ffb = tree_.find_free_box(q, checker_.obs_compact(), checker_.n_obs(),
                                        config_.ffb_max_depth, me);
        if (!ffb.success()) return -1;

        // v4 promotion: use path, absorb occupied subtrees
        auto pr = tree_.try_promote(ffb.path,
                                     checker_.obs_compact(), checker_.n_obs(), 2);
        auto ivs = tree_.get_node_intervals(pr.result_idx);

        int box_id = forest_.allocate_id();
        BoxNode box(box_id, ivs, q);
        forest_.add_box_no_adjacency(box);
        tree_.mark_occupied(pr.result_idx, box_id);

        // Remove absorbed boxes from forest
        if (!pr.absorbed_box_ids.empty()) {
            std::unordered_set<int> absorbed_set(pr.absorbed_box_ids.begin(),
                                                  pr.absorbed_box_ids.end());
            // Also remove from origin tracking
            for (int aid : absorbed_set) {
                box_origin.erase(aid);
                start_box_ids.erase(aid);
                goal_box_ids.erase(aid);
                random_box_ids.erase(aid);
                random_ancestor.erase(aid);
            }
            forest_.remove_boxes_no_adjacency(absorbed_set);
        }

        return box_id;
    };

    // ═══════════════════════════════════════════════════════════════════
    // Phase 1: Create anchor boxes for start and goal (with proxy fallback)
    // ═══════════════════════════════════════════════════════════════════
    has_proxy_start_ = false;
    has_proxy_goal_ = false;

    for (int idx = 0; idx < 2; ++idx) {
        const Eigen::VectorXd& qs = (idx == 0) ? start : goal;
        char origin_tag = (idx == 0) ? 's' : 'g';

        if (tree_.is_occupied(qs)) {
            // Already occupied from a previous grow_boxes call (build_multi).
            auto* bx = forest_.find_containing(qs);
            if (bx) {
                box_origin[bx->id] = origin_tag;
                if (origin_tag == 's')
                    start_box_ids.insert(bx->id);
                else
                    goal_box_ids.insert(bx->id);
            }
        } else {
            Eigen::VectorXd proxy;
            bool used_proxy = false;
            int bid = try_create_proxy_anchor(
                qs, origin_tag,
                box_origin, start_box_ids, goal_box_ids,
                random_box_ids, random_ancestor,
                proxy, used_proxy);

            if (bid >= 0 && used_proxy) {
                if (idx == 0) {
                    proxy_start_ = proxy;
                    has_proxy_start_ = true;
                } else {
                    proxy_goal_ = proxy;
                    has_proxy_goal_ = true;
                }
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════════
    // Phase 2: BFS expand queue — seed all existing boxes
    // ═══════════════════════════════════════════════════════════════════
    struct ExpandEntry {
        int box_id;
        std::set<std::pair<int,int>> excluded_faces;
    };
    std::deque<ExpandEntry> expand_queue;

    for (auto& [bid, bx] : forest_.boxes()) {
        expand_queue.push_back({bid, {}});
    }

    // ═══════════════════════════════════════════════════════════════════
    // Phase 3: Main growth loop
    // ═══════════════════════════════════════════════════════════════════
    int consec = 0;

    // ── Multi-phase min_edge decay tracking ──
    const auto& phase_k = config_.bfs_phase_k;
    const auto& phase_budget = config_.bfs_phase_budget;
    int n_phases = static_cast<int>(std::min(phase_k.size(), phase_budget.size()));
    int phase_idx = 0;        // current phase index
    int phase_boxes = 0;      // successful box creations in current phase

    // Helper: get effective min_edge for current phase
    auto get_phase_min_edge = [&]() -> double {
        if (connected) return config_.ffb_min_edge_relaxed;
        if (phase_idx < n_phases)
            return config_.ffb_min_edge * phase_k[phase_idx];
        return config_.ffb_min_edge;  // all phases exhausted → base k=1
    };

    // Helper: advance phase if budget consumed — clears BFS queue and
    // re-seeds from s/g trees, prioritized by distance to opposing target
    auto advance_phase = [&]() {
        if (connected) return;
        if (phase_idx >= n_phases) return;
        if (phase_boxes >= phase_budget[phase_idx]) {
            double old_k = phase_k[phase_idx];
            phase_idx++;
            phase_boxes = 0;

            // Collect candidate boxes with priority (distance to opposing target)
            struct PriEntry {
                double dist;   // smaller = higher priority
                int box_id;
            };
            std::vector<PriEntry> candidates;

            // s-tree boxes: priority = distance to goal
            for (int bid : start_box_ids) {
                auto it = forest_.boxes().find(bid);
                if (it == forest_.boxes().end()) continue;
                Eigen::VectorXd c(n_dims);
                for (int d = 0; d < n_dims; ++d)
                    c[d] = 0.5 * (it->second.joint_intervals[d].lo +
                                  it->second.joint_intervals[d].hi);
                candidates.push_back({(c - goal).norm(), bid});
            }
            // g-tree boxes: priority = distance to start
            for (int bid : goal_box_ids) {
                auto it = forest_.boxes().find(bid);
                if (it == forest_.boxes().end()) continue;
                Eigen::VectorXd c(n_dims);
                for (int d = 0; d < n_dims; ++d)
                    c[d] = 0.5 * (it->second.joint_intervals[d].lo +
                                  it->second.joint_intervals[d].hi);
                candidates.push_back({(c - start).norm(), bid});
            }

            // Sort by distance (closest to opposing target first)
            std::sort(candidates.begin(), candidates.end(),
                      [](const PriEntry& a, const PriEntry& b) {
                          return a.dist < b.dist;
                      });

            // Enqueue top candidates (cap to avoid huge queues)
            expand_queue.clear();
            int max_enqueue = std::max(50, static_cast<int>(candidates.size() / 4));
            int n_enqueued = std::min(max_enqueue, static_cast<int>(candidates.size()));
            for (int i = 0; i < n_enqueued; ++i)
                expand_queue.push_back({candidates[i].box_id, {}});

            if (phase_idx < n_phases) {
                std::cout << "    [grow] phase " << (phase_idx - 1)
                          << " (k=" << old_k << ") → phase " << phase_idx
                          << " (k=" << phase_k[phase_idx] << ") at "
                          << forest_.n_boxes() << " boxes, queue="
                          << n_enqueued << "/" << candidates.size() << "\n";
            } else {
                std::cout << "    [grow] all phases exhausted, using base min_edge="
                          << config_.ffb_min_edge << " at "
                          << forest_.n_boxes() << " boxes, queue="
                          << n_enqueued << "/" << candidates.size() << "\n";
            }
        }
    };

    while (consec < max_miss) {
        if (forest_.n_boxes() >= max_boxes) break;

        // ── BFS boundary expansion (priority over random sampling) ──
        if (!expand_queue.empty()) {
            auto entry = expand_queue.front();
            expand_queue.pop_front();

            auto box_it = forest_.boxes().find(entry.box_id);
            if (box_it == forest_.boxes().end()) continue;
            const BoxNode& parent_box = box_it->second;
            char parent_origin = box_origin.count(entry.box_id)
                                 ? box_origin[entry.box_id] : 'r';

            // Determine goal point for directed expansion
            const Eigen::VectorXd* goal_pt = nullptr;
            int n_samp = 0;
            if (!connected) {
                n_samp = n_edge_samples;
                if (parent_origin == 's')
                    goal_pt = &goal;
                else if (parent_origin == 'g')
                    goal_pt = &start;
                else {
                    // Random origin: pick closer of start/goal
                    Eigen::VectorXd center(n_dims);
                    for (int d = 0; d < n_dims; ++d)
                        center[d] = 0.5 * (parent_box.joint_intervals[d].lo +
                                            parent_box.joint_intervals[d].hi);
                    double d_s = (center - start).norm();
                    double d_g = (center - goal).norm();
                    goal_pt = (d_s < d_g) ? &start : &goal;
                }
            }
            // post-connection: goal_pt=nullptr, n_samp=0 → all faces, no bias

            // Phase-aware min_edge and eps
            double effective_me = get_phase_min_edge();
            double effective_eps = effective_me * 0.5;

            auto seeds = generate_boundary_seeds(parent_box, entry.excluded_faces,
                                                  goal_pt, n_samp, effective_eps);

            for (auto& [dim, side, q] : seeds) {
                if (forest_.n_boxes() >= max_boxes) break;

                // Check if seed is in occupied region
                if (tree_.is_occupied(q)) {
                    // Connection detection via occupied seed
                    if (!connected && seed_in_opposite_tree(q, parent_origin)) {
                        connected = true;
                        std::cout << "    [grow] CONNECTED (seed overlap) at "
                                  << forest_.n_boxes() << " boxes\n";
                    }
                    continue;
                }

                // Try to create a box at this seed
                int child_id = try_create_box(q, effective_me);
                if (child_id < 0) continue;

                // Phase tracking
                phase_boxes++;
                advance_phase();

                // Track origin
                box_origin[child_id] = parent_origin;
                if (parent_origin == 's')
                    start_box_ids.insert(child_id);
                else if (parent_origin == 'g')
                    goal_box_ids.insert(child_id);
                else {
                    random_box_ids.insert(child_id);
                    random_ancestor[child_id] = random_ancestor.count(entry.box_id)
                        ? random_ancestor[entry.box_id] : entry.box_id;
                }

                // Connection detection via new box
                if (!connected) {
                    char conn = check_box_connection(
                        forest_.boxes().at(child_id), parent_origin);
                    if (conn == 'C') {
                        connected = true;
                        std::cout << "    [grow] CONNECTED (box overlap) at "
                                  << forest_.n_boxes() << " boxes\n";
                    } else if (conn == 'S' || conn == 'G') {
                        char target = (conn == 'S') ? 's' : 'g';
                        int anc = random_ancestor.count(child_id)
                                  ? random_ancestor[child_id] : child_id;
                        absorb_random_tree(anc, target);
                    }
                }

                // Add child to expand queue (exclude opposite face)
                std::set<std::pair<int,int>> new_excluded = entry.excluded_faces;
                new_excluded.insert({dim, 1 - side});
                expand_queue.push_back({child_id, new_excluded});
                consec = 0;
            }
            continue;
        }

        // ── Random sampling fallback (guided/goal-biased/uniform) ──
        Eigen::VectorXd q = sample_seed(goal);

        // Skip if already occupied
        if (tree_.is_occupied(q)) {
            consec++;
            continue;
        }

        // Skip if collision
        if (checker_.check_config(q)) {
            consec++;
            continue;
        }

        // Try to create a box (phase-aware min_edge for random sampling too)
        double rand_me = get_phase_min_edge();
        int new_id = try_create_box(q, rand_me);
        if (new_id < 0) {
            consec++;
            continue;
        }

        // Phase tracking
        phase_boxes++;
        advance_phase();

        // Track as random origin
        box_origin[new_id] = 'r';
        random_box_ids.insert(new_id);
        random_ancestor[new_id] = new_id;

        // Check if random box connects to start/goal tree
        if (!connected) {
            char conn = check_box_connection(
                forest_.boxes().at(new_id), 'r');
            if (conn == 'S' || conn == 'G') {
                char target = (conn == 'S') ? 's' : 'g';
                absorb_random_tree(new_id, target);
            }
        }

        // Add to expand queue for boundary expansion
        expand_queue.push_back({new_id, {}});
        consec = 0;
    }

    std::cout << "    [grow] terminated: " << forest_.n_boxes() << " boxes ("
              << (connected ? "connected" : "disconnected") << "), "
              << "start=" << start_box_ids.size()
              << " goal=" << goal_box_ids.size()
              << " random=" << random_box_ids.size() << "\n";

    // ═══════════════════════════════════════════════════════════════════
    // Phase 4: Gap filling — bidirectional stepping stone paving
    //   (skipped when called from build_multi, which has its own bridge)
    // ═══════════════════════════════════════════════════════════════════
    if (!connected && !skip_final_adjacency) {
        int gap_rounds = 3;
        int n_gap_fill = 0;

        for (int gr = 0; gr < gap_rounds; ++gr) {
            // Rebuild adjacency to check connectivity
            forest_.rebuild_adjacency(config_.adjacency_tol);

            // Find start/goal boxes
            int src_bid = -1, tgt_bid = -1;
            for (auto& [bid, bx] : forest_.boxes()) {
                if (src_bid < 0) {
                    bool contains = true;
                    for (int d = 0; d < n_dims; ++d) {
                        if (start[d] < bx.joint_intervals[d].lo - 1e-12 ||
                            start[d] > bx.joint_intervals[d].hi + 1e-12) {
                            contains = false;
                            break;
                        }
                    }
                    if (contains) src_bid = bid;
                }
                if (tgt_bid < 0) {
                    bool contains = true;
                    for (int d = 0; d < n_dims; ++d) {
                        if (goal[d] < bx.joint_intervals[d].lo - 1e-12 ||
                            goal[d] > bx.joint_intervals[d].hi + 1e-12) {
                            contains = false;
                            break;
                        }
                    }
                    if (contains) tgt_bid = bid;
                }
            }

            if (src_bid < 0 || tgt_bid < 0) break;

            // Check if already connected
            std::vector<int> all_ids;
            for (auto& [id, _] : forest_.boxes()) all_ids.push_back(id);
            auto islands = find_islands(forest_.adjacency(), all_ids);

            // Find which islands contain start/goal
            int src_island = -1, tgt_island = -1;
            for (int i = 0; i < static_cast<int>(islands.size()); ++i) {
                for (int bid : islands[i]) {
                    if (bid == src_bid) src_island = i;
                    if (bid == tgt_bid) tgt_island = i;
                }
            }
            if (src_island == tgt_island) break;  // already connected

            // For start and goal, pave toward nearest box in other island
            for (int side = 0; side < 2; ++side) {
                int st_bid = (side == 0) ? src_bid : tgt_bid;
                int st_island_idx = (side == 0) ? src_island : tgt_island;

                // Find center of start/goal box
                const BoxNode& st_box = forest_.boxes().at(st_bid);
                Eigen::VectorXd st_center(n_dims);
                for (int d = 0; d < n_dims; ++d)
                    st_center[d] = 0.5 * (st_box.joint_intervals[d].lo +
                                           st_box.joint_intervals[d].hi);

                // Find nearest box in other island
                double best_d = 1e18;
                Eigen::VectorXd tgt_center(n_dims);
                for (auto& [bid, bx] : forest_.boxes()) {
                    if (st_island_idx >= 0 &&
                        std::find(islands[st_island_idx].begin(),
                                  islands[st_island_idx].end(), bid)
                        != islands[st_island_idx].end())
                        continue;

                    Eigen::VectorXd mc(n_dims);
                    for (int d = 0; d < n_dims; ++d)
                        mc[d] = 0.5 * (bx.joint_intervals[d].lo + bx.joint_intervals[d].hi);
                    double dd = (mc - st_center).norm();
                    if (dd < best_d) {
                        best_d = dd;
                        tgt_center = mc;
                    }
                }

                if (best_d >= 1e17) continue;

                // Pave stepping stones from st_center toward tgt_center
                Eigen::VectorXd direction = tgt_center - st_center;
                double dist = direction.norm();
                if (dist < 1e-6) continue;
                direction /= dist;

                Eigen::VectorXd pos = st_center;
                std::normal_distribution<double> noise_dist(0.0, 0.05);

                for (int step = 0; step < 300; ++step) {
                    double d_remain = (tgt_center - pos).norm();
                    if (d_remain < 0.05) break;

                    bool placed = false;
                    for (int attempt = 0; attempt < 15; ++attempt) {
                        Eigen::VectorXd q_try = pos;
                        if (attempt > 0) {
                            for (int d = 0; d < n_dims; ++d)
                                q_try[d] += noise_dist(rng_);
                        }
                        for (int d = 0; d < n_dims; ++d)
                            q_try[d] = std::clamp(q_try[d],
                                                   robot_.joint_limits().limits[d].lo,
                                                   robot_.joint_limits().limits[d].hi);

                        if (checker_.check_config(q_try)) continue;
                        if (tree_.is_occupied(q_try)) {
                            pos = pos + direction * 0.03;
                            placed = true;
                            break;
                        }

                        int gap_bid = try_create_box(q_try);
                        if (gap_bid >= 0) {
                            n_gap_fill++;
                            const BoxNode& gap_box = forest_.boxes().at(gap_bid);
                            double min_edge = 1e18;
                            for (int d = 0; d < n_dims; ++d)
                                min_edge = std::min(min_edge,
                                                     gap_box.joint_intervals[d].hi -
                                                     gap_box.joint_intervals[d].lo);
                            double step_size = std::max(0.01,
                                                         std::min(min_edge * 0.4, 0.1));
                            pos = pos + direction * step_size;
                            placed = true;
                            break;
                        }
                    }
                    if (!placed) pos = pos + direction * 0.03;
                }
            }
        }

        if (n_gap_fill > 0) {
            std::cout << "    [gap fill] +" << n_gap_fill << " boxes, "
                      << "total=" << forest_.n_boxes() << "\n";
        }
    }

    // Build adjacency (skip if caller will do it)
    if (!skip_final_adjacency) {
        forest_.rebuild_adjacency(config_.adjacency_tol);
    }
    return connected;
}

// ─── Seed sampling (v4 logic: guided + uniform) ───────────────
Eigen::VectorXd SBFPlanner::sample_seed(const Eigen::VectorXd& goal) {
    (void)goal;
    std::uniform_real_distribution<double> unif(0.0, 1.0);
    double r = unif(rng_);

    if (r < config_.guided_sample_ratio) {
        // Guided: sample from unoccupied tree regions
        Eigen::VectorXd q;
        if (tree_.sample_unoccupied_seed(rng_, q))
            return q;
        // Fallback to uniform if tree is saturated
        return sample_uniform();
    }
    else
        return sample_uniform();
}

Eigen::VectorXd SBFPlanner::sample_uniform() {
    int n = robot_.n_joints();
    Eigen::VectorXd q(n);
    std::uniform_real_distribution<double> unif(0.0, 1.0);
    for (int d = 0; d < n; ++d) {
        auto& lim = robot_.joint_limits().limits[d];
        q[d] = lim.lo + unif(rng_) * lim.width();
    }
    return q;
}

void SBFPlanner::expand_boundaries() {
    // Handled by BFS expand queue in grow_boxes()
}

void SBFPlanner::coarsen_and_connect() {
    sbf::coarsen_forest(forest_, checker_, config_.coarsen_max_rounds);
    forest_.rebuild_adjacency(config_.adjacency_tol);
}

void SBFPlanner::save_tree_cache() {
    if (!config_.use_cache || config_.cache_path.empty()) return;

    try {
        int n_nodes = tree_.store().next_idx();
        int fk = tree_.total_fk_calls();

        // mmap-backed: just flush (writes are already in the mapped region)
        tree_.flush_mmap();

        std::cout << "[SBFPlanner] flushed tree cache: "
                  << config_.cache_path
                  << " (" << n_nodes << " nodes, "
                  << fk << " fk_calls)\n";
    } catch (const std::exception& e) {
        std::cerr << "[SBFPlanner] cache flush failed: " << e.what() << "\n";
    }
}

} // namespace sbf

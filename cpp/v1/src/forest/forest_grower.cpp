// SafeBoxForest — ForestGrower implementation
// Extracted from SBFPlanner::grow_boxes() — bidirectional BFS growth
// with origin tracking, directed boundary expansion, connection detection,
// absorption, and gap-filling.
#include "sbf/forest/forest_grower.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <iostream>
#include <set>

namespace sbf {
namespace forest {

ForestGrower::ForestGrower(const Robot& robot,
                           const CollisionChecker& checker,
                           HierAABBTree& tree,
                           SafeBoxForest& forest,
                           const SBFConfig& config)
    : robot_(&robot), checker_(&checker), tree_(&tree),
      forest_(&forest), config_(config),
      root_sampler_(robot, tree, config),
      boundary_sampler_(robot, config),
      ffb_engine_(tree, config.forest_config())
{}

bool ForestGrower::boxes_touch(const BoxNode& a, const BoxNode& b, double tol) const {
    int n = robot_->n_joints();
    for (int d = 0; d < n; ++d) {
        if (std::max(a.joint_intervals[d].lo, b.joint_intervals[d].lo)
            > std::min(a.joint_intervals[d].hi, b.joint_intervals[d].hi) + tol)
            return false;
    }
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// grow: single-pair bidirectional BFS growth
// ═══════════════════════════════════════════════════════════════════════════
bool ForestGrower::grow(const Eigen::VectorXd& start,
                        const Eigen::VectorXd& goal)
{
    int n_dims = robot_->n_joints();
    int max_boxes = config_.max_boxes;
    int max_miss = config_.max_consecutive_miss;
    int n_edge_samples = config_.n_edge_samples;
    double adj_tol = 1e-8;

    // Reset origin tracking
    box_origin_.clear();
    start_box_ids_.clear();
    goal_box_ids_.clear();
    random_box_ids_.clear();

    std::unordered_map<int, int> random_ancestor;
    bool connected = false;

    // ── Lambda: check if new box connects trees ──
    auto check_box_connection = [&](const BoxNode& new_box, char origin) -> char {
        if (origin == 's') {
            for (int bid : goal_box_ids_) {
                auto it = forest_->boxes().find(bid);
                if (it != forest_->boxes().end() && boxes_touch(new_box, it->second, adj_tol))
                    return 'C';
            }
        } else if (origin == 'g') {
            for (int bid : start_box_ids_) {
                auto it = forest_->boxes().find(bid);
                if (it != forest_->boxes().end() && boxes_touch(new_box, it->second, adj_tol))
                    return 'C';
            }
        } else {
            for (int bid : start_box_ids_) {
                auto it = forest_->boxes().find(bid);
                if (it != forest_->boxes().end() && boxes_touch(new_box, it->second, adj_tol))
                    return 'S';
            }
            for (int bid : goal_box_ids_) {
                auto it = forest_->boxes().find(bid);
                if (it != forest_->boxes().end() && boxes_touch(new_box, it->second, adj_tol))
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
        auto& target_set = (target_origin == 's') ? start_box_ids_ : goal_box_ids_;
        for (int nid : to_flip) {
            box_origin_[nid] = target_origin;
            target_set.insert(nid);
            random_box_ids_.erase(nid);
            random_ancestor.erase(nid);
        }
        // Check if absorption bridges start<->goal
        if (!connected) {
            auto& opposite = (target_origin == 's') ? goal_box_ids_ : start_box_ids_;
            for (int nid : to_flip) {
                auto it = forest_->boxes().find(nid);
                if (it == forest_->boxes().end()) continue;
                for (int obid : opposite) {
                    auto oit = forest_->boxes().find(obid);
                    if (oit != forest_->boxes().end() &&
                        boxes_touch(it->second, oit->second, adj_tol)) {
                        connected = true;
                        std::cout << "    [grow] CONNECTED (absorption bridge) at "
                                  << forest_->n_boxes() << " boxes\n";
                        return;
                    }
                }
            }
        }
    };

    // ── Lambda: check if seed is inside opposite tree ──
    auto seed_in_opposite_tree = [&](const Eigen::VectorXd& q, char parent_origin) -> bool {
        if (parent_origin != 's' && parent_origin != 'g') return false;
        auto& opposite = (parent_origin == 's') ? goal_box_ids_ : start_box_ids_;
        for (int bid : opposite) {
            auto it = forest_->boxes().find(bid);
            if (it == forest_->boxes().end()) continue;
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
    auto try_create_box = [&](const Eigen::VectorXd& q,
                              double min_edge_override = -1.0) -> int {
        double me = (min_edge_override > 0) ? min_edge_override : config_.ffb_min_edge;
        auto ffb = tree_->find_free_box(q, checker_->obs_compact(), checker_->n_obs(),
                                         config_.ffb_max_depth, me);
        if (!ffb.success()) return -1;

        auto pr = tree_->try_promote(ffb.path,
                                      checker_->obs_compact(), checker_->n_obs(), 2);
        auto ivs = tree_->get_node_intervals(pr.result_idx);

        int box_id = forest_->allocate_id();
        BoxNode box(box_id, ivs, q);
        forest_->add_box_no_adjacency(box);
        tree_->mark_occupied(pr.result_idx, box_id);

        if (!pr.absorbed_box_ids.empty()) {
            std::unordered_set<int> absorbed_set(pr.absorbed_box_ids.begin(),
                                                  pr.absorbed_box_ids.end());
            for (int aid : absorbed_set) {
                box_origin_.erase(aid);
                start_box_ids_.erase(aid);
                goal_box_ids_.erase(aid);
                random_box_ids_.erase(aid);
                random_ancestor.erase(aid);
            }
            forest_->remove_boxes_no_adjacency(absorbed_set);
        }
        return box_id;
    };

    // ═══════════════════════════════════════════════════════════════════
    // Phase 1: Create anchor boxes for start and goal
    // ═══════════════════════════════════════════════════════════════════
    for (int idx = 0; idx < 2; ++idx) {
        const Eigen::VectorXd& qs = (idx == 0) ? start : goal;
        char origin_tag = (idx == 0) ? 's' : 'g';

        if (tree_->is_occupied(qs)) {
            auto* bx = forest_->find_containing(qs);
            if (bx) {
                box_origin_[bx->id] = origin_tag;
                if (origin_tag == 's')
                    start_box_ids_.insert(bx->id);
                else
                    goal_box_ids_.insert(bx->id);
            }
        } else {
            int bid = try_create_box(qs, config_.ffb_min_edge);
            if (bid < 0) {
                // Fallback: aggressive min_edge
                bid = try_create_box(qs, config_.ffb_min_edge_anchor);
            }
            if (bid >= 0) {
                box_origin_[bid] = origin_tag;
                if (origin_tag == 's')
                    start_box_ids_.insert(bid);
                else
                    goal_box_ids_.insert(bid);
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════════
    // Phase 2: Seed BFS expand queue from all existing boxes
    // ═══════════════════════════════════════════════════════════════════
    struct ExpandEntry {
        int box_id;
        std::set<std::pair<int,int>> excluded_faces;
    };
    std::deque<ExpandEntry> expand_queue;

    for (auto& [bid, bx] : forest_->boxes())
        expand_queue.push_back({bid, {}});

    // ═══════════════════════════════════════════════════════════════════
    // Phase 3: Main growth loop (BFS + random fallback)
    // ═══════════════════════════════════════════════════════════════════
    int consec = 0;
    const auto& phase_k = config_.bfs_phase_k;
    const auto& phase_budget = config_.bfs_phase_budget;
    int n_phases = static_cast<int>(std::min(phase_k.size(), phase_budget.size()));
    int phase_idx = 0;
    int phase_boxes = 0;

    auto get_phase_min_edge = [&]() -> double {
        if (connected) return config_.ffb_min_edge_relaxed;
        if (phase_idx < n_phases)
            return config_.ffb_min_edge * phase_k[phase_idx];
        return config_.ffb_min_edge;
    };

    auto advance_phase = [&]() {
        if (connected || phase_idx >= n_phases) return;
        if (phase_boxes >= phase_budget[phase_idx]) {
            phase_idx++;
            phase_boxes = 0;

            struct PriEntry { double dist; int box_id; };
            std::vector<PriEntry> candidates;

            for (int bid : start_box_ids_) {
                auto it = forest_->boxes().find(bid);
                if (it == forest_->boxes().end()) continue;
                Eigen::VectorXd c(n_dims);
                for (int d = 0; d < n_dims; ++d)
                    c[d] = 0.5 * (it->second.joint_intervals[d].lo +
                                  it->second.joint_intervals[d].hi);
                candidates.push_back({(c - goal).norm(), bid});
            }
            for (int bid : goal_box_ids_) {
                auto it = forest_->boxes().find(bid);
                if (it == forest_->boxes().end()) continue;
                Eigen::VectorXd c(n_dims);
                for (int d = 0; d < n_dims; ++d)
                    c[d] = 0.5 * (it->second.joint_intervals[d].lo +
                                  it->second.joint_intervals[d].hi);
                candidates.push_back({(c - start).norm(), bid});
            }

            std::sort(candidates.begin(), candidates.end(),
                      [](const PriEntry& a, const PriEntry& b) {
                          return a.dist < b.dist;
                      });

            expand_queue.clear();
            int max_enqueue = std::max(50, static_cast<int>(candidates.size() / 4));
            int n_enqueued = std::min(max_enqueue, static_cast<int>(candidates.size()));
            for (int i = 0; i < n_enqueued; ++i)
                expand_queue.push_back({candidates[i].box_id, {}});
        }
    };

    while (consec < max_miss) {
        if (forest_->n_boxes() >= max_boxes) break;

        // ── BFS boundary expansion ──
        if (!expand_queue.empty()) {
            auto entry = expand_queue.front();
            expand_queue.pop_front();

            auto box_it = forest_->boxes().find(entry.box_id);
            if (box_it == forest_->boxes().end()) continue;
            const BoxNode& parent_box = box_it->second;
            char parent_origin = box_origin_.count(entry.box_id)
                                 ? box_origin_[entry.box_id] : 'r';

            const Eigen::VectorXd* goal_pt = nullptr;
            int n_samp = 0;
            if (!connected) {
                n_samp = n_edge_samples;
                if (parent_origin == 's')
                    goal_pt = &goal;
                else if (parent_origin == 'g')
                    goal_pt = &start;
                else {
                    Eigen::VectorXd center(n_dims);
                    for (int d = 0; d < n_dims; ++d)
                        center[d] = 0.5 * (parent_box.joint_intervals[d].lo +
                                            parent_box.joint_intervals[d].hi);
                    double d_s = (center - start).norm();
                    double d_g = (center - goal).norm();
                    goal_pt = (d_s < d_g) ? &start : &goal;
                }
            }

            double effective_me = get_phase_min_edge();
            double effective_eps = effective_me * 0.5;

            auto seeds = boundary_sampler_.generate_boundary_seeds(
                parent_box, entry.excluded_faces,
                goal_pt, n_samp, effective_eps);

            for (auto& [dim, side, q] : seeds) {
                if (forest_->n_boxes() >= max_boxes) break;

                if (tree_->is_occupied(q)) {
                    if (!connected && seed_in_opposite_tree(q, parent_origin)) {
                        connected = true;
                        std::cout << "    [grow] CONNECTED (seed overlap) at "
                                  << forest_->n_boxes() << " boxes\n";
                    }
                    continue;
                }

                int child_id = try_create_box(q, effective_me);
                if (child_id < 0) continue;

                phase_boxes++;
                advance_phase();

                box_origin_[child_id] = parent_origin;
                if (parent_origin == 's')
                    start_box_ids_.insert(child_id);
                else if (parent_origin == 'g')
                    goal_box_ids_.insert(child_id);
                else {
                    random_box_ids_.insert(child_id);
                    random_ancestor[child_id] = random_ancestor.count(entry.box_id)
                        ? random_ancestor[entry.box_id] : entry.box_id;
                }

                if (!connected) {
                    char conn = check_box_connection(
                        forest_->boxes().at(child_id), parent_origin);
                    if (conn == 'C') {
                        connected = true;
                        std::cout << "    [grow] CONNECTED (box overlap) at "
                                  << forest_->n_boxes() << " boxes\n";
                    } else if (conn == 'S' || conn == 'G') {
                        char target = (conn == 'S') ? 's' : 'g';
                        int anc = random_ancestor.count(child_id)
                                  ? random_ancestor[child_id] : child_id;
                        absorb_random_tree(anc, target);
                    }
                }

                std::set<std::pair<int,int>> new_excluded = entry.excluded_faces;
                new_excluded.insert({dim, 1 - side});
                expand_queue.push_back({child_id, new_excluded});
                consec = 0;
            }
            continue;
        }

        // ── Random sampling fallback ──
        Eigen::VectorXd q = root_sampler_.sample_guided(goal);

        if (tree_->is_occupied(q) || checker_->check_config(q)) {
            consec++;
            continue;
        }

        double rand_me = get_phase_min_edge();
        int new_id = try_create_box(q, rand_me);
        if (new_id < 0) { consec++; continue; }

        phase_boxes++;
        advance_phase();

        box_origin_[new_id] = 'r';
        random_box_ids_.insert(new_id);
        random_ancestor[new_id] = new_id;

        if (!connected) {
            char conn = check_box_connection(forest_->boxes().at(new_id), 'r');
            if (conn == 'S' || conn == 'G') {
                char target = (conn == 'S') ? 's' : 'g';
                absorb_random_tree(new_id, target);
            }
        }

        expand_queue.push_back({new_id, {}});
        consec = 0;
    }

    std::cout << "    [grow] terminated: " << forest_->n_boxes() << " boxes ("
              << (connected ? "connected" : "disconnected") << "), "
              << "start=" << start_box_ids_.size()
              << " goal=" << goal_box_ids_.size()
              << " random=" << random_box_ids_.size() << "\n";

    return connected;
}

// ═══════════════════════════════════════════════════════════════════════════
// grow_multi: multi-pair forest construction (delegated from SBFPlanner)
// ═══════════════════════════════════════════════════════════════════════════
void ForestGrower::grow_multi(
    const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& pairs,
    int n_random_boxes,
    double timeout)
{
    // Multi-pair growth is handled by SBFPlanner::build_multi,
    // which creates sub-ForestGrowers per pair and merges.
    // This stub delegates to grow() for each pair sequentially.
    auto t0 = std::chrono::steady_clock::now();
    auto elapsed = [&t0]() {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
    };

    for (const auto& [s, g] : pairs) {
        if (elapsed() > timeout) break;
        grow(s, g);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// regrow: targeted regrowth into depleted regions
// ═══════════════════════════════════════════════════════════════════════════
int ForestGrower::regrow(int n_target, double timeout) {
    auto t0 = std::chrono::steady_clock::now();
    auto elapsed = [&t0]() {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
    };

    int added = 0;
    int miss = 0;
    int max_miss = config_.max_consecutive_miss * 3;

    while (added < n_target && miss < max_miss && elapsed() < timeout) {
        Eigen::VectorXd q = root_sampler_.sample_uniform();

        if (tree_->is_occupied(q) || checker_->check_config(q)) {
            miss++;
            continue;
        }

        auto ffb = tree_->find_free_box(q, checker_->obs_compact(),
                                         checker_->n_obs(),
                                         config_.ffb_max_depth,
                                         config_.ffb_min_edge_relaxed);
        if (!ffb.success()) { miss++; continue; }

        auto pr = tree_->try_promote(ffb.path,
                                      checker_->obs_compact(), checker_->n_obs(), 2);
        auto ivs = tree_->get_node_intervals(pr.result_idx);

        int box_id = forest_->allocate_id();
        BoxNode box(box_id, ivs, q);
        forest_->add_box_direct(box);
        tree_->mark_occupied(pr.result_idx, box_id);

        if (!pr.absorbed_box_ids.empty()) {
            std::unordered_set<int> abs_set(pr.absorbed_box_ids.begin(),
                                             pr.absorbed_box_ids.end());
            forest_->remove_boxes(abs_set);
        }

        added++;
        miss = 0;
    }

    return added;
}

} // namespace forest
} // namespace sbf

// SafeBoxForest v5 — Coarsening (Phase G)
#include <sbf/forest/coarsen.h>
#include <sbf/forest/adjacency.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace sbf {

// ═════════════════════════════════════════════════════════════════════════════
// Shared hull safety check — LECT tree traversal + member coverage + subdivision
// ═════════════════════════════════════════════════════════════════════════════

/// Walk LECT tree: check if C-space region [qlb, qub] is collision-free
/// using cached node envelopes (link iAABBs).  cur_ivs tracks the node's
/// intervals incrementally (no node_intervals recomputation).
static bool lect_tree_region_safe(
        LECT* lect, int node_idx,
        const Obstacle* obs, int n_obs,
        const Eigen::VectorXd& qlb, const Eigen::VectorXd& qub,
        std::vector<Interval>& cur_ivs, int n_dims, int max_depth) {
    // No overlap → doesn't affect query region
    for (int d = 0; d < n_dims; ++d)
        if (cur_ivs[d].hi <= qlb[d] || cur_ivs[d].lo >= qub[d])
            return true;

    // Check if node is fully inside query region
    bool fully_inside = true;
    for (int d = 0; d < n_dims; ++d) {
        if (cur_ivs[d].lo < qlb[d] - 1e-12 || cur_ivs[d].hi > qub[d] + 1e-12) {
            fully_inside = false; break;
        }
    }

    if (fully_inside && lect->has_data(node_idx)) {
        if (!lect->collides_scene(node_idx, obs, n_obs))
            return true;
        if (lect->is_leaf(node_idx))
            return false;
    }

    if (lect->is_leaf(node_idx) || max_depth <= 0)
        return false;

    int left = lect->left(node_idx);
    int right = lect->right(node_idx);
    int sd = lect->get_split_dim(node_idx);
    double sv = lect->split_val(node_idx);
    if (sd < 0 || sd >= n_dims) return false;

    double old_hi = cur_ivs[sd].hi, old_lo = cur_ivs[sd].lo;

    if (left >= 0) {
        cur_ivs[sd].hi = sv;
        if (!lect_tree_region_safe(lect, left, obs, n_obs, qlb, qub,
                                   cur_ivs, n_dims, max_depth - 1)) {
            cur_ivs[sd].hi = old_hi; return false;
        }
        cur_ivs[sd].hi = old_hi;
    }
    if (right >= 0) {
        cur_ivs[sd].lo = sv;
        if (!lect_tree_region_safe(lect, right, obs, n_obs, qlb, qub,
                                   cur_ivs, n_dims, max_depth - 1)) {
            cur_ivs[sd].lo = old_lo; return false;
        }
        cur_ivs[sd].lo = old_lo;
    }
    return true;
}

/// Check if hull region is collision-free using:
///   1. Member-box coverage pruning (known-safe sub-regions)
///   2. AABB conservative check
///   3. LECT tree traversal (cached envelopes)
///   4. Recursive bisection (depth-limited)
static bool hull_region_safe(
        const Eigen::VectorXd& sub_lb, const Eigen::VectorXd& sub_ub,
        const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& members,
        const CollisionChecker& checker, LECT* lect, int depth) {
    const int n = sub_lb.size();

    // 1. Covered by any member → safe
    for (const auto& [mlb, mub] : members) {
        bool covered = true;
        for (int d = 0; d < n; ++d) {
            if (sub_lb[d] < mlb[d] - 1e-12 || sub_ub[d] > mub[d] + 1e-12) {
                covered = false; break;
            }
        }
        if (covered) return true;
    }

    // 2. AABB check
    std::vector<Interval> ivs(n);
    for (int d = 0; d < n; ++d) { ivs[d].lo = sub_lb[d]; ivs[d].hi = sub_ub[d]; }
    if (!checker.check_box(ivs)) return true;

    // 3. LECT tree traversal
    if (lect != nullptr) {
        auto root_ivs = lect->root_intervals();
        if (lect_tree_region_safe(lect, 0, checker.obstacles(), checker.n_obs(),
                                  sub_lb, sub_ub, root_ivs, n, 30))
            return true;
    }

    // 4. Subdivide
    if (depth <= 0) return false;
    int best_dim = 0; double best_w = 0.0;
    for (int d = 0; d < n; ++d) {
        double w = sub_ub[d] - sub_lb[d];
        if (w > best_w) { best_w = w; best_dim = d; }
    }
    double mid = 0.5 * (sub_lb[best_dim] + sub_ub[best_dim]);
    Eigen::VectorXd left_ub = sub_ub;  left_ub[best_dim] = mid;
    Eigen::VectorXd right_lb = sub_lb; right_lb[best_dim] = mid;
    if (!hull_region_safe(sub_lb, left_ub, members, checker, lect, depth - 1))
        return false;
    return hull_region_safe(right_lb, sub_ub, members, checker, lect, depth - 1);
}

/// Convenience: check hull of two boxes (a, b) for safety
static bool check_hull_safe_pair(
        const BoxNode& box_a, const BoxNode& box_b,
        const std::vector<Interval>& hull_ivs,
        const CollisionChecker& checker, LECT* lect) {
    const int n = box_a.n_dims();
    // Fast path: AABB safe → done
    if (!checker.check_box(hull_ivs)) return true;

    // Build member bounds for coverage pruning
    Eigen::VectorXd alb(n), aub(n), blb(n), bub(n), hlb(n), hub(n);
    for (int d = 0; d < n; ++d) {
        alb[d] = box_a.joint_intervals[d].lo; aub[d] = box_a.joint_intervals[d].hi;
        blb[d] = box_b.joint_intervals[d].lo; bub[d] = box_b.joint_intervals[d].hi;
        hlb[d] = hull_ivs[d].lo; hub[d] = hull_ivs[d].hi;
    }
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> members = {{alb, aub}, {blb, bub}};
    return hull_region_safe(hlb, hub, members, checker, lect, 0);
}

/// Convenience: check hull of multiple boxes for safety
static bool check_hull_safe_multi(
        const std::vector<const BoxNode*>& member_boxes,
        const std::vector<Interval>& hull_ivs,
        const CollisionChecker& checker, LECT* lect) {
    if (!checker.check_box(hull_ivs)) return true;
    if (member_boxes.empty()) return false;

    const int n = member_boxes[0]->n_dims();
    Eigen::VectorXd hlb(n), hub(n);
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> members;
    for (const auto* box : member_boxes) {
        Eigen::VectorXd lb(n), ub(n);
        for (int d = 0; d < n; ++d) {
            lb[d] = box->joint_intervals[d].lo;
            ub[d] = box->joint_intervals[d].hi;
        }
        members.push_back({lb, ub});
    }
    for (int d = 0; d < n; ++d) { hlb[d] = hull_ivs[d].lo; hub[d] = hull_ivs[d].hi; }
    return hull_region_safe(hlb, hub, members, checker, lect, 0);
}

// ═════════════════════════════════════════════════════════════════════════════
// G1: Dimension-Sweep Merge
// ═════════════════════════════════════════════════════════════════════════════

CoarsenResult coarsen_forest(
        std::vector<BoxNode>& boxes,
        const CollisionChecker& /*checker*/,
        int max_rounds,
        double target_ratio) {
    CoarsenResult result;
    result.boxes_before = static_cast<int>(boxes.size());
    if (boxes.empty()) {
        result.boxes_after = 0;
        return result;
    }

    const int n_dims = boxes[0].n_dims();
    const int target_n = (target_ratio > 0.0)
        ? static_cast<int>(result.boxes_before * target_ratio)
        : 0;

    for (int round = 0; round < max_rounds; ++round) {
        result.rounds++;
        int merges_this_round = 0;

        for (int dim = 0; dim < n_dims; ++dim) {
            // Compact dead boxes from previous dimension's merges
            boxes.erase(
                std::remove_if(boxes.begin(), boxes.end(),
                               [](const BoxNode& b) { return b.volume < 0; }),
                boxes.end());

            // Sort by lo bound in this dimension
            std::sort(boxes.begin(), boxes.end(),
                      [dim](const BoxNode& a, const BoxNode& b) {
                          return a.joint_intervals[dim].lo
                               < b.joint_intervals[dim].lo;
                      });

            // Scan for exact-touching pairs (look beyond just consecutive)
            for (size_t i = 0; i + 1 < boxes.size(); ) {
                if (boxes[i].volume < 0) { ++i; continue; }  // skip dead
                bool merged = false;
                for (size_t j = i + 1; j < boxes.size(); ++j) {
                    if (boxes[j].volume < 0) continue;  // skip dead
                    double gap = boxes[j].joint_intervals[dim].lo
                               - boxes[i].joint_intervals[dim].hi;
                    if (gap > 1e-10) break;  // sorted: no more candidates
                    if (gap < -1e-10) continue;  // j overlaps i in dim

                    // gap ≈ 0: touching in dim. Check other dims match.
                    bool exact_match = true;
                    for (int d = 0; d < n_dims; ++d) {
                        if (d == dim) continue;
                        if (std::abs(boxes[i].joint_intervals[d].lo
                                   - boxes[j].joint_intervals[d].lo) > 1e-10 ||
                            std::abs(boxes[i].joint_intervals[d].hi
                                   - boxes[j].joint_intervals[d].hi) > 1e-10) {
                            exact_match = false;
                            break;
                        }
                    }
                    if (!exact_match) continue;

                    // Merge j into i; mark j as dead (volume < 0)
                    boxes[i].joint_intervals[dim].hi =
                        boxes[j].joint_intervals[dim].hi;
                    boxes[i].compute_volume();
                    boxes[i].tree_id = -1;

                    boxes[j].volume = -1.0;  // mark dead (O(1) vs O(N) erase)
                    merges_this_round++;
                    result.merges_performed++;
                    merged = true;
                    break;  // restart from i to catch chains
                }
                if (!merged) ++i;
            }
        }

        // Final compact after all dimensions in this round
        boxes.erase(
            std::remove_if(boxes.begin(), boxes.end(),
                           [](const BoxNode& b) { return b.volume < 0; }),
            boxes.end());

        if (merges_this_round == 0) break;
        // Early exit if target ratio reached
        if (target_n > 0 && static_cast<int>(boxes.size()) <= target_n) break;
    }

    result.boxes_after = static_cast<int>(boxes.size());
    return result;
}
// ═════════════════════════════════════════════════════════════════════════════

CoarsenResult coarsen_sweep_relaxed(
        std::vector<BoxNode>& boxes,
        const CollisionChecker& checker,
        LECT* lect,
        int max_rounds,
        double score_threshold,
        int max_lect_fk_per_round,
        double target_ratio) {
    CoarsenResult result;
    result.boxes_before = static_cast<int>(boxes.size());
    if (boxes.empty()) {
        result.boxes_after = 0;
        return result;
    }

    const int n_dims = boxes[0].n_dims();
    const int target_n = (target_ratio > 0.0)
        ? static_cast<int>(result.boxes_before * target_ratio)
        : 0;

    for (int round = 0; round < max_rounds; ++round) {
        result.rounds++;
        int merges_this_round = 0;

        for (int dim = 0; dim < n_dims; ++dim) {
            // Compact dead boxes
            boxes.erase(
                std::remove_if(boxes.begin(), boxes.end(),
                               [](const BoxNode& b) { return b.volume < 0; }),
                boxes.end());

            // Sort by lo bound in this dimension
            std::sort(boxes.begin(), boxes.end(),
                      [dim](const BoxNode& a, const BoxNode& b) {
                          return a.joint_intervals[dim].lo
                               < b.joint_intervals[dim].lo;
                      });

            for (size_t i = 0; i + 1 < boxes.size(); ) {
                if (boxes[i].volume < 0) { ++i; continue; }
                bool merged = false;

                for (size_t j = i + 1; j < boxes.size(); ++j) {
                    if (boxes[j].volume < 0) continue;
                    double gap = boxes[j].joint_intervals[dim].lo
                               - boxes[i].joint_intervals[dim].hi;
                    if (gap > 1e-10) break;   // sorted: no more candidates
                    if (gap < -1e-10) continue; // j overlaps i in dim

                    // gap ≈ 0: touching in dim.  Other dims must overlap.
                    bool all_overlap = true;
                    for (int d = 0; d < n_dims; ++d) {
                        if (d == dim) continue;
                        if (boxes[i].joint_intervals[d].hi <=
                                boxes[j].joint_intervals[d].lo + 1e-12 ||
                            boxes[j].joint_intervals[d].hi <=
                                boxes[i].joint_intervals[d].lo + 1e-12) {
                            all_overlap = false;
                            break;
                        }
                    }
                    if (!all_overlap) continue;

                    // Hull AABB
                    std::vector<Interval> hull_ivs(n_dims);
                    double hull_vol = 1.0;
                    double sum_vol = boxes[i].volume + boxes[j].volume;
                    for (int d = 0; d < n_dims; ++d) {
                        hull_ivs[d].lo = std::min(
                            boxes[i].joint_intervals[d].lo,
                            boxes[j].joint_intervals[d].lo);
                        hull_ivs[d].hi = std::max(
                            boxes[i].joint_intervals[d].hi,
                            boxes[j].joint_intervals[d].hi);
                        hull_vol *= hull_ivs[d].width();
                    }

                    double score = (sum_vol > 0.0) ? hull_vol / sum_vol
                                                   : 1e18;
                    if (score > score_threshold) continue;

                    // Collision check (hull ⊃ A∪B → must verify)
                    bool hull_safe = check_hull_safe_pair(
                        boxes[i], boxes[j], hull_ivs, checker, lect);
                    if (!hull_safe) continue;

                    // Merge j into i
                    boxes[i].joint_intervals = hull_ivs;
                    boxes[i].compute_volume();
                    boxes[i].tree_id = -1;
                    boxes[i].seed_config = 0.5 * (boxes[i].seed_config
                                                 + boxes[j].seed_config);

                    boxes[j].volume = -1.0;  // mark dead
                    merges_this_round++;
                    result.merges_performed++;
                    merged = true;
                    break;  // restart from i to catch chains
                }
                if (!merged) ++i;
            }
        }

        // Final compact
        boxes.erase(
            std::remove_if(boxes.begin(), boxes.end(),
                           [](const BoxNode& b) { return b.volume < 0; }),
            boxes.end());

        if (merges_this_round == 0) break;
        // Early exit if target ratio reached
        if (target_n > 0 && static_cast<int>(boxes.size()) <= target_n) break;
    }

    result.boxes_after = static_cast<int>(boxes.size());
    return result;
}

// ═════════════════════════════════════════════════════════════════════════════
// G2: Greedy Adjacency-Based Coarsening
// ═════════════════════════════════════════════════════════════════════════════

GreedyCoarsenResult coarsen_greedy(
        std::vector<BoxNode>& boxes,
        const CollisionChecker& checker,
        const GreedyCoarsenConfig& config,
        LECT* lect,
        const std::unordered_set<int>* protected_ids) {
    auto t_start = std::chrono::steady_clock::now();
    GreedyCoarsenResult result;
    result.boxes_before = static_cast<int>(boxes.size());

    if (boxes.size() <= 1) {
        result.boxes_after = static_cast<int>(boxes.size());
        return result;
    }
    if (config.target_boxes > 0 &&
        static_cast<int>(boxes.size()) <= config.target_boxes) {
        result.boxes_after = static_cast<int>(boxes.size());
        return result;
    }

    const int n_dims = boxes[0].n_dims();

    for (int round = 0; round < config.max_rounds; ++round) {
        result.rounds++;

        // Timeout check
        if (config.timeout_ms > 0) {
            double elapsed_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t_start).count();
            if (elapsed_ms >= config.timeout_ms) break;
        }

        // ── Stage 1: build adjacency + collect candidates ───────────────
        auto adj = compute_adjacency(boxes, config.adjacency_tol);

        // Index boxes by id for O(1) lookup
        std::unordered_map<int, int> id_to_idx;
        for (int i = 0; i < static_cast<int>(boxes.size()); ++i)
            id_to_idx[boxes[i].id] = i;

        struct Candidate {
            double score;
            int id_a, id_b;
            std::vector<Interval> hull_ivs;
            double hull_vol;
        };
        std::vector<Candidate> candidates;

        for (auto& [id_a, neighbors] : adj) {
            auto it_a = id_to_idx.find(id_a);
            if (it_a == id_to_idx.end()) continue;
            const BoxNode& box_a = boxes[it_a->second];

            for (int id_b : neighbors) {
                if (id_b <= id_a) continue;  // dedup

                auto it_b = id_to_idx.find(id_b);
                if (it_b == id_to_idx.end()) continue;
                const BoxNode& box_b = boxes[it_b->second];

                // Hull AABB
                std::vector<Interval> hull_ivs(n_dims);
                double hull_vol = 1.0;
                for (int d = 0; d < n_dims; ++d) {
                    hull_ivs[d].lo = std::min(box_a.joint_intervals[d].lo,
                                              box_b.joint_intervals[d].lo);
                    hull_ivs[d].hi = std::max(box_a.joint_intervals[d].hi,
                                              box_b.joint_intervals[d].hi);
                    hull_vol *= hull_ivs[d].width();
                }

                double sum_vol = box_a.volume + box_b.volume;
                double score = (sum_vol > 0.0) ? hull_vol / sum_vol : 1e18;

                if (score > config.score_threshold) continue;

                candidates.push_back(
                    {score, id_a, id_b, std::move(hull_ivs), hull_vol});
            }
        }

        if (candidates.empty()) break;

        // Sort: best (lowest) score first
        std::sort(candidates.begin(), candidates.end(),
                  [](const Candidate& a, const Candidate& b) {
                      return a.score < b.score;
                  });

        // ── Stage 2: greedy execute ─────────────────────────────────────
        std::unordered_set<int> consumed_ids;  // ids that participated
        std::unordered_set<int> remove_ids;    // b-side ids to erase
        int merges_this_round = 0;

        for (auto& cand : candidates) {
            if (config.target_boxes > 0 &&
                static_cast<int>(boxes.size())
                    - static_cast<int>(remove_ids.size())
                    <= config.target_boxes)
                break;

            if (consumed_ids.count(cand.id_a) ||
                consumed_ids.count(cand.id_b))
                continue;

            // Protect bridge boxes (articulation points)
            if (protected_ids &&
                (protected_ids->count(cand.id_b) > 0)) {
                // b-side would be removed; skip if it's a bridge
                continue;
            }

            auto it_a = id_to_idx.find(cand.id_a);
            auto it_b = id_to_idx.find(cand.id_b);
            if (it_a == id_to_idx.end() || it_b == id_to_idx.end())
                continue;

            // ── Collision check ─────────────────────────────────────────
            const BoxNode& box_a = boxes[it_a->second];
            const BoxNode& box_b = boxes[it_b->second];
            if (!check_hull_safe_pair(box_a, box_b, cand.hull_ivs,
                                      checker, lect))
                continue;

            // ── Execute merge ───────────────────────────────────────────

            BoxNode merged;
            merged.id = box_a.id;  // keep a's id
            merged.joint_intervals = cand.hull_ivs;
            merged.volume = cand.hull_vol;
            merged.tree_id = -1;
            merged.seed_config = 0.5 * (box_a.seed_config
                                       + box_b.seed_config);
            merged.parent_box_id = -1;
            merged.root_id = box_a.root_id;

            // Replace a's slot with merged box
            boxes[it_a->second] = merged;

            consumed_ids.insert(cand.id_a);
            consumed_ids.insert(cand.id_b);
            remove_ids.insert(cand.id_b);
            merges_this_round++;
            result.merges_performed++;
        }

        if (merges_this_round == 0) break;

        // Remove the b-side boxes
        boxes.erase(
            std::remove_if(boxes.begin(), boxes.end(),
                           [&remove_ids](const BoxNode& b) {
                               return remove_ids.count(b.id) > 0;
                           }),
            boxes.end());

        // Check target
        if (config.target_boxes > 0 &&
            static_cast<int>(boxes.size()) <= config.target_boxes)
            break;
    }

    result.boxes_after = static_cast<int>(boxes.size());
    result.elapsed_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t_start).count();
    return result;
}

// ═════════════════════════════════════════════════════════════════════════════
// G3: Multi-Box Cluster Merge
// ═════════════════════════════════════════════════════════════════════════════

ClusterCoarsenResult coarsen_cluster(
        std::vector<BoxNode>& boxes,
        const CollisionChecker& checker,
        const ClusterCoarsenConfig& config,
        LECT* lect,
        const std::unordered_set<int>* protected_ids) {
    auto t_start = std::chrono::steady_clock::now();
    ClusterCoarsenResult result;
    result.boxes_before = static_cast<int>(boxes.size());

    if (boxes.size() <= 2) {
        result.boxes_after = static_cast<int>(boxes.size());
        return result;
    }

    const int n_dims = boxes[0].n_dims();

    for (int round = 0; round < config.max_rounds; ++round) {
        result.rounds++;

        // ── Build adjacency ─────────────────────────────────────────────
        auto adj = compute_adjacency(boxes, config.adjacency_tol);

        std::unordered_map<int, int> id_to_idx;
        for (int i = 0; i < static_cast<int>(boxes.size()); ++i)
            id_to_idx[boxes[i].id] = i;

        // ── Build cluster candidates per box ────────────────────────────
        struct ClusterCandidate {
            double score;               // hull_vol / sum_vol
            std::vector<int> member_ids; // ids in cluster
            std::vector<Interval> hull_ivs;
            double hull_vol;
        };
        std::vector<ClusterCandidate> candidates;

        for (int i = 0; i < static_cast<int>(boxes.size()); ++i) {
            const BoxNode& box_i = boxes[i];
            auto it_adj = adj.find(box_i.id);
            if (it_adj == adj.end() || it_adj->second.empty()) continue;

            // Sort neighbors by pairwise expansion ratio (ascending)
            struct NbrScore { double score; int id; };
            std::vector<NbrScore> nbr_scores;
            for (int nbr_id : it_adj->second) {
                auto it_n = id_to_idx.find(nbr_id);
                if (it_n == id_to_idx.end()) continue;
                const BoxNode& box_n = boxes[it_n->second];

                double pair_hull_vol = 1.0;
                for (int d = 0; d < n_dims; ++d) {
                    double lo = std::min(box_i.joint_intervals[d].lo,
                                         box_n.joint_intervals[d].lo);
                    double hi = std::max(box_i.joint_intervals[d].hi,
                                         box_n.joint_intervals[d].hi);
                    pair_hull_vol *= (hi - lo);
                }
                double sum = box_i.volume + box_n.volume;
                double s = (sum > 0.0) ? pair_hull_vol / sum : 1e18;
                nbr_scores.push_back({s, nbr_id});
            }
            std::sort(nbr_scores.begin(), nbr_scores.end(),
                      [](const NbrScore& a, const NbrScore& b) {
                          return a.score < b.score;
                      });

            // Greedily grow cluster from box_i
            std::vector<int> cluster = {box_i.id};
            std::vector<Interval> hull(n_dims);
            for (int d = 0; d < n_dims; ++d) hull[d] = box_i.joint_intervals[d];
            double sum_vol = box_i.volume;

            for (auto& ns : nbr_scores) {
                if (static_cast<int>(cluster.size()) >= config.max_cluster_size)
                    break;

                auto it_n = id_to_idx.find(ns.id);
                if (it_n == id_to_idx.end()) continue;
                const BoxNode& box_n = boxes[it_n->second];

                // Compute extended hull
                std::vector<Interval> try_hull(n_dims);
                double try_hull_vol = 1.0;
                for (int d = 0; d < n_dims; ++d) {
                    try_hull[d].lo = std::min(hull[d].lo,
                                              box_n.joint_intervals[d].lo);
                    try_hull[d].hi = std::max(hull[d].hi,
                                              box_n.joint_intervals[d].hi);
                    try_hull_vol *= try_hull[d].width();
                }
                double try_sum = sum_vol + box_n.volume;
                double try_score = (try_sum > 0.0)
                                       ? try_hull_vol / try_sum
                                       : 1e18;
                if (try_score > config.score_threshold) continue;

                // Accept neighbor into cluster
                hull = try_hull;
                sum_vol = try_sum;
                cluster.push_back(ns.id);
            }

            if (cluster.size() < 2) continue;

            double hull_vol = 1.0;
            for (int d = 0; d < n_dims; ++d) hull_vol *= hull[d].width();
            double final_score = (sum_vol > 0.0) ? hull_vol / sum_vol : 1e18;

            candidates.push_back(
                {final_score, std::move(cluster), std::move(hull), hull_vol});
        }

        if (candidates.empty()) break;

        // Sort by score ascending (tightest clusters first)
        std::sort(candidates.begin(), candidates.end(),
                  [](const ClusterCandidate& a, const ClusterCandidate& b) {
                      return a.score < b.score;
                  });

        // ── Greedy execute ──────────────────────────────────────────────
        std::unordered_set<int> consumed_ids;
        std::unordered_set<int> remove_ids;
        int merges_this_round = 0;

        for (auto& cand : candidates) {
            // Check all members unconsumed
            bool any_consumed = false;
            for (int mid : cand.member_ids) {
                if (consumed_ids.count(mid)) { any_consumed = true; break; }
            }
            if (any_consumed) continue;

            // Check protected (don't remove bridge boxes)
            if (protected_ids) {
                bool any_protected = false;
                for (size_t k = 1; k < cand.member_ids.size(); ++k) {
                    if (protected_ids->count(cand.member_ids[k])) {
                        any_protected = true;
                        break;
                    }
                }
                if (any_protected) continue;
            }

            // Collision check — collect member box pointers
            {
                std::vector<const BoxNode*> member_ptrs;
                member_ptrs.reserve(cand.member_ids.size());
                for (int mid : cand.member_ids) {
                    auto it_m = id_to_idx.find(mid);
                    if (it_m != id_to_idx.end())
                        member_ptrs.push_back(&boxes[it_m->second]);
                }
                if (!check_hull_safe_multi(member_ptrs, cand.hull_ivs,
                                           checker, lect))
                    continue;
            }

            // Merge: keep first member, absorb rest
            int keep_id = cand.member_ids[0];
            auto it_keep = id_to_idx.find(keep_id);
            if (it_keep == id_to_idx.end()) continue;

            // Compute average seed_config
            Eigen::VectorXd avg_seed = Eigen::VectorXd::Zero(n_dims);
            for (int mid : cand.member_ids) {
                auto it_m = id_to_idx.find(mid);
                if (it_m != id_to_idx.end())
                    avg_seed += boxes[it_m->second].seed_config;
            }
            avg_seed /= static_cast<double>(cand.member_ids.size());

            BoxNode merged;
            merged.id = keep_id;
            merged.joint_intervals = cand.hull_ivs;
            merged.volume = cand.hull_vol;
            merged.tree_id = -1;
            merged.seed_config = avg_seed;
            merged.parent_box_id = -1;
            merged.root_id = boxes[it_keep->second].root_id;

            boxes[it_keep->second] = merged;

            for (int mid : cand.member_ids) consumed_ids.insert(mid);
            for (size_t k = 1; k < cand.member_ids.size(); ++k)
                remove_ids.insert(cand.member_ids[k]);

            merges_this_round += static_cast<int>(cand.member_ids.size()) - 1;
            result.merges_performed += static_cast<int>(cand.member_ids.size()) - 1;
            result.clusters_formed++;
        }

        if (merges_this_round == 0) break;

        // Remove absorbed boxes
        boxes.erase(
            std::remove_if(boxes.begin(), boxes.end(),
                           [&remove_ids](const BoxNode& b) {
                               return remove_ids.count(b.id) > 0;
                           }),
            boxes.end());
    }

    result.boxes_after = static_cast<int>(boxes.size());
    result.elapsed_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t_start).count();
    return result;
}

// ═════════════════════════════════════════════════════════════════════════════
// Post-coarsen overlap filter
// ═════════════════════════════════════════════════════════════════════════════

static bool box_contains_box(const BoxNode& big, const BoxNode& small,
                             double tol = 1e-10) {
    for (int d = 0; d < big.n_dims(); ++d) {
        if (small.joint_intervals[d].lo < big.joint_intervals[d].lo - tol ||
            small.joint_intervals[d].hi > big.joint_intervals[d].hi + tol)
            return false;
    }
    return true;
}

static bool boxes_overlap(const BoxNode& a, const BoxNode& b) {
    for (int d = 0; d < a.n_dims(); ++d) {
        if (a.joint_intervals[d].lo >= b.joint_intervals[d].hi - 1e-12 ||
            a.joint_intervals[d].hi <= b.joint_intervals[d].lo + 1e-12)
            return false;
    }
    return true;
}

static double geometric_mean_edge(const BoxNode& box) {
    int nd = box.n_dims();
    double log_sum = 0.0;
    for (int d = 0; d < nd; ++d) {
        double w = box.joint_intervals[d].width();
        if (w <= 0.0) return 0.0;
        log_sum += std::log(w);
    }
    return std::exp(log_sum / nd);
}

int filter_coarsen_overlaps(std::vector<BoxNode>& boxes,
                            double min_gmean_edge,
                            const std::unordered_set<int>* protected_ids) {
    if (boxes.empty()) return 0;

    const int n_before = static_cast<int>(boxes.size());

    // Sort by volume descending (large boxes first)
    std::sort(boxes.begin(), boxes.end(),
              [](const BoxNode& a, const BoxNode& b) {
                  return a.volume > b.volume;
              });

    // Pass 1: mark fully contained boxes for removal
    std::vector<bool> removed(boxes.size(), false);
    for (size_t i = 0; i < boxes.size(); ++i) {
        if (removed[i]) continue;
        for (size_t j = i + 1; j < boxes.size(); ++j) {
            if (removed[j]) continue;
            if (protected_ids && protected_ids->count(boxes[j].id) > 0)
                continue;  // never remove bridge boxes
            if (box_contains_box(boxes[i], boxes[j])) {
                removed[j] = true;
            }
        }
    }

    // [DISABLED] Pass 2: trim partial overlaps — shrink the smaller box
    // Trimming box boundaries breaks face-contact adjacency, causing
    // island fragmentation (5 islands → 10+).  Overlapping boxes do not
    // affect planning correctness; shared_face only checks face contact.

    // Compact
    size_t write = 0;
    for (size_t read = 0; read < boxes.size(); ++read) {
        if (!removed[read]) {
            if (write != read)
                boxes[write] = std::move(boxes[read]);
            ++write;
        }
    }
    boxes.resize(write);

    return n_before - static_cast<int>(boxes.size());
}

}  // namespace sbf

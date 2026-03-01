// SafeBoxForest — Coarsen implementation
#include "sbf/forest/coarsen.h"
#include "sbf/forest/hier_aabb_tree.h"
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <map>
#include <unordered_set>
#include <vector>

namespace sbf {

CoarsenResult coarsen_forest(SafeBoxForest& forest,
                              const CollisionChecker& checker,
                              int max_rounds) {
    CoarsenResult result;
    result.boxes_before = forest.n_boxes();
    int n_dims = forest.n_dims();

    for (int round = 0; round < max_rounds; ++round) {
        result.rounds++;
        int merges_this_round = 0;

        // Try merging along each dimension
        for (int dim = 0; dim < n_dims; ++dim) {
            // Collect all boxes sorted by their lo bound in this dim
            std::vector<std::pair<double, int>> sorted_boxes;
            for (auto& [id, box] : forest.boxes())
                sorted_boxes.emplace_back(box.joint_intervals[dim].lo, id);
            std::sort(sorted_boxes.begin(), sorted_boxes.end());

            // Find touching runs: pairs where hi[i] == lo[j] in this dim
            // and they overlap in all other dims
            for (size_t i = 0; i + 1 < sorted_boxes.size(); ++i) {
                int id_a = sorted_boxes[i].second;
                int id_b = sorted_boxes[i + 1].second;

                auto it_a = forest.boxes().find(id_a);
                auto it_b = forest.boxes().find(id_b);
                if (it_a == forest.boxes().end() || it_b == forest.boxes().end())
                    continue;

                const BoxNode& a = it_a->second;
                const BoxNode& b = it_b->second;

                // Check if they touch exactly in this dimension
                double gap = b.joint_intervals[dim].lo - a.joint_intervals[dim].hi;
                if (std::abs(gap) > 1e-10) continue;

                // Check if they match exactly in all other dimensions
                bool exact_match = true;
                for (int d = 0; d < n_dims; ++d) {
                    if (d == dim) continue;
                    if (std::abs(a.joint_intervals[d].lo - b.joint_intervals[d].lo) > 1e-10 ||
                        std::abs(a.joint_intervals[d].hi - b.joint_intervals[d].hi) > 1e-10) {
                        exact_match = false;
                        break;
                    }
                }
                if (!exact_match) continue;

                // Create merged box intervals
                std::vector<Interval> merged_ivs = a.joint_intervals;
                merged_ivs[dim].hi = b.joint_intervals[dim].hi;

                // Check if merged box is collision-free
                if (checker.check_box(merged_ivs)) continue;

                // Merge: create new box, remove old ones
                int new_id = forest.allocate_id();
                BoxNode merged_box(new_id, merged_ivs, a.seed_config);

                forest.remove_boxes_no_adjacency({id_a, id_b});
                forest.add_box_no_adjacency(merged_box);
                merges_this_round++;
                result.merges_performed++;
            }
        }

        if (merges_this_round == 0) break;
        // Rebuild adjacency after each round
        forest.rebuild_adjacency();
    }

    result.boxes_after = forest.n_boxes();
    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// Greedy adjacency-based coarsen
// ═══════════════════════════════════════════════════════════════════════════

GreedyCoarsenResult coarsen_greedy(SafeBoxForest& forest,
                                    const CollisionChecker& checker,
                                    int target_boxes,
                                    int max_rounds,
                                    double adjacency_tol,
                                    const HierAABBTree* tree,
                                    int tree_split_depth,
                                    int max_tree_fk_per_round) {
    auto t_start = std::chrono::steady_clock::now();
    GreedyCoarsenResult result;
    result.boxes_before = forest.n_boxes();

    if (target_boxes > 0 && forest.n_boxes() <= target_boxes) {
        result.boxes_after = forest.n_boxes();
        return result;
    }

    int n_dims = forest.n_dims();

    for (int round = 0; round < max_rounds; ++round) {
        result.rounds++;

        // Target reached?
        if (target_boxes > 0 && forest.n_boxes() <= target_boxes) break;

        // Rebuild adjacency for current state
        forest.rebuild_adjacency(adjacency_tol);
        const auto& adj = forest.adjacency();

        // ── Collect merge candidates ────────────────────────────────────
        // candidate = (score, id_a, id_b, hull_intervals)
        struct Candidate {
            double score;  // hull_vol / sum_vol — lower = tighter merge
            int id_a, id_b;
            std::vector<Interval> hull_ivs;
        };
        std::vector<Candidate> candidates;
        candidates.reserve(forest.n_boxes());  // rough estimate

        // Avoid duplicates: only consider (a,b) where a < b
        for (auto& [id_a, neighbors] : adj) {
            auto it_a = forest.boxes().find(id_a);
            if (it_a == forest.boxes().end()) continue;
            const BoxNode& box_a = it_a->second;

            for (int id_b : neighbors) {
                if (id_b <= id_a) continue;  // dedup

                auto it_b = forest.boxes().find(id_b);
                if (it_b == forest.boxes().end()) continue;
                const BoxNode& box_b = it_b->second;

                // Compute hull AABB
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
                double score = (sum_vol > 0) ? hull_vol / sum_vol : 1e18;

                // Skip if hull is too much larger — unlikely to be collision-free
                // and not worth checking (heuristic prune)
                if (score > 50.0) continue;

                candidates.push_back({score, id_a, id_b, std::move(hull_ivs)});
            }
        }

        if (candidates.empty()) break;

        // Sort by score (best = smallest ratio first)
        std::sort(candidates.begin(), candidates.end(),
                  [](const Candidate& a, const Candidate& b) {
                      return a.score < b.score;
                  });

        // ── Greedy execute: each box merged at most once per round ──────
        std::unordered_set<int> merged_this_round;
        int merges_this_round = 0;

        // FK budget per round: limit tree_check cost in later rounds
        int64_t fk_at_round_start = tree ? tree->total_fk_calls() : 0;

        for (auto& cand : candidates) {
            // Target reached?
            if (target_boxes > 0 &&
                forest.n_boxes() - merges_this_round <= target_boxes)
                break;

            // Skip if either box already consumed
            if (merged_this_round.count(cand.id_a) ||
                merged_this_round.count(cand.id_b))
                continue;

            // Look up both boxes
            auto it_a = forest.boxes().find(cand.id_a);
            auto it_b = forest.boxes().find(cand.id_b);
            if (it_a == forest.boxes().end() || it_b == forest.boxes().end())
                continue;

            // Collision check on hull AABB
            bool hull_safe;
            // Fast path: try simple hull check first (cheap).
            hull_safe = !checker.check_box(cand.hull_ivs);
            if (!hull_safe && tree) {
                // Check FK budget: skip tree_check if exhausted this round
                int64_t fk_used = tree->total_fk_calls() - fk_at_round_start;
                if (max_tree_fk_per_round <= 0 || fk_used < max_tree_fk_per_round) {
                    // Tree-cached fallback: traverse KD-tree using cached AABBs
                    hull_safe = tree->check_hull_safe(
                        cand.hull_ivs,
                        it_a->second.joint_intervals,
                        it_b->second.joint_intervals,
                        checker.obs_flat(), checker.n_obs(),
                        tree_split_depth);
                }
            }
            if (!hull_safe) continue;

            int new_id = forest.allocate_id();
            BoxNode merged_box(new_id, cand.hull_ivs, it_a->second.seed_config);

            forest.remove_boxes_no_adjacency({cand.id_a, cand.id_b});
            forest.add_box_no_adjacency(merged_box);

            merged_this_round.insert(cand.id_a);
            merged_this_round.insert(cand.id_b);
            merges_this_round++;
            result.merges_performed++;
        }

        if (merges_this_round == 0) break;

        // Print progress every few rounds
        if (round < 5 || (round + 1) % 10 == 0) {
            double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t_start).count();
            std::cout << "    [greedy_coarsen] round " << (round + 1)
                      << ": merged " << merges_this_round
                      << " pairs → " << forest.n_boxes() << " boxes"
                      << " (" << std::fixed << std::setprecision(2)
                      << elapsed << "s)\n";
        }
    }

    // Final adjacency rebuild
    forest.rebuild_adjacency(adjacency_tol);

    result.boxes_after = forest.n_boxes();
    result.elapsed_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t_start).count();
    return result;
}

} // namespace sbf

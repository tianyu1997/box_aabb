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
// G1: Dimension-Sweep Merge
// ═════════════════════════════════════════════════════════════════════════════

CoarsenResult coarsen_forest(
        std::vector<BoxNode>& boxes,
        const CollisionChecker& /*checker*/,
        int max_rounds) {
    CoarsenResult result;
    result.boxes_before = static_cast<int>(boxes.size());
    if (boxes.empty()) {
        result.boxes_after = 0;
        return result;
    }

    const int n_dims = boxes[0].n_dims();

    for (int round = 0; round < max_rounds; ++round) {
        result.rounds++;
        int merges_this_round = 0;

        for (int dim = 0; dim < n_dims; ++dim) {
            // Sort by lo bound in this dimension
            std::sort(boxes.begin(), boxes.end(),
                      [dim](const BoxNode& a, const BoxNode& b) {
                          return a.joint_intervals[dim].lo
                               < b.joint_intervals[dim].lo;
                      });

            // Scan for exact-touching pairs (look beyond just consecutive)
            for (size_t i = 0; i + 1 < boxes.size(); ) {
                bool merged = false;
                for (size_t j = i + 1; j < boxes.size(); ++j) {
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

                    // Merge j into i
                    boxes[i].joint_intervals[dim].hi =
                        boxes[j].joint_intervals[dim].hi;
                    boxes[i].compute_volume();
                    boxes[i].tree_id = -1;

                    boxes.erase(boxes.begin()
                                + static_cast<std::ptrdiff_t>(j));
                    merges_this_round++;
                    result.merges_performed++;
                    merged = true;
                    break;  // restart from i to catch chains
                }
                if (!merged) ++i;
            }
        }

        if (merges_this_round == 0) break;
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
        LECT* lect) {
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
        int fk_budget_used = 0;

        for (auto& cand : candidates) {
            if (config.target_boxes > 0 &&
                static_cast<int>(boxes.size())
                    - static_cast<int>(remove_ids.size())
                    <= config.target_boxes)
                break;

            if (consumed_ids.count(cand.id_a) ||
                consumed_ids.count(cand.id_b))
                continue;

            auto it_a = id_to_idx.find(cand.id_a);
            auto it_b = id_to_idx.find(cand.id_b);
            if (it_a == id_to_idx.end() || it_b == id_to_idx.end())
                continue;

            // ── Collision check ─────────────────────────────────────────
            bool hull_safe = !checker.check_box(cand.hull_ivs);

            if (!hull_safe && lect != nullptr &&
                fk_budget_used < config.max_lect_fk_per_round) {
                hull_safe = !lect->intervals_collide_scene(
                    cand.hull_ivs,
                    checker.obstacles(),
                    checker.n_obs());
                fk_budget_used++;
            }

            if (!hull_safe) continue;

            // ── Execute merge ───────────────────────────────────────────
            const BoxNode& box_a = boxes[it_a->second];
            const BoxNode& box_b = boxes[it_b->second];

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
                            double min_gmean_edge) {
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
            if (box_contains_box(boxes[i], boxes[j])) {
                removed[j] = true;
            }
        }
    }

    // Pass 2: trim partial overlaps — shrink the smaller box
    for (size_t i = 0; i < boxes.size(); ++i) {
        if (removed[i]) continue;
        for (size_t j = i + 1; j < boxes.size(); ++j) {
            if (removed[j]) continue;
            if (!boxes_overlap(boxes[i], boxes[j])) continue;

            // Trim box j (smaller) away from box i (larger) along the
            // dimension where trimming preserves the most volume.
            BoxNode& big = boxes[i];
            BoxNode& small = boxes[j];
            int nd = small.n_dims();

            int best_dim = -1;
            int best_side = -1;   // 0 = clip lo, 1 = clip hi
            double best_remaining = -1.0;

            for (int d = 0; d < nd; ++d) {
                double overlap_lo = std::max(big.joint_intervals[d].lo,
                                             small.joint_intervals[d].lo);
                double overlap_hi = std::min(big.joint_intervals[d].hi,
                                             small.joint_intervals[d].hi);
                if (overlap_lo >= overlap_hi) continue;

                // Option A: clip small's lo up to overlap_hi
                double remaining_a = small.joint_intervals[d].hi - overlap_hi;
                // Option B: clip small's hi down to overlap_lo
                double remaining_b = overlap_lo - small.joint_intervals[d].lo;

                if (remaining_a > best_remaining) {
                    best_remaining = remaining_a;
                    best_dim = d;
                    best_side = 0;  // raise lo
                }
                if (remaining_b > best_remaining) {
                    best_remaining = remaining_b;
                    best_dim = d;
                    best_side = 1;  // lower hi
                }
            }

            if (best_dim >= 0) {
                double overlap_lo = std::max(big.joint_intervals[best_dim].lo,
                                             small.joint_intervals[best_dim].lo);
                double overlap_hi = std::min(big.joint_intervals[best_dim].hi,
                                             small.joint_intervals[best_dim].hi);
                if (best_side == 0)
                    small.joint_intervals[best_dim].lo = overlap_hi;
                else
                    small.joint_intervals[best_dim].hi = overlap_lo;
                small.compute_volume();

                // Delete if too small
                if (geometric_mean_edge(small) < min_gmean_edge) {
                    removed[j] = true;
                }
            }
        }
    }

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

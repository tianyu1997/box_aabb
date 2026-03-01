// SafeBoxForest — Coarsen: dimension-scan greedy merge
#pragma once

#include "sbf/core/types.h"
#include "sbf/forest/safe_box_forest.h"

namespace sbf {

class HierAABBTree;  // forward declaration

struct CoarsenResult {
    int merges_performed = 0;
    int rounds = 0;
    int boxes_before = 0;
    int boxes_after = 0;
};

// Coarsen forest by merging adjacent boxes that can be safely combined
// Uses dimension-scan approach: for each dimension, find touching runs
// and attempt to merge them.
CoarsenResult coarsen_forest(SafeBoxForest& forest,
                              const CollisionChecker& checker,
                              int max_rounds = 20);

// ─── Greedy adjacency-based merge ────────────────────────────────────────
// Iteratively merge adjacent box pairs whose hull AABB is collision-free.
// Unlike coarsen_forest (which requires exact alignment in other dims),
// this considers ANY adjacent pair and uses collision checking to validate.
//
// Algorithm per round:
//   1. Rebuild adjacency
//   2. For each adjacent pair, compute hull AABB and check collision
//   3. Sort valid candidates by merge score (hull_vol / sum_vol, lower=better)
//   4. Greedily execute non-conflicting merges (each box merged at most once)
//   5. Repeat until target_boxes reached or no merges found
//
// Parameters:
//   target_boxes: stop when n_boxes <= target. 0 = run until convergence.
//   max_rounds: safety limit on number of merge rounds.
//   adjacency_tol: tolerance for adjacency rebuild.
struct GreedyCoarsenResult {
    int merges_performed = 0;
    int rounds = 0;
    int boxes_before = 0;
    int boxes_after = 0;
    double elapsed_sec = 0.0;
};

GreedyCoarsenResult coarsen_greedy(SafeBoxForest& forest,
                                    const CollisionChecker& checker,
                                    int target_boxes = 0,
                                    int max_rounds = 100,
                                    double adjacency_tol = 1e-10,
                                    const HierAABBTree* tree = nullptr,
                                    int tree_split_depth = 3,
                                    int max_tree_fk_per_round = 2000);

} // namespace sbf

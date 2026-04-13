#pragma once
/// @file coarsen.h
/// @brief Forest coarsening: dimension-sweep merge + greedy adjacency merge.
///
/// After growing, the forest typically contains many small boxes.
/// Coarsening merges them into fewer, larger boxes:
///
///   1. **Dimension-sweep** (`coarsen_forest`): merge exact-face-touching
///      boxes along each axis.  No collision check needed (hull guaranteed
///      collision-free by construction).
///
///   2. **Greedy merge** (`coarsen_greedy`): iteratively merge adjacent
///      box pairs whose AABB hull passes collision checking, prioritising
///      tightest merges (lowest hull_vol / sum_vol ratio).
///
///   3. **Overlap filter** (`filter_coarsen_overlaps`): remove boxes fully
///      contained inside larger boxes and trim partial overlaps.

#include <sbf/core/types.h>
#include <sbf/scene/collision_checker.h>
#include <sbf/lect/lect.h>

#include <unordered_set>
#include <vector>

namespace sbf {

// ─── Dimension-Sweep Merge result ───────────────────────────────────────────
struct CoarsenResult {
    int merges_performed = 0;
    int rounds = 0;
    int boxes_before = 0;
    int boxes_after = 0;
};

// Dimension-sweep merge: merge exact-face-touching boxes along each axis.
// No collision check needed (safe hull guaranteed by exact alignment).
CoarsenResult coarsen_forest(
    std::vector<BoxNode>& boxes,
    const CollisionChecker& checker,
    int max_rounds = 20);

// Relaxed dimension-sweep merge: merge boxes touching in one dimension with
// overlapping (not necessarily identical) ranges in other dims.  Hull AABB is
// collision-checked because hull ⊃ A∪B when dims don't match exactly.
CoarsenResult coarsen_sweep_relaxed(
    std::vector<BoxNode>& boxes,
    const CollisionChecker& checker,
    LECT* lect = nullptr,
    int max_rounds = 10,
    double score_threshold = 10.0,
    int max_lect_fk_per_round = 5000);

// ─── Greedy Coarsening config + result ──────────────────────────────────────
struct GreedyCoarsenConfig {
    int target_boxes = 0;             // 0 = run until convergence
    int max_rounds = 100;
    double adjacency_tol = 1e-10;
    double score_threshold = 200.0;   // hull_vol/sum_vol upper bound (was 50)
    int max_lect_fk_per_round = 20000; // LECT FK budget per round (was 2000)
};

struct GreedyCoarsenResult {
    int merges_performed = 0;
    int rounds = 0;
    int boxes_before = 0;
    int boxes_after = 0;
    double elapsed_sec = 0.0;
};

// Greedy adjacency-based coarsening.
// Iteratively merge adjacent box pairs whose hull AABB is collision-free,
// prioritising tightest merges (lowest hull_vol / sum_vol score).
GreedyCoarsenResult coarsen_greedy(
    std::vector<BoxNode>& boxes,
    const CollisionChecker& checker,
    const GreedyCoarsenConfig& config,
    LECT* lect = nullptr,
    const std::unordered_set<int>* protected_ids = nullptr);

// Post-coarsen overlap filter:
//  1. Remove boxes fully contained inside a larger box.
//  2. Trim partially overlapping boxes (assign overlap region to the larger box).
//  3. Delete trimmed boxes whose geometric-mean edge < min_gmean_edge.
// Returns number of boxes removed.
int filter_coarsen_overlaps(std::vector<BoxNode>& boxes,
                            double min_gmean_edge = 1e-4,
                            const std::unordered_set<int>* protected_ids = nullptr);

// ─── Cluster merge config + result ──────────────────────────────────────────
struct ClusterCoarsenConfig {
    int max_rounds = 10;
    int max_cluster_size = 8;         // max boxes per cluster
    double score_threshold = 15.0;    // hull_vol/sum_vol upper bound
    double adjacency_tol = 1e-10;
    int max_lect_fk_per_round = 5000; // LECT FK budget per round
};

struct ClusterCoarsenResult {
    int merges_performed = 0;         // total boxes absorbed
    int clusters_formed = 0;          // number of successful clusters
    int rounds = 0;
    int boxes_before = 0;
    int boxes_after = 0;
    double elapsed_sec = 0.0;
};

// Multi-box cluster merge: greedily grow neighbor clusters around each box,
// collision-check the hull AABB, and merge 2-8 boxes at a time.
ClusterCoarsenResult coarsen_cluster(
    std::vector<BoxNode>& boxes,
    const CollisionChecker& checker,
    const ClusterCoarsenConfig& config = {},
    LECT* lect = nullptr,
    const std::unordered_set<int>* protected_ids = nullptr);

}  // namespace sbf

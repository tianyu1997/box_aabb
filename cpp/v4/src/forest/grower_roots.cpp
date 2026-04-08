// SafeBoxForest v4 — Grower roots: partition_uniform, partition_lect_aligned,
//                                   select_roots_in_partitions
// Ported from v3 forest_grower.cpp §partition / §root-selection
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/forest/forest_grower.h"
#include "sbf/forest/lect.h"

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <random>
#include <vector>

namespace sbf {
namespace forest {

// ═══════════════════════════════════════════════════════════════════════════
//  partition_uniform — Widest-dimension bisection (arbitrary n_roots)
// ═══════════════════════════════════════════════════════════════════════════
void ForestGrower::partition_uniform() {
    subtrees_.clear();
    int n_dims  = robot_->n_joints();
    int n_roots = config_.n_roots;
    const auto& jl = robot_->joint_limits().limits;

    struct Cell { std::vector<Interval> limits; };

    std::vector<Cell> cells;
    {
        Cell full;
        full.limits.resize(n_dims);
        for (int d = 0; d < n_dims; ++d)
            full.limits[d] = jl[d];
        cells.push_back(std::move(full));
    }

    // Repeatedly bisect the cell with the widest single dimension
    while (static_cast<int>(cells.size()) < n_roots) {
        int best_ci  = 0;
        int best_dim = 0;
        double best_w = 0.0;
        for (int ci = 0; ci < static_cast<int>(cells.size()); ++ci) {
            for (int d = 0; d < n_dims; ++d) {
                double w = cells[ci].limits[d].width();
                if (w > best_w) { best_w = w; best_ci = ci; best_dim = d; }
            }
        }
        double mid = cells[best_ci].limits[best_dim].center();

        Cell right = cells[best_ci];            // copy
        right.limits[best_dim].lo = mid;
        cells[best_ci].limits[best_dim].hi = mid;   // shrink left in-place
        cells.push_back(std::move(right));
    }

    subtrees_.resize(n_roots);
    for (int i = 0; i < n_roots; ++i) {
        subtrees_[i].root_id = i;
        subtrees_[i].limits  = std::move(cells[i].limits);
    }

    std::printf("[grower] partition_uniform: %d cells, %d dims\n", n_roots, n_dims);
}

// ═══════════════════════════════════════════════════════════════════════════
//  partition_lect_aligned — Match LECT KD-tree splits (power-of-2 n_roots)
//
//  v4 adaptation: reads actual per-node split dimensions from
//  lect_.get_node_split_dim() and split values from lect_.split_val()
//  instead of assuming fixed round-robin (dim = depth % n_dims).
//  This supports BEST_TIGHTEN / WIDEST_FIRST split orders.
// ═══════════════════════════════════════════════════════════════════════════
void ForestGrower::partition_lect_aligned() {
    subtrees_.clear();
    int n_dims  = robot_->n_joints();
    int n_roots = config_.n_roots;
    const auto& jl = robot_->joint_limits().limits;

    assert((n_roots & (n_roots - 1)) == 0 &&
           "n_roots must be power of 2 for LECT-aligned partition");

    int partition_depth = 0;
    { int tmp = n_roots; while (tmp > 1) { tmp >>= 1; ++partition_depth; } }

    // Pre-expand LECT to partition depth so nodes exist and have split dims
    int ws_new = lect_.pre_expand(partition_depth);
    std::printf("[grower] partition_lect_aligned: pre-expanded LECT to depth %d, "
                "%d new nodes, %d total\n",
                partition_depth, ws_new, lect_.n_nodes());
    if (deadline_reached()) return;

    // BFS through LECT to build partition cells
    struct CellState {
        std::vector<Interval> limits;
        int lect_node;
        int depth;
    };

    std::vector<CellState> queue;
    {
        CellState root_cell;
        root_cell.limits.resize(n_dims);
        for (int d = 0; d < n_dims; ++d)
            root_cell.limits[d] = jl[d];
        root_cell.lect_node = 0;
        root_cell.depth = 0;
        queue.push_back(std::move(root_cell));
    }

    std::vector<CellState> leaves;
    while (!queue.empty()) {
        std::vector<CellState> next;
        for (auto& cell : queue) {
            if (cell.depth >= partition_depth) {
                leaves.push_back(std::move(cell));
                continue;
            }
            // v4 key difference: read actual split dim from LECT
            // (supports BEST_TIGHTEN, WIDEST_FIRST, not just round-robin)
            int dim = lect_.get_node_split_dim(cell.lect_node);
            double mid = lect_.split_val(cell.lect_node);

            CellState left_cell;
            left_cell.limits = cell.limits;
            left_cell.limits[dim].hi = mid;
            left_cell.lect_node = lect_.left(cell.lect_node);
            left_cell.depth = cell.depth + 1;

            CellState right_cell;
            right_cell.limits = cell.limits;
            right_cell.limits[dim].lo = mid;
            right_cell.lect_node = lect_.right(cell.lect_node);
            right_cell.depth = cell.depth + 1;

            next.push_back(std::move(left_cell));
            next.push_back(std::move(right_cell));
        }
        queue = std::move(next);
    }

    assert(static_cast<int>(leaves.size()) == n_roots);

    subtrees_.resize(n_roots);
    for (int i = 0; i < n_roots; ++i) {
        subtrees_[i].root_id       = i;
        subtrees_[i].limits        = std::move(leaves[i].limits);
        subtrees_[i].lect_node_idx = leaves[i].lect_node;
    }

    std::printf("[grower] partition_lect_aligned: %d cells, %d dims, "
                "partition_depth=%d\n", n_roots, n_dims, partition_depth);
}

// ═══════════════════════════════════════════════════════════════════════════
//  select_roots_in_partitions — Best-of-N root placement within each cell
// ═══════════════════════════════════════════════════════════════════════════
void ForestGrower::select_roots_in_partitions(const Obstacle* obs, int n_obs) {
    double rme = (config_.root_min_edge > 0.0) ? config_.root_min_edge : -1.0;
    double probe_me = (rme > 0.0) ? rme : config_.min_edge;
    int n_dims = robot_->n_joints();

    const int n_candidates = std::max(1, config_.root_n_candidates);

    for (int ci = 0; ci < static_cast<int>(subtrees_.size()); ++ci) {
        if (deadline_reached()) return;
        const auto& lims = subtrees_[ci].limits;
        int rid = subtrees_[ci].root_id;

        // Compute cell volume for early-exit threshold
        double cell_vol = 1.0;
        for (int d = 0; d < n_dims; ++d)
            cell_vol *= lims[d].width();
        const double vol_threshold = cell_vol * 0.5;

        Eigen::VectorXd best_seed;
        double best_vol = -1.0;

        for (int k = 0; k < n_candidates; ++k) {
            if (deadline_reached()) return;
            Eigen::VectorXd seed(n_dims);
            std::uniform_real_distribution<double> u01(0.0, 1.0);
            for (int d = 0; d < n_dims; ++d)
                seed[d] = lims[d].lo + u01(rng_) * lims[d].width();

            FFBResult ffb = lect_.find_free_box(
                seed, obs, n_obs, probe_me, config_.max_depth);
            if (!ffb.success()) continue;

            auto intervals = lect_.node_intervals(ffb.node_idx);
            double vol = 1.0;
            for (const auto& iv : intervals)
                vol *= iv.width();

            if (vol > best_vol) {
                best_vol = vol;
                best_seed = seed;
                // Early exit: root covers >50% of cell — good enough
                if (best_vol > vol_threshold) break;
            }
        }

        if (best_vol > 0.0) {
            int bid = try_create_box(best_seed, obs, n_obs, -1, -1, -1, rid, rme);
            if (bid >= 0) {
                std::printf("[grower] cell %d: best-of-%d root, probe_vol=%.4f\n",
                            ci, n_candidates, best_vol);
                continue;
            }
        }

        // Fallback: random attempts if best-of-N failed
        for (int attempt = 0; attempt < 100; ++attempt) {
            if (deadline_reached()) return;
            Eigen::VectorXd seed(n_dims);
            std::uniform_real_distribution<double> u01(0.0, 1.0);
            for (int d = 0; d < n_dims; ++d)
                seed[d] = lims[d].lo + u01(rng_) * lims[d].width();

            int bid = try_create_box(seed, obs, n_obs, -1, -1, -1, rid, rme);
            if (bid >= 0) {
                std::printf("[grower] cell %d: fallback root (attempt %d)\n",
                            ci, attempt);
                break;
            }
        }
    }
}

} // namespace forest
} // namespace sbf

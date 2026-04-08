// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v3 — SBF implementation
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/forest/sbf.h"

#include <algorithm>
#include <chrono>
#include <cstdio>

namespace sbf {
namespace forest {

// ─────────────────────────────────────────────────────────────────────────
//  Construction
// ─────────────────────────────────────────────────────────────────────────
SafeBoxForest::SafeBoxForest(const Robot& robot, const SBFConfig& config)
    : robot_(&robot)
    , config_(config)
    , lect_(robot, config.pipeline)
{
}

// ─────────────────────────────────────────────────────────────────────────
//  is_grid_envelope — check if envelope type is grid-based
// ─────────────────────────────────────────────────────────────────────────
bool SafeBoxForest::is_grid_envelope() const {
    auto et = config_.pipeline.envelope.type;
    return et == envelope::EnvelopeType::SubAABB_Grid
        || et == envelope::EnvelopeType::Hull16_Grid;
}

// ─────────────────────────────────────────────────────────────────────────
//  sample_random — uniform random sample within joint limits
// ─────────────────────────────────────────────────────────────────────────
Eigen::VectorXd SafeBoxForest::sample_random(std::mt19937_64& rng) const {
    const auto& limits = robot_->joint_limits().limits;
    int n = static_cast<int>(limits.size());
    Eigen::VectorXd q(n);
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    for (int d = 0; d < n; ++d) {
        double t = dist(rng);
        q[d] = limits[d].lo + t * limits[d].width();
    }
    return q;
}

// ═════════════════════════════════════════════════════════════════════════
//  build — main entry point: random sampling → FFB → promotion
// ═════════════════════════════════════════════════════════════════════════
SBFResult SafeBoxForest::build(const Obstacle* obstacles, int n_obs) {
    auto t0 = std::chrono::high_resolution_clock::now();

    SBFResult result;
    boxes_.clear();

    // Pre-rasterize scene for hull collision if grid-based
    if (is_grid_envelope()) {
        lect_.set_scene(obstacles, n_obs);
    }

    std::mt19937_64 rng(config_.rng_seed);
    int box_id = 0;

    // ── Phase 1: Random sampling + FFB ──────────────────────────────────
    for (int s = 0; s < config_.max_seeds; ++s) {
        Eigen::VectorXd seed = sample_random(rng);
        result.n_seeds_tried++;

        FFBResult ffb = lect_.find_free_box(
            seed, obstacles, n_obs,
            config_.min_edge, config_.max_depth);

        if (!ffb.success()) {
            result.n_ffb_fail++;
            continue;
        }

        result.n_ffb_success++;

        // Build BoxNode from the FFB result
        std::vector<Interval> intervals = lect_.node_intervals(ffb.node_idx);

        BoxNode box(box_id, intervals, seed);
        box.tree_id = ffb.node_idx;

        // Mark the LECT node as occupied
        lect_.mark_occupied(ffb.node_idx, box_id);

        boxes_.push_back(std::move(box));
        box_id++;
    }

    // ── Phase 2: Promotion (merge sibling leaves upward) ────────────────
    result.n_promotions = promote_all(obstacles, n_obs);

    // ── Collect statistics ───────────────────────────────────────────────
    result.boxes = boxes_;
    result.n_tree_nodes = lect_.n_nodes();
    result.total_volume = 0.0;
    for (const auto& b : boxes_)
        result.total_volume += b.volume;

    auto t1 = std::chrono::high_resolution_clock::now();
    result.build_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::printf("[SBF] build done: %d seeds, %d boxes, %d promotions, "
                "vol=%.6f, %.1f ms\n",
                result.n_seeds_tried,
                static_cast<int>(boxes_.size()),
                result.n_promotions,
                result.total_volume,
                result.build_time_ms);

    return result;
}

// ═════════════════════════════════════════════════════════════════════════
//  promote_all — iterative bottom-up promotion until convergence
// ═════════════════════════════════════════════════════════════════════════
int SafeBoxForest::promote_all(const Obstacle* obstacles, int n_obs) {
    int total_promoted = 0;
    bool changed = true;

    while (changed) {
        changed = false;
        int n_nodes = lect_.n_nodes();

        // Scan all internal nodes (non-leaf) for promotion candidates
        for (int i = 0; i < n_nodes; ++i) {
            if (lect_.is_leaf(i)) continue;

            if (try_promote_node(i, obstacles, n_obs)) {
                total_promoted++;
                changed = true;
            }
        }
    }

    return total_promoted;
}

// ─────────────────────────────────────────────────────────────────────────
//  try_promote_node — check if both children are forest leaves, try merge
// ─────────────────────────────────────────────────────────────────────────
bool SafeBoxForest::try_promote_node(int node_idx,
                                     const Obstacle* obstacles, int n_obs) {
    int li = lect_.left(node_idx);
    int ri = lect_.right(node_idx);
    if (li < 0 || ri < 0) return false;

    // Both children must be occupied leaves in the forest
    if (!lect_.is_leaf(li) || !lect_.is_leaf(ri)) return false;
    if (!lect_.is_occupied(li) || !lect_.is_occupied(ri)) return false;

    // Parent must not be occupied already
    if (lect_.is_occupied(node_idx)) return false;

    // Dispatch merge strategy based on envelope type
    bool ok = is_grid_envelope()
        ? try_promote_grid(node_idx, obstacles, n_obs)
        : try_promote_aabb(node_idx, obstacles, n_obs);

    if (!ok) return false;

    // ── Promotion succeeded ─────────────────────────────────────────
    int li_box_id = lect_.forest_id(li);
    int ri_box_id = lect_.forest_id(ri);

    // Unmark children
    lect_.unmark_occupied(li);
    lect_.unmark_occupied(ri);

    // Remove children from boxes_ list
    boxes_.erase(
        std::remove_if(boxes_.begin(), boxes_.end(),
            [li_box_id, ri_box_id](const BoxNode& b) {
                return b.id == li_box_id || b.id == ri_box_id;
            }),
        boxes_.end());

    // Create new parent box
    int new_id = 0;
    for (const auto& b : boxes_)
        new_id = std::max(new_id, b.id + 1);

    std::vector<Interval> parent_ivs = lect_.node_intervals(node_idx);
    Eigen::VectorXd parent_center(static_cast<int>(parent_ivs.size()));
    for (int d = 0; d < static_cast<int>(parent_ivs.size()); ++d)
        parent_center[d] = parent_ivs[d].center();

    BoxNode parent_box(new_id, parent_ivs, parent_center);
    parent_box.tree_id = node_idx;

    lect_.mark_occupied(node_idx, new_id);
    boxes_.push_back(std::move(parent_box));

    return true;
}

// ─────────────────────────────────────────────────────────────────────────
//  try_promote_aabb — AABB-based promotion for SubAABB envelope
// ─────────────────────────────────────────────────────────────────────────
bool SafeBoxForest::try_promote_aabb(int node_idx,
                                     const Obstacle* obstacles, int n_obs) {
    // Parent AABB is already computed and refined in LECT.
    // Check if parent AABB collides with any obstacle.
    if (!lect_.has_aabb(node_idx)) return false;

    // Use LECT's internal AABB collision check.
    // collides_scene returns true if collision detected → we want no collision.
    bool collides = lect_.collides_scene(node_idx, obstacles, n_obs);
    return !collides;
}

// ─────────────────────────────────────────────────────────────────────────
//  try_promote_grid — Grid-based promotion for SubAABB_Grid / Hull16_Grid
// ─────────────────────────────────────────────────────────────────────────
bool SafeBoxForest::try_promote_grid(int node_idx,
                                     const Obstacle* obstacles, int n_obs) {
    // Merge children hull grids into parent (bitwise-OR)
    if (!lect_.merge_children_hulls(node_idx))
        return false;

    // Check if merged parent hull grid collides with scene
    bool collides = lect_.collides_scene(node_idx, obstacles, n_obs);
    return !collides;
}

} // namespace forest
} // namespace sbf

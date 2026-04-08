// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v3 — SBF: SafeBoxForest builder
//  Module: sbf::forest
//
//  Top-level class that owns a LECT tree and builds a forest of
//  collision-free C-space boxes.  Two-stage modular pipeline:
//
//    Stage 1: Endpoint AABB generation
//      {IFK, CritSample, Analytical, GCPC}
//
//    Stage 2: Link envelope construction
//      {SubAABB, SubAABB_Grid, Hull16_Grid}
//
//    4 × 3 = 12 pipeline combinations.
//
//  Promotion (leaf merging) strategy depends on envelope type:
//    - SubAABB:                       AABB union → collision check
//    - SubAABB_Grid / Hull16_Grid:    grid merge (bitwise-OR) → grid collision
//
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/forest/lect.h"
#include "sbf/envelope/envelope_type.h"
#include "sbf/robot/robot.h"
#include "sbf/common/types.h"

#include <Eigen/Core>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace sbf {
namespace forest {

// ─── SBF Configuration ─────────────────────────────────────────────────────
struct SBFConfig {
    int max_seeds  = 100;       // Number of random seeds to try
    double min_edge = 0.01;     // Stop subdivision when edge < this
    int max_depth   = 30;       // Maximum KD-tree depth

    // Pipeline: endpoint source + link envelope (one of 4×3 combos)
    envelope::PipelineConfig pipeline = envelope::PipelineConfig::recommended();

    // Random number generator seed
    uint64_t rng_seed = 42;

    // ══════════════════════════════════════════════════════════════════
    //  Factory methods for 4 × 3 = 12 pipeline combinations
    //  (+ backward-compatible *_aabb aliases → SubAABB with n_sub=1)
    // ══════════════════════════════════════════════════════════════════

    // ── IFK × 3 envelopes ───────────────────────────────────────────

    static SBFConfig ifk_sub_aabb(int max_seeds = 100, double min_edge = 0.01) {
        SBFConfig c;
        c.max_seeds = max_seeds; c.min_edge = min_edge;
        c.pipeline.source = envelope::EndpointSourceConfig::ifk();
        c.pipeline.envelope = envelope::EnvelopeTypeConfig::sub_aabb();
        return c;
    }

    static SBFConfig ifk_sub_aabb_grid(int max_seeds = 100, double min_edge = 0.01) {
        SBFConfig c;
        c.max_seeds = max_seeds; c.min_edge = min_edge;
        c.pipeline.source = envelope::EndpointSourceConfig::ifk();
        c.pipeline.envelope = envelope::EnvelopeTypeConfig::sub_aabb_grid();
        return c;
    }

    static SBFConfig ifk_hull16_grid(int max_seeds = 100, double min_edge = 0.01) {
        SBFConfig c;
        c.max_seeds = max_seeds; c.min_edge = min_edge;
        c.pipeline.source = envelope::EndpointSourceConfig::ifk();
        c.pipeline.envelope = envelope::EnvelopeTypeConfig::hull16_grid();
        return c;
    }

    // Backward-compatible alias: AABB ≡ SubAABB(n_sub=1)
    static SBFConfig ifk_aabb(int max_seeds = 100, double min_edge = 0.01) {
        return ifk_sub_aabb(max_seeds, min_edge);
    }

    // ── CritSample × 3 envelopes ───────────────────────────────────

    static SBFConfig crit_sub_aabb(int max_seeds = 100, double min_edge = 0.01) {
        SBFConfig c;
        c.max_seeds = max_seeds; c.min_edge = min_edge;
        c.pipeline.source = envelope::EndpointSourceConfig::crit_sampling();
        c.pipeline.envelope = envelope::EnvelopeTypeConfig::sub_aabb();
        return c;
    }

    static SBFConfig crit_sub_aabb_grid(int max_seeds = 100, double min_edge = 0.01) {
        SBFConfig c;
        c.max_seeds = max_seeds; c.min_edge = min_edge;
        c.pipeline.source = envelope::EndpointSourceConfig::crit_sampling();
        c.pipeline.envelope = envelope::EnvelopeTypeConfig::sub_aabb_grid();
        return c;
    }

    static SBFConfig crit_hull16_grid(int max_seeds = 100, double min_edge = 0.01) {
        SBFConfig c;
        c.max_seeds = max_seeds; c.min_edge = min_edge;
        c.pipeline.source = envelope::EndpointSourceConfig::crit_sampling();
        c.pipeline.envelope = envelope::EnvelopeTypeConfig::hull16_grid();
        return c;
    }

    // Backward-compatible alias
    static SBFConfig crit_aabb(int max_seeds = 100, double min_edge = 0.01) {
        return crit_sub_aabb(max_seeds, min_edge);
    }

    // ── Analytical × 3 envelopes ────────────────────────────────────

    static SBFConfig analytical_sub_aabb(int max_seeds = 100, double min_edge = 0.01) {
        SBFConfig c;
        c.max_seeds = max_seeds; c.min_edge = min_edge;
        c.pipeline.source = envelope::EndpointSourceConfig::analytical();
        c.pipeline.envelope = envelope::EnvelopeTypeConfig::sub_aabb();
        return c;
    }

    static SBFConfig analytical_sub_aabb_grid(int max_seeds = 100, double min_edge = 0.01) {
        SBFConfig c;
        c.max_seeds = max_seeds; c.min_edge = min_edge;
        c.pipeline.source = envelope::EndpointSourceConfig::analytical();
        c.pipeline.envelope = envelope::EnvelopeTypeConfig::sub_aabb_grid();
        return c;
    }

    static SBFConfig analytical_hull16_grid(int max_seeds = 100, double min_edge = 0.01) {
        SBFConfig c;
        c.max_seeds = max_seeds; c.min_edge = min_edge;
        c.pipeline.source = envelope::EndpointSourceConfig::analytical();
        c.pipeline.envelope = envelope::EnvelopeTypeConfig::hull16_grid_analytical();
        return c;
    }

    // Backward-compatible alias
    static SBFConfig analytical_aabb(int max_seeds = 100, double min_edge = 0.01) {
        return analytical_sub_aabb(max_seeds, min_edge);
    }

    // ── GCPC × 3 envelopes ──────────────────────────────────────────

    static SBFConfig gcpc_sub_aabb(const envelope::GcpcCache* cache,
                                    int max_seeds = 100, double min_edge = 0.01) {
        SBFConfig c;
        c.max_seeds = max_seeds; c.min_edge = min_edge;
        c.pipeline.source = envelope::EndpointSourceConfig::gcpc(cache);
        c.pipeline.envelope = envelope::EnvelopeTypeConfig::sub_aabb();
        return c;
    }

    static SBFConfig gcpc_sub_aabb_grid(const envelope::GcpcCache* cache,
                                         int max_seeds = 100, double min_edge = 0.01) {
        SBFConfig c;
        c.max_seeds = max_seeds; c.min_edge = min_edge;
        c.pipeline.source = envelope::EndpointSourceConfig::gcpc(cache);
        c.pipeline.envelope = envelope::EnvelopeTypeConfig::sub_aabb_grid();
        return c;
    }

    static SBFConfig gcpc_hull16_grid(const envelope::GcpcCache* cache,
                                       int max_seeds = 100, double min_edge = 0.01) {
        SBFConfig c;
        c.max_seeds = max_seeds; c.min_edge = min_edge;
        c.pipeline.source = envelope::EndpointSourceConfig::gcpc(cache);
        c.pipeline.envelope = envelope::EnvelopeTypeConfig::hull16_grid_analytical();
        return c;
    }

    // Backward-compatible alias
    static SBFConfig gcpc_aabb(const envelope::GcpcCache* cache,
                               int max_seeds = 100, double min_edge = 0.01) {
        return gcpc_sub_aabb(cache, max_seeds, min_edge);
    }
};

// ─── SBF Build Result ───────────────────────────────────────────────────────
struct SBFResult {
    std::vector<BoxNode> boxes;     // All collision-free boxes in the forest
    int n_seeds_tried  = 0;         // Total seeds sampled
    int n_ffb_success  = 0;         // Seeds that produced a box
    int n_ffb_fail     = 0;         // Seeds where FFB failed
    int n_promotions   = 0;         // Parent promotions performed
    int n_tree_nodes   = 0;         // Total LECT tree nodes
    double total_volume = 0.0;      // Sum of all box volumes (C-space)
    double build_time_ms = 0.0;     // Wall-clock build time
};

// ═════════════════════════════════════════════════════════════════════════════
//  SafeBoxForest — builder class
// ═════════════════════════════════════════════════════════════════════════════

class SafeBoxForest {
public:
    SafeBoxForest() = default;

    /// Construct from robot model and configuration.
    SafeBoxForest(const Robot& robot, const SBFConfig& config);

    /// Build the forest: random sampling → FFB → promotion.
    SBFResult build(const Obstacle* obstacles, int n_obs);

    // ═════════════════════════════════════════════════════════════════════
    //  Accessors
    // ═════════════════════════════════════════════════════════════════════

    const LECT& lect() const { return lect_; }
    LECT& lect_mut() { return lect_; }
    const std::vector<BoxNode>& boxes() const { return boxes_; }
    int n_boxes() const { return static_cast<int>(boxes_.size()); }
    const SBFConfig& config() const { return config_; }

    /// Whether the envelope type uses voxel grids (affects promotion strategy).
    bool is_grid_envelope() const;

private:
    const Robot* robot_ = nullptr;
    SBFConfig    config_;
    LECT         lect_;
    std::vector<BoxNode> boxes_;

    Eigen::VectorXd sample_random(std::mt19937_64& rng) const;
    int promote_all(const Obstacle* obstacles, int n_obs);
    bool try_promote_node(int node_idx, const Obstacle* obstacles, int n_obs);
    bool try_promote_aabb(int node_idx, const Obstacle* obstacles, int n_obs);
    bool try_promote_grid(int node_idx, const Obstacle* obstacles, int n_obs);
};

} // namespace forest
} // namespace sbf

// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — SBF: SafeBoxForest builder
//  Module: sbf::forest
//
//  Top-level class that owns a LECT tree and builds a forest of
//  collision-free C-space boxes.  Two-stage modular pipeline:
//
//    Stage 1: Endpoint iAABB generation
//      {iFK, CritSample, Analytical, GCPC}  — 4 sources
//
//    Stage 2: Link envelope construction
//      {LinkIAABB, LinkIAABB_Grid, Hull16_Grid}  — 3 representations
//
//    4 × 3 = 12 pipeline combinations.
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/forest/lect.h"
#include "sbf/envelope/pipeline.h"
#include "sbf/robot/robot.h"
#include "sbf/core/types.h"

#include <Eigen/Core>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace sbf {
namespace forest {

// ─── SBF Configuration ─────────────────────────────────────────────────────
struct SBFConfig {
    int max_seeds   = 100;
    double min_edge = 0.01;
    int max_depth   = 30;

    // Pipeline: endpoint source + link envelope (one of 4×3 combos)
    envelope::PipelineConfig pipeline = envelope::PipelineConfig::recommended();

    uint64_t rng_seed = 42;

    // ══════════════════════════════════════════════════════════════════
    //  Factory methods for 4 × 3 = 12 pipeline combinations
    //  v4 命名：使用 link_iaabb 取代 sub_aabb
    // ══════════════════════════════════════════════════════════════════

    // ── iFK × 3 envelopes ───────────────────────────────────────────

    static SBFConfig ifk_link_iaabb(int max_seeds = 100, double min_edge = 0.01) {
        SBFConfig c;
        c.max_seeds = max_seeds; c.min_edge = min_edge;
        c.pipeline.source   = envelope::EndpointSourceConfig::ifk();
        c.pipeline.envelope = envelope::EnvelopeTypeConfig::link_iaabb();
        return c;
    }

    static SBFConfig ifk_link_iaabb_grid(int max_seeds = 100, double min_edge = 0.01) {
        SBFConfig c;
        c.max_seeds = max_seeds; c.min_edge = min_edge;
        c.pipeline.source   = envelope::EndpointSourceConfig::ifk();
        c.pipeline.envelope = envelope::EnvelopeTypeConfig::link_iaabb_grid();
        return c;
    }

    static SBFConfig ifk_hull16_grid(int max_seeds = 100, double min_edge = 0.01) {
        SBFConfig c;
        c.max_seeds = max_seeds; c.min_edge = min_edge;
        c.pipeline.source   = envelope::EndpointSourceConfig::ifk();
        c.pipeline.envelope = envelope::EnvelopeTypeConfig::hull16_grid();
        return c;
    }

    // ── CritSample × 3 envelopes ───────────────────────────────────

    static SBFConfig crit_link_iaabb(int max_seeds = 100, double min_edge = 0.01) {
        SBFConfig c;
        c.max_seeds = max_seeds; c.min_edge = min_edge;
        c.pipeline.source   = envelope::EndpointSourceConfig::crit_sampling();
        c.pipeline.envelope = envelope::EnvelopeTypeConfig::link_iaabb();
        return c;
    }

    static SBFConfig crit_link_iaabb_grid(int max_seeds = 100, double min_edge = 0.01) {
        SBFConfig c;
        c.max_seeds = max_seeds; c.min_edge = min_edge;
        c.pipeline.source   = envelope::EndpointSourceConfig::crit_sampling();
        c.pipeline.envelope = envelope::EnvelopeTypeConfig::link_iaabb_grid();
        return c;
    }

    static SBFConfig crit_hull16_grid(int max_seeds = 100, double min_edge = 0.01) {
        SBFConfig c;
        c.max_seeds = max_seeds; c.min_edge = min_edge;
        c.pipeline.source   = envelope::EndpointSourceConfig::crit_sampling();
        c.pipeline.envelope = envelope::EnvelopeTypeConfig::hull16_grid();
        return c;
    }

    // ── Analytical × 3 envelopes ────────────────────────────────────

    static SBFConfig analytical_link_iaabb(int max_seeds = 100, double min_edge = 0.01) {
        SBFConfig c;
        c.max_seeds = max_seeds; c.min_edge = min_edge;
        c.pipeline.source   = envelope::EndpointSourceConfig::analytical();
        c.pipeline.envelope = envelope::EnvelopeTypeConfig::link_iaabb();
        return c;
    }

    static SBFConfig analytical_link_iaabb_grid(int max_seeds = 100, double min_edge = 0.01) {
        SBFConfig c;
        c.max_seeds = max_seeds; c.min_edge = min_edge;
        c.pipeline.source   = envelope::EndpointSourceConfig::analytical();
        c.pipeline.envelope = envelope::EnvelopeTypeConfig::link_iaabb_grid();
        return c;
    }

    static SBFConfig analytical_hull16_grid(int max_seeds = 100, double min_edge = 0.01) {
        SBFConfig c;
        c.max_seeds = max_seeds; c.min_edge = min_edge;
        c.pipeline.source   = envelope::EndpointSourceConfig::analytical();
        c.pipeline.envelope = envelope::EnvelopeTypeConfig::hull16_grid();
        return c;
    }

    // ── GCPC × 3 envelopes ──────────────────────────────────────────

    static SBFConfig gcpc_link_iaabb(const envelope::GcpcCache* cache,
                                     int max_seeds = 100, double min_edge = 0.01) {
        SBFConfig c;
        c.max_seeds = max_seeds; c.min_edge = min_edge;
        c.pipeline.source   = envelope::EndpointSourceConfig::gcpc(cache);
        c.pipeline.envelope = envelope::EnvelopeTypeConfig::link_iaabb();
        return c;
    }

    static SBFConfig gcpc_link_iaabb_grid(const envelope::GcpcCache* cache,
                                          int max_seeds = 100, double min_edge = 0.01) {
        SBFConfig c;
        c.max_seeds = max_seeds; c.min_edge = min_edge;
        c.pipeline.source   = envelope::EndpointSourceConfig::gcpc(cache);
        c.pipeline.envelope = envelope::EnvelopeTypeConfig::link_iaabb_grid();
        return c;
    }

    static SBFConfig gcpc_hull16_grid(const envelope::GcpcCache* cache,
                                      int max_seeds = 100, double min_edge = 0.01) {
        SBFConfig c;
        c.max_seeds = max_seeds; c.min_edge = min_edge;
        c.pipeline.source   = envelope::EndpointSourceConfig::gcpc(cache);
        c.pipeline.envelope = envelope::EnvelopeTypeConfig::hull16_grid();
        return c;
    }
};

// ─── SBF Build Result ───────────────────────────────────────────────────────
struct SBFResult {
    std::vector<BoxNode> boxes;
    int n_seeds_tried   = 0;
    int n_ffb_success   = 0;
    int n_ffb_fail      = 0;
    int n_promotions    = 0;
    int n_tree_nodes    = 0;
    double total_volume = 0.0;
    double build_time_ms = 0.0;
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

    // ── Accessors ───────────────────────────────────────────────────────
    const LECT& lect() const;
    LECT& lect_mut();
    const std::vector<BoxNode>& boxes() const;
    int n_boxes() const;
    const SBFConfig& config() const;
    bool is_grid_envelope() const;

private:
    const Robot* robot_ = nullptr;
    SBFConfig    config_;
    LECT         lect_;
    std::vector<BoxNode> boxes_;

    // TODO: 从 v3 迁移私有方法实现
    Eigen::VectorXd sample_random(std::mt19937_64& rng) const;
    int promote_all(const Obstacle* obstacles, int n_obs);
    bool try_promote_node(int node_idx, const Obstacle* obstacles, int n_obs);
    bool try_promote_aabb(int node_idx, const Obstacle* obstacles, int n_obs);
    bool try_promote_grid(int node_idx, const Obstacle* obstacles, int n_obs);
};

} // namespace forest
} // namespace sbf

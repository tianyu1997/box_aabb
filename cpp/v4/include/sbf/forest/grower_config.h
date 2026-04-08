// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — GrowerConfig: ForestGrower configuration
//  Module: sbf::forest
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/envelope/pipeline.h"

#include <cstdint>
#include <string>

namespace sbf {
namespace forest {

// ── Partition mode for parallel growth ───────────────────────────────
enum class PartitionMode : uint8_t {
    KDSplit     = 0,   // Root-position-based KD-split (v4 default)
    Uniform     = 1,   // Widest-dimension bisection (any n_roots)
    LectAligned = 2,   // Match LECT KD-tree splits (n_roots must be power of 2)
};

struct GrowerConfig {
    // ── Root selection ──────────────────────────────────────────────────
    int n_roots = 2;

    // ── Expansion mode ──────────────────────────────────────────────────
    enum class Mode { Wavefront, RRT };
    Mode mode = Mode::Wavefront;

    // ── Common parameters ───────────────────────────────────────────────
    int    max_boxes            = 500;
    double min_edge             = 0.01;
    int    max_depth            = 30;
    int    max_consecutive_miss = 200;
    double timeout              = 60.0;
    double adjacency_tol        = 1e-10;
    uint64_t rng_seed           = 42;

    // ── Wavefront-specific ──────────────────────────────────────────────
    int    n_boundary_samples   = 6;
    double boundary_epsilon     = 0.01;
    double goal_face_bias       = 0.6;

    // ── RRT-specific ────────────────────────────────────────────────────
    double rrt_step_ratio       = 0.3;
    double rrt_goal_bias        = 0.1;

    // ── Parallelism ─────────────────────────────────────────────────────
    int    n_threads            = 1;    PartitionMode partition_mode = PartitionMode::LectAligned;
    // ── Adaptive min_edge ───────────────────────────────────────────────
    bool   adaptive_min_edge    = false;
    double coarse_min_edge      = 0.1;
    double coarse_fraction      = 0.6;
    int    adaptive_n_stages    = 2;

    // ── Boundary bridging ───────────────────────────────────────────────
    int    bridge_samples       = 30;

    // ── Root selection tuning ───────────────────────────────────────────
    double root_min_edge        = 0.0;
    int    root_n_candidates    = 50;

    // ── Hull skip threshold ─────────────────────────────────────────────
    double hull_skip_vol        = 0.0;

    // ── Coarsening (greedy merge) ───────────────────────────────────────
    bool   coarsen_enabled           = false;
    int    coarsen_target_boxes      = 0;
    int    max_coarsen_rounds        = 50;
    double coarsen_score_threshold   = 50.0;

    // ── LECT warm-start ─────────────────────────────────────────────────
    int    warm_start_depth     = 0;

    // ── Offline LECT cache ──────────────────────────────────────────────
    std::string lect_cache_dir;

    // ── Pipeline (forwarded to LECT) ────────────────────────────────────
    envelope::PipelineConfig pipeline = envelope::PipelineConfig::recommended();
};

} // namespace forest
} // namespace sbf

// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v3 — GrowerConfig: ForestGrower configuration
//  Module: sbf::forest
//
//  Configuration for multi-root forest growing with two expansion modes:
//    - Wavefront: BFS boundary expansion with goal-directed face bias
//    - RRT: random sampling + nearest-box extension
//
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/envelope/envelope_type.h"

#include <cstdint>
#include <string>

namespace sbf {
namespace forest {

struct GrowerConfig {
    // ── Root selection ──────────────────────────────────────────────────
    int n_roots = 2;            // Number of roots (FPS selection)

    // ── Expansion mode ──────────────────────────────────────────────────
    enum class Mode { Wavefront, RRT };
    Mode mode = Mode::Wavefront;

    // ── Common parameters ───────────────────────────────────────────────
    int    max_boxes            = 500;      // Stop after this many boxes
    double min_edge             = 0.01;     // FFB minimum edge length
    int    max_depth            = 30;       // FFB maximum KD-tree depth
    int    max_consecutive_miss = 200;      // Stop after N consecutive FFB failures
    double timeout              = 60.0;     // Wall-clock timeout (seconds)
    double adjacency_tol        = 1e-10;    // Tolerance for face-adjacency test
    uint64_t rng_seed           = 42;

    // ── Wavefront-specific ──────────────────────────────────────────────
    int    n_boundary_samples   = 6;        // Boundary seeds per box face
    double boundary_epsilon     = 0.01;     // Offset distance beyond box face
    double goal_face_bias       = 0.6;      // Probability of choosing goal-facing face

    // ── RRT-specific ────────────────────────────────────────────────────
    double rrt_step_ratio       = 0.3;      // Step = ratio × max joint range width
    double rrt_goal_bias        = 0.1;      // Probability of sampling goal as target

    // ── Parallelism ─────────────────────────────────────────────────────
    int    n_threads            = 1;        // Number of worker threads (1 = serial)

    // ── Adaptive min_edge (multi-stage progressive expansion) ────────────
    bool   adaptive_min_edge    = false;    // Enable progressive multi-stage expansion
    double coarse_min_edge      = 0.1;      // Stage 0: coarsest min_edge
    double coarse_fraction      = 0.6;      // Budget fraction for non-final stages combined
    int    adaptive_n_stages    = 2;        // Number of stages (2 = coarse+fine, >2 = bisecting)

    // ── Boundary bridging ───────────────────────────────────────────────
    int    bridge_samples       = 0;        // Seeds per shared face (0 = disabled)

    // ── Root selection tuning ────────────────────────────────────────────
    double root_min_edge        = 0.0;      // min_edge for root FFB (0 = use min_edge)
                                            // Larger value → shallower root descent → faster

    // ── Hull skip threshold ──────────────────────────────────────────────
    double hull_skip_vol        = 0.0;      // Skip hull refinement for nodes with
                                            // C-space volume below this threshold.
                                            // 0 = disabled (always compute hull).

    // ── Coarsening (greedy merge) ────────────────────────────────────────
    bool   coarsen_enabled         = false;  // Enable greedy coarsening after expansion
    int    coarsen_target_boxes    = 0;      // Stop when n_boxes <= target (0 = reduce as much as possible)
    int    max_coarsen_rounds      = 50;     // Maximum greedy rounds
    double coarsen_score_threshold = 50.0;   // Skip candidates with hull_vol/sum_vol > threshold

    // ── LECT warm-start ─────────────────────────────────────────────────
    int    warm_start_depth     = 0;        // Pre-expand LECT skeleton to this depth
                                            // before spawning parallel workers.
                                            // 0 = auto (use n_roots-based heuristic).
                                            // -1 = disabled (each worker starts fresh).

    // ── Offline LECT cache ──────────────────────────────────────────────
    std::string lect_cache_dir;             // Path to pre-built LECT cache directory.
                                            // If non-empty, load cached LECT (tree + AABBs +
                                            // hulls) at startup to skip FK + envelope
                                            // computation during root_select.
                                            // Empty string = no cache (default).

    // ── Pipeline (forwarded to LECT) ────────────────────────────────────
    envelope::PipelineConfig pipeline = envelope::PipelineConfig::recommended();
};

} // namespace forest
} // namespace sbf

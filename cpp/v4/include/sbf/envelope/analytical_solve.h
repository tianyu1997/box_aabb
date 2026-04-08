// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Analytical Critical Solve (Analytical endpoint source)
//  Module: sbf::envelope
//
//  Exact analytical gradient-zero enumeration:
//    Phase 0: kπ/2 vertex enumeration
//    Phase 1: 1D atan2 closed-form (edges)
//    Phase 2: 2D degree-8 polynomial solver (faces)
//    Phase 2.5: Pair-constrained analysis (1D + 2D)
//    Phase 3+: Multi-start coordinate descent (interior)
//
//  v4 optimizations:
//    - AA gap pruning: skip links whose AA bounds fit within current AABB
//
//  迁移自 v3 envelope_derive_critical.h
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/core/types.h"
#include "sbf/robot/robot.h"

#include <Eigen/Core>
#include <cstdint>
#include <vector>

namespace sbf {
namespace envelope {

// ─── AnalyticalCriticalConfig ───────────────────────────────────────────────
struct AnalyticalCriticalConfig {
    // Phase 0: kπ/2 baseline enumeration
    bool keep_kpi2_baseline    = true;

    // Phase 1: 1D edge solver (atan2)
    bool enable_edge_solve     = true;
    // Phase B: P1 direct coefficient extraction via DH chain (bypass 3-point sampling + QR)
    bool enable_p1_direct_coeff = true;

    // Phase 2: 2D face solver (degree-8 polynomial)
    bool enable_face_solve     = true;
    bool face_all_pairs        = true;    // true → all C(n,2) joint pairs for completeness
    // Phase C: P2 direct 2D coefficient extraction via DH chain (bypass 9-point sampling + QR)
    bool enable_p2_direct_coeff = true;

    // Phase 2.5: Pair-constrained solvers
    bool enable_pair_1d        = true;
    bool enable_pair_2d        = true;
    int  pair_max_bg           = 64;
    bool pair_kpi2_backgrounds = true;
    bool pair_all_pairs        = false;
    // Phase D: P2.5 direct coefficient extraction via DH chain (bypass FK sampling + QR)
    bool enable_p25_direct_coeff = true;

    // Phase 3+: Interior solver
    bool enable_interior_solve = true;
    int  interior_max_free     = 7;
    int  interior_max_sweeps   = 3;
    bool improved_interior     = true;
    int  interior_n_restarts   = 4;
    bool interior_pair_coupling = true;
    // Phase F: P3 direct coefficient extraction via DH chain (bypass 3-point sampling + QR)
    bool enable_p3_direct_coeff = true;

    // Dual Phase 3: run Phase 3 from both {P0+P1} and {P0+P1+P2+P2.5} checkpoints
    bool dual_phase3           = true;

    // ── v4 optimization: AA gap pruning ─────────────────────────────
    bool enable_aa_pruning     = true;

    // ── v4 optimization: candidate dedup in analytical phases ───────
    bool enable_candidate_dedup = true;

    // ── v4 optimization: adaptive candidate dedup threshold ─────────
    // If enabled, dedup is applied only when candidate-set size is large enough
    // and recent dedup hit-rate is above threshold (after warmup).
    bool enable_adaptive_candidate_dedup = false;
    int  candidate_dedup_min_candidates = 4;
    int  candidate_dedup_warmup_sets = 32;
    double candidate_dedup_min_hit_rate = 0.02;

    // ── v4 optimization: parallel-by-link execution ─────────────────
    // Execute phase loops by splitting active links (ci) into chunks.
    bool enable_parallel_by_link = false;
    // <=0 means auto-detect hardware concurrency.
    int parallel_num_threads = 0;

    // (O5 proto/shadow/hybrid fields removed in Phase E cleanup)

    // ── Factory methods ─────────────────────────────────────────────
    static AnalyticalCriticalConfig all_enabled() {
        return AnalyticalCriticalConfig{};
    }

    static AnalyticalCriticalConfig edges_only() {
        AnalyticalCriticalConfig c;
        c.enable_face_solve = false;
        c.enable_pair_1d = false;
        c.enable_pair_2d = false;
        c.enable_interior_solve = false;
        return c;
    }

    static AnalyticalCriticalConfig edges_and_faces() {
        AnalyticalCriticalConfig c;
        c.enable_pair_1d = false;
        c.enable_pair_2d = false;
        c.enable_interior_solve = false;
        return c;
    }

    static AnalyticalCriticalConfig v1_analytical() {
        AnalyticalCriticalConfig c;
        c.enable_pair_1d = false;
        c.enable_pair_2d = false;
        c.improved_interior = false;
        c.dual_phase3 = false;
        return c;
    }
};

// ─── AnalyticalCriticalStats ────────────────────────────────────────────────
struct AnalyticalCriticalStats {
    int n_phase0_vertices  = 0;
    int n_phase1_edges     = 0;
    int n_phase2_faces     = 0;
    int n_phase25a_pair1d  = 0;
    int n_phase25b_pair2d  = 0;
    int n_phase3_interior  = 0;

    int n_phase1_fk_calls  = 0;
    int n_phase2_fk_calls  = 0;
    int n_phase25_fk_calls = 0;
    int n_phase3_fk_calls  = 0;

    // v4: AA pruning statistics
    int n_aa_pruned_links  = 0;
    int64_t n_aa_pruned_segments = 0;
    int64_t n_aa_pruned_phase_checks = 0;

    // v4: candidate dedup statistics (for A/B benchmarking)
    int64_t n_dedup_raw_candidates_p1 = 0;
    int64_t n_dedup_unique_candidates_p1 = 0;
    int64_t n_dedup_applied_sets_p1 = 0;
    int64_t n_dedup_skipped_sets_p1 = 0;

    int64_t n_dedup_raw_candidates_p2 = 0;
    int64_t n_dedup_unique_candidates_p2 = 0;
    int64_t n_dedup_applied_sets_p2 = 0;
    int64_t n_dedup_skipped_sets_p2 = 0;

    int64_t n_dedup_raw_candidates_p25 = 0;
    int64_t n_dedup_unique_candidates_p25 = 0;
    int64_t n_dedup_applied_sets_p25 = 0;
    int64_t n_dedup_skipped_sets_p25 = 0;

    int64_t n_dedup_raw_candidates_p3 = 0;
    int64_t n_dedup_unique_candidates_p3 = 0;
    int64_t n_dedup_applied_sets_p3 = 0;
    int64_t n_dedup_skipped_sets_p3 = 0;

    // (O5 proto/shadow/hybrid stats removed in Phase E cleanup)
};

// ─── Primary API ────────────────────────────────────────────────────────────

// Full analytical pipeline: produces per-link sub-segment AABBs.
//   out_aabb:           [(n_active * n_sub) × 6], caller-allocated
//   out_endpoint_aabb:  [(n_active * (n_sub+1)) × 6], optional
void derive_aabb_critical_analytical(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub,
    const AnalyticalCriticalConfig& config,
    float* out_aabb,
    AnalyticalCriticalStats* out_stats = nullptr,
    float* out_endpoint_aabb = nullptr);

// Variant that also outputs extreme configurations
void derive_aabb_critical_analytical_with_configs(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub,
    const AnalyticalCriticalConfig& config,
    float* out_aabb,
    Eigen::VectorXd* out_configs,
    AnalyticalCriticalStats* out_stats = nullptr);

} // namespace envelope
} // namespace sbf

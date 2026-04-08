// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v3 — Endpoint Source Module (Endpoint AABB Generation)
//  Module: sbf::envelope
//
//  Stage 1 of the two-stage modular pipeline:
//    Stage 1: Endpoint AABB generation     (this module)
//    Stage 2: Link envelope construction   (envelope_type.h)
//
//  Four endpoint sources produce a unified intermediate representation
//  called "endpoint_aabb": per-endpoint position interval AABBs (geometry-
//  only, no link radius), stored as [n_endpoints × 6].  Each entry bounds
//  a single kinematic endpoint's position over the C-space box.
//
//  Stage 2 derives per-link AABBs and sub-segment AABBs by interpolating
//  between adjacent endpoints and adding link radius.  This fully
//  decouples HOW critical points are found from HOW envelope geometry is
//  represented, and preserves source-level precision for per-link AABBs
//  (proven: min(inf A, inf B) ≡ inf min(A,B) — zero loss for n_sub=1).
//
//    IFK (Interval FK):
//      Fastest method.  Computes interval forward kinematics, then
//      extracts per-endpoint position intervals directly.
//      Conservative outer bound, may overestimate.
//
//    CritSample (Critical Sampling):
//      Enumerates critical joint configurations (boundary + k*π/2 zeros)
//      and evaluates FK at each.  Tighter than IFK at the cost of many
//      FK evaluations.  3-stage pipeline: coupled + manifold + L-BFGS-B.
//
//    Analytical (Analytical Critical):
//      Exact analytical gradient-zero enumeration.  Provably complete
//      up to 2D sub-problems.  Produces analytical-tight per-endpoint
//      position intervals via out_endpoint_aabb.
//
//    GCPC (Global Critical Point Cache):
//      Cached critical points with on-the-fly AA-pruned enrichment.
//      7-phase pipeline (A–G): KD-tree lookup + boundary + atan2 +
//      polynomial faces + pair-constrained + interior descent.
//      Near-analytical tightness at near-IFK speed for warm queries.
//
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/robot/robot.h"
#include "sbf/robot/interval_fk.h"
#include "sbf/envelope/envelope_derive_critical.h"
#include "sbf/envelope/gcpc_cache.h"
#include "sbf/common/types.h"

#include <vector>
#include <string>

namespace sbf {
namespace envelope {

// ─── Endpoint source method enum ────────────────────────────────────────────
enum class EndpointSource : uint8_t {
    IFK              = 0,  // Interval FK (fast, conservative)
    CritSample       = 1,  // Critical-point sampling (medium, tighter)
    Analytical       = 2,  // Analytical solver (slow, tightest)
    GCPC             = 3   // Global Critical Point Cache + AA pruning
};

inline const char* endpoint_source_name(EndpointSource m) {
    switch (m) {
        case EndpointSource::IFK:        return "IFK";
        case EndpointSource::CritSample: return "CritSample";
        case EndpointSource::Analytical: return "Analytical";
        case EndpointSource::GCPC:       return "GCPC";
        default:                         return "Unknown";
    }
}

// ─── Source quality ordering ────────────────────────────────────────────
//  Higher quality = tighter bounds.  A cached source with quality ≥
//  the requested quality can serve the request without recomputation.
//    IFK(0) < CritSample(1) < {Analytical, GCPC}(2)
inline uint8_t endpoint_source_quality(EndpointSource s) {
    switch (s) {
        case EndpointSource::IFK:        return 0;
        case EndpointSource::CritSample: return 1;
        case EndpointSource::Analytical: return 2;
        case EndpointSource::GCPC:       return 2;  // same tier as Analytical
        default:                         return 0;
    }
}

/// Can a cached source serve a request for the given source?
/// True when cached quality ≥ requested quality.
inline bool source_can_serve(EndpointSource cached, EndpointSource requested) {
    return endpoint_source_quality(cached) >= endpoint_source_quality(requested);
}

/// Overload for raw quality values stored per-node.
inline bool source_quality_sufficient(uint8_t cached_quality,
                                      EndpointSource requested) {
    return cached_quality >= endpoint_source_quality(requested);
}

// Backward-compatible aliases
using FrameSourceMethod = EndpointSource;
inline const char* frame_source_name(EndpointSource m) {
    return endpoint_source_name(m);
}

// ─── Endpoint source configuration ──────────────────────────────────────────
struct EndpointSourceConfig {
    EndpointSource method = EndpointSource::IFK;

    // For CritSample method
    CriticalSamplingConfig crit_config = CriticalSamplingConfig::full_fast();

    // For Analytical method
    AnalyticalCriticalConfig analytical_config =
        AnalyticalCriticalConfig::all_enabled();

    // For GCPC method — pointer to pre-loaded cache (NOT owned)
    const GcpcCache* gcpc_cache = nullptr;

    // ── Factory methods ─────────────────────────────────────────────────
    static EndpointSourceConfig ifk() {
        return EndpointSourceConfig{EndpointSource::IFK, {}, {}};
    }
    static EndpointSourceConfig crit_sampling() {
        EndpointSourceConfig c;
        c.method = EndpointSource::CritSample;
        c.crit_config = CriticalSamplingConfig::full_fast();
        return c;
    }
    static EndpointSourceConfig analytical() {
        EndpointSourceConfig c;
        c.method = EndpointSource::Analytical;
        c.analytical_config = AnalyticalCriticalConfig::all_enabled();
        c.analytical_config.dual_phase3 = true;
        return c;
    }
    static EndpointSourceConfig gcpc(const GcpcCache* cache) {
        EndpointSourceConfig c;
        c.method = EndpointSource::GCPC;
        c.gcpc_cache = cache;
        return c;
    }

    // Backward-compatible factory alias
    static EndpointSourceConfig analytical_critical() { return analytical(); }
};

// Backward-compatible alias
using FrameSourceConfig = EndpointSourceConfig;

// ─── Endpoint AABB result (unified output of all endpoint sources) ──────────
//
// All four endpoint sources produce per-endpoint position interval AABBs in a
// single, unified format.  Each entry bounds a kinematic endpoint's Cartesian
// position over the C-space box — geometry only, no link radius.
//
// Layout: endpoint_aabbs[n_endpoints × 6]
//   where 6 = {lo_x, lo_y, lo_z, hi_x, hi_y, hi_z}
//   n_endpoints = n_joints + has_tool
//
// Per-link AABBs and sub-segment AABBs are derived in Stage 2 by
// interpolating between adjacent endpoints and adding link radius:
//   per-link AABB = union(endpoint[parent], endpoint[child]) + radius
//   sub-AABB at t = lerp(endpoint[parent], endpoint[child], t) + radius
//
// For n_sub=1 (per-link AABB), this is mathematically exact:
//   min(inf_q A(q), inf_q B(q)) ≡ inf_q min(A(q), B(q))
//   — zero precision loss regardless of endpoint source.
//
struct EndpointAABBResult {
    // Per-endpoint position interval AABBs: [n_endpoints × 6]
    // Geometry only — no link radius applied.
    // IFK/CritSample: endpoint position intervals from FK evaluation
    // Analytical/GCPC: analytical-tight endpoint position intervals
    std::vector<float> endpoint_aabbs;

    // Metadata
    int n_endpoints = 0;   // number of endpoints (= n_joints + has_tool)
    int n_active = 0;      // number of active links

    // FK state from IFK (needed for incremental FK support).
    // Valid for all sources (IFK is always computed alongside).
    FKState fk_state;

    // Number of critical configurations evaluated
    int n_evaluations = 0;

    // Whether fk_state is valid
    bool has_fk_state() const { return fk_state.valid; }

    // Total number of endpoint position AABBs
    int total_boxes() const { return n_endpoints; }
};

// Backward-compatible alias
using FrameSourceResult = EndpointAABBResult;

// ─── Compute endpoint AABBs (full computation) ─────────────────────────────
//
// Stage 1 entry point: generates per-endpoint position interval AABBs from
// C-space intervals using the configured endpoint source.
//
// @param config      Endpoint source configuration (method + params)
// @param robot       Robot model
// @param intervals   C-space intervals [n_joints]
//
// @return EndpointAABBResult with endpoint_aabbs [n_endpoints × 6]
//
EndpointAABBResult compute_endpoint_aabb(
    const EndpointSourceConfig& config,
    const Robot& robot,
    const std::vector<Interval>& intervals);

// Backward-compatible alias (n_sub parameter ignored — subdivision in Stage 2)
inline EndpointAABBResult compute_frame_source(
    const EndpointSourceConfig& config,
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int /*n_sub*/ = 1) {
    return compute_endpoint_aabb(config, robot, intervals);
}

// ─── Incremental endpoint AABBs (IFK fast-path) ────────────────────────────
//
// Recomputes endpoint AABBs incrementally after a single dimension change.
// Only IFK supports efficient incremental FK; other sources perform full
// recomputation but reuse the IFK incremental for the FK state.
//
// @param config      Endpoint source configuration
// @param parent_fk   FKState from parent node (for IFK incremental)
// @param robot       Robot model
// @param intervals   Updated C-space intervals
// @param changed_dim Dimension that changed
//
EndpointAABBResult compute_endpoint_aabb_incremental(
    const EndpointSourceConfig& config,
    const FKState& parent_fk,
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int changed_dim);

// Backward-compatible alias (n_sub parameter ignored)
inline EndpointAABBResult compute_frame_source_incremental(
    const EndpointSourceConfig& config,
    const FKState& parent_fk,
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int changed_dim,
    int /*n_sub*/ = 1) {
    return compute_endpoint_aabb_incremental(
        config, parent_fk, robot, intervals, changed_dim);
}

// ─── Extract endpoint position intervals from FKState ──────────────────────
//
// Extracts per-endpoint position intervals [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]
// from FKState prefix transforms.
// out: [n_endpoints × 6] where n_endpoints = n_joints + has_tool
//
void fk_to_endpoints(const FKState& fk, const Robot& robot,
                     std::vector<float>& out);

// ─── Extract per-link AABBs from endpoint AABB result ───────────────────────
//
// Derives per-link AABBs from per-endpoint position intervals by computing
// union(endpoint[parent], endpoint[child]) + link_radius for each active link.
//
// out_aabb: [n_active × 6] floats
//
void extract_link_aabbs_from_endpoint(
    const EndpointAABBResult& result,
    const Robot& robot,
    float* out_aabb);

// Backward-compatible alias
inline void extract_link_aabbs_from_result(
    const EndpointAABBResult& result,
    const Robot& robot,
    float* out_aabb) {
    extract_link_aabbs_from_endpoint(result, robot, out_aabb);
}

} // namespace envelope
} // namespace sbf

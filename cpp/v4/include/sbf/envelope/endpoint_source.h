// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Endpoint Source Module (Endpoint iAABB Generation)
//  Module: sbf::envelope
//
//  Stage 1 of the two-stage modular pipeline:
//    Stage 1: Endpoint iAABB generation    (this module)
//    Stage 2: Link envelope construction   (envelope_type.h)
//
//  Four endpoint sources produce a unified intermediate representation
//  called "endpoint_iaabb": per-active-link paired position interval AABBs
//  (geometry-only, no link radius), stored as [n_active × 2 × 6].
//  Only active links (excluding link 0 and zero-length links) are stored.
//
//    iFK (Interval FK):
//      Fastest method.  Computes interval forward kinematics, then
//      extracts per-endpoint position intervals directly.
//
//    CritSample (Critical Sampling):
//      Enumerates critical joint configurations and evaluates FK.
//      Tighter than iFK at the cost of many FK evaluations.
//
//    Analytical (Analytical Critical):
//      Exact analytical gradient-zero enumeration.
//
//    GCPC (Global Critical Point Cache):
//      Cached critical points with on-the-fly AA-pruned enrichment.
//
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/robot/robot.h"
#include "sbf/robot/interval_fk.h"
#include "sbf/core/types.h"

#include <vector>
#include <string>
#include <cstdint>
#include <array>

namespace sbf {
namespace envelope {

// Forward declarations (avoid pulling in heavy headers)
struct CriticalSamplingConfig;
struct AnalyticalCriticalConfig;
class  GcpcCache;

// ─── Endpoint source method enum ────────────────────────────────────────────
enum class EndpointSource : uint8_t {
    IFK        = 0,  // Interval FK (fast, conservative)
    CritSample = 1,  // Critical-point sampling (medium, tighter)
    Analytical = 2,  // Analytical solver (boundary + interior)
    GCPC       = 3,  // Analytical boundary + cache interior (recommended)
    Count      = 4
};

inline const char* endpoint_source_name(EndpointSource m) {
    switch (m) {
        case EndpointSource::IFK:        return "IFK";
        case EndpointSource::CritSample: return "CritSample";
        case EndpointSource::Analytical: return "Analytical";
        case EndpointSource::GCPC:       return "GCPC";
        case EndpointSource::Count:      return "Count";
        default:                         return "Unknown";
    }
}

// ─── Source safety class ────────────────────────────────────────────────
//
//  Two disjoint safety classes:
//
//    SAFE (conservative bounds guaranteed):
//      iFK  ≤  Analytical  ≤  GCPC        quality: 0 < 2 < 3
//
//    UNSAFE (tighter but NOT conservative — no containment guarantee):
//      CritSample                          quality: 1
//
//  Substitution rules:
//    - Within same class: higher quality substitutes lower
//      (e.g. GCPC can serve IFK requests)
//    - SAFE → UNSAFE: allowed  (conservative outer bound is a valid,
//      though looser, substitute for a non-conservative request)
//    - UNSAFE → SAFE: FORBIDDEN  (CritSample's non-conservative
//      result must never be used as a collision safety certificate)
//
//  Rationale:  CritSample enumerates critical points by sampling, which
//  may miss the true extremum.  The resulting iAABBs can underestimate
//  the reachable workspace, making them unsafe for collision certificates.
//  The other three sources (iFK, Analytical, GCPC) all produce conservative
//  outer bounds, so they form a compatible substitution chain AND can
//  validly serve CritSample requests (the bound is wider but correct).
//
enum class SourceSafetyClass : uint8_t { SAFE = 0, UNSAFE = 1 };

inline SourceSafetyClass source_safety_class(EndpointSource s) {
    switch (s) {
        case EndpointSource::IFK:
        case EndpointSource::Analytical:
        case EndpointSource::GCPC:
            return SourceSafetyClass::SAFE;
        case EndpointSource::CritSample:
            return SourceSafetyClass::UNSAFE;
        case EndpointSource::Count:
        default:
            return SourceSafetyClass::SAFE;
    }
}

// ─── Source quality ordering (within same safety class) ─────────────────
inline uint8_t endpoint_source_quality(EndpointSource s) {
    switch (s) {
        case EndpointSource::IFK:        return 0;
        case EndpointSource::CritSample: return 1;
        case EndpointSource::Analytical: return 2;
        case EndpointSource::GCPC:       return 3;
        case EndpointSource::Count:      return 0;
        default:                         return 0;
    }
}

inline constexpr int kEndpointSourceCount = static_cast<int>(EndpointSource::Count);

// Explicit substitution matrix to prevent semantic drift when quality ordering
// or enum extensions change in the future.
//
// Row = cached source, Column = requested source.
// Order: IFK, CritSample, Analytical, GCPC.
inline constexpr bool kSourceSubstitutionMatrix[kEndpointSourceCount][kEndpointSourceCount] = {
    // requested:          IFK    Crit   Analyt GCPC
    /* cached IFK */      {true,  true,  false, false},
    /* cached Crit */     {false, true,  false, false},
    /* cached Analytical*/{true,  true,  true,  false},
    /* cached GCPC */     {true,  true,  true,  true },
};

static_assert(static_cast<int>(EndpointSource::IFK) == 0,
              "EndpointSource enum order changed: update substitution matrix");
static_assert(static_cast<int>(EndpointSource::CritSample) == 1,
              "EndpointSource enum order changed: update substitution matrix");
static_assert(static_cast<int>(EndpointSource::Analytical) == 2,
              "EndpointSource enum order changed: update substitution matrix");
static_assert(static_cast<int>(EndpointSource::GCPC) == 3,
              "EndpointSource enum order changed: update substitution matrix");
static_assert(static_cast<int>(EndpointSource::Count) == 4,
              "EndpointSource::Count changed: update substitution matrix and dispatch entry points");

/// Can a cached result (from `cached`) serve a request for `requested`?
///
/// Rules:
///   - Same safety class + quality(cached) >= quality(requested) → YES
///   - SAFE cached → UNSAFE requested                            → YES
///     (conservative outer bound is a valid substitute)
///   - UNSAFE cached → SAFE requested                            → NO
///     (non-conservative result must not serve as safety certificate)
inline bool source_can_serve(EndpointSource cached, EndpointSource requested) {
    const int ci = static_cast<int>(cached);
    const int ri = static_cast<int>(requested);
    if (ci < 0 || ci >= kEndpointSourceCount ||
        ri < 0 || ri >= kEndpointSourceCount)
        return false;
    return kSourceSubstitutionMatrix[ci][ri];
}

/// Interpret a stored quality byte as EndpointSource and check compatibility.
/// The LECT stores static_cast<uint8_t>(EndpointSource) in source_quality_.
inline bool source_quality_sufficient(uint8_t stored_source_byte,
                                      EndpointSource requested) {
    return source_can_serve(
        static_cast<EndpointSource>(stored_source_byte), requested);
}

// ─── Endpoint source configuration ──────────────────────────────────────────
struct EndpointSourceConfig {
    EndpointSource method = EndpointSource::IFK;

    // For CritSample method
    // (full definition in crit_sample.h; forward-declared above)
    const CriticalSamplingConfig* crit_config_ptr = nullptr;

    // For Analytical method
    const AnalyticalCriticalConfig* analytical_config_ptr = nullptr;

    // For GCPC method — pointer to pre-loaded cache (NOT owned)
    const GcpcCache* gcpc_cache = nullptr;

    // ── Factory methods ─────────────────────────────────────────────────
    static EndpointSourceConfig ifk() {
        return EndpointSourceConfig{EndpointSource::IFK};
    }
    static EndpointSourceConfig crit_sampling();    // defined in .cpp
    static EndpointSourceConfig analytical();       // defined in .cpp
    static EndpointSourceConfig gcpc(const GcpcCache* cache) {
        EndpointSourceConfig c;
        c.method = EndpointSource::GCPC;
        c.gcpc_cache = cache;
        return c;
    }
};

// ─── Endpoint iAABB result (unified output of all endpoint sources) ─────────
//
// All four endpoint sources produce per-active-link paired endpoint iAABBs
// in a unified format.
//
// Layout: endpoint_iaabbs[n_active × 2 × 6]  (= n_active_ep × 6)
//   where 6 = {lo_x, lo_y, lo_z, hi_x, hi_y, hi_z}
//   For active link ci (V = active_link_map[ci]):
//     endpoint_iaabbs[(ci*2)*6]   = proximal = FK prefix[V]
//     endpoint_iaabbs[(ci*2+1)*6] = distal   = FK prefix[V+1]
//
// This paired layout stores only frames needed by active links,
// eliminating storage for link 0, zero-length links, and tool frame.
//
struct EndpointIAABBResult {
    // Fixed-capacity endpoint buffer to avoid per-call heap allocation.
    // Valid payload length is endpoint_iaabb_len() floats.
    static constexpr int kMaxEndpointFloats = MAX_TF * 6;

    // Per-active-link paired endpoint iAABBs: [n_active × 2 × 6]
    // Geometry only — no link radius applied.
    std::array<float, kMaxEndpointFloats> endpoint_iaabbs{};

    // Metadata
    int n_active_ep  = 0;   // number of paired endpoints (= n_active * 2)
    int n_active     = 0;   // number of active links

    // FK state from iFK (needed for incremental FK support).
    FKState fk_state;

    // Number of critical configurations evaluated
    int n_evaluations = 0;

    bool has_fk_state() const { return fk_state.valid; }
    bool has_endpoint_iaabbs() const { return n_active_ep > 0; }
    int endpoint_iaabb_len() const { return n_active_ep * 6; }
    float* endpoint_data() { return endpoint_iaabbs.data(); }
    const float* endpoint_data() const { return endpoint_iaabbs.data(); }
    int total_boxes()   const { return n_active; }
};

// ─── Compute endpoint iAABBs (full computation) ────────────────────────────
//
// Stage 1 entry point: generates per-endpoint position interval iAABBs
// from C-space intervals using the configured endpoint source.
//
EndpointIAABBResult compute_endpoint_iaabb(
    const EndpointSourceConfig& config,
    const Robot& robot,
    const std::vector<Interval>& intervals);

// ─── Incremental endpoint iAABBs (iFK fast-path) ───────────────────────────
EndpointIAABBResult compute_endpoint_iaabb_incremental(
    const EndpointSourceConfig& config,
    const FKState& parent_fk,
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int changed_dim);

// ─── Extract per-link iAABBs from endpoint iAABB result ─────────────────────
//
// Derives per-link iAABBs from per-endpoint position intervals by computing
// union(endpoint[parent], endpoint[child]) + link_radius for each active link.
//
// out_iaabb: [n_active × 6] floats
//
void extract_link_iaabbs(
    const EndpointIAABBResult& result,
    const Robot& robot,
    float* out_iaabb);

// ─── Extract endpoint position intervals from FKState ──────────────────────
void fk_to_endpoints(const FKState& fk, const Robot& robot,
                     float* out, int out_len);

void fk_to_endpoints(const FKState& fk, const Robot& robot,
                     std::vector<float>& out);

} // namespace envelope
} // namespace sbf

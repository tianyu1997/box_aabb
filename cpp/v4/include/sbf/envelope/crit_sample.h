// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Critical Sampling (CritSample endpoint source)
//  Module: sbf::envelope
//
//  GCPC-based critical sampling pipeline (v4 redesign):
//    Phase 1: GCPC cache lookup — query all interior critical points within
//             the interval, reconstruct full configs, FK → endpoint iAABBs.
//    Phase 2: Boundary + kπ/2 enumeration — Cartesian product of
//             {lo, hi, kπ/2 ∩ (lo,hi)} per joint, FK → endpoint iAABBs.
//
//  Unlike v3's 3-stage pipeline (coupled + manifold + L-BFGS-B), v4 uses
//  the GCPC cache for interior critical points and simple boundary enumeration
//  for kπ/2 combinatorics.  This is both faster and tighter.
//
//  Output: per-endpoint iAABBs in [n_endpoints × 6] format, compatible
//          with the unified EndpointIAABBResult.
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/robot/robot.h"
#include "sbf/core/types.h"
#include <vector>
#include <cstdint>

namespace sbf {
namespace envelope {

// Forward declaration
class GcpcCache;

// ─── CriticalSamplingConfig ─────────────────────────────────────────────────
struct CriticalSamplingConfig {
    // Phase 1: GCPC cache lookup
    bool enable_gcpc = true;    // query GCPC cache for interior critical points

    // Phase 2: boundary + kπ/2 enumeration
    bool enable_boundary = true;    // enumerate boundary + kπ/2 combos
    long long max_boundary_combos = 60'000;  // cap on Cartesian product size

    // GCPC cache (NOT owned — caller must keep alive)
    const GcpcCache* gcpc_cache = nullptr;

    // ── Factory methods ─────────────────────────────────────────────────
    static CriticalSamplingConfig defaults() {
        return CriticalSamplingConfig{};
    }

    static CriticalSamplingConfig gcpc_only() {
        CriticalSamplingConfig c;
        c.enable_boundary = false;
        return c;
    }

    static CriticalSamplingConfig boundary_only() {
        CriticalSamplingConfig c;
        c.enable_gcpc = false;
        return c;
    }
};

// ─── CritSampleStats ────────────────────────────────────────────────────────
struct CritSampleStats {
    int n_gcpc_matches = 0;       // Phase 1: points from GCPC cache
    int n_gcpc_fk      = 0;       // Phase 1: FK evaluations
    int n_boundary_combos = 0;    // Phase 2: boundary + kπ/2 configs evaluated
    int n_boundary_fk  = 0;       // Phase 2: FK evaluations (= n_boundary_combos)
    double phase1_ms   = 0;
    double phase2_ms   = 0;
};

// ─── Main entry point ───────────────────────────────────────────────────────
//
// Compute per-endpoint iAABBs via GCPC + boundary sampling.
//
// out_endpoint_iaabb: [n_endpoints × 6] floats — geometry-only iAABBs.
//   Initialized to inverted bounds; each FK evaluation expands them.
//
// Returns the number of FK evaluations performed.
//
int derive_crit_endpoints(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    const CriticalSamplingConfig& config,
    float* out_endpoint_iaabb,
    CritSampleStats* out_stats = nullptr);

} // namespace envelope
} // namespace sbf

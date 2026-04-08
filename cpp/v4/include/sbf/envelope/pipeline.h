// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Pipeline configuration (source + envelope combined)
//  Module: sbf::envelope
//
//  Combined configuration for the full two-stage pipeline:
//    Stage 1: Endpoint Source → endpoint_iaabb
//    Stage 2: endpoint_iaabb → Link Envelope
//
//  Separated from envelope_type.h to reduce header dependency:
//  envelope_type.h no longer pulls in endpoint_source.h.
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/envelope/endpoint_source.h"
#include "sbf/envelope/envelope_type.h"

namespace sbf {
namespace envelope {

// ─── Pipeline configuration (source + envelope) ─────────────────────────────
struct PipelineConfig {
    EndpointSourceConfig  source;
    EnvelopeTypeConfig    envelope;

    // AA/IA hybrid crossover: when max interval width ≤ this threshold,
    // use Affine Arithmetic for iAABB extraction (tighter + faster).
    // 0.0 = disabled (always use IA).  Default: 0.5.
    double aa_crossover_width = 0.5;

    // ── Factory: recommended pipeline (GCPC, best speed+quality) ─────────
    // GCPC + Hull16_Grid — 1.6× faster than Analytical, tighter bounds.
    // Requires a pre-built GcpcCache. For cache-less fallback see fast().
    static PipelineConfig recommended(const GcpcCache* cache) {
        PipelineConfig p;
        p.source   = EndpointSourceConfig::gcpc(cache);
        p.envelope = EnvelopeTypeConfig::hull16_grid();
        return p;
    }

    // Tightest pipeline: GCPC + Hull16_Grid
    // GCPC is tighter than standalone Analytical (cache interior + exact boundary).
    static PipelineConfig tightest(const GcpcCache* cache) {
        PipelineConfig p;
        p.source   = EndpointSourceConfig::gcpc(cache);
        p.envelope = EnvelopeTypeConfig::hull16_grid();
        return p;
    }

    // Production default: GCPC + Hull16_Grid (fastest high-quality)
    static PipelineConfig production(const GcpcCache* cache) {
        PipelineConfig p;
        p.source   = EndpointSourceConfig::gcpc(cache);
        p.envelope = EnvelopeTypeConfig::hull16_grid();
        return p;
    }

    // Fast pipeline (no cache needed): iFK + LinkIAABB(sub=1)
    static PipelineConfig fast() {
        PipelineConfig p;
        p.source   = EndpointSourceConfig::ifk();
        p.envelope = EnvelopeTypeConfig::link_iaabb();
        return p;
    }

    // ── Legacy cache-less overloads ──────────────────────────────────────
    // Use GCPC variants above when a cache is available (recommended).
    static PipelineConfig recommended() {
        PipelineConfig p;
        p.source   = EndpointSourceConfig::crit_sampling();
        p.envelope = EnvelopeTypeConfig::hull16_grid();
        return p;
    }
    static PipelineConfig tightest() {
        PipelineConfig p;
        p.source   = EndpointSourceConfig::analytical();
        p.envelope = EnvelopeTypeConfig::hull16_grid();
        return p;
    }
    static PipelineConfig production() {
        PipelineConfig p;
        p.source   = EndpointSourceConfig::ifk();
        p.envelope = EnvelopeTypeConfig::hull16_grid();
        return p;
    }
};

} // namespace envelope
} // namespace sbf

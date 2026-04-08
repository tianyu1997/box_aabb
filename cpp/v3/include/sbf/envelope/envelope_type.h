// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v3 — Link Envelope Module (Envelope Representation)
//  Module: sbf::envelope
//
//  Stage 2 of the two-stage modular pipeline:
//    Stage 1: Endpoint AABB generation     (frame_source.h)
//    Stage 2: Link envelope construction   (this module)
//
//  Three envelope representations are derived from the unified endpoint_aabb
//  intermediate produced by Stage 1:
//
//    SubAABB:
//      Per-link subdivided AABBs obtained by interpolation from endpoint
//      positions.  Each link is split into n_sub sub-segments.  When n_sub=1,
//      this reduces to a single AABB per link (the old "AABB" type).
//      Storage: n_active × n_sub × 6 floats.
//
//    SubAABB_Grid:
//      Interpolated sub-AABBs rasterised into an R³ byte grid.  Compact
//      representation without wrapping inflation.
//
//    Hull16_Grid:
//      16-point convex hull (fill_hull16) rasterised into a sparse BitBrick
//      VoxelGrid.  Uses per-link proximal/distal endpoint-AABB pairs directly
//      via Conv(B_prox ∪ B_dist) ⊕ Ball(r).  Tightest voxel representation.
//
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/envelope/frame_source.h"
#include "sbf/voxel/voxel_grid.h"

#include <cstdint>
#include <vector>
#include <string>

namespace sbf {
namespace envelope {

// ─── Envelope type enum ─────────────────────────────────────────────────────
// NOTE: AABB was merged into SubAABB (n_sub=1 ≡ single AABB per link).
//       Enum values renumbered — old cache files will be auto-invalidated.
enum class EnvelopeType : uint8_t {
    SubAABB       = 0,  // Per-link subdivided AABBs (n_sub=1 → single AABB)
    SubAABB_Grid  = 1,  // SubAABBs → R³ byte grid
    Hull16_Grid   = 2   // 16-pt convex hull (fill_hull16) → sparse BitBrick VoxelGrid
};

inline const char* envelope_type_name(EnvelopeType t) {
    switch (t) {
        case EnvelopeType::SubAABB:      return "SubAABB";
        case EnvelopeType::SubAABB_Grid: return "SubAABB_Grid";
        case EnvelopeType::Hull16_Grid:  return "Hull16_Grid";
        default:                         return "Unknown";
    }
}

// ─── Envelope type configuration ────────────────────────────────────────────
struct EnvelopeTypeConfig {
    EnvelopeType type = EnvelopeType::Hull16_Grid;

    // Per-link subdivision count for Stage 2 interpolation.
    // SubAABB: n_sub=1 → single AABB per link (cheapest);
    //          n_sub>1 → each link is split into n_sub sub-AABBs.
    // SubAABB_Grid: n_sub controls rasterisation precision.
    // Hull16_Grid: n_sub is ignored (uses fill_hull16 directly from endpoint pairs).
    int n_sub = 1;

    // Voxel resolution for Hull16_Grid (metres per voxel).
    double delta = 0.01;

    // Grid resolution for SubAABB_Grid (voxels per axis).
    int grid_R = 64;

    // World bounds for SubAABB_Grid: [x_lo, y_lo, z_lo, x_hi, y_hi, z_hi].
    float world_bounds[6] = {-0.8f, -1.2f, 0.0f, 1.8f, 1.2f, 1.4f};

    // ── Factory methods with optimal defaults from benchmark sweep ───────

    // SubAABB (n_sub=1): single AABB per link (cheapest, most conservative)
    static EnvelopeTypeConfig sub_aabb() {
        EnvelopeTypeConfig c;
        c.type = EnvelopeType::SubAABB;
        c.n_sub = 1;
        return c;
    }

    // Backward-compatible alias: AABB ≡ SubAABB(n_sub=1)
    static EnvelopeTypeConfig aabb() { return sub_aabb(); }

    // SubAABB_Grid (all sources): n_sub=16, grid_R=64
    static EnvelopeTypeConfig sub_aabb_grid() {
        EnvelopeTypeConfig c;
        c.type = EnvelopeType::SubAABB_Grid;
        c.n_sub = 16;
        c.grid_R = 64;
        return c;
    }

    // Hull16_Grid (IFK / CritSample): delta=0.01 (n_sub ignored)
    static EnvelopeTypeConfig hull16_grid() {
        EnvelopeTypeConfig c;
        c.type = EnvelopeType::Hull16_Grid;
        c.n_sub = 1;
        c.delta = 0.01;
        return c;
    }

    // Hull16_Grid (Analytical/GCPC): delta=0.01 (n_sub ignored)
    static EnvelopeTypeConfig hull16_grid_analytical() {
        EnvelopeTypeConfig c;
        c.type = EnvelopeType::Hull16_Grid;
        c.n_sub = 16;
        c.delta = 0.01;
        return c;
    }
};

// ─── Optimal default config per (source, envelope) combination ──────────────
//
// Returns the optimal EnvelopeTypeConfig for a given (source, type) pair,
// based on full pipeline benchmark sweep results.
//
//  +────────────────────+──────────+──────────────────+──────────────────+
//  |     Source \\ Env  | SubAABB  |   SubAABB_Grid   |   Hull16_Grid    |
//  +────────────────────+──────────+──────────────────+──────────────────+
//  | IFK                | n=1      | n=16, R=64       | n=1,  δ=0.01    |
//  | CritSample         | n=1      | n=16, R=64       | n=1,  δ=0.01    |
//  | Analytical         | n=1      | n=16, R=64       | n=16, δ=0.01    |
//  | GCPC               | n=1      | n=16, R=64       | n=16, δ=0.01    |
//  +────────────────────+──────────+──────────────────+──────────────────+
//
EnvelopeTypeConfig default_envelope_config(
    EndpointSource source, EnvelopeType type);

// ─── Envelope result ────────────────────────────────────────────────────────
struct EnvelopeResult {
    // Per-link sub-AABBs: [n_active × n_sub × 6]  (AABB / SubAABB / SubAABB_Grid)
    std::vector<float> sub_aabbs;

    // Byte grid: [R³] (SubAABB_Grid only)
    std::vector<uint8_t> grid;
    int grid_R = 0;

    // Hull-16 VoxelGrid (Hull16_Grid only)
    voxel::VoxelGrid hull_grid;

    // Scalar metrics
    double volume = 0;      // m³
    int    n_voxels = 0;    // occupied voxels (grid types) or n_sub_aabbs
    int    n_bricks = 0;    // BitBrick count (Hull16_Grid only)

    bool valid = false;
};

// ─── Compute link envelope from endpoint AABBs ─────────────────────────────
//
// Stage 2 entry point: derives the link envelope representation from the
// unified endpoint_aabb intermediate produced by Stage 1.
//
// @param env_config  Envelope type configuration
// @param robot       Robot model
// @param ep_result   EndpointAABBResult from compute_endpoint_aabb()
// @return EnvelopeResult with the computed envelope data
//
EnvelopeResult compute_link_envelope(
    const EnvelopeTypeConfig& env_config,
    const Robot& robot,
    const EndpointAABBResult& ep_result);

// Backward-compatible alias
inline EnvelopeResult compute_envelope_repr(
    const EnvelopeTypeConfig& env_config,
    const Robot& robot,
    const EndpointAABBResult& ep_result) {
    return compute_link_envelope(env_config, robot, ep_result);
}

// ─── Pipeline configuration (source + envelope) ─────────────────────────────
//
// Combined configuration for the full two-stage pipeline:
//   Stage 1: Endpoint Source → endpoint_aabb
//   Stage 2: endpoint_aabb → Link Envelope
//
struct PipelineConfig {
    EndpointSourceConfig  source;
    EnvelopeTypeConfig    envelope;

    // AA/IA hybrid crossover: when max interval width ≤ this threshold,
    // use Affine Arithmetic for AABB extraction (tighter + faster).
    // 0.0 = disabled (always use IA).  Default: 0.5 (validated by scan).
    double aa_crossover_width = 0.5;

    // ── Factory: recommended pipeline (best cost/quality tradeoff) ───────
    // CritSample + Hull16_Grid
    static PipelineConfig recommended() {
        PipelineConfig p;
        p.source = EndpointSourceConfig::crit_sampling();
        p.envelope = EnvelopeTypeConfig::hull16_grid();
        return p;
    }

    // Fast pipeline: IFK + SubAABB(n_sub=1) (fastest, most conservative)
    static PipelineConfig fast() {
        PipelineConfig p;
        p.source = EndpointSourceConfig::ifk();
        p.envelope = EnvelopeTypeConfig::sub_aabb();
        return p;
    }

    // Tightest pipeline: Analytical + Hull16_Grid (slowest, tightest)
    static PipelineConfig tightest() {
        PipelineConfig p;
        p.source = EndpointSourceConfig::analytical();
        p.envelope = EnvelopeTypeConfig::hull16_grid_analytical();
        return p;
    }

    // Production default: IFK + Hull16_Grid (fast + tight)
    static PipelineConfig production() {
        PipelineConfig p;
        p.source = EndpointSourceConfig::ifk();
        p.envelope = EnvelopeTypeConfig::hull16_grid();
        return p;
    }
};

} // namespace envelope
} // namespace sbf

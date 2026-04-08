// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Link Envelope Module (Envelope Representation)
//  Module: sbf::envelope
//
//  Stage 2 of the two-stage modular pipeline:
//    Stage 1: Endpoint iAABB generation    (endpoint_source.h)
//    Stage 2: Link envelope construction   (this module)
//
//  Three envelope representations derived from endpoint_iaabb:
//
//    LinkIAABB:
//      Per-link subdivided iAABBs via interpolation from endpoint positions.
//      link iAABB(sub=1) = single iAABB per link.
//      link iAABB(sub=n) = n sub-segment iAABBs per link.
//
//    LinkIAABB_Grid:
//      Interpolated link iAABBs rasterised into an R³ byte grid.
//
//    Hull16_Grid:
//      16-point convex hull rasterised into a sparse BitBrick VoxelGrid.
//
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/core/types.h"
#include "sbf/voxel/voxel_grid.h"

#include <cstdint>
#include <string>
#include <vector>

// Forward declarations
namespace sbf { class Robot; }
namespace sbf::envelope { struct EndpointIAABBResult; }

namespace sbf {
namespace envelope {

// ─── Envelope type enum ─────────────────────────────────────────────────────
enum class EnvelopeType : uint8_t {
    LinkIAABB      = 0,  // link iAABB(sub=n), n≥1
    LinkIAABB_Grid = 1,  // link iAABB rasterised to VoxelGrid
    Hull16_Grid    = 2   // 16-pt convex hull → sparse BitBrick VoxelGrid
};

inline const char* envelope_type_name(EnvelopeType t) {
    switch (t) {
        case EnvelopeType::LinkIAABB:      return "LinkIAABB";
        case EnvelopeType::LinkIAABB_Grid: return "LinkIAABB_Grid";
        case EnvelopeType::Hull16_Grid:    return "Hull16_Grid";
        default:                           return "Unknown";
    }
}

// ─── Envelope type configuration ────────────────────────────────────────────
struct EnvelopeTypeConfig {
    EnvelopeType type = EnvelopeType::Hull16_Grid;

    // Per-link subdivision count for Stage 2 interpolation.
    int n_sub = 1;

    // Voxel resolution for Hull16_Grid (metres per voxel).
    double delta = 0.01;

    // Grid resolution for LinkIAABB_Grid (voxels per axis).
    int grid_R = 64;

    // World bounds for LinkIAABB_Grid.
    float world_bounds[6] = {-0.8f, -1.2f, 0.0f, 1.8f, 1.2f, 1.4f};

    // ── Factory methods ─────────────────────────────────────────────────

    /// LinkIAABB(sub=1): single iAABB per link (cheapest, most conservative)
    static EnvelopeTypeConfig link_iaabb() {
        EnvelopeTypeConfig c;
        c.type = EnvelopeType::LinkIAABB;
        c.n_sub = 1;
        return c;
    }

    /// LinkIAABB_Grid: n_sub=16, grid_R=64
    static EnvelopeTypeConfig link_iaabb_grid() {
        EnvelopeTypeConfig c;
        c.type = EnvelopeType::LinkIAABB_Grid;
        c.n_sub = 16;
        c.grid_R = 64;
        return c;
    }

    /// Hull16_Grid: delta=0.01, n_sub=1 (subdivision has no effect on convex hull)
    static EnvelopeTypeConfig hull16_grid() {
        EnvelopeTypeConfig c;
        c.type = EnvelopeType::Hull16_Grid;
        c.n_sub = 1;
        c.delta = 0.01;
        return c;
    }
};

/// Returns the optimal EnvelopeTypeConfig for a given (source, type) pair.
/// @param source_enum  EndpointSource enum cast to uint8_t
EnvelopeTypeConfig default_envelope_config(
    uint8_t source_enum, EnvelopeType type);

// ─── Envelope result ────────────────────────────────────────────────────────
struct EnvelopeResult {
    // Per-link iAABBs: [n_active × n_sub × 6]
    std::vector<float> link_iaabbs;

    // Byte grid: [R³] (LinkIAABB_Grid only)
    std::vector<uint8_t> grid;
    int grid_R = 0;

    // Hull-16 VoxelGrid (Hull16_Grid only)
    voxel::VoxelGrid hull_grid;

    // Scalar metrics
    double volume   = 0;     // m³
    int    n_voxels = 0;     // occupied voxels (grid types) or n_link_iaabbs
    int    n_bricks = 0;     // BitBrick count (Hull16_Grid only)

    bool valid = false;
};

// ─── Compute link envelope from endpoint iAABBs ────────────────────────────
//
// Stage 2 entry point: derives the link envelope representation from the
// unified endpoint_iaabb intermediate produced by Stage 1.
//
EnvelopeResult compute_link_envelope(
    const EnvelopeTypeConfig& env_config,
    const Robot& robot,
    const EndpointIAABBResult& ep_result);

} // namespace envelope
} // namespace sbf

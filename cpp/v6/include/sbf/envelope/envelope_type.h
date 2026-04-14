#pragma once
/// @file envelope_type.h
/// @brief Envelope type enumeration, configuration, and unified computation entry point.
///
/// The envelope pipeline converts per-node endpoint iAABBs into link envelopes
/// that bound the workspace swept volume of each robot link.  Three types:
///   - `LinkIAABB` — AABB hull per link (fast, coarse).
///   - `LinkIAABB_Grid` — AABB + voxel grid rasterization (tighter, slower).
///   - `Hull16_Grid` — reserved for future hull-16 turbo scanline.

#include <sbf/envelope/link_grid.h>
#include <sbf/voxel/voxel_grid.h>
#include <sbf/core/types.h>
#include <sbf/core/robot.h>
#include <sbf/core/fk_state.h>

#include <cstdint>
#include <memory>
#include <vector>

namespace sbf {

// ─── Envelope type ──────────────────────────────────────────────────────────
enum class EnvelopeType : uint8_t {
    LinkIAABB      = 0,  // AABB(sub=n)
    LinkIAABB_Grid = 1,  // AABB + voxel grid
    Hull16_Grid    = 2   // reserved — not implemented in v5 first release
};

inline const char* envelope_type_name(EnvelopeType t) {
    switch (t) {
        case EnvelopeType::LinkIAABB:      return "LinkIAABB";
        case EnvelopeType::LinkIAABB_Grid: return "LinkIAABB_Grid";
        case EnvelopeType::Hull16_Grid:    return "Hull16_Grid";
        default:                           return "Unknown";
    }
}

// ─── Configuration ──────────────────────────────────────────────────────────
struct EnvelopeTypeConfig {
    EnvelopeType type = EnvelopeType::LinkIAABB;
    int n_subdivisions = 1;     // sub=1 whole link, sub=n subdivided
    GridConfig grid_config;     // only used by Grid types
};

// ─── Result ─────────────────────────────────────────────────────────────────
struct LinkEnvelope {
    EnvelopeType type = EnvelopeType::LinkIAABB;
    int n_active_links = 0;
    int n_subdivisions = 1;                   // sub count used to produce link_iaabbs
    std::vector<float> link_iaabbs;           // [n_active * n_sub × 6], always present
    std::unique_ptr<voxel::SparseVoxelGrid> sparse_grid;  // non-null for Grid types

    bool has_grid() const { return sparse_grid != nullptr; }
};

// ─── Unified entry point ────────────────────────────────────────────────────
LinkEnvelope compute_link_envelope(
    const float* endpoint_iaabbs,
    int n_active_links,
    const double* link_radii,
    const EnvelopeTypeConfig& config);

}  // namespace sbf

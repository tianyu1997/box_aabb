#pragma once
/// @file envelope_type.h
/// @brief Unified link-envelope computation (AABB + optional voxel grid).
///
/// Three envelope types:
///   - LinkIAABB       — zero-radius link AABB hull only (fast, coarse).
///   - LinkIAABB_Grid  — AABB + voxel-grid swept volume (rasterised
///                       link AABBs inflated by link_radii + safety pad).
///   - Hull16_Grid     — AABB + Hull-16 turbo-scanline grid (tightest).
///
/// Per D1, the *AABBs stored on disk and in caches* are zero-radius. The
/// voxel grid path inflates per-link radii inline during fill; this is the
/// only place where v7 envelope code consumes link_radii.

#include "sbf/voxel/voxel_grid.h"

#include <cstdint>
#include <memory>
#include <vector>

namespace sbf::envelope {

enum class EnvelopeType : uint8_t {
    LinkIAABB      = 0,
    LinkIAABB_Grid = 1,
    Hull16_Grid    = 2,
};

inline const char* envelope_type_name(EnvelopeType t) {
    switch (t) {
        case EnvelopeType::LinkIAABB:      return "LinkIAABB";
        case EnvelopeType::LinkIAABB_Grid: return "LinkIAABB_Grid";
        case EnvelopeType::Hull16_Grid:    return "Hull16_Grid";
        default:                           return "Unknown";
    }
}

struct GridConfig {
    double voxel_delta = 0.05;
};

struct EnvelopeTypeConfig {
    EnvelopeType type = EnvelopeType::LinkIAABB;
    int          n_subdivisions = 1;
    GridConfig   grid_config;
};

struct LinkEnvelope {
    EnvelopeType type           = EnvelopeType::LinkIAABB;
    int          n_active_links = 0;
    int          n_subdivisions = 1;
    /// Zero-radius link AABBs, layout [n_active * n_sub * 6].
    std::vector<float> link_iaabbs;
    std::unique_ptr<sbf::voxel::SparseVoxelGrid> sparse_grid;

    bool has_grid() const { return sparse_grid != nullptr; }
};

/// Compute a link envelope from zero-radius endpoint iAABBs.
///
/// `link_radii` is consumed only for grid-rasterisation inflation.
/// The returned `link_iaabbs` are always zero-radius (D1 invariant).
/// Pass nullptr to disable grid-side radius inflation.
LinkEnvelope compute_link_envelope(
    const float* endpoint_iaabbs,
    int n_active_links,
    const double* link_radii,
    const EnvelopeTypeConfig& config);

}  // namespace sbf::envelope

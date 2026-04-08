// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Envelope Derive: endpoint_iaabbs → per-link iAABB
//  Module: sbf::envelope
//  迁移自 v3 envelope_derive.h
//
//  Pure functions (no state).  Given per-endpoint position intervals stored
//  in EndpointIAABBResult, derive per-link iAABBs / subdivided iAABBs / grids
//  without re-running interval FK.
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/core/types.h"
#include <cstdint>

namespace sbf {
namespace envelope {

// ═════════════════════════════════════════════════════════════════════════════
//  AABB derive — per-link AABB from endpoint position intervals
// ═════════════════════════════════════════════════════════════════════════════
void derive_aabb(
    const float* frames,            // [n_frames × 6]
    int n_frames,
    const int* active_link_map,     // [n_active] compact_idx → frame_idx
    int n_active,
    const float* link_radii,        // [n_active] per-link capsule radius (may be nullptr)
    const float* base_pos,          // [3] base frame position
    float* out_aabb);               // [n_active × 6]

// ═════════════════════════════════════════════════════════════════════════════
//  AABB derive — paired active layout (no active_link_map indirection)
//
//  paired_frames layout: [n_active × 2 × 6]
//    paired_frames[(ci*2)*6]     = proximal iAABB of active link ci
//    paired_frames[(ci*2+1)*6]   = distal   iAABB of active link ci
//
//  This is the preferred path for the LECT pipeline (ChannelData::ep_data uses this).
//  No base_pos needed — proximal is always explicitly stored.
// ═════════════════════════════════════════════════════════════════════════════
void derive_aabb_paired(
    const float* paired_frames,     // [n_active × 2 × 6]
    int n_active,
    const float* link_radii,        // [n_active] per-link capsule radius (may be nullptr)
    float* out_aabb);               // [n_active × 6]

// ═════════════════════════════════════════════════════════════════════════════
//  Subdivided AABB — single link → n_sub sub-segment AABBs
// ═════════════════════════════════════════════════════════════════════════════
void derive_aabb_subdivided(
    const float* frames,
    int n_frames,
    int parent_frame_idx,           // proximal frame (-1 = base)
    int link_frame_idx,             // distal frame
    int n_sub,
    float radius,
    const float* base_pos,
    float* out_sub_aabbs);          // [n_sub × 6]

// ═════════════════════════════════════════════════════════════════════════════
//  Adaptive subdivision count
// ═════════════════════════════════════════════════════════════════════════════
int adaptive_subdivision_count(
    const float* frames,
    int parent_frame_idx,
    int link_frame_idx,
    const float* base_pos,
    float cell_size,
    int max_sub);

// ═════════════════════════════════════════════════════════════════════════════
//  Grid derive — occupancy grid from sub-AABB voxelisation
// ═════════════════════════════════════════════════════════════════════════════
void derive_grid(
    const float* frames,
    int n_frames,
    const int* active_link_map,
    int n_active,
    const float* link_radii,
    const float* base_pos,
    const float* world_bounds,      // [6]: x_lo, y_lo, z_lo, x_hi, y_hi, z_hi
    int R,                          // grid resolution per axis
    int n_sub_per_link,             // fixed subdivision count (-1 = adaptive)
    int adaptive_max_sub,           // max when adaptive
    uint8_t* out_grid);             // [R³] bytes, caller zeros

} // namespace envelope
} // namespace sbf

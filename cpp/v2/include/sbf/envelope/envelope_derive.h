// SafeBoxForest v2 — Envelope Derive: frames → AABB / OBB / subdivided AABB
// Module: sbf::envelope
//
// Pure functions (no state).  Given per-frame position intervals stored in
// FrameStore, derive various envelope representations on-the-fly without
// re-running interval FK.
//
// All functions are self-contained and work on raw float pointers so they
// can be called from tight inner loops (FFB descent) without virtual dispatch.
//
#pragma once

#include "sbf/common/types.h"
#include <cstdint>

namespace sbf {
namespace envelope {

// ═════════════════════════════════════════════════════════════════════════════
//  AABB derive — per-link AABB from frame position intervals
// ═════════════════════════════════════════════════════════════════════════════
//
// For each active link k (0-indexed compact):
//   proximal frame = frames[active_link_map[k] - 1]   (or base_pos if map[k]==0)
//   distal   frame = frames[active_link_map[k]]
//   AABB = bbox(proximal, distal) ⊕ link_radii[k]
//
// out_aabb: [n_active × 6] floats, layout [x_lo, y_lo, z_lo, x_hi, y_hi, z_hi]
//
void derive_aabb(
    const float* frames,            // [n_frames × 6]
    int n_frames,
    const int* active_link_map,     // [n_active] compact_idx → frame_idx (0-based)
    int n_active,
    const float* link_radii,        // [n_active] per-link capsule radius (may be nullptr)
    const float* base_pos,          // [3] base frame position
    float* out_aabb);               // [n_active × 6]

// ═════════════════════════════════════════════════════════════════════════════
//  Subdivided AABB — single link → n_sub sub-segment AABBs
// ═════════════════════════════════════════════════════════════════════════════
//
// Splits link into n_sub equal segments via interval linear interpolation.
// Each sub-segment endpoint = lerp(proximal_frame, distal_frame, t).
//
// parent_frame_idx:  index into frames[] for proximal endpoint (-1 = use base_pos)
// link_frame_idx:    index into frames[] for distal endpoint
//
// out_sub_aabbs: [n_sub × 6] floats
//
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
//  Adaptive subdivision count — returns recommended n_sub for a link
// ═════════════════════════════════════════════════════════════════════════════
//
// Based on AABB diagonal relative to cell_size.
// Returns clamped to [2, max_sub].
//
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
//
// For each active link, subdivides into n_sub segments, computes sub-AABBs,
// and marks intersecting voxels in the output grid.
//
// out_grid: byte array of size R³, initialised to zero by caller.
//   out_grid[x * R*R + y * R + z] = 1 if occupied.
//
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

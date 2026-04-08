// SafeBoxForest v2 — CollisionPolicy: unified collision-checking dispatch
// Module: sbf::envelope
//
// A single entry point that dispatches to different collision-checking
// strategies based on the configured CollisionPolicy enum.
// All strategies work with frame position intervals (HCACHE03).
// A legacy AABB-only path is provided for backward compatibility.
//
#pragma once

#include "sbf/common/config.h"
#include "sbf/common/types.h"

#include <cstdint>

namespace sbf {
namespace envelope {

// ═════════════════════════════════════════════════════════════════════════════
//  Unified collision check — dispatches by CollisionPolicy
// ═════════════════════════════════════════════════════════════════════════════
//
// Returns true if the node's envelope collides with ANY obstacle (unsafe box).
// Returns false if certified collision-free.
//
// obs_compact: flat float array [n_obs × 6] in [lo_x hi_x lo_y hi_y lo_z hi_z]
//              (per-axis interleaved, consistent with Scene::pack_obstacles)
//
// workspace_buf: optional pre-allocated scratch buffer to avoid per-call
//   allocations.  Size depends on policy:
//     AABB:       n_active * 6 floats
//     AABB_SUBDIV: n_active * max_sub * 6 floats
//     GRID:       R*R*R bytes (uint8)
//   If nullptr, the function allocates internally (slower for repeated calls).
//
bool check_collision(
    CollisionPolicy policy,
    const float* frames,              // [n_frames × 6]
    int n_frames,
    const int* active_link_map,       // [n_active]
    int n_active,
    const float* link_radii,          // [n_active] (may be nullptr)
    const float* base_pos,            // [3]
    const float* obs_compact,         // [n_obs × 6]
    int n_obs,
    // ── policy parameters ──
    int subdiv_n,                     // for AABB_SUBDIV
    int subdiv_max,                   // for adaptive subdivision
    int grid_R,                       // for GRID policy
    const float* grid_world_bounds,   // [6], for GRID policy
    void* workspace_buf = nullptr);

// ═════════════════════════════════════════════════════════════════════════════
//  Individual collision policies (exposed for benchmarking / testing)
// ═════════════════════════════════════════════════════════════════════════════

// Policy 0: AABB — derive AABB from frames, then 3-axis SAT per (link, obs)
bool collide_aabb(
    const float* frames, int n_frames,
    const int* active_link_map, int n_active,
    const float* link_radii, const float* base_pos,
    const float* obs_compact, int n_obs,
    float* buf_aabb = nullptr);      // [n_active × 6] scratch

// Policy 1: AABB_SUBDIV — AABB first, subdivide on collision
bool collide_aabb_subdiv(
    const float* frames, int n_frames,
    const int* active_link_map, int n_active,
    const float* link_radii, const float* base_pos,
    const float* obs_compact, int n_obs,
    int subdiv_n, int subdiv_max,
    float* buf_aabb = nullptr,        // [n_active × 6] scratch
    float* buf_sub  = nullptr);       // [subdiv_max × 6] scratch

// Policy 2: GRID — derive grid, voxelise obstacles, bitwise AND
bool collide_grid(
    const float* frames, int n_frames,
    const int* active_link_map, int n_active,
    const float* link_radii, const float* base_pos,
    const float* obs_compact, int n_obs,
    int R, const float* world_bounds,
    int n_sub_per_link, int adaptive_max_sub,
    uint8_t* buf_grid = nullptr);     // [R³] scratch

// ═════════════════════════════════════════════════════════════════════════════
//  Legacy AABB-only collision (no FrameStore required)
// ═════════════════════════════════════════════════════════════════════════════
//
// For HCACHE02 backward compatibility — operates directly on pre-computed
// per-link AABB arrays.
//
// link_aabbs: [n_links × 6] in [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]
// obs_compact: [n_obs × 6]  in [lo_x, hi_x, lo_y, hi_y, lo_z, hi_z]
//
bool check_collision_aabb_legacy(
    const float* link_aabbs, int n_links,
    const float* obs_compact, int n_obs);

} // namespace envelope
} // namespace sbf

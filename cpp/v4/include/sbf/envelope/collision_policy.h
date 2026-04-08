// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Collision policy dispatch
//  Module: sbf::envelope
//
//  Dispatch collision checks based on CollisionPolicy:
//    AABB        → per-link iAABB overlap test against Obstacle*
//    AABB_SUBDIV → coarse iAABB quick-reject, then subdivide colliding links
//    GRID        → delegated to GridStore word-level z-mask bit probe
//
//  v4 API: each policy has a standalone function with the minimal required
//  parameters. The unified check_collision() dispatcher handles AABB; callers
//  needing SUBDIV/GRID invoke the standalone functions directly (different
//  data requirements).
//
//  v4 optimisations vs v3:
//    • AABB_SUBDIV: Obstacle.lo()/hi() replaces interleaved obs_compact;
//      stack buffer for sub-AABBs (up to 64); early-exit per (link,obs) pair.
//    • GRID: delegates to GridStore::check_collision() which uses word-level
//      z-mask AND (1 instruction per (x,y) cell vs up to 32 bit tests in v3).
//
//  迁移自 v3 collision_policy.h
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/core/config.h"
#include "sbf/core/types.h"

#include <cstdint>

namespace sbf {
namespace envelope {

// Forward declaration — full header is heavy (mmap_util etc.)
class GridStore;

// ═════════════════════════════════════════════════════════════════════════════
//  Unified collision check — dispatches by CollisionPolicy (AABB only)
// ═════════════════════════════════════════════════════════════════════════════
//
// For AABB_SUBDIV / GRID, use the standalone functions below directly
// (they require additional data not available through this interface).
// If called with AABB_SUBDIV / GRID, falls back to AABB.
//
bool check_collision(
    CollisionPolicy policy,
    const float* link_iaabbs,
    int n_active_links,
    const Obstacle* obstacles,
    int n_obs);

// ═════════════════════════════════════════════════════════════════════════════
//  Individual collision policies (exposed for benchmarking / testing)
// ═════════════════════════════════════════════════════════════════════════════

/// Policy 0: AABB — per-link iAABB vs Obstacle SAT (3-axis overlap test)
bool collide_aabb(
    const float* link_iaabbs, int n_active_links,
    const Obstacle* obstacles, int n_obs);

/// Policy 1: AABB_SUBDIV — coarse iAABB quick-reject, then subdivide
///
/// Two-stage check: for each (link, obstacle) pair whose coarse iAABBs
/// overlap, generates n_sub subdivided sub-AABBs via derive_aabb_subdivided()
/// and tests each sub-AABB individually.
///
/// link_iaabbs:     [n_active × 6] pre-computed coarse iAABBs (quick-reject)
/// frames:          [n_frames × 6] endpoint position intervals (for subdivision)
/// active_link_map: [n_active] compact_idx → frame_idx
/// link_radii:      [n_active] per-link capsule radius (may be nullptr)
/// base_pos:        [3] base frame position
/// subdiv_n:        fixed subdivision count (>0), or 0 for adaptive
/// subdiv_max:      max subdivisions when adaptive (capped at 64)
///
bool collide_aabb_subdiv(
    const float* link_iaabbs,
    const float* frames, int n_frames,
    const int* active_link_map, int n_active,
    const float* link_radii, const float* base_pos,
    const Obstacle* obstacles, int n_obs,
    int subdiv_n = 8, int subdiv_max = 64);

/// Policy 2: GRID — delegated to GridStore word-level z-mask bit probe
///
/// The GridStore must have a pre-populated grid for node_idx (via
/// derive_from_aabbs). Uses word-level z-mask AND for fast collision.
///
bool collide_grid(
    const GridStore& store, int node_idx,
    const Obstacle* obstacles, int n_obs);

/// Legacy AABB collision check (pre-computed link AABBs vs raw obs compact)
/// obs_compact: [n_obs × 6] in [lo_x, hi_x, lo_y, hi_y, lo_z, hi_z]
bool check_collision_aabb_legacy(
    const float* link_iaabbs, int n_links,
    const float* obs_compact, int n_obs);

} // namespace envelope
} // namespace sbf

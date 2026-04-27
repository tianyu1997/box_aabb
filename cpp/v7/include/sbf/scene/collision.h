#pragma once
/// @file collision.h
/// @brief AABB-vs-Obstacle SAT batch collision tests.
///
/// Two flavours:
///   - `aabbs_collide_obs`           — AABBs are already inflated.
///   - `aabbs_collide_obs_inflated`  — D1: AABBs are zero-radius and a
///                                     per-slot radius is applied inline.
///
/// Obstacle layout is the v6 compact form:
///   `obs_compact[n_obs * 6]` = [lo_x, hi_x, lo_y, hi_y, lo_z, hi_z]
/// (note this differs from `Obstacle::bounds` which is xyzxyz; use
/// `pack_obstacles()` to convert).

#include "sbf/scene/obstacle.h"

namespace sbf::scene {

/// SAT batch test (already-inflated AABBs).
/// @param aabb         link AABBs [n_slots * 6]: [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]
/// @param n_slots      number of link slots
/// @param obs_compact  obstacles [n_obs * 6]: [lo_x, hi_x, lo_y, hi_y, lo_z, hi_z]
/// @param n_obs        number of obstacles
bool aabbs_collide_obs(const float* aabb, int n_slots,
                       const float* obs_compact, int n_obs);

/// D1-style: zero-radius link AABBs + per-slot radius inflation, then SAT.
/// @param zero_aabb              zero-radius link AABBs [n_slots * 6]
/// @param n_slots                number of link slots
/// @param link_radii_per_slot    radius per slot [n_slots] (broadcast already
///                               applied by caller for subdivided mode)
/// @param obs_compact            obstacles [n_obs * 6] (xhxyhyzhz layout)
/// @param n_obs                  number of obstacles
bool aabbs_collide_obs_inflated(const float* zero_aabb, int n_slots,
                                const float* link_radii_per_slot,
                                const float* obs_compact, int n_obs);

/// Pack `Obstacle::bounds` (xyzxyz) into the SAT-friendly compact form
/// (xhxyhyzhz). `out_compact` must have room for n_obs * 6 floats.
void pack_obstacles(const Obstacle* obs, int n_obs, float* out_compact);

/// Helper: expand per-active-link radii to per-slot radii for subdivided
/// envelopes. `out_per_slot[n_active * n_subdivisions]` receives the
/// broadcasted values (radii[ci] repeated n_subdivisions times).
void expand_radii_to_slots(const double* link_radii,
                           int n_active_links,
                           int n_subdivisions,
                           float* out_per_slot);

}  // namespace sbf::scene

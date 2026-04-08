// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Collision policy dispatch implementation
//
//  v4 API: pre-computed link_iaabbs + Obstacle*; AABB_SUBDIV / GRID policies
//  exposed as standalone functions with their specific data requirements.
//
//  v4 optimisations vs v3:
//    AABB_SUBDIV — Obstacle.lo()/hi() instead of interleaved obs_compact;
//                  stack buffer for sub-AABBs; early-exit per (link,obs).
//    GRID        — delegates to GridStore word-level z-mask check_collision()
//                  (1 uint64 AND per (x,y) row vs per-voxel uint8 in v3).
//
//  迁移自 v3 collision_policy.cpp
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/envelope/collision_policy.h"
#include "sbf/envelope/envelope_derive.h"
#include "sbf/envelope/grid_store.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace sbf {
namespace envelope {

// ─── Inline helpers ─────────────────────────────────────────────────────────
//
// Link iAABB format:  [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]  (contiguous lo/hi)
// Obstacle:          centre + half_sizes → lo() / hi() as Eigen::Vector3d
//

// Link iAABB (a) [lo_x,lo_y,lo_z, hi_x,hi_y,hi_z]
// vs Obstacle (o) using o.lo() / o.hi()
static inline bool aabb_vs_obstacle(const float* a, const Obstacle& o) {
    auto olo = o.lo();
    auto ohi = o.hi();
    return a[3] >= static_cast<float>(olo[0]) && a[0] <= static_cast<float>(ohi[0]) &&   // x
           a[4] >= static_cast<float>(olo[1]) && a[1] <= static_cast<float>(ohi[1]) &&   // y
           a[5] >= static_cast<float>(olo[2]) && a[2] <= static_cast<float>(ohi[2]);     // z
}

// Sub-AABB (a) [lo_x,lo_y,lo_z, hi_x,hi_y,hi_z]
// vs Obstacle bounds (pre-computed lo/hi as float)
static inline bool aabb_vs_obs_bounds(const float* a,
                                      float olo0, float olo1, float olo2,
                                      float ohi0, float ohi1, float ohi2) {
    return a[3] >= olo0 && a[0] <= ohi0 &&
           a[4] >= olo1 && a[1] <= ohi1 &&
           a[5] >= olo2 && a[2] <= ohi2;
}

// Link iAABB (a) vs interleaved obs [lo_x,hi_x, lo_y,hi_y, lo_z,hi_z]
static inline bool aabb_vs_obs_interleaved(const float* a, const float* o) {
    return a[3] >= o[0] && a[0] <= o[1] &&   // x
           a[4] >= o[2] && a[1] <= o[3] &&   // y
           a[5] >= o[4] && a[2] <= o[5];     // z
}

// ═════════════════════════════════════════════════════════════════════════════
//  collide_aabb  (Policy 0) — v4 Obstacle* API
// ═════════════════════════════════════════════════════════════════════════════

bool collide_aabb(
    const float* link_iaabbs, int n_active_links,
    const Obstacle* obstacles, int n_obs)
{
    for (int li = 0; li < n_active_links; ++li) {
        const float* la = link_iaabbs + li * 6;
        for (int oi = 0; oi < n_obs; ++oi) {
            if (aabb_vs_obstacle(la, obstacles[oi]))
                return true;
        }
    }
    return false;
}

// ═════════════════════════════════════════════════════════════════════════════
//  collide_aabb_subdiv  (Policy 1) — two-stage coarse→refine
// ═════════════════════════════════════════════════════════════════════════════

bool collide_aabb_subdiv(
    const float* link_iaabbs,
    const float* frames, int n_frames,
    const int* active_link_map, int n_active,
    const float* link_radii, const float* base_pos,
    const Obstacle* obstacles, int n_obs,
    int subdiv_n, int subdiv_max)
{
    // Cap max subdivisions to 64 (stack buffer limit)
    constexpr int kMaxSubStack = 64;
    int max_sub = subdiv_max > 0 ? std::min(subdiv_max, kMaxSubStack)
                                 : (subdiv_n > 0 ? std::min(subdiv_n, kMaxSubStack) : 8);

    // Stack buffer for sub-AABBs (reused per link)
    float stack_sub[kMaxSubStack * 6];
    float* sub = stack_sub;
    std::vector<float> heap_sub;
    if (max_sub > kMaxSubStack) {
        heap_sub.resize(max_sub * 6);
        sub = heap_sub.data();
    }

    for (int li = 0; li < n_active; ++li) {
        const float* la = link_iaabbs + li * 6;

        for (int oi = 0; oi < n_obs; ++oi) {
            const Obstacle& obs = obstacles[oi];

            // --- Stage 1: coarse AABB quick-reject ---
            if (!aabb_vs_obstacle(la, obs)) continue;

            // --- Stage 2: subdivision refinement ---
            int frame_idx  = active_link_map[li];
            int parent_idx = frame_idx - 1;    // proximal frame
            float r = link_radii ? link_radii[li] : 0.f;

            int n_sub;
            if (subdiv_n > 0) {
                n_sub = subdiv_n;
            } else {
                // Adaptive: subdivision count based on obstacle min dimension
                // v4: use Obstacle.lo()/hi() directly (no interleaved obs_compact)
                auto olo = obs.lo();
                auto ohi = obs.hi();
                float dx = static_cast<float>(ohi[0] - olo[0]);
                float dy = static_cast<float>(ohi[1] - olo[1]);
                float dz = static_cast<float>(ohi[2] - olo[2]);
                float min_obs_dim = std::min({dx, dy, dz});
                float cell = std::max(min_obs_dim * 0.5f, 0.01f);
                n_sub = adaptive_subdivision_count(
                    frames, parent_idx, frame_idx, base_pos, cell, max_sub);
            }
            n_sub = std::min(n_sub, max_sub);

            derive_aabb_subdivided(frames, n_frames,
                                   parent_idx, frame_idx,
                                   n_sub, r, base_pos, sub);

            // Pre-compute obstacle bounds as float for tight inner loop
            auto olo = obs.lo();
            auto ohi = obs.hi();
            float olo0 = static_cast<float>(olo[0]);
            float olo1 = static_cast<float>(olo[1]);
            float olo2 = static_cast<float>(olo[2]);
            float ohi0 = static_cast<float>(ohi[0]);
            float ohi1 = static_cast<float>(ohi[1]);
            float ohi2 = static_cast<float>(ohi[2]);

            for (int s = 0; s < n_sub; ++s) {
                if (aabb_vs_obs_bounds(sub + s * 6,
                                       olo0, olo1, olo2,
                                       ohi0, ohi1, ohi2))
                    return true;
            }
        }
    }
    return false;
}

// ═════════════════════════════════════════════════════════════════════════════
//  collide_grid  (Policy 2) — delegates to GridStore word-level z-mask
// ═════════════════════════════════════════════════════════════════════════════

bool collide_grid(
    const GridStore& store, int node_idx,
    const Obstacle* obstacles, int n_obs)
{
    return store.check_collision(node_idx, obstacles, n_obs);
}

// ═════════════════════════════════════════════════════════════════════════════
//  check_collision_aabb_legacy — interleaved obs_compact backward compat
// ═════════════════════════════════════════════════════════════════════════════

bool check_collision_aabb_legacy(
    const float* link_iaabbs, int n_links,
    const float* obs_compact, int n_obs)
{
    for (int li = 0; li < n_links; ++li) {
        const float* la = link_iaabbs + li * 6;
        for (int oi = 0; oi < n_obs; ++oi) {
            if (aabb_vs_obs_interleaved(la, obs_compact + oi * 6))
                return true;
        }
    }
    return false;
}

// ═════════════════════════════════════════════════════════════════════════════
//  check_collision — unified dispatch (AABB only; SUBDIV / GRID = fallback)
// ═════════════════════════════════════════════════════════════════════════════
//
// NOTE: AABB_SUBDIV and GRID require additional data not available through
// this interface (raw frames for SUBDIV, GridStore for GRID). When the caller
// has the required data, use the standalone functions directly:
//   collide_aabb_subdiv(...)
//   collide_grid(store, node_idx, ...)
// This dispatcher falls back to AABB for those policies.

bool check_collision(
    CollisionPolicy policy,
    const float* link_iaabbs,
    int n_active_links,
    const Obstacle* obstacles,
    int n_obs)
{
    switch (policy) {
    case CollisionPolicy::AABB:
        return collide_aabb(link_iaabbs, n_active_links, obstacles, n_obs);

    case CollisionPolicy::AABB_SUBDIV:
        // Requires raw frames — fallback to AABB through this interface.
        // Use collide_aabb_subdiv() directly when frames are available.
        return collide_aabb(link_iaabbs, n_active_links, obstacles, n_obs);

    case CollisionPolicy::GRID:
        // Requires GridStore — fallback to AABB through this interface.
        // Use collide_grid(store, node_idx, ...) directly.
        return collide_aabb(link_iaabbs, n_active_links, obstacles, n_obs);

    default:
        return collide_aabb(link_iaabbs, n_active_links, obstacles, n_obs);
    }
}

} // namespace envelope
} // namespace sbf

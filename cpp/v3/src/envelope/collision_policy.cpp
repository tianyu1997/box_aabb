// SafeBoxForest v2 — CollisionPolicy: unified collision-checking dispatch
#include "sbf/envelope/collision_policy.h"
#include "sbf/envelope/envelope_derive.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace sbf {
namespace envelope {

// ─── Inline helpers ─────────────────────────────────────────────────────────
//
// IMPORTANT: Two different compact formats coexist:
//   Link AABB:  [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]  (contiguous lo/hi)
//   Obstacle:   [lo_x, hi_x, lo_y, hi_y, lo_z, hi_z]  (per-axis interleaved)
//
// The obstacle format is set by Scene::pack_obstacles() and is the standard
// throughout v1/v2.  All functions below handle the mixed formats.

// Link AABB (a) vs Link AABB (b) — both in [lo_x,lo_y,lo_z, hi_x,hi_y,hi_z]
static inline bool aabb_overlap(const float* a, const float* b) {
    return a[0] <= b[3] && a[3] >= b[0] &&
           a[1] <= b[4] && a[4] >= b[1] &&
           a[2] <= b[5] && a[5] >= b[2];
}

// Link AABB (a) [lo_x,lo_y,lo_z, hi_x,hi_y,hi_z]
// vs Obstacle (o) [lo_x,hi_x, lo_y,hi_y, lo_z,hi_z]
static inline bool aabb_vs_obs(const float* a, const float* o) {
    return a[3] >= o[0] && a[0] <= o[1] &&   // x
           a[4] >= o[2] && a[1] <= o[3] &&   // y
           a[5] >= o[4] && a[2] <= o[5];     // z
}

// ═════════════════════════════════════════════════════════════════════════════
//  check_collision_aabb_legacy
// ═════════════════════════════════════════════════════════════════════════════

bool check_collision_aabb_legacy(
    const float* link_aabbs, int n_links,
    const float* obs_compact, int n_obs)
{
    // link_aabbs: [lo_x,lo_y,lo_z, hi_x,hi_y,hi_z] per link
    // obs_compact: [lo_x,hi_x, lo_y,hi_y, lo_z,hi_z] per obs (interleaved)
    for (int li = 0; li < n_links; ++li) {
        const float* la = link_aabbs + li * 6;
        for (int oi = 0; oi < n_obs; ++oi) {
            if (aabb_vs_obs(la, obs_compact + oi * 6))
                return true;
        }
    }
    return false;
}

// ═════════════════════════════════════════════════════════════════════════════
//  collide_aabb  (Policy 0)
// ═════════════════════════════════════════════════════════════════════════════

bool collide_aabb(
    const float* frames, int n_frames,
    const int* active_link_map, int n_active,
    const float* link_radii, const float* base_pos,
    const float* obs_compact, int n_obs,
    float* buf_aabb)
{
    // Stack buffer up to 16 links (16 × 6 = 96 floats = 384 B)
    float stack_buf[16 * 6];
    float* aabb = buf_aabb ? buf_aabb
                           : (n_active <= 16 ? stack_buf : nullptr);

    std::vector<float> heap_buf;
    if (!aabb) {
        heap_buf.resize(n_active * 6);
        aabb = heap_buf.data();
    }

    derive_aabb(frames, n_frames, active_link_map, n_active,
                link_radii, base_pos, aabb);

    // obs_compact is in interleaved format [lo_x,hi_x,lo_y,hi_y,lo_z,hi_z]
    return check_collision_aabb_legacy(aabb, n_active, obs_compact, n_obs);
}

// ═════════════════════════════════════════════════════════════════════════════
//  collide_aabb_subdiv  (Policy 1)
// ═════════════════════════════════════════════════════════════════════════════

bool collide_aabb_subdiv(
    const float* frames, int n_frames,
    const int* active_link_map, int n_active,
    const float* link_radii, const float* base_pos,
    const float* obs_compact, int n_obs,
    int subdiv_n, int subdiv_max,
    float* buf_aabb, float* buf_sub)
{
    // --- Stage 1: coarse AABB check ---
    float stack_aabb[16 * 6];
    float* aabb = buf_aabb ? buf_aabb
                           : (n_active <= 16 ? stack_aabb : nullptr);
    std::vector<float> heap_aabb;
    if (!aabb) { heap_aabb.resize(n_active * 6); aabb = heap_aabb.data(); }

    derive_aabb(frames, n_frames, active_link_map, n_active,
                link_radii, base_pos, aabb);

    // Scratch for sub-AABBs (per-link, reused)
    int max_sub = subdiv_max > 0 ? subdiv_max : (subdiv_n > 0 ? subdiv_n : 8);
    float stack_sub[64 * 6];
    float* sub = buf_sub ? buf_sub
                         : (max_sub <= 64 ? stack_sub : nullptr);
    std::vector<float> heap_sub;
    if (!sub) { heap_sub.resize(max_sub * 6); sub = heap_sub.data(); }

    for (int li = 0; li < n_active; ++li) {
        const float* la = aabb + li * 6;

        for (int oi = 0; oi < n_obs; ++oi) {
            const float* ob = obs_compact + oi * 6;
            // ob is interleaved [lo_x,hi_x,lo_y,hi_y,lo_z,hi_z]
            if (!aabb_vs_obs(la, ob)) continue;

            // Coarse collision → refine with subdivision
            int frame_idx  = active_link_map[li];
            int parent_idx = frame_idx - 1;
            float r = link_radii ? link_radii[li] : 0.f;

            int n_sub;
            if (subdiv_n > 0) {
                n_sub = subdiv_n;
            } else {
                // Adaptive: AABB diagonal vs smallest obs dimension
                // ob[1]-ob[0]=dx, ob[3]-ob[2]=dy, ob[5]-ob[4]=dz
                float min_obs_dim = std::min({ob[1]-ob[0], ob[3]-ob[2], ob[5]-ob[4]});
                float cell = std::max(min_obs_dim * 0.5f, 0.01f);
                n_sub = adaptive_subdivision_count(
                    frames, parent_idx, frame_idx, base_pos, cell, max_sub);
            }
            n_sub = std::min(n_sub, max_sub);

            derive_aabb_subdivided(frames, n_frames,
                                   parent_idx, frame_idx,
                                   n_sub, r, base_pos, sub);

            for (int s = 0; s < n_sub; ++s) {
                if (aabb_vs_obs(sub + s * 6, ob))
                    return true;
            }
        }
    }
    return false;
}

// ═════════════════════════════════════════════════════════════════════════════
//  collide_grid  (Policy 2)
// ═════════════════════════════════════════════════════════════════════════════

bool collide_grid(
    const float* frames, int n_frames,
    const int* active_link_map, int n_active,
    const float* link_radii, const float* base_pos,
    const float* obs_compact, int n_obs,
    int R, const float* world_bounds,
    int n_sub_per_link, int adaptive_max_sub,
    uint8_t* buf_grid)
{
    int grid_sz = R * R * R;
    uint8_t* grid = buf_grid;
    std::vector<uint8_t> heap_grid;
    if (!grid) {
        heap_grid.resize(grid_sz, 0);
        grid = heap_grid.data();
    } else {
        std::memset(grid, 0, grid_sz);
    }

    // Robot occupancy grid
    derive_grid(frames, n_frames, active_link_map, n_active,
                link_radii, base_pos, world_bounds, R,
                n_sub_per_link, adaptive_max_sub, grid);

    // Voxelise obstacles and check overlap
    float inv_cell[3];
    for (int c = 0; c < 3; ++c)
        inv_cell[c] = static_cast<float>(R) / (world_bounds[3 + c] - world_bounds[c]);

    for (int oi = 0; oi < n_obs; ++oi) {
        const float* ob = obs_compact + oi * 6;
        // ob interleaved: [lo_x,hi_x, lo_y,hi_y, lo_z,hi_z]
        int ix0 = static_cast<int>(std::floor((ob[0] - world_bounds[0]) * inv_cell[0]));
        int iy0 = static_cast<int>(std::floor((ob[2] - world_bounds[1]) * inv_cell[1]));
        int iz0 = static_cast<int>(std::floor((ob[4] - world_bounds[2]) * inv_cell[2]));
        int ix1 = static_cast<int>(std::ceil((ob[1] - world_bounds[0]) * inv_cell[0]));
        int iy1 = static_cast<int>(std::ceil((ob[3] - world_bounds[1]) * inv_cell[1]));
        int iz1 = static_cast<int>(std::ceil((ob[5] - world_bounds[2]) * inv_cell[2]));

        ix0 = std::max(ix0, 0); ix1 = std::min(ix1, R);
        iy0 = std::max(iy0, 0); iy1 = std::min(iy1, R);
        iz0 = std::max(iz0, 0); iz1 = std::min(iz1, R);

        for (int x = ix0; x < ix1; ++x)
            for (int y = iy0; y < iy1; ++y)
                for (int z = iz0; z < iz1; ++z)
                    if (grid[x * R * R + y * R + z])
                        return true;
    }
    return false;
}

// ═════════════════════════════════════════════════════════════════════════════
//  check_collision — unified dispatch
// ═════════════════════════════════════════════════════════════════════════════

bool check_collision(
    CollisionPolicy policy,
    const float* frames,
    int n_frames,
    const int* active_link_map,
    int n_active,
    const float* link_radii,
    const float* base_pos,
    const float* obs_compact,
    int n_obs,
    int subdiv_n,
    int subdiv_max,
    int grid_R,
    const float* grid_world_bounds,
    void* workspace_buf)
{
    switch (policy) {
    case CollisionPolicy::AABB:
        return collide_aabb(frames, n_frames, active_link_map, n_active,
                            link_radii, base_pos, obs_compact, n_obs,
                            static_cast<float*>(workspace_buf));

    case CollisionPolicy::AABB_SUBDIV:
        return collide_aabb_subdiv(frames, n_frames, active_link_map, n_active,
                                   link_radii, base_pos, obs_compact, n_obs,
                                   subdiv_n, subdiv_max,
                                   static_cast<float*>(workspace_buf), nullptr);

    case CollisionPolicy::GRID:
        return collide_grid(frames, n_frames, active_link_map, n_active,
                            link_radii, base_pos, obs_compact, n_obs,
                            grid_R, grid_world_bounds,
                            subdiv_n, subdiv_max,
                            static_cast<uint8_t*>(workspace_buf));

    default:
        return collide_aabb(frames, n_frames, active_link_map, n_active,
                            link_radii, base_pos, obs_compact, n_obs,
                            static_cast<float*>(workspace_buf));
    }
}

} // namespace envelope
} // namespace sbf

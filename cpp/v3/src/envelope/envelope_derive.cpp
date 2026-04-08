// SafeBoxForest v2 — Envelope Derive implementation
#include "sbf/envelope/envelope_derive.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>

namespace sbf {
namespace envelope {

// ─── Helper: get frame lo/hi pointers ───────────────────────────────────────

static inline const float* frame_lo(const float* frames, int idx) {
    return frames + idx * 6;       // [x_lo, y_lo, z_lo, ...]
}

static inline const float* frame_hi(const float* frames, int idx) {
    return frames + idx * 6 + 3;   // [..., x_hi, y_hi, z_hi]
}

// Helper: get lo/hi accounting for base_pos when idx == -1
static inline void get_endpoint(const float* frames, int idx,
                                const float* base_pos,
                                float out_lo[3], float out_hi[3]) {
    if (idx < 0) {
        // Base frame: constant, lo == hi == base_pos
        out_lo[0] = base_pos[0]; out_lo[1] = base_pos[1]; out_lo[2] = base_pos[2];
        out_hi[0] = base_pos[0]; out_hi[1] = base_pos[1]; out_hi[2] = base_pos[2];
    } else {
        const float* f = frames + idx * 6;
        out_lo[0] = f[0]; out_lo[1] = f[1]; out_lo[2] = f[2];
        out_hi[0] = f[3]; out_hi[1] = f[4]; out_hi[2] = f[5];
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  derive_aabb
// ═════════════════════════════════════════════════════════════════════════════

void derive_aabb(
    const float* frames, int /*n_frames*/,
    const int* active_link_map, int n_active,
    const float* link_radii, const float* base_pos,
    float* out_aabb)
{
    for (int i = 0; i < n_active; ++i) {
        int frame_idx = active_link_map[i];           // distal frame (0-based)
        int parent_idx = frame_idx - 1;               // proximal frame (-1 = base)

        float s_lo[3], s_hi[3], e_lo[3], e_hi[3];
        get_endpoint(frames, parent_idx, base_pos, s_lo, s_hi);
        get_endpoint(frames, frame_idx, base_pos, e_lo, e_hi);

        float* out = out_aabb + i * 6;
        out[0] = std::min(s_lo[0], e_lo[0]);
        out[1] = std::min(s_lo[1], e_lo[1]);
        out[2] = std::min(s_lo[2], e_lo[2]);
        out[3] = std::max(s_hi[0], e_hi[0]);
        out[4] = std::max(s_hi[1], e_hi[1]);
        out[5] = std::max(s_hi[2], e_hi[2]);

        if (link_radii) {
            float r = link_radii[i];
            out[0] -= r; out[1] -= r; out[2] -= r;
            out[3] += r; out[4] += r; out[5] += r;
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  derive_aabb_subdivided
// ═════════════════════════════════════════════════════════════════════════════

void derive_aabb_subdivided(
    const float* frames, int /*n_frames*/,
    int parent_frame_idx, int link_frame_idx,
    int n_sub, float radius, const float* base_pos,
    float* out_sub_aabbs)
{
    assert(n_sub >= 1);

    float s_lo[3], s_hi[3], e_lo[3], e_hi[3];
    get_endpoint(frames, parent_frame_idx, base_pos, s_lo, s_hi);
    get_endpoint(frames, link_frame_idx, base_pos, e_lo, e_hi);

    const float inv_n = 1.0f / static_cast<float>(n_sub);

    for (int s = 0; s < n_sub; ++s) {
        float t0 = static_cast<float>(s) * inv_n;
        float t1 = static_cast<float>(s + 1) * inv_n;

        // Interval lerp: seg_start = lerp(start, end, t0)
        // For lo:  seg_start_lo = s_lo * (1-t0) + e_lo * t0
        // For hi:  seg_start_hi = s_hi * (1-t0) + e_hi * t0
        float seg_s_lo[3], seg_s_hi[3], seg_e_lo[3], seg_e_hi[3];
        for (int c = 0; c < 3; ++c) {
            seg_s_lo[c] = s_lo[c] * (1.f - t0) + e_lo[c] * t0;
            seg_s_hi[c] = s_hi[c] * (1.f - t0) + e_hi[c] * t0;
            seg_e_lo[c] = s_lo[c] * (1.f - t1) + e_lo[c] * t1;
            seg_e_hi[c] = s_hi[c] * (1.f - t1) + e_hi[c] * t1;
        }

        float* out = out_sub_aabbs + s * 6;
        out[0] = std::min(seg_s_lo[0], seg_e_lo[0]) - radius;
        out[1] = std::min(seg_s_lo[1], seg_e_lo[1]) - radius;
        out[2] = std::min(seg_s_lo[2], seg_e_lo[2]) - radius;
        out[3] = std::max(seg_s_hi[0], seg_e_hi[0]) + radius;
        out[4] = std::max(seg_s_hi[1], seg_e_hi[1]) + radius;
        out[5] = std::max(seg_s_hi[2], seg_e_hi[2]) + radius;
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  adaptive_subdivision_count
// ═════════════════════════════════════════════════════════════════════════════

int adaptive_subdivision_count(
    const float* frames,
    int parent_frame_idx,
    int link_frame_idx,
    const float* base_pos,
    float cell_size,
    int max_sub)
{
    float s_lo[3], s_hi[3], e_lo[3], e_hi[3];
    get_endpoint(frames, parent_frame_idx, base_pos, s_lo, s_hi);
    get_endpoint(frames, link_frame_idx, base_pos, e_lo, e_hi);

    // Full link AABB diagonal
    float diag = 0.f;
    for (int c = 0; c < 3; ++c) {
        float lo = std::min(s_lo[c], e_lo[c]);
        float hi = std::max(s_hi[c], e_hi[c]);
        float d = hi - lo;
        diag += d * d;
    }
    diag = std::sqrt(diag);

    int n = std::max(2, static_cast<int>(std::ceil(diag / cell_size)));
    return std::min(n, max_sub);
}

// ═════════════════════════════════════════════════════════════════════════════
//  derive_grid
// ═════════════════════════════════════════════════════════════════════════════

void derive_grid(
    const float* frames, int n_frames,
    const int* active_link_map, int n_active,
    const float* link_radii, const float* base_pos,
    const float* world_bounds, int R,
    int n_sub_per_link, int adaptive_max_sub,
    uint8_t* out_grid)
{
    // Cell sizes
    float cell[3];
    for (int c = 0; c < 3; ++c)
        cell[c] = (world_bounds[3 + c] - world_bounds[c]) / static_cast<float>(R);

    // Min cell size (for adaptive subdivision)
    float min_cell = std::min({cell[0], cell[1], cell[2]});

    // Temp buffer for sub-segment AABBs (stack-allocate up to 64 subs)
    float sub_buf[64 * 6];

    for (int i = 0; i < n_active; ++i) {
        int frame_idx  = active_link_map[i];
        int parent_idx = frame_idx - 1;
        float r = link_radii ? link_radii[i] : 0.f;

        // Determine n_sub
        int n_sub;
        if (n_sub_per_link > 0) {
            n_sub = n_sub_per_link;
        } else {
            n_sub = adaptive_subdivision_count(
                frames, parent_idx, frame_idx, base_pos,
                min_cell, adaptive_max_sub);
        }
        if (n_sub > 64) n_sub = 64;  // safety clamp

        // Generate sub-segment AABBs
        derive_aabb_subdivided(frames, n_frames,
                               parent_idx, frame_idx,
                               n_sub, r, base_pos, sub_buf);

        // Voxelise each sub-AABB
        for (int s = 0; s < n_sub; ++s) {
            const float* aabb = sub_buf + s * 6;

            // Convert to grid coordinates
            int ix0 = static_cast<int>(std::floor((aabb[0] - world_bounds[0]) / cell[0]));
            int iy0 = static_cast<int>(std::floor((aabb[1] - world_bounds[1]) / cell[1]));
            int iz0 = static_cast<int>(std::floor((aabb[2] - world_bounds[2]) / cell[2]));
            int ix1 = static_cast<int>(std::ceil((aabb[3] - world_bounds[0]) / cell[0]));
            int iy1 = static_cast<int>(std::ceil((aabb[4] - world_bounds[1]) / cell[1]));
            int iz1 = static_cast<int>(std::ceil((aabb[5] - world_bounds[2]) / cell[2]));

            // Clamp
            ix0 = std::max(ix0, 0); ix1 = std::min(ix1, R);
            iy0 = std::max(iy0, 0); iy1 = std::min(iy1, R);
            iz0 = std::max(iz0, 0); iz1 = std::min(iz1, R);

            // Mark voxels
            for (int x = ix0; x < ix1; ++x)
                for (int y = iy0; y < iy1; ++y)
                    for (int z = iz0; z < iz1; ++z)
                        out_grid[x * R * R + y * R + z] = 1;
        }
    }
}

} // namespace envelope
} // namespace sbf

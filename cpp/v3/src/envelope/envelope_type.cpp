// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v3 — Link Envelope Module implementation
//
//  All three envelope types consume the unified endpoint_aabb intermediate
//  (per-endpoint position intervals [n_endpoints × 6], geometry only):
//    SubAABB:      sub-AABBs via interpolation from endpoints + r
//                  (n_sub=1 → per-link AABB = union(endpoint[parent], endpoint[child]) + r)
//    SubAABB_Grid: interpolated sub-AABBs rasterised into R³ byte grid
//    Hull16_Grid:  interpolated sub-AABBs rasterised into sparse VoxelGrid
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/envelope/envelope_type.h"
#include "sbf/envelope/envelope_derive.h"
#include "sbf/voxel/hull_rasteriser.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace sbf {
namespace envelope {

// ═════════════════════════════════════════════════════════════════════════════
//  default_envelope_config — optimal hyperparameters per pipeline combination
// ═════════════════════════════════════════════════════════════════════════════
EnvelopeTypeConfig default_envelope_config(
    EndpointSource source, EnvelopeType type)
{
    EnvelopeTypeConfig c;
    c.type = type;

    switch (type) {
    case EnvelopeType::SubAABB:
        // All sources: n_sub=1 default (single AABB per link)
        c.n_sub = 1;
        c.delta = 0.02;
        c.grid_R = 32;
        break;

    case EnvelopeType::SubAABB_Grid:
        // All sources: best at n_sub=16, grid_R=64
        c.n_sub = 16;
        c.delta = 0.02;
        c.grid_R = 64;
        break;

    case EnvelopeType::Hull16_Grid:
        c.delta = 0.01;
        c.grid_R = 32;
        if (source == EndpointSource::Analytical ||
            source == EndpointSource::GCPC) {
            // Analytical/GCPC: n_sub=16 (finer subdivision for voxel precision)
            c.n_sub = 16;
        } else {
            // IFK / CritSample: n_sub=1
            c.n_sub = 1;
        }
        break;
    }

    return c;
}

// ─── Helper: derive sub-AABBs via interpolation from endpoints ────────────
static void derive_sub_aabbs_from_endpoints(
    const EndpointAABBResult& ep_result,
    const Robot& robot,
    int n_sub,
    std::vector<float>& out)
{
    const int n_act = ep_result.n_active;
    const int n_endpoints = ep_result.n_endpoints;
    const int* alm = robot.active_link_map();
    const double* lr = robot.active_link_radii();

    out.resize(n_act * n_sub * 6);
    float base_pos[3] = {0.f, 0.f, 0.f};

    for (int ci = 0; ci < n_act; ++ci) {
        int parent_fi = (alm[ci] == 0) ? -1 : alm[ci] - 1;
        int link_fi = alm[ci];
        float r = lr ? static_cast<float>(lr[ci]) : 0.f;
        derive_aabb_subdivided(
            ep_result.endpoint_aabbs.data(), n_endpoints,
            parent_fi, link_fi,
            n_sub, r, base_pos,
            out.data() + ci * n_sub * 6);
    }
}

// ─── Helper: compute sub-AABB volume ────────────────────────────────────────
static double compute_sub_aabb_volume(const float* sub_aabbs, int n_boxes)
{
    double vol = 0;
    for (int k = 0; k < n_boxes; ++k) {
        const float* a = sub_aabbs + k * 6;
        double dx = std::max(0.0, static_cast<double>(a[3] - a[0]));
        double dy = std::max(0.0, static_cast<double>(a[4] - a[1]));
        double dz = std::max(0.0, static_cast<double>(a[5] - a[2]));
        vol += dx * dy * dz;
    }
    return vol;
}

// ═════════════════════════════════════════════════════════════════════════════
//  compute_link_envelope — derive envelope from endpoint AABB result
// ═════════════════════════════════════════════════════════════════════════════
EnvelopeResult compute_link_envelope(
    const EnvelopeTypeConfig& env_config,
    const Robot& robot,
    const EndpointAABBResult& ep_result)
{
    const int n_act = ep_result.n_active;
    const int env_n_sub = env_config.n_sub;

    EnvelopeResult res;

    switch (env_config.type) {

    // ─── SubAABB: interpolate sub-AABBs from endpoints ─────────────────
    //     n_sub=1 → single AABB per link (cheapest, most conservative)
    //     n_sub>1 → subdivided sub-AABBs via interpolation
    case EnvelopeType::SubAABB: {
        if (env_n_sub <= 1) {
            // No subdivision: derive per-link AABBs from endpoints
            res.sub_aabbs.resize(n_act * 6);
            extract_link_aabbs_from_endpoint(ep_result, robot,
                                             res.sub_aabbs.data());
        } else {
            // Derive sub-AABBs via interpolation from endpoints
            derive_sub_aabbs_from_endpoints(ep_result, robot, env_n_sub,
                                            res.sub_aabbs);
        }
        int out_n_sub = static_cast<int>(res.sub_aabbs.size()) / (n_act * 6);
        res.volume = compute_sub_aabb_volume(
            res.sub_aabbs.data(), n_act * out_n_sub);
        res.n_voxels = n_act * out_n_sub;
        res.valid = true;
        break;
    }

    // ─── SubAABB_Grid: interpolated sub-AABBs → byte grid ───────────────
    case EnvelopeType::SubAABB_Grid: {
        const int R = env_config.grid_R;
        const float* wb = env_config.world_bounds;

        // Derive sub-AABBs via interpolation from endpoints
        std::vector<float> sub_aabbs;
        if (env_n_sub <= 1) {
            sub_aabbs.resize(n_act * 6);
            extract_link_aabbs_from_endpoint(ep_result, robot,
                                             sub_aabbs.data());
        } else {
            derive_sub_aabbs_from_endpoints(ep_result, robot, env_n_sub,
                                            sub_aabbs);
        }
        int total = static_cast<int>(sub_aabbs.size()) / 6;

        // Rasterise sub-AABBs into byte grid
        float cell[3];
        for (int c = 0; c < 3; ++c)
            cell[c] = (wb[3 + c] - wb[c]) / static_cast<float>(R);

        res.grid.resize(R * R * R, 0);
        for (int k = 0; k < total; ++k) {
            const float* aabb = sub_aabbs.data() + k * 6;
            int ix0 = static_cast<int>(std::floor(
                (aabb[0] - wb[0]) / cell[0]));
            int iy0 = static_cast<int>(std::floor(
                (aabb[1] - wb[1]) / cell[1]));
            int iz0 = static_cast<int>(std::floor(
                (aabb[2] - wb[2]) / cell[2]));
            int ix1 = static_cast<int>(std::ceil(
                (aabb[3] - wb[0]) / cell[0]));
            int iy1 = static_cast<int>(std::ceil(
                (aabb[4] - wb[1]) / cell[1]));
            int iz1 = static_cast<int>(std::ceil(
                (aabb[5] - wb[2]) / cell[2]));

            ix0 = std::max(ix0, 0); ix1 = std::min(ix1, R);
            iy0 = std::max(iy0, 0); iy1 = std::min(iy1, R);
            iz0 = std::max(iz0, 0); iz1 = std::min(iz1, R);

            if (ix0 >= ix1 || iy0 >= iy1 || iz0 >= iz1) continue;

            for (int x = ix0; x < ix1; ++x)
                for (int y = iy0; y < iy1; ++y)
                    std::memset(res.grid.data() + x*R*R + y*R + iz0,
                                1, iz1 - iz0);
        }

        int occ = 0;
        for (auto v : res.grid) occ += v;
        double cell_vol = 1.0;
        for (int a = 0; a < 3; ++a)
            cell_vol *= (wb[a+3] - wb[a]) / R;
        res.volume = occ * cell_vol;
        res.n_voxels = occ;
        res.grid_R = R;
        res.sub_aabbs = std::move(sub_aabbs);
        res.valid = true;
        break;
    }

    // ─── Hull16_Grid: Conv(B₁∪B₂)⊕Ball scanline → VoxelGrid ────────────
    case EnvelopeType::Hull16_Grid: {
        double delta = env_config.delta;
        const int* alm = robot.active_link_map();
        const double* lr = robot.active_link_radii();
        const float* frames = ep_result.endpoint_aabbs.data();
        float base_pos[3] = {0.f, 0.f, 0.f};

        // ── Adaptive delta: auto-coarsen for large bounding volumes ──
        // Estimate total YZ scanline count across all links;
        // if it exceeds MAX_SCANLINES, increase delta to keep speed bounded.
        {
            double sum_yz_area = 0.0;
            for (int ci = 0; ci < n_act; ++ci) {
                int parent_fi = (alm[ci] == 0) ? -1 : alm[ci] - 1;
                int link_fi   = alm[ci];
                double r = lr ? lr[ci] : 0.0;
                double ext_y, ext_z;
                {
                    float p_lo_y = (parent_fi < 0) ? 0.f : frames[parent_fi * 6 + 1];
                    float p_hi_y = (parent_fi < 0) ? 0.f : frames[parent_fi * 6 + 4];
                    float d_lo_y = frames[link_fi * 6 + 1];
                    float d_hi_y = frames[link_fi * 6 + 4];
                    ext_y = std::max(static_cast<double>(p_hi_y),
                                     static_cast<double>(d_hi_y)) + r
                          - std::min(static_cast<double>(p_lo_y),
                                     static_cast<double>(d_lo_y)) + r;
                }
                {
                    float p_lo_z = (parent_fi < 0) ? 0.f : frames[parent_fi * 6 + 2];
                    float p_hi_z = (parent_fi < 0) ? 0.f : frames[parent_fi * 6 + 5];
                    float d_lo_z = frames[link_fi * 6 + 2];
                    float d_hi_z = frames[link_fi * 6 + 5];
                    ext_z = std::max(static_cast<double>(p_hi_z),
                                     static_cast<double>(d_hi_z)) + r
                          - std::min(static_cast<double>(p_lo_z),
                                     static_cast<double>(d_lo_z)) + r;
                }
                sum_yz_area += ext_y * ext_z;
            }
            constexpr double MAX_SCANLINES = 40000.0;
            double min_delta = std::sqrt(sum_yz_area / MAX_SCANLINES);
            if (min_delta > delta) delta = min_delta;
        }

        // safety_pad=0: link_radius already passed to fill_hull16;
        // no extra √3·δ/2 padding for volume-comparison envelopes.
        voxel::VoxelGrid grid(delta, 0, 0, 0, /*safety_pad=*/0.0);

        // n_sub is always 1 for Hull16_Grid (subdivision has no effect
        // on the convex-hull scanline — the union of sub-hulls equals
        // the single hull for linearly interpolated iAABBs).

        // Rasterise Conv(B_prox ∪ B_dist) ⊕ Ball for each link
        for (int ci = 0; ci < n_act; ++ci) {
            int parent_fi = (alm[ci] == 0) ? -1 : alm[ci] - 1;
            int link_fi   = alm[ci];
            double r = lr ? lr[ci] : 0.0;

            // Proximal endpoint iAABB (parent frame, or base_pos)
            float prox_iv[6];
            if (parent_fi < 0) {
                for (int c = 0; c < 3; ++c) {
                    prox_iv[c]     = base_pos[c];
                    prox_iv[c + 3] = base_pos[c];
                }
            } else {
                const float* f = frames + parent_fi * 6;
                for (int c = 0; c < 6; ++c) prox_iv[c] = f[c];
            }

            // Distal endpoint iAABB (link frame)
            float dist_iv[6];
            {
                const float* f = frames + link_fi * 6;
                for (int c = 0; c < 6; ++c) dist_iv[c] = f[c];
            }

            grid.fill_hull16(prox_iv, dist_iv, r);
        }

        res.hull_grid = std::move(grid);
        res.volume = res.hull_grid.occupied_volume();
        res.n_voxels = res.hull_grid.count_occupied();
        res.n_bricks = res.hull_grid.num_bricks();
        res.valid = true;
        break;
    }
    }

    return res;
}

} // namespace envelope
} // namespace sbf

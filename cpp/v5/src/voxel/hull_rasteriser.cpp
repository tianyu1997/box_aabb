// SafeBoxForest v5 — Hull Rasteriser implementation
// Migrated from v4/src/voxel/hull_rasteriser.cpp

#include <sbf/voxel/hull_rasteriser.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace sbf::voxel {

PosInterval frame_pos(const FKState& fk, int frame_idx)
{
    PosInterval p;
    p.lo[0] = static_cast<float>(fk.prefix_lo[frame_idx][3]);
    p.lo[1] = static_cast<float>(fk.prefix_lo[frame_idx][7]);
    p.lo[2] = static_cast<float>(fk.prefix_lo[frame_idx][11]);
    p.hi[0] = static_cast<float>(fk.prefix_hi[frame_idx][3]);
    p.hi[1] = static_cast<float>(fk.prefix_hi[frame_idx][7]);
    p.hi[2] = static_cast<float>(fk.prefix_hi[frame_idx][11]);
    return p;
}

void rasterise_robot_hull16(const Robot& robot,
                            const FKState& fk,
                            SparseVoxelGrid& grid,
                            int n_sub)
{
    const int     n_act = robot.n_active_links();
    const int*    alm   = robot.active_link_map();
    const double* radii = robot.active_link_radii();

    for (int i = 0; i < n_act; ++i) {
        const int li = alm[i];
        auto prox = frame_pos(fk, li);
        auto dist = frame_pos(fk, li + 1);
        const double r = radii ? radii[i] : 0.0;

        const float inv_n = 1.0f / static_cast<float>(n_sub);
        for (int s = 0; s < n_sub; ++s) {
            const float t0 = s * inv_n;
            const float t1 = (s + 1) * inv_n;

            float sp_iv[6], sd_iv[6];
            for (int c = 0; c < 3; ++c) {
                sp_iv[c]     = prox.lo[c] * (1 - t0) + dist.lo[c] * t0;
                sp_iv[c + 3] = prox.hi[c] * (1 - t0) + dist.hi[c] * t0;
                sd_iv[c]     = prox.lo[c] * (1 - t1) + dist.lo[c] * t1;
                sd_iv[c + 3] = prox.hi[c] * (1 - t1) + dist.hi[c] * t1;
            }
            grid.fill_hull16(sp_iv, sd_iv, r);
        }
    }
}

void rasterise_robot_sub_aabbs(const Robot& robot,
                               const FKState& fk,
                               SparseVoxelGrid& grid,
                               int n_sub)
{
    const int     n_act = robot.n_active_links();
    const int*    alm   = robot.active_link_map();
    const double* radii = robot.active_link_radii();

    for (int i = 0; i < n_act; ++i) {
        const int li = alm[i];
        auto prox = frame_pos(fk, li);
        auto dist = frame_pos(fk, li + 1);
        const float r = radii ? static_cast<float>(radii[i]) : 0.0f;

        const float inv_n = 1.0f / static_cast<float>(n_sub);
        for (int s = 0; s < n_sub; ++s) {
            const float t0 = s * inv_n;
            const float t1 = (s + 1) * inv_n;

            float sub_aabb[6];
            for (int c = 0; c < 3; ++c) {
                float sp_lo = prox.lo[c] * (1 - t0) + dist.lo[c] * t0;
                float sp_hi = prox.hi[c] * (1 - t0) + dist.hi[c] * t0;
                float sd_lo = prox.lo[c] * (1 - t1) + dist.lo[c] * t1;
                float sd_hi = prox.hi[c] * (1 - t1) + dist.hi[c] * t1;

                sub_aabb[c]     = std::min(sp_lo, sd_lo) - r;
                sub_aabb[c + 3] = std::max(sp_hi, sd_hi) + r;
            }
            grid.fill_aabb(sub_aabb);
        }
    }
}

void rasterise_robot_aabbs(const Robot& robot,
                           const FKState& fk,
                           SparseVoxelGrid& grid)
{
    const int n = robot.n_active_links();
    std::vector<float> aabbs(n * 6);
    extract_link_aabbs(fk,
                       robot.active_link_map(), n,
                       aabbs.data(),
                       robot.active_link_radii());
    for (int i = 0; i < n; ++i)
        grid.fill_aabb(aabbs.data() + i * 6);
}

void rasterise_box_obstacle(const Obstacle& obs, SparseVoxelGrid& grid)
{
    grid.fill_aabb(obs.bounds);
}

}  // namespace sbf::voxel

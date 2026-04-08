// SafeBoxForest v5 — Link Grid rasterization (Phase C2)
#include <sbf/envelope/link_grid.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>

namespace sbf {

VoxelGrid rasterize_link_iaabbs(
    const float* link_iaabbs,
    int n_active_links,
    const GridConfig& config)
{
    VoxelGrid grid;
    grid.delta = config.voxel_delta;

    if (n_active_links <= 0 || link_iaabbs == nullptr) {
        grid.dims = Eigen::Vector3i::Zero();
        return grid;
    }

    // 1. Compute bounding box = union of all link iAABBs
    double bb_lo[3] = { std::numeric_limits<double>::max(),
                        std::numeric_limits<double>::max(),
                        std::numeric_limits<double>::max() };
    double bb_hi[3] = { std::numeric_limits<double>::lowest(),
                        std::numeric_limits<double>::lowest(),
                        std::numeric_limits<double>::lowest() };

    for (int ci = 0; ci < n_active_links; ++ci) {
        const float* aabb = link_iaabbs + ci * 6;
        for (int d = 0; d < 3; ++d) {
            bb_lo[d] = std::min(bb_lo[d], static_cast<double>(aabb[d]));
            bb_hi[d] = std::max(bb_hi[d], static_cast<double>(aabb[d + 3]));
        }
    }

    // 2. Set grid origin and dimensions
    double inv_delta = 1.0 / config.voxel_delta;
    for (int d = 0; d < 3; ++d) {
        grid.origin[d] = bb_lo[d];
        grid.dims[d] = std::max(1, static_cast<int>(
            std::ceil((bb_hi[d] - bb_lo[d]) * inv_delta)));
    }

    grid.data.assign(grid.n_voxels(), 0);

    // 3. Rasterize each link iAABB
    for (int ci = 0; ci < n_active_links; ++ci) {
        const float* aabb = link_iaabbs + ci * 6;

        int ix0 = static_cast<int>(std::floor((aabb[0] - grid.origin[0]) * inv_delta));
        int iy0 = static_cast<int>(std::floor((aabb[1] - grid.origin[1]) * inv_delta));
        int iz0 = static_cast<int>(std::floor((aabb[2] - grid.origin[2]) * inv_delta));
        int ix1 = static_cast<int>(std::floor((aabb[3] - grid.origin[0]) * inv_delta));
        int iy1 = static_cast<int>(std::floor((aabb[4] - grid.origin[1]) * inv_delta));
        int iz1 = static_cast<int>(std::floor((aabb[5] - grid.origin[2]) * inv_delta));

        ix0 = std::max(0, ix0);
        iy0 = std::max(0, iy0);
        iz0 = std::max(0, iz0);
        ix1 = std::min(grid.dims[0] - 1, ix1);
        iy1 = std::min(grid.dims[1] - 1, iy1);
        iz1 = std::min(grid.dims[2] - 1, iz1);

        for (int iz = iz0; iz <= iz1; ++iz)
            for (int iy = iy0; iy <= iy1; ++iy)
                for (int ix = ix0; ix <= ix1; ++ix)
                    grid.set_occupied(ix, iy, iz);
    }

    return grid;
}

}  // namespace sbf

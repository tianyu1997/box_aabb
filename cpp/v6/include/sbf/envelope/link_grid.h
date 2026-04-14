#pragma once
/// @file link_grid.h
/// @brief Link IAABB grid rasterization (Phase C2).
///
/// Rasterizes link iAABBs into a simple flat voxel grid for coarse
/// volumetric collision checking.  Used as the first stage before
/// upgrading to `SparseVoxelGrid` (brick-based).

#include <Eigen/Dense>

#include <cassert>
#include <cmath>
#include <cstdint>
#include <vector>

namespace sbf {

// ─── Grid configuration ────────────────────────────────────────────────────
struct GridConfig {
    double voxel_delta = 0.05;  // voxel edge length in metres
};

// ─── Simple flat voxel grid ────────────────────────────────────────────────
struct VoxelGrid {
    Eigen::Vector3d origin = Eigen::Vector3d::Zero();  // world-space lower corner
    Eigen::Vector3i dims   = Eigen::Vector3i::Zero();  // [nx, ny, nz]
    double delta = 0.05;
    std::vector<uint8_t> data;  // occupancy (0/1), size = nx*ny*nz

    int flat_index(int ix, int iy, int iz) const {
        return ix + dims[0] * (iy + dims[1] * iz);
    }

    bool in_bounds(int ix, int iy, int iz) const {
        return ix >= 0 && ix < dims[0]
            && iy >= 0 && iy < dims[1]
            && iz >= 0 && iz < dims[2];
    }

    bool occupied(int ix, int iy, int iz) const {
        if (!in_bounds(ix, iy, iz)) return false;
        return data[flat_index(ix, iy, iz)] != 0;
    }

    void set_occupied(int ix, int iy, int iz) {
        if (!in_bounds(ix, iy, iz)) return;
        data[flat_index(ix, iy, iz)] = 1;
    }

    int n_occupied() const {
        int count = 0;
        for (uint8_t v : data) count += v;
        return count;
    }

    int n_voxels() const { return dims[0] * dims[1] * dims[2]; }
};

// ─── Rasterize link iAABBs into a voxel grid ──────────────────────────────
VoxelGrid rasterize_link_iaabbs(
    const float* link_iaabbs,   // [n_active × 6]
    int n_active_links,
    const GridConfig& config);

}  // namespace sbf

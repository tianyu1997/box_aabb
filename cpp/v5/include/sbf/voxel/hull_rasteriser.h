#pragma once
/// @file hull_rasteriser.h
/// @brief Robot envelope → SparseVoxelGrid rasterisation.
///
/// Converts robot link swept volumes (from IFK/FKState) into sparse
/// voxel grids.  Three methods of increasing tightness:
///   - `rasterise_robot_aabbs()` — per-link full AABB (fastest, loosest).
///   - `rasterise_robot_sub_aabbs()` — sub-AABB decomposition.
///   - `rasterise_robot_hull16()` — Hull-16 turbo scanline (tightest).

#include <sbf/voxel/voxel_grid.h>
#include <sbf/core/robot.h>
#include <sbf/core/fk_state.h>
#include <sbf/core/types.h>

namespace sbf::voxel {

// ─── PosInterval — (x,y,z) position interval from a prefix transform ───────
struct PosInterval {
    float lo[3];
    float hi[3];
};

/// Extract (x,y,z) position interval from columns [3],[7],[11] of prefix TF.
PosInterval frame_pos(const FKState& fk, int frame_idx);

// ─── Robot envelope rasterisation ───────────────────────────────────────────

/// Hull-16 turbo scanline: subdivide each link into n_sub segments.
void rasterise_robot_hull16(const Robot& robot,
                            const FKState& fk,
                            SparseVoxelGrid& grid,
                            int n_sub = 8);

/// Sub-AABB decomposition (faster, less tight).
void rasterise_robot_sub_aabbs(const Robot& robot,
                               const FKState& fk,
                               SparseVoxelGrid& grid,
                               int n_sub = 8);

/// Per-link full AABB (no subdivision).
void rasterise_robot_aabbs(const Robot& robot,
                           const FKState& fk,
                           SparseVoxelGrid& grid);

/// Rasterise a box obstacle into a SparseVoxelGrid.
void rasterise_box_obstacle(const Obstacle& obs, SparseVoxelGrid& grid);

}  // namespace sbf::voxel

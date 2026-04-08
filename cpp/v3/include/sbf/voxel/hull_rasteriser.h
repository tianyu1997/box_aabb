// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v3 — Sparse Voxel BitMask
//
//  hull_rasteriser.h
//
//  High-level API for rasterising a robot's link envelopes into a VoxelGrid.
//  Bridges the Robot/FKState types with the voxel module.
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/voxel/voxel_grid.h"
#include "sbf/robot/robot.h"
#include "sbf/robot/interval_fk.h"

namespace sbf {
namespace voxel {

// ─────────────────────────────────────────────────────────────────────────
//  PosInterval — (x, y, z) position interval extracted from a prefix TF
// ─────────────────────────────────────────────────────────────────────────
struct PosInterval {
    float lo[3];
    float hi[3];
};

/// Extract the (x,y,z) position interval from column [3],[7],[11] of
/// the prefix transform at `frame_idx`.
PosInterval frame_pos(const FKState& fk, int frame_idx);

// ─────────────────────────────────────────────────────────────────────────
//  Robot envelope rasterisation
// ─────────────────────────────────────────────────────────────────────────

/// Rasterise all active-link envelopes using Hull-16 turbo scanline.
///
/// For each active link, the envelope between proximal and distal frames
/// is subdivided into `n_sub` sub-segments.  Each sub-segment is rasterised
/// as Conv(B₁ ∪ B₂) ⊕ Ball(r_link) using the turbo scanline algorithm.
///
/// @param robot   Robot model
/// @param fk      Interval FK state (call compute_fk_full first)
/// @param grid    Output voxel grid (merged with existing content)
/// @param n_sub   Number of sub-divisions per link (default 8)
void rasterise_robot_hull16(const Robot& robot,
                            const FKState& fk,
                            VoxelGrid& grid,
                            int n_sub = 8);

/// Rasterise using sub-AABB decomposition (faster but less tight).
///
/// Each link is subdivided into `n_sub` sub-segments whose AABBs are
/// filled individually.  This is the conventional SBF approach.
///
/// @param robot   Robot model
/// @param fk      Interval FK state
/// @param grid    Output voxel grid
/// @param n_sub   Number of sub-divisions per link (default 8)
void rasterise_robot_sub_aabbs(const Robot& robot,
                               const FKState& fk,
                               VoxelGrid& grid,
                               int n_sub = 8);

/// Rasterise per-link AABBs (no sub-division, full AABB per link).
///
/// Uses extract_link_aabbs() then fills each AABB.  This is the coarsest
/// envelope representation.
void rasterise_robot_aabbs(const Robot& robot,
                           const FKState& fk,
                           VoxelGrid& grid);

/// Rasterise a box obstacle into a VoxelGrid.
///
/// @param cx,cy,cz  centre of box
/// @param hx,hy,hz  half-extents
/// @param grid      output voxel grid
void rasterise_box_obstacle(double cx, double cy, double cz,
                            double hx, double hy, double hz,
                            VoxelGrid& grid);

} // namespace voxel
} // namespace sbf

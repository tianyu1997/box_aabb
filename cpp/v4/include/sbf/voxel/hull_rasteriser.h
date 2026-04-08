// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Sparse Voxel BitMask
//
//  hull_rasteriser.h
//
//  High-level API for rasterising a robot's link envelopes into a VoxelGrid.
//  Bridges the Robot/FKState types with the voxel module.
//
//  迁移自 v3 hull_rasteriser.h
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
void rasterise_robot_hull16(const Robot& robot,
                            const FKState& fk,
                            VoxelGrid& grid,
                            int n_sub = 8);

/// Rasterise using sub-AABB decomposition (faster but less tight).
void rasterise_robot_sub_aabbs(const Robot& robot,
                               const FKState& fk,
                               VoxelGrid& grid,
                               int n_sub = 8);

/// Rasterise per-link AABBs (no sub-division, full AABB per link).
void rasterise_robot_aabbs(const Robot& robot,
                           const FKState& fk,
                           VoxelGrid& grid);

/// Rasterise a box obstacle into a VoxelGrid.
void rasterise_box_obstacle(double cx, double cy, double cz,
                            double hx, double hy, double hz,
                            VoxelGrid& grid);

} // namespace voxel
} // namespace sbf

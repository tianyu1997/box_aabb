// SafeBoxForest v2 — GridEnvelope: occupancy grid envelope
// Module: sbf::envelope
//
// Represents the workspace occupancy of a manipulator over a C-space box
// as an R³ voxel grid.  Key advantage: union of grids is a simple bitwise
// OR that does NOT inflate bounds (unlike AABB/OBB unions that accumulate
// wrapping-effect conservatism).
//
// Compression:
//   The raw grid is typically 2-5% occupied (robot = thin arc-shaped sweeps).
//   Z-slice RLE achieves ~10-20× compression for sidecar persistence.
//
// Collision test:
//   Voxelise each obstacle AABB into the grid coordinate system, then
//   check if any voxel in the obstacle region is occupied: O(obstacle volume).
//   obs_compact format: [lo_x, hi_x, lo_y, hi_y, lo_z, hi_z] (interleaved).
//
#pragma once

#include "sbf/envelope/envelope.h"
#include "sbf/robot/robot.h"

#include <cstdint>
#include <vector>

namespace sbf {
namespace envelope {

// ═════════════════════════════════════════════════════════════════════════════
//  GridEnvelope — concrete IEnvelope backed by R³ voxel grid
// ═════════════════════════════════════════════════════════════════════════════
class GridEnvelope : public IEnvelope {
public:
    GridEnvelope() = default;

    // Construct from raw grid data.
    // grid: R*R*R bytes, [x*R*R + y*R + z], 1 = occupied.
    GridEnvelope(std::vector<uint8_t> grid, int R,
                 float world_bounds[6]);

    // ── IEnvelope ───────────────────────────────────────────────────────
    EnvelopeKind kind()        const override { return EnvelopeKind::Grid; }
    bool   collides(const float* obs_compact, int n_obs) const override;
    double volume()            const override;
    int    n_primitives()      const override;
    bool   valid()             const override { return valid_; }

    std::vector<uint8_t> serialise()  const override;
    static EnvelopePtr   deserialise(const uint8_t* data, size_t len);

    // ── Grid-specific accessors ─────────────────────────────────────────
    int R()                     const { return R_; }
    const uint8_t* grid_data()  const { return grid_.data(); }
    const float* world_bounds() const { return world_bounds_; }
    float cell_size(int axis)   const { return cell_[axis]; }
    int   occupied_count()      const;

    // ── Grid operations ─────────────────────────────────────────────────

    // Union: this |= other.  Requires same R and world_bounds.
    void merge_or(const GridEnvelope& other);

private:
    std::vector<uint8_t> grid_;
    int R_    = 0;
    bool valid_ = false;
    float world_bounds_[6] = {};
    float cell_[3] = {};
};

// ═════════════════════════════════════════════════════════════════════════════
//  GridComputer — IEnvelopeComputer that produces GridEnvelope
// ═════════════════════════════════════════════════════════════════════════════
class GridComputer : public IEnvelopeComputer {
public:
    GridComputer() = default;
    GridComputer(const Robot& robot, int R, const float world_bounds[6],
                 int n_sub_per_link = 8);

    EnvelopeKind kind() const override { return EnvelopeKind::Grid; }
    std::string  name() const override { return "grid_sub_aabb"; }

    EnvelopePtr compute(
        const std::vector<Interval>& intervals) const override;

    int n_total_slots() const override { return R_ * R_ * R_; }

private:
    const Robot* robot_ = nullptr;
    int R_ = 32;
    float world_bounds_[6] = {};
    int n_sub_ = 8;
};

// ═════════════════════════════════════════════════════════════════════════════
//  Grid collision helpers
// ═════════════════════════════════════════════════════════════════════════════

// Test if any occupied voxel overlaps with any obstacle AABB.
bool grid_obs_collide(const uint8_t* grid, int R,
                      const float* world_bounds,
                      const float* obs_compact, int n_obs);

// ═════════════════════════════════════════════════════════════════════════════
//  RLE compression / decompression (Z-slice Run-Length Encoding)
// ═════════════════════════════════════════════════════════════════════════════
//
// Format: For each Z-slice z ∈ [0, R):
//   [n_runs: uint16]
//   For each run: [start: uint16] [length: uint16]
//
// Runs encode occupied voxel ranges in row-major (x*R+y) order within
// each Z-slice.  Empty slices are encoded as n_runs=0 (2 bytes).
//
std::vector<uint8_t> grid_compress_rle(const uint8_t* grid, int R);
void grid_decompress_rle(const uint8_t* data, size_t len, int R,
                         uint8_t* out_grid);

} // namespace envelope
} // namespace sbf

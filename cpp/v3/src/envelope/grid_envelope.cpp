// SafeBoxForest v2 — GridEnvelope implementation
#include "sbf/envelope/grid_envelope.h"
#include "sbf/envelope/envelope_derive.h"
#include "sbf/envelope/frame_store.h"
#include "sbf/robot/interval_fk.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>

namespace sbf {
namespace envelope {

// ═════════════════════════════════════════════════════════════════════════════
//  GridEnvelope
// ═════════════════════════════════════════════════════════════════════════════

GridEnvelope::GridEnvelope(std::vector<uint8_t> grid, int R,
                           float world_bounds[6])
    : grid_(std::move(grid)), R_(R), valid_(true)
{
    std::memcpy(world_bounds_, world_bounds, 6 * sizeof(float));
    for (int c = 0; c < 3; ++c)
        cell_[c] = (world_bounds_[3 + c] - world_bounds_[c]) /
                   static_cast<float>(R_);
}

bool GridEnvelope::collides(const float* obs_compact, int n_obs) const {
    return grid_obs_collide(grid_.data(), R_, world_bounds_,
                            obs_compact, n_obs);
}

double GridEnvelope::volume() const {
    int occ = occupied_count();
    double cell_vol = static_cast<double>(cell_[0]) *
                      static_cast<double>(cell_[1]) *
                      static_cast<double>(cell_[2]);
    return occ * cell_vol;
}

int GridEnvelope::n_primitives() const {
    return occupied_count();
}

int GridEnvelope::occupied_count() const {
    int count = 0;
    int total = R_ * R_ * R_;
    for (int i = 0; i < total; ++i)
        count += grid_[i];
    return count;
}

std::vector<uint8_t> GridEnvelope::serialise() const {
    // Header: [kind(1) | R(4) | world_bounds(24)] + RLE payload
    std::vector<uint8_t> rle = grid_compress_rle(grid_.data(), R_);

    std::vector<uint8_t> out;
    out.reserve(1 + 4 + 24 + rle.size());

    out.push_back(static_cast<uint8_t>(EnvelopeKind::Grid));

    // R as 4 bytes LE
    auto push_i32 = [&](int32_t v) {
        for (int b = 0; b < 4; ++b)
            out.push_back(static_cast<uint8_t>((v >> (8 * b)) & 0xFF));
    };
    push_i32(R_);

    // world_bounds as 6 × float32
    const uint8_t* wb = reinterpret_cast<const uint8_t*>(world_bounds_);
    out.insert(out.end(), wb, wb + 24);

    out.insert(out.end(), rle.begin(), rle.end());
    return out;
}

EnvelopePtr GridEnvelope::deserialise(const uint8_t* data, size_t len) {
    if (len < 1 + 4 + 24) return nullptr;

    size_t off = 0;
    // kind tag
    if (data[off] != static_cast<uint8_t>(EnvelopeKind::Grid))
        return nullptr;
    off += 1;

    // R
    int32_t R = 0;
    for (int b = 0; b < 4; ++b)
        R |= static_cast<int32_t>(data[off + b]) << (8 * b);
    off += 4;

    // world_bounds
    float wb[6];
    std::memcpy(wb, data + off, 24);
    off += 24;

    // RLE payload
    std::vector<uint8_t> grid(R * R * R, 0);
    grid_decompress_rle(data + off, len - off, R, grid.data());

    return std::make_shared<GridEnvelope>(std::move(grid), R, wb);
}

void GridEnvelope::merge_or(const GridEnvelope& other) {
    assert(R_ == other.R_);
    int total = R_ * R_ * R_;
    for (int i = 0; i < total; ++i)
        grid_[i] |= other.grid_[i];
}

// ═════════════════════════════════════════════════════════════════════════════
//  GridComputer
// ═════════════════════════════════════════════════════════════════════════════

GridComputer::GridComputer(const Robot& robot, int R,
                           const float world_bounds[6],
                           int n_sub_per_link)
    : robot_(&robot), R_(R), n_sub_(n_sub_per_link)
{
    std::memcpy(world_bounds_, world_bounds, 6 * sizeof(float));
}

EnvelopePtr GridComputer::compute(
    const std::vector<Interval>& intervals) const
{
    // 1. Interval FK → FKState
    FKState fk = compute_fk_full(*robot_, intervals);
    if (!fk.valid) return nullptr;

    // 2. Extract frame position intervals via FrameStore helper
    int n_active = robot_->n_active_links();
    const int* link_map = robot_->active_link_map();

    // Determine n_stored_frames (max frame_idx + 1)
    int n_frames = 0;
    for (int i = 0; i < n_active; ++i)
        n_frames = std::max(n_frames, link_map[i] + 1);

    // Extract frame positions from FKState
    std::vector<float> frames(n_frames * 6);
    for (int f = 0; f < n_frames; ++f) {
        int k = f + 1;  // prefix_lo/hi[k+1] but our convention: frame f = prefix[f+1]
        if (k >= fk.n_tf) k = fk.n_tf - 1;  // safety clamp
        frames[f * 6 + 0] = static_cast<float>(fk.prefix_lo[k][3]);
        frames[f * 6 + 1] = static_cast<float>(fk.prefix_lo[k][7]);
        frames[f * 6 + 2] = static_cast<float>(fk.prefix_lo[k][11]);
        frames[f * 6 + 3] = static_cast<float>(fk.prefix_hi[k][3]);
        frames[f * 6 + 4] = static_cast<float>(fk.prefix_hi[k][7]);
        frames[f * 6 + 5] = static_cast<float>(fk.prefix_hi[k][11]);
    }

    // Link radii (convert double → float)
    std::vector<float> radii(n_active, 0.f);
    const double* dr = robot_->active_link_radii();
    if (dr) {
        for (int i = 0; i < n_active; ++i)
            radii[i] = static_cast<float>(dr[i]);
    }

    // Base position = [0, 0, 0] (or more generally prefix[0] translation)
    float base_pos[3] = {
        static_cast<float>(fk.prefix_lo[0][3]),
        static_cast<float>(fk.prefix_lo[0][7]),
        static_cast<float>(fk.prefix_lo[0][11])
    };

    // 3. Derive grid
    int grid_sz = R_ * R_ * R_;
    std::vector<uint8_t> grid(grid_sz, 0);

    derive_grid(frames.data(), n_frames, link_map, n_active,
                radii.data(), base_pos,
                world_bounds_, R_,
                n_sub_, /*adaptive_max_sub=*/32,
                grid.data());

    // 4. Build GridEnvelope
    float wb[6];
    std::memcpy(wb, world_bounds_, sizeof(wb));
    return std::make_shared<GridEnvelope>(std::move(grid), R_, wb);
}

// ═════════════════════════════════════════════════════════════════════════════
//  grid_obs_collide
// ═════════════════════════════════════════════════════════════════════════════

bool grid_obs_collide(const uint8_t* grid, int R,
                      const float* world_bounds,
                      const float* obs_compact, int n_obs)
{
    float inv_cell[3];
    for (int c = 0; c < 3; ++c)
        inv_cell[c] = static_cast<float>(R) /
                      (world_bounds[3 + c] - world_bounds[c]);

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
//  RLE compression / decompression
// ═════════════════════════════════════════════════════════════════════════════
//
// Format: For each Z-slice z ∈ [0, R):
//   [n_runs: uint16]
//   For each run: [start: uint16] [length: uint16]
//
// "start" and "length" refer to row-major index (x*R + y) within the slice.

std::vector<uint8_t> grid_compress_rle(const uint8_t* grid, int R) {
    std::vector<uint8_t> out;
    out.reserve(R * 2);  // rough estimate for mostly-empty grids

    auto push_u16 = [&](uint16_t v) {
        out.push_back(static_cast<uint8_t>(v & 0xFF));
        out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    };

    int slice_sz = R * R;

    for (int z = 0; z < R; ++z) {
        // Count runs first
        size_t n_runs_pos = out.size();
        push_u16(0);  // placeholder for n_runs

        uint16_t n_runs = 0;
        int idx = 0;
        while (idx < slice_sz) {
            // Skip zeros
            if (grid[idx * R + z] == 0) {
                // Wait, indexing is [x*R*R + y*R + z]
                // For slice z, iterate x ∈ [0,R), y ∈ [0,R)
                // Row-major within slice: pos = x*R + y
                // grid index = x*R*R + y*R + z
                // Let's fix the iteration
            }
            break;
        }

        // Re-do with correct indexing
        out.resize(n_runs_pos);  // rewind
        push_u16(0);             // placeholder again

        n_runs = 0;
        bool in_run = false;
        uint16_t run_start = 0;
        uint16_t run_len = 0;

        for (int pos = 0; pos < slice_sz; ++pos) {
            int x = pos / R;
            int y = pos % R;
            bool occ = (grid[x * R * R + y * R + z] != 0);

            if (occ) {
                if (!in_run) {
                    in_run = true;
                    run_start = static_cast<uint16_t>(pos);
                    run_len = 1;
                } else {
                    run_len++;
                }
            } else {
                if (in_run) {
                    push_u16(run_start);
                    push_u16(run_len);
                    n_runs++;
                    in_run = false;
                }
            }
        }
        // Flush last run
        if (in_run) {
            push_u16(run_start);
            push_u16(run_len);
            n_runs++;
        }

        // Patch n_runs
        out[n_runs_pos]     = static_cast<uint8_t>(n_runs & 0xFF);
        out[n_runs_pos + 1] = static_cast<uint8_t>((n_runs >> 8) & 0xFF);
    }

    return out;
}

void grid_decompress_rle(const uint8_t* data, size_t len, int R,
                         uint8_t* out_grid)
{
    size_t off = 0;

    auto read_u16 = [&]() -> uint16_t {
        if (off + 2 > len) return 0;
        uint16_t v = static_cast<uint16_t>(data[off]) |
                     (static_cast<uint16_t>(data[off + 1]) << 8);
        off += 2;
        return v;
    };

    for (int z = 0; z < R; ++z) {
        uint16_t n_runs = read_u16();
        for (uint16_t r = 0; r < n_runs; ++r) {
            uint16_t start  = read_u16();
            uint16_t length = read_u16();
            for (uint16_t pos = start; pos < start + length && pos < R * R; ++pos) {
                int x = pos / R;
                int y = pos % R;
                out_grid[x * R * R + y * R + z] = 1;
            }
        }
    }
}

} // namespace envelope
} // namespace sbf

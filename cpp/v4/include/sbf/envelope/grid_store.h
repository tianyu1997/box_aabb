// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — GridStore: per-node bitfield occupancy grid storage
//  Module: sbf::envelope
//
//  Stores per-node R³ occupancy grids as packed uint64_t bitfields for
//  zero-inflation union (bitwise OR) in the FFB tree.
//
//  Grid resolution R = 32:
//    32³ = 32768 voxels → 512 × uint64_t = 4096 bytes per node
//
//  Key operations:
//    derive_from_aabbs()  — rasterise AABBs → bitfield grid
//    union_grids()        — dst = a | b  (512 × OR, zero inflation)
//    check_collision()    — Obstacle → voxel range → bit probe
//
//  Two-level collision check (AABB quick-reject + Grid precise):
//    1. AABB per-link SAT: O(n_links × n_obs), ~20ns
//       → pass (no collision) → accept immediately
//    2. Grid bit probe:     O(obstacle voxel volume), ~50-100ns
//       → called only when AABB reports collision (refinement)
//
//  v4 optimisations vs v3:
//    • Word-level z-mask: replace inner z-loop with single uint64_t AND/OR.
//      For R=32, each (x,y) row has 32 z-bits in one word half — the
//      z-range [iz0,iz1) is tested/set with one bitmask operation instead
//      of up to 32 individual bit probes.
//    • Obstacle* API: uses Obstacle.lo()/hi() directly instead of
//      interleaved obs_compact float array.
//    • Decoupled robot metadata: constructor takes only world_bounds.
//      derive_from_aabbs() accepts pre-computed AABBs (link iAABBs or
//      subdivided sub-AABBs), caller handles FK / subdivision.
//    • Dropped v1 (RLE) format: v4 uses GRD3 (simplified flat header).
//    • Removed grid_envelope.h dependency (no IEnvelope OOP, no RLE).
//
//  Persistence — GRD3 (mmap-friendly fixed stride):
//    Header (512B) + n_alloc × RECORD_SIZE bytes
//    Each record: [valid:1B][pad:7B][bitfield:4096B] = 4104 bytes
//
//  迁移自 v3 grid_store.h
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/core/types.h"
#include "sbf/core/mmap_util.h"

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace sbf {
namespace envelope {

// ─── Constants ──────────────────────────────────────────────────────────────
constexpr int GRID_R          = 32;                         // grid resolution per axis
constexpr int GRID_TOTAL      = GRID_R * GRID_R * GRID_R;  // 32768 voxels
constexpr int WORDS_PER_NODE  = GRID_TOTAL / 64;           // 512 uint64_t
constexpr int BYTES_PER_NODE  = WORDS_PER_NODE * 8;        // 4096 bytes

// ─── Bit-indexing helpers (R=32 specialised) ────────────────────────────────
//
// Layout: linear(x,y,z) = x*1024 + y*32 + z
//   word  = x*16 + y/2
//   bit_base = (y & 1) * 32
//   z occupies bit_base+0 .. bit_base+31 within the word.
//
// This means all 32 z-values for a given (x,y) are in ONE uint64_t,
// enabling word-level bitmask operations instead of per-voxel loops.

inline int grid_word(int x, int y)      { return x * 16 + (y >> 1); }
inline int grid_bit_base(int y)         { return (y & 1) * 32; }

/// Build a bitmask covering z-range [iz0, iz1) shifted to the correct half.
/// Pre: 0 <= iz0 <= iz1 <= GRID_R, returns 0 when range is empty.
inline uint64_t z_mask(int iz0, int iz1, int bit_base) {
    int n = iz1 - iz0;
    if (n <= 0) return 0;
    // ((1 << n) - 1) << (bit_base + iz0)
    return ((uint64_t(1) << n) - 1) << (bit_base + iz0);
}

// ═════════════════════════════════════════════════════════════════════════════
//  GridStore — per-node bitfield occupancy grid storage
// ═════════════════════════════════════════════════════════════════════════════
class GridStore {
public:
    GridStore() = default;

    /// Construct with world_bounds [x_lo, y_lo, z_lo, x_hi, y_hi, z_hi].
    explicit GridStore(const float world_bounds[6], int initial_capacity = 1024);

    ~GridStore();

    // Non-copyable
    GridStore(const GridStore&)            = delete;
    GridStore& operator=(const GridStore&) = delete;

    // Move semantics (transfers mmap ownership)
    GridStore(GridStore&& o) noexcept;
    GridStore& operator=(GridStore&& o) noexcept;

    // ── Metadata ─────────────────────────────────────────────────────────
    int capacity() const { return capacity_; }

    const float* world_bounds() const { return world_bounds_; }
    float cell_size(int axis) const   { return cell_[axis]; }

    // ── Derive — rasterise AABBs into bitfield grid ─────────────────────

    /// Rasterise N axis-aligned bounding boxes into the bitfield for node_idx.
    /// aabbs: [n_aabbs × 6] in [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z].
    /// Typically called with pre-computed link iAABBs or subdivided sub-AABBs.
    /// Uses word-level z-mask OR (v4 optimisation).
    void derive_from_aabbs(int node_idx, const float* aabbs, int n_aabbs);

    // ── Grid union ───────────────────────────────────────────────────────

    /// dst = a | b  (512 × uint64_t OR — zero inflation union)
    void union_grids(int dst_idx, int a_idx, int b_idx);

    // ── Collision checking ───────────────────────────────────────────────

    /// Check if any occupied voxel in node_idx's grid overlaps with any
    /// obstacle AABB.  Uses word-level z-mask AND (v4 optimisation).
    bool check_collision(int node_idx,
                         const Obstacle* obstacles, int n_obs) const;

    // ── Access ───────────────────────────────────────────────────────────

    bool has_grid(int node_idx) const {
        if (node_idx < 0 || node_idx >= capacity_) return false;
        return get_valid(node_idx);
    }

    /// Raw bitfield pointer (512 × uint64_t = 4096 bytes).
    const uint64_t* get_grid(int node_idx) const {
        if (mmap_.is_open())
            return reinterpret_cast<const uint64_t*>(
                mmap_.ptr + MMAP_HEADER_SIZE +
                static_cast<size_t>(node_idx) * MMAP_RECORD_SIZE + 8);
        return data_.data() + static_cast<size_t>(node_idx) * WORDS_PER_NODE;
    }

    uint64_t* get_grid_mut(int node_idx) {
        if (mmap_.is_open())
            return reinterpret_cast<uint64_t*>(
                mmap_.ptr + MMAP_HEADER_SIZE +
                static_cast<size_t>(node_idx) * MMAP_RECORD_SIZE + 8);
        return data_.data() + static_cast<size_t>(node_idx) * WORDS_PER_NODE;
    }

    /// Mark/unmark validity
    void set_has_grid(int node_idx, bool val);

    /// Count occupied voxels for a node
    int occupied_count(int node_idx) const;

    // ── Persistence ──────────────────────────────────────────────────────

    /// Save all valid nodes (GRD3 flat format). In mmap mode: flushes.
    void save(const std::string& path);

    /// Incremental: mmap → flush; non-mmap → write dirty nodes in-place.
    void save_incremental();
    void save_incremental(const std::string& path);

    /// Load from .grids file (GRD3 format only).
    void load(const std::string& path);

    // ── Mmap persistence (lazy load) ─────────────────────────────────────

    /// Create a new mmap-backed GRD3 file.
    void create_mmap(const std::string& path, int initial_capacity);

    /// Open existing GRD3 file via mmap (lazy load on demand).
    void load_mmap(const std::string& path);

    /// Sync dirty pages to disk.
    void flush_mmap();

    /// Close mmap (flushes first).
    void close_mmap();

    bool is_mmap() const { return mmap_.is_open(); }

    int n_dirty() const { return n_dirty_; }
    int n_valid() const;
    const std::string& save_path() const { return save_path_; }

    // ── Capacity management ──────────────────────────────────────────────

    void ensure_capacity(int n_nodes);

private:
    int capacity_ = 0;

    float world_bounds_[6] = {};
    float cell_[3]         = {};     // cell size per axis
    float inv_cell_[3]     = {};     // 1/cell per axis

    // ── Vector-backed storage (non-mmap mode) ────────────────────────────
    std::vector<uint64_t> data_;    // [capacity × WORDS_PER_NODE]
    std::vector<uint8_t>  valid_;   // [capacity]
    std::vector<uint8_t>  dirty_;   // [capacity]
    int n_dirty_ = 0;

    // ── Mmap state ───────────────────────────────────────────────────────
    mmap_util::MmapHandle mmap_;

    // ── Persistence state ────────────────────────────────────────────────
    std::string save_path_;

    void mark_dirty(int node_idx);

    // ── Valid flag access (dual-mode) ────────────────────────────────────
    bool get_valid(int node_idx) const {
        if (mmap_.is_open())
            return *(mmap_.ptr + MMAP_HEADER_SIZE +
                     static_cast<size_t>(node_idx) * MMAP_RECORD_SIZE) != 0;
        return valid_[node_idx] != 0;
    }
    void set_valid(int node_idx, bool v) {
        if (mmap_.is_open())
            *(mmap_.ptr + MMAP_HEADER_SIZE +
              static_cast<size_t>(node_idx) * MMAP_RECORD_SIZE) =
                static_cast<char>(v ? 1 : 0);
        else
            valid_[node_idx] = v ? 1 : 0;
    }

    // ── Mmap helpers ─────────────────────────────────────────────────────
    void grow_mmap(int new_cap);
    void write_mmap_header();
    void read_mmap_header();
    void recompute_cells();

    // ── Binary format constants ──────────────────────────────────────────

    // GRD3 (v4 — simplified header, no robot metadata)
    static constexpr uint32_t MAGIC_V3   = 0x33445247;  // "GRD3" LE
    static constexpr uint32_t VERSION_V3 = 3;

    static constexpr int MMAP_HEADER_SIZE = 512;
    static constexpr int MMAP_RECORD_SIZE = BYTES_PER_NODE + 8;  // 4104
};

} // namespace envelope
} // namespace sbf

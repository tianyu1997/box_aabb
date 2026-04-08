// SafeBoxForest v2 — GridStore: per-node bitfield occupancy grid storage
// Module: sbf::envelope
//
// Stores per-node R³ occupancy grids as packed uint64_t bitfields for
// zero-inflation union (bitwise OR) in the FFB tree.
//
// Grid resolution R = 32:
//   32³ = 32768 voxels → 512 × uint64_t = 4096 bytes per node
//
// Key operations:
//   derive_from_frames()  — rasterise frame intervals → bitfield grid
//   union_grids()         — dst = a | b  (512 × OR, zero inflation)
//   check_collision()     — obstacle AABB → voxel range → bit probe
//
// Two-level collision check (AABB quick-reject + Grid precise):
//   1. AABB per-link SAT: O(n_links × n_obs), ~20ns
//      → pass (no collision) → accept immediately
//   2. Grid bit probe:     O(obstacle voxel volume), ~50-100ns
//      → called only when AABB reports collision (refinement)
//
// Persistence:
//   Two on-disk formats:
//
//   GRD1 (legacy) — variable-length RLE compressed records:
//     Header (512B) + [node_idx, payload_len, RLE_data] per valid node
//     Supported by load() only (full read, not mmap-friendly).
//
//   GRD2 (mmap-friendly) — fixed-stride flat indexed:
//     Header (512B) + n_alloc × RECORD_SIZE bytes
//     Each record: [valid:1B][pad:7B][bitfield:4096B] = 4104 bytes
//     load_mmap() maps the file; OS loads pages lazily on demand.
//     save() / save_incremental() always write v2 format.
//
#pragma once

#include "sbf/common/types.h"
#include "sbf/common/mmap_util.h"

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

// ═════════════════════════════════════════════════════════════════════════════
//  GridStore — per-node bitfield occupancy grid storage
// ═════════════════════════════════════════════════════════════════════════════
class GridStore {
public:
    GridStore() = default;

    // Construct with robot metadata for derive operations.
    // world_bounds: [x_lo, y_lo, z_lo, x_hi, y_hi, z_hi]
    GridStore(int n_frames, int n_active_links,
              const int* active_link_map,
              const float* link_radii,
              const float* base_pos,
              const float world_bounds[6],
              int initial_capacity = 1024);

    // Destructor (closes mmap if open)
    ~GridStore();

    // Non-copyable
    GridStore(const GridStore&)            = delete;
    GridStore& operator=(const GridStore&) = delete;

    // Move semantics (transfers mmap ownership)
    GridStore(GridStore&& o) noexcept;
    GridStore& operator=(GridStore&& o) noexcept;

    // ── Metadata ─────────────────────────────────────────────────────────
    int capacity()       const { return capacity_; }
    int n_active_links() const { return n_active_links_; }
    int n_frames()       const { return n_frames_; }

    // ── Derive from frames ───────────────────────────────────────────────

    // Rasterise frame position intervals into a bitfield grid for node_idx.
    // Uses sub-AABB voxelisation (same algorithm as derive_grid but bitfield).
    // frames: [n_frames × 6] float array from FrameStore.
    // n_sub_per_link: subdivision count (0 = adaptive).
    void derive_from_frames(int node_idx, const float* frames,
                            int n_sub_per_link = 8);

    // ── Grid union ───────────────────────────────────────────────────────

    // dst = a | b  (512 × uint64_t OR — zero inflation union)
    void union_grids(int dst_idx, int a_idx, int b_idx);

    // ── Collision checking ───────────────────────────────────────────────

    // Check if any occupied voxel in node_idx's grid overlaps with any
    // obstacle AABB.  obs_compact format: [lo_x,hi_x, lo_y,hi_y, lo_z,hi_z]
    // per obstacle.
    bool check_collision(int node_idx,
                         const float* obs_compact, int n_obs) const;

    // ── Access ───────────────────────────────────────────────────────────

    bool has_grid(int node_idx) const {
        if (node_idx < 0 || node_idx >= capacity_) return false;
        return get_valid(node_idx);
    }

    // Raw bitfield pointer (512 × uint64_t = 4096 bytes).
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

    // Mark/unmark validity
    void set_has_grid(int node_idx, bool val);

    // Count occupied voxels for a node
    int occupied_count(int node_idx) const;

    // ── Persistence ──────────────────────────────────────────────────────

    // Save all valid nodes (v2 flat format). In mmap mode: flushes.
    void save(const std::string& path);

    // Incremental: mmap → flush; non-mmap → write dirty nodes in-place.
    void save_incremental();
    void save_incremental(const std::string& path);

    // Load from .grids file. Auto-detects v1 (RLE) or v2 (flat).
    // Full read into memory (non-mmap).
    void load(const std::string& path);

    // ── Mmap persistence (lazy load) ─────────────────────────────────────

    // Create a new mmap-backed v2 file.  All future writes go to mapping.
    void create_mmap(const std::string& path, int initial_capacity);

    // Open existing v2 file via mmap (lazy load on demand).
    void load_mmap(const std::string& path);

    // Sync dirty pages to disk.
    void flush_mmap();

    // Close mmap (flushes first).
    void close_mmap();

    bool is_mmap() const { return mmap_.is_open(); }

    int n_dirty() const { return n_dirty_; }
    int n_valid() const;
    const std::string& save_path() const { return save_path_; }

    // ── Capacity management ──────────────────────────────────────────────

    void ensure_capacity(int n_nodes);

private:
    int n_frames_       = 0;
    int n_active_links_ = 0;
    int capacity_       = 0;

    int   active_link_map_[MAX_LINKS] = {};
    float link_radii_[MAX_LINKS]      = {};
    float base_pos_[3]                = {0.f, 0.f, 0.f};
    float world_bounds_[6]            = {};
    float cell_[3]                    = {};     // cell size per axis
    float inv_cell_[3]                = {};     // 1/cell per axis

    // ── Vector-backed storage (non-mmap mode) ────────────────────────────
    std::vector<uint64_t> data_;    // [capacity × WORDS_PER_NODE]
    std::vector<uint8_t>  valid_;   // [capacity]
    std::vector<uint8_t>  dirty_;   // [capacity]
    int n_dirty_ = 0;

    // ── Mmap state ───────────────────────────────────────────────────────
    mmap_util::MmapHandle mmap_;

    // ── Common persistence state ─────────────────────────────────────────
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

    // ── Bit indexing helpers ─────────────────────────────────────────────
    static int voxel_linear(int x, int y, int z) {
        return x * GRID_R * GRID_R + y * GRID_R + z;
    }
    static int word_index(int linear) { return linear >> 6; }
    static int bit_index(int linear)  { return linear & 63; }

    // ── Binary format constants ──────────────────────────────────────────

    // Legacy v1 (RLE compressed)
    static constexpr uint32_t MAGIC_V1   = 0x31445247;  // "GRD1" LE
    static constexpr uint32_t VERSION_V1 = 1;

    // v2 (mmap-friendly fixed stride)
    static constexpr uint32_t MAGIC_V2   = 0x32445247;  // "GRD2" LE
    static constexpr uint32_t VERSION_V2 = 2;

    static constexpr int MMAP_HEADER_SIZE = 512;
    static constexpr int MMAP_RECORD_SIZE = BYTES_PER_NODE + 8;  // 4104

    // Legacy aliases for load() v1 path
    static constexpr uint32_t MAGIC   = MAGIC_V1;
    static constexpr uint32_t VERSION = VERSION_V1;
    static constexpr int HEADER_SIZE  = 512;
};

} // namespace envelope
} // namespace sbf

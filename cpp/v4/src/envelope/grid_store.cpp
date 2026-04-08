// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — GridStore implementation
//
//  Per-node bitfield (uint64_t[512]) occupancy grid — zero-inflation union.
//
//  v4 optimisations vs v3:
//    • derive_from_aabbs: word-level z-mask OR replaces per-voxel bit set
//    • check_collision:   word-level z-mask AND replaces per-voxel bit test
//    • Obstacle* API:     .lo()/.hi() instead of interleaved obs_compact
//    • No robot metadata: world_bounds only, caller provides AABBs
//    • GRD3 format:       simplified header (no active_link_map/radii/base_pos)
//
//  迁移自 v3 grid_store.cpp
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/envelope/grid_store.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>

#ifdef _MSC_VER
#include <intrin.h>   // __popcnt64
#pragma intrinsic(__popcnt)
#if defined(_M_X64) || defined(_M_ARM64)
#pragma intrinsic(__popcnt64)
#endif
#endif

namespace sbf {
namespace envelope {

// ═════════════════════════════════════════════════════════════════════════════
//  Construction
// ═════════════════════════════════════════════════════════════════════════════

GridStore::GridStore(const float world_bounds[6], int initial_capacity)
    : capacity_(0)
{
    std::memcpy(world_bounds_, world_bounds, 6 * sizeof(float));
    recompute_cells();
    ensure_capacity(initial_capacity);
}

GridStore::~GridStore() {
    if (mmap_.is_open()) {
        try { close_mmap(); } catch (...) {}
    }
}

GridStore::GridStore(GridStore&& o) noexcept
    : capacity_(o.capacity_)
    , data_(std::move(o.data_))
    , valid_(std::move(o.valid_))
    , dirty_(std::move(o.dirty_))
    , n_dirty_(o.n_dirty_)
    , mmap_(std::move(o.mmap_))
    , save_path_(std::move(o.save_path_))
{
    std::memcpy(world_bounds_, o.world_bounds_, sizeof(world_bounds_));
    std::memcpy(cell_, o.cell_, sizeof(cell_));
    std::memcpy(inv_cell_, o.inv_cell_, sizeof(inv_cell_));
    o.capacity_ = 0;
    o.n_dirty_ = 0;
}

GridStore& GridStore::operator=(GridStore&& o) noexcept {
    if (this == &o) return *this;
    if (mmap_.is_open()) {
        try { close_mmap(); } catch (...) {}
    }
    capacity_ = o.capacity_;
    data_ = std::move(o.data_);
    valid_ = std::move(o.valid_);
    dirty_ = std::move(o.dirty_);
    n_dirty_ = o.n_dirty_;
    mmap_ = std::move(o.mmap_);
    save_path_ = std::move(o.save_path_);
    std::memcpy(world_bounds_, o.world_bounds_, sizeof(world_bounds_));
    std::memcpy(cell_, o.cell_, sizeof(cell_));
    std::memcpy(inv_cell_, o.inv_cell_, sizeof(inv_cell_));
    o.capacity_ = 0;
    o.n_dirty_ = 0;
    return *this;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Derive from AABBs — rasterise bounding boxes into bitfield grid
// ═════════════════════════════════════════════════════════════════════════════
//
//  v4 optimisation: word-level z-mask OR.
//
//  For R=32, each (x,y) row maps to exactly one uint64_t word, with the
//  32 z-values packed into one half (bits 0..31 or 32..63).  Instead of
//  setting bits one-by-one in the inner z-loop, we compute a bitmask
//  covering the z-range and OR it in one instruction.
//
//  Example: z-range [3, 7) in even-y row →
//    mask = ((1<<4)-1) << 3 = 0b01111000
//    grid[word] |= mask
//
//  This replaces up to 32 bit-set operations per (x,y) cell with ONE OR.

void GridStore::derive_from_aabbs(int node_idx,
                                  const float* aabbs, int n_aabbs)
{
    ensure_capacity(node_idx + 1);

    uint64_t* grid = get_grid_mut(node_idx);
    std::memset(grid, 0, BYTES_PER_NODE);

    for (int s = 0; s < n_aabbs; ++s) {
        const float* aabb = aabbs + s * 6;

        // Convert AABB [lo_x,lo_y,lo_z,hi_x,hi_y,hi_z] to grid coordinates
        int ix0 = static_cast<int>(std::floor(
            (aabb[0] - world_bounds_[0]) * inv_cell_[0]));
        int iy0 = static_cast<int>(std::floor(
            (aabb[1] - world_bounds_[1]) * inv_cell_[1]));
        int iz0 = static_cast<int>(std::floor(
            (aabb[2] - world_bounds_[2]) * inv_cell_[2]));
        int ix1 = static_cast<int>(std::ceil(
            (aabb[3] - world_bounds_[0]) * inv_cell_[0]));
        int iy1 = static_cast<int>(std::ceil(
            (aabb[4] - world_bounds_[1]) * inv_cell_[1]));
        int iz1 = static_cast<int>(std::ceil(
            (aabb[5] - world_bounds_[2]) * inv_cell_[2]));

        // Clamp to grid bounds
        ix0 = std::max(ix0, 0); ix1 = std::min(ix1, GRID_R);
        iy0 = std::max(iy0, 0); iy1 = std::min(iy1, GRID_R);
        iz0 = std::max(iz0, 0); iz1 = std::min(iz1, GRID_R);

        if (ix0 >= ix1 || iy0 >= iy1 || iz0 >= iz1) continue;

        // Word-level z-mask OR — replaces inner z-loop
        for (int x = ix0; x < ix1; ++x) {
            for (int y = iy0; y < iy1; ++y) {
                int w  = grid_word(x, y);
                int bb = grid_bit_base(y);
                grid[w] |= z_mask(iz0, iz1, bb);
            }
        }
    }

    set_valid(node_idx, true);
    mark_dirty(node_idx);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Grid union — dst = a | b  (zero-inflation)
// ═════════════════════════════════════════════════════════════════════════════

void GridStore::union_grids(int dst_idx, int a_idx, int b_idx)
{
    ensure_capacity(std::max({dst_idx, a_idx, b_idx}) + 1);

    const uint64_t* a = get_grid(a_idx);
    const uint64_t* b = get_grid(b_idx);
    uint64_t* dst = get_grid_mut(dst_idx);

    for (int i = 0; i < WORDS_PER_NODE; ++i) {
        dst[i] = a[i] | b[i];
    }

    set_valid(dst_idx, true);
    mark_dirty(dst_idx);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Collision checking — Obstacle → voxel range → word-level bit probe
// ═════════════════════════════════════════════════════════════════════════════
//
//  v4 optimisation: word-level z-mask AND.
//
//  Same principle as derive: for a given (x,y), build the z-mask for the
//  obstacle's z-range and AND it with the grid word.  If non-zero, there's
//  an occupied voxel in the overlap → collision.
//
//  This replaces up to 32 individual bit tests per (x,y) cell with ONE AND.

bool GridStore::check_collision(int node_idx,
                                const Obstacle* obstacles, int n_obs) const
{
    if (!has_grid(node_idx)) return true;  // unknown → assume collision

    const uint64_t* grid = get_grid(node_idx);

    for (int oi = 0; oi < n_obs; ++oi) {
        const auto& obs = obstacles[oi];
        auto lo = obs.lo();
        auto hi = obs.hi();

        // Convert obstacle AABB to grid coordinates
        int ix0 = static_cast<int>(std::floor(
            (static_cast<float>(lo.x()) - world_bounds_[0]) * inv_cell_[0]));
        int iy0 = static_cast<int>(std::floor(
            (static_cast<float>(lo.y()) - world_bounds_[1]) * inv_cell_[1]));
        int iz0 = static_cast<int>(std::floor(
            (static_cast<float>(lo.z()) - world_bounds_[2]) * inv_cell_[2]));
        int ix1 = static_cast<int>(std::ceil(
            (static_cast<float>(hi.x()) - world_bounds_[0]) * inv_cell_[0]));
        int iy1 = static_cast<int>(std::ceil(
            (static_cast<float>(hi.y()) - world_bounds_[1]) * inv_cell_[1]));
        int iz1 = static_cast<int>(std::ceil(
            (static_cast<float>(hi.z()) - world_bounds_[2]) * inv_cell_[2]));

        // Clamp
        ix0 = std::max(ix0, 0); ix1 = std::min(ix1, GRID_R);
        iy0 = std::max(iy0, 0); iy1 = std::min(iy1, GRID_R);
        iz0 = std::max(iz0, 0); iz1 = std::min(iz1, GRID_R);

        if (ix0 >= ix1 || iy0 >= iy1 || iz0 >= iz1) continue;

        // Word-level z-mask AND — replaces inner z-loop
        for (int x = ix0; x < ix1; ++x) {
            for (int y = iy0; y < iy1; ++y) {
                int w  = grid_word(x, y);
                int bb = grid_bit_base(y);
                if (grid[w] & z_mask(iz0, iz1, bb))
                    return true;
            }
        }
    }
    return false;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Access helpers
// ═════════════════════════════════════════════════════════════════════════════

void GridStore::set_has_grid(int node_idx, bool val) {
    ensure_capacity(node_idx + 1);
    set_valid(node_idx, val);
}

int GridStore::occupied_count(int node_idx) const {
    if (!has_grid(node_idx)) return 0;
    const uint64_t* grid = get_grid(node_idx);
    int count = 0;
    for (int i = 0; i < WORDS_PER_NODE; ++i) {
#ifdef _MSC_VER
    #if defined(_M_X64) || defined(_M_ARM64)
        count += static_cast<int>(__popcnt64(grid[i]));
    #else
        const uint64_t w = grid[i];
        count += static_cast<int>(__popcnt(static_cast<unsigned int>(w)));
        count += static_cast<int>(__popcnt(static_cast<unsigned int>(w >> 32)));
    #endif
#else
        count += __builtin_popcountll(grid[i]);
#endif
    }
    return count;
}

void GridStore::mark_dirty(int node_idx) {
    if (mmap_.is_open()) {
        ++n_dirty_;
        return;
    }
    if (!dirty_[node_idx]) {
        dirty_[node_idx] = 1;
        ++n_dirty_;
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  Persistence — GRD3 binary file
// ═════════════════════════════════════════════════════════════════════════════
//
//  GRD3 header (512 bytes):
//    [0..3]     magic:  0x33445247 ("GRD3" LE)
//    [4..7]     version: 3
//    [8..11]    capacity (n_alloc)
//    [12..35]   world_bounds (6 × float32)
//    [36..511]  reserved (zero-padded)
//
//  Record per node (fixed stride, same layout as v3 GRD2):
//    [valid:1B][pad:7B][bitfield:4096B] = 4104 bytes
//

void GridStore::save(const std::string& path) {
    if (mmap_.is_open()) {
        flush_mmap();
        return;
    }

    FILE* fp = std::fopen(path.c_str(), "wb");
    if (!fp) throw std::runtime_error("GridStore::save: cannot open " + path);

    int cap = capacity_;

    // Write GRD3 header
    uint8_t hdr[MMAP_HEADER_SIZE];
    std::memset(hdr, 0, MMAP_HEADER_SIZE);
    auto* u = reinterpret_cast<uint32_t*>(hdr);
    u[0] = MAGIC_V3;
    u[1] = VERSION_V3;
    u[2] = static_cast<uint32_t>(cap);
    std::memcpy(hdr + 12, world_bounds_, 6 * sizeof(float));
    std::fwrite(hdr, 1, MMAP_HEADER_SIZE, fp);

    // Write all records (fixed stride)
    uint8_t rec[MMAP_RECORD_SIZE];
    for (int i = 0; i < cap; ++i) {
        std::memset(rec, 0, 8);
        rec[0] = valid_[i];
        std::memcpy(rec + 8, get_grid(i), BYTES_PER_NODE);
        std::fwrite(rec, 1, MMAP_RECORD_SIZE, fp);
    }

    std::fclose(fp);

    std::fill(dirty_.begin(), dirty_.end(), 0);
    n_dirty_ = 0;
    save_path_ = path;
}

void GridStore::save_incremental() {
    if (save_path_.empty()) return;
    save_incremental(save_path_);
}

void GridStore::save_incremental(const std::string& path) {
    if (mmap_.is_open()) {
        flush_mmap();
        return;
    }

    if (save_path_.empty() || save_path_ != path) {
        save(path);
        return;
    }
    if (n_dirty_ == 0) return;

    // Seek to each dirty record and overwrite in-place
    FILE* fp = std::fopen(path.c_str(), "r+b");
    if (!fp) {
        save(path);
        return;
    }

    for (int i = 0; i < capacity_; ++i) {
        if (!dirty_[i]) continue;

        int64_t offset = MMAP_HEADER_SIZE +
                         static_cast<int64_t>(i) * MMAP_RECORD_SIZE;
        mmap_util::portable_fseek(fp, offset, SEEK_SET);

        uint8_t rec[MMAP_RECORD_SIZE];
        std::memset(rec, 0, 8);
        rec[0] = valid_[i];
        std::memcpy(rec + 8, get_grid(i), BYTES_PER_NODE);
        std::fwrite(rec, 1, MMAP_RECORD_SIZE, fp);
    }

    // Update capacity in header (may have grown)
    mmap_util::portable_fseek(fp, 8, SEEK_SET);
    uint32_t cap32 = static_cast<uint32_t>(capacity_);
    std::fwrite(&cap32, sizeof(uint32_t), 1, fp);

    std::fclose(fp);

    std::fill(dirty_.begin(), dirty_.end(), 0);
    n_dirty_ = 0;
    save_path_ = path;
}

void GridStore::load(const std::string& path) {
    FILE* fp = std::fopen(path.c_str(), "rb");
    if (!fp) throw std::runtime_error("GridStore::load: cannot open " + path);

    uint8_t hdr[MMAP_HEADER_SIZE];
    if (std::fread(hdr, 1, MMAP_HEADER_SIZE, fp) !=
        static_cast<size_t>(MMAP_HEADER_SIZE)) {
        std::fclose(fp);
        throw std::runtime_error("GridStore::load: truncated header in " + path);
    }

    auto* u = reinterpret_cast<const uint32_t*>(hdr);
    uint32_t magic = u[0];
    uint32_t ver   = u[1];

    if (magic != MAGIC_V3 || ver != VERSION_V3) {
        std::fclose(fp);
        throw std::runtime_error(
            "GridStore::load: unsupported format (expected GRD3) in " + path);
    }

    int cap = static_cast<int>(u[2]);
    std::memcpy(world_bounds_, hdr + 12, 6 * sizeof(float));
    recompute_cells();

    // Reset
    capacity_ = 0;
    data_.clear(); valid_.clear(); dirty_.clear();
    ensure_capacity(cap);

    // Read all records
    uint8_t rec[MMAP_RECORD_SIZE];
    for (int i = 0; i < cap; ++i) {
        if (std::fread(rec, 1, MMAP_RECORD_SIZE, fp) !=
            static_cast<size_t>(MMAP_RECORD_SIZE)) break;
        valid_[i] = rec[0];
        std::memcpy(get_grid_mut(i), rec + 8, BYTES_PER_NODE);
    }

    std::fclose(fp);
    std::fill(dirty_.begin(), dirty_.end(), 0);
    n_dirty_ = 0;
    save_path_ = path;
}

int GridStore::n_valid() const {
    int count = 0;
    for (int i = 0; i < capacity_; ++i)
        if (get_valid(i)) ++count;
    return count;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Capacity management
// ═════════════════════════════════════════════════════════════════════════════

void GridStore::ensure_capacity(int n_nodes) {
    if (n_nodes <= capacity_) return;
    int new_cap = std::max(n_nodes, capacity_ * 2);
    if (new_cap < 256) new_cap = 256;

    if (mmap_.is_open()) {
        grow_mmap(new_cap);
    } else {
        data_.resize(static_cast<size_t>(new_cap) * WORDS_PER_NODE, 0ULL);
        valid_.resize(static_cast<size_t>(new_cap), 0);
        dirty_.resize(static_cast<size_t>(new_cap), 0);
    }
    capacity_ = new_cap;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Mmap persistence
// ═════════════════════════════════════════════════════════════════════════════

void GridStore::write_mmap_header() {
    assert(mmap_.is_open());
    char* p = mmap_.ptr;
    std::memset(p, 0, MMAP_HEADER_SIZE);
    auto* u = reinterpret_cast<uint32_t*>(p);
    u[0] = MAGIC_V3;
    u[1] = VERSION_V3;
    u[2] = static_cast<uint32_t>(capacity_);
    std::memcpy(p + 12, world_bounds_, 6 * sizeof(float));
}

void GridStore::read_mmap_header() {
    assert(mmap_.is_open());
    const char* p = mmap_.ptr;
    auto* u = reinterpret_cast<const uint32_t*>(p);
    if (u[0] != MAGIC_V3 || u[1] != VERSION_V3)
        throw std::runtime_error("GridStore::read_mmap_header: expected GRD3");
    capacity_ = static_cast<int>(u[2]);
    std::memcpy(world_bounds_, p + 12, 6 * sizeof(float));
    recompute_cells();
}

void GridStore::recompute_cells() {
    for (int c = 0; c < 3; ++c) {
        cell_[c] = (world_bounds_[3 + c] - world_bounds_[c]) /
                   static_cast<float>(GRID_R);
        inv_cell_[c] = 1.0f / cell_[c];
    }
}

void GridStore::grow_mmap(int new_cap) {
    assert(mmap_.is_open());
    size_t new_sz = MMAP_HEADER_SIZE +
                    static_cast<size_t>(new_cap) * MMAP_RECORD_SIZE;
    mmap_util::grow(mmap_, new_sz);

    auto* u = reinterpret_cast<uint32_t*>(mmap_.ptr);
    u[2] = static_cast<uint32_t>(new_cap);
    capacity_ = new_cap;
}

void GridStore::create_mmap(const std::string& path, int initial_capacity) {
    if (mmap_.is_open()) close_mmap();

    int cap = std::max(initial_capacity, 256);
    size_t file_sz = MMAP_HEADER_SIZE +
                     static_cast<size_t>(cap) * MMAP_RECORD_SIZE;

    mmap_ = mmap_util::open_rw(path, file_sz);
    capacity_ = cap;

    data_.clear();  data_.shrink_to_fit();
    valid_.clear(); valid_.shrink_to_fit();
    dirty_.clear(); dirty_.shrink_to_fit();
    n_dirty_ = 0;

    write_mmap_header();

    // Zero all records
    std::memset(mmap_.ptr + MMAP_HEADER_SIZE, 0,
                static_cast<size_t>(cap) * MMAP_RECORD_SIZE);

    save_path_ = path;
}

void GridStore::load_mmap(const std::string& path) {
    if (mmap_.is_open()) close_mmap();

    mmap_ = mmap_util::open_rw(path, 0);
    if (mmap_.size < static_cast<size_t>(MMAP_HEADER_SIZE))
        throw std::runtime_error("GridStore::load_mmap: file too small " + path);

    read_mmap_header();

    size_t expected = MMAP_HEADER_SIZE +
                      static_cast<size_t>(capacity_) * MMAP_RECORD_SIZE;
    if (mmap_.size < expected) {
        mmap_util::grow(mmap_, expected);
    }

    data_.clear();  data_.shrink_to_fit();
    valid_.clear(); valid_.shrink_to_fit();
    dirty_.clear(); dirty_.shrink_to_fit();
    n_dirty_ = 0;

    save_path_ = path;
}

void GridStore::flush_mmap() {
    if (!mmap_.is_open()) return;
    write_mmap_header();
    mmap_util::flush(mmap_);
    n_dirty_ = 0;
}

void GridStore::close_mmap() {
    if (!mmap_.is_open()) return;
    flush_mmap();
    mmap_util::close(mmap_);
    capacity_ = 0;
}

} // namespace envelope
} // namespace sbf

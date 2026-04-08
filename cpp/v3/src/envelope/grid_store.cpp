// SafeBoxForest v2 — GridStore implementation
// Per-node bitfield (uint64_t[512]) occupancy grid — zero-inflation union.
#include "sbf/envelope/grid_store.h"
#include "sbf/envelope/envelope_derive.h"
#include "sbf/envelope/grid_envelope.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>

#ifdef _MSC_VER
#include <intrin.h>   // __popcnt64
#endif

namespace sbf {
namespace envelope {

// ═════════════════════════════════════════════════════════════════════════════
//  Construction
// ═════════════════════════════════════════════════════════════════════════════

GridStore::GridStore(int n_frames, int n_active_links,
                     const int* active_link_map,
                     const float* link_radii,
                     const float* base_pos,
                     const float world_bounds[6],
                     int initial_capacity)
    : n_frames_(n_frames)
    , n_active_links_(n_active_links)
    , capacity_(0)
{
    assert(n_active_links_ <= MAX_LINKS);
    std::memcpy(active_link_map_, active_link_map, n_active_links_ * sizeof(int));
    std::memset(link_radii_, 0, sizeof(link_radii_));
    if (link_radii) {
        std::memcpy(link_radii_, link_radii, n_active_links_ * sizeof(float));
    }
    if (base_pos) {
        std::memcpy(base_pos_, base_pos, 3 * sizeof(float));
    }
    std::memcpy(world_bounds_, world_bounds, 6 * sizeof(float));

    for (int c = 0; c < 3; ++c) {
        cell_[c] = (world_bounds_[3 + c] - world_bounds_[c]) /
                   static_cast<float>(GRID_R);
        inv_cell_[c] = 1.0f / cell_[c];
    }

    ensure_capacity(initial_capacity);
}

GridStore::~GridStore() {
    if (mmap_.is_open()) {
        try { close_mmap(); } catch (...) {}
    }
}

GridStore::GridStore(GridStore&& o) noexcept
    : n_frames_(o.n_frames_)
    , n_active_links_(o.n_active_links_)
    , capacity_(o.capacity_)
    , data_(std::move(o.data_))
    , valid_(std::move(o.valid_))
    , dirty_(std::move(o.dirty_))
    , n_dirty_(o.n_dirty_)
    , mmap_(std::move(o.mmap_))
    , save_path_(std::move(o.save_path_))
{
    std::memcpy(active_link_map_, o.active_link_map_, sizeof(active_link_map_));
    std::memcpy(link_radii_, o.link_radii_, sizeof(link_radii_));
    std::memcpy(base_pos_, o.base_pos_, sizeof(base_pos_));
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
    n_frames_ = o.n_frames_;
    n_active_links_ = o.n_active_links_;
    capacity_ = o.capacity_;
    data_ = std::move(o.data_);
    valid_ = std::move(o.valid_);
    dirty_ = std::move(o.dirty_);
    n_dirty_ = o.n_dirty_;
    mmap_ = std::move(o.mmap_);
    save_path_ = std::move(o.save_path_);
    std::memcpy(active_link_map_, o.active_link_map_, sizeof(active_link_map_));
    std::memcpy(link_radii_, o.link_radii_, sizeof(link_radii_));
    std::memcpy(base_pos_, o.base_pos_, sizeof(base_pos_));
    std::memcpy(world_bounds_, o.world_bounds_, sizeof(world_bounds_));
    std::memcpy(cell_, o.cell_, sizeof(cell_));
    std::memcpy(inv_cell_, o.inv_cell_, sizeof(inv_cell_));
    o.capacity_ = 0;
    o.n_dirty_ = 0;
    return *this;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Derive from frames — rasterise frame intervals into bitfield grid
// ═════════════════════════════════════════════════════════════════════════════

void GridStore::derive_from_frames(int node_idx, const float* frames,
                                   int n_sub_per_link)
{
    ensure_capacity(node_idx + 1);

    uint64_t* grid = get_grid_mut(node_idx);
    std::memset(grid, 0, BYTES_PER_NODE);

    // Temp buffer for sub-segment AABBs (stack-allocate up to 64 subs)
    float sub_buf[64 * 6];

    float min_cell = std::min({cell_[0], cell_[1], cell_[2]});

    for (int i = 0; i < n_active_links_; ++i) {
        int frame_idx  = active_link_map_[i];
        int parent_idx = frame_idx - 1;
        float r = link_radii_[i];

        // Determine n_sub
        int n_sub;
        if (n_sub_per_link > 0) {
            n_sub = n_sub_per_link;
        } else {
            n_sub = adaptive_subdivision_count(
                frames, parent_idx, frame_idx, base_pos_,
                min_cell, /*max_sub=*/32);
        }
        if (n_sub > 64) n_sub = 64;

        // Generate sub-segment AABBs
        derive_aabb_subdivided(frames, n_frames_,
                               parent_idx, frame_idx,
                               n_sub, r, base_pos_, sub_buf);

        // Voxelise each sub-AABB into the bitfield
        for (int s = 0; s < n_sub; ++s) {
            const float* aabb = sub_buf + s * 6;

            // Convert to grid coordinates
            int ix0 = static_cast<int>(std::floor((aabb[0] - world_bounds_[0]) * inv_cell_[0]));
            int iy0 = static_cast<int>(std::floor((aabb[1] - world_bounds_[1]) * inv_cell_[1]));
            int iz0 = static_cast<int>(std::floor((aabb[2] - world_bounds_[2]) * inv_cell_[2]));
            int ix1 = static_cast<int>(std::ceil((aabb[3] - world_bounds_[0]) * inv_cell_[0]));
            int iy1 = static_cast<int>(std::ceil((aabb[4] - world_bounds_[1]) * inv_cell_[1]));
            int iz1 = static_cast<int>(std::ceil((aabb[5] - world_bounds_[2]) * inv_cell_[2]));

            // Clamp to grid bounds
            ix0 = std::max(ix0, 0); ix1 = std::min(ix1, GRID_R);
            iy0 = std::max(iy0, 0); iy1 = std::min(iy1, GRID_R);
            iz0 = std::max(iz0, 0); iz1 = std::min(iz1, GRID_R);

            // Set bits
            for (int x = ix0; x < ix1; ++x) {
                for (int y = iy0; y < iy1; ++y) {
                    for (int z = iz0; z < iz1; ++z) {
                        int lin = voxel_linear(x, y, z);
                        grid[word_index(lin)] |= (uint64_t(1) << bit_index(lin));
                    }
                }
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
//  Collision checking — obstacle AABB → voxel range → bit probe
// ═════════════════════════════════════════════════════════════════════════════

bool GridStore::check_collision(int node_idx,
                                const float* obs_compact, int n_obs) const
{
    if (!has_grid(node_idx)) return true;  // unknown → assume collision

    const uint64_t* grid = get_grid(node_idx);

    for (int oi = 0; oi < n_obs; ++oi) {
        const float* ob = obs_compact + oi * 6;
        // obs interleaved: [lo_x,hi_x, lo_y,hi_y, lo_z,hi_z]
        int ix0 = static_cast<int>(std::floor((ob[0] - world_bounds_[0]) * inv_cell_[0]));
        int iy0 = static_cast<int>(std::floor((ob[2] - world_bounds_[1]) * inv_cell_[1]));
        int iz0 = static_cast<int>(std::floor((ob[4] - world_bounds_[2]) * inv_cell_[2]));
        int ix1 = static_cast<int>(std::ceil((ob[1] - world_bounds_[0]) * inv_cell_[0]));
        int iy1 = static_cast<int>(std::ceil((ob[3] - world_bounds_[1]) * inv_cell_[1]));
        int iz1 = static_cast<int>(std::ceil((ob[5] - world_bounds_[2]) * inv_cell_[2]));

        ix0 = std::max(ix0, 0); ix1 = std::min(ix1, GRID_R);
        iy0 = std::max(iy0, 0); iy1 = std::min(iy1, GRID_R);
        iz0 = std::max(iz0, 0); iz1 = std::min(iz1, GRID_R);

        for (int x = ix0; x < ix1; ++x) {
            for (int y = iy0; y < iy1; ++y) {
                for (int z = iz0; z < iz1; ++z) {
                    int lin = voxel_linear(x, y, z);
                    if (grid[word_index(lin)] & (uint64_t(1) << bit_index(lin)))
                        return true;
                }
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
        count += static_cast<int>(__popcnt64(grid[i]));
#else
        count += __builtin_popcountll(grid[i]);
#endif
    }
    return count;
}

void GridStore::mark_dirty(int node_idx) {
    if (mmap_.is_open()) {
        // mmap mode: OS tracks dirty pages; we just count for stats
        ++n_dirty_;
        return;
    }
    if (!dirty_[node_idx]) {
        dirty_[node_idx] = 1;
        ++n_dirty_;
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  Persistence — .grids binary file
// ═════════════════════════════════════════════════════════════════════════════
//
//  Header (512 bytes):
//    [0..3]   magic: 0x31445247 ("GRD1" LE)
//    [4..7]   version: 1
//    [8..11]  n_frames
//    [12..15] n_active
//    [16..19] n_stored
//    [20..147]  active_link_map (32 × int32)
//    [148..275] link_radii (32 × float32)
//    [276..287] base_pos (3 × float32)
//    [288..311] world_bounds (6 × float32)
//    [312..511] reserved (zero-padded)
//
//  Record per node:
//    [4 bytes]  node_idx (uint32)
//    [N bytes]  bitfield → uint8 → RLE compressed payload
//    [4 bytes]  payload_len (uint32, written BEFORE payload for seeking)
//
//  Actual layout per record:
//    [uint32: node_idx] [uint32: payload_len] [payload_len bytes: RLE data]
//

// Forward-declare RLE helpers — already declared in grid_envelope.h
// (included via grid_store.h build chain)

// Helper: bitfield → uint8 grid
static void bitfield_to_uint8(const uint64_t* bf, uint8_t* out) {
    for (int i = 0; i < GRID_TOTAL; ++i) {
        int w = i >> 6;
        int b = i & 63;
        out[i] = (bf[w] >> b) & 1 ? 1 : 0;
    }
}

// Helper: uint8 grid → bitfield
static void uint8_to_bitfield(const uint8_t* grid, uint64_t* out) {
    std::memset(out, 0, BYTES_PER_NODE);
    for (int i = 0; i < GRID_TOTAL; ++i) {
        if (grid[i]) {
            int w = i >> 6;
            int b = i & 63;
            out[w] |= (uint64_t(1) << b);
        }
    }
}

// (Legacy v1 header helpers removed; load() handles v1 inline.)

void GridStore::save(const std::string& path) {
    // In mmap mode: just flush (data already in the mapped file)
    if (mmap_.is_open()) {
        flush_mmap();
        return;
    }

    // Write v2 flat format
    FILE* fp = std::fopen(path.c_str(), "wb");
    if (!fp) throw std::runtime_error("GridStore::save: cannot open " + path);

    int nv = n_valid();
    int cap = capacity_;

    // Write v2 header
    uint8_t hdr[MMAP_HEADER_SIZE];
    std::memset(hdr, 0, MMAP_HEADER_SIZE);
    auto* u = reinterpret_cast<uint32_t*>(hdr);
    u[0] = MAGIC_V2;
    u[1] = VERSION_V2;
    u[2] = static_cast<uint32_t>(n_frames_);
    u[3] = static_cast<uint32_t>(n_active_links_);
    u[4] = static_cast<uint32_t>(cap);
    std::memcpy(hdr + 20, active_link_map_, MAX_LINKS * sizeof(int));
    std::memcpy(hdr + 148, link_radii_, MAX_LINKS * sizeof(float));
    std::memcpy(hdr + 276, base_pos_, 3 * sizeof(float));
    std::memcpy(hdr + 288, world_bounds_, 6 * sizeof(float));
    std::fwrite(hdr, 1, MMAP_HEADER_SIZE, fp);

    // Write all records (fixed stride: 8 + 4096 bytes each)
    // Record = [valid:1B][pad:7B][bitfield:4096B]
    uint8_t rec[MMAP_RECORD_SIZE];
    for (int i = 0; i < cap; ++i) {
        std::memset(rec, 0, 8);  // clear valid + pad
        rec[0] = valid_[i];
        std::memcpy(rec + 8, get_grid(i), BYTES_PER_NODE);
        std::fwrite(rec, 1, MMAP_RECORD_SIZE, fp);
    }

    std::fclose(fp);

    std::fill(dirty_.begin(), dirty_.end(), 0);
    n_dirty_ = 0;
    save_path_ = path;
    (void)nv;  // nv computed for diagnostics if needed
}

void GridStore::save_incremental() {
    if (save_path_.empty()) return;
    save_incremental(save_path_);
}

void GridStore::save_incremental(const std::string& path) {
    // mmap mode: just flush
    if (mmap_.is_open()) {
        flush_mmap();
        return;
    }

    if (save_path_.empty() || save_path_ != path) {
        save(path);
        return;
    }
    if (n_dirty_ == 0) return;

    // v2 format: seek to each dirty record and overwrite in-place
    FILE* fp = std::fopen(path.c_str(), "r+b");
    if (!fp) {
        save(path);
        return;
    }

    for (int i = 0; i < capacity_; ++i) {
        if (!dirty_[i]) continue;

        // Seek to record i
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
    mmap_util::portable_fseek(fp, 16, SEEK_SET);
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

    if (magic == MAGIC_V2 && ver == VERSION_V2) {
        // ---- v2 flat format ----
        int nf  = static_cast<int>(u[2]);
        int na  = static_cast<int>(u[3]);
        int cap = static_cast<int>(u[4]);

        n_frames_ = nf;
        n_active_links_ = na;
        std::memcpy(active_link_map_, hdr + 20,  MAX_LINKS * sizeof(int));
        std::memcpy(link_radii_,      hdr + 148, MAX_LINKS * sizeof(float));
        std::memcpy(base_pos_,        hdr + 276, 3 * sizeof(float));
        std::memcpy(world_bounds_,    hdr + 288, 6 * sizeof(float));
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

    } else if (magic == MAGIC_V1 && ver == VERSION_V1) {
        // ---- v1 RLE format (legacy) ----
        int nf = static_cast<int>(u[2]);
        int na = static_cast<int>(u[3]);
        int ns = static_cast<int>(u[4]);

        float wb[6];
        std::memcpy(active_link_map_, hdr + 20,  MAX_LINKS * sizeof(int));
        std::memcpy(link_radii_,      hdr + 148, MAX_LINKS * sizeof(float));
        std::memcpy(base_pos_,        hdr + 276, 3 * sizeof(float));
        std::memcpy(wb,               hdr + 288, 6 * sizeof(float));

        n_frames_ = nf;
        n_active_links_ = na;
        std::memcpy(world_bounds_, wb, sizeof(world_bounds_));
        recompute_cells();

        // Reset
        capacity_ = 0;
        data_.clear(); valid_.clear(); dirty_.clear();

        // Read RLE records
        std::vector<uint8_t> uint8_grid(GRID_TOTAL);
        int max_idx = 0;

        struct Record {
            uint32_t idx;
            std::vector<uint8_t> rle_data;
        };
        std::vector<Record> records;
        records.reserve(ns);

        for (int r = 0; r < ns; ++r) {
            uint32_t idx, payload_len;
            if (std::fread(&idx, sizeof(uint32_t), 1, fp) != 1) break;
            if (std::fread(&payload_len, sizeof(uint32_t), 1, fp) != 1) break;

            Record rec;
            rec.idx = idx;
            rec.rle_data.resize(payload_len);
            if (std::fread(rec.rle_data.data(), 1, payload_len, fp) !=
                payload_len) break;
            records.push_back(std::move(rec));

            if (static_cast<int>(idx) + 1 > max_idx)
                max_idx = static_cast<int>(idx) + 1;
        }
        std::fclose(fp);

        // Populate
        ensure_capacity(max_idx);
        for (auto& rec : records) {
            int ni = static_cast<int>(rec.idx);
            std::memset(uint8_grid.data(), 0, GRID_TOTAL);
            grid_decompress_rle(rec.rle_data.data(), rec.rle_data.size(),
                                GRID_R, uint8_grid.data());
            uint8_to_bitfield(uint8_grid.data(), get_grid_mut(ni));
            valid_[ni] = 1;
        }

        std::fill(dirty_.begin(), dirty_.end(), 0);
        n_dirty_ = 0;
        save_path_ = path;

    } else {
        std::fclose(fp);
        throw std::runtime_error(
            "GridStore::load: unknown magic/version in " + path);
    }
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
//  Mmap persistence (lazy load + incremental store)
// ═════════════════════════════════════════════════════════════════════════════

// GRD2 header layout (512 bytes):
//   [0..3]     magic:  0x32445247 ("GRD2" LE)
//   [4..7]     version: 2
//   [8..11]    n_frames
//   [12..15]   n_active
//   [16..19]   capacity (n_alloc)
//   [20..147]  active_link_map (32 × int32)
//   [148..275] link_radii (32 × float32)
//   [276..287] base_pos (3 × float32)
//   [288..311] world_bounds (6 × float32)
//   [312..511] reserved (zero)

void GridStore::write_mmap_header() {
    assert(mmap_.is_open());
    char* p = mmap_.ptr;
    std::memset(p, 0, MMAP_HEADER_SIZE);
    auto* u = reinterpret_cast<uint32_t*>(p);
    u[0] = MAGIC_V2;
    u[1] = VERSION_V2;
    u[2] = static_cast<uint32_t>(n_frames_);
    u[3] = static_cast<uint32_t>(n_active_links_);
    u[4] = static_cast<uint32_t>(capacity_);
    std::memcpy(p + 20, active_link_map_, MAX_LINKS * sizeof(int));
    std::memcpy(p + 148, link_radii_, MAX_LINKS * sizeof(float));
    std::memcpy(p + 276, base_pos_, 3 * sizeof(float));
    std::memcpy(p + 288, world_bounds_, 6 * sizeof(float));
}

void GridStore::read_mmap_header() {
    assert(mmap_.is_open());
    const char* p = mmap_.ptr;
    auto* u = reinterpret_cast<const uint32_t*>(p);
    if (u[0] != MAGIC_V2 || u[1] != VERSION_V2)
        throw std::runtime_error("GridStore::read_mmap_header: bad magic/version");
    n_frames_      = static_cast<int>(u[2]);
    n_active_links_= static_cast<int>(u[3]);
    capacity_      = static_cast<int>(u[4]);
    std::memcpy(active_link_map_, p + 20,  MAX_LINKS * sizeof(int));
    std::memcpy(link_radii_,      p + 148, MAX_LINKS * sizeof(float));
    std::memcpy(base_pos_,        p + 276, 3 * sizeof(float));
    std::memcpy(world_bounds_,    p + 288, 6 * sizeof(float));
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

    // Update capacity in header
    auto* u = reinterpret_cast<uint32_t*>(mmap_.ptr);
    u[4] = static_cast<uint32_t>(new_cap);
    capacity_ = new_cap;
}

void GridStore::create_mmap(const std::string& path, int initial_capacity) {
    if (mmap_.is_open()) close_mmap();

    int cap = std::max(initial_capacity, 256);
    size_t file_sz = MMAP_HEADER_SIZE +
                     static_cast<size_t>(cap) * MMAP_RECORD_SIZE;

    mmap_ = mmap_util::open_rw(path, file_sz);
    capacity_ = cap;

    // Clear vectors (data lives in mmap now)
    data_.clear(); data_.shrink_to_fit();
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

    // Validate file size
    size_t expected = MMAP_HEADER_SIZE +
                      static_cast<size_t>(capacity_) * MMAP_RECORD_SIZE;
    if (mmap_.size < expected) {
        mmap_util::grow(mmap_, expected);
    }

    // Clear vectors (data lives in mmap)
    data_.clear(); data_.shrink_to_fit();
    valid_.clear(); valid_.shrink_to_fit();
    dirty_.clear(); dirty_.shrink_to_fit();
    n_dirty_ = 0;

    save_path_ = path;
}

void GridStore::flush_mmap() {
    if (!mmap_.is_open()) return;
    write_mmap_header();  // update capacity in header
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

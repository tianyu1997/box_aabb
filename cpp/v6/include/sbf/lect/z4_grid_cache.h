#pragma once
/// @file z4_grid_cache.h
/// @brief Z4GridCache — mmap-backed persistent cache for per-node SparseVoxelGrid data.
///
/// Each Z4GridCache instance stores grids for a SINGLE channel (safe OR unsafe).
/// Each cached entry records its quality metadata (EnvelopeType, resolution,
/// n_subdivisions).  On lookup the cache checks whether the stored grid
/// meets the requested quality; if not the caller recomputes and updates.
///
/// File layout:
///   [Header 64B]
///   [Index section: N slots × 32B]
///   [Data section: append-only brick data]
///
/// Index slot (32 bytes):
///   uint64_t key           (0 = empty sentinel)
///   uint64_t data_offset   (byte offset from data section start)
///   uint32_t n_bricks      (number of bricks)
///   uint8_t  envelope_type (EnvelopeType enum)
///   uint8_t  n_sub         (subdivision count, 0 for Hull16_Grid)
///   uint16_t reserved
///   float    resolution    (voxel delta in metres)
///   uint32_t pad
///
/// Brick record (76 bytes):
///   int32_t  bx, by, bz    (12 bytes — BrickCoord)
///   uint64_t words[8]      (64 bytes — BitBrick)

#include <sbf/envelope/envelope_type.h>
#include <sbf/voxel/voxel_grid.h>
#include <cstdint>
#include <string>
#include <mutex>
#include <shared_mutex>
#include <memory>
#include <list>
#include <unordered_map>
#include <atomic>

namespace sbf {

/// Quality descriptor for a cached grid entry.
struct GridQuality {
    EnvelopeType type   = EnvelopeType::LinkIAABB_Grid;
    float  resolution   = 0.05f;   ///< voxel delta (smaller = finer)
    int    n_sub        = 1;       ///< subdivision count (higher = tighter)

    /// Does this cached quality satisfy a request for @p req?
    /// Same type required; finer-or-equal resolution; equal-or-more subs.
    bool satisfies(const GridQuality& req) const {
        return type == req.type
            && resolution <= req.resolution + 1e-6f
            && n_sub >= req.n_sub;
    }
};

class Z4GridCache {
public:
    Z4GridCache() = default;
    ~Z4GridCache();

    Z4GridCache(const Z4GridCache&) = delete;
    Z4GridCache& operator=(const Z4GridCache&) = delete;

    /// Open or create the grid cache file (single channel).
    /// @param path              File path
    /// @param initial_capacity  Initial number of index slots (power of 2)
    /// @param max_capacity      Maximum allowed index capacity (0 = unlimited).
    ///                          grow_index() is skipped when this limit is
    ///                          reached; insert silently drops the entry.
    /// @param mem_cache_bytes   Maximum bytes for in-memory LRU (0 = disabled).
    bool open(const std::string& path,
              int initial_capacity = 4096, int max_capacity = 0,
              size_t mem_cache_bytes = 256ULL * 1024 * 1024);

    void close();
    bool is_open() const { return data_ != nullptr; }

    // ─── Lookup ─────────────────────────────────────────────────────────

    /// Lookup a cached grid by Z4 key with quality check.
    /// Returns a reconstructed SparseVoxelGrid, or nullptr if not cached
    /// or cached quality is insufficient for @p req.
    std::unique_ptr<voxel::SparseVoxelGrid> lookup(
        uint64_t z4_key, const GridQuality& req) const;

    /// Check if a key exists with sufficient quality.
    bool contains(uint64_t z4_key, const GridQuality& req) const;

    // ─── Insert / Update ────────────────────────────────────────────────

    /// Insert or upgrade a grid entry.
    /// If the key already exists and its quality already satisfies @p quality,
    /// the insert is skipped.  Otherwise the entry is replaced (old data
    /// becomes dead space — acceptable for mmap-backed append-only layout).
    /// Thread-safe (uses internal mutex for writes).
    void insert(uint64_t z4_key, const voxel::SparseVoxelGrid& grid,
                const GridQuality& quality);

    // ─── Stats ──────────────────────────────────────────────────────────
    int    capacity()   const;
    int    size()       const;
    const std::string& path() const { return path_; }

    /// In-memory LRU stats.
    int64_t mem_hits()   const { return mem_hits_.load(std::memory_order_relaxed); }
    int64_t mem_misses() const { return mem_misses_.load(std::memory_order_relaxed); }
    int     mem_entries() const;
    size_t  mem_bytes()   const;

private:
    // ── File header (64 bytes) ──────────────────────────────────────────
    struct Header {
        char     magic[8];        // "SBF7GRD\x00"
        uint32_t version;         // 2
        int32_t  index_capacity;  // number of index slots (power of 2)
        int32_t  index_size;      // number of occupied index slots
        uint32_t reserved;
        double   best_resolution; // pre-alloc hint (e.g. 0.01)
        uint64_t data_section_off; // byte offset of data section from file start
        uint64_t data_used;       // bytes used in data section
        uint8_t  pad[16];
    };
    static_assert(sizeof(Header) == 64, "Header must be 64 bytes");

    // ── Index slot (32 bytes) ───────────────────────────────────────────
    struct IndexSlot {
        uint64_t key;          // 0 = empty
        uint64_t data_offset;  // byte offset from data_section_off
        uint32_t n_bricks;
        uint8_t  envelope_type; // EnvelopeType enum
        uint8_t  n_sub;         // subdivision count
        uint16_t reserved;
        float    resolution;    // voxel delta in metres
        uint32_t pad;
    };
    static_assert(sizeof(IndexSlot) == 32, "IndexSlot must be 32 bytes");

    static constexpr uint64_t kEmptyKey = 0;
    static constexpr double kMaxLoadFactor = 0.75;
    static constexpr int kBrickRecordBytes = 76;  // 12 (coord) + 64 (BitBrick)
    static constexpr double kBestResolution = 0.01; // pre-alloc data hint

    IndexSlot* index_slot(int idx) const {
        return reinterpret_cast<IndexSlot*>(
            data_ + sizeof(Header) + static_cast<size_t>(idx) * sizeof(IndexSlot));
    }

    GridQuality slot_quality(const IndexSlot* s) const {
        return {static_cast<EnvelopeType>(s->envelope_type),
                s->resolution,
                static_cast<int>(s->n_sub)};
    }

    int find_index_slot(uint64_t key) const;
    void grow_index();
    void remap_file(size_t new_file_size, size_t new_mmap_size);

    // ── State ───────────────────────────────────────────────────────────
    std::string path_;
    uint8_t*    data_       = nullptr;
    size_t      file_size_  = 0;
    size_t      mmap_size_  = 0;   // mmap'd region (header + index only)
    int         fd_         = -1;
    int         max_capacity_ = 0;  ///< 0 = unlimited
    mutable std::shared_mutex mu_;

    // ── In-memory LRU cache ─────────────────────────────────────────────
    // Evicts least-recently-used entries when total bytes exceed budget.
    // Stores SparseVoxelGrid by value; lookup returns a copy (single memcpy
    // of the flat brick array — much faster than pread + per-brick rebuild).
    struct LRUEntry {
        uint64_t key;
        voxel::SparseVoxelGrid grid;
        size_t byte_size;   ///< estimated memory for this grid
    };
    using LRUList = std::list<LRUEntry>;
    using LRUMap  = std::unordered_map<uint64_t, LRUList::iterator>;

    mutable LRUList   lru_list_;
    mutable LRUMap    lru_map_;
    mutable size_t    lru_bytes_     = 0;
    size_t    lru_max_bytes_ = 0;
    mutable std::atomic<int64_t> mem_hits_{0};
    mutable std::atomic<int64_t> mem_misses_{0};

    /// Estimate memory cost of a SparseVoxelGrid.
    static size_t estimate_grid_bytes(const voxel::SparseVoxelGrid& g);

    /// Evict LRU entries until total bytes <= lru_max_bytes_.
    /// Caller must hold unique lock on mu_.
    void lru_evict() const;

    /// Insert grid into LRU (caller holds unique lock).
    void lru_put(uint64_t key, const voxel::SparseVoxelGrid& grid) const;
};

}  // namespace sbf

#pragma once
/// @file z4_ep_cache.h
/// @brief Z4EpCache — mmap-backed persistent hash table for endpoint iAABB data.
///
/// Each Z4EpCache instance stores a SINGLE channel (safe OR unsafe) of
/// endpoint iAABBs keyed by Z4 canonical interval hash.
/// File format: open-addressing hash table with linear probing.
///
/// Layout:
///   [Header 64B]
///   [Slot array: N slots × slot_bytes]
///
/// Each slot:
///   uint64_t  key           (0 = empty sentinel)
///   uint8_t   source        (EndpointSource enum)
///   uint8_t   pad[7]        (alignment)
///   float     ep[ep_stride]

#include <sbf/envelope/endpoint_source.h>
#include <cstdint>
#include <string>
#include <mutex>
#include <shared_mutex>

namespace sbf {

class Z4EpCache {
public:
    Z4EpCache() = default;
    ~Z4EpCache();

    Z4EpCache(const Z4EpCache&) = delete;
    Z4EpCache& operator=(const Z4EpCache&) = delete;

    /// Open or create the EP cache file (single channel).
    /// @param path      File path (e.g. ~/.sbf_cache/<hash>/ep_safe.cache)
    /// @param ep_stride Number of floats per node (n_active_links * 2 * 6)
    /// @param initial_capacity  Initial number of hash slots (must be power of 2)
    /// @param max_capacity  Maximum allowed capacity (0 = unlimited). grow() is
    ///                      skipped when this limit is reached; insert silently
    ///                      drops the entry instead.
    /// @return true on success
    bool open(const std::string& path, int ep_stride,
              int initial_capacity = 4096, int max_capacity = 0);

    /// Close the mapping.
    void close();

    bool is_open() const { return data_ != nullptr; }

    // ─── Lookup ─────────────────────────────────────────────────────────

    /// Lookup EP data for a Z4 canonical key.
    /// @return pointer to ep_stride floats in the mmap, or nullptr if not found
    ///         or source is incompatible.
    const float* lookup(uint64_t z4_key,
                        EndpointSource requested_source) const;

    /// Check if a key exists with compatible source.
    bool contains(uint64_t z4_key,
                  EndpointSource requested_source) const;

    /// Thread-safe lookup + copy: copies ep_stride floats into @p out under lock.
    /// Returns true if found and copied, false otherwise.
    bool lookup_copy(uint64_t z4_key,
                     EndpointSource requested_source,
                     float* out) const;

    // ─── Insert ─────────────────────────────────────────────────────────

    /// Insert EP data.
    /// @param z4_key     Z4 canonical interval hash (must be != 0)
    /// @param source     EndpointSource that produced this data
    /// @param ep_data    Pointer to ep_stride floats to store
    /// Thread-safe (uses internal mutex for writes).
    void insert(uint64_t z4_key,
                EndpointSource source, const float* ep_data);

    // ─── Stats ──────────────────────────────────────────────────────────
    int  capacity()    const;   ///< Number of hash slots
    int  size()        const;   ///< Number of occupied slots
    int  ep_stride()   const { return ep_stride_; }
    const std::string& path() const { return path_; }

private:
    // ── File header (64 bytes) ──────────────────────────────────────────
    struct Header {
        char     magic[8];       // "SBF7EP\x00\x00"  (version 7 = single-channel)
        uint32_t version;        // 2
        int32_t  ep_stride;      // floats per node
        int32_t  capacity;       // number of slots (power of 2)
        int32_t  size;           // number of occupied slots
        uint8_t  pad[40];        // reserved (64 - 24 = 40)
    };
    static_assert(sizeof(Header) == 64, "Header must be 64 bytes");

    static constexpr uint64_t kEmptyKey = 0;
    static constexpr double kMaxLoadFactor = 0.75;

    // ── Slot layout (single channel) ────────────────────────────────────
    // [key:8][source:1][pad:7][ep:ep_stride*4]
    int slot_bytes() const { return 16 + ep_stride_ * static_cast<int>(sizeof(float)); }

    uint8_t* slot_ptr(int idx) const {
        return data_ + sizeof(Header) + static_cast<size_t>(idx) * slot_bytes();
    }

    uint64_t& slot_key(uint8_t* slot) const {
        return *reinterpret_cast<uint64_t*>(slot);
    }
    uint8_t& slot_source(uint8_t* slot) const {
        return slot[8];
    }
    float* slot_ep(uint8_t* slot) const {
        return reinterpret_cast<float*>(slot + 16);
    }

    int find_slot(uint64_t key) const;   ///< Returns slot index (may be empty)
    void grow();                          ///< Double capacity and rehash

    // ── State ───────────────────────────────────────────────────────────
    std::string path_;
    uint8_t*    data_     = nullptr;   ///< mmap base (MAP_SHARED)
    size_t      file_size_ = 0;
    int         fd_       = -1;
    int         ep_stride_ = 0;
    int         max_capacity_ = 0;     ///< 0 = unlimited
    mutable std::shared_mutex mu_;     ///< Reader-writer lock
};

}  // namespace sbf

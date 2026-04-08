// SafeBoxForest — HCACHE02 binary file I/O
// Binary compatible with v4 Python HCACHE format
#pragma once

#include "sbf/core/types.h"
#include "sbf/forest/node_store.h"
#include <string>
#include <vector>

namespace sbf {

// HCACHE02 file format constants
namespace hcache {
    constexpr char MAGIC[8] = {'H','C','A','C','H','E','0','2'};
    constexpr int VERSION = 2;
    constexpr int HEADER_SIZE = 4096;   // page-aligned

    // Header layout (within first 80 bytes):
    // [0..8)    magic (8 bytes)
    // [8..12)   version (int32)
    // [12..20)  n_nodes (int64)
    // [20..28)  n_alloc (int64)
    // [28..32)  n_dims (int32)
    // [32..36)  n_links (int32)
    // [36..44)  n_fk_calls (int64)
    // [44..48)  stride (int32)
    // [48..80)  fp_sha256 (32 bytes)
    // [80..336) joint_limits: MAX_DIMS * 2 * float64 = 256 bytes
}

class HCacheFile {
public:
    HCacheFile() = default;
    ~HCacheFile();

    // Move-only (NodeStore is non-copyable)
    HCacheFile(HCacheFile&& o) noexcept
        : fd_(o.fd_), mapped_(o.mapped_), file_size_(o.file_size_),
          store_(std::move(o.store_)),
          n_nodes_(o.n_nodes_), n_alloc_(o.n_alloc_),
          n_dims_(o.n_dims_), n_links_(o.n_links_),
          n_fk_calls_(o.n_fk_calls_), stride_(o.stride_)
    {
        o.fd_ = -1; o.mapped_ = nullptr; o.file_size_ = 0;
        setup_resize_callback();  // re-hook to this instance
    }
    HCacheFile& operator=(HCacheFile&& o) noexcept {
        if (this != &o) {
            close();
            fd_ = o.fd_; mapped_ = o.mapped_; file_size_ = o.file_size_;
            store_ = std::move(o.store_);
            n_nodes_ = o.n_nodes_; n_alloc_ = o.n_alloc_;
            n_dims_ = o.n_dims_; n_links_ = o.n_links_;
            n_fk_calls_ = o.n_fk_calls_; stride_ = o.stride_;
            o.fd_ = -1; o.mapped_ = nullptr; o.file_size_ = 0;
            setup_resize_callback();  // re-hook to this instance
        }
        return *this;
    }
    HCacheFile(const HCacheFile&) = delete;
    HCacheFile& operator=(const HCacheFile&) = delete;

    // Open existing HCACHE file (mmap read-write)
    static HCacheFile open(const std::string& path);

    // Create new HCACHE file
    static HCacheFile create(const std::string& path,
                              int n_dims, int n_links,
                              const JointLimits& limits,
                              const std::string& fingerprint = "",
                              int initial_cap = 64);

    // Get the NodeStore backed by this file
    NodeStore& node_store() { return store_; }
    const NodeStore& node_store() const { return store_; }

    // Sync to disk
    void flush();

    // Close and unmap
    void close();

    // Set up resize callback (call after move or construction)
    void setup_resize_callback();

    // ── Header fields ────────────────────────────────────────────────────
    int64_t n_nodes() const { return n_nodes_; }
    int64_t n_alloc() const { return n_alloc_; }
    int n_dims() const { return n_dims_; }
    int n_links() const { return n_links_; }
    int64_t n_fk_calls() const { return n_fk_calls_; }

    void set_n_fk_calls(int64_t v) { n_fk_calls_ = v; write_header(); }

    bool is_open() const { return mapped_ != nullptr; }

private:
    int fd_ = -1;
    char* mapped_ = nullptr;
    size_t file_size_ = 0;

    NodeStore store_;

    int64_t n_nodes_ = 0;
    int64_t n_alloc_ = 0;
    int n_dims_ = 0;
    int n_links_ = 0;
    int64_t n_fk_calls_ = 0;
    int stride_ = 0;

    void read_header();
    void write_header();
    void grow_file(int new_cap);
};

} // namespace sbf

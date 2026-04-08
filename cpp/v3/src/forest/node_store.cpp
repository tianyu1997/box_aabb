// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v3 — NodeStore implementation
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/forest/node_store.h"

#include <algorithm>
#include <fstream>
#include <stdexcept>

namespace sbf {
namespace forest {

// ─────────────────────────────────────────────────────────────────────────
//  Construction
// ─────────────────────────────────────────────────────────────────────────
NodeStore::NodeStore(int n_dims, int n_active_links, int initial_capacity)
    : n_dims_(n_dims)
    , n_active_links_(n_active_links)
    , n_nodes_(0)
    , capacity_(initial_capacity)
    , stride_(compute_stride(n_active_links))
{
    buf_.resize(static_cast<size_t>(capacity_) * stride_, 0);
    occupied_.resize(capacity_, 0);
    forest_id_.resize(capacity_, -1);
    subtree_occ_.resize(capacity_, 0);
    source_quality_.resize(capacity_, 0);
}

// ─────────────────────────────────────────────────────────────────────────
//  Allocation
// ─────────────────────────────────────────────────────────────────────────
int NodeStore::alloc_node() {
    if (n_nodes_ >= capacity_)
        ensure_capacity(capacity_ * 2);
    int idx = n_nodes_++;
    init_node(idx);
    return idx;
}

void NodeStore::init_node(int idx) {
    char* p = node_ptr(idx);
    std::memset(p, 0, stride_);
    // Set defaults: leaf (-1 for left/right), no parent, depth 0
    *reinterpret_cast<int32_t*>(p + OFF_LEFT)   = -1;
    *reinterpret_cast<int32_t*>(p + OFF_RIGHT)  = -1;
    *reinterpret_cast<int32_t*>(p + OFF_PARENT) = -1;
    occupied_[idx] = 0;
    forest_id_[idx] = -1;
    subtree_occ_[idx] = 0;
    source_quality_[idx] = 0;
}

void NodeStore::ensure_capacity(int n) {
    if (n <= capacity_) return;
    int new_cap = std::max(n, capacity_ * 2);

    buf_.resize(static_cast<size_t>(new_cap) * stride_, 0);
    occupied_.resize(new_cap, 0);
    forest_id_.resize(new_cap, -1);
    subtree_occ_.resize(new_cap, 0);
    source_quality_.resize(new_cap, 0);

    capacity_ = new_cap;
}

// ─────────────────────────────────────────────────────────────────────────
//  snapshot — deep copy with cleared occupation
// ─────────────────────────────────────────────────────────────────────────
NodeStore NodeStore::snapshot() const {
    NodeStore copy;
    copy.n_dims_         = n_dims_;
    copy.n_active_links_ = n_active_links_;
    copy.n_nodes_        = n_nodes_;
    copy.capacity_       = capacity_;
    copy.stride_         = stride_;
    copy.buf_            = buf_;              // deep copy of flat node buffer

    // Fresh occupation state for the snapshot
    copy.occupied_.assign(capacity_, 0);
    copy.forest_id_.assign(capacity_, -1);
    copy.subtree_occ_.assign(capacity_, 0);
    copy.source_quality_ = source_quality_;  // preserve cached quality
    return copy;
}

// ─────────────────────────────────────────────────────────────────────────
//  copy_node_from — raw node record copy between stores
// ─────────────────────────────────────────────────────────────────────────
void NodeStore::copy_node_from(const NodeStore& src, int src_idx, int dst_idx) {
    ensure_capacity(dst_idx + 1);
    std::memcpy(node_ptr(dst_idx), src.node_ptr(src_idx), stride_);
    occupied_[dst_idx]    = src.occupied_[src_idx];
    forest_id_[dst_idx]   = src.forest_id_[src_idx];
    subtree_occ_[dst_idx] = src.subtree_occ_[src_idx];
    source_quality_[dst_idx] = src.source_quality_[src_idx];
    // Update n_nodes_ if we're filling beyond current count
    if (dst_idx >= n_nodes_)
        n_nodes_ = dst_idx + 1;
}

// ─────────────────────────────────────────────────────────────────────────
//  Persistence — HCACHE02
// ─────────────────────────────────────────────────────────────────────────
void NodeStore::save(const std::string& path) const {
    std::ofstream f(path, std::ios::binary);
    if (!f)
        throw std::runtime_error("NodeStore::save: cannot open " + path);

    // ── Header (4096 bytes) ──
    char header[HEADER_SIZE] = {};
    std::memcpy(header + 0, MAGIC, 8);                                  // [0-8]   magic
    *reinterpret_cast<int32_t*>(header + 8)   = 2;                      // [8-12]  version
    *reinterpret_cast<int64_t*>(header + 12)  = n_nodes_;               // [12-20] n_nodes
    *reinterpret_cast<int64_t*>(header + 20)  = capacity_;              // [20-28] n_alloc
    *reinterpret_cast<int32_t*>(header + 28)  = n_dims_;                // [28-32] n_dims
    *reinterpret_cast<int32_t*>(header + 32)  = n_active_links_;        // [32-36] n_links
    *reinterpret_cast<int64_t*>(header + 36)  = 0;                      // [36-44] n_fk_calls (stats)
    *reinterpret_cast<int32_t*>(header + 44)  = stride_;                // [44-48] stride

    f.write(header, HEADER_SIZE);

    // ── Node data ──
    if (n_nodes_ > 0)
        f.write(buf_.data(), static_cast<std::streamsize>(n_nodes_) * stride_);
}

void NodeStore::load(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f)
        throw std::runtime_error("NodeStore::load: cannot open " + path);

    // ── Read header ──
    char header[HEADER_SIZE];
    f.read(header, HEADER_SIZE);
    if (!f)
        throw std::runtime_error("NodeStore::load: truncated header");

    if (std::memcmp(header, MAGIC, 8) != 0)
        throw std::runtime_error("NodeStore::load: invalid HCACHE02 magic");

    int version = *reinterpret_cast<int32_t*>(header + 8);
    if (version != 2)
        throw std::runtime_error("NodeStore::load: unsupported version " + std::to_string(version));

    n_nodes_        = static_cast<int>(*reinterpret_cast<int64_t*>(header + 12));
    int n_alloc     = static_cast<int>(*reinterpret_cast<int64_t*>(header + 20));
    n_dims_         = *reinterpret_cast<int32_t*>(header + 28);
    n_active_links_ = *reinterpret_cast<int32_t*>(header + 32);
    stride_         = *reinterpret_cast<int32_t*>(header + 44);

    // Sanity check
    int expected_stride = compute_stride(n_active_links_);
    if (stride_ != expected_stride)
        throw std::runtime_error("NodeStore::load: stride mismatch");

    capacity_ = std::max(n_alloc, n_nodes_);

    // ── Read node data ──
    buf_.resize(static_cast<size_t>(capacity_) * stride_, 0);
    if (n_nodes_ > 0)
        f.read(buf_.data(), static_cast<std::streamsize>(n_nodes_) * stride_);

    // ── Init auxiliary arrays ──
    occupied_.assign(capacity_, 0);
    forest_id_.assign(capacity_, -1);
    subtree_occ_.assign(capacity_, 0);
    source_quality_.assign(capacity_, 0);
}

} // namespace forest
} // namespace sbf

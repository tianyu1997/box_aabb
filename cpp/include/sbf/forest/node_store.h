// SafeBoxForest — NodeStore: mmap-backed SoA node storage
// Binary-compatible with v4 Python HCACHE02 format
#pragma once

#include "sbf/core/types.h"
#include <cstdint>
#include <cstring>
#include <functional>
#include <vector>

namespace sbf {

// ─── Node field offsets (bytes) ─────────────────────────────────────────────
// Must match v4 _hier_layout.py exactly for HCACHE02 compatibility
namespace node_layout {
    constexpr int OFF_LEFT     = 0;   // int32: left child index (-1 = leaf)
    constexpr int OFF_RIGHT    = 4;   // int32: right child index
    constexpr int OFF_PARENT   = 8;   // int32: parent index (-1 = root)
    constexpr int OFF_DEPTH    = 12;  // int32: tree depth
    constexpr int OFF_SPLIT    = 16;  // float64: split value
    constexpr int OFF_HAS_AABB = 24;  // uint8: has AABB computed
    constexpr int OFF_DIRTY    = 25;  // uint8: dirty flag
    constexpr int OFF_AABB     = 28;  // float32[n_links*6]: link AABBs
    constexpr int TOPO_SIZE    = 28;  // fixed topology size before AABB

    // Compute stride aligned to 64-byte cache lines
    inline int compute_stride(int n_links) {
        int raw = TOPO_SIZE + n_links * 6 * static_cast<int>(sizeof(float));
        return ((raw + 63) / 64) * 64;
    }
}

// ─── NodeStore ──────────────────────────────────────────────────────────────
class NodeStore {
public:
    using ResizeCallback = std::function<void(int old_cap, int new_cap)>;

    NodeStore() = default;

    // Construct from parameters (allocates internal buffer)
    NodeStore(int n_links, int n_dims, int initial_cap = 64);

    // Construct from external buffer (mmap)
    NodeStore(char* external_buf, int n_links, int n_dims, int cap, int next_idx);

    ~NodeStore() = default;

    // Move semantics (needed because base_ is a raw pointer into buf_)
    NodeStore(NodeStore&& o) noexcept
        : occupied(std::move(o.occupied)),
          subtree_occ(std::move(o.subtree_occ)),
          subtree_occ_vol(std::move(o.subtree_occ_vol)),
          forest_id(std::move(o.forest_id)),
          buf_(std::move(o.buf_)), base_(o.base_), stride_(o.stride_),
          n_links_(o.n_links_), n_dims_(o.n_dims_), aabb_floats_(o.aabb_floats_),
          cap_(o.cap_), next_idx_(o.next_idx_), n_active_links_(o.n_active_links_),
          resize_cb_(std::move(o.resize_cb_))
    {
        // If base pointed into the old buf_, remap to new buf_
        if (!buf_.empty()) base_ = buf_.data();
        std::memcpy(active_link_map_, o.active_link_map_, sizeof(active_link_map_));
        o.base_ = nullptr;
        o.cap_ = 0; o.next_idx_ = 0;
    }

    NodeStore& operator=(NodeStore&& o) noexcept {
        if (this != &o) {
            buf_ = std::move(o.buf_);
            base_ = o.base_;
            stride_ = o.stride_;
            n_links_ = o.n_links_;
            n_dims_ = o.n_dims_;
            aabb_floats_ = o.aabb_floats_;
            cap_ = o.cap_;
            next_idx_ = o.next_idx_;
            n_active_links_ = o.n_active_links_;
            resize_cb_ = std::move(o.resize_cb_);
            occupied = std::move(o.occupied);
            subtree_occ = std::move(o.subtree_occ);
            subtree_occ_vol = std::move(o.subtree_occ_vol);
            forest_id = std::move(o.forest_id);
            if (!buf_.empty()) base_ = buf_.data();
            std::memcpy(active_link_map_, o.active_link_map_, sizeof(active_link_map_));
            o.base_ = nullptr;
            o.cap_ = 0; o.next_idx_ = 0;
        }
        return *this;
    }

    // No copy (buf_ can be large)
    NodeStore(const NodeStore&) = delete;
    NodeStore& operator=(const NodeStore&) = delete;

    // ── Node allocation ──────────────────────────────────────────────────
    int alloc_node(int parent, int depth);
    void ensure_capacity(int needed);

    // ── Inline field accessors (zero overhead) ───────────────────────────
    char* node_ptr(int idx) const {
        return base_ + static_cast<int64_t>(idx) * stride_;
    }

    // Int32 field access
    int32_t get_i32(const char* node, int offset) const {
        int32_t v;
        std::memcpy(&v, node + offset, sizeof(v));
        return v;
    }
    void set_i32(char* node, int offset, int32_t val) {
        std::memcpy(node + offset, &val, sizeof(val));
        if (offset != node_layout::OFF_DIRTY)
            *(node + node_layout::OFF_DIRTY) = 1;
    }

    // Float64 field access
    double get_f64(const char* node, int offset) const {
        double v;
        std::memcpy(&v, node + offset, sizeof(v));
        return v;
    }
    void set_f64(char* node, int offset, double val) {
        std::memcpy(node + offset, &val, sizeof(val));
        *(node + node_layout::OFF_DIRTY) = 1;
    }

    // Uint8 field access
    uint8_t get_u8(const char* node, int offset) const {
        return static_cast<uint8_t>(*(node + offset));
    }
    void set_u8(char* node, int offset, uint8_t val) {
        *(node + offset) = static_cast<char>(val);
        if (offset != node_layout::OFF_DIRTY)
            *(node + node_layout::OFF_DIRTY) = 1;
    }

    // AABB data pointer
    float* aabb_ptr(char* node) const {
        return reinterpret_cast<float*>(node + node_layout::OFF_AABB);
    }
    const float* aabb_ptr(const char* node) const {
        return reinterpret_cast<const float*>(node + node_layout::OFF_AABB);
    }

    // ── Convenience node field access ────────────────────────────────────
    int32_t left(int idx)   const { return get_i32(node_ptr(idx), node_layout::OFF_LEFT); }
    int32_t right(int idx)  const { return get_i32(node_ptr(idx), node_layout::OFF_RIGHT); }
    int32_t parent(int idx) const { return get_i32(node_ptr(idx), node_layout::OFF_PARENT); }
    int32_t depth(int idx)  const { return get_i32(node_ptr(idx), node_layout::OFF_DEPTH); }
    double  split_val(int idx) const { return get_f64(node_ptr(idx), node_layout::OFF_SPLIT); }
    bool    has_aabb(int idx)  const { return get_u8(node_ptr(idx), node_layout::OFF_HAS_AABB) != 0; }
    bool    dirty(int idx)     const { return get_u8(node_ptr(idx), node_layout::OFF_DIRTY) != 0; }

    void set_left(int idx, int32_t v)   { auto* n = node_ptr(idx); set_i32(n, node_layout::OFF_LEFT, v);   set_u8(n, node_layout::OFF_DIRTY, 1); }
    void set_right(int idx, int32_t v)  { auto* n = node_ptr(idx); set_i32(n, node_layout::OFF_RIGHT, v);  set_u8(n, node_layout::OFF_DIRTY, 1); }
    void set_parent(int idx, int32_t v) { auto* n = node_ptr(idx); set_i32(n, node_layout::OFF_PARENT, v); set_u8(n, node_layout::OFF_DIRTY, 1); }
    void set_depth(int idx, int32_t v)  { auto* n = node_ptr(idx); set_i32(n, node_layout::OFF_DEPTH, v);  set_u8(n, node_layout::OFF_DIRTY, 1); }
    void set_split(int idx, double v)   { auto* n = node_ptr(idx); set_f64(n, node_layout::OFF_SPLIT, v);  set_u8(n, node_layout::OFF_DIRTY, 1); }
    void set_has_aabb(int idx, bool v)  { auto* n = node_ptr(idx); set_u8(n, node_layout::OFF_HAS_AABB, v ? 1 : 0); set_u8(n, node_layout::OFF_DIRTY, 1); }
    void set_dirty(int idx, bool v)     { set_u8(node_ptr(idx), node_layout::OFF_DIRTY, v ? 1 : 0); }

    float* aabb(int idx)             { return aabb_ptr(node_ptr(idx)); }
    const float* aabb(int idx) const { return aabb_ptr(node_ptr(idx)); }

    bool is_leaf(int idx) const { return left(idx) == -1; }

    // ── Subtree collision recursive ──────────────────────────────────────
    // Check if any descendant at `remaining_depth` levels down collides
    bool subtree_collide_recursive(int idx, const float* obs_flat,
                                   int n_obs, int remaining_depth) const;

    // ── AABB union / refine helpers ──────────────────────────────────────
    // out = union(a, b) for n_links * 6 floats
    void union_aabb(const float* a, const float* b, float* out) const;
    // Refine: tgt = intersect(tgt, src) — shrinks tgt
    void refine_aabb(float* tgt, const float* src) const;
    // ── Dirty flag management (for incremental save) ─────────────────────
    // Mark AABB data as modified (auto-set by set_* methods, also call after memcpy to aabb())
    void mark_dirty(int idx) { set_u8(node_ptr(idx), node_layout::OFF_DIRTY, 1); }
    // Iterate all dirty node indices
    std::vector<int> iter_dirty() const;
    // Clear all dirty flags
    void clear_all_dirty();
    // ── Auxiliary arrays (not persisted in HCACHE) ───────────────────────
    std::vector<uint8_t>  occupied;         // per-node occupied flag
    std::vector<int32_t>  subtree_occ;      // subtree occupied count
    std::vector<double>   subtree_occ_vol;  // subtree occupied volume fraction
    std::vector<int32_t>  forest_id;        // forest box ID (-1 = unoccupied)

    void resize_aux(int cap);

    // ── Properties ───────────────────────────────────────────────────────
    int stride()    const { return stride_; }
    int n_links()   const { return n_links_; }
    int n_dims()    const { return n_dims_; }
    int capacity()  const { return cap_; }
    int next_idx()  const { return next_idx_; }
    int aabb_floats() const { return aabb_floats_; }
    char* base()    const { return base_; }

    void set_next_idx(int v) { next_idx_ = v; }
    void set_resize_callback(ResizeCallback cb) { resize_cb_ = std::move(cb); }

    // Re-attach to external buffer (e.g. after mmap remap)
    void attach_buffer(char* new_base, int new_cap);

    // Raw buffer access (for save/load)
    const char* raw_buffer() const { return base_; }
    size_t raw_buffer_bytes() const { return static_cast<size_t>(next_idx_) * stride_; }

    // Active link map
    void set_active_link_map(const int* map, int n);
    const int* active_link_map() const { return active_link_map_; }
    int n_active_links() const { return n_active_links_; }

private:
    std::vector<char> buf_;       // internal buffer (when not using external mmap)
    char* base_ = nullptr;
    int stride_      = 0;
    int n_links_     = 0;
    int n_dims_      = 0;
    int aabb_floats_ = 0;
    int cap_         = 0;
    int next_idx_    = 0;

    int active_link_map_[MAX_LINKS] = {};
    int n_active_links_ = 0;

    ResizeCallback resize_cb_;

    void init_node(int idx, int parent, int depth);
};

} // namespace sbf

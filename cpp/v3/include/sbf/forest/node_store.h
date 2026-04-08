// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v3 — NodeStore: SoA flat-buffer for KD-tree nodes
//  Module: sbf::forest
//
//  Stores tree structure + per-link AABBs in a cache-line aligned flat
//  buffer.  Compatible with v1 HCACHE02 binary format.
//
//  Per-node layout (fixed stride, 64-byte aligned):
//    [0-3]   int32   left         (-1 = leaf)
//    [4-7]   int32   right
//    [8-11]  int32   parent       (-1 = root)
//    [12-15] int32   depth
//    [16-23] float64 split        (midpoint value for split dimension)
//    [24]    uint8   has_aabb     (1 = link AABBs stored)
//    [25]    uint8   dirty        (for incremental save)
//    [28..]  float[n_active_links × 6]  per-link AABBs
//                    [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z] per link
//
//  Stride = ceil((28 + n_active_links × 24) / 64) × 64
//
//  Auxiliary arrays (NOT persisted in HCACHE):
//    occupied[]      — per-node occupation flag
//    forest_id[]     — maps node → forest box ID (-1 = unoccupied)
//    subtree_occ[]   — count of occupied descendants
//
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace sbf {
namespace forest {

class NodeStore {
public:
    NodeStore() = default;

    /// Construct with metadata and initial capacity.
    NodeStore(int n_dims, int n_active_links, int initial_capacity = 1024);

    // ═════════════════════════════════════════════════════════════════════
    //  Tree structure accessors
    // ═════════════════════════════════════════════════════════════════════

    int32_t  left(int i)     const { return field<int32_t>(i, OFF_LEFT); }
    int32_t  right(int i)    const { return field<int32_t>(i, OFF_RIGHT); }
    int32_t  parent(int i)   const { return field<int32_t>(i, OFF_PARENT); }
    int32_t  depth(int i)    const { return field<int32_t>(i, OFF_DEPTH); }
    double   split(int i)    const { return field<double>(i, OFF_SPLIT); }
    bool     has_aabb(int i) const { return field<uint8_t>(i, OFF_HAS_AABB) != 0; }

    void set_left(int i, int32_t v)     { field<int32_t>(i, OFF_LEFT) = v; }
    void set_right(int i, int32_t v)    { field<int32_t>(i, OFF_RIGHT) = v; }
    void set_parent(int i, int32_t v)   { field<int32_t>(i, OFF_PARENT) = v; }
    void set_depth(int i, int32_t v)    { field<int32_t>(i, OFF_DEPTH) = v; }
    void set_split(int i, double v)     { field<double>(i, OFF_SPLIT) = v; }
    void set_has_aabb(int i, bool v)    { field<uint8_t>(i, OFF_HAS_AABB) = v ? 1 : 0; }

    bool is_leaf(int i) const { return left(i) < 0; }

    // ═════════════════════════════════════════════════════════════════════
    //  Per-link AABBs  [n_active_links × 6 floats per node]
    // ═════════════════════════════════════════════════════════════════════

    const float* link_aabbs(int i) const {
        return reinterpret_cast<const float*>(node_ptr(i) + OFF_AABB);
    }
    float* link_aabbs_mut(int i) {
        return reinterpret_cast<float*>(node_ptr(i) + OFF_AABB);
    }

    // ═════════════════════════════════════════════════════════════════════
    //  Auxiliary arrays (not in HCACHE — transient runtime data)
    // ═════════════════════════════════════════════════════════════════════

    bool    is_occupied(int i)  const { return occupied_[i] != 0; }
    void    set_occupied(int i, bool v) { occupied_[i] = v ? 1 : 0; }
    int32_t forest_id(int i)    const { return forest_id_[i]; }
    void    set_forest_id(int i, int32_t v) { forest_id_[i] = v; }
    int32_t subtree_occ(int i)  const { return subtree_occ_[i]; }
    void    set_subtree_occ(int i, int32_t v) { subtree_occ_[i] = v; }

    /// Endpoint source quality (0=IFK, 1=CritSample, 2=Analytical/GCPC).
    /// Transient runtime field — NOT persisted in HCACHE.
    uint8_t source_quality(int i) const { return source_quality_[i]; }
    void    set_source_quality(int i, uint8_t q) { source_quality_[i] = q; }

    /// Clear all occupation markers and subtree_occ counters.
    void clear_all_occupation() {
        std::fill(occupied_.begin(), occupied_.end(), 0);
        std::fill(forest_id_.begin(), forest_id_.end(), -1);
        std::fill(subtree_occ_.begin(), subtree_occ_.end(), 0);
    }

    // ═════════════════════════════════════════════════════════════════════
    //  Allocation & capacity
    // ═════════════════════════════════════════════════════════════════════

    /// Allocate a new node, grow if needed.  Returns new node index.
    int alloc_node();

    /// Initialize node fields to default (leaf, no AABB, depth=0).
    void init_node(int idx);

    int n_nodes()        const { return n_nodes_; }
    int capacity()       const { return capacity_; }
    int stride()         const { return stride_; }
    int n_dims()         const { return n_dims_; }
    int n_active_links() const { return n_active_links_; }

    void ensure_capacity(int n);

    /// Create a deep copy of tree structure + per-link AABBs,
    /// but with cleared occupation / forest_id / subtree_occ.
    /// Used for LECT warm-start: workers get pre-built tree,
    /// each with independent occupation state.
    NodeStore snapshot() const;

    /// Copy raw node record (buffer bytes + occupation) from another store.
    /// Both stores must have the same stride.  Does NOT remap child/parent
    /// indices — caller must handle index remapping separately.
    void copy_node_from(const NodeStore& src, int src_idx, int dst_idx);

    // ═════════════════════════════════════════════════════════════════════
    //  Persistence (HCACHE02 binary format)
    // ═════════════════════════════════════════════════════════════════════

    void save(const std::string& path) const;
    void load(const std::string& path);

private:
    int n_dims_         = 0;
    int n_active_links_ = 0;
    int n_nodes_        = 0;
    int capacity_       = 0;
    int stride_         = 0;

    std::vector<char>    buf_;           // flat node buffer
    std::vector<uint8_t> occupied_;
    std::vector<int32_t> forest_id_;
    std::vector<int32_t> subtree_occ_;
    std::vector<uint8_t> source_quality_;  // per-node endpoint source quality (transient)

    // ── Field offsets within each node record ────────────────────────────
    static constexpr int OFF_LEFT     = 0;
    static constexpr int OFF_RIGHT    = 4;
    static constexpr int OFF_PARENT   = 8;
    static constexpr int OFF_DEPTH    = 12;
    static constexpr int OFF_SPLIT    = 16;
    static constexpr int OFF_HAS_AABB = 24;
    static constexpr int OFF_DIRTY    = 25;
    // 26-27: padding
    static constexpr int OFF_AABB     = 28;

    // ── Buffer navigation ───────────────────────────────────────────────
    char*       node_ptr(int i)       { return buf_.data() + static_cast<size_t>(i) * stride_; }
    const char* node_ptr(int i) const { return buf_.data() + static_cast<size_t>(i) * stride_; }

    template<typename T>
    T& field(int i, int off) {
        return *reinterpret_cast<T*>(node_ptr(i) + off);
    }
    template<typename T>
    const T& field(int i, int off) const {
        return *reinterpret_cast<const T*>(node_ptr(i) + off);
    }

    /// Compute cache-line aligned stride from n_active_links.
    static int compute_stride(int n_active_links) {
        int raw = OFF_AABB + n_active_links * 6 * static_cast<int>(sizeof(float));
        return ((raw + 63) / 64) * 64;
    }

    // ── HCACHE02 format constants ───────────────────────────────────────
    static constexpr int  HEADER_SIZE = 4096;
    static constexpr char MAGIC[9]    = "HCACHE02";
};

} // namespace forest
} // namespace sbf

// SafeBoxForest — NodeStore implementation
#include "sbf/forest/node_store.h"
#include "sbf/forest/collision.h"
#include <algorithm>
#include <cstring>
#include <stdexcept>

namespace sbf {

NodeStore::NodeStore(int n_links, int n_dims, int initial_cap)
    : n_links_(n_links), n_dims_(n_dims), cap_(initial_cap), next_idx_(0)
{
    stride_ = node_layout::compute_stride(n_links);
    aabb_floats_ = n_links * 6;

    buf_.resize(static_cast<size_t>(cap_) * stride_, 0);
    base_ = buf_.data();

    // Default active link map: identity
    n_active_links_ = n_links;
    for (int i = 0; i < n_links; ++i)
        active_link_map_[i] = i;

    resize_aux(cap_);

    // Initialize root node
    alloc_node(-1, 0);  // root: parent=-1, depth=0
}

NodeStore::NodeStore(char* external_buf, int n_links, int n_dims, int cap, int next_idx)
    : base_(external_buf), n_links_(n_links), n_dims_(n_dims),
      cap_(cap), next_idx_(next_idx)
{
    stride_ = node_layout::compute_stride(n_links);
    aabb_floats_ = n_links * 6;

    n_active_links_ = n_links;
    for (int i = 0; i < n_links; ++i)
        active_link_map_[i] = i;

    resize_aux(cap_);
}

int NodeStore::alloc_node(int parent, int depth) {
    ensure_capacity(next_idx_ + 1);
    int idx = next_idx_++;
    init_node(idx, parent, depth);
    return idx;
}

void NodeStore::init_node(int idx, int parent, int depth) {
    char* node = node_ptr(idx);
    std::memset(node, 0, stride_);
    set_i32(node, node_layout::OFF_LEFT, -1);
    set_i32(node, node_layout::OFF_RIGHT, -1);
    set_i32(node, node_layout::OFF_PARENT, parent);
    set_i32(node, node_layout::OFF_DEPTH, depth);
    set_f64(node, node_layout::OFF_SPLIT, 0.0);
    set_u8(node, node_layout::OFF_HAS_AABB, 0);
    set_u8(node, node_layout::OFF_DIRTY, 0);
}

void NodeStore::ensure_capacity(int needed) {
    if (needed <= cap_) return;

    int new_cap = cap_;
    while (new_cap < needed)
        new_cap *= 2;

    if (resize_cb_) {
        // External buffer (mmap) — delegate to callback
        resize_cb_(cap_, new_cap);
    } else {
        // Internal buffer
        buf_.resize(static_cast<size_t>(new_cap) * stride_, 0);
        base_ = buf_.data();
    }

    cap_ = new_cap;
    resize_aux(new_cap);
}

void NodeStore::resize_aux(int cap) {
    occupied.resize(cap, 0);
    subtree_occ.resize(cap, 0);
    subtree_occ_vol.resize(cap, 0.0);
    forest_id.resize(cap, -1);
}

void NodeStore::attach_buffer(char* new_base, int new_cap) {
    base_ = new_base;
    cap_ = new_cap;
    buf_.clear();  // detach internal buffer — now using external
    resize_aux(new_cap);
}

std::vector<int> NodeStore::iter_dirty() const {
    std::vector<int> result;
    for (int i = 0; i < next_idx_; ++i) {
        if (get_u8(node_ptr(i), node_layout::OFF_DIRTY))
            result.push_back(i);
    }
    return result;
}

void NodeStore::clear_all_dirty() {
    for (int i = 0; i < next_idx_; ++i) {
        set_u8(node_ptr(i), node_layout::OFF_DIRTY, 0);
    }
}

void NodeStore::set_active_link_map(const int* map, int n) {
    n_active_links_ = std::min(n, MAX_LINKS);
    for (int i = 0; i < n_active_links_; ++i)
        active_link_map_[i] = map[i];
}

// ─── Subtree collision recursive ────────────────────────────────────────────
bool NodeStore::subtree_collide_recursive(int idx, const float* obs_compact,
                                          int n_obs, int n_slots, int remaining_depth) const {
    char* node = node_ptr(idx);

    if (remaining_depth == 0 || get_i32(node, node_layout::OFF_LEFT) == -1) {
        // Leaf or target depth reached: check this node's AABB
        if (!get_u8(node, node_layout::OFF_HAS_AABB))
            return true;  // Conservative: no AABB → assume collision
        return aabbs_collide_obs(aabb_ptr(node), n_slots, obs_compact, n_obs);
    }

    // Recurse into children
    int left_idx = get_i32(node, node_layout::OFF_LEFT);
    int right_idx = get_i32(node, node_layout::OFF_RIGHT);

    if (subtree_collide_recursive(left_idx, obs_compact, n_obs, n_slots, remaining_depth - 1))
        return true;
    if (right_idx >= 0 &&
        subtree_collide_recursive(right_idx, obs_compact, n_obs, n_slots, remaining_depth - 1))
        return true;

    return false;
}

// ─── AABB helpers ───────────────────────────────────────────────────────────
void NodeStore::union_aabb(const float* a, const float* b, float* out) const {
    // For each AABB slot (links + EE spheres): lo = min, hi = max
    int n_slots = n_links_;  // total AABB slots
    for (int i = 0; i < n_slots; ++i) {
        int off = i * 6;
        out[off + 0] = std::min(a[off + 0], b[off + 0]);  // lo_x
        out[off + 1] = std::min(a[off + 1], b[off + 1]);  // lo_y
        out[off + 2] = std::min(a[off + 2], b[off + 2]);  // lo_z
        out[off + 3] = std::max(a[off + 3], b[off + 3]);  // hi_x
        out[off + 4] = std::max(a[off + 4], b[off + 4]);  // hi_y
        out[off + 5] = std::max(a[off + 5], b[off + 5]);  // hi_z
    }
}

void NodeStore::refine_aabb(float* tgt, const float* src) const {
    // Intersect: lo = max(tgt_lo, src_lo), hi = min(tgt_hi, src_hi)
    int n_slots = n_links_;  // total AABB slots
    for (int i = 0; i < n_slots; ++i) {
        int off = i * 6;
        tgt[off + 0] = std::max(tgt[off + 0], src[off + 0]);
        tgt[off + 1] = std::max(tgt[off + 1], src[off + 1]);
        tgt[off + 2] = std::max(tgt[off + 2], src[off + 2]);
        tgt[off + 3] = std::min(tgt[off + 3], src[off + 3]);
        tgt[off + 4] = std::min(tgt[off + 4], src[off + 4]);
        tgt[off + 5] = std::min(tgt[off + 5], src[off + 5]);
    }
}

// Forward decl needed by aabbs_collide_obs used in subtree_collide_recursive
// Already defined in collision.cpp, but we need it here too for the inline check
// The actual function is in collision.cpp, and the header declares it.

} // namespace sbf

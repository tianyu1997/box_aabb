// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — NodeStore: SoA flat buffer for KD-tree nodes
//  Module: sbf::forest
//
//  Pure tree-structure + occupation store.
//  Per-link iAABBs and source quality have been moved to LECT's
//  dual-channel ChannelData storage; NodeStore no longer stores them.
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace sbf {
namespace forest {

class NodeStore {
public:
    NodeStore() = default;

    /// Initialize with dimensions and active link count
    void init(int n_dims, int n_active_links, int initial_cap) {
        n_dims_ = n_dims;
        n_active_ = n_active_links;
        n_nodes_ = 0;
        capacity_ = 0;
        ensure_capacity(initial_cap);
    }

    int n_nodes()        const { return n_nodes_; }
    int n_dims()         const { return n_dims_; }
    int n_active_links() const { return n_active_; }
    int capacity()       const { return capacity_; }

    // ── KD-tree structure ───────────────────────────────────────────────
    int  left(int i)        const { return left_[i]; }
    int  right(int i)       const { return right_[i]; }
    int  parent(int i)      const { return parent_[i]; }
    int  depth(int i)       const { return depth_[i]; }
    double split(int i)     const { return split_[i]; }
    bool is_leaf(int i)     const { return left_[i] < 0; }

    void set_left(int i, int left_idx)     { left_[i] = left_idx; }
    void set_right(int i, int right_idx)   { right_[i] = right_idx; }
    void set_parent(int i, int parent_idx) { parent_[i] = parent_idx; }
    void set_depth(int i, int d)           { depth_[i] = d; }
    void set_split(int i, double val)      { split_[i] = val; }

    // ── Occupation ──────────────────────────────────────────────────────
    bool is_occupied(int i) const { return forest_id_[i] >= 0; }
    int  forest_id(int i)   const { return forest_id_[i]; }
    void set_occupied(int i, int fid) { forest_id_[i] = fid; }
    void set_unoccupied(int i) { forest_id_[i] = -1; }
    void clear_all_occupation() {
        std::fill(forest_id_.begin(), forest_id_.begin() + n_nodes_, -1);
        std::fill(subtree_occ_.begin(), subtree_occ_.begin() + n_nodes_, 0);
    }

    // ── Subtree occupation counter ──────────────────────────────────────
    int32_t subtree_occ(int i)  const { return subtree_occ_[i]; }
    void    set_subtree_occ(int i, int32_t v) { subtree_occ_[i] = v; }

    // ── Capacity management ─────────────────────────────────────────────
    void ensure_capacity(int min_cap) {
        if (min_cap <= capacity_) return;

        int new_cap = std::max(min_cap, capacity_ * 2);
        if (new_cap < 64) new_cap = 64;

        left_.resize(new_cap, -1);
        right_.resize(new_cap, -1);
        parent_.resize(new_cap, -1);
        depth_.resize(new_cap, 0);
        split_.resize(new_cap, 0.0);
        forest_id_.resize(new_cap, -1);
        subtree_occ_.resize(new_cap, 0);

        capacity_ = new_cap;
    }

    int alloc_node() {
        int idx = n_nodes_++;
        if (idx >= capacity_)
            ensure_capacity(idx + 1);

        left_[idx] = -1;
        right_[idx] = -1;
        parent_[idx] = -1;
        depth_[idx] = 0;
        split_[idx] = 0.0;
        forest_id_[idx] = -1;
        subtree_occ_[idx] = 0;
        return idx;
    }

    // ── Deep-copy / node transfer ───────────────────────────────────────
    /// Deep copy with cleared occupation state.
    NodeStore snapshot() const {
        NodeStore copy;
        copy.n_dims_ = n_dims_;
        copy.n_active_ = n_active_;
        copy.n_nodes_ = n_nodes_;
        copy.capacity_ = capacity_;
        copy.left_ = left_;
        copy.right_ = right_;
        copy.parent_ = parent_;
        copy.depth_ = depth_;
        copy.split_ = split_;
        copy.forest_id_.assign(capacity_, -1);
        copy.subtree_occ_.assign(capacity_, 0);
        return copy;
    }

    /// Copy a single node record (tree structure + occupation) from src.
    void copy_node_from(const NodeStore& src, int src_idx, int dst_idx) {
        ensure_capacity(dst_idx + 1);

        left_[dst_idx] = src.left_[src_idx];
        right_[dst_idx] = src.right_[src_idx];
        parent_[dst_idx] = src.parent_[src_idx];
        depth_[dst_idx] = src.depth_[src_idx];
        split_[dst_idx] = src.split_[src_idx];
        forest_id_[dst_idx] = src.forest_id_[src_idx];
        subtree_occ_[dst_idx] = src.subtree_occ_[src_idx];

        if (dst_idx >= n_nodes_)
            n_nodes_ = dst_idx + 1;
    }

    // ── Persistence (HCACHE04 binary format — tree-only) ────────────────
    void save(const std::string& path) const {
        std::ofstream f(path, std::ios::binary);
        if (!f) throw std::runtime_error("NodeStore::save: cannot open " + path);

        static constexpr int HCACHE_HEADER_SIZE = 4096;
        static constexpr uint32_t HCACHE_MAGIC_V4 = 0x34304348;

        char header[HCACHE_HEADER_SIZE] = {};
        std::memcpy(header + 0, &HCACHE_MAGIC_V4, 4);
        *reinterpret_cast<int32_t*>(header + 4) = 4;
        *reinterpret_cast<int32_t*>(header + 8) = n_nodes_;
        *reinterpret_cast<int32_t*>(header + 12) = capacity_;
        *reinterpret_cast<int32_t*>(header + 16) = n_dims_;
        *reinterpret_cast<int32_t*>(header + 20) = n_active_;
        f.write(header, HCACHE_HEADER_SIZE);

        auto write_vec = [&](const auto& vec, int n) {
            f.write(reinterpret_cast<const char*>(vec.data()),
                    static_cast<std::streamsize>(n) * sizeof(vec[0]));
        };
        write_vec(left_, n_nodes_);
        write_vec(right_, n_nodes_);
        write_vec(parent_, n_nodes_);
        write_vec(depth_, n_nodes_);
        write_vec(split_, n_nodes_);
    }

    void load(const std::string& path) {
        std::ifstream f(path, std::ios::binary);
        if (!f) throw std::runtime_error("NodeStore::load: cannot open " + path);

        static constexpr int HCACHE_HEADER_SIZE = 4096;
        static constexpr uint32_t HCACHE_MAGIC_V3 = 0x33304348;
        static constexpr uint32_t HCACHE_MAGIC_V4 = 0x34304348;

        char header[HCACHE_HEADER_SIZE];
        f.read(header, HCACHE_HEADER_SIZE);
        if (!f) throw std::runtime_error("NodeStore::load: truncated header");

        uint32_t magic = *reinterpret_cast<uint32_t*>(header);
        bool is_v3 = (magic == HCACHE_MAGIC_V3);
        bool is_v4 = (magic == HCACHE_MAGIC_V4);
        if (!is_v3 && !is_v4)
            throw std::runtime_error("NodeStore::load: invalid HCACHE magic");

        n_nodes_ = *reinterpret_cast<int32_t*>(header + 8);
        int alloc = *reinterpret_cast<int32_t*>(header + 12);
        n_dims_ = *reinterpret_cast<int32_t*>(header + 16);
        n_active_ = *reinterpret_cast<int32_t*>(header + 20);

        capacity_ = 0;
        ensure_capacity(std::max(alloc, n_nodes_));

        auto read_vec = [&](auto& vec, int n) {
            f.read(reinterpret_cast<char*>(vec.data()),
                   static_cast<std::streamsize>(n) * sizeof(vec[0]));
        };
        read_vec(left_, n_nodes_);
        read_vec(right_, n_nodes_);
        read_vec(parent_, n_nodes_);
        read_vec(depth_, n_nodes_);
        read_vec(split_, n_nodes_);

        if (is_v3) {
            std::vector<uint8_t> skip_u8(n_nodes_);
            f.read(reinterpret_cast<char*>(skip_u8.data()), n_nodes_);
            f.read(reinterpret_cast<char*>(skip_u8.data()), n_nodes_);
            size_t link_bytes = static_cast<size_t>(n_nodes_) * n_active_ * 6 * sizeof(float);
            f.seekg(static_cast<std::streamoff>(link_bytes), std::ios_base::cur);
        }

        std::fill(forest_id_.begin(), forest_id_.begin() + capacity_, -1);
        std::fill(subtree_occ_.begin(), subtree_occ_.begin() + capacity_, 0);
    }

private:
    // ── KD-tree structure SoA ───────────────────────────────────────────
    std::vector<int>    left_;         // left child (-1 = leaf)
    std::vector<int>    right_;        // right child
    std::vector<int>    parent_;       // parent (-1 = root)
    std::vector<int>    depth_;        // node depth
    std::vector<double> split_;        // split value

    // ── Occupation ──────────────────────────────────────────────────────
    std::vector<int> forest_id_;       // -1 = unoccupied
    std::vector<int32_t> subtree_occ_; // descendant occupation count

    // ── Metadata ────────────────────────────────────────────────────────
    int n_nodes_   = 0;
    int n_dims_    = 0;
    int n_active_  = 0;
    int capacity_  = 0;
};

} // namespace forest
} // namespace sbf

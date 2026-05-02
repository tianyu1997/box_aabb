// SafeBoxForest v6 — LECT snapshot / transplant
#include <sbf/lect/lect.h>
#include <sbf/core/log.h>

#include <algorithm>
#include <cstring>

namespace sbf {

LECT LECT::snapshot() const {
    // R1: materialize all deferred grid lookups so the snapshot carries
    // a complete grid view (workers won't have access to our pending state).
    materialize_all_pending_grids_();

    LECT copy;
    // Config
    copy.robot_           = robot_;
    copy.ep_config_       = ep_config_;
    copy.env_config_      = env_config_;
    copy.root_intervals_  = root_intervals_;
    copy.ep_stride_       = ep_stride_;
    copy.liaabb_stride_   = liaabb_stride_;
    copy.n_dims_          = n_dims_;
    copy.n_active_links_  = n_active_links_;
    copy.n_nodes_         = n_nodes_;
    copy.capacity_        = capacity_;
    copy.split_order_     = split_order_;
    copy.z4_active_       = z4_active_;
    copy.symmetry_q0_     = symmetry_q0_;
    copy.radii_f_         = radii_f_;
    copy.root_fk_         = root_fk_;
    copy.depth_split_dim_cache_ = depth_split_dim_cache_;
    copy.cache_mgr_       = cache_mgr_;   // V6: share persistent cache (mmap, thread-safe)

    // Dual-channel flat buffers (deep copy)
    // If using mmap, clone the mmap so the worker gets its own independent
    // COW mapping.  Only copy the offset vector (new nodes), not loaded data.
    if (use_mmap_) {
        LectMmap cloned = mmap_.clone();
        if (cloned.is_open()) {
            // Fast path: clone mmap — NO 350MB copy for loaded nodes
            for (int ch = 0; ch < N_CHANNELS; ++ch) {
                copy.channels_[ch].has_data       = channels_[ch].has_data;
                copy.channels_[ch].source_quality = channels_[ch].source_quality;
                copy.channels_[ch].ep_data        = channels_[ch].ep_data; // offset vector (small or empty)
            }
            // Set up the cloned mmap for the copy
            size_t base_off = static_cast<size_t>(mmap_ep_base_ - mmap_.writable_data());
            copy.mmap_ = std::move(cloned);
            copy.mmap_ep_base_     = copy.mmap_.writable_data() + base_off;
            copy.mmap_node_stride_ = mmap_node_stride_;
            copy.nn_loaded_        = nn_loaded_;
            copy.use_mmap_         = true;

            // ── Tree structure from cloned mmap (zero-copy COW) ─────────
            // Master's tree is in vectors (loaded via memcpy).  But the
            // same data lives in the mmap file.  Point the copy's tree at
            // the *cloned* mmap, then patch in any delta nodes that the
            // master expanded since load.  Workers get COW access — reads
            // are free, writes fault only the touched pages.
            // This eliminates ~20 MB of vector deep-copies per snapshot.
            if (mmap_tree_cap_ > 0 && n_nodes_ <= mmap_tree_cap_) {
                uint8_t* tb_w = copy.mmap_.writable_data() + mmap_tree_off_;
                const size_t sc = static_cast<size_t>(mmap_tree_cap_);

                // Alignment check: split_val_ is double*, needs 8-byte align
                const size_t sv_byte_off = 5 * sc * 4;
                if ((sv_byte_off & 7) == 0) {
                    copy.left_.set_mmap(reinterpret_cast<int*>(tb_w + 0 * sc * 4));
                    copy.right_.set_mmap(reinterpret_cast<int*>(tb_w + 1 * sc * 4));
                    copy.parent_.set_mmap(reinterpret_cast<int*>(tb_w + 2 * sc * 4));
                    copy.depth_.set_mmap(reinterpret_cast<int*>(tb_w + 3 * sc * 4));
                    copy.split_dim_.set_mmap(reinterpret_cast<int*>(tb_w + 4 * sc * 4));
                    copy.split_val_.set_mmap(reinterpret_cast<double*>(tb_w + sv_byte_off));

                    // Copy delta nodes [nn_loaded_, n_nodes_) from vectors → mmap clone
                    const int delta = n_nodes_ - nn_loaded_;
                    if (delta > 0) {
                        const size_t off = static_cast<size_t>(nn_loaded_);
                        const size_t nb  = static_cast<size_t>(delta);
                        std::memcpy(tb_w + 0*sc*4 + off*4, left_.data() + off,      nb * 4);
                        std::memcpy(tb_w + 1*sc*4 + off*4, right_.data() + off,     nb * 4);
                        std::memcpy(tb_w + 2*sc*4 + off*4, parent_.data() + off,    nb * 4);
                        std::memcpy(tb_w + 3*sc*4 + off*4, depth_.data() + off,     nb * 4);
                        std::memcpy(tb_w + 4*sc*4 + off*4, split_dim_.data() + off, nb * 4);
                        std::memcpy(tb_w + sv_byte_off + off*8, split_val_.data() + off, nb * 8);
                    }

                    copy.mmap_tree_off_ = mmap_tree_off_;
                    copy.mmap_tree_cap_ = mmap_tree_cap_;
                } else {
                    // Odd cap → misaligned double*. Fall back to vector copy.
                    copy.left_      = left_;
                    copy.right_     = right_;
                    copy.parent_    = parent_;
                    copy.depth_     = depth_;
                    copy.split_dim_ = split_dim_;
                    copy.split_val_ = split_val_;
                    copy.mmap_tree_off_ = 0;
                    copy.mmap_tree_cap_ = 0;
                }
            } else {
                // Tree exceeds mmap bounds or no mmap tree info — vector copy
                copy.left_      = left_;
                copy.right_     = right_;
                copy.parent_    = parent_;
                copy.depth_     = depth_;
                copy.split_dim_ = split_dim_;
                copy.split_val_ = split_val_;
                copy.mmap_tree_off_ = 0;
                copy.mmap_tree_cap_ = 0;
            }
        } else {
            // Fallback: materialize into full vectors
            for (int ch = 0; ch < N_CHANNELS; ++ch) {
                copy.channels_[ch].has_data       = channels_[ch].has_data;
                copy.channels_[ch].source_quality = channels_[ch].source_quality;
                copy.channels_[ch].ep_data.resize(
                    static_cast<size_t>(capacity_) * ep_stride_, 0.0f);
                for (int i = 0; i < nn_loaded_; ++i) {
                    const float* src = ep_data_read(i, ch);
                    std::memcpy(copy.channels_[ch].ep_data.data()
                                    + static_cast<size_t>(i) * ep_stride_,
                                src,
                                static_cast<size_t>(ep_stride_) * sizeof(float));
                }
                if (n_nodes_ > nn_loaded_) {
                    size_t len = static_cast<size_t>(n_nodes_ - nn_loaded_) * ep_stride_;
                    std::memcpy(copy.channels_[ch].ep_data.data()
                                    + static_cast<size_t>(nn_loaded_) * ep_stride_,
                                channels_[ch].ep_data.data(),
                                len * sizeof(float));
                }
            }
            copy.use_mmap_ = false;
            copy.nn_loaded_ = 0;
            copy.mmap_ep_base_ = nullptr;
            copy.mmap_node_stride_ = 0;

            // Vector-mode tree copy
            copy.left_      = left_;
            copy.right_     = right_;
            copy.parent_    = parent_;
            copy.depth_     = depth_;
            copy.split_dim_ = split_dim_;
            copy.split_val_ = split_val_;
            copy.mmap_tree_off_ = 0;
            copy.mmap_tree_cap_ = 0;
        }
    } else {
        for (int ch = 0; ch < N_CHANNELS; ++ch)
            copy.channels_[ch] = channels_[ch];
        copy.use_mmap_ = false;
        copy.nn_loaded_ = 0;
        copy.mmap_ep_base_ = nullptr;
        copy.mmap_node_stride_ = 0;

        // Vector-mode tree copy
        copy.left_      = left_;
        copy.right_     = right_;
        copy.parent_    = parent_;
        copy.depth_     = depth_;
        copy.split_dim_ = split_dim_;
        copy.split_val_ = split_val_;
        copy.mmap_tree_off_ = 0;
        copy.mmap_tree_cap_ = 0;
    }
    // Skip deep-copying link_iaabb_cache_ (~10MB for warm LECTs).
    // Workers will lazily materialise link iAABBs from EP data on demand.
    // Leave cache empty (lazy alloc on first access) and mark all dirty.
    // copy.link_iaabb_cache_ stays empty — get_link_iaabbs() lazy-allocates.
    copy.link_iaabb_dirty_.assign(capacity_, 1);

    // Per-node grids (deep copy only if materialised)
    if (!node_grids_.empty()) {
        copy.node_grids_     = node_grids_;
        copy.node_grid_meta_ = node_grid_meta_;
    }

    // Tree structure is already set up above (mmap-backed or vector-copied)
    // depending on mmap path.

    // Occupation (start fresh for worker — worker builds its own forest)
    copy.forest_id_.assign(capacity_, -1);
    copy.subtree_occ_.assign(capacity_, 0);

    // Phase A: copy collide-verified cache + obs_generation. Worker shares
    // master's generation so existing per-node verifications remain valid.
    copy.obs_generation_       = obs_generation_;
    copy.collide_state_        = collide_state_;
    copy.collide_verified_gen_ = collide_verified_gen_;

    // Phase U: copy subtree free-volume so worker starts with same view of
    // unexplored regions; worker's own grow operations will then update it
    // independently in its snapshot.
    copy.subtree_free_volume_ = subtree_free_volume_;

    // Record snapshot watermark so transplant_domain can distinguish inherited
    // nodes (< snapshot_base_) from worker-allocated nodes (>= snapshot_base_).
    copy.snapshot_base_ = n_nodes_;

    // Z4 cache — skip deep copy for workers (they rebuild on demand)
    // copy.z4_cache_ = z4_cache_;  // omitted: saves hash-map copy overhead

    return copy;
}

// ══════════════════════════════════════════════════════════════════════════
//  transplant_subtree() — merge worker-expanded nodes back into master
// ══════════════════════════════════════════════════════════════════════════

int LECT::transplant_subtree(const LECT& worker, int snapshot_base,
                             const std::unordered_map<int, int>& id_map) {
    int n_transplanted = 0;
    int worker_n = worker.n_nodes();

    // Iterate nodes expanded by the worker (>= snapshot_base).
    int wi_lo = std::max(snapshot_base, worker.snapshot_base_);

    // Worker nodes with index >= wi_lo are newly expanded by this worker
    for (int wi = wi_lo; wi < worker_n; ++wi) {
        // Ensure we have capacity for this node index in master
        if (wi >= n_nodes_) {
            ensure_capacity(wi + 1);
            // Fill nodes up to wi
            while (n_nodes_ <= wi) {
                int idx = alloc_node();
                (void)idx;
            }
        }

        // Copy tree structure
        left_[wi]      = worker.left_[wi];
        right_[wi]     = worker.right_[wi];
        parent_[wi]    = worker.parent_[wi];
        depth_[wi]     = worker.depth_[wi];
        split_dim_[wi] = worker.split_dim_[wi];
        split_val_[wi] = worker.split_val_[wi];

        // Copy dual-channel envelope data
        for (int ch = 0; ch < N_CHANNELS; ++ch) {
            channels_[ch].has_data[wi] = worker.channels_[ch].has_data[wi];
            channels_[ch].source_quality[wi] = worker.channels_[ch].source_quality[wi];
            std::memcpy(ep_data_write(wi, ch),
                        worker.ep_data_read(wi, ch),
                        static_cast<size_t>(ep_stride_) * sizeof(float));
        }
        // Copy link iAABBs (if master has them materialised)
        if (!link_iaabb_cache_.empty() && !worker.link_iaabb_cache_.empty()) {
            std::memcpy(link_iaabb_cache_.data() + static_cast<size_t>(wi) * liaabb_stride_,
                        worker.link_iaabb_cache_.data() + static_cast<size_t>(wi) * liaabb_stride_,
                        static_cast<size_t>(liaabb_stride_) * sizeof(float));
            link_iaabb_dirty_[wi] = worker.link_iaabb_dirty_[wi];
        } else {
            link_iaabb_dirty_[wi] = 1;  // force recompute
        }

        // Copy per-node grids
        if (wi < static_cast<int>(worker.node_grids_.size())) {
            // R1: ensure worker's pending grid for this node is materialized
            // before copying out (worker side may have deferred lookups).
            worker.materialize_pending_grid_(wi);
            node_grids_[wi]     = worker.node_grids_[wi];
            node_grid_meta_[wi] = worker.node_grid_meta_[wi];
        }

        // Copy occupation with ID remapping
        int wfid = worker.forest_id_[wi];
        if (wfid >= 0) {
            auto it = id_map.find(wfid);
            forest_id_[wi] = (it != id_map.end()) ? it->second : wfid;
        } else {
            forest_id_[wi] = -1;
        }
        subtree_occ_[wi] = worker.subtree_occ_[wi];

        // Phase A: copy worker's collide-verified cache (worker shares
        // master's obs_generation, so verifications carry over).
        if (wi < static_cast<int>(worker.collide_state_.size())) {
            collide_state_[wi]        = worker.collide_state_[wi];
            collide_verified_gen_[wi] = worker.collide_verified_gen_[wi];
        }

        n_transplanted++;
    }

    // Legacy z4_cache_ removed; V6 cache_mgr_ is shared via mmap, no merge needed.

    return n_transplanted;
}

// ══════════════════════════════════════════════════════════════════════════
//  partition_for_seeds — pre-split master LECT until each seed in distinct leaf
// ══════════════════════════════════════════════════════════════════════════

std::vector<int> LECT::partition_for_seeds(
        const std::vector<Eigen::VectorXd>& seeds, int max_extra_splits) {
    const int n = static_cast<int>(seeds.size());
    std::vector<int> leaf_for(n, -1);
    if (n <= 1) {
        if (n == 1) leaf_for[0] = find_leaf_containing(seeds[0]);
        return leaf_for;
    }

    int splits_used = 0;
    while (splits_used < max_extra_splits) {
        // Map current leaf → list of seed indices landing in it.
        std::unordered_map<int, std::vector<int>> leaf_to_seeds;
        for (int i = 0; i < n; ++i) {
            leaf_for[i] = find_leaf_containing(seeds[i]);
            leaf_to_seeds[leaf_for[i]].push_back(i);
        }
        // Find any leaf with >1 seed; split it.
        int conflict_leaf = -1;
        for (const auto& kv : leaf_to_seeds) {
            if (kv.second.size() > 1) { conflict_leaf = kv.first; break; }
        }
        if (conflict_leaf < 0) break;  // all distinct
        int n_before = n_nodes_;
        expand_leaf(conflict_leaf);
        if (n_nodes_ == n_before) {
            // expand_leaf failed (already split or refused); abort.
            SBF_WARN("[LECT-PART] partition_for_seeds: expand_leaf(%d) made no progress; %d seeds still collide", conflict_leaf, static_cast<int>(leaf_to_seeds[conflict_leaf].size()));
            break;
        }
        ++splits_used;
    }
    if (splits_used >= max_extra_splits) {
        SBF_WARN("[LECT-PART] partition_for_seeds: hit max_extra_splits=%d", max_extra_splits);
    }
    // Final per-seed leaf assignment.
    std::vector<int> leaf_final(n);
    for (int i = 0; i < n; ++i) leaf_final[i] = find_leaf_containing(seeds[i]);

    // Promote each seed's domain to the LARGEST ancestor of its leaf that
    // does not contain any other seed's leaf. This gives every worker the
    // biggest exclusive geometric subtree to grow into, while keeping the
    // domains pairwise disjoint (LCA-style partition).
    for (int i = 0; i < n; ++i) {
        int cur = leaf_final[i];
        while (true) {
            int p = parent_[cur];
            if (p < 0) break;  // already at root
            bool other_inside = false;
            for (int j = 0; j < n; ++j) {
                if (j == i) continue;
                if (is_descendant_of(leaf_final[j], p)) {
                    other_inside = true;
                    break;
                }
            }
            if (other_inside) break;
            cur = p;
        }
        leaf_for[i] = cur;
    }
    return leaf_for;
}

// ══════════════════════════════════════════════════════════════════════════
//  transplant_domain — geometric-partition transplant with index remap
// ══════════════════════════════════════════════════════════════════════════
//
// Worker started with master.snapshot(); domain_root_idx was a leaf in
// master at that time. Worker may have split it (turning it internal) and
// allocated descendants. We ship the entire subtree(domain_root_idx) of
// the worker back, remapping any worker-local indices >= snapshot_base
// to fresh master indices. Inherited indices (< snapshot_base) keep their
// position. Disjoint geometric domains across workers guarantee no
// overlap on inherited nodes.

int LECT::transplant_domain(const LECT& worker, int domain_root_idx,
                            const std::unordered_map<int, int>& id_map,
                            std::unordered_map<int, int>& node_remap) {
    if (domain_root_idx < 0 || domain_root_idx >= worker.n_nodes()) return 0;

    const int snap_base = worker.snapshot_base_;

    // BFS the worker subtree at domain_root_idx; allocate fresh master
    // indices for any node >= snap_base, identity-map inherited nodes.
    std::vector<int> bfs;
    bfs.reserve(64);
    bfs.push_back(domain_root_idx);
    node_remap.clear();
    node_remap.reserve(64);
    node_remap[domain_root_idx] = domain_root_idx;

    for (size_t bi = 0; bi < bfs.size(); ++bi) {
        int wi = bfs[bi];
        int wl = worker.left_[wi];
        int wr = worker.right_[wi];
        if (wl >= 0) {
            int mi = (wl < snap_base) ? wl : alloc_node();
            node_remap[wl] = mi;
            bfs.push_back(wl);
        }
        if (wr >= 0) {
            int mi = (wr < snap_base) ? wr : alloc_node();
            node_remap[wr] = mi;
            bfs.push_back(wr);
        }
    }

    // Copy each node's data, remapping tree pointers.
    int n_shipped = 0;
    for (int wi : bfs) {
        int mi = node_remap[wi];
        // Tree structure (with remapping)
        int wl = worker.left_[wi];
        int wr = worker.right_[wi];
        int wp = worker.parent_[wi];
        left_[mi]      = (wl < 0) ? -1 : node_remap.at(wl);
        right_[mi]     = (wr < 0) ? -1 : node_remap.at(wr);
        // parent_: remap if in subtree; else preserve master's existing value
        // (the domain root's parent is unchanged from master's POV).
        if (mi != domain_root_idx) {
            auto it = node_remap.find(wp);
            parent_[mi] = (it != node_remap.end()) ? it->second : wp;
        }
        depth_[mi]     = worker.depth_[wi];
        split_dim_[mi] = worker.split_dim_[wi];
        split_val_[mi] = worker.split_val_[wi];

        // Dual-channel envelope
        for (int ch = 0; ch < N_CHANNELS; ++ch) {
            channels_[ch].has_data[mi]       = worker.channels_[ch].has_data[wi];
            channels_[ch].source_quality[mi] = worker.channels_[ch].source_quality[wi];
            std::memcpy(ep_data_write(mi, ch),
                        worker.ep_data_read(wi, ch),
                        static_cast<size_t>(ep_stride_) * sizeof(float));
        }
        // Link iAABB cache
        if (!link_iaabb_cache_.empty() && !worker.link_iaabb_cache_.empty()) {
            std::memcpy(link_iaabb_cache_.data() + static_cast<size_t>(mi) * liaabb_stride_,
                        worker.link_iaabb_cache_.data() + static_cast<size_t>(wi) * liaabb_stride_,
                        static_cast<size_t>(liaabb_stride_) * sizeof(float));
            link_iaabb_dirty_[mi] = worker.link_iaabb_dirty_[wi];
        } else {
            link_iaabb_dirty_[mi] = 1;
        }
        // Per-node grids
        if (wi < static_cast<int>(worker.node_grids_.size())) {
            // R1: materialize worker's pending grid before copy.
            worker.materialize_pending_grid_(wi);
            if (mi >= static_cast<int>(node_grids_.size())) {
                node_grids_.resize(capacity_);
                node_grid_meta_.resize(capacity_);
            }
            node_grids_[mi]     = worker.node_grids_[wi];
            node_grid_meta_[mi] = worker.node_grid_meta_[wi];
        }
        // Occupation (with box ID remap)
        int wfid = worker.forest_id_[wi];
        if (wfid >= 0) {
            auto it = id_map.find(wfid);
            forest_id_[mi] = (it != id_map.end()) ? it->second : wfid;
        } else {
            forest_id_[mi] = -1;
        }
        subtree_occ_[mi] = worker.subtree_occ_[wi];

        // Phase A: copy collide-verified cache for the shipped subtree.
        if (wi < static_cast<int>(worker.collide_state_.size())
            && mi < static_cast<int>(collide_state_.size())) {
            collide_state_[mi]        = worker.collide_state_[wi];
            collide_verified_gen_[mi] = worker.collide_verified_gen_[wi];
        }
        ++n_shipped;
    }

    // Legacy z4_cache_ removed; V6 cache_mgr_ is shared via mmap, no merge needed.
    return n_shipped;
}

}  // namespace sbf

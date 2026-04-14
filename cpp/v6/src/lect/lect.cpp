// SafeBoxForest v6 — LECT implementation (Phase D + Phase 23: dual-channel + grids)
#include <sbf/lect/lect.h>
#include <sbf/lect/z4_grid_cache.h>        // GridQuality
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/envelope_type.h>
#include <sbf/envelope/link_iaabb.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <numeric>
#include <sbf/core/log.h>

namespace sbf {

// ══════════════════════════════════════════════════════════════════════════�?
//  Constructor
// ══════════════════════════════════════════════════════════════════════════�?

LECT::LECT(const Robot& robot,
           const std::vector<Interval>& root_intervals,
           const EndpointSourceConfig& ep_config,
           const EnvelopeTypeConfig& env_config,
           int initial_cap)
    : robot_(robot),
      ep_config_(ep_config),
      env_config_(env_config),
      root_intervals_(root_intervals)
{
    n_dims_ = robot.n_joints();
    n_active_links_ = robot.n_active_links();
    ep_stride_ = n_active_links_ * 2 * 6;
    liaabb_stride_ = n_active_links_ * 6;

    // Detect Z4 symmetry
    auto syms = detect_joint_symmetries(robot);
    if (!syms.empty()) symmetry_q0_ = syms[0];
    if (symmetry_q0_.type == JointSymmetryType::Z4_ROTATION)
        z4_active_ = true;

    // Pre-cache link radii as float
    const double* lr = robot.active_link_radii();
    if (lr) {
        radii_f_.resize(n_active_links_);
        for (int i = 0; i < n_active_links_; ++i)
            radii_f_[i] = static_cast<float>(lr[i]);
    }

    // Allocate flat buffers
    capacity_ = 0;
    n_nodes_ = 0;
    ensure_capacity(initial_cap);

    // Allocate root node
    int root_idx = alloc_node();
    (void)root_idx;
    assert(root_idx == 0);
    depth_[0] = 0;
    parent_[0] = -1;

    // Compute root FK + envelope
    root_fk_ = compute_fk_full(robot, root_intervals);
    compute_envelope(0, root_fk_, root_intervals);
}

// ══════════════════════════════════════════════════════════════════════════
//  materialize_mmap — copy loaded ep_data from mmap to vectors
// ══════════════════════════════════════════════════════════════════════════

void LECT::materialize_mmap() const {
    if (!use_mmap_) return;

    // (Grid lazy loading removed in new LECT — no backup needed.)

    // Build full-size ep_data vectors from mmap (loaded) + offset vector (new).
    const int cap = capacity_;
    const int nn  = nn_loaded_;
    const int nn_total = n_nodes_;

    for (int ch = 0; ch < N_CHANNELS; ++ch) {
        std::vector<float> full(static_cast<size_t>(cap) * ep_stride_, 0.0f);
        // Copy loaded nodes from mmap
        for (int i = 0; i < nn; ++i) {
            const float* src = ep_data_read(i, ch);
            std::memcpy(full.data() + static_cast<size_t>(i) * ep_stride_,
                        src,
                        static_cast<size_t>(ep_stride_) * sizeof(float));
        }
        // Copy expanded nodes from offset vector
        if (nn_total > nn) {
            size_t len = static_cast<size_t>(nn_total - nn) * ep_stride_;
            std::memcpy(full.data() + static_cast<size_t>(nn) * ep_stride_,
                        channels_[ch].ep_data.data(),
                        len * sizeof(float));
        }
        const_cast<std::vector<float>&>(channels_[ch].ep_data) = std::move(full);
    }

    // Materialize tree structure from mmap to vectors (if mmap-backed)
    auto& self = const_cast<LECT&>(*this);
    if (self.left_.is_mmap()) {
        const int mat_n = std::min(nn_total,
                                   self.mmap_tree_cap_ > 0 ? self.mmap_tree_cap_
                                                           : nn_total);
        self.left_.materialize(mat_n);
        self.right_.materialize(mat_n);
        self.parent_.materialize(mat_n);
        self.depth_.materialize(mat_n);
        self.split_dim_.materialize(mat_n);
        self.split_val_.materialize(mat_n);
        self.mmap_tree_cap_ = 0;
        self.mmap_tree_off_ = 0;
    }

    mmap_.close();
    mmap_ep_base_     = nullptr;
    mmap_node_stride_ = 0;
    nn_loaded_        = 0;
    use_mmap_         = false;
}

// (Grid lazy-loading helpers removed — new LECT handles grids differently.)

void LECT::ensure_capacity(int min_cap) {
    if (min_cap <= capacity_) return;

    int new_cap = (capacity_ == 0) ? min_cap
                                   : capacity_;
    while (new_cap < min_cap)
        new_cap *= 2;

    // If tree arrays are mmap-backed (worker snapshot), prefer staying within
    // the mmap tree section to avoid premature materialisation.  Only
    // materialise if min_cap actually exceeds the mmap tree capacity.
    bool mmap_tree = left_.is_mmap() && mmap_tree_cap_ > 0;
    if (mmap_tree) {
        if (new_cap > mmap_tree_cap_) {
            if (min_cap <= mmap_tree_cap_) {
                // Cap at mmap boundary — tree stays in mmap
                new_cap = mmap_tree_cap_;
            } else {
                // Must exceed mmap bounds — materialise tree to vectors
                const int mat_n = std::min(n_nodes_, mmap_tree_cap_);
                left_.materialize(mat_n);
                right_.materialize(mat_n);
                parent_.materialize(mat_n);
                depth_.materialize(mat_n);
                split_dim_.materialize(mat_n);
                split_val_.materialize(mat_n);
                mmap_tree_cap_ = 0;
                mmap_tree_off_ = 0;
            }
        }
        // else: new_cap <= mmap_tree_cap_ — tree stays in mmap as-is
    }

    for (int ch = 0; ch < N_CHANNELS; ++ch) {
        channels_[ch].has_data.resize(new_cap, 0);
        channels_[ch].source_quality.resize(new_cap, 0);
        if (use_mmap_) {
            // Offset mode: ep_data vector stores only nodes [nn_loaded_, new_cap).
            channels_[ch].ep_data.resize(
                static_cast<size_t>(new_cap - nn_loaded_) * ep_stride_, 0.0f);
        } else {
            channels_[ch].ep_data.resize(
                static_cast<size_t>(new_cap) * ep_stride_, 0.0f);
        }
    }
    // link_iaabb_cache_: always allocate in ensure_capacity
    // (for loaded LECTs, ensure_capacity is not called during load, so this
    // only fires on first tree expansion — not during load hot path)
    link_iaabb_cache_.resize(static_cast<size_t>(new_cap) * liaabb_stride_, 0.0f);
    link_iaabb_dirty_.resize(new_cap, 1);

    // Lazy alloc: only resize node_grids_ if already populated
    if (!node_grids_.empty()) {
        node_grids_.resize(new_cap);
        node_grid_meta_.resize(new_cap);
    }

    // Resize tree arrays (no-op when mmap-backed and within mmap_tree_cap_)
    left_.resize(new_cap, -1);
    right_.resize(new_cap, -1);
    parent_.resize(new_cap, -1);
    depth_.resize(new_cap, 0);
    split_dim_.resize(new_cap, -1);
    split_val_.resize(new_cap, 0.0);

    forest_id_.resize(new_cap, -1);
    subtree_occ_.resize(new_cap, 0);

    capacity_ = new_cap;

    // Post-condition: capacity_ updated
    // (vec_size() check removed — TreeArray no longer exposes raw vec size.)
}
int LECT::alloc_node() {
    int idx = n_nodes_;
    ++n_nodes_;
    if (n_nodes_ > capacity_) {
        ensure_capacity(n_nodes_);
    }
    // Safety: if idx exceeds capacity, force resize (belt-and-suspenders)
    if (idx >= capacity_) {
        if (true) {
            SBF_WARN("[LECT-BUG] alloc_node: idx=%d >= capacity=%d " "(n=%d). Forcing resize!", idx, capacity_, n_nodes_);
            ensure_capacity(n_nodes_);
        }
    } else if (mmap_tree_cap_ > 0 && idx >= mmap_tree_cap_) {
        SBF_WARN("[LECT-BUG] alloc_node: idx=%d >= mmap_tree_cap=%d → materialise!", idx, mmap_tree_cap_);
        ensure_capacity(n_nodes_);
    }
    left_[idx] = -1;
    right_[idx] = -1;
    parent_[idx] = -1;
    depth_[idx] = 0;
    split_dim_[idx] = -1;
    split_val_[idx] = 0.0;
    for (int ch = 0; ch < N_CHANNELS; ++ch) {
        channels_[ch].has_data[idx] = 0;
        channels_[ch].source_quality[idx] = 0;
        std::memset(ep_data_write(idx, ch), 0,
                    static_cast<size_t>(ep_stride_) * sizeof(float));
    }
    if (idx < static_cast<int>(node_grids_.size())) {
        node_grids_[idx].clear();
        node_grid_meta_[idx].clear();
    }
    // (grid_lazy removed — new LECT handles grids eagerly.)
    forest_id_[idx] = -1;
    subtree_occ_[idx] = 0;
    return idx;
}

// ══════════════════════════════════════════════════════════════════════════�?
//  node_intervals �?reconstruct C-space intervals by walking root �?node
// ══════════════════════════════════════════════════════════════════════════�?

std::vector<Interval> LECT::node_intervals(int node_idx) const {
    auto intervals = root_intervals_;

    // Collect path from root to node_idx
    std::vector<int> path;
    int cur = node_idx;
    while (cur > 0) {
        path.push_back(cur);
        cur = parent_[cur];
    }

    // Walk from root downward
    for (int i = static_cast<int>(path.size()) - 1; i >= 0; --i) {
        int child = path[i];
        int par = parent_[child];
        int dim = split_dim_[par];
        if (dim < 0) continue;

        double sv = split_val_[par];
        if (child == left_[par]) {
            intervals[dim].hi = sv;
        } else {
            intervals[dim].lo = sv;
        }
    }

    return intervals;
}

// ══════════════════════════════════════════════════════════════════════════�?
//  Z4 symmetry helpers
// ══════════════════════════════════════════════════════════════════════════�?

int LECT::z4_canonicalize_interval(double q0_lo, double q0_hi,
                                   double period,
                                   double& can_lo, double& can_hi) {
    int k = static_cast<int>(std::floor(q0_lo / period));
    can_lo = q0_lo - k * period;
    can_hi = q0_hi - k * period;
    return ((k % 4) + 4) % 4;
}

uint64_t LECT::z4_interval_hash(double can_lo, double can_hi,
                                const std::vector<Interval>& intervals) {
    uint64_t h = 14695981039346656037ULL;
    auto mix = [&h](const void* p, size_t n) {
        const auto* b = static_cast<const uint8_t*>(p);
        for (size_t i = 0; i < n; ++i) {
            h ^= b[i];
            h *= 1099511628211ULL;
        }
    };
    mix(&can_lo, sizeof(double));
    mix(&can_hi, sizeof(double));
    for (size_t i = 1; i < intervals.size(); ++i) {
        mix(&intervals[i].lo, sizeof(double));
        mix(&intervals[i].hi, sizeof(double));
    }
    return h;
}

// ══════════════════════════════════════════════════════════════════════════�?
//  compute_envelope �?per-node two-stage pipeline
// ══════════════════════════════════════════════════════════════════════════�?

void LECT::compute_envelope(int node_idx, const FKState& fk,
                            const std::vector<Interval>& intervals,
                            int changed_dim, int parent_idx) {
    ensure_capacity(node_idx + 1);

    const int n_act = n_active_links_;
    const int* alm = robot_.active_link_map();

    // Determine channel from configured source
    const int ch = source_channel(ep_config_.source);

    // ── Z4 persistent cache lookup (V6 path) ────────────────────────────
    // Checks ALL sectors via disk cache, not just non-zero.
    if (z4_active_ && cache_mgr_ &&
        symmetry_q0_.type == JointSymmetryType::Z4_ROTATION) {

        double can_lo, can_hi;
        int sector = z4_canonicalize_interval(
            intervals[0].lo, intervals[0].hi,
            symmetry_q0_.period, can_lo, can_hi);
        uint64_t z4_key = z4_interval_hash(can_lo, can_hi, intervals);

        // Thread-safe lookup: copy EP data into local buffer under lock,
        // so grow() cannot invalidate the pointer while we're using it.
        std::vector<float> cached_ep_buf(ep_stride_);
        bool cache_hit = cache_mgr_->ep_cache(ch).lookup_copy(
            z4_key, ep_config_.source, cached_ep_buf.data());

        if (cache_hit) {
            const float* cached_ep = cached_ep_buf.data();
            float* ep_out = ep_data_write(node_idx, ch);

            if (sector != 0) {
                // Need to transform from canonical sector
                if (ch == CH_UNSAFE && channels_[ch].has_data[node_idx]) {
                    std::vector<float> tmp(ep_stride_);
                    symmetry_q0_.transform_all_endpoint_iaabbs(
                        cached_ep, n_act * 2, sector, tmp.data());
                    hull_endpoint_iaabbs(ep_out, tmp.data(), n_act * 2);
                } else {
                    symmetry_q0_.transform_all_endpoint_iaabbs(
                        cached_ep, n_act * 2, sector, ep_out);
                    channels_[ch].has_data[node_idx] = 1;
                    channels_[ch].source_quality[node_idx] =
                        static_cast<uint8_t>(ep_config_.source);
                }
            } else {
                // Sector 0 = canonical, direct copy
                if (ch == CH_UNSAFE && channels_[ch].has_data[node_idx]) {
                    hull_endpoint_iaabbs(ep_out, cached_ep, n_act * 2);
                } else {
                    std::memcpy(ep_out, cached_ep,
                                static_cast<size_t>(ep_stride_) * sizeof(float));
                    channels_[ch].has_data[node_idx] = 1;
                    channels_[ch].source_quality[node_idx] =
                        static_cast<uint8_t>(ep_config_.source);
                }
            }

            derive_merged_link_iaabb(node_idx);

            // Grid cache lookup (V6) — per-sector keying
            // Use (z4_key << 2) | sector so each sector gets its own
            // cached grid, eliminating recomputation for sector != 0.
            if (env_config_.type != EnvelopeType::LinkIAABB) {
                auto& gc = cache_mgr_->grid_cache(ch);
                const uint64_t grid_key = (z4_key << 2) | static_cast<uint64_t>(sector);
                GridQuality grid_req{env_config_.type,
                    static_cast<float>(env_config_.grid_config.voxel_delta),
                    env_config_.n_subdivisions};
                auto cached_grid = gc.lookup(grid_key, grid_req);
                if (cached_grid) {
                    // Lazy alloc
                    if (node_idx >= static_cast<int>(node_grids_.size())) {
                        node_grids_.resize(capacity_);
                        node_grid_meta_.resize(capacity_);
                    }
                    auto& grids = node_grids_[node_idx];
                    auto& metas = node_grid_meta_[node_idx];
                    grids.push_back(std::move(*cached_grid));
                    metas.push_back({env_config_.type,
                                     static_cast<float>(env_config_.grid_config.voxel_delta),
                                     static_cast<uint8_t>(ch)});
                } else {
                    // Grid cache miss — compute and insert for this sector
                    compute_node_grids(node_idx, ch);
                    if (node_idx < static_cast<int>(node_grids_.size()) &&
                        !node_grids_[node_idx].empty()) {
                        gc.insert(grid_key, node_grids_[node_idx].back(),
                                  grid_req);
                    }
                }
            }
            return;
        }
        // V6 cache miss — fall through to compute, then insert
    }

    // ── Z4 in-memory cache lookup (V5 fallback, sector != 0 only) ───────
    if (z4_active_ && !cache_mgr_ && !z4_cache_.empty() &&
        symmetry_q0_.type == JointSymmetryType::Z4_ROTATION) {

        double can_lo, can_hi;
        int sector = z4_canonicalize_interval(
            intervals[0].lo, intervals[0].hi,
            symmetry_q0_.period, can_lo, can_hi);

        if (sector != 0) {
            uint64_t key = z4_interval_hash(can_lo, can_hi, intervals);
            auto it = z4_cache_.find(key);
            if (it != z4_cache_.end() &&
                it->second.channel == ch &&
                source_can_serve(it->second.source, ep_config_.source)) {
                float* ep_out = ep_data_write(node_idx, ch);

                // For UNSAFE channel: union if node already has data
                if (ch == CH_UNSAFE && channels_[ch].has_data[node_idx]) {
                    std::vector<float> tmp(ep_stride_);
                    symmetry_q0_.transform_all_endpoint_iaabbs(
                        it->second.ep_iaabbs.data(),
                        n_act * 2, sector, tmp.data());
                    hull_endpoint_iaabbs(ep_out, tmp.data(), n_act * 2);
                } else {
                    symmetry_q0_.transform_all_endpoint_iaabbs(
                        it->second.ep_iaabbs.data(),
                        n_act * 2, sector, ep_out);
                    channels_[ch].has_data[node_idx] = 1;
                    channels_[ch].source_quality[node_idx] = static_cast<uint8_t>(it->second.source);
                }

                derive_merged_link_iaabb(node_idx);
                compute_node_grids(node_idx, ch);
                return;
            }
        }
    }

    // ── IFK fast path: extract endpoints directly from FKState ──────────
    if (ep_config_.source == EndpointSource::IFK && fk.valid) {
        float* ep_out = ep_data_write(node_idx, CH_SAFE);

        if (changed_dim >= 0 && parent_idx >= 0 &&
            parent_idx < n_nodes_ && channels_[CH_SAFE].has_data[parent_idx]) {
            // Partial inheritance: copy unchanged links from parent
            const float* parent_ep = ep_data_read(parent_idx, CH_SAFE);

            int inherit_pairs = 0;
            for (int ci = 0; ci < n_act; ++ci) {
                if (alm[ci] + 1 <= changed_dim)
                    inherit_pairs = ci + 1;
                else
                    break;
            }

            if (inherit_pairs > 0)
                std::memcpy(ep_out, parent_ep,
                            static_cast<size_t>(inherit_pairs) * 12 * sizeof(float));

            // Extract changed entries from FKState
            for (int ci = inherit_pairs; ci < n_act; ++ci) {
                int V = alm[ci];
                float* p = ep_out + ci * 12;
                p[0]  = static_cast<float>(fk.prefix_lo[V][3]);
                p[1]  = static_cast<float>(fk.prefix_lo[V][7]);
                p[2]  = static_cast<float>(fk.prefix_lo[V][11]);
                p[3]  = static_cast<float>(fk.prefix_hi[V][3]);
                p[4]  = static_cast<float>(fk.prefix_hi[V][7]);
                p[5]  = static_cast<float>(fk.prefix_hi[V][11]);
                p[6]  = static_cast<float>(fk.prefix_lo[V + 1][3]);
                p[7]  = static_cast<float>(fk.prefix_lo[V + 1][7]);
                p[8]  = static_cast<float>(fk.prefix_lo[V + 1][11]);
                p[9]  = static_cast<float>(fk.prefix_hi[V + 1][3]);
                p[10] = static_cast<float>(fk.prefix_hi[V + 1][7]);
                p[11] = static_cast<float>(fk.prefix_hi[V + 1][11]);
            }
        } else {
            // Full extraction from FKState
            for (int ci = 0; ci < n_act; ++ci) {
                int V = alm[ci];
                float* p = ep_out + ci * 12;
                p[0]  = static_cast<float>(fk.prefix_lo[V][3]);
                p[1]  = static_cast<float>(fk.prefix_lo[V][7]);
                p[2]  = static_cast<float>(fk.prefix_lo[V][11]);
                p[3]  = static_cast<float>(fk.prefix_hi[V][3]);
                p[4]  = static_cast<float>(fk.prefix_hi[V][7]);
                p[5]  = static_cast<float>(fk.prefix_hi[V][11]);
                p[6]  = static_cast<float>(fk.prefix_lo[V + 1][3]);
                p[7]  = static_cast<float>(fk.prefix_lo[V + 1][7]);
                p[8]  = static_cast<float>(fk.prefix_lo[V + 1][11]);
                p[9]  = static_cast<float>(fk.prefix_hi[V + 1][3]);
                p[10] = static_cast<float>(fk.prefix_hi[V + 1][7]);
                p[11] = static_cast<float>(fk.prefix_hi[V + 1][11]);
            }
        }

        channels_[CH_SAFE].has_data[node_idx] = 1;
        channels_[CH_SAFE].source_quality[node_idx] = static_cast<uint8_t>(EndpointSource::IFK);

        derive_merged_link_iaabb(node_idx);
        compute_node_grids(node_idx, CH_SAFE);

        // Z4 cache populate (canonical sector only)
        if (z4_active_ &&
            symmetry_q0_.type == JointSymmetryType::Z4_ROTATION) {
            double can_lo, can_hi;
            int sector = z4_canonicalize_interval(
                intervals[0].lo, intervals[0].hi,
                symmetry_q0_.period, can_lo, can_hi);
            if (sector == 0) {
                uint64_t key = z4_interval_hash(can_lo, can_hi, intervals);

                // V6 persistent cache insert
                if (cache_mgr_) {
                    cache_mgr_->ep_cache(CH_SAFE).insert(
                        key, EndpointSource::IFK, ep_out);

                    // Insert grid if computed (sector==0 → grid_key = key<<2)
                    if (env_config_.type != EnvelopeType::LinkIAABB &&
                        node_idx < static_cast<int>(node_grids_.size()) &&
                        !node_grids_[node_idx].empty()) {
                        GridQuality gq{env_config_.type,
                            static_cast<float>(env_config_.grid_config.voxel_delta),
                            env_config_.n_subdivisions};
                        cache_mgr_->grid_cache(CH_SAFE).insert(
                            key << 2, node_grids_[node_idx].back(), gq);
                    }
                }

                // V5 in-memory cache (fallback)
                auto it = z4_cache_.find(key);
                if (it == z4_cache_.end() ||
                    !source_can_serve(it->second.source, ep_config_.source)) {
                    Z4CacheEntry entry;
                    entry.ep_iaabbs.assign(ep_out, ep_out + ep_stride_);
                    entry.source = EndpointSource::IFK;
                    entry.channel = CH_SAFE;
                    z4_cache_[key] = std::move(entry);
                }
            }
        }
        return;
    }

    // ── Non-IFK fallback: full compute_endpoint_iaabb ───────────────────
    FKState fk_copy;
    auto ep_result = compute_endpoint_iaabb(robot_, intervals, ep_config_,
                                            &fk_copy, changed_dim);

    const int result_ch = source_channel(ep_result.source);
    float* ep_out = ep_data_write(node_idx, result_ch);

    // For UNSAFE channel: union coverage if node already has data
    if (result_ch == CH_UNSAFE && channels_[result_ch].has_data[node_idx]) {
        hull_endpoint_iaabbs(ep_out, ep_result.endpoint_iaabbs.data(), n_act * 2);
    } else {
        std::memcpy(ep_out, ep_result.endpoint_iaabbs.data(),
                    static_cast<size_t>(ep_stride_) * sizeof(float));
        channels_[result_ch].has_data[node_idx] = 1;
        channels_[result_ch].source_quality[node_idx] = static_cast<uint8_t>(ep_result.source);
    }

    derive_merged_link_iaabb(node_idx);
    compute_node_grids(node_idx, result_ch);

    // Z4 cache populate
    if (z4_active_ &&
        symmetry_q0_.type == JointSymmetryType::Z4_ROTATION) {
        double can_lo, can_hi;
        int sector = z4_canonicalize_interval(
            intervals[0].lo, intervals[0].hi,
            symmetry_q0_.period, can_lo, can_hi);
        if (sector == 0) {
            uint64_t key = z4_interval_hash(can_lo, can_hi, intervals);

            // V6 persistent cache insert
            if (cache_mgr_) {
                cache_mgr_->ep_cache(result_ch).insert(
                    key, ep_result.source, ep_out);

                // Insert grid with per-sector key (sector==0 → key<<2)
                if (env_config_.type != EnvelopeType::LinkIAABB &&
                    node_idx < static_cast<int>(node_grids_.size()) &&
                    !node_grids_[node_idx].empty()) {
                    GridQuality gq{env_config_.type,
                        static_cast<float>(env_config_.grid_config.voxel_delta),
                        env_config_.n_subdivisions};
                    cache_mgr_->grid_cache(result_ch).insert(
                        key << 2, node_grids_[node_idx].back(), gq);
                }
            }

            // V5 in-memory cache (fallback)
            auto it = z4_cache_.find(key);
            if (it == z4_cache_.end() ||
                !source_can_serve(it->second.source, ep_config_.source)) {
                Z4CacheEntry entry;
                entry.ep_iaabbs.assign(ep_out, ep_out + ep_stride_);
                entry.source = ep_result.source;
                entry.channel = result_ch;
                z4_cache_[key] = std::move(entry);
            }
        }
    }
}

// ══════════════════════════════════════════════════════════════════════════�?
//  Split dimension selection
// ══════════════════════════════════════════════════════════════════════════�?

int LECT::pick_best_tighten_dim(
    const FKState& parent_fk,
    const std::vector<Interval>& intervals) const
{
    const int nj = n_dims_;
    const int nact = n_active_links_;
    const int* alm = robot_.active_link_map();
    const double* radii = robot_.active_link_radii();

    // Fallback: widest-first
    int best_dim = 0;
    for (int d = 1; d < nj; ++d) {
        if (intervals[d].width() > intervals[best_dim].width())
            best_dim = d;
    }

    double best_metric = std::numeric_limits<double>::max();

    float left_aabbs[MAX_TF * 6];
    float right_aabbs[MAX_TF * 6];

    for (int d = 0; d < nj; ++d) {
        if (intervals[d].width() <= 0.0) continue;

        double mid_d = intervals[d].center();

        // Probe left child
        auto left_ivs = intervals;
        left_ivs[d].hi = mid_d;
        FKState left_fk = compute_fk_incremental(parent_fk, robot_, left_ivs, d);
        extract_link_aabbs(left_fk, alm, nact, left_aabbs, radii);

        // Probe right child
        auto right_ivs = intervals;
        right_ivs[d].lo = mid_d;
        FKState right_fk = compute_fk_incremental(parent_fk, robot_, right_ivs, d);
        extract_link_aabbs(right_fk, alm, nact, right_aabbs, radii);

        // Compute volumes
        double vol_left = 0.0, vol_right = 0.0;
        for (int i = 0; i < nact; ++i) {
            const float* L = left_aabbs + i * 6;
            vol_left += static_cast<double>(L[3] - L[0]) *
                        static_cast<double>(L[4] - L[1]) *
                        static_cast<double>(L[5] - L[2]);
            const float* R = right_aabbs + i * 6;
            vol_right += static_cast<double>(R[3] - R[0]) *
                         static_cast<double>(R[4] - R[1]) *
                         static_cast<double>(R[5] - R[2]);
        }

        double metric = std::max(vol_left, vol_right);
        double metric_tol = 1e-9 * std::max(1.0, std::max(std::abs(metric), std::abs(best_metric)));
        if (metric < best_metric - metric_tol) {
            best_metric = metric;
            best_dim = d;
        }
    }

    return best_dim;
}

// ──────────────────────────────────────────────────────────────────────────
//  pick_best_tighten_dim_v2 — multi-probe version with width awareness
//
//  K virtual nodes are generated at the target depth by replaying the
//  cached split-dim history from the root with random left/right choices.
//  Each probe evaluates all D candidate dimensions using a composite
//  metric that combines envelope volume tightening with a width-balance
//  penalty:
//
//    score_d = normalised_volume_d × (max_width / width_d)
//
//  This prevents repeatedly splitting already-narrow dimensions.
//  Scores are normalised per-probe and averaged across all K probes.
// ──────────────────────────────────────────────────────────────────────────
int LECT::pick_best_tighten_dim_v2(
    const FKState& caller_fk,
    const std::vector<Interval>& caller_intervals,
    int depth) const
{
    const int nj = n_dims_;
    const int nact = n_active_links_;
    const int* alm = robot_.active_link_map();
    const double* radii = robot_.active_link_radii();
    const int K = n_bt_probes_;

    // Accumulate normalised scores per dimension
    std::vector<double> score_sum(nj, 0.0);

    float left_aabbs[MAX_TF * 6];
    float right_aabbs[MAX_TF * 6];

    for (int pi = 0; pi < K; ++pi) {
        // ── Build probe intervals ──
        FKState probe_fk;
        std::vector<Interval> probe_ivs;

        if (pi == 0) {
            // Probe 0 = the actual caller node
            probe_fk  = caller_fk;
            probe_ivs = caller_intervals;
        } else {
            // Generate virtual probe: replay split history from root
            probe_ivs = root_intervals_;
            // Use deterministic pseudo-random per (depth, probe_idx)
            uint32_t rng = static_cast<uint32_t>(depth * 9973u + pi * 6997u);
            for (int lv = 0; lv < depth; ++lv) {
                auto it = depth_split_dim_cache_.find(lv);
                int sd;
                if (it != depth_split_dim_cache_.end()) {
                    sd = it->second;
                } else {
                    // No cached dim for this level yet — use round-robin
                    sd = lv % nj;
                }
                double mid = probe_ivs[sd].center();
                // Deterministic left/right choice
                rng ^= (rng << 13); rng ^= (rng >> 17); rng ^= (rng << 5);
                if (rng & 1u)
                    probe_ivs[sd].hi = mid;  // go left
                else
                    probe_ivs[sd].lo = mid;  // go right
            }
            probe_fk = compute_fk_full(robot_, probe_ivs);
        }

        // ── Find max width for this probe (for width penalty) ──
        double max_width = 0.0;
        for (int d = 0; d < nj; ++d)
            max_width = std::max(max_width, probe_ivs[d].width());

        // ── Compute parent volume for this probe ──
        FKState parent_fk_copy = probe_fk;
        float parent_aabbs[MAX_TF * 6];
        extract_link_aabbs(parent_fk_copy, alm, nact, parent_aabbs, radii);
        double parent_vol = 0.0;
        for (int i = 0; i < nact; ++i) {
            const float* P = parent_aabbs + i * 6;
            parent_vol += static_cast<double>(P[3] - P[0]) *
                          static_cast<double>(P[4] - P[1]) *
                          static_cast<double>(P[5] - P[2]);
        }

        // ── Evaluate all dimensions for this probe ──
        std::vector<double> raw_metrics(nj, std::numeric_limits<double>::max());

        for (int d = 0; d < nj; ++d) {
            double wd = probe_ivs[d].width();
            if (wd <= 0.0) continue;

            double mid_d = probe_ivs[d].center();

            auto left_iv = probe_ivs;
            left_iv[d].hi = mid_d;
            FKState left_fk = compute_fk_incremental(probe_fk, robot_, left_iv, d);
            extract_link_aabbs(left_fk, alm, nact, left_aabbs, radii);

            auto right_iv = probe_ivs;
            right_iv[d].lo = mid_d;
            FKState right_fk = compute_fk_incremental(probe_fk, robot_, right_iv, d);
            extract_link_aabbs(right_fk, alm, nact, right_aabbs, radii);

            double vol_left = 0.0, vol_right = 0.0;
            for (int i = 0; i < nact; ++i) {
                const float* L = left_aabbs + i * 6;
                vol_left += static_cast<double>(L[3] - L[0]) *
                            static_cast<double>(L[4] - L[1]) *
                            static_cast<double>(L[5] - L[2]);
                const float* R = right_aabbs + i * 6;
                vol_right += static_cast<double>(R[3] - R[0]) *
                             static_cast<double>(R[4] - R[1]) *
                             static_cast<double>(R[5] - R[2]);
            }

            // Composite metric: volume tightening × width penalty
            // Sum gives a more balanced signal than max
            double vol_metric = vol_left + vol_right;
            // Auto-disable width penalty for non-IFK sources (CritSample etc.)
            double eff_wp = (ep_config_.source == EndpointSource::IFK)
                ? bt_width_power_ : 0.0;
            double width_penalty = 1.0;
            if (eff_wp > 0.0) {
                double width_ratio = max_width / wd;   // ≥ 1; =1 for widest dim
                width_penalty = (eff_wp == 1.0)
                    ? width_ratio
                    : std::pow(width_ratio, eff_wp);
            }
            raw_metrics[d] = vol_metric * width_penalty;
        }

        // ── Normalise within this probe and accumulate ──
        double max_raw = 0.0;
        for (int d = 0; d < nj; ++d) {
            if (raw_metrics[d] < std::numeric_limits<double>::max() * 0.5)
                max_raw = std::max(max_raw, raw_metrics[d]);
        }
        if (max_raw > 1e-30) {
            for (int d = 0; d < nj; ++d) {
                if (raw_metrics[d] < std::numeric_limits<double>::max() * 0.5)
                    score_sum[d] += raw_metrics[d] / max_raw;
                else
                    score_sum[d] += 1.0;
            }
        } else {
            for (int d = 0; d < nj; ++d)
                score_sum[d] += 1.0;
        }
    }

    // ── Pick dimension with smallest average normalised score ──
    int best_dim = 0;
    double best_score = score_sum[0];
    for (int d = 1; d < nj; ++d) {
        if (score_sum[d] < best_score - 1e-12) {
            best_score = score_sum[d];
            best_dim = d;
        }
    }

    return best_dim;
}

int LECT::pick_split_dim(int depth_val, const FKState& fk,
                         const std::vector<Interval>& intervals) const {
    switch (split_order_) {
    case SplitOrder::ROUND_ROBIN:
        return depth_val % n_dims_;

    case SplitOrder::WIDEST_FIRST: {
        int dim = 0;
        double max_w = intervals[0].width();
        for (int d = 1; d < n_dims_; ++d) {
            double w = intervals[d].width();
            if (w > max_w) { max_w = w; dim = d; }
        }
        return dim;
    }

    case SplitOrder::BEST_TIGHTEN:
        return pick_best_tighten_dim(fk, intervals);

    case SplitOrder::BEST_TIGHTEN_V2:
        // (not used directly — BT_V2 goes through split_leaf_impl cache path)
        return pick_best_tighten_dim(fk, intervals);

    default:
        return pick_best_tighten_dim(fk, intervals);
    }
}

// ══════════════════════════════════════════════════════════════════════════�?
//  split_leaf_impl �?split a leaf node into two children
// ══════════════════════════════════════════════════════════════════════════�?

void LECT::split_leaf_impl(int node_idx, const FKState& parent_fk,
                           const std::vector<Interval>& parent_intervals) {
    using Clock = std::chrono::steady_clock;
    expand_profile_.expand_calls++;
    int parent_depth = depth_[node_idx];

    // Depth-cached split dimension: compute once per depth level
    auto t_dim = Clock::now();
    int dim;
    if (split_order_ == SplitOrder::BEST_TIGHTEN ||
        split_order_ == SplitOrder::BEST_TIGHTEN_V2) {
        auto it = depth_split_dim_cache_.find(parent_depth);
        if (it != depth_split_dim_cache_.end()) {
            dim = it->second;
            expand_profile_.pick_dim_cache_hits++;
        } else {
            if (split_order_ == SplitOrder::BEST_TIGHTEN_V2)
                dim = pick_best_tighten_dim_v2(parent_fk, parent_intervals, parent_depth);
            else
                dim = pick_best_tighten_dim(parent_fk, parent_intervals);
            depth_split_dim_cache_[parent_depth] = dim;
        }
        expand_profile_.pick_dim_calls++;
    } else {
        dim = pick_split_dim(parent_depth, parent_fk, parent_intervals);
    }
    expand_profile_.pick_dim_ms += std::chrono::duration<double, std::milli>(Clock::now() - t_dim).count();
    double mid = parent_intervals[dim].center();

    // Z4: prefer sector boundary (kπ/2) over midpoint for q_0
    if (dim == 0 &&
        symmetry_q0_.type == JointSymmetryType::Z4_ROTATION) {
        double lo = parent_intervals[0].lo;
        double hi = parent_intervals[0].hi;
        double period = symmetry_q0_.period;
        double best_boundary = mid;
        double best_dist = std::numeric_limits<double>::infinity();
        int k_lo = static_cast<int>(std::ceil(lo / period));
        int k_hi = static_cast<int>(std::floor(hi / period));
        for (int k = k_lo; k <= k_hi; ++k) {
            double boundary = k * period;
            if (boundary > lo + 1e-10 && boundary < hi - 1e-10) {
                double dist = std::abs(boundary - mid);
                if (dist < best_dist) {
                    best_dist = dist;
                    best_boundary = boundary;
                }
            }
        }
        mid = best_boundary;
    }

    // Allocate children
    int left_idx = alloc_node();
    int right_idx = alloc_node();

    left_[node_idx] = left_idx;
    right_[node_idx] = right_idx;
    parent_[left_idx] = node_idx;
    parent_[right_idx] = node_idx;
    depth_[left_idx] = parent_depth + 1;
    depth_[right_idx] = parent_depth + 1;
    split_dim_[node_idx] = dim;
    split_val_[node_idx] = mid;

    // Build child intervals
    auto left_ivs = parent_intervals;
    left_ivs[dim].hi = mid;
    auto right_ivs = parent_intervals;
    right_ivs[dim].lo = mid;

    // Incremental FK + envelope for both children
    auto t_fk = Clock::now();
    FKState left_fk = compute_fk_incremental(parent_fk, robot_, left_ivs, dim);
    FKState right_fk = compute_fk_incremental(parent_fk, robot_, right_ivs, dim);
    expand_profile_.fk_inc_ms += std::chrono::duration<double, std::milli>(Clock::now() - t_fk).count();

    auto t_env = Clock::now();
    compute_envelope(left_idx, left_fk, left_ivs, dim, node_idx);
    compute_envelope(right_idx, right_fk, right_ivs, dim, node_idx);
    expand_profile_.envelope_ms += std::chrono::duration<double, std::milli>(Clock::now() - t_env).count();

    // Tighten parent AABBs
    auto t_ref = Clock::now();
    refine_parent_aabb(node_idx);
    expand_profile_.refine_ms += std::chrono::duration<double, std::milli>(Clock::now() - t_ref).count();
}

// ══════════════════════════════════════════════════════════════════════════�?
//  refine_parent_aabb �?intersect(parent, union(left, right))
// ══════════════════════════════════════════════════════════════════════════�?

void LECT::refine_parent_aabb(int parent_idx) {
    int li = left_[parent_idx];
    int ri = right_[parent_idx];
    if (li < 0 || ri < 0) return;

    // Refine each channel independently
    for (int ch = 0; ch < N_CHANNELS; ++ch) {
        auto& cd = channels_[ch];
        if (!cd.has_data[parent_idx] || !cd.has_data[li] || !cd.has_data[ri])
            continue;

        float* pa = ep_data_write(parent_idx, ch);
        const float* la = ep_data_read(li, ch);
        const float* ra = ep_data_read(ri, ch);

        if (ch == CH_SAFE) {
            // SAFE: intersect(parent, union(left, right)) — tighten
            for (int k = 0; k < ep_stride_; k += 6) {
                for (int c = 0; c < 3; ++c) {
                    float child_lo = std::min(la[k + c], ra[k + c]);
                    pa[k + c] = std::max(pa[k + c], child_lo);
                }
                for (int c = 3; c < 6; ++c) {
                    float child_hi = std::max(la[k + c], ra[k + c]);
                    pa[k + c] = std::min(pa[k + c], child_hi);
                }
            }
        } else {
            // UNSAFE: union(parent, union(left, right)) — expand
            hull_endpoint_iaabbs(pa, la, n_active_links_ * 2);
            hull_endpoint_iaabbs(pa, ra, n_active_links_ * 2);
        }
    }

    // Re-derive merged link iAABBs for parent
    derive_merged_link_iaabb(parent_idx);
}

// ══════════════════════════════════════════════════════════════════════════�?
//  expand_leaf �?public entry point
// ══════════════════════════════════════════════════════════════════════════�?

int LECT::expand_leaf(int node_idx) {
    if (!is_leaf(node_idx)) return 0;

    // Reconstruct intervals and FK
    auto intervals = node_intervals(node_idx);

    // Reconstruct FK for this node
    FKState fk = compute_fk_full(robot_, intervals);

    split_leaf_impl(node_idx, fk, intervals);
    return 2;
}

int LECT::expand_leaf(int node_idx, const FKState& fk,
                      const std::vector<Interval>& intervals) {
    if (!is_leaf(node_idx)) return 0;
    split_leaf_impl(node_idx, fk, intervals);
    return 2;
}

// ═════════════════════════════════════════════════════════════════════════════
//  derive_merged_link_iaabb — pick tightest channel for collision cache
// ═════════════════════════════════════════════════════════════════════════════

void LECT::derive_merged_link_iaabb(int node_idx) {
    // Lazy alloc: only allocate link_iaabb_cache_ on first write
    if (link_iaabb_cache_.empty()) {
        link_iaabb_cache_.resize(static_cast<size_t>(capacity_) * liaabb_stride_, 0.0f);
    }
    float* out = link_iaabb_cache_.data()
                 + static_cast<size_t>(node_idx) * liaabb_stride_;
    const double* radii = robot_.active_link_radii();

    // Prefer SAFE (tighter, provably correct) if available
    if (channels_[CH_SAFE].has_data[node_idx]) {
        derive_link_iaabb_paired(
            ep_data_read(node_idx, CH_SAFE),
            n_active_links_, radii, out);
    } else if (channels_[CH_UNSAFE].has_data[node_idx]) {
        derive_link_iaabb_paired(
            ep_data_read(node_idx, CH_UNSAFE),
            n_active_links_, radii, out);
    }
    link_iaabb_dirty_[node_idx] = 0;
}

// ── materialise_link_iaabb (lazy, called from get_link_iaabbs) ──────────
void LECT::materialise_link_iaabb(int i) {
    derive_merged_link_iaabb(i);
}

// ═════════════════════════════════════════════════════════════════════════════
//  compute_node_grids — rasterize link envelope grids for a channel
// ═════════════════════════════════════════════════════════════════════════════

void LECT::compute_node_grids(int node_idx, int channel) {
    if (env_config_.type == EnvelopeType::LinkIAABB) return;  // no grids requested

    // Lazy alloc: ensure node_grids_ covers this node
    if (node_idx >= static_cast<int>(node_grids_.size())) {
        node_grids_.resize(capacity_);
        node_grid_meta_.resize(capacity_);
    }

    const float* ep = ep_data_read(node_idx, channel);
    const double* radii = robot_.active_link_radii();
    const double delta = env_config_.grid_config.voxel_delta;

    auto& grids = node_grids_[node_idx];
    auto& metas = node_grid_meta_[node_idx];

    // Check if a grid matching (type, delta, channel) already exists
    auto find_slot = [&](EnvelopeType t, float d, uint8_t ch) -> int {
        for (int s = 0; s < static_cast<int>(metas.size()); ++s) {
            if (metas[s].type == t &&
                metas[s].delta == d &&
                metas[s].channel == ch)
                return s;
        }
        return -1;
    };

    auto compute_grid = [&](EnvelopeType type) {
        auto env = compute_link_envelope(ep, n_active_links_, radii,
                                         {type, env_config_.n_subdivisions,
                                          {delta}});
        if (!env.sparse_grid) return;

        float fdelta = static_cast<float>(delta);
        uint8_t ch = static_cast<uint8_t>(channel);
        int slot = find_slot(type, fdelta, ch);

        if (slot >= 0) {
            // Merge into existing grid (union for UNSAFE, replace for SAFE)
            if (channel == CH_UNSAFE)
                grids[slot].merge(*env.sparse_grid);
            else
                grids[slot] = std::move(*env.sparse_grid);
        } else {
            grids.push_back(std::move(*env.sparse_grid));
            metas.push_back({type, fdelta, ch});
        }
    };

    // Compute based on configured type
    if (env_config_.type == EnvelopeType::LinkIAABB_Grid ||
        env_config_.type == EnvelopeType::Hull16_Grid) {
        compute_grid(env_config_.type);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  warmup — obstacle-free tree expansion along random C-space paths
// ═══════════════════════════════════════════════════════════════════════════

int LECT::warmup(int max_depth, int n_paths, int seed) {
    if (n_nodes_ == 0) return 0;

    // Simple LCG RNG (sufficient for path sampling)
    uint64_t rng_state = static_cast<uint64_t>(seed);
    auto next_double = [&]() -> double {
        rng_state = rng_state * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<double>((rng_state >> 11) & 0x1FFFFFFFFFFFFFULL)
               / static_cast<double>(0x1FFFFFFFFFFFFFULL);
    };

    const int old_n = n_nodes_;
    for (int p = 0; p < n_paths; ++p) {
        // Generate a random config within root intervals
        std::vector<double> q(n_dims_);
        for (int d = 0; d < n_dims_; ++d) {
            double lo = root_intervals_[d].lo;
            double hi = root_intervals_[d].hi;
            q[d] = lo + next_double() * (hi - lo);
        }

        // Walk from root to leaf, expanding along the way
        int node = 0;
        while (depth_[node] < max_depth) {
            if (is_leaf(node)) {
                expand_leaf(node);
            }
            // Descend: compare q[split_dim] against split_val
            int dim = split_dim_[node];
            if (q[dim] <= split_val_[node])
                node = left_[node];
            else
                node = right_[node];
        }
    }
    return n_nodes_ - old_n;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Collision queries
// ══════════════════════════════════════════════════════════════════════════�?

bool LECT::collides_scene(int node_idx,
                          const Obstacle* obs, int n_obs) const {
    if (!has_data(node_idx)) return false;

    const float* liaabbs = get_link_iaabbs(node_idx);
    for (int ci = 0; ci < n_active_links_; ++ci) {
        const float* lb = liaabbs + ci * 6;
        for (int oi = 0; oi < n_obs; ++oi) {
            const float* ob = obs[oi].bounds;
            if (lb[0] <= ob[3] && lb[3] >= ob[0] &&
                lb[1] <= ob[4] && lb[4] >= ob[1] &&
                lb[2] <= ob[5] && lb[5] >= ob[2])
                return true;
        }
    }
    return false;
}

bool LECT::collides_scene(int node_idx,
                          const Obstacle* obs, int n_obs,
                          const std::unordered_map<float, voxel::SparseVoxelGrid>& obs_grids) const {
    if (!has_data(node_idx)) return false;

    // Layer 1: IAABB coarse test
    const float* liaabbs = get_link_iaabbs(node_idx);
    bool iaabb_hit = false;
    for (int ci = 0; ci < n_active_links_; ++ci) {
        const float* lb = liaabbs + ci * 6;
        for (int oi = 0; oi < n_obs; ++oi) {
            const float* ob = obs[oi].bounds;
            if (lb[0] <= ob[3] && lb[3] >= ob[0] &&
                lb[1] <= ob[4] && lb[4] >= ob[1] &&
                lb[2] <= ob[5] && lb[5] >= ob[2]) {
                iaabb_hit = true;
                goto iaabb_done;
            }
        }
    }
iaabb_done:
    if (!iaabb_hit) return false;

    // Layer 2: Grid fine test (if node has grids)
    if (node_idx < static_cast<int>(node_grids_.size()) &&
        !node_grids_[node_idx].empty()) {
        const auto& grids = node_grids_[node_idx];
        const auto& metas = node_grid_meta_[node_idx];
        // If ANY grid says no collision → safe
        for (int s = 0; s < static_cast<int>(grids.size()); ++s) {
            auto it = obs_grids.find(metas[s].delta);
            if (it != obs_grids.end()) {
                if (!grids[s].collides(it->second))
                    return false;
            }
        }
    }

    return true;
}

bool LECT::intervals_collide_scene(const std::vector<Interval>& intervals,
                                   const Obstacle* obs, int n_obs) const {
    FKState fk = compute_fk_full(robot_, intervals);
    std::vector<float> aabb(n_active_links_ * 6);
    extract_link_aabbs(fk, robot_.active_link_map(), n_active_links_,
                       aabb.data(), robot_.active_link_radii());

    for (int ci = 0; ci < n_active_links_; ++ci) {
        const float* lb = aabb.data() + ci * 6;
        for (int oi = 0; oi < n_obs; ++oi) {
            const float* ob = obs[oi].bounds;
            if (lb[0] <= ob[3] && lb[3] >= ob[0] &&
                lb[1] <= ob[4] && lb[4] >= ob[1] &&
                lb[2] <= ob[5] && lb[5] >= ob[2])
                return true;
        }
    }
    return false;
}

// ══════════════════════════════════════════════════════════════════════════�?
//  Occupation management
// ══════════════════════════════════════════════════════════════════════════�?

void LECT::mark_occupied(int node_idx, int box_id) {
    assert(node_idx >= 0 && node_idx < n_nodes_);
    forest_id_[node_idx] = box_id;
    // Walk up parent chain incrementing subtree occupation count
    for (int p = node_idx; p >= 0; p = parent_[p])
        subtree_occ_[p]++;
}

void LECT::unmark_occupied(int node_idx) {
    assert(node_idx >= 0 && node_idx < n_nodes_);
    forest_id_[node_idx] = -1;
    // Walk up parent chain decrementing subtree occupation count
    for (int p = node_idx; p >= 0; p = parent_[p])
        subtree_occ_[p] = std::max(0, subtree_occ_[p] - 1);
}

void LECT::clear_all_occupation() {
    std::fill(forest_id_.begin(), forest_id_.begin() + n_nodes_, -1);
    std::fill(subtree_occ_.begin(), subtree_occ_.begin() + n_nodes_, 0);
}

void LECT::clear_forest_state() {
    std::fill(forest_id_.begin(), forest_id_.begin() + n_nodes_, -1);
    std::fill(subtree_occ_.begin(), subtree_occ_.begin() + n_nodes_, 0);
}

// ══════════════════════════════════════════════════════════════════════════
//  snapshot() — deep copy for parallel workers
// ══════════════════════════════════════════════════════════════════════════


}  // namespace sbf

// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — LECT implementation
//  Two-stage modular pipeline: EndpointSource → LinkEnvelope
//  Dual-channel (SAFE / UNSAFE) per-node flat-buffer storage.
//  link iAABBs cached at envelope compute time (zero-cost on hot path).
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/forest/lect.h"
#include "sbf/core/joint_symmetry.h"
#include "sbf/envelope/envelope_derive.h"
#include "sbf/envelope/envelope_cache.h"
#include "sbf/envelope/endpoint_source.h"
#include "sbf/robot/interval_fk.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <unordered_map>

namespace sbf {
namespace forest {

// ═══════════════════════════════════════════════════════════════════════════
//  Channel utilities
// ═══════════════════════════════════════════════════════════════════════════

ChannelIdx LECT::source_channel(envelope::EndpointSource src) {
    return (envelope::source_safety_class(src) == envelope::SourceSafetyClass::UNSAFE)
        ? ChannelIdx::UNSAFE : ChannelIdx::SAFE;
}

ChannelIdx LECT::active_channel() const {
    return source_channel(pipeline_config_.source.method);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Constructors
// ═══════════════════════════════════════════════════════════════════════════

LECT::LECT(const Robot& robot, double voxel_delta, int initial_cap)
    : robot_(&robot), voxel_delta_(voxel_delta) {
    auto syms = detect_joint_symmetries(robot);
    if (!syms.empty()) symmetry_q0_ = syms[0];
    // Auto-enable Z4 cache for robots with Z4 rotation symmetry
    if (symmetry_q0_.type == JointSymmetryType::Z4_ROTATION)
        z4_cache_active_ = true;
    init_root(initial_cap);
}

LECT::LECT(const Robot& robot, const envelope::PipelineConfig& pipeline,
           int initial_cap)
    : robot_(&robot), pipeline_config_(pipeline) {
    voxel_delta_ = pipeline.envelope.delta;
    auto syms = detect_joint_symmetries(robot);
    if (!syms.empty()) symmetry_q0_ = syms[0];
    // Auto-enable Z4 cache for robots with Z4 rotation symmetry
    if (symmetry_q0_.type == JointSymmetryType::Z4_ROTATION)
        z4_cache_active_ = true;
    init_root(initial_cap);
}

// ═══════════════════════════════════════════════════════════════════════════
//  init_split_dims — KD-tree dimension cycling sequence
// ═══════════════════════════════════════════════════════════════════════════

void LECT::init_split_dims(int n_joints) {
    split_dims_.clear();

    switch (split_order_) {
    case SplitOrder::ROUND_ROBIN:
        split_dims_.resize(n_joints);
        std::iota(split_dims_.begin(), split_dims_.end(), 0);
        break;

    case SplitOrder::WIDEST_FIRST:
        // Empty: dimension determined dynamically in split_leaf
        break;

    case SplitOrder::BEST_TIGHTEN:
        // Per-depth lazy selection: -1 = not yet decided
        split_dims_.assign(128, -1);
        break;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  set_split_order — change dimension selection strategy
// ═══════════════════════════════════════════════════════════════════════════

void LECT::set_split_order(SplitOrder so) {
    split_order_ = so;
    init_split_dims(robot_->n_joints());
}

// ═══════════════════════════════════════════════════════════════════════════
//  pick_best_tighten_dim — iFK probe dimension selection
//
//  For each candidate dimension d, construct left/right child intervals,
//  compute incremental FK, extract per-link iAABBs, sum volumes.
//  Return the dimension that minimises a width-aware max(vol_left, vol_right)
//  minimax metric (choose the split dimension whose worse child has the
//  smallest total link AABB volume).
// ═══════════════════════════════════════════════════════════════════════════

int LECT::pick_best_tighten_dim(
    const FKState& parent_fk,
    const std::vector<Interval>& intervals) const
{
    const int nj = robot_->n_joints();
    const int nact = robot_->n_active_links();
    const int* alm = robot_->active_link_map();
    const double* radii = robot_->active_link_radii();

    // Fallback: widest-first
    int best_dim = 0;
    for (int d = 1; d < nj; ++d) {
        double w = intervals[d].width();
        if (w > intervals[best_dim].width())
            best_dim = d;
    }

    double best_metric = std::numeric_limits<double>::infinity();

    // Stack-allocated AABB buffers (MAX_TF active links × 6 floats)
    float left_aabbs[MAX_TF * 6];
    float right_aabbs[MAX_TF * 6];

    for (int d = 0; d < nj; ++d) {
        if (intervals[d].width() <= 0.0) continue;

        double mid_d = intervals[d].center();

        // ── Probe left child ────────────────────────────────────────
        auto left_ivs = intervals;
        left_ivs[d].hi = mid_d;
        FKState left_fk = compute_fk_incremental(
            parent_fk, *robot_, left_ivs, d);
        extract_link_aabbs(left_fk, alm, nact, left_aabbs, radii);

        // ── Probe right child ───────────────────────────────────────
        auto right_ivs = intervals;
        right_ivs[d].lo = mid_d;
        FKState right_fk = compute_fk_incremental(
            parent_fk, *robot_, right_ivs, d);
        extract_link_aabbs(right_fk, alm, nact, right_aabbs, radii);

        // ── Compute per-link volumes and aggregate ──────────────────
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

// ═══════════════════════════════════════════════════════════════════════════
//  init_root — allocate root node and compute its envelope
// ═══════════════════════════════════════════════════════════════════════════

void LECT::init_root(int initial_cap) {
    const int nj = robot_->n_joints();
    const int n_act = robot_->n_active_links();

    root_limits_ = robot_->joint_limits();

    store_.init(nj, n_act, initial_cap);
    int root_idx = store_.alloc_node();
    store_.set_depth(root_idx, 0);
    store_.set_parent(root_idx, -1);

    init_split_dims(nj);

    // Set flat-buffer strides BEFORE ensure_channel_capacity
    ep_stride_     = robot_->n_active_endpoints() * 6;
    liaabb_stride_  = n_act * 6;

    ensure_channel_capacity(initial_cap);
    node_split_dim_.resize(initial_cap, -1);

    // Pre-cache link radii as float (avoids per-call double→float in hot loop)
    {
        const double* lr = robot_->active_link_radii();
        if (lr) {
            radii_f_cached_.resize(n_act);
            for (int i = 0; i < n_act; ++i)
                radii_f_cached_[i] = static_cast<float>(lr[i]);
        }
    }

    // Pre-size scratch buffer for derive_link_iaabbs / get_link_iaabbs
    link_iaabbs_scratch_.resize(n_act * 6);

    root_fk_ = compute_fk_full(*robot_, root_limits_.limits);
    compute_envelope(root_idx, root_fk_, root_limits_.limits);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Z4 symmetry helpers (file-local)
// ═══════════════════════════════════════════════════════════════════════════

/// Canonicalize q_0 interval for Z4 symmetry.
/// Returns sector (0-3): how many π/2 steps the canonical is shifted FROM.
static int z4_canonicalize_interval(double q0_lo, double q0_hi, double period,
                                    double& can_lo, double& can_hi) {
    int k = static_cast<int>(std::floor(q0_lo / period));
    can_lo = q0_lo - k * period;
    can_hi = q0_hi - k * period;
    return ((k % 4) + 4) % 4;
}

/// FNV-1a hash over canonical q_0 interval + q_1..q_n intervals.
static uint64_t z4_interval_hash(double can_lo, double can_hi,
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

// ═══════════════════════════════════════════════════════════════════════════
//  compute_envelope — per-node two-stage pipeline
//
//  Stage 1: Compute endpoint iAABBs in Cartesian frame
//  Stage 2: Derive per-link iAABBs from endpoint iAABBs
//
//  IFK fast path: when source is IFK and fk.valid, extract endpoints
//  directly from the already-computed FKState (avoids redundant FK).
//  With changed_dim + parent_idx, inherits unchanged frames from parent
//  (partial FK inheritance — pathway A).
//
//  Z4 optimisation: when z4_cache_active_ and sector ≠ 0, derive from cache.
// ═══════════════════════════════════════════════════════════════════════════

void LECT::compute_envelope(int node_idx, const FKState& fk,
                            const std::vector<Interval>& intervals,
                            int changed_dim, int parent_idx) {
    const auto requested_source = pipeline_config_.source.method;
    const int ch = static_cast<int>(source_channel(requested_source));

    // ── Cache check: skip if existing data has compatible source ────────
    auto& chan = channels_[ch];
    if (node_idx < static_cast<int>(chan.has_data.size()) &&
        chan.has_data[node_idx]) {
        if (envelope::source_quality_sufficient(
                chan.source_quality[node_idx], requested_source))
            return;
    }

    const int n_act    = robot_->n_active_links();
    const int n_act_ep = robot_->n_active_endpoints();
    const uint8_t source_byte = static_cast<uint8_t>(requested_source);

    // ── Z4 cache lookup (short-circuit: only probe when cache non-empty
    //    AND sector ≠ 0, avoiding hash + map lookup in the common case) ──
    if (z4_cache_active_ && !z4_cache_.empty() &&
        symmetry_q0_.type == JointSymmetryType::Z4_ROTATION) {

        double can_lo, can_hi;
        int sector = z4_canonicalize_interval(
            intervals[0].lo, intervals[0].hi,
            symmetry_q0_.period, can_lo, can_hi);

        // Only non-canonical sectors (sector ≠ 0) can benefit from cache
        if (sector != 0) {
        uint64_t key = z4_interval_hash(can_lo, can_hi, intervals);
        auto it = z4_cache_.find(key);

        if (it != z4_cache_.end()) {
            const auto& entry = it->second;

            // Z4 cache source must be compatible with requested source
            if (envelope::source_can_serve(entry.source, requested_source)) {
                // Cache HIT — derive endpoint iAABBs from canonical via Z4 transform
                ensure_channel_capacity(node_idx + 1);
                float* ep_out = chan.ep_data.data() + node_idx * ep_stride_;
                symmetry_q0_.transform_all_endpoint_iaabbs(
                    entry.ep_iaabbs.data(), n_act_ep, sector, ep_out);

                // Derive and cache link iAABBs
                const float* rp = radii_f_cached_.empty() ? nullptr : radii_f_cached_.data();
                envelope::derive_aabb_paired(
                    ep_out, n_act, rp,
                    chan.link_iaabb_cache.data() + node_idx * liaabb_stride_);

                chan.has_data[node_idx] = 1;
                chan.source_quality[node_idx] = source_byte;

                // Invalidate hull grid (must be re-derived after Z4 transform)
                if (node_idx < static_cast<int>(chan.hull_valid.size()))
                    chan.hull_valid[node_idx] = 0;

                return;
            }
        }
        } // sector != 0
    }

    // ── IFK fast path: extract endpoints directly from FKState ──────────
    //    Avoids redundant compute_fk_full inside compute_endpoint_iaabb.
    //    Pathway A: when changed_dim >= 0, inherit unchanged frames from parent.
    if (requested_source == envelope::EndpointSource::IFK && fk.valid) {
        ensure_channel_capacity(node_idx + 1);
        float* ep_out = chan.ep_data.data() + node_idx * ep_stride_;
        const int* alm = robot_->active_link_map();

        if (changed_dim >= 0 && parent_idx >= 0 &&
            parent_idx < static_cast<int>(chan.has_data.size()) &&
            chan.has_data[parent_idx]) {
            // ── Partial inheritance (Pathway A) ─────────────────────────
            // For serial chain: active link ci with V = active_link_map[ci]
            //   proximal frame = prefix[V], distal frame = prefix[V+1]
            //   A link is fully unchanged iff V+1 <= changed_dim (both frames
            //   only depend on joints 0..V, which weren't split).
            const float* parent_ep = chan.ep_data.data() + parent_idx * ep_stride_;

            // Find boundary: how many leading paired entries to inherit
            int inherit_pairs = 0;  // number of complete (prox+dist) pairs to inherit
            for (int ci = 0; ci < n_act; ++ci) {
                if (alm[ci] + 1 <= changed_dim)
                    inherit_pairs = ci + 1;
                else
                    break;
            }

            // Copy unchanged paired entries from parent
            if (inherit_pairs > 0)
                std::memcpy(ep_out, parent_ep,
                            inherit_pairs * 12 * sizeof(float));

            // Extract changed entries from FKState
            for (int ci = inherit_pairs; ci < n_act; ++ci) {
                int V = alm[ci];
                float* p = ep_out + ci * 12;
                // Proximal: prefix[V]
                p[0]  = static_cast<float>(fk.prefix_lo[V][3]);
                p[1]  = static_cast<float>(fk.prefix_lo[V][7]);
                p[2]  = static_cast<float>(fk.prefix_lo[V][11]);
                p[3]  = static_cast<float>(fk.prefix_hi[V][3]);
                p[4]  = static_cast<float>(fk.prefix_hi[V][7]);
                p[5]  = static_cast<float>(fk.prefix_hi[V][11]);
                // Distal: prefix[V+1]
                p[6]  = static_cast<float>(fk.prefix_lo[V+1][3]);
                p[7]  = static_cast<float>(fk.prefix_lo[V+1][7]);
                p[8]  = static_cast<float>(fk.prefix_lo[V+1][11]);
                p[9]  = static_cast<float>(fk.prefix_hi[V+1][3]);
                p[10] = static_cast<float>(fk.prefix_hi[V+1][7]);
                p[11] = static_cast<float>(fk.prefix_hi[V+1][11]);
            }
        } else {
            // ── Full extraction from FKState (no parent to inherit from) ─
            for (int ci = 0; ci < n_act; ++ci) {
                int V = alm[ci];
                float* p = ep_out + ci * 12;
                p[0]  = static_cast<float>(fk.prefix_lo[V][3]);
                p[1]  = static_cast<float>(fk.prefix_lo[V][7]);
                p[2]  = static_cast<float>(fk.prefix_lo[V][11]);
                p[3]  = static_cast<float>(fk.prefix_hi[V][3]);
                p[4]  = static_cast<float>(fk.prefix_hi[V][7]);
                p[5]  = static_cast<float>(fk.prefix_hi[V][11]);
                p[6]  = static_cast<float>(fk.prefix_lo[V+1][3]);
                p[7]  = static_cast<float>(fk.prefix_lo[V+1][7]);
                p[8]  = static_cast<float>(fk.prefix_lo[V+1][11]);
                p[9]  = static_cast<float>(fk.prefix_hi[V+1][3]);
                p[10] = static_cast<float>(fk.prefix_hi[V+1][7]);
                p[11] = static_cast<float>(fk.prefix_hi[V+1][11]);
            }
        }

        // Derive and cache link iAABBs
        const float* rp = radii_f_cached_.empty() ? nullptr : radii_f_cached_.data();
        envelope::derive_aabb_paired(
            ep_out, n_act, rp,
            chan.link_iaabb_cache.data() + node_idx * liaabb_stride_);

        chan.has_data[node_idx] = 1;
        chan.source_quality[node_idx] = source_byte;

        if (node_idx < static_cast<int>(chan.hull_valid.size()))
            chan.hull_valid[node_idx] = 0;

        // ── Z4 cache populate (canonical sector only) ───────────────────
        if (z4_cache_active_ &&
            symmetry_q0_.type == JointSymmetryType::Z4_ROTATION) {
            double can_lo, can_hi;
            int sector = z4_canonicalize_interval(
                intervals[0].lo, intervals[0].hi,
                symmetry_q0_.period, can_lo, can_hi);
            if (sector == 0) {
                uint64_t key = z4_interval_hash(can_lo, can_hi, intervals);
                auto it = z4_cache_.find(key);
                if (it == z4_cache_.end() ||
                    !envelope::source_can_serve(it->second.source,
                                                requested_source)) {
                    Z4CacheEntry entry;
                    entry.ep_iaabbs.assign(ep_out, ep_out + ep_stride_);
                    entry.source = requested_source;
                    z4_cache_[key] = std::move(entry);
                }
            }
        }
        return;
    }

    // ── Non-IFK fallback: full compute_endpoint_iaabb ───────────────────
    auto ep_result = envelope::compute_endpoint_iaabb(
        pipeline_config_.source, *robot_, intervals);

    // Store Cartesian-frame endpoint iAABBs into flat buffer
    ensure_channel_capacity(node_idx + 1);
    float* ep_out = chan.ep_data.data() + node_idx * ep_stride_;
    std::memcpy(ep_out, ep_result.endpoint_iaabbs.data(),
                ep_stride_ * sizeof(float));

    // Derive and cache link iAABBs
    {
        const float* rp = radii_f_cached_.empty() ? nullptr : radii_f_cached_.data();
        envelope::derive_aabb_paired(
            ep_out, n_act, rp,
            chan.link_iaabb_cache.data() + node_idx * liaabb_stride_);
    }

    chan.has_data[node_idx] = 1;
    chan.source_quality[node_idx] = source_byte;

    // Invalidate hull grid (AABB source changed → hull re-derive needed)
    if (node_idx < static_cast<int>(chan.hull_valid.size()))
        chan.hull_valid[node_idx] = 0;

    // ── Populate Z4 cache (canonical sector only) ───────────────────────
    if (z4_cache_active_ &&
        symmetry_q0_.type == JointSymmetryType::Z4_ROTATION) {

        double can_lo, can_hi;
        int sector = z4_canonicalize_interval(
            intervals[0].lo, intervals[0].hi,
            symmetry_q0_.period, can_lo, can_hi);

        if (sector == 0) {
            uint64_t key = z4_interval_hash(can_lo, can_hi, intervals);
            auto it = z4_cache_.find(key);
            // Insert or upgrade: replace if new source is better
            if (it == z4_cache_.end() ||
                !envelope::source_can_serve(it->second.source,
                                            requested_source)) {
                Z4CacheEntry entry;
                entry.ep_iaabbs.assign(ep_out, ep_out + ep_stride_);
                entry.source = requested_source;
                z4_cache_[key] = std::move(entry);
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  pick_split_dim — single source of truth for dimension selection
// ═══════════════════════════════════════════════════════════════════════════

int LECT::pick_split_dim(int depth, const FKState& fk,
                         const std::vector<Interval>& intervals) {
    if (split_order_ == SplitOrder::WIDEST_FIRST) {
        int dim = 0;
        double max_w = intervals[0].width();
        for (int d = 1; d < n_dims(); ++d) {
            double w = intervals[d].width();
            if (w > max_w) { max_w = w; dim = d; }
        }
        return dim;
    }
    if (split_order_ == SplitOrder::BEST_TIGHTEN) {
        if (depth >= static_cast<int>(split_dims_.size()))
            split_dims_.resize(static_cast<size_t>(depth) + 1, -1);
        if (split_dims_[depth] < 0)
            split_dims_[depth] = pick_best_tighten_dim(fk, intervals);
        return split_dims_[depth];
    }
    // ROUND_ROBIN
    return split_dims_[depth % static_cast<int>(split_dims_.size())];
}

// ═══════════════════════════════════════════════════════════════════════════
//  split_leaf — split a leaf node into two children
//
//  Always: incremental FK + full compute_envelope for both children
// ═══════════════════════════════════════════════════════════════════════════

void LECT::split_leaf(int node_idx, const FKState& parent_fk,
                      const std::vector<Interval>& parent_intervals,
                      int dim) {
    const int parent_depth = store_.depth(node_idx);

    // ── Determine split dimension (if not provided by caller) ───────────
    if (dim < 0)
        dim = pick_split_dim(parent_depth, parent_fk, parent_intervals);

    // ── Compute midpoint ────────────────────────────────────────────────
    double mid = parent_intervals[dim].center();

    // ── Z4: prefer sector boundary (kπ/2) over midpoint for q_0 ────────
    if (dim == 0 &&
        symmetry_q0_.type == JointSymmetryType::Z4_ROTATION) {
        const double lo = parent_intervals[0].lo;
        const double hi = parent_intervals[0].hi;
        const double period = symmetry_q0_.period;

        // Find the nearest sector boundary within (lo, hi)
        double best_boundary = mid;
        double best_dist = std::numeric_limits<double>::infinity();
        int k_lo = static_cast<int>(std::ceil(lo / period));
        int k_hi = static_cast<int>(std::floor(hi / period));
        for (int k = k_lo; k <= k_hi; ++k) {
            double boundary = k * period;
            // Must be strictly inside (lo, hi) with some margin
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

    // ── Allocate children ───────────────────────────────────────────────
    int left_idx  = store_.alloc_node();
    int right_idx = store_.alloc_node();

    store_.set_left(node_idx, left_idx);
    store_.set_right(node_idx, right_idx);
    store_.set_parent(left_idx, node_idx);
    store_.set_parent(right_idx, node_idx);
    store_.set_depth(left_idx, parent_depth + 1);
    store_.set_depth(right_idx, parent_depth + 1);
    store_.set_split(node_idx, mid);

    // Record split dimension for node_intervals
    if (node_idx >= static_cast<int>(node_split_dim_.size()))
        node_split_dim_.resize(static_cast<size_t>(node_idx) + 1, -1);
    node_split_dim_[node_idx] = dim;

    // ── Build child intervals ───────────────────────────────────────────
    auto left_ivs  = parent_intervals;
    left_ivs[dim].hi = mid;
    auto right_ivs = parent_intervals;
    right_ivs[dim].lo = mid;

    // Ensure storage capacity
    int max_idx = std::max(left_idx, right_idx);
    ensure_channel_capacity(max_idx + 1);
    if (max_idx >= static_cast<int>(node_split_dim_.size()))
        node_split_dim_.resize(static_cast<size_t>(max_idx) + 1, -1);

    // ── Incremental FK + envelope (IFK fast path + partial inheritance) ─
    FKState left_fk  = compute_fk_incremental(
        parent_fk, *robot_, left_ivs, dim);
    FKState right_fk = compute_fk_incremental(
        parent_fk, *robot_, right_ivs, dim);

    compute_envelope(left_idx,  left_fk,  left_ivs,  dim, node_idx);
    compute_envelope(right_idx, right_fk, right_ivs, dim, node_idx);

    // ── Tighten parent AABBs ────────────────────────────────────────────
    refine_aabb(node_idx);
}

// ═══════════════════════════════════════════════════════════════════════════
//  refine_aabb — intersect(parent, union(left, right))
// ═══════════════════════════════════════════════════════════════════════════

void LECT::refine_aabb(int parent_idx) {
    int li = store_.left(parent_idx);
    int ri = store_.right(parent_idx);
    if (li < 0 || ri < 0) return;

    // Refine endpoint iAABBs in the active channel:
    // parent_ep = intersect(parent_ep, union(left_ep, right_ep))
    const int ch = static_cast<int>(active_channel());
    auto& chan = channels_[ch];

    if (parent_idx >= static_cast<int>(chan.has_data.size()) ||
        !chan.has_data[parent_idx]) return;
    if (li >= static_cast<int>(chan.has_data.size()) ||
        !chan.has_data[li]) return;
    if (ri >= static_cast<int>(chan.has_data.size()) ||
        !chan.has_data[ri]) return;

    float*       pa = chan.ep_data.data() + parent_idx * ep_stride_;
    const float* la = chan.ep_data.data() + li * ep_stride_;
    const float* ra = chan.ep_data.data() + ri * ep_stride_;

    for (int i = 0; i < ep_stride_; ++i) {
        if (i % 6 < 3) {
            // lo: parent = max(parent, min(left, right))
            float u = std::min(la[i], ra[i]);
            pa[i] = std::max(pa[i], u);
        } else {
            // hi: parent = min(parent, max(left, right))
            float u = std::max(la[i], ra[i]);
            pa[i] = std::min(pa[i], u);
        }
    }

    // Re-derive cached link iAABBs after refinement
    const float* rp = radii_f_cached_.empty() ? nullptr : radii_f_cached_.data();
    const int n_act = robot_->n_active_links();
    envelope::derive_aabb_paired(
        pa, n_act, rp,
        chan.link_iaabb_cache.data() + parent_idx * liaabb_stride_);
}

// ═══════════════════════════════════════════════════════════════════════════
//  node_intervals — reconstruct C-space intervals by walking root → node
// ═══════════════════════════════════════════════════════════════════════════

std::vector<Interval> LECT::node_intervals(int node_idx) const {
    auto ivs = root_limits_.limits;

    // Build path from root to node
    std::vector<int> path;
    for (int n = node_idx; n >= 0; n = store_.parent(n))
        path.push_back(n);
    std::reverse(path.begin(), path.end());

    // Walk root → node, narrowing intervals at each split
    for (int i = 0; i + 1 < static_cast<int>(path.size()); ++i) {
        int p = path[i];
        int child = path[i + 1];

        // Determine split dimension
        int dim;
        if (p < static_cast<int>(node_split_dim_.size()) &&
            node_split_dim_[p] >= 0) {
            dim = node_split_dim_[p];
        } else if (!split_dims_.empty()) {
            dim = split_dims_[store_.depth(p) %
                  static_cast<int>(split_dims_.size())];
        } else {
            dim = store_.depth(p) % store_.n_dims();
        }

        double mid = store_.split(p);
        if (child == store_.left(p))
            ivs[dim].hi = mid;
        else
            ivs[dim].lo = mid;
    }

    return ivs;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Metadata accessors
// ═══════════════════════════════════════════════════════════════════════════

int LECT::n_nodes()        const { return store_.n_nodes(); }
int LECT::n_dims()         const { return store_.n_dims(); }
int LECT::n_active_links() const { return store_.n_active_links(); }
int LECT::capacity()       const { return store_.capacity(); }
double LECT::voxel_delta() const { return voxel_delta_; }

const Robot& LECT::robot() const { return *robot_; }
const JointLimits& LECT::root_limits() const { return root_limits_; }
const envelope::PipelineConfig& LECT::pipeline_config() const { return pipeline_config_; }
envelope::EndpointSource LECT::source_method() const { return pipeline_config_.source.method; }
envelope::EnvelopeType   LECT::envelope_type() const { return pipeline_config_.envelope.type; }

// ═══════════════════════════════════════════════════════════════════════════
//  Per-node data access — dual-channel, on-the-fly link iAABB derivation
// ═══════════════════════════════════════════════════════════════════════════

bool LECT::derive_link_iaabbs(int node_idx, float* out, ChannelIdx ch) const {
    const int ci = static_cast<int>(ch);
    const auto& chan = channels_[ci];
    if (node_idx < 0 ||
        node_idx >= static_cast<int>(chan.has_data.size()) ||
        !chan.has_data[node_idx])
        return false;

    const int n_act = robot_->n_active_links();
    // Use pre-cached float radii (set once in init_root) — zero per-call alloc
    const float* radii_ptr = radii_f_cached_.empty() ? nullptr
                                                     : radii_f_cached_.data();

    envelope::derive_aabb_paired(
        chan.ep_data.data() + node_idx * ep_stride_, n_act,
        radii_ptr, out);
    return true;
}

const float* LECT::get_link_iaabbs(int node_idx) const {
    const int n_act = robot_->n_active_links();
    link_iaabbs_scratch_.resize(n_act * 6);
    if (derive_link_iaabbs(node_idx, link_iaabbs_scratch_.data(),
                           active_channel()))
        return link_iaabbs_scratch_.data();
    return nullptr;
}

bool LECT::has_iaabb(int node_idx) const {
    return has_channel_data(node_idx, active_channel());
}

bool LECT::has_channel_data(int node_idx, ChannelIdx ch) const {
    const int ci = static_cast<int>(ch);
    const auto& chan = channels_[ci];
    return node_idx >= 0 &&
           node_idx < static_cast<int>(chan.has_data.size()) &&
           chan.has_data[node_idx] != 0;
}

const float* LECT::get_endpoint_iaabbs(int node_idx, ChannelIdx ch) const {
    const int ci = static_cast<int>(ch);
    const auto& chan = channels_[ci];
    if (node_idx < 0 ||
        node_idx >= static_cast<int>(chan.has_data.size()) ||
        !chan.has_data[node_idx])
        return nullptr;
    return chan.ep_data.data() + node_idx * ep_stride_;
}

bool LECT::has_endpoint_iaabbs(int node_idx, ChannelIdx ch) const {
    const int ci = static_cast<int>(ch);
    const auto& chan = channels_[ci];
    return node_idx >= 0
        && node_idx < static_cast<int>(chan.has_data.size())
        && chan.has_data[node_idx] != 0;
}

bool LECT::has_hull_grid(int node_idx, ChannelIdx ch) const {
    const int ci = static_cast<int>(ch);
    const auto& chan = channels_[ci];
    if (node_idx < 0 ||
        node_idx >= static_cast<int>(chan.hull_valid.size()))
        return false;
    return chan.hull_valid[node_idx] != 0;
}

const voxel::VoxelGrid& LECT::get_hull_grid(int node_idx, ChannelIdx ch) const {
    const int ci = static_cast<int>(ch);
    return channels_[ci].hull_grids[node_idx];
}

// ═══════════════════════════════════════════════════════════════════════════
//  Tree structure — delegate to store_
// ═══════════════════════════════════════════════════════════════════════════

int  LECT::left(int i)       const { return store_.left(i); }
int  LECT::right(int i)      const { return store_.right(i); }
int  LECT::parent(int i)     const { return store_.parent(i); }
int  LECT::depth(int i)      const { return store_.depth(i); }
double LECT::split_val(int i) const { return store_.split(i); }
bool LECT::is_leaf(int i)    const { return store_.is_leaf(i); }

int LECT::get_node_split_dim(int node_idx) const {
    // 1. Authoritative per-node record (set by split_leaf)
    if (node_idx < static_cast<int>(node_split_dim_.size()) &&
        node_split_dim_[node_idx] >= 0) {
        return node_split_dim_[node_idx];
    }
    // 2. Split-dims table (round-robin / lazy BEST_TIGHTEN)
    int d = store_.depth(node_idx);
    if (!split_dims_.empty()) {
        int sd = split_dims_[d % static_cast<int>(split_dims_.size())];
        if (sd >= 0) return sd;
    }
    // 3. Last resort: round-robin
    return d % n_dims();
}

// ═══════════════════════════════════════════════════════════════════════════
//  Scene management (stubs — will be implemented with collision module)
// ═══════════════════════════════════════════════════════════════════════════

void LECT::set_scene(const Obstacle* obstacles, int n_obs) {
    scene_grid_ = voxel::VoxelGrid(voxel_delta_);
    for (int j = 0; j < n_obs; ++j) {
        Eigen::Vector3d lo = obstacles[j].lo();
        Eigen::Vector3d hi = obstacles[j].hi();
        float aabb[6] = {
            static_cast<float>(lo[0]), static_cast<float>(lo[1]),
            static_cast<float>(lo[2]), static_cast<float>(hi[0]),
            static_cast<float>(hi[1]), static_cast<float>(hi[2])
        };
        scene_grid_.fill_aabb(aabb);
    }
    scene_set_ = true;
}
void LECT::clear_scene() { scene_set_ = false; }
bool LECT::has_scene_grid() const { return scene_set_; }
const voxel::VoxelGrid& LECT::scene_grid() const { return scene_grid_; }

void LECT::set_deadline(std::optional<TimePoint> deadline) {
    deadline_ = std::move(deadline);
}

bool LECT::deadline_reached() const {
    return deadline_.has_value() && Clock::now() >= *deadline_;
}

// ═══════════════════════════════════════════════════════════════════════════
//  expand_leaf — public wrapper for split_leaf
// ═══════════════════════════════════════════════════════════════════════════

int LECT::expand_leaf(int node_idx) {
    if (!store_.is_leaf(node_idx)) return 0;

    auto ivs = node_intervals(node_idx);
    FKState fk = compute_fk_full(*robot_, ivs);
    int dim = pick_split_dim(store_.depth(node_idx), fk, ivs);
    split_leaf(node_idx, fk, ivs, dim);
    return 2;
}

// ═══════════════════════════════════════════════════════════════════════════
//  find_free_box — descend KD-tree lazily toward seed
// ═══════════════════════════════════════════════════════════════════════════

FFBResult LECT::find_free_box(const Eigen::VectorXd& seed,
                               const Obstacle* obstacles, int n_obs,
                               double min_edge, int max_depth) {
    FFBResult result;
    result.path.reserve(max_depth + 1);

    int cur = 0;
    FKState fk = root_fk_;
    std::vector<Interval> intervals = root_limits_.limits;
    const ChannelIdx ach = active_channel();

    while (true) {
        if (deadline_reached()) {
            result.fail_code = 4;
            return result;
        }
        result.path.push_back(cur);

        // ── Occupied check ──────────────────────────────────────────
        if (store_.is_occupied(cur)) {
            result.fail_code = 1;
            return result;
        }

        // ── iAABB collision check ───────────────────────────────────
        if (has_channel_data(cur, ach)) {
            bool aabb_hit = iaabbs_collide(cur, obstacles, n_obs);

            if (!aabb_hit && store_.subtree_occ(cur) == 0) {
                result.node_idx = cur;
                result.fail_code = 0;
                propagate_up(result.path);
                return result;
            }

            // Refinement via hull-16
            if (aabb_hit) {
                bool need_hull =
                    pipeline_config_.envelope.type == envelope::EnvelopeType::LinkIAABB_Grid ||
                    pipeline_config_.envelope.type == envelope::EnvelopeType::Hull16_Grid;

                if (need_hull && hull_skip_vol_ > 0.0) {
                    double node_vol = 1.0;
                    for (auto& iv : intervals) node_vol *= iv.width();
                    if (node_vol < hull_skip_vol_) need_hull = false;
                }

                if (need_hull) {
                    if (deadline_reached()) {
                        result.fail_code = 4;
                        return result;
                    }
                    if (!has_hull_grid(cur, ach))
                        derive_hull_grid(cur, intervals, ach);
                    if (deadline_reached()) {
                        result.fail_code = 4;
                        return result;
                    }
                    if (has_hull_grid(cur, ach)) {
                        bool hull_hit = hull_collides(cur, obstacles, n_obs);
                        if (!hull_hit && store_.subtree_occ(cur) == 0) {
                            result.node_idx = cur;
                            result.fail_code = 0;
                            propagate_up(result.path);
                            return result;
                        }
                    }
                }
            }
        }

        // ── Leaf node: try to split ─────────────────────────────────
        if (store_.is_leaf(cur)) {
            int d = store_.depth(cur);
            int dim = pick_split_dim(d, fk, intervals);
            double edge = intervals[dim].width();

            if (d >= max_depth) {
                result.fail_code = 2;
                return result;
            }
            if (edge <= min_edge) {
                result.fail_code = 3;
                return result;
            }

            if (deadline_reached()) {
                result.fail_code = 4;
                return result;
            }
            split_leaf(cur, fk, intervals, dim);
            result.n_new_nodes += 2;
        }

        // ── Descend toward seed ─────────────────────────────────────
        int d = store_.depth(cur);
        int dim;
        if (cur < static_cast<int>(node_split_dim_.size()) &&
            node_split_dim_[cur] >= 0) {
            dim = node_split_dim_[cur];
        } else if (!split_dims_.empty()) {
            dim = split_dims_[d % static_cast<int>(split_dims_.size())];
        } else {
            dim = d % n_dims();
        }
        double mid = store_.split(cur);
        auto child_ivs = intervals;
        int child;

        if (seed[dim] < mid) {
            child = store_.left(cur);
            child_ivs[dim].hi = mid;
        } else {
            child = store_.right(cur);
            child_ivs[dim].lo = mid;
        }

        FKState child_fk = compute_fk_incremental(fk, *robot_, child_ivs, dim);
        result.n_fk_calls++;
        if (deadline_reached()) {
            result.fail_code = 4;
            return result;
        }
        compute_envelope(child, child_fk, child_ivs, dim, cur);
        if (deadline_reached()) {
            result.fail_code = 4;
            return result;
        }
        fk = child_fk;
        intervals = child_ivs;
        cur = child;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Occupation — delegate to store_
// ═══════════════════════════════════════════════════════════════════════════

void LECT::mark_occupied(int node_idx, int box_id) {
    store_.set_occupied(node_idx, box_id);
    for (int p = node_idx; p >= 0; p = store_.parent(p))
        store_.set_subtree_occ(p, store_.subtree_occ(p) + 1);
}
void LECT::unmark_occupied(int node_idx) {
    store_.set_unoccupied(node_idx);
    for (int p = node_idx; p >= 0; p = store_.parent(p))
        store_.set_subtree_occ(p, std::max(int32_t(0), store_.subtree_occ(p) - 1));
}
bool LECT::is_occupied(int node_idx) const {
    return store_.is_occupied(node_idx);
}
int LECT::forest_id(int node_idx) const {
    return store_.forest_id(node_idx);
}
void LECT::clear_all_occupation() {
    store_.clear_all_occupation();
}

// ═══════════════════════════════════════════════════════════════════════════
//  Warm-start — snapshot / pre_expand / compute_all_hull_grids
// ═══════════════════════════════════════════════════════════════════════════

LECT LECT::snapshot() const {
    LECT copy;
    copy.robot_            = robot_;
    copy.voxel_delta_      = voxel_delta_;
    copy.hull_skip_vol_    = hull_skip_vol_;
    copy.pipeline_config_  = pipeline_config_;
    copy.split_order_      = split_order_;
    copy.store_            = store_.snapshot();
    for (int c = 0; c < NUM_CHANNELS; ++c)
        copy.channels_[c]  = channels_[c];
    copy.scene_grid_       = scene_grid_;
    copy.scene_set_        = scene_set_;
    copy.root_limits_      = root_limits_;
    copy.root_fk_          = root_fk_;
    copy.split_dims_       = split_dims_;
    copy.node_split_dim_   = node_split_dim_;
    copy.ep_stride_        = ep_stride_;
    copy.liaabb_stride_    = liaabb_stride_;
    copy.radii_f_cached_   = radii_f_cached_;
    copy.symmetry_q0_      = symmetry_q0_;
    copy.z4_cache_active_  = z4_cache_active_;
    copy.deadline_         = deadline_;
    // z4_cache_ contents are transient — not copied in snapshot
    return copy;
}

int LECT::pre_expand(int target_depth) {
    if (store_.n_nodes() == 0) return 0;
    if (deadline_reached()) return 0;

    // Clear Z4 cache for fresh systematic population (canonical-first)
    if (z4_cache_active_) {
        z4_cache_.clear();
    }

    int new_count = 0;
    pre_expand_recursive(0, root_fk_, root_limits_.limits,
                         target_depth, new_count);

    // Keep Z4 cache active but don't clear — it may be reused by
    // subsequent find_free_box calls.  The cache is only cleared at
    // the START of the next pre_expand.

    return new_count;
}

int LECT::compute_all_hull_grids() {
    if (pipeline_config_.envelope.type != envelope::EnvelopeType::LinkIAABB_Grid &&
        pipeline_config_.envelope.type != envelope::EnvelopeType::Hull16_Grid)
        return 0;

    const ChannelIdx ach = active_channel();
    int count = 0;
    const int n = store_.n_nodes();
    for (int i = 0; i < n; ++i) {
        if (has_channel_data(i, ach) && !has_hull_grid(i, ach)) {
            auto ivs = node_intervals(i);
            derive_hull_grid(i, ivs, ach);
            ++count;
        }
    }
    return count;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Persistence — save / load
// ═══════════════════════════════════════════════════════════════════════════

void LECT::save(const std::string& dir) const {
    store_.save(dir + "/lect.hcache");
    save_hull_grids(dir + "/lect.hulls");

    // Persist pipeline metadata for cache validation
    envelope::CacheMetadata meta;
    meta.source_method  = pipeline_config_.source.method;
    meta.envelope_type  = pipeline_config_.envelope.type;
    meta.n_sub          = pipeline_config_.envelope.n_sub;
    meta.delta          = pipeline_config_.envelope.delta;
    meta.grid_R         = pipeline_config_.envelope.grid_R;
    meta.n_nodes        = store_.n_nodes();
    meta.n_endpoints    = 0;
    meta.n_hulls        = count_nodes_with_hull();
    meta.robot_hash     = envelope::EnvelopeCache::compute_robot_hash(*robot_);
    envelope::EnvelopeCache::save_metadata(dir, meta);
}

LECT LECT::load(const std::string& dir, const Robot& robot) {
    LECT lect;
    lect.robot_ = &robot;

    // Load tree structure (HCACHE04 — pure tree, no link_iaabbs)
    lect.store_.load(dir + "/lect.hcache");

    // Rebuild metadata
    lect.root_limits_ = robot.joint_limits();
    lect.split_order_ = SplitOrder::BEST_TIGHTEN;
    lect.init_split_dims(robot.n_joints());

    // Recompute root FK
    lect.root_fk_ = compute_fk_full(robot, lect.root_limits_.limits);

    // Set strides BEFORE ensure_channel_capacity
    lect.ep_stride_     = robot.n_active_endpoints() * 6;
    lect.liaabb_stride_  = robot.n_active_links() * 6;

    // Pre-cache link radii as float
    {
        const double* lr = robot.active_link_radii();
        if (lr) {
            int n_act = robot.n_active_links();
            lect.radii_f_cached_.resize(n_act);
            for (int i = 0; i < n_act; ++i)
                lect.radii_f_cached_[i] = static_cast<float>(lr[i]);
        }
    }

    // Resize dual-channel stores + node_split_dim
    int n = lect.store_.n_nodes();
    for (int c = 0; c < NUM_CHANNELS; ++c)
        lect.ensure_channel_capacity(n, c);
    lect.node_split_dim_.resize(n, -1);

    // Load hull grids (dual-channel HUL2, backward-compat HUL1)
    try {
        lect.load_hull_grids(dir + "/lect.hulls");
    } catch (...) {
        // Hull grids optional — missing file is OK
    }

    // Recover pipeline metadata
    envelope::CacheMetadata meta;
    if (envelope::EnvelopeCache::load_metadata(dir, meta)) {
        lect.pipeline_config_.source.method  = meta.source_method;
        lect.pipeline_config_.envelope.type  = meta.envelope_type;
        lect.pipeline_config_.envelope.n_sub = meta.n_sub;
        lect.pipeline_config_.envelope.delta = meta.delta;
        lect.pipeline_config_.envelope.grid_R = meta.grid_R;
        lect.voxel_delta_ = meta.delta;

        // Recover source quality into the appropriate channel
        uint8_t src_byte = static_cast<uint8_t>(meta.source_method);
        ChannelIdx ch = source_channel(meta.source_method);
        auto& chan = lect.channels_[static_cast<int>(ch)];
        for (int i = 0; i < n; ++i) {
            // Nodes that had data are marked in the channel
            // (has_data was not persisted in HCACHE04 — rely on hull_valid
            //  to infer which nodes had envelopes; ep_data has no data after load)
            if (i < static_cast<int>(chan.hull_valid.size()) &&
                chan.hull_valid[i]) {
                chan.source_quality[i] = src_byte;
                chan.has_data[i] = 1;
            }
        }
    }

    // Detect joint symmetry and enable Z4 cache
    auto syms = detect_joint_symmetries(robot);
    if (!syms.empty()) lect.symmetry_q0_ = syms[0];
    if (lect.symmetry_q0_.type == JointSymmetryType::Z4_ROTATION)
        lect.z4_cache_active_ = true;

    return lect;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Private helpers — storage capacity management
// ═══════════════════════════════════════════════════════════════════════════

void LECT::ensure_channel_capacity(int min_cap) {
    ensure_channel_capacity(min_cap, static_cast<int>(active_channel()));
}

void LECT::ensure_channel_capacity(int min_cap, int ch_idx) {
    auto& ch = channels_[ch_idx];
    if (static_cast<int>(ch.ep_data.size()) < min_cap * ep_stride_)
        ch.ep_data.resize(min_cap * ep_stride_, 0.f);
    if (static_cast<int>(ch.link_iaabb_cache.size()) < min_cap * liaabb_stride_)
        ch.link_iaabb_cache.resize(min_cap * liaabb_stride_, 0.f);
    if (static_cast<int>(ch.source_quality.size()) < min_cap)
        ch.source_quality.resize(min_cap, 0);
    if (static_cast<int>(ch.has_data.size()) < min_cap)
        ch.has_data.resize(min_cap, 0);
    if (static_cast<int>(ch.hull_grids.size()) < min_cap) {
        ch.hull_grids.resize(min_cap);
        ch.hull_valid.resize(min_cap, 0);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  derive_hull_grid — hull-16 VoxelGrid from endpoint iAABBs
// ═══════════════════════════════════════════════════════════════════════════

void LECT::derive_hull_grid(int node_idx,
                            const std::vector<Interval>& intervals,
                            ChannelIdx ch) {
    ensure_channel_capacity(node_idx + 1, static_cast<int>(ch));
    auto& chan = channels_[static_cast<int>(ch)];

    const int n_act = robot_->n_active_links();
    const double* lr = robot_->active_link_radii();
    if (!lr) { chan.hull_valid[node_idx] = 0; return; }

    // Channel ep_data holds paired Cartesian-frame endpoint iAABBs
    if (node_idx < static_cast<int>(chan.has_data.size()) &&
        chan.has_data[node_idx]) {

        const float* cart_ep = chan.ep_data.data() + node_idx * ep_stride_;

        voxel::VoxelGrid grid(voxel_delta_);

        // Pre-compute total bounding box for reserve
        {
            double total_lo[3] = { 1e30,  1e30,  1e30};
            double total_hi[3] = {-1e30, -1e30, -1e30};
            for (int ci = 0; ci < n_act; ++ci) {
                const float* pf = cart_ep + (ci * 2) * 6;      // proximal
                const float* df = cart_ep + (ci * 2 + 1) * 6;  // distal
                double r = lr[ci];
                for (int a = 0; a < 3; ++a) {
                    double plo = pf[a],  phi = pf[a + 3];
                    double dlo = df[a],  dhi = df[a + 3];
                    total_lo[a] = std::min(total_lo[a],
                        std::min(plo, dlo) - r);
                    total_hi[a] = std::max(total_hi[a],
                        std::max(phi, dhi) + r);
                }
            }
            double sp = std::sqrt(3.0) * voxel_delta_ * 0.5;
            for (int a = 0; a < 3; ++a) {
                total_lo[a] -= sp;
                total_hi[a] += sp;
            }
            grid.reserve_from_bounds(total_lo, total_hi);
        }

        // fill_hull16 for each active link (paired indexing)
        for (int ci = 0; ci < n_act; ++ci) {
            float prox[6], dist[6];
            std::memcpy(prox, cart_ep + (ci * 2) * 6, 6 * sizeof(float));
            std::memcpy(dist, cart_ep + (ci * 2 + 1) * 6, 6 * sizeof(float));
            grid.fill_hull16(prox, dist, lr[ci]);
        }

        chan.hull_grids[node_idx] = std::move(grid);
        chan.hull_valid[node_idx] = 1;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  pre_expand_recursive — split leaves down to target_depth
// ═══════════════════════════════════════════════════════════════════════════

void LECT::pre_expand_recursive(int node_idx, const FKState& fk,
                                const std::vector<Interval>& intervals,
                                int target_depth, int& new_node_count) {
    if (deadline_reached()) return;

    int d = store_.depth(node_idx);
    if (d >= target_depth) return;

    if (store_.is_leaf(node_idx)) {
        int dim = pick_split_dim(d, fk, intervals);
        double edge = intervals[dim].width();
        if (edge <= 1e-6) return;
        split_leaf(node_idx, fk, intervals, dim);
        new_node_count += 2;
    }

    // Determine split dim for descent
    int dim;
    if (node_idx < static_cast<int>(node_split_dim_.size()) &&
        node_split_dim_[node_idx] >= 0) {
        dim = node_split_dim_[node_idx];
    } else if (!split_dims_.empty()) {
        dim = split_dims_[d % static_cast<int>(split_dims_.size())];
    } else {
        dim = d % n_dims();
    }
    double mid = store_.split(node_idx);

    auto left_ivs = intervals;
    left_ivs[dim].hi = mid;
    auto right_ivs = intervals;
    right_ivs[dim].lo = mid;

    // ── Z4 canonical-first ordering: expand the canonical sector before
    //    non-canonical, so Z4 cache is populated when the non-canonical
    //    child is processed.  Canonical has q_0 center closer to 0. ──────
    bool right_first = false;
    if (dim == 0 && z4_cache_active_ &&
        symmetry_q0_.type == JointSymmetryType::Z4_ROTATION) {
        double left_center  = (left_ivs[0].lo  + left_ivs[0].hi)  / 2;
        double right_center = (right_ivs[0].lo + right_ivs[0].hi) / 2;
        // Prefer the child whose center is closer to canonical [0, π/2]
        double left_dist  = std::abs(left_center  - symmetry_q0_.period / 2);
        double right_dist = std::abs(right_center - symmetry_q0_.period / 2);
        right_first = (right_dist < left_dist);
    }

    if (right_first) {
        FKState right_fk = compute_fk_incremental(fk, *robot_, right_ivs, dim);
        pre_expand_recursive(store_.right(node_idx), right_fk, right_ivs,
                             target_depth, new_node_count);
        if (deadline_reached()) return;
        FKState left_fk = compute_fk_incremental(fk, *robot_, left_ivs, dim);
        pre_expand_recursive(store_.left(node_idx), left_fk, left_ivs,
                             target_depth, new_node_count);
    } else {
        FKState left_fk = compute_fk_incremental(fk, *robot_, left_ivs, dim);
        pre_expand_recursive(store_.left(node_idx), left_fk, left_ivs,
                             target_depth, new_node_count);
        if (deadline_reached()) return;
        FKState right_fk = compute_fk_incremental(fk, *robot_, right_ivs, dim);
        pre_expand_recursive(store_.right(node_idx), right_fk, right_ivs,
                             target_depth, new_node_count);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Collision helpers
// ═══════════════════════════════════════════════════════════════════════════

static bool aabb_overlaps_obs(const float* link_aabb, const Obstacle& obs) {
    float obs_lo[3], obs_hi[3];
    for (int a = 0; a < 3; ++a) {
        obs_lo[a] = static_cast<float>(obs.center[a] - obs.half_sizes[a]);
        obs_hi[a] = static_cast<float>(obs.center[a] + obs.half_sizes[a]);
    }
    if (link_aabb[3] < obs_lo[0] || obs_hi[0] < link_aabb[0]) return false;
    if (link_aabb[4] < obs_lo[1] || obs_hi[1] < link_aabb[1]) return false;
    if (link_aabb[5] < obs_lo[2] || obs_hi[2] < link_aabb[2]) return false;
    return true;
}

bool LECT::iaabbs_collide(int node_idx,
                          const Obstacle* obstacles, int n_obs) const {
    // Direct read from pre-computed link iAABB cache (zero derivation)
    const auto& chan = channels_[static_cast<int>(active_channel())];
    if (node_idx < 0 ||
        node_idx >= static_cast<int>(chan.has_data.size()) ||
        !chan.has_data[node_idx])
        return true;  // no data → conservatively collide

    const int n_act = robot_->n_active_links();
    const float* aabb = chan.link_iaabb_cache.data() + node_idx * liaabb_stride_;
    for (int k = 0; k < n_act; ++k) {
        if (deadline_reached()) return true;
        for (int j = 0; j < n_obs; ++j) {
            if (aabb_overlaps_obs(aabb + k * 6, obstacles[j]))
                return true;
        }
    }
    return false;
}

bool LECT::hull_collides(int node_idx,
                         const Obstacle* obstacles, int n_obs) const {
    const ChannelIdx ach = active_channel();
    if (deadline_reached()) return true;
    if (!has_hull_grid(node_idx, ach)) return true;

    const auto& chan = channels_[static_cast<int>(ach)];
    if (scene_set_) {
        return chan.hull_grids[node_idx].collides(scene_grid_);
    }

    voxel::VoxelGrid obs_grid(voxel_delta_);
    for (int j = 0; j < n_obs; ++j) {
        Eigen::Vector3d lo = obstacles[j].lo();
        Eigen::Vector3d hi = obstacles[j].hi();
        float aabb[6] = {
            static_cast<float>(lo[0]), static_cast<float>(lo[1]),
            static_cast<float>(lo[2]), static_cast<float>(hi[0]),
            static_cast<float>(hi[1]), static_cast<float>(hi[2])
        };
        obs_grid.fill_aabb(aabb);
    }
    return chan.hull_grids[node_idx].collides(obs_grid);
}

bool LECT::hull_collides_grid(int node_idx,
                               const voxel::VoxelGrid& obs_grid) const {
    const ChannelIdx ach = active_channel();
    if (!has_hull_grid(node_idx, ach)) return true;
    return channels_[static_cast<int>(ach)].hull_grids[node_idx].collides(obs_grid);
}

void LECT::propagate_up(const std::vector<int>& path) {
    for (int i = static_cast<int>(path.size()) - 2; i >= 0; --i) {
        int p = path[i];
        if (!store_.is_leaf(p))
            refine_aabb(p);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Public collision queries
// ═══════════════════════════════════════════════════════════════════════════

bool LECT::collides_scene(int node_idx,
                          const Obstacle* obstacles, int n_obs) const {
    const ChannelIdx ach = active_channel();

    // Stage 1: iAABB early-out
    if (has_channel_data(node_idx, ach)) {
        if (!iaabbs_collide(node_idx, obstacles, n_obs))
            return false;
    }

    // Stage 2: Hull refinement
    bool need_hull =
        pipeline_config_.envelope.type == envelope::EnvelopeType::LinkIAABB_Grid ||
        pipeline_config_.envelope.type == envelope::EnvelopeType::Hull16_Grid;
    if (need_hull) {
        if (!has_hull_grid(node_idx, ach)) {
            auto ivs = node_intervals(node_idx);
            const_cast<LECT*>(this)->derive_hull_grid(node_idx, ivs, ach);
        }
        if (has_hull_grid(node_idx, ach)) {
            return hull_collides(node_idx, obstacles, n_obs);
        }
    }

    return true;
}

bool LECT::intervals_collide_scene(const std::vector<Interval>& intervals,
                                   const Obstacle* obstacles, int n_obs) const {
    if (deadline_reached()) return true;
    if (!robot_ || n_obs <= 0) return false;

    // Stage 1: endpoint iAABBs → per-link iAABBs
    auto ep = envelope::compute_endpoint_iaabb(
        pipeline_config_.source, *robot_, intervals);

    int n_active = robot_->n_active_links();
    std::vector<float> link_iaabbs(n_active * 6);
    envelope::extract_link_iaabbs(ep, *robot_, link_iaabbs.data());

    // iAABB early-out
    bool aabb_hit = false;
    for (int k = 0; k < n_active && !aabb_hit; ++k) {
        if (deadline_reached()) return true;
        for (int j = 0; j < n_obs; ++j) {
            if (aabb_overlaps_obs(link_iaabbs.data() + k * 6, obstacles[j])) {
                aabb_hit = true;
                break;
            }
        }
    }
    if (!aabb_hit) return false;

    // Stage 2: Hull grid refinement (grid-based envelopes only)
    bool need_hull =
        pipeline_config_.envelope.type == envelope::EnvelopeType::LinkIAABB_Grid ||
        pipeline_config_.envelope.type == envelope::EnvelopeType::Hull16_Grid;
    if (need_hull) {
        voxel::VoxelGrid tmp_grid(voxel_delta_);
        const int n_sub = pipeline_config_.envelope.n_sub;
        const double* lr = robot_->active_link_radii();

        if (n_sub > 1 && ep.has_endpoint_iaabbs()) {
            // Paired layout: use ci*2/ci*2+1 as frame indices
            float base_pos[3] = {0.f, 0.f, 0.f};
            std::vector<float> sub_aabbs(n_active * n_sub * 6);
            for (int ci = 0; ci < n_active; ++ci) {
                int parent_fi = ci * 2;       // proximal
                int link_fi   = ci * 2 + 1;   // distal
                float r = lr ? static_cast<float>(lr[ci]) : 0.f;
                envelope::derive_aabb_subdivided(
                    ep.endpoint_iaabbs.data(), ep.n_active_ep,
                    parent_fi, link_fi,
                    n_sub, r, base_pos,
                    sub_aabbs.data() + ci * n_sub * 6);
            }
            int total = n_active * n_sub;
            for (int k = 0; k < total; ++k)
                tmp_grid.fill_aabb(sub_aabbs.data() + k * 6);
        } else {
            for (int k = 0; k < n_active; ++k)
                tmp_grid.fill_aabb(link_iaabbs.data() + k * 6);
        }

        if (scene_set_) {
            return tmp_grid.collides(scene_grid_);
        } else {
            voxel::VoxelGrid obs_grid(voxel_delta_);
            for (int j = 0; j < n_obs; ++j) {
                if (deadline_reached()) return true;
                Eigen::Vector3d lo = obstacles[j].lo();
                Eigen::Vector3d hi = obstacles[j].hi();
                float aabb[6] = {
                    static_cast<float>(lo[0]), static_cast<float>(lo[1]),
                    static_cast<float>(lo[2]), static_cast<float>(hi[0]),
                    static_cast<float>(hi[1]), static_cast<float>(hi[2])
                };
                obs_grid.fill_aabb(aabb);
            }
            return tmp_grid.collides(obs_grid);
        }
    }

    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
//  transplant_subtree — merge worker subtree into coordinator
// ═══════════════════════════════════════════════════════════════════════════

int LECT::transplant_subtree(const LECT& worker,
                             int coordinator_node,
                             int snapshot_n_nodes,
                             const std::unordered_map<int, int>& box_id_remap) {
    if (worker.is_leaf(coordinator_node))
        return 0;

    std::unordered_map<int, int> remap;
    remap[coordinator_node] = coordinator_node;

    std::vector<int> bfs_queue;
    bfs_queue.push_back(coordinator_node);
    int transplanted = 0;
    std::vector<int> new_worker_nodes;

    size_t qi = 0;
    while (qi < bfs_queue.size()) {
        int w_node = bfs_queue[qi++];

        if (worker.store_.is_leaf(w_node))
            continue;

        int w_left  = worker.store_.left(w_node);
        int w_right = worker.store_.right(w_node);

        if (w_left >= 0) {
            if (w_left >= snapshot_n_nodes) {
                int c_idx = store_.alloc_node();
                remap[w_left] = c_idx;
                new_worker_nodes.push_back(w_left);
                ++transplanted;
            } else {
                remap[w_left] = w_left;
            }
            bfs_queue.push_back(w_left);
        }

        if (w_right >= 0) {
            if (w_right >= snapshot_n_nodes) {
                int c_idx = store_.alloc_node();
                remap[w_right] = c_idx;
                new_worker_nodes.push_back(w_right);
                ++transplanted;
            } else {
                remap[w_right] = w_right;
            }
            bfs_queue.push_back(w_right);
        }
    }

    if (transplanted == 0)
        return 0;

    int needed = store_.n_nodes();
    for (int c = 0; c < NUM_CHANNELS; ++c)
        ensure_channel_capacity(needed, c);
    if (needed > static_cast<int>(node_split_dim_.size()))
        node_split_dim_.resize(needed, -1);

    for (int w_idx : new_worker_nodes) {
        int c_idx = remap[w_idx];

        store_.copy_node_from(worker.store_, w_idx, c_idx);

        int w_left   = worker.store_.left(w_idx);
        int w_right  = worker.store_.right(w_idx);
        int w_parent = worker.store_.parent(w_idx);

        if (w_left >= 0) {
            auto it = remap.find(w_left);
            store_.set_left(c_idx, (it != remap.end()) ? it->second : w_left);
        }
        if (w_right >= 0) {
            auto it = remap.find(w_right);
            store_.set_right(c_idx, (it != remap.end()) ? it->second : w_right);
        }
        if (w_parent >= 0) {
            auto it = remap.find(w_parent);
            store_.set_parent(c_idx, (it != remap.end()) ? it->second : w_parent);
        }

        int fid = worker.store_.forest_id(w_idx);
        if (fid >= 0) {
            auto it = box_id_remap.find(fid);
            if (it != box_id_remap.end()) {
                store_.set_occupied(c_idx, it->second);
            }
        }

        // Copy dual-channel data from worker
        for (int c = 0; c < NUM_CHANNELS; ++c) {
            const auto& w_chan = worker.channels_[c];
            auto& my_chan = channels_[c];
            ensure_channel_capacity(c_idx + 1, c);

            if (w_idx < static_cast<int>(w_chan.has_data.size()) &&
                w_chan.has_data[w_idx]) {
                std::memcpy(
                    my_chan.ep_data.data() + c_idx * ep_stride_,
                    w_chan.ep_data.data() + w_idx * worker.ep_stride_,
                    ep_stride_ * sizeof(float));
                std::memcpy(
                    my_chan.link_iaabb_cache.data() + c_idx * liaabb_stride_,
                    w_chan.link_iaabb_cache.data() + w_idx * worker.liaabb_stride_,
                    liaabb_stride_ * sizeof(float));
                my_chan.has_data[c_idx] = w_chan.has_data[w_idx];
                my_chan.source_quality[c_idx] = w_chan.source_quality[w_idx];
            }

            if (w_idx < static_cast<int>(w_chan.hull_valid.size()) &&
                w_chan.hull_valid[w_idx]) {
                my_chan.hull_grids[c_idx] = w_chan.hull_grids[w_idx];
                my_chan.hull_valid[c_idx] = 1;
            }
        }

        // Copy node_split_dim
        if (w_idx < static_cast<int>(worker.node_split_dim_.size()) &&
            worker.node_split_dim_[w_idx] >= 0) {
            if (c_idx >= static_cast<int>(node_split_dim_.size()))
                node_split_dim_.resize(c_idx + 1, -1);
            node_split_dim_[c_idx] = worker.node_split_dim_[w_idx];
        }
    }

    // Update coordinator_node: copy left/right + split from worker
    {
        int w_left  = worker.store_.left(coordinator_node);
        int w_right = worker.store_.right(coordinator_node);
        if (w_left >= 0) {
            auto it = remap.find(w_left);
            store_.set_left(coordinator_node,
                           (it != remap.end()) ? it->second : w_left);
        }
        if (w_right >= 0) {
            auto it = remap.find(w_right);
            store_.set_right(coordinator_node,
                            (it != remap.end()) ? it->second : w_right);
        }
        store_.set_split(coordinator_node,
                         worker.store_.split(coordinator_node));

        // Copy split dim
        if (coordinator_node < static_cast<int>(worker.node_split_dim_.size()) &&
            worker.node_split_dim_[coordinator_node] >= 0) {
            if (coordinator_node >= static_cast<int>(node_split_dim_.size()))
                node_split_dim_.resize(coordinator_node + 1, -1);
            node_split_dim_[coordinator_node] =
                worker.node_split_dim_[coordinator_node];
        }
    }

    return transplanted;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Hull-based Greedy Coarsening
// ═══════════════════════════════════════════════════════════════════════════

double LECT::merged_children_hull_volume(int node_idx) const {
    const ChannelIdx ach = active_channel();
    const int ci = static_cast<int>(ach);
    int li = store_.left(node_idx);
    int ri = store_.right(node_idx);
    if (li < 0 || ri < 0) return -1.0;
    if (!has_hull_grid(li, ach) || !has_hull_grid(ri, ach)) return -1.0;

    voxel::VoxelGrid merged = channels_[ci].hull_grids[li];
    merged.merge(channels_[ci].hull_grids[ri]);
    return merged.occupied_volume();
}

double LECT::coarsen_volume_ratio(int node_idx) const {
    const ChannelIdx ach = active_channel();
    const int ci = static_cast<int>(ach);
    int li = store_.left(node_idx);
    int ri = store_.right(node_idx);
    if (li < 0 || ri < 0) return -1.0;
    if (!has_hull_grid(li, ach) || !has_hull_grid(ri, ach)) return -1.0;

    double lv = channels_[ci].hull_grids[li].occupied_volume();
    double rv = channels_[ci].hull_grids[ri].occupied_volume();
    double max_child = std::max(lv, rv);
    if (max_child < 1e-12) return -1.0;

    double merged_vol = merged_children_hull_volume(node_idx);
    if (merged_vol < 0) return -1.0;

    return merged_vol / max_child;
}

bool LECT::merge_children_hulls(int node_idx) {
    const ChannelIdx ach = active_channel();
    const int ci = static_cast<int>(ach);
    int li = store_.left(node_idx);
    int ri = store_.right(node_idx);
    if (li < 0 || ri < 0) return false;

    if (!has_hull_grid(li, ach)) {
        auto ivs = node_intervals(li);
        derive_hull_grid(li, ivs, ach);
    }
    if (!has_hull_grid(ri, ach)) {
        auto ivs = node_intervals(ri);
        derive_hull_grid(ri, ivs, ach);
    }
    if (!has_hull_grid(li, ach) || !has_hull_grid(ri, ach)) return false;

    ensure_channel_capacity(node_idx + 1);
    auto& chan = channels_[ci];
    chan.hull_grids[node_idx] = chan.hull_grids[li];
    chan.hull_grids[node_idx].merge(chan.hull_grids[ri]);
    chan.hull_valid[node_idx] = 1;
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Statistics
// ═══════════════════════════════════════════════════════════════════════════

int LECT::count_nodes_with_iaabb() const {
    const ChannelIdx ach = active_channel();
    int count = 0;
    for (int i = 0; i < store_.n_nodes(); ++i)
        if (has_channel_data(i, ach)) ++count;
    return count;
}

int LECT::count_nodes_with_hull() const {
    const ChannelIdx ach = active_channel();
    const auto& chan = channels_[static_cast<int>(ach)];
    int count = 0;
    int n = std::min(store_.n_nodes(),
                     static_cast<int>(chan.hull_valid.size()));
    for (int i = 0; i < n; ++i)
        if (chan.hull_valid[i]) ++count;
    return count;
}

int LECT::total_hull_voxels() const {
    const ChannelIdx ach = active_channel();
    const auto& chan = channels_[static_cast<int>(ach)];
    int total = 0;
    int n = std::min(store_.n_nodes(),
                     static_cast<int>(chan.hull_valid.size()));
    for (int i = 0; i < n; ++i)
        if (chan.hull_valid[i])
            total += chan.hull_grids[i].count_occupied();
    return total;
}

int LECT::scene_grid_voxels() const {
    if (!scene_set_) return 0;
    return scene_grid_.count_occupied();
}

// ═══════════════════════════════════════════════════════════════════════════
//  Hull persistence — HUL2 binary format (dual-channel)
//  Backward-compatible reading of HUL1 (single-channel → SAFE)
// ═══════════════════════════════════════════════════════════════════════════

// Helper: write one channel's hull grids for n_alloc nodes
static void write_channel_hulls(std::ostream& f,
                                const LECT::ChannelData& chan,
                                int n_alloc) {
    int n_valid = 0;
    for (int i = 0; i < n_alloc; ++i)
        if (i < static_cast<int>(chan.hull_valid.size()) && chan.hull_valid[i])
            ++n_valid;

    int32_t nv32 = n_valid;
    f.write(reinterpret_cast<const char*>(&nv32), sizeof(nv32));

    for (int i = 0; i < n_alloc; ++i) {
        uint8_t valid = (i < static_cast<int>(chan.hull_valid.size()) &&
                         chan.hull_valid[i]) ? 1 : 0;
        f.write(reinterpret_cast<const char*>(&valid), 1);
        if (!valid) continue;

        const auto& bricks = chan.hull_grids[i].bricks();
        int32_t n_bricks = static_cast<int32_t>(bricks.size());
        f.write(reinterpret_cast<const char*>(&n_bricks), sizeof(n_bricks));

        for (const auto& [coord, brick] : bricks) {
            f.write(reinterpret_cast<const char*>(&coord.bx), sizeof(int));
            f.write(reinterpret_cast<const char*>(&coord.by), sizeof(int));
            f.write(reinterpret_cast<const char*>(&coord.bz), sizeof(int));
            f.write(reinterpret_cast<const char*>(brick.words),
                    sizeof(brick.words));
        }
    }
}

// Helper: read one channel's hull grids for n_alloc nodes
static void read_channel_hulls(std::istream& f,
                               LECT::ChannelData& chan,
                               int64_t n_alloc, double delta) {
    chan.hull_grids.resize(n_alloc);
    chan.hull_valid.resize(n_alloc, 0);

    for (int64_t i = 0; i < n_alloc; ++i) {
        uint8_t valid;
        f.read(reinterpret_cast<char*>(&valid), 1);
        chan.hull_valid[i] = valid;
        if (!valid) continue;

        int32_t n_bricks;
        f.read(reinterpret_cast<char*>(&n_bricks), sizeof(n_bricks));

        chan.hull_grids[i] = voxel::VoxelGrid(delta);

        for (int32_t j = 0; j < n_bricks; ++j) {
            voxel::BrickCoord coord;
            voxel::BitBrick brick;
            f.read(reinterpret_cast<char*>(&coord.bx), sizeof(int));
            f.read(reinterpret_cast<char*>(&coord.by), sizeof(int));
            f.read(reinterpret_cast<char*>(&coord.bz), sizeof(int));
            f.read(reinterpret_cast<char*>(brick.words), sizeof(brick.words));
            chan.hull_grids[i].set_brick(coord, brick);
        }
    }
}

void LECT::save_hull_grids(const std::string& path) const {
    std::ofstream f(path, std::ios::binary);
    if (!f)
        throw std::runtime_error("LECT::save_hull_grids: cannot open " + path);

    // Determine n_alloc as max across both channels
    int n_alloc = 0;
    for (int c = 0; c < NUM_CHANNELS; ++c) {
        n_alloc = std::max(n_alloc,
                           static_cast<int>(channels_[c].hull_grids.size()));
    }

    // HUL2 header
    char header[HULL_HEADER_SIZE] = {};
    std::memcpy(header + 0, &HULL_MAGIC_V2, 4);
    *reinterpret_cast<uint32_t*>(header + 4)  = 2;  // version 2
    *reinterpret_cast<int64_t*>(header + 8)   = 0;  // reserved (was n_valid)
    *reinterpret_cast<int64_t*>(header + 16)  = n_alloc;
    *reinterpret_cast<double*>(header + 24)   = voxel_delta_;
    *reinterpret_cast<int32_t*>(header + 32)  = NUM_CHANNELS;
    f.write(header, HULL_HEADER_SIZE);

    // Write each channel
    for (int c = 0; c < NUM_CHANNELS; ++c) {
        write_channel_hulls(f, channels_[c], n_alloc);
    }
}

void LECT::load_hull_grids(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f)
        throw std::runtime_error("LECT::load_hull_grids: cannot open " + path);

    char header[HULL_HEADER_SIZE];
    f.read(header, HULL_HEADER_SIZE);
    if (!f)
        throw std::runtime_error("LECT::load_hull_grids: truncated header");

    uint32_t magic = *reinterpret_cast<uint32_t*>(header);
    int64_t n_alloc = *reinterpret_cast<int64_t*>(header + 16);
    double delta    = *reinterpret_cast<double*>(header + 24);
    voxel_delta_    = delta;

    if (magic == HULL_MAGIC_V2) {
        // HUL2: dual-channel
        int32_t n_ch = *reinterpret_cast<int32_t*>(header + 32);
        for (int c = 0; c < std::min(n_ch, static_cast<int32_t>(NUM_CHANNELS)); ++c) {
            read_channel_hulls(f, channels_[c], n_alloc, delta);
        }
        // Skip extra channels if file has more than NUM_CHANNELS
        for (int c = NUM_CHANNELS; c < n_ch; ++c) {
            ChannelData dummy;
            read_channel_hulls(f, dummy, n_alloc, delta);
        }
    } else if (magic == HULL_MAGIC_V1) {
        // HUL1 legacy: single-channel → load into SAFE channel
        read_channel_hulls(f, channels_[static_cast<int>(ChannelIdx::SAFE)],
                           n_alloc, delta);
    } else {
        throw std::runtime_error("LECT::load_hull_grids: invalid hull magic");
    }
}

} // namespace forest
} // namespace sbf

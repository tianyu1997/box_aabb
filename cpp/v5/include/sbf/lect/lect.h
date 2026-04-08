#pragma once
/// @file lect.h
/// @brief LECT: Link Envelope Collision Tree — the central spatial data structure.
///
/// The LECT is a binary KD-tree over configuration space where each leaf
/// node stores pre-computed link envelope data.  It supports:
///
///   - **Dual-channel architecture**:
///     - CH_SAFE (0) — IFK-derived envelopes (tight, provably conservative)
///     - CH_UNSAFE (1) — CritSample / Analytical / GCPC (larger coverage)
///
///   - **Per-node grid storage**: optional `SparseVoxelGrid` slots tagged with
///     (EnvelopeType, delta, channel) for fine-grained collision checking.
///
///   - **Z4 symmetry cache**: for robots with Z4 rotational symmetry on
///     joint 0, envelopes are computed once per canonical sector and
///     transformed to other sectors (up to 4× speedup).
///
///   - **Parallel snapshots**: `snapshot()` creates a deep copy for
///     independent parallel workers; `transplant_subtree()` merges results.
///
/// Typical lifecycle:
///   1. Construct with Robot + root intervals + endpoint/envelope configs.
///   2. FFB calls `expand_leaf()` and `compute_envelope()` during search.
///   3. ForestGrower marks/unmarks nodes as occupied by boxes.
///   4. Persist to disk via `lect_save_binary()` / `lect_load_binary()`.

#include <sbf/core/types.h>
#include <sbf/core/robot.h>
#include <sbf/core/fk_state.h>
#include <sbf/core/joint_symmetry.h>
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/envelope_type.h>
#include <sbf/envelope/link_iaabb.h>
#include <sbf/voxel/voxel_grid.h>

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace sbf {

// ─── Channel constants ──────────────────────────────────────────────────────
static constexpr int CH_SAFE   = 0;   ///< Conservative (IFK) channel.
static constexpr int CH_UNSAFE = 1;   ///< Coverage (CritSample/Analytical/GCPC) channel.
static constexpr int N_CHANNELS = 2;  ///< Total number of envelope channels.

// ─── Split order strategy ───────────────────────────────────────────────────
/// @brief Split dimension selection strategy for LECT expansion.
enum class SplitOrder : uint8_t {
    ROUND_ROBIN   = 0,   ///< Cycle through dimensions in order.
    WIDEST_FIRST  = 1,   ///< Split the widest interval first.
    BEST_TIGHTEN  = 2    ///< Split the dimension that minimises child envelope volume (default).
};

// ─── Per-channel flat-buffer storage ────────────────────────────────────────
/// @brief Per-channel flat-buffer storage for endpoint iAABB data.
///
/// Each channel (CH_SAFE, CH_UNSAFE) stores endpoint iAABB data in a
/// flat array indexed by node.  `has_data[i]` indicates whether node i
/// has valid data, and `source_quality[i]` records the EndpointSource
/// that produced it.
struct ChannelData {
    std::vector<float>   ep_data;         ///< [capacity × ep_stride] endpoint iAABBs per node.
    std::vector<uint8_t> source_quality;  ///< [capacity] EndpointSource that produced data.
    std::vector<uint8_t> has_data;        ///< [capacity] 0/1 whether node has valid data.

    void resize(int cap, int ep_stride) {
        ep_data.resize(static_cast<size_t>(cap) * ep_stride, 0.0f);
        source_quality.resize(cap, 0);
        has_data.resize(cap, 0);
    }

    void clear_node(int i, int ep_stride) {
        has_data[i] = 0;
        source_quality[i] = 0;
        std::memset(ep_data.data() + static_cast<size_t>(i) * ep_stride,
                    0, static_cast<size_t>(ep_stride) * sizeof(float));
    }
};

// ─── Per-node grid slot metadata ────────────────────────────────────────────
/// @brief Metadata for a per-node voxel grid slot.
struct GridSlot {
    EnvelopeType type  = EnvelopeType::LinkIAABB_Grid;  ///< Envelope type that produced this grid.
    float        delta = 0.02f;   ///< Voxel edge length (m).
    uint8_t      channel = 0;     ///< CH_SAFE or CH_UNSAFE.
};

// ─── LECT: KD-tree with per-node link envelope cache ────────────────────────

/// @brief LECT: Link Envelope Collision Tree.
///
/// Binary KD-tree over configuration space with per-node link envelope
/// caching.  This is the core spatial index of SafeBoxForest.  The FFB
/// algorithm traverses the LECT to find collision-free leaf nodes;
/// the ForestGrower manages node occupation to track which leaves are
/// already claimed by boxes.
class LECT {
public:
    LECT() = default;

    /// Construct a new LECT rooted at @p root_intervals.
    /// @param robot         Robot kinematic model.
    /// @param root_intervals  Per-joint intervals defining the root node.
    /// @param ep_config     Endpoint source configuration (IFK, Analytical, etc.).
    /// @param env_config    Envelope type configuration (LinkIAABB, Grid, etc.).
    /// @param initial_cap   Initial node capacity (power of 2 minus 1).
    LECT(const Robot& robot,
         const std::vector<Interval>& root_intervals,
         const EndpointSourceConfig& ep_config,
         const EnvelopeTypeConfig& env_config,
         int initial_cap = 1023);

    /// Move constructor (for parallel worker results).
    LECT(LECT&&) = default;
    LECT& operator=(LECT&&) = default;

    /// Deep-copy snapshot for parallel workers.
    /// The returned LECT is fully independent (no shared state).
    LECT snapshot() const;

    // --- Metadata ---
    int n_nodes()        const { return n_nodes_; }
    int n_dims()         const { return n_dims_; }
    int n_active_links() const { return n_active_links_; }
    int capacity()       const { return capacity_; }

    const Robot& robot() const { return robot_; }
    const EndpointSourceConfig& ep_config()  const { return ep_config_; }
    const EnvelopeTypeConfig&   env_config() const { return env_config_; }
    void set_ep_config(const EndpointSourceConfig& cfg) { ep_config_ = cfg; }

    // --- Tree structure access ---
    int    left(int i)          const { return left_[i]; }
    int    right(int i)         const { return right_[i]; }
    int    parent(int i)        const { return parent_[i]; }
    int    depth(int i)         const { return depth_[i]; }
    bool   is_leaf(int i)       const { return left_[i] < 0 && right_[i] < 0; }
    int    get_split_dim(int i) const { return split_dim_[i]; }
    double split_val(int i)     const { return split_val_[i]; }

    std::vector<Interval> node_intervals(int node_idx) const;

    // --- Envelope data access (dual-channel) ---
    bool has_data(int i) const {
        return channels_[CH_SAFE].has_data[i] != 0 ||
               channels_[CH_UNSAFE].has_data[i] != 0;
    }
    bool has_safe_data(int i)   const { return channels_[CH_SAFE].has_data[i] != 0; }
    bool has_unsafe_data(int i) const { return channels_[CH_UNSAFE].has_data[i] != 0; }

    const float* get_endpoint_iaabbs(int i, int channel = CH_SAFE) const {
        return channels_[channel].ep_data.data() + i * ep_stride_;
    }

    const float* get_link_iaabbs(int i) const {
        if (link_iaabb_dirty_[i]) {
            const_cast<LECT*>(this)->materialise_link_iaabb(i);
        }
        return link_iaabb_cache_.data() + i * liaabb_stride_;
    }

    EndpointSource source_quality(int i, int channel = CH_SAFE) const {
        return static_cast<EndpointSource>(channels_[channel].source_quality[i]);
    }

    // --- Per-node grid access ---
    int num_grid_slots(int i) const {
        return (i < static_cast<int>(node_grids_.size()))
               ? static_cast<int>(node_grids_[i].size()) : 0;
    }
    const std::vector<voxel::SparseVoxelGrid>& node_grids(int i) const {
        return node_grids_[i];
    }
    const std::vector<GridSlot>& node_grid_meta(int i) const {
        return node_grid_meta_[i];
    }

    // --- Root accessors (for FK reuse in FFB traversal) ---
    const FKState& root_fk() const { return root_fk_; }
    const std::vector<Interval>& root_intervals() const { return root_intervals_; }

    // --- Core operations ---
    int expand_leaf(int node_idx);
    int expand_leaf(int node_idx, const FKState& fk,
                    const std::vector<Interval>& intervals);

    void compute_envelope(int node_idx, const FKState& fk,
                          const std::vector<Interval>& intervals,
                          int changed_dim = -1,
                          int parent_idx = -1);

    // --- Collision queries ---
    bool collides_scene(int node_idx,
                        const Obstacle* obs, int n_obs) const;

    /// Two-layer collision: IAABB coarse → Grid fine.
    /// obs_grids: pre-rasterized obstacle grids keyed by delta.
    bool collides_scene(int node_idx,
                        const Obstacle* obs, int n_obs,
                        const std::unordered_map<float, voxel::SparseVoxelGrid>& obs_grids) const;

    bool intervals_collide_scene(const std::vector<Interval>& intervals,
                                 const Obstacle* obs, int n_obs) const;

    // --- Occupation management ---
    void mark_occupied(int node_idx, int box_id);
    void unmark_occupied(int node_idx);
    bool is_occupied(int node_idx)  const { return forest_id_[node_idx] >= 0; }
    int  forest_id(int node_idx)    const { return forest_id_[node_idx]; }
    int  subtree_occ(int node_idx)  const { return subtree_occ_[node_idx]; }
    void clear_all_occupation();

    /// Clear forest planning state (forest_id, subtree_occ) while
    /// preserving tree structure and all envelope data.
    /// Used after loading a cached LECT for a new planning session.
    void clear_forest_state();

    /// Transplant expanded nodes from a worker LECT snapshot back into
    /// this (master) LECT.  Only copies nodes that were expanded by the
    /// worker (i.e. node indices >= snapshot_base).  Box IDs are remapped
    /// via id_map.  Returns the number of nodes transplanted.
    int transplant_subtree(const LECT& worker, int snapshot_base,
                           const std::unordered_map<int, int>& id_map);

    // --- Split strategy ---
    void set_split_order(SplitOrder so) { split_order_ = so; }
    SplitOrder split_order() const { return split_order_; }

    // --- Z4 symmetry ---
    const JointSymmetry& symmetry_q0() const { return symmetry_q0_; }
    bool z4_active() const { return z4_active_; }
    void disable_z4() { z4_active_ = false; z4_cache_.clear(); }

    // --- Obstacle-free tree expansion ---
    /// Expand the tree along random C-space paths to the given depth.
    /// Each path descends from root, expanding leaves as needed.
    /// Returns the number of newly created nodes.
    int warmup(int max_depth, int n_paths, int seed = 42);

private:
    // ── Dual-channel flat buffer storage ─────────────────────────────────
    ChannelData channels_[N_CHANNELS];
    std::vector<float>   link_iaabb_cache_;   // derived (merged tightest)
    std::vector<uint8_t> link_iaabb_dirty_;   // 1 = needs recompute

    // ── Per-node grid storage ───────────────────────────────────────────
    std::vector<std::vector<voxel::SparseVoxelGrid>> node_grids_;
    std::vector<std::vector<GridSlot>>               node_grid_meta_;

    // ── Tree structure ──────────────────────────────────────────────────
    std::vector<int>    left_, right_, parent_, depth_;
    std::vector<int>    split_dim_;
    std::vector<double> split_val_;

    // ── Occupation ──────────────────────────────────────────────────────
    std::vector<int> forest_id_;    std::vector<int> subtree_occ_;
    // ── Config ──────────────────────────────────────────────────────────
    Robot robot_;
    EndpointSourceConfig ep_config_;
    EnvelopeTypeConfig   env_config_;
    std::vector<Interval> root_intervals_;

    int ep_stride_     = 0;    // n_active_links * 2 * 6
    int liaabb_stride_  = 0;   // n_active_links * 6
    int n_dims_        = 0;
    int n_active_links_ = 0;
    int n_nodes_       = 0;
    int capacity_      = 0;

    SplitOrder split_order_ = SplitOrder::BEST_TIGHTEN;

    // ── Z4 cache ────────────────────────────────────────────────────────
    struct Z4CacheEntry {
        std::vector<float> ep_iaabbs;
        EndpointSource source = EndpointSource::IFK;
        int channel = CH_SAFE;
    };
    std::unordered_map<uint64_t, Z4CacheEntry> z4_cache_;
    JointSymmetry symmetry_q0_;
    bool z4_active_ = false;

    // ── Cached link radii (float) ───────────────────────────────────────
    std::vector<float> radii_f_;

    // ── FK state cache for root ─────────────────────────────────────────
    FKState root_fk_;

    // ── Depth → split-dim cache (BEST_TIGHTEN: compute once per depth) ──
    std::unordered_map<int, int> depth_split_dim_cache_;

public:
    // ── Expand profiling counters ────────────────────────────────────────
    struct ExpandProfile {
        double pick_dim_ms  = 0;
        double fk_inc_ms    = 0;
        double envelope_ms  = 0;
        double refine_ms    = 0;
        int    pick_dim_calls = 0;
        int    pick_dim_cache_hits = 0;
        int    expand_calls = 0;
        void reset() { *this = {}; }
        void merge(const ExpandProfile& o) {
            pick_dim_ms += o.pick_dim_ms;
            fk_inc_ms   += o.fk_inc_ms;
            envelope_ms += o.envelope_ms;
            refine_ms   += o.refine_ms;
            pick_dim_calls += o.pick_dim_calls;
            pick_dim_cache_hits += o.pick_dim_cache_hits;
            expand_calls += o.expand_calls;
        }
    };
    ExpandProfile expand_profile_;
private:

    // ── IO friends ──────────────────────────────────────────────────────
    friend bool lect_save_binary(const LECT& lect, const std::string& path);
    friend bool lect_save_incremental(const LECT& lect, const std::string& path,
                                      int old_n_nodes);
    friend bool lect_load_binary(LECT& lect, const Robot& robot, const std::string& path);
    friend struct LectIOHelper;

    // ── Private helpers ─────────────────────────────────────────────────
    void ensure_capacity(int min_cap);
    int  alloc_node();
    void materialise_link_iaabb(int i);

    int  pick_split_dim(int depth_val, const FKState& fk,
                        const std::vector<Interval>& intervals) const;
    int  pick_best_tighten_dim(const FKState& fk,
                               const std::vector<Interval>& intervals) const;

    void split_leaf_impl(int node_idx, const FKState& parent_fk,
                         const std::vector<Interval>& parent_intervals);

    void refine_parent_aabb(int parent_idx);
    void derive_merged_link_iaabb(int node_idx);
    void compute_node_grids(int node_idx, int channel);

    static int  z4_canonicalize_interval(double q0_lo, double q0_hi,
                                         double period,
                                         double& can_lo, double& can_hi);
    static uint64_t z4_interval_hash(double can_lo, double can_hi,
                                     const std::vector<Interval>& intervals);
};

}  // namespace sbf

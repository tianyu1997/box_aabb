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
#include <sbf/lect/lect_mmap.h>
#include <sbf/lect/lect_cache_manager.h>
#include <sbf/voxel/voxel_grid.h>

#include <cstdint>
#include <cstring>
#include <unordered_map>
#include <vector>

namespace sbf {

// ─── TreeArray: mmap-or-vector transparent storage ──────────────────────────
/// @brief Thin wrapper providing operator[] access to either an mmap-backed
///        region (MAP_PRIVATE COW) or a std::vector.  Used for LECT tree
///        structure arrays so that parallel worker snapshots can share the
///        master's mmap without deep-copying 6 large vectors (~20 MB).
///
/// In mmap mode (set_mmap called), all operator[] accesses go through the
/// mmap pointer.  Writes trigger page-level COW in MAP_PRIVATE mappings.
/// In vector mode (default), operator[] delegates to std::vector.
template <typename T>
class TreeArray {
public:
    TreeArray() = default;
    TreeArray(TreeArray&& o) noexcept = default;
    TreeArray& operator=(TreeArray&& o) noexcept = default;

    /// Copy constructor: copies vector data only, never copies mmap pointer.
    TreeArray(const TreeArray& o) : vec_(o.vec_), mmap_ptr_(nullptr) {}
    /// Copy assignment: copies vector data, clears mmap pointer.
    TreeArray& operator=(const TreeArray& o) {
        if (this != &o) { vec_ = o.vec_; mmap_ptr_ = nullptr; }
        return *this;
    }

    // ── Element access ──────────────────────────────────────────────────
    const T& operator[](int i) const { return mmap_ptr_ ? mmap_ptr_[i] : vec_[i]; }
    T&       operator[](int i)       { return mmap_ptr_ ? mmap_ptr_[i] : vec_[i]; }

    // ── Raw data pointer (for memcpy / save) ────────────────────────────
    const T* data() const { return mmap_ptr_ ? mmap_ptr_ : vec_.data(); }
    T*       data()       { return mmap_ptr_ ? mmap_ptr_ : vec_.data(); }

    // ── Mode management ─────────────────────────────────────────────────
    /// Switch to mmap-backed mode.  Frees any existing vector storage.
    void set_mmap(T* ptr) { mmap_ptr_ = ptr; vec_.clear(); vec_.shrink_to_fit(); }
    bool is_mmap() const  { return mmap_ptr_ != nullptr; }

    /// Materialize: copy @p n elements from mmap into vector, switch to vec mode.
    void materialize(int n) {
        if (!mmap_ptr_) return;
        vec_.resize(static_cast<size_t>(n));
        std::memcpy(vec_.data(), mmap_ptr_, static_cast<size_t>(n) * sizeof(T));
        mmap_ptr_ = nullptr;
    }

    // ── Vector-mode operations ──────────────────────────────────────────
    void resize(int n, T val = T{}) {
        if (!mmap_ptr_) vec_.resize(static_cast<size_t>(n), val);
    }
    void assign(int n, T val) {
        if (mmap_ptr_) {
            for (int i = 0; i < n; ++i) mmap_ptr_[i] = val;
        } else {
            vec_.assign(static_cast<size_t>(n), val);
        }
    }
    void clear() { mmap_ptr_ = nullptr; vec_.clear(); }

private:
    T* mmap_ptr_ = nullptr;
    std::vector<T> vec_;
};

// ─── Channel constants ──────────────────────────────────────────────────────
static constexpr int CH_SAFE   = 0;   ///< Conservative (IFK) channel.
static constexpr int CH_UNSAFE = 1;   ///< Coverage (CritSample/Analytical/GCPC) channel.
static constexpr int N_CHANNELS = 2;  ///< Total number of envelope channels.

// ─── Split order strategy ───────────────────────────────────────────────────
/// @brief Split dimension selection strategy for LECT expansion.
enum class SplitOrder : uint8_t {
    ROUND_ROBIN      = 0,   ///< Cycle through dimensions in order.
    WIDEST_FIRST     = 1,   ///< Split the widest interval first.
    BEST_TIGHTEN     = 2,   ///< Split the dimension that minimises child envelope volume (default).
    BEST_TIGHTEN_V2  = 3    ///< Multi-probe BT: sample K virtual nodes per depth for robust dim selection.
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
    void set_env_config(const EnvelopeTypeConfig& cfg) { env_config_ = cfg; }

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
        return ep_data_read(i, channel);
    }

    /// Read-only access to ep_data for node @p i on @p ch.
    /// Returns a pointer into the mmap region for loaded nodes,
    /// or into the heap vector (with offset) for newly expanded nodes.
    const float* ep_data_read(int i, int ch = CH_SAFE) const {
        if (use_mmap_ && i < nn_loaded_) {
            // V5 SoA: EP section has per-node [safe|unsafe], no tree header
            const uint8_t* rec = mmap_ep_base_
                + static_cast<size_t>(i) * mmap_node_stride_;
            if (ch == CH_UNSAFE)
                rec += static_cast<size_t>(ep_stride_) * sizeof(float);
            return reinterpret_cast<const float*>(rec);
        }
        if (use_mmap_) {
            // New node in offset vector
            return channels_[ch].ep_data.data()
                + static_cast<size_t>(i - nn_loaded_) * ep_stride_;
        }
        return channels_[ch].ep_data.data()
            + static_cast<size_t>(i) * ep_stride_;
    }

    /// Writable access to ep_data for node @p i on @p ch.
    /// For loaded nodes, returns a writable mmap pointer (COW via MAP_PRIVATE).
    /// For new nodes, returns offset vector pointer.
    float* ep_data_write(int i, int ch) {
        if (use_mmap_ && i < nn_loaded_) {
            uint8_t* rec = mmap_ep_base_
                + static_cast<size_t>(i) * mmap_node_stride_;
            if (ch == CH_UNSAFE)
                rec += static_cast<size_t>(ep_stride_) * sizeof(float);
            return reinterpret_cast<float*>(rec);
        }
        if (use_mmap_) {
            return channels_[ch].ep_data.data()
                + static_cast<size_t>(i - nn_loaded_) * ep_stride_;
        }
        return channels_[ch].ep_data.data()
            + static_cast<size_t>(i) * ep_stride_;
    }

    const float* get_link_iaabbs(int i) const {
        // Lazy-allocate link_iaabb_cache_ on first use (avoids 110MB alloc at load)
        if (link_iaabb_cache_.empty()) {
            auto& cache = const_cast<std::vector<float>&>(link_iaabb_cache_);
            cache.resize(static_cast<size_t>(capacity_) * liaabb_stride_, 0.0f);
        }
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

    /// Fast check: does the point @p q fall inside any occupied LECT leaf?
    /// Walks from root to the containing leaf in O(depth) time.
    bool is_point_occupied(const Eigen::VectorXd& q) const {
        int node = 0;
        while (true) {
            if (is_occupied(node)) return true;
            if (is_leaf(node)) return false;
            int sd = split_dim_[node];
            node = (q[sd] <= split_val_[node]) ? left_[node] : right_[node];
        }
    }

    /// Mark the LECT leaf containing point @p q as occupied (box_id=0).
    /// Walks from root to leaf in O(depth), skips if any ancestor already occupied.
    void mark_point_occupied(const Eigen::VectorXd& q) {
        int node = 0;
        while (true) {
            if (is_occupied(node)) return;  // already occupied
            if (is_leaf(node)) { mark_occupied(node, 0); return; }
            int sd = split_dim_[node];
            node = (q[sd] <= split_val_[node]) ? left_[node] : right_[node];
        }
    }

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
    void set_n_bt_probes(int k) { n_bt_probes_ = std::max(1, k); }
    int  n_bt_probes() const { return n_bt_probes_; }
    void set_bt_width_power(double p) { bt_width_power_ = p; }
    double bt_width_power() const { return bt_width_power_; }

    // --- Z4 symmetry ---
    const JointSymmetry& symmetry_q0() const { return symmetry_q0_; }
    bool z4_active() const { return z4_active_; }
    void disable_z4() { z4_active_ = false; z4_cache_.clear(); }

    // --- V6 Z4 persistent cache ---
    void set_cache_manager(LectCacheManager* mgr) { cache_mgr_ = mgr; }
    LectCacheManager* cache_manager() const { return cache_mgr_; }

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

    // ── Tree structure (mmap-backed or vector-backed via TreeArray) ─────
    TreeArray<int>    left_, right_, parent_, depth_;
    TreeArray<int>    split_dim_;
    TreeArray<double> split_val_;

    // ── Memory-mapped ep_data for lazy loading ──────────────────────────
    // These are mutable because mmap is a transparent cache: materializing
    // data from mmap into vectors does not change LECT's logical state.
    mutable LectMmap mmap_;                      // RAII mmap handle
    mutable uint8_t* mmap_ep_base_ = nullptr;    // start of V5 EP section (writable for COW)
    mutable int  mmap_node_stride_ = 0;          // bytes per node in EP section (2*ep_stride*4)
    mutable int  nn_loaded_        = 0;          // nodes loaded from file
    mutable bool use_mmap_         = false;      // true if ep_data comes from mmap

    // ── mmap-backed tree structure metadata ─────────────────────────────
    // When the LECT is loaded from a V5 file, the tree section (left_,
    // right_, …) can stay in the mmap (MAP_PRIVATE, COW) instead of
    // being deep-copied into vectors.  snapshot() then only needs to
    // clone the mmap — no vector copies — saving ~20 MB per worker.
    size_t mmap_tree_off_  = 0;   ///< Byte offset of tree section in mmap file.
    int    mmap_tree_cap_  = 0;   ///< SoA stride (file capacity) of mmap tree arrays.

    /// Materialize all loaded ep_data from mmap into vectors, then close mmap.
    /// Called before save operations that may truncate the underlying file.
    void materialize_mmap() const;

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
    int n_bt_probes_ = 8;   ///< Number of probes for BEST_TIGHTEN_V2 (default 8).
    double bt_width_power_ = 0.01;  ///< Exponent for width penalty in BT_V2 (default 0.01 for IFK; auto 0 for CritSample).

    // ── Z4 cache (V5 in-memory hash, kept as runtime fallback) ────────────
    struct Z4CacheEntry {
        std::vector<float> ep_iaabbs;
        EndpointSource source = EndpointSource::IFK;
        int channel = CH_SAFE;
    };
    std::unordered_map<uint64_t, Z4CacheEntry> z4_cache_;
    JointSymmetry symmetry_q0_;
    bool z4_active_ = false;

    // ── V6 persistent cache manager (external, shared across workers) ───
    LectCacheManager* cache_mgr_ = nullptr;

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

    // ── Grid lazy-loading support (used by V5 .lect file IO) ────────────
    struct GridLazyEntry {
        const uint8_t* data_ptr = nullptr;  ///< Pointer into mmap'd buffer
        size_t byte_length = 0;             ///< Total bytes for this node's grid data
        int n_slots = 0;                    ///< Number of grid slots
    };
    std::unordered_map<int, GridLazyEntry> grid_lazy_index_;
    bool grid_lazy_ = false;

private:

    // ── IO friends ──────────────────────────────────────────────────────
    friend bool lect_save_binary(const LECT& lect, const std::string& path);
    friend bool lect_save_incremental(const LECT& lect, const std::string& path,
                                      int old_n_nodes);
    friend bool lect_load_binary(LECT& lect, const Robot& robot, const std::string& path);
    friend bool lect_save_v6(const LECT& lect, const std::string& path);
    friend bool lect_load_v6(LECT& lect, const Robot& robot, const std::string& path);
    friend struct LectIOHelper;

    // ── Private helpers ─────────────────────────────────────────────────
    void ensure_capacity(int min_cap);
    int  alloc_node();
    void materialise_link_iaabb(int i);

    int  pick_split_dim(int depth_val, const FKState& fk,
                        const std::vector<Interval>& intervals) const;
    int  pick_best_tighten_dim(const FKState& fk,
                               const std::vector<Interval>& intervals) const;
    int  pick_best_tighten_dim_v2(const FKState& caller_fk,
                                  const std::vector<Interval>& caller_intervals,
                                  int depth) const;

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

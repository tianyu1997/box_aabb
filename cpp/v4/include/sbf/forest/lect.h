// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — LECT: Link Envelope Collision Tree
//  Module: sbf::forest
//
//  Unified KD-tree that caches per-node link envelopes.
//  Built on the two-stage modular pipeline:
//    Stage 1: Endpoint iAABB generation (iFK / CritSample / Analytical / GCPC)
//    Stage 2: Link envelope construction (LinkIAABB / LinkIAABB_Grid / Hull16_Grid)
//
//  Dual-channel architecture (v4.1 → v4.2 flat-buffer optimisation):
//
//    ChannelData channels_[2]   — indexed by ChannelIdx (SAFE=0, UNSAFE=1)
//      ├── ep_data           flat contiguous endpoint iAABBs [cap × ep_stride_]
//      ├── link_iaabb_cache  flat contiguous link iAABBs    [cap × liaabb_stride_]
//      ├── source_quality    EndpointSource enum per node
//      ├── has_data          validity flag per node
//      ├── hull_grids        VoxelGrid per node (lazy)
//      └── hull_valid        hull validity flag per node
//
//  Per-link iAABBs are cached in link_iaabb_cache at compute_envelope() /
//  refine_aabb() time, so iaabbs_collide() does a single pointer read
//  (zero derivation, matching v3 performance).
//
//  Source safety classes (two disjoint groups):
//    SAFE:   iFK ≤ Analytical ≤ GCPC   (conservative bounds, substitutable)
//    UNSAFE: CritSample                 (tighter, not conservative, standalone)
//  → Each class occupies its own channel; no cross-class substitution.
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/forest/node_store.h"
#include "sbf/envelope/endpoint_store.h"
#include "sbf/envelope/endpoint_source.h"
#include "sbf/envelope/envelope_type.h"
#include "sbf/envelope/pipeline.h"
#include "sbf/envelope/envelope_cache.h"
#include "sbf/robot/robot.h"
#include "sbf/voxel/voxel_grid.h"
#include "sbf/core/types.h"
#include "sbf/core/config.h"
#include "sbf/core/joint_symmetry.h"

#include <Eigen/Core>
#include <chrono>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace sbf {
namespace forest {

/// Channel index for dual-channel LECT storage.
enum class ChannelIdx : int { SAFE = 0, UNSAFE = 1 };
static constexpr int NUM_CHANNELS = 2;

class LECT {
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    LECT() = default;

    /// Construct from robot model with default pipeline.
    LECT(const Robot& robot, double voxel_delta = 0.02, int initial_cap = 1024);

    /// Construct with modular pipeline configuration.
    LECT(const Robot& robot, const envelope::PipelineConfig& pipeline,
         int initial_cap = 1024);


    // ── Metadata ────────────────────────────────────────────────────────
    int n_nodes()        const;
    int n_dims()         const;
    int n_active_links() const;
    int capacity()       const;
    double voxel_delta() const;

    /// Hull skip volume threshold: skip hull refinement for small nodes.
    double hull_skip_vol()  const { return hull_skip_vol_; }
    void set_hull_skip_vol(double v) { hull_skip_vol_ = v; }

    /// Set split order strategy (ROUND_ROBIN / WIDEST_FIRST / BEST_TIGHTEN).
    void set_split_order(SplitOrder so);
    /// Get current split order.
    SplitOrder split_order() const { return split_order_; }

    const Robot& robot() const;
    const JointLimits& root_limits() const;
    const envelope::PipelineConfig& pipeline_config() const;
    envelope::EndpointSource source_method() const;
    envelope::EnvelopeType   envelope_type() const;

    // ── Per-node data access ────────────────────────────────────────────

    /// Get on-the-fly derived per-link iAABBs (from SAFE channel ep data).
    /// Writes n_active_links()*6 floats into caller-supplied buffer.
    /// Returns false if no data available.
    bool derive_link_iaabbs(int node_idx, float* out,
                            ChannelIdx ch = ChannelIdx::SAFE) const;

    /// Legacy compatibility: returns pointer to internal scratch buffer.
    /// Valid until next call.  Returns nullptr if no data.
    const float* get_link_iaabbs(int node_idx) const;

    /// Does node have endpoint data in the pipeline's configured channel?
    bool has_iaabb(int node_idx) const;

    /// Does node have endpoint data in a specific channel?
    bool has_channel_data(int node_idx, ChannelIdx ch) const;

    const float* get_endpoint_iaabbs(int node_idx,
                                     ChannelIdx ch = ChannelIdx::SAFE) const;
    bool has_endpoint_iaabbs(int node_idx,
                             ChannelIdx ch = ChannelIdx::SAFE) const;

    bool has_hull_grid(int node_idx,
                       ChannelIdx ch = ChannelIdx::SAFE) const;
    const voxel::VoxelGrid& get_hull_grid(int node_idx,
                                          ChannelIdx ch = ChannelIdx::SAFE) const;

    // ── Tree structure ──────────────────────────────────────────────────
    int  left(int i)       const;
    int  right(int i)      const;
    int  parent(int i)     const;
    int  depth(int i)      const;
    double split_val(int i) const;
    bool is_leaf(int i)    const;

    /// Get the actual split dimension used at a node.
    /// Falls back to split_dims_ or depth%n_dims if no per-node record.
    int  get_node_split_dim(int node_idx) const;

    std::vector<Interval> node_intervals(int node_idx) const;

    // ── Scene management ────────────────────────────────────────────────
    void set_scene(const Obstacle* obstacles, int n_obs);
    void clear_scene();
    bool has_scene_grid() const;
    const voxel::VoxelGrid& scene_grid() const;

    void set_deadline(std::optional<TimePoint> deadline);
    bool deadline_reached() const;

    // ── Core tree operations ────────────────────────────────────────────
    FFBResult find_free_box(const Eigen::VectorXd& seed,
                            const Obstacle* obstacles, int n_obs,
                            double min_edge = 1e-4, int max_depth = 30);

    /// Expand a leaf node: reconstruct intervals, compute FK, split.
    /// Returns number of new nodes (2 on success, 0 if not a leaf).
    int expand_leaf(int node_idx);

    void mark_occupied(int node_idx, int box_id);
    void unmark_occupied(int node_idx);
    bool is_occupied(int node_idx) const;
    int  forest_id(int node_idx) const;
    void clear_all_occupation();

    // ── Warm-start ──────────────────────────────────────────────────────
    LECT snapshot() const;
    int  pre_expand(int target_depth);
    int  compute_all_hull_grids();

    /// Transplant a worker's subtree back into the coordinator LECT.
    int transplant_subtree(const LECT& worker,
                           int coordinator_node,
                           int snapshot_n_nodes,
                           const std::unordered_map<int, int>& box_id_remap);

    // ── Public collision queries ────────────────────────────────────────

    /// Two-stage collision test: iAABB early-out → Hull-16 refinement.
    bool collides_scene(int node_idx,
                        const Obstacle* obstacles, int n_obs) const;

    /// Collision test for ARBITRARY C-space intervals (no tree node needed).
    bool intervals_collide_scene(const std::vector<Interval>& intervals,
                                 const Obstacle* obstacles, int n_obs) const;

    /// Hull-16 collision test against a pre-built VoxelGrid.
    bool hull_collides_grid(int node_idx,
                            const voxel::VoxelGrid& obs_grid) const;

    // ── Hull-based Greedy Coarsening ────────────────────────────────────
    double merged_children_hull_volume(int node_idx) const;
    double coarsen_volume_ratio(int node_idx) const;
    bool   merge_children_hulls(int node_idx);

    // ── Statistics / debugging ──────────────────────────────────────────
    int count_nodes_with_iaabb() const;
    int count_nodes_with_hull()  const;
    int total_hull_voxels()      const;
    int scene_grid_voxels()      const;

    /// q_0 symmetry descriptor (Z4 if α_0 ≈ 0, otherwise NONE).
    const JointSymmetry& symmetry_q0() const { return symmetry_q0_; }

    // ── Persistence ─────────────────────────────────────────────────────
    void save(const std::string& dir) const;
    static LECT load(const std::string& dir, const Robot& robot);

    // ── Channel utilities ───────────────────────────────────────────────

    /// Map EndpointSource to channel index.
    static ChannelIdx source_channel(envelope::EndpointSource src);

    /// Channel for the currently configured pipeline source.
    ChannelIdx active_channel() const;

    // ── Dual-channel per-node storage type ──────────────────────────────
    //  Index 0 = SAFE (iFK, Analytical, GCPC)
    //  Index 1 = UNSAFE (CritSample)
    struct ChannelData {
        /// Flat contiguous endpoint iAABBs:
        ///   ep_data[node * ep_stride .. + ep_stride - 1]
        ///   where ep_stride = n_active_endpoints * 6
        std::vector<float> ep_data;

        /// Flat contiguous cached per-link iAABBs (derived at envelope time):
        ///   link_iaabb_cache[node * liaabb_stride .. + liaabb_stride - 1]
        ///   where liaabb_stride = n_active_links * 6
        std::vector<float> link_iaabb_cache;

        /// EndpointSource enum per node (who produced the data)
        std::vector<uint8_t> source_quality;

        /// 1 = node has valid ep data in this channel
        std::vector<uint8_t> has_data;

        /// Hull VoxelGrid per node (derived from ep_data, lazy)
        std::vector<voxel::VoxelGrid> hull_grids;

        /// 1 = hull_grids[i] is valid
        std::vector<uint8_t> hull_valid;
    };

private:
    ChannelData channels_[NUM_CHANNELS];

    // ── Core state ──────────────────────────────────────────────────────
    const Robot* robot_ = nullptr;
    envelope::PipelineConfig pipeline_config_;
    NodeStore store_;
    double voxel_delta_ = 0.02;
    double hull_skip_vol_ = 0.0;
    JointLimits root_limits_;
    FKState root_fk_;              // full FK over root joint range
    bool scene_set_ = false;
    SplitOrder split_order_ = SplitOrder::BEST_TIGHTEN;

    // ── q_0 symmetry (Z4 rotation if α_0 ≈ 0) ─────────────────────────
    JointSymmetry symmetry_q0_;

    // ── Z4 envelope cache ──────────────────────────────────────────────
    struct Z4CacheEntry {
        std::vector<float> ep_iaabbs;
        envelope::EndpointSource source = envelope::EndpointSource::IFK;
    };
    std::unordered_map<uint64_t, Z4CacheEntry> z4_cache_;
    bool z4_cache_active_ = false;

    // ── Scene obstacle grid (for hull collision) ────────────────────────
    voxel::VoxelGrid scene_grid_;
    std::optional<TimePoint> deadline_;

    // ── Flat-buffer strides (set once in init_root) ─────────────────────
    int ep_stride_ = 0;       // n_active_endpoints * 6
    int liaabb_stride_ = 0;   // n_active_links * 6

    // ── Scratch buffers ────────────────────────────────────────────────
    mutable std::vector<float> link_iaabbs_scratch_;
    std::vector<float> radii_f_cached_;   // robot link radii as float, set once in init_root

    // ── Split dimension cycling (round-robin over joints) ───────────────
    std::vector<int> split_dims_;

    // ── Per-node split dimension (for WIDEST_FIRST + node_intervals) ───
    std::vector<int> node_split_dim_;

    // ── Private helpers ─────────────────────────────────────────────────
    void init_split_dims(int n_joints);

    int pick_best_tighten_dim(const FKState& parent_fk,
                              const std::vector<Interval>& intervals) const;

    int pick_split_dim(int depth, const FKState& fk,
                       const std::vector<Interval>& intervals);

    void compute_envelope(int node_idx, const FKState& fk,
                          const std::vector<Interval>& intervals,
                          int changed_dim = -1, int parent_idx = -1);

    void split_leaf(int node_idx, const FKState& parent_fk,
                    const std::vector<Interval>& parent_intervals,
                    int dim = -1);

    void ensure_channel_capacity(int min_cap);          // active channel only
    void ensure_channel_capacity(int min_cap, int ch);  // specific channel
    void init_root(int initial_cap = 1024);
    void refine_aabb(int node_idx);

    void derive_hull_grid(int node_idx, const std::vector<Interval>& intervals,
                          ChannelIdx ch = ChannelIdx::SAFE);

    void pre_expand_recursive(int node_idx, const FKState& fk,
                              const std::vector<Interval>& intervals,
                              int target_depth, int& new_node_count);

    bool iaabbs_collide(int node_idx,
                        const Obstacle* obstacles, int n_obs) const;

    bool hull_collides(int node_idx,
                       const Obstacle* obstacles, int n_obs) const;

    void propagate_up(const std::vector<int>& path);

    // ── Hull persistence (HUL2 format — dual-channel) ──────────────────
    void save_hull_grids(const std::string& path) const;
    void load_hull_grids(const std::string& path);

    static constexpr int      HULL_HEADER_SIZE = 512;
    static constexpr uint32_t HULL_MAGIC_V1    = 0x314C5548;  // "HUL1" LE
    static constexpr uint32_t HULL_MAGIC_V2    = 0x324C5548;  // "HUL2" LE
};

} // namespace forest
} // namespace sbf

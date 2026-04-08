// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v3 — LECT: Link Envelope Collision Tree
//  Module: sbf::forest
//
//  Unified KD-tree that caches per-node link envelopes.
//  Built on the two-stage modular pipeline:
//    Stage 1: Endpoint AABB generation (IFK / CritSample / Analytical / GCPC)
//    Stage 2: Link envelope construction (SubAABB / SubAABB_Grid / Hull16_Grid)
//
//  Per-node cached data:
//    1. Per-link AABBs       (derived from endpoint_aabbs, stored in NodeStore)
//    2. Endpoint AABBs       (per-endpoint [n_endpoints × 6], stored in ep_store_)
//    3. Hull VoxelGrids      (sparse BitBrick, stored in hull_grids_)
//
//  All layers are auto-computed when a node is first visited
//  (lazy evaluation during find_free_box descent) and auto-persisted.
//
//  Persistence layout (directory-based):
//    dir/lect.hcache    — tree structure + per-link AABBs  (HCACHE02)
//    dir/lect.frames    — endpoint AABBs (legacy FRM4 compat)
//    dir/lect.hulls     — hull VoxelGrids                  (HUL1)
//    dir/lect.meta.json — pipeline metadata
//
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/forest/node_store.h"
#include "sbf/envelope/frame_store.h"
#include "sbf/envelope/frame_source.h"
#include "sbf/envelope/envelope_type.h"
#include "sbf/envelope/envelope_cache.h"
#include "sbf/common/interval_trig.h"
#include "sbf/robot/robot.h"
#include "sbf/robot/interval_fk.h"
#include "sbf/voxel/voxel_grid.h"
#include "sbf/common/types.h"

#include <Eigen/Core>
#include <string>
#include <unordered_map>
#include <vector>

namespace sbf {
namespace forest {

class LECT {
public:
    LECT() = default;

    /// Construct from robot model.
    /// @param robot       Robot model (DH params, joint limits, link radii)
    /// @param voxel_delta Hull-16 voxel resolution in metres (e.g. 0.02)
    /// @param initial_cap Initial node capacity
    LECT(const Robot& robot, double voxel_delta = 0.02, int initial_cap = 1024);

    /// Construct from robot model with modular pipeline configuration.
    /// @param robot       Robot model
    /// @param pipeline    Pipeline config (source method + envelope type + hparams)
    /// @param initial_cap Initial node capacity
    LECT(const Robot& robot, const envelope::PipelineConfig& pipeline,
         int initial_cap = 1024);

    // ═════════════════════════════════════════════════════════════════════
    //  Metadata
    // ═════════════════════════════════════════════════════════════════════

    int n_nodes()         const { return store_.n_nodes(); }
    int n_dims()          const { return store_.n_dims(); }
    int n_active_links()  const { return store_.n_active_links(); }
    int capacity()        const { return store_.capacity(); }
    double voxel_delta()  const { return voxel_delta_; }

    /// Hull skip volume threshold: skip hull refinement for small nodes.
    double hull_skip_vol()  const { return hull_skip_vol_; }
    void set_hull_skip_vol(double v) { hull_skip_vol_ = v; }

    const Robot& robot()  const { return *robot_; }
    const JointLimits& root_limits() const { return root_limits_; }

    /// Pipeline configuration (source method + envelope type).
    const envelope::PipelineConfig& pipeline_config() const {
        return pipeline_config_;
    }
    envelope::FrameSourceMethod source_method() const {
        return pipeline_config_.source.method;
    }
    envelope::EnvelopeType envelope_type() const {
        return pipeline_config_.envelope.type;
    }

    // ═════════════════════════════════════════════════════════════════════
    //  Per-node data access
    // ═════════════════════════════════════════════════════════════════════

    /// Per-link AABBs: float[n_active_links × 6] per node.
    /// Each link: [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z].
    const float* get_link_aabbs(int node_idx) const;
    bool has_aabb(int node_idx) const { return store_.has_aabb(node_idx); }

    /// Endpoint AABBs for a node: float[n_endpoints × 6] (endpoint position intervals).
    const float* get_endpoint_aabbs(int node_idx) const;
    bool has_endpoint_aabbs(int node_idx) const;

    /// Endpoint position intervals (legacy, computed from FKState).
    const float* get_frames(int node_idx) const;
    bool has_frames(int node_idx) const;

    /// Hull-16 VoxelGrid for a node (sparse BitBrick).
    bool has_hull_grid(int node_idx) const;
    const voxel::VoxelGrid& get_hull_grid(int node_idx) const;

    // ═════════════════════════════════════════════════════════════════════
    //  Tree structure access
    // ═════════════════════════════════════════════════════════════════════

    int  left(int i)      const { return store_.left(i); }
    int  right(int i)     const { return store_.right(i); }
    int  parent(int i)    const { return store_.parent(i); }
    int  depth(int i)     const { return store_.depth(i); }
    double split_val(int i) const { return store_.split(i); }
    bool is_leaf(int i)   const { return store_.is_leaf(i); }

    /// Reconstruct C-space intervals for a node by descending from root.
    std::vector<Interval> node_intervals(int node_idx) const;

    // ═════════════════════════════════════════════════════════════════════
    //  Scene management (pre-rasterized obstacle VoxelGrid)
    // ═════════════════════════════════════════════════════════════════════

    /// Pre-rasterize obstacles into a shared VoxelGrid for fast hull collision.
    /// Call once before find_free_box; reuse across all FFB queries.
    /// If obstacles change, call set_scene again to rebuild.
    void set_scene(const Obstacle* obstacles, int n_obs);

    /// Clear the pre-rasterized scene grid.
    void clear_scene();

    /// Whether a pre-rasterized scene grid is available.
    bool has_scene_grid() const { return scene_set_; }

    /// Access the pre-rasterized scene grid (const).
    const voxel::VoxelGrid& scene_grid() const { return scene_grid_; }

    // ═════════════════════════════════════════════════════════════════════
    //  Core tree operations
    // ═════════════════════════════════════════════════════════════════════

    /// Find a collision-free box containing the seed config.
    /// Descends the KD-tree lazily, splitting leaves and computing
    /// envelopes (AABBs + frames + hull-16) on each new node.
    ///
    /// @param seed        Target configuration (n_dims)
    /// @param obstacles   Obstacle array (AABB boxes)
    /// @param n_obs       Number of obstacles
    /// @param min_edge    Minimum box edge length to split
    /// @param max_depth   Maximum tree depth
    /// @return FFBResult with node_idx and path on success
    FFBResult find_free_box(const Eigen::VectorXd& seed,
                            const Obstacle* obstacles, int n_obs,
                            double min_edge = 1e-4, int max_depth = 30);

    /// Mark node as occupied with given forest box ID.
    void mark_occupied(int node_idx, int box_id);

    /// Unmark node occupation.
    void unmark_occupied(int node_idx);

    /// Query occupation.
    bool is_occupied(int node_idx) const { return store_.is_occupied(node_idx); }
    int  forest_id(int node_idx)   const { return store_.forest_id(node_idx); }

    /// Clear all node occupation markers (keeps tree structure + cached envelopes).
    void clear_all_occupation() { store_.clear_all_occupation(); }

    // ═════════════════════════════════════════════════════════════════════
    //  Warm-start: snapshot + pre-expansion
    // ═════════════════════════════════════════════════════════════════════

    /// Create a deep copy of the entire LECT (tree structure + all cached
    /// envelopes), but with cleared occupation state.  The snapshot has a
    /// fresh, empty FrameStore (workers don't need persistence).
    /// Used by parallel grow: main thread pre-expands, then each worker
    /// gets its own snapshot to independently descend and mutate.
    LECT snapshot() const;

    /// Pre-expand the tree by splitting all leaves up to target_depth.
    /// Computes FK + envelope for every new node.
    /// @return Number of new nodes created.
    int pre_expand(int target_depth);

    /// Compute hull grids for all nodes that have AABBs but no hull yet.
    /// Intended for offline cache building after pre_expand().
    /// @return Number of hull grids computed.
    int compute_all_hull_grids();

    /// Transplant a worker's subtree back into the coordinator LECT.
    /// After parallel growth, each worker independently expands nodes
    /// under its partition cell.  This method copies those new nodes
    /// into the coordinator, preserving envelopes and occupation.
    ///
    /// @param worker              The worker's LECT (post-growth)
    /// @param coordinator_node    The coordinator leaf node that the worker grew under
    /// @param snapshot_n_nodes    Coordinator's n_nodes() at snapshot time
    ///                           (worker nodes with index >= this are new)
    /// @param box_id_remap        Maps worker box IDs → coordinator box IDs
    ///                           (for remapping forest_id occupation)
    /// @return Number of nodes transplanted
    int transplant_subtree(const LECT& worker,
                           int coordinator_node,
                           int snapshot_n_nodes,
                           const std::unordered_map<int, int>& box_id_remap);

    // ═════════════════════════════════════════════════════════════════════
    //  Persistence
    // ═════════════════════════════════════════════════════════════════════

    /// Save all data to directory:
    ///   dir/lect.hcache  — tree structure + per-link AABBs
    ///   dir/lect.frames  — endpoint position intervals
    ///   dir/lect.hulls   — hull-16 VoxelGrids
    void save(const std::string& dir) const;

    /// Load from directory.  Requires the same Robot model.
    void load(const std::string& dir, const Robot& robot);

    // ═════════════════════════════════════════════════════════════════════
    //  Public collision queries
    // ═════════════════════════════════════════════════════════════════════

    /// Two-stage collision test: AABB early-out → Hull-16 refinement.
    /// Uses pre-rasterized scene grid if available, else builds on-the-fly.
    /// @return true if node's envelope collides with scene obstacles
    bool collides_scene(int node_idx,
                        const Obstacle* obstacles, int n_obs) const;

    /// Collision test for ARBITRARY C-space intervals (no tree node needed).
    /// Computes endpoint AABBs via the configured source, extracts link AABBs,
    /// and checks AABB overlap with obstacles.  If grid-based envelope, also
    /// builds a temporary hull grid and checks against the scene grid.
    /// @return true if the interval box collides with obstacles
    bool intervals_collide_scene(const std::vector<Interval>& intervals,
                                 const Obstacle* obstacles, int n_obs) const;

    /// Hull-16 collision test against a pre-built VoxelGrid.
    /// Public API for manual scene grid management.
    bool hull_collides_grid(int node_idx,
                            const voxel::VoxelGrid& obs_grid) const;

    // ═════════════════════════════════════════════════════════════════════
    //  Hull-based Greedy Coarsening
    // ═════════════════════════════════════════════════════════════════════

    /// Compute the bitwise-OR merged hull volume of a node's two children.
    /// Returns the merged grid's occupied volume (m³).
    /// If the node has no children or children lack hull grids, returns -1.
    double merged_children_hull_volume(int node_idx) const;

    /// Compute the coarsening quality ratio:
    ///   merged_vol / max(left_vol, right_vol)
    /// A ratio of 1.0 means perfect zero-loss merge (children don't overlap
    /// in different directions). Lower is better for coverage efficiency.
    /// Returns -1 if not computable.
    double coarsen_volume_ratio(int node_idx) const;

    /// Merge child hull grids into parent hull grid (bitwise-OR).
    /// After merging, the parent's hull grid represents the union of both
    /// children's swept volumes — a non-convex, zero-loss merge.
    /// Returns true if merge was performed successfully.
    bool merge_children_hulls(int node_idx);

    // ═════════════════════════════════════════════════════════════════════
    //  Statistics / debugging
    // ═════════════════════════════════════════════════════════════════════

    int count_nodes_with_aabb() const;
    int count_nodes_with_hull() const;
    int total_hull_voxels()     const;

    /// Total scene grid voxels (0 if no scene set)
    int scene_grid_voxels() const;

private:
    const Robot* robot_ = nullptr;
    double voxel_delta_ = 0.02;
    double hull_skip_vol_ = 0.0;  // P5: skip hull refinement for nodes with C-space vol < this

    // ── Joint-freezing depth for q₀/q₁/... symmetry caching ────────────
    //  freeze_depth_ = number of leading joints frozen to q=0 during
    //  Stage 1 endpoint AABB computation.  Enables inheritance of cached
    //  endpoint AABBs when splitting on any frozen dimension.
    //  Default: 2 (freeze q₀ and q₁) for robots with a_i=0, α∈{0,±π/2}.
    int freeze_depth_ = 2;
    std::vector<FrozenJointDesc> frozen_descs_;  // DH descriptors for frozen joints

    // ── Modular pipeline configuration ──────────────────────────────────
    envelope::PipelineConfig pipeline_config_ =
        envelope::PipelineConfig::production();

    // ── Three-layer storage ─────────────────────────────────────────────
    NodeStore                    store_;         // tree + per-link AABBs
    envelope::FrameStore         frame_store_;   // per-node frame intervals (legacy compat)

    // Endpoint AABB store: per-node endpoint_aabbs [n_endpoints × 6]
    // (per-endpoint position intervals, geometry only — no radius)
    std::vector<std::vector<float>> ep_store_;    // per-node endpoint_aabbs

    std::vector<voxel::VoxelGrid> hull_grids_;   // per-node hull VoxelGrids
    std::vector<uint8_t>          hull_valid_;   // validity flags

    // ── Pre-rasterized scene (obstacle grid) ────────────────────────────
    voxel::VoxelGrid scene_grid_;     // all obstacles merged into one grid
    bool             scene_set_ = false;

    // ── KD-tree metadata ────────────────────────────────────────────────
    JointLimits root_limits_;
    FKState     root_fk_;
    std::vector<int> split_dims_;   // dimension cycling sequence

    // ────────────────────────────────────────────────────────────────────
    //  Internal helpers
    // ────────────────────────────────────────────────────────────────────

    /// Initialize root node (full FK → frames → AABBs → hull-16).
    void init_root();

    /// Compute and cache all envelope data for a node via modular pipeline.
    /// @param fk         FKState (for IFK source + incremental support)
    /// @param intervals  C-space intervals for this node
    void compute_envelope(int node_idx, const FKState& fk,
                          const std::vector<Interval>& intervals);

    /// Derive hull VoxelGrid from stored endpoint AABBs using fill_hull16.
    /// Constructs Conv(B_prox ∪ B_dist) ⊕ Ball(r) per active link directly
    /// from proximal/distal endpoint-AABB pairs (no sub-AABB intermediate).
    /// @param intervals  Full C-space intervals for the node (frozen joints
    ///                   are extracted internally based on freeze_depth_).
    void derive_hull_grid(int node_idx, const std::vector<Interval>& intervals);

    /// Ensure endpoint_aabb storage capacity.
    void ensure_ep_capacity(int n);

    /// Reconstruct Cartesian link AABBs from local-frame ep_store_.
    /// Applies multi-level rotation using frozen joints' intervals.
    void reconstruct_and_derive_link_aabbs(int node_idx,
                                           const std::vector<Interval>& intervals);

    /// Split a leaf node at its midpoint; computes envelopes for both children.
    void split_leaf(int node_idx, const FKState& parent_fk,
                    const std::vector<Interval>& parent_intervals);

    /// Recursive helper for pre_expand.
    void pre_expand_recursive(int node_idx, const FKState& fk,
                              const std::vector<Interval>& intervals,
                              int target_depth, int& new_node_count);

    /// AABB collision test: any (link, obstacle) pair overlaps?
    bool aabbs_collide(int node_idx,
                       const Obstacle* obstacles, int n_obs) const;

    /// Hull-16 grid collision test (refinement after AABB positive).
    /// Uses pre-rasterized scene grid if available, else builds on-the-fly.
    bool hull_collides(int node_idx,
                       const Obstacle* obstacles, int n_obs) const;

    /// Refine parent AABBs by intersecting with union(left, right).
    void refine_aabb(int parent_idx);

    /// Propagate AABB refinement upward from node to root.
    void propagate_up(const std::vector<int>& path);

    /// Ensure hull grid storage has capacity for at least n nodes.
    void ensure_hull_capacity(int n);

    // ── Hull persistence (HUL1 format) ──────────────────────────────────
    void save_hull_grids(const std::string& path) const;
    void load_hull_grids(const std::string& path);

    static constexpr int      HULL_HEADER_SIZE = 512;
    static constexpr uint32_t HULL_MAGIC       = 0x314C5548;  // "HUL1" LE
};

} // namespace forest
} // namespace sbf

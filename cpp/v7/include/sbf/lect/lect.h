#pragma once
/// @file lect.h
/// @brief LECT — Link Envelope Collision Tree (binary KD over C-space).
///
/// v7 P3 minimal port: vector-backed, single-channel, no Z4/mmap.
/// D1 invariant: stored link AABBs are zero-radius; `collides_scene`
/// applies link_radii inline via `aabbs_collide_obs_inflated`.
/// Minimal binary save/load support is provided for cross-process reuse.

#include "sbf/core/endpoint_iaabb.h"
#include "sbf/core/fk_state.h"
#include "sbf/core/robot.h"
#include "sbf/core/types.h"
#include "sbf/envelope/envelope_type.h"

#include <Eigen/Dense>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace sbf::lect {

enum class SplitOrder : uint8_t {
    ROUND_ROBIN  = 0,
    WIDEST_FIRST = 1,   ///< v7 P3 default
};

struct LECTConfig {
    SplitOrder split_order   = SplitOrder::WIDEST_FIRST;
    int        initial_cap   = 1023;
    double     min_norm_width = 1e-4;   ///< skip dims narrower than this when picking
    sbf::core::EndpointSourceConfig endpoint_source = {};
};

class LECT {
public:
    LECT(const sbf::core::Robot& robot,
         const std::vector<sbf::core::Interval>& root_intervals,
         const sbf::envelope::EnvelopeTypeConfig& env_config,
         LECTConfig cfg = {});

    LECT(LECT&&) noexcept            = default;
    LECT& operator=(LECT&&) noexcept = default;
    LECT(const LECT&)                = default;
    LECT& operator=(const LECT&)     = default;

    /// Deep-copy snapshot; sets snapshot_base on the clone to its own n_nodes.
    LECT snapshot() const;

    /// Persist the current LECT state to a binary file.
    /// The file stores the robot fingerprint, root intervals, envelope config,
    /// and tree arrays needed to reconstruct the cache in another process.
    void save_binary(const std::string& path) const;

    /// Load a persisted LECT from a binary file and validate that it matches
    /// the supplied robot fingerprint and active-link layout.
    static LECT load_binary(const std::string& path,
                            const sbf::core::Robot& robot);

    // ── Multi-thread support (P4.5) ──────────────────────────────────
    /// Partition the LECT so that each input seed lands in a distinct leaf,
    /// then promote each seed's leaf upward to the largest ancestor whose
    /// subtree does not contain any other seed's leaf. Returns a vector of
    /// the same size as `seeds`; element i is the master node index that
    /// serves as the exclusive domain root for seed i.
    /// May call expand_leaf at most `max_extra_splits` times in total.
    std::vector<int> partition_for_seeds(
        const std::vector<Eigen::VectorXd>& seeds,
        int max_extra_splits = 256);

    /// Merge a worker LECT's subtree rooted at `domain_root_idx` (an index
    /// valid in BOTH worker and master) back into this master LECT. Worker
    /// nodes with index < snapshot_base() are inherited (skipped); fresh
    /// indices are allocated for nodes >= snapshot_base. `id_map` remaps
    /// box ids stored in worker.forest_id_ → global ids. `node_remap` is
    /// filled with worker_idx → master_idx for every visited worker node.
    /// Returns the number of new master nodes shipped.
    int transplant_domain(const LECT& worker, int domain_root_idx,
                          const std::unordered_map<int,int>& id_map,
                          std::unordered_map<int,int>& node_remap);

    // ── Tree introspection ────────────────────────────────────────────
    int n_nodes()        const { return n_nodes_; }
    int n_dims()         const { return n_dims_; }
    int n_active_links() const { return n_active_links_; }
    int snapshot_base()  const { return snapshot_base_; }
    int left(int i)      const { return left_[i]; }
    int right(int i)     const { return right_[i]; }
    int parent(int i)    const { return parent_[i]; }
    int depth(int i)     const { return depth_[i]; }
    bool is_leaf(int i)  const { return left_[i] < 0 && right_[i] < 0; }
    int  split_dim(int i)const { return split_dim_[i]; }
    double split_val(int i) const { return split_val_[i]; }

    /// Reconstruct the joint intervals at node `i` by walking root → i.
    std::vector<sbf::core::Interval> node_intervals(int i) const;

    /// Zero-radius link AABBs of node i: pointer to
    /// [n_active_links * n_subdivisions * 6]. Lazy materialisation
    /// from `ep_data_`. When `n_subdivisions() == 1` this is the
    /// classic per-link hull; otherwise each link is split into
    /// `n_subdivisions()` segments and the union of segment AABBs is
    /// substantially tighter than the hull.
    const float* get_link_iaabbs(int i) const;

    /// Number of envelope slots per node (n_active_links * n_subdivisions).
    int n_slots() const { return n_active_links_ * n_subdivisions_; }

    /// Per-link subdivisions used by the envelope cache (\(\geq 1\)).
    int n_subdivisions() const { return n_subdivisions_; }

    // ── Core operations ───────────────────────────────────────────────
    /// Split leaf along chosen dim; returns 2 on success, 0 if not a leaf.
    int expand_leaf(int node_idx);

    // ── D1 collision check ────────────────────────────────────────────
    bool collides_scene(int node_idx,
                        const float* obs_compact, int n_obs) const;

    // ── Geometric queries ─────────────────────────────────────────────
    int find_leaf_containing(const double* q) const;
    bool is_descendant_of(int node, int ancestor) const;

    // ── Occupation ────────────────────────────────────────────────────
    void mark_occupied(int node_idx, int box_id);
    void unmark_occupied(int node_idx);
    bool is_occupied(int node_idx) const { return forest_id_[node_idx] >= 0; }
    int  forest_id(int node_idx)   const { return forest_id_[node_idx]; }
    int  subtree_occ(int node_idx) const { return subtree_occ_[node_idx]; }
    void clear_all_occupation();
    bool is_point_occupied(const double* q) const;
    void mark_point_occupied(const double* q, int box_id);

    // ── Free-volume tracking ──────────────────────────────────────────
    double subtree_free_volume(int i) const { return subtree_free_volume_[i]; }
    void   refresh_free_volumes();

    // ── Accessors ─────────────────────────────────────────────────────
    const sbf::core::Robot& robot() const { return robot_; }
    const LECTConfig& config() const { return cfg_; }
    const sbf::envelope::EnvelopeTypeConfig& env_config() const { return env_config_; }
    const std::vector<sbf::core::Interval>& root_intervals() const { return root_intervals_; }

private:
    // ── Tree structure ────────────────────────────────────────────
    std::vector<int>    left_, right_, parent_, depth_, split_dim_;
    std::vector<double> split_val_;

    // ── Envelope cache (zero-radius) ──────────────────────────────
    std::vector<uint8_t> has_data_;             // 0/1
    std::vector<float>   ep_data_;              // [cap × ep_stride]
    mutable std::vector<float>   link_iaabb_cache_;  // [cap × liaabb_stride]
    mutable std::vector<uint8_t> link_iaabb_dirty_;

    // ── Occupation ────────────────────────────────────────────────
    std::vector<int>    forest_id_;             // -1 = free
    std::vector<int>    subtree_occ_;
    std::vector<double> subtree_free_volume_;

    // ── Config ────────────────────────────────────────────────────
    sbf::core::Robot robot_;
    sbf::envelope::EnvelopeTypeConfig env_config_;
    std::vector<sbf::core::Interval> root_intervals_;
    LECTConfig cfg_;
    std::vector<float> radii_per_slot_;         // [n_active_links_], D1 inflation

    int ep_stride_      = 0;      // n_active * 2 * 6
    int liaabb_stride_  = 0;      // n_active * n_subdivisions * 6
    int n_dims_         = 0;
    int n_active_links_ = 0;
    int n_subdivisions_ = 1;
    int n_nodes_        = 0;
    int capacity_       = 0;
    int snapshot_base_  = 0;

    // ── Helpers ───────────────────────────────────────────────────
    void ensure_capacity(int min_cap);
    int  alloc_node();
    void materialise_link_iaabb(int i) const;
    int  pick_split_dim(int depth, const std::vector<sbf::core::Interval>& iv) const;
    void compute_envelope(int node_idx,
                          const sbf::core::FKState& fk,
                          const std::vector<sbf::core::Interval>& intervals,
                          int changed_dim,
                          int parent_idx);
    void split_leaf_impl(int node_idx,
                         const sbf::core::FKState& parent_fk,
                         const std::vector<sbf::core::Interval>& parent_iv);
    double node_box_volume(int i) const;
    static double interval_volume(const std::vector<sbf::core::Interval>& iv);
    float* ep_write(int i)       { return ep_data_.data() + static_cast<size_t>(i) * ep_stride_; }
    const float* ep_read(int i) const { return ep_data_.data() + static_cast<size_t>(i) * ep_stride_; }
};

}  // namespace sbf::lect

// SafeBoxForest v2  LECT: Lifelong Envelope Cache Tree
// Module: sbf::forest
//
// The LECT (Lifelong Envelope Cache Tree) is the core certified region cache
// described in the SBF paper.  It composes two layers:
//
//   Layer 1  HierAABBTree
//   --------------------------------------------------------------------------
//   Lazy KD-tree over C-space.  Each node stores:
//     * topology (parent, children, split dim/value)
//     * per-link AABB data computed via interval-FK
//   Drives the FFB algorithm and is persisted in HCACHE02 format.
//
//   Layer 2  FrameStore (HCACHE03)
//   --------------------------------------------------------------------------
//   Per-node frame position intervals: the universal base representation
//   from which all envelope types (AABB, subdivided AABB, IFK OBB, grid)
//   are derived on demand via pure functions in envelope_derive.h.
//
//   Collision checking is dispatched through CollisionPolicy (config.h)
//   via check_collision() (collision_policy.h) -- no virtual dispatch on
//   the hot path.
//
// Typical setup
// --------------------------------------------------------------------------
//   LECT lect(robot);
//   lect.set_envelope_config(cfg);
//
//   FFBResult r = lect.find_free_box(seed, obs, n_obs);
//   if (r.success()) {
//       bool safe = !lect.check_node_collision_policy(r.node_idx, obs, n_obs);
//   }
//
#pragma once

#include "sbf/forest/hier_aabb_tree.h"
#include "sbf/envelope/frame_store.h"
#include "sbf/envelope/grid_store.h"
#include "sbf/envelope/collision_policy.h"
#include "sbf/robot/robot.h"
#include "sbf/common/types.h"
#include "sbf/common/config.h"

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

namespace sbf {
namespace forest {

// =============================================================================
//  LECT -- Lifelong Envelope Cache Tree
// =============================================================================
class LECT {
public:
    // -- Construction ---------------------------------------------------------

    LECT() = default;
    explicit LECT(const Robot& robot, const std::string& tag = "ifk");

    LECT(const LECT&)            = delete;
    LECT& operator=(const LECT&) = delete;
    LECT(LECT&&)                 = default;
    LECT& operator=(LECT&&)      = default;

    // -- Find Free Box --------------------------------------------------------

    FFBResult find_free_box(
        const Eigen::VectorXd& seed,
        const float* obs_compact, int n_obs,
        int max_depth = 200,
        double min_edge = 0.01) const;

    // -- Promotion ------------------------------------------------------------

    HierAABBTree::PromotionResult try_promote(
        const std::vector<int>& path,
        const float* obs_compact, int n_obs,
        int promotion_depth = 2) const;

    // -- Node intervals -------------------------------------------------------

    std::vector<Interval> get_node_intervals(int node_idx) const;

    // -- Hull safety check (for greedy coarsening) ----------------------------

    bool check_hull_safe(
        const std::vector<Interval>& hull,
        const std::vector<Interval>& a_ivs,
        const std::vector<Interval>& b_ivs,
        const float* obs_compact, int n_obs,
        int max_split_depth = 3,
        double min_edge = 0.001) const;

    // -- Persistence ----------------------------------------------------------

    void save(const std::string& tree_path);
    void save_incremental(const std::string& tree_path);

    static LECT load(const std::string& tree_path, const Robot& robot);
    static LECT load_mmap(const std::string& tree_path, const Robot& robot);

    void flush_mmap();

    // -- Occupation management ------------------------------------------------

    void mark_occupied(int node_idx, int forest_box_id);
    void unmark_occupied(int node_idx);
    void clear_boxes_occupation(const std::unordered_set<int>& box_ids);
    void clear_all_occupation();
    int  find_containing_box_id(const Eigen::VectorXd& config) const;
    bool is_occupied(const Eigen::VectorXd& config) const;
    bool sample_unoccupied_seed(std::mt19937& rng, Eigen::VectorXd& out,
                                int max_walk_depth = 12) const;

    // -- Direct accessors -----------------------------------------------------

    HierAABBTree&       tree()        { return tree_; }
    const HierAABBTree& tree()  const { return tree_; }
    const Robot&        robot() const { return tree_.robot(); }
    int total_fk_calls()        const { return tree_.total_fk_calls(); }

    // -- Frame Store (HCACHE03) -----------------------------------------------

    envelope::FrameStore&       frame_store()       { return frame_store_; }
    const envelope::FrameStore& frame_store() const { return frame_store_; }

    void store_node_frames(int node_idx, const FKState& fk);
    void union_node_frames(int parent_idx, int child_a, int child_b);
    bool refine_node_frames(int node_idx, int parent_idx);

    // -- Grid Store (bitfield occupancy union) --------------------------------

    envelope::GridStore&       grid_store()       { return grid_store_; }
    const envelope::GridStore& grid_store() const { return grid_store_; }

    // Initialize grid store from robot + config.  Called lazily on first FFB
    // if use_grid_union is enabled but grid_store not yet initialized.
    void init_grid_store();
    bool grid_store_ready() const { return grid_store_ready_; }

    // -- Frame-based collision checking ---------------------------------------

    bool check_node_collision_policy(
        int node_idx,
        const float* obs_compact, int n_obs,
        const envelope::EnvelopeConfig& env_cfg) const;

    bool check_node_collision_policy(
        int node_idx,
        const float* obs_compact, int n_obs) const;

    // -- Tag / fingerprint (tree identity) -------------------------------------

    const std::string& tag()  const { return tag_; }
    void set_tag(const std::string& t) { tag_ = t; }

    bool is_certified() const { return tag_ == "ifk"; }

    // -- Envelope configuration -----------------------------------------------

    void set_envelope_config(const envelope::EnvelopeConfig& cfg) { env_config_ = cfg; }
    const envelope::EnvelopeConfig& envelope_config() const { return env_config_; }

private:
    HierAABBTree                tree_;
    envelope::FrameStore        frame_store_;
    envelope::GridStore         grid_store_;
    envelope::EnvelopeConfig    env_config_;
    std::string                 tag_ = "ifk";  // "ifk" = certified, "crit" = heuristic
    bool                        grid_store_ready_ = false;

    // Build FFBCallbacks for grid-enabled FFB
    FFBCallbacks make_grid_callbacks() const;
};

} // namespace forest
} // namespace sbf

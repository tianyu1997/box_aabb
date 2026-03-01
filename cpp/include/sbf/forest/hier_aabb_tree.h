// SafeBoxForest — HierAABBTree: hierarchical KD-tree with interval FK
// The core find_free_box (FFB) algorithm lives here.
#pragma once

#include "sbf/aabb/interval_fk.h"
#include "sbf/core/robot.h"
#include "sbf/core/types.h"
#include "sbf/forest/node_store.h"
#include <random>
#include <string>
#include <vector>

namespace sbf {

class HierAABBTree {
public:
    HierAABBTree() = default;
    ~HierAABBTree();

    // Move-only (mmap state is non-copyable)
    HierAABBTree(HierAABBTree&& o) noexcept;
    HierAABBTree& operator=(HierAABBTree&& o) noexcept;
    HierAABBTree(const HierAABBTree&) = delete;
    HierAABBTree& operator=(const HierAABBTree&) = delete;

    // Construct a new tree for given robot
    HierAABBTree(const Robot& robot, int initial_cap = 64);

    // Construct from existing NodeStore (e.g. loaded from HCACHE)
    HierAABBTree(const Robot& robot, NodeStore store);

    // ── Core: Find Free Box ──────────────────────────────────────────────
    // Descend from root, lazily splitting KD nodes, computing interval FK,
    // and SAT-testing link AABBs against obstacles.
    // Returns the largest collision-free box containing `seed`.
    //
    FFBResult find_free_box(const Eigen::VectorXd& seed,
                            const float* obs_flat, int n_obs,
                            int max_depth = 200,
                            double min_edge = 0.01) const;

    // ── Promotion ────────────────────────────────────────────────────────
    // Try to promote a found free box upward (merge with sibling).
    // Matches v4 Python: walks up the path, absorbs occupied subtrees
    // when parent AABB is collision-free (returns absorbed forest_box_ids).
    // promotion_depth controls recursive subtree collision check precision.
    struct PromotionResult {
        int result_idx;
        std::vector<int> absorbed_box_ids;
    };
    PromotionResult try_promote(const std::vector<int>& path,
                                 const float* obs_flat, int n_obs,
                                 int promotion_depth = 2) const;

    // Legacy: simple promotion (no absorption) — kept for backward compat
    int try_promote_simple(int node_idx, const float* obs_flat, int n_obs) const;

    // ── Mark node as occupied ────────────────────────────────────────────
    void mark_occupied(int node_idx, int forest_box_id);
    void unmark_occupied(int node_idx);

    // ── Occupancy queries ────────────────────────────────────────────────
    // Check if config falls inside an occupied node (returns forest_box_id or -1)
    int find_containing_box_id(const Eigen::VectorXd& config) const;

    // Check if config is inside any occupied region
    bool is_occupied(const Eigen::VectorXd& config) const {
        return find_containing_box_id(config) >= 0;
    }

    // ── Guided sampling ──────────────────────────────────────────────────
    // Sample from unoccupied regions weighted by free volume
    // Returns false if entire space is occupied
    bool sample_unoccupied_seed(std::mt19937& rng, Eigen::VectorXd& out,
                                 int max_walk_depth = 12) const;

    // ── Extract box intervals from node ──────────────────────────────────
    // Reconstruct the C-space intervals for a given node by tracing from root
    std::vector<Interval> get_node_intervals(int node_idx) const;

    // ── Hull safety check (for greedy coarsen) ───────────────────────────
    // Check if hull(A,B) \ (A ∪ B) is collision-free, using cached AABBs.
    // Traverses the KD-tree; gap regions use cached link AABBs for SAT test
    // (skipping interval FK). If a leaf collides, lazily splits up to
    // max_split_depth levels (computing FK + storing AABB permanently).
    // min_edge: minimum joint interval width to allow splitting.
    bool check_hull_safe(const std::vector<Interval>& hull,
                         const std::vector<Interval>& a_ivs,
                         const std::vector<Interval>& b_ivs,
                         const float* obs_flat, int n_obs,
                         int max_split_depth = 3,
                         double min_edge = 0.001) const;

    // ── Persistence (HCACHE02 binary format) ───────────────────────────
    // Save tree to HCACHE02 binary file (full write, standard file I/O)
    // Format: [4096B header][node0][node1]...
    void save(const std::string& path) const;

    // Incremental save: only write dirty + newly allocated nodes.
    // Requires prior save() to same path. Reduces I/O 10-100×.
    // After call, source_path_ is updated and dirty flags cleared.
    void save_incremental(const std::string& path);

    // Load tree from HCACHE02 binary file (full read into memory)
    // The Robot must match the one used during save (same n_joints, n_links)
    static HierAABBTree load(const std::string& path, const Robot& robot);

    // Load tree via mmap (lazy: pages loaded on demand by OS).
    // Tree writes go to mmap → auto-persisted. Supports dynamic grow.
    // Close with close_mmap() or destructor.
    static HierAABBTree load_mmap(const std::string& path, const Robot& robot);

    // Flush mmap to disk (no-op if not mmap-backed)
    void flush_mmap();

    // Close mmap mapping (no-op if not mmap-backed)
    void close_mmap();

    // ── Access ───────────────────────────────────────────────────────────
    NodeStore& store() { return store_; }
    const NodeStore& store() const { return store_; }
    const Robot& robot() const { return *robot_; }

    // Split dimensions sequence
    const std::vector<int>& split_dims() const { return split_dims_; }

    // Root FK state (computed once, reused for all FFB calls)
    const FKState& root_fk_state() const { return root_fk_; }

    // Statistics
    int total_fk_calls() const { return total_fk_calls_; }

private:
    const Robot* robot_ = nullptr;
    const double* link_radii_ = nullptr;  // compact link radii for AABB inflation
    mutable NodeStore store_;
    std::vector<int> split_dims_;      // dimension cycling sequence
    JointLimits root_intervals_;       // full joint limits as root intervals
    FKState root_fk_;                  // FK state for root (full joint ranges)
    mutable int total_fk_calls_ = 0;

    // Persistence state
    std::string source_path_;          // path from last save/load (for incremental)
    int source_n_alloc_ = 0;           // n_alloc at time of last save/load

    // mmap state (only set by load_mmap)
    int mmap_fd_ = -1;
    char* mmap_ptr_ = nullptr;
    size_t mmap_size_ = 0;

    void init_root();
    void compute_root_fk();

    // Write HCACHE02 header to buffer (shared by save/save_incremental)
    void write_header_to(char* header, int n_alloc_override = -1) const;

    // mmap grow callback (called by NodeStore::ensure_capacity)
    void grow_mmap(int new_cap);

    // Propagate AABB refinement from node upward to root
    // aabb[parent] = intersect(old, union(left, right))
    void propagate_up(int node_idx) const;

    // Recursive subtree collision check for promotion
    // promotion_depth=0: check this node's AABB
    // promotion_depth=k: recurse 2^k descendants
    bool promotion_collide_check(int idx, const float* obs_flat,
                                  int n_obs, int remaining_depth) const;

    // Collect all forest_box_ids from occupied nodes in subtree
    void collect_forest_ids(int idx, std::vector<int>& out) const;

    // Clear occupation marks in subtree
    void clear_subtree_occupation(int idx) const;

    // Recursive helper for check_hull_safe
    bool hull_safe_recurse(int idx, double* ivs_lo, double* ivs_hi,
                           const std::vector<Interval>& hull,
                           const std::vector<Interval>& a_ivs,
                           const std::vector<Interval>& b_ivs,
                           const float* obs_flat, int n_obs,
                           int remaining_splits, double min_edge) const;
};

} // namespace sbf

// SafeBoxForest — HierAABBTree implementation
// Core find_free_box (FFB) algorithm
#include "sbf/forest/hier_aabb_tree.h"
#include "sbf/aabb/fk_scalar.h"
#include "sbf/forest/collision.h"
#include "sbf/io/hcache.h"
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <random>

namespace sbf {

HierAABBTree::HierAABBTree(const Robot& robot, int initial_cap)
    : robot_(&robot), link_radii_(robot.active_link_radii())
{
    store_ = NodeStore(robot.n_active_links(), robot.n_joints(), initial_cap);
    store_.set_active_link_map(robot.active_link_map(), robot.n_active_links());
    init_root();
    compute_root_fk();
}

HierAABBTree::HierAABBTree(const Robot& robot, NodeStore store)
    : robot_(&robot), link_radii_(robot.active_link_radii()), store_(std::move(store))
{
    store_.set_active_link_map(robot.active_link_map(), robot.n_active_links());
    init_root();
    compute_root_fk();
}

HierAABBTree::~HierAABBTree() {
    close_mmap();
}

HierAABBTree::HierAABBTree(HierAABBTree&& o) noexcept
    : robot_(o.robot_), link_radii_(o.link_radii_),
      store_(std::move(o.store_)),
      split_dims_(std::move(o.split_dims_)),
      root_intervals_(o.root_intervals_),
      root_fk_(o.root_fk_),
      total_fk_calls_(o.total_fk_calls_),
      source_path_(std::move(o.source_path_)),
      source_n_alloc_(o.source_n_alloc_),
      mmap_fd_(o.mmap_fd_), mmap_ptr_(o.mmap_ptr_), mmap_size_(o.mmap_size_)
{
    o.mmap_fd_ = -1;
    o.mmap_ptr_ = nullptr;
    o.mmap_size_ = 0;
    // Re-hook resize callback to this instance
    if (mmap_ptr_) {
        store_.set_resize_callback([this](int /*old*/, int new_cap) {
            grow_mmap(new_cap);
        });
    }
}

HierAABBTree& HierAABBTree::operator=(HierAABBTree&& o) noexcept {
    if (this != &o) {
        close_mmap();
        robot_ = o.robot_;
        link_radii_ = o.link_radii_;
        store_ = std::move(o.store_);
        split_dims_ = std::move(o.split_dims_);
        root_intervals_ = o.root_intervals_;
        root_fk_ = o.root_fk_;
        total_fk_calls_ = o.total_fk_calls_;
        source_path_ = std::move(o.source_path_);
        source_n_alloc_ = o.source_n_alloc_;
        mmap_fd_ = o.mmap_fd_;
        mmap_ptr_ = o.mmap_ptr_;
        mmap_size_ = o.mmap_size_;
        o.mmap_fd_ = -1;
        o.mmap_ptr_ = nullptr;
        o.mmap_size_ = 0;
        if (mmap_ptr_) {
            store_.set_resize_callback([this](int /*old*/, int new_cap) {
                grow_mmap(new_cap);
            });
        }
    }
    return *this;
}

void HierAABBTree::init_root() {
    // Build split dimensions sequence (round-robin over joints)
    split_dims_.resize(robot_->n_joints());
    for (int i = 0; i < robot_->n_joints(); ++i)
        split_dims_[i] = i;

    // Root intervals = full joint limits
    root_intervals_ = robot_->joint_limits();
}

void HierAABBTree::compute_root_fk() {
    std::vector<Interval> ivs = root_intervals_.limits;
    root_fk_ = compute_fk_full(*robot_, ivs);

    // Extract root AABB and store on root node
    if (store_.next_idx() > 0) {
        float* root_aabb = store_.aabb(0);
        // Store raw (uninflated) AABB — inflation lives in CollisionChecker::obs_flat_
        extract_link_aabbs(root_fk_,
                           store_.active_link_map(), store_.n_active_links(),
                           root_aabb, nullptr);
        store_.set_has_aabb(0, true);
        ++total_fk_calls_;
    }
}

// ─── Core: find_free_box ────────────────────────────────────────────────────
FFBResult HierAABBTree::find_free_box(const Eigen::VectorXd& seed,
                                       const float* obs_flat, int n_obs,
                                       int max_depth, double min_edge) const {
    FFBResult result;
    int n_dims = robot_->n_joints();
    int n_active = store_.n_active_links();
    int n_split_dims = static_cast<int>(split_dims_.size());

    // Stack-allocated FK state (~35KB)
    FKState fk_state;
    std::memcpy(&fk_state, &root_fk_, sizeof(FKState));

    // Current intervals (start at root = full joint limits)
    double ivs_lo[MAX_JOINTS], ivs_hi[MAX_JOINTS];
    for (int d = 0; d < n_dims; ++d) {
        ivs_lo[d] = root_intervals_.limits[d].lo;
        ivs_hi[d] = root_intervals_.limits[d].hi;
    }

    // Temporary AABB buffers
    std::vector<float> aabb_left(n_active * 6);
    std::vector<float> aabb_right(n_active * 6);
    std::vector<float> aabb_union_buf(n_active * 6);

    int idx = 0;  // start at root
    result.n_new_nodes = 0;
    result.n_fk_calls = 0;

    while (true) {
        // 4a. Occupied check
        if (store_.occupied[idx]) {
            result.fail_code = 1;
            break;
        }

        result.path.push_back(idx);
        char* node = store_.node_ptr(idx);

        // 4c. Safety check: if AABB exists and no collision and no subtree occupied
        if (store_.get_u8(node, node_layout::OFF_HAS_AABB)) {
            bool collide = link_aabbs_collide_flat(store_.aabb_ptr(node),
                                                    obs_flat, n_obs);
            if (!collide && store_.subtree_occ[idx] == 0) {
                // Success! This node represents a collision-free box
                result.node_idx = idx;
                result.fail_code = 0;
                break;
            }
        }

        // 4d. Depth / edge length limit
        int depth = store_.get_i32(node, node_layout::OFF_DEPTH);
        if (depth >= max_depth) {
            result.fail_code = 2;
            break;
        }

          int dim = split_dims_[depth % n_split_dims];
          double edge = ivs_hi[dim] - ivs_lo[dim];
        if (edge < min_edge * 2) {
            result.fail_code = 3;
            break;
        }

        // 4e. Get left child
        int left_idx = store_.get_i32(node, node_layout::OFF_LEFT);

        if (left_idx == -1) {
            // 4f. Leaf node — lazy split
            double mid = 0.5 * (ivs_lo[dim] + ivs_hi[dim]);

            // Allocate two children
            int li = store_.alloc_node(idx, depth + 1);
            int ri = store_.alloc_node(idx, depth + 1);

            // Re-fetch node ptr after potential realloc in alloc_node()
            node = store_.node_ptr(idx);

            store_.set_i32(node, node_layout::OFF_LEFT, li);
            store_.set_i32(node, node_layout::OFF_RIGHT, ri);
            store_.set_f64(node, node_layout::OFF_SPLIT, mid);
            result.n_new_nodes += 2;

            bool going_left = seed[dim] < mid;

            // === Fix: Save parent FK state BEFORE computing either child ===
            FKState parent_fk;
            std::memcpy(&parent_fk, &fk_state, sizeof(FKState));

            // Left child FK: incremental from parent
            {
                double save_hi = ivs_hi[dim];
                ivs_hi[dim] = mid;
                std::vector<Interval> civs(n_dims);
                for (int d = 0; d < n_dims; ++d)
                    civs[d] = {ivs_lo[d], ivs_hi[d]};
                FKState left_fk = compute_fk_incremental(parent_fk, *robot_, civs, dim);
                // Store raw (uninflated) AABB
                extract_link_aabbs(left_fk, store_.active_link_map(),
                                   n_active, aabb_left.data(), nullptr);
                std::memcpy(store_.aabb(li), aabb_left.data(), n_active * 6 * sizeof(float));
                store_.set_has_aabb(li, true);
                result.n_fk_calls++;

                if (going_left) {
                    std::memcpy(&fk_state, &left_fk, sizeof(FKState));
                }
                ivs_hi[dim] = save_hi;
            }

            // Right child FK: incremental from SAVED parent (not from left!)
            {
                double save_lo = ivs_lo[dim];
                ivs_lo[dim] = mid;
                std::vector<Interval> civs(n_dims);
                for (int d = 0; d < n_dims; ++d)
                    civs[d] = {ivs_lo[d], ivs_hi[d]};
                FKState right_fk = compute_fk_incremental(parent_fk, *robot_, civs, dim);
                // Store raw (uninflated) AABB
                extract_link_aabbs(right_fk, store_.active_link_map(),
                                   n_active, aabb_right.data(), nullptr);
                std::memcpy(store_.aabb(ri), aabb_right.data(), n_active * 6 * sizeof(float));
                store_.set_has_aabb(ri, true);
                result.n_fk_calls++;

                if (!going_left) {
                    std::memcpy(&fk_state, &right_fk, sizeof(FKState));
                }
                ivs_lo[dim] = save_lo;
            }

            // Refine parent AABB: intersect(current, union(left, right))
            store_.union_aabb(aabb_left.data(), aabb_right.data(), aabb_union_buf.data());
            if (store_.get_u8(node, node_layout::OFF_HAS_AABB)) {
                store_.refine_aabb(store_.aabb(idx), aabb_union_buf.data());
            } else {
                std::memcpy(store_.aabb(idx), aabb_union_buf.data(),
                           n_active * 6 * sizeof(float));
                store_.set_has_aabb(idx, true);
            }

            // Descend
            if (going_left) {
                ivs_hi[dim] = mid;
                idx = li;
            } else {
                ivs_lo[dim] = mid;
                idx = ri;
            }
        } else {
            // 4g. Already split — descend
            double sv = store_.get_f64(node, node_layout::OFF_SPLIT);
            int right_idx = store_.get_i32(node, node_layout::OFF_RIGHT);

            if (seed[dim] < sv) {
                ivs_hi[dim] = sv;
                idx = left_idx;
            } else {
                ivs_lo[dim] = sv;
                idx = right_idx;
            }

            // Incremental FK for new interval
            std::vector<Interval> civs(n_dims);
            for (int d = 0; d < n_dims; ++d)
                civs[d] = {ivs_lo[d], ivs_hi[d]};
            fk_state = compute_fk_incremental(fk_state, *robot_, civs, dim);
            result.n_fk_calls++;
        }
    }

    // === Fix: propagate_up after descent (v4 logic) ===
    if (result.success()) {
        int parent_idx = store_.parent(result.node_idx);
        if (parent_idx >= 0) {
            propagate_up(parent_idx);
        }
    }

    total_fk_calls_ += result.n_fk_calls;
    return result;
}

// ─── Propagate AABB refinement upward ───────────────────────────────────────
void HierAABBTree::propagate_up(int idx) const {
    int n_active = store_.n_active_links();
    std::vector<float> union_buf(n_active * 6);

    while (idx >= 0) {
        int left_idx = store_.left(idx);
        if (left_idx < 0) break;  // leaf, nothing to propagate
        int right_idx = store_.right(idx);
        if (right_idx < 0) break;

        // Both children must have AABBs
        if (!store_.has_aabb(left_idx) || !store_.has_aabb(right_idx)) break;

        // union = union(left, right)
        store_.union_aabb(store_.aabb(left_idx), store_.aabb(right_idx), union_buf.data());

        // Refine: current = intersect(current, union)
        if (store_.has_aabb(idx)) {
            float* cur = store_.aabb(idx);
            float old_lo0 = cur[0], old_hi3 = cur[3];
            store_.refine_aabb(cur, union_buf.data());
            // Early stop: if nothing changed, ancestors won't change either
            if (std::abs(cur[0] - old_lo0) < 1e-12f &&
                std::abs(cur[3] - old_hi3) < 1e-12f) {
                break;
            }
        } else {
            std::memcpy(store_.aabb(idx), union_buf.data(), n_active * 6 * sizeof(float));
            store_.set_has_aabb(idx, true);
        }

        idx = store_.parent(idx);
    }
}

// ─── Promotion collide check (recursive subtree) ────────────────────────────
bool HierAABBTree::promotion_collide_check(int idx, const float* obs_flat,
                                             int n_obs, int remaining_depth) const {
    if (remaining_depth <= 0) {
        if (!store_.has_aabb(idx)) return true;  // conservative: no AABB → assume collision
        return link_aabbs_collide_flat(store_.aabb(idx), obs_flat, n_obs);
    }

    int left_idx = store_.left(idx);
    if (left_idx < 0) {
        // Leaf: fall back to this node's AABB
        if (!store_.has_aabb(idx)) return true;
        return link_aabbs_collide_flat(store_.aabb(idx), obs_flat, n_obs);
    }

    int right_idx = store_.right(idx);

    // Children must have AABBs
    if (!store_.has_aabb(left_idx) || !store_.has_aabb(right_idx)) {
        if (!store_.has_aabb(idx)) return true;
        return link_aabbs_collide_flat(store_.aabb(idx), obs_flat, n_obs);
    }

    // Both children must be collision-free for parent to be collision-free
    if (promotion_collide_check(left_idx, obs_flat, n_obs, remaining_depth - 1))
        return true;
    if (promotion_collide_check(right_idx, obs_flat, n_obs, remaining_depth - 1))
        return true;
    return false;
}

// ─── Collect forest_box_ids from occupied subtree ───────────────────────────
void HierAABBTree::collect_forest_ids(int idx, std::vector<int>& out) const {
    if (store_.occupied[idx]) {
        int fid = store_.forest_id[idx];
        if (fid >= 0)
            out.push_back(fid);
    }
    int left_idx = store_.left(idx);
    if (left_idx >= 0) {
        collect_forest_ids(left_idx, out);
        int right_idx = store_.right(idx);
        if (right_idx >= 0)
            collect_forest_ids(right_idx, out);
    }
}

// ─── Clear subtree occupation (matches Python v4 _clear_subtree_occupation_c) ──
void HierAABBTree::clear_subtree_occupation(int idx) const {
    // DFS iteration: for each occupied node, clear and propagate decrements up
    std::vector<int> stack;
    stack.push_back(idx);

    while (!stack.empty()) {
        int i = stack.back();
        stack.pop_back();

        if (store_.occupied[i]) {
            store_.occupied[i] = 0;
            store_.forest_id[i] = -1;

            // Compute volume fraction for this node
            int depth = store_.depth(i);
            double vol = std::ldexp(1.0, -depth);

            // Walk up from PARENT, decrementing subtree_occ / subtree_occ_vol
            int pidx = store_.parent(i);
            while (pidx >= 0) {
                store_.subtree_occ[pidx]--;
                store_.subtree_occ_vol[pidx] -= vol;
                pidx = store_.parent(pidx);
            }

            // Clear the node's own subtree counters
            store_.subtree_occ[i] = 0;
            store_.subtree_occ_vol[i] = 0.0;
        }

        int left_idx = store_.left(i);
        if (left_idx >= 0) {
            stack.push_back(left_idx);
            int right_idx = store_.right(i);
            if (right_idx >= 0)
                stack.push_back(right_idx);
        }
    }
}

// ─── Promotion (v4 logic: walk up path, absorb occupied subtrees) ───────────
HierAABBTree::PromotionResult HierAABBTree::try_promote(
    const std::vector<int>& path,
    const float* obs_flat, int n_obs,
    int promotion_depth) const
{
    PromotionResult pr;
    if (path.empty()) {
        pr.result_idx = -1;
        return pr;
    }
    pr.result_idx = path.back();

    // Walk backward along path (skip the found node itself)
    for (int i = static_cast<int>(path.size()) - 2; i >= 0; --i) {
        int pidx = path[i];
        if (!store_.has_aabb(pidx)) break;

        if (store_.subtree_occ[pidx] > 0) {
            // Parent has occupied descendants —
            // check if AABB is still collision-free (with promotion_depth)
            if (promotion_collide_check(pidx, obs_flat, n_obs, promotion_depth))
                break;  // collision — stop promotion

            // Absorb: collect all forest_ids in subtree and clear occupation
            collect_forest_ids(pidx, pr.absorbed_box_ids);
            clear_subtree_occupation(pidx);
            pr.result_idx = pidx;
        } else {
            // No occupied descendants — simple check
            if (promotion_collide_check(pidx, obs_flat, n_obs, promotion_depth))
                break;
            pr.result_idx = pidx;
        }
    }
    return pr;
}

// ─── Legacy simple promotion ────────────────────────────────────────────────
int HierAABBTree::try_promote_simple(int node_idx, const float* obs_flat, int n_obs) const {
    int idx = node_idx;

    while (true) {
        int parent_idx = store_.parent(idx);
        if (parent_idx < 0) break;  // reached root

        // Check if parent's AABB is collision-free
        if (!store_.has_aabb(parent_idx)) break;
        if (store_.subtree_occ[parent_idx] > 0) break;

        bool collide = link_aabbs_collide_flat(store_.aabb(parent_idx),
                                                obs_flat, n_obs);
        if (collide) break;

        idx = parent_idx;
    }
    return idx;
}

// ─── Mark occupied ──────────────────────────────────────────────────────────
void HierAABBTree::mark_occupied(int node_idx, int forest_box_id) {
    store_.occupied[node_idx] = 1;
    store_.forest_id[node_idx] = forest_box_id;

    // Propagate subtree_occ up to root
    int idx = node_idx;
    while (idx >= 0) {
        store_.subtree_occ[idx]++;
        int depth = store_.depth(idx);
        store_.subtree_occ_vol[idx] += std::ldexp(1.0, -depth);
        idx = store_.parent(idx);
    }
}

void HierAABBTree::unmark_occupied(int node_idx) {
    store_.occupied[node_idx] = 0;
    int forest_box_id = store_.forest_id[node_idx];
    store_.forest_id[node_idx] = -1;

    int idx = node_idx;
    while (idx >= 0) {
        store_.subtree_occ[idx]--;
        int depth = store_.depth(idx);
        store_.subtree_occ_vol[idx] -= std::ldexp(1.0, -depth);
        idx = store_.parent(idx);
    }
    (void)forest_box_id;
}

// ─── Occupancy queries ──────────────────────────────────────────────────────
int HierAABBTree::find_containing_box_id(const Eigen::VectorXd& config) const {
    int n_split_dims = static_cast<int>(split_dims_.size());
    int idx = 0;

    while (true) {
        if (store_.occupied[idx]) {
            int fid = store_.forest_id[idx];
            return (fid >= 0) ? fid : -1;
        }

        int left_idx = store_.left(idx);
        if (left_idx < 0 || store_.subtree_occ[idx] == 0) {
            return -1;  // leaf or no occupied descendants
        }

        int depth_val = store_.depth(idx);
        int dim = split_dims_[depth_val % n_split_dims];
        double sv = store_.split_val(idx);

        if (config[dim] < sv) {
            idx = left_idx;
        } else {
            idx = store_.right(idx);
        }
    }
}

// ─── Guided sampling: sample from unoccupied space ──────────────────────────
bool HierAABBTree::sample_unoccupied_seed(std::mt19937& rng,
                                            Eigen::VectorXd& out,
                                            int max_walk_depth) const {
    int n_dims = robot_->n_joints();
    int n_split_dims = static_cast<int>(split_dims_.size());
    int idx = 0;

    // Root fully occupied
    if (store_.occupied[idx]) return false;

    // Running intervals — start at root
    std::vector<double> ivs_lo(n_dims), ivs_hi(n_dims);
    for (int d = 0; d < n_dims; ++d) {
        ivs_lo[d] = root_intervals_.limits[d].lo;
        ivs_hi[d] = root_intervals_.limits[d].hi;
    }

    std::uniform_real_distribution<double> unif(0.0, 1.0);

    for (int step = 0; step < max_walk_depth; ++step) {
        int left_idx = store_.left(idx);

        // Leaf node — sample here
        if (left_idx < 0) break;

        int right_idx = store_.right(idx);
        int child_depth = store_.depth(idx) + 1;

        // Child volume fraction = 2^(-child_depth)
        double child_vol = std::ldexp(1.0, -child_depth);

        // Free volume = total - occupied
        double occ_vol_l = store_.subtree_occ_vol[left_idx];
        double occ_vol_r = store_.subtree_occ_vol[right_idx];
        double w_l = std::max(0.0, child_vol - occ_vol_l);
        double w_r = std::max(0.0, child_vol - occ_vol_r);

        // If one side is an occupied leaf, force weight to zero
        if (store_.occupied[left_idx] && store_.left(left_idx) < 0)
            w_l = 0.0;
        if (store_.occupied[right_idx] && store_.left(right_idx) < 0)
            w_r = 0.0;

        if (w_l + w_r <= 0.0) break;  // both saturated

        int dim = split_dims_[store_.depth(idx) % n_split_dims];
        double sv = store_.split_val(idx);

        bool go_left = (unif(rng) < w_l / (w_l + w_r));

        if (go_left) {
            ivs_hi[dim] = sv;
            idx = left_idx;
        } else {
            ivs_lo[dim] = sv;
            idx = right_idx;
        }

        if (store_.occupied[idx]) return false;
    }

    // Uniform sample in the current interval
    out.resize(n_dims);
    for (int d = 0; d < n_dims; ++d) {
        out[d] = ivs_lo[d] + unif(rng) * (ivs_hi[d] - ivs_lo[d]);
    }
    return true;
}

// ─── Hull safety check for greedy coarsen ───────────────────────────────────
// Check if hull(A,B) \ (A ∪ B) is collision-free by traversing the KD-tree.
//
// Optimizations:
//   1. LCA descent: skip to deepest node containing hull (avoids traversing
//      the entire tree from root — typically saves ~99% of nodes).
//   2. Cached AABB SAT: gap regions use pre-computed link AABBs (no FK).
//   3. Incremental FK for splits: when a leaf needs splitting, compute parent
//      FK once (full), then incremental for children and cascade. Turns
//      2^depth full FKs into 1 full + (2^depth - 1) incremental calls.

bool HierAABBTree::check_hull_safe(const std::vector<Interval>& hull,
                                    const std::vector<Interval>& a_ivs,
                                    const std::vector<Interval>& b_ivs,
                                    const float* obs_flat, int n_obs,
                                    int max_split_depth,
                                    double min_edge) const {
    int n_dims = robot_->n_joints();
    int n_split_dims = static_cast<int>(split_dims_.size());

    // Initialize node intervals to root
    double ivs_lo[MAX_JOINTS], ivs_hi[MAX_JOINTS];
    for (int d = 0; d < n_dims; ++d) {
        ivs_lo[d] = root_intervals_.limits[d].lo;
        ivs_hi[d] = root_intervals_.limits[d].hi;
    }

    // ── LCA descent: find deepest node whose intervals contain hull ─────
    int idx = 0;
    while (true) {
        int left_idx = store_.left(idx);
        if (left_idx < 0) break;  // leaf — can't descend further

        int depth_val = store_.depth(idx);
        int dim = split_dims_[depth_val % n_split_dims];
        double sv = store_.split_val(idx);

        if (hull[dim].hi <= sv + 1e-12) {
            // hull fits entirely in left child
            ivs_hi[dim] = sv;
            idx = left_idx;
        } else if (hull[dim].lo >= sv - 1e-12) {
            // hull fits entirely in right child
            ivs_lo[dim] = sv;
            idx = store_.right(idx);
        } else {
            break;  // hull spans both children → this is the LCA
        }
    }

    return hull_safe_recurse(idx, ivs_lo, ivs_hi, hull, a_ivs, b_ivs,
                              obs_flat, n_obs, max_split_depth, min_edge);
}

bool HierAABBTree::hull_safe_recurse(int idx, double* ivs_lo, double* ivs_hi,
                                      const std::vector<Interval>& hull,
                                      const std::vector<Interval>& a_ivs,
                                      const std::vector<Interval>& b_ivs,
                                      const float* obs_flat, int n_obs,
                                      int remaining_splits,
                                      double min_edge) const {
    int n_dims = robot_->n_joints();
    int n_active = store_.n_active_links();
    int n_split_dims = static_cast<int>(split_dims_.size());

    // Step 1: Check if node intersects hull at all
    for (int d = 0; d < n_dims; ++d) {
        if (ivs_lo[d] >= hull[d].hi - 1e-12 || ivs_hi[d] <= hull[d].lo + 1e-12)
            return true;  // disjoint → irrelevant
    }

    // Step 2: Check if (node ∩ hull) ⊆ A or ⊆ B
    {
        bool in_a = true, in_b = true;
        for (int d = 0; d < n_dims; ++d) {
            double lo = std::max(ivs_lo[d], hull[d].lo);
            double hi = std::min(ivs_hi[d], hull[d].hi);
            if (in_a && (lo < a_ivs[d].lo - 1e-12 || hi > a_ivs[d].hi + 1e-12))
                in_a = false;
            if (in_b && (lo < b_ivs[d].lo - 1e-12 || hi > b_ivs[d].hi + 1e-12))
                in_b = false;
            if (!in_a && !in_b) break;
        }
        if (in_a || in_b) return true;  // covered by known-safe box
    }

    // Step 3: If node has cached AABB, do SAT test (no FK needed!)
    if (store_.has_aabb(idx)) {
        bool collide = link_aabbs_collide_flat(store_.aabb(idx), obs_flat, n_obs);
        if (!collide) return true;  // cached AABB says safe → done
    }

    // Step 4: If internal node → recurse children (no FK hint for existing nodes)
    int left_idx = store_.left(idx);
    if (left_idx >= 0) {
        int right_idx = store_.right(idx);
        int depth_val = store_.depth(idx);
        int dim = split_dims_[depth_val % n_split_dims];
        double sv = store_.split_val(idx);

        // Left child: ivs[dim].hi = sv
        double save = ivs_hi[dim];
        ivs_hi[dim] = sv;
        if (!hull_safe_recurse(left_idx, ivs_lo, ivs_hi, hull, a_ivs, b_ivs,
                                obs_flat, n_obs, remaining_splits, min_edge))
            { ivs_hi[dim] = save; return false; }
        ivs_hi[dim] = save;

        // Right child: ivs[dim].lo = sv
        save = ivs_lo[dim];
        ivs_lo[dim] = sv;
        if (!hull_safe_recurse(right_idx, ivs_lo, ivs_hi, hull, a_ivs, b_ivs,
                                obs_flat, n_obs, remaining_splits, min_edge))
            { ivs_lo[dim] = save; return false; }
        ivs_lo[dim] = save;

        return true;
    }

    // Step 5: Leaf node with AABB collision (or no AABB).
    // Try lazy split if we have budget and edge is wide enough.
    if (remaining_splits <= 0) return false;  // no budget → conservative collision

    int depth_val = store_.depth(idx);
    int dim = split_dims_[depth_val % n_split_dims];
    double edge = ivs_hi[dim] - ivs_lo[dim];
    if (edge < min_edge * 2) return false;  // too narrow to split

    // Lazy split: create two children, only compute FK for children
    // that actually need checking (skip disjoint or A/B-contained children).
    double mid = 0.5 * (ivs_lo[dim] + ivs_hi[dim]);

    int li = store_.alloc_node(idx, depth_val + 1);
    int ri = store_.alloc_node(idx, depth_val + 1);

    // Re-fetch node ptr after potential realloc
    char* node = store_.node_ptr(idx);
    store_.set_i32(node, node_layout::OFF_LEFT, li);
    store_.set_i32(node, node_layout::OFF_RIGHT, ri);
    store_.set_f64(node, node_layout::OFF_SPLIT, mid);

    // Helper: check if child needs FK+SAT (returns true if it does)
    auto child_needs_check = [&](bool is_left) -> bool {
        double clo[MAX_JOINTS], chi[MAX_JOINTS];
        for (int d = 0; d < n_dims; ++d) {
            clo[d] = ivs_lo[d]; chi[d] = ivs_hi[d];
        }
        if (is_left) chi[dim] = mid; else clo[dim] = mid;

        // Disjoint with hull?
        for (int d = 0; d < n_dims; ++d) {
            if (clo[d] >= hull[d].hi - 1e-12 || chi[d] <= hull[d].lo + 1e-12)
                return false;
        }
        // (child ∩ hull) ⊆ A or ⊆ B?
        bool in_a = true, in_b = true;
        for (int d = 0; d < n_dims; ++d) {
            double lo = std::max(clo[d], hull[d].lo);
            double hi = std::min(chi[d], hull[d].hi);
            if (in_a && (lo < a_ivs[d].lo - 1e-12 || hi > a_ivs[d].hi + 1e-12))
                in_a = false;
            if (in_b && (lo < b_ivs[d].lo - 1e-12 || hi > b_ivs[d].hi + 1e-12))
                in_b = false;
            if (!in_a && !in_b) break;
        }
        return !in_a && !in_b;  // needs check only if not contained
    };

    bool left_needs = child_needs_check(true);
    bool right_needs = child_needs_check(false);

    // Fast path: neither child needs checking
    if (!left_needs && !right_needs) return true;

    // Process left child (compute FK only if needed)
    if (left_needs) {
        double save = ivs_hi[dim];
        ivs_hi[dim] = mid;
        std::vector<Interval> civs(n_dims);
        for (int d = 0; d < n_dims; ++d)
            civs[d] = {ivs_lo[d], ivs_hi[d]};
        FKState fk = compute_fk_full(*robot_, civs);
        extract_link_aabbs(fk, store_.active_link_map(), n_active,
                           store_.aabb(li), nullptr);
        store_.set_has_aabb(li, true);
        ++total_fk_calls_;
        if (!hull_safe_recurse(li, ivs_lo, ivs_hi, hull, a_ivs, b_ivs,
                                obs_flat, n_obs, remaining_splits - 1, min_edge))
            { ivs_hi[dim] = save; return false; }  // early return!
        ivs_hi[dim] = save;
    }

    // Process right child (only reached if left succeeded)
    if (right_needs) {
        double save = ivs_lo[dim];
        ivs_lo[dim] = mid;
        std::vector<Interval> civs(n_dims);
        for (int d = 0; d < n_dims; ++d)
            civs[d] = {ivs_lo[d], ivs_hi[d]};
        FKState fk = compute_fk_full(*robot_, civs);
        extract_link_aabbs(fk, store_.active_link_map(), n_active,
                           store_.aabb(ri), nullptr);
        store_.set_has_aabb(ri, true);
        ++total_fk_calls_;
        if (!hull_safe_recurse(ri, ivs_lo, ivs_hi, hull, a_ivs, b_ivs,
                                obs_flat, n_obs, remaining_splits - 1, min_edge))
            { ivs_lo[dim] = save; return false; }
        ivs_lo[dim] = save;
    }

    // Refine parent AABB from children (if both have AABBs)
    if (store_.has_aabb(li) && store_.has_aabb(ri)) {
        std::vector<float> union_buf(n_active * 6);
        store_.union_aabb(store_.aabb(li), store_.aabb(ri), union_buf.data());
        if (store_.has_aabb(idx)) {
            store_.refine_aabb(store_.aabb(idx), union_buf.data());
        } else {
            std::memcpy(store_.aabb(idx), union_buf.data(),
                       n_active * 6 * sizeof(float));
            store_.set_has_aabb(idx, true);
        }
        int parent_idx = store_.parent(idx);
        if (parent_idx >= 0) propagate_up(parent_idx);
    }

    return true;
}

// ─── Get node intervals ─────────────────────────────────────────────────────
std::vector<Interval> HierAABBTree::get_node_intervals(int node_idx) const {
    int n_dims = robot_->n_joints();
    (void)n_dims;
    std::vector<Interval> ivs = root_intervals_.limits;

    // Trace path from root to this node
    // Build path by going up from node to root, then reverse
    std::vector<int> path_to_root;
    int idx = node_idx;
    while (idx > 0) {
        path_to_root.push_back(idx);
        idx = store_.parent(idx);
    }

    // Apply splits from root downward
    for (int i = static_cast<int>(path_to_root.size()) - 1; i >= 0; --i) {
        int child_idx = path_to_root[i];
        int par_idx = store_.parent(child_idx);
        int par_depth = store_.depth(par_idx);
        int dim = split_dims_[par_depth % static_cast<int>(split_dims_.size())];
        double sv = store_.split_val(par_idx);

        if (child_idx == store_.left(par_idx)) {
            ivs[dim].hi = sv;
        } else {
            ivs[dim].lo = sv;
        }
    }

    return ivs;
}

// ─── Persistence (HCACHE02 file I/O) ────────────────────────────────────────

void HierAABBTree::write_header_to(char* header, int n_alloc_override) const {
    int n_dims = robot_->n_joints();
    int n_links = robot_->n_active_links();
    int stride = store_.stride();
    int n_nodes = store_.next_idx();
    int n_alloc = (n_alloc_override >= 0) ? n_alloc_override : n_nodes;

    std::memset(header, 0, hcache::HEADER_SIZE);
    std::memcpy(header, hcache::MAGIC, 8);
    int32_t ver = hcache::VERSION;
    std::memcpy(header + 8, &ver, 4);
    int64_t nn = n_nodes;
    std::memcpy(header + 12, &nn, 8);
    int64_t na = n_alloc;
    std::memcpy(header + 20, &na, 8);
    int32_t nd = n_dims;
    std::memcpy(header + 28, &nd, 4);
    int32_t nl = n_links;
    std::memcpy(header + 32, &nl, 4);
    int64_t nfk = total_fk_calls_;
    std::memcpy(header + 36, &nfk, 8);
    int32_t st = stride;
    std::memcpy(header + 44, &st, 4);

    std::string fp = robot_->fingerprint();
    size_t fp_len = std::min(fp.size(), size_t(32));
    if (fp_len > 0)
        std::memcpy(header + 48, fp.data(), fp_len);

    for (int d = 0; d < n_dims && d < MAX_JOINTS; ++d) {
        double lo = root_intervals_.limits[d].lo;
        double hi = root_intervals_.limits[d].hi;
        std::memcpy(header + 80 + d * 16,     &lo, 8);
        std::memcpy(header + 80 + d * 16 + 8, &hi, 8);
    }
}

void HierAABBTree::save(const std::string& path) const {
    int stride = store_.stride();
    int n_nodes = store_.next_idx();

    std::vector<char> header(hcache::HEADER_SIZE);
    write_header_to(header.data());

    std::ofstream out(path, std::ios::binary);
    if (!out)
        throw std::runtime_error("HierAABBTree::save: cannot open " + path);

    out.write(header.data(), hcache::HEADER_SIZE);
    out.write(store_.raw_buffer(), static_cast<std::streamsize>(
        static_cast<size_t>(n_nodes) * stride));
    if (!out)
        throw std::runtime_error("HierAABBTree::save: write failed to " + path);

    // Update source tracking for subsequent incremental saves
    const_cast<HierAABBTree*>(this)->source_path_ = path;
    const_cast<HierAABBTree*>(this)->source_n_alloc_ = n_nodes;
    const_cast<HierAABBTree*>(this)->store_.clear_all_dirty();
}

void HierAABBTree::save_incremental(const std::string& path) {
    if (source_path_.empty() || source_path_ != path)
        throw std::runtime_error(
            "HierAABBTree::save_incremental: no prior save to '" + path +
            "'. Call save() first.");

    int stride = store_.stride();
    int n_nodes = store_.next_idx();
    int old_n = source_n_alloc_;

    // Open existing file for in-place update
    std::fstream f(path, std::ios::in | std::ios::out | std::ios::binary);
    if (!f)
        throw std::runtime_error("HierAABBTree::save_incremental: cannot open " + path);

    // Update header (write full header)
    std::vector<char> header(hcache::HEADER_SIZE);
    write_header_to(header.data(), n_nodes);
    f.seekp(0);
    f.write(header.data(), hcache::HEADER_SIZE);

    // Append new nodes (beyond old file boundary)
    if (n_nodes > old_n) {
        f.seekp(hcache::HEADER_SIZE + static_cast<std::streamoff>(old_n) * stride);
        f.write(store_.raw_buffer() + static_cast<size_t>(old_n) * stride,
                static_cast<std::streamsize>(
                    static_cast<size_t>(n_nodes - old_n) * stride));
    }

    // Write back dirty old nodes (selective seek+write)
    auto dirty_indices = store_.iter_dirty();
    for (int idx : dirty_indices) {
        if (idx < old_n) {
            auto off = static_cast<std::streamoff>(idx) * stride;
            f.seekp(hcache::HEADER_SIZE + off);
            f.write(store_.raw_buffer() + static_cast<size_t>(idx) * stride, stride);
        }
    }

    if (!f)
        throw std::runtime_error("HierAABBTree::save_incremental: write failed to " + path);

    store_.clear_all_dirty();
    source_path_ = path;
    source_n_alloc_ = n_nodes;
}

HierAABBTree HierAABBTree::load(const std::string& path, const Robot& robot) {
    std::ifstream in(path, std::ios::binary);
    if (!in)
        throw std::runtime_error("HierAABBTree::load: cannot open " + path);

    std::vector<char> header(hcache::HEADER_SIZE);
    in.read(header.data(), hcache::HEADER_SIZE);
    if (!in)
        throw std::runtime_error("HierAABBTree::load: cannot read header from " + path);

    if (std::memcmp(header.data(), hcache::MAGIC, 8) != 0)
        throw std::runtime_error("HierAABBTree::load: invalid magic in " + path);

    int32_t ver;
    std::memcpy(&ver, header.data() + 8, 4);
    if (ver != hcache::VERSION)
        throw std::runtime_error("HierAABBTree::load: unsupported version in " + path);

    int64_t n_nodes, n_alloc;
    int32_t n_dims, n_links, stride;
    int64_t n_fk_calls;
    std::memcpy(&n_nodes,    header.data() + 12, 8);
    std::memcpy(&n_alloc,    header.data() + 20, 8);
    std::memcpy(&n_dims,     header.data() + 28, 4);
    std::memcpy(&n_links,    header.data() + 32, 4);
    std::memcpy(&n_fk_calls, header.data() + 36, 8);
    std::memcpy(&stride,     header.data() + 44, 4);

    if (n_dims != robot.n_joints())
        throw std::runtime_error("HierAABBTree::load: n_dims mismatch ("
            + std::to_string(n_dims) + " vs robot " + std::to_string(robot.n_joints()) + ")");
    if (n_links != robot.n_active_links())
        throw std::runtime_error("HierAABBTree::load: n_links mismatch ("
            + std::to_string(n_links) + " vs robot " + std::to_string(robot.n_active_links()) + ")");

    int expected_stride = node_layout::compute_stride(n_links);
    if (stride != expected_stride)
        throw std::runtime_error("HierAABBTree::load: stride mismatch");

    int cap = static_cast<int>(std::max(n_nodes, n_alloc));
    NodeStore store(n_links, n_dims, cap);

    size_t data_bytes = static_cast<size_t>(n_nodes) * stride;
    in.read(store.base(), static_cast<std::streamsize>(data_bytes));
    if (!in)
        throw std::runtime_error("HierAABBTree::load: cannot read node data from " + path);

    store.set_next_idx(static_cast<int>(n_nodes));

    HierAABBTree tree(robot, std::move(store));
    tree.total_fk_calls_ = static_cast<int>(n_fk_calls);
    tree.source_path_ = path;
    tree.source_n_alloc_ = static_cast<int>(n_nodes);
    return tree;
}

// ─── mmap-backed lazy load ──────────────────────────────────────────────────

} // namespace sbf (temporarily close for system includes)

#if defined(__linux__) || defined(__APPLE__)
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#endif

namespace sbf { // reopen

void HierAABBTree::grow_mmap(int new_cap) {
#if defined(__linux__) || defined(__APPLE__)
    if (!mmap_ptr_ || mmap_fd_ < 0) return;

    int stride = store_.stride();
    size_t new_size = hcache::HEADER_SIZE + static_cast<size_t>(new_cap) * stride;

    // Unmap old mapping
    munmap(mmap_ptr_, mmap_size_);

    // Extend file
    if (ftruncate(mmap_fd_, static_cast<off_t>(new_size)) < 0)
        throw std::runtime_error("HierAABBTree::grow_mmap: ftruncate failed");

    // Re-map
    mmap_ptr_ = static_cast<char*>(
        mmap(nullptr, new_size, PROT_READ | PROT_WRITE, MAP_SHARED, mmap_fd_, 0));
    if (mmap_ptr_ == MAP_FAILED) {
        mmap_ptr_ = nullptr;
        throw std::runtime_error("HierAABBTree::grow_mmap: mmap failed");
    }
    mmap_size_ = new_size;

    // Update header n_alloc
    int64_t na64 = new_cap;
    std::memcpy(mmap_ptr_ + 20, &na64, 8);

    // Re-attach NodeStore to new mapping
    store_.attach_buffer(mmap_ptr_ + hcache::HEADER_SIZE, new_cap);
#else
    (void)new_cap;
    throw std::runtime_error("mmap not supported on this platform");
#endif
}

HierAABBTree HierAABBTree::load_mmap(const std::string& path, const Robot& robot) {
#if defined(__linux__) || defined(__APPLE__)
    int fd = ::open(path.c_str(), O_RDWR);
    if (fd < 0)
        throw std::runtime_error("HierAABBTree::load_mmap: cannot open " + path);

    struct stat st;
    if (fstat(fd, &st) < 0) { ::close(fd); throw std::runtime_error("fstat failed"); }
    size_t file_size = static_cast<size_t>(st.st_size);

    if (file_size < static_cast<size_t>(hcache::HEADER_SIZE)) {
        ::close(fd);
        throw std::runtime_error("HierAABBTree::load_mmap: file too small");
    }

    char* mapped = static_cast<char*>(
        mmap(nullptr, file_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
    if (mapped == MAP_FAILED) {
        ::close(fd);
        throw std::runtime_error("HierAABBTree::load_mmap: mmap failed");
    }

    // Read header
    if (std::memcmp(mapped, hcache::MAGIC, 8) != 0) {
        munmap(mapped, file_size); ::close(fd);
        throw std::runtime_error("HierAABBTree::load_mmap: invalid magic");
    }
    int32_t ver;
    std::memcpy(&ver, mapped + 8, 4);
    if (ver != hcache::VERSION) {
        munmap(mapped, file_size); ::close(fd);
        throw std::runtime_error("HierAABBTree::load_mmap: unsupported version");
    }

    int64_t n_nodes, n_alloc;
    int32_t n_dims, n_links, stride;
    int64_t n_fk_calls;
    std::memcpy(&n_nodes,    mapped + 12, 8);
    std::memcpy(&n_alloc,    mapped + 20, 8);
    std::memcpy(&n_dims,     mapped + 28, 4);
    std::memcpy(&n_links,    mapped + 32, 4);
    std::memcpy(&n_fk_calls, mapped + 36, 8);
    std::memcpy(&stride,     mapped + 44, 4);

    if (n_dims != robot.n_joints()) {
        munmap(mapped, file_size); ::close(fd);
        throw std::runtime_error("HierAABBTree::load_mmap: n_dims mismatch");
    }
    if (n_links != robot.n_active_links()) {
        munmap(mapped, file_size); ::close(fd);
        throw std::runtime_error("HierAABBTree::load_mmap: n_links mismatch");
    }

    // Create NodeStore backed by mmap data region
    char* data_base = mapped + hcache::HEADER_SIZE;
    NodeStore store(data_base, n_links, n_dims,
                    static_cast<int>(n_alloc),
                    static_cast<int>(n_nodes));

    // Construct tree
    HierAABBTree tree(robot, std::move(store));
    tree.total_fk_calls_ = static_cast<int>(n_fk_calls);
    tree.source_path_ = path;
    tree.source_n_alloc_ = static_cast<int>(n_nodes);
    tree.mmap_fd_ = fd;
    tree.mmap_ptr_ = mapped;
    tree.mmap_size_ = file_size;

    // Hook resize callback — captures `this` which will be fixed by move ctor
    tree.store_.set_resize_callback([&tree](int /*old*/, int new_cap) {
        tree.grow_mmap(new_cap);
    });

    // NOTE: move constructor/NRVO will re-hook the callback to the final location
    return tree;
#else
    (void)path; (void)robot;
    throw std::runtime_error("mmap not supported on this platform");
#endif
}

void HierAABBTree::flush_mmap() {
#if defined(__linux__) || defined(__APPLE__)
    if (!mmap_ptr_) return;

    // Update header fields in-place
    int64_t nn = store_.next_idx();
    std::memcpy(mmap_ptr_ + 12, &nn, 8);
    int64_t nfk = total_fk_calls_;
    std::memcpy(mmap_ptr_ + 36, &nfk, 8);

    msync(mmap_ptr_, mmap_size_, MS_SYNC);
#endif
}

void HierAABBTree::close_mmap() {
#if defined(__linux__) || defined(__APPLE__)
    if (mmap_ptr_) {
        flush_mmap();
        munmap(mmap_ptr_, mmap_size_);
        mmap_ptr_ = nullptr;
    }
    if (mmap_fd_ >= 0) {
        ::close(mmap_fd_);
        mmap_fd_ = -1;
    }
    mmap_size_ = 0;
#endif
}

} // namespace sbf

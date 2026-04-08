// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v3 — LECT implementation
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/forest/lect.h"
#include "sbf/common/interval_trig.h"
#include "sbf/envelope/frame_source.h"
#include "sbf/envelope/envelope_type.h"
#include "sbf/envelope/envelope_derive.h"
#include "sbf/envelope/envelope_cache.h"
#include "sbf/robot/hybrid_aabb.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace sbf {
namespace forest {

// ─────────────────────────────────────────────────────────────────────────
//  Construction
// ─────────────────────────────────────────────────────────────────────────
LECT::LECT(const Robot& robot, double voxel_delta, int initial_cap)
    : robot_(&robot)
    , voxel_delta_(voxel_delta)
    , store_(robot.n_joints(), robot.n_active_links(), initial_cap)
    , frame_store_(robot, initial_cap)
{
    // Default pipeline: IFK + Hull16_Grid with given voxel_delta
    pipeline_config_ = envelope::PipelineConfig::production();
    pipeline_config_.envelope.delta = voxel_delta;

    root_limits_ = robot.joint_limits();

    // Build split dimension cycling sequence (round-robin over joints)
    split_dims_.resize(robot.n_joints());
    for (int d = 0; d < robot.n_joints(); ++d)
        split_dims_[d] = d;

    // Determine freeze depth from DH params: count leading joints with
    // a_i = 0 and α_i ∈ {0, ±π/2}. Cap at 2 (practical sweet-spot).
    freeze_depth_ = 0;
    for (int d = 0; d < robot.n_joints() && d < 2; ++d) {
        const auto& dh = robot.dh_params()[d];
        int atype = classify_alpha(dh.alpha);
        if (atype < 0 || std::abs(dh.a) > 1e-9) break;
        freeze_depth_ = d + 1;
        frozen_descs_.push_back({atype, dh.d, {0.0, 0.0}});  // q filled at recon time
    }

    ensure_hull_capacity(initial_cap);

    // Initialize root node (index 0)
    init_root();
}

LECT::LECT(const Robot& robot, const envelope::PipelineConfig& pipeline,
           int initial_cap)
    : robot_(&robot)
    , voxel_delta_(pipeline.envelope.delta)
    , pipeline_config_(pipeline)
    , store_(robot.n_joints(), robot.n_active_links(), initial_cap)
    , frame_store_(robot, initial_cap)
{
    root_limits_ = robot.joint_limits();

    split_dims_.resize(robot.n_joints());
    for (int d = 0; d < robot.n_joints(); ++d)
        split_dims_[d] = d;

    // Determine freeze depth from DH params
    freeze_depth_ = 0;
    for (int d = 0; d < robot.n_joints() && d < 2; ++d) {
        const auto& dh = robot.dh_params()[d];
        int atype = classify_alpha(dh.alpha);
        if (atype < 0 || std::abs(dh.a) > 1e-9) break;
        freeze_depth_ = d + 1;
        frozen_descs_.push_back({atype, dh.d, {0.0, 0.0}});
    }

    ensure_hull_capacity(initial_cap);

    init_root();
}

// ═════════════════════════════════════════════════════════════════════════
//  snapshot — deep copy of tree + cached envelopes, cleared occupation
// ═════════════════════════════════════════════════════════════════════════
LECT LECT::snapshot() const {
    LECT copy;
    copy.robot_            = robot_;
    copy.voxel_delta_      = voxel_delta_;
    copy.pipeline_config_  = pipeline_config_;
    copy.store_            = store_.snapshot();      // deep copy, cleared occ
    // frame_store_ left as default (empty) — workers don't need persistence
    copy.ep_store_         = ep_store_;              // deep copy
    copy.hull_grids_       = hull_grids_;            // deep copy
    copy.hull_valid_       = hull_valid_;            // deep copy
    copy.scene_grid_       = scene_grid_;            // deep copy
    copy.scene_set_        = scene_set_;
    copy.root_limits_      = root_limits_;
    copy.root_fk_          = root_fk_;               // large but stack-based
    copy.split_dims_       = split_dims_;
    copy.freeze_depth_     = freeze_depth_;
    copy.frozen_descs_     = frozen_descs_;
    copy.hull_skip_vol_    = hull_skip_vol_;
    return copy;
}

// ═════════════════════════════════════════════════════════════════════════
//  transplant_subtree — merge worker-grown nodes back into coordinator
//
//  The worker was created from a snapshot() of the coordinator LECT.
//  Both share identical node indices for nodes 0..snapshot_n_nodes-1.
//  The worker grew new nodes (index >= snapshot_n_nodes) under its
//  partition cell's LECT subtree.  This method:
//    1. Finds all new nodes in the worker that descend from coordinator_node
//    2. Allocates corresponding nodes in the coordinator
//    3. Copies tree structure (with remapped indices), AABBs, ep_store,
//       hull_grids, hull_valid, occupation state
//    4. Overwrites the coordinator_node's left/right pointers to link
//       the transplanted subtree
// ═════════════════════════════════════════════════════════════════════════
int LECT::transplant_subtree(const LECT& worker,
                             int coordinator_node,
                             int snapshot_n_nodes,
                             const std::unordered_map<int, int>& box_id_remap)
{
    // If worker didn't split this node at all, nothing to transplant
    if (worker.is_leaf(coordinator_node))
        return 0;

    // The coordinator_node should currently be a leaf (pre-expanded to
    // partition depth).  The worker has split it further.
    // We need to collect all nodes in the worker subtree rooted at
    // coordinator_node that are NEW (index >= snapshot_n_nodes).

    // BFS to collect all worker nodes under coordinator_node
    // that are new (plus we need to handle the shared root node itself)
    struct NodeMapping {
        int worker_idx;
        int coord_idx;
    };

    std::unordered_map<int, int> remap;  // worker_idx → coord_idx

    // The coordinator_node already exists in both — same index
    remap[coordinator_node] = coordinator_node;

    // BFS from coordinator_node through worker's tree
    std::vector<int> bfs_queue;
    bfs_queue.push_back(coordinator_node);
    int transplanted = 0;

    // First pass: collect all new nodes and allocate coordinator indices
    std::vector<int> new_worker_nodes;  // worker indices that need copying

    size_t qi = 0;
    while (qi < bfs_queue.size()) {
        int w_node = bfs_queue[qi++];

        if (worker.store_.is_leaf(w_node))
            continue;

        int w_left  = worker.store_.left(w_node);
        int w_right = worker.store_.right(w_node);

        // Process left child
        if (w_left >= 0) {
            if (w_left >= snapshot_n_nodes) {
                // New node — allocate in coordinator
                int c_idx = store_.alloc_node();
                remap[w_left] = c_idx;
                new_worker_nodes.push_back(w_left);
                ++transplanted;
            } else {
                remap[w_left] = w_left;  // shared node
            }
            bfs_queue.push_back(w_left);
        }

        // Process right child
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

    // Ensure ep_store / hull capacity
    int needed = store_.n_nodes();
    ensure_ep_capacity(needed);
    ensure_hull_capacity(needed);

    // Second pass: copy node data for all new nodes
    for (int w_idx : new_worker_nodes) {
        int c_idx = remap[w_idx];

        // Copy raw node record (tree structure + link AABBs)
        store_.copy_node_from(worker.store_, w_idx, c_idx);

        // Remap left/right/parent indices
        int w_left = worker.store_.left(w_idx);
        int w_right = worker.store_.right(w_idx);
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

        // Remap forest_id via box_id_remap
        int fid = worker.store_.forest_id(w_idx);
        if (fid >= 0) {
            auto it = box_id_remap.find(fid);
            if (it != box_id_remap.end()) {
                store_.set_forest_id(c_idx, it->second);
                store_.set_occupied(c_idx, true);
            }
        }

        // Copy ep_store
        if (w_idx < static_cast<int>(worker.ep_store_.size()) &&
            !worker.ep_store_[w_idx].empty()) {
            if (c_idx >= static_cast<int>(ep_store_.size()))
                ensure_ep_capacity(c_idx + 1);
            ep_store_[c_idx] = worker.ep_store_[w_idx];
        }

        // Copy hull_grids and hull_valid
        if (w_idx < static_cast<int>(worker.hull_valid_.size()) &&
            worker.hull_valid_[w_idx]) {
            if (c_idx >= static_cast<int>(hull_grids_.size()))
                ensure_hull_capacity(c_idx + 1);
            hull_grids_[c_idx] = worker.hull_grids_[w_idx];
            hull_valid_[c_idx] = 1;
        }
    }

    // Update the coordinator_node itself: copy left/right from worker
    // (this converts the coordinator leaf into an interior node)
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
        // Copy split value from worker (the worker set it when it split)
        store_.set_split(coordinator_node, worker.store_.split(coordinator_node));
    }

    return transplanted;
}

// ═════════════════════════════════════════════════════════════════════════
//  pre_expand — split all leaves up to target_depth
// ═════════════════════════════════════════════════════════════════════════
int LECT::pre_expand(int target_depth) {
    if (store_.n_nodes() == 0) return 0;
    int new_count = 0;
    pre_expand_recursive(0, root_fk_, root_limits_.limits,
                         target_depth, new_count);
    return new_count;
}

int LECT::compute_all_hull_grids() {
    // Only compute hull grids for pipelines that use hull refinement
    if (pipeline_config_.envelope.type != envelope::EnvelopeType::SubAABB_Grid &&
        pipeline_config_.envelope.type != envelope::EnvelopeType::Hull16_Grid)
        return 0;

    int count = 0;
    const int n = store_.n_nodes();
    for (int i = 0; i < n; ++i) {
        if (store_.has_aabb(i) && !has_hull_grid(i)) {
            auto ivs = node_intervals(i);
            derive_hull_grid(i, ivs);
            ++count;
        }
    }
    return count;
}

void LECT::pre_expand_recursive(int node_idx, const FKState& fk,
                                const std::vector<Interval>& intervals,
                                int target_depth, int& new_node_count)
{
    int depth = store_.depth(node_idx);
    if (depth >= target_depth) return;

    // Split if leaf
    if (store_.is_leaf(node_idx)) {
        int dim = split_dims_[depth % n_dims()];
        double edge = intervals[dim].width();
        // Skip if too narrow to split
        if (edge <= 1e-6) return;
        split_leaf(node_idx, fk, intervals);
        new_node_count += 2;
    }

    // Recurse into children
    int dim = split_dims_[depth % n_dims()];
    double mid = store_.split(node_idx);

    auto left_ivs = intervals;
    left_ivs[dim].hi = mid;
    FKState left_fk = compute_fk_incremental(fk, *robot_, left_ivs, dim);
    pre_expand_recursive(store_.left(node_idx), left_fk, left_ivs,
                         target_depth, new_node_count);

    auto right_ivs = intervals;
    right_ivs[dim].lo = mid;
    FKState right_fk = compute_fk_incremental(fk, *robot_, right_ivs, dim);
    pre_expand_recursive(store_.right(node_idx), right_fk, right_ivs,
                         target_depth, new_node_count);
}

// ─────────────────────────────────────────────────────────────────────────
//  init_root — compute full FK for root C-space box, cache envelopes
// ─────────────────────────────────────────────────────────────────────────
void LECT::init_root() {
    int root_idx = store_.alloc_node();   // always 0
    store_.set_depth(root_idx, 0);
    store_.set_parent(root_idx, -1);

    // Compute full FK over entire joint range
    root_fk_ = compute_fk_full(*robot_, root_limits_.limits);

    // Store frames + AABBs + hull-16 grid
    compute_envelope(root_idx, root_fk_, root_limits_.limits);
}

// ─────────────────────────────────────────────────────────────────────────
//  reconstruct_and_derive_link_aabbs — multi-level from local-frame ep_store_
//
//  Reads frozen-frame endpoints from ep_store_[node_idx], reconstructs
//  Cartesian (x,y,z) using the frozen joints' intervals from `intervals`,
//  then derives per-link AABBs.
// ─────────────────────────────────────────────────────────────────────────
void LECT::reconstruct_and_derive_link_aabbs(
    int node_idx, const std::vector<Interval>& intervals)
{
    const auto& local_ep = ep_store_[node_idx];
    int n_ep = robot_->n_joints() + (robot_->has_tool() ? 1 : 0);

    // Build frozen joint descriptors with actual q intervals for this node
    std::vector<FrozenJointDesc> descs = frozen_descs_;
    for (int j = 0; j < freeze_depth_; ++j)
        descs[j].q = intervals[j];

    // Reconstruct Cartesian endpoints via multi-level rotation
    std::vector<float> cart_ep = reconstruct_cartesian_endpoints_multilevel(
        local_ep, n_ep, descs.data(), freeze_depth_);

    // Derive per-link AABBs from Cartesian endpoints + radius
    envelope::EndpointAABBResult cart_result;
    cart_result.endpoint_aabbs = std::move(cart_ep);
    cart_result.n_active    = robot_->n_active_links();
    cart_result.n_endpoints = n_ep;

    store_.ensure_capacity(node_idx + 1);
    float* aabb = store_.link_aabbs_mut(node_idx);
    envelope::extract_link_aabbs_from_endpoint(cart_result, *robot_, aabb);
    store_.set_has_aabb(node_idx, true);
}

// ─────────────────────────────────────────────────────────────────────────
//  compute_envelope — two-stage modular pipeline: endpoint source → link envelope
//
//  Multi-level joint-freezing optimisation:
//    Stage 1 freezes the leading freeze_depth_ joints to q=0, producing
//    q₀..q_{k-1}-independent endpoint AABBs stored in ep_store_.
//    Stage 2 reconstructs Cartesian via multi-level interval rotation.
// ─────────────────────────────────────────────────────────────────────────
void LECT::compute_envelope(int node_idx, const FKState& /*fk*/,
                            const std::vector<Interval>& intervals) {
    // ── Cache check: skip if existing AABB has sufficient source quality ──
    if (store_.has_aabb(node_idx)) {
        uint8_t cached_q = store_.source_quality(node_idx);
        uint8_t needed_q = envelope::endpoint_source_quality(
            pipeline_config_.source.method);
        if (cached_q >= needed_q)
            return;  // cached envelope is at least as tight — reuse
    }

    // ── Stage 1: Compute endpoint AABBs in LOCAL FRAME ──────────────────
    //    Freeze the leading freeze_depth_ joints to q=0.
    //    This removes their rotation-induced interval wrapping from FK,
    //    producing tighter bounds, and allows split caching.
    std::vector<Interval> local_intervals = intervals;
    for (int j = 0; j < freeze_depth_; ++j)
        local_intervals[j] = {0.0, 0.0};

    auto ep = envelope::compute_endpoint_aabb(
        pipeline_config_.source, *robot_, local_intervals);

    // Store LOCAL-FRAME endpoint_aabbs [n_endpoints × 6]
    ensure_ep_capacity(node_idx + 1);
    ep_store_[node_idx] = ep.endpoint_aabbs;

    // ── Stage 2: Reconstruct Cartesian endpoints, derive link AABBs ──────
    reconstruct_and_derive_link_aabbs(node_idx, intervals);

    // ── Persist LOCAL-FRAME endpoints for disk cache ─────────────────────
    frame_store_.ensure_capacity(node_idx + 1);
    frame_store_.store_frames(node_idx, ep.endpoint_aabbs.data());

    // ── Record source quality for this node ─────────────────────────────
    store_.set_source_quality(node_idx,
        envelope::endpoint_source_quality(pipeline_config_.source.method));

    // Hull VoxelGrid: LAZY — NOT computed here.
    // derive_hull_grid() called on-demand during collision refinement.
}

// ─────────────────────────────────────────────────────────────────────────
//  derive_hull_grid — rasterise hull via fill_hull16 (Conv(B₁∪B₂)⊕Ball)
//
//  ep_store_ holds LOCAL-FRAME intervals.  The full intervals vector is
//  required to reconstruct Cartesian endpoints for hull rasterisation.
//  Both the primary path (ep_store_) and fallback path (frame_store_)
//  use fill_hull16 for tight convex-hull-based envelopes.
// ─────────────────────────────────────────────────────────────────────────
void LECT::derive_hull_grid(int node_idx, const std::vector<Interval>& intervals) {
    ensure_hull_capacity(node_idx + 1);

    const int n_act = robot_->n_active_links();
    const int* link_map = robot_->active_link_map();
    const double* lr = robot_->active_link_radii();
    if (!lr) { hull_valid_[node_idx] = 0; return; }

    // ── Primary path: ep_store_ (local-frame endpoint intervals) ────────
    if (node_idx < static_cast<int>(ep_store_.size()) &&
        !ep_store_[node_idx].empty()) {

        const int n_endpoints = robot_->n_joints() +
                             (robot_->has_tool() ? 1 : 0);

        // Reconstruct Cartesian endpoints from local-frame via multi-level
        std::vector<FrozenJointDesc> descs = frozen_descs_;
        for (int j = 0; j < freeze_depth_; ++j)
            descs[j].q = intervals[j];

        std::vector<float> cart_ep = reconstruct_cartesian_endpoints_multilevel(
            ep_store_[node_idx], n_endpoints, descs.data(), freeze_depth_);

        // Base frame (frame 0 proximal) is constant [0,0,0]
        const float* base = frame_store_.base_pos();

        voxel::VoxelGrid grid(voxel_delta_);

        // Pre-compute total bounding box for reserve
        {
            double total_lo[3] = { 1e30,  1e30,  1e30};
            double total_hi[3] = {-1e30, -1e30, -1e30};
            for (int ci = 0; ci < n_act; ++ci) {
                int fidx = link_map[ci];
                const float* pf = (fidx == 0) ? base : (cart_ep.data() + (fidx - 1) * 6);
                const float* df = cart_ep.data() + fidx * 6;
                double r = lr[ci];
                for (int a = 0; a < 3; ++a) {
                    double plo = (fidx == 0) ? base[a] : pf[a];
                    double phi = (fidx == 0) ? base[a] : pf[a + 3];
                    double dlo = df[a],  dhi = df[a + 3];
                    total_lo[a] = std::min(total_lo[a], std::min(plo, dlo) - r);
                    total_hi[a] = std::max(total_hi[a], std::max(phi, dhi) + r);
                }
            }
            double sp = std::sqrt(3.0) * voxel_delta_ * 0.5;
            for (int a = 0; a < 3; ++a) {
                total_lo[a] -= sp;
                total_hi[a] += sp;
            }
            grid.reserve_from_bounds(total_lo, total_hi);
        }

        // fill_hull16 for each active link
        for (int ci = 0; ci < n_act; ++ci) {
            int fidx = link_map[ci];
            float prox[6], dist[6];
            if (fidx == 0) {
                for (int a = 0; a < 3; ++a) {
                    prox[a]     = base[a];
                    prox[a + 3] = base[a];
                }
            } else {
                std::memcpy(prox, cart_ep.data() + (fidx - 1) * 6, 6 * sizeof(float));
            }
            std::memcpy(dist, cart_ep.data() + fidx * 6, 6 * sizeof(float));
            grid.fill_hull16(prox, dist, lr[ci]);
        }

        hull_grids_[node_idx] = std::move(grid);
        hull_valid_[node_idx] = 1;
        return;
    }

    // Fallback: legacy frames path (for loaded caches without ep_store_)
    const float* legacy_frames = frame_store_.get_frames(node_idx);
    if (!legacy_frames) return;

    const float* base = frame_store_.base_pos();

    voxel::VoxelGrid grid(voxel_delta_);

    // ── Pre-compute total bounding box across all links for reserve ─────
    {
        double total_lo[3] = { 1e30,  1e30,  1e30};
        double total_hi[3] = {-1e30, -1e30, -1e30};

        for (int k = 0; k < n_act; ++k) {
            int fidx = link_map[k];
            const float* pf = (fidx == 0) ? base : (legacy_frames + (fidx - 1) * 6);
            const float* df = legacy_frames + fidx * 6;
            double r = lr[k];
            for (int a = 0; a < 3; ++a) {
                double plo = (fidx == 0) ? base[a] : pf[a];
                double phi = (fidx == 0) ? base[a] : pf[a + 3];
                double dlo = df[a],  dhi = df[a + 3];
                total_lo[a] = std::min(total_lo[a], std::min(plo, dlo) - r);
                total_hi[a] = std::max(total_hi[a], std::max(phi, dhi) + r);
            }
        }
        double sp = std::sqrt(3.0) * voxel_delta_ * 0.5;
        for (int a = 0; a < 3; ++a) {
            total_lo[a] -= sp;
            total_hi[a] += sp;
        }
        grid.reserve_from_bounds(total_lo, total_hi);
    }

    for (int k = 0; k < n_act; ++k) {
        int fidx = link_map[k];

        float prox[6], dist[6];

        if (fidx == 0) {
            for (int a = 0; a < 3; ++a) {
                prox[a]     = base[a];
                prox[a + 3] = base[a];
            }
        } else {
            std::memcpy(prox, legacy_frames + (fidx - 1) * 6, 6 * sizeof(float));
        }

        std::memcpy(dist, legacy_frames + fidx * 6, 6 * sizeof(float));

        grid.fill_hull16(prox, dist, lr[k]);
    }

    hull_grids_[node_idx] = std::move(grid);
    hull_valid_[node_idx] = 1;
}

// ─────────────────────────────────────────────────────────────────────────
//  Per-node data access
// ─────────────────────────────────────────────────────────────────────────
const float* LECT::get_link_aabbs(int node_idx) const {
    return store_.link_aabbs(node_idx);
}

const float* LECT::get_endpoint_aabbs(int node_idx) const {
    if (node_idx < 0 || node_idx >= static_cast<int>(ep_store_.size()))
        return nullptr;
    if (ep_store_[node_idx].empty()) return nullptr;
    return ep_store_[node_idx].data();
}

bool LECT::has_endpoint_aabbs(int node_idx) const {
    if (node_idx < 0 || node_idx >= static_cast<int>(ep_store_.size()))
        return false;
    return !ep_store_[node_idx].empty();
}

const float* LECT::get_frames(int node_idx) const {
    return frame_store_.get_frames(node_idx);
}

bool LECT::has_frames(int node_idx) const {
    return frame_store_.has_frames(node_idx);
}

bool LECT::has_hull_grid(int node_idx) const {
    if (node_idx < 0 || node_idx >= static_cast<int>(hull_valid_.size()))
        return false;
    return hull_valid_[node_idx] != 0;
}

const voxel::VoxelGrid& LECT::get_hull_grid(int node_idx) const {
    return hull_grids_[node_idx];
}

// ─────────────────────────────────────────────────────────────────────────
//  node_intervals — reconstruct C-space box by descending from root
// ─────────────────────────────────────────────────────────────────────────
std::vector<Interval> LECT::node_intervals(int node_idx) const {
    std::vector<Interval> ivs = root_limits_.limits;

    // Build path from root to node
    std::vector<int> path;
    for (int cur = node_idx; cur >= 0; cur = store_.parent(cur))
        path.push_back(cur);
    std::reverse(path.begin(), path.end());

    // Walk from root, narrowing intervals at each split
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        int p = path[i];
        int dim = split_dims_[store_.depth(p) % n_dims()];
        double mid = store_.split(p);

        if (path[i + 1] == store_.left(p)) {
            ivs[dim].hi = mid;
        } else {
            ivs[dim].lo = mid;
        }
    }
    return ivs;
}

// ═════════════════════════════════════════════════════════════════════════
//  Scene management — pre-rasterise obstacles into shared VoxelGrid
// ═════════════════════════════════════════════════════════════════════════

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

void LECT::clear_scene() {
    scene_grid_ = voxel::VoxelGrid();
    scene_set_ = false;
}

// ═════════════════════════════════════════════════════════════════════════
//  find_free_box — KD-tree descent with lazy splitting
// ═════════════════════════════════════════════════════════════════════════
FFBResult LECT::find_free_box(
    const Eigen::VectorXd& seed,
    const Obstacle* obstacles, int n_obs,
    double min_edge, int max_depth)
{
    FFBResult result;
    result.path.reserve(max_depth + 1);

    int cur = 0;  // start at root
    FKState fk = root_fk_;
    std::vector<Interval> intervals = root_limits_.limits;

    while (true) {
        result.path.push_back(cur);

        // ── Occupied check ──────────────────────────────────────────
        if (store_.is_occupied(cur)) {
            result.fail_code = 1;
            return result;
        }

        // ── AABB collision check ────────────────────────────────────
        if (store_.has_aabb(cur)) {
            bool aabb_hit = aabbs_collide(cur, obstacles, n_obs);

            if (!aabb_hit && store_.subtree_occ(cur) == 0) {
                // No collision and no occupied descendants → SUCCESS
                result.node_idx = cur;
                result.fail_code = 0;
                propagate_up(result.path);
                return result;
            }

            // Refinement: if AABB says collision, try hull-16  (lazy compute)
            if (aabb_hit) {
                bool need_hull =
                    pipeline_config_.envelope.type == envelope::EnvelopeType::SubAABB_Grid ||
                    pipeline_config_.envelope.type == envelope::EnvelopeType::Hull16_Grid;

                // P5: skip hull refinement for small C-space nodes.
                // For tiny intervals AABB is already tight, hull adds
                // little value but costs hash-map allocation.
                if (need_hull && hull_skip_vol_ > 0.0) {
                    double node_vol = 1.0;
                    for (auto& iv : intervals) node_vol *= iv.width();
                    if (node_vol < hull_skip_vol_) need_hull = false;
                }

                if (need_hull) {
                    if (!has_hull_grid(cur))
                        derive_hull_grid(cur, intervals);  // pass full intervals for Cartesian reconstruction
                    if (has_hull_grid(cur)) {
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
            int dim = split_dims_[d % n_dims()];
            double edge = intervals[dim].width();

            if (d >= max_depth) {
                result.fail_code = 2;
                return result;
            }
            if (edge <= min_edge) {
                result.fail_code = 3;
                return result;
            }

            // Split this leaf
            split_leaf(cur, fk, intervals);
            result.n_new_nodes += 2;
        }

        // ── Descend toward seed ─────────────────────────────────────
        int d = store_.depth(cur);
        int dim = split_dims_[d % n_dims()];
        double mid = store_.split(cur);
        int child;
        std::vector<Interval> child_ivs = intervals;

        if (seed[dim] < mid) {
            child = store_.left(cur);
            child_ivs[dim].hi = mid;
        } else {
            child = store_.right(cur);
            child_ivs[dim].lo = mid;
        }

        // Always compute FK incrementally + envelope for child
        FKState child_fk = compute_fk_incremental(fk, *robot_, child_ivs, dim);
        result.n_fk_calls++;
        // compute_envelope self-checks: skips if cached quality is sufficient
        compute_envelope(child, child_fk, child_ivs);
        fk = child_fk;

        intervals = child_ivs;
        cur = child;
    }
}

// ─────────────────────────────────────────────────────────────────────────
//  split_leaf
// ─────────────────────────────────────────────────────────────────────────
void LECT::split_leaf(int node_idx, const FKState& parent_fk,
                      const std::vector<Interval>& parent_intervals)
{
    int d = store_.depth(node_idx);
    int dim = split_dims_[d % n_dims()];
    double mid = parent_intervals[dim].center();

    // Allocate children
    int left_idx  = store_.alloc_node();
    int right_idx = store_.alloc_node();

    store_.set_left(node_idx, left_idx);
    store_.set_right(node_idx, right_idx);

    store_.set_parent(left_idx, node_idx);
    store_.set_parent(right_idx, node_idx);
    store_.set_depth(left_idx, d + 1);
    store_.set_depth(right_idx, d + 1);
    store_.set_split(node_idx, mid);

    // Compute child intervals
    std::vector<Interval> left_ivs = parent_intervals;
    left_ivs[dim].hi = mid;

    std::vector<Interval> right_ivs = parent_intervals;
    right_ivs[dim].lo = mid;

    // Check if ep_store_ has valid data for this node (may be empty after cache load)
    bool has_ep = (node_idx < static_cast<int>(ep_store_.size()) &&
                   !ep_store_[node_idx].empty());

    if (dim < freeze_depth_ && has_ep) {
        // ── Frozen-joint split: inherit local-frame endpoint cache ───────
        // Local-frame intervals are independent of frozen joints →
        // skip Stage 1 entirely; only reconstruct Cartesian link AABBs
        // with the narrower child intervals.
        ensure_ep_capacity(std::max(left_idx, right_idx) + 1);
        ep_store_[left_idx]  = ep_store_[node_idx];   // inherit
        ep_store_[right_idx] = ep_store_[node_idx];

        // Persist inherited endpoints
        frame_store_.ensure_capacity(std::max(left_idx, right_idx) + 1);
        frame_store_.store_frames(left_idx,  ep_store_[node_idx].data());
        frame_store_.store_frames(right_idx, ep_store_[node_idx].data());

        // Reconstruct Cartesian link AABBs with child intervals
        reconstruct_and_derive_link_aabbs(left_idx,  left_ivs);
        reconstruct_and_derive_link_aabbs(right_idx, right_ivs);

        // Inherit parent's source quality for frozen-joint split
        uint8_t parent_q = store_.source_quality(node_idx);
        store_.set_source_quality(left_idx,  parent_q);
        store_.set_source_quality(right_idx, parent_q);
    } else {
        // ── Non-q₀ split: full Stage 1 + Stage 2 ───────────────────────
        // Incremental FK for both children
        FKState left_fk  = compute_fk_incremental(parent_fk, *robot_, left_ivs, dim);
        FKState right_fk = compute_fk_incremental(parent_fk, *robot_, right_ivs, dim);

        // Compute and cache envelopes for both children
        compute_envelope(left_idx, left_fk, left_ivs);
        compute_envelope(right_idx, right_fk, right_ivs);
    }

    // Refine parent AABBs: parent = intersect(parent, union(left, right))
    refine_aabb(node_idx);
}

// ─────────────────────────────────────────────────────────────────────────
//  Collision checking helpers
// ─────────────────────────────────────────────────────────────────────────

/// Test if a single link AABB overlaps an obstacle.
static bool aabb_overlaps_obs(const float* link_aabb, const Obstacle& obs) {
    // link_aabb: [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]
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

bool LECT::aabbs_collide(int node_idx,
                         const Obstacle* obstacles, int n_obs) const
{
    const float* aabb = store_.link_aabbs(node_idx);
    int n_links = store_.n_active_links();
    for (int k = 0; k < n_links; ++k) {
        for (int j = 0; j < n_obs; ++j) {
            if (aabb_overlaps_obs(aabb + k * 6, obstacles[j]))
                return true;
        }
    }
    return false;
}

bool LECT::hull_collides(int node_idx,
                         const Obstacle* obstacles, int n_obs) const
{
    if (!has_hull_grid(node_idx)) return true;  // conservative

    // Fast path: use pre-rasterized scene grid
    if (scene_set_) {
        return hull_grids_[node_idx].collides(scene_grid_);
    }

    // Slow path: build obstacle grid on-the-fly
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
    return hull_grids_[node_idx].collides(obs_grid);
}

bool LECT::hull_collides_grid(int node_idx,
                               const voxel::VoxelGrid& obs_grid) const
{
    if (!has_hull_grid(node_idx)) return true;  // conservative
    return hull_grids_[node_idx].collides(obs_grid);
}

// ─────────────────────────────────────────────────────────────────────────
//  AABB refinement & propagation
// ─────────────────────────────────────────────────────────────────────────
void LECT::refine_aabb(int parent_idx) {
    int li = store_.left(parent_idx);
    int ri = store_.right(parent_idx);
    if (li < 0 || ri < 0) return;

    if (!store_.has_aabb(li) || !store_.has_aabb(ri)) return;

    float* p_aabb = store_.link_aabbs_mut(parent_idx);
    const float* l_aabb = store_.link_aabbs(li);
    const float* r_aabb = store_.link_aabbs(ri);
    int n = store_.n_active_links() * 6;

    // union_aabb = hull(left, right);  refined = intersect(parent, union)
    for (int i = 0; i < n; ++i) {
        if (i % 6 < 3) {
            // lo component: union = min(l, r); refine = max(parent, union)
            float u = std::min(l_aabb[i], r_aabb[i]);
            p_aabb[i] = std::max(p_aabb[i], u);
        } else {
            // hi component: union = max(l, r); refine = min(parent, union)
            float u = std::max(l_aabb[i], r_aabb[i]);
            p_aabb[i] = std::min(p_aabb[i], u);
        }
    }
}

void LECT::propagate_up(const std::vector<int>& path) {
    // Walk from deepest split node toward root, refining AABBs
    for (int i = static_cast<int>(path.size()) - 2; i >= 0; --i) {
        int p = path[i];
        if (!store_.is_leaf(p))
            refine_aabb(p);
    }
}

// ─────────────────────────────────────────────────────────────────────────
//  Occupation management
// ─────────────────────────────────────────────────────────────────────────
void LECT::mark_occupied(int node_idx, int box_id) {
    store_.set_occupied(node_idx, true);
    store_.set_forest_id(node_idx, box_id);
    // Increment subtree_occ upward
    for (int p = store_.parent(node_idx); p >= 0; p = store_.parent(p))
        store_.set_subtree_occ(p, store_.subtree_occ(p) + 1);
}

void LECT::unmark_occupied(int node_idx) {
    store_.set_occupied(node_idx, false);
    store_.set_forest_id(node_idx, -1);
    // Decrement subtree_occ upward
    for (int p = store_.parent(node_idx); p >= 0; p = store_.parent(p))
        store_.set_subtree_occ(p, store_.subtree_occ(p) - 1);
}

// ─────────────────────────────────────────────────────────────────────────
//  Public collision query — two-stage (AABB → Hull)
// ─────────────────────────────────────────────────────────────────────────
bool LECT::collides_scene(int node_idx,
                          const Obstacle* obstacles, int n_obs) const
{
    // Stage 1: AABB early-out
    if (store_.has_aabb(node_idx)) {
        if (!aabbs_collide(node_idx, obstacles, n_obs))
            return false;  // AABB says no collision → safe
    }

    // Stage 2: Hull refinement (lazily compute if needed)
    bool need_hull =
        pipeline_config_.envelope.type == envelope::EnvelopeType::SubAABB_Grid ||
        pipeline_config_.envelope.type == envelope::EnvelopeType::Hull16_Grid;
    if (need_hull) {
        if (!has_hull_grid(node_idx)) {
            // Lazy compute — cast away const (hull is a cache, not logical state)
            auto ivs = node_intervals(node_idx);
            const_cast<LECT*>(this)->derive_hull_grid(node_idx, ivs);
        }
        if (has_hull_grid(node_idx)) {
            return hull_collides(node_idx, obstacles, n_obs);
        }
    }

    return true;  // conservative: assume collision if no data
}

// ─────────────────────────────────────────────────────────────────────────
//  Public collision query for ARBITRARY intervals (no tree node needed)
// ─────────────────────────────────────────────────────────────────────────
bool LECT::intervals_collide_scene(const std::vector<Interval>& intervals,
                                   const Obstacle* obstacles, int n_obs) const
{
    if (!robot_ || n_obs <= 0) return false;

    // Stage 1: Compute endpoint AABBs → extract per-link AABBs
    auto ep = envelope::compute_endpoint_aabb(
        pipeline_config_.source, *robot_, intervals);

    int n_active = robot_->n_active_links();
    std::vector<float> link_aabbs(n_active * 6);
    envelope::extract_link_aabbs_from_endpoint(ep, *robot_, link_aabbs.data());

    // AABB early-out: check each link against each obstacle
    bool aabb_hit = false;
    for (int k = 0; k < n_active && !aabb_hit; ++k) {
        for (int j = 0; j < n_obs; ++j) {
            if (aabb_overlaps_obs(link_aabbs.data() + k * 6, obstacles[j])) {
                aabb_hit = true;
                break;
            }
        }
    }
    if (!aabb_hit) return false;  // no AABB overlap → safe

    // Stage 2: Hull grid refinement (grid-based envelopes only)
    bool need_hull =
        pipeline_config_.envelope.type == envelope::EnvelopeType::SubAABB_Grid ||
        pipeline_config_.envelope.type == envelope::EnvelopeType::Hull16_Grid;
    if (need_hull) {
        // Build temporary hull grid from link AABBs
        voxel::VoxelGrid tmp_grid(voxel_delta_);
        const int n_sub = pipeline_config_.envelope.n_sub;
        const int n_endpoints = robot_->n_joints() +
                               (robot_->has_tool() ? 1 : 0);
        const int* link_map = robot_->active_link_map();
        const double* lr = robot_->active_link_radii();

        if (n_sub > 1 && !ep.endpoint_aabbs.empty()) {
            // SubAABB path: derive sub-AABBs from endpoint AABBs
            float base_pos[3] = {0.f, 0.f, 0.f};
            std::vector<float> sub_aabbs(n_active * n_sub * 6);
            for (int ci = 0; ci < n_active; ++ci) {
                int parent_fi = (link_map[ci] == 0) ? -1 : link_map[ci] - 1;
                int link_fi = link_map[ci];
                float r = lr ? static_cast<float>(lr[ci]) : 0.f;
                envelope::derive_aabb_subdivided(
                    ep.endpoint_aabbs.data(), n_endpoints, parent_fi, link_fi,
                    n_sub, r, base_pos,
                    sub_aabbs.data() + ci * n_sub * 6);
            }
            int total = n_active * n_sub;
            for (int k = 0; k < total; ++k)
                tmp_grid.fill_aabb(sub_aabbs.data() + k * 6);
        } else {
            // Simple path: fill from link AABBs directly
            for (int k = 0; k < n_active; ++k)
                tmp_grid.fill_aabb(link_aabbs.data() + k * 6);
        }

        // Check against scene grid
        if (scene_set_) {
            return tmp_grid.collides(scene_grid_);
        } else {
            // Build obstacle grid on-the-fly
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
            return tmp_grid.collides(obs_grid);
        }
    }

    return true;  // conservative: assume collision if AABB positive and no hull
}

// ─────────────────────────────────────────────────────────────────────────
//  Hull-based Greedy Coarsening helpers
// ─────────────────────────────────────────────────────────────────────────

double LECT::merged_children_hull_volume(int node_idx) const
{
    int li = store_.left(node_idx);
    int ri = store_.right(node_idx);
    if (li < 0 || ri < 0) return -1.0;
    if (!has_hull_grid(li) || !has_hull_grid(ri)) return -1.0;

    // Compute union: left | right
    voxel::VoxelGrid merged = hull_grids_[li];
    merged.merge(hull_grids_[ri]);
    return merged.occupied_volume();
}

double LECT::coarsen_volume_ratio(int node_idx) const
{
    int li = store_.left(node_idx);
    int ri = store_.right(node_idx);
    if (li < 0 || ri < 0) return -1.0;
    if (!has_hull_grid(li) || !has_hull_grid(ri)) return -1.0;

    double lv = hull_grids_[li].occupied_volume();
    double rv = hull_grids_[ri].occupied_volume();
    double max_child = std::max(lv, rv);
    if (max_child < 1e-12) return -1.0;  // degenerate

    double merged_vol = merged_children_hull_volume(node_idx);
    if (merged_vol < 0) return -1.0;

    return merged_vol / max_child;
}

bool LECT::merge_children_hulls(int node_idx)
{
    int li = store_.left(node_idx);
    int ri = store_.right(node_idx);
    if (li < 0 || ri < 0) return false;

    // Lazy hull grid computation for children (if not yet done)
    if (!has_hull_grid(li)) {
        auto ivs = node_intervals(li);
        derive_hull_grid(li, ivs);
    }
    if (!has_hull_grid(ri)) {
        auto ivs = node_intervals(ri);
        derive_hull_grid(ri, ivs);
    }
    if (!has_hull_grid(li) || !has_hull_grid(ri)) return false;

    ensure_hull_capacity(node_idx + 1);

    // Parent hull = left | right (bitwise-OR, zero-loss merge)
    hull_grids_[node_idx] = hull_grids_[li];
    hull_grids_[node_idx].merge(hull_grids_[ri]);
    hull_valid_[node_idx] = 1;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────
//  Hull capacity management
// ─────────────────────────────────────────────────────────────────────────
void LECT::ensure_hull_capacity(int n) {
    if (static_cast<int>(hull_grids_.size()) >= n) return;
    hull_grids_.resize(n);
    hull_valid_.resize(n, 0);
}

void LECT::ensure_ep_capacity(int n) {
    if (static_cast<int>(ep_store_.size()) >= n) return;
    ep_store_.resize(n);
}

// ═════════════════════════════════════════════════════════════════════════
//  Persistence
// ═════════════════════════════════════════════════════════════════════════

void LECT::save(const std::string& dir) const {
    store_.save(dir + "/lect.hcache");
    // FrameStore save needs a non-const method; we cast it safely
    const_cast<envelope::FrameStore&>(frame_store_).save(dir + "/lect.frames");
    save_hull_grids(dir + "/lect.hulls");

    // Persist pipeline metadata for cache validation
    envelope::CacheMetadata meta;
    meta.source_method  = pipeline_config_.source.method;
    meta.envelope_type  = pipeline_config_.envelope.type;
    meta.n_sub          = pipeline_config_.envelope.n_sub;
    meta.delta          = pipeline_config_.envelope.delta;
    meta.grid_R         = pipeline_config_.envelope.grid_R;
    meta.n_nodes        = store_.n_nodes();
    meta.n_frames       = frame_store_.n_frames();
    meta.n_hulls        = count_nodes_with_hull();
    meta.robot_hash     = envelope::EnvelopeCache::compute_robot_hash(*robot_);
    envelope::EnvelopeCache::save_metadata(dir, meta);
}

void LECT::load(const std::string& dir, const Robot& robot) {
    robot_ = &robot;

    // Load tree structure + AABBs
    store_.load(dir + "/lect.hcache");

    // Rebuild metadata
    root_limits_ = robot.joint_limits();
    split_dims_.resize(robot.n_joints());
    for (int d = 0; d < robot.n_joints(); ++d)
        split_dims_[d] = d;

    // Recompute root FK (needed for future FFB calls)
    root_fk_ = compute_fk_full(*robot_, root_limits_.limits);

    // Load frame store
    frame_store_ = envelope::FrameStore(robot, store_.capacity());
    frame_store_.load(dir + "/lect.frames");

    // Load hull grids
    load_hull_grids(dir + "/lect.hulls");

    // ── Recover ep_store_ from frame_store_ ─────────────────────────────
    //    ep_store_ is transient (not persisted directly), but frame_store_
    //    contains the identical LOCAL-FRAME endpoint AABBs.  Recovering
    //    them enables derive_hull_grid() to use the primary (sub-AABB)
    //    path for nodes that need hull computation after cache load.
    {
        const int n = store_.n_nodes();
        ensure_ep_capacity(n);
        const int fpn = frame_store_.floats_per_node();
        for (int i = 0; i < n; ++i) {
            if (frame_store_.has_frames(i)) {
                const float* ptr = frame_store_.get_frames(i);
                ep_store_[i].assign(ptr, ptr + fpn);
            }
        }
    }

    // ── Tag all loaded nodes with cached source quality ─────────────────
    envelope::CacheMetadata meta;
    if (envelope::EnvelopeCache::load_metadata(dir, meta)) {
        uint8_t q = envelope::endpoint_source_quality(meta.source_method);
        for (int i = 0; i < store_.n_nodes(); ++i) {
            if (store_.has_aabb(i))
                store_.set_source_quality(i, q);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────
//  Hull persistence — HUL1 format
// ─────────────────────────────────────────────────────────────────────────
//
//  Header (512B):
//    [0-4]   uint32 magic = "HUL1"
//    [4-8]   uint32 version = 1
//    [8-16]  int64  n_valid (nodes with hull data)
//    [16-24] int64  n_alloc (total slots)
//    [24-32] float64 voxel_delta
//    [32-40] float64 safety_pad
//    [40-512] reserved
//
//  Node data (sequential, for all allocated slots):
//    Per slot:
//      uint8 valid  (0 = no data, 1 = has data)
//      If valid:
//        int32 n_bricks
//        Per brick:
//          int32 bx, int32 by, int32 bz  (BrickCoord, 12B)
//          uint64[8] words               (BitBrick, 64B)
//        Total: 76 bytes per brick
//
void LECT::save_hull_grids(const std::string& path) const {
    std::ofstream f(path, std::ios::binary);
    if (!f)
        throw std::runtime_error("LECT::save_hull_grids: cannot open " + path);

    int n_alloc = static_cast<int>(hull_grids_.size());
    int n_valid = 0;
    for (int i = 0; i < n_alloc; ++i)
        if (hull_valid_[i]) ++n_valid;

    // ── Header ──
    char header[HULL_HEADER_SIZE] = {};
    std::memcpy(header + 0, &HULL_MAGIC, 4);
    *reinterpret_cast<uint32_t*>(header + 4)  = 1;           // version
    *reinterpret_cast<int64_t*>(header + 8)   = n_valid;
    *reinterpret_cast<int64_t*>(header + 16)  = n_alloc;
    *reinterpret_cast<double*>(header + 24)   = voxel_delta_;
    *reinterpret_cast<double*>(header + 32)   = 0.0;         // safety_pad (default)
    f.write(header, HULL_HEADER_SIZE);

    // ── Node data ──
    for (int i = 0; i < n_alloc; ++i) {
        uint8_t valid = (i < static_cast<int>(hull_valid_.size()) && hull_valid_[i]) ? 1 : 0;
        f.write(reinterpret_cast<const char*>(&valid), 1);

        if (!valid) continue;

        const auto& bricks = hull_grids_[i].bricks();
        int32_t n_bricks = static_cast<int32_t>(bricks.size());
        f.write(reinterpret_cast<const char*>(&n_bricks), sizeof(n_bricks));

        for (const auto& [coord, brick] : bricks) {
            f.write(reinterpret_cast<const char*>(&coord.bx), sizeof(int));
            f.write(reinterpret_cast<const char*>(&coord.by), sizeof(int));
            f.write(reinterpret_cast<const char*>(&coord.bz), sizeof(int));
            f.write(reinterpret_cast<const char*>(brick.words), sizeof(brick.words));
        }
    }
}

void LECT::load_hull_grids(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f)
        throw std::runtime_error("LECT::load_hull_grids: cannot open " + path);

    // ── Read header ──
    char header[HULL_HEADER_SIZE];
    f.read(header, HULL_HEADER_SIZE);
    if (!f)
        throw std::runtime_error("LECT::load_hull_grids: truncated header");

    uint32_t magic = *reinterpret_cast<uint32_t*>(header);
    if (magic != HULL_MAGIC)
        throw std::runtime_error("LECT::load_hull_grids: invalid HUL1 magic");

    int64_t n_alloc = *reinterpret_cast<int64_t*>(header + 16);
    double delta    = *reinterpret_cast<double*>(header + 24);
    voxel_delta_    = delta;

    // ── Read node data ──
    hull_grids_.resize(n_alloc);
    hull_valid_.resize(n_alloc, 0);

    for (int64_t i = 0; i < n_alloc; ++i) {
        uint8_t valid;
        f.read(reinterpret_cast<char*>(&valid), 1);
        hull_valid_[i] = valid;

        if (!valid) continue;

        int32_t n_bricks;
        f.read(reinterpret_cast<char*>(&n_bricks), sizeof(n_bricks));

        hull_grids_[i] = voxel::VoxelGrid(delta);

        for (int32_t j = 0; j < n_bricks; ++j) {
            voxel::BrickCoord coord;
            voxel::BitBrick brick;
            f.read(reinterpret_cast<char*>(&coord.bx), sizeof(int));
            f.read(reinterpret_cast<char*>(&coord.by), sizeof(int));
            f.read(reinterpret_cast<char*>(&coord.bz), sizeof(int));
            f.read(reinterpret_cast<char*>(brick.words), sizeof(brick.words));
            hull_grids_[i].set_brick(coord, brick);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────
//  Statistics
// ─────────────────────────────────────────────────────────────────────────
int LECT::count_nodes_with_aabb() const {
    int count = 0;
    for (int i = 0; i < store_.n_nodes(); ++i)
        if (store_.has_aabb(i)) ++count;
    return count;
}

int LECT::count_nodes_with_hull() const {
    int count = 0;
    int n = std::min(store_.n_nodes(), static_cast<int>(hull_valid_.size()));
    for (int i = 0; i < n; ++i)
        if (hull_valid_[i]) ++count;
    return count;
}

int LECT::total_hull_voxels() const {
    int total = 0;
    int n = std::min(store_.n_nodes(), static_cast<int>(hull_valid_.size()));
    for (int i = 0; i < n; ++i)
        if (hull_valid_[i])
            total += hull_grids_[i].count_occupied();
    return total;
}

int LECT::scene_grid_voxels() const {
    if (!scene_set_) return 0;
    return scene_grid_.count_occupied();
}

} // namespace forest
} // namespace sbf

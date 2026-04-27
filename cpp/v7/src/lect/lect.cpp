/// @file lect.cpp
/// @brief LECT implementation — v7 P3 minimal port.

#include "sbf/lect/lect.h"

#include "sbf/core/endpoint_iaabb.h"
#include "sbf/core/fk_state.h"
#include "sbf/envelope/link_iaabb.h"
#include "sbf/scene/collision.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <type_traits>

namespace sbf::lect {

using sbf::core::FKState;
using sbf::core::Interval;
using sbf::core::Robot;

namespace {

constexpr char kLectDiskMagic[8] = {'S', 'B', 'F', 'L', 'E', 'C', 'T', '\0'};
constexpr std::uint32_t kLectDiskVersion = 2;

struct LectDiskHeader {
    char magic[8];
    std::uint32_t version;
    std::uint64_t robot_fingerprint;
    std::int32_t n_dims;
    std::int32_t n_active_links;
    std::int32_t n_subdivisions;
    std::int32_t n_nodes;
    std::int32_t initial_cap;
    std::uint8_t env_type;
    std::uint8_t split_order;
    std::uint8_t endpoint_source;
    std::uint8_t reserved = 0;
    double grid_voxel_delta;
    double min_norm_width;
};

static_assert(std::is_trivially_copyable_v<LectDiskHeader>);
static_assert(std::is_trivially_copyable_v<Interval>);

template <typename T>
void write_pod(std::ofstream& out, const T& value) {
    static_assert(std::is_trivially_copyable_v<T>);
    out.write(reinterpret_cast<const char*>(&value), sizeof(T));
    if (!out) throw std::runtime_error("LECT::save_binary: write failed");
}

template <typename T>
void write_pod_array(std::ofstream& out, const T* data, std::size_t count) {
    static_assert(std::is_trivially_copyable_v<T>);
    if (count == 0) return;
    out.write(reinterpret_cast<const char*>(data),
              static_cast<std::streamsize>(count * sizeof(T)));
    if (!out) throw std::runtime_error("LECT::save_binary: write failed");
}

template <typename T>
T read_pod(std::ifstream& in) {
    static_assert(std::is_trivially_copyable_v<T>);
    T value{};
    in.read(reinterpret_cast<char*>(&value), sizeof(T));
    if (!in) throw std::runtime_error("LECT::load_binary: read failed");
    return value;
}

template <typename T>
void read_pod_array(std::ifstream& in, T* data, std::size_t count) {
    static_assert(std::is_trivially_copyable_v<T>);
    if (count == 0) return;
    in.read(reinterpret_cast<char*>(data),
            static_cast<std::streamsize>(count * sizeof(T)));
    if (!in) throw std::runtime_error("LECT::load_binary: read failed");
}

void check_or_throw(bool cond, const char* msg) {
    if (!cond) throw std::runtime_error(msg);
}

}  // namespace

// ════════════════════════════════════════════════════════════════════
//  Construction
// ════════════════════════════════════════════════════════════════════

LECT::LECT(const Robot& robot,
           const std::vector<Interval>& root_intervals,
           const sbf::envelope::EnvelopeTypeConfig& env_config,
           LECTConfig cfg)
    : robot_(robot),
      env_config_(env_config),
      root_intervals_(root_intervals),
      cfg_(cfg),
      n_dims_(static_cast<int>(root_intervals.size())),
      n_active_links_(robot.n_active_links())
{
    if (n_dims_ <= 0)
        throw std::invalid_argument("LECT: empty root intervals");
    if (n_active_links_ <= 0)
        throw std::invalid_argument("LECT: robot has no active links");

    n_subdivisions_ = std::max(1, env_config.n_subdivisions);
    ep_stride_      = n_active_links_ * 2 * 6;
    liaabb_stride_  = n_active_links_ * n_subdivisions_ * 6;

    // Cache D1 inflation radii broadcast to every per-slot entry.
    // Layout: [n_active_links_ * n_subdivisions_]; each link's radius
    // is repeated n_subdivisions_ times so it can be applied inline by
    // `aabbs_collide_obs_inflated`.
    const int n_slots_total = n_active_links_ * n_subdivisions_;
    radii_per_slot_.assign(n_slots_total, 0.0f);
    if (const double* r = robot.active_link_radii()) {
        for (int ci = 0; ci < n_active_links_; ++ci) {
            const float rf = static_cast<float>(r[ci]);
            for (int s = 0; s < n_subdivisions_; ++s)
                radii_per_slot_[ci * n_subdivisions_ + s] = rf;
        }
    }

    ensure_capacity(std::max(1, cfg_.initial_cap));

    // Allocate root.
    int root = alloc_node();
    assert(root == 0);
    parent_[root] = -1;
    depth_[root]  = 0;

    // Compute root envelope.
    auto root_fk_res = sbf::core::compute_endpoint_iaabb(
        robot_, root_intervals_, cfg_.endpoint_source,
        /*fk_cache=*/nullptr, /*changed_dim=*/-1);

    std::memcpy(ep_write(root),
                root_fk_res.endpoint_iaabbs.data(),
                ep_stride_ * sizeof(float));
    has_data_[root] = 1;
    link_iaabb_dirty_[root] = 1;

    // Initial subtree free volume = root box volume.
    subtree_free_volume_[root] = interval_volume(root_intervals_);
}

// ════════════════════════════════════════════════════════════════════
//  Capacity / allocation
// ════════════════════════════════════════════════════════════════════

void LECT::ensure_capacity(int min_cap) {
    if (capacity_ >= min_cap) return;
    int new_cap = capacity_ > 0 ? capacity_ : 1;
    while (new_cap < min_cap) new_cap *= 2;

    left_.resize(new_cap, -1);
    right_.resize(new_cap, -1);
    parent_.resize(new_cap, -1);
    depth_.resize(new_cap, 0);
    split_dim_.resize(new_cap, -1);
    split_val_.resize(new_cap, 0.0);
    has_data_.resize(new_cap, 0);
    ep_data_.resize(static_cast<size_t>(new_cap) * ep_stride_, 0.0f);
    link_iaabb_cache_.resize(static_cast<size_t>(new_cap) * liaabb_stride_, 0.0f);
    link_iaabb_dirty_.resize(new_cap, 1);
    forest_id_.resize(new_cap, -1);
    subtree_occ_.resize(new_cap, 0);
    subtree_free_volume_.resize(new_cap, 0.0);
    capacity_ = new_cap;
}

int LECT::alloc_node() {
    if (n_nodes_ >= capacity_) ensure_capacity(capacity_ * 2);
    int idx = n_nodes_++;
    left_[idx] = right_[idx] = -1;
    parent_[idx] = -1;
    depth_[idx] = 0;
    split_dim_[idx] = -1;
    split_val_[idx] = 0.0;
    has_data_[idx] = 0;
    link_iaabb_dirty_[idx] = 1;
    forest_id_[idx] = -1;
    subtree_occ_[idx] = 0;
    subtree_free_volume_[idx] = 0.0;
    return idx;
}

// ════════════════════════════════════════════════════════════════════
//  Snapshot
// ════════════════════════════════════════════════════════════════════

LECT LECT::snapshot() const {
    LECT clone(*this);                      // deep copy via vectors
    clone.snapshot_base_ = clone.n_nodes_;
    return clone;
}

void LECT::save_binary(const std::string& path) const {
    std::filesystem::path out_path(path);
    if (out_path.has_parent_path()) {
        std::filesystem::create_directories(out_path.parent_path());
    }

    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out.is_open()) {
        throw std::runtime_error("LECT::save_binary: cannot open output file");
    }

    LectDiskHeader header{};
    std::memcpy(header.magic, kLectDiskMagic, sizeof(kLectDiskMagic));
    header.version           = kLectDiskVersion;
    header.robot_fingerprint = robot_.fingerprint();
    header.n_dims            = n_dims_;
    header.n_active_links    = n_active_links_;
    header.n_subdivisions    = n_subdivisions_;
    header.n_nodes           = n_nodes_;
    header.initial_cap       = std::max(capacity_, cfg_.initial_cap);
    header.env_type          = static_cast<std::uint8_t>(env_config_.type);
    header.split_order       = static_cast<std::uint8_t>(cfg_.split_order);
    header.endpoint_source   = static_cast<std::uint8_t>(cfg_.endpoint_source.kind);
    header.grid_voxel_delta  = env_config_.grid_config.voxel_delta;
    header.min_norm_width    = cfg_.min_norm_width;

    write_pod(out, header);
    write_pod_array(out, root_intervals_.data(), root_intervals_.size());
    write_pod_array(out, left_.data(), static_cast<std::size_t>(n_nodes_));
    write_pod_array(out, right_.data(), static_cast<std::size_t>(n_nodes_));
    write_pod_array(out, parent_.data(), static_cast<std::size_t>(n_nodes_));
    write_pod_array(out, depth_.data(), static_cast<std::size_t>(n_nodes_));
    write_pod_array(out, split_dim_.data(), static_cast<std::size_t>(n_nodes_));
    write_pod_array(out, split_val_.data(), static_cast<std::size_t>(n_nodes_));
    write_pod_array(out, has_data_.data(), static_cast<std::size_t>(n_nodes_));
    write_pod_array(out, ep_data_.data(),
                    static_cast<std::size_t>(n_nodes_) * ep_stride_);
    write_pod_array(out, forest_id_.data(), static_cast<std::size_t>(n_nodes_));
    write_pod_array(out, subtree_occ_.data(), static_cast<std::size_t>(n_nodes_));
    write_pod_array(out, subtree_free_volume_.data(),
                    static_cast<std::size_t>(n_nodes_));
}

LECT LECT::load_binary(const std::string& path, const Robot& robot) {
    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) {
        throw std::runtime_error("LECT::load_binary: cannot open input file");
    }

    LectDiskHeader header = read_pod<LectDiskHeader>(in);
    check_or_throw(std::memcmp(header.magic, kLectDiskMagic,
                               sizeof(kLectDiskMagic)) == 0,
                   "LECT::load_binary: bad file magic");
    check_or_throw(header.version == 1 || header.version == kLectDiskVersion,
                   "LECT::load_binary: unsupported file version");
    check_or_throw(header.n_nodes >= 1,
                   "LECT::load_binary: invalid node count");
    check_or_throw(header.n_dims >= 1,
                   "LECT::load_binary: invalid dimension count");
    check_or_throw(header.n_subdivisions >= 1,
                   "LECT::load_binary: invalid subdivision count");
    check_or_throw(header.robot_fingerprint == robot.fingerprint(),
                   "LECT::load_binary: robot fingerprint mismatch");
    check_or_throw(header.n_dims == robot.n_joints(),
                   "LECT::load_binary: robot dimension mismatch");
    check_or_throw(header.n_active_links == robot.n_active_links(),
                   "LECT::load_binary: active-link layout mismatch");

    std::vector<Interval> root_intervals(static_cast<std::size_t>(header.n_dims));
    read_pod_array(in, root_intervals.data(), root_intervals.size());

    sbf::envelope::EnvelopeTypeConfig env_config;
    env_config.type = static_cast<sbf::envelope::EnvelopeType>(header.env_type);
    env_config.n_subdivisions = header.n_subdivisions;
    env_config.grid_config.voxel_delta = header.grid_voxel_delta;

    LECTConfig cfg;
    cfg.split_order = static_cast<SplitOrder>(header.split_order);
    cfg.initial_cap = std::max(header.initial_cap, header.n_nodes);
    cfg.min_norm_width = header.min_norm_width;
    cfg.endpoint_source.kind = header.version >= 2
        ? static_cast<sbf::core::EndpointSourceKind>(header.endpoint_source)
        : sbf::core::EndpointSourceKind::IFK;

    LECT tree(robot, root_intervals, env_config, cfg);
    tree.ensure_capacity(std::max(header.initial_cap, header.n_nodes));
    tree.n_nodes_ = header.n_nodes;
    tree.snapshot_base_ = 0;

    read_pod_array(in, tree.left_.data(), static_cast<std::size_t>(tree.n_nodes_));
    read_pod_array(in, tree.right_.data(), static_cast<std::size_t>(tree.n_nodes_));
    read_pod_array(in, tree.parent_.data(), static_cast<std::size_t>(tree.n_nodes_));
    read_pod_array(in, tree.depth_.data(), static_cast<std::size_t>(tree.n_nodes_));
    read_pod_array(in, tree.split_dim_.data(), static_cast<std::size_t>(tree.n_nodes_));
    read_pod_array(in, tree.split_val_.data(), static_cast<std::size_t>(tree.n_nodes_));
    read_pod_array(in, tree.has_data_.data(), static_cast<std::size_t>(tree.n_nodes_));
    read_pod_array(in, tree.ep_data_.data(),
                   static_cast<std::size_t>(tree.n_nodes_) * tree.ep_stride_);
    read_pod_array(in, tree.forest_id_.data(), static_cast<std::size_t>(tree.n_nodes_));
    read_pod_array(in, tree.subtree_occ_.data(), static_cast<std::size_t>(tree.n_nodes_));
    read_pod_array(in, tree.subtree_free_volume_.data(),
                   static_cast<std::size_t>(tree.n_nodes_));

    std::fill(tree.link_iaabb_dirty_.begin(), tree.link_iaabb_dirty_.end(), 1);
    std::fill(tree.link_iaabb_cache_.begin(), tree.link_iaabb_cache_.end(), 0.0f);
    return tree;
}

// ════════════════════════════════════════════════════════════════════
//  partition_for_seeds  (P4.5)
// ════════════════════════════════════════════════════════════════════

std::vector<int> LECT::partition_for_seeds(
    const std::vector<Eigen::VectorXd>& seeds,
    int max_extra_splits) {

    int n = static_cast<int>(seeds.size());
    std::vector<int> leaf_for(n, -1);
    if (n == 0) return leaf_for;

    // Convert seeds to flat double[n_dims_] arrays.
    auto seed_ptr = [&](int i) -> const double* {
        return seeds[i].data();
    };

    // Phase 1: split conflict leaves until all seeds in distinct leaves.
    int splits_used = 0;
    while (splits_used < max_extra_splits) {
        std::unordered_map<int, std::vector<int>> leaf_to_seeds;
        for (int i = 0; i < n; ++i) {
            leaf_for[i] = find_leaf_containing(seed_ptr(i));
            leaf_to_seeds[leaf_for[i]].push_back(i);
        }
        int conflict = -1;
        for (const auto& kv : leaf_to_seeds)
            if (kv.second.size() > 1) { conflict = kv.first; break; }
        if (conflict < 0) break;
        int n_before = n_nodes_;
        expand_leaf(conflict);
        if (n_nodes_ == n_before) break;     // split made no progress
        ++splits_used;
    }

    // Phase 2: LCA-promote — push each leaf upward to largest ancestor whose
    // subtree contains no OTHER seed's leaf.
    std::vector<int> domain_for = leaf_for;
    for (int i = 0; i < n; ++i) {
        int cur = leaf_for[i];
        while (true) {
            int p = parent_[cur];
            if (p < 0) break;
            bool other_inside = false;
            for (int j = 0; j < n; ++j) {
                if (j == i) continue;
                if (is_descendant_of(leaf_for[j], p)) { other_inside = true; break; }
            }
            if (other_inside) break;
            cur = p;
        }
        domain_for[i] = cur;
    }
    return domain_for;
}

// ════════════════════════════════════════════════════════════════════
//  transplant_domain  (P4.5)
// ════════════════════════════════════════════════════════════════════

int LECT::transplant_domain(const LECT& worker, int domain_root_idx,
                            const std::unordered_map<int,int>& id_map,
                            std::unordered_map<int,int>& node_remap) {
    if (domain_root_idx < 0 || domain_root_idx >= worker.n_nodes_) return 0;
    int snap_base = worker.snapshot_base_;

    // BFS phase: allocate master indices for every node in worker.subtree(domain_root).
    std::vector<int> bfs;
    bfs.reserve(64);
    bfs.push_back(domain_root_idx);
    node_remap.clear();
    node_remap[domain_root_idx] = domain_root_idx;     // domain root identity-mapped

    for (size_t bi = 0; bi < bfs.size(); ++bi) {
        int wi = bfs[bi];
        int wl = worker.left_[wi];
        int wr = worker.right_[wi];
        if (wl >= 0 && !node_remap.count(wl)) {
            int mi = (wl < snap_base) ? wl : alloc_node();
            node_remap[wl] = mi;
            bfs.push_back(wl);
        }
        if (wr >= 0 && !node_remap.count(wr)) {
            int mi = (wr < snap_base) ? wr : alloc_node();
            node_remap[wr] = mi;
            bfs.push_back(wr);
        }
    }

    // Copy phase with pointer remap.
    int n_shipped = 0;
    for (int wi : bfs) {
        int mi = node_remap[wi];
        bool is_new = (wi >= snap_base);

        // Tree pointers and split info (always remap; worker may have
        // promoted an inherited leaf into an internal node by expand_leaf).
        int wl = worker.left_[wi];
        int wr = worker.right_[wi];
        left_[mi]  = (wl < 0) ? -1 : node_remap.at(wl);
        right_[mi] = (wr < 0) ? -1 : node_remap.at(wr);
        split_dim_[mi] = worker.split_dim_[wi];
        split_val_[mi] = worker.split_val_[wi];
        if (mi != domain_root_idx) {
            int wp = worker.parent_[wi];
            auto it = (wp >= 0) ? node_remap.find(wp) : node_remap.end();
            parent_[mi] = (it != node_remap.end()) ? it->second : worker.parent_[wi];
        }

        if (is_new) {
            depth_[mi]     = worker.depth_[wi];
            has_data_[mi]  = worker.has_data_[wi];
            if (worker.has_data_[wi]) {
                std::memcpy(ep_write(mi), worker.ep_read(wi),
                            ep_stride_ * sizeof(float));
            }
            link_iaabb_dirty_[mi] = 1;
            ++n_shipped;
        }

        // Occupation (always — workers may newly mark inherited leaves too).
        int wfid = worker.forest_id_[wi];
        if (wfid >= 0) {
            auto it = id_map.find(wfid);
            forest_id_[mi] = (it != id_map.end()) ? it->second : wfid;
        }
    }
    return n_shipped;
}

// ════════════════════════════════════════════════════════════════════
//  Node intervals (walk root → i)
// ════════════════════════════════════════════════════════════════════

std::vector<Interval> LECT::node_intervals(int i) const {
    auto iv = root_intervals_;
    if (i == 0) return iv;

    // Walk from root down to i; collect path of (parent, child_branch).
    std::vector<int> path;
    int cur = i;
    while (cur != 0) {
        path.push_back(cur);
        cur = parent_[cur];
    }
    std::reverse(path.begin(), path.end());

    for (int child : path) {
        int p = parent_[child];
        int d = split_dim_[p];
        double v = split_val_[p];
        if (d < 0 || d >= n_dims_) continue;
        if (child == left_[p]) {
            iv[d].hi = v;
        } else {
            iv[d].lo = v;
        }
    }
    return iv;
}

// ════════════════════════════════════════════════════════════════════
//  Link AABB lazy materialisation
// ════════════════════════════════════════════════════════════════════

void LECT::materialise_link_iaabb(int i) const {
    float* out = link_iaabb_cache_.data()
                 + static_cast<size_t>(i) * liaabb_stride_;
    if (n_subdivisions_ <= 1) {
        sbf::envelope::derive_link_iaabb_paired_zero(
            ep_read(i), n_active_links_, out);
    } else {
        sbf::envelope::derive_link_iaabb_subdivided_zero(
            ep_read(i), n_active_links_, n_subdivisions_, out);
    }
    link_iaabb_dirty_[i] = 0;
}

const float* LECT::get_link_iaabbs(int i) const {
    if (link_iaabb_dirty_[i]) materialise_link_iaabb(i);
    return link_iaabb_cache_.data()
           + static_cast<size_t>(i) * liaabb_stride_;
}

// ════════════════════════════════════════════════════════════════════
//  Envelope computation (zero-radius, P1+P2)
// ════════════════════════════════════════════════════════════════════

void LECT::compute_envelope(int node_idx,
                            const FKState& fk,
                            const std::vector<Interval>& intervals,
                            int changed_dim,
                            int /*parent_idx*/) {
    FKState fk_local = fk;
    auto res = sbf::core::compute_endpoint_iaabb(
        robot_, intervals, cfg_.endpoint_source, &fk_local, changed_dim);
    std::memcpy(ep_write(node_idx),
                res.endpoint_iaabbs.data(),
                ep_stride_ * sizeof(float));
    has_data_[node_idx]        = 1;
    link_iaabb_dirty_[node_idx] = 1;
}

// ════════════════════════════════════════════════════════════════════
//  Split-dim selection
// ════════════════════════════════════════════════════════════════════

int LECT::pick_split_dim(int /*depth*/,
                         const std::vector<Interval>& iv) const {
    // WIDEST_FIRST: pick dim with largest normalised width relative to root.
    int best = -1;
    double best_w = -1.0;
    for (int d = 0; d < n_dims_; ++d) {
        double root_w = root_intervals_[d].width();
        if (root_w <= 0) continue;
        double norm_w = iv[d].width() / root_w;
        if (norm_w < cfg_.min_norm_width) continue;
        if (norm_w > best_w) {
            best_w = norm_w;
            best = d;
        }
    }
    if (best < 0) {
        // Fallback: pick any dim with positive width.
        for (int d = 0; d < n_dims_; ++d) {
            if (iv[d].width() > 0) return d;
        }
    }
    return best;
}

// ════════════════════════════════════════════════════════════════════
//  Split leaf
// ════════════════════════════════════════════════════════════════════

void LECT::split_leaf_impl(int node_idx,
                           const FKState& parent_fk,
                           const std::vector<Interval>& parent_iv) {
    int d = pick_split_dim(depth_[node_idx], parent_iv);
    if (d < 0) return;
    double mid = parent_iv[d].center();

    // Make sure we have room for two more nodes; this can grow ep_data_,
    // so re-derive `parent_fk` is unaffected (it's a value copy).
    ensure_capacity(n_nodes_ + 2);

    int li = alloc_node();
    int ri = alloc_node();

    parent_[li] = parent_[ri] = node_idx;
    depth_[li]  = depth_[ri]  = depth_[node_idx] + 1;

    split_dim_[node_idx] = d;
    split_val_[node_idx] = mid;
    left_[node_idx]  = li;
    right_[node_idx] = ri;

    // Build child intervals.
    auto left_iv  = parent_iv; left_iv[d].hi  = mid;
    auto right_iv = parent_iv; right_iv[d].lo = mid;

    compute_envelope(li, parent_fk, left_iv,  d, node_idx);
    compute_envelope(ri, parent_fk, right_iv, d, node_idx);

    // Phase U: redistribute parent free volume to children.
    if (forest_id_[node_idx] >= 0) {
        // Parent occupied → both children inherit 0 free volume.
        subtree_free_volume_[li] = 0.0;
        subtree_free_volume_[ri] = 0.0;
    } else {
        double lv = interval_volume(left_iv);
        double rv = interval_volume(right_iv);
        subtree_free_volume_[li] = lv;
        subtree_free_volume_[ri] = rv;
        // Parent total stays = lv + rv (== parent volume).
        subtree_free_volume_[node_idx] = lv + rv;
    }
}

int LECT::expand_leaf(int node_idx) {
    if (!is_leaf(node_idx)) return 0;
    auto iv = node_intervals(node_idx);
    FKState fk = sbf::core::compute_fk_full(robot_, iv);
    split_leaf_impl(node_idx, fk, iv);
    return 2;
}

// ════════════════════════════════════════════════════════════════════
//  Collision (D1)
// ════════════════════════════════════════════════════════════════════

bool LECT::collides_scene(int node_idx,
                          const float* obs_compact, int n_obs) const {
    if (n_obs <= 0) return false;
    const float* zero_aabbs = get_link_iaabbs(node_idx);
    return sbf::scene::aabbs_collide_obs_inflated(
        zero_aabbs, n_active_links_ * n_subdivisions_,
        radii_per_slot_.data(),
        obs_compact, n_obs);
}

// ════════════════════════════════════════════════════════════════════
//  Geometric queries
// ════════════════════════════════════════════════════════════════════

int LECT::find_leaf_containing(const double* q) const {
    int cur = 0;
    while (!is_leaf(cur)) {
        int d = split_dim_[cur];
        double v = split_val_[cur];
        if (d < 0 || d >= n_dims_) return cur;
        cur = (q[d] <= v) ? left_[cur] : right_[cur];
        if (cur < 0) return -1;
    }
    return cur;
}

bool LECT::is_descendant_of(int node, int ancestor) const {
    int cur = node;
    while (cur >= 0) {
        if (cur == ancestor) return true;
        cur = parent_[cur];
    }
    return false;
}

// ════════════════════════════════════════════════════════════════════
//  Occupation
// ════════════════════════════════════════════════════════════════════

void LECT::mark_occupied(int node_idx, int box_id) {
    if (forest_id_[node_idx] >= 0) return;        // already occupied
    forest_id_[node_idx] = box_id;
    double removed = subtree_free_volume_[node_idx];
    subtree_free_volume_[node_idx] = 0.0;
    int cur = node_idx;
    while (cur >= 0) {
        subtree_occ_[cur] += 1;
        if (cur != node_idx) subtree_free_volume_[cur] -= removed;
        cur = parent_[cur];
    }
}

void LECT::unmark_occupied(int node_idx) {
    if (forest_id_[node_idx] < 0) return;
    forest_id_[node_idx] = -1;
    // Restore free volume = node's own box volume (assumes node has no
    // occupied descendants — true when the leaf is occupied directly).
    double restored = node_box_volume(node_idx);
    subtree_free_volume_[node_idx] = restored;
    int cur = parent_[node_idx];
    while (cur >= 0) {
        subtree_occ_[cur] -= 1;
        subtree_free_volume_[cur] += restored;
        cur = parent_[cur];
    }
    subtree_occ_[node_idx] -= 1;
}

void LECT::clear_all_occupation() {
    std::fill(forest_id_.begin(), forest_id_.end(), -1);
    std::fill(subtree_occ_.begin(), subtree_occ_.end(), 0);
    refresh_free_volumes();
}

bool LECT::is_point_occupied(const double* q) const {
    int cur = 0;
    while (cur >= 0) {
        if (forest_id_[cur] >= 0) return true;
        if (is_leaf(cur)) return false;
        int d = split_dim_[cur];
        double v = split_val_[cur];
        if (d < 0 || d >= n_dims_) return false;
        cur = (q[d] <= v) ? left_[cur] : right_[cur];
    }
    return false;
}

void LECT::mark_point_occupied(const double* q, int box_id) {
    int leaf = find_leaf_containing(q);
    if (leaf >= 0) mark_occupied(leaf, box_id);
}

// ════════════════════════════════════════════════════════════════════
//  Free-volume tracking
// ════════════════════════════════════════════════════════════════════

double LECT::interval_volume(const std::vector<Interval>& iv) {
    double v = 1.0;
    for (const auto& x : iv) v *= std::max(0.0, x.width());
    return v;
}

double LECT::node_box_volume(int i) const {
    return interval_volume(node_intervals(i));
}

void LECT::refresh_free_volumes() {
    // Post-order over n_nodes_ (children always have larger indices than parent
    // in this tree thanks to alloc_node ordering).
    for (int i = n_nodes_ - 1; i >= 0; --i) {
        if (is_leaf(i)) {
            subtree_free_volume_[i] =
                (forest_id_[i] >= 0) ? 0.0 : node_box_volume(i);
            subtree_occ_[i] = (forest_id_[i] >= 0) ? 1 : 0;
        } else {
            int li = left_[i], ri = right_[i];
            subtree_free_volume_[i] =
                subtree_free_volume_[li] + subtree_free_volume_[ri];
            subtree_occ_[i] = subtree_occ_[li] + subtree_occ_[ri]
                              + ((forest_id_[i] >= 0) ? 1 : 0);
            if (forest_id_[i] >= 0) {
                subtree_free_volume_[i] = 0.0;
            }
        }
    }
}

}  // namespace sbf::lect

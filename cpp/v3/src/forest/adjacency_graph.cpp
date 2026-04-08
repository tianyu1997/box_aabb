// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v3 — AdjacencyGraph implementation
//  Sweep-and-prune accelerated box adjacency graph.
//  With optional AVX2 SIMD for the adjacency kernel.
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/forest/adjacency_graph.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <queue>
#include <unordered_set>

// ── SIMD detection ──────────────────────────────────────────────────────────
#if defined(__AVX2__)
#define SBF_HAS_AVX2 1
#include <immintrin.h>
#endif

namespace sbf {
namespace forest {

// Static empty neighbor list for missing box queries.
const std::vector<int> AdjacencyGraph::empty_neighbors_;

// ─────────────────────────────────────────────────────────────────────────
//  Construction
// ─────────────────────────────────────────────────────────────────────────
AdjacencyGraph::AdjacencyGraph(const JointLimits& limits, double tol)
    : limits_(limits)
    , tol_(tol)
    , n_dims_(limits.n_dims())
    , padded_dims_(((limits.n_dims() + 3) / 4) * 4)
{
    // Initialize sweep/filter dims (will be refined when boxes are added)
    if (n_dims_ >= 2) {
        sweep_dim_ = 0;
        filter_dim_ = 1;
        dim_order_.reserve(n_dims_);
        for (int d = 2; d < n_dims_; ++d)
            dim_order_.push_back(d);
    } else if (n_dims_ == 1) {
        sweep_dim_ = 0;
        filter_dim_ = 0;
    }
}

// ═════════════════════════════════════════════════════════════════════════
//  rank_dimensions — select sweep/filter dims by fill ratio
// ═════════════════════════════════════════════════════════════════════════
void AdjacencyGraph::rank_dimensions() {
    if (n_dims_ < 2 || box_map_.empty()) return;

    // Compute average box width per dimension
    std::vector<double> avg_width(n_dims_, 0.0);
    for (const auto& [id, box] : box_map_) {
        for (int d = 0; d < n_dims_; ++d) {
            avg_width[d] += box.joint_intervals[d].width();
        }
    }
    double n = static_cast<double>(box_map_.size());
    for (int d = 0; d < n_dims_; ++d)
        avg_width[d] /= n;

    // fill_ratio = avg_width / joint_range  (lower = more spread = better pruning)
    std::vector<double> fill_ratio(n_dims_);
    for (int d = 0; d < n_dims_; ++d) {
        double range = limits_.limits[d].width();
        fill_ratio[d] = (range > 1e-15) ? avg_width[d] / range : 1.0;
    }

    // Sort dimensions by fill ratio (ascending)
    std::vector<int> sorted_dims(n_dims_);
    std::iota(sorted_dims.begin(), sorted_dims.end(), 0);
    std::sort(sorted_dims.begin(), sorted_dims.end(),
              [&](int a, int b) { return fill_ratio[a] < fill_ratio[b]; });

    sweep_dim_  = sorted_dims[0];
    filter_dim_ = sorted_dims[1];

    dim_order_.clear();
    for (int i = 2; i < n_dims_; ++i)
        dim_order_.push_back(sorted_dims[i]);
}

// ═════════════════════════════════════════════════════════════════════════
//  rebuild_flat_arrays — pack box intervals into cache-friendly layout
// ═════════════════════════════════════════════════════════════════════════
void AdjacencyGraph::rebuild_flat_arrays() {
    int N = static_cast<int>(box_map_.size());
    if (padded_dims_ == 0)
        padded_dims_ = ((n_dims_ + 3) / 4) * 4;

    flat_lo_.resize(N * padded_dims_);
    flat_hi_.resize(N * padded_dims_);
    flat_ids_.resize(N);
    id_to_flat_.clear();

    int fi = 0;
    for (const auto& [id, box] : box_map_) {
        flat_ids_[fi] = id;
        id_to_flat_[id] = fi;
        int base = fi * padded_dims_;
        for (int d = 0; d < n_dims_; ++d) {
            flat_lo_[base + d] = box.joint_intervals[d].lo;
            flat_hi_[base + d] = box.joint_intervals[d].hi;
        }
        // Padding: neutral values so overlap = 1.0 >> tol (non-touching, non-separated)
        for (int d = n_dims_; d < padded_dims_; ++d) {
            flat_lo_[base + d] = 0.0;
            flat_hi_[base + d] = 1.0;
        }
        fi++;
    }
}

// ═════════════════════════════════════════════════════════════════════════
//  sort_sweep_order — sort flat indices by sweep dimension lo value
// ═════════════════════════════════════════════════════════════════════════
void AdjacencyGraph::sort_sweep_order() {
    int N = static_cast<int>(flat_ids_.size());
    sweep_order_.resize(N);
    std::iota(sweep_order_.begin(), sweep_order_.end(), 0);
    std::sort(sweep_order_.begin(), sweep_order_.end(),
              [this](int a, int b) { return sweep_lo(a) < sweep_lo(b); });
}

// ═════════════════════════════════════════════════════════════════════════
//  check_adjacent_flat — full adjacency test (all dims)
//  AVX2 SIMD: processes 4 doubles at once; scalar fallback otherwise.
//  Padding dims filled with lo=0, hi=1 → overlap=1.0 (neutral).
// ═════════════════════════════════════════════════════════════════════════
#ifdef SBF_HAS_AVX2

bool AdjacencyGraph::check_adjacent_flat(int fi, int fj) const {
    const int PD = padded_dims_;
    const double* lo_i = &flat_lo_[fi * PD];
    const double* hi_i = &flat_hi_[fi * PD];
    const double* lo_j = &flat_lo_[fj * PD];
    const double* hi_j = &flat_hi_[fj * PD];

    const __m256d neg_tol_v = _mm256_set1_pd(-tol_);
    const __m256d pos_tol_v = _mm256_set1_pd(tol_);

    int any_separated = 0;
    int any_touching  = 0;

    for (int d = 0; d < PD; d += 4) {
        __m256d hi_iv = _mm256_loadu_pd(hi_i + d);
        __m256d hi_jv = _mm256_loadu_pd(hi_j + d);
        __m256d lo_iv = _mm256_loadu_pd(lo_i + d);
        __m256d lo_jv = _mm256_loadu_pd(lo_j + d);

        // overlap[k] = min(hi_i[k], hi_j[k]) - max(lo_i[k], lo_j[k])
        __m256d min_hi  = _mm256_min_pd(hi_iv, hi_jv);
        __m256d max_lo  = _mm256_max_pd(lo_iv, lo_jv);
        __m256d overlap = _mm256_sub_pd(min_hi, max_lo);

        // separated in any dim: overlap < -tol
        any_separated |= _mm256_movemask_pd(
            _mm256_cmp_pd(overlap, neg_tol_v, _CMP_LT_OQ));

        // touching in any dim: overlap <= tol
        any_touching |= _mm256_movemask_pd(
            _mm256_cmp_pd(overlap, pos_tol_v, _CMP_LE_OQ));
    }

    if (any_separated) return false;
    return any_touching != 0;
}

#else  // scalar fallback (padded layout)

bool AdjacencyGraph::check_adjacent_flat(int fi, int fj) const {
    const int PD = padded_dims_;
    const double* lo_i = &flat_lo_[fi * PD];
    const double* hi_i = &flat_hi_[fi * PD];
    const double* lo_j = &flat_lo_[fj * PD];
    const double* hi_j = &flat_hi_[fj * PD];

    bool has_touching = false;

    for (int d = 0; d < n_dims_; ++d) {
        double overlap = std::min(hi_i[d], hi_j[d])
                       - std::max(lo_i[d], lo_j[d]);
        if (overlap < -tol_) return false;
        if (overlap <= tol_) has_touching = true;
    }

    return has_touching;
}

#endif  // SBF_HAS_AVX2

// ═════════════════════════════════════════════════════════════════════════
//  Edge management
// ═════════════════════════════════════════════════════════════════════════
void AdjacencyGraph::add_edge(int a, int b) {
    adj_[a].push_back(b);
    adj_[b].push_back(a);
    n_edges_++;
}

void AdjacencyGraph::remove_edge(int a, int b) {
    auto& na = adj_[a];
    na.erase(std::remove(na.begin(), na.end(), b), na.end());
    auto& nb = adj_[b];
    nb.erase(std::remove(nb.begin(), nb.end(), a), nb.end());
    n_edges_--;
}

// ═════════════════════════════════════════════════════════════════════════
//  rebuild — full batch sweep-and-prune
// ═════════════════════════════════════════════════════════════════════════
void AdjacencyGraph::rebuild(const std::vector<BoxNode>& boxes) {
    // Clear existing
    box_map_.clear();
    adj_.clear();
    n_edges_ = 0;
    kd_dirty_ = true;

    if (boxes.empty()) return;

    // Infer n_dims from first box if not set
    if (n_dims_ == 0) {
        n_dims_ = boxes[0].n_dims();
    }
    padded_dims_ = ((n_dims_ + 3) / 4) * 4;

    // Populate box_map
    for (const auto& box : boxes) {
        box_map_[box.id] = box;
        adj_[box.id];  // ensure entry
    }

    // Rank dimensions & build flat arrays
    rank_dimensions();
    rebuild_flat_arrays();
    sort_sweep_order();

    // Sweep-and-prune
    int N = static_cast<int>(sweep_order_.size());
    int sweep_candidates = 0;
    int filter_pass = 0;
    const int PD = padded_dims_;

    for (int i = 0; i < N; ++i) {
        int fi = sweep_order_[i];
        double hi_sweep_i = sweep_hi(fi);

        for (int j = i + 1; j < N; ++j) {
            int fj = sweep_order_[j];

            // Sweep-dim prune: sorted break
            if (sweep_lo(fj) > hi_sweep_i + tol_)
                break;

            sweep_candidates++;

            // Filter-dim prune
            const double* lo_i = &flat_lo_[fi * PD];
            const double* hi_i = &flat_hi_[fi * PD];
            const double* lo_j = &flat_lo_[fj * PD];
            const double* hi_j = &flat_hi_[fj * PD];

            if (filter_dim_ != sweep_dim_) {
                if (lo_j[filter_dim_] > hi_i[filter_dim_] + tol_ ||
                    lo_i[filter_dim_] > hi_j[filter_dim_] + tol_)
                    continue;
            }

            filter_pass++;

            // Full adjacency check (SIMD-accelerated when AVX2 available)
            if (check_adjacent_flat(fi, fj)) {
                int id_i = flat_ids_[fi];
                int id_j = flat_ids_[fj];
                add_edge(id_i, id_j);
            }
        }
    }

    int brute = N * (N - 1) / 2;
    std::printf("[adjacency] rebuild N=%d sweep=%d filter=%d | "
                "sweep_cand=%d filter_pass=%d pairs=%d | brute=%d\n",
                N, sweep_dim_, filter_dim_,
                sweep_candidates, filter_pass, n_edges_, brute);
}

// ═════════════════════════════════════════════════════════════════════════
//  add_box — incremental insertion with binary-search scan
// ═════════════════════════════════════════════════════════════════════════
void AdjacencyGraph::add_box(const BoxNode& box) {
    if (box_map_.count(box.id)) return;  // already present

    // Infer n_dims from first box if not set
    if (n_dims_ == 0) {
        n_dims_ = box.n_dims();
        padded_dims_ = ((n_dims_ + 3) / 4) * 4;
        rank_dimensions();
    }

    box_map_[box.id] = box;
    adj_[box.id];  // ensure entry
    kd_dirty_ = true;  // KD-tree needs rebuild

    // Append to flat arrays (padded layout)
    int new_fi = static_cast<int>(flat_ids_.size());
    flat_ids_.push_back(box.id);
    id_to_flat_[box.id] = new_fi;
    for (int d = 0; d < n_dims_; ++d) {
        flat_lo_.push_back(box.joint_intervals[d].lo);
        flat_hi_.push_back(box.joint_intervals[d].hi);
    }
    // Padding: neutral values
    for (int d = n_dims_; d < padded_dims_; ++d) {
        flat_lo_.push_back(0.0);
        flat_hi_.push_back(1.0);
    }

    double new_sweep_lo = sweep_lo(new_fi);
    double new_sweep_hi = sweep_hi(new_fi);

    // Binary search for insertion position in sweep_order
    auto it = std::lower_bound(
        sweep_order_.begin(), sweep_order_.end(), new_sweep_lo,
        [this](int fi, double val) { return sweep_lo(fi) < val; });
    int pos = static_cast<int>(it - sweep_order_.begin());
    sweep_order_.insert(it, new_fi);

    // Scan left: indices < pos where their sweep_hi >= new_sweep_lo - tol
    for (int k = pos - 1; k >= 0; --k) {
        int fj = sweep_order_[k];
        if (sweep_hi(fj) < new_sweep_lo - tol_)
            break;
        if (check_adjacent_flat(new_fi, fj)) {
            add_edge(box.id, flat_ids_[fj]);
        }
    }

    // Scan right: indices > pos where their sweep_lo <= new_sweep_hi + tol
    for (int k = pos + 1; k < static_cast<int>(sweep_order_.size()); ++k) {
        int fj = sweep_order_[k];
        if (sweep_lo(fj) > new_sweep_hi + tol_)
            break;
        if (check_adjacent_flat(new_fi, fj)) {
            add_edge(box.id, flat_ids_[fj]);
        }
    }

    // Re-rank dimensions periodically (every 256 boxes)
    if ((box_map_.size() & 0xFF) == 0 && box_map_.size() >= 8) {
        rank_dimensions();
        // Full rebuild of flat arrays + sweep order after re-ranking
        rebuild_flat_arrays();
        sort_sweep_order();
    }
}

// ═════════════════════════════════════════════════════════════════════════
//  remove_box
// ═════════════════════════════════════════════════════════════════════════
void AdjacencyGraph::remove_box(int box_id) {
    auto it = box_map_.find(box_id);
    if (it == box_map_.end()) return;

    kd_dirty_ = true;  // KD-tree needs rebuild

    // Remove all edges
    if (adj_.count(box_id)) {
        auto nbrs = adj_[box_id];  // copy
        for (int nb : nbrs) {
            remove_edge(box_id, nb);
        }
        adj_.erase(box_id);
    }

    box_map_.erase(it);

    // Rebuild flat arrays (simpler than patching)
    rebuild_flat_arrays();
    sort_sweep_order();
}

// ═════════════════════════════════════════════════════════════════════════
//  clear
// ═════════════════════════════════════════════════════════════════════════
void AdjacencyGraph::clear() {
    box_map_.clear();
    adj_.clear();
    n_edges_ = 0;
    flat_lo_.clear();
    flat_hi_.clear();
    flat_ids_.clear();
    id_to_flat_.clear();
    sweep_order_.clear();
    kd_tree_.clear();
    kd_dirty_ = true;
}

// ═════════════════════════════════════════════════════════════════════════
//  neighbors
// ═════════════════════════════════════════════════════════════════════════
const std::vector<int>& AdjacencyGraph::neighbors(int box_id) const {
    auto it = adj_.find(box_id);
    if (it != adj_.end()) return it->second;
    return empty_neighbors_;
}

// ═════════════════════════════════════════════════════════════════════════
//  connected — BFS reachability
// ═════════════════════════════════════════════════════════════════════════
bool AdjacencyGraph::connected(int a, int b) const {
    if (a == b) return true;
    if (!box_map_.count(a) || !box_map_.count(b)) return false;

    std::unordered_set<int> visited;
    std::queue<int> queue;
    queue.push(a);
    visited.insert(a);

    while (!queue.empty()) {
        int cur = queue.front();
        queue.pop();
        for (int nb : neighbors(cur)) {
            if (nb == b) return true;
            if (visited.insert(nb).second) {
                queue.push(nb);
            }
        }
    }
    return false;
}

// ═════════════════════════════════════════════════════════════════════════
//  n_components
// ═════════════════════════════════════════════════════════════════════════
int AdjacencyGraph::n_components() const {
    return static_cast<int>(components().size());
}

// ═════════════════════════════════════════════════════════════════════════
//  components — all connected components (BFS)
// ═════════════════════════════════════════════════════════════════════════
std::vector<std::vector<int>> AdjacencyGraph::components() const {
    std::vector<std::vector<int>> result;
    std::unordered_set<int> visited;

    for (const auto& [id, _] : box_map_) {
        if (visited.count(id)) continue;

        std::vector<int> comp;
        std::queue<int> queue;
        queue.push(id);
        visited.insert(id);

        while (!queue.empty()) {
            int cur = queue.front();
            queue.pop();
            comp.push_back(cur);
            for (int nb : neighbors(cur)) {
                if (visited.insert(nb).second) {
                    queue.push(nb);
                }
            }
        }
        result.push_back(std::move(comp));
    }
    return result;
}

// ═════════════════════════════════════════════════════════════════════════
//  find_nearest_box — KD-tree accelerated (O(log N) avg)
// ═════════════════════════════════════════════════════════════════════════
int AdjacencyGraph::find_nearest_box(const Eigen::VectorXd& q) const {
    if (box_map_.empty()) return -1;

    // Lazy rebuild of KD-tree when dirty
    if (kd_dirty_) {
        std::vector<BoxNode> all_boxes;
        all_boxes.reserve(box_map_.size());
        for (const auto& [id, box] : box_map_)
            all_boxes.push_back(box);
        kd_tree_.build(all_boxes);
        kd_dirty_ = false;
    }

    return kd_tree_.find_nearest(q);
}

} // namespace forest
} // namespace sbf

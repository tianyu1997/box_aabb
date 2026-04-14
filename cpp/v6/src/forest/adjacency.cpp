// SafeBoxForest v6 — Adjacency (Phase F6)
#include <sbf/forest/adjacency.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>

namespace sbf {

// ─── boxes_adjacent: single source of truth for box adjacency ───────────────
bool boxes_adjacent(const BoxNode& a, const BoxNode& b, double tol) {
    const int nd = a.n_dims();
    if (nd != b.n_dims()) return false;

    int shared_dims = 0;   // face contact (overlap < tol)
    int overlap_dims = 0;  // genuine overlap (overlap >= tol)

    for (int d = 0; d < nd; ++d) {
        double overlap_lo = std::max(a.joint_intervals[d].lo, b.joint_intervals[d].lo);
        double overlap_hi = std::min(a.joint_intervals[d].hi, b.joint_intervals[d].hi);

        if (overlap_hi < overlap_lo - tol)
            return false;  // separated

        if (overlap_hi - overlap_lo < tol)
            shared_dims++;   // face contact / near-touching
        else
            overlap_dims++;  // genuine overlap in this dim
    }

    // Adjacent: face contact in 1+ dims OR full volumetric overlap
    return (shared_dims >= 1) || (overlap_dims == nd);
}

// ─── shared_face ────────────────────────────────────────────────────────────
std::optional<SharedFace> shared_face(
        const BoxNode& a, const BoxNode& b, double tol) {
    const int nd = a.n_dims();
    if (nd != b.n_dims()) return std::nullopt;

    int shared_dims = 0;
    int overlap_dims = 0;
    int last_shared_dim = -1;
    double last_shared_val = 0.0;

    for (int d = 0; d < nd; ++d) {
        double overlap_lo = std::max(a.joint_intervals[d].lo, b.joint_intervals[d].lo);
        double overlap_hi = std::min(a.joint_intervals[d].hi, b.joint_intervals[d].hi);

        if (overlap_hi < overlap_lo - tol)
            return std::nullopt;  // separated

        if (overlap_hi - overlap_lo < tol) {
            shared_dims++;
            last_shared_dim = d;
            last_shared_val = 0.5 * (overlap_lo + overlap_hi);
        } else {
            overlap_dims++;
        }
    }

    // Full volumetric overlap → virtual face at shortest-overlap dimension
    if (overlap_dims == nd) {
        double min_overlap = std::numeric_limits<double>::max();
        int min_dim = 0;
        for (int d = 0; d < nd; ++d) {
            double ov = std::min(a.joint_intervals[d].hi, b.joint_intervals[d].hi)
                      - std::max(a.joint_intervals[d].lo, b.joint_intervals[d].lo);
            if (ov < min_overlap) { min_overlap = ov; min_dim = d; }
        }
        SharedFace sf;
        sf.dim = min_dim;
        sf.value = 0.5 * (std::max(a.joint_intervals[min_dim].lo,
                                     b.joint_intervals[min_dim].lo) +
                           std::min(a.joint_intervals[min_dim].hi,
                                     b.joint_intervals[min_dim].hi));
        for (int d = 0; d < nd; ++d) {
            if (d == min_dim) continue;
            double lo = std::max(a.joint_intervals[d].lo, b.joint_intervals[d].lo);
            double hi = std::min(a.joint_intervals[d].hi, b.joint_intervals[d].hi);
            sf.face_ivs.push_back({lo, hi});
        }
        return sf;
    }

    // Face contact: need shared_dims >= 1 (same criterion as boxes_adjacent)
    if (shared_dims == 0) return std::nullopt;

    SharedFace sf;
    sf.dim = last_shared_dim;
    sf.value = last_shared_val;

    for (int d = 0; d < nd; ++d) {
        if (d == last_shared_dim) continue;
        double lo = std::max(a.joint_intervals[d].lo, b.joint_intervals[d].lo);
        double hi = std::min(a.joint_intervals[d].hi, b.joint_intervals[d].hi);
        sf.face_ivs.push_back({lo, hi});
    }
    return sf;
}

AdjacencyGraph compute_adjacency(
        const std::vector<BoxNode>& boxes, double tol, int max_degree) {
    AdjacencyGraph adj;
    const int n = static_cast<int>(boxes.size());

    // Initialize empty lists for all boxes
    for (const auto& b : boxes)
        adj[b.id] = {};

    if (n <= 1) return adj;

    const int nd = boxes[0].n_dims();

    // ── Step 1: Pick optimal sweep dimension ────────────────────────
    // For each dimension, sort boxes by lo and count 1D-overlapping
    // pairs via binary search.  The dimension with the fewest overlaps
    // gives the best pruning in the sweep phase.
    // Cost: O(D · N log N).

    struct DimInfo {
        std::vector<int>    idx;     // box indices sorted by lo[d]
        std::vector<double> lo_vals; // extracted sorted lo values
        long long           pairs;   // # of 1D-overlapping pairs
    };
    std::vector<DimInfo> dims(nd);

    for (int d = 0; d < nd; ++d) {
        auto& di = dims[d];
        di.idx.resize(n);
        std::iota(di.idx.begin(), di.idx.end(), 0);
        std::sort(di.idx.begin(), di.idx.end(), [&](int a, int b) {
            return boxes[a].joint_intervals[d].lo
                 < boxes[b].joint_intervals[d].lo;
        });

        di.lo_vals.resize(n);
        for (int k = 0; k < n; ++k)
            di.lo_vals[k] = boxes[di.idx[k]].joint_intervals[d].lo;

        di.pairs = 0;
        for (int si = 0; si < n; ++si) {
            double hi = boxes[di.idx[si]].joint_intervals[d].hi;
            auto it = std::upper_bound(
                di.lo_vals.begin() + si + 1, di.lo_vals.end(), hi + tol);
            di.pairs += static_cast<long long>(
                it - (di.lo_vals.begin() + si + 1));
        }
    }

    // Dimension indices sorted by ascending pair count (most discriminating
    // first).  sweep_dim = dim_order[0] has the fewest 1D overlaps.
    // Remaining dimensions are checked in discriminating order for early exit.
    std::vector<int> dim_order(nd);
    std::iota(dim_order.begin(), dim_order.end(), 0);
    std::sort(dim_order.begin(), dim_order.end(), [&](int a, int b) {
        return dims[a].pairs < dims[b].pairs;
    });

    // ── Step 2: Sweep on the best dimension ─────────────────────────
    // For each candidate pair (1D-overlapping on sweep_dim), verify the
    // remaining D−1 dimensions in discriminating order.  Two boxes are
    // adjacent iff they are non-separated on every dimension — equivalent
    // to boxes_adjacent() since (shared_dims≥1 || overlap_dims==nd) is
    // always true once no dimension is separated.
    // Cost: O(P₀ · D)  where P₀ = dims[sweep_dim].pairs.

    const int sweep_dim = dim_order[0];
    const auto& order = dims[sweep_dim].idx;

    for (int si = 0; si < n; ++si) {
        const int i = order[si];
        const auto& ai = boxes[i].joint_intervals;
        const double hi_sweep = ai[sweep_dim].hi;

        for (int sj = si + 1; sj < n; ++sj) {
            const int j = order[sj];
            if (boxes[j].joint_intervals[sweep_dim].lo > hi_sweep + tol)
                break;

            const auto& bj = boxes[j].joint_intervals;
            bool separated = false;
            for (int dk = 1; dk < nd; ++dk) {
                const int d = dim_order[dk];
                if (std::min(ai[d].hi, bj[d].hi) <
                    std::max(ai[d].lo, bj[d].lo) - tol) {
                    separated = true;
                    break;
                }
            }

            if (!separated) {
                adj[boxes[i].id].push_back(boxes[j].id);
                adj[boxes[j].id].push_back(boxes[i].id);
            }
        }
    }

    // ── Step 3 (optional): Degree pruning ───────────────────────────
    // Keep only top-K neighbors per box, ranked by face overlap volume
    // (product of overlap widths on non-shared dimensions).  Larger
    // overlap faces → better connectivity for Dijkstra traversal.
    // An edge is kept if EITHER endpoint ranks the other in its top-K,
    // preserving the undirected graph invariant.
    if (max_degree > 0) {
        // Build id → box index map
        std::unordered_map<int, int> id_to_idx;
        id_to_idx.reserve(n);
        for (int i = 0; i < n; ++i)
            id_to_idx[boxes[i].id] = i;

        // Phase 1: compute each node's top-K set
        std::unordered_map<int, std::unordered_set<int>> keep_sets;
        for (auto& [box_id, nbrs] : adj) {
            if (static_cast<int>(nbrs.size()) <= max_degree) {
                keep_sets[box_id] = std::unordered_set<int>(nbrs.begin(), nbrs.end());
                continue;
            }

            auto it_a = id_to_idx.find(box_id);
            if (it_a == id_to_idx.end()) continue;
            const auto& a = boxes[it_a->second];

            // Compute overlap volume proxy for each neighbor
            std::vector<std::pair<double, int>> scored;
            scored.reserve(nbrs.size());
            for (int nid : nbrs) {
                auto it_b = id_to_idx.find(nid);
                if (it_b == id_to_idx.end()) { scored.push_back({0.0, nid}); continue; }
                const auto& b = boxes[it_b->second];
                double vol = 1.0;
                for (int d = 0; d < nd; ++d) {
                    double ov = std::min(a.joint_intervals[d].hi, b.joint_intervals[d].hi)
                              - std::max(a.joint_intervals[d].lo, b.joint_intervals[d].lo);
                    if (ov > 0) vol *= ov;
                }
                scored.push_back({vol, nid});
            }

            std::partial_sort(scored.begin(),
                              scored.begin() + max_degree,
                              scored.end(),
                              [](const auto& a, const auto& b) {
                                  return a.first > b.first;
                              });
            auto& ks = keep_sets[box_id];
            for (int k = 0; k < max_degree; ++k)
                ks.insert(scored[k].second);
        }

        // Phase 2: rebuild adj — keep edge if either endpoint wants it
        for (auto& [box_id, nbrs] : adj) {
            auto& my_ks = keep_sets[box_id];
            std::vector<int> kept;
            kept.reserve(nbrs.size());
            for (int nid : nbrs) {
                if (my_ks.count(nid) || keep_sets[nid].count(box_id))
                    kept.push_back(nid);
            }
            nbrs = std::move(kept);
        }
    }

    return adj;
}

// ─── Articulation Point Detection (Tarjan's algorithm) ──────────────────────
std::unordered_set<int> find_articulation_points(const AdjacencyGraph& adj) {
    std::unordered_set<int> result;
    if (adj.empty()) return result;

    // Map box IDs to dense indices
    std::vector<int> ids;
    ids.reserve(adj.size());
    std::unordered_map<int, int> id_to_idx;
    for (const auto& kv : adj) {
        id_to_idx[kv.first] = static_cast<int>(ids.size());
        ids.push_back(kv.first);
    }
    int n = static_cast<int>(ids.size());

    std::vector<int> disc(n, -1), low(n, -1), parent(n, -1);
    std::vector<bool> is_ap(n, false);
    int timer = 0;

    // DFS from each unvisited node (handles multiple components)
    std::function<void(int)> dfs = [&](int u) {
        disc[u] = low[u] = timer++;
        int children = 0;

        auto it = adj.find(ids[u]);
        if (it == adj.end()) return;

        for (int neighbor_id : it->second) {
            auto nit = id_to_idx.find(neighbor_id);
            if (nit == id_to_idx.end()) continue;
            int v = nit->second;

            if (disc[v] == -1) {
                children++;
                parent[v] = u;
                dfs(v);
                low[u] = std::min(low[u], low[v]);

                // u is AP if: (1) root with 2+ children, or
                //              (2) non-root with low[v] >= disc[u]
                if (parent[u] == -1 && children > 1)
                    is_ap[u] = true;
                if (parent[u] != -1 && low[v] >= disc[u])
                    is_ap[u] = true;
            } else if (v != parent[u]) {
                low[u] = std::min(low[u], disc[v]);
            }
        }
    };

    for (int i = 0; i < n; ++i) {
        if (disc[i] == -1)
            dfs(i);
    }

    for (int i = 0; i < n; ++i) {
        if (is_ap[i])
            result.insert(ids[i]);
    }

    return result;
}

}  // namespace sbf

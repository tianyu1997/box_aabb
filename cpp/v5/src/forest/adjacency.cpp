// SafeBoxForest v5 — Adjacency (Phase F6)
#include <sbf/forest/adjacency.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>

namespace sbf {

std::optional<SharedFace> shared_face(
        const BoxNode& a, const BoxNode& b, double tol) {
    const int nd = a.n_dims();
    if (nd != b.n_dims()) return std::nullopt;

    int n_touching = 0;
    int n_overlapping = 0;
    int touch_dim = -1;
    double touch_val = 0.0;
    int touch_side = -1;  // 0 = a.hi touches b.lo, 1 = b.hi touches a.lo

    for (int d = 0; d < nd; ++d) {
        double a_lo = a.joint_intervals[d].lo;
        double a_hi = a.joint_intervals[d].hi;
        double b_lo = b.joint_intervals[d].lo;
        double b_hi = b.joint_intervals[d].hi;

        // Separation check
        if (a_hi + tol < b_lo || b_hi + tol < a_lo)
            return std::nullopt;

        // Touching check
        bool touch_a_hi = std::abs(a_hi - b_lo) <= tol;
        bool touch_b_hi = std::abs(b_hi - a_lo) <= tol;
        if (touch_a_hi || touch_b_hi) {
            n_touching++;
            touch_dim = d;
            if (touch_a_hi) {
                touch_val = 0.5 * (a_hi + b_lo);
                touch_side = 0;
            } else {
                touch_val = 0.5 * (b_hi + a_lo);
                touch_side = 1;
            }
        }

        // Overlap width in this dimension
        double overlap = std::min(a_hi, b_hi) - std::max(a_lo, b_lo);
        if (overlap > tol)
            n_overlapping++;
    }

    // Adjacency: boxes that share a face OR volumetrically overlap
    // Face-sharing: at least 1 touching dim, at least D-1 overlapping dims
    // Overlap: all D dims overlap (one box partially/fully inside another)
    if (n_overlapping == nd) {
        // Full overlap — create a virtual face along the shortest overlap dim
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

    if (n_touching == 0) return std::nullopt;
    if (n_overlapping < nd - 1) return std::nullopt;

    // Use the first touching dimension found (there should be exactly 1 for
    // a proper shared face)
    SharedFace sf;
    sf.dim = touch_dim;
    sf.value = touch_val;

    // Compute shared face intervals (intersection of the non-touching dims)
    for (int d = 0; d < nd; ++d) {
        if (d == touch_dim) continue;
        double lo = std::max(a.joint_intervals[d].lo, b.joint_intervals[d].lo);
        double hi = std::min(a.joint_intervals[d].hi, b.joint_intervals[d].hi);
        sf.face_ivs.push_back({lo, hi});
    }
    return sf;
}

AdjacencyGraph compute_adjacency(
        const std::vector<BoxNode>& boxes, double tol) {
    AdjacencyGraph adj;
    const int n = static_cast<int>(boxes.size());

    // Initialize empty lists for all boxes
    for (const auto& b : boxes)
        adj[b.id] = {};

    if (n <= 1) return adj;

    const int nd = boxes[0].n_dims();

    // T1: 1D sweep-line pruning — O(N log N + K·D) instead of O(N²·D).
    // Two boxes can only be adjacent (shared_face OR full overlap) if they
    // are non-separated in ALL dimensions. We pick the "tightest" dimension
    // (smallest total interval span) as the sweep axis. Pairs separated on
    // this axis are pruned without checking the other D-1 dimensions.

    // 1. Select sweep dimension: smallest total span → fewest active pairs
    int sweep_dim = 0;
    {
        double best_span = std::numeric_limits<double>::max();
        for (int d = 0; d < nd; ++d) {
            double span = 0.0;
            for (const auto& b : boxes)
                span += b.joint_intervals[d].width();
            if (span < best_span) {
                best_span = span;
                sweep_dim = d;
            }
        }
    }

    // 2. Sort box indices by lo[sweep_dim]
    std::vector<int> order(n);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return boxes[a].joint_intervals[sweep_dim].lo
             < boxes[b].joint_intervals[sweep_dim].lo;
    });

    // 3. Sweep: for each box i, check only boxes j where lo_j <= hi_i + tol.
    //    Combined adjacency test (shared_face OR boxes_overlap) inline to
    //    avoid SharedFace allocation and double iteration over dimensions.
    for (int si = 0; si < n; ++si) {
        const int i = order[si];
        const double hi_i = boxes[i].joint_intervals[sweep_dim].hi;

        for (int sj = si + 1; sj < n; ++sj) {
            const int j = order[sj];
            const double lo_j = boxes[j].joint_intervals[sweep_dim].lo;

            // Prune: all subsequent boxes are also separated from i
            if (lo_j > hi_i + tol) break;

            // Inline adjacency test — single pass over D dimensions.
            // Computes shared_face criteria AND boxes_overlap criteria
            // simultaneously, avoiding SharedFace struct allocation.
            const auto& ai = boxes[i].joint_intervals;
            const auto& bi = boxes[j].joint_intervals;

            bool separated = false;
            bool all_strict_overlap = true;
            int n_touching = 0;
            int n_overlapping = 0;

            for (int d = 0; d < nd; ++d) {
                double a_lo = ai[d].lo, a_hi = ai[d].hi;
                double b_lo = bi[d].lo, b_hi = bi[d].hi;

                // Separation check (with tolerance) — shared_face criterion
                if (a_hi + tol < b_lo || b_hi + tol < a_lo) {
                    separated = true;
                    break;
                }

                // Strict overlap (no tolerance) — boxes_overlap criterion
                if (a_hi <= b_lo || b_hi <= a_lo)
                    all_strict_overlap = false;

                // Touching check
                if (std::abs(a_hi - b_lo) <= tol || std::abs(b_hi - a_lo) <= tol)
                    n_touching++;

                // Overlap beyond tolerance
                double overlap = std::min(a_hi, b_hi) - std::max(a_lo, b_lo);
                if (overlap > tol)
                    n_overlapping++;
            }

            if (separated) continue;

            // Adjacent if: full strict overlap OR full tol-overlap OR face-sharing
            bool adjacent = all_strict_overlap
                         || (n_overlapping == nd)
                         || (n_touching >= 1 && n_overlapping >= nd - 1);
            if (adjacent) {
                adj[boxes[i].id].push_back(boxes[j].id);
                adj[boxes[j].id].push_back(boxes[i].id);
            }
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

// SafeBoxForest v5 — Adjacency (Phase F6)
#include <sbf/forest/adjacency.h>

#include <algorithm>
#include <cmath>
#include <limits>

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

    // O(N^2) pairwise check
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (shared_face(boxes[i], boxes[j], tol).has_value()) {
                adj[boxes[i].id].push_back(boxes[j].id);
                adj[boxes[j].id].push_back(boxes[i].id);
            }
        }
    }
    return adj;
}

}  // namespace sbf

/// @file adjacency.cpp
#include "sbf/forest/adjacency.h"
#include "sbf/forest/union_find.h"

#include <algorithm>

namespace sbf::forest {

bool boxes_adjacent(const sbf::scene::BoxNode& a,
                    const sbf::scene::BoxNode& b,
                    double tol) {
    const int nd = a.n_dims();
    if (nd != b.n_dims() || nd == 0) return false;

    int shared_dims = 0;     // face contact (overlap < tol but ≥ 0)
    int overlap_dims = 0;    // genuine overlap (overlap >= tol)

    for (int d = 0; d < nd; ++d) {
        double lo = std::max(a.joint_intervals[d].lo, b.joint_intervals[d].lo);
        double hi = std::min(a.joint_intervals[d].hi, b.joint_intervals[d].hi);
        double w  = hi - lo;
        if (w < -tol) return false;          // genuinely separated
        if (w < tol)  ++shared_dims;
        else          ++overlap_dims;
    }

    // Adjacent: ≥ 1 face contact OR full volumetric overlap.
    return (shared_dims >= 1) || (overlap_dims == nd);
}

AdjacencyGraph compute_adjacency_graph(
    const std::vector<sbf::scene::BoxNode>& boxes, double tol) {
    AdjacencyGraph g;
    int n = static_cast<int>(boxes.size());
    g.nbrs.assign(n, {});
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (boxes_adjacent(boxes[i], boxes[j], tol)) {
                g.nbrs[i].push_back(j);
                ++g.n_edges;
            }
        }
    }
    return g;
}

IslandResult find_islands(const AdjacencyGraph& g) {
    IslandResult r;
    int n = g.n_nodes();
    UnionFind uf(n);
    for (int i = 0; i < n; ++i)
        for (int j : g.nbrs[i]) uf.unite(i, j);

    r.component_id.assign(n, -1);
    std::vector<int> root_to_id;
    std::vector<int> sizes;
    for (int i = 0; i < n; ++i) {
        int rt = uf.find(i);
        // map root → compact id
        int cid = -1;
        for (int k = 0; k < (int)root_to_id.size(); ++k) {
            if (root_to_id[k] == rt) { cid = k; break; }
        }
        if (cid < 0) {
            cid = static_cast<int>(root_to_id.size());
            root_to_id.push_back(rt);
            sizes.push_back(0);
        }
        r.component_id[i] = cid;
        ++sizes[cid];
    }
    r.n_components = static_cast<int>(root_to_id.size());
    r.largest_size = sizes.empty() ? 0 : *std::max_element(sizes.begin(), sizes.end());
    return r;
}

}  // namespace sbf::forest

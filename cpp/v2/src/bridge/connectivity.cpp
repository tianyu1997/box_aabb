// SafeBoxForest v2 — Connectivity implementation
// Module: sbf (bridge)
#include "sbf/forest/connectivity.h"
#include <algorithm>
#include <limits>
#include <queue>

namespace sbf {

// ─── UnionFind ──────────────────────────────────────────────────────────────
UnionFind::UnionFind(int n) : n_(n) {
    parent_.resize(n);
    rank_.resize(n, 0);
    for (int i = 0; i < n; ++i)
        parent_[i] = i;
}

int UnionFind::find(int x) {
    while (parent_[x] != x) {
        parent_[x] = parent_[parent_[x]];  // path halving
        x = parent_[x];
    }
    return x;
}

bool UnionFind::unite(int x, int y) {
    int rx = find(x), ry = find(y);
    if (rx == ry) return false;
    if (rank_[rx] < rank_[ry]) std::swap(rx, ry);
    parent_[ry] = rx;
    if (rank_[rx] == rank_[ry]) rank_[rx]++;
    return true;
}

bool UnionFind::connected(int x, int y) {
    return find(x) == find(y);
}

int UnionFind::n_components() const {
    std::unordered_set<int> roots;
    UnionFind* mutable_this = const_cast<UnionFind*>(this);
    for (int i = 0; i < n_; ++i)
        roots.insert(mutable_this->find(i));
    return static_cast<int>(roots.size());
}

std::unordered_map<int, std::vector<int>> UnionFind::components() const {
    std::unordered_map<int, std::vector<int>> result;
    UnionFind* mutable_this = const_cast<UnionFind*>(this);
    for (int i = 0; i < n_; ++i) {
        int root = mutable_this->find(i);
        result[root].push_back(i);
    }
    return result;
}

// ─── Find islands ───────────────────────────────────────────────────────────
std::unordered_map<int, std::unordered_set<int>>
find_islands(const std::unordered_map<int, std::vector<int>>& adjacency,
             const std::vector<int>& all_ids) {
    // Map IDs to indices
    std::unordered_map<int, int> id_to_idx;
    for (int i = 0; i < static_cast<int>(all_ids.size()); ++i)
        id_to_idx[all_ids[i]] = i;

    int n = static_cast<int>(all_ids.size());
    UnionFind uf(n);

    for (auto& [id, neighbors] : adjacency) {
        auto it = id_to_idx.find(id);
        if (it == id_to_idx.end()) continue;
        int idx = it->second;

        for (int nb : neighbors) {
            auto jt = id_to_idx.find(nb);
            if (jt == id_to_idx.end()) continue;
            uf.unite(idx, jt->second);
        }
    }

    // Collect components
    std::unordered_map<int, std::unordered_set<int>> islands;
    for (int i = 0; i < n; ++i) {
        int root = uf.find(i);
        islands[root].insert(all_ids[i]);
    }

    return islands;
}

// ─── Bridge islands ─────────────────────────────────────────────────────────
BridgeResult bridge_islands(
    const std::unordered_map<int, std::unordered_set<int>>& islands,
    const std::unordered_map<int, BoxNode>& boxes,
    int max_attempts) {
    BridgeResult result;

    if (islands.size() <= 1) {
        result.fully_connected = true;
        return result;
    }

    // Collect island representatives
    std::vector<int> island_ids;
    std::vector<std::vector<int>> island_members;
    for (auto& [rep, members] : islands) {
        island_ids.push_back(rep);
        std::vector<int> mem(members.begin(), members.end());
        island_members.push_back(std::move(mem));
    }

    int n_islands = static_cast<int>(island_ids.size());

    // Pre-compute island centroids
    int ndim = 0;
    for (auto& [id, bx] : boxes) { ndim = static_cast<int>(bx.joint_intervals.size()); break; }
    if (ndim == 0) return result;

    std::vector<Eigen::VectorXd> island_centroids(n_islands);
    for (int ia = 0; ia < n_islands; ++ia) {
        Eigen::VectorXd sum = Eigen::VectorXd::Zero(ndim);
        int cnt = 0;
        for (int bid : island_members[ia]) {
            auto it = boxes.find(bid);
            if (it == boxes.end()) continue;
            sum += it->second.center();
            cnt++;
        }
        island_centroids[ia] = (cnt > 0) ? Eigen::VectorXd(sum / cnt)
                                          : Eigen::VectorXd::Zero(ndim);
    }

    for (int attempt = 0; attempt < max_attempts && n_islands > 1; ++attempt) {
        // Find closest pair of islands by centroid distance
        double best_centroid_dist = std::numeric_limits<double>::max();
        int best_ia = -1, best_ib = -1;

        for (int ia = 0; ia < n_islands; ++ia) {
            for (int ib = ia + 1; ib < n_islands; ++ib) {
                double d = (island_centroids[ia] - island_centroids[ib]).norm();
                if (d < best_centroid_dist) {
                    best_centroid_dist = d;
                    best_ia = ia;
                    best_ib = ib;
                }
            }
        }
        if (best_ia < 0) break;

        // Frontier sampling: only check K closest boxes
        auto find_frontier = [&](int island_idx, const Eigen::VectorXd& target, int k) {
            struct BoxDist { double dist; int id; };
            std::vector<BoxDist> bd;
            bd.reserve(island_members[island_idx].size());
            for (int bid : island_members[island_idx]) {
                auto it = boxes.find(bid);
                if (it == boxes.end()) continue;
                bd.push_back({(it->second.center() - target).norm(), bid});
            }
            if (static_cast<int>(bd.size()) > k) {
                std::partial_sort(bd.begin(), bd.begin() + k, bd.end(),
                    [](const BoxDist& a, const BoxDist& b) { return a.dist < b.dist; });
                bd.resize(k);
            }
            return bd;
        };

        int frontier_k = std::min(50, std::max(10,
            static_cast<int>(std::max(island_members[best_ia].size(),
                                       island_members[best_ib].size())) / 10));

        auto front_a = find_frontier(best_ia, island_centroids[best_ib], frontier_k);
        auto front_b = find_frontier(best_ib, island_centroids[best_ia], frontier_k);

        double best_dist = std::numeric_limits<double>::max();
        int best_a = -1, best_b = -1;
        for (auto& fa : front_a) {
            auto it_a = boxes.find(fa.id);
            if (it_a == boxes.end()) continue;
            Eigen::VectorXd ca = it_a->second.center();
            for (auto& fb : front_b) {
                auto it_b = boxes.find(fb.id);
                if (it_b == boxes.end()) continue;
                double d = (ca - it_b->second.center()).norm();
                if (d < best_dist) {
                    best_dist = d;
                    best_a = fa.id;
                    best_b = fb.id;
                }
            }
        }

        if (best_a < 0) break;

        result.bridges.emplace_back(best_a, best_b);
        result.n_bridges++;

        // Merge islands
        int size_a = static_cast<int>(island_members[best_ia].size());
        int size_b = static_cast<int>(island_members[best_ib].size());
        island_centroids[best_ia] = (island_centroids[best_ia] * size_a +
                                      island_centroids[best_ib] * size_b) /
                                     (size_a + size_b);

        for (int id : island_members[best_ib])
            island_members[best_ia].push_back(id);
        island_members.erase(island_members.begin() + best_ib);
        island_ids.erase(island_ids.begin() + best_ib);
        island_centroids.erase(island_centroids.begin() + best_ib);
        n_islands--;
    }

    result.fully_connected = (n_islands <= 1);
    return result;
}

} // namespace sbf

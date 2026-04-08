// SafeBoxForest v5 — Connectivity (Phase F7)
#include <sbf/forest/connectivity.h>

#include <algorithm>
#include <limits>
#include <unordered_map>
#include <unordered_set>

#include <Eigen/Dense>

namespace sbf {

// ─── UnionFind ──────────────────────────────────────────────────────────────
UnionFind::UnionFind(int n) : parent_(n), rank_(n, 0) {
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

void UnionFind::unite(int x, int y) {
    int rx = find(x), ry = find(y);
    if (rx == ry) return;
    if (rank_[rx] < rank_[ry]) std::swap(rx, ry);
    parent_[ry] = rx;
    if (rank_[rx] == rank_[ry]) rank_[rx]++;
}

bool UnionFind::connected(int x, int y) {
    return find(x) == find(y);
}

// ─── find_islands ───────────────────────────────────────────────────────────
std::vector<std::vector<int>> find_islands(const AdjacencyGraph& adj) {
    // Collect all box ids
    std::vector<int> all_ids;
    all_ids.reserve(adj.size());
    for (const auto& kv : adj)
        all_ids.push_back(kv.first);

    if (all_ids.empty()) return {};

    // Map id → compact index
    std::unordered_map<int, int> id_to_idx;
    for (int i = 0; i < static_cast<int>(all_ids.size()); ++i)
        id_to_idx[all_ids[i]] = i;

    int n = static_cast<int>(all_ids.size());
    UnionFind uf(n);

    for (const auto& kv : adj) {
        int u = id_to_idx[kv.first];
        for (int neighbor_id : kv.second) {
            auto it = id_to_idx.find(neighbor_id);
            if (it != id_to_idx.end())
                uf.unite(u, it->second);
        }
    }

    // Group by root
    std::unordered_map<int, std::vector<int>> groups;
    for (int i = 0; i < n; ++i)
        groups[uf.find(i)].push_back(all_ids[i]);

    std::vector<std::vector<int>> islands;
    islands.reserve(groups.size());
    for (auto& kv : groups)
        islands.push_back(std::move(kv.second));

    return islands;
}

// ─── bridge_islands ─────────────────────────────────────────────────────────
int bridge_islands(
        std::vector<BoxNode>& boxes,
        LECT& lect,
        const Obstacle* obs, int n_obs,
        AdjacencyGraph& adj,
        const FFBConfig& ffb_config,
        int& next_box_id,
        std::chrono::steady_clock::time_point deadline) {
    auto islands = find_islands(adj);
    if (islands.size() <= 1) return 0;
    auto deadline_reached = [&]() {
        return deadline != std::chrono::steady_clock::time_point::max()
               && std::chrono::steady_clock::now() >= deadline;
    };

    // Build id → box index
    std::unordered_map<int, int> id_to_idx;
    for (int i = 0; i < static_cast<int>(boxes.size()); ++i)
        id_to_idx[boxes[i].id] = i;

    int bridges_created = 0;
    const int nd = boxes.empty() ? 0 : boxes[0].n_dims();
    constexpr int MAX_CANDIDATES_PER_PAIR = 100;

    for (int ia = 0; ia < static_cast<int>(islands.size()); ++ia) {
        if (deadline_reached()) return bridges_created;
        for (int ib = ia + 1; ib < static_cast<int>(islands.size()); ++ib) {
            if (deadline_reached()) return bridges_created;
            // Find closest pair of boxes between islands
            double best_dist = std::numeric_limits<double>::max();
            int best_a = -1, best_b = -1;

            for (int aid : islands[ia]) {
                auto ita = id_to_idx.find(aid);
                if (ita == id_to_idx.end()) continue;
                Eigen::VectorXd ca_tmp = boxes[ita->second].center();
                for (int bid : islands[ib]) {
                    auto itb = id_to_idx.find(bid);
                    if (itb == id_to_idx.end()) continue;
                    double d = (ca_tmp - boxes[itb->second].center()).squaredNorm();
                    if (d < best_dist) { best_dist = d; best_a = aid; best_b = bid; }
                }
            }
            if (best_a < 0 || best_b < 0) continue;

            Eigen::VectorXd ca = boxes[id_to_idx[best_a]].center();
            Eigen::VectorXd cb = boxes[id_to_idx[best_b]].center();
            auto root_ivs = lect.node_intervals(0);

            // Build candidate seeds (budget limited)
            std::vector<Eigen::VectorXd> candidates;
            candidates.reserve(MAX_CANDIDATES_PER_PAIR);

            // Strategy 1: direct line interpolation (9 seeds)
            for (double alpha : {0.5, 0.3, 0.7, 0.2, 0.8, 0.1, 0.9, 0.4, 0.6})
                candidates.push_back((1.0 - alpha) * ca + alpha * cb);

            // Strategy 2: perpendicular offsets (limited)
            Eigen::VectorXd dir = cb - ca;
            double dir_norm = dir.norm();
            if (dir_norm > 1e-12 && nd >= 2) {
                dir /= dir_norm;
                for (int d = 0; d < nd && static_cast<int>(candidates.size()) < MAX_CANDIDATES_PER_PAIR; ++d) {
                    Eigen::VectorXd perp = Eigen::VectorXd::Zero(nd);
                    perp[d] = 1.0;
                    perp -= dir * dir.dot(perp);
                    double pn = perp.norm();
                    if (pn < 1e-12) continue;
                    perp /= pn;
                    for (double offset : {0.5, 1.0, 2.0, -0.5, -1.0, -2.0}) {
                        for (double alpha : {0.3, 0.5, 0.7}) {
                            Eigen::VectorXd seed = (1.0 - alpha) * ca + alpha * cb + offset * perp;
                            for (int dd = 0; dd < nd; ++dd)
                                seed[dd] = std::clamp(seed[dd], root_ivs[dd].lo, root_ivs[dd].hi);
                            candidates.push_back(std::move(seed));
                        }
                    }
                }
            }

            // Try candidate seeds, limit to budget
            int tried = 0;
            for (const auto& seed : candidates) {
                if (tried++ >= MAX_CANDIDATES_PER_PAIR) break;
                if (deadline_reached()) return bridges_created;

                FFBResult ffb = find_free_box(lect, seed, obs, n_obs, ffb_config);
                if (!ffb.success() || lect.is_occupied(ffb.node_idx))
                    continue;

                BoxNode new_box;
                new_box.id = next_box_id++;
                new_box.joint_intervals = lect.node_intervals(ffb.node_idx);
                new_box.seed_config = seed;
                new_box.tree_id = ffb.node_idx;
                new_box.parent_box_id = -1;
                new_box.root_id = -1;
                new_box.compute_volume();

                lect.mark_occupied(ffb.node_idx, new_box.id);
                adj[new_box.id] = {};
                id_to_idx[new_box.id] = static_cast<int>(boxes.size());

                // Check adjacency with all existing boxes
                bool has_neighbor = false;
                for (const auto& b : boxes) {
                    if (shared_face(new_box, b).has_value()) {
                        adj[new_box.id].push_back(b.id);
                        adj[b.id].push_back(new_box.id);
                        has_neighbor = true;
                    }
                }

                if (!has_neighbor) {
                    // Don't keep isolated boxes — undo
                    lect.unmark_occupied(ffb.node_idx);
                    next_box_id--;
                    adj.erase(new_box.id);
                    continue;
                }

                boxes.push_back(std::move(new_box));
                bridges_created++;
            }
        }
    }
    return bridges_created;
}

}  // namespace sbf

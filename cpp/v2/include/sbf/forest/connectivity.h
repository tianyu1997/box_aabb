// SafeBoxForest v2 — Connectivity: UnionFind, island detection, bridging
// Module: sbf (bridge)
#pragma once

#include "sbf/common/types.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace sbf {

// ─── Array-based Union-Find (path compression + union by rank) ──────────────
class UnionFind {
public:
    UnionFind() = default;
    explicit UnionFind(int n);

    int find(int x);
    bool unite(int x, int y);  // returns true if actually merged
    bool connected(int x, int y);
    int n_components() const;

    // Get all components as {representative → {members}}
    std::unordered_map<int, std::vector<int>> components() const;

private:
    std::vector<int> parent_;
    std::vector<int> rank_;
    int n_ = 0;
};

// ─── Island detection ───────────────────────────────────────────────────────
// Given adjacency graph, find connected components (islands)
// Returns: {island_id → {box_ids}}
std::unordered_map<int, std::unordered_set<int>>
find_islands(const std::unordered_map<int, std::vector<int>>& adjacency,
             const std::vector<int>& all_ids);

// ─── Bridge islands ─────────────────────────────────────────────────────────
// Try to connect disconnected islands by finding closest box pairs
// between different islands and inserting bridge boxes.
struct BridgeResult {
    std::vector<std::pair<int, int>> bridges;  // (box_a, box_b) pairs
    int n_bridges = 0;
    bool fully_connected = false;
};

BridgeResult bridge_islands(
    const std::unordered_map<int, std::unordered_set<int>>& islands,
    const std::unordered_map<int, BoxNode>& boxes,
    int max_attempts = 20);

} // namespace sbf

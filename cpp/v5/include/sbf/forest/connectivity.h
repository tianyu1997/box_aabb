#pragma once
/// @file connectivity.h
/// @brief Connected-component analysis and inter-island bridging.
///
/// Uses Union-Find for fast island detection, then attempts to
/// bridge disconnected components by finding collision-free boxes
/// in the gap between the closest box pairs across islands.

#include <sbf/core/types.h>
#include <sbf/ffb/ffb.h>
#include <sbf/forest/adjacency.h>
#include <sbf/lect/lect.h>

#include <vector>
#include <chrono>

namespace sbf {

/// @brief Weighted Union-Find with path compression.
///
/// Used for O(α(n)) connected-component queries on the adjacency graph.
class UnionFind {
public:
    explicit UnionFind(int n);

    int  find(int x);                  ///< Find root representative.
    void unite(int x, int y);          ///< Merge components of x and y.
    bool connected(int x, int y);      ///< Test same-component membership.

private:
    std::vector<int> parent_;
    std::vector<int> rank_;
};

/// Decompose the adjacency graph into connected components ("islands").
/// Each inner vector contains the box IDs of one connected component.
std::vector<std::vector<int>> find_islands(const AdjacencyGraph& adj);

/// Attempt to bridge disconnected islands by growing FFB boxes in the
/// gap between closest box pairs.  Returns number of bridge boxes added.
int bridge_islands(
    std::vector<BoxNode>& boxes,
    LECT& lect,
    const Obstacle* obs, int n_obs,
    AdjacencyGraph& adj,
    const FFBConfig& ffb_config,
    int& next_box_id,
    std::chrono::steady_clock::time_point deadline = std::chrono::steady_clock::time_point::max());

}  // namespace sbf

// SafeBoxForest — Planner configuration
#pragma once

#include <string>
#include <vector>

namespace sbf {

struct SBFConfig {
    // Forest growth
    int max_boxes = 500;
    int max_consecutive_miss = 20;
    double ffb_min_edge = 0.01;
    double ffb_min_edge_anchor = 0.001;   // aggressive min_edge for start/goal anchor boxes
    double ffb_min_edge_relaxed = 0.05;   // relaxed min_edge after connectivity established
    int ffb_max_depth = 1000;

    // Proxy anchor: when start/goal is in a narrow passage where ffb_min_edge
    // cannot create a box, sample nearby configs that are directly reachable
    // and can host a normal-sized anchor box.
    int proxy_anchor_max_samples = 200;    // max sampling attempts per endpoint
    double proxy_anchor_radius = 0.3;      // max sampling radius

    // Multi-phase BFS min_edge decay (when not yet connected)
    // bfs_phase_k[i] * ffb_min_edge = effective min_edge for phase i
    // bfs_phase_budget[i] = max successful box creations before advancing to phase i+1
    // After all phases exhausted, stays at k=1 (i.e. ffb_min_edge)
    std::vector<double> bfs_phase_k     = {5.0, 2.0, 1.0};
    std::vector<int>    bfs_phase_budget = {100, 200, 200};
    int min_boxes_per_pair = 500;  // build_multi isolated phase: minimum boxes per pair
    int max_boxes_per_pair = 5000; // build_multi: keep growing isolated pair until connected or this limit

    // Sampling strategy
    double guided_sample_ratio = 0.6;
    double boundary_expand_epsilon = 0.01;

    // Connectivity
    int n_edge_samples = 3;
    int coarsen_max_rounds = 20;
    double adjacency_tol = 1e-10;

    // Greedy coarsen: merge adjacent boxes to reduce count
    int coarsen_target_boxes = 0;     // 0 = disabled; >0 = target box count
    int coarsen_greedy_rounds = 200;  // max rounds for greedy merge
    bool coarsen_grid_check = false;  // use tree-cached AABB for hull safety check
    int coarsen_split_depth = 3;       // max extra splits when leaf AABB collides
    int coarsen_max_tree_fk = 2000;   // FK budget per coarsen round (0=unlimited)

    // GCS
    int corridor_hops = 2;
    bool use_gcs = false;         // false → Dijkstra fallback

    // Path smoothing
    int shortcut_max_iters = 100;
    double segment_resolution = 0.05;

    // Parallel
    bool parallel_grow = false;
    int n_partitions_depth = 3;
    int parallel_workers = 4;

    // I/O
    bool use_cache = true;
    std::string cache_path;

    // Random seed
    int seed = 0;
};

struct PlannerDefaults {
    static constexpr double CONNECTION_RADIUS = 2.0;
    static constexpr double SEGMENT_RESOLUTION = 0.05;
    static constexpr int    MAX_CONNECT_ATTEMPTS = 50;
    static constexpr int    MAX_BRIDGE_ATTEMPTS = 20;
    static constexpr int    INITIAL_CACHE_CAP = 64;
};

} // namespace sbf

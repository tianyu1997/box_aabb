// SafeBoxForest v2 — Configuration structures for all modules
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace sbf {

// ─── Module-level config structures ─────────────────────────────────────────

namespace envelope {

// Main per-node data: AABB_LEGACY stores n_active_links×6 floats (HCACHE02),
// FRAMES stores n_stored_frames×6 floats (HCACHE03) — universal base for all
// envelope types.
enum class StoreFormat : uint8_t {
    AABB_LEGACY = 0,   // per-link AABB only (backward compat)
    FRAMES      = 1,   // per-frame position intervals (new default)
};

// Collision-checking strategy inside FFB descent.  Multiple strategies are
// implemented behind this switch so they can be benchmarked head-to-head.
enum class CollisionPolicy : uint8_t {
    AABB         = 0,  // per-link AABB 3-axis SAT (fastest, most conservative)
    AABB_SUBDIV  = 1,  // AABB + link subdivision fallback on collision
    GRID         = 2,  // occupancy grid from frames → bitwise AND
};

struct EnvelopeConfig {
    int max_depth = 1000;
    double min_edge = 0.01;
    double min_edge_anchor = 0.001;
    double min_edge_relaxed = 0.05;
    int promotion_depth = 2;

    // Recursive safety check depth for FFB descent.
    // 0 = legacy (check parent AABB only)
    // >0 = recurse into children's tighter AABBs (avoids union inflation)
    // Recommended: 2 (checks up to 4 grandchildren, ~7 SAT ops worst case)
    int safety_check_depth = 2;

    // ── HCACHE03 extensions ─────────────────────────────────────────
    StoreFormat     store_format     = StoreFormat::FRAMES;
    CollisionPolicy collision_policy = CollisionPolicy::AABB;

    // Link subdivision (used by AABB_SUBDIV policy)
    int link_subdivision_n   = 0;    // 0 = disabled, >0 = fixed n_sub, -1 = adaptive
    int link_subdivision_max = 8;    // max segments when adaptive (-1)

    // Grid (used by GRID policy and GridEnvelope sidecar)
    int   grid_resolution = 32;
    float grid_world_bounds[6] = {-0.8f, -1.2f, 0.0f, 1.8f, 1.2f, 1.4f};

    // Grid-union for FFB: bitfield occupancy grid stored per node.
    // When enabled, AABB collision → grid refinement (no false positives
    // from inflated union AABBs).
    bool use_grid_union = false;

    // Grid subdivision count for grid derive (per link)
    int grid_n_sub_per_link = 8;

    // ── Affine Arithmetic (AA) hybrid AABB ──────────────────────────
    // When max interval width ≤ aa_crossover_width, use Affine Arithmetic
    // for AABB extraction (tighter + faster at narrow intervals).
    // When max width > crossover, fall back to standard Interval Arithmetic.
    // Set to 0.0 to disable AA entirely.  Default: 0.5 (validated by scan).
    double aa_crossover_width = 0.5;
};

} // namespace envelope

namespace forest {
struct ForestConfig {
    int max_boxes = 500;
    int max_consecutive_miss = 20;

    // Multi-phase BFS min_edge decay
    std::vector<double> bfs_phase_k     = {5.0, 2.0, 1.0};
    std::vector<int>    bfs_phase_budget = {100, 200, 200};
    int min_boxes_per_pair = 500;
    int max_boxes_per_pair = 5000;

    // Sampling strategy
    double guided_sample_ratio = 0.6;
    double boundary_expand_epsilon = 0.01;

    // Connectivity
    int n_edge_samples = 3;
    double adjacency_tol = 1e-10;

    // Proxy anchor
    int proxy_anchor_max_samples = 200;
    double proxy_anchor_radius = 0.3;
};
} // namespace forest

namespace bridge {
struct BridgeConfig {
    int coarsen_max_rounds = 20;
    int coarsen_target_boxes = 0;
    int coarsen_greedy_rounds = 200;
    bool coarsen_grid_check = false;
    int coarsen_split_depth = 3;
    int coarsen_max_tree_fk = 2000;
    int corridor_hops = 2;
    bool use_gcs = false;
};
} // namespace bridge

namespace planner {
struct PlannerConfig {
    int shortcut_max_iters = 100;
    double segment_resolution = 0.05;
    bool parallel_grow = false;
    int n_partitions_depth = 3;
    int parallel_workers = 4;
    int seed = 0;
};
} // namespace planner

// ─── Aggregate config (backwards-compatible) ────────────────────────────────
struct SBFConfig {
    // Forest growth
    int max_boxes = 500;
    int max_consecutive_miss = 20;
    double ffb_min_edge = 0.01;
    double ffb_min_edge_anchor = 0.001;
    double ffb_min_edge_relaxed = 0.05;
    int ffb_max_depth = 1000;

    // Proxy anchor
    int proxy_anchor_max_samples = 200;
    double proxy_anchor_radius = 0.3;

    // Multi-phase BFS
    std::vector<double> bfs_phase_k     = {5.0, 2.0, 1.0};
    std::vector<int>    bfs_phase_budget = {100, 200, 200};
    int min_boxes_per_pair = 500;
    int max_boxes_per_pair = 5000;

    // Sampling strategy
    double guided_sample_ratio = 0.6;
    double boundary_expand_epsilon = 0.01;

    // Connectivity
    int n_edge_samples = 3;
    int coarsen_max_rounds = 20;
    double adjacency_tol = 1e-10;

    // Greedy coarsen
    int coarsen_target_boxes = 0;
    int coarsen_greedy_rounds = 200;
    bool coarsen_grid_check = false;
    int coarsen_split_depth = 3;
    int coarsen_max_tree_fk = 2000;

    // GCS
    int corridor_hops = 2;
    bool use_gcs = false;

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

    // ── HCACHE03 / multi-envelope ────────────────────────────────────
    envelope::StoreFormat     store_format     = envelope::StoreFormat::FRAMES;
    envelope::CollisionPolicy collision_policy = envelope::CollisionPolicy::AABB;
    int link_subdivision_n   = 0;
    int link_subdivision_max = 8;
    int grid_resolution      = 32;
    float grid_world_bounds[6] = {-0.8f, -1.2f, 0.0f, 1.8f, 1.2f, 1.4f};

    // ── AA hybrid AABB crossover ─────────────────────────────────────
    double aa_crossover_width = 0.5;  // 0 = disabled; 0.5 validated optimal

    // ── Decompose into module configs ────────────────────────────────
    envelope::EnvelopeConfig envelope_config() const {
        envelope::EnvelopeConfig ec;
        ec.max_depth         = ffb_max_depth;
        ec.min_edge          = ffb_min_edge;
        ec.min_edge_anchor   = ffb_min_edge_anchor;
        ec.min_edge_relaxed  = ffb_min_edge_relaxed;
        ec.promotion_depth   = 2;
        ec.store_format      = store_format;
        ec.collision_policy  = collision_policy;
        ec.link_subdivision_n   = link_subdivision_n;
        ec.link_subdivision_max = link_subdivision_max;
        ec.grid_resolution   = grid_resolution;
        for (int i = 0; i < 6; ++i)
            ec.grid_world_bounds[i] = grid_world_bounds[i];
        ec.aa_crossover_width = aa_crossover_width;
        return ec;
    }

    forest::ForestConfig forest_config() const {
        forest::ForestConfig fc;
        fc.max_boxes = max_boxes;
        fc.max_consecutive_miss = max_consecutive_miss;
        fc.bfs_phase_k = bfs_phase_k;
        fc.bfs_phase_budget = bfs_phase_budget;
        fc.min_boxes_per_pair = min_boxes_per_pair;
        fc.max_boxes_per_pair = max_boxes_per_pair;
        fc.guided_sample_ratio = guided_sample_ratio;
        fc.boundary_expand_epsilon = boundary_expand_epsilon;
        fc.n_edge_samples = n_edge_samples;
        fc.adjacency_tol = adjacency_tol;
        fc.proxy_anchor_max_samples = proxy_anchor_max_samples;
        fc.proxy_anchor_radius = proxy_anchor_radius;
        return fc;
    }

    bridge::BridgeConfig bridge_config() const {
        return {coarsen_max_rounds, coarsen_target_boxes,
                coarsen_greedy_rounds, coarsen_grid_check,
                coarsen_split_depth, coarsen_max_tree_fk,
                corridor_hops, use_gcs};
    }

    planner::PlannerConfig planner_config() const {
        return {shortcut_max_iters, segment_resolution,
                parallel_grow, n_partitions_depth,
                parallel_workers, seed};
    }
};

struct PlannerDefaults {
    static constexpr double CONNECTION_RADIUS = 2.0;
    static constexpr double SEGMENT_RESOLUTION = 0.05;
    static constexpr int    MAX_CONNECT_ATTEMPTS = 50;
    static constexpr int    MAX_BRIDGE_ATTEMPTS = 20;
    static constexpr int    INITIAL_CACHE_CAP = 64;
};

} // namespace sbf

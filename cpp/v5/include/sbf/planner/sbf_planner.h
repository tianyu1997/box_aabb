#pragma once
/// @file sbf_planner.h
/// @brief SBFPlanner — top-level SafeBoxForest motion planner.
///
/// Orchestrates the full planning pipeline:
///   1. **LECT construction** (with optional cache load/save).
///   2. **Forest growth** via ForestGrower (serial or parallel).
///   3. **Coarsening** (dimension-sweep + greedy adjacency merge).
///   4. **Path search** (Dijkstra or GCS) over the box adjacency graph.
///   5. **Smoothing** (shortcut + moving-average).
///
/// Also supports a **build / query split**: `build()` pre-computes the
/// forest offline, then `query()` can be called repeatedly for different
/// start/goal pairs without rebuilding.
///
/// LECT persistent caching is enabled by default: the tree is saved to
/// `~/.sbf_cache/<robot_fingerprint>.lect` and loaded on subsequent runs.
///
/// @see SBFPlannerConfig, PlanResult, ForestGrower, LECT

#include <sbf/core/types.h>
#include <sbf/core/robot.h>
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/envelope_type.h>
#include <sbf/lect/lect.h>
#include <sbf/lect/lect_cache_manager.h>
#include <sbf/forest/adjacency.h>
#include <sbf/forest/grower.h>
#include <sbf/forest/coarsen.h>
#include <sbf/planner/dijkstra.h>
#include <sbf/planner/gcs_planner.h>
#include <sbf/planner/path_smoother.h>
#include <sbf/planner/i_planner.h>

#include <Eigen/Dense>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

namespace sbf {

// Return user-global cache directory: $HOME/.sbf_cache (or $USERPROFILE on Windows)
inline std::string default_lect_cache_dir() {
    std::string home;
#ifdef _MSC_VER
    char* buf = nullptr;
    size_t len = 0;
    if (_dupenv_s(&buf, &len, "USERPROFILE") == 0 && buf) {
        home = buf;
        free(buf);
    } else if (_dupenv_s(&buf, &len, "HOME") == 0 && buf) {
        home = buf;
        free(buf);
    }
#else
    const char* h = std::getenv("HOME");
    if (h) home = h;
#endif
    if (home.empty()) return ".sbf_cache";
    return home + "/.sbf_cache";
}

/// @brief Aggregate result of a complete SBF planning query.
struct PlanResult {
    bool success = false;                          ///< Whether a collision-free path was found.
    std::vector<Eigen::VectorXd> path;             ///< Smoothed C-space waypoints.
    std::vector<int> box_sequence;                 ///< Box-ID sequence through the forest.
    double path_length = 0.0;                      ///< Total Euclidean path length.
    double planning_time_ms = 0.0;                 ///< Wall-clock time for the full pipeline.
    int n_boxes = 0;                               ///< Number of boxes after coarsening.
    int n_coarsen_merges = 0;                      ///< Number of coarsen merge operations.

    double envelope_volume_total = 0.0;            ///< Sum of envelope volumes.
    double build_time_ms = 0.0;                    ///< Forest build time.
    double lect_time_ms = 0.0;                     ///< LECT construction/load time.
};

/// @brief Configuration for proxy RRT search when start/goal is isolated.
///
/// Uses tiered timeouts: nearby candidates get short probes, farther ones
/// get progressively longer timeouts, with a hard total budget cap.
struct ProxySearchConfig {
    int    max_candidates      = 25;     ///< Max candidate boxes to try.
    int    tier1_count         = 5;      ///< Number of Tier-1 (fast) candidates.
    int    tier2_count         = 10;     ///< Number of Tier-2 (medium) candidates.
    double tier1_timeout_ms    = 200.0;  ///< Per-candidate timeout for Tier 1.
    double tier2_timeout_ms    = 500.0;  ///< Per-candidate timeout for Tier 2.
    double tier3_timeout_ms    = 2000.0; ///< Per-candidate timeout for Tier 3.
    double total_budget_ms     = 8000.0; ///< Hard cap on total proxy search time.
    int    rrt_max_iters       = 100000; ///< RRT max iterations per candidate.
    int    rrt_segment_res     = 20;     ///< RRT collision-check segment resolution.
};

/// @brief Full configuration for SBFPlanner.
///
/// Bundles sub-configs for grower, coarsening, smoothing, GCS, envelope
/// source, LECT split strategy, Z4 symmetry, and persistent caching.
struct SBFPlannerConfig {
    GrowerConfig grower;                    ///< Forest grower parameters.
    GreedyCoarsenConfig coarsen;            ///< Greedy coarsening parameters.
    SmootherConfig smoother;                ///< Path smoothing parameters.
    bool use_gcs = false;                   ///< Use GCS planner instead of Dijkstra.
    GCSConfig gcs;                          ///< GCS-specific parameters.
    ProxySearchConfig proxy;                ///< Proxy RRT search parameters.

    EndpointSourceConfig endpoint_source;   ///< How to compute link endpoints (IFK / CritSample).
    EnvelopeTypeConfig envelope_type;       ///< Envelope representation (LinkIAABB / LinkGrid).

    SplitOrder split_order = SplitOrder::BEST_TIGHTEN;  ///< LECT split strategy.

    bool z4_enabled = true;                 ///< Auto-detect and enable Z4 symmetry cache.

    bool lect_no_cache = false;             ///< Disable LECT persistent cache.
    bool use_v6_cache = true;               ///< Use V6 Z4-keyed mmap persistent cache.
    std::string lect_cache_dir = default_lect_cache_dir();  ///< Cache directory path.
};

/// @brief Top-level SafeBoxForest motion planner.
///
/// Manages the full lifecycle: LECT construction → forest growth →
/// coarsening → path search → smoothing.  Supports one-shot `plan()`
/// and split `build()` / `query()` workflows.
class SBFPlanner : public IPlanner {
public:
    SBFPlanner(const Robot& robot, const SBFPlannerConfig& config = {});

    /// One-shot: build forest + search + smooth.
    PlanResult plan(
        const Eigen::VectorXd& start,
        const Eigen::VectorXd& goal,
        const Obstacle* obs, int n_obs,
        double timeout_ms = 30000.0) override;

    /// Pre-build the forest for given start/goal (no path search).
    void build(const Eigen::VectorXd& start,
               const Eigen::VectorXd& goal,
               const Obstacle* obs, int n_obs,
               double timeout_ms = 30000.0);

    /// Coverage-only build: multi-goal RRT expansion.
    /// @param seed_points  C-space points used as multi-goal RRT roots.
    ///                     If empty, falls back to farthest-point sampling.
    void build_coverage(const Obstacle* obs, int n_obs,
                        double timeout_ms = 30000.0,
                        const std::vector<Eigen::VectorXd>& seed_points = {});

    /// Query a pre-built forest for a new start/goal pair.
    /// Includes proxy RRT, bridge, and RRT fallback for robustness.
    PlanResult query(const Eigen::VectorXd& start,
                     const Eigen::VectorXd& goal,
                     const Obstacle* obs = nullptr, int n_obs = 0);

    /// Discard the current forest and adjacency graph.
    void clear_forest();

    /// Obstacle-free LECT warm-up: expand random paths to deepen the tree.
    /// @param max_depth  Target tree depth.
    /// @param n_paths    Number of random descent paths.
    /// @param seed       RNG seed.
    /// @return Number of new LECT nodes created.
    int warmup_lect(int max_depth, int n_paths, int seed = 42);

    const std::vector<BoxNode>& boxes() const { return boxes_; }
    const std::vector<BoxNode>& raw_boxes() const { return raw_boxes_; }
    const AdjacencyGraph& adjacency() const { return adj_; }
    int n_boxes() const { return static_cast<int>(boxes_.size()); }

private:
    const Robot& robot_;
    SBFPlannerConfig config_;
    std::unique_ptr<LECT> lect_;
    std::unique_ptr<LectCacheManager> cache_mgr_;   ///< V6 persistent cache (shared across workers)
    std::vector<BoxNode> boxes_;
    std::vector<BoxNode> raw_boxes_;
    AdjacencyGraph adj_;
    bool built_ = false;
    double last_build_time_ms_ = 0.0;
    double last_lect_time_ms_ = 0.0;

    // Stored obstacles for query-time collision checking
    std::vector<Obstacle> stored_obs_;

    std::string lect_auto_cache_path() const;
};

}  // namespace sbf

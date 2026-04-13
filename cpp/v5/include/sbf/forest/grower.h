#pragma once
/// @file grower.h
/// @brief ForestGrower — collision-free box forest construction.
///
/// Implements two growth strategies for expanding a forest of BoxNodes:
///   - **RRT mode**: random tree expansion with goal bias.
///   - **Wavefront mode**: multi-stage boundary-seeded BFS with
///     progressive `box_limit` thresholds for stage progression.
///
/// Each box is constructed via Find-Free-Box (FFB) within the LECT tree.
/// After the main growth phase, an optional **promotion** pass attempts
/// to enlarge existing boxes by re-running FFB with relaxed constraints.
///
/// **Parallel growth** (`n_threads > 1`): root seeds are distributed
/// across worker threads, each with an independent LECT snapshot.
/// Workers grow subtrees in isolation; results are merged with
/// box-ID remapping and LECT `transplant_subtree()`.
///
/// @see FFBResult, LECT, GrowerConfig

#include <sbf/core/types.h>
#include <sbf/ffb/ffb.h>
#include <sbf/lect/lect.h>

#include <Eigen/Dense>

#include <atomic>
#include <chrono>
#include <memory>
#include <random>
#include <thread>
#include <unordered_map>
#include <vector>

namespace sbf {

// ─── Grower configuration ───────────────────────────────────────────────────

/// @brief Full configuration for ForestGrower.
///
/// Controls growth mode, FFB parameters, wavefront stage progression,
/// promotion, boundary sampling, RNG seed, and parallel thread count.
struct GrowerConfig {
    /// Growth strategy: RRT (random tree) or WAVEFRONT (boundary BFS).
    enum class Mode { RRT, WAVEFRONT };
    Mode mode = Mode::RRT;

    FFBConfig ffb_config;             ///< Configuration forwarded to Find-Free-Box.

    int max_boxes = 500;                ///< Stop after creating this many boxes.
    double timeout_ms = 30000.0;        ///< Wall-clock timeout in milliseconds.
    int max_consecutive_miss = 2000;    ///< Abort current stage after N consecutive FFB failures.

    // ── RRT parameters ──
    double rrt_goal_bias = 0.8;         ///< Probability of sampling goal config.
    double rrt_step_ratio = 0.05;       ///< Step size as fraction of joint range.

    // ── Wavefront parameters ──
    /// @brief One stage of wavefront expansion.
    /// Each stage sets a cumulative `box_limit` threshold for progression.
    struct WavefrontStage {
        int box_limit;      ///< Max cumulative boxes at end of this stage.
    };
    std::vector<WavefrontStage> wavefront_stages = {
        {50}, {150}, {300}, {500}
    };

    // ── Promotion ──
    bool enable_promotion = true;          ///< Run promotion pass after main growth.

    // ── Boundary sampling ──
    double boundary_epsilon = 1e-11;       ///< Epsilon offset when snapping seeds to box faces.
    int n_boundary_samples = 4;            ///< Number of boundary seeds per box per expansion.
    double goal_face_bias = 0.5;           ///< Probability of biasing toward goal face.

    // ── RNG ──
    uint64_t rng_seed = 42;                ///< Master RNG seed (workers derive from this).

    /// Number of threads for parallel growth.
    /// Default: all hardware threads.  Set to 1 to force serial mode.
    /// Workers each get an independent LECT snapshot; results are merged.
    int n_threads = std::max(1u, std::thread::hardware_concurrency());

    /// Number of threads for bridge_all_islands (island merging).
    /// 0 = use n_threads.  Set separately because serial grow + parallel bridge
    /// is often the best strategy (unified wavefront + fast bridging).
    int bridge_n_threads = 0;

    /// Connect mode: stop wavefront growth as soon as all multi-goal trees
    /// are connected via box adjacency (no RRT bridge needed).
    /// Uses an inline UnionFind to track inter-tree merges incrementally.
    bool connect_mode = true;

    /// If true, break the grow loop immediately after all trees connect.
    /// When false (default), growth continues for coverage until timeout.
    bool stop_after_connect = false;

    /// FFB batch size per iteration.  0 = auto (= n_threads).
    /// Higher values reduce master idle time between batches.
    int batch_size = 0;
};

// ─── Grower result ──────────────────────────────────────────────────────────

/// @brief Aggregate statistics returned by ForestGrower::grow().
struct GrowerResult {
    std::vector<BoxNode> boxes;
    int n_roots = 0;
    int n_ffb_success = 0;
    int n_ffb_fail = 0;
    int n_promotions = 0;
    bool start_goal_connected = false;
    double total_volume = 0.0;
    double build_time_ms = 0.0;

    // FFB aggregate timing
    int    ffb_total_calls = 0;
    double ffb_total_ms = 0.0;
    double ffb_envelope_ms = 0.0;
    double ffb_collide_ms = 0.0;
    double ffb_expand_ms = 0.0;
    double ffb_intervals_ms = 0.0;
    int    ffb_cache_hits = 0;
    int    ffb_cache_misses = 0;
    int    ffb_collide_calls = 0;
    int    ffb_expand_calls = 0;
    int    ffb_total_steps = 0;
    int    lect_nodes_final = 0;

    /// True if all multi-goal trees became connected (connect_mode).
    bool all_connected = false;
    /// Wall-clock time (ms) when all trees first became connected.
    double connect_time_ms = 0.0;
    /// Number of boxes when all trees became connected.
    int connect_n_boxes = 0;
};

// ─── Parallel worker result ─────────────────────────────────────────────

/// @brief Result from one parallel worker thread, bundling grown boxes
/// with the worker's modified LECT snapshot for later transplant.
struct ParallelWorkerResult {
    GrowerResult result;
    LECT lect;
};

// ─── ForestGrower ───────────────────────────────────────────────────────────

/// @brief Grows a forest of collision-free BoxNodes in configuration space.
///
/// The grower holds a reference to an external LECT and mutates it
/// (expanding nodes, marking occupation).  After `grow()`, call
/// `boxes()` to retrieve the produced forest.
///
/// @par Parallel mode
/// When `config.n_threads > 1`, root seeds are distributed across
/// independent worker threads.  Each worker operates on a deep copy
/// of the LECT (`snapshot()`).  After all workers finish, their boxes
/// are collected with remapped IDs and their LECT expansions are merged
/// back via `transplant_subtree()`.
class ForestGrower {
public:
    using Clock = std::chrono::steady_clock;

    /// Construct a grower referencing an *external* LECT (serial mode).
    ForestGrower(const Robot& robot, LECT& lect, const GrowerConfig& config);

    /// Constructor for parallel workers: takes ownership of a LECT snapshot
    ForestGrower(const Robot& robot, LECT&& lect_owned, const GrowerConfig& config);

    /// Set start/goal configurations for endpoint-directed growth.
    void set_endpoints(const Eigen::VectorXd& start, const Eigen::VectorXd& goal);

    /// Set multiple goal configurations for multi-goal RRT coverage growth.
    /// Each goal becomes a root; during RRT growth, goal bias picks from
    /// the other goals (excluding the current tree's root).
    void set_multi_goals(const std::vector<Eigen::VectorXd>& goals);

    /// Set an absolute deadline (used by parallel workers).
    void set_deadline(Clock::time_point deadline);

    /// Run the full growth pipeline: root selection → grow → promote.
    /// @param obs  Obstacle array (may be nullptr if n_obs == 0).
    /// @param n_obs  Number of obstacles.
    /// @return  Aggregate statistics and produced boxes.
    GrowerResult grow(const Obstacle* obs, int n_obs);

    /// Grow a single subtree (used by parallel workers).
    GrowerResult grow_subtree(const Eigen::VectorXd& root_seed, int root_id,
                              const Obstacle* obs, int n_obs,
                              std::shared_ptr<std::atomic<int>> shared_counter);

    const std::vector<BoxNode>& boxes() const { return boxes_; }
    const LECT& lect() const { return lect_; }
    LECT&& take_lect() { return std::move(lect_owned_); }

private:
    int try_create_box(const Eigen::VectorXd& seed,
                       const Obstacle* obs, int n_obs,
                       int parent_box_id, int face_dim,
                       int face_side, int root_id);

    void grow_rrt(const Obstacle* obs, int n_obs);
    void grow_wavefront(const Obstacle* obs, int n_obs);
    void select_roots(const Obstacle* obs, int n_obs);
    int  promote_all(const Obstacle* obs, int n_obs);

    Eigen::VectorXd sample_random() const;
    Eigen::VectorXd clamp_to_limits(const Eigen::VectorXd& q) const;

    struct SnapResult {
        Eigen::VectorXd seed;
        int face_dim = -1;
        int face_side = -1;
    };
    SnapResult snap_to_face(const BoxNode& nearest,
                            const Eigen::VectorXd& direction) const;

    struct BoundarySeed {
        int dim;
        int side;
        Eigen::VectorXd config;
    };
    std::vector<BoundarySeed> sample_boundary(
        const BoxNode& box,
        const Eigen::VectorXd* bias_target) const;

    bool deadline_reached() const;
    /// Check if global box budget is exhausted (via shared atomic counter).
    bool global_budget_reached() const;

    /// Extend a newly-created box to share a face with its parent.
    /// Returns true if the box now shares a face (or already did).
    bool enforce_parent_adjacency(int parent_id, int face_dim, int face_side,
                                  const Obstacle* obs, int n_obs);

    void grow_parallel(const Obstacle* obs, int n_obs, GrowerResult& result);

    /// Master-worker coordinated parallel growth.
    /// Master: RRT sampling + box management + adjacency tracking.
    /// Workers: FFB computation only (each with its own LECT snapshot).
    /// Guarantees no duplicate/overlapping boxes across trees.
    void grow_coordinated(const Obstacle* obs, int n_obs);

    const Robot& robot_;
    LECT lect_owned_;          // owned copy for parallel workers (must be before lect_)
    LECT& lect_;
    GrowerConfig config_;
    std::vector<BoxNode> boxes_;
    Eigen::VectorXd start_, goal_;
    bool has_endpoints_ = false;
    std::vector<Eigen::VectorXd> multi_goals_;
    bool has_multi_goals_ = false;
    int next_box_id_ = 0;
    int n_ffb_success_ = 0;
    int n_ffb_fail_ = 0;
    std::shared_ptr<std::atomic<int>> shared_box_count_;

    // FFB aggregate timing accumulators
    double ffb_total_ms_ = 0.0;
    double ffb_envelope_ms_ = 0.0;
    double ffb_collide_ms_ = 0.0;
    double ffb_expand_ms_ = 0.0;
    double ffb_intervals_ms_ = 0.0;
    int    ffb_cache_hits_ = 0;
    int    ffb_cache_misses_ = 0;
    int    ffb_collide_calls_ = 0;
    int    ffb_expand_calls_ = 0;
    int    ffb_total_steps_ = 0;
    int    ffb_total_calls_ = 0;

    mutable std::mt19937_64 rng_;

    Clock::time_point deadline_;
    bool has_deadline_ = false;

    // Wavefront connectivity results (set by grow_wavefront in connect_mode)
    bool wf_all_connected_ = false;
    double wf_connect_time_ms_ = -1.0;
    int wf_connect_boxes_ = 0;
};

}  // namespace sbf

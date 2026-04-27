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
    double rrt_goal_bias = 0.1;         ///< Probability of sampling goal config.
    double rrt_step_ratio = 0.05;       ///< Step size as fraction of joint range.

    /// Volume-bonus coefficient for nearest-box search in grow_rrt.
    /// Adds bonus = alpha * diag_sq * (V_box / V_dom)^(2/nd) to favour
    /// larger boxes (more open free space) as RRT parents. Dimensionally
    /// consistent across nd via the (2/nd) exponent. 0 disables the bonus.
    double vol_bonus_alpha = 0.05;

    // ── Phase U: unexplored-region sampling ──
    /// Probability that an RRT random sample is drawn from a free-volume-
    /// weighted walk down the LECT (biasing toward unexplored regions)
    /// rather than uniform C-space. 0 = legacy behaviour.
    double unexplored_sample_prob = 0.7;

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

    /// Maximum additional boxes to grow after all trees become connected.
    /// 0 = unlimited (use timeout or max_consecutive_miss).
    /// Positive values allow a fixed budget of extra boxes for coverage
    /// before stopping.  Combined with stop_after_connect=false.
    int post_connect_extra_boxes = 0;

    /// FFB batch size per iteration.  0 = auto (= n_threads).
    /// Higher values reduce master idle time between batches.
    int batch_size = 0;

    // ── Phase B: hierarchical depth schedule ──
    /// Optional multi-stage growth: each stage temporarily overrides
    /// `ffb_config.max_depth` and the per-stage box quota
    /// (max_boxes * quota_ratio, accumulated). Empty = legacy single-pass
    /// behaviour. Suggested: {{30,0.4},{80,0.4},{300,0.2}}.
    /// Each pair is (max_depth, box_quota_ratio).
    /// `stop_after_connect=true` lets early stages exit once endpoints connect.
    std::vector<std::pair<int,double>> ffb_depth_stages;

    // ── Phase C-1: endpoint-mode auto bridge ──
    /// In endpoint mode (start_/goal_ set, no multi_goals), after parallel
    /// growth the grower may leave start_box and goal_box in disjoint
    /// adjacency components. When this flag is true (default), grow()
    /// runs an extra serial RRT pass with goal_bias toward start_/goal_
    /// to bridge the gap, up to `endpoint_bridge_max_boxes` extra boxes.
    bool endpoint_auto_bridge = true;

    /// Max extra boxes for the endpoint auto-bridge pass.
    /// 0 = use 5% of max_boxes (clamped to [50, 500]).
    int endpoint_bridge_max_boxes = 0;
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

    /// Canonical connected flag: final box-level UF connectivity.
    /// This is computed from the final current box geometry.
    bool all_connected = false;
    /// Historical tree-UF first-connect time (diagnostic, not canonical).
    double connect_time_ms = 0.0;
    /// Historical tree-UF first-connect box count (diagnostic).
    int connect_n_boxes = 0;

    /// Tree-level UF connectivity from coordinated grow phase (diagnostic).
    bool tree_all_connected = false;
    /// Tree-level UF first-connect time in ms (diagnostic).
    double tree_connect_time_ms = 0.0;
    /// Tree-level UF first-connect boxes (diagnostic).
    int tree_connect_n_boxes = 0;

    /// Canonical connectivity from final adjacency graph (islands <= 1).
    bool adjacency_all_connected = false;
    /// Number of connected components in final adjacency graph.
    int adjacency_islands = 0;
    /// Size of largest island in final adjacency graph.
    int adjacency_largest_island = 0;
    /// Time for final adjacency+island check in grow() result assembly.
    double adjacency_check_ms = 0.0;
};

// ─── Parallel worker result ─────────────────────────────────────────────

/// @brief Result from one parallel worker thread, bundling grown boxes
/// with the worker's modified LECT snapshot for later transplant.
struct ParallelWorkerResult {
    GrowerResult result;
    LECT lect;
    int domain_root = -1;   ///< LECT subtree this worker was confined to
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
    /// @param skip_promotion  When true, skip the local promotion pass.
    ///   `grow_parallel` sets this so promotion runs once on master after
    ///   transplant — so the union-based envelope updates are written into
    ///   master LECT cache (worker-local cache writes would be lost since
    ///   transplant_subtree only copies newly-expanded nodes).
    GrowerResult grow_subtree(const Eigen::VectorXd& root_seed, int root_id,
                              const Obstacle* obs, int n_obs,
                              std::shared_ptr<std::atomic<int>> shared_counter,
                              bool skip_promotion = false);

    /// Geometric domain restriction: when @p idx >= 0, this grower only
    /// accepts boxes whose tree_id is a descendant of @p idx in the LECT,
    /// and only promotes nodes within that subtree. Used by grow_parallel
    /// to give each worker an exclusive LECT subdomain so workers cannot
    /// interfere with each other (no shared writes, no index races).
    void set_domain_root(int idx) { domain_root_ = idx; }
    int  domain_root() const { return domain_root_; }

    const std::vector<BoxNode>& boxes() const { return boxes_; }
    const LECT& lect() const { return lect_; }
    LECT&& take_lect() { return std::move(lect_owned_); }

    /// Set worker thread ID (0-indexed). Used for tracing in parallel mode.
    void set_worker_tid(int tid) { worker_tid_ = tid; }
    int  get_worker_tid() const { return worker_tid_; }

private:
    int try_create_box(const Eigen::VectorXd& seed,
                       const Obstacle* obs, int n_obs,
                       int parent_box_id, int face_dim,
                       int face_side, int root_id);

    void grow_rrt(const Obstacle* obs, int n_obs);
    void grow_wavefront(const Obstacle* obs, int n_obs);
    void select_roots(const Obstacle* obs, int n_obs);
    int  promote_all(const Obstacle* obs, int n_obs,
                      const std::vector<int>& start_nodes = {});

    Eigen::VectorXd sample_random() const;
    /// Phase U: weighted walk-down of the LECT biased toward subtrees with
    /// the most remaining free volume, then uniform sample within the
    /// reached leaf's intervals (clamped to joint limits and current
    /// `domain_root_` if any). Falls back to `sample_random()` when no
    /// free volume is available.
    Eigen::VectorXd sample_unexplored() const;
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

    /// Phase C-1: serial RRT pass biased toward start_/goal_ to bridge the
    /// gap when post-parallel growth left start_box and goal_box in disjoint
    /// adjacency components. No-op in non-endpoint mode or if already
    /// connected. Returns number of boxes added.
    int endpoint_bridge_pass(const Obstacle* obs, int n_obs);

    /// Returns true iff the boxes containing start_ and goal_ are in the
    /// same adjacency component (BFS over boxes_adjacent). False if either
    /// endpoint has no containing box. O(n_boxes^2) — only call after grow.
    bool start_goal_adj_connected() const;

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

    /// Geometric domain restriction (parallel workers): when >= 0, only
    /// boxes/promotions whose tree_id is a descendant of this LECT node
    /// are accepted. -1 means no restriction (serial / master mode).
    int domain_root_ = -1;

    /// Worker thread ID (0-indexed in parallel mode, -1 in serial).
    /// Used for tagging TRACE log lines in multi-threaded builds.
    int worker_tid_ = -1;

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

    // Promotion count from grow_coordinated (set by grow_coordinated, read by grow())
    int n_coordinated_promotions_ = 0;

    // Diagnostic: cross-tree touch pairs (box_id_a, box_id_b)
    std::vector<std::pair<int,int>> cross_tree_pairs_;
};

}  // namespace sbf

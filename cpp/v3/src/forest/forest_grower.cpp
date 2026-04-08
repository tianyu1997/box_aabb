// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v3 — ForestGrower implementation
//  Multi-root forest growing with wavefront / RRT expansion.
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/forest/forest_grower.h"
#include "sbf/forest/thread_pool.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <numeric>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace sbf {
namespace forest {

// ─────────────────────────────────────────────────────────────────────────
//  Construction
// ─────────────────────────────────────────────────────────────────────────
ForestGrower::ForestGrower(const Robot& robot, const GrowerConfig& config)
    : robot_(&robot)
    , config_(config)
    , lect_(robot, config.pipeline)
    , graph_(robot.joint_limits(), config.adjacency_tol)
    , rng_(config.rng_seed)
{
    if (config.hull_skip_vol > 0.0)
        lect_.set_hull_skip_vol(config.hull_skip_vol);
}

// ─────────────────────────────────────────────────────────────────────────
//  Construction (warm-start: receive a pre-built LECT)
// ─────────────────────────────────────────────────────────────────────────
ForestGrower::ForestGrower(const Robot& robot, const GrowerConfig& config,
                           LECT warm_lect)
    : robot_(&robot)
    , config_(config)
    , lect_(std::move(warm_lect))       // take ownership of pre-built LECT
    , graph_(robot.joint_limits(), config.adjacency_tol)
    , rng_(config.rng_seed)
{
    if (config.hull_skip_vol > 0.0)
        lect_.set_hull_skip_vol(config.hull_skip_vol);
}

// ─────────────────────────────────────────────────────────────────────────
//  set_endpoints
// ─────────────────────────────────────────────────────────────────────────
void ForestGrower::set_endpoints(const Eigen::VectorXd& start,
                                 const Eigen::VectorXd& goal) {
    has_endpoints_ = true;
    start_config_ = start;
    goal_config_ = goal;
}

// ─────────────────────────────────────────────────────────────────────────
//  Per-tree goal bias
// ─────────────────────────────────────────────────────────────────────────
const Eigen::VectorXd* ForestGrower::get_bias_target(int root_id) {
    if (!has_endpoints_) return nullptr;
    if (root_id == 0) return &goal_config_;   // start tree  → bias toward goal
    if (root_id == 1) return &start_config_;  // goal tree   → bias toward start
    // random tree (root ≥ 2): 50/50 toward start or goal
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    return (u01(rng_) < 0.5) ? &start_config_ : &goal_config_;
}

// ─────────────────────────────────────────────────────────────────────────
//  sample_near_existing_boundary — pick random box face to reduce fragments
// ─────────────────────────────────────────────────────────────────────────
Eigen::VectorXd ForestGrower::sample_near_existing_boundary(int root_id) {
    if (boxes_.empty()) return {};  // empty VectorXd (size == 0)

    // Collect candidate boxes (optionally filtered by root_id)
    std::vector<int> candidates;
    if (root_id >= 0) {
        for (int i = 0; i < static_cast<int>(boxes_.size()); ++i)
            if (boxes_[i].root_id == root_id)
                candidates.push_back(i);
    }
    if (candidates.empty()) {
        // If no box of that root (or root_id < 0), use all boxes
        candidates.resize(boxes_.size());
        std::iota(candidates.begin(), candidates.end(), 0);
    }

    std::uniform_int_distribution<int> box_dist(
        0, static_cast<int>(candidates.size()) - 1);
    const BoxNode& box = boxes_[candidates[box_dist(rng_)]];

    int n_dims = box.n_dims();
    const auto& limits = robot_->joint_limits().limits;
    double eps = config_.boundary_epsilon;

    // Collect valid faces
    struct Face { int dim; int side; };
    std::vector<Face> faces;
    for (int d = 0; d < n_dims; ++d) {
        if (box.joint_intervals[d].lo - eps >= limits[d].lo)
            faces.push_back({d, 0});
        if (box.joint_intervals[d].hi + eps <= limits[d].hi)
            faces.push_back({d, 1});
    }
    if (faces.empty()) return {};

    std::uniform_int_distribution<int> face_dist(
        0, static_cast<int>(faces.size()) - 1);
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    const Face& f = faces[face_dist(rng_)];

    Eigen::VectorXd seed(n_dims);
    for (int d = 0; d < n_dims; ++d) {
        if (d == f.dim) {
            seed[d] = (f.side == 0)
                ? box.joint_intervals[d].lo - eps
                : box.joint_intervals[d].hi + eps;
        } else {
            double lo = box.joint_intervals[d].lo;
            double hi = box.joint_intervals[d].hi;
            seed[d] = lo + u01(rng_) * (hi - lo);
        }
    }
    return robot_->joint_limits().clamp(seed);
}

// ─────────────────────────────────────────────────────────────────────────
//  rrt_snap_to_face — snap RRT seed to nearest box face
// ─────────────────────────────────────────────────────────────────────────
ForestGrower::SnapResult ForestGrower::rrt_snap_to_face(
        const BoxNode& nearest,
        const Eigen::VectorXd& direction,
        double step) {
    int n_dims = nearest.n_dims();
    const auto& limits = robot_->joint_limits().limits;
    double eps = config_.boundary_epsilon;
    Eigen::VectorXd nearest_center = nearest.center();

    // Find the best face aligned with direction
    int best_dim = -1, best_side = -1;
    double best_score = -1e30;

    for (int d = 0; d < n_dims; ++d) {
        for (int side = 0; side < 2; ++side) {
            // face normal: side 0 → -1, side 1 → +1
            double normal_sign = (side == 0) ? -1.0 : 1.0;
            double score = direction[d] * normal_sign;
            if (score <= 0) continue;  // face points away from direction
            // Check face is within joint limits
            if (side == 0 && nearest.joint_intervals[d].lo - eps < limits[d].lo)
                continue;
            if (side == 1 && nearest.joint_intervals[d].hi + eps > limits[d].hi)
                continue;
            if (score > best_score) {
                best_score = score;
                best_dim = d;
                best_side = side;
            }
        }
    }

    if (best_dim < 0) {
        // No valid face: fallback to pure directional extend
        Eigen::VectorXd seed = nearest_center + direction * step;
        return {robot_->joint_limits().clamp(seed), -1, -1};
    }

    // Place seed on the best face with directional jitter
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    Eigen::VectorXd seed(n_dims);
    for (int d = 0; d < n_dims; ++d) {
        if (d == best_dim) {
            // Face dimension: place at boundary ± eps
            seed[d] = (best_side == 0)
                ? nearest.joint_intervals[d].lo - eps
                : nearest.joint_intervals[d].hi + eps;
        } else {
            // Other dimensions: 70% directional bias + 30% random within face
            double face_lo = nearest.joint_intervals[d].lo;
            double face_hi = nearest.joint_intervals[d].hi;
            double target_on_face = nearest_center[d] + direction[d] * step;
            target_on_face = std::clamp(target_on_face, face_lo, face_hi);
            double rand_on_face = face_lo + u01(rng_) * (face_hi - face_lo);
            seed[d] = 0.7 * target_on_face + 0.3 * rand_on_face;
        }
    }
    return {robot_->joint_limits().clamp(seed), best_dim, best_side};
}

// ─────────────────────────────────────────────────────────────────────────
//  is_grid_envelope
// ─────────────────────────────────────────────────────────────────────────
bool ForestGrower::is_grid_envelope() const {
    auto et = config_.pipeline.envelope.type;
    return et == envelope::EnvelopeType::SubAABB_Grid
        || et == envelope::EnvelopeType::Hull16_Grid;
}

// ═════════════════════════════════════════════════════════════════════════
//  grow — main entry point
// ═════════════════════════════════════════════════════════════════════════
GrowerResult ForestGrower::grow(const Obstacle* obstacles, int n_obs) {
    auto t0 = std::chrono::high_resolution_clock::now();
    GrowerResult result;

    boxes_.clear();
    box_id_to_idx_.clear();
    graph_.clear();
    next_box_id_ = 0;
    start_box_id_ = -1;
    goal_box_id_ = -1;
    n_coarse_created_ = 0;
    n_fine_created_   = 0;

    // Pre-rasterize scene for hull collision if grid-based
    if (is_grid_envelope()) {
        lect_.set_scene(obstacles, n_obs);
    }

    // ── Load offline LECT cache (if configured) ─────────────────────────
    //  A pre-built deep LECT avoids expensive FK + envelope computation
    //  during find_free_box() in root_select.  The loaded tree already has
    //  AABBs and hulls cached for all nodes up to a certain depth.
    bool lect_cache_loaded = false;
    if (!config_.lect_cache_dir.empty()) {
        namespace fs = std::filesystem;
        std::string hcache = config_.lect_cache_dir + "/lect.hcache";
        if (fs::exists(hcache)) {
            auto t_cache0 = std::chrono::high_resolution_clock::now();
            lect_.load(config_.lect_cache_dir, *robot_);
            // Re-apply hull_skip_vol (not persisted in cache)
            if (config_.hull_skip_vol > 0.0)
                lect_.set_hull_skip_vol(config_.hull_skip_vol);
            // Re-apply scene grid (load() doesn't preserve it)
            if (is_grid_envelope())
                lect_.set_scene(obstacles, n_obs);
            auto t_cache1 = std::chrono::high_resolution_clock::now();
            double cache_ms = std::chrono::duration<double, std::milli>(
                t_cache1 - t_cache0).count();
            std::printf("[grower] loaded LECT cache: %d nodes, %.1f ms\n",
                        lect_.n_nodes(), cache_ms);
            lect_cache_loaded = true;
        } else {
            std::printf("[grower] LECT cache not found: %s\n", hcache.c_str());
        }
    }

    // ── Phase 1: Root selection + Subtree partitioning ────────────────
    auto t_root0 = std::chrono::high_resolution_clock::now();

    if (config_.n_threads > 1 && !has_endpoints_) {
        // Parallel (no endpoints): LECT-aligned partition → roots within cells
        // n_roots must be power of 2 for LECT alignment; fall back to uniform if not
        bool is_power_of_2 = (config_.n_roots > 0) &&
                             (config_.n_roots & (config_.n_roots - 1)) == 0;
        if (is_power_of_2) {
            partition_lect_aligned();
        } else {
            std::printf("[grower] n_roots=%d not power-of-2, falling back to partition_uniform\n",
                        config_.n_roots);
            partition_uniform();
        }
        select_roots_in_partitions(obstacles, n_obs);
    } else {
        // Serial or endpoints: legacy flow (roots → partition by positions)
        select_roots(obstacles, n_obs);
        partition_subtrees();
    }

    auto t_root1 = std::chrono::high_resolution_clock::now();
    result.n_roots = static_cast<int>(boxes_.size());
    result.phase_times["root_select_ms"] =
        std::chrono::duration<double, std::milli>(t_root1 - t_root0).count();

    std::printf("[grower] %d roots placed, %d boxes\n",
                result.n_roots, static_cast<int>(boxes_.size()));

    // Add root boxes to graph (needed for RRT find_nearest_box, harmless for wavefront)
    for (const auto& b : boxes_)
        graph_.add_box(b);

    // ── Phase 2+3: Expansion + Promotion ────────────────────────────────
    auto t_expand0 = std::chrono::high_resolution_clock::now();

    if (config_.n_threads > 1 && static_cast<int>(subtrees_.size()) > 1) {
        // ── Parallel path ──
        grow_parallel(obstacles, n_obs, result);
    } else {
        // ── Serial path ──
        if (config_.mode == GrowerConfig::Mode::Wavefront) {
            grow_wavefront(obstacles, n_obs);
        } else {
            grow_rrt(obstacles, n_obs);
        }

        std::printf("[grower] expansion done: %d boxes\n",
                    static_cast<int>(boxes_.size()));

        result.n_promotions = promote_all(obstacles, n_obs);
    }

    auto t_expand1 = std::chrono::high_resolution_clock::now();
    result.phase_times["expand_ms"] =
        std::chrono::duration<double, std::milli>(t_expand1 - t_expand0).count();

    // ── Phase 3b: Coarsening (greedy merge) ─────────────────────────────
    if (config_.coarsen_enabled) {
        auto t_coarsen0 = std::chrono::high_resolution_clock::now();
        result.n_coarsen_merges = coarsen_greedy(obstacles, n_obs);
        auto t_coarsen1 = std::chrono::high_resolution_clock::now();
        result.phase_times["coarsen_ms"] =
            std::chrono::duration<double, std::milli>(t_coarsen1 - t_coarsen0).count();
    }

    // ── Phase 4: Final adjacency rebuild ────────────────────────────────
    auto t_adj0 = std::chrono::high_resolution_clock::now();
    graph_.rebuild(boxes_);
    auto t_adj1 = std::chrono::high_resolution_clock::now();
    result.phase_times["adj_rebuild_ms"] =
        std::chrono::duration<double, std::milli>(t_adj1 - t_adj0).count();

    // ── Collect statistics ──────────────────────────────────────────────
    result.boxes = boxes_;
    result.n_boxes_total = static_cast<int>(boxes_.size());
    result.n_components = graph_.n_components();
    result.total_volume = 0.0;
    for (const auto& b : boxes_)
        result.total_volume += b.volume;

    // Adaptive min_edge phase statistics
    result.n_coarse_boxes = n_coarse_created_;
    result.n_fine_boxes   = n_fine_created_;
    if (config_.adaptive_min_edge && result.n_boxes_total > 0) {
        std::printf("[grower] adaptive min_edge: n_stages=%d coarse=%d fine=%d "
                    "(coarse_me=%.3f, fine_me=%.4f)\n",
                    config_.adaptive_n_stages,
                    result.n_coarse_boxes, result.n_fine_boxes,
                    config_.coarse_min_edge, config_.min_edge);
    }

    // Check start-goal connectivity
    if (has_endpoints_ && start_box_id_ >= 0 && goal_box_id_ >= 0) {
        result.start_goal_connected = graph_.connected(start_box_id_, goal_box_id_);
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    result.build_time_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::printf("[grower] done: %d boxes, %d edges, %d components, "
                "vol=%.6f, %.1f ms%s\n",
                result.n_boxes_total,
                graph_.n_edges(),
                result.n_components,
                result.total_volume,
                result.build_time_ms,
                result.start_goal_connected ? " [S-G connected]" : "");

    return result;
}

// ═════════════════════════════════════════════════════════════════════════
//  Root selection
// ═════════════════════════════════════════════════════════════════════════
void ForestGrower::select_roots(const Obstacle* obs, int n_obs) {
    if (has_endpoints_) {
        select_roots_with_endpoints(obs, n_obs);
    } else {
        select_roots_no_endpoints(obs, n_obs);
    }
}

// ─────────────────────────────────────────────────────────────────────────
//  FPS root selection (no endpoints)
// ─────────────────────────────────────────────────────────────────────────
void ForestGrower::select_roots_no_endpoints(const Obstacle* obs, int n_obs) {
    int n_roots = config_.n_roots;
    std::vector<Eigen::VectorXd> root_centers;
    double rme = (config_.root_min_edge > 0.0) ? config_.root_min_edge : -1.0;

    // First root: random seed
    for (int attempt = 0; attempt < 100; ++attempt) {
        Eigen::VectorXd seed = sample_random();
        int bid = try_create_box(seed, obs, n_obs, -1, -1, -1, 0, rme);
        if (bid >= 0) {
            root_centers.push_back(boxes_.back().center());
            break;
        }
    }
    if (root_centers.empty()) {
        std::printf("[grower] WARNING: could not place first root\n");
        return;
    }

    // Remaining roots: Farthest Point Sampling
    constexpr int K_CANDIDATES = 50;
    for (int r = 1; r < n_roots; ++r) {
        // Generate K candidates
        std::vector<Eigen::VectorXd> candidates;
        candidates.reserve(K_CANDIDATES);
        for (int k = 0; k < K_CANDIDATES; ++k)
            candidates.push_back(sample_random());

        // Find candidate with max min-distance to existing roots
        double best_score = -1.0;
        int best_k = 0;
        for (int k = 0; k < K_CANDIDATES; ++k) {
            double min_dist = std::numeric_limits<double>::max();
            for (const auto& rc : root_centers) {
                double d = (candidates[k] - rc).norm();
                min_dist = std::min(min_dist, d);
            }
            if (min_dist > best_score) {
                best_score = min_dist;
                best_k = k;
            }
        }

        int bid = try_create_box(candidates[best_k], obs, n_obs, -1, -1, -1, r, rme);
        if (bid >= 0) {
            root_centers.push_back(boxes_.back().center());
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────
//  Root selection with start/goal endpoints
// ─────────────────────────────────────────────────────────────────────────
void ForestGrower::select_roots_with_endpoints(const Obstacle* obs, int n_obs) {
    int n_roots = config_.n_roots;
    double rme = (config_.root_min_edge > 0.0) ? config_.root_min_edge : -1.0;

    // Root 0: start
    {
        int bid = try_create_box(start_config_, obs, n_obs, -1, -1, -1, 0, rme);
        if (bid >= 0) {
            start_box_id_ = bid;
        } else {
            std::printf("[grower] WARNING: start seed FFB failed\n");
        }
    }

    // Root 1: goal
    {
        int bid = try_create_box(goal_config_, obs, n_obs, -1, -1, -1, 1, rme);
        if (bid >= 0) {
            goal_box_id_ = bid;
        } else {
            std::printf("[grower] WARNING: goal seed FFB failed\n");
        }
    }

    // Remaining roots: sample along start→goal line with Gaussian perturbation
    if (n_roots > 2 && boxes_.size() >= 2) {
        Eigen::VectorXd sg = goal_config_ - start_config_;
        double sg_norm = sg.norm();
        int n_dims = robot_->n_joints();

        constexpr int K_CANDIDATES = 30;
        std::vector<Eigen::VectorXd> root_centers;
        for (const auto& b : boxes_)
            root_centers.push_back(b.center());

        for (int r = 2; r < n_roots; ++r) {
            // Uniform t along start→goal
            double best_score = -1.0;
            Eigen::VectorXd best_cand;

            std::uniform_real_distribution<double> t_dist(0.1, 0.9);
            std::normal_distribution<double> perturb(0.0, 0.3 * sg_norm);

            for (int k = 0; k < K_CANDIDATES; ++k) {
                double t = t_dist(rng_);
                Eigen::VectorXd cand = start_config_ + t * sg;

                // Add Gaussian perturbation perpendicular to start→goal
                for (int d = 0; d < n_dims; ++d) {
                    cand[d] += perturb(rng_);
                }
                cand = robot_->joint_limits().clamp(cand);

                // FPS scoring
                double min_dist = std::numeric_limits<double>::max();
                for (const auto& rc : root_centers)
                    min_dist = std::min(min_dist, (cand - rc).norm());

                if (min_dist > best_score) {
                    best_score = min_dist;
                    best_cand = cand;
                }
            }

            int bid = try_create_box(best_cand, obs, n_obs, -1, -1, -1, r, rme);
            if (bid >= 0) {
                root_centers.push_back(boxes_.back().center());
            }
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════
//  partition_subtrees — divide C-space into per-root sub-regions
// ═════════════════════════════════════════════════════════════════════════
void ForestGrower::partition_subtrees() {
    subtrees_.clear();
    if (boxes_.empty()) return;

    int n_dims = robot_->n_joints();
    const auto& jl = robot_->joint_limits().limits;

    // Collect root boxes (parent_box_id == -1 → root)
    struct RootInfo {
        int root_id;
        Eigen::VectorXd seed;
    };
    std::vector<RootInfo> roots;
    for (const auto& b : boxes_) {
        if (b.parent_box_id == -1) {
            roots.push_back({b.root_id, b.seed_config});
        }
    }
    if (roots.empty()) return;

    // Assign each root to a cell by simulating KD-tree descent.
    // Each root starts in the full C-space. We repeatedly split
    // along the cycling dimension until each cell has ≤ 1 root,
    // or max partition depth is reached.
    struct Cell {
        std::vector<Interval> limits;
        std::vector<int> root_indices;   // indices into roots[]
    };

    Cell initial;
    initial.limits.resize(n_dims);
    for (int d = 0; d < n_dims; ++d)
        initial.limits[d] = jl[d];
    for (int i = 0; i < static_cast<int>(roots.size()); ++i)
        initial.root_indices.push_back(i);

    std::vector<Cell> work = {initial};
    std::vector<Cell> done;

    int max_partition_depth = 3 * n_dims;  // avoid infinite splitting
    int split_count = 0;

    while (!work.empty() && split_count < max_partition_depth) {
        std::vector<Cell> next;
        for (auto& cell : work) {
            if (cell.root_indices.size() <= 1) {
                done.push_back(std::move(cell));
                continue;
            }

            // Choose split dimension: round-robin by split_count
            int dim = split_count % n_dims;
            double mid = cell.limits[dim].center();

            Cell left_cell, right_cell;
            left_cell.limits = cell.limits;
            left_cell.limits[dim].hi = mid;
            right_cell.limits = cell.limits;
            right_cell.limits[dim].lo = mid;

            for (int ri : cell.root_indices) {
                if (roots[ri].seed[dim] <= mid)
                    left_cell.root_indices.push_back(ri);
                else
                    right_cell.root_indices.push_back(ri);
            }

            // If split didn't actually separate anything, stop splitting this cell
            if (left_cell.root_indices.empty() || right_cell.root_indices.empty()) {
                done.push_back(std::move(cell));
            } else {
                next.push_back(std::move(left_cell));
                next.push_back(std::move(right_cell));
            }
        }
        work = std::move(next);
        split_count++;
    }

    // Remaining unsplit cells → done
    for (auto& cell : work)
        done.push_back(std::move(cell));

    // Build subtrees_: for each root, record its cell limits
    int n_roots_total = static_cast<int>(roots.size());
    subtrees_.resize(n_roots_total);
    for (const auto& cell : done) {
        for (int ri : cell.root_indices) {
            subtrees_[ri].root_id = roots[ri].root_id;
            subtrees_[ri].limits = cell.limits;
        }
    }

    std::printf("[grower] partition_subtrees: %d roots -> %d cells, %d splits\n",
                n_roots_total, static_cast<int>(done.size()), split_count);
}

// ═════════════════════════════════════════════════════════════════════════
//  partition_uniform — recursive widest-dimension bisection
//
//  Creates exactly n_roots cells by repeatedly splitting the cell with
//  the widest single dimension at its midpoint.  Independent of root
//  positions — gives balanced, deterministic spatial subdivision.
// ═════════════════════════════════════════════════════════════════════════
void ForestGrower::partition_uniform() {
    subtrees_.clear();
    int n_dims  = robot_->n_joints();
    int n_roots = config_.n_roots;
    const auto& jl = robot_->joint_limits().limits;

    // Start with one cell covering full C-space
    struct Cell { std::vector<Interval> limits; };

    std::vector<Cell> cells;
    {
        Cell full;
        full.limits.resize(n_dims);
        for (int d = 0; d < n_dims; ++d)
            full.limits[d] = jl[d];
        cells.push_back(std::move(full));
    }

    // Repeatedly bisect the cell with the widest single dimension
    while (static_cast<int>(cells.size()) < n_roots) {
        // Find (cell, dim) with largest edge
        int best_ci  = 0;
        int best_dim = 0;
        double best_w = 0.0;
        for (int ci = 0; ci < static_cast<int>(cells.size()); ++ci) {
            for (int d = 0; d < n_dims; ++d) {
                double w = cells[ci].limits[d].width();
                if (w > best_w) { best_w = w; best_ci = ci; best_dim = d; }
            }
        }
        double mid = cells[best_ci].limits[best_dim].center();

        Cell right = cells[best_ci];         // copy
        right.limits[best_dim].lo = mid;
        cells[best_ci].limits[best_dim].hi = mid;   // shrink left in-place
        cells.push_back(std::move(right));
    }

    // Assign root_id = index
    subtrees_.resize(n_roots);
    for (int i = 0; i < n_roots; ++i) {
        subtrees_[i].root_id = i;
        subtrees_[i].limits  = std::move(cells[i].limits);
    }

    std::printf("[grower] partition_uniform: %d cells, %d dims\n", n_roots, n_dims);
}

// ═════════════════════════════════════════════════════════════════════════
//  partition_lect_aligned — partition matching LECT KD-tree structure
//
//  Splits C-space using dim = depth % n_dims and midpoint, exactly
//  matching LECT's split_leaf() order.  n_roots must be a power of 2.
//  Also pre-expands the coordinator LECT to partition depth so each
//  resulting cell maps 1:1 to a LECT leaf node.
//
//  Each SubtreeInfo records its corresponding LECT node index
//  (lect_node_idx) for later transplant_subtree() use.
// ═════════════════════════════════════════════════════════════════════════
void ForestGrower::partition_lect_aligned() {
    subtrees_.clear();
    int n_dims  = robot_->n_joints();
    int n_roots = config_.n_roots;
    const auto& jl = robot_->joint_limits().limits;

    // n_roots must be a power of 2
    assert((n_roots & (n_roots - 1)) == 0 && "n_roots must be power of 2 for LECT-aligned partition");

    // Compute partition depth: 2^depth == n_roots
    int partition_depth = 0;
    { int tmp = n_roots; while (tmp > 1) { tmp >>= 1; ++partition_depth; } }

    // Pre-expand coordinator LECT to partition depth so the node indices exist
    int ws_new = lect_.pre_expand(partition_depth);
    std::printf("[grower] partition_lect_aligned: pre-expanded LECT to depth %d, "
                "%d new nodes, %d total\n",
                partition_depth, ws_new, lect_.n_nodes());

    // Recursively build cells matching LECT's split order.
    // Each cell = (limits, lect_node_idx) corresponding to a leaf at partition_depth.
    struct CellState {
        std::vector<Interval> limits;
        int lect_node;                    // corresponding LECT node index
        int depth;
    };

    // BFS-like iterative splitting
    std::vector<CellState> queue;
    {
        CellState root_cell;
        root_cell.limits.resize(n_dims);
        for (int d = 0; d < n_dims; ++d)
            root_cell.limits[d] = jl[d];
        root_cell.lect_node = 0;  // LECT root
        root_cell.depth = 0;
        queue.push_back(std::move(root_cell));
    }

    std::vector<CellState> leaves;
    while (!queue.empty()) {
        std::vector<CellState> next;
        for (auto& cell : queue) {
            if (cell.depth >= partition_depth) {
                // This cell is a leaf at the partition depth
                leaves.push_back(std::move(cell));
                continue;
            }
            // Split on dim = depth % n_dims (matching LECT's split_leaf)
            int dim = cell.depth % n_dims;
            double mid = cell.limits[dim].center();

            // Left child
            CellState left_cell;
            left_cell.limits = cell.limits;
            left_cell.limits[dim].hi = mid;
            left_cell.lect_node = lect_.left(cell.lect_node);
            left_cell.depth = cell.depth + 1;

            // Right child
            CellState right_cell;
            right_cell.limits = cell.limits;
            right_cell.limits[dim].lo = mid;
            right_cell.lect_node = lect_.right(cell.lect_node);
            right_cell.depth = cell.depth + 1;

            next.push_back(std::move(left_cell));
            next.push_back(std::move(right_cell));
        }
        queue = std::move(next);
    }

    assert(static_cast<int>(leaves.size()) == n_roots);

    // Build subtrees_ from leaves
    subtrees_.resize(n_roots);
    for (int i = 0; i < n_roots; ++i) {
        subtrees_[i].root_id       = i;
        subtrees_[i].limits        = std::move(leaves[i].limits);
        subtrees_[i].lect_node_idx = leaves[i].lect_node;
    }

    std::printf("[grower] partition_lect_aligned: %d cells, %d dims, "
                "partition_depth=%d\n", n_roots, n_dims, partition_depth);
}

// ═════════════════════════════════════════════════════════════════════════
//  select_roots_in_partitions — place one root per partition cell
//
//  For each cell in subtrees_ (populated by partition_uniform()),
//  use best-of-N selection: sample N candidate seeds within the cell,
//  evaluate each via find_free_box(), and keep the seed that produces
//  the largest free volume.  This ensures roots land in the largest
//  free regions within each cell (similar to FPS quality), while also
//  enriching the LECT tree as a side effect.
// ═════════════════════════════════════════════════════════════════════════
void ForestGrower::select_roots_in_partitions(const Obstacle* obs, int n_obs) {
    double rme = (config_.root_min_edge > 0.0) ? config_.root_min_edge : -1.0;
    double probe_me = (rme > 0.0) ? rme : config_.min_edge;
    int n_dims = robot_->n_joints();

    constexpr int N_CANDIDATES = 50;

    for (int ci = 0; ci < static_cast<int>(subtrees_.size()); ++ci) {
        const auto& lims = subtrees_[ci].limits;
        int rid = subtrees_[ci].root_id;

        // ── Best-of-N: probe free volume at N random seeds ──────────
        Eigen::VectorXd best_seed;
        double best_vol = -1.0;

        for (int k = 0; k < N_CANDIDATES; ++k) {
            Eigen::VectorXd seed(n_dims);
            std::uniform_real_distribution<double> u01(0.0, 1.0);
            for (int d = 0; d < n_dims; ++d)
                seed[d] = lims[d].lo + u01(rng_) * lims[d].width();

            // Probe: find_free_box locates the LECT leaf for this seed.
            // This also builds up the LECT tree (beneficial side effect).
            FFBResult ffb = lect_.find_free_box(
                seed, obs, n_obs, probe_me, config_.max_depth);
            if (!ffb.success()) continue;

            // Compute free-box volume from node intervals
            auto intervals = lect_.node_intervals(ffb.node_idx);
            double vol = 1.0;
            for (const auto& iv : intervals)
                vol *= iv.width();

            if (vol > best_vol) {
                best_vol = vol;
                best_seed = seed;
            }
        }

        // ── Place root box at best seed ─────────────────────────────
        if (best_vol > 0.0) {
            int bid = try_create_box(best_seed, obs, n_obs, -1, -1, -1, rid, rme);
            if (bid >= 0) {
                std::printf("[grower] cell %d: best-of-%d root, probe_vol=%.4f\n",
                            ci, N_CANDIDATES, best_vol);
                continue;
            }
        }

        // ── Fallback: try random seeds until one works ──────────────
        for (int attempt = 0; attempt < 100; ++attempt) {
            Eigen::VectorXd seed(n_dims);
            std::uniform_real_distribution<double> u01(0.0, 1.0);
            for (int d = 0; d < n_dims; ++d)
                seed[d] = lims[d].lo + u01(rng_) * lims[d].width();

            int bid = try_create_box(seed, obs, n_obs, -1, -1, -1, rid, rme);
            if (bid >= 0) {
                std::printf("[grower] cell %d: fallback root (attempt %d)\n",
                            ci, attempt);
                break;
            }
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════
//  grow_wavefront — BFS boundary expansion
// ═════════════════════════════════════════════════════════════════════════
void ForestGrower::grow_wavefront(const Obstacle* obs, int n_obs) {
    auto t0 = std::chrono::high_resolution_clock::now();
    int miss_count = 0;
    int n_dims = robot_->n_joints();

    // ── Multi-stage adaptive min_edge schedule ──────────────────────────
    //
    //  Stage progression by halving (逐步二分):
    //    stage 0: coarse_min_edge
    //    stage 1: coarse_min_edge / 2
    //    stage 2: coarse_min_edge / 4
    //    ...
    //    stage N-1: min_edge  (final stage)
    //
    //  Budget is distributed evenly across all stages.
    //  At each stage transition, the wavefront queue is re-seeded with
    //  all existing boxes so that the finer min_edge can explore sites
    //  that were unreachable before.
    // ────────────────────────────────────────────────────────────────────
    struct AdaptStage { double min_edge; int box_limit; };
    std::vector<AdaptStage> stages;

    if (config_.adaptive_min_edge) {
        int n_stages = std::max(2, config_.adaptive_n_stages);
        double me = config_.coarse_min_edge;
        for (int s = 0; s < n_stages - 1; ++s) {
            int limit = static_cast<int>(
                config_.max_boxes * config_.coarse_fraction * (s + 1.0) / (n_stages - 1));
            stages.push_back({me, limit});
            me = std::max(me * 0.5, config_.min_edge);
        }
        stages.push_back({config_.min_edge, config_.max_boxes});
    } else {
        stages.push_back({config_.min_edge, config_.max_boxes});
    }
    int current_stage = 0;

    auto current_box_count = [&]() -> int {
        return shared_box_count_
            ? shared_box_count_->load(std::memory_order_relaxed)
            : static_cast<int>(boxes_.size());
    };
    auto effective_min_edge = [&]() -> double {
        return stages[current_stage].min_edge;
    };
    auto is_last_stage = [&]() -> bool {
        return current_stage >= static_cast<int>(stages.size()) - 1;
    };
    auto advance_stage = [&]() {
        if (is_last_stage()) return;
        current_stage++;
        miss_count = 0;
    };

    // Wavefront queue: priority queue ordered by parent box volume (large first)
    struct WaveEntry {
        int box_id;
        int from_face_dim;
        int from_face_side;
        double priority;  // parent box volume — larger boxes expanded first
        bool operator<(const WaveEntry& o) const { return priority < o.priority; }
    };

    std::priority_queue<WaveEntry> queue;
    for (const auto& box : boxes_) {
        queue.push({box.id, -1, -1, box.volume});
    }

    // Budget check: use shared atomic counter if available (work-stealing),
    // otherwise use local boxes_.size()
    auto budget_ok = [&]() -> bool {
        if (shared_box_count_) {
            return shared_box_count_->load(std::memory_order_relaxed) < config_.max_boxes;
        }
        return static_cast<int>(boxes_.size()) < config_.max_boxes;
    };

    while (budget_ok()
           && miss_count < config_.max_consecutive_miss) {
        // Timeout check
        auto now = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(now - t0).count();
        if (elapsed > config_.timeout) {
            std::printf("[grower] wavefront timeout at %.1f s\n", elapsed);
            break;
        }

        // ── Adaptive stage transition ────────────────────────────────────
        if (!is_last_stage() && current_box_count() >= stages[current_stage].box_limit) {
            advance_stage();
            // Re-queue ALL existing boxes for expansion with finer min_edge
            while (!queue.empty()) queue.pop();
            for (const auto& box : boxes_)
                queue.push({box.id, -1, -1, box.volume});
        }

        if (!queue.empty()) {
            // Pop from wavefront queue (highest-volume box first)
            WaveEntry entry = queue.top();
            queue.pop();

            // Find the box via O(1) lookup
            auto it = box_id_to_idx_.find(entry.box_id);
            if (it == box_id_to_idx_.end()) continue;

            // IMPORTANT: Copy needed fields BEFORE the for-loop.
            // try_create_box() calls boxes_.push_back() which can
            // reallocate the vector and invalidate any reference into it.
            const int expand_from_id = boxes_[it->second].id;
            const int expand_root_id = boxes_[it->second].root_id;

            // Per-tree bias target: start-tree→goal, goal-tree→start, random-tree→50/50
            const Eigen::VectorXd* bias_ptr = get_bias_target(expand_root_id);

            // Generate boundary seeds (reads box intervals, safe here)
            auto seeds = sample_boundary(boxes_[it->second], bias_ptr);

            for (const auto& bs : seeds) {
                if (!budget_ok()) break;

                int bid = try_create_box(
                    bs.config, obs, n_obs,
                    expand_from_id, bs.dim, bs.side,
                    expand_root_id, effective_min_edge());

                if (bid >= 0) {
                    graph_.add_box(boxes_.back());
                    queue.push({bid, bs.dim, 1 - bs.side, boxes_.back().volume});
                    miss_count = 0;
                    (is_last_stage() ? n_fine_created_ : n_coarse_created_)++;
                } else {
                    miss_count++;
                }
            }
        } else {
            // Queue empty: boundary-aware fallback
            // First try sampling from an existing box's boundary face
            // (keeps new box adjacent to tree, avoids isolated fragments).
            // Falls back to subtree random if boundary sampling fails.

            // Non-final stage refill: try boundary samples before advancing
            if (!is_last_stage()) {
                bool refilled = false;
                for (int attempt = 0; attempt < 30; ++attempt) {
                    int rand_root = -1;
                    if (!subtrees_.empty()) {
                        std::uniform_int_distribution<int> root_dist(
                            0, static_cast<int>(subtrees_.size()) - 1);
                        rand_root = subtrees_[root_dist(rng_)].root_id;
                    }
                    Eigen::VectorXd seed = sample_near_existing_boundary(rand_root);
                    if (seed.size() == 0) break;  // no valid boundary
                    int bid = try_create_box(seed, obs, n_obs, -1, -1, -1,
                                            rand_root, effective_min_edge());
                    if (bid >= 0) {
                        graph_.add_box(boxes_.back());
                        queue.push({bid, -1, -1, boxes_.back().volume});
                        miss_count = 0;
                        n_coarse_created_++;
                        refilled = true;
                        break;
                    }
                }
                if (refilled) continue;
                // Refill failed → advance to next stage
                advance_stage();
                while (!queue.empty()) queue.pop();
                for (const auto& box : boxes_)
                    queue.push({box.id, -1, -1, box.volume});
                continue;
            }

            // Final-stage fallback: boundary then random
            int rand_root = -1;
            if (!subtrees_.empty()) {
                std::uniform_int_distribution<int> root_dist(
                    0, static_cast<int>(subtrees_.size()) - 1);
                rand_root = subtrees_[root_dist(rng_)].root_id;
            }
            Eigen::VectorXd seed = sample_near_existing_boundary(rand_root);
            if (seed.size() == 0) {
                seed = (rand_root >= 0)
                    ? sample_random_in_subtree(rand_root)
                    : sample_random();
            }
            // Inherit root_id from nearest existing box
            int inherit_root = rand_root;
            if (inherit_root < 0 && !boxes_.empty()) {
                int nid = graph_.find_nearest_box(seed);
                if (nid >= 0) {
                    auto jt = box_id_to_idx_.find(nid);
                    if (jt != box_id_to_idx_.end())
                        inherit_root = boxes_[jt->second].root_id;
                }
            }
            int bid = try_create_box(seed, obs, n_obs, -1, -1, -1,
                                    inherit_root, effective_min_edge());
            if (bid >= 0) {
                graph_.add_box(boxes_.back());
                queue.push({bid, -1, -1, boxes_.back().volume});
                miss_count = 0;
                n_fine_created_++;
            } else {
                miss_count++;
            }
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════
//  sample_boundary — generate boundary seeds from a box
// ═════════════════════════════════════════════════════════════════════════
std::vector<ForestGrower::BoundarySeed>
ForestGrower::sample_boundary(const BoxNode& box,
                              const Eigen::VectorXd* bias_target) {
    std::vector<BoundarySeed> seeds;
    int n_dims = box.n_dims();
    const auto& limits = robot_->joint_limits().limits;
    double eps = config_.boundary_epsilon;

    // Collect valid faces (not out of joint limits)
    struct Face { int dim; int side; double priority; };
    std::vector<Face> faces;
    for (int d = 0; d < n_dims; ++d) {
        // lo face (side=0): seed is below box.lo
        double lo_seed = box.joint_intervals[d].lo - eps;
        if (lo_seed >= limits[d].lo)
            faces.push_back({d, 0, 0.0});

        // hi face (side=1): seed is above box.hi
        double hi_seed = box.joint_intervals[d].hi + eps;
        if (hi_seed <= limits[d].hi)
            faces.push_back({d, 1, 0.0});
    }

    // Skip the face we came from (parent expand face)
    if (box.expand_face_dim >= 0) {
        int skip_dim = box.expand_face_dim;
        int skip_side = box.expand_face_side;
        faces.erase(
            std::remove_if(faces.begin(), faces.end(),
                [skip_dim, skip_side](const Face& f) {
                    return f.dim == skip_dim && f.side == skip_side;
                }),
            faces.end());
    }

    if (faces.empty()) return seeds;

    // Goal-directed face priority (uses per-tree bias target)
    if (bias_target) {
        Eigen::VectorXd box_center = box.center();
        Eigen::VectorXd to_target = *bias_target - box_center;
        for (auto& f : faces) {
            // Priority = how much this face points toward target
            double val = (f.side == 1) ? to_target[f.dim] : -to_target[f.dim];
            f.priority = val;
        }
        // Sort descending by priority
        std::sort(faces.begin(), faces.end(),
                  [](const Face& a, const Face& b) { return a.priority > b.priority; });
    } else {
        // Random shuffle
        std::shuffle(faces.begin(), faces.end(), rng_);
    }

    // Generate seeds for top faces
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    int n_samples = std::min(config_.n_boundary_samples,
                             static_cast<int>(faces.size()));

    // With bias: select target-facing face with higher probability
    for (int s = 0; s < n_samples; ++s) {
        int face_idx;
        if (bias_target && u01(rng_) < config_.goal_face_bias && !faces.empty()) {
            face_idx = 0;  // Best goal-facing face
        } else {
            face_idx = s % static_cast<int>(faces.size());
        }
        const Face& face = faces[face_idx];

        // Generate seed: center on the face, jitter in other dims
        Eigen::VectorXd seed(n_dims);
        for (int d = 0; d < n_dims; ++d) {
            if (d == face.dim) {
                if (face.side == 0) {
                    seed[d] = box.joint_intervals[d].lo - eps;
                } else {
                    seed[d] = box.joint_intervals[d].hi + eps;
                }
            } else {
                // Random within box range
                double lo = box.joint_intervals[d].lo;
                double hi = box.joint_intervals[d].hi;
                seed[d] = lo + u01(rng_) * (hi - lo);
            }
        }

        // Clamp to joint limits
        seed = robot_->joint_limits().clamp(seed);

        seeds.push_back({face.dim, face.side, seed});
    }

    return seeds;
}

// ═════════════════════════════════════════════════════════════════════════
//  grow_rrt — RRT-like expansion
// ═════════════════════════════════════════════════════════════════════════
void ForestGrower::grow_rrt(const Obstacle* obs, int n_obs) {
    auto t0 = std::chrono::high_resolution_clock::now();
    int miss_count = 0;
    int n_dims = robot_->n_joints();
    const auto& limits = robot_->joint_limits().limits;

    // Compute max dimension width for step size
    double max_width = 0.0;
    for (int d = 0; d < n_dims; ++d)
        max_width = std::max(max_width, limits[d].width());
    double step = config_.rrt_step_ratio * max_width;

    // Adaptive min_edge: coarse→fine two-phase strategy
    int coarse_limit = config_.adaptive_min_edge
        ? static_cast<int>(config_.max_boxes * config_.coarse_fraction)
        : 0;
    bool in_coarse_phase = config_.adaptive_min_edge;
    auto current_box_count = [&]() -> int {
        return shared_box_count_
            ? shared_box_count_->load(std::memory_order_relaxed)
            : static_cast<int>(boxes_.size());
    };
    auto effective_min_edge = [&]() -> double {
        if (!config_.adaptive_min_edge) return config_.min_edge;
        return in_coarse_phase ? config_.coarse_min_edge : config_.min_edge;
    };

    std::uniform_real_distribution<double> u01(0.0, 1.0);

    // Budget check: use shared atomic counter if available (work-stealing),
    // otherwise use local boxes_.size()
    auto budget_ok = [&]() -> bool {
        if (shared_box_count_) {
            return shared_box_count_->load(std::memory_order_relaxed) < config_.max_boxes;
        }
        return static_cast<int>(boxes_.size()) < config_.max_boxes;
    };

    while (budget_ok()
           && miss_count < config_.max_consecutive_miss) {
        // Timeout check
        auto now = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(now - t0).count();
        if (elapsed > config_.timeout) {
            std::printf("[grower] RRT timeout at %.1f s\n", elapsed);
            break;
        }

        // ── Adaptive phase transition: coarse → fine ────────────────────
        if (in_coarse_phase && current_box_count() >= coarse_limit) {
            in_coarse_phase = false;
            // Reset miss count: fine min_edge opens up many new FFB sites
            miss_count = 0;
        }

        // Sample random target (subtree-constrained when possible)
        Eigen::VectorXd q_rand;
        if (has_endpoints_ && u01(rng_) < config_.rrt_goal_bias) {
            // Per-tree bias: pick a random tree, use its bias target
            // start-tree(0)→goal, goal-tree(1)→start, random-tree(≥2)→50/50
            int chosen_root = -1;
            if (!boxes_.empty()) {
                std::uniform_int_distribution<int> box_dist(
                    0, static_cast<int>(boxes_.size()) - 1);
                chosen_root = boxes_[box_dist(rng_)].root_id;
            }
            const Eigen::VectorXd* bias = get_bias_target(chosen_root);
            q_rand = bias ? *bias : sample_random();
        } else if (!subtrees_.empty()) {
            // Sample within a random root's subtree for balanced coverage
            std::uniform_int_distribution<int> root_dist(
                0, static_cast<int>(subtrees_.size()) - 1);
            int ri = root_dist(rng_);
            q_rand = sample_random_in_subtree(subtrees_[ri].root_id);
        } else {
            q_rand = sample_random();
        }

        // Find nearest box
        int nearest_id = graph_.find_nearest_box(q_rand);
        if (nearest_id < 0) {
            miss_count++;
            continue;
        }

        // Find the box via O(1) lookup
        auto it = box_id_to_idx_.find(nearest_id);
        if (it == box_id_to_idx_.end()) { miss_count++; continue; }

        // Copy fields before any potential reallocation
        const int extend_from_id = boxes_[it->second].id;
        const int extend_root_id = boxes_[it->second].root_id;
        Eigen::VectorXd nearest_center = boxes_[it->second].center();

        // Compute direction toward target
        Eigen::VectorXd direction = q_rand - nearest_center;
        double dir_norm = direction.norm();
        if (dir_norm > 1e-12) {
            direction = direction / dir_norm;  // normalize
        } else {
            // Random direction if q_rand == center
            direction = Eigen::VectorXd::Random(n_dims);
            dir_norm = direction.norm();
            if (dir_norm < 1e-12) { miss_count++; continue; }
            direction /= dir_norm;
        }

        // ── Boundary-snap: place seed on the best face of nearest box ──
        // Snap to face ensures new box is adjacent to existing tree,
        // preventing isolated small boxes in the C-space.
        auto snap = rrt_snap_to_face(boxes_[it->second], direction, step);

        // Try to create box with face info for adjacency tracking
        int bid = try_create_box(
            snap.seed, obs, n_obs,
            extend_from_id, snap.face_dim, snap.face_side,
            extend_root_id, effective_min_edge());

        if (bid >= 0) {
            graph_.add_box(boxes_.back());
            miss_count = 0;
            (in_coarse_phase ? n_coarse_created_ : n_fine_created_)++;
        } else {
            miss_count++;
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════
//  try_create_box — FFB wrapper
// ═════════════════════════════════════════════════════════════════════════
int ForestGrower::try_create_box(const Eigen::VectorXd& seed,
                                 const Obstacle* obs, int n_obs,
                                 int parent_box_id,
                                 int face_dim, int face_side,
                                 int root_id,
                                 double min_edge_override) {
    double me = (min_edge_override > 0.0) ? min_edge_override : config_.min_edge;
    FFBResult ffb = lect_.find_free_box(
        seed, obs, n_obs,
        me, config_.max_depth);

    if (!ffb.success()) return -1;

    // Check if this node is already occupied
    if (lect_.is_occupied(ffb.node_idx)) return -1;

    // Build BoxNode
    std::vector<Interval> intervals = lect_.node_intervals(ffb.node_idx);
    int box_id = next_box_id_++;

    BoxNode box(box_id, intervals, seed);
    box.tree_id = ffb.node_idx;
    box.parent_box_id = parent_box_id;
    box.expand_face_dim = face_dim;
    box.expand_face_side = face_side;
    box.root_id = root_id;

    lect_.mark_occupied(ffb.node_idx, box_id);
    box_id_to_idx_[box_id] = static_cast<int>(boxes_.size());
    boxes_.push_back(std::move(box));

    // Increment shared counter for work-stealing budget enforcement
    if (shared_box_count_) {
        shared_box_count_->fetch_add(1, std::memory_order_relaxed);
    }

    return box_id;
}

// ═════════════════════════════════════════════════════════════════════════
//  promote_all — iterative bottom-up promotion until convergence
// ═════════════════════════════════════════════════════════════════════════
int ForestGrower::promote_all(const Obstacle* obs, int n_obs) {
    int total_promoted = 0;
    bool changed = true;

    while (changed) {
        changed = false;
        int n_nodes = lect_.n_nodes();

        for (int i = 0; i < n_nodes; ++i) {
            if (lect_.is_leaf(i)) continue;
            if (try_promote_node(i, obs, n_obs)) {
                total_promoted++;
                changed = true;
            }
        }
    }

    std::printf("[grower] promote_all: %d promotions\n", total_promoted);
    return total_promoted;
}

// ─────────────────────────────────────────────────────────────────────────
//  try_promote_node — check if both children are occupied leaves, merge
// ─────────────────────────────────────────────────────────────────────────
bool ForestGrower::try_promote_node(int node_idx,
                                    const Obstacle* obs, int n_obs) {
    int li = lect_.left(node_idx);
    int ri = lect_.right(node_idx);
    if (li < 0 || ri < 0) return false;

    // Both children must be occupied leaves
    if (!lect_.is_leaf(li) || !lect_.is_leaf(ri)) return false;
    if (!lect_.is_occupied(li) || !lect_.is_occupied(ri)) return false;

    // Parent must not already be occupied
    if (lect_.is_occupied(node_idx)) return false;

    // Collision check on parent depends on envelope type
    bool ok;
    if (is_grid_envelope()) {
        // Grid merge + collision check
        if (!lect_.merge_children_hulls(node_idx)) return false;
        ok = !lect_.collides_scene(node_idx, obs, n_obs);
    } else {
        // AABB collision check
        if (!lect_.has_aabb(node_idx)) return false;
        ok = !lect_.collides_scene(node_idx, obs, n_obs);
    }
    if (!ok) return false;

    // Promotion succeeded
    int li_box_id = lect_.forest_id(li);
    int ri_box_id = lect_.forest_id(ri);

    lect_.unmark_occupied(li);
    lect_.unmark_occupied(ri);

    // Remember child metadata for the promoted box via O(1) lookup
    int promoted_root_id = -1;
    {
        auto it_li = box_id_to_idx_.find(li_box_id);
        if (it_li != box_id_to_idx_.end() && boxes_[it_li->second].root_id >= 0)
            promoted_root_id = boxes_[it_li->second].root_id;
        auto it_ri = box_id_to_idx_.find(ri_box_id);
        if (it_ri != box_id_to_idx_.end() && boxes_[it_ri->second].root_id >= 0)
            promoted_root_id = boxes_[it_ri->second].root_id;
    }

    // Remove children from boxes_ and rebuild index map for moved elements
    // Collect indices to remove (descending order to avoid invalidation)
    std::vector<int> remove_indices;
    {
        auto it = box_id_to_idx_.find(li_box_id);
        if (it != box_id_to_idx_.end()) remove_indices.push_back(it->second);
        it = box_id_to_idx_.find(ri_box_id);
        if (it != box_id_to_idx_.end()) remove_indices.push_back(it->second);
    }
    std::sort(remove_indices.rbegin(), remove_indices.rend());

    box_id_to_idx_.erase(li_box_id);
    box_id_to_idx_.erase(ri_box_id);

    for (int idx : remove_indices) {
        int last = static_cast<int>(boxes_.size()) - 1;
        if (idx < last) {
            // Swap with last element and update its index
            box_id_to_idx_[boxes_[last].id] = idx;
            boxes_[idx] = std::move(boxes_[last]);
        }
        boxes_.pop_back();
    }

    // Create promoted parent box
    int new_id = next_box_id_++;
    std::vector<Interval> parent_ivs = lect_.node_intervals(node_idx);
    Eigen::VectorXd parent_center(static_cast<int>(parent_ivs.size()));
    for (int d = 0; d < static_cast<int>(parent_ivs.size()); ++d)
        parent_center[d] = parent_ivs[d].center();

    BoxNode parent_box(new_id, parent_ivs, parent_center);
    parent_box.tree_id = node_idx;
    parent_box.root_id = promoted_root_id;

    lect_.mark_occupied(node_idx, new_id);

    // Update start/goal box IDs if children were endpoints
    if (li_box_id == start_box_id_ || ri_box_id == start_box_id_)
        start_box_id_ = new_id;
    if (li_box_id == goal_box_id_ || ri_box_id == goal_box_id_)
        goal_box_id_ = new_id;

    box_id_to_idx_[new_id] = static_cast<int>(boxes_.size());
    boxes_.push_back(std::move(parent_box));
    return true;
}

// ═════════════════════════════════════════════════════════════════════════
//  remove_box_by_id — swap-with-last removal from boxes_ + index map
// ═════════════════════════════════════════════════════════════════════════
void ForestGrower::remove_box_by_id(int box_id) {
    auto it = box_id_to_idx_.find(box_id);
    if (it == box_id_to_idx_.end()) return;

    int idx = it->second;
    int last = static_cast<int>(boxes_.size()) - 1;

    // Unmark LECT occupation
    if (boxes_[idx].tree_id >= 0)
        lect_.unmark_occupied(boxes_[idx].tree_id);

    box_id_to_idx_.erase(box_id);
    if (idx < last) {
        box_id_to_idx_[boxes_[last].id] = idx;
        boxes_[idx] = std::move(boxes_[last]);
    }
    boxes_.pop_back();
}

// ═════════════════════════════════════════════════════════════════════════
//  coarsen_greedy — greedy adjacency-based hull merge
//
//  Per round:
//    1. Rebuild adjacency graph
//    2. For each adjacent pair compute hull-AABB and collision-check
//    3. Sort by merge score (hull_vol / sum_vol, lower = better)
//    4. Greedily execute non-conflicting merges (each box at most once)
//    5. Repeat until target or convergence
//
//  Returns total number of merges performed.
// ═════════════════════════════════════════════════════════════════════════
int ForestGrower::coarsen_greedy(const Obstacle* obs, int n_obs) {
    const int target = config_.coarsen_target_boxes;
    const int max_rounds = config_.max_coarsen_rounds;
    const double score_thr = config_.coarsen_score_threshold;
    const int n_dims = static_cast<int>(robot_->n_joints());

    int total_merges = 0;

    if (target > 0 && n_boxes() <= target) return 0;

    for (int round = 0; round < max_rounds; ++round) {

        if (target > 0 && n_boxes() <= target) break;

        // Rebuild adjacency for current boxes
        graph_.rebuild(boxes_);

        // ── Collect merge candidates ────────────────────────────────────
        struct Candidate {
            double score;
            int id_a, id_b;
            std::vector<Interval> hull_ivs;
        };
        std::vector<Candidate> candidates;
        candidates.reserve(boxes_.size());

        // Iterate adjacency: for each box, scan its neighbors
        for (const auto& box_a : boxes_) {
            const auto& nbrs = graph_.neighbors(box_a.id);
            for (int id_b : nbrs) {
                if (id_b <= box_a.id) continue;  // each pair once

                auto it_b = box_id_to_idx_.find(id_b);
                if (it_b == box_id_to_idx_.end()) continue;
                const BoxNode& box_b = boxes_[it_b->second];

                // Compute hull intervals
                std::vector<Interval> hull_ivs(n_dims);
                double hull_vol = 1.0;
                for (int d = 0; d < n_dims; ++d) {
                    hull_ivs[d].lo = std::min(box_a.joint_intervals[d].lo,
                                               box_b.joint_intervals[d].lo);
                    hull_ivs[d].hi = std::max(box_a.joint_intervals[d].hi,
                                               box_b.joint_intervals[d].hi);
                    hull_vol *= hull_ivs[d].width();
                }

                double sum_vol = box_a.volume + box_b.volume;
                double score = (sum_vol > 0) ? hull_vol / sum_vol : 1e18;

                if (score > score_thr) continue;

                candidates.push_back({score, box_a.id, id_b, std::move(hull_ivs)});
            }
        }

        if (candidates.empty()) break;

        // Sort by score (lower = better merge)
        std::sort(candidates.begin(), candidates.end(),
                  [](const Candidate& a, const Candidate& b) {
                      return a.score < b.score;
                  });

        // ── Greedy execute ──────────────────────────────────────────────
        std::unordered_set<int> merged_this_round;
        int merges_this_round = 0;

        for (auto& cand : candidates) {
            if (target > 0 &&
                n_boxes() - merges_this_round <= target)
                break;

            if (merged_this_round.count(cand.id_a) ||
                merged_this_round.count(cand.id_b))
                continue;

            // Verify both boxes still exist
            auto it_a = box_id_to_idx_.find(cand.id_a);
            auto it_b = box_id_to_idx_.find(cand.id_b);
            if (it_a == box_id_to_idx_.end() ||
                it_b == box_id_to_idx_.end())
                continue;

            // Collision check on hull intervals
            bool hull_collides = lect_.intervals_collide_scene(
                cand.hull_ivs, obs, n_obs);
            if (hull_collides) continue;

            // Remember metadata from box A
            const BoxNode& ba = boxes_[it_a->second];
            Eigen::VectorXd seed = ba.seed_config;
            int root_id = ba.root_id;

            // Remove both boxes
            remove_box_by_id(cand.id_a);
            remove_box_by_id(cand.id_b);

            // Create merged box
            int new_id = next_box_id_++;
            BoxNode merged(new_id, cand.hull_ivs, seed);
            merged.root_id = root_id;
            merged.tree_id = -1;  // no LECT node for hull merge

            box_id_to_idx_[new_id] = static_cast<int>(boxes_.size());
            boxes_.push_back(std::move(merged));

            // Update start/goal if either was merged
            if (cand.id_a == start_box_id_ || cand.id_b == start_box_id_)
                start_box_id_ = new_id;
            if (cand.id_a == goal_box_id_ || cand.id_b == goal_box_id_)
                goal_box_id_ = new_id;

            merged_this_round.insert(cand.id_a);
            merged_this_round.insert(cand.id_b);
            merges_this_round++;
            total_merges++;
        }

        if (merges_this_round == 0) break;

        if (round < 5 || (round + 1) % 10 == 0) {
            std::printf("    [coarsen_greedy] round %d: merged %d pairs → %d boxes\n",
                        round + 1, merges_this_round, n_boxes());
        }
    }

    // Final adjacency rebuild
    graph_.rebuild(boxes_);

    std::printf("[grower] coarsen_greedy: %d merges, %d → %d boxes\n",
                total_merges, total_merges > 0 ? n_boxes() + total_merges : n_boxes(),
                n_boxes());
    return total_merges;
}

// ═════════════════════════════════════════════════════════════════════════
//  grow_subtree — single-root expansion for parallel workers
// ═════════════════════════════════════════════════════════════════════════
GrowerResult ForestGrower::grow_subtree(
    const Eigen::VectorXd& root_seed,
    int root_id,
    const std::vector<Interval>& subtree_limits,
    const Obstacle* obs, int n_obs,
    std::shared_ptr<std::atomic<int>> shared_box_count)
{
    GrowerResult result;

    shared_box_count_ = shared_box_count;

    boxes_.clear();
    box_id_to_idx_.clear();
    graph_.clear();
    next_box_id_ = 0;
    start_box_id_ = -1;
    goal_box_id_ = -1;
    n_coarse_created_ = 0;
    n_fine_created_   = 0;

    // Set scene on LECT for grid-based envelopes (skip if warm-start already has it)
    if (is_grid_envelope() && !lect_.has_scene_grid()) {
        lect_.set_scene(obs, n_obs);
    }

    // Place root box (use root_min_edge for larger initial boxes)
    double rme = (config_.root_min_edge > 0.0) ? config_.root_min_edge : -1.0;
    int bid = try_create_box(root_seed, obs, n_obs, -1, -1, -1, root_id, rme);
    if (bid < 0) {
        return result;
    }
    graph_.add_box(boxes_.back());

    // Set up single subtree for this root
    subtrees_.clear();
    subtrees_.push_back({root_id, subtree_limits});

    // Expansion
    if (config_.mode == GrowerConfig::Mode::Wavefront) {
        grow_wavefront(obs, n_obs);
    } else {
        grow_rrt(obs, n_obs);
    }

    // Per-worker promotion
    result.n_promotions = promote_all(obs, n_obs);

    // Collect
    result.boxes = boxes_;
    result.n_boxes_total = static_cast<int>(boxes_.size());
    result.n_roots = 1;
    result.n_coarse_boxes = n_coarse_created_;
    result.n_fine_boxes   = n_fine_created_;

    return result;
}

// ═════════════════════════════════════════════════════════════════════════
//  grow_parallel — dispatch per-subtree workers via ThreadPool
//
//  Uses LECT-aligned partitioning: each worker's LECT subtree maps 1:1
//  to a coordinator LECT leaf.  After growth, worker LECTs are
//  transplanted back into the coordinator via transplant_subtree(),
//  preserving cached envelopes and occupation state.
// ═════════════════════════════════════════════════════════════════════════
void ForestGrower::grow_parallel(const Obstacle* obs, int n_obs,
                                 GrowerResult& result) {
    int n_subtrees = static_cast<int>(subtrees_.size());
    int n_workers = std::min(config_.n_threads, n_subtrees);

    // Work-stealing: shared atomic counter for global budget enforcement
    int n_root_boxes = static_cast<int>(boxes_.size());
    auto shared_counter = std::make_shared<std::atomic<int>>(n_root_boxes);

    // Collect root seeds indexed by root_id
    std::unordered_map<int, Eigen::VectorXd> root_seeds;
    for (const auto& b : boxes_) {
        if (b.parent_box_id == -1) {
            root_seeds[b.root_id] = b.seed_config;
        }
    }

    // Shared read-only data
    const Robot* robot_ptr = robot_;
    bool has_ep = has_endpoints_;
    Eigen::VectorXd start_cfg = has_ep ? start_config_ : Eigen::VectorXd();
    Eigen::VectorXd goal_cfg  = has_ep ? goal_config_  : Eigen::VectorXd();

    // ── LECT warm-start: additional pre-expand beyond partition depth ───
    // partition_lect_aligned() already pre-expanded to partition_depth.
    // If ws_depth > partition_depth, expand further for better warm-start.
    bool use_warm_start = (config_.warm_start_depth >= 0);
    int ws_depth = config_.warm_start_depth;
    if (ws_depth == 0) {
        ws_depth = 3;
    }

    if (use_warm_start && ws_depth > 0) {
        auto t_ws0 = std::chrono::high_resolution_clock::now();
        int ws_new_nodes = lect_.pre_expand(ws_depth);
        auto t_ws1 = std::chrono::high_resolution_clock::now();
        double ws_ms = std::chrono::duration<double, std::milli>(t_ws1 - t_ws0).count();
        result.phase_times["warm_start_ms"] = ws_ms;
        std::printf("[grower] warm-start: pre-expanded to depth %d, "
                    "%d new nodes, lect %d total, %.1f ms\n",
                    ws_depth, ws_new_nodes, lect_.n_nodes(), ws_ms);
    }

    // Pre-set scene if grid-based (shared across snapshots)
    if (is_grid_envelope() && !lect_.has_scene_grid()) {
        lect_.set_scene(obs, n_obs);
    }

    // ── Record snapshot_n_nodes before snapshotting ─────────────────────
    int snapshot_n_nodes = lect_.n_nodes();

    // Check if partitions are LECT-aligned (lect_node_idx populated)
    bool lect_aligned = (n_subtrees > 0 && subtrees_[0].lect_node_idx >= 0);

    std::printf("[grower] parallel: %d subtrees, %d workers, work-stealing budget %d"
                "%s%s, snapshot_n_nodes=%d\n",
                n_subtrees, n_workers, config_.max_boxes,
                use_warm_start ? " [warm-start]" : "",
                lect_aligned ? " [lect-aligned]" : "",
                snapshot_n_nodes);

    // Create thread pool and submit tasks
    ThreadPool pool(n_workers);
    std::vector<std::future<ParallelWorkerResult>> futures;
    std::vector<int> future_subtree_idx;  // maps future index → subtrees_ index

    for (int i = 0; i < n_subtrees; ++i) {
        int rid = subtrees_[i].root_id;
        auto it = root_seeds.find(rid);
        if (it == root_seeds.end()) continue;

        Eigen::VectorXd seed = it->second;
        std::vector<Interval> limits = subtrees_[i].limits;

        GrowerConfig worker_cfg = config_;
        worker_cfg.max_boxes = config_.max_boxes;
        worker_cfg.rng_seed  = config_.rng_seed + static_cast<uint64_t>(i) * 12345ULL + 1;
        worker_cfg.n_threads = 1;
        worker_cfg.warm_start_depth = -1;

        if (use_warm_start) {
            auto warm_ptr = std::make_shared<LECT>(lect_.snapshot());

            futures.push_back(pool.submit(
                [robot_ptr, worker_cfg, has_ep, start_cfg, goal_cfg,
                 seed, rid, limits, obs, n_obs, shared_counter,
                 warm_ptr]() -> ParallelWorkerResult
                {
                    ForestGrower worker(*robot_ptr, worker_cfg, std::move(*warm_ptr));
                    if (has_ep) {
                        worker.set_endpoints(start_cfg, goal_cfg);
                    }
                    ParallelWorkerResult pwr;
                    pwr.result = worker.grow_subtree(seed, rid, limits, obs, n_obs,
                                                    shared_counter);
                    pwr.lect = std::move(worker.lect_mut());
                    return pwr;
                }
            ));
        } else {
            futures.push_back(pool.submit(
                [robot_ptr, worker_cfg, has_ep, start_cfg, goal_cfg,
                 seed, rid, limits, obs, n_obs, shared_counter]() -> ParallelWorkerResult
                {
                    ForestGrower worker(*robot_ptr, worker_cfg);
                    if (has_ep) {
                        worker.set_endpoints(start_cfg, goal_cfg);
                    }
                    ParallelWorkerResult pwr;
                    pwr.result = worker.grow_subtree(seed, rid, limits, obs, n_obs,
                                                    shared_counter);
                    pwr.lect = std::move(worker.lect_mut());
                    return pwr;
                }
            ));
        }
        future_subtree_idx.push_back(i);
    }

    // ── Collect results and merge ───────────────────────────────────────
    boxes_.clear();
    box_id_to_idx_.clear();
    next_box_id_ = 0;
    int total_promotions = 0;
    n_coarse_created_ = 0;
    n_fine_created_   = 0;
    int total_transplanted = 0;

    for (int fi = 0; fi < static_cast<int>(futures.size()); ++fi) {
        ParallelWorkerResult pwr = futures[fi].get();
        GrowerResult& wr = pwr.result;
        total_promotions += wr.n_promotions;
        n_coarse_created_ += wr.n_coarse_boxes;
        n_fine_created_   += wr.n_fine_boxes;

        // Build local-to-global ID map
        std::unordered_map<int, int> id_map;
        for (auto& box : wr.boxes) {
            int new_id = next_box_id_++;
            id_map[box.id] = new_id;
            box.id = new_id;
        }

        // Remap parent_box_id
        for (auto& box : wr.boxes) {
            if (box.parent_box_id >= 0) {
                auto mit = id_map.find(box.parent_box_id);
                box.parent_box_id = (mit != id_map.end()) ? mit->second : -1;
            }
            // tree_id will be handled by transplant (or invalidated below)
            box.tree_id = -1;
        }

        // ── Transplant worker LECT into coordinator ─────────────────
        if (lect_aligned && use_warm_start) {
            int si = future_subtree_idx[fi];
            int coord_node = subtrees_[si].lect_node_idx;
            int n_tp = lect_.transplant_subtree(pwr.lect, coord_node,
                                                snapshot_n_nodes, id_map);
            total_transplanted += n_tp;
        }

        for (auto& box : wr.boxes) {
            box_id_to_idx_[box.id] = static_cast<int>(boxes_.size());
            boxes_.push_back(std::move(box));
        }
    }

    // ── Update start/goal box IDs ───────────────────────────────────────
    if (has_endpoints_) {
        start_box_id_ = -1;
        goal_box_id_  = -1;

        auto contains_q = [](const BoxNode& box, const Eigen::VectorXd& q) {
            for (int d = 0; d < static_cast<int>(box.joint_intervals.size()); ++d) {
                if (q[d] < box.joint_intervals[d].lo - 1e-10 ||
                    q[d] > box.joint_intervals[d].hi + 1e-10)
                    return false;
            }
            return true;
        };

        for (const auto& box : boxes_) {
            if (start_box_id_ < 0 && box.root_id == 0 &&
                contains_q(box, start_config_))
                start_box_id_ = box.id;
            if (goal_box_id_ < 0 && box.root_id == 1 &&
                contains_q(box, goal_config_))
                goal_box_id_ = box.id;
        }
    }

    result.n_promotions = total_promotions;

    // ── Boundary bridging: create bridge boxes at subtree interfaces ────
    bridge_subtree_boundaries(obs, n_obs, result);

    std::printf("[grower] parallel done: %d boxes merged (%d bridge), %d promotions, "
                "%d nodes transplanted\n",
                static_cast<int>(boxes_.size()), result.n_bridge_boxes,
                total_promotions, total_transplanted);
}

// ═════════════════════════════════════════════════════════════════════════
//  sync_lect_occupation — mark merged boxes in coordinator's LECT
// ═════════════════════════════════════════════════════════════════════════
void ForestGrower::sync_lect_occupation() {
    int n_dims = robot_->n_joints();

    for (const auto& box : boxes_) {
        // Walk the coordinator LECT from root to the deepest existing leaf
        // containing the box's seed configuration.
        // split_dim at depth d = d % n_dims (identity cycling).
        int node = 0;
        while (!lect_.is_leaf(node)) {
            int dim = lect_.depth(node) % n_dims;
            double sv = lect_.split_val(node);
            if (box.seed_config[dim] <= sv)
                node = lect_.left(node);
            else
                node = lect_.right(node);
        }
        if (!lect_.is_occupied(node)) {
            lect_.mark_occupied(node, box.id);
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════
//  bridge_subtree_boundaries — connect adjacent subtree regions
//
//  Improvements over baseline:
//    1. Sync LECT occupation first → avoids creating boxes overlapping workers'
//    2. Proximity-based seeding → uses existing near-boundary boxes as seeds
//    3. Cross-root alternation → half bridges get root_i, half root_j
//       so that bridge boxes create cross-tree adjacency edges
// ═════════════════════════════════════════════════════════════════════════
void ForestGrower::bridge_subtree_boundaries(const Obstacle* obs, int n_obs,
                                             GrowerResult& result) {
    if (subtrees_.size() < 2) return;
    if (config_.bridge_samples <= 0) return;

    // Phase 0: Sync coordinator LECT occupation from merged worker boxes.
    // If LECT-aligned transplant was used, occupation is already correct.
    // Only sync if partitions were not LECT-aligned (legacy path).
    bool lect_aligned = (subtrees_.size() > 0 && subtrees_[0].lect_node_idx >= 0);
    if (!lect_aligned) {
        sync_lect_occupation();
    }

    int n_dims = robot_->n_joints();
    int n_st = static_cast<int>(subtrees_.size());
    double eps = config_.boundary_epsilon;
    int seeds_per_face = config_.bridge_samples;
    int bridge_count = 0;

    std::uniform_real_distribution<double> u01(0.0, 1.0);

    // For each pair of subtrees, check if they share a face
    for (int i = 0; i < n_st; ++i) {
        for (int j = i + 1; j < n_st; ++j) {
            const auto& li = subtrees_[i].limits;
            const auto& lj = subtrees_[j].limits;

            // Find shared face: a dimension where hi_i == lo_j or hi_j == lo_i
            for (int d = 0; d < n_dims; ++d) {
                bool face_ij = (std::abs(li[d].hi - lj[d].lo) < 1e-10);
                bool face_ji = (std::abs(lj[d].hi - li[d].lo) < 1e-10);
                if (!face_ij && !face_ji) continue;

                // Check overlap in all other dimensions
                bool overlap = true;
                for (int dd = 0; dd < n_dims; ++dd) {
                    if (dd == d) continue;
                    double lo_max = std::max(li[dd].lo, lj[dd].lo);
                    double hi_min = std::min(li[dd].hi, lj[dd].hi);
                    if (lo_max >= hi_min) { overlap = false; break; }
                }
                if (!overlap) continue;

                // Shared face found at boundary_val in dimension d.
                // j_sign = +1 means j is on the hi side, -1 means lo side.
                double boundary_val = face_ij ? li[d].hi : li[d].lo;
                double j_sign = face_ij ? +1.0 : -1.0;

                // ── Collect near-boundary boxes from each subtree ────────
                // A box is "near" if any of its d-faces is within 10*eps
                // of the boundary.
                std::vector<const BoxNode*> near_i, near_j;
                double proximity = eps * 10.0;

                for (const auto& box : boxes_) {
                    double d_lo = box.joint_intervals[d].lo;
                    double d_hi = box.joint_intervals[d].hi;
                    double dist = std::min(std::abs(d_lo - boundary_val),
                                           std::abs(d_hi - boundary_val));
                    if (dist > proximity) continue;

                    if (box.root_id == subtrees_[i].root_id)
                        near_i.push_back(&box);
                    if (box.root_id == subtrees_[j].root_id)
                        near_j.push_back(&box);
                }

                // ── Sample bridge seeds ──────────────────────────────────
                for (int s = 0; s < seeds_per_face; ++s) {
                    // Alternate direction: even → i-side seed extending into j
                    //                      odd  → j-side seed extending into i
                    bool to_j = (s % 2 == 0);
                    const auto& near_src = to_j ? near_i : near_j;

                    Eigen::VectorXd seed(n_dims);

                    if (!near_src.empty()) {
                        // Strategy 1: Use a near-boundary box's config as base
                        // and shift the boundary dimension across.
                        const BoxNode* src = near_src[s / 2 % near_src.size()];
                        seed = src->seed_config;
                        // Place seed just across the boundary into the other subtree
                        double offset = u01(rng_) * eps;
                        seed[d] = boundary_val + (to_j ? j_sign : -j_sign) * offset;
                    } else {
                        // Strategy 2: Pure random in the overlap region
                        for (int dd = 0; dd < n_dims; ++dd) {
                            if (dd == d) {
                                double offset = u01(rng_) * eps;
                                seed[dd] = boundary_val +
                                           (to_j ? j_sign : -j_sign) * offset;
                            } else {
                                double lo_max = std::max(li[dd].lo, lj[dd].lo);
                                double hi_min = std::min(li[dd].hi, lj[dd].hi);
                                seed[dd] = lo_max + u01(rng_) * (hi_min - lo_max);
                            }
                        }
                    }

                    seed = robot_->joint_limits().clamp(seed);

                    // Cross-root: seed going into j gets root_j, into i gets root_i
                    int bridge_root = to_j ? subtrees_[j].root_id
                                           : subtrees_[i].root_id;

                    int bid = try_create_box(seed, obs, n_obs, -1, d,
                                             face_ij ? 1 : 0, bridge_root);
                    if (bid >= 0) {
                        bridge_count++;
                    }
                }
            }
        }
    }

    result.n_bridge_boxes = bridge_count;
    if (bridge_count > 0) {
        std::printf("[grower] bridge: %d boxes at subtree boundaries\n",
                    bridge_count);
    }
}

// ═════════════════════════════════════════════════════════════════════════
//  Helpers
// ═════════════════════════════════════════════════════════════════════════

Eigen::VectorXd ForestGrower::sample_random() {
    const auto& limits = robot_->joint_limits().limits;
    int n = static_cast<int>(limits.size());
    Eigen::VectorXd q(n);
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    for (int d = 0; d < n; ++d) {
        double t = dist(rng_);
        q[d] = limits[d].lo + t * limits[d].width();
    }
    return q;
}

Eigen::VectorXd ForestGrower::sample_random_in_subtree(int root_id) {
    // Find subtree for this root_id
    for (const auto& st : subtrees_) {
        if (st.root_id == root_id && !st.limits.empty()) {
            int n = static_cast<int>(st.limits.size());
            Eigen::VectorXd q(n);
            std::uniform_real_distribution<double> dist(0.0, 1.0);
            for (int d = 0; d < n; ++d) {
                double t = dist(rng_);
                q[d] = st.limits[d].lo + t * st.limits[d].width();
            }
            return robot_->joint_limits().clamp(q);
        }
    }
    // Fallback to global random
    return sample_random();
}

Eigen::VectorXd ForestGrower::sample_random_in_region(
    const Eigen::VectorXd& center, double sigma) {
    int n = static_cast<int>(center.size());
    Eigen::VectorXd q(n);
    std::normal_distribution<double> dist(0.0, sigma);
    for (int d = 0; d < n; ++d)
        q[d] = center[d] + dist(rng_);
    return robot_->joint_limits().clamp(q);
}

} // namespace forest
} // namespace sbf

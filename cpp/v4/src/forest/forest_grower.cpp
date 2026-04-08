// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — ForestGrower implementation
//  基础迁移版：仅包含最基本的串行 Wavefront / RRT grower
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/forest/forest_grower.h"
#include "sbf/forest/thread_pool.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <numeric>
#include <queue>
#include <unordered_set>

namespace sbf {
namespace forest {

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

ForestGrower::ForestGrower(const Robot& robot, const GrowerConfig& config, LECT warm_lect)
    : robot_(&robot)
    , config_(config)
    , lect_(std::move(warm_lect))
    , graph_(robot.joint_limits(), config.adjacency_tol)
    , rng_(config.rng_seed)
{
    if (config.hull_skip_vol > 0.0)
        lect_.set_hull_skip_vol(config.hull_skip_vol);
}

void ForestGrower::set_deadline(std::optional<TimePoint> deadline) {
    deadline_ = std::move(deadline);
    lect_.set_deadline(deadline_);
}

bool ForestGrower::deadline_reached() const {
    return deadline_.has_value() && Clock::now() >= *deadline_;
}

void ForestGrower::set_endpoints(const Eigen::VectorXd& start,
                                 const Eigen::VectorXd& goal) {
    has_endpoints_ = true;
    start_config_ = start;
    goal_config_ = goal;
}

const Eigen::VectorXd* ForestGrower::get_bias_target(int root_id) {
    if (!has_endpoints_) return nullptr;
    if (root_id == 0) return &goal_config_;
    if (root_id == 1) return &start_config_;

    std::uniform_real_distribution<double> u01(0.0, 1.0);
    return (u01(rng_) < 0.5) ? &start_config_ : &goal_config_;
}

Eigen::VectorXd ForestGrower::sample_near_existing_boundary(int root_id) {
    if (boxes_.empty()) return {};

    std::vector<int> candidates;
    if (root_id >= 0) {
        for (int i = 0; i < static_cast<int>(boxes_.size()); ++i) {
            if (boxes_[i].root_id == root_id)
                candidates.push_back(i);
        }
    }
    if (candidates.empty()) {
        candidates.resize(boxes_.size());
        std::iota(candidates.begin(), candidates.end(), 0);
    }

    std::uniform_int_distribution<int> box_dist(
        0, static_cast<int>(candidates.size()) - 1);
    const BoxNode& box = boxes_[candidates[box_dist(rng_)]];

    const auto& limits = robot_->joint_limits().limits;
    const int n_dims = box.n_dims();
    const double eps = config_.boundary_epsilon;

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

ForestGrower::SnapResult ForestGrower::rrt_snap_to_face(
        const BoxNode& nearest,
        const Eigen::VectorXd& direction,
        double step) {
    const int n_dims = nearest.n_dims();
    const auto& limits = robot_->joint_limits().limits;
    const double eps = config_.boundary_epsilon;
    const Eigen::VectorXd nearest_center = nearest.center();

    int best_dim = -1;
    int best_side = -1;
    double best_score = -1e30;

    for (int d = 0; d < n_dims; ++d) {
        for (int side = 0; side < 2; ++side) {
            const double normal_sign = (side == 0) ? -1.0 : 1.0;
            const double score = direction[d] * normal_sign;
            if (score <= 0.0) continue;
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
        Eigen::VectorXd seed = nearest_center + direction * step;
        return {robot_->joint_limits().clamp(seed), -1, -1};
    }

    std::uniform_real_distribution<double> u01(0.0, 1.0);
    Eigen::VectorXd seed(n_dims);
    for (int d = 0; d < n_dims; ++d) {
        if (d == best_dim) {
            seed[d] = (best_side == 0)
                ? nearest.joint_intervals[d].lo - eps
                : nearest.joint_intervals[d].hi + eps;
        } else {
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

bool ForestGrower::is_grid_envelope() const {
    auto et = config_.pipeline.envelope.type;
    return et == envelope::EnvelopeType::LinkIAABB_Grid
        || et == envelope::EnvelopeType::Hull16_Grid;
}

GrowerResult ForestGrower::grow(const Obstacle* obs, int n_obs) {
    auto t0 = Clock::now();
    GrowerResult result;
    auto make_deadline = [&](TimePoint stage_start) -> std::optional<TimePoint> {
        if (config_.timeout <= 0.0)
            return std::nullopt;
        return stage_start + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(config_.timeout));
    };

    boxes_.clear();
    box_id_to_idx_.clear();
    graph_.clear();
    shared_box_count_.reset();
    next_box_id_ = 0;
    start_box_id_ = -1;
    goal_box_id_ = -1;
    n_coarse_created_ = 0;
    n_fine_created_ = 0;
    subtrees_.clear();
    lect_.clear_all_occupation();

    if (is_grid_envelope())
        lect_.set_scene(obs, n_obs);
    else
        lect_.clear_scene();

    // ── Load offline LECT cache (if configured) ─────────────────────────
    lect_cache_loaded_ = false;
    if (!config_.lect_cache_dir.empty()) {
        namespace fs = std::filesystem;
        std::string hcache = config_.lect_cache_dir + "/lect.hcache";
        if (fs::exists(hcache)) {
            auto t_cache0 = std::chrono::high_resolution_clock::now();
            lect_ = LECT::load(config_.lect_cache_dir, *robot_);
            // Re-apply hull_skip_vol (not persisted in cache)
            if (config_.hull_skip_vol > 0.0)
                lect_.set_hull_skip_vol(config_.hull_skip_vol);
            // Re-apply scene grid (load() doesn't preserve it)
            if (is_grid_envelope())
                lect_.set_scene(obs, n_obs);
            auto t_cache1 = std::chrono::high_resolution_clock::now();
            double cache_ms = std::chrono::duration<double, std::milli>(
                t_cache1 - t_cache0).count();
            std::printf("[grower] loaded LECT cache: %d nodes, %.1f ms\n",
                        lect_.n_nodes(), cache_ms);
            lect_cache_loaded_ = true;
        } else {
            std::printf("[grower] LECT cache not found: %s\n", hcache.c_str());
        }
    }

    auto t_root0 = Clock::now();
    set_deadline(make_deadline(t_root0));

    // Dispatch partition + root selection based on partition_mode
    bool is_pow2 = (config_.n_roots > 0) &&
                   ((config_.n_roots & (config_.n_roots - 1)) == 0);
    if (config_.partition_mode == PartitionMode::LectAligned && is_pow2) {
        // Partition-first: build cells aligned with LECT, then best-of-N roots
        partition_lect_aligned();
        if (!deadline_reached())
            select_roots_in_partitions(obs, n_obs);
    } else if (config_.partition_mode == PartitionMode::Uniform) {
        partition_uniform();
        if (!deadline_reached())
            select_roots_in_partitions(obs, n_obs);
    } else {
        // KDSplit (or LectAligned with non-power-of-2 n_roots): roots first
        // Pre-expand LECT so FFB can find valid leaves (same depth as warm_start)
        if (!lect_cache_loaded_) {
            int wd = config_.warm_start_depth;
            if (wd == 0) wd = 3;
            if (wd > 0) {
                int kd_new = lect_.pre_expand(wd);
                std::printf("[grower] KDSplit: pre-expanded LECT to depth %d (%d new nodes)\n",
                            wd, kd_new);
            }
        }
        if (!deadline_reached())
            select_roots(obs, n_obs);
        if (!deadline_reached())
            partition_subtrees();
    }
    auto t_root1 = Clock::now();
    bool root_timed_out = deadline_reached();
    set_deadline(std::nullopt);

    result.n_roots = static_cast<int>(boxes_.size());
    result.phase_times["root_select_ms"] =
        std::chrono::duration<double, std::milli>(t_root1 - t_root0).count();
    if (root_timed_out) {
        std::printf("[grower] root_select timeout at %.3f s\n",
                    std::chrono::duration<double>(t_root1 - t_root0).count());
    }

    auto t_expand0 = Clock::now();
    set_deadline(make_deadline(t_expand0));
    if (config_.n_threads > 1 && static_cast<int>(subtrees_.size()) > 1) {
        grow_parallel(obs, n_obs, result);
    } else {
        for (const auto& b : boxes_)
            graph_.add_box(b);

        if (config_.mode == GrowerConfig::Mode::Wavefront) {
            grow_wavefront(obs, n_obs);
        } else {
            grow_rrt(obs, n_obs);
        }

        if (!deadline_reached())
            result.n_promotions = promote_all(obs, n_obs);
        // Bridge subtree boundaries (single-threaded path;
        // parallel path calls this inside grow_parallel)
        if (!deadline_reached())
            bridge_subtree_boundaries(obs, n_obs, result);
    }
    auto t_expand1 = Clock::now();
    bool expand_timed_out = deadline_reached();
    set_deadline(std::nullopt);

    result.phase_times["expand_ms"] =
        std::chrono::duration<double, std::milli>(t_expand1 - t_expand0).count();
    if (expand_timed_out) {
        std::printf("[grower] expand timeout at %.3f s\n",
                    std::chrono::duration<double>(t_expand1 - t_expand0).count());
    }

    auto t_adj0 = Clock::now();
    graph_.rebuild(boxes_);
    auto t_adj1 = Clock::now();

    result.phase_times["adj_rebuild_ms"] =
        std::chrono::duration<double, std::milli>(t_adj1 - t_adj0).count();
    result.boxes = boxes_;
    result.n_boxes_total = static_cast<int>(boxes_.size());
    result.n_components = graph_.n_components();
    result.n_coarse_boxes = n_coarse_created_;
    result.n_fine_boxes = n_fine_created_;

    if (config_.coarsen_enabled) {
        result.n_coarsen_merges = coarsen_greedy(obs, n_obs);
        graph_.rebuild(boxes_);
        result.boxes = boxes_;
        result.n_boxes_total = static_cast<int>(boxes_.size());
        result.n_components = graph_.n_components();
    }

    for (const auto& b : boxes_)
        result.total_volume += b.volume;

    if (has_endpoints_) {
        // ── Ensure start/goal boxes exist ───────────────────────────────
        // With partition-based root selection, no box may cover start/goal.
        // Scan first, then create if needed.
        auto contains_q = [](const BoxNode& box, const Eigen::VectorXd& q) {
            for (int d = 0; d < static_cast<int>(box.joint_intervals.size()); ++d) {
                if (q[d] < box.joint_intervals[d].lo - 1e-10 ||
                    q[d] > box.joint_intervals[d].hi + 1e-10)
                    return false;
            }
            return true;
        };

        if (start_box_id_ < 0) {
            for (const auto& box : boxes_) {
                if (contains_q(box, start_config_)) { start_box_id_ = box.id; break; }
            }
        }
        if (goal_box_id_ < 0) {
            for (const auto& box : boxes_) {
                if (contains_q(box, goal_config_)) { goal_box_id_ = box.id; break; }
            }
        }

        // Endpoint retry with deeper FFB: min_edge=1e-4, max_depth=50
        // Use incremental add_box instead of full rebuild.
        // Occupied-node bypass: when FFB fails with code=1 (blocked by
        // an occupied ancestor that doesn't contain the seed), temporarily
        // unmark the occupied node and retry, up to MAX_BYPASS times.
        auto try_recover_endpoint = [&](const Eigen::VectorXd& config,
                                        int& box_id_out, const char* label) {
            if (box_id_out >= 0) return;
            int bid = try_create_box(config, obs, n_obs, -1, -1, -1, -1, 1e-4, 50);
            if (bid >= 0) { box_id_out = bid; graph_.add_box(boxes_.back()); return; }

            // Iterative occupied-node bypass: up to 10 retries
            constexpr int MAX_BYPASS = 10;
            std::vector<std::pair<int,int>> bypassed;  // (node_idx, box_id)
            for (int attempt = 0; attempt < MAX_BYPASS; ++attempt) {
                FFBResult diag = lect_.find_free_box(config, obs, n_obs, 1e-4, 50);
                if (diag.fail_code != 1 || diag.path.empty()) {
                    std::printf("[grower] WARNING: post-grow %s FFB failed "
                                "(code=%d, depth=%d, nodes=%d)\n",
                                label, diag.fail_code,
                                static_cast<int>(diag.path.size()), diag.n_new_nodes);
                    break;
                }
                int occ_node = diag.path.back();
                int fid = lect_.forest_id(occ_node);
                auto it = box_id_to_idx_.find(fid);
                // If existing box contains the config, use it directly
                if (it != box_id_to_idx_.end() &&
                    contains_q(boxes_[it->second], config)) {
                    box_id_out = fid;
                    std::printf("[grower] %s: recovered from occupied node "
                                "(box_id=%d, depth=%d)\n",
                                label, fid, static_cast<int>(diag.path.size()));
                    break;
                }
                // Save mapping before unmark (unmark clears forest_id)
                bypassed.push_back({occ_node, fid});
                lect_.unmark_occupied(occ_node);

                bid = try_create_box(config, obs, n_obs, -1, -1, -1, -1, 1e-4, 50);
                if (bid >= 0) {
                    box_id_out = bid;
                    graph_.add_box(boxes_.back());
                    std::printf("[grower] %s: bypass succeeded after %d "
                                "unmarks (new box_id=%d)\n",
                                label, attempt + 1, bid);
                    break;
                }
                // FFB still failed — loop to see what blocked next
            }
            // Re-mark all temporarily bypassed nodes
            for (auto [node, orig_bid] : bypassed) {
                if (!lect_.is_occupied(node) && orig_bid >= 0)
                    lect_.mark_occupied(node, orig_bid);
            }
        };
        try_recover_endpoint(start_config_, start_box_id_, "start");
        try_recover_endpoint(goal_config_,  goal_box_id_,  "goal");
        // Stats update (incremental — no full rebuild needed)
        result.n_boxes_total = static_cast<int>(boxes_.size());
        result.n_components = graph_.n_components();
        result.boxes = boxes_;

        // ── Targeted S-G connector: interpolation-based seeding ─────
        // Run even when one endpoint box is missing — interpolation
        // boxes may cover the missing endpoint in narrow passages.
        if (start_box_id_ >= 0 || goal_box_id_ >= 0) {
            if (start_box_id_ >= 0 && goal_box_id_ >= 0)
                result.start_goal_connected = graph_.connected(start_box_id_, goal_box_id_);

            if (!result.start_goal_connected) {
                constexpr int N_INTERP = 100;
                int connect_count = 0;
                for (int k = 1; k <= N_INTERP; ++k) {
                    double t = static_cast<double>(k) / (N_INTERP + 1);
                    Eigen::VectorXd seed = (1.0 - t) * start_config_ + t * goal_config_;
                    seed = robot_->joint_limits().clamp(seed);
                    int bid = try_create_box(seed, obs, n_obs, -1, -1, -1, -1, 1e-4, 50);
                    if (bid >= 0) {
                        graph_.add_box(boxes_.back());
                        connect_count++;
                        // Check if this box covers a previously-missing endpoint
                        if (start_box_id_ < 0 && contains_q(boxes_.back(), start_config_))
                            start_box_id_ = bid;
                        if (goal_box_id_ < 0 && contains_q(boxes_.back(), goal_config_))
                            goal_box_id_ = bid;
                        // Early termination: both endpoints found and connected
                        if (start_box_id_ >= 0 && goal_box_id_ >= 0 &&
                            graph_.connected(start_box_id_, goal_box_id_))
                            break;
                    }
                }
                if (connect_count > 0) {
                    if (start_box_id_ >= 0 && goal_box_id_ >= 0)
                        result.start_goal_connected =
                            graph_.connected(start_box_id_, goal_box_id_);
                    result.n_boxes_total = static_cast<int>(boxes_.size());
                    result.n_components = graph_.n_components();
                    result.boxes = boxes_;
                    std::printf("[grower] S-G connector: %d interp boxes, connected=%d\n",
                                connect_count,
                                static_cast<int>(result.start_goal_connected));
                }
            }
        }
    }

    auto t1 = Clock::now();
    result.build_time_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
}

void ForestGrower::select_roots(const Obstacle* obs, int n_obs) {
    if (has_endpoints_) {
        select_roots_with_endpoints(obs, n_obs);
    } else {
        select_roots_no_endpoints(obs, n_obs);
    }
}

void ForestGrower::select_roots_no_endpoints(const Obstacle* obs, int n_obs) {
    const int n_roots = config_.n_roots;
    std::vector<Eigen::VectorXd> root_centers;
    const double rme = (config_.root_min_edge > 0.0) ? config_.root_min_edge : -1.0;

    for (int attempt = 0; attempt < 100; ++attempt) {
        if (deadline_reached()) return;
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

    constexpr int K_CANDIDATES = 50;
    for (int r = 1; r < n_roots; ++r) {
        if (deadline_reached()) return;
        std::vector<Eigen::VectorXd> candidates;
        candidates.reserve(K_CANDIDATES);
        for (int k = 0; k < K_CANDIDATES; ++k)
            candidates.push_back(sample_random());

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
        if (bid >= 0)
            root_centers.push_back(boxes_.back().center());
    }
}

void ForestGrower::select_roots_with_endpoints(const Obstacle* obs, int n_obs) {
    const int n_roots = config_.n_roots;
    const double rme = (config_.root_min_edge > 0.0) ? config_.root_min_edge : -1.0;

    {
        if (deadline_reached()) return;
        int bid = try_create_box(start_config_, obs, n_obs, -1, -1, -1, 0, rme);
        if (bid >= 0) start_box_id_ = bid;
        else std::printf("[grower] WARNING: start seed FFB failed\n");
    }
    {
        if (deadline_reached()) return;
        int bid = try_create_box(goal_config_, obs, n_obs, -1, -1, -1, 1, rme);
        if (bid >= 0) goal_box_id_ = bid;
        else std::printf("[grower] WARNING: goal seed FFB failed\n");
    }

    if (n_roots > 2 && boxes_.size() >= 2) {
        const Eigen::VectorXd sg = goal_config_ - start_config_;
        const double sg_norm = sg.norm();
        const int n_dims = robot_->n_joints();

        constexpr int K_CANDIDATES = 30;
        std::vector<Eigen::VectorXd> root_centers;
        for (const auto& b : boxes_)
            root_centers.push_back(b.center());

        for (int r = 2; r < n_roots; ++r) {
            if (deadline_reached()) return;
            double best_score = -1.0;
            Eigen::VectorXd best_cand;

            std::uniform_real_distribution<double> t_dist(0.1, 0.9);
            std::normal_distribution<double> perturb(0.0, 0.3 * sg_norm);

            for (int k = 0; k < K_CANDIDATES; ++k) {
                double t = t_dist(rng_);
                Eigen::VectorXd cand = start_config_ + t * sg;
                for (int d = 0; d < n_dims; ++d)
                    cand[d] += perturb(rng_);
                cand = robot_->joint_limits().clamp(cand);

                double min_dist = std::numeric_limits<double>::max();
                for (const auto& rc : root_centers)
                    min_dist = std::min(min_dist, (cand - rc).norm());

                if (min_dist > best_score) {
                    best_score = min_dist;
                    best_cand = cand;
                }
            }

            int bid = try_create_box(best_cand, obs, n_obs, -1, -1, -1, r, rme);
            if (bid >= 0)
                root_centers.push_back(boxes_.back().center());
        }
    }
}

void ForestGrower::partition_subtrees() {
    subtrees_.clear();
    if (boxes_.empty()) return;

    const int n_dims = robot_->n_joints();
    const auto& jl = robot_->joint_limits().limits;

    struct RootInfo {
        int root_id;
        Eigen::VectorXd seed;
    };
    std::vector<RootInfo> roots;
    for (const auto& b : boxes_) {
        if (b.parent_box_id == -1)
            roots.push_back({b.root_id, b.seed_config});
    }
    if (roots.empty()) return;

    struct Cell {
        std::vector<Interval> limits;
        std::vector<int> root_indices;
    };

    Cell initial;
    initial.limits.resize(n_dims);
    for (int d = 0; d < n_dims; ++d)
        initial.limits[d] = jl[d];
    for (int i = 0; i < static_cast<int>(roots.size()); ++i)
        initial.root_indices.push_back(i);

    std::vector<Cell> work = {initial};
    std::vector<Cell> done;
    const int max_partition_depth = 3 * n_dims;
    int split_count = 0;

    while (!work.empty() && split_count < max_partition_depth) {
        std::vector<Cell> next;
        for (auto& cell : work) {
            if (cell.root_indices.size() <= 1) {
                done.push_back(std::move(cell));
                continue;
            }

            const int dim = split_count % n_dims;
            const double mid = cell.limits[dim].center();

            Cell left_cell, right_cell;
            left_cell.limits = cell.limits;
            left_cell.limits[dim].hi = mid;
            right_cell.limits = cell.limits;
            right_cell.limits[dim].lo = mid;

            for (int ri : cell.root_indices) {
                if (roots[ri].seed[dim] <= mid) left_cell.root_indices.push_back(ri);
                else right_cell.root_indices.push_back(ri);
            }

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

    for (auto& cell : work)
        done.push_back(std::move(cell));

    subtrees_.resize(static_cast<int>(roots.size()));
    for (const auto& cell : done) {
        for (int ri : cell.root_indices) {
            subtrees_[ri].root_id = roots[ri].root_id;
            subtrees_[ri].limits = cell.limits;
        }
    }
}

void ForestGrower::grow_wavefront(const Obstacle* obs, int n_obs) {
    int miss_count = 0;

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

    struct WaveEntry {
        int box_id;
        int from_face_dim;
        int from_face_side;
        double priority;
        bool operator<(const WaveEntry& o) const { return priority < o.priority; }
    };

    std::priority_queue<WaveEntry> queue;
    for (const auto& box : boxes_)
        queue.push({box.id, -1, -1, box.volume});

    auto budget_ok = [&]() -> bool {
        if (shared_box_count_) {
            return shared_box_count_->load(std::memory_order_relaxed) < config_.max_boxes;
        }
        return static_cast<int>(boxes_.size()) < config_.max_boxes;
    };

    while (budget_ok() && miss_count < config_.max_consecutive_miss &&
           !deadline_reached()) {

        if (!is_last_stage() && current_box_count() >= stages[current_stage].box_limit) {
            advance_stage();
            while (!queue.empty()) queue.pop();
            for (const auto& box : boxes_)
                queue.push({box.id, -1, -1, box.volume});
        }

        if (!queue.empty()) {
            WaveEntry entry = queue.top();
            queue.pop();

            auto it = box_id_to_idx_.find(entry.box_id);
            if (it == box_id_to_idx_.end()) continue;

            const int expand_from_id = boxes_[it->second].id;
            const int expand_root_id = boxes_[it->second].root_id;
            const Eigen::VectorXd* bias_ptr = get_bias_target(expand_root_id);
            auto seeds = sample_boundary(boxes_[it->second], bias_ptr);

            for (const auto& bs : seeds) {
                if (!budget_ok() || deadline_reached()) break;

                int bid = try_create_box(
                    bs.config, obs, n_obs,
                    expand_from_id, bs.dim, bs.side,
                    expand_root_id, effective_min_edge());

                if (bid >= 0) {
                    // Skip incremental add_box during expansion;
                    // graph_.rebuild() after expansion covers all boxes.
                    queue.push({bid, bs.dim, 1 - bs.side, boxes_.back().volume});
                    miss_count = 0;
                    (is_last_stage() ? n_fine_created_ : n_coarse_created_)++;
                } else {
                    if (deadline_reached()) break;
                    miss_count++;
                }
            }
        } else {
            if (deadline_reached()) break;
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
                queue.push({bid, -1, -1, boxes_.back().volume});
                miss_count = 0;
                (is_last_stage() ? n_fine_created_ : n_coarse_created_)++;
            } else {
                if (deadline_reached()) break;
                miss_count++;
            }
        }
    }
}

std::vector<ForestGrower::BoundarySeed>
ForestGrower::sample_boundary(const BoxNode& box,
                              const Eigen::VectorXd* bias_target) {
    std::vector<BoundarySeed> seeds;
    const int n_dims = box.n_dims();
    const auto& limits = robot_->joint_limits().limits;
    const double eps = config_.boundary_epsilon;

    struct Face { int dim; int side; double priority; };
    std::vector<Face> faces;
    for (int d = 0; d < n_dims; ++d) {
        double lo_seed = box.joint_intervals[d].lo - eps;
        if (lo_seed >= limits[d].lo)
            faces.push_back({d, 0, 0.0});

        double hi_seed = box.joint_intervals[d].hi + eps;
        if (hi_seed <= limits[d].hi)
            faces.push_back({d, 1, 0.0});
    }

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

    if (bias_target) {
        Eigen::VectorXd box_center = box.center();
        Eigen::VectorXd to_target = *bias_target - box_center;
        for (auto& f : faces) {
            f.priority = (f.side == 1) ? to_target[f.dim] : -to_target[f.dim];
        }
        std::sort(faces.begin(), faces.end(),
                  [](const Face& a, const Face& b) { return a.priority > b.priority; });
    } else {
        std::shuffle(faces.begin(), faces.end(), rng_);
    }

    std::uniform_real_distribution<double> u01(0.0, 1.0);
    int n_samples = std::min(config_.n_boundary_samples,
                             static_cast<int>(faces.size()));

    for (int s = 0; s < n_samples; ++s) {
        int face_idx;
        if (bias_target && u01(rng_) < config_.goal_face_bias && !faces.empty()) {
            face_idx = 0;
        } else {
            face_idx = s % static_cast<int>(faces.size());
        }
        const Face& face = faces[face_idx];

        Eigen::VectorXd seed(n_dims);
        for (int d = 0; d < n_dims; ++d) {
            if (d == face.dim) {
                seed[d] = (face.side == 0)
                    ? box.joint_intervals[d].lo - eps
                    : box.joint_intervals[d].hi + eps;
            } else {
                double lo = box.joint_intervals[d].lo;
                double hi = box.joint_intervals[d].hi;
                seed[d] = lo + u01(rng_) * (hi - lo);
            }
        }
        seed = robot_->joint_limits().clamp(seed);
        seeds.push_back({face.dim, face.side, seed});
    }

    return seeds;
}

void ForestGrower::grow_rrt(const Obstacle* obs, int n_obs) {
    int miss_count = 0;
    const int n_dims = robot_->n_joints();
    const auto& limits = robot_->joint_limits().limits;

    double max_width = 0.0;
    for (int d = 0; d < n_dims; ++d)
        max_width = std::max(max_width, limits[d].width());
    const double step = config_.rrt_step_ratio * max_width;

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
    auto budget_ok = [&]() -> bool {
        if (shared_box_count_) {
            return shared_box_count_->load(std::memory_order_relaxed) < config_.max_boxes;
        }
        return static_cast<int>(boxes_.size()) < config_.max_boxes;
    };

    std::uniform_real_distribution<double> u01(0.0, 1.0);

    while (budget_ok() && miss_count < config_.max_consecutive_miss &&
           !deadline_reached()) {

        if (in_coarse_phase && current_box_count() >= coarse_limit) {
            in_coarse_phase = false;
            miss_count = 0;
        }

        Eigen::VectorXd q_rand;
        if (has_endpoints_ && u01(rng_) < config_.rrt_goal_bias) {
            int chosen_root = -1;
            if (!boxes_.empty()) {
                std::uniform_int_distribution<int> box_dist(
                    0, static_cast<int>(boxes_.size()) - 1);
                chosen_root = boxes_[box_dist(rng_)].root_id;
            }
            const Eigen::VectorXd* bias = get_bias_target(chosen_root);
            q_rand = bias ? *bias : sample_random();
        } else if (!subtrees_.empty()) {
            std::uniform_int_distribution<int> root_dist(
                0, static_cast<int>(subtrees_.size()) - 1);
            int ri = root_dist(rng_);
            q_rand = sample_random_in_subtree(subtrees_[ri].root_id);
        } else {
            q_rand = sample_random();
        }

        int nearest_id = graph_.find_nearest_box(q_rand);
        if (nearest_id < 0) {
            miss_count++;
            continue;
        }

        auto it = box_id_to_idx_.find(nearest_id);
        if (it == box_id_to_idx_.end()) {
            miss_count++;
            continue;
        }

        const int extend_from_id = boxes_[it->second].id;
        const int extend_root_id = boxes_[it->second].root_id;
        const Eigen::VectorXd nearest_center = boxes_[it->second].center();

        Eigen::VectorXd direction = q_rand - nearest_center;
        double dir_norm = direction.norm();
        if (dir_norm > 1e-12) {
            direction /= dir_norm;
        } else {
            direction = Eigen::VectorXd::Random(n_dims);
            dir_norm = direction.norm();
            if (dir_norm < 1e-12) {
                miss_count++;
                continue;
            }
            direction /= dir_norm;
        }

        auto snap = rrt_snap_to_face(boxes_[it->second], direction, step);
        int bid = try_create_box(
            snap.seed, obs, n_obs,
            extend_from_id, snap.face_dim, snap.face_side,
            extend_root_id, effective_min_edge());

        if (bid >= 0) {
            graph_.add_box(boxes_.back());
            miss_count = 0;
            (in_coarse_phase ? n_coarse_created_ : n_fine_created_)++;
        } else {
            if (deadline_reached()) break;
            miss_count++;
        }
    }
}

int ForestGrower::try_create_box(const Eigen::VectorXd& seed,
                                 const Obstacle* obs, int n_obs,
                                 int parent_box_id,
                                 int face_dim, int face_side,
                                 int root_id,
                                 double min_edge_override,
                                 int max_depth_override) {
    if (deadline_reached()) return -1;

    double me = (min_edge_override > 0.0) ? min_edge_override : config_.min_edge;
    int md = (max_depth_override > 0) ? max_depth_override : config_.max_depth;
    FFBResult ffb = lect_.find_free_box(seed, obs, n_obs, me, md);
    if (!ffb.success()) return -1;
    if (lect_.is_occupied(ffb.node_idx)) return -1;

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

    if (shared_box_count_)
        shared_box_count_->fetch_add(1, std::memory_order_relaxed);

    return box_id;
}

int ForestGrower::promote_all(const Obstacle* obs, int n_obs) {
    int total_promoted = 0;
    bool changed = true;

    while (changed && !deadline_reached()) {
        changed = false;
        int n_nodes = lect_.n_nodes();

        for (int i = 0; i < n_nodes; ++i) {
            if (deadline_reached()) return total_promoted;
            if (lect_.is_leaf(i)) continue;
            if (try_promote_node(i, obs, n_obs)) {
                total_promoted++;
                changed = true;
            }
        }
    }

    return total_promoted;
}

bool ForestGrower::try_promote_node(int node_idx,
                                    const Obstacle* obs, int n_obs) {
    int li = lect_.left(node_idx);
    int ri = lect_.right(node_idx);
    if (li < 0 || ri < 0) return false;

    if (!lect_.is_leaf(li) || !lect_.is_leaf(ri)) return false;
    if (!lect_.is_occupied(li) || !lect_.is_occupied(ri)) return false;
    if (lect_.is_occupied(node_idx)) return false;

    bool ok;
    if (is_grid_envelope()) {
        if (!lect_.merge_children_hulls(node_idx)) return false;
        ok = !lect_.collides_scene(node_idx, obs, n_obs);
    } else {
        if (!lect_.has_iaabb(node_idx)) return false;
        ok = !lect_.collides_scene(node_idx, obs, n_obs);
    }
    if (!ok) return false;

    int li_box_id = lect_.forest_id(li);
    int ri_box_id = lect_.forest_id(ri);

    lect_.unmark_occupied(li);
    lect_.unmark_occupied(ri);

    int promoted_root_id = -1;
    {
        auto it_li = box_id_to_idx_.find(li_box_id);
        if (it_li != box_id_to_idx_.end() && boxes_[it_li->second].root_id >= 0)
            promoted_root_id = boxes_[it_li->second].root_id;
        auto it_ri = box_id_to_idx_.find(ri_box_id);
        if (it_ri != box_id_to_idx_.end() && boxes_[it_ri->second].root_id >= 0)
            promoted_root_id = boxes_[it_ri->second].root_id;
    }

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
            box_id_to_idx_[boxes_[last].id] = idx;
            boxes_[idx] = std::move(boxes_[last]);
        }
        boxes_.pop_back();
    }

    int new_id = next_box_id_++;
    std::vector<Interval> parent_ivs = lect_.node_intervals(node_idx);
    Eigen::VectorXd parent_center(static_cast<int>(parent_ivs.size()));
    for (int d = 0; d < static_cast<int>(parent_ivs.size()); ++d)
        parent_center[d] = parent_ivs[d].center();

    BoxNode parent_box(new_id, parent_ivs, parent_center);
    parent_box.tree_id = node_idx;
    parent_box.root_id = promoted_root_id;

    lect_.mark_occupied(node_idx, new_id);

    if (li_box_id == start_box_id_ || ri_box_id == start_box_id_)
        start_box_id_ = new_id;
    if (li_box_id == goal_box_id_ || ri_box_id == goal_box_id_)
        goal_box_id_ = new_id;

    box_id_to_idx_[new_id] = static_cast<int>(boxes_.size());
    boxes_.push_back(std::move(parent_box));
    return true;
}

void ForestGrower::remove_box_by_id(int box_id) {
    auto it = box_id_to_idx_.find(box_id);
    if (it == box_id_to_idx_.end()) return;

    int idx = it->second;
    int last = static_cast<int>(boxes_.size()) - 1;

    if (boxes_[idx].tree_id >= 0)
        lect_.unmark_occupied(boxes_[idx].tree_id);

    box_id_to_idx_.erase(box_id);
    if (idx < last) {
        box_id_to_idx_[boxes_[last].id] = idx;
        boxes_[idx] = std::move(boxes_[last]);
    }
    boxes_.pop_back();
}

int ForestGrower::coarsen_greedy(const Obstacle* obs, int n_obs) {
    const int target = config_.coarsen_target_boxes;
    const int max_rounds = config_.max_coarsen_rounds;
    const double score_thr = config_.coarsen_score_threshold;
    const int n_dims = static_cast<int>(robot_->n_joints());

    int total_merges = 0;
    if (target > 0 && n_boxes() <= target) return 0;

    for (int round = 0; round < max_rounds; ++round) {
        if (target > 0 && n_boxes() <= target) break;

        graph_.rebuild(boxes_);

        struct Candidate {
            double score;
            int id_a, id_b;
            std::vector<Interval> hull_ivs;
        };
        std::vector<Candidate> candidates;
        candidates.reserve(boxes_.size());

        for (const auto& box_a : boxes_) {
            const auto& nbrs = graph_.neighbors(box_a.id);
            for (int id_b : nbrs) {
                if (id_b <= box_a.id) continue;

                auto it_b = box_id_to_idx_.find(id_b);
                if (it_b == box_id_to_idx_.end()) continue;
                const BoxNode& box_b = boxes_[it_b->second];

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

        std::sort(candidates.begin(), candidates.end(),
                  [](const Candidate& a, const Candidate& b) {
                      return a.score < b.score;
                  });

        std::unordered_set<int> merged_this_round;
        int merges_this_round = 0;

        for (auto& cand : candidates) {
            if (target > 0 && n_boxes() - merges_this_round <= target)
                break;

            if (merged_this_round.count(cand.id_a) || merged_this_round.count(cand.id_b))
                continue;

            auto it_a = box_id_to_idx_.find(cand.id_a);
            auto it_b = box_id_to_idx_.find(cand.id_b);
            if (it_a == box_id_to_idx_.end() || it_b == box_id_to_idx_.end())
                continue;

            bool hull_collides = lect_.intervals_collide_scene(cand.hull_ivs, obs, n_obs);
            if (hull_collides) continue;

            const BoxNode& ba = boxes_[it_a->second];
            Eigen::VectorXd seed = ba.seed_config;
            int root_id = ba.root_id;

            remove_box_by_id(cand.id_a);
            remove_box_by_id(cand.id_b);

            int new_id = next_box_id_++;
            BoxNode merged(new_id, cand.hull_ivs, seed);
            merged.root_id = root_id;
            merged.tree_id = -1;

            box_id_to_idx_[new_id] = static_cast<int>(boxes_.size());
            boxes_.push_back(std::move(merged));

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
    }

    graph_.rebuild(boxes_);
    return total_merges;
}

GrowerResult ForestGrower::grow_subtree(const Eigen::VectorXd& root_seed,
                                        int root_id,
                                        const std::vector<Interval>& subtree_limits,
                                        const Obstacle* obs, int n_obs,
                                        std::shared_ptr<std::atomic<int>> shared_box_count) {
    GrowerResult result;

    boxes_.clear();
    box_id_to_idx_.clear();
    graph_.clear();
    lect_.clear_all_occupation();
    next_box_id_ = 0;
    start_box_id_ = -1;
    goal_box_id_ = -1;
    n_coarse_created_ = 0;
    n_fine_created_ = 0;
    shared_box_count_ = std::move(shared_box_count);
    subtrees_.clear();
    lect_.set_deadline(deadline_);

    if (!subtree_limits.empty()) {
        SubtreeInfo st;
        st.root_id = root_id;
        st.limits = subtree_limits;
        subtrees_.push_back(std::move(st));
    }

    int bid = try_create_box(root_seed, obs, n_obs, -1, -1, -1,
                             root_id, config_.root_min_edge > 0.0 ? config_.root_min_edge : -1.0);
    if (bid < 0)
        return result;

    graph_.add_box(boxes_.back());
    if (config_.mode == GrowerConfig::Mode::Wavefront) {
        grow_wavefront(obs, n_obs);
    } else {
        grow_rrt(obs, n_obs);
    }

    if (!deadline_reached())
        result.n_promotions = promote_all(obs, n_obs);

    // Skip full rebuild in worker: these stats are NOT used by grow_parallel.
    // The main thread re-computes everything after merge + its own rebuild.
    result.boxes = boxes_;
    result.n_roots = 1;
    result.n_boxes_total = static_cast<int>(boxes_.size());
    result.n_components = 0;  // not computed (skipped worker rebuild)
    result.n_coarse_boxes = n_coarse_created_;
    result.n_fine_boxes = n_fine_created_;
    result.start_goal_connected = false;  // not computed
    for (const auto& b : boxes_)
        result.total_volume += b.volume;
    return result;
}

void ForestGrower::grow_parallel(const Obstacle* obs, int n_obs,
                                 GrowerResult& result) {
    int n_subtrees = static_cast<int>(subtrees_.size());
    int n_workers = std::min(config_.n_threads, n_subtrees);

    int n_root_boxes = static_cast<int>(boxes_.size());
    auto shared_counter = std::make_shared<std::atomic<int>>(n_root_boxes);

    std::unordered_map<int, Eigen::VectorXd> root_seeds;
    for (const auto& b : boxes_) {
        if (b.parent_box_id == -1)
            root_seeds[b.root_id] = b.seed_config;
    }

    const Robot* robot_ptr = robot_;
    bool has_ep = has_endpoints_;
    Eigen::VectorXd start_cfg = has_ep ? start_config_ : Eigen::VectorXd();
    Eigen::VectorXd goal_cfg  = has_ep ? goal_config_  : Eigen::VectorXd();

    bool use_warm_start = (config_.warm_start_depth >= 0);
    int ws_depth = config_.warm_start_depth;
    if (ws_depth == 0) ws_depth = 3;

    if (use_warm_start && ws_depth > 0 && !lect_cache_loaded_) {
        auto t_ws0 = std::chrono::high_resolution_clock::now();
        int ws_new_nodes = lect_.pre_expand(ws_depth);
        auto t_ws1 = std::chrono::high_resolution_clock::now();
        double ws_ms = std::chrono::duration<double, std::milli>(t_ws1 - t_ws0).count();
        result.phase_times["warm_start_ms"] = ws_ms;
        std::printf("[grower] warm-start: pre-expanded to depth %d, %d new nodes, lect %d total, %.1f ms\n",
                    ws_depth, ws_new_nodes, lect_.n_nodes(), ws_ms);
    }

    if (is_grid_envelope() && !lect_.has_scene_grid()) {
        lect_.set_scene(obs, n_obs);
    }

    int snapshot_n_nodes = lect_.n_nodes();
    bool lect_aligned = (n_subtrees > 0 && subtrees_[0].lect_node_idx >= 0);

    ThreadPool pool(n_workers);
    std::vector<std::future<ParallelWorkerResult>> futures;
    std::vector<int> future_subtree_idx;
    const auto worker_deadline = deadline_;

    for (int i = 0; i < n_subtrees; ++i) {
        int rid = subtrees_[i].root_id;
        auto it = root_seeds.find(rid);
        if (it == root_seeds.end()) continue;

        Eigen::VectorXd seed = it->second;
        std::vector<Interval> limits = subtrees_[i].limits;

        GrowerConfig worker_cfg = config_;
        worker_cfg.max_boxes = config_.max_boxes;
        worker_cfg.rng_seed = config_.rng_seed + static_cast<uint64_t>(i) * 12345ULL + 1;
        worker_cfg.n_threads = 1;
        worker_cfg.warm_start_depth = -1;

        if (use_warm_start) {
            auto warm_ptr = std::make_shared<LECT>(lect_.snapshot());
            futures.push_back(pool.submit(
                [robot_ptr, worker_cfg, has_ep, start_cfg, goal_cfg,
                      seed, rid, limits, obs, n_obs, shared_counter, warm_ptr, worker_deadline]() -> ParallelWorkerResult {
                    ForestGrower worker(*robot_ptr, worker_cfg, std::move(*warm_ptr));
                          worker.set_deadline(worker_deadline);
                    if (has_ep) worker.set_endpoints(start_cfg, goal_cfg);
                    ParallelWorkerResult pwr;
                    pwr.result = worker.grow_subtree(seed, rid, limits, obs, n_obs, shared_counter);
                    pwr.lect = std::move(worker.lect_mut());
                    return pwr;
                }
            ));
        } else {
            futures.push_back(pool.submit(
                [robot_ptr, worker_cfg, has_ep, start_cfg, goal_cfg,
                      seed, rid, limits, obs, n_obs, shared_counter, worker_deadline]() -> ParallelWorkerResult {
                    ForestGrower worker(*robot_ptr, worker_cfg);
                          worker.set_deadline(worker_deadline);
                    if (has_ep) worker.set_endpoints(start_cfg, goal_cfg);
                    ParallelWorkerResult pwr;
                    pwr.result = worker.grow_subtree(seed, rid, limits, obs, n_obs, shared_counter);
                    pwr.lect = std::move(worker.lect_mut());
                    return pwr;
                }
            ));
        }
        future_subtree_idx.push_back(i);
    }

    boxes_.clear();
    box_id_to_idx_.clear();
    next_box_id_ = 0;
    int total_promotions = 0;
    n_coarse_created_ = 0;
    n_fine_created_ = 0;
    int total_transplanted = 0;

    for (int fi = 0; fi < static_cast<int>(futures.size()); ++fi) {
        ParallelWorkerResult pwr = futures[fi].get();
        GrowerResult& wr = pwr.result;
        total_promotions += wr.n_promotions;
        n_coarse_created_ += wr.n_coarse_boxes;
        n_fine_created_ += wr.n_fine_boxes;

        std::unordered_map<int, int> id_map;
        for (auto& box : wr.boxes) {
            int new_id = next_box_id_++;
            id_map[box.id] = new_id;
            box.id = new_id;
        }

        for (auto& box : wr.boxes) {
            if (box.parent_box_id >= 0) {
                auto mit = id_map.find(box.parent_box_id);
                box.parent_box_id = (mit != id_map.end()) ? mit->second : -1;
            }
            box.tree_id = -1;
        }

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

    if (has_endpoints_) {
        start_box_id_ = -1;
        goal_box_id_ = -1;

        auto contains_q = [](const BoxNode& box, const Eigen::VectorXd& q) {
            for (int d = 0; d < static_cast<int>(box.joint_intervals.size()); ++d) {
                if (q[d] < box.joint_intervals[d].lo - 1e-10 ||
                    q[d] > box.joint_intervals[d].hi + 1e-10)
                    return false;
            }
            return true;
        };

        for (const auto& box : boxes_) {
            if (start_box_id_ < 0 && contains_q(box, start_config_))
                start_box_id_ = box.id;
            if (goal_box_id_ < 0 && contains_q(box, goal_config_))
                goal_box_id_ = box.id;
        }
    }

    result.n_promotions = total_promotions;
    if (!deadline_reached())
        bridge_subtree_boundaries(obs, n_obs, result);

    std::printf("[grower] parallel done: %d boxes merged (%d bridge), %d promotions, %d nodes transplanted\n",
                static_cast<int>(boxes_.size()), result.n_bridge_boxes,
                total_promotions, total_transplanted);
}

void ForestGrower::sync_lect_occupation() {
    for (const auto& box : boxes_) {
        int node = 0;
        while (!lect_.is_leaf(node)) {
            int dim = lect_.get_node_split_dim(node);
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

void ForestGrower::bridge_subtree_boundaries(const Obstacle* obs, int n_obs,
                                             GrowerResult& result) {
    if (subtrees_.size() < 2) return;
    if (config_.bridge_samples <= 0) return;

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

    for (int i = 0; i < n_st; ++i) {
        if (deadline_reached()) break;
        for (int j = i + 1; j < n_st; ++j) {
            if (deadline_reached()) break;
            const auto& li = subtrees_[i].limits;
            const auto& lj = subtrees_[j].limits;

            for (int d = 0; d < n_dims; ++d) {
                if (deadline_reached()) break;
                bool face_ij = (std::abs(li[d].hi - lj[d].lo) < 1e-10);
                bool face_ji = (std::abs(lj[d].hi - li[d].lo) < 1e-10);
                if (!face_ij && !face_ji) continue;

                bool overlap = true;
                for (int dd = 0; dd < n_dims; ++dd) {
                    if (dd == d) continue;
                    double lo_max = std::max(li[dd].lo, lj[dd].lo);
                    double hi_min = std::min(li[dd].hi, lj[dd].hi);
                    if (lo_max >= hi_min) { overlap = false; break; }
                }
                if (!overlap) continue;

                double boundary_val = face_ij ? li[d].hi : li[d].lo;
                double j_sign = face_ij ? +1.0 : -1.0;

                // Collect seed_configs from each subtree near the boundary.
                // IMPORTANT: store copies (not pointers into boxes_) because
                //   try_create_box → boxes_.push_back may invalidate pointers.
                struct SeedInfo { Eigen::VectorXd cfg; };
                std::vector<SeedInfo> near_i, near_j;
                double proximity = std::max(eps * 50.0, 0.5);

                for (const auto& box : boxes_) {
                    double d_lo = box.joint_intervals[d].lo;
                    double d_hi = box.joint_intervals[d].hi;
                    double dist = std::min(std::abs(d_lo - boundary_val),
                                           std::abs(d_hi - boundary_val));
                    if (dist > proximity) continue;

                    if (box.root_id == subtrees_[i].root_id)
                        near_i.push_back({box.seed_config});
                    if (box.root_id == subtrees_[j].root_id)
                        near_j.push_back({box.seed_config});
                }

                // Pre-compute nearest pairs for midpoint seeding
                struct NearPair { int i_idx; int j_idx; double dist; };
                std::vector<NearPair> pairs;
                if (!near_i.empty() && !near_j.empty()) {
                    // Limit pair search to avoid O(N²) blowup
                    int max_pairs = std::min(
                        static_cast<int>(near_i.size()) * static_cast<int>(near_j.size()),
                        500);
                    int ni = static_cast<int>(near_i.size());
                    int nj = static_cast<int>(near_j.size());
                    for (int ii = 0; ii < ni && static_cast<int>(pairs.size()) < max_pairs; ++ii) {
                        for (int jj = 0; jj < nj && static_cast<int>(pairs.size()) < max_pairs; ++jj) {
                            double dd2 = 0.0;
                            for (int dd = 0; dd < n_dims; ++dd) {
                                double diff = near_i[ii].cfg[dd] - near_j[jj].cfg[dd];
                                dd2 += diff * diff;
                            }
                            pairs.push_back({ii, jj, dd2});
                        }
                    }
                    std::sort(pairs.begin(), pairs.end(),
                        [](const NearPair& a, const NearPair& b) {
                            return a.dist < b.dist;
                        });
                }

                for (int s = 0; s < seeds_per_face; ++s) {
                    if (deadline_reached()) break;
                    bool to_j = (s % 2 == 0);

                    Eigen::VectorXd seed(n_dims);
                    if (!pairs.empty()) {
                        // Midpoint-based seed: choose near pair, use midpoint
                        // in non-boundary dims for proximity to both subtrees
                        int pidx = s % static_cast<int>(pairs.size());
                        const auto& pr = pairs[pidx];
                        const auto& cfg_i = near_i[pr.i_idx].cfg;
                        const auto& cfg_j = near_j[pr.j_idx].cfg;
                        for (int dd = 0; dd < n_dims; ++dd) {
                            if (dd == d) continue;
                            double alpha = 0.3 + u01(rng_) * 0.4;
                            seed[dd] = alpha * cfg_i[dd]
                                     + (1.0 - alpha) * cfg_j[dd];
                        }
                        double offset = u01(rng_) * eps;
                        seed[d] = boundary_val + (to_j ? j_sign : -j_sign) * offset;
                    } else {
                        // Fallback: one side empty — use source or random
                        const auto& near_src = to_j ? near_i : near_j;
                        if (!near_src.empty()) {
                            seed = near_src[s / 2 % near_src.size()].cfg;
                        } else {
                            for (int dd = 0; dd < n_dims; ++dd) {
                                double lo_max = std::max(li[dd].lo, lj[dd].lo);
                                double hi_min = std::min(li[dd].hi, lj[dd].hi);
                                seed[dd] = lo_max + u01(rng_) * (hi_min - lo_max);
                            }
                        }
                        double offset = u01(rng_) * eps;
                        seed[d] = boundary_val + (to_j ? j_sign : -j_sign) * offset;
                    }

                    seed = robot_->joint_limits().clamp(seed);
                    int bridge_root = to_j ? subtrees_[j].root_id : subtrees_[i].root_id;

                    int bid = try_create_box(seed, obs, n_obs, -1, d,
                                             face_ij ? 1 : 0, bridge_root);
                    if (bid >= 0) {
                        bridge_count++;
                    } else if (deadline_reached()) {
                        break;
                    }
                }
            }
        }
    }

    result.n_bridge_boxes = bridge_count;
}

const LECT& ForestGrower::lect() const { return lect_; }
LECT& ForestGrower::lect_mut() { return lect_; }
const AdjacencyGraph& ForestGrower::graph() const { return graph_; }
const std::vector<BoxNode>& ForestGrower::boxes() const { return boxes_; }
const GrowerConfig& ForestGrower::config() const { return config_; }
int ForestGrower::n_boxes() const { return static_cast<int>(boxes_.size()); }

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

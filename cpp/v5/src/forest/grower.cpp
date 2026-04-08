// SafeBoxForest v5 — Forest Grower (Phase F + parallel)
#include <sbf/forest/grower.h>
#include <sbf/forest/thread_pool.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <queue>
#include <unordered_map>

namespace sbf {

// ─── Constructor ────────────────────────────────────────────────────────────
ForestGrower::ForestGrower(const Robot& robot, LECT& lect,
                           const GrowerConfig& config)
    : robot_(robot), lect_owned_(), lect_(lect), config_(config),
      rng_(config.rng_seed)
{}

ForestGrower::ForestGrower(const Robot& robot, LECT&& lect_owned,
                           const GrowerConfig& config)
    : robot_(robot), lect_owned_(std::move(lect_owned)),
      lect_(lect_owned_), config_(config),
      rng_(config.rng_seed)
{}

void ForestGrower::set_endpoints(const Eigen::VectorXd& start,
                                 const Eigen::VectorXd& goal) {
    has_endpoints_ = true;
    start_ = start;
    goal_ = goal;
}

void ForestGrower::set_deadline(Clock::time_point deadline) {
    deadline_ = deadline;
    has_deadline_ = true;
}

// ─── Helpers ────────────────────────────────────────────────────────────────
bool ForestGrower::deadline_reached() const {
    if (!has_deadline_) return false;
    return Clock::now() >= deadline_;
}

Eigen::VectorXd ForestGrower::sample_random() const {
    const auto& limits = robot_.joint_limits().limits;
    const int nd = static_cast<int>(limits.size());
    Eigen::VectorXd q(nd);
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    for (int d = 0; d < nd; ++d)
        q[d] = limits[d].lo + u01(rng_) * limits[d].width();
    return q;
}

Eigen::VectorXd ForestGrower::clamp_to_limits(const Eigen::VectorXd& q) const {
    const auto& limits = robot_.joint_limits().limits;
    Eigen::VectorXd c = q;
    for (int d = 0; d < static_cast<int>(limits.size()); ++d)
        c[d] = std::clamp(q[d], limits[d].lo, limits[d].hi);
    return c;
}

// ─── try_create_box ─────────────────────────────────────────────────────────
int ForestGrower::try_create_box(const Eigen::VectorXd& seed,
                                 const Obstacle* obs, int n_obs,
                                 int parent_box_id, int /*face_dim*/,
                                 int /*face_side*/, int root_id) {
    if (deadline_reached()) return -1;

    // In parallel mode, check global box count
    if (shared_box_count_ &&
        shared_box_count_->load(std::memory_order_relaxed) >= config_.max_boxes)
        return -1;

    // Reject seeds that already lie inside an existing box.
    for (const auto& b : boxes_) {
        if (b.contains(seed)) {
            n_ffb_fail_++;
            return -1;
        }
    }

    FFBResult ffb = find_free_box(lect_, seed, obs, n_obs, config_.ffb_config);

    // Accumulate FFB stats
    ffb_total_calls_++;
    ffb_total_ms_ += ffb.total_ms;
    ffb_envelope_ms_ += ffb.envelope_ms;
    ffb_collide_ms_ += ffb.collide_ms;
    ffb_expand_ms_ += ffb.expand_ms;
    ffb_intervals_ms_ += ffb.intervals_ms;
    ffb_cache_hits_ += ffb.n_cache_hits;
    ffb_cache_misses_ += ffb.n_cache_misses;
    ffb_collide_calls_ += ffb.n_collide_calls;
    ffb_expand_calls_ += ffb.n_expand_calls;
    ffb_total_steps_ += ffb.n_steps;

    if (!ffb.success()) {
        n_ffb_fail_++;
        return -1;
    }
    if (lect_.is_occupied(ffb.node_idx)) {
        n_ffb_fail_++;
        return -1;
    }

    BoxNode box;
    box.id = next_box_id_++;
    box.joint_intervals = lect_.node_intervals(ffb.node_idx);
    box.seed_config = seed;
    box.tree_id = ffb.node_idx;
    box.parent_box_id = parent_box_id;
    box.root_id = root_id;
    box.compute_volume();

    lect_.mark_occupied(ffb.node_idx, box.id);
    n_ffb_success_++;

    boxes_.push_back(std::move(box));
    if (shared_box_count_)
        shared_box_count_->fetch_add(1, std::memory_order_relaxed);
    return boxes_.back().id;
}

// ─── snap_to_face ───────────────────────────────────────────────────────────
ForestGrower::SnapResult ForestGrower::snap_to_face(
        const BoxNode& nearest,
        const Eigen::VectorXd& direction) const {
    const int nd = nearest.n_dims();
    const auto& limits = robot_.joint_limits().limits;
    const double eps = config_.boundary_epsilon;

    int best_dim = -1;
    int best_side = -1;
    double best_score = -1e30;

    for (int d = 0; d < nd; ++d) {
        for (int side = 0; side < 2; ++side) {
            double normal_sign = (side == 1) ? 1.0 : -1.0;
            double score = direction[d] * normal_sign;
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

    std::uniform_real_distribution<double> u01(0.0, 1.0);
    Eigen::VectorXd seed(nd);
    Eigen::VectorXd nc = nearest.center();

    if (best_dim < 0) {
        // No valid face — just step in direction
        double max_width = 0.0;
        for (int d = 0; d < nd; ++d)
            max_width = std::max(max_width, limits[d].width());
        double step = config_.rrt_step_ratio * max_width;
        seed = nc + direction * step;
        return {clamp_to_limits(seed), -1, -1};
    }

    for (int d = 0; d < nd; ++d) {
        if (d == best_dim) {
            seed[d] = (best_side == 0)
                ? nearest.joint_intervals[d].lo - eps
                : nearest.joint_intervals[d].hi + eps;
        } else {
            double lo = nearest.joint_intervals[d].lo;
            double hi = nearest.joint_intervals[d].hi;
            double target = nc[d] + direction[d] *
                            config_.rrt_step_ratio * limits[d].width();
            target = std::clamp(target, lo, hi);
            double rand_on_face = lo + u01(rng_) * (hi - lo);
            seed[d] = 0.7 * target + 0.3 * rand_on_face;
        }
    }
    return {clamp_to_limits(seed), best_dim, best_side};
}

// ─── sample_boundary ────────────────────────────────────────────────────────
std::vector<ForestGrower::BoundarySeed> ForestGrower::sample_boundary(
        const BoxNode& box,
        const Eigen::VectorXd* bias_target) const {
    std::vector<BoundarySeed> seeds;
    const int nd = box.n_dims();
    const auto& limits = robot_.joint_limits().limits;
    const double eps = config_.boundary_epsilon;

    struct Face { int dim; int side; double priority; };
    std::vector<Face> faces;
    for (int d = 0; d < nd; ++d) {
        if (box.joint_intervals[d].lo - eps >= limits[d].lo)
            faces.push_back({d, 0, 0.0});
        if (box.joint_intervals[d].hi + eps <= limits[d].hi)
            faces.push_back({d, 1, 0.0});
    }
    if (faces.empty()) return seeds;

    if (bias_target) {
        Eigen::VectorXd bc = box.center();
        Eigen::VectorXd to_target = *bias_target - bc;
        for (auto& f : faces)
            f.priority = (f.side == 1) ? to_target[f.dim] : -to_target[f.dim];
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

        Eigen::VectorXd seed(nd);
        for (int d = 0; d < nd; ++d) {
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
        seeds.push_back({face.dim, face.side, clamp_to_limits(seed)});
    }
    return seeds;
}

// ─── select_roots ───────────────────────────────────────────────────────────
void ForestGrower::select_roots(const Obstacle* obs, int n_obs) {
    // Use the first wavefront stage's min_edge for root creation —
    // avoids expensive deep FFB at default min_edge=1e-4 for high-DOF robots.
    FFBConfig saved_ffb = config_.ffb_config;
    if (!config_.wavefront_stages.empty())
        config_.ffb_config.min_edge = config_.wavefront_stages[0].min_edge;

    if (has_endpoints_) {
        // Root 0 = start, Root 1 = goal
        int id0 = try_create_box(start_, obs, n_obs, -1, -1, -1, 0);
        int id1 = try_create_box(goal_, obs, n_obs, -1, -1, -1, 1);
        fprintf(stderr, "[GRW] roots: id0=%d id1=%d boxes=%d\n",
                id0, id1, (int)boxes_.size());
    }

    // Diversity roots if we have room (time-budgeted to avoid
    // spending the whole timeout on failed FFB for high-DOF robots)
    {
        int n_desired = std::max(2, static_cast<int>(
            std::sqrt(static_cast<double>(config_.max_boxes))));
        int n_have = static_cast<int>(boxes_.size());

        // Budget: at most 10% of timeout (or 2s if no timeout)
        auto root_deadline = Clock::now() + std::chrono::milliseconds(
            config_.timeout_ms > 0.0
                ? static_cast<int64_t>(config_.timeout_ms * 0.10)
                : 2000);

        constexpr int K_CANDIDATES = 30;
        for (int r = n_have; r < n_desired; ++r) {
            if (deadline_reached()) return;
            if (Clock::now() >= root_deadline) break;

            // Farthest point sampling
            std::vector<Eigen::VectorXd> candidates;
            candidates.reserve(K_CANDIDATES);
            for (int k = 0; k < K_CANDIDATES; ++k)
                candidates.push_back(sample_random());

            double best_score = -1.0;
            int best_k = 0;
            for (int k = 0; k < K_CANDIDATES; ++k) {
                double min_dist = std::numeric_limits<double>::max();
                for (const auto& b : boxes_)
                    min_dist = std::min(min_dist, (candidates[k] - b.center()).norm());
                if (boxes_.empty()) min_dist = 1.0;  // any is fine
                if (min_dist > best_score) {
                    best_score = min_dist;
                    best_k = k;
                }
            }

            try_create_box(candidates[best_k], obs, n_obs, -1, -1, -1, r);
        }
    }

    config_.ffb_config = saved_ffb;
}

// ─── grow_rrt ───────────────────────────────────────────────────────────────
void ForestGrower::grow_rrt(const Obstacle* obs, int n_obs) {
    int miss_count = 0;
    const int nd = robot_.n_joints();
    const auto& limits = robot_.joint_limits().limits;

    double max_width = 0.0;
    for (int d = 0; d < nd; ++d)
        max_width = std::max(max_width, limits[d].width());

    std::uniform_real_distribution<double> u01(0.0, 1.0);

    while (static_cast<int>(boxes_.size()) < config_.max_boxes &&
           miss_count < config_.max_consecutive_miss &&
           !deadline_reached()) {

        // 1. Sample
        Eigen::VectorXd q_rand;
        if (has_endpoints_ && u01(rng_) < config_.rrt_goal_bias) {
            q_rand = (u01(rng_) < 0.5) ? goal_ : start_;
        } else {
            q_rand = sample_random();
        }

        // 2. Find nearest box
        if (boxes_.empty()) { miss_count++; continue; }
        double best_dist = std::numeric_limits<double>::max();
        int best_idx = 0;
        for (int i = 0; i < static_cast<int>(boxes_.size()); ++i) {
            double d = (boxes_[i].center() - q_rand).squaredNorm();
            if (d < best_dist) { best_dist = d; best_idx = i; }
        }

        // 3. Direction + snap
        Eigen::VectorXd direction = q_rand - boxes_[best_idx].center();
        double dir_norm = direction.norm();
        if (dir_norm < 1e-12) { miss_count++; continue; }
        direction /= dir_norm;

        auto snap = snap_to_face(boxes_[best_idx], direction);

        // 4. Create box
        int bid = try_create_box(
            snap.seed, obs, n_obs,
            boxes_[best_idx].id, snap.face_dim, snap.face_side,
            boxes_[best_idx].root_id);

        if (bid >= 0)
            miss_count = 0;
        else
            miss_count++;
    }
}

// ─── grow_wavefront ─────────────────────────────────────────────────────────
void ForestGrower::grow_wavefront(const Obstacle* obs, int n_obs) {
    int miss_count = 0;
    const auto& stages = config_.wavefront_stages;
    if (stages.empty()) return;

    // Override FFB min_edge per stage
    FFBConfig stage_ffb = config_.ffb_config;

    int current_stage = 0;
    stage_ffb.min_edge = stages[0].min_edge;

    struct WaveEntry {
        int box_id;
        double priority;
        bool operator<(const WaveEntry& o) const { return priority < o.priority; }
    };

    std::priority_queue<WaveEntry> pq;
    for (const auto& b : boxes_)
        pq.push({b.id, b.volume});

    // Build box_id → index map
    std::unordered_map<int, int> id_to_idx;
    for (int i = 0; i < static_cast<int>(boxes_.size()); ++i)
        id_to_idx[boxes_[i].id] = i;

    // Save original ffb config, swap in stage config
    FFBConfig saved_ffb = config_.ffb_config;

    while (static_cast<int>(boxes_.size()) < config_.max_boxes &&
           miss_count < config_.max_consecutive_miss &&
           !deadline_reached()) {

        // Adaptive staging
        if (current_stage + 1 < static_cast<int>(stages.size()) &&
            static_cast<int>(boxes_.size()) >= stages[current_stage].box_limit) {
            current_stage++;
            stage_ffb.min_edge = stages[current_stage].min_edge;

            // Rebuild queue
            while (!pq.empty()) pq.pop();
            for (const auto& b : boxes_)
                pq.push({b.id, b.volume});
            miss_count = 0;
        }

        config_.ffb_config.min_edge = stage_ffb.min_edge;

        if (!pq.empty()) {
            WaveEntry entry = pq.top();
            pq.pop();

            auto it = id_to_idx.find(entry.box_id);
            if (it == id_to_idx.end()) continue;

            const BoxNode& box = boxes_[it->second];
            // Copy data needed after try_create_box (which may reallocate boxes_)
            const int box_id = box.id;
            const int box_root_id = box.root_id;
            const Eigen::VectorXd* bias = nullptr;
            if (has_endpoints_) {
                if (box_root_id == 0) bias = &goal_;
                else if (box_root_id == 1) bias = &start_;
            }

            auto bseeds = sample_boundary(box, bias);
            if (bseeds.empty()) { miss_count++; continue; }
            for (const auto& bs : bseeds) {
                if (static_cast<int>(boxes_.size()) >= config_.max_boxes ||
                    deadline_reached()) break;

                int bid = try_create_box(
                    bs.config, obs, n_obs,
                    box_id, bs.dim, bs.side, box_root_id);

                if (bid >= 0) {
                    int new_idx = static_cast<int>(boxes_.size()) - 1;
                    id_to_idx[bid] = new_idx;
                    pq.push({bid, boxes_.back().volume});
                    miss_count = 0;
                } else {
                    miss_count++;
                }
            }
        } else {
            // Fallback: random boundary from existing box.
            int fallback_limit = std::min(config_.max_consecutive_miss,
                                          config_.n_boundary_samples * 3);
            if (miss_count >= fallback_limit) break;

            if (boxes_.empty()) break;
            std::uniform_int_distribution<int> box_dist(
                0, static_cast<int>(boxes_.size()) - 1);
            const BoxNode& rb = boxes_[box_dist(rng_)];
            // Copy data needed after try_create_box (which may reallocate boxes_)
            const int rb_id = rb.id;
            const int rb_root_id = rb.root_id;
            auto bseeds = sample_boundary(rb, nullptr);
            if (bseeds.empty()) { miss_count++; continue; }
            for (const auto& bs : bseeds) {
                if (static_cast<int>(boxes_.size()) >= config_.max_boxes ||
                    deadline_reached()) break;
                int bid = try_create_box(
                    bs.config, obs, n_obs,
                    rb_id, bs.dim, bs.side, rb_root_id);
                if (bid >= 0) {
                    int new_idx = static_cast<int>(boxes_.size()) - 1;
                    id_to_idx[bid] = new_idx;
                    pq.push({bid, boxes_.back().volume});
                    miss_count = 0;
                } else {
                    miss_count++;
                }
            }
        }
    }

    config_.ffb_config = saved_ffb;
}

// ─── promote_all ────────────────────────────────────────────────────────────
int ForestGrower::promote_all(const Obstacle* obs, int n_obs) {
    int total = 0;
    bool changed = true;

    // Build box_id → index for fast removal
    std::unordered_map<int, int> id_to_idx;
    for (int i = 0; i < static_cast<int>(boxes_.size()); ++i)
        id_to_idx[boxes_[i].id] = i;

    while (changed && !deadline_reached()) {
        changed = false;
        int n_nodes = lect_.n_nodes();

        for (int i = 0; i < n_nodes; ++i) {
            if (deadline_reached()) break;
            if (lect_.is_leaf(i)) continue;
            if (lect_.is_occupied(i)) continue;

            int li = lect_.left(i);
            int ri = lect_.right(i);
            if (li < 0 || ri < 0) continue;
            if (!lect_.is_leaf(li) || !lect_.is_leaf(ri)) continue;
            if (!lect_.is_occupied(li) || !lect_.is_occupied(ri)) continue;

            // Check parent envelope collision
            auto parent_ivs = lect_.node_intervals(i);
            if (lect_.intervals_collide_scene(parent_ivs, obs, n_obs))
                continue;

            int li_box_id = lect_.forest_id(li);
            int ri_box_id = lect_.forest_id(ri);

            // Determine root_id from children
            int promoted_root = -1;
            {
                auto it = id_to_idx.find(li_box_id);
                if (it != id_to_idx.end())
                    promoted_root = boxes_[it->second].root_id;
            }

            // Remove child boxes
            lect_.unmark_occupied(li);
            lect_.unmark_occupied(ri);

            std::vector<int> remove_idxs;
            {
                auto it = id_to_idx.find(li_box_id);
                if (it != id_to_idx.end()) remove_idxs.push_back(it->second);
                it = id_to_idx.find(ri_box_id);
                if (it != id_to_idx.end()) remove_idxs.push_back(it->second);
            }
            std::sort(remove_idxs.rbegin(), remove_idxs.rend());

            id_to_idx.erase(li_box_id);
            id_to_idx.erase(ri_box_id);

            for (int idx : remove_idxs) {
                int last = static_cast<int>(boxes_.size()) - 1;
                if (idx < last) {
                    id_to_idx[boxes_[last].id] = idx;
                    boxes_[idx] = std::move(boxes_[last]);
                }
                boxes_.pop_back();
            }

            // Create promoted box
            BoxNode new_box;
            new_box.id = next_box_id_++;
            new_box.joint_intervals = parent_ivs;
            new_box.tree_id = i;
            new_box.root_id = promoted_root;
            new_box.parent_box_id = -1;
            Eigen::VectorXd pc(static_cast<int>(parent_ivs.size()));
            for (int d = 0; d < static_cast<int>(parent_ivs.size()); ++d)
                pc[d] = parent_ivs[d].center();
            new_box.seed_config = pc;
            new_box.compute_volume();

            lect_.mark_occupied(i, new_box.id);
            id_to_idx[new_box.id] = static_cast<int>(boxes_.size());
            boxes_.push_back(std::move(new_box));

            total++;
            changed = true;
        }
    }
    return total;
}

// ─── grow (main entry) ─────────────────────────────────────────────────────
GrowerResult ForestGrower::grow(const Obstacle* obs, int n_obs) {
    auto t0 = Clock::now();

    // Set deadline
    if (config_.timeout_ms > 0.0) {
        deadline_ = t0 + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double, std::milli>(config_.timeout_ms));
        has_deadline_ = true;
    }

    boxes_.clear();
    next_box_id_ = 0;
    n_ffb_success_ = 0;
    n_ffb_fail_ = 0;
    ffb_total_ms_ = 0.0;
    ffb_envelope_ms_ = 0.0;
    ffb_collide_ms_ = 0.0;
    ffb_expand_ms_ = 0.0;
    ffb_intervals_ms_ = 0.0;
    ffb_cache_hits_ = 0;
    ffb_cache_misses_ = 0;
    ffb_collide_calls_ = 0;
    ffb_expand_calls_ = 0;
    ffb_total_steps_ = 0;
    ffb_total_calls_ = 0;
    lect_.expand_profile_.reset();

    // 1. Root selection
    auto t_roots = Clock::now();
    select_roots(obs, n_obs);
    double roots_ms = std::chrono::duration<double, std::milli>(Clock::now() - t_roots).count();

    // 2. Growth
    auto t_wave = Clock::now();
    int n_promotions = 0;

    if (config_.n_threads > 1 && static_cast<int>(boxes_.size()) >= 2) {
        // Parallel path
        GrowerResult par_result;
        grow_parallel(obs, n_obs, par_result);
        n_promotions = par_result.n_promotions;
        double wave_ms = std::chrono::duration<double, std::milli>(Clock::now() - t_wave).count();
        fprintf(stderr, "[GRW] timing: roots=%.0fms parallel_grow=%.0fms\n",
                roots_ms, wave_ms);
    } else {
        // Serial path
        if (config_.mode == GrowerConfig::Mode::WAVEFRONT)
            grow_wavefront(obs, n_obs);
        else
            grow_rrt(obs, n_obs);
        double wave_ms = std::chrono::duration<double, std::milli>(Clock::now() - t_wave).count();

        // 3. Promotion
        auto t_promo = Clock::now();
        if (config_.enable_promotion && !deadline_reached())
            n_promotions = promote_all(obs, n_obs);
        double promo_ms = std::chrono::duration<double, std::milli>(Clock::now() - t_promo).count();

        fprintf(stderr, "[GRW] timing: roots=%.0fms wave=%.0fms promo=%.0fms\n",
                roots_ms, wave_ms, promo_ms);
    }

    fprintf(stderr, "[GRW] final: boxes=%d success=%d fail=%d promo=%d\n",
            (int)boxes_.size(), n_ffb_success_, n_ffb_fail_, n_promotions);

    fprintf(stderr, "[GRW] ffb_stats: calls=%d steps=%d cache_hits=%d cache_misses=%d\n",
            ffb_total_calls_, ffb_total_steps_, ffb_cache_hits_, ffb_cache_misses_);
    fprintf(stderr, "[GRW] ffb_timing: total=%.1fms envelope=%.1fms collide=%.1fms expand=%.1fms intervals=%.1fms\n",
            ffb_total_ms_, ffb_envelope_ms_, ffb_collide_ms_, ffb_expand_ms_, ffb_intervals_ms_);
    if (ffb_cache_misses_ > 0)
        fprintf(stderr, "[GRW] ffb_avg_miss: envelope=%.3fms/call\n",
                ffb_envelope_ms_ / ffb_cache_misses_);
    if (ffb_expand_calls_ > 0)
        fprintf(stderr, "[GRW] ffb_avg_expand: %.3fms/call (%d calls)\n",
                ffb_expand_ms_ / ffb_expand_calls_, ffb_expand_calls_);
    {
        const auto& ep = lect_.expand_profile_;
        if (ep.expand_calls > 0) {
            fprintf(stderr, "[GRW] expand_profile: calls=%d new_nodes=%d pick_dim=%.1fms fk=%.1fms env=%.1fms refine=%.1fms\n",
                    ep.expand_calls, ep.expand_calls * 2, ep.pick_dim_ms, ep.fk_inc_ms, ep.envelope_ms, ep.refine_ms);
            fprintf(stderr, "[GRW] expand_avg: pick_dim=%.3fms fk=%.3fms env=%.3fms refine=%.3fms per_call=%.3fms\n",
                    ep.pick_dim_ms / ep.expand_calls,
                    ep.fk_inc_ms / ep.expand_calls,
                    ep.envelope_ms / ep.expand_calls,
                    ep.refine_ms / ep.expand_calls,
                    (ep.pick_dim_ms + ep.fk_inc_ms + ep.envelope_ms + ep.refine_ms) / ep.expand_calls);
            fprintf(stderr, "[GRW] expand_dim: calls=%d cache_hits=%d (%.1f%%)\n",
                    ep.pick_dim_calls, ep.pick_dim_cache_hits,
                    ep.pick_dim_calls > 0 ? 100.0 * ep.pick_dim_cache_hits / ep.pick_dim_calls : 0.0);
        }
    }
    fprintf(stderr, "[GRW] lect_nodes: %d\n", lect_.n_nodes());
    fflush(stderr);

    // 4. Result
    GrowerResult result;
    result.boxes = boxes_;
    result.n_roots = static_cast<int>(
        std::count_if(boxes_.begin(), boxes_.end(),
                      [](const BoxNode& b) { return b.parent_box_id == -1; }));
    result.n_ffb_success = n_ffb_success_;
    result.n_ffb_fail = n_ffb_fail_;
    result.n_promotions = n_promotions;

    for (const auto& b : boxes_)
        result.total_volume += b.volume;

    result.ffb_total_calls = ffb_total_calls_;
    result.ffb_total_ms = ffb_total_ms_;
    result.ffb_envelope_ms = ffb_envelope_ms_;
    result.ffb_collide_ms = ffb_collide_ms_;
    result.ffb_expand_ms = ffb_expand_ms_;
    result.ffb_intervals_ms = ffb_intervals_ms_;
    result.ffb_cache_hits = ffb_cache_hits_;
    result.ffb_cache_misses = ffb_cache_misses_;
    result.ffb_collide_calls = ffb_collide_calls_;
    result.ffb_expand_calls = ffb_expand_calls_;
    result.ffb_total_steps = ffb_total_steps_;
    result.lect_nodes_final = lect_.n_nodes();

    auto t1 = Clock::now();
    result.build_time_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();

    return result;
}

// ─── grow_subtree (worker entry point) ──────────────────────────────────────
GrowerResult ForestGrower::grow_subtree(const Eigen::VectorXd& root_seed,
                                        int root_id,
                                        const Obstacle* obs, int n_obs,
                                        std::shared_ptr<std::atomic<int>> shared_counter) {
    shared_box_count_ = std::move(shared_counter);
    boxes_.clear();
    next_box_id_ = 0;
    n_ffb_success_ = 0;
    n_ffb_fail_ = 0;
    ffb_total_ms_ = 0.0;
    ffb_envelope_ms_ = 0.0;
    ffb_collide_ms_ = 0.0;
    ffb_expand_ms_ = 0.0;
    ffb_intervals_ms_ = 0.0;
    ffb_cache_hits_ = 0;
    ffb_cache_misses_ = 0;
    ffb_collide_calls_ = 0;
    ffb_expand_calls_ = 0;
    ffb_total_steps_ = 0;
    ffb_total_calls_ = 0;

    // Create root box for this subtree
    FFBConfig saved_ffb = config_.ffb_config;
    if (!config_.wavefront_stages.empty())
        config_.ffb_config.min_edge = config_.wavefront_stages[0].min_edge;

    int bid = try_create_box(root_seed, obs, n_obs, -1, -1, -1, root_id);
    config_.ffb_config = saved_ffb;

    if (bid < 0) {
        GrowerResult result;
        return result;
    }

    // Grow
    if (config_.mode == GrowerConfig::Mode::WAVEFRONT)
        grow_wavefront(obs, n_obs);
    else
        grow_rrt(obs, n_obs);

    // Promote
    int n_promotions = 0;
    if (config_.enable_promotion && !deadline_reached())
        n_promotions = promote_all(obs, n_obs);

    // Assemble result
    GrowerResult result;
    result.boxes = boxes_;
    result.n_roots = 1;
    result.n_ffb_success = n_ffb_success_;
    result.n_ffb_fail = n_ffb_fail_;
    result.n_promotions = n_promotions;
    for (const auto& b : boxes_)
        result.total_volume += b.volume;
    result.ffb_total_calls = ffb_total_calls_;
    result.ffb_total_ms = ffb_total_ms_;
    result.ffb_envelope_ms = ffb_envelope_ms_;
    result.ffb_collide_ms = ffb_collide_ms_;
    result.ffb_expand_ms = ffb_expand_ms_;
    result.ffb_intervals_ms = ffb_intervals_ms_;
    result.ffb_cache_hits = ffb_cache_hits_;
    result.ffb_cache_misses = ffb_cache_misses_;
    result.ffb_collide_calls = ffb_collide_calls_;
    result.ffb_expand_calls = ffb_expand_calls_;
    result.ffb_total_steps = ffb_total_steps_;
    result.lect_nodes_final = lect_.n_nodes();

    return result;
}

// ─── grow_parallel ──────────────────────────────────────────────────────────
void ForestGrower::grow_parallel(const Obstacle* obs, int n_obs,
                                 GrowerResult& result) {
    // Collect root boxes and their seeds
    struct RootInfo {
        int root_id;
        Eigen::VectorXd seed;
    };
    std::vector<RootInfo> roots;
    for (const auto& b : boxes_) {
        if (b.parent_box_id == -1)
            roots.push_back({b.root_id, b.seed_config});
    }

    int n_subtrees = static_cast<int>(roots.size());
    int n_workers = std::min(config_.n_threads, n_subtrees);

    if (n_workers <= 1) {
        // Fall back to serial
        if (config_.mode == GrowerConfig::Mode::WAVEFRONT)
            grow_wavefront(obs, n_obs);
        else
            grow_rrt(obs, n_obs);
        if (config_.enable_promotion && !deadline_reached())
            result.n_promotions = promote_all(obs, n_obs);
        return;
    }

    int n_root_boxes = static_cast<int>(boxes_.size());
    auto shared_counter = std::make_shared<std::atomic<int>>(n_root_boxes);
    int snapshot_base = lect_.n_nodes();

    const Robot* robot_ptr = &robot_;
    const auto worker_deadline = deadline_;

    // Submit workers to thread pool
    ThreadPool pool(n_workers);
    std::vector<std::future<ParallelWorkerResult>> futures;

    for (int i = 0; i < n_subtrees; ++i) {
        GrowerConfig worker_cfg = config_;
        worker_cfg.max_boxes = config_.max_boxes;
        worker_cfg.rng_seed = config_.rng_seed +
                              static_cast<uint64_t>(i) * 12345ULL + 1;
        worker_cfg.n_threads = 1;  // no recursive parallelism

        auto warm_ptr = std::make_shared<LECT>(lect_.snapshot());
        Eigen::VectorXd seed = roots[i].seed;
        int rid = roots[i].root_id;
        bool has_ep = has_endpoints_;
        Eigen::VectorXd start_cfg = has_ep ? start_ : Eigen::VectorXd();
        Eigen::VectorXd goal_cfg = has_ep ? goal_ : Eigen::VectorXd();

        futures.push_back(pool.submit(
            [robot_ptr, worker_cfg, has_ep, start_cfg, goal_cfg,
             seed, rid, obs, n_obs, shared_counter, warm_ptr,
             worker_deadline]() -> ParallelWorkerResult {
                ForestGrower worker(*robot_ptr, std::move(*warm_ptr), worker_cfg);
                worker.set_deadline(worker_deadline);
                if (has_ep) worker.set_endpoints(start_cfg, goal_cfg);
                ParallelWorkerResult pwr;
                pwr.result = worker.grow_subtree(seed, rid, obs, n_obs,
                                                 shared_counter);
                pwr.lect = std::move(worker.take_lect());
                return pwr;
            }
        ));
    }

    // Clear master boxes (we'll rebuild from worker results)
    boxes_.clear();
    next_box_id_ = 0;
    n_ffb_success_ = 0;
    n_ffb_fail_ = 0;
    ffb_total_ms_ = 0.0;
    ffb_envelope_ms_ = 0.0;
    ffb_collide_ms_ = 0.0;
    ffb_expand_ms_ = 0.0;
    ffb_intervals_ms_ = 0.0;
    ffb_cache_hits_ = 0;
    ffb_cache_misses_ = 0;
    ffb_collide_calls_ = 0;
    ffb_expand_calls_ = 0;
    ffb_total_steps_ = 0;
    ffb_total_calls_ = 0;
    int total_promotions = 0;
    int total_transplanted = 0;

    // Collect results and merge
    for (int fi = 0; fi < static_cast<int>(futures.size()); ++fi) {
        ParallelWorkerResult pwr = futures[fi].get();
        GrowerResult& wr = pwr.result;

        // Accumulate stats
        total_promotions += wr.n_promotions;
        n_ffb_success_ += wr.n_ffb_success;
        n_ffb_fail_ += wr.n_ffb_fail;
        ffb_total_ms_ += wr.ffb_total_ms;
        ffb_envelope_ms_ += wr.ffb_envelope_ms;
        ffb_collide_ms_ += wr.ffb_collide_ms;
        ffb_expand_ms_ += wr.ffb_expand_ms;
        ffb_intervals_ms_ += wr.ffb_intervals_ms;
        ffb_cache_hits_ += wr.ffb_cache_hits;
        ffb_cache_misses_ += wr.ffb_cache_misses;
        ffb_collide_calls_ += wr.ffb_collide_calls;
        ffb_expand_calls_ += wr.ffb_expand_calls;
        ffb_total_steps_ += wr.ffb_total_steps;
        ffb_total_calls_ += wr.ffb_total_calls;

        // Remap box IDs to global unique IDs
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
        }

        // Transplant LECT nodes expanded by this worker
        int n_tp = lect_.transplant_subtree(pwr.lect, snapshot_base, id_map);
        total_transplanted += n_tp;

        // Merge expand profiling
        lect_.expand_profile_.merge(pwr.lect.expand_profile_);

        // Merge boxes
        for (auto& box : wr.boxes)
            boxes_.push_back(std::move(box));
    }

    result.n_promotions = total_promotions;

    fprintf(stderr, "[GRW] parallel done: %d boxes merged, %d promotions, "
            "%d nodes transplanted, %d threads\n",
            static_cast<int>(boxes_.size()), total_promotions,
            total_transplanted, n_workers);
}

}  // namespace sbf

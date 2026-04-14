// SafeBoxForest v6 — Forest Grower (Phase F + parallel)
#include <sbf/forest/grower.h>
#include <sbf/forest/adjacency.h>
#include <sbf/forest/thread_pool.h>
#include <sbf/core/union_find.h>
#include <sbf/scene/collision_checker.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <queue>
#include <unordered_map>
#include <sbf/core/log.h>

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

void ForestGrower::set_multi_goals(const std::vector<Eigen::VectorXd>& goals) {
    multi_goals_ = goals;
    has_multi_goals_ = !goals.empty();
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

bool ForestGrower::global_budget_reached() const {
    if (!shared_box_count_) return false;
    return shared_box_count_->load(std::memory_order_relaxed) >= config_.max_boxes;
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

    // Reject seeds that already lie inside an occupied LECT region (O(depth) vs O(n)).
    if (lect_.is_point_occupied(seed)) {
        n_ffb_fail_++;
        return -1;
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

// ─── enforce_parent_adjacency ───────────────────────────────────────────────
bool ForestGrower::enforce_parent_adjacency(int parent_id, int face_dim,
                                            int face_side,
                                            const Obstacle* obs, int n_obs) {
    auto& new_box = boxes_.back();
    // Find parent — reverse scan (parent is usually near the end)
    const BoxNode* parent_ptr = nullptr;
    for (int i = (int)boxes_.size() - 2; i >= 0; --i) {
        if (boxes_[i].id == parent_id) { parent_ptr = &boxes_[i]; break; }
    }
    if (!parent_ptr) return false;
    if (shared_face(new_box, *parent_ptr).has_value()) return true;

    // Auto-detect face_dim for bridge boxes (face_dim == -1):
    // Find the dimension with the smallest gap between new_box and parent.
    if (face_dim < 0) {
        const int nd = new_box.n_dims();
        double best_gap = 1e18;
        for (int d = 0; d < nd; ++d) {
            // gap from new_box.lo to parent.hi
            double g1 = new_box.joint_intervals[d].lo - parent_ptr->joint_intervals[d].hi;
            if (g1 > 0 && g1 < best_gap) {
                best_gap = g1; face_dim = d; face_side = 1;
            }
            // gap from parent.lo to new_box.hi
            double g2 = parent_ptr->joint_intervals[d].lo - new_box.joint_intervals[d].hi;
            if (g2 > 0 && g2 < best_gap) {
                best_gap = g2; face_dim = d; face_side = 0;
            }
        }
        if (face_dim < 0) return false;  // boxes overlap or are inside each other
    }

    // Extend new box boundary to touch parent face
    // Measure gap size — only extend if small (< 0.05 rad)
    double gap = 0.0;
    if (face_side == 1) {
        double face_val = parent_ptr->joint_intervals[face_dim].hi;
        gap = new_box.joint_intervals[face_dim].lo - face_val;
        if (gap > 0.0 && gap < 0.05)
            new_box.joint_intervals[face_dim].lo = face_val;
    } else {
        double face_val = parent_ptr->joint_intervals[face_dim].lo;
        gap = face_val - new_box.joint_intervals[face_dim].hi;
        if (gap > 0.0 && gap < 0.05)
            new_box.joint_intervals[face_dim].hi = face_val;
    }

    // For larger gaps, do full collision check before extending
    if (gap >= 0.05) {
        auto orig_intervals = new_box.joint_intervals;
        if (face_side == 1) {
            double face_val = parent_ptr->joint_intervals[face_dim].hi;
            if (new_box.joint_intervals[face_dim].lo > face_val)
                new_box.joint_intervals[face_dim].lo = face_val;
        } else {
            double face_val = parent_ptr->joint_intervals[face_dim].lo;
            if (new_box.joint_intervals[face_dim].hi < face_val)
                new_box.joint_intervals[face_dim].hi = face_val;
        }
        CollisionChecker ext_checker(robot_, {});
        ext_checker.set_obstacles(obs, n_obs);
        if (ext_checker.check_box(new_box.joint_intervals)) {
            new_box.joint_intervals = orig_intervals;
            new_box.compute_volume();
            return false;
        }
    }

    new_box.compute_volume();
    return shared_face(new_box, *parent_ptr).has_value();
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
    FFBConfig saved_ffb = config_.ffb_config;

    // Multi-goal roots: create a root at each goal point
    if (has_multi_goals_) {
        config_.ffb_config.max_depth = std::max(saved_ffb.max_depth, 60);

        for (int i = 0; i < static_cast<int>(multi_goals_.size()); i++) {
            int id = try_create_box(multi_goals_[i], obs, n_obs, -1, -1, -1, i);
            SBF_INFO("[GRW] multi-goal root %d: id=%d", i, id);
        }
        SBF_INFO("[GRW] multi-goal roots: %d/%d created", (int)boxes_.size(), (int)multi_goals_.size());
        config_.ffb_config = saved_ffb;
        return;
    }

    if (has_endpoints_) {
        // For start/goal roots, use deeper max_depth
        // to maximise chance of certifying a free box at exact s/t positions.
        config_.ffb_config.max_depth = std::max(saved_ffb.max_depth, 60);

        int id0 = try_create_box(start_, obs, n_obs, -1, -1, -1, 0);
        int id1 = try_create_box(goal_, obs, n_obs, -1, -1, -1, 1);
        SBF_INFO("[GRW] roots: id0=%d id1=%d boxes=%d", id0, id1, (int)boxes_.size());
    }

    // Diversity roots: coarse settings to keep fast.
    config_.ffb_config.max_depth = saved_ffb.max_depth;

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

    // P4: Flat center cache — avoids heap-allocating Eigen::VectorXd per box per
    //     iteration.  center_cache[i*nd + d] = center of box i in dimension d.
    std::vector<double> center_cache;
    center_cache.reserve(config_.max_boxes * nd);
    for (const auto& b : boxes_) {
        for (int d = 0; d < nd; ++d)
            center_cache.push_back(b.joint_intervals[d].center());
    }

    // Temp buffer for q_rand as raw pointer (avoid Eigen per-element overhead)
    std::vector<double> q_buf(nd);

    while (static_cast<int>(boxes_.size()) < config_.max_boxes &&
           miss_count < config_.max_consecutive_miss &&
           !deadline_reached() &&
           !global_budget_reached()) {

        // 1. Sample
        Eigen::VectorXd q_rand;
        int goal_tree_id = -1;  // for multi-goal: which tree the goal belongs to

        if (has_multi_goals_ && u01(rng_) < config_.rrt_goal_bias) {
            // Multi-goal bias: pick a random goal
            int gi = std::uniform_int_distribution<int>(
                0, static_cast<int>(multi_goals_.size()) - 1)(rng_);
            q_rand = multi_goals_[gi];
            goal_tree_id = gi;
        } else if (has_endpoints_ && u01(rng_) < config_.rrt_goal_bias) {
            q_rand = (u01(rng_) < 0.5) ? goal_ : start_;
        } else {
            q_rand = sample_random();
        }

        // 2. Find nearest box using flat center cache (P4: zero heap alloc)
        if (boxes_.empty()) { miss_count++; continue; }
        // Copy q_rand into raw buffer for fast inner loop
        for (int d = 0; d < nd; ++d) q_buf[d] = q_rand[d];

        double best_dist = std::numeric_limits<double>::max();
        int best_idx = -1;
        const int n_boxes = static_cast<int>(boxes_.size());
        {
            const double* cc = center_cache.data();
            const double* qp = q_buf.data();
            for (int i = 0; i < n_boxes; ++i) {
                if (goal_tree_id >= 0 && boxes_[i].root_id == goal_tree_id) {
                    cc += nd; continue;
                }
                double d = 0.0;
                for (int k = 0; k < nd; ++k) {
                    double dk = cc[k] - qp[k];
                    d += dk * dk;
                }
                if (d < best_dist) { best_dist = d; best_idx = i; }
                cc += nd;
            }
        }
        // Fallback: if all boxes belong to goal's tree, use any box
        if (best_idx < 0) {
            const double* cc = center_cache.data();
            const double* qp = q_buf.data();
            for (int i = 0; i < n_boxes; ++i) {
                double d = 0.0;
                for (int k = 0; k < nd; ++k) {
                    double dk = cc[k] - qp[k];
                    d += dk * dk;
                }
                if (d < best_dist) { best_dist = d; best_idx = i; }
                cc += nd;
            }
        }
        if (best_idx < 0) { miss_count++; continue; }

        // 3. Direction + snap
        Eigen::VectorXd direction = q_rand - boxes_[best_idx].center();
        double dir_norm = direction.norm();
        if (dir_norm < 1e-12) { miss_count++; continue; }
        direction /= dir_norm;

        auto snap = snap_to_face(boxes_[best_idx], direction);

        // 4. Create box
        int parent_id = boxes_[best_idx].id;
        int parent_idx = best_idx;
        int bid = try_create_box(
            snap.seed, obs, n_obs,
            parent_id, snap.face_dim, snap.face_side,
            boxes_[best_idx].root_id);

        if (bid >= 0) {
            miss_count = 0;
            // 5. Enforce adjacency with parent box
            enforce_parent_adjacency(parent_id, snap.face_dim, snap.face_side,
                                     obs, n_obs);
            // P4: Update center cache for new box (after forced adjacency finalized)
            const auto& nb = boxes_.back();
            for (int d = 0; d < nd; ++d)
                center_cache.push_back(nb.joint_intervals[d].center());
        } else {
            miss_count++;
        }
    }
}

// ─── grow_wavefront ─────────────────────────────────────────────────────────
void ForestGrower::grow_wavefront(const Obstacle* obs, int n_obs) {
    int miss_count = 0;
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    const auto& stages = config_.wavefront_stages;
    if (stages.empty()) return;

    int current_stage = 0;

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

    // ── Connect mode: track inter-tree connectivity via UnionFind ───────
    // UF operates on root_id space (one entry per tree, not per box).
    const bool cm = config_.connect_mode && has_multi_goals_;
    const int n_trees = cm ? static_cast<int>(multi_goals_.size()) : 0;
    UnionFind tree_uf(n_trees);  // tree_uf[root_id] tracks which component
    bool all_connected = false;
    int n_components = n_trees;
    auto t_wave_start = Clock::now();

    // If connect_mode, initialise connectivity from existing root boxes
    if (cm) {
        // Check if any existing boxes already touch across trees
        for (int i = 0; i < (int)boxes_.size(); i++) {
            for (int j = i + 1; j < (int)boxes_.size(); j++) {
                int ri = boxes_[i].root_id, rj = boxes_[j].root_id;
                if (ri != rj && ri < n_trees && rj < n_trees &&
                    boxes_adjacent(boxes_[i], boxes_[j])) {
                    if (tree_uf.unite(ri, rj)) {
                        n_components--;
                        SBF_INFO("[GRW-CM] initial merge tree %d <-> %d (comp=%d)", ri, rj, n_components);
                    }
                }
            }
        }
        all_connected = (n_components <= 1);
    }

    // Save original ffb config, swap in stage config
    FFBConfig saved_ffb = config_.ffb_config;

    // Lambda: check new box for cross-tree adjacency (connect_mode)
    auto check_cross_tree = [&](int new_box_idx) {
        if (!cm || all_connected) return;
        const BoxNode& nb = boxes_[new_box_idx];
        int nr = nb.root_id;
        if (nr < 0 || nr >= n_trees) return;
        for (int i = 0; i < new_box_idx; i++) {
            int ir = boxes_[i].root_id;
            if (ir == nr || ir < 0 || ir >= n_trees) continue;
            if (tree_uf.connected(nr, ir)) continue;  // already same comp
            if (boxes_adjacent(nb, boxes_[i])) {
                tree_uf.unite(nr, ir);
                n_components--;
                double elapsed = std::chrono::duration<double, std::milli>(
                    Clock::now() - t_wave_start).count();
                SBF_INFO("[GRW-CM] merge tree %d <-> %d at box %d (comp=%d, %.0fms, boxes=%d)", nr, ir, (int)boxes_.size(), n_components, elapsed, (int)boxes_.size());
                if (n_components <= 1) {
                    all_connected = true;
                    SBF_INFO("[GRW-CM] *** ALL %d TREES CONNECTED *** (%.0fms, boxes=%d)", n_trees, elapsed, (int)boxes_.size());
                    return;
                }
            }
        }
    };

    while (static_cast<int>(boxes_.size()) < config_.max_boxes &&
           miss_count < config_.max_consecutive_miss &&
           !deadline_reached() &&
           !global_budget_reached() &&
           !(cm && all_connected)) {

        // Adaptive staging
        if (current_stage + 1 < static_cast<int>(stages.size()) &&
            static_cast<int>(boxes_.size()) >= stages[current_stage].box_limit) {
            current_stage++;

            // Rebuild queue
            while (!pq.empty()) pq.pop();
            for (const auto& b : boxes_)
                pq.push({b.id, b.volume});
            miss_count = 0;
        }

        if (!pq.empty()) {
            WaveEntry entry = pq.top();
            pq.pop();

            auto it = id_to_idx.find(entry.box_id);
            if (it == id_to_idx.end()) continue;

            const BoxNode& box = boxes_[it->second];
            // Copy data needed after try_create_box (which may reallocate boxes_)
            const int box_id = box.id;
            const int box_root_id = box.root_id;

            // ── Bias target: drive boxes toward other trees ─────────────
            const Eigen::VectorXd* bias = nullptr;
            Eigen::VectorXd cross_tree_target;  // storage for multi-goal bias
            if (has_multi_goals_ && n_trees > 1) {
                // Pick nearest root from a DIFFERENT tree (or unconnected tree)
                Eigen::VectorXd bc = box.center();
                double best_d = std::numeric_limits<double>::max();
                int best_g = -1;
                for (int g = 0; g < n_trees; g++) {
                    if (g == box_root_id) continue;
                    // In connect_mode, prefer roots not yet connected to this tree
                    if (cm && tree_uf.connected(g, box_root_id)) continue;
                    double d = (multi_goals_[g] - bc).squaredNorm();
                    if (d < best_d) { best_d = d; best_g = g; }
                }
                if (best_g < 0) {
                    // All connected or only one tree — pick any other root
                    for (int g = 0; g < n_trees; g++) {
                        if (g == box_root_id) continue;
                        double d = (multi_goals_[g] - bc).squaredNorm();
                        if (d < best_d) { best_d = d; best_g = g; }
                    }
                }
                if (best_g >= 0) {
                    cross_tree_target = multi_goals_[best_g];
                    bias = &cross_tree_target;
                }
            } else if (has_endpoints_) {
                if (box_root_id == 0) bias = &goal_;
                else if (box_root_id == 1) bias = &start_;
                else bias = (u01(rng_) < 0.5) ? &start_ : &goal_;
            }

            auto bseeds = sample_boundary(box, bias);
            if (bseeds.empty()) { miss_count++; continue; }
            for (const auto& bs : bseeds) {
                if (static_cast<int>(boxes_.size()) >= config_.max_boxes ||
                    deadline_reached() || (cm && all_connected)) break;

                int bid = try_create_box(
                    bs.config, obs, n_obs,
                    box_id, bs.dim, bs.side, box_root_id);

                if (bid >= 0) {
                    int new_idx = static_cast<int>(boxes_.size()) - 1;
                    id_to_idx[bid] = new_idx;
                    pq.push({bid, boxes_.back().volume});
                    miss_count = 0;
                    check_cross_tree(new_idx);
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
                    deadline_reached() || (cm && all_connected)) break;
                int bid = try_create_box(
                    bs.config, obs, n_obs,
                    rb_id, bs.dim, bs.side, rb_root_id);
                if (bid >= 0) {
                    int new_idx = static_cast<int>(boxes_.size()) - 1;
                    id_to_idx[bid] = new_idx;
                    pq.push({bid, boxes_.back().volume});
                    miss_count = 0;
                    check_cross_tree(new_idx);
                } else {
                    miss_count++;
                }
            }
        }
    }

    config_.ffb_config = saved_ffb;

    // Store connectivity result into instance variables for grow() to pick up
    wf_all_connected_ = all_connected;
    wf_connect_time_ms_ = all_connected
        ? std::chrono::duration<double, std::milli>(Clock::now() - t_wave_start).count()
        : -1.0;
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

    if (config_.connect_mode && config_.n_threads > 1
        && has_multi_goals_ && static_cast<int>(boxes_.size()) >= 2) {
        // Coordinated parallel: master manages boxes, workers do FFB
        grow_coordinated(obs, n_obs);
        double wave_ms = std::chrono::duration<double, std::milli>(Clock::now() - t_wave).count();
        SBF_INFO("[GRW] timing: roots=%.0fms coordinated_grow=%.0fms", roots_ms, wave_ms);
    } else if (config_.n_threads > 1 && static_cast<int>(boxes_.size()) >= 2) {
        // Parallel path — each tree grows independently with its own budget
        GrowerResult par_result;
        grow_parallel(obs, n_obs, par_result);
        n_promotions = par_result.n_promotions;
        double wave_ms = std::chrono::duration<double, std::milli>(Clock::now() - t_wave).count();
        SBF_INFO("[GRW] timing: roots=%.0fms parallel_grow=%.0fms", roots_ms, wave_ms);

        // Post-parallel connect_mode: check cross-tree adjacency
        if (config_.connect_mode && has_multi_goals_) {
            const int n_trees = static_cast<int>(multi_goals_.size());
            UnionFind tree_uf(n_trees);
            int n_comp = n_trees;
            for (int i = 0; i < (int)boxes_.size() && n_comp > 1; i++) {
                for (int j = i + 1; j < (int)boxes_.size() && n_comp > 1; j++) {
                    int ri = boxes_[i].root_id, rj = boxes_[j].root_id;
                    if (ri != rj && ri >= 0 && ri < n_trees && rj >= 0 && rj < n_trees
                        && !tree_uf.connected(ri, rj)
                        && boxes_adjacent(boxes_[i], boxes_[j])) {
                        tree_uf.unite(ri, rj);
                        n_comp--;
                    }
                }
            }
            wf_all_connected_ = (n_comp <= 1);
            wf_connect_time_ms_ = wave_ms;
            // Diagnostic: print which trees are in which component
            {
                std::unordered_map<int, std::vector<int>> comp_trees;
                for (int t = 0; t < n_trees; ++t)
                    comp_trees[tree_uf.find(t)].push_back(t);
                SBF_INFO("[GRW] post-parallel connect check: %d components%s", n_comp, wf_all_connected_ ? " — ALL CONNECTED" : "");
                for (auto& [rep, members] : comp_trees) {
                    SBF_INFO("  component[%d]:", rep);
                    for (int m : members) SBF_INFO(" tree%d", m);
                    SBF_INFO("");
                }
            }
        }
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

        SBF_INFO("[GRW] timing: roots=%.0fms wave=%.0fms promo=%.0fms", roots_ms, wave_ms, promo_ms);
    }

    SBF_WARN("[GRW] final: boxes=%d success=%d fail=%d promo=%d", (int)boxes_.size(), n_ffb_success_, n_ffb_fail_, n_promotions);

    SBF_INFO("[GRW] ffb_stats: calls=%d steps=%d cache_hits=%d cache_misses=%d", ffb_total_calls_, ffb_total_steps_, ffb_cache_hits_, ffb_cache_misses_);
    SBF_INFO("[GRW] ffb_timing: total=%.1fms envelope=%.1fms collide=%.1fms expand=%.1fms intervals=%.1fms", ffb_total_ms_, ffb_envelope_ms_, ffb_collide_ms_, ffb_expand_ms_, ffb_intervals_ms_);
    if (ffb_cache_misses_ > 0)
        SBF_INFO("[GRW] ffb_avg_miss: envelope=%.3fms/call", ffb_envelope_ms_ / ffb_cache_misses_);
    if (ffb_expand_calls_ > 0)
        SBF_INFO("[GRW] ffb_avg_expand: %.3fms/call (%d calls)", ffb_expand_ms_ / ffb_expand_calls_, ffb_expand_calls_);
    {
        const auto& ep = lect_.expand_profile_;
        if (ep.expand_calls > 0) {
            SBF_INFO("[GRW] expand_profile: calls=%d new_nodes=%d pick_dim=%.1fms fk=%.1fms env=%.1fms refine=%.1fms", ep.expand_calls, ep.expand_calls * 2, ep.pick_dim_ms, ep.fk_inc_ms, ep.envelope_ms, ep.refine_ms);
            SBF_INFO("[GRW] expand_avg: pick_dim=%.3fms fk=%.3fms env=%.3fms refine=%.3fms per_call=%.3fms", ep.pick_dim_ms / ep.expand_calls, ep.fk_inc_ms / ep.expand_calls, ep.envelope_ms / ep.expand_calls, ep.refine_ms / ep.expand_calls, (ep.pick_dim_ms + ep.fk_inc_ms + ep.envelope_ms + ep.refine_ms) / ep.expand_calls);
            SBF_INFO("[GRW] expand_dim: calls=%d cache_hits=%d (%.1f%%)", ep.pick_dim_calls, ep.pick_dim_cache_hits, ep.pick_dim_calls > 0 ? 100.0 * ep.pick_dim_cache_hits / ep.pick_dim_calls : 0.0);
        }
    }
    SBF_INFO("[GRW] lect_nodes: %d", lect_.n_nodes());
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

    // Connect mode results
    result.all_connected = wf_all_connected_;
    result.connect_time_ms = wf_connect_time_ms_ >= 0 ? wf_connect_time_ms_ : 0.0;
    result.connect_n_boxes = wf_connect_boxes_ > 0 ? wf_connect_boxes_
        : (wf_all_connected_ ? static_cast<int>(boxes_.size()) : 0);

    auto t1 = Clock::now();
    result.build_time_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();

    return result;
}

// ─── grow_subtree (worker entry point) ──────────────────────────────────────

}  // namespace sbf

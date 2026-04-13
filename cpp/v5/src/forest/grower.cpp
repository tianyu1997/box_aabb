// SafeBoxForest v5 — Forest Grower (Phase F + parallel)
#include <sbf/forest/grower.h>
#include <sbf/forest/adjacency.h>
#include <sbf/forest/thread_pool.h>
#include <sbf/scene/collision_checker.h>

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
            fprintf(stderr, "[GRW] multi-goal root %d: id=%d\n", i, id);
        }
        fprintf(stderr, "[GRW] multi-goal roots: %d/%d created\n",
                (int)boxes_.size(), (int)multi_goals_.size());
        fflush(stderr);
        config_.ffb_config = saved_ffb;
        return;
    }

    if (has_endpoints_) {
        // For start/goal roots, use deeper max_depth
        // to maximise chance of certifying a free box at exact s/t positions.
        config_.ffb_config.max_depth = std::max(saved_ffb.max_depth, 60);

        int id0 = try_create_box(start_, obs, n_obs, -1, -1, -1, 0);
        int id1 = try_create_box(goal_, obs, n_obs, -1, -1, -1, 1);
        fprintf(stderr, "[GRW] roots: id0=%d id1=%d boxes=%d\n",
                id0, id1, (int)boxes_.size());
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

// ─── Inline union-find for connect_mode ─────────────────────────────────────
namespace {
struct InlineUF {
    std::vector<int> parent, rank_;
    explicit InlineUF(int n) : parent(n), rank_(n, 0) {
        std::iota(parent.begin(), parent.end(), 0);
    }
    int find(int x) {
        while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
        return x;
    }
    bool unite(int a, int b) {
        a = find(a); b = find(b);
        if (a == b) return false;
        if (rank_[a] < rank_[b]) std::swap(a, b);
        parent[b] = a;
        if (rank_[a] == rank_[b]) rank_[a]++;
        return true;
    }
    bool connected(int a, int b) { return find(a) == find(b); }
    void grow(int new_n) {
        int old = (int)parent.size();
        parent.resize(new_n);
        rank_.resize(new_n, 0);
        for (int i = old; i < new_n; i++) parent[i] = i;
    }
};
}  // anonymous namespace

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
    InlineUF tree_uf(n_trees);  // tree_uf[root_id] tracks which component
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
                        fprintf(stderr, "[GRW-CM] initial merge tree %d <-> %d (comp=%d)\n",
                                ri, rj, n_components);
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
                fprintf(stderr, "[GRW-CM] merge tree %d <-> %d at box %d (comp=%d, %.0fms, boxes=%d)\n",
                        nr, ir, (int)boxes_.size(), n_components, elapsed, (int)boxes_.size());
                if (n_components <= 1) {
                    all_connected = true;
                    fprintf(stderr, "[GRW-CM] *** ALL %d TREES CONNECTED *** (%.0fms, boxes=%d)\n",
                            n_trees, elapsed, (int)boxes_.size());
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
        fprintf(stderr, "[GRW] timing: roots=%.0fms coordinated_grow=%.0fms\n",
                roots_ms, wave_ms);
    } else if (config_.n_threads > 1 && static_cast<int>(boxes_.size()) >= 2) {
        // Parallel path — each tree grows independently with its own budget
        GrowerResult par_result;
        grow_parallel(obs, n_obs, par_result);
        n_promotions = par_result.n_promotions;
        double wave_ms = std::chrono::duration<double, std::milli>(Clock::now() - t_wave).count();
        fprintf(stderr, "[GRW] timing: roots=%.0fms parallel_grow=%.0fms\n",
                roots_ms, wave_ms);

        // Post-parallel connect_mode: check cross-tree adjacency
        if (config_.connect_mode && has_multi_goals_) {
            const int n_trees = static_cast<int>(multi_goals_.size());
            InlineUF tree_uf(n_trees);
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
                fprintf(stderr, "[GRW] post-parallel connect check: %d components%s\n",
                        n_comp, wf_all_connected_ ? " — ALL CONNECTED" : "");
                for (auto& [rep, members] : comp_trees) {
                    fprintf(stderr, "  component[%d]:", rep);
                    for (int m : members) fprintf(stderr, " tree%d", m);
                    fprintf(stderr, "\n");
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

    // For multi-goal: each tree gets an independent budget, no shared counter.
    // This eliminates the Matthew effect — small trees are no longer starved.
    bool per_tree_mode = has_multi_goals_;
    auto worker_counter = per_tree_mode
        ? std::shared_ptr<std::atomic<int>>(nullptr)
        : shared_counter;

    for (int i = 0; i < n_subtrees; ++i) {
        GrowerConfig worker_cfg = config_;
        if (per_tree_mode) {
            // Equal budget per tree
            worker_cfg.max_boxes = std::max(config_.max_boxes / n_subtrees, 50);
            // Trees in tight C-space regions need more miss tolerance
            worker_cfg.max_consecutive_miss =
                std::max(config_.max_consecutive_miss, 500);
        } else {
            worker_cfg.max_boxes = config_.max_boxes;
        }
        worker_cfg.rng_seed = config_.rng_seed +
                              static_cast<uint64_t>(i) * 12345ULL + 1;
        worker_cfg.n_threads = 1;  // no recursive parallelism

        auto warm_ptr = std::make_shared<LECT>(lect_.snapshot());
        Eigen::VectorXd seed = roots[i].seed;
        int rid = roots[i].root_id;
        bool has_ep = has_endpoints_;
        Eigen::VectorXd start_cfg = has_ep ? start_ : Eigen::VectorXd();
        Eigen::VectorXd goal_cfg = has_ep ? goal_ : Eigen::VectorXd();
        // Pass multi_goals to workers so RRT goal_bias drives toward other trees
        bool has_mg = has_multi_goals_;
        std::vector<Eigen::VectorXd> worker_multi_goals = multi_goals_;

        futures.push_back(pool.submit(
            [robot_ptr, worker_cfg, has_ep, start_cfg, goal_cfg,
             has_mg, worker_multi_goals,
             seed, rid, obs, n_obs, worker_counter, warm_ptr,
             worker_deadline]() -> ParallelWorkerResult {
                ForestGrower worker(*robot_ptr, std::move(*warm_ptr), worker_cfg);
                worker.set_deadline(worker_deadline);
                if (has_ep) worker.set_endpoints(start_cfg, goal_cfg);
                if (has_mg) worker.set_multi_goals(worker_multi_goals);
                ParallelWorkerResult pwr;
                pwr.result = worker.grow_subtree(seed, rid, obs, n_obs,
                                                 worker_counter);
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

    // Log per-tree box counts (useful for diagnosing balance)
    {
        std::unordered_map<int, int> tree_sizes;
        for (const auto& b : boxes_) tree_sizes[b.root_id]++;
        fprintf(stderr, "[GRW] tree sizes:");
        for (const auto& kv : tree_sizes)
            fprintf(stderr, " root%d=%d", kv.first, kv.second);
        fprintf(stderr, "\n");
        fflush(stderr);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// grow_coordinated — Master-worker architecture
//
// Master thread:  RRT sampling loop, nearest-box search, box acceptance,
//                 adjacency graph maintenance, connectivity tracking.
// Worker threads: FFB-only computation, each with its own LECT snapshot.
//
// Key guarantee: NO duplicate or overlapping boxes across trees because
// the master is the single point of truth for box list management.
// ═══════════════════════════════════════════════════════════════════════════

void ForestGrower::grow_coordinated(const Obstacle* obs, int n_obs) {
    const int n_workers = std::min(config_.n_threads,
                                   std::max(1, (int)std::thread::hardware_concurrency()));
    const int nd = robot_.n_joints();
    const auto& limits = robot_.joint_limits().limits;

    // ── Per-worker LECT snapshots for thread-safe FFB ───────────────────
    std::vector<std::unique_ptr<LECT>> worker_lects;
    worker_lects.reserve(n_workers);
    for (int i = 0; i < n_workers; ++i)
        worker_lects.push_back(std::make_unique<LECT>(lect_.snapshot()));

    ThreadPool pool(n_workers);

    // LECT refresh interval (every N batches, re-snapshot from master)
    const int lect_refresh_interval = 10;

    // ── Flat center cache for fast nearest-box search ───────────────────
    std::vector<double> center_cache;
    center_cache.reserve(config_.max_boxes * nd);
    for (const auto& b : boxes_)
        for (int d = 0; d < nd; ++d)
            center_cache.push_back(b.joint_intervals[d].center());

    // ── Connectivity tracking ───────────────────────────────────────────
    const bool cm = config_.connect_mode && has_multi_goals_;
    const int n_trees = has_multi_goals_ ? (int)multi_goals_.size() : 0;
    InlineUF tree_uf(std::max(n_trees, 1));
    int n_comp = n_trees;

    // Per-tree box count and index lists for fast tree-based lookups
    std::vector<int> tree_box_count(std::max(n_trees, 1), 0);
    std::vector<std::vector<int>> tree_box_indices(std::max(n_trees, 1));
    for (int i = 0; i < (int)boxes_.size(); ++i) {
        if (boxes_[i].root_id >= 0 && boxes_[i].root_id < n_trees) {
            tree_box_count[boxes_[i].root_id]++;
            tree_box_indices[boxes_[i].root_id].push_back(i);
        }
    }

    std::uniform_real_distribution<double> u01(0.0, 1.0);
    std::vector<double> q_buf(nd);

    int miss_count = 0;
    int total_batches = 0;

    // ── Task / result types ─────────────────────────────────────────────
    struct TaskInfo {
        Eigen::VectorXd seed;
        int parent_box_id;
        int face_dim, face_side;
        int root_id;
    };
    struct WorkerResult {
        bool success = false;
        BoxNode box;
        FFBResult ffb;
    };

    // Progress logging interval
    auto t_start = Clock::now();
    int last_log_boxes = (int)boxes_.size();

    // Stall detection: track when component count last changed
    int last_comp_change_boxes = (int)boxes_.size();
    int stall_threshold = 150;  // P19: lowered from 500 for faster bridge activation

    // Connectivity time tracking: record FIRST time n_comp reaches 1
    double first_connect_time_ms = -1.0;
    int first_connect_boxes = 0;

    // ── P12: Pre-compute & cache component sizes (update on component change) ──
    std::vector<int> comp_size(std::max(n_trees, 1), 0);
    if (n_trees > 0)
        for (int t = 0; t < n_trees; ++t)
            comp_size[tree_uf.find(t)]++;

    // ── P10: Adaptive step_ratio ──
    const double base_step_ratio = config_.rrt_step_ratio;

    // ── P17: Inline promotion tracking ──
    int n_inline_promotions = 0;
    int boxes_at_last_promote = 0;  // box count when last promotion ran
    const int promote_interval = 500;  // P23: lowered from 1000 for more frequent promotion

    // ── Timing accumulators for profiling ──
    double t_task_gen_ms = 0, t_ffb_wall_ms = 0, t_post_accept_ms = 0;
    double t_cross_tree_ms = 0, t_prefilter_ms = 0, t_nn_ms = 0;
    double t_promote_ms = 0;
    int n_prefilter_rejects = 0, n_postfilter_rejects = 0, n_isolated_rejects = 0;

    while ((int)boxes_.size() < config_.max_boxes &&
           miss_count < config_.max_consecutive_miss * 4 &&
           !deadline_reached()) {

        // Don't break on connectivity — continue growing for coverage
        // unless stop_after_connect is set
        if (cm && n_comp <= 1 && config_.stop_after_connect) break;

        // Detect stall (for logging and adaptive params)
        int boxes_since_comp_change = (int)boxes_.size() - last_comp_change_boxes;
        bool stalled = cm && n_comp > 1 &&
            (boxes_since_comp_change > stall_threshold);

        // P10: Adaptive step_ratio — decrease when stalled for finer exploration
        //   stall_level 0..3 → step_ratio = base, base/2, base/4, base/8
        int stall_level = stalled ? std::min(3, (boxes_since_comp_change - stall_threshold) / 3000) : 0;
        double cur_step_ratio = base_step_ratio / (1 << stall_level);

        // ── P15: Post-connectivity exploration mode ─────────────────────
        bool connected_phase = cm && n_comp <= 1;
        // P22: Wider step ratio for coverage after connectivity (3× base)
        double effective_step = connected_phase
            ? std::min(base_step_ratio * 3.0, 0.2) : cur_step_ratio;

        // ── P20: Time-aware urgency — force bridge when running out of time ──
        double elapsed_frac = config_.timeout_ms > 0
            ? std::chrono::duration<double, std::milli>(Clock::now() - t_start).count() / config_.timeout_ms
            : 0.0;
        bool urgent = cm && n_comp > 1 && elapsed_frac > 0.4;

        // ── Generate a batch of FFB tasks (master, single-threaded) ─────
        auto t_gen0 = Clock::now();
        int effective_batch = (config_.batch_size > 0)
            ? std::min(config_.batch_size, n_workers) : n_workers;
        int batch_cap = std::min(effective_batch,
                                 config_.max_boxes - (int)boxes_.size());
        std::vector<TaskInfo> tasks;
        tasks.reserve(batch_cap);

        // Try up to 3x candidates to fill the batch (some get pre-filtered)
        for (int attempt = 0; attempt < batch_cap * 3 &&
                              (int)tasks.size() < batch_cap; ++attempt) {
            // Connection-driven scheduling: prefer trees in smaller components
            int tree_id = 0;
            if (n_trees > 0) {
                if (u01(rng_) < 0.7) {
                    // P12: Use cached comp_size (no recomputation per attempt)
                    int best_comp = comp_size[tree_uf.find(0)];
                    int best_cnt = tree_box_count[0];
                    for (int t = 1; t < n_trees; ++t) {
                        int cs = comp_size[tree_uf.find(t)];
                        if (cs < best_comp ||
                            (cs == best_comp && tree_box_count[t] < best_cnt)) {
                            best_comp = cs;
                            best_cnt = tree_box_count[t];
                            tree_id = t;
                        }
                    }
                } else {
                    tree_id = std::uniform_int_distribution<int>(
                        0, n_trees - 1)(rng_);
                }
            }

            // P11: Stall-aware sampling strategy
            // Normal: goal_bias toward unconnected trees
            // Stalled: midpoint bridging — sample along interpolation line
            //   between disconnected tree frontiers (much more effective than
            //   random frontier perturbation)
            Eigen::VectorXd q_rand;
            double gb = config_.rrt_goal_bias;

            // P20: urgent mode → 95% bridge; stalled → 90% bridge
            double bridge_prob = urgent ? 0.95 : 0.90;
            bool use_bridge = (stalled || urgent) && cm && n_comp > 1 && u01(rng_) < bridge_prob;
            if (use_bridge) {
                // P14: Midpoint bridging — sample along line between closest
                // boxes of two disconnected components
                std::vector<int> unconnected;
                for (int t = 0; t < n_trees; ++t)
                    if (t != tree_id && !tree_uf.connected(t, tree_id))
                        unconnected.push_back(t);
                if (!unconnected.empty()) {
                    int target_tree = unconnected[std::uniform_int_distribution<int>(
                        0, (int)unconnected.size() - 1)(rng_)];

                    // Find a frontier box from source tree (nearest to target)
                    Eigen::VectorXd target_center = multi_goals_[target_tree];
                    if (!tree_box_indices[target_tree].empty()) {
                        int tbi_rand = tree_box_indices[target_tree][
                            std::uniform_int_distribution<int>(
                                0, (int)tree_box_indices[target_tree].size() - 1)(rng_)];
                        target_center = boxes_[tbi_rand].center();
                    }

                    // Source: pick box from this tree closest to target
                    int src_idx = -1;
                    if (!tree_box_indices[tree_id].empty()) {
                        double bd = std::numeric_limits<double>::max();
                        const auto& stbi = tree_box_indices[tree_id];
                        int scan_n = std::min((int)stbi.size(), 64);
                        for (int si = 0; si < scan_n; ++si) {
                            int bi = (scan_n < (int)stbi.size())
                                ? stbi[std::uniform_int_distribution<int>(0, (int)stbi.size()-1)(rng_)]
                                : stbi[si];
                            double d2 = (boxes_[bi].center() - target_center).squaredNorm();
                            if (d2 < bd) { bd = d2; src_idx = bi; }
                        }
                    }

                    if (src_idx >= 0) {
                        Eigen::VectorXd src_center = boxes_[src_idx].center();
                        // Sample a random point along the line src→target
                        // Bias toward the middle and target end
                        double alpha = 0.3 + u01(rng_) * 0.7;  // [0.3, 1.0]
                        q_rand = src_center + alpha * (target_center - src_center);
                        // Add small random perturbation
                        for (int d = 0; d < nd; ++d)
                            q_rand[d] += std::normal_distribution<double>(0.0, 0.02)(rng_);
                        q_rand = clamp_to_limits(q_rand);
                    } else {
                        q_rand = sample_random();
                    }
                } else {
                    q_rand = sample_random();
                }
            // P21: After connectivity, use pure random for max coverage spread
            } else if (!connected_phase && has_multi_goals_ && u01(rng_) < gb) {
                // Collect trees not yet connected to tree_id
                std::vector<int> unconnected;
                for (int t = 0; t < n_trees; ++t)
                    if (t != tree_id && !tree_uf.connected(t, tree_id))
                        unconnected.push_back(t);
                if (!unconnected.empty()) {
                    int gi = unconnected[std::uniform_int_distribution<int>(
                        0, (int)unconnected.size() - 1)(rng_)];
                    // Sample a random box center from target tree (not just root)
                    const auto& tbi = tree_box_indices[gi];
                    if (!tbi.empty()) {
                        int bi = tbi[std::uniform_int_distribution<int>(
                            0, (int)tbi.size() - 1)(rng_)];
                        q_rand = boxes_[bi].center();
                    } else {
                        q_rand = multi_goals_[gi];
                    }
                } else {
                    q_rand = sample_random();  // all connected, fall back
                }
            } else {
                q_rand = sample_random();
            }

            // P13/P15: Find nearest box
            // Connected phase: search ALL boxes across all trees (better coverage)
            // Pre-connectivity: search within selected tree only
            for (int d = 0; d < nd; ++d) q_buf[d] = q_rand[d];
            double best_dist = std::numeric_limits<double>::max();
            int best_idx = -1;
            {
                const double* base_cc = center_cache.data();
                const double* qp = q_buf.data();
                constexpr int K_SUBSAMPLE = 64;
                constexpr int K_SUB_ALL = 128;

                if (connected_phase) {
                    // P15: Search all boxes for truly nearest
                    const int total = (int)boxes_.size();
                    if (total <= K_SUB_ALL) {
                        for (int bi = 0; bi < total; ++bi) {
                            const double* cc = base_cc + bi * nd;
                            double d2 = 0.0;
                            for (int k = 0; k < nd; ++k) {
                                double dk = cc[k] - qp[k];
                                d2 += dk * dk;
                            }
                            if (d2 < best_dist) { best_dist = d2; best_idx = bi; }
                        }
                    } else {
                        for (int s = 0; s < K_SUB_ALL; ++s) {
                            int bi = std::uniform_int_distribution<int>(0, total - 1)(rng_);
                            const double* cc = base_cc + bi * nd;
                            double d2 = 0.0;
                            for (int k = 0; k < nd; ++k) {
                                double dk = cc[k] - qp[k];
                                d2 += dk * dk;
                            }
                            if (d2 < best_dist) { best_dist = d2; best_idx = bi; }
                        }
                    }
                } else {
                    // Pre-connectivity: search within selected tree
                    const auto& tbi = tree_box_indices[tree_id];
                    const int tsize = (int)tbi.size();
                    if (tsize <= K_SUBSAMPLE) {
                        for (int bi : tbi) {
                            const double* cc = base_cc + bi * nd;
                            double d2 = 0.0;
                            for (int k = 0; k < nd; ++k) {
                                double dk = cc[k] - qp[k];
                                d2 += dk * dk;
                            }
                            if (d2 < best_dist) { best_dist = d2; best_idx = bi; }
                        }
                    } else {
                        for (int s = 0; s < K_SUBSAMPLE; ++s) {
                            int ri = std::uniform_int_distribution<int>(0, tsize - 1)(rng_);
                            int bi = tbi[ri];
                            const double* cc = base_cc + bi * nd;
                            double d2 = 0.0;
                            for (int k = 0; k < nd; ++k) {
                                double dk = cc[k] - qp[k];
                                d2 += dk * dk;
                            }
                            if (d2 < best_dist) { best_dist = d2; best_idx = bi; }
                        }
                    }
                }
            }
            if (best_idx < 0) continue;

            // P14: Bridge seeds bypass snap_to_face — use directly as FFB seed
            Eigen::VectorXd seed_for_ffb;
            int parent_id_for_task;
            int face_dim_for_task = -1, face_side_for_task = -1;

            if (use_bridge) {
                // Direct seed from interpolation line
                seed_for_ffb = q_rand;
                parent_id_for_task = boxes_[best_idx].id;
            } else {
                // Normal: snap to face of nearest box
                double saved_step_ratio = config_.rrt_step_ratio;
                config_.rrt_step_ratio = effective_step;

                Eigen::VectorXd dir = q_rand - boxes_[best_idx].center();
                double dir_norm = dir.norm();
                if (dir_norm < 1e-12) { config_.rrt_step_ratio = saved_step_ratio; continue; }
                dir /= dir_norm;
                auto snap = snap_to_face(boxes_[best_idx], dir);

                config_.rrt_step_ratio = saved_step_ratio;

                seed_for_ffb = snap.seed;
                parent_id_for_task = boxes_[best_idx].id;
                face_dim_for_task = snap.face_dim;
                face_side_for_task = snap.face_side;
            }

            // Pre-filter: reject if seed inside any existing box (O(n))
            {
                auto t_pf0 = Clock::now();
                bool inside = false;
                for (const auto& b : boxes_) {
                    if (b.contains(seed_for_ffb)) { inside = true; break; }
                }
                t_prefilter_ms += std::chrono::duration<double, std::milli>(Clock::now() - t_pf0).count();
                if (inside) { n_prefilter_rejects++; continue; }
            }

            tasks.push_back({seed_for_ffb, parent_id_for_task,
                             face_dim_for_task, face_side_for_task,
                             boxes_[best_idx].root_id});
        }

        if (tasks.empty()) {
            miss_count++;
            continue;
        }

        t_task_gen_ms += std::chrono::duration<double, std::milli>(Clock::now() - t_gen0).count();

        // ── Dispatch FFB to workers ─────────────────────────────────────
        // Task i → worker_lects[i].  batch ≤ n_workers, so each LECT is
        // accessed by at most one concurrent task (no data race).
        std::vector<std::future<WorkerResult>> futures;
        futures.reserve(tasks.size());

        for (int ti = 0; ti < (int)tasks.size(); ++ti) {
            LECT* lp = worker_lects[ti % n_workers].get();
            auto seed = tasks[ti].seed;
            int parent_id = tasks[ti].parent_box_id;
            int root_id = tasks[ti].root_id;
            FFBConfig fcfg = config_.ffb_config;
            // O3: Adaptive FFB depth — shallower after connectivity for speed
            if (connected_phase && fcfg.max_depth > 100)
                fcfg.max_depth = 100;

            futures.push_back(pool.submit(
                [lp, seed, parent_id, root_id,
                 obs, n_obs, fcfg]() -> WorkerResult {
                    WorkerResult wr;
                    wr.ffb = find_free_box(*lp, seed, obs, n_obs, fcfg);
                    if (!wr.ffb.success() || lp->is_occupied(wr.ffb.node_idx))
                        return wr;
                    wr.box.joint_intervals = lp->node_intervals(wr.ffb.node_idx);
                    wr.box.seed_config = seed;
                    wr.box.tree_id = wr.ffb.node_idx;
                    wr.box.parent_box_id = parent_id;
                    wr.box.root_id = root_id;
                    wr.box.compute_volume();
                    wr.success = true;
                    // Mark occupied in worker LECT to prevent self-reuse
                    lp->mark_occupied(wr.ffb.node_idx, 0);
                    return wr;
                }
            ));
        }

        // ── Collect results (master accepts/rejects) ────────────────────
        // First, wait for all futures to complete (FFB wall time)
        std::vector<WorkerResult> results(futures.size());
        {
            auto t_wait0 = Clock::now();
            for (int fi = 0; fi < (int)futures.size(); ++fi)
                results[fi] = futures[fi].get();
            t_ffb_wall_ms += std::chrono::duration<double, std::milli>(Clock::now() - t_wait0).count();
        }
        auto t_post0 = Clock::now();
        int batch_success = 0;
        int batch_start_idx = (int)boxes_.size();
        for (int fi = 0; fi < (int)results.size(); ++fi) {
            auto& wr = results[fi];

            // Accumulate FFB statistics
            ffb_total_calls_++;
            ffb_total_ms_      += wr.ffb.total_ms;
            ffb_envelope_ms_   += wr.ffb.envelope_ms;
            ffb_collide_ms_    += wr.ffb.collide_ms;
            ffb_expand_ms_     += wr.ffb.expand_ms;
            ffb_intervals_ms_  += wr.ffb.intervals_ms;
            ffb_cache_hits_    += wr.ffb.n_cache_hits;
            ffb_cache_misses_  += wr.ffb.n_cache_misses;
            ffb_collide_calls_ += wr.ffb.n_collide_calls;
            ffb_expand_calls_  += wr.ffb.n_expand_calls;
            ffb_total_steps_   += wr.ffb.n_steps;

            if (!wr.success) { n_ffb_fail_++; continue; }

            // Master-side validation: reject if seed inside any existing box (O(n))
            {
                bool reject = false;
                for (const auto& b : boxes_) {
                    if (b.contains(wr.box.seed_config)) { reject = true; break; }
                }
                if (reject) { n_ffb_fail_++; n_postfilter_rejects++; continue; }
            }

            // Accept box
            wr.box.id = next_box_id_++;
            boxes_.push_back(std::move(wr.box));
            n_ffb_success_++;
            batch_success++;

            // Update center cache
            const auto& nb = boxes_.back();
            for (int d = 0; d < nd; ++d)
                center_cache.push_back(nb.joint_intervals[d].center());

            // Update tree box count and index
            if (nb.root_id >= 0 && nb.root_id < n_trees) {
                tree_box_count[nb.root_id]++;
                tree_box_indices[nb.root_id].push_back((int)boxes_.size() - 1);
            }

            // Enforce parent adjacency (serial, master-side)
            bool adj_ok = enforce_parent_adjacency(tasks[fi].parent_box_id,
                                     tasks[fi].face_dim, tasks[fi].face_side,
                                     obs, n_obs);

            // Cross-tree adjacency for connect_mode
            // P9: Only check boxes from OTHER trees (skip same-tree)
            bool cross_tree_touch = false;
            if (cm && n_comp > 1) {
                auto t_ct0 = Clock::now();
                int old_comp = n_comp;
                int new_idx = (int)boxes_.size() - 1;
                int rj = boxes_[new_idx].root_id;
                if (rj >= 0 && rj < n_trees) {
                    for (int ti = 0; ti < n_trees && n_comp > 1; ++ti) {
                        if (ti == rj || tree_uf.connected(ti, rj)) continue;
                        for (int ei : tree_box_indices[ti]) {
                            if (boxes_adjacent(boxes_[ei], boxes_[new_idx])) {
                                tree_uf.unite(ti, rj);
                                n_comp--;
                                cross_tree_touch = true;
                                // P12: Update cached comp_size
                                std::fill(comp_size.begin(), comp_size.end(), 0);
                                for (int t = 0; t < n_trees; ++t)
                                    comp_size[tree_uf.find(t)]++;
                                break;
                            }
                        }
                    }
                }
                if (n_comp < old_comp) {
                    last_comp_change_boxes = (int)boxes_.size();
                    // Record first connectivity time
                    if (n_comp <= 1 && first_connect_time_ms < 0) {
                        first_connect_time_ms = std::chrono::duration<double>(
                            Clock::now() - t_start).count() * 1000.0;
                        first_connect_boxes = (int)boxes_.size();
                    }
                }
                t_cross_tree_ms += std::chrono::duration<double, std::milli>(Clock::now() - t_ct0).count();
            }

            // Reject isolated boxes: if enforce_parent_adjacency failed
            // AND the box didn't contribute to cross-tree connectivity,
            // check if it touches any existing box.  Isolated boxes create
            // orphan islands in the adjacency graph.
            if (!adj_ok && !cross_tree_touch) {
                int new_idx = (int)boxes_.size() - 1;
                bool touches_any = false;
                // Check parent directly
                for (int pi = new_idx - 1; pi >= 0; --pi) {
                    if (boxes_[pi].id == tasks[fi].parent_box_id) {
                        touches_any = boxes_adjacent(boxes_[pi], boxes_[new_idx]);
                        break;
                    }
                }
                // If not touching parent, scan nearby boxes (adaptive window)
                if (!touches_any) {
                    int window = std::min(new_idx / 5 + 100, 1000);
                    int scan_start = std::max(0, new_idx - window);
                    for (int si = new_idx - 1; si >= scan_start; --si) {
                        if (boxes_adjacent(boxes_[si], boxes_[new_idx])) {
                            touches_any = true;
                            break;
                        }
                    }
                }
                if (!touches_any) {
                    // Reject isolated box
                    n_isolated_rejects++;
                    boxes_.pop_back();
                    n_ffb_fail_++;
                    batch_success--;
                    // Remove from center cache
                    for (int d = 0; d < nd; ++d)
                        center_cache.pop_back();
                    // Remove from tree indices
                    int rj_rej = tasks[fi].root_id;
                    if (rj_rej >= 0 && rj_rej < n_trees) {
                        tree_box_count[rj_rej]--;
                        if (!tree_box_indices[rj_rej].empty())
                            tree_box_indices[rj_rej].pop_back();
                    }
                    continue;
                }
            }
        }
        t_post_accept_ms += std::chrono::duration<double, std::milli>(Clock::now() - t_post0).count();

        if (batch_success > 0) miss_count = 0;
        else miss_count += (int)tasks.size();

        total_batches++;

        // Periodic LECT refresh for workers
        if (total_batches % lect_refresh_interval == 0) {
            for (int i = 0; i < n_workers; ++i)
                *worker_lects[i] = lect_.snapshot();
        }

        // ── P17: Periodic inline promotion after connectivity ───────────
        //  Merge sibling LECT leaves into parent → fewer, larger boxes.
        //  Runs every promote_interval new boxes after first connectivity.
        if (connected_phase && config_.enable_promotion &&
            (int)boxes_.size() - boxes_at_last_promote >= promote_interval) {
            int before = (int)boxes_.size();
            int np = promote_all(obs, n_obs);
            n_inline_promotions += np;
            boxes_at_last_promote = (int)boxes_.size();

            if (np > 0) {
                // Rebuild center cache (promote modifies boxes_)
                center_cache.clear();
                center_cache.reserve(boxes_.size() * nd);
                for (const auto& b : boxes_)
                    for (int d = 0; d < nd; ++d)
                        center_cache.push_back(b.joint_intervals[d].center());

                // Rebuild tree indices
                for (auto& v : tree_box_indices) v.clear();
                std::fill(tree_box_count.begin(), tree_box_count.end(), 0);
                for (int i = 0; i < (int)boxes_.size(); ++i) {
                    if (boxes_[i].root_id >= 0 && boxes_[i].root_id < n_trees) {
                        tree_box_count[boxes_[i].root_id]++;
                        tree_box_indices[boxes_[i].root_id].push_back(i);
                    }
                }

                // Refresh worker LECTs
                for (int i = 0; i < n_workers; ++i)
                    *worker_lects[i] = lect_.snapshot();

                fprintf(stderr, "[GRW] inline promote: %d→%d boxes (%d merges)\n",
                        before, (int)boxes_.size(), np);
            }
        }

        // Periodic progress log
        int cur_boxes = (int)boxes_.size();
        if (cur_boxes - last_log_boxes >= 500) {
            double elapsed = std::chrono::duration<double>(
                Clock::now() - t_start).count();
            fprintf(stderr, "[GRW] coordinated progress: %d boxes, %.1fs",
                    cur_boxes, elapsed);
            if (cm) fprintf(stderr, ", %d components", n_comp);
            if (connected_phase) fprintf(stderr, " [COV sr=%.4f]", effective_step);
            else if (stalled) fprintf(stderr, " [STALL lv%d sr=%.4f]", stall_level, cur_step_ratio);
            fprintf(stderr, "\n");
            last_log_boxes = cur_boxes;
        }
    }

    // ── Merge worker LECT expand profiles ───────────────────────────────
    for (auto& wl : worker_lects)
        lect_.expand_profile_.merge(wl->expand_profile_);

    // ── Report results ──────────────────────────────────────────────────
    double elapsed = std::chrono::duration<double>(Clock::now() - t_start).count();
    fprintf(stderr, "[GRW] coordinated done: %d boxes, %d batches, %.1fs",
            (int)boxes_.size(), total_batches, elapsed);
    if (n_inline_promotions > 0)
        fprintf(stderr, ", %d inline promotions", n_inline_promotions);
    fprintf(stderr, "\n");

    // ── Profiling breakdown ─────────────────────────────────────────────
    fprintf(stderr, "[GRW] profile: task_gen=%.0fms ffb_wall=%.0fms post_accept=%.0fms\n",
            t_task_gen_ms, t_ffb_wall_ms, t_post_accept_ms);
    fprintf(stderr, "[GRW] profile:   prefilter=%.0fms(rej=%d) cross_tree=%.0fms\n",
            t_prefilter_ms, n_prefilter_rejects, t_cross_tree_ms);
    fprintf(stderr, "[GRW] profile:   post_rej=%d isolated_rej=%d\n",
            n_postfilter_rejects, n_isolated_rejects);

    if (cm) {
        wf_all_connected_ = (n_comp <= 1);
        // Use first connectivity time if achieved, otherwise total elapsed
        wf_connect_time_ms_ = (first_connect_time_ms >= 0)
            ? first_connect_time_ms : elapsed * 1000.0;
        wf_connect_boxes_ = first_connect_boxes;
        std::unordered_map<int, std::vector<int>> comp_trees;
        for (int t = 0; t < n_trees; ++t)
            comp_trees[tree_uf.find(t)].push_back(t);
        fprintf(stderr, "[GRW] connect: %d components%s",
                n_comp, wf_all_connected_ ? " — ALL CONNECTED" : "");
        if (first_connect_time_ms >= 0)
            fprintf(stderr, " (first at %.0fms, %d boxes)",
                    first_connect_time_ms, first_connect_boxes);
        fprintf(stderr, "\n");
        for (auto& [rep, members] : comp_trees) {
            fprintf(stderr, "  component[%d]:", rep);
            for (int m : members) fprintf(stderr, " tree%d", m);
            fprintf(stderr, "\n");
        }
    }
    {
        std::unordered_map<int, int> tree_sizes;
        for (const auto& b : boxes_) tree_sizes[b.root_id]++;
        fprintf(stderr, "[GRW] tree sizes:");
        for (const auto& kv : tree_sizes)
            fprintf(stderr, " root%d=%d", kv.first, kv.second);
        fprintf(stderr, "\n");
        fflush(stderr);
    }
}

}  // namespace sbf

// SafeBoxForest v6 — Forest Grower (Phase F + parallel)
#include <sbf/forest/grower.h>
#include <sbf/forest/adjacency.h>
#include <sbf/forest/connectivity.h>
#include <sbf/forest/thread_pool.h>
#include <sbf/core/union_find.h>
#include <sbf/scene/collision_checker.h>
#include <sbf/voxel/hull_rasteriser.h>

#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <unordered_map>
#include <sbf/core/log.h>
#include <sbf/core/log_format.h>

namespace sbf {

namespace {

// ─── C-space distance with revolute (S^1) wrap-around ──────────────────────
// For a joint whose limits span ≥ 2π (within tol), 0 and 2π are the same
// configuration, so the shortest signed displacement uses atan2-style wrap.
// Otherwise the joint is treated as Euclidean (linear or bounded revolute).

inline std::vector<uint8_t> compute_wrap_mask(
    const std::vector<Interval>& limits)
{
    std::vector<uint8_t> mask(limits.size(), 0);
    constexpr double kFullCircleTol = 1e-6;
    for (size_t d = 0; d < limits.size(); ++d) {
        if (limits[d].width() >= 2.0 * M_PI - kFullCircleTol) {
            mask[d] = 1;
        }
    }
    return mask;
}

inline double wrap_signed_diff(double a, double b) {
    // Returns the shortest signed delta a - b in (-π, π].
    double d = a - b;
    constexpr double kTwoPi = 2.0 * M_PI;
    d = std::fmod(d + M_PI, kTwoPi);
    if (d <= 0.0) d += kTwoPi;
    return d - M_PI;
}

inline Eigen::VectorXd cspace_diff(const Eigen::VectorXd& a,
                                   const Eigen::VectorXd& b,
                                   const std::vector<uint8_t>& wrap_mask)
{
    const int nd = static_cast<int>(std::min<size_t>({
        static_cast<size_t>(a.size()),
        static_cast<size_t>(b.size()),
        wrap_mask.size()}));
    static std::atomic<int> warn_count{0};
    if ((a.size() != b.size() ||
         static_cast<size_t>(a.size()) != wrap_mask.size()) &&
        warn_count.fetch_add(1) < 5) {
        SBF_INFO("[GRW-RRT] cspace_diff size mismatch: a=%ld b=%ld mask=%zu",
                 (long)a.size(), (long)b.size(), wrap_mask.size());
    }
    Eigen::VectorXd out(nd);
    for (int d = 0; d < nd; ++d) {
        out[d] = wrap_mask[d] ? wrap_signed_diff(a[d], b[d]) : (a[d] - b[d]);
    }
    return out;
}

inline double cspace_squared_dist_flat(const double* a, const double* b,
                                       int nd,
                                       const std::vector<uint8_t>& wrap_mask)
{
    double s = 0.0;
    for (int d = 0; d < nd; ++d) {
        double dk = wrap_mask[d] ? wrap_signed_diff(a[d], b[d]) : (a[d] - b[d]);
        s += dk * dk;
    }
    return s;
}

bool point_occupied_from(LECT& lect, int start_node, const Eigen::VectorXd& q) {
    int node = (start_node >= 0) ? start_node : 0;
    while (true) {
        if (lect.is_occupied(node)) return true;
        if (lect.is_leaf(node)) return false;
        int sd = lect.get_split_dim(node);
        node = (q[sd] <= lect.split_val(node)) ? lect.left(node) : lect.right(node);
    }
}

}  // namespace


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
    if (stop_requested_ && stop_requested_->load(std::memory_order_relaxed))
        return true;
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
    // Geometric-domain restriction: when assigned to a LECT subtree,
    // sample only from that subtree's intervals so seeds land inside the
    // worker's exclusive geometric subdomain. Falls back to full joint
    // limits when no domain is set (serial / master mode).
    if (domain_root_ >= 0) {
        const auto domain_ivs = lect_.node_intervals(domain_root_);
        for (int d = 0; d < nd; ++d) {
            const double lo = std::max(limits[d].lo, domain_ivs[d].lo);
            const double hi = std::min(limits[d].hi, domain_ivs[d].hi);
            q[d] = lo + u01(rng_) * std::max(0.0, hi - lo);
        }
        return q;
    }
    for (int d = 0; d < nd; ++d)
        q[d] = limits[d].lo + u01(rng_) * limits[d].width();
    return q;
}

Eigen::VectorXd ForestGrower::sample_unexplored() const {
    const auto& limits = robot_.joint_limits().limits;
    const int nd = static_cast<int>(limits.size());
    std::uniform_real_distribution<double> u01(0.0, 1.0);

    // Walk-down start: domain_root_ when assigned (parallel worker), else 0.
    const int start = (domain_root_ >= 0) ? domain_root_ : 0;
    if (start < 0 || lect_.subtree_free_volume(start) <= 0.0)
        return sample_random();

    int node = start;
    while (!lect_.is_leaf(node)) {
        const int l = lect_.left(node);
        const int r = lect_.right(node);
        const double fl = (l >= 0) ? lect_.subtree_free_volume(l) : 0.0;
        const double fr = (r >= 0) ? lect_.subtree_free_volume(r) : 0.0;
        const double sum = fl + fr;
        if (sum <= 0.0) break;
        const double pick = u01(rng_) * sum;
        node = (pick < fl) ? l : r;
    }

    const auto ivs = lect_.node_intervals(node);
    Eigen::VectorXd q(nd);
    for (int d = 0; d < nd; ++d) {
        const double lo = std::max(limits[d].lo, ivs[d].lo);
        const double hi = std::min(limits[d].hi, ivs[d].hi);
        q[d] = (hi > lo) ? lo + u01(rng_) * (hi - lo) : lo;
    }
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

    // Geometric domain restriction (parallel workers): clamp the seed
    // strictly inside the worker's assigned LECT subdomain BEFORE FFB
    // descent. Seeds that land exactly on a split plane (e.g. sample_random
    // hitting domain.lo, or sample_boundary boxes that touch the domain
    // boundary) would otherwise be routed by `q < split_val` into a sibling
    // subtree. Use a margin proportional to interval width so the inset is
    // larger than any FP roundoff in node_intervals/split_val.
    Eigen::VectorXd q = seed;
    if (domain_root_ >= 0) {
        const auto domain_ivs = lect_.node_intervals(domain_root_);
        for (int d = 0; d < q.size(); ++d) {
            const double w = domain_ivs[d].hi - domain_ivs[d].lo;
            const double margin = std::max(1e-9, 1e-6 * w);
            const double lo = domain_ivs[d].lo + margin;
            const double hi = domain_ivs[d].hi - margin;
            if (lo <= hi) q[d] = std::clamp(q[d], lo, hi);
            else          q[d] = 0.5 * (domain_ivs[d].lo + domain_ivs[d].hi);
        }
    }

    // Reject seeds that already lie inside an occupied LECT region in serial
    // and snapshot modes. In shared partition mode we let FFB descend through
    // occupied ancestors so workers can continue growing from pre-existing
    // root boxes without recreating them.
    if (!config_.enable_partitioned_lect_parallel &&
        point_occupied_from(lect_, domain_root_, q)) {
        SBF_TRACE("[GRW-FFB] reject(occupied) tid=%d seed=%s parent=%d root=%d",
                  worker_tid_, fmt_vec(q).c_str(), parent_box_id, root_id);
        n_ffb_fail_++;
        return -1;
    }

        // Reject seeds whose configuration is itself in collision with obstacles.
        {
            CollisionChecker seed_checker(robot_, {});
            seed_checker.set_obstacles(obs, n_obs);
            if (seed_checker.check_config(q)) {
                SBF_TRACE("[GRW-FFB] reject(seed_collision) tid=%d seed=%s parent=%d root=%d",
                          worker_tid_, fmt_vec(q).c_str(), parent_box_id, root_id);
                n_ffb_fail_++;
                return -1;
            }
        }

        // Emit FFB begin marker with root intervals
    const auto start_iv = (domain_root_ >= 0)
        ? lect_.node_intervals(domain_root_)
        : lect_.root_intervals();
    SBF_TRACE("[FFB] begin tid=%d seed=%s start_iv=%s max_depth=%d",
              worker_tid_, fmt_vec(q).c_str(), fmt_intervals(start_iv).c_str(),
              config_.ffb_config.max_depth);

    FFBResult ffb;
    {
        // Phase D1: tell FFB the seed has already been verified collision-free
        // (we just ran check_config above), saves one redundant check_config.
        FFBConfig ffb_cfg = config_.ffb_config;
        ffb_cfg.seed_known_free = true;
        ffb = (domain_root_ >= 0)
            ? find_free_box_in_domain(lect_, domain_root_, q, obs, n_obs, ffb_cfg)
            : find_free_box(lect_, q, obs, n_obs, ffb_cfg);
    }

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
        SBF_TRACE("[GRW-FFB] fail tid=%d code=%d steps=%d t=%.2fms seed=%s parent=%d",
                  worker_tid_, ffb.fail_code, ffb.n_steps, ffb.total_ms,
                  fmt_vec(q).c_str(), parent_box_id);
        n_ffb_fail_++;
        return -1;
    }
    if (lect_.is_occupied(ffb.node_idx)) {
        SBF_TRACE("[GRW-FFB] reject(post-occupied) tid=%d leaf=%d depth=%d seed=%s",
                  worker_tid_, ffb.node_idx, (int)ffb.path.size() - 1, fmt_vec(q).c_str());
        n_ffb_fail_++;
        return -1;
    }
    // Geometric domain restriction (parallel workers): reject any box
    // whose LECT node is outside the worker's assigned subdomain. With
    // the seed clamp above this should be vanishingly rare.
    if (domain_root_ >= 0 &&
        !lect_.is_descendant_of(ffb.node_idx, domain_root_)) {
        n_ffb_fail_++;
        return -1;
    }

    BoxNode box;
    box.id = shared_next_box_id_
        ? shared_next_box_id_->fetch_add(1, std::memory_order_relaxed)
        : next_box_id_++;
    box.joint_intervals = lect_.node_intervals(ffb.node_idx);
    box.seed_config = q;
    box.tree_id = ffb.node_idx;
    box.parent_box_id = parent_box_id;
    box.root_id = root_id;
    box.compute_volume();

    if (config_.enable_partitioned_lect_parallel && domain_root_ >= 0)
        lect_.mark_occupied_until(ffb.node_idx, box.id, domain_root_);
    else
        lect_.mark_occupied(ffb.node_idx, box.id);
    n_ffb_success_++;

    SBF_TRACE("[GRW-FFB] OK tid=%d box=%d leaf=%d depth=%d steps=%d t=%.2fms "
              "new_nodes=%d cache_h/m=%d/%d parent=%d root=%d seed=%s intervals=%s",
              worker_tid_, box.id, ffb.node_idx, (int)ffb.path.size() - 1, ffb.n_steps,
              ffb.total_ms, ffb.n_new_nodes,
              ffb.n_cache_hits, ffb.n_cache_misses,
              parent_box_id, root_id, fmt_vec(q).c_str(),
              fmt_intervals(box.joint_intervals).c_str());

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

    // Geometric-domain restriction: boundary seeds must stay inside the
    // worker's assigned LECT subdomain so FFB descends into subtree(domain_root_).
    // Empty domain_ivs vector when domain_root_ < 0 (no restriction).
    std::vector<Interval> domain_ivs;
    if (domain_root_ >= 0)
        domain_ivs = lect_.node_intervals(domain_root_);

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
            // Geometric-domain clamp: keep seed strictly inside the
            // worker's LECT subdomain. The 1e-9 inset avoids ambiguity at
            // ancestor split planes where seed == split_val would route
            // FFB descent in either direction (left on `<=`).
            if (!domain_ivs.empty()) {
                const double margin = std::max(
                    1e-9, 1e-12 * (domain_ivs[d].hi - domain_ivs[d].lo));
                seed[d] = std::clamp(seed[d],
                                     domain_ivs[d].lo + margin,
                                     domain_ivs[d].hi - margin);
            }
        }
        seeds.push_back({face.dim, face.side, clamp_to_limits(seed)});
    }
    SBF_TRACE("[GRW-BNDY] box=%d generated %d boundary seeds (faces=%d, "
              "bias=%s)", box.id, (int)seeds.size(), (int)faces.size(),
              bias_target ? fmt_vec(*bias_target).c_str() : "none");
    return seeds;
}

// ─── select_roots ───────────────────────────────────────────────────────────
void ForestGrower::select_roots(const Obstacle* obs, int n_obs) {
    FFBConfig saved_ffb = config_.ffb_config;

    // Multi-goal roots: create a root at each goal point
    if (has_multi_goals_) {

        for (int i = 0; i < static_cast<int>(multi_goals_.size()); i++) {
            SBF_TRACE("[GRW-ROOT] tid=%d multi-goal seed %d/%zu = %s",
                      worker_tid_, i, multi_goals_.size(), fmt_vec(multi_goals_[i]).c_str());
            int id = try_create_box(multi_goals_[i], obs, n_obs, -1, -1, -1, i);
            SBF_INFO("[GRW] multi-goal root %d: id=%d", i, id);
        }
        SBF_INFO("[GRW] multi-goal roots: %d/%d created", (int)boxes_.size(), (int)multi_goals_.size());
        config_.ffb_config = saved_ffb;
        return;
    }

    if (has_endpoints_) {
        // Use user-specified max_depth for start/goal roots.

        SBF_TRACE("[GRW-ROOT] endpoint start = %s", fmt_vec(start_).c_str());
            SBF_TRACE("[GRW-ROOT] tid=%d endpoint start = %s", worker_tid_, fmt_vec(start_).c_str());
        int id0 = try_create_box(start_, obs, n_obs, -1, -1, -1, 0);
        SBF_TRACE("[GRW-ROOT] endpoint goal  = %s", fmt_vec(goal_).c_str());
            SBF_TRACE("[GRW-ROOT] tid=%d endpoint goal  = %s", worker_tid_, fmt_vec(goal_).c_str());
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

            SBF_TRACE("[GRW-ROOT] tid=%d diversity r=%d/K=%d best_k=%d best_min_dist=%.3f "
                      "seed=%s (existing_boxes=%d)",
                      worker_tid_, r, K_CANDIDATES, best_k, best_score,
                      fmt_vec(candidates[best_k]).c_str(), (int)boxes_.size());
            try_create_box(candidates[best_k], obs, n_obs, -1, -1, -1, r);
        }
    }

    config_.ffb_config = saved_ffb;
}

// ─── grow_rrt ───────────────────────────────────────────────────────────────
void ForestGrower::grow_rrt(const Obstacle* obs, int n_obs) {
    // Auto-build obs_grid for Grid-mode margin refinement if configured.
    // obs_grid_ is rebuilt on each grow_rrt call so obstacles are current.
    // Only active when margin_threshold > 0 and envelope is Grid type.
    std::unique_ptr<voxel::SparseVoxelGrid> obs_grid_owned;
    if (config_.ffb_config.grid_margin_threshold > 0.0f &&
        lect_.env_config().type != EnvelopeType::LinkIAABB) {
        const double delta = lect_.env_config().grid_config.voxel_delta;
        obs_grid_owned = std::make_unique<voxel::SparseVoxelGrid>(
            voxel::build_obs_grid(obs, n_obs, delta));
        config_.ffb_config.obs_grid = obs_grid_owned.get();
    }

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

    // Wrap-aware mask for revolute joints (S^1: 0 and 2π identical).
    const std::vector<uint8_t> wrap_mask = compute_wrap_mask(limits);

    // Temp buffer for q_rand as raw pointer (avoid Eigen per-element overhead)
    std::vector<double> q_buf(nd);

    // ── Anti-stuck heuristic ─────────────────────────────────────────────
    // Per-box consecutive failure counter; failing box gets a tabu penalty
    // in nearest search so other boxes get a chance.  Counter is reset on
    // any successful expansion from that box.  We deliberately keep
    // snap_to_face for adjacency: a box must share a face with its parent
    // to preserve forest connectivity, so we cannot use q_rand as seed.
    std::vector<int> box_fail_count(boxes_.size(), 0);
    double diag_sq = 0.0;
    for (int d = 0; d < nd; ++d)
        diag_sq += limits[d].width() * limits[d].width();
    const double tabu_step_sq = 0.0025 * diag_sq;

    // ── Volume-bonus heuristic ───────────────────────────────────────────
    // Larger boxes (more open free space) are preferred as RRT parents.
    // Bonus = alpha * diag_sq * (V_box / V_dom)^(2/nd). The (2/nd) exponent
    // makes the bonus dimensionally consistent (squared length) and scale-
    // invariant across nd. Maintained in log space to stay numerically
    // stable in high dimensions.
    const double vol_alpha = config_.vol_bonus_alpha;
    const double vol_coef = vol_alpha * diag_sq;
    const double vol_exp = (nd > 0) ? (2.0 / static_cast<double>(nd)) : 0.0;
    double log_V_dom = 0.0;
    for (int d = 0; d < nd; ++d)
        log_V_dom += std::log(std::max(limits[d].width(), 1e-300));
    std::vector<double> box_log_volume(boxes_.size(), 0.0);
    for (size_t i = 0; i < boxes_.size(); ++i) {
        double lv = 0.0;
        for (int d = 0; d < nd; ++d)
            lv += std::log(std::max(boxes_[i].joint_intervals[d].width(),
                                    1e-300));
        box_log_volume[i] = lv;
    }

    while (static_cast<int>(boxes_.size()) < config_.max_boxes &&
           miss_count < config_.max_consecutive_miss &&
           !deadline_reached() &&
           !global_budget_reached()) {

        // 1. Sample
        Eigen::VectorXd q_rand;
        int goal_tree_id = -1;  // for multi-goal: which tree the goal belongs to

        if (has_multi_goals_ && u01(rng_) < config_.rrt_goal_bias) {
            int gi = std::uniform_int_distribution<int>(
                0, static_cast<int>(multi_goals_.size()) - 1)(rng_);
            q_rand = multi_goals_[gi];
            goal_tree_id = gi;
        } else if (has_endpoints_ && u01(rng_) < config_.rrt_goal_bias) {
            q_rand = (u01(rng_) < 0.5) ? goal_ : start_;
        } else if (config_.unexplored_sample_prob > 0.0 &&
                   u01(rng_) < config_.unexplored_sample_prob) {
            q_rand = sample_unexplored();
        } else {
            q_rand = sample_random();
        }

        // 2. Find nearest box using flat center cache + tabu penalty.
        if (boxes_.empty()) { miss_count++; continue; }
        for (int d = 0; d < nd; ++d) q_buf[d] = q_rand[d];

        double best_dist = std::numeric_limits<double>::max();
        double best_geom_dist = std::numeric_limits<double>::max();
        double best_vol_r = 0.0;
        int best_idx = -1;
        const int n_boxes = static_cast<int>(boxes_.size());
        {
            const double* cc = center_cache.data();
            const double* qp = q_buf.data();
            for (int i = 0; i < n_boxes; ++i) {
                if (goal_tree_id >= 0 && boxes_[i].root_id == goal_tree_id) {
                    cc += nd; continue;
                }
                double d = cspace_squared_dist_flat(cc, qp, nd, wrap_mask);
                double r_pow = (vol_coef > 0.0)
                    ? std::exp(vol_exp * (box_log_volume[i] - log_V_dom))
                    : 0.0;
                double scored = d + tabu_step_sq * box_fail_count[i]
                                  - vol_coef * r_pow;
                if (scored < best_dist) {
                    best_dist = scored; best_geom_dist = d;
                    best_vol_r = r_pow; best_idx = i;
                }
                cc += nd;
            }
        }
        if (best_idx < 0) {
            const double* cc = center_cache.data();
            const double* qp = q_buf.data();
            for (int i = 0; i < n_boxes; ++i) {
                double d = cspace_squared_dist_flat(cc, qp, nd, wrap_mask);
                double r_pow = (vol_coef > 0.0)
                    ? std::exp(vol_exp * (box_log_volume[i] - log_V_dom))
                    : 0.0;
                double scored = d + tabu_step_sq * box_fail_count[i]
                                  - vol_coef * r_pow;
                if (scored < best_dist) {
                    best_dist = scored; best_geom_dist = d;
                    best_vol_r = r_pow; best_idx = i;
                }
                cc += nd;
            }
        }
        if (best_idx < 0) { miss_count++; continue; }

        // 3. Direction + snap (wrap-aware for revolute joints).
        // Snap-to-face is mandatory: it ensures the new box shares a face
        // with the parent, preserving forest adjacency / connectivity.
        Eigen::VectorXd direction =
            cspace_diff(q_rand, boxes_[best_idx].center(), wrap_mask);
        double dir_norm = direction.norm();
        if (dir_norm < 1e-12) { miss_count++; continue; }
        direction /= dir_norm;

        auto snap = snap_to_face(boxes_[best_idx], direction);

        SBF_TRACE("[GRW-RRT] sample=%s nearest=%d (root=%d) "
                  "geom_dist=%.3f tabu=%d vol_r=%.4f "
                  "snap_face=d%d/s%d snap_seed=%s",
                  fmt_vec(q_rand).c_str(),
                  boxes_[best_idx].id, boxes_[best_idx].root_id,
                  std::sqrt(best_geom_dist), box_fail_count[best_idx],
                  best_vol_r,
                  snap.face_dim, snap.face_side,
                  fmt_vec(snap.seed).c_str());

        // 4. Create box
        int parent_id = boxes_[best_idx].id;
        int bid = try_create_box(
            snap.seed, obs, n_obs,
            parent_id, snap.face_dim, snap.face_side,
            boxes_[best_idx].root_id);

        if (bid >= 0) {
            miss_count = 0;
            box_fail_count[best_idx] = 0;
            enforce_parent_adjacency(parent_id, snap.face_dim, snap.face_side,
                                     obs, n_obs);
            if (box_callback_)
                box_callback_(boxes_.back());
            const auto& nb = boxes_.back();
            for (int d = 0; d < nd; ++d)
                center_cache.push_back(nb.joint_intervals[d].center());
            box_fail_count.push_back(0);
            double lv_new = 0.0;
            for (int d = 0; d < nd; ++d)
                lv_new += std::log(std::max(nb.joint_intervals[d].width(),
                                            1e-300));
            box_log_volume.push_back(lv_new);
        } else {
            miss_count++;
            ++box_fail_count[best_idx];
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
                    if (box_callback_)
                        box_callback_(boxes_.back());
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
                    if (box_callback_)
                        box_callback_(boxes_.back());
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
// Bottom-up coarsening with **children-union** envelope semantics.
//
// At each candidate internal node `i` whose two children are both occupied,
// we attempt to merge by computing the parent envelope as the 3D spatial
// union of the children's cached envelopes (per-link element-wise min/max
// for EP iAABBs, SparseVoxelGrid::merge for grid slots) — NOT by re-running
// FK over the parent's joint intervals. The union is always tighter than
// re-FK, so this both succeeds more often and updates the LECT envelope
// cache with a strictly tighter envelope that is reusable for future
// queries. After a successful promotion at `i`, we immediately try to
// promote `i`'s ancestor, propagating the union upward as far as possible
// in a single sweep.
int ForestGrower::promote_all(const Obstacle* obs, int n_obs,
                               const std::vector<int>& start_nodes) {
    int total = 0;

    // Build box_id → boxes_ index for fast removal.
    std::unordered_map<int, int> id_to_idx;
    for (int i = 0; i < static_cast<int>(boxes_.size()); ++i)
        id_to_idx[boxes_[i].id] = i;

    auto remove_box_by_id = [&](int box_id) {
        auto it = id_to_idx.find(box_id);
        if (it == id_to_idx.end()) return;
        int idx  = it->second;
        int last = static_cast<int>(boxes_.size()) - 1;
        id_to_idx.erase(it);
        if (idx < last) {
            id_to_idx[boxes_[last].id] = idx;
            boxes_[idx] = std::move(boxes_[last]);
        }
        boxes_.pop_back();
    };

    // Try to promote at internal node `i`. On success returns true and
    // populates `*out_parent` with parent index (for upward chaining).
    auto try_promote_at = [&](int i) -> bool {
        if (i < 0) return false;
        if (lect_.is_leaf(i)) return false;
        if (lect_.is_occupied(i)) return false;

        int li = lect_.left(i);
        int ri = lect_.right(i);
        if (li < 0 || ri < 0) return false;
        if (!lect_.is_occupied(li) || !lect_.is_occupied(ri)) return false;

        // Build & verify parent envelope as union of children's envelopes.
        // On failure parent cache is rolled back; nothing else changes.
        if (!lect_.try_promote_envelope_union(i, li, ri, obs, n_obs))
            return false;

        // Determine root_id from a child box (both belong to one tree).
        int li_box_id = lect_.forest_id(li);
        int ri_box_id = lect_.forest_id(ri);
        int promoted_root = -1;
        {
            auto it = id_to_idx.find(li_box_id);
            if (it != id_to_idx.end())
                promoted_root = boxes_[it->second].root_id;
        }

        // Unmark children (preserves their envelope cache for later reuse).
        lect_.unmark_occupied(li);
        lect_.unmark_occupied(ri);

        // Remove child forest boxes.
        remove_box_by_id(li_box_id);
        remove_box_by_id(ri_box_id);

        // Create promoted parent box.
        auto parent_ivs = lect_.node_intervals(i);
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

        ++total;
        return true;
    };

    // Outer sweep: scan candidate nodes.  When start_nodes is non-empty
    // (parallel post-merge mode), we only need to try the cross-domain
    // ancestor chain (parent(R_i) upward) instead of scanning all n_nodes.
    // Serial mode passes an empty start_nodes; fall back to full scan.
    bool changed = true;
    if (!start_nodes.empty()) {
        // Candidate-restricted sweep: no need for an outer `changed` loop
        // because the candidate set already traces the unique ancestor chains
        // that could become newly promote-able after the worker merge.
        // A single pass with bubble-upward is sufficient.
        while (changed && !deadline_reached()) {
            changed = false;
            for (int i : start_nodes) {
                if (deadline_reached()) break;
                int cur = i;
                while (cur >= 0 && try_promote_at(cur)) {
                    changed = true;
                    cur = lect_.parent(cur);
                    if (deadline_reached()) break;
                }
            }
        }
        return total;
    }
    while (changed && !deadline_reached()) {
        changed = false;
        const int n_nodes = lect_.n_nodes();
        for (int i = 0; i < n_nodes; ++i) {
            if (deadline_reached()) break;
            // Geometric-domain workers only promote within their subtree.
            if (domain_root_ >= 0 && !lect_.is_descendant_of(i, domain_root_))
                continue;
            int cur = i;
            while (cur >= 0 && try_promote_at(cur)) {
                changed = true;
                cur = lect_.parent(cur);   // recurse up
                if (deadline_reached()) break;
                // Don't bubble past the worker's domain root.
                if (domain_root_ >= 0 && cur == lect_.parent(domain_root_))
                    break;
            }
        }
    }
    return total;
}

// ─── grow (main entry) ─────────────────────────────────────────────────────
GrowerResult ForestGrower::grow(const Obstacle* obs, int n_obs) {
    auto t0 = Clock::now();

    // Phase A: bump obstacle generation; per-node collide-verified cache
    // entries from a previous grow() with a different obstacle set are
    // invalidated automatically (gen mismatch).
    lect_.bump_obs_generation();

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
    double growth_ms = 0.0;
    double promotion_ms = 0.0;

    // 2. Growth
    auto t_wave = Clock::now();
    int n_promotions = 0;

    if (config_.enable_partitioned_lect_parallel &&
        config_.n_threads > 1 && static_cast<int>(boxes_.size()) >= 2) {
        GrowerResult par_result;
        grow_partitioned_shared(obs, n_obs, par_result);
        n_promotions = par_result.n_promotions;
        growth_ms = std::chrono::duration<double, std::milli>(Clock::now() - t_wave).count();
        SBF_INFO("[GRW] timing: roots=%.0fms partitioned_grow=%.0fms", roots_ms, growth_ms);
    } else if (config_.enable_coordinated_multi_goal &&
        config_.connect_mode && config_.n_threads > 1
        && has_multi_goals_ && static_cast<int>(boxes_.size()) >= 2) {
        // Coordinated parallel: master manages boxes, workers do FFB
        grow_coordinated(obs, n_obs);
        n_promotions = n_coordinated_promotions_;
        growth_ms = std::chrono::duration<double, std::milli>(Clock::now() - t_wave).count();
        SBF_INFO("[GRW] timing: roots=%.0fms coordinated_grow=%.0fms", roots_ms, growth_ms);
    } else if (config_.n_threads > 1 && static_cast<int>(boxes_.size()) >= 2) {
        // Parallel path — each tree grows independently with its own budget
        GrowerResult par_result;
        grow_parallel(obs, n_obs, par_result);
        n_promotions = par_result.n_promotions;
        growth_ms = std::chrono::duration<double, std::milli>(Clock::now() - t_wave).count();
        SBF_INFO("[GRW] timing: roots=%.0fms parallel_grow=%.0fms", roots_ms, growth_ms);

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
            wf_connect_time_ms_ = growth_ms;
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
        // Phase B: optional hierarchical max_depth schedule. Empty = legacy.
        auto run_one_pass = [&]() {
            if (config_.mode == GrowerConfig::Mode::WAVEFRONT)
                grow_wavefront(obs, n_obs);
            else
                grow_rrt(obs, n_obs);
        };
        if (config_.ffb_depth_stages.empty()) {
            run_one_pass();
        } else {
            const int orig_max_depth = config_.ffb_config.max_depth;
            const int orig_max_boxes = config_.max_boxes;
            double cum_ratio = 0.0;
            for (const auto& [stg_depth, stg_ratio] : config_.ffb_depth_stages) {
                if (deadline_reached()) break;
                if (config_.stop_after_connect && wf_all_connected_) break;
                cum_ratio += stg_ratio;
                config_.ffb_config.max_depth = stg_depth;
                config_.max_boxes = std::max(
                    static_cast<int>(boxes_.size()) + 1,
                    static_cast<int>(orig_max_boxes * std::min(1.0, cum_ratio)));
                run_one_pass();
                if ((int)boxes_.size() >= orig_max_boxes) break;
            }
            config_.ffb_config.max_depth = orig_max_depth;
            config_.max_boxes = orig_max_boxes;
        }
        growth_ms = std::chrono::duration<double, std::milli>(Clock::now() - t_wave).count();

        // 3. Promotion
        auto t_promo = Clock::now();
        if (config_.enable_promotion && !deadline_reached())
            n_promotions = promote_all(obs, n_obs);
        promotion_ms = std::chrono::duration<double, std::milli>(Clock::now() - t_promo).count();

        SBF_INFO("[GRW] timing: roots=%.0fms wave=%.0fms promo=%.0fms", roots_ms, growth_ms, promotion_ms);
    }

    // Phase C-1: endpoint-mode auto bridge (no-op unless start/goal disconnected).
    if (config_.endpoint_auto_bridge && has_endpoints_ && !has_multi_goals_) {
        int added = endpoint_bridge_pass(obs, n_obs);
        if (added > 0 && config_.enable_promotion && !deadline_reached())
            n_promotions += promote_all(obs, n_obs);
    }

    SBF_WARN("[GRW] final: boxes=%d success=%d fail=%d promo=%d", (int)boxes_.size(), n_ffb_success_, n_ffb_fail_, n_promotions);

    SBF_INFO("[GRW] ffb_stats: calls=%d steps=%d cache_hits=%d cache_misses=%d collide_calls=%d expand_calls=%d", ffb_total_calls_, ffb_total_steps_, ffb_cache_hits_, ffb_cache_misses_, ffb_collide_calls_, ffb_expand_calls_);
    SBF_INFO("[GRW] ffb_timing: total=%.1fms envelope=%.1fms collide=%.1fms expand=%.1fms intervals=%.1fms", ffb_total_ms_, ffb_envelope_ms_, ffb_collide_ms_, ffb_expand_ms_, ffb_intervals_ms_);
    if (ffb_total_calls_ > 0) {
        double other_ms = ffb_total_ms_ - ffb_envelope_ms_ - ffb_collide_ms_ - ffb_expand_ms_ - ffb_intervals_ms_;
        SBF_INFO("[GRW] ffb_breakdown: total=%.0fms env=%.0fms(%.0f%%) col=%.0fms(%.0f%%) exp=%.0fms(%.0f%%) other=%.0fms(%.0f%%) per_call_total=%.3fms",
                 ffb_total_ms_,
                 ffb_envelope_ms_, 100.0*ffb_envelope_ms_/ffb_total_ms_,
                 ffb_collide_ms_, 100.0*ffb_collide_ms_/ffb_total_ms_,
                 ffb_expand_ms_, 100.0*ffb_expand_ms_/ffb_total_ms_,
                 other_ms, 100.0*other_ms/ffb_total_ms_,
                 ffb_total_ms_ / ffb_total_calls_);
    }
    if (ffb_collide_calls_ > 0)
        SBF_INFO("[GRW] ffb_avg_collide: %.4fms/call", ffb_collide_ms_ / ffb_collide_calls_);
    if (ffb_cache_misses_ > 0)
        SBF_INFO("[GRW] ffb_avg_miss: envelope=%.3fms/call", ffb_envelope_ms_ / ffb_cache_misses_);
    if (ffb_expand_calls_ > 0)
        SBF_INFO("[GRW] ffb_avg_expand: %.3fms/call (%d calls)", ffb_expand_ms_ / ffb_expand_calls_, ffb_expand_calls_);
    {
        const auto& ep = lect_.expand_profile_;
        if (ep.expand_calls > 0) {
            SBF_INFO("[GRW] expand_profile: calls=%d new_nodes=%d pick_dim=%.1fms fk=%.1fms env=%.1fms refine=%.1fms", ep.expand_calls, ep.expand_calls * 2, ep.pick_dim_ms, ep.fk_inc_ms, ep.envelope_ms, ep.refine_ms);
            const double prof_sum = ep.pick_dim_ms + ep.fk_inc_ms + ep.envelope_ms + ep.refine_ms;
            SBF_INFO("[GRW] expand_total: total=%.1fms profiled=%.1fms unprofiled=%.1fms (%.0f%%)",
                     ep.total_ms, prof_sum, ep.total_ms - prof_sum,
                     ep.total_ms > 0 ? 100.0 * (ep.total_ms - prof_sum) / ep.total_ms : 0.0);
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
    result.root_select_ms = roots_ms;
    result.growth_ms = growth_ms;
    result.promotion_ms = promotion_ms;

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

    // Tree-level UF connectivity from coordinated growth.
    // Restore v6 (304e108) semantics: result.all_connected = tree-UF.
    // The sbf_planner_build.cpp "skip bridge when grow_connected" fast path
    // relies on this cheap tree-UF predicate; switching it to strict box-UF
    // silently forces the full bridge+coarsen2+seed_bridge pipeline every
    // build (~3s of extra work) even when the forest is already well
    // connected. The box-level canonical check is preserved below as
    // adjacency_all_connected / adjacency_islands for diagnostics.
    result.tree_all_connected = wf_all_connected_;
    result.tree_connect_time_ms = wf_connect_time_ms_ >= 0 ? wf_connect_time_ms_ : 0.0;
    result.tree_connect_n_boxes = wf_connect_boxes_ > 0 ? wf_connect_boxes_
        : (wf_all_connected_ ? static_cast<int>(boxes_.size()) : 0);
    result.all_connected = result.tree_all_connected;
    // Backward-compatible aliases for existing fields.
    result.connect_time_ms = result.tree_connect_time_ms;
    result.connect_n_boxes = result.tree_connect_n_boxes;

    // Diagnostic: full adjacency graph + box-level UF (not used as the
    // fast-path gate; see comment above).
    {
        auto t_adj_check = Clock::now();
        auto final_adj = compute_adjacency(boxes_);
        auto final_islands = find_islands(final_adj);
        int largest_island = 0;
        for (const auto& isl : final_islands)
            largest_island = std::max(largest_island, static_cast<int>(isl.size()));

        result.adjacency_islands = static_cast<int>(final_islands.size());
        result.adjacency_largest_island = largest_island;
        result.adjacency_all_connected = (result.adjacency_islands <= 1);

        result.adjacency_check_ms = std::chrono::duration<double, std::milli>(
            Clock::now() - t_adj_check).count();

        // Diagnostic: check if cross-tree pairs are in adjacency graph
        SBF_INFO("[GRW-ADJ] islands=%d boxes=%d", result.adjacency_islands, (int)boxes_.size());

        // Island composition: which trees are in each island
        {
            std::unordered_map<int, int> id2idx;
            for (int i = 0; i < (int)boxes_.size(); ++i)
                id2idx[boxes_[i].id] = i;
            for (int ii = 0; ii < std::min((int)final_islands.size(), 10); ++ii) {
                std::map<int, int> tree_counts;
                for (int bid : final_islands[ii]) {
                    auto it = id2idx.find(bid);
                    if (it != id2idx.end())
                        tree_counts[boxes_[it->second].root_id]++;
                }
                std::string tc_str;
                for (auto& [tid, cnt] : tree_counts)
                    tc_str += " t" + std::to_string(tid) + "=" + std::to_string(cnt);
                SBF_INFO("[GRW-ADJ] island[%d]: %d boxes |%s",
                         ii, (int)final_islands[ii].size(), tc_str.c_str());
            }
        }

        // Check specific cross-tree pairs in adj graph
        for (auto& [ida, idb] : cross_tree_pairs_) {
            auto it_a = final_adj.find(ida), it_b = final_adj.find(idb);
            bool a_exists = it_a != final_adj.end();
            bool b_exists = it_b != final_adj.end();
            bool edge_found = false;
            if (a_exists) {
                for (int nid : it_a->second)
                    if (nid == idb) { edge_found = true; break; }
            }
            // Check which island each belongs to
            int isl_a = -1, isl_b = -1;
            for (int ii = 0; ii < (int)final_islands.size(); ++ii) {
                for (int bid : final_islands[ii]) {
                    if (bid == ida) isl_a = ii;
                    if (bid == idb) isl_b = ii;
                }
            }
            SBF_INFO("[GRW-ADJ] XTOUCH id=%d(isl%d) <-> id=%d(isl%d) in_adj=%d a_exists=%d b_exists=%d",
                     ida, isl_a, idb, isl_b, edge_found, a_exists, b_exists);
        }

        // Brute-force: find box pairs that are adjacent but NOT in adj graph
        int n_adj_missing = 0;
        std::unordered_map<int, int> id2idx;
        for (int i = 0; i < (int)boxes_.size(); ++i)
            id2idx[boxes_[i].id] = i;
        // check every pair in cross-tree contacts via find_islands membership
        if (result.adjacency_islands > 1) {
            // build island membership
            std::unordered_map<int, int> box_island;
            for (int ii = 0; ii < (int)final_islands.size(); ++ii)
                for (int bid : final_islands[ii])
                    box_island[bid] = ii;
            // scan for missing cross-island edges (sample 50k pairs)
            int checked = 0;
            for (int i = 0; i < (int)boxes_.size() && checked < 50000; ++i) {
                for (int j = i + 1; j < (int)boxes_.size() && checked < 50000; ++j) {
                    int isl_i = box_island[boxes_[i].id];
                    int isl_j = box_island[boxes_[j].id];
                    if (isl_i == isl_j) continue;
                    checked++;
                    if (boxes_adjacent(boxes_[i], boxes_[j])) {
                        // Check if edge exists in adj graph
                        auto& nbrs = final_adj[boxes_[i].id];
                        bool in_adj = false;
                        for (int nid : nbrs)
                            if (nid == boxes_[j].id) { in_adj = true; break; }
                        if (!in_adj) {
                            n_adj_missing++;
                            if (n_adj_missing <= 5) {
                                double max_gap = 0; int gap_dims = 0;
                                for (int d = 0; d < boxes_[i].n_dims(); ++d) {
                                    double gap = std::max(boxes_[i].joint_intervals[d].lo, boxes_[j].joint_intervals[d].lo)
                                               - std::min(boxes_[i].joint_intervals[d].hi, boxes_[j].joint_intervals[d].hi);
                                    if (gap > 1e-6) { max_gap = std::max(max_gap, gap); gap_dims++; }
                                }
                                SBF_INFO("[GRW-ADJ] CROSS-ISLAND EDGE MISSING from adj graph: id=%d(isl%d) id=%d(isl%d) max_gap=%.2e gap_dims=%d",
                                         boxes_[i].id, isl_i, boxes_[j].id, isl_j, max_gap, gap_dims);
                            }
                        }
                    }
                }
            }
            SBF_INFO("[GRW-ADJ] cross-island pairs checked=%d missing_edges=%d", checked, n_adj_missing);
        }
    }

    auto t1 = Clock::now();
    result.build_time_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();

    return result;
}

// ─── grow_subtree (worker entry point) ──────────────────────────────────────

// Phase C-1: endpoint-mode auto bridge helpers.
bool ForestGrower::start_goal_adj_connected() const {
    if (!has_endpoints_) return true;  // not endpoint mode → vacuously OK
    int s_idx = -1, g_idx = -1;
    for (int i = 0; i < (int)boxes_.size(); ++i) {
        if (s_idx < 0 && boxes_[i].contains(start_)) s_idx = i;
        if (g_idx < 0 && boxes_[i].contains(goal_))  g_idx = i;
        if (s_idx >= 0 && g_idx >= 0) break;
    }
    if (s_idx < 0 || g_idx < 0) return false;
    if (s_idx == g_idx) return true;
    // BFS on boxes_adjacent.
    const int n = (int)boxes_.size();
    std::vector<uint8_t> seen(n, 0);
    std::vector<int> q; q.reserve(n);
    q.push_back(s_idx); seen[s_idx] = 1;
    for (size_t h = 0; h < q.size(); ++h) {
        int i = q[h];
        if (i == g_idx) return true;
        for (int j = 0; j < n; ++j) {
            if (seen[j]) continue;
            if (boxes_adjacent(boxes_[i], boxes_[j])) { seen[j] = 1; q.push_back(j); }
        }
    }
    return seen[g_idx] != 0;
}

int ForestGrower::endpoint_bridge_pass(const Obstacle* obs, int n_obs) {
    if (!config_.endpoint_auto_bridge) return 0;
    if (!has_endpoints_ || has_multi_goals_) return 0;
    if (deadline_reached()) return 0;
    if (start_goal_adj_connected()) return 0;

    int extra = config_.endpoint_bridge_max_boxes;
    if (extra <= 0) {
        extra = std::clamp(config_.max_boxes / 20, 50, 500);
    }
    const int n_before = (int)boxes_.size();
    const int orig_max_boxes = config_.max_boxes;
    const int orig_domain_root = domain_root_;
    config_.max_boxes = n_before + extra;
    domain_root_ = -1;  // full domain for bridge pass

    SBF_INFO("[GRW] endpoint bridge: start_box and goal_box disconnected; running serial RRT pass with extra=%d",
             extra);
    grow_rrt(obs, n_obs);

    config_.max_boxes = orig_max_boxes;
    domain_root_ = orig_domain_root;

    int added = (int)boxes_.size() - n_before;
    bool ok = start_goal_adj_connected();
    SBF_INFO("[GRW] endpoint bridge: added=%d, connected=%s", added, ok ? "YES" : "NO");
    return added;
}

}  // namespace sbf

// SafeBoxForest v6 — RRT bridge helpers
#include <sbf/forest/connectivity.h>
#include <sbf/forest/adjacency.h>
#include <sbf/forest/thread_pool.h>
#include <sbf/core/union_find.h>
#include <sbf/core/log.h>
#include <sbf/core/log_format.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <limits>
#include <random>
#include <set>
#include <unordered_set>

#include <Eigen/Dense>

namespace sbf {

namespace {

/// Nearest-neighbor on cache-friendly flat double array using Eigen SIMD.
/// Flat storage eliminates VectorXd pointer chasing; Eigen::Map enables SIMD.
template<int NDIM>
int nearest_flat(const double* __restrict__ coords, int n,
                 const double* __restrict__ query) {
    using Vec = Eigen::Matrix<double, NDIM, 1>;
    Eigen::Map<const Vec> q(query);
    int best = 0;
    double best_d = std::numeric_limits<double>::max();
    for (int i = 0; i < n; ++i) {
        Eigen::Map<const Vec> p(coords + i * NDIM);
        double d = (p - q).squaredNorm();
        if (d < best_d) {
            best_d = d;
            best = i;
        }
    }
    return best;
}

/// Helper: add node to flat coordinate array.
template<int NDIM>
inline void flat_push(std::vector<double>& flat, const Eigen::VectorXd& q) {
    flat.insert(flat.end(), q.data(), q.data() + NDIM);
}

/// Steer from q_near toward q_target by at most step_size.
/// Returns false if q_near ≈ q_target.
bool rrt_steer(const Eigen::VectorXd& q_near,
               const Eigen::VectorXd& q_target,
               double step_size,
               Eigen::VectorXd& q_new) {
    Eigen::VectorXd diff = q_target - q_near;
    double d = diff.norm();
    if (d < 1e-8) return false;
    if (d <= step_size) {
        q_new = q_target;
    } else {
        q_new = q_near + (diff / d) * step_size;
    }
    return true;
}

/// Extend tree toward q_target. Returns new node index or -1.
int rrt_extend(std::vector<Eigen::VectorXd>& nodes,
               std::vector<double>& flat,
               std::vector<int>& parent,
               const Eigen::VectorXd& q_target,
               double step_size,
               const Eigen::VectorXd& lo,
               const Eigen::VectorXd& hi,
               const CollisionChecker& checker,
               int seg_res) {
    int near_idx = nearest_flat<7>(flat.data(),
                   static_cast<int>(nodes.size()), q_target.data());
    Eigen::VectorXd q_new;
    if (!rrt_steer(nodes[near_idx], q_target, step_size, q_new))
        return -1;
    for (int d = 0; d < q_new.size(); ++d)
        q_new[d] = std::clamp(q_new[d], lo[d], hi[d]);
    if (checker.check_config(q_new))
        return -1;
    if (checker.check_segment(nodes[near_idx], q_new, seg_res))
        return -1;
    int new_idx = static_cast<int>(nodes.size());
    nodes.push_back(q_new);
    flat_push<7>(flat, q_new);
    parent.push_back(near_idx);
    return new_idx;
}

/// Greedily extend tree toward q_target until reached or blocked.
/// Optimized: finds nearest ONCE, then tracks current node (no re-scan).
int rrt_connect_greedy(std::vector<Eigen::VectorXd>& nodes,
                       std::vector<double>& flat,
                       std::vector<int>& parent,
                       const Eigen::VectorXd& q_target,
                       double step_size,
                       const Eigen::VectorXd& lo,
                       const Eigen::VectorXd& hi,
                       const CollisionChecker& checker,
                       int seg_res,
                       int max_steps = 50) {
    int cur = nearest_flat<7>(flat.data(),
                  static_cast<int>(nodes.size()), q_target.data());
    for (int s = 0; s < max_steps; ++s) {
        double dist = (nodes[cur] - q_target).norm();
        if (dist < 1e-6)
            return cur;
        Eigen::VectorXd q_new;
        if (!rrt_steer(nodes[cur], q_target, step_size, q_new))
            return -1;
        for (int d = 0; d < q_new.size(); ++d)
            q_new[d] = std::clamp(q_new[d], lo[d], hi[d]);
        if (checker.check_config(q_new))
            return -1;
        if (checker.check_segment(nodes[cur], q_new, seg_res))
            return -1;
        int new_idx = static_cast<int>(nodes.size());
        nodes.push_back(q_new);
        flat_push<7>(flat, q_new);
        parent.push_back(cur);
        cur = new_idx;
        double dist_new = (q_new - q_target).norm();
        if (dist_new < step_size * 0.5) {
            if (!checker.check_segment(q_new, q_target, seg_res)) {
                nodes.push_back(q_target);
                flat_push<7>(flat, q_target);
                parent.push_back(cur);
                return static_cast<int>(nodes.size()) - 1;
            }
            return -1;
        }
    }
    return -1;
}

/// Extract path from root to node idx.
std::vector<Eigen::VectorXd> rrt_extract_path(
        const std::vector<Eigen::VectorXd>& nodes,
        const std::vector<int>& parent,
        int idx) {
    std::vector<Eigen::VectorXd> path;
    while (idx >= 0) {
        path.push_back(nodes[idx]);
        idx = parent[idx];
    }
    std::reverse(path.begin(), path.end());
    return path;
}

}  // anonymous namespace

std::vector<Eigen::VectorXd> rrt_connect(
        const Eigen::VectorXd& q_a,
        const Eigen::VectorXd& q_b,
        const CollisionChecker& checker,
        const Robot& robot,
        const RRTConnectConfig& cfg,
        int seed,
        std::shared_ptr<std::atomic<bool>> cancel) {
    const int ndim = static_cast<int>(q_a.size());
    const auto& limits = robot.joint_limits().limits;
    Eigen::VectorXd lo(ndim), hi(ndim);
    for (int d = 0; d < ndim; ++d) {
        lo[d] = limits[d].lo;
        hi[d] = limits[d].hi;
    }

    std::mt19937 rng(static_cast<unsigned>(seed));
    std::uniform_real_distribution<double> dist01(0.0, 1.0);

    // Dual trees + flat coordinate arrays for cache-friendly nearest-neighbor
    std::vector<Eigen::VectorXd> nodes_a = {q_a};
    std::vector<int> parent_a = {-1};
    std::vector<Eigen::VectorXd> nodes_b = {q_b};
    std::vector<int> parent_b = {-1};
    std::vector<double> flat_a(q_a.data(), q_a.data() + ndim);
    std::vector<double> flat_b(q_b.data(), q_b.data() + ndim);
    constexpr int RESERVE = 8192;
    nodes_a.reserve(RESERVE);  parent_a.reserve(RESERVE);  flat_a.reserve(RESERVE * 7);
    nodes_b.reserve(RESERVE);  parent_b.reserve(RESERVE);  flat_b.reserve(RESERVE * 7);

    auto t0 = std::chrono::steady_clock::now();
    const double deadline_ms = cfg.timeout_ms;

    for (int it = 0; it < cfg.max_iters; ++it) {
        // Timeout + cancel check every 8 iterations
        if ((it & 7) == 7) {
            if (cancel && cancel->load(std::memory_order_relaxed))
                return {};  // island already merged — abort early
            double elapsed = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t0).count();
            if (elapsed > deadline_ms) {
                SBF_INFO("[RRT] timeout: %.1fms > %.1fms after %d iters "
                         "(tree_a=%d, tree_b=%d)",
                         elapsed, deadline_ms, it + 1,
                         (int)nodes_a.size(), (int)nodes_b.size());
                return {};
            }
        }

        // Alternate trees each iteration
        auto* a_nodes  = (it % 2 == 0) ? &nodes_a  : &nodes_b;
        auto* a_parent = (it % 2 == 0) ? &parent_a : &parent_b;
        auto* a_flat   = (it % 2 == 0) ? &flat_a   : &flat_b;
        auto* p_nodes  = (it % 2 == 0) ? &nodes_b  : &nodes_a;
        auto* p_parent = (it % 2 == 0) ? &parent_b : &parent_a;
        auto* p_flat   = (it % 2 == 0) ? &flat_b   : &flat_a;
        const Eigen::VectorXd& target_root = (it % 2 == 0) ? q_b : q_a;

        // Sample: goal-biased toward other tree's root
        Eigen::VectorXd q_rand(ndim);
        if (dist01(rng) < cfg.goal_bias) {
            q_rand = target_root;
        } else {
            for (int d = 0; d < ndim; ++d) {
                std::uniform_real_distribution<double> dim_dist(lo[d], hi[d]);
                q_rand[d] = dim_dist(rng);
            }
        }

        // Extend active tree toward q_rand
        int new_idx = rrt_extend(*a_nodes, *a_flat, *a_parent, q_rand,
                                 cfg.step_size, lo, hi, checker,
                                 cfg.segment_resolution);
        if (new_idx < 0)
            continue;

        // Try to connect passive tree to the new node
        const Eigen::VectorXd& q_new = (*a_nodes)[new_idx];
        int conn_idx = rrt_connect_greedy(*p_nodes, *p_flat, *p_parent, q_new,
                                          cfg.step_size, lo, hi, checker,
                                          cfg.segment_resolution);
        if (conn_idx >= 0) {
            // Connected! Build path
            std::vector<Eigen::VectorXd> path_a, path_b;
            if (it % 2 == 0) {
                path_a = rrt_extract_path(nodes_a, parent_a, new_idx);
                path_b = rrt_extract_path(nodes_b, parent_b, conn_idx);
            } else {
                path_a = rrt_extract_path(nodes_a, parent_a, conn_idx);
                path_b = rrt_extract_path(nodes_b, parent_b, new_idx);
            }
            std::reverse(path_b.begin(), path_b.end());
            // Merge: path_a + path_b[1:]
            std::vector<Eigen::VectorXd> full_path = std::move(path_a);
            for (size_t i = 1; i < path_b.size(); ++i)
                full_path.push_back(std::move(path_b[i]));

            if (it + 1 > 50) {
                SBF_INFO("[RRT] connected: %d wp, %d iters "
                         "(ta=%d tb=%d)", (int)full_path.size(), it + 1,
                         (int)nodes_a.size(), (int)nodes_b.size());
            }
            return full_path;
        }
    }

    SBF_WARN("[RRT] fail after %d iters (ta=%d tb=%d)",
             cfg.max_iters, (int)nodes_a.size(), (int)nodes_b.size());
    return {};
}

// ─── bitstar_bridge (OMPL BIT*) ─────────────────────────────────────────────
#ifdef SBF_HAS_OMPL

std::vector<Eigen::VectorXd> bitstar_bridge(
        const Eigen::VectorXd& q_a,
        const Eigen::VectorXd& q_b,
        const CollisionChecker& checker,
        const Robot& robot,
        double timeout_ms,
        int seed,
        std::shared_ptr<std::atomic<bool>> cancel) {
    namespace ob = ompl::base;
    namespace og = ompl::geometric;

    // Suppress OMPL info/debug messages — only show warnings and errors
    ompl::msg::setLogLevel(ompl::msg::LOG_WARN);

    const int ndim = robot.n_joints();
    const auto& limits = robot.joint_limits().limits;

    // 1. Setup state space
    auto space = std::make_shared<ob::RealVectorStateSpace>(ndim);
    ob::RealVectorBounds bounds(ndim);
    for (int d = 0; d < ndim; ++d) {
        bounds.setLow(d, limits[d].lo);
        bounds.setHigh(d, limits[d].hi);
    }
    space->setBounds(bounds);

    auto si = std::make_shared<ob::SpaceInformation>(space);

    // 2. State validity checker (lightweight — no SBF box cache for bridge)
    si->setStateValidityChecker([&checker, ndim](const ob::State* state) -> bool {
        const auto* rv = state->as<ob::RealVectorStateSpace::StateType>();
        Eigen::VectorXd q(ndim);
        for (int d = 0; d < ndim; ++d)
            q[d] = rv->values[d];
        return !checker.check_config(q);
    });

    // 3. Motion validator — segment collision check
    si->setStateValidityCheckingResolution(0.01);  // 1% of space extent
    si->setup();

    // 4. Start/goal states
    ob::ScopedState<ob::RealVectorStateSpace> s_start(space);
    ob::ScopedState<ob::RealVectorStateSpace> s_goal(space);
    for (int d = 0; d < ndim; ++d) {
        s_start[d] = q_a[d];
        s_goal[d] = q_b[d];
    }

    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(s_start, s_goal);

    // 5. BIT* planner
    auto planner = std::make_shared<og::BITstar>(si);
    planner->setProblemDefinition(pdef);
    planner->setup();

    // 6. Termination condition: timeout OR cancel flag
    auto t0 = std::chrono::steady_clock::now();
    double timeout_sec = timeout_ms / 1000.0;
    auto ptc = ob::plannerOrTerminationCondition(
        ob::timedPlannerTerminationCondition(timeout_sec),
        ob::PlannerTerminationCondition([&cancel]() -> bool {
            return cancel && cancel->load(std::memory_order_relaxed);
        }));

    // 7. Solve
    ob::PlannerStatus status = planner->solve(ptc);

    double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();

    if (status != ob::PlannerStatus::EXACT_SOLUTION &&
        status != ob::PlannerStatus::APPROXIMATE_SOLUTION) {
        SBF_WARN("[BIT*] fail: %.1fms, status=%s", elapsed_ms, status.asString().c_str());
        return {};
    }

    // 8. Extract + simplify path
    auto path_ptr = pdef->getSolutionPath()->as<og::PathGeometric>();
    if (!path_ptr || path_ptr->getStateCount() < 2) return {};

    // Simplify to reduce waypoints
    og::PathSimplifier simplifier(si);
    simplifier.simplifyMax(*path_ptr);

    // 9. Convert to VectorXd path
    std::vector<Eigen::VectorXd> result;
    result.reserve(path_ptr->getStateCount());
    for (std::size_t i = 0; i < path_ptr->getStateCount(); ++i) {
        const auto* rv = path_ptr->getState(i)->as<ob::RealVectorStateSpace::StateType>();
        Eigen::VectorXd q(ndim);
        for (int d = 0; d < ndim; ++d)
            q[d] = rv->values[d];
        result.push_back(std::move(q));
    }

    SBF_INFO("[BIT*] connected: %d wp, %.1fms", (int)result.size(), elapsed_ms);
    return result;
}
#endif  // SBF_HAS_OMPL

// ─── chain_pave_along_path ───────────────────────────────────────────────────

namespace {

/// Generate a seed point just outside the best face of @p box toward @p target.
/// Mirrors ForestGrower::snap_to_face logic but is standalone.
Eigen::VectorXd pave_snap_seed(
        const BoxNode& box,
        const Eigen::VectorXd& target,
        const std::vector<Interval>& limits) {
    const int nd = box.n_dims();
    constexpr double eps = 1e-6;

    Eigen::VectorXd direction = target - box.center();
    double dnorm = direction.norm();
    if (dnorm > 1e-12) direction /= dnorm;

    // Pick the best face: highest direction component, excluding faces
    // that are already at joint limits.
    int best_dim = -1;
    int best_side = -1;    // 0=lo, 1=hi
    double best_score = -1e30;

    for (int d = 0; d < nd; ++d) {
        for (int side = 0; side < 2; ++side) {
            double normal_sign = (side == 1) ? 1.0 : -1.0;
            double score = direction[d] * normal_sign;
            if (score <= 0.0) continue;
            if (side == 0 && box.joint_intervals[d].lo - eps < limits[d].lo)
                continue;
            if (side == 1 && box.joint_intervals[d].hi + eps > limits[d].hi)
                continue;
            if (score > best_score) {
                best_score = score;
                best_dim = d;
                best_side = side;
            }
        }
    }

    Eigen::VectorXd seed(nd);
    if (best_dim < 0) {
        // No valid face — step toward target
        double step = 0.0;
        for (int d = 0; d < nd; ++d)
            step = std::max(step, box.joint_intervals[d].width());
        step *= 0.1;
        seed = box.center() + direction * step;
    } else {
        for (int d = 0; d < nd; ++d) {
            if (d == best_dim) {
                seed[d] = (best_side == 0)
                    ? box.joint_intervals[d].lo - eps
                    : box.joint_intervals[d].hi + eps;
            } else {
                // Bias toward target, but stay within box interval
                double lo_d = box.joint_intervals[d].lo;
                double hi_d = box.joint_intervals[d].hi;
                double t = std::clamp(target[d], lo_d, hi_d);
                seed[d] = t;
            }
        }
    }

    // Clamp to joint limits
    for (int d = 0; d < nd; ++d)
        seed[d] = std::clamp(seed[d], limits[d].lo, limits[d].hi);

    return seed;
}

/// Find box id containing point q, searching in @p boxes.
/// Returns -1 if not found.
int find_containing_box(const std::vector<BoxNode>& boxes,
                        const Eigen::VectorXd& q) {
    for (const auto& b : boxes) {
        if (b.contains(q)) return b.id;
    }
    return -1;
}

/// Find box whose AABB exterior distance to @p q is smallest.
/// Returns -1 if @p boxes is empty.  Inside-box always has dist 0 and wins.
int find_nearest_box_id(const std::vector<BoxNode>& boxes,
                        const Eigen::VectorXd& q) {
    int best_id = -1;
    double best_d2 = std::numeric_limits<double>::infinity();
    const int nd = static_cast<int>(q.size());
    for (const auto& b : boxes) {
        if (b.volume < 0) continue;  // dead box (compacted later)
        double s = 0.0;
        const int bnd = std::min(nd, b.n_dims());
        for (int d = 0; d < bnd; ++d) {
            double v = std::max({0.0,
                                 b.joint_intervals[d].lo - q[d],
                                 q[d] - b.joint_intervals[d].hi});
            s += v * v;
            if (s >= best_d2) break;  // early-exit
        }
        if (s < best_d2) {
            best_d2 = s;
            best_id = b.id;
            if (s == 0.0) return best_id;  // contained — can't beat
        }
    }
    return best_id;
}

/// Add an adjacency edge (if not already present).
void add_adj_edge(AdjacencyGraph& adj, int a, int b) {
    auto& va = adj[a];
    if (std::find(va.begin(), va.end(), b) == va.end()) {
        va.push_back(b);
        adj[b].push_back(a);
    }
}

/// Commit a new box: add to boxes, adj, id_to_idx, update adjacency with
/// all existing boxes.  Uses ONLY geometric adjacency (boxes_adjacent) —
/// never adds phantom edges.  The caller is responsible for ensuring
/// @p new_box geometrically overlaps/touches its intended parent.
void commit_box(BoxNode&& new_box,
                std::vector<BoxNode>& boxes,
                AdjacencyGraph& adj,
                std::unordered_map<int, int>& id_to_idx) {
    int new_id = new_box.id;
    adj[new_id] = {};
    id_to_idx[new_id] = static_cast<int>(boxes.size());

    // Check adjacency with all existing boxes (geometric only)
    for (const auto& b : boxes) {
        if (boxes_adjacent(new_box, b)) {
            add_adj_edge(adj, new_id, b.id);
        }
    }

    boxes.push_back(std::move(new_box));
}

}  // anonymous namespace

int chain_pave_along_path(
        const std::vector<Eigen::VectorXd>& rrt_path,
        int anchor_box_id,
        std::vector<BoxNode>& boxes,
        LECT& lect,
        const Obstacle* obs, int n_obs,
        const FFBConfig& ffb_config,
        AdjacencyGraph& adj,
        int& next_box_id,
        const Robot& robot,
        int max_chain,
        int max_steps_per_wp,
        const CollisionChecker* checker,
        double max_safe_gap) {
    if (rrt_path.empty()) return 0;

    const auto& limits = robot.joint_limits().limits;

    // Build id → box index map
    std::unordered_map<int, int> id_to_idx;
    for (int i = 0; i < static_cast<int>(boxes.size()); ++i)
        id_to_idx[boxes[i].id] = i;

    // Geometric-adjacency test helper (nullable if id missing).
    auto geom_adj = [&](int id_a, int id_b) -> bool {
        auto ia = id_to_idx.find(id_a);
        auto ib = id_to_idx.find(id_b);
        if (ia == id_to_idx.end() || ib == id_to_idx.end()) return false;
        return boxes_adjacent(boxes[ia->second], boxes[ib->second]);
    };

    // Current box: the anchor from which we start chaining
    int cur_box_id = anchor_box_id;
    int created = 0;

    // ── Coverage diagnostics ────────────────────────────────────────────────
    // Tracks per-waypoint outcomes so we can answer: "Can the box chain
    // fully cover the RRT path?" and "Is FFB depth the bottleneck?".
    int n_wp_existing = 0;   // wp already inside an existing box on entry
    int n_wp_paved    = 0;   // wp ended up inside a newly-paved box
    int n_wp_uncov    = 0;   // wp not covered by any box after attempts
    int n_ffb_try     = 0;
    int n_ffb_ok      = 0;
    int n_ffb_occ     = 0;   // fail_code 1
    int n_ffb_depth   = 0;   // fail_code 2 (max_depth)
    int n_ffb_dead    = 0;   // fail_code 4
    int n_adj_fail    = 0;   // parent extension rejected
    const bool diag_on =
        rrt_path.size() >= 8 && std::getenv("SBF_CHAIN_DIAG") != nullptr;

    // Optional FFB max_depth override (Q: "is FFB depth the bottleneck?").
    // FFB can degenerate to a single LECT cell when depth is unlimited; if
    // bumping max_depth still leaves uncov > 0 the bottleneck is NOT depth
    // but the seed/cell topology.
    FFBConfig ffb_local = ffb_config;
    if (const char* env = std::getenv("SBF_CHAIN_FFB_DEPTH")) {
        int v = std::atoi(env);
        if (v > 0) ffb_local.max_depth = v;
    }
    const FFBConfig& ffb_use = ffb_local;

    // ── Per-new-box parent-island diagnostic (only when diag_on) ────────────
    // Builds an island map over EXISTING boxes via UF on adj.  For each
    // newly-paved box, records its parent's island.  Answers: "Does the
    // chain extend FROM an existing island, or does it form an isolated
    // mid-air chain?".
    std::unordered_map<int, int> island_of;     // existing box_id -> island root
    std::unordered_map<int, int> ext_island_of; // new box_id      -> inherited island
    int anchor_island = -2;
    int n_nb_to_anchor   = 0;
    int n_nb_to_other    = 0;
    int n_nb_to_orphan   = 0;
    if (diag_on) {
        // Lightweight UF over existing boxes
        std::unordered_map<int, int> parent;
        parent.reserve(boxes.size() * 2);
        std::function<int(int)> find = [&](int x) {
            while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
            return x;
        };
        for (const auto& b : boxes) parent[b.id] = b.id;
        for (const auto& kv : adj) {
            int a = kv.first;
            if (!parent.count(a)) continue;
            int ra = find(a);
            for (int b : kv.second) {
                if (!parent.count(b)) continue;
                int rb = find(b);
                if (ra != rb) parent[rb] = ra;
            }
        }
        for (const auto& b : boxes) island_of[b.id] = find(b.id);
        auto it_anch = island_of.find(anchor_box_id);
        anchor_island = (it_anch != island_of.end()) ? it_anch->second : -1;
        std::fprintf(stderr,
            "[CHN-NB] === chain start: wp=%zu anchor=%d anchor_isl=%d "
            "existing_boxes=%zu existing_edges=%zu ===\n",
            rrt_path.size(), anchor_box_id, anchor_island,
            boxes.size(), adj.size());
    }

    for (size_t wi = 0; wi < rrt_path.size() && created < max_chain; ++wi) {
        const Eigen::VectorXd& wp = rrt_path[wi];
        const int created_before_wp = created;
        const bool wp_was_in_existing =
            (find_containing_box(boxes, wp) >= 0);
        if (wp_was_in_existing) ++n_wp_existing;

        // Waypoint-inside-existing jump.
        // Only honor the jump (i.e. add adj edge + hand off chain) when the
        // two boxes are GENUINELY geometrically adjacent.  Otherwise the
        // chain is broken at this point — we abandon the chain rather than
        // injecting a phantom edge through unchecked C-space.
        int existing = find_containing_box(boxes, wp);
        if (existing >= 0) {
            if (existing == cur_box_id) continue;
            if (geom_adj(cur_box_id, existing)) {
                add_adj_edge(adj, cur_box_id, existing);
                SBF_TRACE("[CHN] wp%zu jump cur=%d -> existing=%d (adj edge)",
                          wi, cur_box_id, existing);
                cur_box_id = existing;
                continue;
            }
            // Non-adjacent jump: existing waypoint sits inside another
            // island's box.  Hand off the chain to it as the new anchor
            // (no phantom edge added), and continue paving from there
            // toward the next waypoint.  This lets the chain naturally
            // "jump" between islands at every waypoint that lands inside
            // an existing box.
            SBF_TRACE("[CHN] wp%zu re-anchor to existing=%d (was cur=%d, no adj)",
                      wi, existing, cur_box_id);
            cur_box_id = existing;
            continue;
        }

        // Re-anchor to whichever SBF box is geometrically closest to this
        // waypoint (mirrors grow's frontier strategy).  This lets the chain
        // hop between islands: if @p wp lies near a box from a different
        // island than @p cur_box, the next paved box will be face-adjacent
        // to that other-island box, which @c commit_box detects via
        // boxes_adjacent and merges the islands automatically.
        {
            int nearest = find_nearest_box_id(boxes, wp);
            if (nearest >= 0) cur_box_id = nearest;
        }

        // Chain-extend from cur_box toward waypoint
        for (int step = 0; step < max_steps_per_wp && created < max_chain; ++step) {
            auto it = id_to_idx.find(cur_box_id);
            if (it == id_to_idx.end()) break;
            const BoxNode& cur_box = boxes[it->second];

            // If current box already contains wp, we're done with this waypoint
            if (cur_box.contains(wp)) break;

            // Generate snap_to_face seed toward the waypoint.
            // SBF_CHAIN_SEED_WP=1 overrides: use wp itself as the FFB seed.
            // This answers "if seed is wp directly, can FFB always cover wp?".
            // FFB at wp is guaranteed to land in the LECT cell containing wp
            // (or fail with occupied if that cell is already taken), so this
            // probes the cell-topology limit purely.
            Eigen::VectorXd seed;
            static const bool seed_wp_mode =
                std::getenv("SBF_CHAIN_SEED_WP") != nullptr;
            if (seed_wp_mode) {
                seed = wp;
            } else {
                seed = pave_snap_seed(cur_box, wp, limits);
            }

            // Seed-inside-existing jump (same discipline as waypoint).
            int seed_inside = find_containing_box(boxes, seed);
            if (seed_inside >= 0) {
                if (seed_inside == cur_box_id) {
                    // seed fell back inside cur_box — no progress possible
                    break;
                }
                if (geom_adj(cur_box_id, seed_inside)) {
                    add_adj_edge(adj, cur_box_id, seed_inside);
                    cur_box_id = seed_inside;
                    auto jt = id_to_idx.find(seed_inside);
                    if (jt != id_to_idx.end() && boxes[jt->second].contains(wp))
                        break;
                    continue;
                }
                // Non-adjacent seed jump: abandon this waypoint's chain.
                break;
            }

            // FFB at the seed point
            ++n_ffb_try;
            FFBResult ffb = find_free_box(lect, seed, obs, n_obs, ffb_use);
            if (!ffb.success() || lect.is_occupied(ffb.node_idx)) {
                if (ffb.fail_code == 1) ++n_ffb_occ;
                else if (ffb.fail_code == 2) ++n_ffb_depth;
                else if (ffb.fail_code == 4) ++n_ffb_dead;
                // FFB failed — skip this step, try next waypoint
                SBF_TRACE("[CHN] wp%zu step%d FFB fail code=%d steps=%d "
                          "seed=%s cur=%d",
                          wi, step, ffb.fail_code, ffb.n_steps,
                          fmt_vec(seed).c_str(), cur_box_id);
                break;
            }
            SBF_TRACE("[CHN] wp%zu step%d FFB OK leaf=%d depth=%d steps=%d "
                      "seed=%s cur=%d",
                      wi, step, ffb.node_idx, (int)ffb.path.size() - 1,
                      ffb.n_steps, fmt_vec(seed).c_str(), cur_box_id);
            ++n_ffb_ok;
            BoxNode new_box;
            new_box.id = next_box_id++;
            new_box.joint_intervals = lect.node_intervals(ffb.node_idx);
            new_box.seed_config = seed;
            new_box.tree_id = ffb.node_idx;
            new_box.parent_box_id = cur_box_id;
            // Inherit root_id from the anchor box
            {
                auto anch_it = id_to_idx.find(cur_box_id);
                if (anch_it != id_to_idx.end())
                    new_box.root_id = boxes[anch_it->second].root_id;
                else
                    new_box.root_id = -1;
            }

            // ── Guarantee geometric adjacency with parent box ──────────────
            // Two regimes:
            //   (a) Without checker: only close tiny floating-point gaps
            //       (≤ 1e-4) — these are LECT-cell roundoff between cells
            //       that already share a face.  Larger gaps abort.
            //   (b) With checker: gaps up to @p max_safe_gap (default 0.2
            //       rad) may be closed, provided the resulting extended
            //       interval product passes @c check_box.  This certifies
            //       the full extended box is collision-free, so the GCS
            //       path that uses it stays safe.
            bool adj_ok = false;
            {
                auto par_it = id_to_idx.find(cur_box_id);
                if (par_it != id_to_idx.end()) {
                    const BoxNode& parent = boxes[par_it->second];
                    const int nd = parent.n_dims();
                    constexpr double tiny_gap = 1e-4;
                    constexpr double overlap_margin = 1e-8;
                    const double gap_limit =
                        checker ? std::max(max_safe_gap, tiny_gap) : tiny_gap;

                    // Save original intervals so we can roll back if
                    // check_box rejects the extension.
                    auto orig_intervals = new_box.joint_intervals;
                    bool extended = false;

                    for (int d = 0; d < nd; ++d) {
                        double gap_hi = new_box.joint_intervals[d].lo
                                      - parent.joint_intervals[d].hi;
                        double gap_lo = parent.joint_intervals[d].lo
                                      - new_box.joint_intervals[d].hi;
                        if (gap_hi > 0 && gap_hi < gap_limit) {
                            new_box.joint_intervals[d].lo =
                                parent.joint_intervals[d].hi - overlap_margin;
                            if (gap_hi > tiny_gap) extended = true;
                        }
                        if (gap_lo > 0 && gap_lo < gap_limit) {
                            new_box.joint_intervals[d].hi =
                                parent.joint_intervals[d].lo + overlap_margin;
                            if (gap_lo > tiny_gap) extended = true;
                        }
                    }
                    adj_ok = boxes_adjacent(new_box, parent);

                    // If we extended beyond the tiny-gap regime, certify
                    // the new extended box via check_box.  If unsafe, roll
                    // back the extension and abort the chain.
                    if (adj_ok && extended && checker) {
                        if (checker->check_box(new_box.joint_intervals)) {
                            // collision: roll back
                            new_box.joint_intervals = std::move(orig_intervals);
                            adj_ok = false;
                        }
                    }
                }
            }
            if (!adj_ok) {
                ++n_adj_fail;
                if (!seed_wp_mode) {
                    // Roll back: do not commit the new FFB box, do not mark
                    // LECT cell occupied, do not increment next_box_id.
                    --next_box_id;
                    break;  // abort chain for this waypoint
                }
                // SBF_CHAIN_SEED_WP mode: pure coverage probe — commit
                // even without parent face-adjacency.  Connectivity is
                // delegated to the cross-box-extend block below.
            }

            new_box.compute_volume();

            // ── Best-effort extend for face-contact with NEARBY non-parent
            // boxes (small-gap closures across LECT cell boundaries from
            // other islands).  Each successful extension is validated by
            // check_box.  Without this, two paved chains coming from
            // different islands stay face-disconnected even when their
            // boxes lie within fractions of a radian of each other.
            if (checker && max_safe_gap > 0.0) {
                const int nd = new_box.n_dims();
                constexpr double tiny_gap = 1e-4;
                constexpr double overlap_margin = 1e-8;

                // Quick-screen: scan boxes whose center is within
                // (max_safe_gap + half-width) of new_box.center.
                Eigen::VectorXd nb_center(nd);
                for (int d = 0; d < nd; ++d)
                    nb_center[d] = 0.5 * (new_box.joint_intervals[d].lo +
                                          new_box.joint_intervals[d].hi);

                int n_extended = 0;
                constexpr int kMaxExtendsPerBox = 4;
                for (size_t bi = 0; bi < boxes.size() && n_extended < kMaxExtendsPerBox; ++bi) {
                    const BoxNode& cand = boxes[bi];
                    if (cand.id == new_box.id) continue;
                    if (cand.id == cur_box_id) continue;  // parent already handled
                    if (cand.volume < 0) continue;
                    // Pre-screen by AABB-AABB distance (Linf gap)
                    double max_gap = 0.0;
                    bool ok_screen = true;
                    for (int d = 0; d < nd; ++d) {
                        double gap_hi = new_box.joint_intervals[d].lo
                                      - cand.joint_intervals[d].hi;
                        double gap_lo = cand.joint_intervals[d].lo
                                      - new_box.joint_intervals[d].hi;
                        double g = std::max(0.0, std::max(gap_hi, gap_lo));
                        if (g > max_safe_gap) { ok_screen = false; break; }
                        if (g > max_gap) max_gap = g;
                    }
                    if (!ok_screen) continue;
                    if (max_gap == 0.0 && boxes_adjacent(new_box, cand))
                        continue;  // already face-adj

                    // Try extending new_box to face-contact cand.
                    auto orig = new_box.joint_intervals;
                    bool extended = false;
                    for (int d = 0; d < nd; ++d) {
                        double gap_hi = new_box.joint_intervals[d].lo
                                      - cand.joint_intervals[d].hi;
                        double gap_lo = cand.joint_intervals[d].lo
                                      - new_box.joint_intervals[d].hi;
                        if (gap_hi > 0 && gap_hi <= max_safe_gap) {
                            new_box.joint_intervals[d].lo =
                                cand.joint_intervals[d].hi - overlap_margin;
                            if (gap_hi > tiny_gap) extended = true;
                        }
                        if (gap_lo > 0 && gap_lo <= max_safe_gap) {
                            new_box.joint_intervals[d].hi =
                                cand.joint_intervals[d].lo + overlap_margin;
                            if (gap_lo > tiny_gap) extended = true;
                        }
                    }
                    bool now_adj = boxes_adjacent(new_box, cand);
                    if (!now_adj) {
                        new_box.joint_intervals = std::move(orig);
                        continue;
                    }
                    if (extended && checker->check_box(new_box.joint_intervals)) {
                        // collision in extended region: roll back
                        new_box.joint_intervals = std::move(orig);
                        continue;
                    }
                    new_box.compute_volume();
                    n_extended++;
                    SBF_TRACE("[CHN] wp%zu step%d extend new_box=%d -> cand=%d "
                              "(max_gap=%.4f)", wi, step, new_box.id,
                              cand.id, max_gap);
                }
            }

            lect.mark_occupied(ffb.node_idx, new_box.id);

            int new_id = new_box.id;
            const int parent_at_commit = cur_box_id;
            commit_box(std::move(new_box), boxes, adj, id_to_idx);
            created++;

            // Per-new-box parent-island log
            if (diag_on) {
                int par_isl;
                auto it_e = ext_island_of.find(parent_at_commit);
                if (it_e != ext_island_of.end()) {
                    par_isl = it_e->second;          // parent is itself a chain box
                } else {
                    auto it_i = island_of.find(parent_at_commit);
                    par_isl = (it_i != island_of.end()) ? it_i->second : -1;
                }
                ext_island_of[new_id] = par_isl;
                if (par_isl == anchor_island)      ++n_nb_to_anchor;
                else if (par_isl < 0)              ++n_nb_to_orphan;
                else                               ++n_nb_to_other;
                std::fprintf(stderr,
                    "[CHN-NB] wi=%zu step=%d new=%d par=%d par_isl=%d "
                    "anchor_isl=%d %s\n",
                    wi, step, new_id, parent_at_commit, par_isl,
                    anchor_island,
                    (par_isl == anchor_island) ? "OK_anchor"
                        : (par_isl < 0 ? "ORPHAN" : "OTHER_ISLAND"));
            }

            cur_box_id = new_id;

            // Check if new box already contains the waypoint
            {
                auto jt = id_to_idx.find(new_id);
                if (jt != id_to_idx.end() && boxes[jt->second].contains(wp))
                    break;
            }
        }

        // Per-wp coverage tally (only for wp NOT already in existing).
        if (!wp_was_in_existing) {
            const bool covered_now =
                (created > created_before_wp) &&
                (find_containing_box(boxes, wp) >= 0);
            if (covered_now) ++n_wp_paved;
            else             ++n_wp_uncov;
        }
    }

    if (diag_on) {
        std::fprintf(stderr,
            "[CHN-DIAG] wp=%zu existing=%d paved=%d uncov=%d  "
            "ffb_try=%d ok=%d occ=%d depth=%d dead=%d  "
            "adj_fail=%d created=%d  "
            "nb_to_anchor=%d nb_to_other=%d nb_orphan=%d  "
            "ffb_max_depth=%d\n",
            rrt_path.size(), n_wp_existing, n_wp_paved, n_wp_uncov,
            n_ffb_try, n_ffb_ok, n_ffb_occ, n_ffb_depth, n_ffb_dead,
            n_adj_fail, created,
            n_nb_to_anchor, n_nb_to_other, n_nb_to_orphan,
            ffb_use.max_depth);
    }

    return created;
}

// ─── repair_bridge_adjacency ─────────────────────────────────────────────────
// After bridge, adj_ may contain "forced" edges between boxes that are NOT
// geometrically adjacent (e.g. waypoint jumps, seed-inside-existing jumps).
// compute_adjacency() cannot reproduce these edges, causing island fragmentation
// when adjacency is recomputed after coarsening.
//
// Fix: for each forced edge, extend the SMALLER box to overlap with the larger
// box in every dimension where there's a gap.  The extension is tiny (< max_gap)
// and always INTO a collision-free box, so safety is preserved.
int repair_bridge_adjacency(std::vector<BoxNode>& boxes,
                            const AdjacencyGraph& adj,
                            double max_safe_extend) {
    std::unordered_map<int, int> id_to_idx;
    for (int i = 0; i < static_cast<int>(boxes.size()); ++i)
        id_to_idx[boxes[i].id] = i;

    constexpr double overlap_margin = 1e-8;
    int repaired = 0;
    int n_non_geom = 0;
    int n_separated = 0;  // fully separated in >= 1 dim
    int n_skipped_unsafe = 0;
    double max_gap_seen = 0.0;
    int n_gap_dims_total = 0;

    for (auto& [id_a, neighbors] : adj) {
        auto ia = id_to_idx.find(id_a);
        if (ia == id_to_idx.end()) continue;

        for (int id_b : neighbors) {
            if (id_b <= id_a) continue;  // dedup
            auto ib = id_to_idx.find(id_b);
            if (ib == id_to_idx.end()) continue;

            auto& box_a = boxes[ia->second];
            auto& box_b = boxes[ib->second];

            // Check if already geometrically adjacent
            if (boxes_adjacent(box_a, box_b)) continue;

            n_non_geom++;

            // Not geometrically adjacent — candidate for extension.
            auto& smaller = (box_a.volume <= box_b.volume) ? box_a : box_b;
            const auto& larger = (box_a.volume <= box_b.volume) ? box_b : box_a;
            const int nd = smaller.n_dims();

            // Measure gaps and check separation + safety cap.
            bool any_separated = false;
            bool unsafe = false;   // any dim needs extension larger than cap
            int gap_dims = 0;
            for (int d = 0; d < nd; ++d) {
                double gap_hi = smaller.joint_intervals[d].lo
                              - larger.joint_intervals[d].hi;
                double gap_lo = larger.joint_intervals[d].lo
                              - smaller.joint_intervals[d].hi;
                double gap = std::max(gap_hi, gap_lo);
                if (gap > 1e-10) {
                    any_separated = true;
                    gap_dims++;
                    max_gap_seen = std::max(max_gap_seen, gap);
                    if (gap > max_safe_extend) unsafe = true;
                }
            }
            if (any_separated) {
                n_separated++;
                n_gap_dims_total += gap_dims;
            }

            // Safety gate: refuse to inflate across large unchecked gaps.
            // The forced edge will be dropped by the next compute_adjacency().
            if (unsafe) {
                n_skipped_unsafe++;
                continue;
            }

            // Ensure real overlap (> overlap_margin) in EVERY dimension.
            // Handles both gaps (> 0) AND near-touching (≈ 0) cases.
            // Without this, pairs touching in 2+ dims fail the adjacency
            // condition: n_touching ≥ 1 && n_overlapping ≥ nd-1.
            bool modified = false;
            for (int d = 0; d < nd; ++d) {
                const auto& s = smaller.joint_intervals[d];
                const auto& l = larger.joint_intervals[d];
                double overlap = std::min(s.hi, l.hi) - std::max(s.lo, l.lo);
                if (overlap < 2 * overlap_margin) {
                    // Need more overlap — extend smaller toward larger
                    double l_center = 0.5 * (l.lo + l.hi);
                    if (s.lo >= l_center) {
                        // smaller is right of larger center → extend lo left
                        smaller.joint_intervals[d].lo =
                            l.hi - 2 * overlap_margin;
                    } else {
                        // smaller is left of larger center → extend hi right
                        smaller.joint_intervals[d].hi =
                            l.lo + 2 * overlap_margin;
                    }
                    modified = true;
                }
            }
            if (modified) {
                smaller.compute_volume();
                repaired++;
            }
        }
    }

    SBF_INFO("[BRG-REPAIR] non-geom=%d separated=%d repaired=%d "
             "skipped_unsafe=%d max_gap=%.6f max_safe=%.6f avg_gap_dims=%.1f",
             n_non_geom, n_separated, repaired, n_skipped_unsafe,
             max_gap_seen, max_safe_extend,
             n_separated > 0 ? (double)n_gap_dims_total / n_separated : 0.0);
    return repaired;
}

// ─── bridge_s_t (best-first S↔T only) ───────────────────────────────────────

}  // namespace sbf

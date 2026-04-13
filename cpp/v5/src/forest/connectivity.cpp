// SafeBoxForest v5 — Connectivity (Phase F7 + RRT-Connect bridge)
#include <sbf/forest/connectivity.h>

#include <algorithm>
#include <atomic>
#include <future>
#include <limits>
#include <random>
#include <set>
#include <unordered_map>
#include <unordered_set>

#include <Eigen/Dense>

#include <sbf/forest/thread_pool.h>

#ifdef SBF_HAS_OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/util/Console.h>
#endif

namespace sbf {

// ─── UnionFind ──────────────────────────────────────────────────────────────
UnionFind::UnionFind(int n) : parent_(n), rank_(n, 0) {
    for (int i = 0; i < n; ++i)
        parent_[i] = i;
}

int UnionFind::find(int x) {
    while (parent_[x] != x) {
        parent_[x] = parent_[parent_[x]];  // path halving
        x = parent_[x];
    }
    return x;
}

void UnionFind::unite(int x, int y) {
    int rx = find(x), ry = find(y);
    if (rx == ry) return;
    if (rank_[rx] < rank_[ry]) std::swap(rx, ry);
    parent_[ry] = rx;
    if (rank_[rx] == rank_[ry]) rank_[rx]++;
}

bool UnionFind::connected(int x, int y) {
    return find(x) == find(y);
}

void UnionFind::resize(int new_size) {
    int old_size = static_cast<int>(parent_.size());
    if (new_size <= old_size) return;
    parent_.resize(new_size);
    rank_.resize(new_size, 0);
    for (int i = old_size; i < new_size; ++i)
        parent_[i] = i;
}

// ─── find_islands ───────────────────────────────────────────────────────────
std::vector<std::vector<int>> find_islands(const AdjacencyGraph& adj) {
    // Collect all box ids
    std::vector<int> all_ids;
    all_ids.reserve(adj.size());
    for (const auto& kv : adj)
        all_ids.push_back(kv.first);

    if (all_ids.empty()) return {};

    // Map id → compact index
    std::unordered_map<int, int> id_to_idx;
    for (int i = 0; i < static_cast<int>(all_ids.size()); ++i)
        id_to_idx[all_ids[i]] = i;

    int n = static_cast<int>(all_ids.size());
    UnionFind uf(n);

    for (const auto& kv : adj) {
        int u = id_to_idx[kv.first];
        for (int neighbor_id : kv.second) {
            auto it = id_to_idx.find(neighbor_id);
            if (it != id_to_idx.end())
                uf.unite(u, it->second);
        }
    }

    // Group by root
    std::unordered_map<int, std::vector<int>> groups;
    for (int i = 0; i < n; ++i)
        groups[uf.find(i)].push_back(all_ids[i]);

    std::vector<std::vector<int>> islands;
    islands.reserve(groups.size());
    for (auto& kv : groups)
        islands.push_back(std::move(kv.second));

    return islands;
}

// ─── bridge_islands ─────────────────────────────────────────────────────────

// ─── rrt_connect (ported from Python v4 _rrt_connect_bridge) ────────────────

namespace {

/// Find index of nearest node in tree by L2 distance.
int rrt_nearest(const std::vector<Eigen::VectorXd>& nodes,
                const Eigen::VectorXd& q) {
    int best = 0;
    double best_d = (nodes[0] - q).squaredNorm();
    for (int i = 1; i < static_cast<int>(nodes.size()); ++i) {
        double d = (nodes[i] - q).squaredNorm();
        if (d < best_d) { best_d = d; best = i; }
    }
    return best;
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
               std::vector<int>& parent,
               const Eigen::VectorXd& q_target,
               double step_size,
               const Eigen::VectorXd& lo,
               const Eigen::VectorXd& hi,
               const CollisionChecker& checker,
               int seg_res) {
    int near_idx = rrt_nearest(nodes, q_target);
    Eigen::VectorXd q_new;
    if (!rrt_steer(nodes[near_idx], q_target, step_size, q_new))
        return -1;
    // Clamp to joint limits
    for (int d = 0; d < q_new.size(); ++d)
        q_new[d] = std::clamp(q_new[d], lo[d], hi[d]);
    // Collision checks
    if (checker.check_config(q_new))
        return -1;
    if (checker.check_segment(nodes[near_idx], q_new, seg_res))
        return -1;
    int new_idx = static_cast<int>(nodes.size());
    nodes.push_back(q_new);
    parent.push_back(near_idx);
    return new_idx;
}

/// Greedily extend tree toward q_target until reached or blocked.
int rrt_connect_greedy(std::vector<Eigen::VectorXd>& nodes,
                       std::vector<int>& parent,
                       const Eigen::VectorXd& q_target,
                       double step_size,
                       const Eigen::VectorXd& lo,
                       const Eigen::VectorXd& hi,
                       const CollisionChecker& checker,
                       int seg_res,
                       int max_steps = 50) {
    for (int s = 0; s < max_steps; ++s) {
        int near_idx = rrt_nearest(nodes, q_target);
        double dist = (nodes[near_idx] - q_target).norm();
        if (dist < 1e-6)
            return near_idx;  // Already at target
        int new_idx = rrt_extend(nodes, parent, q_target, step_size,
                                 lo, hi, checker, seg_res);
        if (new_idx < 0)
            return -1;  // Blocked
        double dist_new = (nodes[new_idx] - q_target).norm();
        if (dist_new < step_size * 0.5) {
            // Close enough — try direct connect
            if (!checker.check_segment(nodes[new_idx], q_target, seg_res)) {
                nodes.push_back(q_target);
                parent.push_back(new_idx);
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

    // Dual trees
    std::vector<Eigen::VectorXd> nodes_a = {q_a};
    std::vector<int> parent_a = {-1};
    std::vector<Eigen::VectorXd> nodes_b = {q_b};
    std::vector<int> parent_b = {-1};

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
                fprintf(stderr, "[RRT] timeout: %.1fms > %.1fms after %d iters "
                        "(tree_a=%d, tree_b=%d)\n",
                        elapsed, deadline_ms, it + 1,
                        (int)nodes_a.size(), (int)nodes_b.size());
                return {};
            }
        }

        // Alternate trees each iteration
        auto* a_nodes  = (it % 2 == 0) ? &nodes_a  : &nodes_b;
        auto* a_parent = (it % 2 == 0) ? &parent_a : &parent_b;
        auto* p_nodes  = (it % 2 == 0) ? &nodes_b  : &nodes_a;
        auto* p_parent = (it % 2 == 0) ? &parent_b : &parent_a;
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
        int new_idx = rrt_extend(*a_nodes, *a_parent, q_rand, cfg.step_size,
                                 lo, hi, checker, cfg.segment_resolution);
        if (new_idx < 0)
            continue;

        // Try to connect passive tree to the new node
        const Eigen::VectorXd& q_new = (*a_nodes)[new_idx];
        int conn_idx = rrt_connect_greedy(*p_nodes, *p_parent, q_new,
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
                fprintf(stderr, "[RRT] connected: %d wp, %d iters "
                        "(ta=%d tb=%d)\n",
                        (int)full_path.size(), it + 1,
                        (int)nodes_a.size(), (int)nodes_b.size());
            }
            return full_path;
        }
    }

    fprintf(stderr, "[RRT] fail after %d iters (ta=%d tb=%d)\n",
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
        fprintf(stderr, "[BIT*] fail: %.1fms, status=%s\n",
                elapsed_ms, status.asString().c_str());
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

    fprintf(stderr, "[BIT*] connected: %d wp, %.1fms\n",
            (int)result.size(), elapsed_ms);
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

/// Check if two boxes overlap in ALL dimensions (= non-empty intersection).
bool boxes_overlap(const BoxNode& a, const BoxNode& b) {
    int nd = a.n_dims();
    for (int d = 0; d < nd; ++d) {
        if (a.joint_intervals[d].hi <= b.joint_intervals[d].lo ||
            b.joint_intervals[d].hi <= a.joint_intervals[d].lo)
            return false;
    }
    return true;
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
/// all existing boxes.  Also accepts parent_id for forced-adjacency.
void commit_box(BoxNode&& new_box,
                std::vector<BoxNode>& boxes,
                AdjacencyGraph& adj,
                std::unordered_map<int, int>& id_to_idx,
                int parent_id = -1) {
    int new_id = new_box.id;
    adj[new_id] = {};
    id_to_idx[new_id] = static_cast<int>(boxes.size());

    // Check shared_face OR overlap with all existing boxes
    for (const auto& b : boxes) {
        if (shared_face(new_box, b).has_value() || boxes_overlap(new_box, b)) {
            add_adj_edge(adj, new_id, b.id);
        }
    }

    // Force adjacency to parent (even if shared_face/overlap somehow missed)
    if (parent_id >= 0 && adj[new_id].end() ==
        std::find(adj[new_id].begin(), adj[new_id].end(), parent_id)) {
        add_adj_edge(adj, new_id, parent_id);
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
        int max_steps_per_wp) {
    if (rrt_path.empty()) return 0;

    const auto& limits = robot.joint_limits().limits;

    // Build id → box index map
    std::unordered_map<int, int> id_to_idx;
    for (int i = 0; i < static_cast<int>(boxes.size()); ++i)
        id_to_idx[boxes[i].id] = i;

    // Current box: the anchor from which we start chaining
    int cur_box_id = anchor_box_id;
    int created = 0;

    for (size_t wi = 0; wi < rrt_path.size() && created < max_chain; ++wi) {
        const Eigen::VectorXd& wp = rrt_path[wi];

        // If current box already contains this waypoint, skip
        {
            auto it = id_to_idx.find(cur_box_id);
            if (it != id_to_idx.end() && boxes[it->second].contains(wp))
                continue;
        }

        // Check if waypoint is inside any existing box → jump to that box
        // and force adjacency with the chain's previous box.
        int existing = find_containing_box(boxes, wp);
        if (existing >= 0) {
            if (existing != cur_box_id) {
                add_adj_edge(adj, cur_box_id, existing);
            }
            cur_box_id = existing;
            continue;
        }

        // Chain-extend from cur_box toward waypoint
        for (int step = 0; step < max_steps_per_wp && created < max_chain; ++step) {
            auto it = id_to_idx.find(cur_box_id);
            if (it == id_to_idx.end()) break;
            const BoxNode& cur_box = boxes[it->second];

            // If current box already contains wp, we're done with this waypoint
            if (cur_box.contains(wp)) break;

            // Generate snap_to_face seed toward the waypoint
            Eigen::VectorXd seed = pave_snap_seed(cur_box, wp, limits);

            // Check if seed is inside an existing box
            int seed_inside = find_containing_box(boxes, seed);
            if (seed_inside >= 0) {
                if (seed_inside != cur_box_id) {
                    add_adj_edge(adj, cur_box_id, seed_inside);
                }
                cur_box_id = seed_inside;
                // Check if this box contains wp
                auto jt = id_to_idx.find(seed_inside);
                if (jt != id_to_idx.end() && boxes[jt->second].contains(wp))
                    break;
                continue;
            }

            // FFB at the seed point
            FFBResult ffb = find_free_box(lect, seed, obs, n_obs, ffb_config);
            if (!ffb.success() || lect.is_occupied(ffb.node_idx)) {
                // FFB failed — skip this step, try next waypoint
                break;
            }

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

            // ── Ensure geometric adjacency with parent box ──────────────
            // The FFB box might not overlap/touch the parent due to LECT
            // tree splits between the parent boundary and the seed point.
            // Close any small gaps so that compute_adjacency() detects the
            // connection without relying on forced adj_ edges.
            {
                auto par_it = id_to_idx.find(cur_box_id);
                if (par_it != id_to_idx.end()) {
                    const BoxNode& parent = boxes[par_it->second];
                    const int nd = parent.n_dims();
                    constexpr double max_gap = 1e-4;  // only close tiny gaps
                    constexpr double overlap_margin = 1e-8;  // ensure > tol
                    for (int d = 0; d < nd; ++d) {
                        double gap_hi = new_box.joint_intervals[d].lo
                                      - parent.joint_intervals[d].hi;
                        double gap_lo = parent.joint_intervals[d].lo
                                      - new_box.joint_intervals[d].hi;
                        if (gap_hi > 0 && gap_hi < max_gap) {
                            // new_box is above parent in dim d — extend lo down
                            new_box.joint_intervals[d].lo =
                                parent.joint_intervals[d].hi - overlap_margin;
                        }
                        if (gap_lo > 0 && gap_lo < max_gap) {
                            // new_box is below parent in dim d — extend hi up
                            new_box.joint_intervals[d].hi =
                                parent.joint_intervals[d].lo + overlap_margin;
                        }
                    }
                }
            }

            new_box.compute_volume();

            lect.mark_occupied(ffb.node_idx, new_box.id);

            int new_id = new_box.id;
            commit_box(std::move(new_box), boxes, adj, id_to_idx, cur_box_id);
            created++;

            cur_box_id = new_id;

            // Check if new box already contains the waypoint
            {
                auto jt = id_to_idx.find(new_id);
                if (jt != id_to_idx.end() && boxes[jt->second].contains(wp))
                    break;
            }
        }
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
                            const AdjacencyGraph& adj) {
    std::unordered_map<int, int> id_to_idx;
    for (int i = 0; i < static_cast<int>(boxes.size()); ++i)
        id_to_idx[boxes[i].id] = i;

    constexpr double overlap_margin = 1e-8;
    int repaired = 0;
    int n_non_geom = 0;
    int n_separated = 0;  // fully separated in >= 1 dim
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
            if (shared_face(box_a, box_b).has_value()) continue;
            if (boxes_overlap(box_a, box_b)) continue;

            n_non_geom++;

            // Not geometrically adjacent — extend the smaller box
            auto& smaller = (box_a.volume <= box_b.volume) ? box_a : box_b;
            const auto& larger = (box_a.volume <= box_b.volume) ? box_b : box_a;
            const int nd = smaller.n_dims();

            // Measure gaps and check separation
            bool any_separated = false;
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
                }
            }
            if (any_separated) {
                n_separated++;
                n_gap_dims_total += gap_dims;
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

    fprintf(stderr, "[BRG-REPAIR] non-geom=%d separated=%d repaired=%d max_gap=%.6f"
            " avg_gap_dims=%.1f\n",
            n_non_geom, n_separated, repaired, max_gap_seen,
            n_separated > 0 ? (double)n_gap_dims_total / n_separated : 0.0);
    fflush(stderr);

    return repaired;
}

// ─── bridge_s_t (best-first S↔T only) ───────────────────────────────────────
int bridge_s_t(
        int start_box_id,
        int goal_box_id,
        std::vector<BoxNode>& boxes,
        LECT& lect,
        const Obstacle* obs, int n_obs,
        AdjacencyGraph& adj,
        const FFBConfig& ffb_config,
        int& next_box_id,
        const Robot& robot,
        const CollisionChecker& checker,
        double per_pair_timeout_ms,
        int max_pairs,
        std::chrono::steady_clock::time_point deadline) {

    auto deadline_reached = [&]() {
        return deadline != std::chrono::steady_clock::time_point::max()
               && std::chrono::steady_clock::now() >= deadline;
    };

    // Helper: check if start_box and goal_box are in the same island
    auto s_t_connected = [&]() -> bool {
        auto islands = find_islands(adj);
        for (const auto& isl : islands) {
            bool has_s = false, has_g = false;
            for (int id : isl) {
                if (id == start_box_id) has_s = true;
                if (id == goal_box_id)  has_g = true;
            }
            if (has_s && has_g) return true;
        }
        return false;
    };

    if (s_t_connected()) return 0;

    // Build id → idx map
    std::unordered_map<int, int> id_to_idx;
    for (int i = 0; i < static_cast<int>(boxes.size()); ++i)
        id_to_idx[boxes[i].id] = i;

    // Collect boxes belonging to S-island and G-island by connectivity
    // (not root_id — start/goal may reside in diversity-root boxes)
    auto islands = find_islands(adj);
    std::vector<int> s_boxes, g_boxes;
    for (const auto& isl : islands) {
        bool has_s = false, has_g = false;
        for (int id : isl) {
            if (id == start_box_id) has_s = true;
            if (id == goal_box_id)  has_g = true;
        }
        if (has_s) s_boxes = isl;
        if (has_g) g_boxes = isl;
    }
    if (s_boxes.empty() || g_boxes.empty()) return 0;

    // Build candidate pairs sorted by distance (best-first)
    struct CandPair {
        int s_id, g_id;
        double dist;
        bool operator<(const CandPair& o) const { return dist < o.dist; }
    };
    std::vector<CandPair> candidates;
    candidates.reserve(std::min((int)s_boxes.size(), 200) *
                       std::min((int)g_boxes.size(), 200));

    for (int sid : s_boxes) {
        auto its = id_to_idx.find(sid);
        if (its == id_to_idx.end()) continue;
        Eigen::VectorXd cs = boxes[its->second].center();
        for (int gid : g_boxes) {
            auto itg = id_to_idx.find(gid);
            if (itg == id_to_idx.end()) continue;
            double d = (cs - boxes[itg->second].center()).squaredNorm();
            candidates.push_back({sid, gid, d});
        }
    }
    std::sort(candidates.begin(), candidates.end());

    // Limit to max_pairs
    if (static_cast<int>(candidates.size()) > max_pairs)
        candidates.resize(max_pairs);

    fprintf(stderr, "[BRG] bridge_s_t: s_tree=%d boxes, g_tree=%d boxes, "
            "candidates=%d (max_pairs=%d)\n",
            (int)s_boxes.size(), (int)g_boxes.size(),
            (int)candidates.size(), max_pairs);

    int bridges_created = 0;
    int rrt_seed = 42;

    for (int ci = 0; ci < static_cast<int>(candidates.size()); ++ci) {
        if (deadline_reached()) break;
        if (s_t_connected()) break;

        const auto& cp = candidates[ci];

        // Re-lookup box indices (may have shifted due to new boxes)
        id_to_idx.clear();
        for (int i = 0; i < static_cast<int>(boxes.size()); ++i)
            id_to_idx[boxes[i].id] = i;

        auto its = id_to_idx.find(cp.s_id);
        auto itg = id_to_idx.find(cp.g_id);
        if (its == id_to_idx.end() || itg == id_to_idx.end()) continue;

        Eigen::VectorXd ca = boxes[its->second].center();
        Eigen::VectorXd cb = boxes[itg->second].center();

        RRTConnectConfig pair_rrt;
        pair_rrt.max_iters = 100000;
        pair_rrt.step_size = 0.1;
        pair_rrt.goal_bias = 0.20;
        pair_rrt.timeout_ms = per_pair_timeout_ms;
        pair_rrt.segment_resolution = 10;

        auto path = rrt_connect(ca, cb, checker, robot, pair_rrt, rrt_seed++);
        if (path.empty()) continue;

        // Chain-pave from the S-side box along the RRT path
        int added = chain_pave_along_path(
            path, cp.s_id, boxes, lect, obs, n_obs,
            ffb_config, adj, next_box_id, robot,
            /*max_chain=*/500, /*max_steps_per_wp=*/15);
        bridges_created += added;

        fprintf(stderr, "[BRG] pair %d/%d: box %d->%d, dist=%.3f, "
                "rrt=%dwp, paved=%d, connected=%s\n",
                ci + 1, (int)candidates.size(), cp.s_id, cp.g_id,
                std::sqrt(cp.dist), (int)path.size(), added,
                s_t_connected() ? "YES" : "no");
    }
    return bridges_created;
}

// ─── bridge_all_islands (merge ALL disconnected components) ──────────────────
int bridge_all_islands(
        std::vector<BoxNode>& boxes,
        LECT& lect,
        const Obstacle* obs, int n_obs,
        AdjacencyGraph& adj,
        const FFBConfig& ffb_config,
        int& next_box_id,
        const Robot& robot,
        const CollisionChecker& checker,
        double per_pair_timeout_ms,
        int max_pairs_per_gap,
        int max_total_bridges,
        int n_threads,
        std::chrono::steady_clock::time_point deadline) {

    auto deadline_reached = [&]() {
        return deadline != std::chrono::steady_clock::time_point::max()
               && std::chrono::steady_clock::now() >= deadline;
    };

    int total_bridges = 0;
    int rrt_seed = 1000;

    // ── Thread pool for parallel RRT ───────────────────────────────────────
    const int n_rrt_threads = (n_threads > 0)
        ? n_threads
        : std::max(1, std::min((int)std::thread::hardware_concurrency() - 1, 8));
    ThreadPool rrt_pool(n_rrt_threads);
    fprintf(stderr, "[BRG-ALL] parallel RRT: %d threads\n", n_rrt_threads);

    // ── Persistent id_to_idx ───────────────────────────────────────────────
    std::unordered_map<int, int> id_to_idx;
    id_to_idx.reserve(boxes.size() * 2);
    for (int i = 0; i < static_cast<int>(boxes.size()); ++i)
        id_to_idx[boxes[i].id] = i;

    // ── Persistent UnionFind (initialised from adj) ────────────────────────
    int n = static_cast<int>(boxes.size());
    UnionFind uf(n);
    for (const auto& kv : adj) {
        auto it_u = id_to_idx.find(kv.first);
        if (it_u == id_to_idx.end()) continue;
        for (int nid : kv.second) {
            auto it_v = id_to_idx.find(nid);
            if (it_v != id_to_idx.end())
                uf.unite(it_u->second, it_v->second);
        }
    }

    // Helper: extract islands from UF (O(n), no adj traversal)
    auto get_islands = [&]() {
        std::unordered_map<int, std::vector<int>> groups;
        int sz = uf.size();
        for (int i = 0; i < sz; ++i)
            groups[uf.find(i)].push_back(boxes[i].id);
        std::vector<std::vector<int>> islands;
        islands.reserve(groups.size());
        for (auto& kv : groups)
            islands.push_back(std::move(kv.second));
        return islands;
    };

    // Helper: incrementally absorb newly added boxes into UF + id_to_idx
    auto absorb_new_boxes = [&](int old_n) {
        int new_n = static_cast<int>(boxes.size());
        if (new_n <= old_n) return;
        uf.resize(new_n);
        for (int i = old_n; i < new_n; ++i) {
            id_to_idx[boxes[i].id] = i;
            auto it = adj.find(boxes[i].id);
            if (it != adj.end()) {
                for (int nid : it->second) {
                    auto jt = id_to_idx.find(nid);
                    if (jt != id_to_idx.end())
                        uf.unite(i, jt->second);
                }
            }
        }
    };

    // Outer loop: keep merging until 1 island or budget exhausted
    for (int round = 0; round < 50; ++round) {
        if (deadline_reached()) break;
        if (total_bridges >= max_total_bridges) break;

        auto islands = get_islands();
        if (islands.size() <= 1) {
            fprintf(stderr, "[BRG-ALL] round %d: single component (%d boxes) — done\n",
                    round, islands.empty() ? 0 : (int)islands[0].size());
            break;
        }

        // Sort islands by size descending (largest = main component)
        std::sort(islands.begin(), islands.end(),
                  [](const std::vector<int>& a, const std::vector<int>& b) {
                      return a.size() > b.size();
                  });

        fprintf(stderr, "[BRG-ALL] round %d: %d islands (main=%d",
                round, (int)islands.size(), (int)islands[0].size());
        for (size_t k = 1; k < std::min(islands.size(), (size_t)6); ++k)
            fprintf(stderr, ", #%d=%d", (int)k, (int)islands[k].size());
        fprintf(stderr, ")\n");

        const auto& main_island = islands[0];

        // Pre-compute main island box centers for candidate generation
        // Sample at most 300 boxes from main island to limit O(n*m)
        std::vector<std::pair<int, Eigen::VectorXd>> main_centers;
        {
            int stride = std::max(1, (int)main_island.size() / 300);
            for (int k = 0; k < (int)main_island.size(); k += stride) {
                int mid = main_island[k];
                auto it = id_to_idx.find(mid);
                if (it != id_to_idx.end())
                    main_centers.push_back({mid, boxes[it->second].center()});
            }
        }

        bool merged_any = false;

        // Try to merge each small island into the main component
        for (size_t ii = 1; ii < islands.size(); ++ii) {
            if (deadline_reached()) break;
            if (total_bridges >= max_total_bridges) break;

            const auto& small_island = islands[ii];

            // Build candidate pairs: small × main, sorted by distance
            struct CandPair {
                int small_id, main_id;
                double dist;
                bool operator<(const CandPair& o) const { return dist < o.dist; }
            };

            // Sample at most 200 boxes from small island
            std::vector<CandPair> candidates;
            int s_stride = std::max(1, (int)small_island.size() / 200);
            for (int k = 0; k < (int)small_island.size(); k += s_stride) {
                int sid = small_island[k];
                auto its = id_to_idx.find(sid);
                if (its == id_to_idx.end()) continue;
                Eigen::VectorXd sc = boxes[its->second].center();
                for (const auto& [mid, mc] : main_centers) {
                    double d = (sc - mc).squaredNorm();
                    candidates.push_back({sid, mid, d});
                }
            }
            std::sort(candidates.begin(), candidates.end());

            if ((int)candidates.size() > max_pairs_per_gap)
                candidates.resize(max_pairs_per_gap);

            // ── Phase 1: parallel RRT for all candidates ────────────────────
            struct RRTJob {
                CandPair cp;
                Eigen::VectorXd ca, cb;
                int seed;
            };
            std::vector<RRTJob> jobs;
            jobs.reserve(candidates.size());

            for (int ci = 0; ci < (int)candidates.size(); ++ci) {
                const auto& cp = candidates[ci];
                auto its = id_to_idx.find(cp.small_id);
                auto itm = id_to_idx.find(cp.main_id);
                if (its == id_to_idx.end() || itm == id_to_idx.end()) continue;
                RRTJob job;
                job.cp = cp;
                job.ca = boxes[its->second].center();
                job.cb = boxes[itm->second].center();
                job.seed = rrt_seed++;
                jobs.push_back(std::move(job));
            }

            // Cancellation flag: set when island is merged → remaining
            // queued RRT tasks return immediately, freeing pool workers.
            auto cancel = std::make_shared<std::atomic<bool>>(false);

            // Launch all RRT calls in parallel
            using PathVec = std::vector<Eigen::VectorXd>;
            std::vector<std::future<PathVec>> futures;
            futures.reserve(jobs.size());

            RRTConnectConfig pair_rrt;
            // B6: progressive timeout — lower in early rounds, full later.
            // Easy pairs merge in round 0 quickly; hard pairs get more time later.
            // C4: Reduced timeouts — tree_b saturation at ~26K nodes indicates
            // full exploration; extra time yields diminishing returns.
            double round_timeout;
            int round_max_iters;
            if (round == 0) {
                round_timeout = std::min(per_pair_timeout_ms, 2500.0);
                round_max_iters = 50000;
            } else {
                round_timeout = per_pair_timeout_ms;
                round_max_iters = 80000;
            }
            pair_rrt.timeout_ms = round_timeout;
            pair_rrt.max_iters = round_max_iters;
            pair_rrt.step_size = 0.1;
            pair_rrt.goal_bias = 0.20;
            pair_rrt.segment_resolution = 10;

            for (const auto& job : jobs) {
                futures.push_back(rrt_pool.submit(
                    [&checker, &robot, pair_rrt, cancel,
                     ca = job.ca, cb = job.cb, seed = job.seed]() -> PathVec {
                        if (cancel->load(std::memory_order_relaxed))
                            return {};  // island already merged
                        return rrt_connect(ca, cb, checker, robot,
                                           pair_rrt, seed, cancel);
                    }));
            }

            // ── Phase 2: serial chain_pave for first successful path ────────
            // P1-A: early termination on consecutive RRT failures.
            // When only 2 islands remain, try all pairs (the last gap matters most —
            // even paved=0 RRT connections can provide adjacency updates).
            // With ≥3 islands, skip after 2 consecutive fails and move to next island.
            int consecutive_fails = 0;
            const int max_consecutive_fails =
                (islands.size() <= 2) ? (int)futures.size() : 5;

            for (size_t fi = 0; fi < futures.size(); ++fi) {
                if (deadline_reached()) break;
                if (total_bridges >= max_total_bridges) break;

                auto path = futures[fi].get();
                const auto& cp = jobs[fi].cp;

                if (path.empty()) {
                    ++consecutive_fails;
                    fprintf(stderr, "[BRG-ALL] island #%d pair %d/%d: box %d->%d, "
                            "dist=%.3f, rrt=FAIL (consec=%d/%d)\n",
                            (int)ii, (int)fi + 1, (int)futures.size(),
                            cp.small_id, cp.main_id, std::sqrt(cp.dist),
                            consecutive_fails, max_consecutive_fails);
                    if (consecutive_fails >= max_consecutive_fails) {
                        fprintf(stderr, "[BRG-ALL] island #%d: %d consecutive fails, "
                                "skipping remaining pairs\n",
                                (int)ii, consecutive_fails);
                        cancel->store(true, std::memory_order_relaxed);
                        break;
                    }
                    continue;
                }

                consecutive_fails = 0;  // reset on success

                // Chain-pave from small-island box along the RRT path
                int old_n = static_cast<int>(boxes.size());
                int added = chain_pave_along_path(
                    path, cp.small_id, boxes, lect, obs, n_obs,
                    ffb_config, adj, next_box_id, robot,
                    /*max_chain=*/500, /*max_steps_per_wp=*/15);

                total_bridges += added;

                // Incrementally absorb new boxes into UF
                absorb_new_boxes(old_n);

                // Check merge via O(1) UF query
                auto it_s = id_to_idx.find(small_island[0]);
                auto it_m = id_to_idx.find(main_island[0]);
                bool merged = (it_s != id_to_idx.end() && it_m != id_to_idx.end()
                               && uf.connected(it_s->second, it_m->second));

                fprintf(stderr, "[BRG-ALL] island #%d pair %d/%d: box %d->%d, "
                        "dist=%.3f, rrt=%dwp, paved=%d, merged=%s\n",
                        (int)ii, (int)fi + 1, (int)futures.size(),
                        cp.small_id, cp.main_id, std::sqrt(cp.dist),
                        (int)path.size(), added, merged ? "YES" : "no");

                if (merged) {
                    merged_any = true;
                    cancel->store(true, std::memory_order_relaxed);
                    break;  // Move to next small island
                }
            }
        }

        // If no island was merged in this round, stop
        if (!merged_any) {
            fprintf(stderr, "[BRG-ALL] no island merged in round %d — stopping\n", round);
            break;
        }
    }

    // Final report (O(n) via UF, no full adj traversal)
    auto final_islands = get_islands();
    fprintf(stderr, "[BRG-ALL] done: %d islands remain, %d bridge boxes created\n",
            (int)final_islands.size(), total_bridges);

    return total_bridges;
}

}  // namespace sbf

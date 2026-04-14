// SafeBoxForest v5 — GCS Planner (Phase H3 + Phase L)
#include <sbf/planner/gcs_planner.h>
#include <sbf/planner/dijkstra.h>
#include <sbf/planner/path_extract.h>

#include <algorithm>
#include <cmath>

#ifdef SBF_HAS_DRAKE
#include <drake/geometry/optimization/graph_of_convex_sets.h>
#include <drake/geometry/optimization/hpolyhedron.h>
#include <drake/geometry/optimization/point.h>
#include <drake/solvers/solve.h>
#include <drake/solvers/cost.h>
#include <drake/solvers/solution_result.h>
#endif

namespace sbf {

// ─── Corridor expansion ─────────────────────────────────────────────────────

std::unordered_set<int> expand_corridor(
    const AdjacencyGraph& adj,
    const std::vector<int>& path_boxes,
    int hops,
    int max_size)
{
    std::unordered_set<int> corridor(path_boxes.begin(), path_boxes.end());
    if (hops <= 0) return corridor;

    std::unordered_set<int> frontier = corridor;

    for (int h = 0; h < hops; ++h) {
        std::unordered_set<int> next_frontier;
        for (int box_id : frontier) {
            if (max_size > 0 && (int)corridor.size() >= max_size) break;
            auto it = adj.find(box_id);
            if (it == adj.end()) continue;
            for (int nbr : it->second) {
                if (max_size > 0 && (int)corridor.size() >= max_size) break;
                if (corridor.insert(nbr).second) {
                    next_frontier.insert(nbr);
                }
            }
        }
        if (max_size > 0 && (int)corridor.size() >= max_size) break;
        frontier = std::move(next_frontier);
    }
    return corridor;
}

// ─── Drake GCS implementation ────────────────────────────────────────────────

#ifdef SBF_HAS_DRAKE

using drake::geometry::optimization::GraphOfConvexSets;
using drake::geometry::optimization::HPolyhedron;
using drake::geometry::optimization::Point;

// ─── Adaptive recursive hull safety check (v1-style subdivision) ────────────
// Checks if the region (sub_lb, sub_ub) within the hull is collision-free.
// Member boxes are known-safe; if a sub-region is fully covered by any member,
// we skip it. Otherwise we check collision via LECT tree traversal + AABB.
//
// Strategy:
//   1. If sub-region fully covered by a member box → safe (skip).
//   2. AABB conservative check → if safe, done.
//   3. LECT tree walk: find all LECT nodes whose C-space intervals
//      overlap the query region. For nodes fully INSIDE the query,
//      use their cached link iAABBs (collides_scene). If any overlapping
//      LECT node says safe, the region is safe.
//   4. Subdivide and recurse if budget remains.

/// Walk LECT tree: check if region [qlb, qub] is collision-free
/// by finding LECT nodes whose intervals are fully inside the query region
/// and using their cached envelopes.
/// cur_ivs: the C-space intervals of node_idx (passed down, avoiding recomputation)
/// Returns: true = region safe, false = uncertain/collision
static bool lect_tree_region_safe(
        LECT* lect,
        int node_idx,
        const Obstacle* obs, int n_obs,
        const Eigen::VectorXd& qlb,
        const Eigen::VectorXd& qub,
        std::vector<Interval>& cur_ivs,
        int n_dims,
        int max_depth) {
    // Check overlap between node interval and query region
    for (int d = 0; d < n_dims; ++d) {
        if (cur_ivs[d].hi <= qlb[d] || cur_ivs[d].lo >= qub[d])
            return true;  // no overlap → doesn't affect query region
    }

    // Check if node is fully inside query region
    bool fully_inside = true;
    for (int d = 0; d < n_dims; ++d) {
        if (cur_ivs[d].lo < qlb[d] - 1e-12 || cur_ivs[d].hi > qub[d] + 1e-12) {
            fully_inside = false;
            break;
        }
    }

    if (fully_inside && lect->has_data(node_idx)) {
        // Use cached envelope to check collision
        if (!lect->collides_scene(node_idx, obs, n_obs))
            return true;  // this LECT node says safe
        // Envelope says collision — if leaf, trust it
        if (lect->is_leaf(node_idx))
            return false;
        // Internal node: try children for finer resolution
    }

    // If leaf: can't subdivide further
    if (lect->is_leaf(node_idx) || max_depth <= 0)
        return false;  // conservative: unknown region

    // Recurse into children, passing narrowed intervals
    int left = lect->left(node_idx);
    int right = lect->right(node_idx);
    int sd = lect->get_split_dim(node_idx);
    double sv = lect->split_val(node_idx);

    if (sd < 0 || sd >= n_dims) return false;

    double old_hi = cur_ivs[sd].hi;
    double old_lo = cur_ivs[sd].lo;

    // Left child: cur_ivs[sd].hi = sv
    if (left >= 0) {
        cur_ivs[sd].hi = sv;
        if (!lect_tree_region_safe(lect, left, obs, n_obs, qlb, qub,
                                   cur_ivs, n_dims, max_depth - 1)) {
            cur_ivs[sd].hi = old_hi;
            return false;
        }
        cur_ivs[sd].hi = old_hi;
    }

    // Right child: cur_ivs[sd].lo = sv
    if (right >= 0) {
        cur_ivs[sd].lo = sv;
        if (!lect_tree_region_safe(lect, right, obs, n_obs, qlb, qub,
                                   cur_ivs, n_dims, max_depth - 1)) {
            cur_ivs[sd].lo = old_lo;
            return false;
        }
        cur_ivs[sd].lo = old_lo;
    }

    return true;
}

static bool hull_region_safe(
        const Eigen::VectorXd& sub_lb,
        const Eigen::VectorXd& sub_ub,
        const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& members,
        const CollisionChecker& checker,
        LECT* lect,
        int depth) {
    const int n = sub_lb.size();

    // Check if fully covered by any member box → safe (no check needed)
    for (const auto& [mlb, mub] : members) {
        bool covered = true;
        for (int d = 0; d < n; ++d) {
            if (sub_lb[d] < mlb[d] - 1e-12 || sub_ub[d] > mub[d] + 1e-12) {
                covered = false;
                break;
            }
        }
        if (covered) return true;
    }

    // Build intervals for collision check
    std::vector<Interval> ivs(n);
    for (int d = 0; d < n; ++d) {
        ivs[d].lo = sub_lb[d];
        ivs[d].hi = sub_ub[d];
    }

    // Fast conservative AABB check — if safe, done
    if (!checker.check_box(ivs))
        return true;

    // LECT tree traversal: use cached node envelopes for precise check
    if (lect != nullptr) {
        auto root_ivs = lect->root_intervals();
        if (lect_tree_region_safe(lect, 0,
                checker.obstacles(), checker.n_obs(),
                sub_lb, sub_ub, root_ivs, n, /*max_depth=*/30))
            return true;
    }

    // Still collision → subdivide if budget remains
    if (depth <= 0)
        return false;  // out of subdivision budget

    // Split along widest dimension
    int best_dim = 0;
    double best_w = 0.0;
    for (int d = 0; d < n; ++d) {
        double w = sub_ub[d] - sub_lb[d];
        if (w > best_w) { best_w = w; best_dim = d; }
    }
    double mid = 0.5 * (sub_lb[best_dim] + sub_ub[best_dim]);

    // Left half
    Eigen::VectorXd left_ub = sub_ub;
    left_ub[best_dim] = mid;
    Eigen::VectorXd right_lb = sub_lb;
    right_lb[best_dim] = mid;

    if (!hull_region_safe(sub_lb, left_ub, members, checker, lect, depth - 1))
        return false;
    if (!hull_region_safe(right_lb, sub_ub, members, checker, lect, depth - 1))
        return false;
    return true;
}

GCSResult gcs_plan(
    const AdjacencyGraph& adj,
    const std::vector<BoxNode>& boxes,
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& goal,
    const GCSConfig& config,
    const CollisionChecker* checker,
    LECT* lect)
{
    const int n = static_cast<int>(start.size());

    // 1. Find containing boxes
    int start_box = -1, goal_box = -1;
    for (const auto& b : boxes) {
        if (start_box < 0 && b.contains(start)) start_box = b.id;
        if (goal_box < 0 && b.contains(goal))   goal_box = b.id;
        if (start_box >= 0 && goal_box >= 0) break;
    }
    if (start_box < 0 || goal_box < 0)
        return {false, {}, 0.0};

    // 2. Dijkstra baseline
    auto dij = dijkstra_search(adj, boxes, start_box, goal_box);
    if (!dij.found)
        return {false, {}, 0.0};

    // 3. Shortcut Dijkstra path to remove redundant boxes
    auto shortcut_seq = shortcut_box_sequence(dij.box_sequence, adj);
    fprintf(stderr, "[GCS] dijkstra path: %d boxes → shortcut: %d boxes\n",
            (int)dij.box_sequence.size(), (int)shortcut_seq.size());

    // 4. Build corridor from shortcut seq + fill 2-hop gaps with intermediaries.
    //    For each consecutive pair (A,B) in shortcut_seq that are NOT directly
    //    adjacent, find a common neighbor C to bridge the gap.
    std::vector<int> corridor_seq;  // ordered sequence with gaps filled
    corridor_seq.push_back(shortcut_seq[0]);
    {
        std::unordered_map<int, std::unordered_set<int>> adj_set;
        for (int id : shortcut_seq) {
            auto it = adj.find(id);
            if (it != adj.end())
                adj_set[id] = std::unordered_set<int>(it->second.begin(), it->second.end());
        }

        for (size_t k = 0; k + 1 < shortcut_seq.size(); ++k) {
            int a = shortcut_seq[k];
            int b = shortcut_seq[k + 1];
            bool direct = false;
            auto it_a = adj_set.find(a);
            if (it_a != adj_set.end() && it_a->second.count(b))
                direct = true;
            if (!direct && it_a != adj_set.end()) {
                auto it_b = adj.find(b);
                if (it_b != adj.end()) {
                    for (int nbr : it_a->second) {
                        for (int nbr_b : it_b->second) {
                            if (nbr == nbr_b) {
                                corridor_seq.push_back(nbr);
                                goto found_intermediate;
                            }
                        }
                    }
                }
                found_intermediate:;
            }
            corridor_seq.push_back(b);
        }
    }

    // Deduplicate corridor_seq (preserve order)
    std::vector<int> deduped_seq;
    {
        std::unordered_set<int> seen;
        for (int id : corridor_seq)
            if (seen.insert(id).second) deduped_seq.push_back(id);
    }

    // Build box lookup by id
    std::unordered_map<int, const BoxNode*> box_map;
    for (const auto& b : boxes)
        box_map[b.id] = &b;

    // 4b. Greedy collision-safe coarsening: merge consecutive corridor boxes
    //     into groups whose AABB hull is collision-free.
    //     Each group → one HPolyhedron vertex in GCS.
    //     group_size is the *max* boxes per group; actual groups may be smaller
    //     when the hull would introduce collisions.
    const int max_v = config.max_gcs_verts > 0 ? config.max_gcs_verts - 2 : 0;
    int group_size = 2;  // always merge at least 2:1
    if (max_v > 0 && (int)deduped_seq.size() > max_v) {
        group_size = ((int)deduped_seq.size() + max_v - 1) / max_v;
        if (group_size < 2) group_size = 2;
    }

    // Build groups of consecutive corridor boxes (collision-safe)
    struct CorridorGroup {
        Eigen::VectorXd lb, ub;
        std::unordered_set<int> members;
    };
    std::vector<CorridorGroup> groups;
    std::unordered_map<int, int> box_to_group;
    int n_collision_splits = 0;

    {
        CorridorGroup cur_group;
        cur_group.lb = Eigen::VectorXd::Constant(n, 1e30);
        cur_group.ub = Eigen::VectorXd::Constant(n, -1e30);
        int cur_count = 0;

        for (int i = 0; i < (int)deduped_seq.size(); ++i) {
            int bid = deduped_seq[i];
            auto bit = box_map.find(bid);
            if (bit == box_map.end()) continue;
            const BoxNode& box = *bit->second;

            // Compute tentative hull with this box added
            Eigen::VectorXd tent_lb = cur_group.lb;
            Eigen::VectorXd tent_ub = cur_group.ub;
            for (int d = 0; d < n; ++d) {
                tent_lb[d] = std::min(tent_lb[d], box.joint_intervals[d].lo);
                tent_ub[d] = std::max(tent_ub[d], box.joint_intervals[d].hi);
            }

            bool should_close = (cur_count >= group_size);

            // Collision check on tentative hull via adaptive subdivision.
            // Sub-regions fully covered by member boxes are skipped;
            // only "gap" regions are checked (AABB → LECT → subdivide).
            if (!should_close && checker != nullptr && cur_count > 0) {
                // Collect member box bounds for coverage pruning
                std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> member_bounds;
                for (int mid : cur_group.members) {
                    auto mit = box_map.find(mid);
                    if (mit == box_map.end()) continue;
                    const BoxNode& mb = *mit->second;
                    Eigen::VectorXd mlb(n), mub(n);
                    for (int d = 0; d < n; ++d) {
                        mlb[d] = mb.joint_intervals[d].lo;
                        mub[d] = mb.joint_intervals[d].hi;
                    }
                    member_bounds.push_back({mlb, mub});
                }
                // Add the new box being merged
                {
                    Eigen::VectorXd blb(n), bub(n);
                    for (int d = 0; d < n; ++d) {
                        blb[d] = box.joint_intervals[d].lo;
                        bub[d] = box.joint_intervals[d].hi;
                    }
                    member_bounds.push_back({blb, bub});
                }

                bool hull_safe = hull_region_safe(
                    tent_lb, tent_ub, member_bounds,
                    *checker, lect, /*max_depth=*/8);

                if (!hull_safe) {
                    should_close = true;
                    n_collision_splits++;
                }
            }

            if (should_close && cur_count > 0) {
                // Close current group
                for (int mid : cur_group.members)
                    box_to_group[mid] = (int)groups.size();
                groups.push_back(std::move(cur_group));
                // Reset
                cur_group = CorridorGroup();
                cur_group.lb = Eigen::VectorXd::Constant(n, 1e30);
                cur_group.ub = Eigen::VectorXd::Constant(n, -1e30);
                cur_count = 0;
            }

            // Add box to current group
            for (int d = 0; d < n; ++d) {
                cur_group.lb[d] = std::min(cur_group.lb[d], box.joint_intervals[d].lo);
                cur_group.ub[d] = std::max(cur_group.ub[d], box.joint_intervals[d].hi);
            }
            cur_group.members.insert(bid);
            cur_count++;
        }

        // Flush last group
        if (cur_count > 0) {
            for (int mid : cur_group.members)
                box_to_group[mid] = (int)groups.size();
            groups.push_back(std::move(cur_group));
        }
    }

    int n_groups = (int)groups.size();
    fprintf(stderr, "[GCS] corridor: %d unique → %d groups (max %d:1, %d collision splits)\n",
            (int)deduped_seq.size(), n_groups, group_size, n_collision_splits);

    // 5. Build GCS graph with group vertices
    GraphOfConvexSets gcs;
    std::vector<GraphOfConvexSets::Vertex*> gverts(n_groups);
    for (int gi = 0; gi < n_groups; ++gi) {
        auto hpoly = HPolyhedron::MakeBox(groups[gi].lb, groups[gi].ub);
        gverts[gi] = gcs.AddVertex(hpoly, "g" + std::to_string(gi));
    }

    // 5b. Start/Goal point vertices
    auto* v_start = gcs.AddVertex(Point(start), "start");
    auto* v_goal  = gcs.AddVertex(Point(goal),  "goal");

    // 5c. Start → group containing start_box, goal group → Goal
    int start_gi = box_to_group.at(start_box);
    int goal_gi  = box_to_group.at(goal_box);
    gcs.AddEdge(v_start, gverts[start_gi]);
    gcs.AddEdge(gverts[goal_gi], v_goal);

    // 5d. Edges between groups that share adjacency (from member boxes)
    std::unordered_set<int64_t> group_edge_set;
    for (const auto& [bid, gi] : box_to_group) {
        auto adj_it = adj.find(bid);
        if (adj_it == adj.end()) continue;
        for (int nbr : adj_it->second) {
            auto git = box_to_group.find(nbr);
            if (git == box_to_group.end()) continue;
            int gj = git->second;
            if (gi == gj) continue;
            int lo = std::min(gi, gj), hi = std::max(gi, gj);
            int64_t key = (int64_t)lo * 100000 + hi;
            if (group_edge_set.insert(key).second) {
                gcs.AddEdge(gverts[lo], gverts[hi]);
                gcs.AddEdge(gverts[hi], gverts[lo]);
            }
        }
    }

    // 5e. Edge cost: ||x_u - x_v||₂²
    int n_cost_added = 0;
    for (auto* edge : gcs.Edges()) {
        const auto& xu_vars = edge->xu();
        const auto& xv_vars = edge->xv();
        const int xu_n = static_cast<int>(xu_vars.size());
        const int xv_n = static_cast<int>(xv_vars.size());

        drake::symbolic::Expression cost_expr{0.0};
        const int dim = std::min(xu_n, xv_n);
        for (int d = 0; d < dim; ++d) {
            drake::symbolic::Expression diff = xu_vars[d] - xv_vars[d];
            cost_expr += diff * diff * config.cost_weight_length;
        }
        edge->AddCost(cost_expr);
        ++n_cost_added;
    }

    fprintf(stderr, "[GCS] graph: %d verts, %d edges — solving...\n",
            (int)gcs.Vertices().size(), (int)gcs.Edges().size());
    fflush(stderr);

    // 6. Solve
    drake::geometry::optimization::GraphOfConvexSetsOptions gcs_opts;
    gcs_opts.convex_relaxation = config.convex_relaxation;
    auto result = gcs.SolveShortestPath(*v_start, *v_goal, gcs_opts);
    fprintf(stderr, "[GCS] solve done, success=%d, solver=%s, status=%s\n",
            (int)result.is_success(),
            result.get_solver_id().name().c_str(),
            result.get_solution_result() == drake::solvers::SolutionResult::kSolutionFound
            ? "found" : "other");
    fflush(stderr);

    if (!result.is_success()) {
        return {false, {}, 0.0};
    }

    // 7. Extract path: one waypoint per group vertex (s→t order),
    //    then simplify by removing collinear/near-collinear points.
    std::vector<Eigen::VectorXd> raw_pts;
    raw_pts.push_back(start);
    for (int gi = 0; gi < n_groups; ++gi) {
        if (gi == start_gi || gi == goal_gi) continue;
        Eigen::VectorXd wp = result.GetSolution(gverts[gi]->x());
        raw_pts.push_back(wp);
    }
    raw_pts.push_back(goal);

    // 8. Collinear simplification (deviation < 0.10 rad in C-space)
    std::vector<Eigen::VectorXd> path;
    path.push_back(raw_pts[0]);
    size_t cur = 0;
    while (cur < raw_pts.size() - 1) {
        size_t best = cur + 1;
        for (size_t j = raw_pts.size() - 1; j > cur + 1; --j) {
            Eigen::VectorXd dir = raw_pts[j] - raw_pts[cur];
            double seg_len = dir.norm();
            if (seg_len < 1e-10) { best = j; break; }
            dir /= seg_len;
            double max_dev = 0.0;
            for (size_t k = cur + 1; k < j; ++k) {
                Eigen::VectorXd diff = raw_pts[k] - raw_pts[cur];
                double proj = diff.dot(dir);
                double dev = (diff - proj * dir).norm();
                max_dev = std::max(max_dev, dev);
            }
            if (max_dev < 0.10) { best = j; break; }
        }
        path.push_back(raw_pts[best]);
        cur = best;
    }

    double cost = result.get_optimal_cost();
    fprintf(stderr, "[GCS] extracted: %d → %d waypoints (%d groups), cost=%.3f\n",
            (int)raw_pts.size(), (int)path.size(), n_groups, cost);

    // 9. Post-hoc segment collision check (if checker available)
    //    When GCS-optimized path has collisions (common because solver
    //    places waypoints at convex-hull edges), fall back to a face-center
    //    path along the Dijkstra box sequence, then simplify with
    //    collision-checked shortcutting.
    if (checker != nullptr) {
        int n_col_segs = 0;
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            double slen = (path[i+1] - path[i]).norm();
            int res = std::max(20, (int)std::ceil(slen / 0.02));
            if (checker->check_segment(path[i], path[i+1], res))
                n_col_segs++;
        }
        if (n_col_segs > 0) {
            // Build face-center path from the original Dijkstra sequence
            // (whose consecutive boxes are guaranteed adjacent).
            auto face_path = extract_waypoints(
                dij.box_sequence, boxes, start, goal);

            // Greedy collision-free simplification: skip ahead as far as
            // possible while the shortcut segment is collision-free.
            std::vector<Eigen::VectorXd> cpath;
            cpath.push_back(face_path[0]);
            size_t ccur = 0;
            while (ccur < face_path.size() - 1) {
                size_t cbest = ccur + 1;
                for (size_t j = face_path.size() - 1; j > ccur + 1; --j) {
                    double slen = (face_path[j] - face_path[ccur]).norm();
                    int res = std::max(20, (int)std::ceil(slen / 0.02));
                    if (!checker->check_segment(face_path[ccur],
                                                face_path[j], res)) {
                        cbest = j;
                        break;
                    }
                }
                cpath.push_back(face_path[cbest]);
                ccur = cbest;
            }

            fprintf(stderr, "[GCS] post-hoc: %d/%d col → face-path %d pts\n",
                    n_col_segs, (int)(path.size() - 1), (int)cpath.size());

            return {true, std::move(cpath), cost, deduped_seq};
        }
    }

    return {true, std::move(path), cost, deduped_seq};
}

#endif  // SBF_HAS_DRAKE

// ─── Fallback (no Drake) ────────────────────────────────────────────────────

GCSResult gcs_plan_fallback(
    const AdjacencyGraph& adj,
    const std::vector<BoxNode>& boxes,
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& goal)
{
    GCSResult result;

    // Find start/goal containing boxes
    int start_id = -1, goal_id = -1;
    for (const auto& b : boxes) {
        if (start_id < 0 && b.contains(start)) start_id = b.id;
        if (goal_id < 0 && b.contains(goal))   goal_id = b.id;
        if (start_id >= 0 && goal_id >= 0) break;
    }
    if (start_id < 0 || goal_id < 0) return result;

    auto dij = dijkstra_search(adj, boxes, start_id, goal_id, goal);
    if (!dij.found) return result;

    result.found = true;
    result.path = extract_waypoints(dij.box_sequence, boxes, start, goal);
    result.cost = dij.total_cost;
    return result;
}

}  // namespace sbf

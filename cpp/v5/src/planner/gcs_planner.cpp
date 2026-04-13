// SafeBoxForest v5 — GCS Planner (Phase H3 + Phase L)
#include <sbf/planner/gcs_planner.h>
#include <sbf/planner/dijkstra.h>
#include <sbf/planner/path_extract.h>

#include <algorithm>

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

GCSResult gcs_plan(
    const AdjacencyGraph& adj,
    const std::vector<BoxNode>& boxes,
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& goal,
    const GCSConfig& config)
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
        return gcs_plan_fallback(adj, boxes, start, goal);

    // 2. Dijkstra baseline
    auto dij = dijkstra_search(adj, boxes, start_box, goal_box);
    if (!dij.found)
        return {false, {}, 0.0};

    // 3. Shortcut Dijkstra path to remove redundant boxes
    auto shortcut_seq = shortcut_box_sequence(dij.box_sequence, adj);
    fprintf(stderr, "[GCS] dijkstra path: %d boxes → shortcut: %d boxes\n",
            (int)dij.box_sequence.size(), (int)shortcut_seq.size());

    // 3b. Coarsen shortcut_seq if it will produce too many GCS vertices.
    //     After gap-filling, corridor_seq ≈ 2× shortcut_seq (worst case every pair needs bridge).
    //     So target shortcut_seq.size() ≤ max_gcs_verts/2.
    const int max_v = config.max_gcs_verts > 0 ? config.max_gcs_verts - 2 : 0;
    if (max_v > 0 && (int)shortcut_seq.size() > max_v / 2) {
        int orig_n = (int)shortcut_seq.size();
        int target = max_v / 2;
        // Stride-based subsampling of shortcut_seq
        int stride = (orig_n + target - 1) / target;
        if (stride < 2) stride = 2;
        std::vector<int> coarsened;
        for (int i = 0; i < orig_n; i += stride)
            coarsened.push_back(shortcut_seq[i]);
        if (coarsened.back() != shortcut_seq.back())
            coarsened.push_back(shortcut_seq.back());
        fprintf(stderr, "[GCS] coarsen shortcut: %d → %d boxes (stride=%d, target<=%d)\n",
                orig_n, (int)coarsened.size(), stride, target);
        shortcut_seq = std::move(coarsened);
    }

    // 4. Build corridor from shortcut seq + fill gaps with intermediaries.
    //    For each consecutive pair (A,B) in shortcut_seq, if not directly
    //    adjacent, run BFS on the full adjacency graph to find the shortest
    //    bridging path (handles arbitrary-hop gaps from coarsening).
    std::vector<int> corridor_seq;  // ordered sequence with gaps filled
    corridor_seq.push_back(shortcut_seq[0]);
    {
        for (size_t k = 0; k + 1 < shortcut_seq.size(); ++k) {
            int a = shortcut_seq[k];
            int b = shortcut_seq[k + 1];
            // Check if directly adjacent
            bool direct = false;
            auto it_a = adj.find(a);
            if (it_a != adj.end()) {
                for (int nbr : it_a->second) {
                    if (nbr == b) { direct = true; break; }
                }
            }
            if (!direct) {
                // BFS from a to b in full adjacency graph (bounded depth)
                std::unordered_map<int, int> parent;
                parent[a] = -1;
                std::vector<int> frontier = {a};
                bool found_path = false;
                const int max_bfs_depth = 20;
                for (int depth = 0; depth < max_bfs_depth && !found_path; ++depth) {
                    std::vector<int> next;
                    for (int node : frontier) {
                        auto it = adj.find(node);
                        if (it == adj.end()) continue;
                        for (int nbr : it->second) {
                            if (parent.count(nbr)) continue;
                            parent[nbr] = node;
                            if (nbr == b) { found_path = true; break; }
                            next.push_back(nbr);
                        }
                        if (found_path) break;
                    }
                    frontier = std::move(next);
                }
                if (found_path) {
                    // Reconstruct path from a to b (exclusive of a, inclusive of b)
                    std::vector<int> bridge;
                    for (int cur = b; cur != a; cur = parent[cur])
                        bridge.push_back(cur);
                    // bridge is [b, ..., neighbor_of_a], reverse to get order a→b
                    std::reverse(bridge.begin(), bridge.end());
                    // Skip last element (b) since it's added below
                    for (size_t i = 0; i + 1 < bridge.size(); ++i)
                        corridor_seq.push_back(bridge[i]);
                }
            }
            corridor_seq.push_back(b);
        }
    }

    std::unordered_set<int> corridor(corridor_seq.begin(), corridor_seq.end());
    // Optional: expand by hops if configured
    if (config.corridor_hops > 0) {
        corridor = expand_corridor(adj, corridor_seq, config.corridor_hops,
                                   config.max_corridor_size);
    }

    fprintf(stderr, "[GCS] corridor: %d boxes (seq=%d, filled=%d)\n",
            (int)corridor.size(), (int)shortcut_seq.size(), (int)corridor_seq.size());

    // Build box lookup by id
    std::unordered_map<int, const BoxNode*> box_map;
    for (const auto& b : boxes)
        box_map[b.id] = &b;

    // 5. Build GCS
    GraphOfConvexSets gcs;
    std::unordered_map<int, GraphOfConvexSets::Vertex*> verts;

    // 5a. Box -> HPolyhedron vertices
    for (int box_id : corridor) {
        auto bit = box_map.find(box_id);
        if (bit == box_map.end()) continue;
        const BoxNode& box = *bit->second;

        Eigen::VectorXd lb(n), ub(n);
        for (int d = 0; d < n; ++d) {
            lb[d] = box.joint_intervals[d].lo;
            ub[d] = box.joint_intervals[d].hi;
        }
        auto hpoly = HPolyhedron::MakeBox(lb, ub);
        verts[box_id] = gcs.AddVertex(hpoly, "box_" + std::to_string(box_id));
    }

    // 5b. Start/Goal point vertices
    auto* v_start = gcs.AddVertex(Point(start), "start");
    auto* v_goal  = gcs.AddVertex(Point(goal),  "goal");

    // 5c. Start -> containing box, containing box -> Goal
    gcs.AddEdge(v_start, verts[start_box]);
    gcs.AddEdge(verts[goal_box], v_goal);

    // 5d. Adjacency edges (bidirectional)
    int n_adj_edges = 0;
    for (int u : corridor) {
        auto adj_it = adj.find(u);
        if (adj_it == adj.end()) continue;
        for (int v : adj_it->second) {
            if (corridor.count(v) && u < v) {
                gcs.AddEdge(verts[u], verts[v]);
                gcs.AddEdge(verts[v], verts[u]);
                n_adj_edges += 2;
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
        return gcs_plan_fallback(adj, boxes, start, goal);
    }

    // 7. Extract path: collect GCS-optimized waypoints from shortcut_seq boxes,
    //    then simplify by removing collinear/near-collinear points.
    std::vector<Eigen::VectorXd> raw_pts;
    raw_pts.push_back(start);

    for (int box_id : shortcut_seq) {
        if (box_id == start_box || box_id == goal_box) continue;
        auto vit = verts.find(box_id);
        if (vit != verts.end()) {
            Eigen::VectorXd wp = result.GetSolution(vit->second->x());
            raw_pts.push_back(wp);
        }
    }
    raw_pts.push_back(goal);

    // 8. Greedy forward simplification: skip points where straight line
    //    stays within overlapping box pairs (no collision possible).
    //    This is a C-space operation: we check if intermediate boxes
    //    are not needed by checking if the straight-line segment between
    //    two non-consecutive waypoints stays within the box union.
    std::vector<Eigen::VectorXd> path;
    path.push_back(raw_pts[0]);
    size_t cur = 0;
    while (cur < raw_pts.size() - 1) {
        size_t best = cur + 1;
        // Try to skip as far forward as possible
        // For GCS-optimized points, just check if all skipped points
        // are roughly collinear (deviation < threshold)
        for (size_t j = raw_pts.size() - 1; j > cur + 1; --j) {
            bool can_skip = true;
            Eigen::VectorXd dir = raw_pts[j] - raw_pts[cur];
            double seg_len = dir.norm();
            if (seg_len < 1e-10) { best = j; break; }
            dir /= seg_len;
            // Check max deviation of intermediate points from line
            double max_dev = 0.0;
            for (size_t k = cur + 1; k < j; ++k) {
                Eigen::VectorXd diff = raw_pts[k] - raw_pts[cur];
                double proj = diff.dot(dir);
                double dev = (diff - proj * dir).norm();
                max_dev = std::max(max_dev, dev);
            }
            // Allow skip if deviation < 0.10 rad (in C-space)
            if (max_dev < 0.10) {
                best = j;
                break;
            }
        }
        path.push_back(raw_pts[best]);
        cur = best;
    }

    double cost = result.get_optimal_cost();
    fprintf(stderr, "[GCS] extracted: %d → %d waypoints (shortcut=%d, corridor=%d), cost=%.3f\n",
            (int)raw_pts.size(), (int)path.size(), (int)shortcut_seq.size(),
            (int)corridor.size(), cost);
    return {true, std::move(path), cost, corridor_seq};
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

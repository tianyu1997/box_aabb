#!/usr/bin/env python3
"""
gcs_query_v2.py — SBF forest → Chain SOCP 最短路径求解

读取 C++ exp2_e2e_planning 导出的 paths.json,
运行 Dijkstra → Chain SOCP (全 backbone, 无需子采样) → Shortcut smoothing.

相比 gcs_query.py (Drake GCS + rounding):
  - Chain SOCP 对线性 box 链是精确全局最优 (无 relaxation/rounding 误差)
  - 直接用 MOSEK SOCP, 无需 GraphOfConvexSets 开销
  - 可处理完整 ~470 box backbone (MOSEK 高效处理 SOCP)

Usage:
    python gcs_query_v2.py <paths.json> [--verbose]
"""

import argparse
import heapq
import json
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

from pydrake.solvers import MathematicalProgram, MosekSolver


# ═══════════════════════════════════════════════════════════════════════════
# Geometry helpers
# ═══════════════════════════════════════════════════════════════════════════

def find_containing_rows(q, lo, hi, tol=1e-6):
    """Return row indices of boxes containing q."""
    return list(np.where(np.all((q >= lo - tol) & (q <= hi + tol), axis=1))[0])


# ═══════════════════════════════════════════════════════════════════════════
# Dijkstra on box adjacency
# ═══════════════════════════════════════════════════════════════════════════

def dijkstra_box_path(lo, hi, ids, adj, q_start, q_goal):
    """Dijkstra shortest path on box centers. Returns (path_ids, info)."""
    t0 = time.perf_counter()
    N = len(ids)
    id2row = {bid: i for i, bid in enumerate(ids)}
    centers = 0.5 * (lo + hi)

    # Start/goal candidate boxes: containing + K-nearest
    diff_s = np.max(np.maximum(np.maximum(lo - q_start, 0),
                               np.maximum(q_start - hi, 0)), axis=1)
    diff_g = np.max(np.maximum(np.maximum(lo - q_goal, 0),
                               np.maximum(q_goal - hi, 0)), axis=1)
    start_rows = set(find_containing_rows(q_start, lo, hi))
    goal_rows = set(find_containing_rows(q_goal, lo, hi))
    start_rows |= set(np.argsort(diff_s)[:20].tolist())
    goal_rows |= set(np.argsort(diff_g)[:20].tolist())

    start_ids = {ids[r] for r in start_rows if r < N}
    goal_ids = {ids[r] for r in goal_rows if r < N}

    dist, prev, pq = {}, {}, []
    for sid in start_ids:
        r = id2row[sid]
        d = float(np.linalg.norm(q_start - centers[r]))
        dist[sid] = d
        heapq.heappush(pq, (d, sid))

    found = None
    while pq:
        d_u, u = heapq.heappop(pq)
        if d_u > dist.get(u, float("inf")):
            continue
        if u in goal_ids:
            found = u
            break
        for v in adj.get(u, []):
            if v not in id2row:
                continue
            w = float(np.linalg.norm(centers[id2row[u]] - centers[id2row[v]]))
            d_v = d_u + w
            if d_v < dist.get(v, float("inf")):
                dist[v] = d_v
                prev[v] = u
                heapq.heappush(pq, (d_v, v))

    dt = time.perf_counter() - t0
    if found is None:
        return None, {"time": dt, "error": "no path"}

    path_ids = []
    cur = found
    while cur is not None:
        path_ids.append(cur)
        cur = prev.get(cur)
    path_ids.reverse()
    return path_ids, {"time": dt, "n_boxes": len(path_ids)}


def bfs_box_path(lo, hi, ids, adj, q_start, q_goal):
    """BFS shortest path (minimum hops) on box adjacency. Returns (path_ids, info)."""
    from collections import deque
    t0 = time.perf_counter()
    N = len(ids)
    id2row = {bid: i for i, bid in enumerate(ids)}

    # Start/goal candidate boxes
    diff_s = np.max(np.maximum(np.maximum(lo - q_start, 0),
                               np.maximum(q_start - hi, 0)), axis=1)
    diff_g = np.max(np.maximum(np.maximum(lo - q_goal, 0),
                               np.maximum(q_goal - hi, 0)), axis=1)
    start_rows = set(find_containing_rows(q_start, lo, hi))
    goal_rows = set(find_containing_rows(q_goal, lo, hi))
    start_rows |= set(np.argsort(diff_s)[:20].tolist())
    goal_rows |= set(np.argsort(diff_g)[:20].tolist())

    start_ids = {ids[r] for r in start_rows if r < N}
    goal_ids = {ids[r] for r in goal_rows if r < N}

    # BFS
    prev = {}
    visited = set()
    queue = deque()
    for sid in start_ids:
        visited.add(sid)
        queue.append(sid)

    found = None
    while queue:
        u = queue.popleft()
        if u in goal_ids:
            found = u
            break
        for v in adj.get(u, []):
            if v not in visited and v in id2row:
                visited.add(v)
                prev[v] = u
                queue.append(v)

    dt = time.perf_counter() - t0
    if found is None:
        return None, {"time": dt, "error": "no path"}

    path_ids = []
    cur = found
    while cur is not None:
        path_ids.append(cur)
        cur = prev.get(cur)
    path_ids.reverse()
    return path_ids, {"time": dt, "n_boxes": len(path_ids)}


def astar_box_path(lo, hi, ids, adj, q_start, q_goal):
    """A* with goal-directed heuristic favoring boxes closer to goal."""
    t0 = time.perf_counter()
    N = len(ids)
    id2row = {bid: i for i, bid in enumerate(ids)}
    centers = 0.5 * (lo + hi)

    # Precompute goal distances for heuristic
    goal_dists = np.linalg.norm(centers - q_goal, axis=1)

    # Start/goal candidate boxes
    diff_s = np.max(np.maximum(np.maximum(lo - q_start, 0),
                               np.maximum(q_start - hi, 0)), axis=1)
    diff_g = np.max(np.maximum(np.maximum(lo - q_goal, 0),
                               np.maximum(q_goal - hi, 0)), axis=1)
    start_rows = set(find_containing_rows(q_start, lo, hi))
    goal_rows = set(find_containing_rows(q_goal, lo, hi))
    start_rows |= set(np.argsort(diff_s)[:20].tolist())
    goal_rows |= set(np.argsort(diff_g)[:20].tolist())

    start_ids = {ids[r] for r in start_rows if r < N}
    goal_ids = {ids[r] for r in goal_rows if r < N}

    # A* with edge weight = 1 (hop count) + heuristic = L2 to goal
    # This finds minimum-hop paths that prefer boxes closer to goal
    g_score, prev, pq = {}, {}, []
    for sid in start_ids:
        r = id2row[sid]
        g = 1  # hop count from start
        h = goal_dists[r]
        g_score[sid] = g
        heapq.heappush(pq, (g + h, sid))

    found = None
    while pq:
        f_u, u = heapq.heappop(pq)
        if u in goal_ids:
            found = u
            break
        g_u = g_score.get(u, float("inf"))
        if f_u > g_u + goal_dists[id2row[u]] + 1e-6:
            continue
        for v in adj.get(u, []):
            if v not in id2row:
                continue
            g_v = g_u + 1  # hop weight = 1
            if g_v < g_score.get(v, float("inf")):
                g_score[v] = g_v
                prev[v] = u
                h_v = goal_dists[id2row[v]]
                heapq.heappush(pq, (g_v + h_v, v))

    dt = time.perf_counter() - t0
    if found is None:
        return None, {"time": dt, "error": "no path"}

    path_ids = []
    cur = found
    while cur is not None:
        path_ids.append(cur)
        cur = prev.get(cur)
    path_ids.reverse()
    return path_ids, {"time": dt, "n_boxes": len(path_ids)}


# ═══════════════════════════════════════════════════════════════════════════
# Chain SOCP — optimal path through linear chain of boxes
# ═══════════════════════════════════════════════════════════════════════════

def chain_socp_solve(lo_chain, hi_chain, q_start, q_goal):
    """
    Solve min Σ ||x_i - x_{i+1}||_2 s.t. lo_i ≤ x_i ≤ hi_i.

    This is a second-order cone program (SOCP), solved exactly by MOSEK.
    For a linear chain of boxes, this gives the globally optimal path.

    Args:
        lo_chain: (n, D) box lower bounds along the chain
        hi_chain: (n, D) box upper bounds along the chain
        q_start: (D,) start configuration (fixed)
        q_goal: (D,) goal configuration (fixed)

    Returns:
        (path, info) where path is (n+2, D) array [start, x0, ..., xn-1, goal]
    """
    n = len(lo_chain)
    D = lo_chain.shape[1]

    mp = MathematicalProgram()

    # Decision variables: one waypoint per box
    x = [mp.NewContinuousVariables(D, f"x{i}") for i in range(n)]

    # Box constraints
    for i in range(n):
        mp.AddBoundingBoxConstraint(lo_chain[i], hi_chain[i], x[i])

    # L2 norm costs (each becomes a Lorentz cone constraint internally)
    I_D = np.eye(D)

    # Segment 0: ||x[0] - q_start||
    mp.AddL2NormCost(I_D, -q_start, x[0])

    # Segments 1..n-1: ||x[i] - x[i-1]||
    A_mid = np.hstack([I_D, -I_D])  # (D, 2D)
    b_zero = np.zeros(D)
    for i in range(1, n):
        v = np.concatenate([x[i], x[i - 1]])
        mp.AddL2NormCost(A_mid, b_zero, v)

    # Segment n: ||q_goal - x[n-1]||
    mp.AddL2NormCost(I_D, -q_goal, x[n - 1])

    # Solve with MOSEK
    t0 = time.perf_counter()
    solver = MosekSolver()
    result = solver.Solve(mp)
    dt = time.perf_counter() - t0

    if not result.is_success():
        return None, {"success": False, "solver_time": dt, "error": "SOCP failed"}

    # Extract waypoints
    pts = [q_start.copy()]
    for i in range(n):
        pts.append(result.GetSolution(x[i]))
    pts.append(q_goal.copy())
    path = np.array(pts)
    path_len = float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))

    return path, {
        "success": True,
        "solver_time": dt,
        "path_length": path_len,
        "optimal_cost": float(result.get_optimal_cost()),
        "n_raw_waypoints": n + 2,
    }


# ═══════════════════════════════════════════════════════════════════════════
# Shortcut smoother
# ═══════════════════════════════════════════════════════════════════════════

def simplify_path(waypoints, deviation_tol=1e-3):
    """Remove near-collinear waypoints that don't add steering.
    
    Keeps waypoints where the path deviates by > deviation_tol from
    the straight line between kept neighbors.
    """
    if len(waypoints) <= 2:
        return waypoints
    
    keep = [0]
    i = 0
    while i < len(waypoints) - 1:
        # Find farthest j such that all intermediate points are within
        # deviation_tol of the line i→j
        best_j = i + 1
        for j in range(i + 2, len(waypoints)):
            # Check if all points i+1..j-1 are close to line i→j
            p0 = waypoints[i]
            pj = waypoints[j]
            d = pj - p0
            d_len = np.linalg.norm(d)
            if d_len < 1e-15:
                continue
            d_hat = d / d_len
            
            ok = True
            for k in range(i + 1, j):
                pk = waypoints[k]
                # Distance from pk to line p0→pj
                t = np.dot(pk - p0, d_hat)
                proj = p0 + t * d_hat
                dist = np.linalg.norm(pk - proj)
                if dist > deviation_tol:
                    ok = False
                    break
            if ok:
                best_j = j
            else:
                break
        keep.append(best_j)
        i = best_j
    
    return [waypoints[k] for k in keep]


# ═══════════════════════════════════════════════════════════════════════════
# Full solve pipeline
# ═══════════════════════════════════════════════════════════════════════════

def solve_query(lo, hi, ids, adj, q_start, q_goal, verbose=False):
    """
    Full pipeline:
      1. Dijkstra → full backbone
      2. Chain SOCP → globally optimal path through backbone boxes (MOSEK)
      3. Simplify path (remove near-collinear waypoints)

    Returns: (path, info)
    """
    D = lo.shape[1]
    id2row = {bid: i for i, bid in enumerate(ids)}
    info = {"success": False}

    # ── Step 1: Dijkstra ──
    path_ids, dij_info = dijkstra_box_path(lo, hi, ids, adj, q_start, q_goal)
    info["dijkstra_time"] = dij_info.get("time", 0)
    if path_ids is None:
        info["error"] = "dijkstra failed"
        return None, info
    info["n_dijkstra_boxes"] = len(path_ids)

    # ── Step 1.5: Project start/goal into first/last box if outside ──
    real_start, real_goal = q_start.copy(), q_goal.copy()
    start_projected = goal_projected = False
    EPS = 1e-6
    if not find_containing_rows(q_start, lo, hi):
        r0 = id2row[path_ids[0]]
        q_start = np.clip(q_start, lo[r0] + EPS, hi[r0] - EPS)
        start_projected = True
    if not find_containing_rows(q_goal, lo, hi):
        rn = id2row[path_ids[-1]]
        q_goal = np.clip(q_goal, lo[rn] + EPS, hi[rn] - EPS)
        goal_projected = True

    # ── Step 2: Chain SOCP ──
    chain_rows = [id2row[bid] for bid in path_ids]
    chain_lo = lo[chain_rows]
    chain_hi = hi[chain_rows]

    path, socp_info = chain_socp_solve(chain_lo, chain_hi, q_start, q_goal)
    info["solver_time"] = socp_info.get("solver_time", 0)

    if path is None:
        info["error"] = socp_info.get("error", "SOCP failed")
        return None, info

    info["socp_optimal_cost"] = socp_info.get("optimal_cost", 0)
    info["n_raw_waypoints"] = socp_info.get("n_raw_waypoints", 0)
    info["socp_path_length"] = socp_info.get("path_length", 0)

    # Prepend/append real start/goal if projected
    if start_projected:
        path = np.vstack([real_start.reshape(1, -1), path])
    if goal_projected:
        path = np.vstack([path, real_goal.reshape(1, -1)])

    # ── Step 3: Simplify path ──
    waypoints = list(path)
    n_before = len(waypoints)

    t_smooth = time.perf_counter()
    waypoints = simplify_path(waypoints, deviation_tol=0.01)
    dt_smooth = time.perf_counter() - t_smooth

    path = np.array(waypoints)
    path_len = float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))

    info.update({
        "success": True,
        "path_length": path_len,
        "n_before_shortcut": n_before,
        "n_waypoints": len(waypoints),
        "smooth_time": dt_smooth,
    })
    return path, info


# ═══════════════════════════════════════════════════════════════════════════
# Load from paths.json + batch solve
# ═══════════════════════════════════════════════════════════════════════════

def load_forest_from_json(json_path: str) -> dict:
    """Load boxes + adjacency + queries from exp2 paths.json."""
    with open(json_path) as f:
        data = json.load(f)

    boxes = data["boxes"]
    N = len(boxes)
    D = data["dof"]

    lo = np.zeros((N, D))
    hi = np.zeros((N, D))
    ids = []
    for i, b in enumerate(boxes):
        lo[i] = b["lo"]
        hi[i] = b["hi"]
        ids.append(b["id"])

    adj: Dict[int, List[int]] = {}
    for k, v in data.get("adjacency", {}).items():
        adj[int(k)] = v

    queries = []
    for q in data.get("queries", []):
        queries.append({
            "label": q["label"],
            "start": np.array(q["start"]),
            "goal": np.array(q["goal"]),
        })

    # Also load C++ path lengths for comparison
    cpp_paths = {}
    for p in data.get("paths", []):
        cpp_paths[p["pair_idx"]] = {
            "success": p["success"],
            "path_length": p["path_length"],
            "n_waypoints": len(p.get("waypoints", [])),
        }

    return {
        "lo": lo, "hi": hi, "ids": ids, "adj": adj,
        "queries": queries, "cpp_paths": cpp_paths,
        "robot": data.get("robot", "?"), "dof": D,
        "n_boxes": N,
    }


def main():
    parser = argparse.ArgumentParser(
        description="SBF → Chain SOCP path optimization (MOSEK)")
    parser.add_argument("input_json", help="paths.json from C++ exp2")
    parser.add_argument("--verbose", "-v", action="store_true")
    parser.add_argument("--output", type=str, default=None)
    args = parser.parse_args()

    forest = load_forest_from_json(args.input_json)
    print(f"Robot: {forest['robot']}, DOF={forest['dof']}, "
          f"Boxes={forest['n_boxes']}, Queries={len(forest['queries'])}")

    lo, hi, ids, adj = forest["lo"], forest["hi"], forest["ids"], forest["adj"]
    results = []

    print(f"\n{'Pair':<10} {'C++len':>8} {'SOCP':>8} {'Ratio':>6} "
          f"{'Dijk':>6} {'Solve':>7} {'Simp':>6} {'Bkbn':>5} {'Pts':>4}")
    print("-" * 72)

    total_time = 0.0

    for qi, qp in enumerate(forest["queries"]):
        label = qp["label"]
        start, goal = qp["start"], qp["goal"]

        t0 = time.perf_counter()
        path, info = solve_query(
            lo, hi, ids, adj, start, goal,
            verbose=args.verbose,
        )
        wall_time = time.perf_counter() - t0
        total_time += wall_time

        cpp = forest["cpp_paths"].get(qi, {})
        cpp_len = cpp.get("path_length", 0)

        if info.get("success"):
            socp_len = info["path_length"]
            ratio = socp_len / cpp_len if cpp_len > 0 else 0
            dijk_ms = info["dijkstra_time"] * 1000
            solve_ms = info["solver_time"] * 1000
            simp_ms = info.get("smooth_time", 0) * 1000
            n_bkbn = info.get("n_dijkstra_boxes", 0)
            n_pts = info["n_waypoints"]

            print(f"{label:<10} {cpp_len:8.3f} {socp_len:8.3f} {ratio:6.2f} "
                  f"{dijk_ms:5.1f}ms {solve_ms:6.1f}ms {simp_ms:5.1f}ms "
                  f"{n_bkbn:5d} {n_pts:4d}")

            if args.verbose:
                print(f"    raw={info['n_raw_waypoints']}, "
                      f"before={info['n_before_shortcut']}, after={n_pts}, "
                      f"socp_len={info.get('socp_path_length', 0):.3f}, "
                      f"final_len={socp_len:.3f}")

            results.append({
                "pair_idx": qi, "label": label, "success": True,
                "socp_path_length": socp_len, "cpp_path_length": cpp_len,
                "ratio": ratio, "solver_time": info["solver_time"],
                "dijkstra_time": info["dijkstra_time"],
                "n_backbone": n_bkbn, "n_waypoints": n_pts,
                "wall_time": wall_time,
                "waypoints": path.tolist() if path is not None else [],
            })
        else:
            print(f"{label:<10} {cpp_len:8.3f}     FAIL  — {info.get('error', '?')}")
            results.append({
                "pair_idx": qi, "label": label, "success": False,
                "error": info.get("error", "unknown"),
                "wall_time": wall_time,
            })

    n_ok = sum(1 for r in results if r.get("success"))
    print("-" * 72)
    print(f"Success: {n_ok}/{len(results)}, Total time: {total_time:.3f}s "
          f"({total_time/max(len(results),1)*1000:.0f}ms/query avg)")

    # Save results
    out_path = args.output
    if out_path is None:
        out_path = str(Path(args.input_json).parent / "socp_query_results.json")
    with open(out_path, "w") as f:
        json.dump({"results": results, "total_time": total_time}, f, indent=2)
    print(f"Saved → {out_path}")


if __name__ == "__main__":
    main()

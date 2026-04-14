#!/usr/bin/env python3
"""
gcs_query.py — SBF forest → Python GCS query pipeline

读取 C++ exp2_e2e_planning 导出的 paths.json (含 boxes + adjacency + queries),
在 Python 端运行 Dijkstra + GCS (pydrake + MOSEK) 最短路径求解.

Key features (from gcs_optimize_v2.py):
  - Backbone subsampling for long Dijkstra paths (9098 boxes → ~40 backbone)
  - Flow-based waypoint extraction (max-phi greedy walk)
  - Shortcut smoothing (skip redundant intermediate points)
  - Automatic corridor_hops fallback (2 → 1 → 0)

Usage:
    python gcs_query.py <paths.json> [--corridor-hops N] [--max-backbone M]

Requirements:
    pip install drake mosek
"""

import argparse
import heapq
import json
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

import numpy as np

from pydrake.geometry.optimization import (
    GraphOfConvexSets,
    GraphOfConvexSetsOptions,
    HPolyhedron,
    Point,
)
from pydrake.solvers import Binding, Cost, L2NormCost


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


# ═══════════════════════════════════════════════════════════════════════════
# Backbone subsampling (critical for 9000+ box forests)
# ═══════════════════════════════════════════════════════════════════════════

def subsample_backbone(path_ids, id2row, lo, hi, max_boxes=40):
    """Reduce a long Dijkstra backbone, keeping first/last + regular + width-jump points."""
    n = len(path_ids)
    if n <= max_boxes:
        return path_ids

    keep = {0, n - 1}

    # Regular-interval sampling
    step = max(1, n // max_boxes)
    for i in range(0, n, step):
        keep.add(i)

    # Keep width-jump transition points (where bridge boxes start/end)
    widths = []
    for bid in path_ids:
        r = id2row.get(bid)
        if r is not None:
            widths.append(float(np.min(hi[r] - lo[r])))
        else:
            widths.append(0)
    for i in range(1, n):
        if widths[i] > 0 and widths[i - 1] > 0:
            ratio = widths[i] / widths[i - 1]
            if ratio > 3.0 or ratio < 1.0 / 3.0:
                keep.add(i)
                keep.add(max(0, i - 1))

    result = sorted(keep)
    return [path_ids[i] for i in result]


# ═══════════════════════════════════════════════════════════════════════════
# Corridor expansion
# ═══════════════════════════════════════════════════════════════════════════

def expand_corridor(path_ids, adj, hops):
    """BFS expand path_ids by ±hops layers."""
    expanded = set(path_ids)
    frontier = set(path_ids)
    for _ in range(hops):
        nxt = set()
        for bid in frontier:
            for nb in adj.get(bid, []):
                if nb not in expanded:
                    expanded.add(nb)
                    nxt.add(nb)
        frontier = nxt
    return expanded


# ═══════════════════════════════════════════════════════════════════════════
# Flow-based waypoint extraction
# ═══════════════════════════════════════════════════════════════════════════

def extract_flow_path(gcs, result, verts, v_start, v_goal):
    """Walk along max-phi edges from start to goal."""
    # Build vertex id → box_id map
    vid2bid = {v_start.id(): "__start__", v_goal.id(): "__goal__"}
    for bid, v in verts.items():
        vid2bid[v.id()] = bid

    # Collect outgoing edges with flow values
    out_edges = {}
    for edge in gcs.Edges():
        uid = edge.u().id()
        vid = edge.v().id()
        try:
            phi = float(result.GetSolution(edge.phi()))
        except Exception:
            phi = 0.0
        out_edges.setdefault(uid, []).append((vid, phi))

    # Greedy max-flow walk
    path_vids = [v_start.id()]
    current = v_start.id()
    visited = {current}
    for _ in range(len(verts) + 10):
        if current == v_goal.id():
            break
        candidates = out_edges.get(current, [])
        candidates.sort(key=lambda x: -x[1])
        found = False
        for vid, phi in candidates:
            if vid not in visited and phi > 1e-6:
                path_vids.append(vid)
                visited.add(vid)
                current = vid
                found = True
                break
        if not found:
            # Fallback: any unvisited
            for vid, phi in candidates:
                if vid not in visited:
                    path_vids.append(vid)
                    visited.add(vid)
                    current = vid
                    found = True
                    break
        if not found:
            break

    reached_goal = (current == v_goal.id())
    box_ids_on_path = [vid2bid[vid] for vid in path_vids
                       if vid2bid.get(vid) not in ("__start__", "__goal__", None)]
    return box_ids_on_path, reached_goal


# ═══════════════════════════════════════════════════════════════════════════
# Shortcut smoother (within corridor)
# ═══════════════════════════════════════════════════════════════════════════

def segment_in_boxes(p1, p2, box_lo, box_hi, n_checks=20):
    """Check if segment p1→p2 stays within the union of boxes."""
    for t in np.linspace(0, 1, n_checks):
        pt = p1 + t * (p2 - p1)
        if not np.any(np.all((pt >= box_lo - 1e-10) & (pt <= box_hi + 1e-10), axis=1)):
            return False
    return True


def shortcut_smooth(waypoints, corridor_lo, corridor_hi, n_checks=20):
    """Greedy forward shortcut: skip redundant waypoints if segment stays in corridor."""
    if len(waypoints) <= 2:
        return waypoints
    smoothed = [waypoints[0]]
    i = 0
    while i < len(waypoints) - 1:
        best_j = i + 1
        for j in range(len(waypoints) - 1, i + 1, -1):
            if segment_in_boxes(waypoints[i], waypoints[j],
                                corridor_lo, corridor_hi, n_checks):
                best_j = j
                break
        smoothed.append(waypoints[best_j])
        i = best_j
    return smoothed


# ═══════════════════════════════════════════════════════════════════════════
# GCS solve (Drake GraphOfConvexSets)
# ═══════════════════════════════════════════════════════════════════════════

def gcs_solve(
    lo, hi, ids, adj, q_start, q_goal,
    corridor_hops=2, max_backbone=40, verbose=False,
):
    """
    Full Python GCS pipeline:
      1. Dijkstra → box path (backbone)
      2. Subsample backbone (~40 boxes max)
      3. Corridor expansion
      4. Build Drake GCS + SOCP solve (MOSEK)
      5. Flow-based waypoint extraction
      6. Shortcut smoothing

    Returns: (path, info).
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

    # ── Step 1.5: Project start/goal ──
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

    # ── Step 2: Subsample backbone ──
    backbone = subsample_backbone(path_ids, id2row, lo, hi, max_backbone)
    info["n_backbone"] = len(backbone)

    # ── Step 3: Corridor with auto-fallback hops ──
    for hops in range(corridor_hops, -1, -1):
        corridor = expand_corridor(backbone, adj, hops)

        # Ensure start/goal containing boxes are in corridor
        for q in [q_start, q_goal]:
            for r in find_containing_rows(q, lo, hi):
                corridor.add(ids[r])

        corridor_list = list(corridor)
        n_crr = len(corridor_list)
        crr_rows = [id2row[bid] for bid in corridor_list if bid in id2row]
        crr_lo = lo[crr_rows]
        crr_hi = hi[crr_rows]

        # ── Step 4: Build GCS ──
        t_build = time.perf_counter()
        gcs = GraphOfConvexSets()
        verts = {}
        for i, bid in enumerate(corridor_list):
            verts[bid] = gcs.AddVertex(
                HPolyhedron.MakeBox(crr_lo[i], crr_hi[i]), f"b{bid}")

        v_start = gcs.AddVertex(Point(q_start), "s")
        v_goal = gcs.AddVertex(Point(q_goal), "g")

        # ALWAYS connect start→backbone[0] and backbone[-1]→goal
        # (these guarantee a path exists through the backbone chain)
        if backbone[0] in verts:
            gcs.AddEdge(v_start, verts[backbone[0]])
        if backbone[-1] in verts:
            gcs.AddEdge(verts[backbone[-1]], v_goal)

        # Additionally connect to containing boxes (shortcuts)
        for r in find_containing_rows(q_start, crr_lo, crr_hi):
            bid = corridor_list[r]
            gcs.AddEdge(v_start, verts[bid])
            # Bridge containing box → backbone start for path connectivity
            if bid != backbone[0] and backbone[0] in verts:
                gcs.AddEdge(verts[bid], verts[backbone[0]])
        for r in find_containing_rows(q_goal, crr_lo, crr_hi):
            bid = corridor_list[r]
            gcs.AddEdge(verts[bid], v_goal)
            # Bridge backbone end → containing box for path connectivity
            if bid != backbone[-1] and backbone[-1] in verts:
                gcs.AddEdge(verts[backbone[-1]], verts[bid])

        # Backbone sequential edges (ensure path connectivity)
        for i in range(len(backbone) - 1):
            u, v = backbone[i], backbone[i + 1]
            if u in verts and v in verts:
                gcs.AddEdge(verts[u], verts[v])
                gcs.AddEdge(verts[v], verts[u])

        # Corridor adjacency edges
        seen = set()
        n_edges = 0
        for u in corridor_list:
            for v in adj.get(u, []):
                if v in corridor:
                    pair = (min(u, v), max(u, v))
                    if pair not in seen:
                        seen.add(pair)
                        gcs.AddEdge(verts[u], verts[v])
                        gcs.AddEdge(verts[v], verts[u])
                        n_edges += 2

        # Edge cost: ||x_u - x_v||_2
        A = np.hstack((-np.eye(D), np.eye(D)))
        b = np.zeros(D)
        l2 = L2NormCost(A, b)
        for edge in gcs.Edges():
            edge.AddCost(Binding[Cost](l2, np.concatenate([edge.xu(), edge.xv()])))

        dt_build = time.perf_counter() - t_build

        # ── Solve ──
        t_solve = time.perf_counter()
        opts = GraphOfConvexSetsOptions()
        opts.convex_relaxation = True
        opts.preprocessing = True
        opts.max_rounded_paths = 10
        opts.max_rounding_trials = 100

        try:
            result = gcs.SolveShortestPath(v_start, v_goal, opts)
        except Exception as e:
            if verbose:
                print(f"    hops={hops} exception: {e}")
            continue
        dt_solve = time.perf_counter() - t_solve

        if not result.is_success():
            if verbose:
                print(f"    hops={hops} failed (n={n_crr}, e={n_edges}, {dt_solve:.2f}s)")
            continue

        # ── Step 5: Flow-based extraction ──
        flow_bids, reached = extract_flow_path(gcs, result, verts, v_start, v_goal)
        if not reached:
            # Fallback: backbone extraction
            flow_bids = [bid for bid in backbone if bid in verts]

        waypoints = [q_start.copy()]
        for bid in flow_bids:
            if bid in verts:
                waypoints.append(result.GetSolution(verts[bid].x()))
        waypoints.append(q_goal.copy())

        # Prepend/append real start/goal if projected
        if start_projected:
            waypoints.insert(0, real_start.copy())
        if goal_projected:
            waypoints.append(real_goal.copy())

        # ── Step 6: Shortcut smoothing ──
        n_before = len(waypoints)
        waypoints = shortcut_smooth(waypoints, crr_lo, crr_hi)

        path = np.array(waypoints)
        path_len = float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))

        info.update({
            "success": True,
            "corridor_hops": hops,
            "n_corridor_boxes": n_crr,
            "n_edges": n_edges,
            "build_time": dt_build,
            "solver_time": dt_solve,
            "path_length": path_len,
            "n_flow_boxes": len(flow_bids),
            "n_before_shortcut": n_before,
            "n_waypoints": len(waypoints),
            "optimal_cost": float(result.get_optimal_cost()),
        })
        return path, info

    info["error"] = "all corridor_hops failed"
    return None, info


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
        description="SBF → Python GCS query (pydrake + MOSEK)")
    parser.add_argument("input_json", help="paths.json from C++ exp2")
    parser.add_argument("--corridor-hops", type=int, default=2)
    parser.add_argument("--max-backbone", type=int, default=40)
    parser.add_argument("--verbose", "-v", action="store_true")
    parser.add_argument("--output", type=str, default=None,
                        help="Output JSON path (default: alongside input)")
    args = parser.parse_args()

    forest = load_forest_from_json(args.input_json)
    print(f"Robot: {forest['robot']}, DOF={forest['dof']}, "
          f"Boxes={forest['n_boxes']}, Queries={len(forest['queries'])}")

    lo, hi, ids, adj = forest["lo"], forest["hi"], forest["ids"], forest["adj"]
    results = []

    print(f"\n{'Pair':<10} {'C++len':>8} {'GCSlen':>8} {'Ratio':>6} "
          f"{'Dijk':>6} {'Solve':>7} {'Bkbn':>5} {'Crr':>5} {'Hps':>3} {'Pts':>4}")
    print("-" * 75)

    total_gcs_time = 0.0

    for qi, qp in enumerate(forest["queries"]):
        label = qp["label"]
        start, goal = qp["start"], qp["goal"]

        t0 = time.perf_counter()
        path, info = gcs_solve(
            lo, hi, ids, adj, start, goal,
            corridor_hops=args.corridor_hops,
            max_backbone=args.max_backbone,
            verbose=args.verbose,
        )
        wall_time = time.perf_counter() - t0
        total_gcs_time += wall_time

        cpp = forest["cpp_paths"].get(qi, {})
        cpp_len = cpp.get("path_length", 0)

        if info.get("success"):
            gcs_len = info["path_length"]
            ratio = gcs_len / cpp_len if cpp_len > 0 else 0
            dijk_ms = info["dijkstra_time"] * 1000
            solve_ms = info["solver_time"] * 1000
            hops = info["corridor_hops"]
            n_crr = info["n_corridor_boxes"]
            n_bkbn = info.get("n_backbone", 0)
            n_pts = info["n_waypoints"]

            print(f"{label:<10} {cpp_len:8.3f} {gcs_len:8.3f} {ratio:6.2f} "
                  f"{dijk_ms:5.1f}ms {solve_ms:6.1f}ms {n_bkbn:5d} {n_crr:5d} {hops:3d} {n_pts:4d}")

            results.append({
                "pair_idx": qi, "label": label, "success": True,
                "gcs_path_length": gcs_len, "cpp_path_length": cpp_len,
                "ratio": ratio, "solver_time": info["solver_time"],
                "dijkstra_time": info["dijkstra_time"],
                "corridor_hops": hops, "n_corridor_boxes": n_crr,
                "n_backbone": n_bkbn, "wall_time": wall_time,
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
    print("-" * 75)
    print(f"Success: {n_ok}/{len(results)}, Total GCS time: {total_gcs_time:.3f}s "
          f"({total_gcs_time/max(len(results),1)*1000:.0f}ms/query avg)")

    # Save results
    out_path = args.output
    if out_path is None:
        out_path = str(Path(args.input_json).parent / "gcs_query_results.json")
    with open(out_path, "w") as f:
        json.dump({"results": results, "total_time": total_gcs_time}, f, indent=2)
    print(f"Saved → {out_path}")


if __name__ == "__main__":
    main()

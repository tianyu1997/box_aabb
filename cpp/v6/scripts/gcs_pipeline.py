#!/usr/bin/env python3
"""
gcs_pipeline.py — Optimized GCS pipeline for SBF v5

End-to-end GCS planning: reads boxes+adjacency from paths.json,
computes own Dijkstra backbone, runs GCS-SOCP, compares vs C++ paths.

Key improvements over gcs_optimize_v2.py:
  1. Computes fresh Dijkstra backbone in Python (not relying on C++ box_sequence)
  2. Smart GCS skip: only runs GCS when corridor_ratio < threshold
  3. Multi-corridor-hops fallback with adaptive limits
  4. Proper L2NormCost for true shortest-path optimization

Usage:
    python gcs_pipeline.py <paths.json> [--corridor-hops N] [--max-corridor N]
"""

import argparse
import heapq
import json
import logging
import sys
import time
from pathlib import Path

import numpy as np

from pydrake.geometry.optimization import (
    GraphOfConvexSets,
    GraphOfConvexSetsOptions,
    HPolyhedron,
    Point,
)
from pydrake.solvers import Binding, Cost, L2NormCost

logging.basicConfig(level=logging.INFO, format="[GCS] %(message)s")
log = logging.getLogger(__name__)


# ─────────────────────── Box helper ───────────────────────

class Box:
    __slots__ = ("id", "lo", "hi")

    def __init__(self, id_: int, lo: np.ndarray, hi: np.ndarray):
        self.id = id_
        self.lo = lo
        self.hi = hi

    @property
    def center(self):
        return 0.5 * (self.lo + self.hi)

    def contains(self, q: np.ndarray, tol: float = 1e-10) -> bool:
        return bool(np.all(q >= self.lo - tol) and np.all(q <= self.hi + tol))

    def clamp(self, q: np.ndarray) -> np.ndarray:
        return np.clip(q, self.lo, self.hi)


# ─────────────────── Dijkstra on adj graph ────────────────

def dijkstra(adj, box_map, start_id, goal_id):
    """
    Dijkstra shortest path on box adjacency graph.
    Edge weight = Euclidean distance between box centers.
    Returns (found, box_sequence, total_cost).
    """
    dist = {start_id: 0.0}
    prev = {}
    pq = [(0.0, start_id)]

    while pq:
        d, u = heapq.heappop(pq)
        if d > dist.get(u, float("inf")):
            continue
        if u == goal_id:
            break
        for v in adj.get(u, []):
            if v not in box_map or u not in box_map:
                continue
            w = float(np.linalg.norm(box_map[u].center - box_map[v].center))
            nd = d + w
            if nd < dist.get(v, float("inf")):
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))

    if goal_id not in dist:
        return False, [], 0.0

    # Reconstruct path
    seq = []
    cur = goal_id
    while cur != start_id:
        seq.append(cur)
        cur = prev[cur]
    seq.append(start_id)
    seq.reverse()
    return True, seq, dist[goal_id]


# ─────────────────── Corridor expansion ───────────────────

def expand_corridor(adj, path_boxes, hops):
    corridor = set(path_boxes)
    frontier = set(path_boxes)
    for _ in range(hops):
        nxt = set()
        for bid in frontier:
            for nbr in adj.get(bid, []):
                if nbr not in corridor:
                    corridor.add(nbr)
                    nxt.add(nbr)
        frontier = nxt
    return corridor


# ────────────────── GCS solve single query ────────────────

def gcs_solve(adj, box_map, start, goal, backbone,
              corridor_hops=2, max_corridor=600):
    """
    Build and solve GCS for one query.
    Returns (success, waypoints_list, path_length, n_corridor, solve_time).
    """
    n = len(start)
    if len(backbone) < 2:
        return False, [], 0.0, 0, 0.0

    first_box = box_map[backbone[0]]
    last_box = box_map[backbone[-1]]

    eff_start = start.copy() if first_box.contains(start) else first_box.clamp(start)
    eff_goal = goal.copy() if last_box.contains(goal) else last_box.clamp(goal)

    corridor = expand_corridor(adj, backbone, corridor_hops)

    # Limit corridor size to avoid huge SOCP
    if len(corridor) > max_corridor:
        # Reduce hops until small enough
        for h in range(corridor_hops - 1, -1, -1):
            corridor = expand_corridor(adj, backbone, h)
            if len(corridor) <= max_corridor:
                break

    log.info(f"    Corridor: {len(corridor)} boxes "
             f"(backbone={len(backbone)}, hops≤{corridor_hops})")

    # Build GCS graph
    gcs = GraphOfConvexSets()
    verts = {}
    for bid in corridor:
        box = box_map[bid]
        verts[bid] = gcs.AddVertex(HPolyhedron.MakeBox(box.lo, box.hi),
                                   f"box_{bid}")

    v_start = gcs.AddVertex(Point(eff_start), "start")
    v_goal  = gcs.AddVertex(Point(eff_goal), "goal")

    # Start → first backbone box, last backbone box → Goal
    gcs.AddEdge(v_start, verts[backbone[0]])
    gcs.AddEdge(verts[backbone[-1]], v_goal)

    # All adjacency edges within corridor (bidirectional)
    edge_set = set()
    for u_bid in corridor:
        for v_bid in adj.get(u_bid, []):
            if v_bid in corridor:
                pair = (min(u_bid, v_bid), max(u_bid, v_bid))
                if pair not in edge_set:
                    edge_set.add(pair)
                    gcs.AddEdge(verts[u_bid], verts[v_bid])
                    gcs.AddEdge(verts[v_bid], verts[u_bid])

    # Edge costs: ||x_u - x_v||_2 (Euclidean distance, via SOCP)
    A = np.hstack((-np.eye(n), np.eye(n)))
    b = np.zeros(n)
    l2_cost = L2NormCost(A, b)
    for edge in gcs.Edges():
        xu, xv = edge.xu(), edge.xv()
        edge.AddCost(Binding[Cost](l2_cost, np.concatenate([xu, xv])))

    # Solve
    opts = GraphOfConvexSetsOptions()
    opts.convex_relaxation = True
    opts.preprocessing = True
    opts.max_rounded_paths = 10
    opts.max_rounding_trials = 100

    t0 = time.time()
    result = gcs.SolveShortestPath(v_start, v_goal, opts)
    solve_time = time.time() - t0

    if not result.is_success():
        log.warning(f"    GCS FAILED (time={solve_time:.3f}s)")
        return False, [], 0.0, len(corridor), solve_time

    # Extract waypoints along backbone (ordered, guaranteed reachable)
    path = [eff_start.tolist()]
    for bid in backbone:
        if bid in verts:
            wp = result.GetSolution(verts[bid].x())
            path.append(wp.tolist())
    path.append(eff_goal.tolist())

    # Remove near-duplicate consecutive waypoints
    filtered = [path[0]]
    for i in range(1, len(path)):
        if np.linalg.norm(np.array(path[i]) - np.array(filtered[-1])) > 1e-8:
            filtered.append(path[i])
    path = filtered

    # Greedy shortcut within corridor
    path = shortcut_smooth(path, corridor, box_map)

    path_np = [np.array(p) for p in path]
    path_length = sum(
        float(np.linalg.norm(path_np[i + 1] - path_np[i]))
        for i in range(len(path_np) - 1)
    )

    log.info(f"    GCS solved: len={path_length:.4f}, pts={len(path)}, "
             f"time={solve_time:.3f}s, cost={result.get_optimal_cost():.4f}")

    return True, path, path_length, len(corridor), solve_time


# ─────────────── Shortcut smoother ────────────────────────

def point_in_box(p, box):
    return bool(np.all(p >= box.lo - 1e-10) and np.all(p <= box.hi + 1e-10))


def segment_in_corridor(p1, p2, corridor_boxes, n_checks=30):
    p1, p2 = np.asarray(p1), np.asarray(p2)
    for t in np.linspace(0, 1, n_checks):
        pt = p1 + t * (p2 - p1)
        if not any(point_in_box(pt, box) for box in corridor_boxes):
            return False
    return True


def shortcut_smooth(path, corridor_bids, box_map, n_checks=30):
    if len(path) <= 2:
        return path
    corridor_boxes = [box_map[bid] for bid in corridor_bids if bid in box_map]
    if not corridor_boxes:
        return path

    smoothed = [path[0]]
    i = 0
    while i < len(path) - 1:
        best_j = i + 1
        for j in range(len(path) - 1, i + 1, -1):
            if segment_in_corridor(path[i], path[j], corridor_boxes, n_checks):
                best_j = j
                break
        smoothed.append(path[best_j])
        i = best_j

    return smoothed


# ────────────────────── Main ───────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Optimized GCS pipeline for SBF v5")
    parser.add_argument("input_json",
                        help="paths.json from C++ exp2")
    parser.add_argument("--corridor-hops", type=int, default=2)
    parser.add_argument("--max-corridor", type=int, default=600,
                        help="Max corridor boxes (larger = slower but better)")
    parser.add_argument("--ratio-threshold", type=float, default=4.0,
                        help="Skip GCS when Dijkstra/euclidean ratio > this")
    parser.add_argument("--output", type=str, default=None)
    args = parser.parse_args()

    input_path = Path(args.input_json)
    with open(input_path) as f:
        data = json.load(f)

    dof = data["dof"]
    robot_name = data.get("robot", "unknown")

    # Load boxes
    box_map = {}
    for bd in data.get("boxes", []):
        box_map[bd["id"]] = Box(bd["id"], np.array(bd["lo"]), np.array(bd["hi"]))

    # Load adjacency
    adj = {int(k): v for k, v in data.get("adjacency", {}).items()}

    queries = data.get("queries", [])
    cpp_paths = {}
    for p in data.get("paths", []):
        # Group by (seed_idx, pair_idx) — keep last one per pair_idx
        cpp_paths[p["pair_idx"]] = p

    log.info(f"Robot: {robot_name}, DOF={dof}, boxes={len(box_map)}, "
             f"queries={len(queries)}")

    results = []
    total_gcs_time = 0.0
    n_gcs_better = 0
    n_gcs_run = 0

    for qi, qp in enumerate(queries):
        label = qp["label"]
        start = np.array(qp["start"])
        goal = np.array(qp["goal"])
        euclid = float(np.linalg.norm(goal - start))

        cpp_path = cpp_paths.get(qi)
        cpp_len = cpp_path["path_length"] if cpp_path and cpp_path.get("success") else 0.0

        log.info(f"\n{'='*60}")
        log.info(f"Query {qi}: {label}  euclid={euclid:.3f}  cpp_len={cpp_len:.3f}")

        # Find start/goal containing boxes
        start_id = goal_id = -1
        best_s_d = best_g_d = float("inf")
        for bid, box in box_map.items():
            if box.contains(start):
                start_id = bid
                best_s_d = -1
            elif start_id < 0:
                d = float(np.linalg.norm(start - box.center))
                if d < best_s_d:
                    best_s_d = d
                    start_id = bid
            if box.contains(goal):
                goal_id = bid
                best_g_d = -1
            elif goal_id < 0:
                d = float(np.linalg.norm(goal - box.center))
                if d < best_g_d:
                    best_g_d = d
                    goal_id = bid

        if start_id < 0 or goal_id < 0:
            log.warning(f"  Cannot find start/goal containing box")
            results.append({"pair_idx": qi, "label": label, "success": False})
            continue

        # Compute fresh Dijkstra backbone
        found, backbone, dij_cost = dijkstra(adj, box_map, start_id, goal_id)
        if not found:
            log.warning(f"  Dijkstra: no path (start_box={start_id}, goal_box={goal_id})")
            results.append({"pair_idx": qi, "label": label, "success": False})
            continue

        corridor_ratio = dij_cost / max(euclid, 0.01)
        log.info(f"  Dijkstra: {len(backbone)} boxes, cost={dij_cost:.3f}, "
                 f"ratio={corridor_ratio:.2f}")

        # Skip GCS if corridor is too winding (won't beat RRT)
        if corridor_ratio > args.ratio_threshold:
            log.info(f"  SKIP GCS: ratio {corridor_ratio:.2f} > {args.ratio_threshold}")
            results.append({
                "pair_idx": qi, "label": label, "success": False,
                "reason": "ratio_skip",
                "dijkstra_backbone": len(backbone),
                "corridor_ratio": round(corridor_ratio, 3),
                "cpp_len": round(cpp_len, 4),
            })
            continue

        # Run GCS
        n_gcs_run += 1
        success, gcs_path, gcs_len, n_corridor, solve_time = gcs_solve(
            adj, box_map, start, goal, backbone,
            corridor_hops=args.corridor_hops,
            max_corridor=args.max_corridor,
        )
        total_gcs_time += solve_time

        entry = {
            "pair_idx": qi,
            "label": label,
            "success": success,
            "dijkstra_backbone": len(backbone),
            "corridor_ratio": round(corridor_ratio, 3),
            "cpp_len": round(cpp_len, 4),
        }

        if success:
            ratio_vs_cpp = gcs_len / cpp_len if cpp_len > 0 else 0
            better = gcs_len < cpp_len
            if better:
                n_gcs_better += 1

            entry.update({
                "gcs_len": round(gcs_len, 4),
                "n_corridor": n_corridor,
                "solve_time": round(solve_time, 3),
                "ratio_vs_cpp": round(ratio_vs_cpp, 4),
                "waypoints": gcs_path,
            })
            marker = "✓ BETTER" if better else "✗ worse"
            log.info(f"  GCS={gcs_len:.3f} vs C++={cpp_len:.3f} "
                     f"({marker}, ratio={ratio_vs_cpp:.3f})")
        else:
            entry["solve_time"] = round(solve_time, 3)
            log.info(f"  GCS FAILED")

        results.append(entry)

    # Summary
    log.info(f"\n{'='*60}")
    log.info(f"Summary: GCS SOCP vs C++ Dijkstra+RRT+EB")
    log.info(f"{'Pair':<10} {'C++':>8} {'GCS':>8} {'Ratio':>7} {'Corr':>6} "
             f"{'Solve':>6} {'Result':>8}")
    log.info("-" * 60)
    for r in results:
        lbl = r["label"]
        cpp = r.get("cpp_len", 0)
        if r.get("reason") == "ratio_skip":
            log.info(f"{lbl:<10} {cpp:>8.3f} {'SKIP':>8} "
                     f"{'':>7} {r.get('dijkstra_backbone',0):>6} "
                     f"{'':>6} {'ratio>{:.0f}'.format(args.ratio_threshold):>8}")
        elif r["success"]:
            gcs = r["gcs_len"]
            ratio = r["ratio_vs_cpp"]
            nc = r.get("n_corridor", 0)
            st = r.get("solve_time", 0)
            marker = "BETTER" if gcs < cpp else "worse"
            log.info(f"{lbl:<10} {cpp:>8.3f} {gcs:>8.3f} {ratio:>7.3f} "
                     f"{nc:>6} {st:>6.1f}s {marker:>8}")
        else:
            log.info(f"{lbl:<10} {cpp:>8.3f} {'FAIL':>8}")

    valid = [r for r in results if r["success"]]
    better = [r for r in valid if r["gcs_len"] < r["cpp_len"]]
    log.info(f"\nGCS ran: {n_gcs_run}/{len(queries)}  "
             f"Better: {len(better)}/{len(valid)}  "
             f"Total solve: {total_gcs_time:.1f}s")
    if better:
        savings = [1 - r["ratio_vs_cpp"] for r in better]
        log.info(f"Mean improvement (better only): {np.mean(savings)*100:.1f}%")

    # Save
    output_dir = Path(args.output) if args.output else input_path.parent
    output_dir.mkdir(parents=True, exist_ok=True)
    out_path = output_dir / "gcs_pipeline_results.json"
    out_data = {
        "robot": robot_name, "dof": dof,
        "method": "gcs_pipeline",
        "corridor_hops": args.corridor_hops,
        "max_corridor": args.max_corridor,
        "ratio_threshold": args.ratio_threshold,
        "n_boxes": len(box_map),
        "results": results,
    }
    with open(out_path, "w") as f:
        json.dump(out_data, f, indent=2)
    log.info(f"Saved to: {out_path}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
gcs_optimize.py — Python GCS post-optimization for SBF v5

Reads a paths.json (with boxes + adjacency + box_sequence exported by C++ exp2),
runs Drake GCS shortest-path optimization on each query pair using the
C++ box_sequence as corridor backbone.

Usage:
    conda activate sbf
    python gcs_optimize.py <paths.json> [--corridor-hops N] [--output DIR]
"""

import argparse
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
from pydrake.solvers import Binding, Cost, QuadraticCost

logging.basicConfig(level=logging.INFO, format="[GCS-PY] %(message)s")
log = logging.getLogger(__name__)


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


def gcs_optimize_with_sequence(
    adj, box_map, start, goal, box_sequence,
    corridor_hops=2, convex_relaxation=True
):
    """
    GCS optimization using C++ box_sequence as corridor backbone.
    Start/goal are clamped into first/last box if not already inside.
    Returns (success, waypoints, path_length, n_corridor, eff_start, eff_goal).
    """
    n = len(start)
    if not box_sequence:
        return False, [], 0.0, 0, start, goal

    valid_seq = [bid for bid in box_sequence if bid in box_map]
    if not valid_seq:
        return False, [], 0.0, 0, start, goal

    first_box = box_map[valid_seq[0]]
    last_box = box_map[valid_seq[-1]]

    eff_start = start.copy() if first_box.contains(start) else first_box.clamp(start)
    eff_goal = goal.copy() if last_box.contains(goal) else last_box.clamp(goal)

    if not first_box.contains(start):
        log.info(f"  Start clamped into box {first_box.id} (dist={np.linalg.norm(start - eff_start):.4f})")
    if not last_box.contains(goal):
        log.info(f"  Goal clamped into box {last_box.id} (dist={np.linalg.norm(goal - eff_goal):.4f})")

    corridor = expand_corridor(adj, valid_seq, corridor_hops)
    log.info(f"  Corridor: {len(corridor)} boxes (hops={corridor_hops}, backbone={len(valid_seq)})")

    # Build GCS graph
    gcs = GraphOfConvexSets()
    verts = {}
    for bid in corridor:
        box = box_map[bid]
        verts[bid] = gcs.AddVertex(HPolyhedron.MakeBox(box.lo, box.hi), f"box_{bid}")

    v_start = gcs.AddVertex(Point(eff_start), "start")
    v_goal = gcs.AddVertex(Point(eff_goal), "goal")

    gcs.AddEdge(v_start, verts[valid_seq[0]])
    gcs.AddEdge(verts[valid_seq[-1]], v_goal)

    edge_set = set()
    for u in corridor:
        for v in adj.get(u, []):
            if v in corridor:
                pair = (min(u, v), max(u, v))
                if pair not in edge_set:
                    edge_set.add(pair)
                    gcs.AddEdge(verts[u], verts[v])
                    gcs.AddEdge(verts[v], verts[u])

    # Edge costs: ||x_u - x_v||^2
    for edge in gcs.Edges():
        xu, xv = edge.xu(), edge.xv()
        Q = np.zeros((2 * n, 2 * n))
        Q[:n, :n] = np.eye(n)
        Q[:n, n:] = -np.eye(n)
        Q[n:, :n] = -np.eye(n)
        Q[n:, n:] = np.eye(n)
        edge.AddCost(Binding[Cost](QuadraticCost(Q, np.zeros(2 * n), 0.0),
                                   np.concatenate([xu, xv])))

    opts = GraphOfConvexSetsOptions()
    opts.convex_relaxation = convex_relaxation

    t0 = time.time()
    result = gcs.SolveShortestPath(v_start, v_goal, opts)
    solve_time = time.time() - t0

    if not result.is_success():
        log.warning(f"  GCS solve FAILED (time={solve_time:.3f}s)")
        return False, [], 0.0, len(corridor), eff_start, eff_goal

    # Extract waypoints along box_sequence
    path = [eff_start.tolist()]
    for bid in valid_seq:
        if bid in verts:
            path.append(result.GetSolution(verts[bid].x()).tolist())
    path.append(eff_goal.tolist())

    path_np = [np.array(p) for p in path]
    path_length = sum(
        float(np.linalg.norm(path_np[i + 1] - path_np[i]))
        for i in range(len(path_np) - 1)
    )

    log.info(
        f"  GCS solved: cost={result.get_optimal_cost():.4f}, path_len={path_length:.4f}, "
        f"pts={len(path)}, time={solve_time:.3f}s"
    )

    return True, path, path_length, len(corridor), eff_start, eff_goal


def main():
    parser = argparse.ArgumentParser(description="Python GCS post-optimization for SBF v5")
    parser.add_argument("input_json", help="paths.json from C++ exp2 (with boxes + box_sequence)")
    parser.add_argument("--corridor-hops", type=int, default=2)
    parser.add_argument("--output", type=str, default=None)
    parser.add_argument("--no-relaxation", action="store_true")
    args = parser.parse_args()

    input_path = Path(args.input_json)
    with open(input_path) as f:
        data = json.load(f)

    dof = data["dof"]
    robot_name = data.get("robot", "unknown")
    log.info(f"Robot: {robot_name}, DOF={dof}")

    box_map = {}
    for bd in data.get("boxes", []):
        box_map[bd["id"]] = Box(bd["id"], np.array(bd["lo"]), np.array(bd["hi"]))
    log.info(f"Loaded {len(box_map)} boxes")

    adj = {int(k): v for k, v in data.get("adjacency", {}).items()}
    queries = data.get("queries", [])
    log.info(f"Queries: {len(queries)} pairs")

    cpp_paths = {p["pair_idx"]: p for p in data.get("paths", [])}

    results = []
    for qi, qp in enumerate(queries):
        label = qp["label"]
        start = np.array(qp["start"])
        goal = np.array(qp["goal"])

        log.info(f"\n{'='*60}")
        log.info(f"Query {qi}: {label}")

        cpp_path = cpp_paths.get(qi)
        if not cpp_path or not cpp_path.get("success"):
            log.warning("  C++ Dijkstra failed, skipping")
            results.append({"pair_idx": qi, "label": label, "success": False,
                            "gcs_path_length": 0, "waypoints": []})
            continue

        box_sequence = cpp_path.get("box_sequence", [])
        dijk_len = cpp_path["path_length"]
        log.info(f"  Dijkstra (smoothed): len={dijk_len:.4f}, box_seq={len(box_sequence)} boxes")

        success, path, path_length, n_corridor, eff_start, eff_goal = \
            gcs_optimize_with_sequence(
                adj, box_map, start, goal, box_sequence,
                corridor_hops=args.corridor_hops,
                convex_relaxation=not args.no_relaxation,
            )

        entry = {
            "pair_idx": qi,
            "label": label,
            "success": success,
            "gcs_path_length": round(path_length, 6) if success else 0,
            "dijkstra_path_length": round(dijk_len, 6),
            "n_corridor_boxes": n_corridor,
            "n_backbone_boxes": len(box_sequence),
            "waypoints": path,
        }

        if success:
            proxy_s = float(np.linalg.norm(start - eff_start))
            proxy_g = float(np.linalg.norm(goal - eff_goal))
            total_gcs = path_length + proxy_s + proxy_g
            entry["proxy_start_dist"] = round(proxy_s, 6)
            entry["proxy_goal_dist"] = round(proxy_g, 6)
            entry["total_path_length"] = round(total_gcs, 6)
            ratio = total_gcs / dijk_len if dijk_len > 0 else 0
            entry["improvement_ratio"] = round(ratio, 6)
            log.info(f"  Proxy overhead: start={proxy_s:.4f}, goal={proxy_g:.4f}")
            log.info(f"  Total (GCS + proxy): {total_gcs:.4f}")
            log.info(f"  vs Dijkstra (smoothed): {dijk_len:.4f}  ratio={ratio:.4f} ({(1-ratio)*100:+.1f}%)")

        results.append(entry)

    # Summary
    log.info(f"\n{'='*60}")
    log.info("Summary: Python GCS vs C++ Dijkstra (smoothed)")
    log.info(f"{'Pair':<10} {'Dijk':>8} {'GCS-box':>8} {'Proxy':>8} {'GCS-tot':>8} {'Ratio':>7}")
    log.info("-" * 55)
    for r in results:
        d = r.get("dijkstra_path_length", 0)
        g = r.get("gcs_path_length", 0)
        ps = r.get("proxy_start_dist", 0)
        pg = r.get("proxy_goal_dist", 0)
        t = r.get("total_path_length", 0)
        ratio = r.get("improvement_ratio", 0)
        log.info(f"{r['label']:<10} {d:>8.3f} {g:>8.3f} {ps+pg:>8.3f} {t:>8.3f} {ratio:>7.3f}")

    output_dir = Path(args.output) if args.output else input_path.parent
    output_dir.mkdir(parents=True, exist_ok=True)
    out_path = output_dir / "gcs_results.json"
    out_data = {
        "robot": robot_name, "dof": dof, "method": "python_gcs",
        "corridor_hops": args.corridor_hops,
        "convex_relaxation": not args.no_relaxation,
        "n_boxes": len(box_map), "results": results,
    }
    with open(out_path, "w") as f:
        json.dump(out_data, f, indent=2)
    log.info(f"\nSaved to: {out_path}")


if __name__ == "__main__":
    main()

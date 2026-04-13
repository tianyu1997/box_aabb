#!/usr/bin/env python3
"""
gcs_optimize_v2.py — Improved Python GCS post-optimization for SBF v5

Key improvements over v1:
  1. Flow-based waypoint extraction (follow max-flow edges, not all backbone boxes)
  2. Backbone subsampling for long sequences (merge tiny bridge boxes)
  3. Automatic corridor_hops fallback (try N, N-1, ..., 0 on failure)
  4. Shortcut smoother (skip redundant waypoints that stay within corridor)

Usage:
    conda activate sbf
    python gcs_optimize_v2.py <paths.json> [--corridor-hops N] [--output DIR]
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
from pydrake.solvers import Binding, Cost, L2NormCost, QuadraticCost

logging.basicConfig(level=logging.INFO, format="[GCS-PY] %(message)s")
log = logging.getLogger(__name__)

# ──────────────────────────── Box helper ────────────────────────────

class Box:
    __slots__ = ("id", "lo", "hi")

    def __init__(self, id_: int, lo: np.ndarray, hi: np.ndarray):
        self.id = id_
        self.lo = lo
        self.hi = hi

    @property
    def center(self):
        return 0.5 * (self.lo + self.hi)

    @property
    def widths(self):
        return self.hi - self.lo

    @property
    def min_width(self):
        return float(np.min(self.hi - self.lo))

    def contains(self, q: np.ndarray, tol: float = 1e-10) -> bool:
        return bool(np.all(q >= self.lo - tol) and np.all(q <= self.hi + tol))

    def clamp(self, q: np.ndarray) -> np.ndarray:
        return np.clip(q, self.lo, self.hi)

    def union(self, other: "Box") -> "Box":
        """Return bounding box of self and other (NOT necessarily a valid free-space box)."""
        return Box(-1, np.minimum(self.lo, other.lo), np.maximum(self.hi, other.hi))


# ──────────────────────── Corridor helpers ──────────────────────────

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


def subsample_backbone(box_map, box_sequence, max_boxes=60):
    """
    Reduce a long backbone by keeping:
      - first and last box always
      - boxes at regular intervals
      - boxes adjacent to 'significant' width changes
    This avoids the GCS solver operating over hundreds of tiny bridge boxes.
    """
    if len(box_sequence) <= max_boxes:
        return box_sequence

    n = len(box_sequence)
    keep = set()
    keep.add(0)
    keep.add(n - 1)

    # Regular interval sampling
    step = max(1, n // max_boxes)
    for i in range(0, n, step):
        keep.add(i)

    # Also keep transition points (where bridge starts/ends tend to have width jumps)
    widths = []
    for bid in box_sequence:
        if bid in box_map:
            widths.append(box_map[bid].min_width)
        else:
            widths.append(0)

    for i in range(1, n):
        if widths[i] > 0 and widths[i - 1] > 0:
            ratio = widths[i] / widths[i - 1]
            if ratio > 3.0 or ratio < 1.0 / 3.0:
                keep.add(i)
                if i > 0:
                    keep.add(i - 1)

    result = sorted(keep)

    log.info(f"    Subsampled backbone: {n} -> {len(result)} boxes")
    return [box_sequence[i] for i in result]


# ──────────────────── GCS solve single query ────────────────────────

def gcs_solve(adj, box_map, start, goal, box_sequence,
              corridor_hops=2, convex_relaxation=True, max_backbone=60):
    """
    Build and solve GCS for one query.
    Returns (success, waypoints, path_length, n_corridor, eff_start, eff_goal, solve_time).
    """
    n = len(start)
    if not box_sequence:
        return False, [], 0.0, 0, start, goal, 0.0

    valid_seq = [bid for bid in box_sequence if bid in box_map]
    if not valid_seq:
        return False, [], 0.0, 0, start, goal, 0.0

    # Subsample long backbones
    if len(valid_seq) > max_backbone:
        valid_seq = subsample_backbone(box_map, valid_seq, max_backbone)

    first_box = box_map[valid_seq[0]]
    last_box = box_map[valid_seq[-1]]

    eff_start = start.copy() if first_box.contains(start) else first_box.clamp(start)
    eff_goal = goal.copy() if last_box.contains(goal) else last_box.clamp(goal)

    if not first_box.contains(start):
        log.info(f"    Start clamped into box {first_box.id} (dist={np.linalg.norm(start - eff_start):.4f})")
    if not last_box.contains(goal):
        log.info(f"    Goal clamped into box {last_box.id} (dist={np.linalg.norm(goal - eff_goal):.4f})")

    corridor = expand_corridor(adj, valid_seq, corridor_hops)
    log.info(f"    Corridor: {len(corridor)} boxes (hops={corridor_hops}, backbone={len(valid_seq)})")

    # Build GCS graph
    gcs = GraphOfConvexSets()
    verts = {}
    for bid in corridor:
        box = box_map[bid]
        verts[bid] = gcs.AddVertex(HPolyhedron.MakeBox(box.lo, box.hi), f"box_{bid}")

    v_start = gcs.AddVertex(Point(eff_start), "start")
    v_goal = gcs.AddVertex(Point(eff_goal), "goal")

    # Edges: start -> first, last -> goal
    gcs.AddEdge(v_start, verts[valid_seq[0]])
    gcs.AddEdge(verts[valid_seq[-1]], v_goal)

    # Backbone edges (sequential)
    backbone_set = set(valid_seq)
    for i in range(len(valid_seq) - 1):
        u, v = valid_seq[i], valid_seq[i + 1]
        gcs.AddEdge(verts[u], verts[v])
        gcs.AddEdge(verts[v], verts[u])

    # Corridor expansion edges
    edge_set = set()
    for u_bid in corridor:
        for v_bid in adj.get(u_bid, []):
            if v_bid in corridor:
                pair = (min(u_bid, v_bid), max(u_bid, v_bid))
                if pair not in edge_set:
                    edge_set.add(pair)
                    gcs.AddEdge(verts[u_bid], verts[v_bid])
                    gcs.AddEdge(verts[v_bid], verts[u_bid])

    # Edge costs: ||x_u - x_v||_2 (L2 norm = actual Euclidean distance)
    # This minimizes total path length sum(||x_i - x_{i+1}||)
    # NOT sum of squared distances which would favor many small steps.
    A = np.hstack((-np.eye(n), np.eye(n)))  # A @ [x_u, x_v] = x_v - x_u
    b = np.zeros(n)
    l2_cost = L2NormCost(A, b)
    for edge in gcs.Edges():
        xu, xv = edge.xu(), edge.xv()
        edge.AddCost(Binding[Cost](l2_cost, np.concatenate([xu, xv])))

    opts = GraphOfConvexSetsOptions()
    opts.convex_relaxation = convex_relaxation
    opts.preprocessing = True
    opts.max_rounded_paths = 10
    opts.max_rounding_trials = 100

    t0 = time.time()
    result = gcs.SolveShortestPath(v_start, v_goal, opts)
    solve_time = time.time() - t0

    if not result.is_success():
        log.warning(f"    GCS solve FAILED (hops={corridor_hops}, time={solve_time:.3f}s)")
        return False, [], 0.0, len(corridor), eff_start, eff_goal, solve_time

    # ─── Flow-based waypoint extraction ───
    # Build directed graph of edge flows
    # Each edge in GCS: u -> v with flow phi ∈ [0,1]
    vert_to_id = {v_start.id(): "start", v_goal.id(): "goal"}
    for bid, v in verts.items():
        vert_to_id[v.id()] = bid

    # Collect flows
    out_edges = {}  # vertex_id -> list of (target_vertex_id, flow, edge)
    for edge in gcs.Edges():
        uid = edge.u().id()
        vid = edge.v().id()
        try:
            phi = float(result.GetSolution(edge.phi()))
        except Exception:
            phi = 0.0
        if uid not in out_edges:
            out_edges[uid] = []
        out_edges[uid].append((vid, phi, edge))

    # Greedy max-flow path extraction: start -> ... -> goal
    flow_path_vids = [v_start.id()]
    current = v_start.id()
    visited = {current}
    max_steps = len(corridor) + 10

    for _ in range(max_steps):
        if current == v_goal.id():
            break
        candidates = out_edges.get(current, [])
        if not candidates:
            break
        # Pick the edge with the highest flow
        candidates.sort(key=lambda x: -x[1])
        found_next = False
        for vid, phi, edge in candidates:
            if vid not in visited and phi > 1e-6:
                flow_path_vids.append(vid)
                visited.add(vid)
                current = vid
                found_next = True
                break
        if not found_next:
            # No unvisited high-flow edge found, take the best anyway
            for vid, phi, edge in candidates:
                if vid not in visited:
                    flow_path_vids.append(vid)
                    visited.add(vid)
                    current = vid
                    found_next = True
                    break
        if not found_next:
            break

    if current != v_goal.id():
        log.warning(f"    Flow extraction didn't reach goal, falling back to backbone extraction")
        # Fallback: extract from backbone
        path = [eff_start.tolist()]
        for bid in valid_seq:
            if bid in verts:
                path.append(result.GetSolution(verts[bid].x()).tolist())
        path.append(eff_goal.tolist())
    else:
        # Extract waypoints from flow path
        path = []
        for vid in flow_path_vids:
            label = vert_to_id.get(vid)
            if label == "start":
                path.append(eff_start.tolist())
            elif label == "goal":
                path.append(eff_goal.tolist())
            else:
                bid = label
                path.append(result.GetSolution(verts[bid].x()).tolist())
        log.info(f"    Flow extraction: {len(flow_path_vids)} vertices "
                 f"(backbone had {len(valid_seq)} boxes)")

    # ─── Shortcut smoothing ───
    path = shortcut_smooth(path, corridor, box_map)

    path_np = [np.array(p) for p in path]
    path_length = sum(
        float(np.linalg.norm(path_np[i + 1] - path_np[i]))
        for i in range(len(path_np) - 1)
    )

    log.info(
        f"    GCS solved: cost={result.get_optimal_cost():.4f}, path_len={path_length:.4f}, "
        f"pts={len(path)}, time={solve_time:.3f}s"
    )

    return True, path, path_length, len(corridor), eff_start, eff_goal, solve_time


# ─────────────────── Shortcut smoother ──────────────────────

def point_in_box(p, box):
    return bool(np.all(p >= box.lo - 1e-10) and np.all(p <= box.hi + 1e-10))


def segment_in_corridor(p1, p2, corridor_boxes, n_checks=20):
    """
    Check if the line segment p1->p2 stays within the union of corridor boxes.
    We sample n_checks points along the segment and verify each is in at least one box.
    """
    p1, p2 = np.asarray(p1), np.asarray(p2)
    for t in np.linspace(0, 1, n_checks):
        pt = p1 + t * (p2 - p1)
        in_some_box = False
        for box in corridor_boxes:
            if point_in_box(pt, box):
                in_some_box = True
                break
        if not in_some_box:
            return False
    return True


def shortcut_smooth(path, corridor_bids, box_map, n_checks=20):
    """
    Greedy shortcut: try to skip intermediate waypoints if the direct segment
    stays within the corridor union.
    """
    if len(path) <= 2:
        return path

    corridor_boxes = [box_map[bid] for bid in corridor_bids if bid in box_map]
    if not corridor_boxes:
        return path

    smoothed = [path[0]]
    i = 0
    while i < len(path) - 1:
        # Try to jump as far ahead as possible
        best_j = i + 1
        for j in range(len(path) - 1, i + 1, -1):
            if segment_in_corridor(path[i], path[j], corridor_boxes, n_checks):
                best_j = j
                break
        smoothed.append(path[best_j])
        i = best_j

    if len(smoothed) < len(path):
        log.info(f"    Shortcut: {len(path)} -> {len(smoothed)} waypoints")

    return smoothed


# ─────────────────── Path densification ──────────────────────

def densify_path(waypoints, step_rad=0.05):
    """
    Densify a path so no segment exceeds step_rad in joint-space distance.
    Input: list of np.ndarray waypoints.
    Output: list of np.ndarray with intermediate points inserted.
    """
    if len(waypoints) < 2:
        return list(waypoints)
    dense = [waypoints[0]]
    for i in range(len(waypoints) - 1):
        p1, p2 = waypoints[i], waypoints[i + 1]
        dist = float(np.linalg.norm(p2 - p1))
        if dist <= step_rad:
            dense.append(p2)
        else:
            n_sub = int(np.ceil(dist / step_rad))
            for k in range(1, n_sub + 1):
                t = k / n_sub
                dense.append(p1 + t * (p2 - p1))
    return dense


# ────────────────────── Main entry point ──────────────────────

def main():
    parser = argparse.ArgumentParser(description="Python GCS post-optimization for SBF v5 (v2)")
    parser.add_argument("input_json", help="paths.json from C++ exp2 (with boxes + box_sequence)")
    parser.add_argument("--corridor-hops", type=int, default=2,
                        help="Max corridor expansion hops (will fall back to 0 on failure)")
    parser.add_argument("--max-backbone", type=int, default=60,
                        help="Max backbone boxes before subsampling")
    parser.add_argument("--output", type=str, default=None)
    parser.add_argument("--no-relaxation", action="store_true")
    parser.add_argument("--no-fallback", action="store_true",
                        help="Don't try lower hops on failure")
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

        # Try with requested corridor_hops, fall back on failure
        max_hops = args.corridor_hops
        success = False
        path = []
        path_length = 0.0
        n_corridor = 0
        eff_start = start
        eff_goal = goal
        total_solve_time = 0.0
        used_hops = max_hops

        for hops in (range(max_hops, -1, -1) if not args.no_fallback else [max_hops]):
            success, path, path_length, n_corridor, eff_start, eff_goal, solve_time = \
                gcs_solve(
                    adj, box_map, start, goal, box_sequence,
                    corridor_hops=hops,
                    convex_relaxation=not args.no_relaxation,
                    max_backbone=args.max_backbone,
                )
            total_solve_time += solve_time
            if success:
                used_hops = hops
                break
            elif hops > 0:
                log.info(f"  Retrying with corridor_hops={hops - 1}...")

        # ── Proxy-link stitching: complete path from real start to real goal ──
        gcs_corridor_length = path_length  # GCS corridor-only length

        entry = {
            "pair_idx": qi,
            "label": label,
            "success": success,
            "gcs_corridor_length": round(gcs_corridor_length, 6) if success else 0,
            "dijkstra_path_length": round(dijk_len, 6),
            "n_corridor_boxes": n_corridor,
            "n_backbone_boxes": len(box_sequence),
            "corridor_hops_used": used_hops,
            "solve_time": round(total_solve_time, 4),
            "start": start.tolist(),
            "goal": goal.tolist(),
        }

        if success:
            proxy_s = float(np.linalg.norm(start - eff_start))
            proxy_g = float(np.linalg.norm(goal - eff_goal))

            complete_path = path  # default: GCS-only path (if no proxy gap)
            cpp_wps_raw = cpp_path.get("waypoints", [])

            if (proxy_s > 0.01 or proxy_g > 0.01) and len(cpp_wps_raw) >= 2:
                # ── Densify C++ path for fine-grained proxy link extraction ──
                # C++ waypoints can be very sparse (segments > 2 rad), so we
                # interpolate them at fine resolution (step_rad) first.
                cpp_wps = [np.array(w) for w in cpp_wps_raw]
                dense_cpp = densify_path(cpp_wps, step_rad=0.05)

                # Build cumulative arc-length along densified C++ path
                cum_dist = [0.0]
                for i in range(1, len(dense_cpp)):
                    cum_dist.append(cum_dist[-1] + float(np.linalg.norm(
                        dense_cpp[i] - dense_cpp[i - 1])))
                total_cpp_len = cum_dist[-1]

                prefix = []  # densified C++ wps for start proxy link
                suffix = []  # densified C++ wps for goal proxy link

                # Forward walk: extract [start ... proxy_s] from dense C++ path
                if proxy_s > 0.01:
                    for i in range(len(dense_cpp)):
                        if cum_dist[i] >= proxy_s:
                            prefix = [w.tolist() for w in dense_cpp[:i]]
                            log.info(f"  Proxy-start: {len(prefix)} dense wps "
                                     f"(cum={cum_dist[i]:.4f} >= proxy_s={proxy_s:.4f})")
                            break
                    else:
                        prefix = [w.tolist() for w in dense_cpp]

                # Backward walk: extract [total-proxy_g ... goal] from dense C++ path
                if proxy_g > 0.01:
                    cutoff = total_cpp_len - proxy_g
                    for i in range(len(dense_cpp) - 1, -1, -1):
                        if cum_dist[i] <= cutoff:
                            suffix = [w.tolist() for w in dense_cpp[i + 1:]]
                            log.info(f"  Proxy-goal: {len(suffix)} dense wps "
                                     f"(cum={cum_dist[i]:.4f} <= cutoff={cutoff:.4f})")
                            break
                    else:
                        suffix = [w.tolist() for w in dense_cpp]

                stitched = prefix + path + suffix
                if stitched:
                    stitched[0] = start.tolist()
                    stitched[-1] = goal.tolist()
                complete_path = stitched
                log.info(f"  Stitched path: {len(prefix)} prefix + "
                         f"{len(path)} GCS + {len(suffix)} suffix = "
                         f"{len(complete_path)} total wps")

            # If proxy gap remains (stitching not applied), add real start/goal
            cp0 = np.array(complete_path[0])
            cpN = np.array(complete_path[-1])
            if float(np.linalg.norm(cp0 - start)) > 0.01:
                complete_path = [start.tolist()] + complete_path
                log.info(f"  Prepended real start (gap={np.linalg.norm(cp0 - start):.4f})")
            if float(np.linalg.norm(cpN - goal)) > 0.01:
                complete_path = complete_path + [goal.tolist()]
                log.info(f"  Appended real goal (gap={np.linalg.norm(cpN - goal):.4f})")

            # ── Densify long GCS corridor segments to prevent mid-segment collisions ──
            complete_path = densify_path(
                [np.array(p) for p in complete_path], step_rad=0.05)
            complete_path = [p.tolist() for p in complete_path]

            # Compute total path length of complete path
            cp_np = [np.array(p) for p in complete_path]
            total_length = sum(
                float(np.linalg.norm(cp_np[i + 1] - cp_np[i]))
                for i in range(len(cp_np) - 1)
            )

            entry["waypoints"] = complete_path
            entry["gcs_path_length"] = round(gcs_corridor_length, 6)
            entry["total_path_length"] = round(total_length, 6)
            entry["proxy_start_dist"] = round(proxy_s, 6)
            entry["proxy_goal_dist"] = round(proxy_g, 6)
            ratio = total_length / dijk_len if dijk_len > 0 else 0
            entry["improvement_ratio"] = round(ratio, 6)
            log.info(f"  Proxy gaps: start={proxy_s:.4f}, goal={proxy_g:.4f}")
            log.info(f"  GCS corridor length: {gcs_corridor_length:.4f}")
            log.info(f"  Total path length:   {total_length:.4f}")
            log.info(f"  vs Dijkstra (smoothed): {dijk_len:.4f}  "
                     f"ratio={ratio:.4f} ({(1-ratio)*100:+.1f}%)")
        else:
            entry["waypoints"] = path

        results.append(entry)

    # Summary table
    log.info(f"\n{'='*60}")
    log.info("Summary: Python GCS v2 vs C++ Dijkstra (smoothed)")
    log.info(f"{'Pair':<10} {'Dijk':>8} {'GCS-crr':>8} {'Proxy':>8} {'Total':>8} {'Ratio':>7} {'Hops':>5} {'Pts':>5}")
    log.info("-" * 70)
    for r in results:
        if not r["success"]:
            log.info(f"{r['label']:<10} {'FAIL':>8}")
            continue
        d = r.get("dijkstra_path_length", 0)
        g = r.get("gcs_corridor_length", r.get("gcs_path_length", 0))
        ps = r.get("proxy_start_dist", 0)
        pg = r.get("proxy_goal_dist", 0)
        t = r.get("total_path_length", 0)
        ratio = r.get("improvement_ratio", 0)
        hops = r.get("corridor_hops_used", 0)
        npts = len(r.get("waypoints", []))
        log.info(f"{r['label']:<10} {d:>8.3f} {g:>8.3f} {ps+pg:>8.3f} {t:>8.3f} {ratio:>7.3f} {hops:>5d} {npts:>5d}")

    # Mean ratio
    valid = [r for r in results if r["success"] and r.get("improvement_ratio", 0) > 0]
    if valid:
        mean_ratio = np.mean([r["improvement_ratio"] for r in valid])
        log.info(f"\nMean GCS/Dijkstra ratio: {mean_ratio:.4f} ({(1-mean_ratio)*100:+.1f}%)")

    output_dir = Path(args.output) if args.output else input_path.parent
    output_dir.mkdir(parents=True, exist_ok=True)
    out_path = output_dir / "gcs_results_v2.json"
    out_data = {
        "robot": robot_name, "dof": dof, "method": "python_gcs_v2",
        "corridor_hops_max": args.corridor_hops,
        "max_backbone": args.max_backbone,
        "convex_relaxation": not args.no_relaxation,
        "n_boxes": len(box_map),
        "queries": [{"label": q["label"], "start": q["start"], "goal": q["goal"]}
                     for q in queries],
        "results": results,
    }
    with open(out_path, "w") as f:
        json.dump(out_data, f, indent=2)
    log.info(f"\nSaved to: {out_path}")


if __name__ == "__main__":
    main()

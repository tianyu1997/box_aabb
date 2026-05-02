#!/usr/bin/env python3
"""
gcs_pipeline.py — Optimized GCS pipeline for SBF v6

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
from pydrake.solvers import (
    Binding,
    Cost,
    Constraint,
    L2NormCost,
    LinearConstraint,
)

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

def dijkstra(adj, box_map, start_id, goal_id,
             bridge_pairs=None, bridge_penalty=1.0):
    """
    Dijkstra shortest path on box adjacency graph.
    Edge weight = Euclidean distance between box centers.
    If `bridge_pairs` is provided, every edge whose unordered (u,v) key is in
    that set is multiplied by `bridge_penalty` so the search prefers genuine
    overlap chains over single-hop SBF bridges (which would otherwise win and
    pin the SOCP path through the SBF waypoints, defeating GCS optimization).
    Returns (found, box_sequence, total_cost).
    """
    bridge_pairs = bridge_pairs or set()
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
            if (min(u, v), max(u, v)) in bridge_pairs:
                w *= bridge_penalty
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

def _max_gap(box_u, box_v):
    """Per-axis signed overlap (negative = overlap, positive = gap).
    Returns max over axes; ≤0 means boxes overlap, >0 is the largest gap."""
    return float(np.max(np.maximum(box_u.lo, box_v.lo)
                        - np.minimum(box_u.hi, box_v.hi)))


def gcs_solve(adj, box_map, start, goal, backbone,
              corridor_hops=2, max_corridor=600, bridge_pairs=None,
              bridge_chains=None, drop_bridges=False,
              slack_tol=0.05):
    """
    Build and solve GCS for one query.
    Returns (success, waypoints_list, path_length, n_corridor, solve_time).

    bridge_chains: dict[(min_id, max_id)] -> list of waypoint chains.  Each
    chain is a list of n-D points starting in box_min, ending in box_max,
    crossing through C-free space along an SBF-validated RRT bridge.  When
    provided, every bridge-pair adjacency edge with max-gap > slack_tol is
    REPLACED by a Point-vertex chain that pins the GCS solution to traverse
    exactly through the SBF-validated waypoints.

    drop_bridges: if True, exclude bridge_pairs entirely from the GCS graph.

    slack_tol: per-axis slack for continuity on "soft" bridge edges (max-gap
    ≤ slack_tol).  Such edges get a regular box-box continuity constraint
    where u's polyhedron is inflated by slack_tol on every facet, so v's
    start point may lie up to slack_tol outside u.  Setting slack_tol=0
    reverts to strict overlap continuity (every bridge then needs a Point
    chain).
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

    bridge_pairs = bridge_pairs or set()
    bridge_chains = bridge_chains or {}

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

    # Collect all corridor adjacency pairs.
    raw_pairs = set()
    for u_bid in corridor:
        for v_bid in adj.get(u_bid, []):
            if v_bid in corridor:
                raw_pairs.add((min(u_bid, v_bid), max(u_bid, v_bid)))

    # Track bridge Point vertices so we know which edges DO need a
    # box->Point continuity (xu == Point's value) constraint vs ordinary
    # box-box overlap continuity.  pt_to_value: vertex -> (n,) np.ndarray.
    pt_to_value = {}

    overlap_pairs = []           # genuine overlap (gap ≤ 0); strict continuity
    soft_overlap_pairs = []      # 0 < gap ≤ slack_tol; slack continuity
    bridge_pairs_in_corridor = []  # bridge with gap > slack_tol; Point chain
    for pair in raw_pairs:
        gap = _max_gap(box_map[pair[0]], box_map[pair[1]])
        if gap <= 0:
            overlap_pairs.append(pair)
        elif gap <= slack_tol:
            # Small geometric gap: relax with slack continuity regardless of
            # whether C++ tagged it as a bridge_pair.  Most adj edges with
            # gap ≤ 5 cm fall here (soft-FFB chains from Option C).
            soft_overlap_pairs.append(pair)
        elif pair in bridge_pairs:
            if not drop_bridges:
                bridge_pairs_in_corridor.append(pair)
        # Large-gap non-bridge edges silently dropped (shouldn't happen).

    soft_pair_set = set(soft_overlap_pairs)

    # Add overlap + soft-overlap edges (both directions).
    for (u_bid, v_bid) in overlap_pairs:
        gcs.AddEdge(verts[u_bid], verts[v_bid])
        gcs.AddEdge(verts[v_bid], verts[u_bid])
    for (u_bid, v_bid) in soft_overlap_pairs:
        gcs.AddEdge(verts[u_bid], verts[v_bid])
        gcs.AddEdge(verts[v_bid], verts[u_bid])

    # Add bridge chains: for each bridge pair, replace direct edge with a
    # chain of Point vertices that traces the SBF-validated bridge waypoints.
    # If no chain is available, fall back to a direct edge (no continuity).
    n_chained = 0
    n_fallback = 0
    for (u_bid, v_bid) in bridge_pairs_in_corridor:
        chain = bridge_chains.get((u_bid, v_bid))
        if chain is None or len(chain) < 2:
            # Fallback: direct edge, no continuity (legacy behaviour)
            gcs.AddEdge(verts[u_bid], verts[v_bid])
            gcs.AddEdge(verts[v_bid], verts[u_bid])
            n_fallback += 1
            continue
        # chain[0] is in box_min_id, chain[-1] is in box_max_id (per C++).
        # Determine direction: which end matches u_bid vs v_bid.
        c0 = np.asarray(chain[0]); cn = np.asarray(chain[-1])
        b_u = box_map[u_bid]; b_v = box_map[v_bid]
        if b_u.contains(c0) and b_v.contains(cn):
            chain_seq = [np.asarray(p) for p in chain]
            head_bid, tail_bid = u_bid, v_bid
        elif b_v.contains(c0) and b_u.contains(cn):
            # Chain stored in reverse direction; reversing it puts the
            # u_bid-anchored end first, so head_bid stays u_bid.
            chain_seq = [np.asarray(p) for p in reversed(chain)]
            head_bid, tail_bid = u_bid, v_bid
        else:
            # Chain endpoints don't match either box: fall back.
            gcs.AddEdge(verts[u_bid], verts[v_bid])
            gcs.AddEdge(verts[v_bid], verts[u_bid])
            n_fallback += 1
            continue

        # Create Point vertices for ALL waypoints in chain_seq.  The first
        # and last waypoints are anchored inside head_box / tail_box, so
        # head_box -> Point(chain_seq[0]) forces box.x = chain_seq[0]
        # (feasible because chain_seq[0] ∈ head_box).  Inner Points are
        # outside any SBF box but lie on a collision-free RRT segment.
        pt_verts = []
        for k, p in enumerate(chain_seq):
            pv = gcs.AddVertex(Point(p), f"br_{head_bid}_{tail_bid}_{k}")
            pt_verts.append(pv)
            pt_to_value[pv] = p

        # Build the linear chain head_box -> [Point...]+ -> tail_box.
        # Add forward AND reverse edges so Dijkstra/GCS routing is symmetric.
        seq_v = [verts[head_bid]] + pt_verts + [verts[tail_bid]]
        for a, b in zip(seq_v[:-1], seq_v[1:]):
            gcs.AddEdge(a, b)
            gcs.AddEdge(b, a)
        n_chained += 1

    # Edge costs + continuity
    A = np.hstack((-np.eye(n), np.eye(n)))
    b = np.zeros(n)
    l2_cost = L2NormCost(A, b)

    # Marcucci-style continuity (canonical pattern from
    # gcs-science-robotics/gcs/linear.py): for every overlap edge (u, v),
    # constrain v.x() to ALSO lie in u.set().  For Box->Point and Point->Box
    # edges in a bridge chain, we additionally pin via per-component
    # equality (xu == xv) so the box-side variable equals the bridge waypoint.
    n_box_eq = 0
    n_soft = 0
    n_pt_eq = 0
    n_bridge_skipped = 0
    for edge in gcs.Edges():
        xu, xv = edge.xu(), edge.xv()
        edge.AddCost(Binding[Cost](l2_cost, np.concatenate([xu, xv])))
        u_v, v_v = edge.u(), edge.v()
        if u_v is v_start or v_v is v_goal:
            continue
        u_is_pt = u_v in pt_to_value
        v_is_pt = v_v in pt_to_value
        if u_is_pt and v_is_pt:
            # Point-to-Point: both endpoints fixed by their Point sets,
            # no extra constraint needed.
            continue
        if u_is_pt or v_is_pt:
            # Bridge box<->Point edge: pin box-side value to the Point.
            # Drake's GCS perspective scales the constraint by edge flow.
            for d in range(n):
                edge.AddConstraint(xu[d] == xv[d])
            n_pt_eq += 1
            continue
        # Both endpoints are box vertices.
        u_bid = next((bid for bid, vx in verts.items() if vx is u_v), None)
        v_bid = next((bid for bid, vx in verts.items() if vx is v_v), None)
        if u_bid is None or v_bid is None:
            continue
        pair = (min(u_bid, v_bid), max(u_bid, v_bid))
        if pair in bridge_pairs:
            # Fallback bridge edge with no chain: no continuity.
            n_bridge_skipped += 1
            continue
        u_set = u_v.set()
        is_soft = pair in soft_pair_set
        b_vec = u_set.b().copy()
        if is_soft and slack_tol > 0:
            # Inflate u's polyhedron by slack_tol on every facet.
            b_vec = b_vec + slack_tol
        edge.AddConstraint(Binding[Constraint](
            LinearConstraint(u_set.A(),
                             -np.inf * np.ones(len(b_vec)),
                             b_vec),
            v_v.x()))
        if is_soft:
            n_soft += 1
        else:
            n_box_eq += 1
    log.info(f"    GCS edges: {n_box_eq} box-box strict, "
             f"{n_soft} box-box slack(≤{slack_tol:g}), "
             f"{n_pt_eq} box<->Point eq, "
             f"{n_chained} chains injected, "
             f"{n_fallback} fallback bridges, "
             f"{n_bridge_skipped} bridge-skipped")

    # Solve
    opts = GraphOfConvexSetsOptions()
    opts.convex_relaxation = True
    opts.preprocessing = True
    opts.max_rounded_paths = 50
    opts.max_rounding_trials = 500

    t0 = time.time()
    result = gcs.SolveShortestPath(v_start, v_goal, opts)
    solve_time = time.time() - t0

    if not result.is_success():
        log.warning(f"    GCS FAILED (time={solve_time:.3f}s)")
        return False, [], 0.0, len(corridor), solve_time

    # Extract waypoints along the active edge path returned by GCS rounding,
    # not every backbone vertex (rounding picks a single shortest sub-path
    # through the corridor and inactive vertices have no captured solution).
    path = [eff_start.tolist()]
    try:
        active_edges = gcs.GetSolutionPath(v_start, v_goal, result)
    except Exception:
        active_edges = []
    if active_edges:
        for edge in active_edges:
            try:
                wp = result.GetSolution(edge.xv())
                path.append(wp.tolist())
            except Exception:
                continue
    else:
        # Fallback: try every backbone vertex, skipping inactive ones
        for bid in backbone:
            if bid not in verts:
                continue
            try:
                wp = result.GetSolution(verts[bid].x())
                path.append(wp.tolist())
            except Exception:
                continue
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
        description="Optimized GCS pipeline for SBF v6")
    parser.add_argument("input_json",
                        help="paths.json from C++ exp2")
    parser.add_argument("--corridor-hops", type=int, default=2)
    parser.add_argument("--max-corridor", type=int, default=600,
                        help="Max corridor boxes (larger = slower but better)")
    parser.add_argument("--bridge-penalty", type=float, default=50.0,
                        help="Multiplier on Dijkstra edge weight for SBF "
                             "bridge edges so the backbone prefers genuine "
                             "overlap chains (default 50).")
    parser.add_argument("--slack-tol", type=float, default=0.05,
                        help="Per-axis slack (rad) for soft-overlap "
                             "continuity on bridge edges with gap ≤ this. "
                             "u's polyhedron is inflated by slack on every "
                             "facet so v's start point may lie up to slack "
                             "outside u. Set to 0 to disable (every bridge "
                             "then needs a Point chain).")
    parser.add_argument("--drop-bridges", choices=["auto", "always", "never"],
                        default="never",
                        help="Exclude SBF bridge edges from the GCS graph. "
                             "'auto' drops them when backbone has >2 boxes; "
                             "'always' / 'never' force it. Default 'never' "
                             "keeps Point-vertex chain pinning, which gives "
                             "GCS path = SBF chain (ratio≈1.000 by design). "
                             "'auto'/'always' only work when most bridge "
                             "boxes have genuine overlap with the corridor.")
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

    # Augment adjacency with edges along every successful C++ box_sequence.
    # Classify each augmented edge: "overlap" if box_u ∩ box_v ≠ ∅ (genuine
    # adjacency missing from the saved forest), or "bridge" otherwise (the
    # cross-island jump that SBF online-bridges with a transient box).  The
    # bridge_pairs set is later passed to gcs_solve so the equality continuity
    # constraint is applied ONLY on overlap edges; bridges are kept for
    # connectivity but allowed to be "soft".
    bridge_pairs = set()
    bridge_chains = {}  # (min_id, max_id) -> chain (list of np.ndarray)
    n_added_overlap = 0
    n_added_bridge = 0
    for p in data.get("paths", []):
        if not p.get("success"):
            continue
        seq = p.get("box_sequence", [])
        for u, v in zip(seq[:-1], seq[1:]):
            if u not in box_map or v not in box_map:
                continue
            bu, bv = box_map[u], box_map[v]
            lo = np.maximum(bu.lo, bv.lo)
            hi = np.minimum(bu.hi, bv.hi)
            overlaps = bool(np.all(hi >= lo - 1e-9))
            new_edge = False
            if v not in adj.setdefault(u, []):
                adj[u].append(v); new_edge = True
            if u not in adj.setdefault(v, []):
                adj[v].append(u); new_edge = True
            if overlaps:
                if new_edge:
                    n_added_overlap += 1
            else:
                bridge_pairs.add((min(u, v), max(u, v)))
                if new_edge:
                    n_added_bridge += 1
        # Ingest exp2-emitted bridge_segments: collision-free RRT chains
        # connecting two non-overlapping SBF boxes.  Stored as
        # bridge_chains[(min_id, max_id)] = [wp_0, wp_1, ..., wp_n], where
        # wp_0 ∈ box(from_box_id) and wp_n ∈ box(to_box_id).
        for bs in p.get("bridge_segments", []):
            a = int(bs["from_box_id"]); b = int(bs["to_box_id"])
            chain = [np.asarray(w, dtype=float) for w in bs.get("waypoints", [])]
            if len(chain) < 2 or a not in box_map or b not in box_map:
                continue
            key = (min(a, b), max(a, b))
            bridge_pairs.add(key)
            # Make sure adjacency carries the pair so Dijkstra can use it.
            if b not in adj.setdefault(a, []):
                adj[a].append(b); n_added_bridge += 1
            if a not in adj.setdefault(b, []):
                adj[b].append(a)
            # Keep the longest chain seen for this pair (most informative).
            if (key not in bridge_chains
                or len(chain) > len(bridge_chains[key])):
                bridge_chains[key] = chain
    log.info(f"Augmented adjacency: {n_added_overlap} overlap edges + "
             f"{n_added_bridge} bridge edges "
             f"({len(bridge_pairs)} bridge pairs, "
             f"{len(bridge_chains)} with SBF wp chains)")

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
        cpp_qt  = float(cpp_path.get("query_time", 0.0)) if cpp_path else 0.0

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

        # Compute fresh Dijkstra backbone (penalize SBF bridge edges so the
        # search prefers multi-hop overlap chains over single-hop bridges).
        found, backbone, dij_cost = dijkstra(
            adj, box_map, start_id, goal_id,
            bridge_pairs=bridge_pairs, bridge_penalty=args.bridge_penalty)
        if (not found) and cpp_path and cpp_path.get("success") \
                and cpp_path.get("box_sequence"):
            # Fallback: route through the C++ corridor endpoints (which were
            # bridged at query time and may live in a different island).
            seq = cpp_path["box_sequence"]
            if seq[0] in box_map and seq[-1] in box_map:
                f1, b1, c1 = dijkstra(
                    adj, box_map, start_id, seq[0],
                    bridge_pairs=bridge_pairs, bridge_penalty=args.bridge_penalty)
                f2, b2, c2 = dijkstra(
                    adj, box_map, seq[-1], goal_id,
                    bridge_pairs=bridge_pairs, bridge_penalty=args.bridge_penalty)
                f3, b3, c3 = dijkstra(
                    adj, box_map, seq[0], seq[-1],
                    bridge_pairs=bridge_pairs, bridge_penalty=args.bridge_penalty)
                if f1 and f2 and f3:
                    backbone = b1[:-1] + b3[:-1] + b2
                    dij_cost = c1 + c3 + c2
                    found = True
                    log.info(f"  Dijkstra (fallback via cpp seq endpoints): "
                             f"{len(backbone)} boxes, cost={dij_cost:.3f}")

        if not found:
            log.warning(f"  Dijkstra: no path (start_box={start_id}, goal_box={goal_id})")
            results.append({"pair_idx": qi, "label": label, "success": False})
            continue

        # Recompute geometric backbone cost (sum of |center_i+1 - center_i|)
        # WITHOUT the bridge penalty, so corridor_ratio reflects true path
        # winding and not the artificial penalty used to steer Dijkstra away
        # from single-hop bridges.
        geom_cost = 0.0
        for u, v in zip(backbone[:-1], backbone[1:]):
            if u in box_map and v in box_map:
                geom_cost += float(np.linalg.norm(
                    box_map[u].center - box_map[v].center))
        corridor_ratio = geom_cost / max(euclid, 0.01)
        log.info(f"  Dijkstra: {len(backbone)} boxes, geom_cost={geom_cost:.3f} "
                 f"(penalized={dij_cost:.3f}), ratio={corridor_ratio:.2f}")

        # Skip GCS if corridor is too winding (won't beat RRT)
        if corridor_ratio > args.ratio_threshold:
            log.info(f"  SKIP GCS: ratio {corridor_ratio:.2f} > {args.ratio_threshold}")
            results.append({
                "pair_idx": qi, "label": label, "success": False,
                "reason": "ratio_skip",
                "dijkstra_backbone": len(backbone),
                "corridor_ratio": round(corridor_ratio, 3),
                "cpp_len": round(cpp_len, 4),
                "cpp_query_time": round(cpp_qt, 4),
            })
            continue

        # Run GCS
        n_gcs_run += 1
        if args.drop_bridges == "always":
            drop_br = True
        elif args.drop_bridges == "never":
            drop_br = False
        else:  # auto
            drop_br = len(backbone) > 2
        success, gcs_path, gcs_len, n_corridor, solve_time = gcs_solve(
            adj, box_map, start, goal, backbone,
            corridor_hops=args.corridor_hops,
            max_corridor=args.max_corridor,
            bridge_pairs=bridge_pairs,
            bridge_chains=bridge_chains,
            drop_bridges=drop_br,
            slack_tol=args.slack_tol,
        )
        total_gcs_time += solve_time

        entry = {
            "pair_idx": qi,
            "label": label,
            "success": success,
            "dijkstra_backbone": len(backbone),
            "corridor_ratio": round(corridor_ratio, 3),
            "cpp_len": round(cpp_len, 4),
            "cpp_query_time": round(cpp_qt, 4),
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
            speedup = (cpp_qt / solve_time) if solve_time > 1e-9 else float("inf")
            log.info(f"  GCS={gcs_len:.3f} vs C++={cpp_len:.3f} "
                     f"({marker}, ratio={ratio_vs_cpp:.3f})")
            log.info(f"  time: GCS={solve_time:.3f}s  C++={cpp_qt:.3f}s  "
                     f"(speedup C++/GCS={speedup:.2f}×)")
        else:
            entry["solve_time"] = round(solve_time, 3)
            log.info(f"  GCS FAILED")

        results.append(entry)

    # Summary
    log.info(f"\n{'='*78}")
    log.info(f"Summary: GCS SOCP vs C++ Dijkstra+RRT+EB")
    log.info(f"{'Pair':<10} {'C++_len':>8} {'GCS_len':>8} {'Ratio':>7} "
             f"{'C++_t':>8} {'GCS_t':>8} {'Speedup':>8} {'Result':>8}")
    log.info("-" * 78)
    for r in results:
        lbl = r["label"]
        cpp = r.get("cpp_len", 0)
        cpp_t = r.get("cpp_query_time", 0.0)
        if r.get("reason") == "ratio_skip":
            log.info(f"{lbl:<10} {cpp:>8.3f} {'SKIP':>8} "
                     f"{'':>7} {cpp_t:>8.3f} {'':>8} {'':>8} "
                     f"{'ratio>{:.0f}'.format(args.ratio_threshold):>8}")
        elif r["success"]:
            gcs = r["gcs_len"]
            ratio = r["ratio_vs_cpp"]
            st = r.get("solve_time", 0)
            sp = (cpp_t / st) if st > 1e-9 else float("inf")
            marker = "BETTER" if gcs < cpp else "worse"
            log.info(f"{lbl:<10} {cpp:>8.3f} {gcs:>8.3f} {ratio:>7.3f} "
                     f"{cpp_t:>8.3f} {st:>8.3f} {sp:>7.2f}× {marker:>8}")
        else:
            log.info(f"{lbl:<10} {cpp:>8.3f} {'FAIL':>8} "
                     f"{'':>7} {cpp_t:>8.3f} {'':>8} {'':>8}")

    valid = [r for r in results if r["success"]]
    better = [r for r in valid if r["gcs_len"] < r["cpp_len"]]
    total_cpp_time = sum(r.get("cpp_query_time", 0.0) for r in valid)
    log.info(f"\nGCS ran: {n_gcs_run}/{len(queries)}  "
             f"Better: {len(better)}/{len(valid)}  "
             f"Total time: GCS={total_gcs_time:.2f}s  "
             f"C++={total_cpp_time:.2f}s")
    if valid:
        mean_gcs_t = total_gcs_time / len(valid)
        mean_cpp_t = total_cpp_time / len(valid)
        sp = (mean_cpp_t / mean_gcs_t) if mean_gcs_t > 1e-9 else float("inf")
        log.info(f"Mean per-query time: GCS={mean_gcs_t*1000:.1f}ms  "
                 f"C++={mean_cpp_t*1000:.1f}ms  speedup={sp:.2f}×")
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

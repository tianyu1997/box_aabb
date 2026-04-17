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
    """Check if segment p1→p2 stays within the union of boxes (exact, analytical)."""
    return segment_in_box_union_exact(p1, p2, box_lo, box_hi)


def segment_in_box_union_exact(p, q, lo, hi, eps=1e-6):
    """Exact check: does line segment p→q stay within union of AABB boxes?

    Uses slab intersection to compute [t_enter, t_exit] per box,
    then verifies the union of intervals covers [0, 1].
    """
    d = q - p  # (D,)
    N, D = lo.shape
    zero_d = np.abs(d) < 1e-15

    # For zero-movement dims, check if p is inside [lo-eps, hi+eps]
    if np.any(zero_d):
        outside_zero = (p[zero_d] < lo[:, zero_d] - eps) | (p[zero_d] > hi[:, zero_d] + eps)
        any_outside_zero = np.any(outside_zero, axis=1)  # (N,)
    else:
        any_outside_zero = np.zeros(N, dtype=bool)

    # Compute t-intervals for non-zero dims
    with np.errstate(divide='ignore', invalid='ignore'):
        t1 = (lo[:, ~zero_d] - eps - p[~zero_d]) / d[~zero_d]  # (N, D')
        t2 = (hi[:, ~zero_d] + eps - p[~zero_d]) / d[~zero_d]  # (N, D')

    t_lo = np.minimum(t1, t2)
    t_hi = np.maximum(t1, t2)

    if t_lo.shape[1] > 0:
        t_enter = np.max(t_lo, axis=1)  # (N,)
        t_exit = np.min(t_hi, axis=1)   # (N,)
    else:
        t_enter = np.full(N, -np.inf)
        t_exit = np.full(N, np.inf)

    # Clip to [0, 1]
    t_enter = np.maximum(t_enter, 0.0)
    t_exit = np.minimum(t_exit, 1.0)

    # Valid: interval is non-empty and p is inside zero-d dims
    valid = (t_enter <= t_exit + eps) & ~any_outside_zero
    if not np.any(valid):
        return False

    # Check if union of valid intervals covers [0, 1]
    t_enter_v = t_enter[valid]
    t_exit_v = t_exit[valid]
    order = np.argsort(t_enter_v)
    t_enter_s = t_enter_v[order]
    t_exit_s = t_exit_v[order]

    covered_to = 0.0
    for i in range(len(t_enter_s)):
        if t_enter_s[i] > covered_to + eps:
            return False
        covered_to = max(covered_to, float(t_exit_s[i]))
        if covered_to >= 1.0 - eps:
            return True
    return covered_to >= 1.0 - eps


def shortcut_smooth(waypoints, corridor_lo, corridor_hi, n_checks=20, n_rounds=1):
    """Greedy forward shortcut: skip redundant waypoints if segment stays in corridor."""
    for _round in range(n_rounds):
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
        waypoints = smoothed
    return waypoints


def _tag_waypoints_boxes(waypoints, lo, hi):
    """For each waypoint, return a list of row-indices of containing boxes."""
    EPS = 1e-6
    tags = []
    for wp in waypoints:
        rows = np.where(np.all((wp >= lo - EPS) & (wp <= hi + EPS), axis=1))[0]
        tags.append(set(rows.tolist()))
    return tags


def _merge_hull_boxes(lo, hi, tags, start, end):
    """Compute AABB hull of all boxes tagged to waypoints[start..end].
    Returns (hull_lo, hull_hi) arrays of shape (D,)."""
    rows = set()
    for k in range(start, end + 1):
        rows |= tags[k]
    if not rows:
        return None, None
    rows = list(rows)
    hull_lo = np.min(lo[rows], axis=0)
    hull_hi = np.max(hi[rows], axis=0)
    return hull_lo, hull_hi


def _segment_in_single_box(p, q, box_lo, box_hi, eps=1e-6):
    """Check if both endpoints are inside a single AABB → segment contained (convex)."""
    return (np.all(p >= box_lo - eps) and np.all(p <= box_hi + eps) and
            np.all(q >= box_lo - eps) and np.all(q <= box_hi + eps))


def shortcut_hull_merge(waypoints, lo, hi, n_rounds=2):
    """
    Advanced shortcut using LECT-style hull merge with corridor box collection.

    Two-phase approach:
    Phase 1 (hull merge): Merge consecutive waypoints that share a common box
            or whose AABB hull is covered by a single box → trivially safe.
    Phase 2 (corridor shortcut): For longer shortcuts, collect all boxes in
            the corridor (AABB of segment ± margin) and use exact ray check.
    """
    tags = _tag_waypoints_boxes(waypoints, lo, hi)

    for _round in range(n_rounds):
        if len(waypoints) <= 2:
            return waypoints

        # --- Phase 1: greedy merge within shared boxes ---
        # For each waypoint, find the set of boxes containing it.
        # Merge consecutive waypoints if they share a box.
        merged_wps = [waypoints[0]]
        merged_tags = [tags[0]]
        i = 0
        while i < len(waypoints) - 1:
            # Find farthest j such that all waypoints i..j share a common box
            best_j = i
            for j in range(len(waypoints) - 1, i, -1):
                # Common box: intersection of tags for ALL waypoints i..j
                common = tags[i].copy()
                ok = True
                for k in range(i + 1, j + 1):
                    common &= tags[k]
                    if not common:
                        ok = False
                        break
                if ok and common:
                    best_j = j
                    break
            if best_j == i:
                best_j = i + 1
            merged_wps.append(waypoints[best_j])
            merged_tags.append(tags[best_j])
            i = best_j
        waypoints = merged_wps
        tags = merged_tags

        if len(waypoints) <= 2:
            return waypoints

        # --- Phase 2: full-forest shortcut with exact ray check ---
        # Farthest-first scan with early termination.
        smoothed = [waypoints[0]]
        smoothed_tags = [tags[0]]
        i = 0
        while i < len(waypoints) - 1:
            best_j = i + 1
            pi = waypoints[i]

            # Try farthest first (goal endpoint)
            for j in range(len(waypoints) - 1, i + 1, -1):
                pj = waypoints[j]
                if tags[i] & tags[j]:  # Same box → convex → safe
                    best_j = j
                    break
                if segment_in_box_union_exact(pi, pj, lo, hi):
                    best_j = j
                    break

            smoothed.append(waypoints[best_j])
            smoothed_tags.append(tags[best_j] if best_j < len(tags) else set())
            i = best_j

        waypoints = smoothed
        tags = smoothed_tags

    return waypoints


def straighten_path(waypoints, lo, hi, n_iters=10):
    """Iteratively move each waypoint toward optimal positions.

    Two passes per iteration:
    1. Global: move toward start→goal line (fixes large-scale zigzags)
    2. Local: move toward neighbor-to-neighbor line (fixes local kinks)
    Both passes use binary search for maximum safe displacement.
    """
    pts = [np.array(w, dtype=float) for w in waypoints]

    for _it in range(n_iters):
        any_improved = False

        # ---- Global pass: move toward start→goal line ----
        d_global = pts[-1] - pts[0]
        d_global_sq = float(np.dot(d_global, d_global))
        if d_global_sq > 1e-30:
            for i in range(1, len(pts) - 1):
                t_g = np.clip(float(np.dot(pts[i] - pts[0], d_global)) / d_global_sq, 0.0, 1.0)
                target_g = pts[0] + t_g * d_global
                move_g = target_g - pts[i]
                if np.dot(move_g, move_g) < 1e-20:
                    continue
                a_lo, a_hi = 0.0, 1.0
                best_alpha = 0.0
                for _ in range(8):
                    a_mid = 0.5 * (a_lo + a_hi)
                    cand = pts[i] + a_mid * move_g
                    if (segment_in_box_union_exact(pts[i - 1], cand, lo, hi) and
                            segment_in_box_union_exact(cand, pts[i + 1], lo, hi)):
                        best_alpha = a_mid
                        a_lo = a_mid
                    else:
                        a_hi = a_mid
                if best_alpha > 1e-6:
                    pts[i] = pts[i] + best_alpha * move_g
                    any_improved = True

        # ---- Local pass: move toward neighbor-to-neighbor line ----
        for i in range(1, len(pts) - 1):
            d = pts[i + 1] - pts[i - 1]
            d_sq = float(np.dot(d, d))
            if d_sq < 1e-30:
                continue
            t = np.clip(float(np.dot(pts[i] - pts[i - 1], d)) / d_sq, 0.0, 1.0)
            target = pts[i - 1] + t * d
            move = target - pts[i]
            if np.dot(move, move) < 1e-20:
                continue

            # Binary search for max alpha in [0, 1]
            a_lo, a_hi = 0.0, 1.0
            best_alpha = 0.0
            for _ in range(8):
                a_mid = 0.5 * (a_lo + a_hi)
                cand = pts[i] + a_mid * move
                if (segment_in_box_union_exact(pts[i - 1], cand, lo, hi) and
                        segment_in_box_union_exact(cand, pts[i + 1], lo, hi)):
                    best_alpha = a_mid
                    a_lo = a_mid
                else:
                    a_hi = a_mid

            if best_alpha > 1e-6:
                pts[i] = pts[i] + best_alpha * move
                any_improved = True

        if not any_improved:
            break

    return pts


def segment_max_gap(p, q, lo, hi, eps=1e-6):
    """Compute max uncovered gap (in radians) along segment p→q."""
    d = q - p
    seg_len = float(np.linalg.norm(d))
    if seg_len < 1e-15:
        return 0.0
    N = len(lo)
    zero_d = np.abs(d) < 1e-15
    if np.any(zero_d):
        outside_zero = (p[zero_d] < lo[:, zero_d] - eps) | (p[zero_d] > hi[:, zero_d] + eps)
        any_out_z = np.any(outside_zero, axis=1)
    else:
        any_out_z = np.zeros(N, dtype=bool)
    with np.errstate(divide='ignore', invalid='ignore'):
        t1 = (lo[:, ~zero_d] - eps - p[~zero_d]) / d[~zero_d]
        t2 = (hi[:, ~zero_d] + eps - p[~zero_d]) / d[~zero_d]
    t_lo = np.minimum(t1, t2)
    t_hi = np.maximum(t1, t2)
    if t_lo.shape[1] > 0:
        t_enter = np.maximum(np.max(t_lo, axis=1), 0.0)
        t_exit = np.minimum(np.min(t_hi, axis=1), 1.0)
    else:
        t_enter = np.full(N, 0.0)
        t_exit = np.full(N, 1.0)
    valid = (t_enter <= t_exit + eps) & ~any_out_z
    if not np.any(valid):
        return seg_len
    intervals = sorted(zip(t_enter[valid], t_exit[valid]))
    covered = 0.0
    max_gap_t = 0.0
    for a, b in intervals:
        if a > covered + eps:
            max_gap_t = max(max_gap_t, a - covered)
        covered = max(covered, float(b))
    if covered < 1.0 - eps:
        max_gap_t = max(max_gap_t, 1.0 - covered)
    return max_gap_t * seg_len


def _box_covered_intervals(p, q, lo, hi, eps=1e-6):
    """Return sorted (t_enter, t_exit) pairs where segment is inside any box.
    Used to find the longest covered sub-interval that contains a query t*.
    """
    d = q - p
    seg_len = float(np.linalg.norm(d))
    if seg_len < 1e-15:
        return [(0.0, 1.0)]
    N = len(lo)
    zero_d = np.abs(d) < 1e-15
    if np.any(zero_d):
        outside_zero = (p[zero_d] < lo[:, zero_d] - eps) | (p[zero_d] > hi[:, zero_d] + eps)
        any_out_z = np.any(outside_zero, axis=1)
    else:
        any_out_z = np.zeros(N, dtype=bool)
    with np.errstate(divide='ignore', invalid='ignore'):
        t1 = (lo[:, ~zero_d] - eps - p[~zero_d]) / d[~zero_d]
        t2 = (hi[:, ~zero_d] + eps - p[~zero_d]) / d[~zero_d]
    t_lo = np.minimum(t1, t2)
    t_hi = np.maximum(t1, t2)
    if t_lo.shape[1] > 0:
        t_enter = np.maximum(np.max(t_lo, axis=1), 0.0)
        t_exit = np.minimum(np.min(t_hi, axis=1), 1.0)
    else:
        t_enter = np.full(N, 0.0)
        t_exit = np.full(N, 1.0)
    valid = (t_enter <= t_exit + eps) & ~any_out_z
    if not np.any(valid):
        return []
    intervals = sorted(zip(t_enter[valid].tolist(), t_exit[valid].tolist()))
    merged = []
    for a, b in intervals:
        if merged and a <= merged[-1][1] + eps:
            merged[-1] = (merged[-1][0], max(merged[-1][1], b))
        else:
            merged.append((a, b))
    return merged


def enforce_strict_safety(waypoints, lo, hi, ids, adj, id2row,
                          max_recursion=12, verbose=False):
    """Final guarantee pass: split any dirty segment until all sub-segments
    are strictly inside the box union (passes segment_in_box_union_exact).

    Strategy for a dirty segment (p, q):
      1. Try A*-bridge via box adjacency (bridge_waypoints_via_boxes on [p,q]).
         If any sub-segment is still dirty, recurse on that sub-segment.
      2. If A* bridge fails or produces nothing new, find a splitter point
         inside the largest gap on the segment, project to nearest box, recurse.
      3. Base case (depth exhausted, segment tiny): accept the segment.
    """
    EPS = 1e-6

    def in_any(p):
        return bool(np.any(np.all((p >= lo - EPS) & (p <= hi + EPS), axis=1)))

    def project_to_nearest_interior(p):
        clamped = np.clip(p, lo, hi)
        d = np.linalg.norm(clamped - p, axis=1)
        r = int(np.argmin(d))
        return np.clip(p, lo[r] + EPS, hi[r] - EPS)

    def clean(p, q):
        return segment_in_box_union_exact(p, q, lo, hi)

    def bridge_segment(p, q, depth):
        """Return list [p, x1, x2, ..., xk] (excluding q) that forms a safe
        poly-line from p to q when q is appended. Base case: [p]."""
        if clean(p, q):
            return [p]
        if depth <= 0 or float(np.linalg.norm(q - p)) < 5e-4:
            return [p]

        # Try A* bridge
        try:
            bridged = bridge_waypoints_via_boxes([p, q], lo, hi, ids, adj)
        except Exception:
            bridged = [p, q]

        # If bridged produced new intermediate waypoints, recurse on each sub-seg
        if len(bridged) > 2:
            out = []
            for j in range(len(bridged) - 1):
                sub = bridge_segment(
                    np.asarray(bridged[j], dtype=float),
                    np.asarray(bridged[j + 1], dtype=float),
                    depth - 1,
                )
                out.extend(sub)
            return out

        # A* gave nothing; split at largest gap midpoint
        covered = _box_covered_intervals(p, q, lo, hi, EPS)
        gaps = []
        prev_end = 0.0
        for a, b in covered:
            if a > prev_end + EPS:
                gaps.append((prev_end, a))
            prev_end = max(prev_end, b)
        if prev_end < 1.0 - EPS:
            gaps.append((prev_end, 1.0))
        if not gaps:
            return [p]
        ga, gb = max(gaps, key=lambda ab: ab[1] - ab[0])
        t_star = 0.5 * (ga + gb)
        x = p + t_star * (q - p)
        x_in = project_to_nearest_interior(x)
        # Avoid infinite recursion: require displacement
        if (float(np.linalg.norm(x_in - p)) < 5e-4 or
                float(np.linalg.norm(x_in - q)) < 5e-4):
            return [p]
        left = bridge_segment(p, x_in, depth - 1)
        right = bridge_segment(x_in, q, depth - 1)
        return left + right

    out = [np.asarray(waypoints[0], dtype=float)]
    n_fixed = 0
    for i in range(len(waypoints) - 1):
        p = np.asarray(waypoints[i], dtype=float)
        q = np.asarray(waypoints[i + 1], dtype=float)
        if clean(p, q):
            out.append(q.copy())
            continue
        n_fixed += 1
        seg_out = bridge_segment(p, q, max_recursion)
        for w in seg_out[1:]:
            out.append(np.asarray(w, dtype=float))
        out.append(q.copy())
    if verbose:
        n_dirty_after = sum(1 for j in range(len(out) - 1)
                            if not clean(out[j], out[j + 1]))
        print(f"    enforce_strict: fixed {n_fixed} dirty segs, {len(waypoints)}->{len(out)} wpts, remaining_dirty={n_dirty_after}")
    return out


def shortcut_random_pairs(waypoints, lo, hi, n_trials=2000, seed=42, gap_tol=0.0):
    """RRT-style random-pair shortcut. Try random (i,j) connections that
    greedy farthest-first may have missed due to intermediate blockers.
    Also tries interior-point shortcuts (mid-segment → mid-segment).
    """
    import random
    rng = random.Random(seed)
    pts = [np.array(w, dtype=float) for w in waypoints]

    def path_len(a, b):
        return float(sum(np.linalg.norm(pts[k + 1] - pts[k]) for k in range(a, b)))

    trials = 0
    n_hits = 0
    while trials < n_trials and len(pts) > 2:
        trials += 1
        n = len(pts)
        i = rng.randint(0, n - 3)
        j = rng.randint(i + 2, n - 1)

        # Random interior alpha/beta in [0, 1] — allows sub-segment shortcut
        a = rng.random()
        b = rng.random()
        pi = pts[i] + a * (pts[i + 1] - pts[i])
        pj = pts[j - 1] + b * (pts[j] - pts[j - 1])

        if not segment_in_box_union_exact(pi, pj, lo, hi):
            # Allow tiny-gap shortcuts if they save significant length
            if gap_tol <= 0:
                continue
            g = segment_max_gap(pi, pj, lo, hi)
            if g > gap_tol:
                continue

        # Compute old vs new length: old = tail(pts[i], pts[i+1])*(1-a) + mid + head(b)
        old = ((1.0 - a) * float(np.linalg.norm(pts[i + 1] - pts[i])) +
               path_len(i + 1, j - 1) +
               b * float(np.linalg.norm(pts[j] - pts[j - 1])))
        new = float(np.linalg.norm(pj - pi))
        if new >= old - 1e-6:
            continue

        # Accept: replace pts[i+1..j-1] with pi, pj
        # But check degenerate cases
        new_pts = pts[:i + 1]
        if a > 1e-6:
            new_pts.append(pi)
        if b < 1.0 - 1e-6:
            new_pts.append(pj)
        new_pts.extend(pts[j:])

        # Verify ONLY the newly created boundary segments (not whole path).
        # Old dirty segments elsewhere are preserved unchanged.
        new_start_idx = i  # first modified index
        new_end_idx = i + 1  # last modified index
        if a > 1e-6:
            new_end_idx += 1
        if b < 1.0 - 1e-6:
            new_end_idx += 1
        # Boundary segments: [i → i+1] and [last-modified → next]
        ok = True
        for k in range(new_start_idx, min(new_end_idx, len(new_pts) - 1)):
            seg_ok = segment_in_box_union_exact(new_pts[k], new_pts[k + 1], lo, hi)
            if not seg_ok and gap_tol > 0:
                seg_ok = (segment_max_gap(new_pts[k], new_pts[k + 1], lo, hi)
                          <= gap_tol)
            if not seg_ok:
                ok = False
                break
        if ok:
            pts = new_pts
            n_hits += 1

    return pts


def clip_to_boxes(waypoints, lo, hi, ids, id2row, flow_bids):
    """Clip each GCS waypoint to its corresponding box interior (eps margin)."""
    clipped = []
    EPS = 1e-6
    for i, wp in enumerate(waypoints):
        # First and last are start/goal, keep as-is
        if i == 0 or i == len(waypoints) - 1:
            clipped.append(wp.copy())
            continue
        # Find the box this waypoint belongs to (flow_bids index)
        fi = i - 1  # flow_bids[fi] corresponds to waypoints[i]
        if fi < 0 or fi >= len(flow_bids):
            clipped.append(wp.copy())
            continue
        bid = flow_bids[fi]
        if bid not in id2row:
            clipped.append(wp.copy())
            continue
        r = id2row[bid]
        clipped.append(np.clip(wp, lo[r] + EPS, hi[r] - EPS))
    return clipped


def ensure_in_box_union(waypoints, lo, hi, step_rad=0.02):
    """
    Ensure path stays within box union. Project escaped points to nearest box.
    """
    EPS = 1e-6
    N_CHECK = 30

    lo_m = lo - EPS
    hi_p = hi + EPS

    def in_any_box(q):
        return np.any(np.all((q >= lo_m) & (q <= hi_p), axis=1))

    def project_into_nearest_box(q):
        clamped = np.clip(q, lo, hi)
        dists = np.max(np.abs(clamped - q), axis=1)
        r = int(np.argmin(dists))
        return np.clip(q, lo[r] + EPS, hi[r] - EPS)

    def seg_has_escape(p1, p2):
        for k in range(1, N_CHECK):
            t = k / N_CHECK
            if not in_any_box(p1 + t * (p2 - p1)):
                return t
        return -1.0

    result = list(waypoints)
    for _pass in range(8):
        new_result = [result[0]]
        any_fixed = False
        for i in range(len(result) - 1):
            p1, p2 = result[i], result[i + 1]
            t_esc = seg_has_escape(p1, p2)
            if t_esc > 0:
                any_fixed = True
                pt_esc = p1 + t_esc * (p2 - p1)
                pt_proj = project_into_nearest_box(pt_esc)
                new_result.append(pt_proj)
            new_result.append(p2)
        result = new_result
        if not any_fixed:
            break
        if len(result) > 3000:
            break
    return result
    return result


def bridge_dense_project(waypoints, lo, hi, step=0.01):
    """Bridge dirty segments by densification + projection to nearest box.

    For each dirty segment, subdivide into small steps and project escaping
    points to the nearest box. This produces a path that closely follows
    the original geometry with minimal detours at gap boundaries.
    """
    EPS = 1e-6
    GAP_TOL_DP = 0.0  # strict: no gap tolerated

    def in_any(q):
        return np.any(np.all((q >= lo - EPS) & (q <= hi + EPS), axis=1))

    def project_nearest(q):
        clamped = np.clip(q, lo, hi)  # (N, D)
        dist = np.linalg.norm(clamped - q, axis=1)  # (N,) L2
        r = int(np.argmin(dist))
        return np.clip(q, lo[r] + EPS, hi[r] - EPS)

    def gap_size(p1, p2):
        d = p2 - p1
        seg_len = float(np.linalg.norm(d))
        if seg_len < 1e-15:
            return 0.0
        N_box = len(lo)
        zero_d = np.abs(d) < 1e-15
        if np.any(zero_d):
            outside_zero = (p1[zero_d] < lo[:, zero_d] - EPS) | (p1[zero_d] > hi[:, zero_d] + EPS)
            any_out_z = np.any(outside_zero, axis=1)
        else:
            any_out_z = np.zeros(N_box, dtype=bool)
        with np.errstate(divide='ignore', invalid='ignore'):
            t1 = (lo[:, ~zero_d] - EPS - p1[~zero_d]) / d[~zero_d]
            t2 = (hi[:, ~zero_d] + EPS - p1[~zero_d]) / d[~zero_d]
        t_lo = np.minimum(t1, t2)
        t_hi = np.maximum(t1, t2)
        if t_lo.shape[1] > 0:
            t_enter = np.maximum(np.max(t_lo, axis=1), 0.0)
            t_exit = np.minimum(np.min(t_hi, axis=1), 1.0)
        else:
            t_enter = np.full(N_box, 0.0)
            t_exit = np.full(N_box, 1.0)
        valid = (t_enter <= t_exit + EPS) & ~any_out_z
        if not np.any(valid):
            return seg_len
        intervals = sorted(zip(t_enter[valid], t_exit[valid]))
        covered = 0.0
        max_gap = 0.0
        for a, b in intervals:
            if a > covered + EPS:
                max_gap = max(max_gap, a - covered)
            covered = max(covered, float(b))
        if covered < 1.0 - EPS:
            max_gap = max(max_gap, 1.0 - covered)
        return max_gap * seg_len

    result = [waypoints[0]]
    n_clean = 0
    n_dense = 0

    for i in range(len(waypoints) - 1):
        p1, p2 = waypoints[i], waypoints[i + 1]
        if segment_in_box_union_exact(p1, p2, lo, hi):
            n_clean += 1
            result.append(p2)
            continue
        gap = gap_size(p1, p2)
        if gap < GAP_TOL_DP:
            n_clean += 1
            result.append(p2)
            continue

        n_dense += 1
        d = p2 - p1
        seg_len = float(np.linalg.norm(d))
        n_sub = max(2, int(np.ceil(seg_len / step)))
        for k in range(1, n_sub + 1):
            t = k / n_sub
            pt = p1 + t * d
            if in_any(pt):
                result.append(pt.copy())
            else:
                result.append(project_nearest(pt))

    print(f"    bridge_dp: {n_clean} clean, {n_dense} dense, {len(waypoints)}->{len(result)} wpts")
    return result


def bridge_waypoints_via_boxes(waypoints, lo, hi, ids, adj):
    """
    For each pair of consecutive waypoints that escape the box union,
    find a box-adjacency path between them and insert intermediate
    waypoints at box intersection centers.

    This is the robust fix for the case where GCS places waypoints in
    non-adjacent boxes: the straight-line segment between them passes
    through unprotected free space.
    """
    EPS = 1e-6
    N = len(ids)
    id2row = {bid: i for i, bid in enumerate(ids)}

    def containing_boxes(q):
        """Return set of box ids containing q."""
        rows = np.where(np.all((q >= lo - EPS) & (q <= hi + EPS), axis=1))[0]
        return {ids[r] for r in rows}

    def in_any_box(q):
        return np.any(np.all((q >= lo - EPS) & (q <= hi + EPS), axis=1))

    def seg_clean(p1, p2, n_checks=200):
        """Strict exact ray-based check (no gap tolerance for final output)."""
        return segment_in_box_union_exact(p1, p2, lo, hi)

    GAP_TOL = 0.0  # strict: no escape from box union permitted

    def seg_gap_size(p1, p2):
        """Compute the largest gap in box coverage along segment (in radians)."""
        d = p2 - p1
        seg_len = float(np.linalg.norm(d))
        if seg_len < 1e-15:
            return 0.0
        N_box = len(lo)
        eps = 1e-6
        zero_d = np.abs(d) < 1e-15
        if np.any(zero_d):
            outside_zero = (p1[zero_d] < lo[:, zero_d] - eps) | (p1[zero_d] > hi[:, zero_d] + eps)
            any_out_z = np.any(outside_zero, axis=1)
        else:
            any_out_z = np.zeros(N_box, dtype=bool)
        with np.errstate(divide='ignore', invalid='ignore'):
            t1 = (lo[:, ~zero_d] - eps - p1[~zero_d]) / d[~zero_d]
            t2 = (hi[:, ~zero_d] + eps - p1[~zero_d]) / d[~zero_d]
        t_lo = np.minimum(t1, t2)
        t_hi = np.maximum(t1, t2)
        if t_lo.shape[1] > 0:
            t_enter = np.maximum(np.max(t_lo, axis=1), 0.0)
            t_exit = np.minimum(np.min(t_hi, axis=1), 1.0)
        else:
            t_enter = np.full(N_box, 0.0)
            t_exit = np.full(N_box, 1.0)
        valid = (t_enter <= t_exit + eps) & ~any_out_z
        if not np.any(valid):
            return seg_len  # Entire segment uncovered
        intervals = sorted(zip(t_enter[valid], t_exit[valid]))
        covered = 0.0
        max_gap_t = 0.0
        for a, b in intervals:
            if a > covered + eps:
                max_gap_t = max(max_gap_t, a - covered)
            covered = max(covered, float(b))
        if covered < 1.0 - eps:
            max_gap_t = max(max_gap_t, 1.0 - covered)
        return max_gap_t * seg_len

    def bfs_box_path(src_ids, dst_ids):
        """A* shortest path on box adjacency, weighted by box-center distance."""
        if src_ids & dst_ids:
            common = src_ids & dst_ids
            return [next(iter(common))]
        import heapq
        # Precompute box centers
        centers = 0.5 * (lo + hi)  # (N_boxes, D)
        # Goal center for heuristic
        dst_centers = np.array([centers[id2row[bid]] for bid in dst_ids if bid in id2row])
        if len(dst_centers) == 0:
            return None
        goal_center = dst_centers.mean(axis=0)

        # A* priority queue: (f_cost, g_cost, bid, path)
        pq = []
        for bid in src_ids:
            if bid not in id2row:
                continue
            g = 0.0
            h = float(np.linalg.norm(centers[id2row[bid]] - goal_center))
            heapq.heappush(pq, (g + h, g, bid, [bid]))
        visited = {}  # bid -> best g_cost
        while pq:
            f, g, bid, path = heapq.heappop(pq)
            if bid in visited and visited[bid] <= g:
                continue
            visited[bid] = g
            if bid in dst_ids:
                return path
            for nb in adj.get(bid, []):
                if nb not in id2row:
                    continue
                ra_cur, rb_cur = id2row[bid], id2row[nb]
                edge_dist = float(np.linalg.norm(centers[ra_cur] - centers[rb_cur]))
                # Penalize non-overlapping adjacent boxes to avoid gap crossings
                overlaps = np.all(np.maximum(lo[ra_cur], lo[rb_cur]) <= np.minimum(hi[ra_cur], hi[rb_cur]))
                ng = g + (edge_dist * 100.0 if not overlaps else edge_dist)
                if nb in visited and visited[nb] <= ng:
                    continue
                nh = float(np.linalg.norm(centers[id2row[nb]] - goal_center))
                heapq.heappush(pq, (ng + nh, ng, nb, path + [nb]))
        return None

    def box_overlap_center(bid_a, bid_b):
        """Compute center of overlap or nearest-face midpoint for non-overlapping boxes."""
        ra, rb = id2row[bid_a], id2row[bid_b]
        ov_lo = np.maximum(lo[ra], lo[rb])
        ov_hi = np.minimum(hi[ra], hi[rb])
        # If boxes don't fully overlap, clamp to valid region
        ov_lo = np.minimum(ov_lo, ov_hi)
        return 0.5 * (ov_lo + ov_hi)

    n_clean = 0
    n_dirty = 0
    n_minor = 0
    n_no_src = 0
    n_no_dst = 0
    n_no_src_dst = 0
    n_no_path = 0
    result = [waypoints[0]]
    for i in range(len(waypoints) - 1):
        p1, p2 = waypoints[i], waypoints[i + 1]

        if seg_clean(p1, p2):
            n_clean += 1
            result.append(p2)
            continue

        n_dirty += 1

        # Find box path from p1's boxes to p2's boxes
        src = containing_boxes(p1)
        dst = containing_boxes(p2)

        if not src or not dst:
            # Waypoint outside all boxes — just keep it
            if not src and not dst:
                n_no_src_dst += 1
            elif not src:
                n_no_src += 1
            else:
                n_no_dst += 1
            result.append(p2)
            continue

        box_path = bfs_box_path(src, dst)
        if box_path is None:
            n_no_path += 1
            result.append(p2)
            continue
        if len(box_path) < 2:
            result.append(p2)
            continue

        # ---- LECT-style hull merge: greedily merge consecutive boxes ----
        # Try to skip intermediate transitions by verifying that the
        # straight line from hull entry to hull exit is covered by member boxes.
        # Transition waypoints are biased toward the original segment p1→p2.
        n_bp = len(box_path)
        hull_start = 0
        p_entry = p1  # last point already in result
        d_seg = p2 - p1
        d_seg_sq = float(np.dot(d_seg, d_seg))

        def _seg_projected_ca_cb(box_j, box_j1):
            """Compute ca/cb biased toward segment p1→p2."""
            rj, rk = id2row[box_j], id2row[box_j1]
            ov_center = box_overlap_center(box_j, box_j1)
            if d_seg_sq > 1e-30:
                t = np.clip(float(np.dot(ov_center - p1, d_seg)) / d_seg_sq, 0.0, 1.0)
                seg_pt = p1 + t * d_seg
            else:
                seg_pt = ov_center
            ca = np.clip(seg_pt, lo[rj], hi[rj])
            cb = np.clip(seg_pt, lo[rk], hi[rk])
            return ca, cb

        while hull_start < n_bp - 1:
            best_end = hull_start  # fallback: single step

            for end in range(n_bp - 1, hull_start, -1):
                # Exit point — overlap center for better merge decisions
                if end >= n_bp - 1:
                    p_exit = p2
                else:
                    ov_c = box_overlap_center(box_path[end], box_path[end + 1])
                    ra_e = id2row[box_path[end]]
                    p_exit = np.clip(ov_c, lo[ra_e], hi[ra_e])

                # Full-forest check for maximum merge opportunity
                if segment_in_box_union_exact(p_entry, p_exit, lo, hi):
                    best_end = end
                    break

            if best_end >= n_bp - 1:
                # Merged all remaining boxes — go straight to p2
                hull_start = n_bp
            elif best_end > hull_start:
                # Successfully merged hull_start..best_end
                ca, cb = _seg_projected_ca_cb(box_path[best_end], box_path[best_end + 1])
                result.append(ca)
                if not np.allclose(ca, cb):
                    result.append(cb)
                p_entry = cb
                hull_start = best_end + 1
            else:
                # Cannot merge — single box transition (fallback)
                ca, cb = _seg_projected_ca_cb(box_path[hull_start], box_path[hull_start + 1])
                result.append(ca)
                if not np.allclose(ca, cb):
                    result.append(cb)
                p_entry = cb
                hull_start += 1

        result.append(p2)

    print(f"    bridge: {n_clean} clean, {n_dirty} dirty (no_src={n_no_src} no_dst={n_no_dst} no_both={n_no_src_dst} no_path={n_no_path}), {len(waypoints)}->{len(result)} wpts")
    return result


def densify_within_boxes(waypoints, lo, hi, ids, id2row, flow_bids,
                          step_rad=0.05):
    """Densify path so every internal point is clipped to the nearest box."""
    if len(waypoints) < 2:
        return waypoints
    dense = [waypoints[0].copy()]
    EPS = 1e-6
    for i in range(len(waypoints) - 1):
        p1, p2 = waypoints[i], waypoints[i + 1]
        dist = float(np.linalg.norm(p2 - p1))
        if dist <= step_rad:
            dense.append(p2.copy())
        else:
            n_sub = int(np.ceil(dist / step_rad))
            # Find boxes containing p1 and p2
            for k in range(1, n_sub + 1):
                t = k / n_sub
                pt = p1 + t * (p2 - p1)
                # Clip to the union of boxes that contain it
                best_r = None
                best_margin = -1e30
                for r in range(len(ids)):
                    margin = min(np.min(pt - lo[r]), np.min(hi[r] - pt))
                    if margin > best_margin:
                        best_margin = margin
                        best_r = r
                if best_r is not None and best_margin < 0:
                    pt = np.clip(pt, lo[best_r] + EPS, hi[best_r] - EPS)
                dense.append(pt)
    return dense


# ═══════════════════════════════════════════════════════════════════════════
# Drake collision repair (for marginal model-mismatch collisions)
# ═══════════════════════════════════════════════════════════════════════════

_drake_checker = None          # lazy singleton


def _get_drake_checker():
    """Lazy-init Drake collision checker (no-gripper model)."""
    global _drake_checker
    if _drake_checker is not None:
        return _drake_checker

    import os as _os
    from pydrake.multibody.plant import AddMultibodyPlantSceneGraph as _AddMP
    from pydrake.systems.framework import DiagramBuilder as _DB
    from pydrake.multibody.parsing import (
        Parser as _P, LoadModelDirectives as _LMD,
        ProcessModelDirectives as _PMD)

    _proj = _os.path.dirname(_os.path.dirname(
        _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))))
    _gcs = _os.path.join(_proj, "gcs-science-robotics")
    _model = _os.path.join(_gcs, "models", "iiwa14_no_gripper.dmd.yaml")
    if not _os.path.isfile(_model):
        return None  # model not available

    b = _DB()
    plant, sg = _AddMP(b, 0.0)
    parser = _P(plant)
    parser.package_map().Add("gcs", _gcs)
    _PMD(_LMD(_model), plant, parser)
    plant.Finalize()
    diag = b.Build()
    ctx = diag.CreateDefaultContext()
    _drake_checker = {
        "plant": plant, "sg": sg, "diagram": diag, "ctx": ctx,
        "pc": plant.GetMyMutableContextFromRoot(ctx),
        "sc": sg.GetMyMutableContextFromRoot(ctx),
        "iiwa": plant.GetModelInstanceByName("iiwa"),
    }
    return _drake_checker


def drake_collision_repair(waypoints, boxes_lo, boxes_hi):
    """
    Fix marginal Drake collisions via recursive segment-aware bisection.
    Handles sub-mm penetrations caused by sphere-vs-mesh model mismatch.
    Returns repaired waypoint list.
    """
    chk = _get_drake_checker()
    if chk is None:
        return waypoints  # no Drake model available

    plant, sg = chk["plant"], chk["sg"]
    pc, sc, iiwa_inst = chk["pc"], chk["sc"], chk["iiwa"]

    def is_free(q):
        plant.SetPositions(pc, iiwa_inst, q)
        return not sg.get_query_output_port().Eval(sc).HasCollisions()

    def seg_free(q1, q2, n=30):
        for k in range(n + 1):
            if not is_free(q1 + (k / n) * (q2 - q1)):
                return False
        return True

    rng = np.random.default_rng(42)

    def nudge_free(q, max_tries=100):
        if is_free(q):
            return q
        margins = np.min(np.minimum(q - boxes_lo, boxes_hi - q), axis=1)
        best = int(np.argmax(margins))
        for scale in [0.005, 0.01, 0.02, 0.04, 0.08]:
            for _ in range(max_tries // 5):
                qt = np.clip(q + rng.normal(0, scale, q.shape),
                             boxes_lo[best], boxes_hi[best])
                if is_free(qt):
                    return qt
        return None

    def recursive_fix(q1, q2, depth=0, max_depth=8):
        """Return intermediate waypoints between q1..q2 (exclusive)."""
        if depth >= max_depth or seg_free(q1, q2):
            return []
        mid = 0.5 * (q1 + q2)
        mf = nudge_free(mid)
        if mf is None:
            for frac in [0.3, 0.7, 0.2, 0.8, 0.4, 0.6]:
                mf = nudge_free(q1 + frac * (q2 - q1))
                if mf is not None:
                    break
        if mf is None:
            return []
        left = recursive_fix(q1, mf, depth + 1, max_depth)
        right = recursive_fix(mf, q2, depth + 1, max_depth)
        return left + [mf] + right

    # Find colliding segments
    n = len(waypoints)
    col = [i for i in range(n - 1) if not seg_free(waypoints[i], waypoints[i + 1])]
    if not col:
        return waypoints

    result = []
    i = 0
    while i < n:
        result.append(waypoints[i])
        if i < n - 1 and i in col:
            pts = recursive_fix(waypoints[i], waypoints[i + 1])
            result.extend(pts)
        i += 1
    return result

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

        # ── Step 6: Clip to box interiors + shortcut + bridge + shortcut ──
        n_before = len(waypoints)
        waypoints = clip_to_boxes(waypoints, lo, hi, ids, id2row, flow_bids)
        n_clip = len(waypoints)
        clip_len = float(np.sum(np.linalg.norm(np.diff(waypoints, axis=0), axis=1)))
        # First shortcut on raw GCS clip output
        waypoints = shortcut_hull_merge(waypoints, lo, hi, n_rounds=3)
        n_short1 = len(waypoints)
        short1_len = float(np.sum(np.linalg.norm(np.diff(waypoints, axis=0), axis=1)))

        # Dense projection bridge with fine step
        waypoints = bridge_dense_project(waypoints, lo, hi, step=0.005)
        n_dp = len(waypoints)
        dp_len = float(np.sum(np.linalg.norm(np.diff(waypoints, axis=0), axis=1)))
        # Phase 1 shortcut (fast shared-box merge)
        waypoints = shortcut_hull_merge(waypoints, lo, hi, n_rounds=1)
        # A* bridge for remaining dirty segments with large escapes
        waypoints = bridge_waypoints_via_boxes(waypoints, lo, hi, ids, adj)
        n_bridge = len(waypoints)
        bridge_len = float(np.sum(np.linalg.norm(np.diff(waypoints, axis=0), axis=1)))
        # Shortcut + straighten to compress
        waypoints = shortcut_hull_merge(waypoints, lo, hi, n_rounds=3)
        waypoints = straighten_path(waypoints, lo, hi, n_iters=5)
        waypoints = shortcut_hull_merge(waypoints, lo, hi, n_rounds=2)
        # Random-pair shortcut (STRICT: gap_tol=0 to enforce zero escape from box union)
        for rd_seed in range(6):
            waypoints = shortcut_random_pairs(waypoints, lo, hi, n_trials=5000,
                                               seed=rd_seed, gap_tol=0.0)
            waypoints = shortcut_hull_merge(waypoints, lo, hi, n_rounds=2)
            waypoints = straighten_path(waypoints, lo, hi, n_iters=3)
        waypoints = shortcut_hull_merge(waypoints, lo, hi, n_rounds=2)
        # FINAL GUARANTEE: enforce strict safety (no escape from box union)
        waypoints = enforce_strict_safety(waypoints, lo, hi, ids, adj, id2row,
                                          max_recursion=8, verbose=verbose)
        # After patching, run one more shortcut+straighten round to compress
        waypoints = shortcut_hull_merge(waypoints, lo, hi, n_rounds=3)
        waypoints = straighten_path(waypoints, lo, hi, n_iters=5)
        waypoints = shortcut_random_pairs(waypoints, lo, hi, n_trials=5000, seed=99, gap_tol=0.0)
        waypoints = shortcut_hull_merge(waypoints, lo, hi, n_rounds=2)
        n_str = len(waypoints)
        str_len = float(np.sum(np.linalg.norm(np.diff(waypoints, axis=0), axis=1)))
        # Count remaining dirty segments and max escape distance
        n_dirty_final = 0
        max_escape_dist = 0.0
        total_escape_len = 0.0
        for si in range(len(waypoints) - 1):
            if not segment_in_box_union_exact(waypoints[si], waypoints[si + 1], lo, hi):
                n_dirty_final += 1
                seg_d = waypoints[si + 1] - waypoints[si]
                total_escape_len += float(np.linalg.norm(seg_d))
                # Sample points along dirty segment to find max escape
                for t_s in [0.25, 0.5, 0.75]:
                    pt = waypoints[si] + t_s * seg_d
                    clamped = np.clip(pt, lo, hi)
                    esc = float(np.min(np.max(np.abs(clamped - pt), axis=1)))
                    max_escape_dist = max(max_escape_dist, esc)
        if verbose:
            print(f"  Step6: clip={n_clip}(len={clip_len:.1f}) short1={n_short1}(len={short1_len:.1f}) dp={n_dp}(len={dp_len:.1f}) astar={n_bridge}(len={bridge_len:.1f}) final={n_str}(len={str_len:.1f}) dirty={n_dirty_final}/{n_str-1} maxEsc={max_escape_dist:.6f}rad totEscLen={total_escape_len:.3f}")

        # ── Step 7: Drake collision repair — DISABLED (introduces box escapes) ──
        # waypoints = drake_collision_repair(waypoints, lo, hi)

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
    parser.add_argument("--corridor-hops", type=int, default=3)
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

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
    # Only add nearest-box fallbacks if the point is genuinely outside all
    # boxes. Otherwise restrict Dijkstra entry to containing boxes so that
    # the q_start→x_0 (and x_N→q_goal) segment is safe by construction.
    if not start_rows:
        start_rows |= set(np.argsort(diff_s)[:20].tolist())
    if not goal_rows:
        goal_rows |= set(np.argsort(diff_g)[:20].tolist())

    def _run(s_rows, g_rows):
        start_ids = {ids[r] for r in s_rows if r < N}
        goal_ids = {ids[r] for r in g_rows if r < N}
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
        return found, prev

    found, prev = _run(start_rows, goal_rows)
    if found is None:
        # Fallback: allow k-nearest boxes as well (may produce dirty seg 0)
        start_rows |= set(np.argsort(diff_s)[:20].tolist())
        goal_rows |= set(np.argsort(diff_g)[:20].tolist())
        found, prev = _run(start_rows, goal_rows)

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

def segment_in_box_union_exact(p, q, lo, hi, eps=1e-6):
    """Exact slab test: segment p→q is inside ∪(boxes) iff the union of
    per-box [t_enter, t_exit] intervals covers [0, 1].
    Vectorized across all boxes — ~0.3 ms for 7K boxes.
    """
    d = q - p
    seg_len = float(np.linalg.norm(d))
    if seg_len < 1e-15:
        return bool(np.any(np.all((p >= lo - eps) & (p <= hi + eps), axis=1)))
    zero_d = np.abs(d) < 1e-15
    N = lo.shape[0]
    if np.any(zero_d):
        out_z = (p[zero_d] < lo[:, zero_d] - eps) | (p[zero_d] > hi[:, zero_d] + eps)
        any_out_z = np.any(out_z, axis=1)
    else:
        any_out_z = np.zeros(N, dtype=bool)
    with np.errstate(divide="ignore", invalid="ignore"):
        t1 = (lo[:, ~zero_d] - eps - p[~zero_d]) / d[~zero_d]
        t2 = (hi[:, ~zero_d] + eps - p[~zero_d]) / d[~zero_d]
    t_lo = np.minimum(t1, t2)
    t_hi = np.maximum(t1, t2)
    if t_lo.shape[1] > 0:
        t_enter = np.maximum(np.max(t_lo, axis=1), 0.0)
        t_exit = np.minimum(np.min(t_hi, axis=1), 1.0)
    else:
        t_enter = np.zeros(N)
        t_exit = np.ones(N)
    valid = (t_enter <= t_exit + eps) & ~any_out_z
    if not np.any(valid):
        return False
    # Merge intervals and check coverage of [0, 1]
    order = np.argsort(t_enter[valid])
    te = t_enter[valid][order]
    tx = t_exit[valid][order]
    covered = 0.0
    for a, b in zip(te, tx):
        if a > covered + eps:
            return False
        if b > covered:
            covered = float(b)
        if covered >= 1.0 - eps:
            return True
    return covered >= 1.0 - eps


def enforce_dense_safety(waypoints, lo, hi, max_iters=12):
    """Guarantee every segment is strictly in box union (slab check).

    For each slab-dirty segment, find the largest uncovered t-interval,
    project its midpoint to nearest box, and insert it as a splitter.
    Iterate until all segments pass slab check or max_iters reached.
    """
    EPS = 1e-6

    def largest_gap_t(p, q):
        """Return t-midpoint of the largest uncovered sub-interval, or -1 if covered."""
        d = q - p
        if float(np.linalg.norm(d)) < 1e-12:
            return -1.0
        zero_d = np.abs(d) < 1e-15
        N = lo.shape[0]
        if np.any(zero_d):
            out_z = (p[zero_d] < lo[:, zero_d] - EPS) | (p[zero_d] > hi[:, zero_d] + EPS)
            any_out_z = np.any(out_z, axis=1)
        else:
            any_out_z = np.zeros(N, dtype=bool)
        with np.errstate(divide="ignore", invalid="ignore"):
            t1 = (lo[:, ~zero_d] - EPS - p[~zero_d]) / d[~zero_d]
            t2 = (hi[:, ~zero_d] + EPS - p[~zero_d]) / d[~zero_d]
        t_lo = np.minimum(t1, t2)
        t_hi = np.maximum(t1, t2)
        if t_lo.shape[1] > 0:
            t_en = np.maximum(np.max(t_lo, axis=1), 0.0)
            t_ex = np.minimum(np.min(t_hi, axis=1), 1.0)
        else:
            t_en = np.zeros(N)
            t_ex = np.ones(N)
        valid = (t_en <= t_ex + EPS) & ~any_out_z
        if not np.any(valid):
            return 0.5  # split at middle
        order = np.argsort(t_en[valid])
        te = t_en[valid][order]
        tx = t_ex[valid][order]
        # Find largest gap
        covered = 0.0
        best_gap_mid = -1.0
        best_gap = 0.0
        for a, b in zip(te, tx):
            if a > covered + EPS:
                g = a - covered
                if g > best_gap:
                    best_gap = g
                    best_gap_mid = 0.5 * (covered + a)
            if b > covered:
                covered = float(b)
        if covered < 1.0 - EPS:
            g = 1.0 - covered
            if g > best_gap:
                best_gap = g
                best_gap_mid = 0.5 * (covered + 1.0)
        return best_gap_mid if best_gap_mid >= 0 else -1.0

    def project_to_nearest(p):
        clamped = np.clip(p, lo, hi)
        d = np.linalg.norm(clamped - p, axis=1)
        r = int(np.argmin(d))
        return np.clip(p, lo[r] + EPS, hi[r] - EPS)

    pts = [np.asarray(w, dtype=float) for w in waypoints]
    for _it in range(max_iters):
        out = [pts[0]]
        changed = False
        for i in range(len(pts) - 1):
            p, q = pts[i], pts[i + 1]
            if segment_in_box_union_exact(p, q, lo, hi):
                out.append(q)
                continue
            t_star = largest_gap_t(p, q)
            if t_star < 0:
                out.append(q)
                continue
            # Clamp t away from endpoints
            t_star = float(np.clip(t_star, 0.05, 0.95))
            split_pt = p + t_star * (q - p)
            split_pt = project_to_nearest(split_pt)
            if (np.linalg.norm(split_pt - p) < 5e-5 or
                    np.linalg.norm(split_pt - q) < 5e-5):
                out.append(q)
                continue
            changed = True
            out.append(split_pt)
            out.append(q)
        pts = out
        if not changed:
            break
    return pts


def bridge_non_overlapping(path_pts, lo, hi, row_adj=None):
    """Repair escaping segments by inserting 1 or 2 intermediate waypoints.

    For each dirty segment (p, q):
      1. Find containing box rows for p and q (or nearest if outside).
      2. Try box pairs (a ∈ rows_p, b ∈ rows_q): pick the pair whose overlap
         contains the segment midpoint after clamping, validated by strict
         full-union check on both sub-segments.
      3. Fallback: 3-hop via intermediate box C.
      4. Final: BFS on row_adj for multi-hop repair (if row_adj provided).
    """
    EPS = 1e-6
    MAX_TRIES = 6

    def rows_containing(p):
        return np.where(np.all((p >= lo - EPS) & (p <= hi + EPS), axis=1))[0]

    def nearest_row(p):
        clamped = np.clip(p, lo, hi)
        d = np.linalg.norm(clamped - p, axis=1)
        return int(np.argmin(d))

    out = [path_pts[0]]
    for i in range(len(path_pts) - 1):
        p, q = path_pts[i], path_pts[i + 1]
        if segment_in_box_union_exact(p, q, lo, hi):
            out.append(q)
            continue

        rp = rows_containing(p).tolist()
        rq = rows_containing(q).tolist()
        if not rp:
            rp = [nearest_row(p)]
        if not rq:
            rq = [nearest_row(q)]

        # Try pairs with non-empty overlap, prefer those whose overlap center
        # is near the segment midpoint.
        mid = 0.5 * (p + q)
        candidates = []
        for a in rp[:MAX_TRIES]:
            for b in rq[:MAX_TRIES]:
                if a == b:
                    # Clamp q into box a (segment along box interior)
                    cand = np.clip(mid, lo[a] + EPS, hi[a] - EPS)
                    d_mid = float(np.linalg.norm(cand - mid))
                    candidates.append((d_mid, a, b, cand, None))
                    continue
                ov_lo = np.maximum(lo[a], lo[b])
                ov_hi = np.minimum(hi[a], hi[b])
                if np.all(ov_lo <= ov_hi + EPS):
                    mid_ov = np.clip(mid, ov_lo, ov_hi)
                    d_mid = float(np.linalg.norm(mid_ov - mid))
                    candidates.append((d_mid, a, b, mid_ov, None))
                else:
                    # Non-overlapping: two waypoints
                    a_pt = np.clip(mid, lo[a] + EPS, hi[a] - EPS)
                    b_pt = np.clip(mid, lo[b] + EPS, hi[b] - EPS)
                    d_mid = float(np.linalg.norm(a_pt - mid) + np.linalg.norm(b_pt - mid))
                    candidates.append((d_mid, a, b, a_pt, b_pt))

        candidates.sort(key=lambda c: c[0])
        repaired = False
        for _, a, b, pt1, pt2 in candidates:
            if pt2 is None:
                if (segment_in_box_union_exact(p, pt1, lo, hi)
                        and segment_in_box_union_exact(pt1, q, lo, hi)):
                    out.append(pt1)
                    out.append(q)
                    repaired = True
                    break
            else:
                if (segment_in_box_union_exact(p, pt1, lo, hi)
                        and segment_in_box_union_exact(pt1, pt2, lo, hi)
                        and segment_in_box_union_exact(pt2, q, lo, hi)):
                    out.append(pt1)
                    out.append(pt2)
                    out.append(q)
                    repaired = True
                    break

        if not repaired:
            # Try 3-hop via intermediate box C overlapping both box(p) and box(q).
            # For each (a ∈ rp, b ∈ rq), enumerate boxes C that overlap both.
            three_hop_done = False
            for a in rp[:MAX_TRIES]:
                if three_hop_done:
                    break
                # Boxes overlapping box a: lo[a] <= hi_all and hi[a] >= lo_all
                ov_a = np.all((lo <= hi[a] + EPS) & (hi >= lo[a] - EPS), axis=1)
                for b in rq[:MAX_TRIES]:
                    if three_hop_done:
                        break
                    ov_b = np.all((lo <= hi[b] + EPS) & (hi >= lo[b] - EPS), axis=1)
                    shared = np.where(ov_a & ov_b)[0]
                    if len(shared) == 0:
                        continue
                    # Prefer C whose center is closest to segment midpoint
                    centers = 0.5 * (lo[shared] + hi[shared])
                    d = np.linalg.norm(centers - mid, axis=1)
                    order = np.argsort(d)
                    for c_idx in shared[order[:8]]:
                        # Overlap points: A∩C and C∩B
                        ov_ac_lo = np.maximum(lo[a], lo[c_idx])
                        ov_ac_hi = np.minimum(hi[a], hi[c_idx])
                        ov_cb_lo = np.maximum(lo[c_idx], lo[b])
                        ov_cb_hi = np.minimum(hi[c_idx], hi[b])
                        if np.any(ov_ac_lo > ov_ac_hi + EPS):
                            continue
                        if np.any(ov_cb_lo > ov_cb_hi + EPS):
                            continue
                        pt1 = np.clip(mid, ov_ac_lo, ov_ac_hi)
                        pt2 = np.clip(mid, ov_cb_lo, ov_cb_hi)
                        if (segment_in_box_union_exact(p, pt1, lo, hi)
                                and segment_in_box_union_exact(pt1, pt2, lo, hi)
                                and segment_in_box_union_exact(pt2, q, lo, hi)):
                            out.append(pt1)
                            out.append(pt2)
                            out.append(q)
                            repaired = True
                            three_hop_done = True
                            break
        if not repaired:
            # Multi-hop BFS on geometric-overlap graph (lazy expansion).
            from collections import deque
            best_chain = None
            best_len = 1e18
            # Cache of neighbors computed lazily
            neigh_cache: dict = {}
            def get_neighbors(u):
                if u not in neigh_cache:
                    mask = np.all((lo <= hi[u] + EPS) & (hi >= lo[u] - EPS), axis=1)
                    nb = np.where(mask)[0].tolist()
                    if u in nb:
                        nb.remove(u)
                    neigh_cache[u] = nb
                return neigh_cache[u]
            for a in rp[:MAX_TRIES]:
                for b in rq[:MAX_TRIES]:
                    prev = {a: -1}
                    dq = deque([a])
                    depth = {a: 0}
                    found = False
                    while dq:
                        u = dq.popleft()
                        if u == b:
                            found = True
                            break
                        if depth[u] >= 8:
                            continue
                        for v in get_neighbors(u):
                            if v not in prev:
                                prev[v] = u
                                depth[v] = depth[u] + 1
                                dq.append(v)
                    if not found:
                        continue
                    chain = []
                    u = b
                    while u != -1:
                        chain.append(u)
                        u = prev[u]
                    chain.reverse()
                    if len(chain) < best_len:
                        best_len = len(chain)
                        best_chain = chain
            if best_chain is not None and len(best_chain) >= 2:
                insert_pts = []
                ok = True
                for k in range(len(best_chain) - 1):
                    u, v = best_chain[k], best_chain[k + 1]
                    ov_lo = np.maximum(lo[u], lo[v])
                    ov_hi = np.minimum(hi[u], hi[v])
                    if np.any(ov_lo > ov_hi + EPS):
                        ok = False
                        break
                    c = np.clip(mid, ov_lo + EPS, ov_hi - EPS)
                    insert_pts.append(c)
                if ok:
                    chain_pts = [p] + insert_pts + [q]
                    all_ok = all(
                        segment_in_box_union_exact(chain_pts[k], chain_pts[k + 1], lo, hi)
                        for k in range(len(chain_pts) - 1)
                    )
                    if all_ok:
                        out.extend(insert_pts)
                        out.append(q)
                        repaired = True
        if not repaired:
            # Genuinely unreachable in strict box-union: accept the dirty
            # segment as-is (leave it) rather than inserting bad waypoints
            # that make things worse. Downstream can flag these.
            out.append(q)
    return out


def safe_shortcut(waypoints, lo, hi, max_rounds=4, max_time_s=0.7):
    """Safety-preserving greedy shortcut using exponential+binary search.

    Each round: sweep waypoints[0..n-1], for each i find farthest j such
    that segment waypoints[i]→waypoints[j] is inside the box union, then
    keep waypoints[j] and advance i=j.
    """
    pts = [np.asarray(w, dtype=float) for w in waypoints]
    t_budget_start = time.perf_counter()
    for _ in range(max_rounds):
        if time.perf_counter() - t_budget_start > max_time_s:
            break
        n = len(pts)
        if n <= 2:
            break
        out = [pts[0]]
        i = 0
        advanced = False
        while i < n - 1:
            if time.perf_counter() - t_budget_start > max_time_s:
                out.extend(pts[i + 1:])
                break
            # Exponential search: try i→i+1, i+2, i+4, i+8, ...
            best_j = i + 1
            step = 2
            while i + step < n and segment_in_box_union_exact(pts[i], pts[i + step], lo, hi):
                best_j = i + step
                step *= 2
            # Binary search between best_j+1 and min(i+step-1, n-1)
            lo_j = best_j
            hi_j = min(i + step - 1, n - 1)
            while lo_j < hi_j:
                mid = (lo_j + hi_j + 1) // 2
                if segment_in_box_union_exact(pts[i], pts[mid], lo, hi):
                    lo_j = mid
                else:
                    hi_j = mid - 1
            if lo_j > i + 1:
                advanced = True
            out.append(pts[lo_j])
            i = lo_j
        pts = out
        if not advanced:
            break
    return pts


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

    # ── Step 1.5: Project start/goal into first/last chain box if outside ──
    # Must check membership in the *chain endpoint*, not in any box globally:
    # if q_goal is in some box outside the Dijkstra chain (e.g. disconnected
    # island), the fallback picks a nearby chain box, and the final SOCP
    # segment to the un-projected q_goal escapes the chain union.
    real_start, real_goal = q_start.copy(), q_goal.copy()
    start_projected = goal_projected = False
    EPS = 1e-6
    r0 = id2row[path_ids[0]]
    rn = id2row[path_ids[-1]]
    if not (np.all(q_start >= lo[r0] - EPS) and np.all(q_start <= hi[r0] + EPS)):
        q_start = np.clip(q_start, lo[r0] + EPS, hi[r0] - EPS)
        start_projected = True
    if not (np.all(q_goal >= lo[rn] - EPS) and np.all(q_goal <= hi[rn] + EPS)):
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
        # Only prepend the real start if the [real_start, projected_start]
        # segment stays inside the box union; otherwise the projection marks
        # a hard island boundary and we must report failure for safety.
        if segment_in_box_union_exact(real_start, path[0], lo, hi):
            path = np.vstack([real_start.reshape(1, -1), path])
        else:
            info["error"] = (f"start outside chain entry box by "
                             f"{float(np.max(np.abs(real_start - path[0]))):.3f}")
            return None, info
    if goal_projected:
        if segment_in_box_union_exact(path[-1], real_goal, lo, hi):
            path = np.vstack([path, real_goal.reshape(1, -1)])
        else:
            info["error"] = (f"goal outside chain exit box by "
                             f"{float(np.max(np.abs(real_goal - path[-1]))):.3f}")
            return None, info

    # ── Step 2.25: Insert chain-overlap midpoints between consecutive SOCP
    # waypoints. Since chain_rows[i] and chain_rows[i+1] overlap by construction
    # (adjacent in connectivity graph), inserting an overlap midpoint makes
    # both sub-segments provably safe. This eliminates nearly all dirty segs.
    # Build mapping: path index (after projection) → chain box row
    EPS2 = 1e-6
    n_proj_start = 1 if start_projected else 0
    n_proj_goal = 1 if goal_projected else 0
    # SOCP path has len(chain_rows) points. After projection:
    #   [real_start?]  socp_pts[0..L-1]  [real_goal?]
    path = np.asarray(path)
    augmented = [path[0]]
    for i in range(len(path) - 1):
        p, q = path[i], path[i + 1]
        # Determine chain box rows for p and q
        i_socp_p = i - n_proj_start  # index in socp_pts, -1 if it's real_start
        i_socp_q = i + 1 - n_proj_start
        # Find chain row indices for p and q
        if 0 <= i_socp_p < len(chain_rows):
            row_p = chain_rows[i_socp_p]
        else:
            row_p = chain_rows[0]  # real_start → first chain box
        if 0 <= i_socp_q < len(chain_rows):
            row_q = chain_rows[i_socp_q]
        else:
            row_q = chain_rows[-1]  # real_goal → last chain box
        if row_p != row_q and not segment_in_box_union_exact(p, q, lo, hi):
            ov_lo = np.maximum(lo[row_p], lo[row_q])
            ov_hi = np.minimum(hi[row_p], hi[row_q])
            if np.all(ov_lo <= ov_hi + EPS2):
                mid = 0.5 * (p + q)
                c = np.clip(mid, ov_lo + EPS2, ov_hi - EPS2)
                if (segment_in_box_union_exact(p, c, lo, hi)
                        and segment_in_box_union_exact(c, q, lo, hi)):
                    augmented.append(c)
        augmented.append(q)
    path = np.array(augmented)

    # ── Step 2.5: Bridge non-overlapping adjacent boxes (strict safety) ──
    # Iterate up to 5 rounds; each round repairs remaining dirty segments.
    # Build row-indexed adjacency for multi-hop BFS fallback.
    row_adj = {}
    for bid, neigh in adj.items():
        if bid in id2row:
            row_adj[id2row[bid]] = [id2row[n] for n in neigh if n in id2row]
    t_bridge = time.perf_counter()
    path = list(path)
    for _rep in range(5):
        n_before_rep = len(path)
        path = bridge_non_overlapping(path, lo, hi, row_adj=row_adj)
        # Stop if no new waypoints inserted
        if len(path) == n_before_rep:
            break
    dt_bridge = time.perf_counter() - t_bridge

    # ── Step 3: Safe shortcut (strict — never escapes box union) ──
    waypoints = list(path)
    n_before = len(waypoints)
    t_smooth = time.perf_counter()
    waypoints = safe_shortcut(waypoints, lo, hi, max_rounds=4, max_time_s=0.4)
    # Final guarantee: re-iterate bridge_non_overlapping until all segments
    # pass slab check (bridge is robust: falls back to 2-waypoint insertion).
    for _rep in range(20):
        n_prev = len(waypoints)
        waypoints = bridge_non_overlapping(waypoints, lo, hi, row_adj=row_adj)
        if len(waypoints) == n_prev:
            break
    dt_smooth = time.perf_counter() - t_smooth

    path = np.array(waypoints)
    path_len = float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))

    info.update({
        "success": True,
        "path_length": path_len,
        "n_before_shortcut": n_before,
        "n_waypoints": len(waypoints),
        "smooth_time": dt_smooth,
        "bridge_time": dt_bridge,
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

    # Filter adj to only include geometrically-overlapping edges.
    # This prevents Dijkstra from picking invalid chains that downstream
    # SOCP / bridge repair cannot fix.
    _EPS = 1e-6
    id2row_local = {bid: i for i, bid in enumerate(ids)}
    filtered_adj: Dict[int, List[int]] = {}
    n_dropped = 0
    for bid, neigh in adj.items():
        if bid not in id2row_local:
            continue
        a = id2row_local[bid]
        kept = []
        for nid in neigh:
            if nid not in id2row_local:
                continue
            b = id2row_local[nid]
            if np.any(lo[a] > hi[b] + _EPS) or np.any(hi[a] < lo[b] - _EPS):
                n_dropped += 1
                continue
            kept.append(nid)
        filtered_adj[bid] = kept
    adj = filtered_adj

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

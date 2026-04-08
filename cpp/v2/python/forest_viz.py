"""SafeBoxForest v2 — Forest visualization using matplotlib.

Usage:
    python forest_viz.py forest_data.json [--output fig.png]

Loads JSON exported by sbf::viz::export_forest_json and renders:
- 2D pairwise projections of box intervals
- Adjacency graph overlay
- Path waypoints
- Obstacles (3D view)
"""
import argparse
import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.patches as mpl_patches
import numpy as np


def load_forest(path: str) -> dict:
    with open(path) as f:
        return json.load(f)


def plot_boxes_2d(ax, boxes: list, dim_x: int, dim_y: int, **kwargs):
    """Plot axis-aligned boxes projected onto (dim_x, dim_y) plane."""
    color = kwargs.get("color", "steelblue")
    alpha = kwargs.get("alpha", 0.25)
    edge_color = kwargs.get("edgecolor", "navy")

    for bx in boxes:
        ivs = bx["intervals"]
        lo_x, hi_x = ivs[dim_x]
        lo_y, hi_y = ivs[dim_y]
        rect = mpl_patches.Rectangle(
            (lo_x, lo_y), hi_x - lo_x, hi_y - lo_y,
            linewidth=0.5, edgecolor=edge_color, facecolor=color, alpha=alpha
        )
        ax.add_patch(rect)


def plot_adjacency_2d(ax, boxes: list, adjacency: list, dim_x: int, dim_y: int):
    """Draw adjacency edges between box centers."""
    id_to_center = {}
    for bx in boxes:
        ivs = bx["intervals"]
        cx = 0.5 * (ivs[dim_x][0] + ivs[dim_x][1])
        cy = 0.5 * (ivs[dim_y][0] + ivs[dim_y][1])
        id_to_center[bx["id"]] = (cx, cy)

    for a, b in adjacency:
        if a in id_to_center and b in id_to_center:
            ca, cb = id_to_center[a], id_to_center[b]
            ax.plot([ca[0], cb[0]], [ca[1], cb[1]],
                    color="gray", linewidth=0.3, alpha=0.5)


def plot_path_2d(ax, path: list, dim_x: int, dim_y: int):
    """Plot path waypoints."""
    xs = [p[dim_x] for p in path]
    ys = [p[dim_y] for p in path]
    ax.plot(xs, ys, "r-o", markersize=2, linewidth=1.0, label="path")
    if xs:
        ax.plot(xs[0], ys[0], "g*", markersize=10, label="start")
        ax.plot(xs[-1], ys[-1], "m*", markersize=10, label="goal")


def main():
    parser = argparse.ArgumentParser(description="Visualize SBF forest")
    parser.add_argument("input", help="JSON file from export_forest_json")
    parser.add_argument("--output", "-o", help="Save figure to file")
    parser.add_argument("--dims", nargs=2, type=int, default=[0, 1],
                        help="Joint dimensions to project (default: 0 1)")
    args = parser.parse_args()

    data = load_forest(args.input)
    boxes = data.get("boxes", [])
    adjacency = data.get("adjacency", [])
    path = data.get("path", [])

    dx, dy = args.dims

    fig, ax = plt.subplots(1, 1, figsize=(10, 8))
    plot_boxes_2d(ax, boxes, dx, dy)
    plot_adjacency_2d(ax, boxes, adjacency, dx, dy)
    if path:
        plot_path_2d(ax, path, dx, dy)

    ax.set_xlabel(f"Joint {dx}")
    ax.set_ylabel(f"Joint {dy}")
    ax.set_title(f"SBF Forest ({len(boxes)} boxes)")
    ax.set_aspect("equal")
    ax.legend(loc="upper right")
    ax.autoscale()

    if args.output:
        fig.savefig(args.output, dpi=150, bbox_inches="tight")
        print(f"Saved to {args.output}")
    else:
        plt.show()


if __name__ == "__main__":
    main()


# =============================================================================
#  AABB Visualization Utilities
#  Added to support examples 09 and 10 (AABB envelope comparison)
# =============================================================================

import math
import itertools
import datetime
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

# Module-level persistent thread pool — created once at import time.
# Avoids ~4 ms thread-spawn overhead that occurs when using
# `with ThreadPoolExecutor()` as a per-call context manager.
_THREAD_POOL = ThreadPoolExecutor()


# ─── Internal interval arithmetic ────────────────────────────────────────────

_TWO_PI = 2.0 * math.pi
_HP = math.pi / 2.0
_3HP = 3.0 * math.pi / 2.0


def _iv_sin(lo: float, hi: float):
    """Conservative interval enclosure of sin over [lo, hi]."""
    w = hi - lo
    if w >= _TWO_PI:
        return (-1.0, 1.0)
    s_lo, s_hi = math.sin(lo), math.sin(hi)
    r_lo, r_hi = min(s_lo, s_hi), max(s_lo, s_hi)
    ln = lo % _TWO_PI
    hn = ln + w
    if math.ceil((ln - _HP)  / _TWO_PI) <= math.floor((hn - _HP)  / _TWO_PI):
        r_hi = 1.0
    if math.ceil((ln - _3HP) / _TWO_PI) <= math.floor((hn - _3HP) / _TWO_PI):
        r_lo = -1.0
    return (r_lo, r_hi)


def _iv_cos(lo: float, hi: float):
    """Conservative interval enclosure of cos over [lo, hi]."""
    w = hi - lo
    if w >= _TWO_PI:
        return (-1.0, 1.0)
    c_lo, c_hi = math.cos(lo), math.cos(hi)
    r_lo, r_hi = min(c_lo, c_hi), max(c_lo, c_hi)
    ln = lo % _TWO_PI
    hn = ln + w
    if math.ceil(ln / _TWO_PI) <= math.floor(hn / _TWO_PI):
        r_hi = 1.0
    if math.ceil((ln - math.pi) / _TWO_PI) <= math.floor((hn - math.pi) / _TWO_PI):
        r_lo = -1.0
    return (r_lo, r_hi)


def _iv_add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def _iv_mul(a, b):
    p = [a[0]*b[0], a[0]*b[1], a[1]*b[0], a[1]*b[1]]
    return (min(p), max(p))


def _iv_scale(s, a):
    return (s*a[0], s*a[1]) if s >= 0 else (s*a[1], s*a[0])


def _iv_c(v):
    return (v, v)


def _iv_neg(a):
    return (-a[1], -a[0])


def _iv_dh_mat(alpha, a, d, ct, st):
    """4×4 interval-valued Modified DH matrix. ct/st are interval cos/sin."""
    ca, sa = math.cos(alpha), math.sin(alpha)
    z, o = _iv_c(0.0), _iv_c(1.0)
    return [
        [ct,              _iv_neg(st),          z,          _iv_c(a)],
        [_iv_scale(ca, st), _iv_scale(ca, ct),  _iv_c(-sa), _iv_scale(-sa, _iv_c(d))],
        [_iv_scale(sa, st), _iv_scale(sa, ct),  _iv_c(ca),  _iv_scale(ca,  _iv_c(d))],
        [z,               z,                    z,          o],
    ]


def _iv_mat4_mul(A, B):
    """Multiply two 4×4 interval matrices."""
    C = [[_iv_c(0.0)] * 4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            s = _iv_c(0.0)
            for k in range(4):
                s = _iv_add(s, _iv_mul(A[i][k], B[k][j]))
            C[i][j] = s
    return C


def _numpy_to_iv_mat4(M):
    """Convert a 4×4 numpy matrix to an interval matrix."""
    return [[_iv_c(float(M[i, j])) for j in range(4)] for i in range(4)]


# ─── DH FK helpers ────────────────────────────────────────────────────────────

# Panda DH parameters — geometrically accurate Modified DH (Craig convention).
# a values from the Franka Emika Panda technical specification.
PANDA_DH_PARAMS = [
    {"alpha": 0.0,           "a":  0.0,    "d": 0.333, "theta": 0.0},  # J1
    {"alpha": -math.pi / 2,  "a":  0.0,    "d": 0.0,   "theta": 0.0},  # J2
    {"alpha":  math.pi / 2,  "a":  0.0,    "d": 0.316, "theta": 0.0},  # J3
    {"alpha":  math.pi / 2,  "a":  0.0825, "d": 0.0,   "theta": 0.0},  # J4
    {"alpha": -math.pi / 2,  "a": -0.0825, "d": 0.384, "theta": 0.0},  # J5
    {"alpha":  math.pi / 2,  "a":  0.0,    "d": 0.0,   "theta": 0.0},  # J6
    {"alpha":  math.pi / 2,  "a":  0.088,  "d": 0.0,   "theta": 0.0},  # J7
]
PANDA_TOOL_D = 0.107  # tool z-offset beyond last joint

# Panda joint limits [rad]
PANDA_JOINT_LIMITS = [
    (-2.8973,  2.8973),
    (-1.7628,  1.7628),
    (-2.8973,  2.8973),
    (-3.0718, -0.0698),
    (-2.8973,  2.8973),
    (-0.0175,  3.7525),
    (-2.8973,  2.8973),
]


def _dh_mat(alpha: float, a: float, d: float, theta: float) -> np.ndarray:
    """Compute a single 4×4 Modified DH transform matrix."""
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct,     -st,      0,      a],
        [st*ca,   ct*ca,  -sa,    -sa*d],
        [st*sa,   ct*sa,   ca,     ca*d],
        [0,       0,       0,      1],
    ])


def _dh_mat_batch(alpha: float, a: float, d: float,
                  theta_offset: float, thetas: np.ndarray) -> np.ndarray:
    """Vectorised DH transform matrix for a batch of joint angles.

    Parameters
    ----------
    alpha, a, d  : scalar DH link parameters
    theta_offset : constant theta offset (from dh_params["theta"])
    thetas       : (N,) array of joint angles

    Returns
    -------
    (N, 4, 4) array of Modified DH transform matrices
    """
    th = thetas + theta_offset
    ct = np.cos(th); st = np.sin(th)
    ca = math.cos(alpha); sa = math.sin(alpha)
    N = len(thetas)
    M = np.zeros((N, 4, 4))
    M[:, 0, 0] = ct;       M[:, 0, 1] = -st
    M[:, 0, 3] = a
    M[:, 1, 0] = st * ca;  M[:, 1, 1] = ct * ca
    M[:, 1, 2] = -sa;      M[:, 1, 3] = -sa * d
    M[:, 2, 0] = st * sa;  M[:, 2, 1] = ct * sa
    M[:, 2, 2] =  ca;      M[:, 2, 3] =  ca * d
    M[:, 3, 3] = 1.0
    return M


def _fk_combos_batch(dh_params: list, q_batch: np.ndarray,
                     link_idx: int) -> np.ndarray:
    """Batched FK returning parent and child endpoint positions for all combos.

    Parameters
    ----------
    dh_params : full DH parameter list
    q_batch   : (N, n_dof) joint angles — irrelevant columns are ignored
    link_idx  : 1-based link index (joints 0..link_idx-1 are used)

    Returns
    -------
    pts : (2N, 3) row 0..N-1 = parent endpoints, rows N..2N-1 = child endpoints
    """
    N = len(q_batch)
    T = np.zeros((N, 4, 4))
    T[:] = np.eye(4)
    parent_pts = np.zeros((N, 3))           # base origin (link 1 parent)
    for i in range(link_idx):
        if i == link_idx - 1:
            parent_pts = T[:, :3, 3].copy() # snapshot before last joint
        dh = dh_params[i]
        M = _dh_mat_batch(dh["alpha"], dh["a"], dh["d"],
                          dh["theta"], q_batch[:, i])
        T = np.matmul(T, M)
    child_pts = T[:, :3, 3].copy()
    return np.vstack([parent_pts, child_pts])   # (2N, 3)


def _make_q_batch_np(csets: list, mid_q: np.ndarray, relevant: list) -> np.ndarray:
    """Build a (N, n_dof) joint-angle batch from per-joint critical-value lists.

    Uses ``np.meshgrid`` instead of ``itertools.product`` + a Python loop,
    which avoids materialising a large Python list of tuples and is therefore
    significantly faster for large combo counts.

    Parameters
    ----------
    csets   : list of 1-D arrays/lists of critical angles, one per relevant joint
    mid_q   : (n_dof,) default joint angles (fills non-relevant columns)
    relevant: joint indices corresponding to each entry of ``csets``

    Returns
    -------
    q_batch : (N, n_dof) float64 array, N = prod(len(cs) for cs in csets)
    """
    if not csets:
        return mid_q.reshape(1, -1)
    arrs = [np.asarray(cs, dtype=float) for cs in csets]
    if len(arrs) == 1:
        N = len(arrs[0])
        q_batch = np.tile(mid_q, (N, 1))
        q_batch[:, relevant[0]] = arrs[0]
        return q_batch
    grids = np.meshgrid(*arrs, indexing='ij')   # each grid: shape (n0, n1, ...)
    N = grids[0].size
    q_batch = np.tile(mid_q, (N, 1))
    for idx_r, j in enumerate(relevant):
        q_batch[:, j] = grids[idx_r].ravel()
    return q_batch


def dh_fk_positions(dh_params: list, q: np.ndarray) -> np.ndarray:
    """
    Forward kinematics for a serial DH robot.

    Parameters
    ----------
    dh_params : list of dict, each with keys alpha, a, d, theta
    q         : joint angles (n,) array

    Returns
    -------
    np.ndarray, shape (n+2, 3)
        Row 0 is the base origin. Rows 1..n are link frame origins.
        Row n+1 is the tool frame (adds PANDA_TOOL_D z-offset if
        len(dh_params) == len(PANDA_DH_PARAMS)).
    """
    T = np.eye(4)
    positions = [T[:3, 3].copy()]
    for i, dh in enumerate(dh_params):
        T = T @ _dh_mat(dh["alpha"], dh["a"], dh["d"], dh["theta"] + float(q[i]))
        positions.append(T[:3, 3].copy())
    # Tool offset: only for Panda-like robots
    if len(dh_params) == len(PANDA_DH_PARAMS):
        T_tool = T.copy()
        T_tool[:3, 3] = T[:3, :3] @ np.array([0, 0, PANDA_TOOL_D]) + T[:3, 3]
        positions.append(T_tool[:3, 3].copy())
    return np.array(positions)  # (n+1 or n+2, 3)


# ─── AABB computation: three methods ─────────────────────────────────────────

def aabb_volume(aabb: dict) -> float:
    """Compute volume of an AABB dict (keys: x_min/x_max/y_min/y_max/z_min/z_max)."""
    dx = max(0.0, aabb["x_max"] - aabb["x_min"])
    dy = max(0.0, aabb["y_max"] - aabb["y_min"])
    dz = max(0.0, aabb["z_max"] - aabb["z_min"])
    return dx * dy * dz


def compute_aabbs_interval_fk(dh_params: list, q_intervals: list) -> list:
    """
    Compute per-link AABB envelopes via interval arithmetic on a DH chain.

    Parameters
    ----------
    dh_params    : list of dicts with keys alpha, a, d, theta
    q_intervals  : list of (lo, hi) tuples, one per joint

    Returns
    -------
    list of dicts, one per link frame (plus tool if dh_params == PANDA_DH_PARAMS).
    Keys: frame (0-based), label, x_min, x_max, y_min, y_max, z_min, z_max.
    """
    T = _numpy_to_iv_mat4(np.eye(4))
    # prev_pos: interval position of the previous frame origin (starts at base)
    prev_pos = [T[r][3] for r in range(3)]  # = [(0,0),(0,0),(0,0)]
    aabbs = []
    for i, dh in enumerate(dh_params):
        lo = dh["theta"] + q_intervals[i][0]
        hi = dh["theta"] + q_intervals[i][1]
        ct = _iv_cos(lo, hi)
        st = _iv_sin(lo, hi)
        A = _iv_dh_mat(dh["alpha"], dh["a"], dh["d"], ct, st)
        T = _iv_mat4_mul(T, A)
        cur_pos = [T[r][3] for r in range(3)]
        # AABB of the full link segment = hull of parent endpoint ∪ child endpoint
        aabbs.append({
            "frame": i + 1, "label": f"Link {i+1}",
            "x_min": min(prev_pos[0][0], cur_pos[0][0]),
            "x_max": max(prev_pos[0][1], cur_pos[0][1]),
            "y_min": min(prev_pos[1][0], cur_pos[1][0]),
            "y_max": max(prev_pos[1][1], cur_pos[1][1]),
            "z_min": min(prev_pos[2][0], cur_pos[2][0]),
            "z_max": max(prev_pos[2][1], cur_pos[2][1]),
        })
        prev_pos = cur_pos
    # Tool (Panda-specific z-offset along local z)
    if len(dh_params) == len(PANDA_DH_PARAMS):
        Tt = [row[:] for row in T]
        for row in range(3):
            shift = _iv_scale(PANDA_TOOL_D, T[row][2])
            Tt[row][3] = _iv_add(T[row][3], shift)
        tool_pos = [Tt[r][3] for r in range(3)]
        aabbs.append({
            "frame": len(dh_params) + 1, "label": "Tool",
            "x_min": min(prev_pos[0][0], tool_pos[0][0]),
            "x_max": max(prev_pos[0][1], tool_pos[0][1]),
            "y_min": min(prev_pos[1][0], tool_pos[1][0]),
            "y_max": max(prev_pos[1][1], tool_pos[1][1]),
            "z_min": min(prev_pos[2][0], tool_pos[2][0]),
            "z_max": max(prev_pos[2][1], tool_pos[2][1]),
        })
    return aabbs


def compute_aabbs_critical(dh_params: list, q_intervals: list,
                            max_combos: int = 60_000) -> list:
    """
    Compute per-link AABB envelopes via critical-point enumeration.

    For a serial DH robot, link i's position depends only on joints 0..i-1.
    Critical points are k*pi/2 (gradient-zero) angles plus boundary values.
    Uses vectorised batch FK (_fk_combos_batch) and a ThreadPoolExecutor
    to compute all links in parallel.

    Parameters
    ----------
    dh_params   : list of dicts (alpha, a, d, theta)
    q_intervals : list of (lo, hi)
    max_combos  : cap on combinatorial explosion per link

    Returns
    -------
    list of aabb dicts (same format as compute_aabbs_interval_fk)
    """
    n = len(dh_params)
    mid_q = np.array([(lo + hi) / 2 for lo, hi in q_intervals])

    def _crit_angles(lo, hi):
        vals = [lo, hi]
        for k in range(-20, 21):
            v = k * math.pi / 2.0
            if lo < v < hi:
                vals.append(v)
        return sorted(set(vals))

    per_joint = [_crit_angles(*q_intervals[j]) for j in range(n)]

    def _link_qbatch(link_idx):
        relevant = list(range(link_idx))
        csets = [per_joint[j] for j in relevant]
        total = 1
        for cs in csets:
            total *= len(cs)
        if total > max_combos:
            csets = []
            for j in relevant:
                lo, hi = q_intervals[j]
                vals = [lo, (lo + hi) / 2, hi]
                for k in range(-20, 21):
                    v = k * math.pi / 2.0
                    if lo < v < hi:
                        vals.append(v)
                csets.append(sorted(set(vals)))
        return _make_q_batch_np(csets, mid_q, relevant)

    def _process_link(link_idx):
        pts = _fk_combos_batch(dh_params, _link_qbatch(link_idx), link_idx)
        return {
            "frame": link_idx, "label": f"Link {link_idx}",
            "x_min": float(pts[:, 0].min()), "x_max": float(pts[:, 0].max()),
            "y_min": float(pts[:, 1].min()), "y_max": float(pts[:, 1].max()),
            "z_min": float(pts[:, 2].min()), "z_max": float(pts[:, 2].max()),
        }

    def _process_tool():
        tool_csets = []
        for j in range(n):
            lo, hi = q_intervals[j]
            vals = [lo, (lo + hi) / 2, hi]
            for k in range(-20, 21):
                v = k * math.pi / 2.0
                if lo < v < hi:
                    vals.append(v)
            tool_csets.append(sorted(set(vals)))
        q_batch = _make_q_batch_np(tool_csets, mid_q, list(range(n)))
        N = len(q_batch)
        T = np.zeros((N, 4, 4)); T[:] = np.eye(4)
        for i, dh in enumerate(dh_params):
            M = _dh_mat_batch(dh["alpha"], dh["a"], dh["d"],
                              dh["theta"], q_batch[:, i])
            T = np.matmul(T, M)
        last_pos = T[:, :3, 3]
        tool_pos = T[:, :3, 2] * PANDA_TOOL_D + T[:, :3, 3]
        pts = np.vstack([last_pos, tool_pos])               # (2N, 3)
        return {
            "frame": n + 1, "label": "Tool",
            "x_min": float(pts[:, 0].min()), "x_max": float(pts[:, 0].max()),
            "y_min": float(pts[:, 1].min()), "y_max": float(pts[:, 1].max()),
            "z_min": float(pts[:, 2].min()), "z_max": float(pts[:, 2].max()),
        }

    link_futs = [_THREAD_POOL.submit(_process_link, idx) for idx in range(1, n + 1)]
    tool_fut = (_THREAD_POOL.submit(_process_tool)
                if len(dh_params) == len(PANDA_DH_PARAMS) else None)
    aabbs = [f.result() for f in link_futs]
    if tool_fut is not None:
        aabbs.append(tool_fut.result())
    return aabbs


def compute_aabbs_interval_planar_2dof(link_lengths: tuple,
                                       q_intervals: list) -> list:
    """
    Conservative interval AABB for a 2-DOF planar robot using direct
    analytic interval arithmetic (correct, unlike the DH-based approach).

    The robot FK is:
      elbow = (L1*cos(q1),         L1*sin(q1))
      ee    = (L1*cos(q1)+L2*cos(q1+q2), L1*sin(q1)+L2*sin(q1+q2))

    Parameters
    ----------
    link_lengths : (L1, L2)
    q_intervals  : [(q1_lo, q1_hi), (q2_lo, q2_hi)]

    Returns
    -------
    list of 2 aabb dicts: [Elbow (frame 1), EE (frame 2)]
    """
    L1, L2 = float(link_lengths[0]), float(link_lengths[1])
    q1lo, q1hi = float(q_intervals[0][0]), float(q_intervals[0][1])
    q2lo, q2hi = float(q_intervals[1][0]), float(q_intervals[1][1])
    q12lo = q1lo + q2lo
    q12hi = q1hi + q2hi

    cx1 = _iv_scale(L1, _iv_cos(q1lo, q1hi))
    cy1 = _iv_scale(L1, _iv_sin(q1lo, q1hi))
    cx2 = _iv_add(cx1, _iv_scale(L2, _iv_cos(q12lo, q12hi)))
    cy2 = _iv_add(cy1, _iv_scale(L2, _iv_sin(q12lo, q12hi)))

    # Link 1 body: segment from base (0,0) to elbow
    # Link 2 body: segment from elbow to EE
    return [
        {"frame": 1, "label": "Link 1 (base→elbow)",
         "x_min": min(0.0, cx1[0]), "x_max": max(0.0, cx1[1]),
         "y_min": min(0.0, cy1[0]), "y_max": max(0.0, cy1[1]),
         "z_min": 0.0, "z_max": 0.0},
        {"frame": 2, "label": "Link 2 (elbow→EE)",
         "x_min": min(cx1[0], cx2[0]), "x_max": max(cx1[1], cx2[1]),
         "y_min": min(cy1[0], cy2[0]), "y_max": max(cy1[1], cy2[1]),
         "z_min": 0.0, "z_max": 0.0},
    ]


def compute_aabbs_critical_2dof(link_lengths: tuple,
                                 q_intervals: list) -> list:
    """
    AABB via critical-angle sampling for a 2-DOF planar robot.

    Uses the correct analytic FK: critical arguments are q1 and q1+q2,
    so the critical angles per joint are k*pi/2 per joint (plus boundaries).

    Parameters
    ----------
    link_lengths : (L1, L2)
    q_intervals  : [(q1_lo, q1_hi), (q2_lo, q2_hi)]

    Returns
    -------
    list of 2 aabb dicts: [Elbow (frame 1), EE (frame 2)]
    """
    L1, L2 = float(link_lengths[0]), float(link_lengths[1])
    q1lo, q1hi = float(q_intervals[0][0]), float(q_intervals[0][1])
    q2lo, q2hi = float(q_intervals[1][0]), float(q_intervals[1][1])

    def _crit(lo, hi):
        vals = [lo, hi]
        for k in range(-20, 21):
            v = k * math.pi / 2.0
            if lo < v < hi:
                vals.append(v)
        return sorted(set(vals))

    # Link 1: segment from base(0,0) to elbow — depends only on q1
    crit_q1 = _crit(q1lo, q1hi)
    # always include origin as part of link 1 body
    xn1, xp1 = 0.0, 0.0
    yn1, yp1 = 0.0, 0.0
    # track elbow range separately (without base origin) for link 2 initialization
    xe_n, xe_p = math.inf, -math.inf
    ye_n, ye_p = math.inf, -math.inf
    for q1 in crit_q1:
        x = L1 * math.cos(q1); y = L1 * math.sin(q1)
        xn1, xp1 = min(xn1, x), max(xp1, x)
        yn1, yp1 = min(yn1, y), max(yp1, y)
        xe_n, xe_p = min(xe_n, x), max(xe_p, x)
        ye_n, ye_p = min(ye_n, y), max(ye_p, y)

    # Link 2: segment from elbow to EE — depends on q1 and q1+q2
    q12lo = q1lo + q2lo
    q12hi = q1hi + q2hi
    crit_q12 = _crit(q12lo, q12hi)
    crit_q2  = _crit(q2lo, q2hi)
    # initialize from actual elbow range (not link 1 which includes origin)
    xn2, xp2 = xe_n, xe_p
    yn2, yp2 = ye_n, ye_p
    for q12 in crit_q12:
        for q1 in [q1lo, q1hi]:
            q2 = q12 - q1
            if q2 < q2lo - 1e-9 or q2 > q2hi + 1e-9:
                continue
            ex = L1 * math.cos(q1); ey = L1 * math.sin(q1)
            x = ex + L2 * math.cos(q12); y = ey + L2 * math.sin(q12)
            # hull both the elbow and the EE point
            xn2, xp2 = min(xn2, ex, x), max(xp2, ex, x)
            yn2, yp2 = min(yn2, ey, y), max(yp2, ey, y)
    for q1 in crit_q1:
        for q2 in crit_q2:
            ex = L1 * math.cos(q1); ey = L1 * math.sin(q1)
            q12 = q1 + q2
            x = ex + L2 * math.cos(q12); y = ey + L2 * math.sin(q12)
            xn2, xp2 = min(xn2, ex, x), max(xp2, ex, x)
            yn2, yp2 = min(yn2, ey, y), max(yp2, ey, y)

    return [
        {"frame": 1, "label": "Link 1 (base→elbow)",
         "x_min": xn1, "x_max": xp1, "y_min": yn1, "y_max": yp1,
         "z_min": 0.0, "z_max": 0.0},
        {"frame": 2, "label": "Link 2 (elbow→EE)",
         "x_min": xn2, "x_max": xp2, "y_min": yn2, "y_max": yp2,
         "z_min": 0.0, "z_max": 0.0},
    ]


def compute_aabbs_random(fk_func, q_intervals: list,
                          n_samples: int = 5000, seed: int = 0) -> list:
    """
    Compute per-frame AABB envelopes via uniform random sampling.

    Parameters
    ----------
    fk_func     : callable, fk_func(q) -> np.ndarray of shape (n_frames, D).
                  D=2 for planar robots, D=3 for spatial robots.
    q_intervals : list of (lo, hi) tuples
    n_samples   : number of random samples
    seed        : RNG seed

    Returns
    -------
    list of aabb dicts with keys frame, label, x_min, x_max,
    y_min, y_max, z_min, z_max (z = 0 for 2D robots).
    """
    rng = np.random.default_rng(seed)
    n = len(q_intervals)

    test_q = np.array([(lo + hi) / 2 for lo, hi in q_intervals])
    test_pos = np.atleast_2d(np.asarray(fk_func(test_q), dtype=float))
    n_frames, D = test_pos.shape

    mn = np.full((n_frames, D),  np.inf)
    mx = np.full((n_frames, D), -np.inf)

    def _update(q):
        pos = np.atleast_2d(np.asarray(fk_func(q), dtype=float))
        np.minimum(mn, pos, out=mn)
        np.maximum(mx, pos, out=mx)

    for _ in range(n_samples):
        _update(np.array([rng.uniform(lo, hi) for lo, hi in q_intervals]))
    for combo in itertools.product(*[(lo, hi) for lo, hi in q_intervals]):
        _update(np.array(combo, dtype=float))

    dim_keys = ["x", "y", "z"]
    # Per-frame endpoint AABBs
    frame_aabbs = []
    for i in range(n_frames):
        d = {"frame": i, "label": f"Frame {i}"}
        for j in range(D):
            k = dim_keys[j]
            d[f"{k}_min"] = float(mn[i, j])
            d[f"{k}_max"] = float(mx[i, j])
        for k in ("x", "y", "z"):
            d.setdefault(f"{k}_min", 0.0)
            d.setdefault(f"{k}_max", 0.0)
        frame_aabbs.append(d)

    # Convert to link-body AABBs: hull each frame with its parent frame.
    # frame 0 is the robot base (always at origin) — kept as a point.
    # frame i (i>=1) covers the segment from frame i-1 to frame i.
    aabbs = [frame_aabbs[0]]  # base frame unchanged
    for i in range(1, n_frames):
        pa = frame_aabbs[i - 1]
        ca = frame_aabbs[i]
        aabbs.append({
            "frame": ca["frame"],
            "label": ca["label"],
            "x_min": min(pa["x_min"], ca["x_min"]),
            "x_max": max(pa["x_max"], ca["x_max"]),
            "y_min": min(pa["y_min"], ca["y_min"]),
            "y_max": max(pa["y_max"], ca["y_max"]),
            "z_min": min(pa["z_min"], ca["z_min"]),
            "z_max": max(pa["z_max"], ca["z_max"]),
        })
    return aabbs


# ─── OBB computation ─────────────────────────────────────────────────────────

def _idot_lo(e: np.ndarray, p_lo: np.ndarray, p_hi: np.ndarray) -> float:
    """Conservative lower bound of e·p for p in [p_lo, p_hi] (interval dot product)."""
    return float(sum(e[k] * p_lo[k] if e[k] >= 0 else e[k] * p_hi[k] for k in range(3)))


def _idot_hi(e: np.ndarray, p_lo: np.ndarray, p_hi: np.ndarray) -> float:
    """Conservative upper bound of e·p for p in [p_lo, p_hi] (interval dot product)."""
    return float(sum(e[k] * p_hi[k] if e[k] >= 0 else e[k] * p_lo[k] for k in range(3)))


def _gram_schmidt_frame(direction: np.ndarray):
    """
    Build a right-handed orthonormal frame (ax, ay, az) given a primary direction.
    Falls back to world-X if direction is degenerate.
    """
    ax = np.asarray(direction, dtype=float).copy()
    n = float(np.linalg.norm(ax))
    if n < 1e-8:
        ax = np.array([1.0, 0.0, 0.0])
    else:
        ax = ax / n
    # Choose world axis least collinear with ax
    abs_ax = np.abs(ax)
    world = [np.array([1., 0., 0.]), np.array([0., 1., 0.]), np.array([0., 0., 1.])]
    candidate = world[int(np.argmin(abs_ax))]
    ay = candidate - np.dot(candidate, ax) * ax
    ay = ay / np.linalg.norm(ay)
    az = np.cross(ax, ay)
    return ax, ay, az


def compute_obbs_interval_fk(dh_params: list, q_intervals: list) -> list:
    """
    Compute per-link OBB envelopes via interval arithmetic on a DH chain.

    The algorithm mirrors the former ``extract_link_obbs`` (removed in HCACHE03
    cleanup; see ``collision_policy.h`` for the current C++ implementation):
    for each link segment the primary OBB axis is aligned with the midpoint
    direction of the link, a Gram-Schmidt frame is built, then conservative
    interval dot-product projections of both endpoint intervals onto that
    frame are unioned to produce the half-extents.

    Parameters
    ----------
    dh_params   : list of dicts with keys alpha, a, d, theta
    q_intervals : list of (lo, hi) tuples, one per joint

    Returns
    -------
    list of dicts, one per link (plus tool for Panda), each with keys:
        frame  : int  (1-based)
        label  : str
        center : np.ndarray (3,) — projection of center onto OBB axes
        half   : np.ndarray (3,) — half-extents along OBB axes
        axes   : np.ndarray (3, 3) — rows are world-space unit axis vectors

    The world-space center is:   axes.T @ center  =  sum_j center[j]*axes[j]
    World-space corners: see _obb_world_corners().
    """
    T = _numpy_to_iv_mat4(np.eye(4))
    prev_pos = [T[r][3] for r in range(3)]  # interval (lo, hi) per coordinate
    obbs = []

    for i, dh in enumerate(dh_params):
        lo = dh["theta"] + q_intervals[i][0]
        hi = dh["theta"] + q_intervals[i][1]
        ct = _iv_cos(lo, hi)
        st = _iv_sin(lo, hi)
        A = _iv_dh_mat(dh["alpha"], dh["a"], dh["d"], ct, st)
        T = _iv_mat4_mul(T, A)
        cur_pos = [T[r][3] for r in range(3)]

        s_lo = np.array([prev_pos[r][0] for r in range(3)])
        s_hi = np.array([prev_pos[r][1] for r in range(3)])
        e_lo = np.array([cur_pos[r][0]  for r in range(3)])
        e_hi = np.array([cur_pos[r][1]  for r in range(3)])

        mid_s = 0.5 * (s_lo + s_hi)
        mid_e = 0.5 * (e_lo + e_hi)
        ax, ay, az = _gram_schmidt_frame(mid_e - mid_s)
        axes = np.array([ax, ay, az])  # (3, 3)

        proj_lo = np.zeros(3)
        proj_hi = np.zeros(3)
        for j, e in enumerate(axes):
            sl = _idot_lo(e, s_lo, s_hi);  sh = _idot_hi(e, s_lo, s_hi)
            el = _idot_lo(e, e_lo, e_hi);  eh = _idot_hi(e, e_lo, e_hi)
            proj_lo[j] = min(sl, el)
            proj_hi[j] = max(sh, eh)

        obbs.append({
            "frame":  i + 1,
            "label":  f"Link {i + 1}",
            "center": 0.5 * (proj_lo + proj_hi),
            "half":   0.5 * (proj_hi - proj_lo),
            "axes":   axes,
        })
        prev_pos = cur_pos

    # Tool (Panda-specific z-offset along local z)
    if len(dh_params) == len(PANDA_DH_PARAMS):
        Tt = [row[:] for row in T]
        for row in range(3):
            shift = _iv_scale(PANDA_TOOL_D, T[row][2])
            Tt[row][3] = _iv_add(T[row][3], shift)
        tool_pos = [Tt[r][3] for r in range(3)]

        s_lo = np.array([prev_pos[r][0] for r in range(3)])
        s_hi = np.array([prev_pos[r][1] for r in range(3)])
        e_lo = np.array([tool_pos[r][0] for r in range(3)])
        e_hi = np.array([tool_pos[r][1] for r in range(3)])

        mid_s = 0.5 * (s_lo + s_hi)
        mid_e = 0.5 * (e_lo + e_hi)
        ax, ay, az = _gram_schmidt_frame(mid_e - mid_s)
        axes = np.array([ax, ay, az])

        proj_lo = np.zeros(3)
        proj_hi = np.zeros(3)
        for j, e in enumerate(axes):
            sl = _idot_lo(e, s_lo, s_hi);  sh = _idot_hi(e, s_lo, s_hi)
            el = _idot_lo(e, e_lo, e_hi);  eh = _idot_hi(e, e_lo, e_hi)
            proj_lo[j] = min(sl, el)
            proj_hi[j] = max(sh, eh)

        obbs.append({
            "frame":  len(dh_params) + 1,
            "label":  "Tool",
            "center": 0.5 * (proj_lo + proj_hi),
            "half":   0.5 * (proj_hi - proj_lo),
            "axes":   axes,
        })

    return obbs


def obb_volume(obb: dict) -> float:
    """Compute volume of an OBB dict (8 * hx * hy * hz)."""
    h = obb["half"]
    return 8.0 * max(0.0, float(h[0])) * max(0.0, float(h[1])) * max(0.0, float(h[2]))


def _obb_world_corners(obb: dict) -> np.ndarray:
    """
    Compute the 8 world-space corners of an OBB.

    Layout (row index  →  sign combination s0, s1, s2):
        0 → (-,-,-)    1 → (-,-,+)    2 → (-,+,-)    3 → (-,+,+)
        4 → (+,-,-)    5 → (+,-,+)    6 → (+,+,-)    7 → (+,+,+)

    Returns (8, 3) float64 array.
    """
    axes = obb["axes"]          # (3, 3)
    c    = obb["center"]        # (3,) in projected space
    h    = obb["half"]          # (3,)
    # World-space center
    wc = c[0]*axes[0] + c[1]*axes[1] + c[2]*axes[2]
    signs = [(-1, -1, -1), (-1, -1, +1), (-1, +1, -1), (-1, +1, +1),
             (+1, -1, -1), (+1, -1, +1), (+1, +1, -1), (+1, +1, +1)]
    corners = np.array([
        wc + s0*h[0]*axes[0] + s1*h[1]*axes[1] + s2*h[2]*axes[2]
        for s0, s1, s2 in signs
    ])
    return corners  # (8, 3)


def _pca_obb_from_points(pts: np.ndarray, frame: int, label: str) -> dict:
    """
    Compute a tight OBB for a point cloud via PCA.

    The three principal axes of ``pts`` (eigenvectors of the covariance matrix,
    sorted by descending eigenvalue) define the OBB orientation.  All points
    are projected onto these axes and the min/max projections become the OBB
    extents.  This is provably tighter than the AABB for elongated point clouds.

    Parameters
    ----------
    pts   : (N, 3) float array — point cloud (at least 2 points)
    frame : int   — frame index stored in the result dict
    label : str   — label stored in the result dict

    Returns
    -------
    OBB dict with keys frame, label, center (3,), half (3,), axes (3×3)
    (same layout as compute_obbs_interval_fk output).
    """
    if len(pts) < 2:
        # Degenerate: single point
        zero = np.zeros(3)
        return {"frame": frame, "label": label,
                "center": zero.copy(), "half": zero.copy(),
                "axes": np.eye(3)}
    pts = np.asarray(pts, dtype=float)
    mean = pts.mean(axis=0)
    cov  = np.cov((pts - mean).T)          # (3, 3)
    # Ensure well-conditioned
    cov += np.eye(3) * 1e-12
    eigvals, eigvecs = np.linalg.eigh(cov) # sorted ascending
    # Axes: descending eigenvalue order (primary = most variance)
    axes = eigvecs[:, ::-1].T.copy()       # (3, 3), rows are unit axes
    # Ensure right-handed
    axes[2] = np.cross(axes[0], axes[1])
    axes[2] /= np.linalg.norm(axes[2])
    # Project all points
    proj = pts @ axes.T                    # (N, 3)
    proj_lo = proj.min(axis=0)
    proj_hi = proj.max(axis=0)
    center  = 0.5 * (proj_lo + proj_hi)
    half    = 0.5 * (proj_hi - proj_lo)
    return {"frame": frame, "label": label,
            "center": center, "half": half, "axes": axes}


def compute_obbs_critical(dh_params: list, q_intervals: list,
                           max_combos: int = 60_000) -> list:
    """
    Compute tight per-link OBB envelopes via critical-point enumeration + PCA.

    For each link segment, critical joint angles (k*π/2 grid ∪ boundaries) are
    enumerated.  The resulting point cloud of {parent_endpoint, child_endpoint}
    positions across all critical configurations is passed to PCA, which yields
    the three axes of minimum-volume oriented bounding box.  The OBB is
    **always smaller or equal in volume** compared to the AABB from
    ``compute_aabbs_critical``.
    Uses vectorised batch FK (_fk_combos_batch) and a ThreadPoolExecutor
    to compute all links in parallel.

    Parameters
    ----------
    dh_params   : list of dicts (alpha, a, d, theta)
    q_intervals : list of (lo, hi) tuples
    max_combos  : cap on combinatorial explosion per link

    Returns
    -------
    list of OBB dicts (frame, label, center, half, axes) — same layout as
    ``compute_obbs_interval_fk``.
    """
    n = len(dh_params)
    mid_q = np.array([(lo + hi) / 2 for lo, hi in q_intervals])

    def _crit_angles(lo, hi):
        vals = [lo, hi]
        for k in range(-20, 21):
            v = k * math.pi / 2.0
            if lo < v < hi:
                vals.append(v)
        return sorted(set(vals))

    per_joint = [_crit_angles(*q_intervals[j]) for j in range(n)]

    def _link_qbatch(link_idx):
        relevant = list(range(link_idx))
        csets = [per_joint[j] for j in relevant]
        total = 1
        for cs in csets:
            total *= len(cs)
        if total > max_combos:
            csets = []
            for j in relevant:
                lo, hi = q_intervals[j]
                vals = [lo, (lo + hi) / 2, hi]
                for k in range(-20, 21):
                    v = k * math.pi / 2.0
                    if lo < v < hi:
                        vals.append(v)
                csets.append(sorted(set(vals)))
        return _make_q_batch_np(csets, mid_q, relevant)

    def _process_link(link_idx):
        pts = _fk_combos_batch(dh_params, _link_qbatch(link_idx), link_idx)
        return _pca_obb_from_points(pts, frame=link_idx, label=f"Link {link_idx}")

    def _process_tool():
        tool_csets = []
        for j in range(n):
            lo, hi = q_intervals[j]
            vals = [lo, (lo + hi) / 2, hi]
            for k in range(-20, 21):
                v = k * math.pi / 2.0
                if lo < v < hi:
                    vals.append(v)
            tool_csets.append(sorted(set(vals)))
        q_batch = _make_q_batch_np(tool_csets, mid_q, list(range(n)))
        N = len(q_batch)
        T = np.zeros((N, 4, 4)); T[:] = np.eye(4)
        for i, dh in enumerate(dh_params):
            M = _dh_mat_batch(dh["alpha"], dh["a"], dh["d"],
                              dh["theta"], q_batch[:, i])
            T = np.matmul(T, M)
        last_pos = T[:, :3, 3].copy()
        tool_pos = T[:, :3, 2] * PANDA_TOOL_D + T[:, :3, 3]
        pts = np.vstack([last_pos, tool_pos])               # (2N, 3)
        return _pca_obb_from_points(pts, frame=n + 1, label="Tool")

    link_futs = [_THREAD_POOL.submit(_process_link, idx) for idx in range(1, n + 1)]
    tool_fut = (_THREAD_POOL.submit(_process_tool)
                if len(dh_params) == len(PANDA_DH_PARAMS) else None)
    obbs = [f.result() for f in link_futs]
    if tool_fut is not None:
        obbs.append(tool_fut.result())
    return obbs


# ─── 2D OBB computation ──────────────────────────────────────────────────────

def _pca_obb2d_from_points(pts: np.ndarray, frame: int, label: str) -> dict:
    """
    Compute a tight 2D OBB for a point cloud via PCA.

    Parameters
    ----------
    pts   : (N, 2) float array
    frame : int
    label : str

    Returns
    -------
    dict with keys:
        frame  : int
        label  : str
        center : np.ndarray (2,) — world-space center
        half   : np.ndarray (2,) — half-extents along local axes
        axes   : np.ndarray (2, 2) — rows are 2D unit axis vectors
    """
    pts = np.asarray(pts, dtype=float)
    if len(pts) < 2:
        return {"frame": frame, "label": label,
                "center": np.zeros(2), "half": np.zeros(2),
                "axes": np.eye(2)}
    mean = pts.mean(axis=0)
    cov  = np.cov((pts - mean).T)   # (2, 2)
    cov += np.eye(2) * 1e-12
    eigvals, eigvecs = np.linalg.eigh(cov)   # ascending order
    # Primary axis = eigenvector with largest eigenvalue
    axes = eigvecs[:, ::-1].T.copy()          # (2, 2), rows are unit axes
    # Ensure ax0 points in +x half-plane (cosmetic consistency)
    if axes[0, 0] < 0:
        axes[0] = -axes[0]
    # ax1 perpendicular, CCW from ax0
    axes[1] = np.array([-axes[0, 1], axes[0, 0]])
    proj    = pts @ axes.T          # (N, 2)
    proj_lo = proj.min(axis=0)
    proj_hi = proj.max(axis=0)
    center  = 0.5 * (proj_lo + proj_hi)
    half    = 0.5 * (proj_hi - proj_lo)
    return {"frame": frame, "label": label,
            "center": center, "half": half, "axes": axes}


def _obb2d_corners(obb2d: dict) -> np.ndarray:
    """
    Compute the 4 world-space corners of a 2D OBB.

    Returns (4, 2) array in counter-clockwise order.
    """
    c  = obb2d["center"]   # (2,) in projected space
    h  = obb2d["half"]     # (2,)
    ax = obb2d["axes"]     # (2, 2)
    # World-space center
    wc = c[0] * ax[0] + c[1] * ax[1]
    return np.array([
        wc - h[0]*ax[0] - h[1]*ax[1],
        wc + h[0]*ax[0] - h[1]*ax[1],
        wc + h[0]*ax[0] + h[1]*ax[1],
        wc - h[0]*ax[0] + h[1]*ax[1],
    ])


def obb_area_2d(obb2d: dict) -> float:
    """Area of a 2D OBB: 4 * half[0] * half[1]."""
    h = obb2d["half"]
    return 4.0 * max(0.0, float(h[0])) * max(0.0, float(h[1]))


def compute_obbs_critical_2dof(link_lengths: tuple,
                               q_intervals: list) -> list:
    """
    Compute tight per-link 2D OBB envelopes for a planar 2-DOF robot.

    The algorithm:
    * Enumerates critical joint angles (k*π/2 grid ∪ interval boundaries).
    * Collects the point cloud {parent, child} endpoint positions for each
      link across all critical configurations.
    * Fits a 2D PCA OBB to that point cloud.

    Parameters
    ----------
    link_lengths : (L1, L2)
    q_intervals  : [(q1_lo, q1_hi), (q2_lo, q2_hi)]

    Returns
    -------
    list of 2 OBB dicts (frame 1 = base→elbow, frame 2 = elbow→EE), each with
    keys frame, label, center (2,), half (2,), axes (2×2).
    """
    L1, L2  = float(link_lengths[0]), float(link_lengths[1])
    q1lo, q1hi = float(q_intervals[0][0]), float(q_intervals[0][1])
    q2lo, q2hi = float(q_intervals[1][0]), float(q_intervals[1][1])

    def _crit(lo, hi):
        vals = [lo, hi]
        for k in range(-20, 21):
            v = k * math.pi / 2.0
            if lo < v < hi:
                vals.append(v)
        return sorted(set(vals))

    crit_q1  = _crit(q1lo, q1hi)
    crit_q2  = _crit(q2lo, q2hi)
    q12lo    = q1lo + q2lo
    q12hi    = q1hi + q2hi
    crit_q12 = _crit(q12lo, q12hi)

    # --- Link 1: base (0,0) → elbow ---
    pts1 = [[0.0, 0.0]]   # base is always part of link-1 body
    for q1 in crit_q1:
        pts1.append([L1 * math.cos(q1), L1 * math.sin(q1)])
    obb1 = _pca_obb2d_from_points(np.array(pts1), frame=1,
                                   label="Link 1 (base→elbow)")

    # --- Link 2: elbow → EE ---
    pts2 = []
    for q1 in crit_q1:
        ex = L1 * math.cos(q1); ey = L1 * math.sin(q1)
        for q2 in crit_q2:
            q12 = q1 + q2
            pts2.append([ex, ey])
            pts2.append([ex + L2 * math.cos(q12), ey + L2 * math.sin(q12)])
    for q12 in crit_q12:
        for q1 in [q1lo, q1hi]:
            q2 = q12 - q1
            if q2lo - 1e-9 <= q2 <= q2hi + 1e-9:
                ex = L1 * math.cos(q1); ey = L1 * math.sin(q1)
                pts2.append([ex, ey])
                pts2.append([ex + L2 * math.cos(q12), ey + L2 * math.sin(q12)])
    obb2 = _pca_obb2d_from_points(np.array(pts2), frame=2,
                                   label="Link 2 (elbow→EE)")

    return [obb1, obb2]


def compute_obbs_interval_fk_2dof(link_lengths: tuple,
                                   q_intervals: list) -> list:
    """
    Compute per-link 2D OBB envelopes for a planar 2-DOF robot via
    interval arithmetic (mirrors ``compute_obbs_interval_fk`` for DH robots).

    For each link the primary OBB axis is the midpoint direction of the link.
    Gram-Schmidt gives the perpendicular axis.  Conservative interval
    dot-product projections of both endpoint intervals onto those two axes
    are unioned to the final half-extents.

    Note: because the primary axis is fixed to the *midpoint* configuration,
    this method tends to produce *larger* OBBs than the PCA-critical approach,
    especially for wide intervals (same bias as the 3-D IFK version).

    Parameters
    ----------
    link_lengths : (L1, L2)
    q_intervals  : [(q1_lo, q1_hi), (q2_lo, q2_hi)]

    Returns
    -------
    list of 2 OBB dicts (same layout as ``compute_obbs_critical_2dof``).
    """
    L1, L2    = float(link_lengths[0]), float(link_lengths[1])
    q1lo, q1hi = float(q_intervals[0][0]), float(q_intervals[0][1])
    q2lo, q2hi = float(q_intervals[1][0]), float(q_intervals[1][1])
    q12lo = q1lo + q2lo
    q12hi = q1hi + q2hi
    q1mid = 0.5 * (q1lo + q1hi)
    q2mid = 0.5 * (q2lo + q2hi)

    # Interval 2D endpoint positions
    cx1 = _iv_scale(L1, _iv_cos(q1lo, q1hi))  # (lo, hi)
    cy1 = _iv_scale(L1, _iv_sin(q1lo, q1hi))
    cx2 = _iv_add(cx1, _iv_scale(L2, _iv_cos(q12lo, q12hi)))
    cy2 = _iv_add(cy1, _iv_scale(L2, _iv_sin(q12lo, q12hi)))

    def _idot2d_lo(e, xlo, xhi, ylo, yhi):
        return (e[0]*xlo if e[0] >= 0 else e[0]*xhi) + \
               (e[1]*ylo if e[1] >= 0 else e[1]*yhi)

    def _idot2d_hi(e, xlo, xhi, ylo, yhi):
        return (e[0]*xhi if e[0] >= 0 else e[0]*xlo) + \
               (e[1]*yhi if e[1] >= 0 else e[1]*ylo)

    def _2d_obb_from_intervals(s_x, s_y, e_x, e_y,
                                mid_sx, mid_sy, mid_ex, mid_ey,
                                frame, label):
        """Build a 2D OBB from start and end interval rectangles."""
        # Primary axis: midpoint direction of the link
        dx = mid_ex - mid_sx
        dy = mid_ey - mid_sy
        n = math.hypot(dx, dy)
        if n < 1e-8:
            ax0 = np.array([1.0, 0.0])
        else:
            ax0 = np.array([dx / n, dy / n])
        # Ensure ax0 points into the right half-plane (positive x component)
        if ax0[0] < 0:
            ax0 = -ax0
        ax1 = np.array([-ax0[1], ax0[0]])  # CCW perpendicular
        axes = np.array([ax0, ax1])

        proj_lo = np.zeros(2)
        proj_hi = np.zeros(2)
        for j, e in enumerate(axes):
            # start interval (if it's a point, xlo==xhi and ylo==yhi)
            sl = _idot2d_lo(e, s_x[0], s_x[1], s_y[0], s_y[1])
            sh = _idot2d_hi(e, s_x[0], s_x[1], s_y[0], s_y[1])
            # end interval
            el = _idot2d_lo(e, e_x[0], e_x[1], e_y[0], e_y[1])
            eh = _idot2d_hi(e, e_x[0], e_x[1], e_y[0], e_y[1])
            proj_lo[j] = min(sl, el)
            proj_hi[j] = max(sh, eh)

        return {
            "frame":  frame,
            "label":  label,
            "center": 0.5 * (proj_lo + proj_hi),
            "half":   0.5 * (proj_hi - proj_lo),
            "axes":   axes,
        }

    # Link 1: base (0,0) → elbow
    mid_ex1 = L1 * math.cos(q1mid)
    mid_ey1 = L1 * math.sin(q1mid)
    obb1 = _2d_obb_from_intervals(
        (_iv_c(0.0)), (_iv_c(0.0)),   # base is a point
        cx1, cy1,
        0.0, 0.0, mid_ex1, mid_ey1,
        frame=1, label="Link 1 (base\u2192elbow)")

    # Link 2: elbow → EE
    mid_ex2 = mid_ex1 + L2 * math.cos(q1mid + q2mid)
    mid_ey2 = mid_ey1 + L2 * math.sin(q1mid + q2mid)
    obb2 = _2d_obb_from_intervals(
        cx1, cy1,
        cx2, cy2,
        mid_ex1, mid_ey1, mid_ex2, mid_ey2,
        frame=2, label="Link 2 (elbow\u2192EE)")

    return [obb1, obb2]


# ─── Random scene generation ──────────────────────────────────────────────────

def random_joint_subintervals(joint_limits: list, rng,
                               width_lo: float = 0.3,
                               width_hi: float = 0.7) -> list:
    """
    Sample random joint sub-intervals within joint limits.

    Parameters
    ----------
    joint_limits : list of (lo, hi) tuples (full range per joint)
    rng          : numpy random Generator
    width_lo     : minimum interval half-width (rad)
    width_hi     : maximum interval half-width (rad)

    Returns
    -------
    list of (lo, hi) tuples, one per joint
    """
    result = []
    for lo_lim, hi_lim in joint_limits:
        half_w = rng.uniform(width_lo / 2, width_hi / 2)
        center = rng.uniform(lo_lim + half_w, hi_lim - half_w)
        result.append((max(lo_lim, center - half_w),
                        min(hi_lim, center + half_w)))
    return result


def random_workspace_obstacles(rng, n: int = 3,
                                center_lo=None, center_hi=None,
                                size_lo: float = 0.04,
                                size_hi: float = 0.14) -> list:
    """
    Generate random AABB workspace obstacles.

    Parameters
    ----------
    rng        : numpy random Generator
    n          : number of obstacles
    center_lo  : (3,) lower bound for obstacle centers (default [-0.5,-0.5, 0.1])
    center_hi  : (3,) upper bound for obstacle centers (default [ 0.5, 0.5, 0.9])
    size_lo    : minimum half-size per axis
    size_hi    : maximum half-size per axis

    Returns
    -------
    list of dicts with keys center, half, name, color
    """
    if center_lo is None:
        center_lo = np.array([-0.5, -0.5, 0.1])
    if center_hi is None:
        center_hi = np.array([ 0.5,  0.5, 0.9])
    center_lo = np.asarray(center_lo, dtype=float)
    center_hi = np.asarray(center_hi, dtype=float)

    # Distinct, readable colors for obstacles — muted reds/oranges/blues
    colors = ["#B71C1C", "#E65100", "#1A237E", "#1B5E20", "#4A148C"]
    obs = []
    for i in range(n):
        center = rng.uniform(center_lo, center_hi).tolist()
        half   = rng.uniform(size_lo, size_hi, size=3).tolist()
        obs.append({
            "center": center,
            "half":   half,
            "name":   f"obs_{i}",
            "color":  colors[i % len(colors)],
        })
    return obs


# ─── Output directory helper ──────────────────────────────────────────────────

def make_output_dir(base_dir, prefix: str = "run") -> Path:
    """
    Create a timestamped output directory.

    Parameters
    ----------
    base_dir : str or Path — parent directory
    prefix   : string prefix for the folder name

    Returns
    -------
    Path to the created directory.
    """
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out = Path(base_dir) / f"{prefix}_{ts}"
    out.mkdir(parents=True, exist_ok=True)
    return out


# ─── Matplotlib 2D AABB drawing helpers ──────────────────────────────────────

def draw_aabb_2d(ax, aabb: dict, dim_x: int = 0, dim_y: int = 1,
                 color: str = "steelblue", alpha: float = 0.12,
                 edgecolor: str = None, linewidth: float = 1.2,
                 linestyle: str = "--", label: str = None):
    """
    Draw an AABB as a 2D rectangle on a matplotlib axis.

    aabb must have keys {x,y,z}_min / {x,y,z}_max.
    dim_x/dim_y select which axes to project onto (0=x,1=y,2=z).
    """
    try:
        import matplotlib.patches as mp
    except ImportError:
        return
    keys = ["x", "y", "z"]
    kx, ky = keys[dim_x], keys[dim_y]
    lo_x = aabb[f"{kx}_min"]; hi_x = aabb[f"{kx}_max"]
    lo_y = aabb[f"{ky}_min"]; hi_y = aabb[f"{ky}_max"]
    if (hi_x - lo_x) < 1e-8 or (hi_y - lo_y) < 1e-8:
        return
    rect = mp.Rectangle(
        (lo_x, lo_y), hi_x - lo_x, hi_y - lo_y,
        facecolor=color, alpha=alpha,
        edgecolor=edgecolor or color,
        linewidth=linewidth, linestyle=linestyle,
        label=label,
    )
    ax.add_patch(rect)


def draw_obb_2d(ax, obb2d: dict, color: str = "steelblue", alpha: float = 0.12,
                edgecolor: str = None, linewidth: float = 1.6,
                linestyle: str = "--", label: str = None):
    """
    Draw a 2D OBB as a filled rotated rectangle on a matplotlib axis.

    Parameters
    ----------
    ax      : matplotlib Axes
    obb2d   : dict with keys center (2,), half (2,), axes (2,2)
    color   : face fill color
    alpha   : face opacity
    edgecolor : edge color (defaults to ``color``)
    linewidth : edge line width
    linestyle : edge line style
    label   : legend label
    """
    try:
        from matplotlib.patches import Polygon
    except ImportError:
        return
    h = obb2d["half"]
    if float(h[0]) < 1e-8 or float(h[1]) < 1e-8:
        return
    corners = _obb2d_corners(obb2d)   # (4, 2)
    patch = Polygon(
        corners, closed=True,
        facecolor=color, alpha=alpha,
        edgecolor=edgecolor or color,
        linewidth=linewidth, linestyle=linestyle,
        label=label,
    )
    ax.add_patch(patch)


def draw_arm_2d(ax, positions: np.ndarray, dim_x: int = 0, dim_y: int = 1,
                color: str = "#333333", alpha: float = 0.15,
                linewidth: float = 1.0, linestyle: str = "-"):
    """Draw a 2D arm configuration (list of 2D or 3D positions)."""
    pos = np.atleast_2d(positions)
    xs = pos[:, dim_x]
    ys = pos[:, dim_y]
    ax.plot(xs, ys, color=color, alpha=alpha,
            linewidth=linewidth, linestyle=linestyle,
            solid_capstyle="round")


# ─── Plotly 3D AABB visualization helpers ─────────────────────────────────────

# Professional, perceptually-distinct per-link colors (inspired by ColorBrewer)
_LINK_COLORS_DEFAULT = [
    "#E53935",  # Link 1 — red
    "#FB8C00",  # Link 2 — orange
    "#C6A800",  # Link 3 — golden
    "#2E7D32",  # Link 4 — forest green
    "#1565C0",  # Link 5 — royal blue
    "#6A1B9A",  # Link 6 — deep purple
    "#00838F",  # Link 7 — teal
    "#AD1457",  # Tool  — raspberry
]


def plotly_aabb_mesh(aabb: dict, color: str, opacity: float = 0.15,
                     name: str = "", showlegend: bool = True):
    """
    Build a Plotly Mesh3d for one AABB box.

    Returns None for degenerate (zero-width) boxes.
    """
    try:
        import plotly.graph_objects as go
    except ImportError:
        raise ImportError("plotly is required for 3D visualization.")
    x0, x1 = aabb["x_min"], aabb["x_max"]
    y0, y1 = aabb["y_min"], aabb["y_max"]
    z0, z1 = aabb["z_min"], aabb["z_max"]
    if (x1-x0) < 1e-7 or (y1-y0) < 1e-7 or (z1-z0) < 1e-7:
        return None
    vx = [x0, x1, x1, x0, x0, x1, x1, x0]
    vy = [y0, y0, y1, y1, y0, y0, y1, y1]
    vz = [z0, z0, z0, z0, z1, z1, z1, z1]
    i  = [0, 0, 4, 4, 0, 0, 2, 2, 0, 0, 1, 1]
    j  = [1, 2, 5, 6, 1, 5, 3, 7, 3, 7, 2, 6]
    k  = [2, 3, 6, 7, 5, 4, 7, 6, 7, 4, 6, 5]
    vol = aabb_volume(aabb)
    return go.Mesh3d(
        x=vx, y=vy, z=vz, i=i, j=j, k=k,
        color=color, opacity=opacity,
        name=name, showlegend=showlegend, flatshading=True,
        hovertemplate=(
            f"<b>{name}</b><br>"
            f"X:[{x0:.3f},{x1:.3f}]  Y:[{y0:.3f},{y1:.3f}]  Z:[{z0:.3f},{z1:.3f}]<br>"
            f"Vol: {vol:.5f} m^3<extra></extra>"
        ),
    )


def plotly_aabb_wireframe(aabb: dict, color: str, width: float = 2.0,
                           name: str = ""):
    """Build a Plotly Scatter3d wireframe for one AABB box. Returns None if degenerate."""
    try:
        import plotly.graph_objects as go
    except ImportError:
        raise ImportError("plotly is required.")
    x0, x1 = aabb["x_min"], aabb["x_max"]
    y0, y1 = aabb["y_min"], aabb["y_max"]
    z0, z1 = aabb["z_min"], aabb["z_max"]
    if (x1-x0) < 1e-7 or (y1-y0) < 1e-7 or (z1-z0) < 1e-7:
        return None
    corners = [
        (x0,y0,z0),(x1,y0,z0),(x1,y1,z0),(x0,y1,z0),
        (x0,y0,z1),(x1,y0,z1),(x1,y1,z1),(x0,y1,z1),
    ]
    edges = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),
             (0,4),(1,5),(2,6),(3,7)]
    xs, ys, zs = [], [], []
    for a_i, b_i in edges:
        xs += [corners[a_i][0], corners[b_i][0], None]
        ys += [corners[a_i][1], corners[b_i][1], None]
        zs += [corners[a_i][2], corners[b_i][2], None]
    return go.Scatter3d(x=xs, y=ys, z=zs, mode="lines",
                        line=dict(color=color, width=width),
                        name=name, showlegend=False, hoverinfo="skip")


def plotly_obb_mesh(obb: dict, color: str, opacity: float = 0.15,
                    name: str = "", showlegend: bool = True):
    """Build a Plotly Mesh3d for one OBB. Returns None if degenerate."""
    try:
        import plotly.graph_objects as go
    except ImportError:
        raise ImportError("plotly is required.")
    h = obb["half"]
    if any(float(v) < 1e-7 for v in h):
        return None
    corners = _obb_world_corners(obb)  # (8, 3)
    # Triangulated faces — index into corners using same sign layout:
    #   bottom (s2=-1): 0,2,4,6   top (s2=+1): 1,3,5,7
    #   front  (s1=-1): 0,1,4,5   back  (s1=+1): 2,3,6,7
    #   left   (s0=-1): 0,1,2,3   right (s0=+1): 4,5,6,7
    i_idx = [0, 2,  1, 3,  0, 1,  2, 3,  0, 1,  4, 5]
    j_idx = [2, 4,  3, 5,  1, 4,  3, 6,  1, 2,  5, 6]
    k_idx = [4, 6,  5, 7,  4, 5,  6, 7,  2, 3,  6, 7]
    vol = obb_volume(obb)
    wc = obb["center"][0]*obb["axes"][0] + obb["center"][1]*obb["axes"][1] + obb["center"][2]*obb["axes"][2]
    return go.Mesh3d(
        x=corners[:, 0].tolist(), y=corners[:, 1].tolist(), z=corners[:, 2].tolist(),
        i=i_idx, j=j_idx, k=k_idx,
        color=color, opacity=opacity,
        name=name, showlegend=showlegend, flatshading=True,
        hovertemplate=(
            f"<b>{name}</b><br>"
            f"Center: ({wc[0]:.3f},{wc[1]:.3f},{wc[2]:.3f})<br>"
            f"Half: ({h[0]:.3f},{h[1]:.3f},{h[2]:.3f})<br>"
            f"Vol: {vol:.5f} m^3<extra></extra>"
        ),
    )


def plotly_obb_wireframe(obb: dict, color: str, width: float = 2.0, name: str = ""):
    """Build a Plotly Scatter3d wireframe for one OBB. Returns None if degenerate."""
    try:
        import plotly.graph_objects as go
    except ImportError:
        raise ImportError("plotly is required.")
    h = obb["half"]
    if any(float(v) < 1e-7 for v in h):
        return None
    corners = _obb_world_corners(obb)  # (8, 3), sign layout matches _obb_world_corners
    # 12 edges: 4 bottom (s2=-1), 4 top (s2=+1), 4 vertical
    edges = [
        (0, 2), (2, 6), (6, 4), (4, 0),   # bottom face (s2=-1): 0,2,4,6
        (1, 3), (3, 7), (7, 5), (5, 1),   # top face    (s2=+1): 1,3,5,7
        (0, 1), (2, 3), (4, 5), (6, 7),   # vertical edges
    ]
    xs, ys, zs = [], [], []
    for ai, bi in edges:
        xs += [corners[ai, 0], corners[bi, 0], None]
        ys += [corners[ai, 1], corners[bi, 1], None]
        zs += [corners[ai, 2], corners[bi, 2], None]
    return go.Scatter3d(x=xs, y=ys, z=zs, mode="lines",
                        line=dict(color=color, width=width),
                        name=name, showlegend=False, hoverinfo="skip")


def plotly_obstacle_traces(obstacles: list) -> list:
    """Build Plotly traces (mesh + wireframe) for a list of obstacle dicts."""
    traces = []
    for obs in obstacles:
        c = np.asarray(obs["center"]); h = np.asarray(obs["half"])
        aabb = {"x_min": c[0]-h[0], "x_max": c[0]+h[0],
                "y_min": c[1]-h[1], "y_max": c[1]+h[1],
                "z_min": c[2]-h[2], "z_max": c[2]+h[2]}
        m = plotly_aabb_mesh(aabb, color=obs["color"], opacity=0.40,
                              name=f"Obs:{obs['name']}", showlegend=True)
        w = plotly_aabb_wireframe(aabb, color=obs["color"], width=2.5)
        if m: traces.append(m)
        if w: traces.append(w)
    return traces


def plotly_arm_trace(positions: np.ndarray, color: str = "#888",
                     width: float = 3.0, opacity: float = 1.0,
                     name: str = "arm", showlegend: bool = True):
    """Build a single Plotly Scatter3d trace for one arm configuration."""
    try:
        import plotly.graph_objects as go
    except ImportError:
        raise ImportError("plotly is required.")
    pos = np.atleast_2d(positions)
    n = pos.shape[0]
    labels = (["Base"] + [f"J{i}" for i in range(1, n - 1)] + ["EE"])[:n]
    return go.Scatter3d(
        x=pos[:, 0].tolist(), y=pos[:, 1].tolist(), z=pos[:, 2].tolist(),
        mode="lines+markers",
        line=dict(color=color, width=width),
        marker=dict(size=3, color=color, opacity=opacity),
        opacity=opacity, name=name, showlegend=showlegend,
        hovertemplate="<b>%{text}</b><br>(%{x:.3f},%{y:.3f},%{z:.3f})<extra></extra>",
        text=labels,
    )


def build_3d_aabb_figure(method_name: str, aabbs: list,
                          arm_samples_pos: list, mid_pos: np.ndarray,
                          obstacles: list, method_color: str,
                          t_comp: float = 0.0,
                          link_colors: list = None,
                          skip_frames: set = None):
    """
    Build a standalone interactive Plotly figure for one AABB method.

    Parameters
    ----------
    method_name     : display name of the method
    aabbs           : list of aabb dicts (from compute_aabbs_*)
    arm_samples_pos : list of np.ndarray (n_frames, 3), ghost arm positions
    mid_pos         : np.ndarray (n_frames, 3), bold midpoint arm
    obstacles       : list of obstacle dicts
    method_color    : hex color for AABB wireframes
    t_comp          : computation time (s)
    link_colors     : per-link hex colors (default _LINK_COLORS_DEFAULT)
    skip_frames     : set of frame indices to omit from AABB display
    """
    try:
        import plotly.graph_objects as go
    except ImportError:
        raise ImportError("plotly is required.")
    if link_colors is None:
        link_colors = _LINK_COLORS_DEFAULT
    if skip_frames is None:
        skip_frames = set()

    traces = []
    traces += plotly_obstacle_traces(obstacles)

    for k, pos in enumerate(arm_samples_pos):
        traces.append(plotly_arm_trace(
            pos, color="#aaaaaa", width=1.2, opacity=0.10,
            name="Sampled arms" if k == 0 else "", showlegend=(k == 0)))

    mid_pos = np.atleast_2d(mid_pos)
    for j in range(len(mid_pos) - 1):
        c = link_colors[j % len(link_colors)]
        traces.append(go.Scatter3d(
            x=[mid_pos[j, 0], mid_pos[j+1, 0]],
            y=[mid_pos[j, 1], mid_pos[j+1, 1]],
            z=[mid_pos[j, 2], mid_pos[j+1, 2]],
            mode="lines+markers",
            line=dict(color=c, width=8),
            marker=dict(size=5, color=c),
            name=f"Link {j+1}" if j < 7 else "Tool link",
            showlegend=(j == 0),
        ))

    for aabb in aabbs:
        if aabb.get("frame") in skip_frames:
            continue
        lc = link_colors[(aabb["frame"] - 1) % len(link_colors)]
        vol = aabb_volume(aabb)
        label = aabb.get("label", f"Frame {aabb['frame']}")
        m = plotly_aabb_mesh(aabb, color=lc, opacity=0.13,
                              name=f"{label} ({vol:.4f} m^3)", showlegend=True)
        w = plotly_aabb_wireframe(aabb, color=method_color, width=1.8)
        if m: traces.append(m)
        if w: traces.append(w)

    total_vol = sum(aabb_volume(a) for a in aabbs if a.get("frame") not in skip_frames)
    title = (f"<b>Panda AABB -- {method_name}</b>"
             f"   |   Total vol: {total_vol:.4f} m^3"
             f"   |   Time: {t_comp*1000:.1f} ms")
    fig = go.Figure(data=traces)
    fig.update_layout(
        title=dict(text=title, font=dict(size=14)),
        scene=dict(
            xaxis=dict(title="X (m)", range=[-0.8, 0.8]),
            yaxis=dict(title="Y (m)", range=[-0.8, 0.8]),
            zaxis=dict(title="Z (m)", range=[-0.05, 1.1]),
            aspectmode="cube",
            camera=dict(eye=dict(x=1.6, y=-1.4, z=1.0)),
        ),
        width=950, height=720,
        margin=dict(l=0, r=0, t=55, b=0),
        paper_bgcolor="white",
        legend=dict(x=0.01, y=0.99, bgcolor="rgba(255,255,255,0.88)",
                    bordercolor="#ccc", borderwidth=1, font=dict(size=10)),
    )
    return fig


def build_3d_comparison_figure(methods_data: list,
                                arm_samples_pos: list,
                                mid_pos: np.ndarray,
                                obstacles: list,
                                link_colors: list = None,
                                skip_frames: set = None):
    """
    Build a 1×3 subplot Plotly figure comparing three AABB methods.

    Parameters
    ----------
    methods_data    : list of (name, aabbs, color, t_comp) tuples
    arm_samples_pos : list of (n_frames,3) position arrays for ghost arms
    mid_pos         : (n_frames, 3) midpoint arm positions
    obstacles       : list of obstacle dicts
    link_colors     : per-link hex colors
    skip_frames     : set of frame indices to omit

    Returns
    -------
    plotly.graph_objects.Figure
    """
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError:
        raise ImportError("plotly is required.")
    if link_colors is None:
        link_colors = _LINK_COLORS_DEFAULT
    if skip_frames is None:
        skip_frames = set()

    fig = make_subplots(
        rows=1, cols=3,
        specs=[[{"type": "scene"}]*3],
        subplot_titles=[m[0] for m in methods_data],
        horizontal_spacing=0.02,
    )
    mid_pos = np.atleast_2d(mid_pos)
    scene_base = dict(
        xaxis=dict(title="X", range=[-0.8, 0.8]),
        yaxis=dict(title="Y", range=[-0.8, 0.8]),
        zaxis=dict(title="Z", range=[-0.05, 1.1]),
        aspectmode="cube",
        camera=dict(eye=dict(x=1.6, y=-1.4, z=1.0)),
    )

    for col, (name, aabbs, method_color, t_comp) in enumerate(methods_data):
        c1 = col + 1
        first = (col == 0)

        for obs in obstacles:
            c = np.asarray(obs["center"]); h = np.asarray(obs["half"])
            oa = {"x_min":c[0]-h[0],"x_max":c[0]+h[0],
                  "y_min":c[1]-h[1],"y_max":c[1]+h[1],
                  "z_min":c[2]-h[2],"z_max":c[2]+h[2]}
            m = plotly_aabb_mesh(oa, color=obs["color"], opacity=0.40,
                                  name=f"Obs:{obs['name']}", showlegend=first)
            if m: fig.add_trace(m, row=1, col=c1)

        for k, pos in enumerate(arm_samples_pos):
            pos = np.atleast_2d(pos)
            t = plotly_arm_trace(pos, color="#cccccc", width=1.0, opacity=0.07,
                                  name="Samples" if (k == 0 and first) else "",
                                  showlegend=(k == 0 and first))
            fig.add_trace(t, row=1, col=c1)

        for j in range(len(mid_pos) - 1):
            c = link_colors[j % len(link_colors)]
            fig.add_trace(go.Scatter3d(
                x=[mid_pos[j,0], mid_pos[j+1,0]],
                y=[mid_pos[j,1], mid_pos[j+1,1]],
                z=[mid_pos[j,2], mid_pos[j+1,2]],
                mode="lines+markers",
                line=dict(color=c, width=5), marker=dict(size=4, color=c),
                name="Mid arm" if (j==0 and first) else "",
                showlegend=(j==0 and first),
            ), row=1, col=c1)

        for aabb in aabbs:
            if aabb.get("frame") in skip_frames:
                continue
            lc = link_colors[(aabb["frame"] - 1) % len(link_colors)]
            m = plotly_aabb_mesh(aabb, color=lc, opacity=0.15,
                                  name=aabb.get("label",""), showlegend=False)
            w = plotly_aabb_wireframe(aabb, color=method_color, width=1.8)
            if m: fig.add_trace(m, row=1, col=c1)
            if w: fig.add_trace(w, row=1, col=c1)

        sk = "scene" if col == 0 else f"scene{col+1}"
        fig.update_layout(**{sk: scene_base})

    for col, (name, aabbs, _, t_comp) in enumerate(methods_data):
        vol = sum(aabb_volume(a) for a in aabbs if a.get("frame") not in skip_frames)
        fig.layout.annotations[col].update(
            text=f"<b>{name}</b><br>Vol:{vol:.4f} m^3 | {t_comp*1000:.1f}ms",
            font=dict(size=11),
        )

    fig.update_layout(
        title=dict(text="<b>Panda 7-DOF -- AABB Envelope Method Comparison</b>",
                   font=dict(size=15), x=0.5),
        width=1850, height=760,
        margin=dict(l=0, r=0, t=85, b=0),
        paper_bgcolor="white",
        legend=dict(x=0.0, y=1.02, bgcolor="rgba(255,255,255,0.9)",
                    bordercolor="#ccc", borderwidth=1),
    )
    return fig


def build_3d_obb_figure(method_name: str, obbs: list,
                         arm_samples_pos: list, mid_pos: np.ndarray,
                         obstacles: list, method_color: str,
                         t_comp: float = 0.0,
                         link_colors: list = None,
                         skip_frames: set = None):
    """
    Build a standalone interactive Plotly figure for the OBB envelope.

    Parameters
    ----------
    method_name     : display name
    obbs            : list of OBB dicts (from compute_obbs_interval_fk)
    arm_samples_pos : list of (n_frames, 3) ghost arm position arrays
    mid_pos         : (n_frames, 3) midpoint arm positions
    obstacles       : list of obstacle dicts
    method_color    : hex color for OBB wireframes
    t_comp          : computation time (s)
    link_colors     : per-link hex colors
    skip_frames     : set of frame indices to omit
    """
    try:
        import plotly.graph_objects as go
    except ImportError:
        raise ImportError("plotly is required.")
    if link_colors is None:
        link_colors = _LINK_COLORS_DEFAULT
    if skip_frames is None:
        skip_frames = set()

    traces = []
    traces += plotly_obstacle_traces(obstacles)

    for k, pos in enumerate(arm_samples_pos):
        traces.append(plotly_arm_trace(
            pos, color="#aaaaaa", width=1.2, opacity=0.10,
            name="Sampled arms" if k == 0 else "", showlegend=(k == 0)))

    mid_pos = np.atleast_2d(mid_pos)
    for j in range(len(mid_pos) - 1):
        c = link_colors[j % len(link_colors)]
        traces.append(go.Scatter3d(
            x=[mid_pos[j, 0], mid_pos[j+1, 0]],
            y=[mid_pos[j, 1], mid_pos[j+1, 1]],
            z=[mid_pos[j, 2], mid_pos[j+1, 2]],
            mode="lines+markers",
            line=dict(color=c, width=8),
            marker=dict(size=5, color=c),
            name=f"Link {j+1}" if j < 7 else "Tool link",
            showlegend=(j == 0),
        ))

    for obb in obbs:
        if obb.get("frame") in skip_frames:
            continue
        lc = link_colors[(obb["frame"] - 1) % len(link_colors)]
        vol = obb_volume(obb)
        label = obb.get("label", f"Frame {obb['frame']}")
        m = plotly_obb_mesh(obb, color=lc, opacity=0.13,
                             name=f"{label} ({vol:.4f} m^3)", showlegend=True)
        w = plotly_obb_wireframe(obb, color=method_color, width=1.8)
        if m: traces.append(m)
        if w: traces.append(w)

    total_vol = sum(obb_volume(o) for o in obbs if o.get("frame") not in skip_frames)
    title = (f"<b>Panda OBB -- {method_name}</b>"
             f"   |   Total vol: {total_vol:.4f} m^3"
             f"   |   Time: {t_comp*1000:.1f} ms")
    fig = go.Figure(data=traces)
    fig.update_layout(
        title=dict(text=title, font=dict(size=14)),
        scene=dict(
            xaxis=dict(title="X (m)", range=[-0.8, 0.8]),
            yaxis=dict(title="Y (m)", range=[-0.8, 0.8]),
            zaxis=dict(title="Z (m)", range=[-0.05, 1.1]),
            aspectmode="cube",
            camera=dict(eye=dict(x=1.6, y=-1.4, z=1.0)),
        ),
        width=950, height=720,
        margin=dict(l=0, r=0, t=55, b=0),
        paper_bgcolor="white",
        legend=dict(x=0.01, y=0.99, bgcolor="rgba(255,255,255,0.88)",
                    bordercolor="#ccc", borderwidth=1, font=dict(size=10)),
    )
    return fig


def build_3d_obb_aabb_comparison_figure(panels: list,
                                          arm_samples_pos: list,
                                          mid_pos: np.ndarray,
                                          obstacles: list,
                                          link_colors: list = None,
                                          skip_frames: set = None,
                                          title: str = None):
    """
    Build a multi-panel Plotly figure comparing OBB and AABB envelopes side-by-side.

    Each panel entry is a dict::

        {
            "name":      str,           # subplot title
            "envelopes": list,          # aabb dicts OR obb dicts
            "color":     str,           # wireframe hex color
            "t_comp":    float,         # computation time (s)
            "kind":      "aabb"|"obb",  # envelope type (detected automatically if absent)
        }

    AABB dicts have keys ``x_min/x_max/y_min/y_max/z_min/z_max``.
    OBB dicts have keys ``center/half/axes``.

    Parameters
    ----------
    panels          : list of panel dicts (see above)
    arm_samples_pos : list of (n_frames, 3) ghost arm position arrays
    mid_pos         : (n_frames, 3) midpoint arm positions
    obstacles       : list of obstacle dicts
    link_colors     : per-link hex colors
    skip_frames     : set of frame indices to omit
    title           : figure-level title (default auto-generated)

    Returns
    -------
    plotly.graph_objects.Figure
    """
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError:
        raise ImportError("plotly is required.")
    if link_colors is None:
        link_colors = _LINK_COLORS_DEFAULT
    if skip_frames is None:
        skip_frames = set()

    n_panels = len(panels)
    fig = make_subplots(
        rows=1, cols=n_panels,
        specs=[[{"type": "scene"}] * n_panels],
        subplot_titles=[p["name"] for p in panels],
        horizontal_spacing=0.02,
    )
    mid_pos = np.atleast_2d(mid_pos)
    scene_base = dict(
        xaxis=dict(title="X", range=[-0.8, 0.8]),
        yaxis=dict(title="Y", range=[-0.8, 0.8]),
        zaxis=dict(title="Z", range=[-0.05, 1.1]),
        aspectmode="cube",
        camera=dict(eye=dict(x=1.6, y=-1.4, z=1.0)),
    )

    for col, panel in enumerate(panels):
        c1   = col + 1
        first = (col == 0)
        envelopes = panel["envelopes"]
        method_color = panel.get("color", "#555555")

        # Detect envelope type
        if panel.get("kind") == "aabb" or (envelopes and "x_min" in envelopes[0]):
            kind = "aabb"
        else:
            kind = "obb"

        # Obstacles
        for obs in obstacles:
            c = np.asarray(obs["center"]); h = np.asarray(obs["half"])
            oa = {"x_min": c[0]-h[0], "x_max": c[0]+h[0],
                  "y_min": c[1]-h[1], "y_max": c[1]+h[1],
                  "z_min": c[2]-h[2], "z_max": c[2]+h[2]}
            m = plotly_aabb_mesh(oa, color=obs["color"], opacity=0.40,
                                  name=f"Obs:{obs['name']}", showlegend=first)
            if m: fig.add_trace(m, row=1, col=c1)

        # Ghost arms
        for k, pos in enumerate(arm_samples_pos):
            pos = np.atleast_2d(pos)
            t = plotly_arm_trace(pos, color="#cccccc", width=1.0, opacity=0.07,
                                  name="Samples" if (k == 0 and first) else "",
                                  showlegend=(k == 0 and first))
            fig.add_trace(t, row=1, col=c1)

        # Mid arm
        for j in range(len(mid_pos) - 1):
            c = link_colors[j % len(link_colors)]
            fig.add_trace(go.Scatter3d(
                x=[mid_pos[j, 0], mid_pos[j+1, 0]],
                y=[mid_pos[j, 1], mid_pos[j+1, 1]],
                z=[mid_pos[j, 2], mid_pos[j+1, 2]],
                mode="lines+markers",
                line=dict(color=c, width=5), marker=dict(size=4, color=c),
                name="Mid arm" if (j == 0 and first) else "",
                showlegend=(j == 0 and first),
            ), row=1, col=c1)

        # Envelopes
        if kind == "aabb":
            for env in envelopes:
                if env.get("frame") in skip_frames:
                    continue
                lc = link_colors[(env["frame"] - 1) % len(link_colors)]
                m = plotly_aabb_mesh(env, color=lc, opacity=0.15,
                                      name=env.get("label", ""), showlegend=False)
                w = plotly_aabb_wireframe(env, color=method_color, width=1.8)
                if m: fig.add_trace(m, row=1, col=c1)
                if w: fig.add_trace(w, row=1, col=c1)
        else:  # obb
            for env in envelopes:
                if env.get("frame") in skip_frames:
                    continue
                lc = link_colors[(env["frame"] - 1) % len(link_colors)]
                m = plotly_obb_mesh(env, color=lc, opacity=0.15,
                                     name=env.get("label", ""), showlegend=False)
                w = plotly_obb_wireframe(env, color=method_color, width=1.8)
                if m: fig.add_trace(m, row=1, col=c1)
                if w: fig.add_trace(w, row=1, col=c1)

        sk = "scene" if col == 0 else f"scene{col + 1}"
        fig.update_layout(**{sk: scene_base})

    # Update subplot title annotations with volume + time
    for col, panel in enumerate(panels):
        envelopes = panel["envelopes"]
        t_comp = panel.get("t_comp", 0.0)
        if panel.get("kind") == "aabb" or (envelopes and "x_min" in envelopes[0]):
            vol = sum(aabb_volume(e) for e in envelopes if e.get("frame") not in skip_frames)
        else:
            vol = sum(obb_volume(e)  for e in envelopes if e.get("frame") not in skip_frames)
        fig.layout.annotations[col].update(
            text=f"<b>{panel['name']}</b><br>Vol:{vol:.4f} m^3 | {t_comp*1000:.1f}ms",
            font=dict(size=11),
        )

    w_map = {1: 950, 2: 1250, 3: 1850, 4: 2400}
    fig.update_layout(
        title=dict(
            text=title or "<b>Panda 7-DOF — OBB vs AABB Envelope Comparison</b>",
            font=dict(size=15), x=0.5,
        ),
        width=w_map.get(n_panels, 1850), height=760,
        margin=dict(l=0, r=0, t=85, b=0),
        paper_bgcolor="white",
        legend=dict(x=0.0, y=1.02, bgcolor="rgba(255,255,255,0.9)",
                    bordercolor="#ccc", borderwidth=1),
    )
    return fig

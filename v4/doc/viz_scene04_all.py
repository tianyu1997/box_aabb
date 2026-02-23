"""
viz_scene04_all.py — 场景4统一可视化: 6 合 1

生成内容:
  1. img_s4_aabb_envelope.png    — Interval FK → AABB 包络示例
  2. img_s4_forest_growth.gif    — Box forest 生长过程动画
  3. img_s4_coarsen.png          — Coarsen 前后对比
  4. img_s4_incremental.png      — 障碍物增加时 box forest 变化
  5. img_s4_cspace_path.png      — C-space 路径规划结果
  6. img_s4_workspace_path.gif   — 工作空间执行路径动画

用法:
    cd v3/doc
    python viz_scene04_all.py
"""
from __future__ import annotations

import heapq
import json
import math
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Rectangle, FancyBboxPatch
from matplotlib.lines import Line2D
import matplotlib.patches as mpatches
import matplotlib.colors as mcolors
import numpy as np

# ── v3 path setup ──────────────────────────────────────────────────────────
_ROOT = Path(__file__).resolve().parents[1]  # v3/
_SRC = _ROOT / "src"
for p in (_ROOT, _SRC):
    sp = str(p)
    if sp not in sys.path:
        sys.path.insert(0, sp)

from aabb.robot import load_robot
from aabb.interval_fk import compute_interval_aabb
from forest.scene import Scene
from forest.collision import CollisionChecker
from forest.models import BoxNode, Obstacle
from forest.connectivity import find_islands, bridge_islands, UnionFind
from forest.safe_box_forest import SafeBoxForest
from planner.defaults import TWODOF_DEFAULTS
from planner.pipeline import (
    grow_and_prepare, _build_adjacency_and_islands, find_box_containing,
    _add_bridge_to_adj,
)
from forest.coarsen import coarsen_forest

# ═══════════════════════════════════════════════════════════════════════════
#  Scene 04 config (from scene_candidates/scene_04.json)
# ═══════════════════════════════════════════════════════════════════════════

_SCENE_JSON = Path(__file__).parent / "scene_candidates" / "scene_04.json"

def _load_scene04():
    with open(_SCENE_JSON, encoding="utf-8") as f:
        return json.load(f)

CFG = _load_scene04()

INITIAL_OBSTACLES = CFG["initial_obstacles"]
NEW_OBSTACLE      = CFG["new_obstacle"]
Q_START           = np.asarray(CFG["q_start"], dtype=np.float64)
Q_GOAL            = np.asarray(CFG["q_goal"],  dtype=np.float64)
L1, L2            = 1.0, 1.0
DPI               = 180
SEED              = 42
OUT_DIR           = Path(__file__).parent / "scene4_assets"


# ═══════════════════════════════════════════════════════════════════════════
#  Shared helpers
# ═══════════════════════════════════════════════════════════════════════════

def build_scene(obstacles):
    scene = Scene()
    for o in obstacles:
        scene.add_obstacle(o["min"], o["max"], name=o["name"])
    return scene


def fk_2dof(q1: float, q2: float):
    x1 = L1 * math.cos(q1)
    y1 = L1 * math.sin(q1)
    x2 = x1 + L2 * math.cos(q1 + q2)
    y2 = y1 + L2 * math.sin(q1 + q2)
    return np.array([[0.0, 0.0], [x1, y1], [x2, y2]], dtype=np.float64)


def scan_collision_map(robot, scene, resolution=0.04):
    checker = CollisionChecker(robot=robot, scene=scene)
    xs = np.arange(-np.pi, np.pi, resolution)
    ys = np.arange(-np.pi, np.pi, resolution)
    cmap = np.zeros((len(ys), len(xs)), dtype=np.float32)
    for i, y in enumerate(ys):
        row = np.column_stack([xs, np.full(len(xs), y)])
        cmap[i, :] = checker.check_config_collision_batch(row).astype(np.float32)
    return cmap, [-np.pi, np.pi, -np.pi, np.pi]


def snapshot_boxes(forest_obj):
    return {bid: BoxNode(
        node_id=b.node_id,
        joint_intervals=[tuple(iv) for iv in b.joint_intervals],
        seed_config=b.seed_config.copy() if b.seed_config is not None else None,
        volume=b.volume)
        for bid, b in forest_obj.boxes.items()}


def draw_collision_bg(ax, cmap_data, extent, alpha=0.25):
    ax.imshow(cmap_data, origin="lower", extent=extent,
              cmap="Reds", alpha=alpha, aspect="auto")


def draw_boxes(ax, boxes, alpha=0.30, highlight_ids=None,
               highlight_color='#ff4444', highlight_alpha=0.6, cmap_name='viridis'):
    highlight_ids = highlight_ids or set()
    cmap_c = plt.colormaps.get_cmap(cmap_name)
    vols = [b.volume for b in boxes.values()] if boxes else [0, 1]
    norm = mcolors.Normalize(vmin=min(vols), vmax=max(vols))
    for bid, box in boxes.items():
        lo_x, hi_x = box.joint_intervals[0]
        lo_y, hi_y = box.joint_intervals[1]
        if bid in highlight_ids:
            ec, fc, a, lw = highlight_color, highlight_color, highlight_alpha, 1.8
        else:
            c = cmap_c(norm(box.volume))
            ec, fc, a, lw = c, c, alpha, 0.5
        rect = Rectangle((lo_x, lo_y), hi_x - lo_x, hi_y - lo_y,
                          linewidth=lw, edgecolor=ec, facecolor=fc, alpha=a)
        ax.add_patch(rect)


def setup_cspace_ax(ax, title=""):
    ax.set_xlim(-np.pi, np.pi)
    ax.set_ylim(-np.pi, np.pi)
    ax.set_aspect('equal')
    ax.set_title(title, fontsize=11, fontweight='bold', pad=8)
    ax.set_xlabel("θ₁")
    ax.set_ylabel("θ₂")
    ax.grid(True, alpha=0.1)


def setup_workspace_ax(ax, title=""):
    ax.set_title(title, fontsize=11, fontweight='bold', pad=8)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_aspect('equal')
    ax.set_xlim(-2.3, 2.3)
    ax.set_ylim(-2.3, 2.3)
    ax.grid(True, alpha=0.12)


def draw_workspace_obstacles(ax, obstacles, new_obstacle=None):
    for obs in obstacles:
        w = obs['max'][0] - obs['min'][0]
        h = obs['max'][1] - obs['min'][1]
        ax.add_patch(Rectangle((obs['min'][0], obs['min'][1]), w, h,
                               facecolor='#9e9e9e', edgecolor='#616161',
                               alpha=0.3, linewidth=1.0))
    if new_obstacle is not None:
        w = new_obstacle['max'][0] - new_obstacle['min'][0]
        h = new_obstacle['max'][1] - new_obstacle['min'][1]
        ax.add_patch(Rectangle((new_obstacle['min'][0], new_obstacle['min'][1]),
                               w, h, facecolor='#ef5350', edgecolor='#c62828',
                               alpha=0.55, linewidth=1.2))


def draw_arm(ax, q1, q2, color='cyan', lw=2.2, ms=4, zorder=15, marker='o',
             marker_ms=None, label=None):
    pts = fk_2dof(q1, q2)
    kw = dict(linewidth=lw, markersize=ms, markeredgecolor='black', zorder=zorder)
    if label:
        kw['label'] = label
    ax.plot(pts[:, 0], pts[:, 1], f'-{marker}', color=color, **kw)
    if marker_ms:
        ax.plot(pts[-1, 0], pts[-1, 1], marker, color=color, markersize=marker_ms,
                markeredgecolor='black', zorder=zorder + 1)


def make_forest_cfg(seed_val, max_boxes_val=250, coarsen_val=0):
    """Light config dataclass mimic for grow_and_prepare."""
    class _Cfg:
        pass
    c = _Cfg()
    c.seed = seed_val
    c.max_consecutive_miss = 300
    c.max_boxes = max_boxes_val
    c.ffb_min_edge = TWODOF_DEFAULTS.ffb_min_edge
    c.guided_sample_ratio = TWODOF_DEFAULTS.guided_sample_ratio
    c.boundary_expand_epsilon = 0.01
    c.parallel_grow = False
    c.n_partitions_depth = 0
    c.parallel_workers = 1
    c.coarsen_max_rounds = coarsen_val
    return c


def sample_from_boxes(rng, boxes, n_samples: int):
    if not boxes:
        return np.zeros((0, 2), dtype=np.float64)
    keys = list(boxes.keys())
    sampled = []
    for _ in range(n_samples):
        bid = keys[rng.integers(0, len(keys))]
        b = boxes[bid]
        q1 = rng.uniform(b.joint_intervals[0][0], b.joint_intervals[0][1])
        q2 = rng.uniform(b.joint_intervals[1][0], b.joint_intervals[1][1])
        sampled.append((q1, q2))
    return np.asarray(sampled, dtype=np.float64)


# ═══════════════════════════════════════════════════════════════════════════
#  Geodesic helpers
# ═══════════════════════════════════════════════════════════════════════════

# 必须使用 robot joint_limits 导出 period, 与 box 边界精度一致
# (json 中 joint_limits = [-3.14159, 3.14159], 不是 np.pi)
_robot_tmp = load_robot("2dof_planar")
PERIOD = float(_robot_tmp.joint_limits[0][1] - _robot_tmp.joint_limits[0][0])
del _robot_tmp

def geo_diff(a, b):
    half = PERIOD / 2.0
    return ((b - a) + half) % PERIOD - half

def geo_dist(a, b):
    return float(np.linalg.norm(geo_diff(a, b)))


# ═══════════════════════════════════════════════════════════════════════════
#  Dijkstra on box adjacency graph (from viz_2dof_full_pipeline.py)
# ═══════════════════════════════════════════════════════════════════════════

def dijkstra_box_graph(boxes, adj, src, tgt):
    half = PERIOD / 2.0
    centers = {}
    box_lo, box_hi = {}, {}
    for bid, box in boxes.items():
        lo = np.array([lo_d for lo_d, hi_d in box.joint_intervals])
        hi = np.array([hi_d for lo_d, hi_d in box.joint_intervals])
        box_lo[bid] = lo
        box_hi[bid] = hi
        centers[bid] = (lo + hi) * 0.5

    def _edge_weight(u, v):
        lo_u, hi_u = box_lo[u], box_hi[u]
        lo_v, hi_v = box_lo[v], box_hi[v]
        ndim = len(lo_u)
        gap_sq = 0.0
        for d in range(ndim):
            if lo_u[d] <= hi_v[d] + 1e-9 and lo_v[d] <= hi_u[d] + 1e-9:
                continue
            g1 = (lo_v[d] - hi_u[d]) % PERIOD
            g2 = (lo_u[d] - hi_v[d]) % PERIOD
            g = min(g1, g2)
            if g > half:
                g = PERIOD - g
            gap_sq += g * g
        surface_dist = math.sqrt(gap_sq)
        if surface_dist > 1e-10:
            return surface_dist
        diff = ((centers[v] - centers[u]) + half) % PERIOD - half
        return max(0.3 * float(np.linalg.norm(diff)), 1e-12)

    dist_map = {bid: float('inf') for bid in boxes}
    prev_map = {bid: None for bid in boxes}
    dist_map[src] = 0.0
    heap = [(0.0, src)]
    while heap:
        d, u = heapq.heappop(heap)
        if d > dist_map[u]:
            continue
        if u == tgt:
            break
        for v in adj.get(u, set()):
            w = _edge_weight(u, v)
            nd = d + w
            if nd < dist_map[v]:
                dist_map[v] = nd
                prev_map[v] = u
                heapq.heappush(heap, (nd, v))
    if dist_map[tgt] == float('inf'):
        return None, float('inf')
    seq = []
    cur = tgt
    while cur is not None:
        seq.append(cur)
        cur = prev_map[cur]
    seq.reverse()
    return seq, dist_map[tgt]


def overlap_bounds_periodic(box_a, box_b):
    half = PERIOD / 2.0
    ndim = len(box_a.joint_intervals)
    lo_out = np.empty(ndim)
    hi_out = np.empty(ndim)
    eps = 1e-9
    for d, ((lo1, hi1), (lo2, hi2)) in enumerate(
            zip(box_a.joint_intervals, box_b.joint_intervals)):
        best_lo, best_hi, best_width = lo1, hi1, -1.0
        for offset in [0.0, PERIOD, -PERIOD]:
            ov_lo = max(lo1, lo2 + offset)
            ov_hi = min(hi1, hi2 + offset)
            if ov_hi >= ov_lo - eps:
                w = max(0.0, ov_hi - ov_lo)
                if w > best_width:
                    best_lo, best_hi = ov_lo, max(ov_lo, ov_hi)
                    best_width = w
        if best_width < 0:
            best_lo = best_hi = (hi1 + lo2) / 2.0
        lo_out[d] = best_lo
        hi_out[d] = best_hi
    return lo_out, hi_out


def _shortcut_box_seq(box_seq, adj):
    if len(box_seq) <= 2:
        return box_seq
    result = [box_seq[0]]
    i = 0
    n = len(box_seq)
    while i < n - 1:
        farthest = i + 1
        nbrs = adj.get(box_seq[i], set())
        for j in range(n - 1, i + 1, -1):
            if box_seq[j] in nbrs:
                farthest = j
                break
        result.append(box_seq[farthest])
        i = farthest
    return result


def build_waypoints(q_start, q_goal, box_seq, boxes):
    half = PERIOD / 2.0
    waypoints = [q_start.copy()]
    for k in range(len(box_seq) - 1):
        ov_lo, ov_hi = overlap_bounds_periodic(boxes[box_seq[k]], boxes[box_seq[k + 1]])
        transition = ((ov_lo + ov_hi) / 2.0 + half) % PERIOD - half
        waypoints.append(transition)
    waypoints.append(q_goal.copy())
    return waypoints


def pull_tight(waypoints, box_seq, boxes, n_iters=30):
    if len(waypoints) <= 2:
        return waypoints[:]
    half = PERIOD / 2.0
    # Build bounds for each waypoint
    wp_bounds = []
    box0 = boxes[box_seq[0]]
    wp_bounds.append((np.array([lo for lo, hi in box0.joint_intervals]),
                      np.array([hi for lo, hi in box0.joint_intervals])))
    for k in range(len(box_seq) - 1):
        ov_lo, ov_hi = overlap_bounds_periodic(boxes[box_seq[k]], boxes[box_seq[k + 1]])
        wp_bounds.append((ov_lo, ov_hi))
    box_last = boxes[box_seq[-1]]
    wp_bounds.append((np.array([lo for lo, hi in box_last.joint_intervals]),
                      np.array([hi for lo, hi in box_last.joint_intervals])))

    wps = [w.copy() for w in waypoints]
    for _it in range(n_iters):
        max_move = 0.0
        for i in range(1, len(wps) - 1):
            prev, nxt = wps[i - 1], wps[i + 1]
            diff = ((nxt - prev) + half) % PERIOD - half
            seg_len_sq = float(np.dot(diff, diff))
            if seg_len_sq < 1e-20:
                target = prev.copy()
            else:
                d_cur = ((wps[i] - prev) + half) % PERIOD - half
                t = float(np.dot(d_cur, diff)) / seg_len_sq
                t = np.clip(t, 0.0, 1.0)
                target = prev + t * diff
                target = ((target + half) % PERIOD) - half
            lo_b, hi_b = wp_bounds[i]
            for d in range(len(target)):
                target[d] = np.clip(target[d], lo_b[d], hi_b[d])
            move = float(np.linalg.norm(target - wps[i]))
            if move > max_move:
                max_move = move
            wps[i] = target
        if max_move < 1e-8:
            break
    return wps


def resample_path(waypoints, ds=0.05):
    """Resample path to approximately uniform spacing ds."""
    half = PERIOD / 2.0
    pts = [waypoints[0].copy()]
    for i in range(len(waypoints) - 1):
        diff = ((waypoints[i + 1] - waypoints[i]) + half) % PERIOD - half
        seg_len = float(np.linalg.norm(diff))
        n_seg = max(1, int(math.ceil(seg_len / ds)))
        for k in range(1, n_seg + 1):
            t = k / n_seg
            p = waypoints[i] + t * diff
            p = ((p + half) % PERIOD) - half
            pts.append(p)
    return pts


def draw_wrapped_line(ax, p1, p2, **kwargs):
    """Draw a line segment, handling periodic wrap if needed."""
    half = PERIOD / 2.0
    diff = ((p2 - p1) + half) % PERIOD - half
    dist = float(np.linalg.norm(diff))
    naive_dist = float(np.linalg.norm(p2 - p1))
    if abs(dist - naive_dist) < 0.1:
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], **kwargs)
    else:
        # wrap: draw two segments
        n_pts = max(20, int(dist / 0.05))
        ts = np.linspace(0, 1, n_pts)
        xs, ys = [], []
        for t in ts:
            p = p1 + t * diff
            p = ((p + half) % PERIOD) - half
            xs.append(p[0])
            ys.append(p[1])
        # Split at discontinuities
        segments_x, segments_y = [[]], [[]]
        for k in range(len(xs)):
            if k > 0 and (abs(xs[k] - xs[k-1]) > 3.0 or abs(ys[k] - ys[k-1]) > 3.0):
                segments_x.append([])
                segments_y.append([])
            segments_x[-1].append(xs[k])
            segments_y[-1].append(ys[k])
        first = True
        for sx, sy in zip(segments_x, segments_y):
            kw = dict(kwargs)
            if not first:
                kw.pop('label', None)
            ax.plot(sx, sy, **kw)
            first = False


def draw_cspace_path(ax, waypoints, color='#00ff00', lw=2.5, label='Path', zorder=15):
    xs = [w[0] for w in waypoints]
    ys = [w[1] for w in waypoints]
    ax.plot(xs, ys, 'o', color=color, markersize=3.5,
            markeredgecolor='black', markeredgewidth=0.4, zorder=zorder + 1)
    first = True
    for i in range(len(waypoints) - 1):
        kw = dict(color=color, linewidth=lw, alpha=0.9, zorder=zorder)
        if first:
            kw['label'] = label
            first = False
        draw_wrapped_line(ax, waypoints[i], waypoints[i + 1], **kw)


# ═══════════════════════════════════════════════════════════════════════════
#  AABB envelope helpers
# ═══════════════════════════════════════════════════════════════════════════

def _overlap_1d(a_lo, a_hi, b_lo, b_hi):
    return max(a_lo, b_lo) <= min(a_hi, b_hi)


def _aabb_hits_obstacles_xy(link_aabbs, obstacles):
    for la in link_aabbs:
        ax0, ay0 = float(la.min_point[0]), float(la.min_point[1])
        ax1, ay1 = float(la.max_point[0]), float(la.max_point[1])
        for obs in obstacles:
            ox0, oy0 = float(obs["min"][0]), float(obs["min"][1])
            ox1, oy1 = float(obs["max"][0]), float(obs["max"][1])
            if _overlap_1d(ax0, ax1, ox0, ox1) and _overlap_1d(ay0, ay1, oy0, oy1):
                return True
    return False


def find_collision_free_interval(robot, checker, obstacles, rng):
    candidates = []
    for _ in range(600):
        c1 = float(rng.uniform(-np.pi + 0.45, np.pi - 0.45))
        c2 = float(rng.uniform(-np.pi + 0.45, np.pi - 0.45))
        w1 = float(rng.uniform(0.25, 0.55))
        w2 = float(rng.uniform(0.25, 0.55))
        iv = [
            (max(-np.pi, c1 - w1 / 2), min(np.pi, c1 + w1 / 2)),
            (max(-np.pi, c2 - w2 / 2), min(np.pi, c2 + w2 / 2)),
        ]
        if iv[0][1] - iv[0][0] < 0.15 or iv[1][1] - iv[1][0] < 0.15:
            continue
        candidates.append(iv)

    best = None
    best_score = float("inf")
    for iv in candidates:
        if checker.check_box_collision(iv):
            continue
        link_aabbs, _ = compute_interval_aabb(robot, iv, set(), skip_zero_length=True)
        if _aabb_hits_obstacles_xy(link_aabbs, obstacles):
            continue
        s1 = iv[0][1] - iv[0][0]
        s2 = iv[1][1] - iv[1][0]
        compact_penalty = max(0.0, 0.45 - min(s1, s2))
        score = compact_penalty
        if score < best_score:
            best_score = score
            best = (iv, link_aabbs)

    if best is not None:
        return best
    raise RuntimeError("未找到满足条件的无碰撞 joint interval AABB")


# ═══════════════════════════════════════════════════════════════════════════
#  1. AABB Envelope Visualization
# ═══════════════════════════════════════════════════════════════════════════

def gen_aabb_envelope():
    print("\n[1/6] AABB Envelope (Interval FK → AABB) ...")
    rng = np.random.default_rng(SEED + 1)
    robot = load_robot("2dof_planar")
    scene = build_scene(INITIAL_OBSTACLES)
    checker = CollisionChecker(robot=robot, scene=scene)

    q_interval, link_aabbs = find_collision_free_interval(
        robot, checker, INITIAL_OBSTACLES, rng)
    print(f"  Selected interval: {q_interval}")

    N_SAMPLES = 80
    q1_lo, q1_hi = q_interval[0]
    q2_lo, q2_hi = q_interval[1]
    q1_samples = rng.uniform(q1_lo, q1_hi, N_SAMPLES)
    q2_samples = rng.uniform(q2_lo, q2_hi, N_SAMPLES)
    q1_mid = (q1_lo + q1_hi) / 2
    q2_mid = (q2_lo + q2_hi) / 2

    cmap, extent = scan_collision_map(robot, scene, resolution=0.04)

    fig, (ax_cs, ax_ws) = plt.subplots(1, 2, figsize=(13, 5.5),
                                       gridspec_kw={'width_ratios': [1.2, 1]})
    fig.patch.set_facecolor('white')

    # ── Left: C-space ──
    ax_cs.set_title("C-space: Collision-Free Joint Interval", fontsize=12,
                    fontweight='bold', pad=10)
    setup_cspace_ax(ax_cs)
    draw_collision_bg(ax_cs, cmap, extent, alpha=0.24)

    rect_q = Rectangle((q1_lo, q2_lo), q1_hi - q1_lo, q2_hi - q2_lo,
                        facecolor='#4CAF50', alpha=0.25,
                        edgecolor='#4CAF50', linewidth=2.5,
                        label='Safe Box Q')
    ax_cs.add_patch(rect_q)
    ax_cs.scatter(q1_samples, q2_samples, s=8, c='#2196F3', alpha=0.4,
                  zorder=5, label='Sampled configs')
    ax_cs.plot(q1_mid, q2_mid, '*', color='#FF9800', markersize=14,
               markeredgecolor='black', markeredgewidth=0.8, zorder=10,
               label='Midpoint')
    ax_cs.legend(loc='lower right', fontsize=8, framealpha=0.9)

    # ── Right: Workspace ──
    ax_ws.set_title("Workspace: FK Samples + Link AABBs", fontsize=12,
                    fontweight='bold', pad=10)
    setup_workspace_ax(ax_ws)
    draw_workspace_obstacles(ax_ws, INITIAL_OBSTACLES)

    link_colors = ['#2196F3', '#FF9800']
    for i in range(N_SAMPLES):
        pts = fk_2dof(q1_samples[i], q2_samples[i])
        for j in range(2):
            ax_ws.plot([pts[j, 0], pts[j+1, 0]], [pts[j, 1], pts[j+1, 1]],
                       color=link_colors[j], alpha=0.10, linewidth=0.9)

    pts_mid = fk_2dof(q1_mid, q2_mid)
    for j in range(2):
        ax_ws.plot([pts_mid[j, 0], pts_mid[j+1, 0]],
                   [pts_mid[j, 1], pts_mid[j+1, 1]],
                   color=link_colors[j], alpha=0.92, linewidth=3.0,
                   solid_capstyle='round')
    ax_ws.plot(0, 0, 'ko', markersize=8, zorder=10)
    ax_ws.plot(pts_mid[1, 0], pts_mid[1, 1], 'o', color='#2196F3',
               markersize=6, zorder=10)
    ax_ws.plot(pts_mid[2, 0], pts_mid[2, 1], '*', color='#4CAF50',
               markersize=12, zorder=10)

    aabb_colors = ['#2196F3', '#FF9800', '#4CAF50']
    aabb_hatches = ['///', '\\\\\\', '...']
    aabb_labels = ['Link 1 AABB', 'Link 2 AABB', 'Tool AABB']
    for idx, la in enumerate(link_aabbs):
        lo_x, lo_y = float(la.min_point[0]), float(la.min_point[1])
        hi_x, hi_y = float(la.max_point[0]), float(la.max_point[1])
        c = aabb_colors[idx % len(aabb_colors)]
        rect = Rectangle((lo_x, lo_y), hi_x - lo_x, hi_y - lo_y,
                          facecolor=c, alpha=0.07,
                          edgecolor=c, linewidth=2.2, linestyle='--',
                          hatch=aabb_hatches[idx % len(aabb_hatches)],
                          label=aabb_labels[idx] if idx < len(aabb_labels) else None)
        ax_ws.add_patch(rect)

    handles_ws = [
        mpatches.Patch(facecolor='#2196F3', alpha=0.12, edgecolor='#2196F3',
                       hatch='///', label='Link 1 AABB'),
        mpatches.Patch(facecolor='#FF9800', alpha=0.12, edgecolor='#FF9800',
                       hatch='\\\\\\', label='Link 2 AABB'),
        Line2D([], [], color='#2196F3', lw=1.0, alpha=0.35,
               label=f'{N_SAMPLES} sampled arms'),
        Line2D([], [], color='#2196F3', lw=3.0, alpha=0.95,
               label='Midpoint arm'),
        mpatches.Patch(facecolor='#9e9e9e', alpha=0.3, label='Obstacle'),
    ]
    ax_ws.legend(handles=handles_ws, loc='lower left', fontsize=8,
                 framealpha=0.9)

    fig.text(0.55, 0.50, 'FK\n⟶\nAABB', ha='center', va='center',
             fontsize=11, fontweight='bold', color='#666')
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    out = OUT_DIR / "img_s4_aabb_envelope.png"
    fig.savefig(str(out), dpi=DPI, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f"  Saved: {out.name}")


# ═══════════════════════════════════════════════════════════════════════════
#  Build shared forest (used by steps 2-5)
# ═══════════════════════════════════════════════════════════════════════════

def build_forests():
    """Build forest v1 (initial) and v2 (with new obstacle). Returns shared data."""
    robot = load_robot("2dof_planar")

    # ── Forest V1 (initial obstacles) ──
    print("  Building forest v1 (initial obstacles) ...")
    scene_v1 = build_scene(INITIAL_OBSTACLES)
    cfg1 = make_forest_cfg(SEED, max_boxes_val=300, coarsen_val=0)
    prep1 = grow_and_prepare(robot, scene_v1, cfg1, Q_START, Q_GOAL,
                             ndim=2, no_cache=True)
    boxes_v1 = snapshot_boxes(prep1['forest_obj'])
    forest_v1 = prep1['forest_obj']
    print(f"    v1 boxes: {len(boxes_v1)}")

    # ── Invalidate against new obstacle ──
    new_obs = Obstacle(
        min_point=np.asarray(NEW_OBSTACLE["min"]),
        max_point=np.asarray(NEW_OBSTACLE["max"]),
        name=NEW_OBSTACLE["name"],
    )
    colliding_ids = forest_v1.invalidate_against_obstacle(
        new_obs, robot, safety_margin=0.0)
    surviving_ids = set(boxes_v1.keys()) - colliding_ids
    print(f"    Invalidated: {len(colliding_ids)}, surviving: {len(surviving_ids)}")

    # ── Forest V2 (all obstacles) — grow WITHOUT coarsen first ──
    print("  Building forest v2 (all obstacles) ...")
    all_obstacles = INITIAL_OBSTACLES + [NEW_OBSTACLE]
    scene_v2 = build_scene(all_obstacles)
    checker_v2 = CollisionChecker(robot=robot, scene=scene_v2)
    # Grow with coarsen=0 to preserve raw growth order from start/goal anchors
    cfg2 = make_forest_cfg(SEED + 100, max_boxes_val=500, coarsen_val=0)
    prep2 = grow_and_prepare(robot, scene_v2, cfg2, Q_START, Q_GOAL,
                             ndim=2, no_cache=True)
    # Snapshot raw boxes (insertion order = growth from start/goal anchors)
    boxes_v2_raw = snapshot_boxes(prep2['forest_obj'])
    n_raw = len(boxes_v2_raw)
    print(f"    v2 raw boxes: {n_raw}")

    # Manually coarsen for path planning
    coarsen_stats = coarsen_forest(
        tree=prep2['planner'].hier_tree,
        forest=prep2['forest_obj'],
        obstacles=prep2['planner'].obstacles,
        safety_margin=0.0,
        max_rounds=20,
    )
    boxes_v2 = snapshot_boxes(prep2['forest_obj'])
    forest_v2 = prep2['forest_obj']
    print(f"    v2 coarsened: {n_raw} -> {len(boxes_v2)} boxes "
          f"({coarsen_stats.n_merges} merges)")

    # ── Collision maps ──
    print("  Scanning collision maps ...")
    cmap_v1, extent = scan_collision_map(robot, scene_v1, resolution=0.04)
    cmap_v2, _ = scan_collision_map(robot, scene_v2, resolution=0.04)

    # ── Adjacency & path planning (on v2) ──
    print("  Building adjacency graph ...")
    adj, uf, islands = _build_adjacency_and_islands(boxes_v2, period=PERIOD)
    src = find_box_containing(Q_START, boxes_v2)
    tgt = find_box_containing(Q_GOAL, boxes_v2)
    n_edges = sum(len(v) for v in adj.values()) // 2
    print(f"    {len(adj)} vertices, {n_edges} edges, {len(islands)} islands")
    print(f"    src={src}, tgt={tgt}, same_island={uf.same(src, tgt) if src and tgt else 'N/A'}")

    # Bridge if disconnected
    bridge_edges = []
    if src is not None and tgt is not None and not uf.same(src, tgt):
        print("  Bridging disconnected islands ...")
        br_edges, br_islands, _, br_boxes, _ = bridge_islands(
            boxes=boxes_v2,
            collision_checker=checker_v2,
            segment_resolution=0.03,
            period=PERIOD,
            precomputed_uf=uf,
            precomputed_islands=islands,
            target_pair=(src, tgt),
        )
        bridge_edges = br_edges
        _add_bridge_to_adj(adj, bridge_edges, uf)
        print(f"    Added {len(bridge_edges)} bridge edges")

    # Path planning
    waypoints = []
    box_seq = []
    if src is not None and tgt is not None:
        # Try direct geodesic connect
        if not checker_v2.check_segment_collision(Q_START, Q_GOAL, 0.03, period=PERIOD):
            waypoints = [Q_START.copy(), Q_GOAL.copy()]
            box_seq = [src] if src == tgt else [src, tgt]
            print("  Direct connect OK!")
        else:
            box_seq_raw, raw_dist = dijkstra_box_graph(boxes_v2, adj, src, tgt)
            if box_seq_raw is not None:
                box_seq = _shortcut_box_seq(box_seq_raw, adj)
                wp_raw = build_waypoints(Q_START, Q_GOAL, box_seq, boxes_v2)
                waypoints = pull_tight(wp_raw, box_seq, boxes_v2, n_iters=30)
                # Compute path cost
                path_cost = sum(geo_dist(waypoints[i], waypoints[i+1])
                                for i in range(len(waypoints) - 1))
                print(f"  Path found: {len(box_seq_raw)} → {len(box_seq)} boxes, "
                      f"{len(waypoints)} wp, cost={path_cost:.3f}")
                # Debug: check if path wraps ±π
                th1_range = [w[0] for w in waypoints]
                print(f"    θ₁ range: [{min(th1_range):.2f}, {max(th1_range):.2f}]")
            else:
                print("  WARNING: No path found!")
    else:
        print(f"  WARNING: start/goal not in boxes (src={src}, tgt={tgt})")

    return dict(
        robot=robot,
        scene_v1=scene_v1, scene_v2=scene_v2,
        checker_v2=checker_v2,
        boxes_v1=boxes_v1, boxes_v2=boxes_v2,
        boxes_v2_raw=boxes_v2_raw,  # pre-coarsen, for growth GIF
        forest_v1=forest_v1, forest_v2=forest_v2,
        colliding_ids=colliding_ids, surviving_ids=surviving_ids,
        cmap_v1=cmap_v1, cmap_v2=cmap_v2, extent=extent,
        adj=adj, box_seq=box_seq, waypoints=waypoints,
    )


# ═══════════════════════════════════════════════════════════════════════════
#  2. Box Forest Growth Animation (GIF)
# ═══════════════════════════════════════════════════════════════════════════

def gen_forest_growth(data):
    print("\n[2/6] Box Forest Growth Animation ...")
    # Use pre-coarsen raw boxes: insertion order reflects growth from start/goal
    boxes_v2 = data['boxes_v2_raw']
    cmap_v2 = data['cmap_v2']
    extent = data['extent']

    ordered_items = list(boxes_v2.items())
    n_total = len(ordered_items)
    stride = max(1, n_total // 40)
    frame_sizes = list(range(stride, n_total + 1, stride))
    if frame_sizes[-1] != n_total:
        frame_sizes.append(n_total)

    fig, ax = plt.subplots(1, 1, figsize=(7.0, 6.2))
    fig.patch.set_facecolor('white')

    legend_elements = [
        Rectangle((0, 0), 1, 1, facecolor='#4CAF50', alpha=0.30,
                  edgecolor='#4CAF50', label='Safe boxes'),
        Rectangle((0, 0), 1, 1, facecolor='#d32f2f', alpha=0.26,
                  edgecolor='none', label='C-obstacle'),
        Line2D([], [], marker='o', color='cyan', markersize=6,
               markeredgecolor='black', linestyle='none', label='Start'),
        Line2D([], [], marker='*', color='yellow', markersize=10,
               markeredgecolor='black', linestyle='none', label='Goal'),
    ]

    def _render_frame(frame_idx: int):
        ax.clear()
        n_show = frame_sizes[frame_idx]
        setup_cspace_ax(ax, f"Forest Growth  {n_show}/{n_total}")
        draw_collision_bg(ax, cmap_v2, extent, alpha=0.26)
        sub_boxes = dict(ordered_items[:n_show])
        draw_boxes(ax, sub_boxes, alpha=0.30)
        ax.plot(Q_START[0], Q_START[1], 'o', color='cyan', markersize=8,
                markeredgecolor='black', zorder=20)
        ax.plot(Q_GOAL[0], Q_GOAL[1], '*', color='yellow', markersize=12,
                markeredgecolor='black', zorder=20)
        ax.legend(handles=legend_elements, loc='lower right', fontsize=8.5,
                  framealpha=0.92)

    anim = FuncAnimation(fig, _render_frame, frames=len(frame_sizes),
                         interval=120, repeat=True)
    out = OUT_DIR / "img_s4_forest_growth.gif"
    anim.save(str(out), writer=PillowWriter(fps=7), dpi=120)
    plt.close(fig)
    print(f"  Saved: {out.name}")


# ═══════════════════════════════════════════════════════════════════════════
#  3. Coarsen Before/After Comparison
# ═══════════════════════════════════════════════════════════════════════════

def gen_coarsen_compare(data):
    """Side-by-side comparison of forest before and after coarsening."""
    print("\n[3/6] Coarsen Before / After ...")
    boxes_raw = data['boxes_v2_raw']
    boxes_coarsened = data['boxes_v2']
    cmap_v2 = data['cmap_v2']
    extent = data['extent']

    n_raw = len(boxes_raw)
    n_coarsened = len(boxes_coarsened)

    fig, axes = plt.subplots(1, 2, figsize=(13.5, 5.8))
    fig.patch.set_facecolor('white')

    # ── Left: before coarsen (raw) ──
    ax = axes[0]
    setup_cspace_ax(ax, f"Before Coarsen: {n_raw} boxes")
    draw_collision_bg(ax, cmap_v2, extent, alpha=0.20)
    draw_boxes(ax, boxes_raw, alpha=0.30)
    ax.plot(Q_START[0], Q_START[1], 'o', color='cyan', markersize=7,
            markeredgecolor='black', zorder=20)
    ax.plot(Q_GOAL[0], Q_GOAL[1], '*', color='yellow', markersize=11,
            markeredgecolor='black', zorder=20)

    # ── Right: after coarsen ──
    ax = axes[1]
    setup_cspace_ax(ax, f"After Coarsen: {n_coarsened} boxes")
    draw_collision_bg(ax, cmap_v2, extent, alpha=0.20)
    draw_boxes(ax, boxes_coarsened, alpha=0.30)
    ax.plot(Q_START[0], Q_START[1], 'o', color='cyan', markersize=7,
            markeredgecolor='black', zorder=20)
    ax.plot(Q_GOAL[0], Q_GOAL[1], '*', color='yellow', markersize=11,
            markeredgecolor='black', zorder=20)

    # Legend
    legend_elements = [
        Rectangle((0, 0), 1, 1, facecolor='#4CAF50', alpha=0.30,
                  edgecolor='#4CAF50', label='Safe boxes'),
        Rectangle((0, 0), 1, 1, facecolor='#d32f2f', alpha=0.20,
                  edgecolor='none', label='C-obstacle'),
        Line2D([], [], marker='o', color='cyan', markersize=6,
               markeredgecolor='black', linestyle='none', label='Start'),
        Line2D([], [], marker='*', color='yellow', markersize=10,
               markeredgecolor='black', linestyle='none', label='Goal'),
    ]
    fig.legend(handles=legend_elements, loc='lower center', ncol=4,
               fontsize=9, framealpha=0.92, bbox_to_anchor=(0.5, -0.01))

    fig.suptitle(f"Coarsen: {n_raw} → {n_coarsened} boxes "
                 f"({n_raw - n_coarsened} merged)",
                 fontsize=13, fontweight='bold', y=0.99)
    plt.tight_layout(rect=[0, 0.05, 1, 0.96])
    out = OUT_DIR / "img_s4_coarsen.png"
    fig.savefig(str(out), dpi=DPI, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f"  Saved: {out.name}")


# ═══════════════════════════════════════════════════════════════════════════
#  4. Incremental Update (obstacle added → box forest changes)
# ═══════════════════════════════════════════════════════════════════════════

def gen_incremental_update(data):
    print("\n[4/6] Incremental Update (obstacle added) ...")
    boxes_v1 = data['boxes_v1']
    boxes_v2 = data['boxes_v2']
    colliding_ids = data['colliding_ids']
    surviving_ids = data['surviving_ids']
    cmap_v1 = data['cmap_v1']
    cmap_v2 = data['cmap_v2']
    extent = data['extent']
    rng = np.random.default_rng(SEED + 7)

    fig, axes = plt.subplots(1, 3, figsize=(19.5, 5.8))
    fig.patch.set_facecolor('white')

    # ── Panel 1: Before (initial forest) ──
    ax = axes[0]
    setup_cspace_ax(ax, f"① Before: {len(boxes_v1)} boxes")
    draw_collision_bg(ax, cmap_v1, extent, alpha=0.24)
    draw_boxes(ax, boxes_v1, alpha=0.28)
    ax.plot(Q_START[0], Q_START[1], 'o', color='cyan', markersize=7,
            markeredgecolor='black', zorder=20)
    ax.plot(Q_GOAL[0], Q_GOAL[1], '*', color='yellow', markersize=11,
            markeredgecolor='black', zorder=20)

    # ── Panel 2: Invalidation (show old + colliding highlighted) ──
    ax = axes[1]
    setup_cspace_ax(ax, f"② Invalidate: {len(colliding_ids)} removed")
    draw_collision_bg(ax, cmap_v2, extent, alpha=0.24)
    surviving_boxes = {bid: boxes_v1[bid] for bid in surviving_ids}
    colliding_boxes = {bid: boxes_v1[bid] for bid in colliding_ids if bid in boxes_v1}
    draw_boxes(ax, surviving_boxes, alpha=0.14)
    draw_boxes(ax, colliding_boxes,
               highlight_ids=set(colliding_boxes.keys()),
               highlight_color='#ff1744', highlight_alpha=0.45)
    ax.plot(Q_START[0], Q_START[1], 'o', color='cyan', markersize=7,
            markeredgecolor='black', zorder=20)
    ax.plot(Q_GOAL[0], Q_GOAL[1], '*', color='yellow', markersize=11,
            markeredgecolor='black', zorder=20)

    # ── Panel 3: After (rebuilt forest v2) ──
    ax = axes[2]
    setup_cspace_ax(ax, f"③ After: {len(boxes_v2)} boxes")
    draw_collision_bg(ax, cmap_v2, extent, alpha=0.24)
    draw_boxes(ax, boxes_v2, alpha=0.28)
    ax.plot(Q_START[0], Q_START[1], 'o', color='cyan', markersize=7,
            markeredgecolor='black', zorder=20)
    ax.plot(Q_GOAL[0], Q_GOAL[1], '*', color='yellow', markersize=11,
            markeredgecolor='black', zorder=20)

    # Legend
    legend_elements = [
        Rectangle((0, 0), 1, 1, facecolor='#4CAF50', alpha=0.28,
                  edgecolor='#4CAF50', label='Safe boxes'),
        Rectangle((0, 0), 1, 1, facecolor='#ff1744', alpha=0.45,
                  edgecolor='#ff1744', label='Invalidated boxes'),
        Rectangle((0, 0), 1, 1, facecolor='#d32f2f', alpha=0.24,
                  edgecolor='none', label='C-obstacle'),
        Line2D([], [], marker='o', color='cyan', markersize=6,
               markeredgecolor='black', linestyle='none', label='Start'),
        Line2D([], [], marker='*', color='yellow', markersize=10,
               markeredgecolor='black', linestyle='none', label='Goal'),
    ]
    fig.legend(handles=legend_elements, loc='lower center', ncol=5,
               fontsize=9, framealpha=0.92, bbox_to_anchor=(0.5, -0.01))

    fig.suptitle("Incremental Update: New Obstacle Added",
                 fontsize=13, fontweight='bold', y=0.99)
    plt.tight_layout(rect=[0, 0.05, 1, 0.96])
    out = OUT_DIR / "img_s4_incremental.png"
    fig.savefig(str(out), dpi=DPI, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f"  Saved: {out.name}")


# ═══════════════════════════════════════════════════════════════════════════
#  5. C-space Path Visualization
# ═══════════════════════════════════════════════════════════════════════════

def gen_cspace_path(data):
    print("\n[5/6] C-space Path ...")
    boxes_v2 = data['boxes_v2']
    cmap_v2 = data['cmap_v2']
    extent = data['extent']
    box_seq = data['box_seq']
    waypoints = data['waypoints']

    fig, ax = plt.subplots(1, 1, figsize=(7.5, 6.5))
    fig.patch.set_facecolor('white')

    setup_cspace_ax(ax, "C-space Path Planning")
    draw_collision_bg(ax, cmap_v2, extent, alpha=0.24)
    draw_boxes(ax, boxes_v2, alpha=0.12)

    # Highlight box sequence
    if box_seq:
        for bid in box_seq:
            if bid not in boxes_v2:
                continue
            box = boxes_v2[bid]
            lo_x, hi_x = box.joint_intervals[0]
            lo_y, hi_y = box.joint_intervals[1]
            rect = Rectangle((lo_x, lo_y), hi_x - lo_x, hi_y - lo_y,
                              linewidth=1.5, edgecolor='#00ccff',
                              facecolor='#00ccff', alpha=0.25, zorder=8)
            ax.add_patch(rect)

    # Draw path
    if waypoints:
        draw_cspace_path(ax, waypoints, color='#00ff00', lw=2.5,
                         label=f'Path ({len(waypoints)} wp)')

    ax.plot(Q_START[0], Q_START[1], 'o', color='cyan', markersize=10,
            markeredgecolor='black', linewidth=1.5, zorder=25, label='Start')
    ax.plot(Q_GOAL[0], Q_GOAL[1], '*', color='yellow', markersize=14,
            markeredgecolor='black', linewidth=1.5, zorder=25, label='Goal')

    # Stats text
    n_boxes = len(boxes_v2)
    n_path = len(box_seq)
    path_info = f"boxes={n_boxes}, path_boxes={n_path}"
    if waypoints:
        cost = sum(geo_dist(waypoints[i], waypoints[i+1])
                   for i in range(len(waypoints) - 1))
        path_info += f", cost={cost:.3f}"
    ax.text(-3.0, 2.85, path_info, fontsize=9, color='#263238',
            bbox=dict(boxstyle='round,pad=0.25', facecolor='white', alpha=0.85,
                      edgecolor='none'))

    ax.legend(loc='upper right', fontsize=9, framealpha=0.92)
    plt.tight_layout()
    out = OUT_DIR / "img_s4_cspace_path.png"
    fig.savefig(str(out), dpi=DPI, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f"  Saved: {out.name}")


# ═══════════════════════════════════════════════════════════════════════════
#  6. Workspace Execution Path Animation (GIF)
# ═══════════════════════════════════════════════════════════════════════════

def gen_workspace_path_animation(data):
    print("\n[6/6] Workspace Execution Path Animation ...")
    waypoints = data['waypoints']
    all_obstacles = INITIAL_OBSTACLES + [NEW_OBSTACLE]

    if not waypoints:
        print("  SKIP: no path to animate.")
        return

    # Resample to smooth trajectory
    dense_path = resample_path(waypoints, ds=0.06)
    n_frames = len(dense_path)
    print(f"  Resampled: {len(waypoints)} wp → {n_frames} frames")

    # Pre-compute FK for all frames
    fk_all = [fk_2dof(float(q[0]), float(q[1])) for q in dense_path]
    ee_all = np.array([pts[-1] for pts in fk_all])

    fig, ax = plt.subplots(1, 1, figsize=(7.0, 7.0))
    fig.patch.set_facecolor('white')

    # Static elements
    def _draw_static():
        ax.clear()
        setup_workspace_ax(ax, "")
        draw_workspace_obstacles(ax, INITIAL_OBSTACLES, new_obstacle=NEW_OBSTACLE)
        # Draw EE trace (full, faint)
        ax.plot(ee_all[:, 0], ee_all[:, 1], '-', color='#4CAF50', alpha=0.25,
                linewidth=1.5, zorder=2)
        # Base
        ax.plot(0, 0, 'ko', markersize=8, zorder=10)

    # Arm trail colors
    trail_color = '#90CAF9'

    def _render_frame(frame_idx: int):
        _draw_static()
        ax.set_title(f"Workspace Execution  frame {frame_idx+1}/{n_frames}",
                     fontsize=11, fontweight='bold', pad=8)

        # Draw ghost trail (every 8th past frame)
        step = max(1, n_frames // 15)
        for k in range(0, frame_idx, step):
            pts = fk_all[k]
            ax.plot(pts[:, 0], pts[:, 1], '-', color=trail_color,
                    alpha=0.2, linewidth=1.2, zorder=3)

        # Draw EE trace so far (brighter)
        ax.plot(ee_all[:frame_idx+1, 0], ee_all[:frame_idx+1, 1],
                '-', color='#4CAF50', alpha=0.6, linewidth=2.0, zorder=4)

        # Draw current arm
        pts = fk_all[frame_idx]
        ax.plot(pts[:, 0], pts[:, 1], '-o', color='#1565C0', linewidth=3.0,
                markersize=6, markeredgecolor='black', zorder=15)
        ax.plot(pts[-1, 0], pts[-1, 1], 'o', color='#E53935', markersize=8,
                markeredgecolor='black', zorder=16)

        # Start / Goal arms (faint reference)
        pts_s = fk_all[0]
        pts_g = fk_all[-1]
        ax.plot(pts_s[:, 0], pts_s[:, 1], '--o', color='cyan', linewidth=1.5,
                markersize=3, alpha=0.5, zorder=5)
        ax.plot(pts_g[:, 0], pts_g[:, 1], '--*', color='yellow', linewidth=1.5,
                markersize=6, alpha=0.5, zorder=5)

        legend_elements = [
            Line2D([], [], color='#1565C0', lw=3, marker='o', markersize=5,
                   markeredgecolor='black', label='Current arm'),
            Line2D([], [], color='#4CAF50', lw=2, alpha=0.6, label='EE trace'),
            Line2D([], [], color='cyan', lw=1.5, linestyle='--', alpha=0.5,
                   label='Start'),
            Line2D([], [], color='yellow', lw=1.5, linestyle='--', alpha=0.5,
                   label='Goal'),
            mpatches.Patch(facecolor='#9e9e9e', alpha=0.3, label='Obstacle'),
            mpatches.Patch(facecolor='#ef5350', alpha=0.55, label='New obstacle'),
        ]
        ax.legend(handles=legend_elements, loc='upper right', fontsize=7.5,
                  framealpha=0.9)

    # Subsample frames for GIF (aim for ~60 frames)
    if n_frames > 80:
        frame_stride = max(1, n_frames // 60)
        frame_indices = list(range(0, n_frames, frame_stride))
        if frame_indices[-1] != n_frames - 1:
            frame_indices.append(n_frames - 1)
    else:
        frame_indices = list(range(n_frames))

    print(f"  Rendering {len(frame_indices)} GIF frames ...")
    anim = FuncAnimation(fig, lambda fi: _render_frame(frame_indices[fi]),
                         frames=len(frame_indices),
                         interval=80, repeat=True)
    out = OUT_DIR / "img_s4_workspace_path.gif"
    anim.save(str(out), writer=PillowWriter(fps=12), dpi=110)
    plt.close(fig)
    print(f"  Saved: {out.name}")


# ═══════════════════════════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════════════════════════

def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    print("=" * 60)
    print("  Scene 04 — 6-in-1 Visualization")
    print("=" * 60)

    # 1. AABB Envelope (independent, uses only initial obstacles)
    gen_aabb_envelope()

    # Build shared forests for steps 2-5
    print("\n[Shared] Building forests & planning path ...")
    data = build_forests()

    # 2. Forest Growth GIF
    gen_forest_growth(data)

    # 3. Coarsen Before/After
    gen_coarsen_compare(data)

    # 4. Incremental Update
    gen_incremental_update(data)

    # 5. C-space Path
    gen_cspace_path(data)

    # 6. Workspace Path Animation
    gen_workspace_path_animation(data)

    print("\n" + "=" * 60)
    print(f"  All outputs in: {OUT_DIR}")
    for p in sorted(OUT_DIR.glob("img_s4_*")):
        print(f"    {p.name}  ({p.stat().st_size / 1024:.0f} KB)")
    print("=" * 60)


if __name__ == "__main__":
    main()

"""
examples/viz_2dof_full_pipeline.py — 2DOF 场景完整流程可视化

生成分阶段可视化图:
  1. C-space 碰撞底图
  2. Forest expansion (逐步快照 → GIF)
  3. Coarsen 前后对比
  4. 岛检测 & 桥接
  5. Dijkstra 路径规划 (box序列 + 精炼路径)
  6. 最终总览图

用法:
    cd v3
    python examples/viz_2dof_full_pipeline.py [--seed 42] [--obstacles 8]
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrowPatch
from matplotlib.collections import PatchCollection
import matplotlib.colors as mcolors
import numpy as np

# ── v3 path setup ──
_ROOT = Path(__file__).resolve().parents[1]  # v3/
_SRC = _ROOT / "src"
for p in (_ROOT, _SRC):
    sp = str(p)
    if sp not in sys.path:
        sys.path.insert(0, sp)

from aabb.robot import load_robot
from forest.scene import Scene
from forest.collision import CollisionChecker
from forest.models import BoxNode
from forest.connectivity import find_islands, bridge_islands, UnionFind
from forest.safe_box_forest import SafeBoxForest
from forest.hier_aabb_tree import HierAABBTree
from forest.coarsen import coarsen_forest
from planner.sbf_planner import SBFPlanner
from planner.models import SBFConfig, gmean_edge_length
from planner.defaults import TWODOF_DEFAULTS
from planner.pipeline import (
    PandaGCSConfig, make_planner_config,
    _build_adjacency_and_islands, find_box_containing,
)


# ═══════════════════════════════════════════════════════════════════════════
# Config
# ═══════════════════════════════════════════════════════════════════════════

@dataclass
class Viz2DConfig:
    seed: int = 42
    robot_name: str = "2dof_planar"
    n_obstacles: int = 8

    q_start: List[float] = field(
        default_factory=lambda: [0.8 * np.pi, 0.2])
    q_goal: List[float] = field(
        default_factory=lambda: [-0.7 * np.pi, -0.4])

    # forest
    max_consecutive_miss: int = TWODOF_DEFAULTS.max_consecutive_miss
    max_boxes: int = TWODOF_DEFAULTS.max_boxes
    ffb_min_edge: float = TWODOF_DEFAULTS.ffb_min_edge
    guided_sample_ratio: float = TWODOF_DEFAULTS.guided_sample_ratio

    # coarsen
    coarsen_max_rounds: int = 20

    # island / bridge
    min_island_size: float = 0.3

    # viz
    snapshot_every: int = 3
    collision_resolution: float = 0.025
    dpi: int = 150
    gif_frame_ms: int = 250

    # scene obstacles
    obs_cx_range: Tuple[float, float] = (-1.8, 1.8)
    obs_cy_range: Tuple[float, float] = (-1.8, 1.8)
    obs_w_range: Tuple[float, float] = (0.3, 0.8)
    obs_h_range: Tuple[float, float] = (0.3, 0.8)


# ═══════════════════════════════════════════════════════════════════════════
# Random scene
# ═══════════════════════════════════════════════════════════════════════════

def build_random_2d_scene(robot, q_start, q_goal, rng, cfg: Viz2DConfig,
                          max_trials=300, period=None) -> Scene:
    """生成随机 2D 配置空间障碍物场景，确保起终点无碰撞且直连路径被阻挡。"""
    for _ in range(max_trials):
        scene = Scene()
        for i in range(cfg.n_obstacles):
            cx = float(rng.uniform(*cfg.obs_cx_range))
            cy = float(rng.uniform(*cfg.obs_cy_range))
            w = float(rng.uniform(*cfg.obs_w_range))
            h = float(rng.uniform(*cfg.obs_h_range))
            scene.add_obstacle([cx - w/2, cy - h/2],
                               [cx + w/2, cy + h/2], name=f"obs_{i}")

        checker = CollisionChecker(robot=robot, scene=scene)
        if checker.check_config_collision(q_start):
            continue
        if checker.check_config_collision(q_goal):
            continue
        # 确保直连路径被阻挡(才有规划价值)
        if not checker.check_segment_collision(q_start, q_goal, 0.03):
            continue
        # 确保 geodesic (wrap-around) 路径也被阻挡
        if period is not None:
            if not checker.check_segment_collision(
                    q_start, q_goal, 0.03, period=period):
                continue
        return scene
    raise RuntimeError("无法生成满足条件的随机场景")


# ═══════════════════════════════════════════════════════════════════════════
# C-space collision map
# ═══════════════════════════════════════════════════════════════════════════

def scan_collision_map(robot, scene, joint_limits, resolution=0.03):
    """扫描 2DOF C-space 碰撞区域 → 二值矩阵."""
    checker = CollisionChecker(robot=robot, scene=scene)
    lo_x, hi_x = joint_limits[0]
    lo_y, hi_y = joint_limits[1]
    xs = np.arange(lo_x, hi_x, resolution)
    ys = np.arange(lo_y, hi_y, resolution)
    cmap = np.zeros((len(ys), len(xs)), dtype=np.float32)
    for i, y in enumerate(ys):
        row = np.column_stack([xs, np.full(len(xs), y)])
        cmap[i, :] = checker.check_config_collision_batch(row).astype(np.float32)
    extent = [lo_x, hi_x, lo_y, hi_y]
    return cmap, extent


# ═══════════════════════════════════════════════════════════════════════════
# Forest grow with snapshots
# ═══════════════════════════════════════════════════════════════════════════

def grow_forest_with_snapshots(
    planner: SBFPlanner,
    q_start: np.ndarray,
    q_goal: np.ndarray,
    seed: int,
    max_miss: int,
    max_boxes: int,
    snapshot_every: int = 3,
):
    """串行 forest 生长, 每 snapshot_every 个 box 拍一次快照."""
    ndim = 2
    rng = np.random.default_rng(seed)
    forest = planner._load_or_create_forest()
    forest.hier_tree = planner.hier_tree
    obs_packed = planner.hier_tree._prepack_obstacles_c(planner.obstacles)

    snapshots = []   # [(n_boxes, boxes_copy, new_box_id)]
    added_since_snap = 0

    def _snap(new_id=-1, force=False):
        nonlocal added_since_snap
        added_since_snap += 1
        if force or added_since_snap >= snapshot_every:
            boxes_copy = {}
            for bid, b in forest.boxes.items():
                boxes_copy[bid] = BoxNode(
                    node_id=b.node_id,
                    joint_intervals=[tuple(iv) for iv in b.joint_intervals],
                    seed_config=b.seed_config.copy() if b.seed_config is not None else None,
                    volume=b.volume)
            snapshots.append((forest.n_boxes, boxes_copy, new_id))
            added_since_snap = 0

    def _try_add(q):
        """尝试在 q 处生长 box, 返回 box id 或 -1."""
        if planner.hier_tree.is_occupied(q):
            return -1
        nid = forest.allocate_id()
        ffb = planner.hier_tree.find_free_box(
            q, planner.obstacles, mark_occupied=True,
            forest_box_id=nid, obs_packed=obs_packed)
        if ffb is None:
            return -1
        vol = 1.0
        for lo, hi in ffb.intervals:
            vol *= max(hi - lo, 0)
        box = BoxNode(node_id=nid, joint_intervals=ffb.intervals,
                      seed_config=q.copy(), volume=vol)
        if ffb.absorbed_box_ids:
            forest.remove_boxes(ffb.absorbed_box_ids)
        forest.add_box_direct(box)
        return nid

    # seed points
    from collections import deque
    bfs_queue = deque()  # (box, excluded_faces)

    for qs in [q_start, q_goal]:
        nid = _try_add(qs)
        if nid >= 0:
            _snap(nid)
            bfs_queue.append((forest.boxes[nid], frozenset()))

    # sampling
    intervals = planner.joint_limits
    lows = np.array([lo for lo, _ in intervals])
    highs = np.array([hi for _, hi in intervals])
    guided_ratio = getattr(planner.config, 'guided_sample_ratio', 0.8)
    has_hier_tree = hasattr(planner, 'hier_tree') and planner.hier_tree is not None

    consec = 0

    while consec < max_miss:
        if forest.n_boxes >= max_boxes:
            break

        # BFS boundary expand
        if bfs_queue:
            bfs_box, excluded = bfs_queue.popleft()
            seeds = planner._generate_boundary_seeds(bfs_box, rng, excluded)
            for dim, side, q in seeds:
                if forest.n_boxes >= max_boxes:
                    break
                nid = _try_add(q)
                if nid >= 0:
                    consec = 0
                    _snap(nid)
                    child_excluded = frozenset({(dim, 1 - side)})
                    bfs_queue.append((forest.boxes[nid], child_excluded))
            continue

        # guided / uniform sampling
        roll = rng.uniform()
        if roll < guided_ratio and has_hier_tree:
            try:
                q = planner.hier_tree.sample_unoccupied_seed(rng)
            except ValueError:
                q = None
            if q is None:
                q = rng.uniform(lows, highs)
        else:
            q = rng.uniform(lows, highs)

        if planner.collision_checker.check_config_collision(q):
            consec += 1
            continue

        nid = _try_add(q)
        if nid >= 0:
            consec = 0
            _snap(nid)
            bfs_queue.append((forest.boxes[nid], frozenset()))
        else:
            consec += 1

    # final snapshot
    if not snapshots or snapshots[-1][0] != forest.n_boxes:
        _snap(-1, force=True)

    return snapshots, forest


# ═══════════════════════════════════════════════════════════════════════════
# Plotting helpers
# ═══════════════════════════════════════════════════════════════════════════

def _draw_collision_bg(ax, cmap_data, extent):
    """在 ax 上绘制碰撞底图."""
    ax.imshow(cmap_data, origin="lower", extent=extent,
              cmap="Reds", alpha=0.30, aspect="auto")


def _draw_boxes(ax, boxes, color_by="volume", highlight_ids=None,
                alpha=0.30, edge_alpha=0.8, lw=0.5):
    """绘制 box 集合."""
    highlight_ids = highlight_ids or set()
    cmap = plt.cm.viridis

    if color_by == "volume":
        vols = [b.volume for b in boxes.values()]
        vmin = min(vols) if vols else 0
        vmax = max(vols) if vols else 1
        norm = mcolors.Normalize(vmin=vmin, vmax=vmax)

    for bid, box in boxes.items():
        lo_x, hi_x = box.joint_intervals[0]
        lo_y, hi_y = box.joint_intervals[1]

        if bid in highlight_ids:
            ec, fc, a = "#ff6600", "#ff9933", 0.50
            rect_lw = 1.5
        else:
            if color_by == "volume":
                c = cmap(norm(box.volume))
            else:
                c = (0.2, 0.5, 0.8, 1.0)
            ec, fc, a = c, c, alpha
            rect_lw = lw

        rect = Rectangle((lo_x, lo_y), hi_x - lo_x, hi_y - lo_y,
                          linewidth=rect_lw, edgecolor=ec,
                          facecolor=fc, alpha=a)
        ax.add_patch(rect)


def _draw_start_goal(ax, q_start, q_goal):
    """标记起点和终点."""
    ax.plot(q_start[0], q_start[1], 'o', color='cyan', markersize=10,
            markeredgecolor='black', markeredgewidth=1.2, zorder=20,
            label='Start')
    ax.plot(q_goal[0], q_goal[1], '*', color='yellow', markersize=14,
            markeredgecolor='black', markeredgewidth=1.0, zorder=20,
            label='Goal')


def _draw_islands(ax, boxes, islands):
    """按岛着色绘制 boxes."""
    cmap = plt.cm.tab20
    n_islands = max(len(islands), 1)
    node_island = {}
    for idx, island in enumerate(islands):
        for bid in island:
            node_island[bid] = idx

    for bid, box in boxes.items():
        lo_x, hi_x = box.joint_intervals[0]
        lo_y, hi_y = box.joint_intervals[1]
        isl_idx = node_island.get(bid, 0)
        c = cmap(isl_idx / n_islands)
        rect = Rectangle((lo_x, lo_y), hi_x - lo_x, hi_y - lo_y,
                          linewidth=0.5, edgecolor=c, facecolor=c, alpha=0.35)
        ax.add_patch(rect)


def _geodesic_points(qa, qb, period, n_pts=200):
    """沿环面最短路径采样 n_pts 个点, 归一化到 [-period/2, period/2)."""
    half = period / 2.0
    diff = ((qb - qa) + half) % period - half
    pts = []
    for i in range(n_pts):
        t = i / (n_pts - 1)
        q = qa + t * diff
        q = ((q + half) % period) - half
        pts.append(q)
    return pts


def _draw_wrapped_line(ax, qa, qb, period, n_pts=200, **kwargs):
    """绘制环面最短路径线段, 在周期边界处自动断开."""
    if period is None or period <= 0:
        ax.plot([qa[0], qb[0]], [qa[1], qb[1]], **kwargs)
        return
    pts = _geodesic_points(qa, qb, period, n_pts)
    # 检测相邻点是否跳变 (跳变 = 周期 wrap)
    segments = [[pts[0]]]
    half = period / 2.0
    for i in range(1, len(pts)):
        if np.any(np.abs(pts[i] - pts[i - 1]) > half):
            segments.append([pts[i]])  # 新段
        else:
            segments[-1].append(pts[i])
    first_drawn = True
    for seg in segments:
        if len(seg) < 2:
            continue
        xs = [p[0] for p in seg]
        ys = [p[1] for p in seg]
        kw = dict(kwargs)
        if not first_drawn:
            kw.pop('label', None)  # 仅首段保留 label, 避免重复图例
        first_drawn = False
        ax.plot(xs, ys, **kw)


def _draw_bridge_edges(ax, bridge_edges, period=None):
    """绘制桥接边 (周期感知)."""
    for edge in bridge_edges:
        _draw_wrapped_line(
            ax, edge.source_config, edge.target_config, period,
            color='lime', linewidth=2.0, alpha=0.9, zorder=5)


def _draw_bridge_boxes(ax, bridge_boxes):
    """高亮桥接 box."""
    for bb in bridge_boxes:
        lo_x, hi_x = bb.joint_intervals[0]
        lo_y, hi_y = bb.joint_intervals[1]
        rect = Rectangle((lo_x, lo_y), hi_x - lo_x, hi_y - lo_y,
                          linewidth=2.0, edgecolor='lime',
                          facecolor='lime', alpha=0.35, zorder=4)
        ax.add_patch(rect)


def _draw_path(ax, waypoints, color='#00ff00', lw=2.5, label='Path',
               zorder=15, period=None):
    """绘制路径 (周期感知)."""
    # 画节点 markers
    xs = [w[0] for w in waypoints]
    ys = [w[1] for w in waypoints]
    ax.plot(xs, ys, 'o', color=color, markersize=4,
            markeredgecolor='black', markeredgewidth=0.5, zorder=zorder + 1)
    # 画线段 (每段独立, 检测 wrap)
    first_seg = True
    for i in range(len(waypoints) - 1):
        kw = dict(color=color, linewidth=lw, alpha=0.9, zorder=zorder)
        if first_seg:
            kw['label'] = label
            first_seg = False
        _draw_wrapped_line(ax, waypoints[i], waypoints[i + 1], period, **kw)


def _draw_box_sequence(ax, box_seq, boxes, alpha=0.25):
    """高亮 Dijkstra 选出的 box 序列."""
    for bid in box_seq:
        if bid not in boxes:
            continue
        box = boxes[bid]
        lo_x, hi_x = box.joint_intervals[0]
        lo_y, hi_y = box.joint_intervals[1]
        rect = Rectangle((lo_x, lo_y), hi_x - lo_x, hi_y - lo_y,
                          linewidth=1.5, edgecolor='#00ccff',
                          facecolor='#00ccff', alpha=alpha, zorder=8)
        ax.add_patch(rect)


# ═══════════════════════════════════════════════════════════════════════════
# Geodesic Dijkstra (for periodic C-space)
# ═══════════════════════════════════════════════════════════════════════════

def _dijkstra_box_graph_geodesic(boxes, adj, src, tgt, period):
    """Dijkstra on box graph using boundary-aware geodesic weights.

    改进: 边权使用 box 边界最小距离而非中心距离,
    避免大 box 的中心距离过长导致路径绕行.
    相邻/重叠的 box 使用 30% 折扣的中心距离.
    """
    import heapq
    half = period / 2.0

    # 预计算 box 边界和中心
    box_lo, box_hi, centers = {}, {}, {}
    for bid, box in boxes.items():
        lo = np.array([lo_d for lo_d, hi_d in box.joint_intervals])
        hi = np.array([hi_d for lo_d, hi_d in box.joint_intervals])
        box_lo[bid] = lo
        box_hi[bid] = hi
        centers[bid] = (lo + hi) * 0.5

    def _edge_weight(u, v):
        """边权: 边界最小 geodesic 距离, 相邻时用折扣中心距."""
        lo_u, hi_u = box_lo[u], box_hi[u]
        lo_v, hi_v = box_lo[v], box_hi[v]
        ndim = len(lo_u)
        gap_sq = 0.0
        for d in range(ndim):
            # 直接重叠检测
            if lo_u[d] <= hi_v[d] + 1e-9 and lo_v[d] <= hi_u[d] + 1e-9:
                continue  # 该维度重叠, gap=0
            # 周期间隙: 两个方向取最小
            g1 = (lo_v[d] - hi_u[d]) % period
            g2 = (lo_u[d] - hi_v[d]) % period
            g = min(g1, g2)
            if g > half:
                g = period - g
            gap_sq += g * g
        surface_dist = np.sqrt(gap_sq)
        if surface_dist > 1e-10:
            return surface_dist
        # 相邻/重叠 → 30% 折扣中心距离, 避免大 box 被过度惩罚
        diff = ((centers[v] - centers[u]) + half) % period - half
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


def _overlap_bounds_periodic(box_a, box_b, period):
    """计算两个相邻 box 在周期空间中的交集区间.

    Returns (lo_array, hi_array).  若某维度仅 touch 不重叠,
    lo == hi 退化为一点.
    """
    ndim = len(box_a.joint_intervals)
    lo_out = np.empty(ndim)
    hi_out = np.empty(ndim)
    eps = 1e-9
    for d, ((lo1, hi1), (lo2, hi2)) in enumerate(
            zip(box_a.joint_intervals, box_b.joint_intervals)):
        best_lo, best_hi, best_width = lo1, hi1, -1.0
        for offset in [0.0, period, -period]:
            ov_lo = max(lo1, lo2 + offset)
            ov_hi = min(hi1, hi2 + offset)
            if ov_hi >= ov_lo - eps:
                w = max(0.0, ov_hi - ov_lo)
                if w > best_width:
                    best_lo, best_hi = ov_lo, max(ov_lo, ov_hi)
                    best_width = w
        if best_width < 0:
            # 退化: 取两 box 最近边界点
            best_lo = best_hi = (hi1 + lo2) / 2.0
        lo_out[d] = best_lo
        hi_out[d] = best_hi
    return lo_out, hi_out


def _shortcut_box_sequence(box_seq, adj):
    """贪心跳跃: 跳过可直接相邻到达的中间 box, 缩短 box 序列."""
    if len(box_seq) <= 2:
        return box_seq
    n = len(box_seq)
    result = [box_seq[0]]
    i = 0
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


def _pull_tight_in_boxes(waypoints, wp_bounds, period, n_iters=30):
    """迭代拉紧: 将每个内部 waypoint 拉向邻居连线上, 约束在其 bounds 内.

    wp_bounds: list of (lo_array, hi_array) — 每个 waypoint 的可行域.
    对于 transition 点, bounds 为两个相邻 box 的交集, 保证段落不出 box.
    """
    if len(waypoints) <= 2:
        return waypoints[:]

    wps = [w.copy() for w in waypoints]
    half = period / 2.0

    for _it in range(n_iters):
        max_move = 0.0
        for i in range(1, len(wps) - 1):
            prev, nxt = wps[i - 1], wps[i + 1]
            # geodesic 方向: prev → nxt
            diff = ((nxt - prev) + half) % period - half
            seg_len_sq = float(np.dot(diff, diff))
            if seg_len_sq < 1e-20:
                target = prev.copy()
            else:
                # 将当前点投影到 prev→nxt geodesic 线段上
                d_cur = ((wps[i] - prev) + half) % period - half
                t = float(np.dot(d_cur, diff)) / seg_len_sq
                t = np.clip(t, 0.0, 1.0)
                target = prev + t * diff
                target = ((target + half) % period) - half

            # clip 到 bounds
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


def _segment_in_box_periodic(p_a, p_b, box, period, n_samples=8):
    """检查线段 p_a → p_b (geodesic) 是否完全在 box 内."""
    half = period / 2.0
    diff = ((p_b - p_a) + half) % period - half
    for t in np.linspace(0.0, 1.0, n_samples):
        pt = p_a + t * diff
        pt = ((pt + half) % period) - half
        for d, (lo, hi) in enumerate(box.joint_intervals):
            if pt[d] < lo - 1e-9 or pt[d] > hi + 1e-9:
                return False
    return True


def _shortcut_waypoints_in_boxes(waypoints, boxes, box_seq, period):
    """贪心路径缩短: 跳过中间 waypoint, 仅当线段完全在某 box 内时才跳过.

    保证: 缩短后每个线段都在某个 box 内 → 无碰撞保证.
    """
    if len(waypoints) <= 2:
        return waypoints[:]
    # 收集 box_seq 中所有 box
    unique_boxes = [boxes[bid] for bid in box_seq if bid in boxes]
    n = len(waypoints)
    result = [waypoints[0]]
    i = 0
    while i < n - 1:
        farthest = i + 1
        for j in range(n - 1, i + 1, -1):
            # 检查线段 waypoints[i] → waypoints[j] 是否完全在某个 box 内
            for box in unique_boxes:
                if _segment_in_box_periodic(
                        waypoints[i], waypoints[j], box, period):
                    farthest = j
                    break
            if farthest == j:
                break
        result.append(waypoints[farthest])
        i = farthest
    return result


def _setup_ax(ax, extent, title=""):
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_xlabel("q₀ (rad)", fontsize=9)
    ax.set_ylabel("q₁ (rad)", fontsize=9)
    ax.set_title(title, fontsize=10, fontweight='bold')
    ax.set_aspect("equal")
    ax.grid(False)


# ═══════════════════════════════════════════════════════════════════════════
# GIF composition
# ═══════════════════════════════════════════════════════════════════════════

def compose_gif(frames_dir, gif_path, duration_ms=300):
    frame_paths = sorted(Path(frames_dir).glob("step_*.png"))
    if not frame_paths:
        return False
    try:
        from PIL import Image
    except ImportError:
        print("  [WARNING] Pillow not installed, GIF not generated")
        return False

    images = [Image.open(p).convert("P", palette=Image.ADAPTIVE)
              for p in frame_paths]
    durations = [duration_ms] * len(images)
    durations[-1] = 2000  # 最后一帧停留 2s
    images[0].save(gif_path, save_all=True, append_images=images[1:],
                   duration=durations, loop=0, optimize=False)
    for img in images:
        img.close()
    return True


# ═══════════════════════════════════════════════════════════════════════════
# Main pipeline
# ═══════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="2DOF 完整流程可视化")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--obstacles", type=int, default=8)
    args = parser.parse_args()

    cfg = Viz2DConfig()
    cfg.seed = args.seed
    cfg.n_obstacles = args.obstacles

    rng = np.random.default_rng(cfg.seed)
    robot = load_robot(cfg.robot_name)
    q_start = np.array(cfg.q_start, dtype=np.float64)
    q_goal = np.array(cfg.q_goal, dtype=np.float64)
    ndim = 2

    from datetime import datetime
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = _ROOT / "output" / f"viz_2dof_pipeline_{ts}"
    out_dir.mkdir(parents=True, exist_ok=True)
    frames_dir = out_dir / "frames"
    frames_dir.mkdir(exist_ok=True)

    print("=" * 60)
    print(f"  2DOF Full Pipeline Visualization")
    print(f"  seed={cfg.seed}, obstacles={cfg.n_obstacles}")
    print("=" * 60)

    # ══════════════════════════════════════════════════════════════
    # Phase 0: 场景 & 碰撞底图
    # ══════════════════════════════════════════════════════════════
    print("\n[Phase 0] Building scene & collision map ...")
    jl = robot.joint_limits[0]
    period = float(jl[1] - jl[0])  # 2π
    scene = build_random_2d_scene(robot, q_start, q_goal, rng, cfg,
                                  period=period)
    t0 = time.perf_counter()
    cmap_data, extent = scan_collision_map(
        robot, scene, robot.joint_limits, cfg.collision_resolution)
    cmap_ms = (time.perf_counter() - t0) * 1000
    print(f"  Collision map: {cmap_data.shape}, {cmap_ms:.0f}ms")

    # 保存碰撞底图
    fig0, ax0 = plt.subplots(1, 1, figsize=(8, 7))
    _draw_collision_bg(ax0, cmap_data, extent)
    _draw_start_goal(ax0, q_start, q_goal)
    ax0.legend(loc="upper right", fontsize=8)

    # 绘制 C-space 障碍物边界
    obs_list = scene.get_obstacles()
    for obs in obs_list:
        lo_x, lo_y = obs.min_point[0], obs.min_point[1]
        hi_x, hi_y = obs.max_point[0], obs.max_point[1]
        rect = Rectangle((lo_x, lo_y), hi_x - lo_x, hi_y - lo_y,
                          linewidth=0.8, edgecolor='red',
                          facecolor='none', linestyle='--', alpha=0.5)
        ax0.add_patch(rect)

    _setup_ax(ax0, extent, f"Phase 0: C-space Collision Map ({cfg.n_obstacles} obs)")
    fig0.tight_layout()
    fig0.savefig(out_dir / "phase0_collision_map.png", dpi=cfg.dpi)
    plt.close(fig0)
    print(f"  Saved: phase0_collision_map.png")

    # ══════════════════════════════════════════════════════════════
    # Phase 1: Forest grow (with snapshots)
    # ══════════════════════════════════════════════════════════════
    print("\n[Phase 1] Growing forest ...")
    planner_cfg = PandaGCSConfig()
    planner_cfg.seed = cfg.seed
    planner_cfg.max_boxes = cfg.max_boxes
    planner_cfg.max_consecutive_miss = cfg.max_consecutive_miss
    planner_cfg.guided_sample_ratio = cfg.guided_sample_ratio

    sbf_config = make_planner_config(planner_cfg)
    planner = SBFPlanner(robot=robot, scene=scene, config=sbf_config,
                         no_cache=True)  # 2D 不需要大缓存

    t0 = time.perf_counter()
    snapshots, forest_obj = grow_forest_with_snapshots(
        planner, q_start, q_goal,
        seed=cfg.seed, max_miss=cfg.max_consecutive_miss,
        max_boxes=cfg.max_boxes,
        snapshot_every=cfg.snapshot_every,
    )
    grow_ms = (time.perf_counter() - t0) * 1000
    n_before_coarsen = len(forest_obj.boxes)
    print(f"  Grown: {n_before_coarsen} boxes, {len(snapshots)} snapshots, "
          f"{grow_ms:.0f}ms")

    # 渲染每帧
    print("  Rendering growth frames ...")
    for idx, (n_boxes, boxes_snap, new_id) in enumerate(snapshots):
        fig, ax = plt.subplots(1, 1, figsize=(8, 7))
        _draw_collision_bg(ax, cmap_data, extent)
        _draw_boxes(ax, boxes_snap, highlight_ids={new_id} if new_id >= 0 else set())
        _draw_start_goal(ax, q_start, q_goal)
        _setup_ax(ax, extent,
                  f"Phase 1: Forest Growth — {n_boxes} boxes")
        fig.tight_layout()
        fig.savefig(frames_dir / f"step_{idx:04d}.png", dpi=cfg.dpi)
        plt.close(fig)

    # GIF
    gif_path = out_dir / "phase1_growth.gif"
    gif_ok = compose_gif(frames_dir, gif_path, cfg.gif_frame_ms)
    print(f"  GIF: {'OK' if gif_ok else 'FAILED'} ({len(snapshots)} frames)")

    # 最终 forest 静态图
    final_boxes_pre = snapshots[-1][1] if snapshots else {}
    fig1, ax1 = plt.subplots(1, 1, figsize=(8, 7))
    _draw_collision_bg(ax1, cmap_data, extent)
    _draw_boxes(ax1, final_boxes_pre, color_by="volume")
    _draw_start_goal(ax1, q_start, q_goal)
    ax1.legend(loc="upper right", fontsize=8)
    _setup_ax(ax1, extent,
              f"Phase 1: Forest Complete — {n_before_coarsen} boxes")
    fig1.tight_layout()
    fig1.savefig(out_dir / "phase1_forest_final.png", dpi=cfg.dpi)
    plt.close(fig1)

    # ══════════════════════════════════════════════════════════════
    # Phase 2: Coarsen
    # ══════════════════════════════════════════════════════════════
    print("\n[Phase 2] Coarsening ...")
    t0 = time.perf_counter()
    coarsen_stats = coarsen_forest(
        tree=planner.hier_tree, forest=forest_obj,
        obstacles=planner.obstacles, safety_margin=0.0,
        max_rounds=cfg.coarsen_max_rounds)
    coarsen_ms = (time.perf_counter() - t0) * 1000
    n_after_coarsen = len(forest_obj.boxes)
    print(f"  Coarsen: {n_before_coarsen} → {n_after_coarsen} boxes "
          f"({coarsen_stats.n_merges} merges, {coarsen_ms:.0f}ms)")

    boxes_after_coarsen = {bid: BoxNode(
        node_id=b.node_id,
        joint_intervals=[tuple(iv) for iv in b.joint_intervals],
        seed_config=b.seed_config.copy() if b.seed_config is not None else None,
        volume=b.volume)
        for bid, b in forest_obj.boxes.items()}

    # Before/After coarsen 对比图
    fig2, (ax2a, ax2b) = plt.subplots(1, 2, figsize=(16, 7))
    _draw_collision_bg(ax2a, cmap_data, extent)
    _draw_boxes(ax2a, final_boxes_pre, color_by="volume")
    _draw_start_goal(ax2a, q_start, q_goal)
    _setup_ax(ax2a, extent, f"Before Coarsen — {n_before_coarsen} boxes")

    _draw_collision_bg(ax2b, cmap_data, extent)
    _draw_boxes(ax2b, boxes_after_coarsen, color_by="volume")
    _draw_start_goal(ax2b, q_start, q_goal)
    _setup_ax(ax2b, extent, f"After Coarsen — {n_after_coarsen} boxes")

    fig2.suptitle("Phase 2: Coarsen", fontsize=12, fontweight='bold')
    fig2.tight_layout()
    fig2.savefig(out_dir / "phase2_coarsen.png", dpi=cfg.dpi)
    plt.close(fig2)
    print(f"  Saved: phase2_coarsen.png")

    # ══════════════════════════════════════════════════════════════
    # Phase 3: Adjacency + Islands + Bridge
    # ══════════════════════════════════════════════════════════════
    print("\n[Phase 3] Adjacency, islands & bridging ...")
    boxes = forest_obj.boxes

    # 周期 (所有 revolute joint 假设同周期)
    jl = robot.joint_limits[0]
    period = float(jl[1] - jl[0])  # 2π

    adj, uf, islands = _build_adjacency_and_islands(boxes, period=period)
    n_islands_before = len(islands)
    print(f"  Islands: {n_islands_before}, boxes: {len(boxes)}")

    src = find_box_containing(q_start, boxes)
    tgt = find_box_containing(q_goal, boxes)

    bridge_edges = []
    bridge_boxes_list = []

    if src is not None and tgt is not None and not uf.same(src, tgt):
        print(f"  s-t disconnected, bridging ...")
        t0 = time.perf_counter()
        bridge_result = bridge_islands(
            boxes=boxes,
            collision_checker=planner.collision_checker,
            segment_resolution=0.03,
            max_pairs_per_island_pair=10,
            max_rounds=5,
            period=period,
            hier_tree=planner.hier_tree,
            obstacles=planner.obstacles,
            forest=forest_obj,
            n_bridge_seeds=7,
            min_island_size=cfg.min_island_size,
            precomputed_uf=uf,
            precomputed_islands=islands,
            target_pair=(src, tgt),
        )
        bridge_edges, final_islands, _, bridge_boxes_list, discarded = bridge_result
        bridge_ms = (time.perf_counter() - t0) * 1000
        n_islands_after = len(final_islands)
        print(f"  Bridge: {n_islands_before} → {n_islands_after} islands, "
              f"{len(bridge_edges)} edges, {len(bridge_boxes_list)} boxes "
              f"({bridge_ms:.0f}ms)")

        # 重建 adj
        boxes = forest_obj.boxes
        adj, uf, islands = _build_adjacency_and_islands(boxes, period=period)
        # add bridge edges
        for e in bridge_edges:
            s_bid = find_box_containing(e.source_config, boxes)
            t_bid = find_box_containing(e.target_config, boxes)
            if s_bid is not None and t_bid is not None:
                adj.setdefault(s_bid, set()).add(t_bid)
                adj.setdefault(t_bid, set()).add(s_bid)
                uf.union(s_bid, t_bid)
    else:
        print(f"  s-t already connected (or not found)")
        islands = [set(boxes.keys())]

    # Island map
    fig3, ax3 = plt.subplots(1, 1, figsize=(8, 7))
    _draw_collision_bg(ax3, cmap_data, extent)
    _draw_islands(ax3, boxes, islands)
    _draw_bridge_boxes(ax3, bridge_boxes_list)
    _draw_bridge_edges(ax3, bridge_edges, period=period)
    _draw_start_goal(ax3, q_start, q_goal)
    ax3.legend(loc="upper right", fontsize=8)
    _setup_ax(ax3, extent,
              f"Phase 3: Islands & Bridge — {len(islands)} islands, "
              f"{len(bridge_edges)} bridges")
    fig3.tight_layout()
    fig3.savefig(out_dir / "phase3_islands_bridge.png", dpi=cfg.dpi)
    plt.close(fig3)
    print(f"  Saved: phase3_islands_bridge.png")

    # ══════════════════════════════════════════════════════════════
    # Phase 4: Path planning (Dijkstra)
    # ══════════════════════════════════════════════════════════════
    print("\n[Phase 4] Path planning (Dijkstra) ...")
    src = find_box_containing(q_start, boxes)
    tgt = find_box_containing(q_goal, boxes)

    path_found = False
    waypoints = []
    box_seq = []
    raw_dist = 0.0
    refined_cost = 0.0

    # 构建 bridge edge 的 box-pair 映射, 用于 Dijkstra 后插入桥接 waypoint
    bridge_edge_map = {}  # (bid_a, bid_b) -> Edge
    for e in bridge_edges:
        s_bid = find_box_containing(e.source_config, boxes)
        t_bid = find_box_containing(e.target_config, boxes)
        if s_bid is not None and t_bid is not None:
            bridge_edge_map[(s_bid, t_bid)] = e
            bridge_edge_map[(t_bid, s_bid)] = e

    if src is not None and tgt is not None:
        # ── 直连检测: geodesic 路径无碰撞则跳过 Dijkstra ──
        _cc = planner.collision_checker
        if not _cc.check_segment_collision(q_start, q_goal,
                                           resolution=0.03, period=period):
            half_p = period / 2.0
            diff_geo = ((q_goal - q_start) + half_p) % period - half_p
            direct_cost = float(np.linalg.norm(diff_geo))
            waypoints_raw = [q_start.copy(), q_goal.copy()]
            waypoints_before = [q_start.copy(), q_goal.copy()]
            waypoints = [q_start.copy(), q_goal.copy()]
            raw_wp_cost = direct_cost
            tight_cost = direct_cost
            refined_cost = direct_cost
            box_seq = [src] if src == tgt else [src, tgt]
            path_found = True
            print(f"  Direct geodesic connect OK! cost={direct_cost:.4f}")
        # ── 否则使用 geodesic 距离的 Dijkstra ──
        else:
            box_seq_result, raw_dist = _dijkstra_box_graph_geodesic(
                boxes, adj, src, tgt, period)
            if box_seq_result is not None:
                box_seq_raw = box_seq_result
                n_raw = len(box_seq_raw)

                # ── 贪心跳跃: 跳过可直达的中间 box ──
                box_seq = _shortcut_box_sequence(box_seq_raw, adj)
                n_short = len(box_seq)
                if n_short < n_raw:
                    print(f"  Box shortcut: {n_raw} → {n_short} boxes")

                # ── 构建 waypoints: 使用 box 交集中心作为转折点 ──
                # 保证每段线段的两个端点都在同一个凸 box 内 → 段落无碰撞
                half_p = period / 2.0
                raw_waypoints = [q_start.copy()]
                # bounds: 每个 waypoint 的可行域 (lo, hi)
                box_src = boxes[box_seq[0]]
                wp_bounds = [(np.array([lo for lo, hi in box_src.joint_intervals]),
                              np.array([hi for lo, hi in box_src.joint_intervals]))]

                for k in range(len(box_seq) - 1):
                    bid_cur = box_seq[k]
                    bid_nxt = box_seq[k + 1]
                    # 检查是否为桥接边
                    be = bridge_edge_map.get((bid_cur, bid_nxt))
                    if be is not None:
                        # 桥接边: 插入经过碰撞检测的端点 (已验证无碰撞)
                        s_bid = find_box_containing(be.source_config, boxes)
                        if s_bid == bid_cur:
                            raw_waypoints.append(be.source_config.copy())
                            b = boxes[bid_cur]
                            wp_bounds.append((
                                np.array([lo for lo, hi in b.joint_intervals]),
                                np.array([hi for lo, hi in b.joint_intervals])))
                            raw_waypoints.append(be.target_config.copy())
                            b = boxes[bid_nxt]
                            wp_bounds.append((
                                np.array([lo for lo, hi in b.joint_intervals]),
                                np.array([hi for lo, hi in b.joint_intervals])))
                        else:
                            raw_waypoints.append(be.target_config.copy())
                            b = boxes[bid_nxt]
                            wp_bounds.append((
                                np.array([lo for lo, hi in b.joint_intervals]),
                                np.array([hi for lo, hi in b.joint_intervals])))
                            raw_waypoints.append(be.source_config.copy())
                            b = boxes[bid_cur]
                            wp_bounds.append((
                                np.array([lo for lo, hi in b.joint_intervals]),
                                np.array([hi for lo, hi in b.joint_intervals])))
                    else:
                        # 普通相邻: 在两 box 交集中心放置转折点
                        ov_lo, ov_hi = _overlap_bounds_periodic(
                            boxes[bid_cur], boxes[bid_nxt], period)
                        transition = (ov_lo + ov_hi) / 2.0
                        # 归一化到 [-π, π]
                        transition = ((transition + half_p) % period) - half_p
                        raw_waypoints.append(transition)
                        wp_bounds.append((ov_lo, ov_hi))

                raw_waypoints.append(q_goal.copy())
                box_tgt = boxes[box_seq[-1]]
                wp_bounds.append((
                    np.array([lo for lo, hi in box_tgt.joint_intervals]),
                    np.array([hi for lo, hi in box_tgt.joint_intervals])))

                # 计算 raw waypoint 路径长度 (pull-tight 前)
                half_p = period / 2.0
                raw_wp_cost = 0.0
                for k in range(len(raw_waypoints) - 1):
                    diff_geo = ((raw_waypoints[k+1] - raw_waypoints[k])
                                + half_p) % period - half_p
                    raw_wp_cost += float(np.linalg.norm(diff_geo))

                # ── Pull-tight: 在 bounds 约束内拉紧路径 ──
                tight_waypoints = _pull_tight_in_boxes(
                    raw_waypoints, wp_bounds, period, n_iters=30)

                # 计算 geodesic 路径长度 (pull-tight 后)
                tight_cost = 0.0
                for k in range(len(tight_waypoints) - 1):
                    diff_geo = ((tight_waypoints[k+1] - tight_waypoints[k])
                                + half_p) % period - half_p
                    tight_cost += float(np.linalg.norm(diff_geo))

                # ── Post-process: 贪心路径缩短 (仅当线段在 box 内) ──
                smooth_waypoints = _shortcut_waypoints_in_boxes(
                    tight_waypoints, boxes, box_seq, period)

                refined_cost = 0.0
                for k in range(len(smooth_waypoints) - 1):
                    diff_geo = ((smooth_waypoints[k+1] - smooth_waypoints[k])
                                + half_p) % period - half_p
                    refined_cost += float(np.linalg.norm(diff_geo))

                waypoints_raw = [w.copy() for w in raw_waypoints]  # 保存 pull-tight 前
                waypoints_before = tight_waypoints  # pull-tight 后
                waypoints = smooth_waypoints
                path_found = True
                print(f"  Path found: {n_raw} boxes → {n_short} (shortcut)")
                print(f"    raw_dist={raw_dist:.3f}, "
                      f"raw_wp={raw_wp_cost:.3f} ({len(raw_waypoints)} wp), "
                      f"tight={tight_cost:.3f}, "
                      f"final={refined_cost:.3f} ({len(smooth_waypoints)} wp)")
            else:
                waypoints_raw = []
                waypoints_before = []
                print(f"  Dijkstra: no path found (disconnected)")
    else:
        waypoints_raw = []
        waypoints_before = []
        print(f"  Start or goal not in any box "
              f"(src={src}, tgt={tgt})")

    # Path visualization
    fig4, ax4 = plt.subplots(1, 1, figsize=(8, 7))
    _draw_collision_bg(ax4, cmap_data, extent)
    _draw_boxes(ax4, boxes, color_by="uniform", alpha=0.15, lw=0.3)
    if box_seq:
        _draw_box_sequence(ax4, box_seq, boxes, alpha=0.30)
    if waypoints_raw:
        _draw_path(ax4, waypoints_raw, color='#ff4444', lw=1.2,
                   label=f'Raw transition ({len(waypoints_raw)} wp, '
                         f'cost={raw_wp_cost:.2f})',
                   period=period)
    if waypoints_before:
        _draw_path(ax4, waypoints_before, color='#ffaa00', lw=1.8,
                   label=f'Pull-tight ({len(waypoints_before)} wp, '
                         f'cost={tight_cost:.2f})',
                   period=period)
    if waypoints:
        _draw_path(ax4, waypoints, color='#00ff00', lw=2.5,
                   label=f'Final ({len(waypoints)} wp, '
                         f'cost={refined_cost:.2f})',
                   period=period)
    _draw_start_goal(ax4, q_start, q_goal)
    ax4.legend(loc="upper right", fontsize=8)
    status = f"cost={refined_cost:.3f}" if path_found else "NO PATH"
    _setup_ax(ax4, extent,
              f"Phase 4: Path Planning — {len(box_seq)} boxes, {status}")
    fig4.tight_layout()
    fig4.savefig(out_dir / "phase4_path.png", dpi=cfg.dpi)
    plt.close(fig4)
    print(f"  Saved: phase4_path.png")

    # ══════════════════════════════════════════════════════════════
    # Phase 5: Overview (all-in-one)
    # ══════════════════════════════════════════════════════════════
    print("\n[Phase 5] Generating overview ...")
    fig5, axes = plt.subplots(2, 3, figsize=(22, 14))

    # (0,0) collision map
    ax = axes[0, 0]
    _draw_collision_bg(ax, cmap_data, extent)
    _draw_start_goal(ax, q_start, q_goal)
    _setup_ax(ax, extent, f"① C-space ({cfg.n_obstacles} obs)")

    # (0,1) forest before coarsen
    ax = axes[0, 1]
    _draw_collision_bg(ax, cmap_data, extent)
    _draw_boxes(ax, final_boxes_pre, color_by="volume")
    _draw_start_goal(ax, q_start, q_goal)
    _setup_ax(ax, extent, f"② Forest — {n_before_coarsen} boxes")

    # (0,2) after coarsen
    ax = axes[0, 2]
    _draw_collision_bg(ax, cmap_data, extent)
    _draw_boxes(ax, boxes_after_coarsen, color_by="volume")
    _draw_start_goal(ax, q_start, q_goal)
    _setup_ax(ax, extent, f"③ Coarsen — {n_after_coarsen} boxes")

    # (1,0) islands
    ax = axes[1, 0]
    _draw_collision_bg(ax, cmap_data, extent)
    _draw_islands(ax, boxes, islands)
    _draw_bridge_boxes(ax, bridge_boxes_list)
    _draw_bridge_edges(ax, bridge_edges, period=period)
    _draw_start_goal(ax, q_start, q_goal)
    _setup_ax(ax, extent, f"④ Islands & Bridge — {len(islands)} isl")

    # (1,1) path with boxes (raw + pull-tight + final)
    ax = axes[1, 1]
    _draw_collision_bg(ax, cmap_data, extent)
    _draw_boxes(ax, boxes, color_by="uniform", alpha=0.12, lw=0.2)
    if box_seq:
        _draw_box_sequence(ax, box_seq, boxes, alpha=0.35)
    if waypoints_raw:
        _draw_path(ax, waypoints_raw, color='#ff4444', lw=1.0,
                   label='Raw', period=period)
    if waypoints_before:
        _draw_path(ax, waypoints_before, color='#ffaa00', lw=1.5,
                   label='Pull-tight', period=period)
    if waypoints:
        _draw_path(ax, waypoints, color='#00ff00', lw=2.5,
                   label='Final', period=period)
    _draw_start_goal(ax, q_start, q_goal)
    ax.legend(loc='upper right', fontsize=7)
    _setup_ax(ax, extent,
              f"⑤ Path — {len(box_seq)} boxes, {status}")

    # (1,2) clean path only
    ax = axes[1, 2]
    _draw_collision_bg(ax, cmap_data, extent)
    if waypoints:
        _draw_path(ax, waypoints, color='#00ff00', lw=3.0,
                   label=f'Path (cost={refined_cost:.2f})',
                   period=period)
    _draw_start_goal(ax, q_start, q_goal)
    ax.legend(loc="upper right", fontsize=8)
    _setup_ax(ax, extent, f"⑥ Final Path")

    fig5.suptitle(
        f"SBF 2DOF Pipeline — seed={cfg.seed}, "
        f"{cfg.n_obstacles} obs, {n_after_coarsen} boxes, "
        f"grow={grow_ms:.0f}ms, coarsen={coarsen_ms:.0f}ms",
        fontsize=13, fontweight='bold')
    fig5.tight_layout(rect=[0, 0, 1, 0.96])
    fig5.savefig(out_dir / "phase5_overview.png", dpi=cfg.dpi)
    plt.close(fig5)
    print(f"  Saved: phase5_overview.png")

    # ══════════════════════════════════════════════════════════════
    # Summary
    # ══════════════════════════════════════════════════════════════
    summary = {
        "seed": cfg.seed,
        "n_obstacles": cfg.n_obstacles,
        "boxes_before_coarsen": n_before_coarsen,
        "boxes_after_coarsen": n_after_coarsen,
        "coarsen_merges": coarsen_stats.n_merges,
        "n_islands": len(islands),
        "n_bridge_edges": len(bridge_edges),
        "n_bridge_boxes": len(bridge_boxes_list),
        "path_found": path_found,
        "path_cost": float(refined_cost) if path_found else None,
        "path_waypoints_pre": len(waypoints_before) if path_found else 0,
        "path_waypoints_post": len(waypoints) if path_found else 0,
        "path_boxes": len(box_seq),
        "grow_ms": grow_ms,
        "coarsen_ms": coarsen_ms,
        "collision_map_ms": cmap_ms,
        "n_snapshots": len(snapshots),
    }
    with open(out_dir / "summary.json", "w") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)

    print(f"\n{'=' * 60}")
    print(f"  Output: {out_dir}")
    print(f"  Files:")
    for p in sorted(out_dir.glob("*.png")) + sorted(out_dir.glob("*.gif")):
        print(f"    {p.name}")
    print(f"  Summary: summary.json")
    print(f"  Path: {'FOUND' if path_found else 'NOT FOUND'}"
          f"{f', cost={refined_cost:.3f}' if path_found else ''}")
    print(f"{'=' * 60}")


if __name__ == "__main__":
    main()

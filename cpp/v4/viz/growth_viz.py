from __future__ import annotations

import argparse
import importlib
import json
import math
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple, Union

import matplotlib

matplotlib.use("Agg")

import matplotlib.colors as mcolors
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle

_PI = math.pi


@dataclass
class WorkspaceObstacle:
    min_point: np.ndarray
    max_point: np.ndarray
    name: str = ""

    def __post_init__(self) -> None:
        self.min_point = np.asarray(self.min_point, dtype=float)
        self.max_point = np.asarray(self.max_point, dtype=float)

    def to_json(self) -> Dict[str, object]:
        return {
            "min": self.min_point.tolist(),
            "max": self.max_point.tolist(),
            "name": self.name,
        }


def build_random_obstacles_2d(
    n_obs: int = 5,
    seed: int = 42,
    ws_range: float = 1.6,
    size_lo: float = 0.18,
    size_hi: float = 0.42,
    z_half: float = 0.5,
    base_clearance: float = 0.45,
) -> List[WorkspaceObstacle]:
    rng = np.random.default_rng(seed)
    obstacles: List[WorkspaceObstacle] = []
    attempts = 0
    while len(obstacles) < n_obs:
        attempts += 1
        if attempts > 2000:
            raise RuntimeError("Unable to place the requested number of obstacles")

        cx = float(rng.uniform(-ws_range, ws_range))
        cy = float(rng.uniform(-ws_range, ws_range))
        hw = float(rng.uniform(size_lo, size_hi) / 2.0)
        hh = float(rng.uniform(size_lo, size_hi) / 2.0)
        if math.hypot(cx, cy) < base_clearance + max(hw, hh):
            continue

        obstacles.append(
            WorkspaceObstacle(
                min_point=np.array([cx - hw, cy - hh, -z_half], dtype=float),
                max_point=np.array([cx + hw, cy + hh, z_half], dtype=float),
                name=f"obs_{len(obstacles)}",
            )
        )
    return obstacles


def fk_2dof(q1: float, q2: float, link_1: float = 1.0, link_2: float = 1.0) -> np.ndarray:
    x1 = link_1 * math.cos(q1)
    y1 = link_1 * math.sin(q1)
    x2 = x1 + link_2 * math.cos(q1 + q2)
    y2 = y1 + link_2 * math.sin(q1 + q2)
    return np.array([[0.0, 0.0], [x1, y1], [x2, y2]], dtype=float)


def _seg_intersects_aabb_2d(p0: np.ndarray, p1: np.ndarray, bmin: np.ndarray, bmax: np.ndarray) -> bool:
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, p0[0] - bmin[0]), (dx, bmax[0] - p0[0]), (-dy, p0[1] - bmin[1]), (dy, bmax[1] - p0[1])):
        if abs(p) < 1e-15:
            if q < 0.0:
                return False
        else:
            r = q / p
            if p < 0.0:
                t0 = max(t0, r)
            else:
                t1 = min(t1, r)
            if t0 > t1:
                return False
    return True


def config_collides_2d(
    q1: float,
    q2: float,
    obstacles: Sequence[WorkspaceObstacle],
    link_1: float,
    link_2: float,
) -> bool:
    pts = fk_2dof(q1, q2, link_1, link_2)
    for obs in obstacles:
        bmin = obs.min_point[:2]
        bmax = obs.max_point[:2]
        if _seg_intersects_aabb_2d(pts[0], pts[1], bmin, bmax):
            return True
        if _seg_intersects_aabb_2d(pts[1], pts[2], bmin, bmax):
            return True
    return False


def scan_collision_map_2d(
    obstacles: Sequence[WorkspaceObstacle],
    resolution: float = 0.04,
    link_1: float = 1.0,
    link_2: float = 1.0,
) -> Tuple[np.ndarray, List[float]]:
    qs = np.arange(-_PI, _PI, resolution, dtype=float)
    cmap = np.zeros((len(qs), len(qs)), dtype=np.float32)
    for i, q2_val in enumerate(qs):
        for j, q1_val in enumerate(qs):
            if config_collides_2d(float(q1_val), float(q2_val), obstacles, link_1, link_2):
                cmap[i, j] = 1.0
    return cmap, [-_PI, _PI, -_PI, _PI]


def setup_cspace_ax(ax, title: str = "") -> None:
    ax.set_xlim(-_PI, _PI)
    ax.set_ylim(-_PI, _PI)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=11, fontweight="bold", pad=8)
    ax.set_xlabel("q1")
    ax.set_ylabel("q2")
    ax.grid(True, alpha=0.1)


def setup_workspace_ax(ax, title: str = "", ws_lim: float = 2.3) -> None:
    ax.set_title(title, fontsize=11, fontweight="bold", pad=8)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_aspect("equal")
    ax.set_xlim(-ws_lim, ws_lim)
    ax.set_ylim(-ws_lim, ws_lim)
    ax.grid(True, alpha=0.12)


def draw_collision_bg(ax, cmap_data: np.ndarray, extent: Sequence[float], alpha: float = 0.25) -> None:
    ax.imshow(cmap_data, origin="lower", extent=extent, cmap="Reds", alpha=alpha, aspect="auto")


def draw_boxes(
    ax,
    boxes: Sequence[object],
    alpha: float = 0.30,
    cmap_name: str = "viridis",
    highlight_ids: Optional[Sequence[int]] = None,
    highlight_color: str = "#ff4444",
    highlight_alpha: float = 0.6,
) -> None:
    highlight = set(highlight_ids or [])
    cmap_c = plt.colormaps.get_cmap(cmap_name)
    vols = [float(box.volume) for box in boxes] if boxes else [0.0, 1.0]
    vmin, vmax = min(vols), max(vols)
    if vmax - vmin < 1e-15:
        vmax = vmin + 1.0
    norm = mcolors.Normalize(vmin=vmin, vmax=vmax)
    for box in boxes:
        lo_x = float(box.joint_intervals[0].lo)
        hi_x = float(box.joint_intervals[0].hi)
        lo_y = float(box.joint_intervals[1].lo)
        hi_y = float(box.joint_intervals[1].hi)
        if int(box.id) in highlight:
            edge_color = highlight_color
            face_color = highlight_color
            rect_alpha = highlight_alpha
            line_width = 1.8
        else:
            color = cmap_c(norm(float(box.volume)))
            width = hi_x - lo_x
            edge_color = color
            face_color = color
            rect_alpha = alpha
            line_width = max(1.2, min(2.5, 0.15 / max(width, 1e-6)))
        rect = Rectangle(
            (lo_x, lo_y),
            hi_x - lo_x,
            hi_y - lo_y,
            linewidth=line_width,
            edgecolor=edge_color,
            facecolor=face_color,
            alpha=rect_alpha,
        )
        ax.add_patch(rect)


def draw_workspace_obstacles(ax, obstacles: Sequence[WorkspaceObstacle]) -> None:
    for obs in obstacles:
        width = float(obs.max_point[0] - obs.min_point[0])
        height = float(obs.max_point[1] - obs.min_point[1])
        ax.add_patch(
            Rectangle(
                (float(obs.min_point[0]), float(obs.min_point[1])),
                width,
                height,
                facecolor="#9e9e9e",
                edgecolor="#616161",
                alpha=0.35,
                linewidth=1.0,
            )
        )


def draw_arm(
    ax,
    q1: float,
    q2: float,
    link_1: float = 1.0,
    link_2: float = 1.0,
    color: str = "cyan",
    line_width: float = 2.2,
    marker_size: int = 4,
    zorder: int = 15,
) -> None:
    pts = fk_2dof(q1, q2, link_1, link_2)
    ax.plot(
        pts[:, 0],
        pts[:, 1],
        "-o",
        color=color,
        linewidth=line_width,
        markersize=marker_size,
        markeredgecolor="black",
        zorder=zorder,
    )


def _sample_configs_from_boxes(rng: np.random.Generator, boxes: Sequence[object], n: int) -> np.ndarray:
    if not boxes:
        return np.zeros((0, 2), dtype=float)
    vols = np.array([max(float(box.volume), 1e-12) for box in boxes], dtype=float)
    probs = vols / vols.sum()
    idx = rng.choice(len(boxes), size=n, p=probs)
    configs = np.empty((n, 2), dtype=float)
    for k, box_idx in enumerate(idx):
        box = boxes[int(box_idx)]
        configs[k, 0] = float(rng.uniform(box.joint_intervals[0].lo, box.joint_intervals[0].hi))
        configs[k, 1] = float(rng.uniform(box.joint_intervals[1].lo, box.joint_intervals[1].hi))
    return configs


def _geo_diff(a: np.ndarray, b: np.ndarray, period: float) -> np.ndarray:
    half = period / 2.0
    return ((b - a) + half) % period - half


def _geo_dist(a: np.ndarray, b: np.ndarray, period: float) -> float:
    return float(np.linalg.norm(_geo_diff(a, b, period)))


def _find_box_containing(q: np.ndarray, boxes: Sequence[object]) -> Optional[int]:
    for box in boxes:
        if box.joint_intervals[0].lo <= q[0] <= box.joint_intervals[0].hi and box.joint_intervals[1].lo <= q[1] <= box.joint_intervals[1].hi:
            return int(box.id)
    return None


def _shortcut_seq(seq: List[int], adj: Dict[int, List[int]]) -> List[int]:
    if len(seq) <= 2:
        return seq
    adj_set = {key: set(values) for key, values in adj.items()}
    result = [seq[0]]
    i = 0
    while i < len(seq) - 1:
        farthest = i + 1
        for j in range(len(seq) - 1, i + 1, -1):
            if seq[j] in adj_set.get(seq[i], set()):
                farthest = j
                break
        result.append(seq[farthest])
        i = farthest
    return result


def _overlap_bounds(box_a: object, box_b: object, period: float) -> Tuple[np.ndarray, np.ndarray]:
    lo_out = np.empty(2, dtype=float)
    hi_out = np.empty(2, dtype=float)
    for dim in range(2):
        lo1 = float(box_a.joint_intervals[dim].lo)
        hi1 = float(box_a.joint_intervals[dim].hi)
        lo2 = float(box_b.joint_intervals[dim].lo)
        hi2 = float(box_b.joint_intervals[dim].hi)
        best_lo, best_hi, best_width = lo1, hi1, -1.0
        for offset in (0.0, period, -period):
            ov_lo = max(lo1, lo2 + offset)
            ov_hi = min(hi1, hi2 + offset)
            if ov_hi >= ov_lo - 1e-9:
                width = max(0.0, ov_hi - ov_lo)
                if width > best_width:
                    best_lo = ov_lo
                    best_hi = max(ov_lo, ov_hi)
                    best_width = width
        if best_width < 0.0:
            best_lo = best_hi = (hi1 + lo2) / 2.0
        lo_out[dim] = best_lo
        hi_out[dim] = best_hi
    return lo_out, hi_out


def _build_waypoints(q_start: np.ndarray, q_goal: np.ndarray, seq: Sequence[int], box_map: Dict[int, object], period: float) -> List[np.ndarray]:
    half = period / 2.0
    waypoints = [q_start.copy()]
    for idx in range(len(seq) - 1):
        ov_lo, ov_hi = _overlap_bounds(box_map[seq[idx]], box_map[seq[idx + 1]], period)
        trans = ((ov_lo + ov_hi) / 2.0 + half) % period - half
        waypoints.append(trans)
    waypoints.append(q_goal.copy())
    return waypoints


def _pull_tight(
    waypoints: Sequence[np.ndarray],
    seq: Sequence[int],
    box_map: Dict[int, object],
    period: float,
    n_iters: int = 30,
) -> List[np.ndarray]:
    if len(waypoints) <= 2:
        return [point.copy() for point in waypoints]

    half = period / 2.0
    waypoint_bounds: List[Tuple[np.ndarray, np.ndarray]] = []
    first_box = box_map[seq[0]]
    waypoint_bounds.append(
        (
            np.array([float(first_box.joint_intervals[dim].lo) for dim in range(2)], dtype=float),
            np.array([float(first_box.joint_intervals[dim].hi) for dim in range(2)], dtype=float),
        )
    )
    for idx in range(len(seq) - 1):
        waypoint_bounds.append(_overlap_bounds(box_map[seq[idx]], box_map[seq[idx + 1]], period))
    last_box = box_map[seq[-1]]
    waypoint_bounds.append(
        (
            np.array([float(last_box.joint_intervals[dim].lo) for dim in range(2)], dtype=float),
            np.array([float(last_box.joint_intervals[dim].hi) for dim in range(2)], dtype=float),
        )
    )

    wps = [point.copy() for point in waypoints]
    for _ in range(n_iters):
        max_move = 0.0
        for idx in range(1, len(wps) - 1):
            prev = wps[idx - 1]
            nxt = wps[idx + 1]
            diff = ((nxt - prev) + half) % period - half
            seg_sq = float(np.dot(diff, diff))
            if seg_sq < 1e-20:
                target = prev.copy()
            else:
                cur_diff = ((wps[idx] - prev) + half) % period - half
                ratio = np.clip(float(np.dot(cur_diff, diff)) / seg_sq, 0.0, 1.0)
                target = prev + ratio * diff
                target = ((target + half) % period) - half
            lo_b, hi_b = waypoint_bounds[idx]
            target = np.clip(target, lo_b, hi_b)
            move = float(np.linalg.norm(target - wps[idx]))
            max_move = max(max_move, move)
            wps[idx] = target
        if max_move < 1e-8:
            break
    return wps


def _resample_path(waypoints: Sequence[np.ndarray], period: float, ds: float = 0.05) -> List[np.ndarray]:
    half = period / 2.0
    pts = [waypoints[0].copy()]
    for idx in range(len(waypoints) - 1):
        diff = ((waypoints[idx + 1] - waypoints[idx]) + half) % period - half
        seg_len = float(np.linalg.norm(diff))
        n_seg = max(1, int(math.ceil(seg_len / ds)))
        for step in range(1, n_seg + 1):
            t = step / n_seg
            point = waypoints[idx] + t * diff
            point = ((point + half) % period) - half
            pts.append(point)
    return pts


def dijkstra_box_path(
    boxes: Sequence[object],
    adj: Dict[int, List[int]],
    q_start: np.ndarray,
    q_goal: np.ndarray,
    period: float,
) -> Tuple[Optional[List[int]], Optional[List[np.ndarray]]]:
    import heapq

    box_map = {int(box.id): box for box in boxes}
    src = _find_box_containing(q_start, boxes)
    tgt = _find_box_containing(q_goal, boxes)
    if src is None or tgt is None:
        return None, None

    def center(box_id: int) -> np.ndarray:
        box = box_map[box_id]
        return np.array(
            [
                0.5 * (float(box.joint_intervals[0].lo) + float(box.joint_intervals[0].hi)),
                0.5 * (float(box.joint_intervals[1].lo) + float(box.joint_intervals[1].hi)),
            ],
            dtype=float,
        )

    def edge_weight(u: int, v: int) -> float:
        return max(_geo_dist(center(u), center(v), period), 1e-12)

    dist_map = {box_id: float("inf") for box_id in box_map}
    prev_map: Dict[int, Optional[int]] = {box_id: None for box_id in box_map}
    dist_map[src] = 0.0
    heap: List[Tuple[float, int]] = [(0.0, src)]
    while heap:
        cur_dist, node = heapq.heappop(heap)
        if cur_dist > dist_map[node]:
            continue
        if node == tgt:
            break
        for neigh in adj.get(node, []):
            next_dist = cur_dist + edge_weight(node, neigh)
            if next_dist < dist_map[neigh]:
                dist_map[neigh] = next_dist
                prev_map[neigh] = node
                heapq.heappush(heap, (next_dist, neigh))
    if dist_map[tgt] == float("inf"):
        return None, None

    seq: List[int] = []
    cur = tgt
    while cur is not None:
        seq.append(cur)
        cur = prev_map[cur]
    seq.reverse()
    seq = _shortcut_seq(seq, adj)
    waypoints = _build_waypoints(q_start, q_goal, seq, box_map, period)
    waypoints = _pull_tight(waypoints, seq, box_map, period)
    return seq, waypoints


def connected_components(boxes: Sequence[object], adj: Dict[int, List[int]]) -> List[List[int]]:
    box_ids = {int(box.id) for box in boxes}
    visited = set()
    components: List[List[int]] = []
    for start in sorted(box_ids):
        if start in visited:
            continue
        stack = [start]
        component: List[int] = []
        visited.add(start)
        while stack:
            node = stack.pop()
            component.append(node)
            for neigh in adj.get(node, []):
                if neigh in box_ids and neigh not in visited:
                    visited.add(neigh)
                    stack.append(neigh)
        components.append(component)
    return components


def _center_of_box(box: object) -> np.ndarray:
    return np.array(
        [
            0.5 * (float(box.joint_intervals[0].lo) + float(box.joint_intervals[0].hi)),
            0.5 * (float(box.joint_intervals[1].lo) + float(box.joint_intervals[1].hi)),
        ],
        dtype=float,
    )


def select_start_goal_from_component(
    boxes: Sequence[object],
    component_ids: Sequence[int],
    period: float,
) -> Tuple[np.ndarray, np.ndarray, List[int]]:
    box_map = {int(box.id): box for box in boxes}
    ids = [box_id for box_id in component_ids if box_id in box_map]
    if not ids:
        raise RuntimeError("Largest component is empty")
    if len(ids) == 1:
        center = _center_of_box(box_map[ids[0]])
        return center.copy(), center.copy(), ids

    best_pair = (ids[0], ids[1])
    best_dist = -1.0
    centers = {box_id: _center_of_box(box_map[box_id]) for box_id in ids}
    for i, left in enumerate(ids[:-1]):
        for right in ids[i + 1 :]:
            dist = _geo_dist(centers[left], centers[right], period)
            if dist > best_dist:
                best_dist = dist
                best_pair = (left, right)
    return centers[best_pair[0]], centers[best_pair[1]], [best_pair[0], best_pair[1]]


def render_growth_gif(
    boxes: Sequence[object],
    obstacles: Sequence[WorkspaceObstacle],
    cmap_data: np.ndarray,
    extent: Sequence[float],
    q_start: np.ndarray,
    q_goal: np.ndarray,
    out_path: Union[str, Path],
    *,
    link_1: float = 1.0,
    link_2: float = 1.0,
    n_frames: int = 40,
    n_arm_samples: int = 12,
    fps: int = 7,
    dpi: int = 120,
    seed: int = 0,
) -> None:
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    total_boxes = len(boxes)
    if total_boxes == 0:
        return
    stride = max(1, total_boxes // n_frames)
    frame_sizes = list(range(stride, total_boxes + 1, stride))
    if frame_sizes[-1] != total_boxes:
        frame_sizes.append(total_boxes)

    rng = np.random.default_rng(seed)
    fig, (ax_cs, ax_ws) = plt.subplots(1, 2, figsize=(14, 6), gridspec_kw={"width_ratios": [1, 1]})
    fig.patch.set_facecolor("white")

    legend_cs = [
        Rectangle((0, 0), 1, 1, facecolor="#4CAF50", alpha=0.3, edgecolor="#4CAF50", label="Safe boxes"),
        Rectangle((0, 0), 1, 1, facecolor="#d32f2f", alpha=0.26, edgecolor="none", label="C-obstacle"),
        Line2D([], [], marker="o", color="cyan", markersize=6, markeredgecolor="black", linestyle="none", label="Start"),
        Line2D([], [], marker="*", color="yellow", markersize=10, markeredgecolor="black", linestyle="none", label="Goal"),
    ]
    legend_ws = [
        mpatches.Patch(facecolor="#9e9e9e", alpha=0.35, label="Obstacle"),
        Line2D([], [], color="#1E88E5", linewidth=2, label="Sampled arms"),
        Line2D([], [], color="cyan", linewidth=2.5, linestyle="--", label="Start"),
        Line2D([], [], color="#FFD600", linewidth=2.5, linestyle="--", label="Goal"),
    ]

    def render(frame_idx: int) -> None:
        n_show = frame_sizes[frame_idx]
        subset = boxes[:n_show]

        ax_cs.clear()
        setup_cspace_ax(ax_cs, f"C-space: {n_show} / {total_boxes} boxes")
        draw_collision_bg(ax_cs, cmap_data, extent, alpha=0.26)
        draw_boxes(ax_cs, subset, alpha=0.50)
        ax_cs.plot(q_start[0], q_start[1], "o", color="cyan", markersize=8, markeredgecolor="black", zorder=20)
        ax_cs.plot(q_goal[0], q_goal[1], "*", color="yellow", markersize=12, markeredgecolor="black", zorder=20)
        ax_cs.legend(handles=legend_cs, loc="lower right", fontsize=8, framealpha=0.92)

        ax_ws.clear()
        setup_workspace_ax(ax_ws, f"Workspace: sampled from {n_show} boxes")
        draw_workspace_obstacles(ax_ws, obstacles)
        draw_arm(ax_ws, q_start[0], q_start[1], link_1=link_1, link_2=link_2, color="cyan", line_width=2.0, marker_size=3, zorder=10)
        draw_arm(ax_ws, q_goal[0], q_goal[1], link_1=link_1, link_2=link_2, color="#FFD600", line_width=2.0, marker_size=3, zorder=10)

        configs = _sample_configs_from_boxes(rng, subset, n_arm_samples)
        for cfg in configs:
            draw_arm(ax_ws, float(cfg[0]), float(cfg[1]), link_1=link_1, link_2=link_2, color="#1E88E5", line_width=1.0, marker_size=2, zorder=5)
        ax_ws.plot(0.0, 0.0, "ko", markersize=7, zorder=18)
        ax_ws.legend(handles=legend_ws, loc="upper right", fontsize=7.5, framealpha=0.9)

    anim = FuncAnimation(fig, render, frames=len(frame_sizes), interval=1000 // fps, repeat=True)
    anim.save(str(out_path), writer=PillowWriter(fps=fps), dpi=dpi)
    plt.close(fig)


def render_execution_gif(
    boxes: Sequence[object],
    waypoints: Sequence[np.ndarray],
    obstacles: Sequence[WorkspaceObstacle],
    cmap_data: np.ndarray,
    extent: Sequence[float],
    q_start: np.ndarray,
    q_goal: np.ndarray,
    out_path: Union[str, Path],
    *,
    period: float = 2.0 * _PI,
    link_1: float = 1.0,
    link_2: float = 1.0,
    fps: int = 12,
    dpi: int = 120,
    ds: float = 0.06,
    max_gif_frames: int = 80,
) -> None:
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    dense = _resample_path(waypoints, period, ds=ds)
    n_pts = len(dense)
    if n_pts > max_gif_frames:
        stride = max(1, n_pts // max_gif_frames)
        indices = list(range(0, n_pts, stride))
        if indices[-1] != n_pts - 1:
            indices.append(n_pts - 1)
    else:
        indices = list(range(n_pts))

    fk_all = [fk_2dof(float(cfg[0]), float(cfg[1]), link_1, link_2) for cfg in dense]
    ee_all = np.array([pts[-1] for pts in fk_all], dtype=float)

    fig, (ax_cs, ax_ws) = plt.subplots(1, 2, figsize=(14, 6), gridspec_kw={"width_ratios": [1, 1]})
    fig.patch.set_facecolor("white")

    def render(frame_idx: int) -> None:
        dense_idx = indices[frame_idx]

        ax_cs.clear()
        setup_cspace_ax(ax_cs, f"C-space path {frame_idx + 1}/{len(indices)}")
        draw_collision_bg(ax_cs, cmap_data, extent, alpha=0.24)
        draw_boxes(ax_cs, boxes, alpha=0.25)
        xs = [point[0] for point in dense[: dense_idx + 1]]
        ys = [point[1] for point in dense[: dense_idx + 1]]
        ax_cs.plot(xs, ys, "-", color="#00ff00", linewidth=2.0, alpha=0.8, zorder=12)
        ax_cs.plot(dense[dense_idx][0], dense[dense_idx][1], "o", color="#E53935", markersize=8, markeredgecolor="black", zorder=25)
        ax_cs.plot(q_start[0], q_start[1], "o", color="cyan", markersize=8, markeredgecolor="black", zorder=20)
        ax_cs.plot(q_goal[0], q_goal[1], "*", color="yellow", markersize=12, markeredgecolor="black", zorder=20)

        ax_ws.clear()
        setup_workspace_ax(ax_ws, "Workspace execution")
        draw_workspace_obstacles(ax_ws, obstacles)
        ax_ws.plot(ee_all[:, 0], ee_all[:, 1], "-", color="#4CAF50", alpha=0.25, linewidth=1.5, zorder=2)
        ax_ws.plot(ee_all[: dense_idx + 1, 0], ee_all[: dense_idx + 1, 1], "-", color="#4CAF50", alpha=0.6, linewidth=2.0, zorder=4)

        step = max(1, len(indices) // 12)
        for ghost_idx in range(0, frame_idx, step):
            pts = fk_all[indices[ghost_idx]]
            ax_ws.plot(pts[:, 0], pts[:, 1], "-", color="#90CAF9", alpha=0.2, linewidth=1.0, zorder=3)

        pts = fk_all[dense_idx]
        ax_ws.plot(pts[:, 0], pts[:, 1], "-o", color="#1565C0", linewidth=3.0, markersize=6, markeredgecolor="black", zorder=15)
        ax_ws.plot(pts[-1, 0], pts[-1, 1], "o", color="#E53935", markersize=8, markeredgecolor="black", zorder=16)
        draw_arm(ax_ws, q_start[0], q_start[1], link_1=link_1, link_2=link_2, color="cyan", line_width=1.5, marker_size=3, zorder=5)
        draw_arm(ax_ws, q_goal[0], q_goal[1], link_1=link_1, link_2=link_2, color="#FFD600", line_width=1.5, marker_size=3, zorder=5)
        ax_ws.plot(0.0, 0.0, "ko", markersize=7, zorder=18)

    anim = FuncAnimation(fig, render, frames=len(indices), interval=1000 // fps, repeat=True)
    anim.save(str(out_path), writer=PillowWriter(fps=fps), dpi=dpi)
    plt.close(fig)


def _load_cpp_module(repo_root: Path):
    python_dir = repo_root / "python"
    if str(python_dir) not in sys.path:
        sys.path.insert(0, str(python_dir))
    try:
        return importlib.import_module("_sbf4_cpp")
    except ImportError as exc:
        raise SystemExit(
            "Unable to import _sbf4_cpp. Build the v4 Python bindings first (SBF_WITH_PYTHON=ON)."
        ) from exc


def _normalize_adjacency(adj_like: object) -> Dict[int, List[int]]:
    raw = dict(adj_like)
    return {int(key): [int(value) for value in values] for key, values in raw.items()}


def _box_to_json(box: object) -> Dict[str, object]:
    return {
        "id": int(box.id),
        "root_id": int(getattr(box, "root_id", -1)),
        "tree_id": int(getattr(box, "tree_id", -1)),
        "parent_box_id": int(getattr(box, "parent_box_id", -1)),
        "volume": float(box.volume),
        "joint_intervals": [
            {"lo": float(box.joint_intervals[0].lo), "hi": float(box.joint_intervals[0].hi)},
            {"lo": float(box.joint_intervals[1].lo), "hi": float(box.joint_intervals[1].hi)},
        ],
        "seed_config": np.asarray(box.seed_config, dtype=float).tolist(),
    }


def _resolve_output_dir(repo_root: Path, output_dir: Optional[Union[str, Path]], run_name: str) -> Path:
    if output_dir is not None:
        resolved = Path(output_dir).expanduser()
        if not resolved.is_absolute():
            resolved = repo_root / resolved
        return resolved
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return repo_root / "results" / f"{run_name}_{stamp}"


def _resolve_pipeline_config(cpp, pipeline_preset: str):
    preset_name = str(pipeline_preset).strip()
    preset_key = preset_name.lower()

    if preset_key == "fast":
        return preset_name, cpp.PipelineConfig.fast(), "IFK", "LinkIAABB"

    if preset_key == "recommended":
        return preset_name, cpp.PipelineConfig.recommended(), "CritSample", "Hull16_Grid"

    pipeline = cpp.PipelineConfig()
    if preset_key == "tightest":
        pipeline.source = cpp.EndpointSourceConfig.analytical()
        pipeline.envelope = cpp.EnvelopeTypeConfig.hull16_grid()
        return preset_name, pipeline, "Analytical", "Hull16_Grid"

    if preset_key == "production":
        pipeline.source = cpp.EndpointSourceConfig.ifk()
        pipeline.envelope = cpp.EnvelopeTypeConfig.hull16_grid()
        return preset_name, pipeline, "IFK", "Hull16_Grid"

    raise ValueError(f"Unsupported pipeline preset: {pipeline_preset}")


def run_growth_visualization(
    repo_root: Union[str, Path],
    output_dir: Union[str, Path],
    *,
    grower_mode: str = "Wavefront",
    pipeline_preset: str = "fast",
    obstacle_count: int = 5,
    obstacle_seed: int = 42,
    grower_seed: int = 7,
    n_roots: int = 8,
    max_boxes: int = 700,
    min_edge: float = 0.18,
    boundary_samples: int = 24,
    collision_resolution: float = 0.04,
    growth_frames: int = 42,
    growth_arm_samples: int = 12,
    link_1: float = 1.0,
    link_2: float = 1.0,
) -> Dict[str, object]:
    repo_root = Path(repo_root).resolve()
    output_dir = Path(output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    cpp = _load_cpp_module(repo_root)
    robot_path = repo_root / "robots" / "2dof_planar" / "2dof_planar_dh.json"
    robot = cpp.Robot.from_json(str(robot_path))

    obstacles = build_random_obstacles_2d(n_obs=obstacle_count, seed=obstacle_seed)

    config = cpp.GrowerConfig()
    mode_name = str(grower_mode).strip()
    mode_key = mode_name.lower()
    mode_map = {
        "wavefront": cpp.GrowerConfig.Mode.Wavefront,
        "rrt": cpp.GrowerConfig.Mode.RRT,
    }
    if mode_key not in mode_map:
        raise ValueError(f"Unsupported grower_mode: {grower_mode}")
    config.mode = mode_map[mode_key]
    pipeline_name, pipeline_cfg, endpoint_source_name, envelope_name = _resolve_pipeline_config(cpp, pipeline_preset)
    config.n_roots = n_roots
    config.max_boxes = max_boxes
    config.min_edge = min_edge
    config.rng_seed = grower_seed
    config.n_boundary_samples = boundary_samples
    config.partition_mode = cpp.PartitionMode.LectAligned
    config.pipeline = pipeline_cfg

    grower = cpp.ForestGrower(robot, config)
    grower.set_split_order(cpp.SplitOrder.BEST_TIGHTEN)
    result = grower.grow(obstacles)

    boxes = sorted(list(result.boxes), key=lambda box: int(box.id))
    adjacency = _normalize_adjacency(grower.get_adjacency_dict())
    cmap_data, extent = scan_collision_map_2d(obstacles, resolution=collision_resolution, link_1=link_1, link_2=link_2)

    components = connected_components(boxes, adjacency)
    largest_component = max(components, key=len) if components else []
    q_start = None
    q_goal = None
    path_box_ids = None
    waypoints = None
    if largest_component:
        q_start, q_goal, _ = select_start_goal_from_component(boxes, largest_component, 2.0 * _PI)
        path_box_ids, waypoints = dijkstra_box_path(boxes, adjacency, q_start, q_goal, 2.0 * _PI)

    if boxes and q_start is not None and q_goal is not None:
        render_growth_gif(
            boxes,
            obstacles,
            cmap_data,
            extent,
            q_start,
            q_goal,
            output_dir / "forest_growth.gif",
            link_1=link_1,
            link_2=link_2,
            n_frames=growth_frames,
            n_arm_samples=growth_arm_samples,
            seed=grower_seed,
        )

    if boxes and q_start is not None and q_goal is not None and waypoints is not None:
        render_execution_gif(
            boxes,
            waypoints,
            obstacles,
            cmap_data,
            extent,
            q_start,
            q_goal,
            output_dir / "path_execution.gif",
            link_1=link_1,
            link_2=link_2,
        )

    summary = {
        "robot_json": str(robot_path.relative_to(repo_root)).replace("\\", "/"),
        "grower_mode": mode_name,
        "partition_mode": "LectAligned",
        "split_order": "BEST_TIGHTEN",
        "pipeline": f"PipelineConfig.{pipeline_name}()",
        "pipeline_preset": pipeline_name,
        "endpoint_source": endpoint_source_name,
        "envelope_type": envelope_name,
        "obstacle_seed": obstacle_seed,
        "grower_seed": grower_seed,
        "obstacle_count": obstacle_count,
        "n_roots": n_roots,
        "max_boxes": max_boxes,
        "min_edge": min_edge,
        "boundary_samples": boundary_samples,
        "n_boxes": len(boxes),
        "n_components": len(components),
        "largest_component_size": len(largest_component),
        "result": {
            "n_boxes_total": int(result.n_boxes_total),
            "n_roots": int(result.n_roots),
            "n_components": int(result.n_components),
            "n_ffb_success": int(result.n_ffb_success),
            "n_ffb_fail": int(result.n_ffb_fail),
            "n_promotions": int(result.n_promotions),
            "n_bridge_boxes": int(result.n_bridge_boxes),
            "n_coarse_boxes": int(result.n_coarse_boxes),
            "n_fine_boxes": int(result.n_fine_boxes),
            "n_coarsen_merges": int(result.n_coarsen_merges),
            "start_goal_connected": bool(result.start_goal_connected),
            "total_volume": float(result.total_volume),
            "build_time_ms": float(result.build_time_ms),
            "phase_times": {str(key): float(value) for key, value in dict(result.phase_times).items()},
        },
        "start": None if q_start is None else np.asarray(q_start, dtype=float).tolist(),
        "goal": None if q_goal is None else np.asarray(q_goal, dtype=float).tolist(),
        "path_box_ids": None if path_box_ids is None else [int(box_id) for box_id in path_box_ids],
        "waypoints": None if waypoints is None else [np.asarray(point, dtype=float).tolist() for point in waypoints],
        "outputs": {
            "growth_gif": "forest_growth.gif" if (output_dir / "forest_growth.gif").exists() else None,
            "execution_gif": "path_execution.gif" if (output_dir / "path_execution.gif").exists() else None,
            "boxes_json": "boxes.json",
            "adjacency_json": "adjacency.json",
            "obstacles_json": "obstacles.json",
            "collision_map": "collision_map.npy",
            "summary_json": "summary.json",
        },
    }

    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    (output_dir / "obstacles.json").write_text(
        json.dumps([obstacle.to_json() for obstacle in obstacles], indent=2),
        encoding="utf-8",
    )
    (output_dir / "boxes.json").write_text(json.dumps([_box_to_json(box) for box in boxes], indent=2), encoding="utf-8")
    (output_dir / "adjacency.json").write_text(
        json.dumps({str(key): values for key, values in adjacency.items()}, indent=2),
        encoding="utf-8",
    )
    (output_dir / "collision_meta.json").write_text(
        json.dumps({"extent": extent, "resolution": collision_resolution}, indent=2),
        encoding="utf-8",
    )
    np.save(output_dir / "collision_map.npy", cmap_data)

    return summary


def build_cli_parser(description: str) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--output-dir", type=str, default=None, help="Output directory. Defaults to results/<run_name>_<timestamp>.")
    parser.add_argument("--mode", type=str, choices=["Wavefront", "RRT"], default="Wavefront", help="ForestGrower mode for visualization.")
    parser.add_argument(
        "--pipeline-preset",
        type=str,
        choices=["fast", "recommended", "tightest", "production"],
        default="fast",
        help="Endpoint iAABB / envelope pipeline preset.",
    )
    parser.add_argument("--obstacle-count", type=int, default=5, help="Number of workspace obstacles.")
    parser.add_argument("--obstacle-seed", type=int, default=42, help="Random seed for obstacle generation.")
    parser.add_argument("--grower-seed", type=int, default=7, help="Random seed for the v4 grower.")
    parser.add_argument("--n-roots", type=int, default=8, help="ForestGrower root count.")
    parser.add_argument("--max-boxes", type=int, default=700, help="Maximum number of boxes.")
    parser.add_argument("--min-edge", type=float, default=0.18, help="Minimum edge length for box growth.")
    parser.add_argument("--boundary-samples", type=int, default=24, help="Boundary sample count for growth.")
    parser.add_argument("--collision-resolution", type=float, default=0.04, help="Resolution for the C-space collision heatmap.")
    parser.add_argument("--growth-frames", type=int, default=42, help="Frame count target for the growth GIF.")
    parser.add_argument("--growth-arm-samples", type=int, default=12, help="Workspace arm samples per growth frame.")
    return parser


def run_cli(run_name: str, description: str) -> Dict[str, object]:
    parser = build_cli_parser(description)
    args = parser.parse_args()
    repo_root = Path(__file__).resolve().parents[1]
    output_dir = _resolve_output_dir(repo_root, args.output_dir, run_name)
    summary = run_growth_visualization(
        repo_root,
        output_dir,
        grower_mode=args.mode,
        pipeline_preset=args.pipeline_preset,
        obstacle_count=args.obstacle_count,
        obstacle_seed=args.obstacle_seed,
        grower_seed=args.grower_seed,
        n_roots=args.n_roots,
        max_boxes=args.max_boxes,
        min_edge=args.min_edge,
        boundary_samples=args.boundary_samples,
        collision_resolution=args.collision_resolution,
        growth_frames=args.growth_frames,
        growth_arm_samples=args.growth_arm_samples,
    )
    print(f"Wrote 2DOF growth visualization outputs to: {output_dir}")
    return summary
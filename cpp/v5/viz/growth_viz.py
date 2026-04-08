"""2DOF planar grower visualization — interactive Plotly HTML.

Migrated from v4/viz/growth_viz.py.  Uses v5 _sbf5_cpp bindings
(SBFPlanner) for forest building and path planning.

Outputs:
  - forest_growth.html   : slider animation of box growth (C-space + workspace)
  - path_execution.html  : slider animation of path following
  - summary.json, boxes.json, obstacles.json

Usage (from v5/):
    $env:PYTHONPATH = "build_x64/Release;python"
    python viz/growth_viz.py --max-boxes 200 --output-dir results/demo
"""
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

import numpy as np

_PI = math.pi


def _cmap_to_b64_png(cmap_data: np.ndarray, color: Tuple[int, int, int] = (211, 47, 47),
                     alpha: float = 0.30) -> str:
    """Convert a collision bitmap to a base64-encoded PNG for layout.images."""
    import base64
    import io
    try:
        from PIL import Image
    except ImportError:
        # Fallback: encode via matplotlib
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        h, w = cmap_data.shape
        rgba = np.zeros((h, w, 4), dtype=np.uint8)
        mask = cmap_data > 0.5
        rgba[mask] = [color[0], color[1], color[2], int(alpha * 255)]
        fig_tmp, ax_tmp = plt.subplots(1, 1, figsize=(1, 1), dpi=max(h, w))
        ax_tmp.imshow(rgba, origin="lower", aspect="auto")
        ax_tmp.axis("off")
        buf = io.BytesIO()
        fig_tmp.savefig(buf, format="png", bbox_inches="tight", pad_inches=0, transparent=True)
        plt.close(fig_tmp)
        buf.seek(0)
        return "data:image/png;base64," + base64.b64encode(buf.read()).decode()

    h, w = cmap_data.shape
    rgba = np.zeros((h, w, 4), dtype=np.uint8)
    mask = cmap_data > 0.5
    rgba[mask] = [color[0], color[1], color[2], int(alpha * 255)]
    # Flip vertically so origin="lower" matches axis orientation
    rgba = rgba[::-1]
    img = Image.fromarray(rgba, "RGBA")
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    buf.seek(0)
    return "data:image/png;base64," + base64.b64encode(buf.read()).decode()


# ─── Data classes ────────────────────────────────────────────────────────────

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


# ─── Obstacle generation ────────────────────────────────────────────────────

def build_random_obstacles_2d(
    n_obs: int = 5,
    seed: int = 42,
    size_lo: float = 0.18,
    size_hi: float = 0.42,
    z_half: float = 0.5,
    base_clearance: float = 0.45,
    link_1: float = 1.0,
    link_2: float = 1.0,
) -> List[WorkspaceObstacle]:
    """Generate random obstacles in the 2-link arm workspace.

    The reachable workspace is an annulus from |L1-L2| to L1+L2 centered
    at the origin.  Obstacles are placed within this region, excluding a
    clearance zone around the base.
    """
    rng = np.random.default_rng(seed)
    ws_range = (link_1 + link_2) * 0.85
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
        # Keep within reachable workspace
        if math.hypot(cx, cy) > ws_range:
            continue

        obstacles.append(
            WorkspaceObstacle(
                min_point=np.array([cx - hw, cy - hh, -z_half], dtype=float),
                max_point=np.array([cx + hw, cy + hh, z_half], dtype=float),
                name=f"obs_{len(obstacles)}",
            )
        )
    return obstacles


# ─── 2DOF FK & collision helpers ─────────────────────────────────────────────

def fk_2dof(q1: float, q2: float, link_1: float = 1.0, link_2: float = 1.0) -> np.ndarray:
    """Forward kinematics for a 2-link planar arm (modified DH with tool frame).

    Returns the three key points: base (0,0), elbow, end-effector.
    """
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
    """Check collision for both active links of the 2-link planar arm."""
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


# ─── Box sampling helper ────────────────────────────────────────────────────

def _sample_configs_from_boxes(rng: np.random.Generator, boxes: Sequence[object], n: int) -> np.ndarray:
    if not boxes:
        return np.zeros((0, 2), dtype=float)
    vols = np.array([max(float(b.volume), 1e-12) for b in boxes], dtype=float)
    probs = vols / vols.sum()
    idx = rng.choice(len(boxes), size=n, p=probs)
    configs = np.empty((n, 2), dtype=float)
    for k, box_idx in enumerate(idx):
        box = boxes[int(box_idx)]
        ivs = box.joint_intervals
        configs[k, 0] = float(rng.uniform(ivs[0].lo, ivs[0].hi))
        configs[k, 1] = float(rng.uniform(ivs[1].lo, ivs[1].hi))
    return configs


# ─── Path resampling ────────────────────────────────────────────────────────

def _resample_path(waypoints: Sequence[np.ndarray], ds: float = 0.05) -> List[np.ndarray]:
    pts = [waypoints[0].copy()]
    for idx in range(len(waypoints) - 1):
        diff = waypoints[idx + 1] - waypoints[idx]
        seg_len = float(np.linalg.norm(diff))
        n_seg = max(1, int(math.ceil(seg_len / ds)))
        for step in range(1, n_seg + 1):
            t = step / n_seg
            pts.append(waypoints[idx] + t * diff)
    return pts


# ─── Plotly helpers ──────────────────────────────────────────────────────────

# Depth-based color palette (viridis-like, 16 discrete levels)
_DEPTH_COLORS = [
    (68, 1, 84),     # depth 0  — dark purple
    (72, 26, 108),   # depth 1
    (71, 47, 126),   # depth 2
    (65, 68, 135),   # depth 3
    (57, 86, 140),   # depth 4
    (47, 104, 142),  # depth 5
    (38, 121, 142),  # depth 6
    (31, 138, 141),  # depth 7
    (30, 155, 138),  # depth 8
    (42, 171, 127),  # depth 9
    (74, 187, 110),  # depth 10
    (110, 201, 88),  # depth 11
    (153, 213, 67),  # depth 12
    (194, 223, 35),  # depth 13
    (230, 231, 33),  # depth 14
    (253, 231, 37),  # depth 15 — bright yellow
]


def _box_depth(vol: float, max_vol: float) -> int:
    """Estimate LECT depth from volume ratio (each split halves one dimension)."""
    if max_vol <= 0 or vol <= 0:
        return 0
    ratio = max_vol / vol
    if ratio <= 1.0:
        return 0
    return min(int(round(math.log2(ratio))), len(_DEPTH_COLORS) - 1)


def _depth_fill_color(depth: int, alpha: float = 0.40) -> str:
    r, g, b = _DEPTH_COLORS[min(depth, len(_DEPTH_COLORS) - 1)]
    return f"rgba({r},{g},{b},{alpha})"


def _depth_line_color(depth: int) -> str:
    r, g, b = _DEPTH_COLORS[min(depth, len(_DEPTH_COLORS) - 1)]
    return f"rgb({r},{g},{b})"


def _group_boxes_by_depth(
    boxes: Sequence[object],
) -> Tuple[Dict[int, List[object]], int]:
    """Group boxes by LECT depth. Returns {depth: [box, ...]}, max_depth."""
    if not boxes:
        return {}, 0
    max_vol = max(float(b.volume) for b in boxes)
    groups: Dict[int, List[object]] = {}
    max_d = 0
    for b in boxes:
        d = _box_depth(float(b.volume), max_vol)
        groups.setdefault(d, []).append(b)
        max_d = max(max_d, d)
    return groups, max_d


def _boxes_to_rect_xy(boxes: Sequence[object]) -> Tuple[List[Optional[float]], List[Optional[float]]]:
    xs: List[Optional[float]] = []
    ys: List[Optional[float]] = []
    for box in boxes:
        ivs = box.joint_intervals
        lo_x, hi_x = float(ivs[0].lo), float(ivs[0].hi)
        lo_y, hi_y = float(ivs[1].lo), float(ivs[1].hi)
        xs.extend([lo_x, hi_x, hi_x, lo_x, lo_x, None])
        ys.extend([lo_y, lo_y, hi_y, hi_y, lo_y, None])
    return xs, ys


def _arm_line_xy(q1: float, q2: float, link_1: float, link_2: float) -> Tuple[List[float], List[float]]:
    pts = fk_2dof(q1, q2, link_1, link_2)
    return pts[:, 0].tolist(), pts[:, 1].tolist()


def _obs_rect_xy(obstacles: Sequence[WorkspaceObstacle]) -> Tuple[List[Optional[float]], List[Optional[float]]]:
    xs: List[Optional[float]] = []
    ys: List[Optional[float]] = []
    for obs in obstacles:
        lx, ly = float(obs.min_point[0]), float(obs.min_point[1])
        hx, hy = float(obs.max_point[0]), float(obs.max_point[1])
        xs.extend([lx, hx, hx, lx, lx, None])
        ys.extend([ly, ly, hy, hy, ly, None])
    return xs, ys


# ─── Plotly: growth animation ───────────────────────────────────────────────

def render_growth_html(
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
    seed: int = 0,
) -> None:
    from plotly.subplots import make_subplots
    import plotly.graph_objects as go

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

    # Pre-compute arm samples for each frame (fixed seed per frame for reproducibility)
    arm_samples_per_frame: List[np.ndarray] = []
    for n_show in frame_sizes:
        arm_samples_per_frame.append(
            _sample_configs_from_boxes(rng, boxes[:n_show], n_arm_samples)
        )

    # Build figure with subplots
    fig = make_subplots(
        rows=1, cols=2,
        subplot_titles=["C-space", "Workspace"],
        horizontal_spacing=0.08,
    )

    # ── Static traces (visible in all frames) ──

    # C-space collision map as layout image (survives animation redraw)
    cmap_b64 = _cmap_to_b64_png(cmap_data, color=(211, 47, 47), alpha=0.30)

    # Workspace obstacles
    obs_x, obs_y = _obs_rect_xy(obstacles)
    fig.add_trace(go.Scatter(
        x=obs_x, y=obs_y,
        fill="toself",
        fillcolor="rgba(158,158,158,0.35)",
        line=dict(color="rgb(97,97,97)", width=1),
        mode="lines",
        name="Obstacles",
        showlegend=True,
        hoverinfo="skip",
    ), row=1, col=2)

    # Start/goal arms in workspace
    sx, sy = _arm_line_xy(float(q_start[0]), float(q_start[1]), link_1, link_2)
    gx, gy = _arm_line_xy(float(q_goal[0]), float(q_goal[1]), link_1, link_2)
    fig.add_trace(go.Scatter(
        x=sx, y=sy, mode="lines+markers",
        line=dict(color="cyan", width=2.5, dash="dash"),
        marker=dict(size=5, color="cyan", line=dict(color="black", width=1)),
        name="Start arm", showlegend=True,
    ), row=1, col=2)
    fig.add_trace(go.Scatter(
        x=gx, y=gy, mode="lines+markers",
        line=dict(color="#FFD600", width=2.5, dash="dash"),
        marker=dict(size=5, color="#FFD600", line=dict(color="black", width=1)),
        name="Goal arm", showlegend=True,
    ), row=1, col=2)

    # Base joint marker
    fig.add_trace(go.Scatter(
        x=[0.0], y=[0.0], mode="markers",
        marker=dict(size=8, color="black"),
        showlegend=False, hoverinfo="skip",
    ), row=1, col=2)

    # Start/goal markers in C-space
    fig.add_trace(go.Scatter(
        x=[float(q_start[0])], y=[float(q_start[1])],
        mode="markers",
        marker=dict(size=10, color="cyan", symbol="circle",
                    line=dict(color="black", width=1.5)),
        name="Start", showlegend=True,
    ), row=1, col=1)
    fig.add_trace(go.Scatter(
        x=[float(q_goal[0])], y=[float(q_goal[1])],
        mode="markers",
        marker=dict(size=12, color="#FFD600", symbol="star",
                    line=dict(color="black", width=1.5)),
        name="Goal", showlegend=True,
    ), row=1, col=1)

    # ── Animated traces (updated per frame) ──
    # Determine depth levels present across all boxes
    all_groups, max_depth = _group_boxes_by_depth(boxes)
    depth_levels = sorted(all_groups.keys())
    n_depth = len(depth_levels)

    # One trace per depth level for C-space boxes
    depth_trace_start = len(fig.data)
    init_boxes = boxes[:frame_sizes[0]]
    init_groups, _ = _group_boxes_by_depth(init_boxes)
    for d in depth_levels:
        bx_d, by_d = _boxes_to_rect_xy(init_groups.get(d, []))
        fig.add_trace(go.Scatter(
            x=bx_d, y=by_d,
            fill="toself",
            fillcolor=_depth_fill_color(d, alpha=0.40),
            line=dict(color=_depth_line_color(d), width=0.8),
            mode="lines",
            name=f"depth {d}" if d == depth_levels[0] or d == depth_levels[-1] else None,
            showlegend=(d == depth_levels[0] or d == depth_levels[-1]),
            legendgroup="boxes",
            hoverinfo="skip",
        ), row=1, col=1)

    # Workspace sampled arms trace
    arm_trace_idx = len(fig.data)
    arm_xs0: List[Optional[float]] = []
    arm_ys0: List[Optional[float]] = []
    for cfg in arm_samples_per_frame[0]:
        ax, ay = _arm_line_xy(float(cfg[0]), float(cfg[1]), link_1, link_2)
        arm_xs0.extend(ax + [None])
        arm_ys0.extend(ay + [None])
    fig.add_trace(go.Scatter(
        x=arm_xs0, y=arm_ys0,
        mode="lines+markers",
        line=dict(color="rgb(30,136,229)", width=1.2),
        marker=dict(size=3, color="rgb(30,136,229)"),
        name="Sampled arms",
        showlegend=True,
    ), row=1, col=2)

    # ── Build frames ──
    frames = []
    animated_trace_ids = list(range(depth_trace_start, depth_trace_start + n_depth)) + [arm_trace_idx]

    for fi, n_show in enumerate(frame_sizes):
        frame_boxes = boxes[:n_show]
        frame_groups, _ = _group_boxes_by_depth(frame_boxes)

        # One scatter per depth level
        frame_data = []
        for d in depth_levels:
            bx_d, by_d = _boxes_to_rect_xy(frame_groups.get(d, []))
            frame_data.append(go.Scatter(x=bx_d, y=by_d))

        # Sampled arms
        arm_xs: List[Optional[float]] = []
        arm_ys: List[Optional[float]] = []
        for cfg in arm_samples_per_frame[fi]:
            ax, ay = _arm_line_xy(float(cfg[0]), float(cfg[1]), link_1, link_2)
            arm_xs.extend(ax + [None])
            arm_ys.extend(ay + [None])
        frame_data.append(go.Scatter(x=arm_xs, y=arm_ys))

        frames.append(go.Frame(
            data=frame_data,
            traces=animated_trace_ids,
            name=str(n_show),
        ))

    fig.frames = frames

    # ── Slider + buttons ──
    sliders = [dict(
        active=0,
        currentvalue=dict(prefix="Boxes: "),
        pad=dict(t=50),
        steps=[
            dict(args=[[str(n_show)], dict(frame=dict(duration=150, redraw=True), mode="immediate")],
                 label=str(n_show), method="animate")
            for n_show in frame_sizes
        ],
    )]

    fig.update_layout(
        sliders=sliders,
        updatemenus=[dict(
            type="buttons",
            showactive=False,
            y=1.05, x=0.0, xanchor="left",
            buttons=[
                dict(label="Play",
                     method="animate",
                     args=[None, dict(frame=dict(duration=200, redraw=True),
                                      fromcurrent=True, mode="immediate")]),
                dict(label="Pause",
                     method="animate",
                     args=[[None], dict(frame=dict(duration=0, redraw=False),
                                        mode="immediate")]),
            ],
        )],
        title=dict(text=f"Forest Growth — {total_boxes} boxes", x=0.5),
        height=550,
        width=1200,
        template="plotly_white",
        images=[dict(
            source=cmap_b64,
            xref="x", yref="y",
            x=extent[0], y=extent[3],
            sizex=extent[1] - extent[0],
            sizey=extent[3] - extent[2],
            sizing="stretch",
            layer="below",
        )],
    )

    # Axis settings
    fig.update_xaxes(title_text="q1", range=[-_PI, _PI], scaleanchor="y", row=1, col=1)
    fig.update_yaxes(title_text="q2", range=[-_PI, _PI], row=1, col=1)
    ws_lim = 2.3
    fig.update_xaxes(title_text="x", range=[-ws_lim, ws_lim], scaleanchor="y2", row=1, col=2)
    fig.update_yaxes(title_text="y", range=[-ws_lim, ws_lim], row=1, col=2)

    fig.write_html(str(out_path), include_plotlyjs="cdn")


# ─── Plotly: execution animation ────────────────────────────────────────────

def render_execution_html(
    boxes: Sequence[object],
    waypoints: Sequence[np.ndarray],
    obstacles: Sequence[WorkspaceObstacle],
    cmap_data: np.ndarray,
    extent: Sequence[float],
    q_start: np.ndarray,
    q_goal: np.ndarray,
    out_path: Union[str, Path],
    *,
    link_1: float = 1.0,
    link_2: float = 1.0,
    ds: float = 0.06,
    max_frames: int = 80,
) -> None:
    from plotly.subplots import make_subplots
    import plotly.graph_objects as go

    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    dense = _resample_path(waypoints, ds=ds)
    n_pts = len(dense)
    if n_pts > max_frames:
        step = max(1, n_pts // max_frames)
        indices = list(range(0, n_pts, step))
        if indices[-1] != n_pts - 1:
            indices.append(n_pts - 1)
    else:
        indices = list(range(n_pts))

    fk_all = [fk_2dof(float(cfg[0]), float(cfg[1]), link_1, link_2) for cfg in dense]
    ee_all = np.array([pts[-1] for pts in fk_all], dtype=float)

    # Ghost arm interval
    ghost_step = max(1, len(indices) // 12)

    fig = make_subplots(
        rows=1, cols=2,
        subplot_titles=["C-space path", "Workspace execution"],
        horizontal_spacing=0.08,
    )

    # ── Static traces ──

    # C-space collision map as layout image (survives animation redraw)
    cmap_b64 = _cmap_to_b64_png(cmap_data, color=(211, 47, 47), alpha=0.25)

    # C-space boxes (faded, static) — colored by depth
    exec_groups, _ = _group_boxes_by_depth(boxes)
    exec_depths = sorted(exec_groups.keys())
    for d in exec_depths:
        bx_d, by_d = _boxes_to_rect_xy(exec_groups[d])
        fig.add_trace(go.Scatter(
            x=bx_d, y=by_d,
            fill="toself",
            fillcolor=_depth_fill_color(d, alpha=0.22),
            line=dict(color=_depth_line_color(d), width=0.5),
            mode="lines",
            name="Safe boxes" if d == exec_depths[0] else None,
            showlegend=(d == exec_depths[0]),
            legendgroup="exec_boxes",
            hoverinfo="skip",
        ), row=1, col=1)

    # Full EE trail (faded) in workspace
    fig.add_trace(go.Scatter(
        x=ee_all[:, 0].tolist(), y=ee_all[:, 1].tolist(),
        mode="lines",
        line=dict(color="rgba(76,175,80,0.25)", width=1.5),
        name="EE trail (full)",
        showlegend=False, hoverinfo="skip",
    ), row=1, col=2)

    # Workspace obstacles
    obs_x, obs_y = _obs_rect_xy(obstacles)
    fig.add_trace(go.Scatter(
        x=obs_x, y=obs_y,
        fill="toself",
        fillcolor="rgba(158,158,158,0.35)",
        line=dict(color="rgb(97,97,97)", width=1),
        mode="lines",
        name="Obstacles",
        showlegend=True,
        hoverinfo="skip",
    ), row=1, col=2)

    # Start/goal arms
    sx, sy = _arm_line_xy(float(q_start[0]), float(q_start[1]), link_1, link_2)
    gx, gy = _arm_line_xy(float(q_goal[0]), float(q_goal[1]), link_1, link_2)
    fig.add_trace(go.Scatter(
        x=sx, y=sy, mode="lines+markers",
        line=dict(color="cyan", width=1.5, dash="dash"),
        marker=dict(size=3, color="cyan"),
        name="Start arm", showlegend=True,
    ), row=1, col=2)
    fig.add_trace(go.Scatter(
        x=gx, y=gy, mode="lines+markers",
        line=dict(color="#FFD600", width=1.5, dash="dash"),
        marker=dict(size=3, color="#FFD600"),
        name="Goal arm", showlegend=True,
    ), row=1, col=2)

    # Start/goal C-space markers
    fig.add_trace(go.Scatter(
        x=[float(q_start[0])], y=[float(q_start[1])],
        mode="markers",
        marker=dict(size=10, color="cyan", symbol="circle",
                    line=dict(color="black", width=1.5)),
        name="Start", showlegend=True,
    ), row=1, col=1)
    fig.add_trace(go.Scatter(
        x=[float(q_goal[0])], y=[float(q_goal[1])],
        mode="markers",
        marker=dict(size=12, color="#FFD600", symbol="star",
                    line=dict(color="black", width=1.5)),
        name="Goal", showlegend=True,
    ), row=1, col=1)

    # Base joint
    fig.add_trace(go.Scatter(
        x=[0.0], y=[0.0], mode="markers",
        marker=dict(size=8, color="black"),
        showlegend=False, hoverinfo="skip",
    ), row=1, col=2)

    n_static = len(fig.data)

    # ── Animated traces ──
    # trace N+0: C-space path line (grows)
    # trace N+1: C-space current position marker
    # trace N+2: Workspace EE trail (grows)
    # trace N+3: Workspace ghost arms
    # trace N+4: Workspace current arm

    di0 = indices[0]
    fig.add_trace(go.Scatter(
        x=[dense[0][0]], y=[dense[0][1]],
        mode="lines", line=dict(color="#00ff00", width=2.5),
        name="Path", showlegend=True,
    ), row=1, col=1)

    fig.add_trace(go.Scatter(
        x=[dense[di0][0]], y=[dense[di0][1]],
        mode="markers",
        marker=dict(size=10, color="#E53935", line=dict(color="black", width=1.5)),
        name="Current", showlegend=True,
    ), row=1, col=1)

    fig.add_trace(go.Scatter(
        x=[ee_all[0, 0]], y=[ee_all[0, 1]],
        mode="lines", line=dict(color="rgb(76,175,80)", width=2.0),
        name="EE trail", showlegend=True,
    ), row=1, col=2)

    # Ghost arms
    fig.add_trace(go.Scatter(
        x=[], y=[], mode="lines",
        line=dict(color="rgba(144,202,249,0.3)", width=1),
        name="Ghost arms", showlegend=False,
    ), row=1, col=2)

    # Current arm
    cur_ax, cur_ay = _arm_line_xy(float(dense[di0][0]), float(dense[di0][1]), link_1, link_2)
    fig.add_trace(go.Scatter(
        x=cur_ax, y=cur_ay,
        mode="lines+markers",
        line=dict(color="rgb(21,101,192)", width=3),
        marker=dict(size=6, color="rgb(21,101,192)", line=dict(color="black", width=1)),
        name="Current arm", showlegend=True,
    ), row=1, col=2)

    # ── Build frames ──
    frames = []
    for fi, idx in enumerate(indices):
        # C-space path so far
        path_xs = [float(dense[i][0]) for i in range(idx + 1)]
        path_ys = [float(dense[i][1]) for i in range(idx + 1)]

        # EE trail so far
        ee_xs = ee_all[:idx + 1, 0].tolist()
        ee_ys = ee_all[:idx + 1, 1].tolist()

        # Ghost arms
        ghost_xs: List[Optional[float]] = []
        ghost_ys: List[Optional[float]] = []
        for gi in range(0, fi, ghost_step):
            g_idx = indices[gi]
            gpts = fk_all[g_idx]
            ghost_xs.extend(gpts[:, 0].tolist() + [None])
            ghost_ys.extend(gpts[:, 1].tolist() + [None])

        # Current arm
        ca_x, ca_y = _arm_line_xy(float(dense[idx][0]), float(dense[idx][1]), link_1, link_2)

        frames.append(go.Frame(
            data=[
                go.Scatter(x=path_xs, y=path_ys),
                go.Scatter(x=[float(dense[idx][0])], y=[float(dense[idx][1])]),
                go.Scatter(x=ee_xs, y=ee_ys),
                go.Scatter(x=ghost_xs, y=ghost_ys),
                go.Scatter(x=ca_x, y=ca_y),
            ],
            traces=[n_static + i for i in range(5)],
            name=str(fi),
        ))

    fig.frames = frames

    # ── Slider + buttons ──
    sliders = [dict(
        active=0,
        currentvalue=dict(prefix="Step: "),
        pad=dict(t=50),
        steps=[
            dict(args=[[str(fi)], dict(frame=dict(duration=80, redraw=True), mode="immediate")],
                 label=str(fi + 1), method="animate")
            for fi in range(len(indices))
        ],
    )]

    fig.update_layout(
        sliders=sliders,
        updatemenus=[dict(
            type="buttons",
            showactive=False,
            y=1.05, x=0.0, xanchor="left",
            buttons=[
                dict(label="Play",
                     method="animate",
                     args=[None, dict(frame=dict(duration=100, redraw=True),
                                      fromcurrent=True, mode="immediate")]),
                dict(label="Pause",
                     method="animate",
                     args=[[None], dict(frame=dict(duration=0, redraw=False),
                                        mode="immediate")]),
            ],
        )],
        title=dict(text="Path Execution", x=0.5),
        height=550,
        width=1200,
        template="plotly_white",
        images=[dict(
            source=cmap_b64,
            xref="x", yref="y",
            x=extent[0], y=extent[3],
            sizex=extent[1] - extent[0],
            sizey=extent[3] - extent[2],
            sizing="stretch",
            layer="below",
        )],
    )

    fig.update_xaxes(title_text="q1", range=[-_PI, _PI], scaleanchor="y", row=1, col=1)
    fig.update_yaxes(title_text="q2", range=[-_PI, _PI], row=1, col=1)
    ws_lim = 2.3
    fig.update_xaxes(title_text="x", range=[-ws_lim, ws_lim], scaleanchor="y2", row=1, col=2)
    fig.update_yaxes(title_text="y", range=[-ws_lim, ws_lim], row=1, col=2)

    fig.write_html(str(out_path), include_plotlyjs="cdn")


# ─── C++ integration ────────────────────────────────────────────────────────

def _load_cpp_module(repo_root: Path):
    python_dir = repo_root / "python"
    if str(python_dir) not in sys.path:
        sys.path.insert(0, str(python_dir))
    build_dirs = [
        repo_root / "build_x64" / "Release",
        repo_root / "build_x64" / "Debug",
        repo_root / "build" / "Release",
        repo_root / "build" / "Debug",
    ]
    for bd in build_dirs:
        if bd.is_dir() and str(bd) not in sys.path:
            sys.path.insert(0, str(bd))
    try:
        return importlib.import_module("_sbf5_cpp")
    except ImportError as exc:
        raise SystemExit(
            "Unable to import _sbf5_cpp. Build the v5 Python bindings first.\n"
            "  cmake --build build_x64 --config Release --target _sbf5_cpp"
        ) from exc


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
        "seed_config": np.asarray(box.seed_config, dtype=float).tolist()
        if hasattr(box, "seed_config") else [],
    }


# ─── Orchestrator ────────────────────────────────────────────────────────────

def run_growth_visualization(
    repo_root: Union[str, Path],
    output_dir: Union[str, Path],
    *,
    grower_mode: str = "Wavefront",
    q_start: Optional[Sequence[float]] = None,
    q_goal: Optional[Sequence[float]] = None,
    obstacle_count: int = 4,
    obstacle_seed: int = 123,
    grower_seed: int = 7,
    max_boxes: int = 200,
    timeout_ms: float = 30000.0,
    collision_resolution: float = 0.04,
    growth_frames: int = 40,
    growth_arm_samples: int = 12,
    link_1: float = 1.0,
    link_2: float = 1.0,
) -> Dict[str, object]:
    repo_root = Path(repo_root).resolve()
    output_dir = Path(output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    if q_start is None:
        q_start = [0.5, 0.5]
    if q_goal is None:
        q_goal = [2.5, -1.0]
    q_start_arr = np.array(q_start, dtype=float)
    q_goal_arr = np.array(q_goal, dtype=float)

    cpp = _load_cpp_module(repo_root)

    # Load robot
    robot_path = repo_root / "data" / "2dof_planar.json"
    robot = cpp.Robot.from_json(str(robot_path))
    print(f"Robot: {robot.name()}, {robot.n_joints()} joints")

    # Obstacles
    obstacles = build_random_obstacles_2d(n_obs=obstacle_count, seed=obstacle_seed)
    cpp_obstacles = []
    for obs in obstacles:
        cpp_obstacles.append(cpp.Obstacle(
            float(obs.min_point[0]), float(obs.min_point[1]), float(obs.min_point[2]),
            float(obs.max_point[0]), float(obs.max_point[1]), float(obs.max_point[2]),
        ))
    print(f"Obstacles: {len(obstacles)}")

    # Configure planner
    config = cpp.SBFPlannerConfig()
    mode_key = grower_mode.lower().strip()
    mode_map = {"wavefront": cpp.GrowerMode.WAVEFRONT, "rrt": cpp.GrowerMode.RRT}
    if mode_key not in mode_map:
        raise ValueError(f"Unsupported grower_mode: {grower_mode}")
    config.grower.mode = mode_map[mode_key]
    config.grower.max_boxes = max_boxes
    config.grower.rng_seed = grower_seed

    planner = cpp.SBFPlanner(robot, config)
    print(f"Planning: mode={grower_mode}, max_boxes={max_boxes}, seed={grower_seed}")
    result = planner.plan(q_start_arr, q_goal_arr, cpp_obstacles, timeout_ms)

    boxes = sorted(list(planner.boxes()), key=lambda b: int(b.id))
    raw_boxes = sorted(list(planner.raw_boxes()), key=lambda b: int(b.id))
    n_boxes = len(boxes)
    print(f"Result: {n_boxes} boxes ({len(raw_boxes)} before coarsen), success={result.success}, "
          f"path_len={result.path_length:.3f}, time={result.planning_time_ms:.1f}ms")

    # Collision heatmap
    print(f"Computing collision map (resolution={collision_resolution})...")
    cmap_data, extent = scan_collision_map_2d(
        obstacles, resolution=collision_resolution, link_1=link_1, link_2=link_2)
    print("Collision map done.")

    # Render growth HTML (use raw boxes — no coarsening overlaps)
    if raw_boxes:
        growth_path = output_dir / "forest_growth.html"
        print(f"Rendering growth animation ({growth_frames} frames)...")
        render_growth_html(
            raw_boxes, obstacles, cmap_data, extent,
            q_start_arr, q_goal_arr,
            growth_path,
            link_1=link_1, link_2=link_2,
            n_frames=growth_frames,
            n_arm_samples=growth_arm_samples,
            seed=grower_seed,
        )
        print(f"  -> {growth_path}")

    # Render execution HTML
    if result.success and result.path:
        waypoints = [np.array(w, dtype=float) for w in result.path]
        exec_path = output_dir / "path_execution.html"
        print("Rendering execution animation...")
        render_execution_html(
            boxes, waypoints, obstacles, cmap_data, extent,
            q_start_arr, q_goal_arr,
            exec_path,
            link_1=link_1, link_2=link_2,
        )
        print(f"  -> {exec_path}")

    # Export JSON artifacts
    summary = {
        "grower_mode": grower_mode,
        "obstacle_seed": obstacle_seed,
        "grower_seed": grower_seed,
        "obstacle_count": obstacle_count,
        "max_boxes": max_boxes,
        "n_boxes": n_boxes,
        "start": q_start_arr.tolist(),
        "goal": q_goal_arr.tolist(),
        "success": result.success,
        "path_length": float(result.path_length),
        "planning_time_ms": float(result.planning_time_ms),
        "build_time_ms": float(result.build_time_ms),
        "n_coarsen_merges": int(result.n_coarsen_merges),
        "outputs": {
            "growth_html": "forest_growth.html" if (output_dir / "forest_growth.html").exists() else None,
            "execution_html": "path_execution.html" if (output_dir / "path_execution.html").exists() else None,
            "boxes_json": "boxes.json",
            "obstacles_json": "obstacles.json",
            "summary_json": "summary.json",
        },
    }

    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8")
    (output_dir / "obstacles.json").write_text(
        json.dumps([obs.to_json() for obs in obstacles], indent=2), encoding="utf-8")
    (output_dir / "boxes.json").write_text(
        json.dumps([_box_to_json(b) for b in boxes], indent=2), encoding="utf-8")

    print(f"Summary written to {output_dir / 'summary.json'}")
    return summary


# ─── CLI ─────────────────────────────────────────────────────────────────────

def _parse_float_pair(s: str) -> List[float]:
    parts = s.split(",")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError(f"Expected 2 comma-separated floats, got: {s!r}")
    return [float(x.strip()) for x in parts]


def build_cli_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Render 2DOF dual-panel growth and execution HTML with the v5 planner.")
    parser.add_argument("--output-dir", type=str, default=None,
                        help="Output directory (default: results/growth_viz_<timestamp>).")
    parser.add_argument("--mode", type=str, choices=["Wavefront", "RRT"],
                        default="Wavefront", help="Grower mode.")
    parser.add_argument("--start", type=_parse_float_pair, default=None,
                        help="Start config as 'q1,q2' (default: 0.5,0.5).")
    parser.add_argument("--goal", type=_parse_float_pair, default=None,
                        help="Goal config as 'q1,q2' (default: 2.5,-1.0).")
    parser.add_argument("--obstacle-count", type=int, default=4,
                        help="Number of workspace obstacles.")
    parser.add_argument("--obstacle-seed", type=int, default=123,
                        help="Random seed for obstacle generation.")
    parser.add_argument("--grower-seed", type=int, default=7,
                        help="Random seed for the grower.")
    parser.add_argument("--max-boxes", type=int, default=200,
                        help="Maximum number of boxes.")
    parser.add_argument("--timeout-ms", type=float, default=30000.0,
                        help="Planning timeout in milliseconds.")
    parser.add_argument("--collision-resolution", type=float, default=0.04,
                        help="Resolution for collision heatmap.")
    parser.add_argument("--growth-frames", type=int, default=40,
                        help="Number of growth animation frames.")
    parser.add_argument("--growth-arm-samples", type=int, default=12,
                        help="Workspace arm samples per growth frame.")
    return parser


def run_cli() -> Dict[str, object]:
    parser = build_cli_parser()
    args = parser.parse_args()
    repo_root = Path(__file__).resolve().parents[1]  # v5/viz/.. -> v5/

    if args.output_dir is not None:
        output_dir = Path(args.output_dir)
        if not output_dir.is_absolute():
            output_dir = repo_root / output_dir
    else:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = repo_root / "results" / f"growth_viz_{stamp}"

    summary = run_growth_visualization(
        repo_root,
        output_dir,
        grower_mode=args.mode,
        q_start=args.start,
        q_goal=args.goal,
        obstacle_count=args.obstacle_count,
        obstacle_seed=args.obstacle_seed,
        grower_seed=args.grower_seed,
        max_boxes=args.max_boxes,
        timeout_ms=args.timeout_ms,
        collision_resolution=args.collision_resolution,
        growth_frames=args.growth_frames,
        growth_arm_samples=args.growth_arm_samples,
    )
    print(f"\nDone. Outputs in: {output_dir}")
    return summary


if __name__ == "__main__":
    run_cli()

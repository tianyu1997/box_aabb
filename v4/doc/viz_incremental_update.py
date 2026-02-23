"""
viz_incremental_update.py — 2DOF: C-space 与笛卡尔空间对照图

生成 1×2 面板:
    ① C-space: 新障碍场景下的碰撞底图 + 增量更新结果
    ② Workspace(笛卡尔): 障碍物 + 由更新后 box 采样得到的机械臂构型

用法:
        cd v3/doc
        python viz_incremental_update.py
"""
from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Dict, Set

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Rectangle
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
from planner.defaults import TWODOF_DEFAULTS
from planner.pipeline import grow_and_prepare

# ═══════════════════════════════════════════════════════════════════════════
# Config
# ═══════════════════════════════════════════════════════════════════════════

SEED = 42
DPI = 180
OUT_PATH = Path(__file__).parent / "img_incremental_update.png"
GROWTH_GIF_PATH = Path(__file__).parent / "img_incremental_update_forest_growth.gif"

INITIAL_OBSTACLES = [
    {"min": [-0.5, 1.0], "max": [0.2, 1.5], "name": "obs_0"},
    {"min": [1.0, 0.3], "max": [1.6, 0.9], "name": "obs_1"},
    {"min": [-1.5, -0.5], "max": [-0.8, 0.2], "name": "obs_2"},
]

NEW_OBSTACLE = {"min": [0.5, 0.5], "max": [1.0, 1.0], "name": "obs_new"}
L1, L2 = 1.0, 1.0


# ═══════════════════════════════════════════════════════════════════════════

def build_scene(obstacles):
    scene = Scene()
    for o in obstacles:
        scene.add_obstacle(o["min"], o["max"], name=o["name"])
    return scene


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
               highlight_color='#ff4444', highlight_alpha=0.6):
    highlight_ids = highlight_ids or set()
    cmap_c = plt.cm.viridis
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


def setup_ax(ax, title):
    ax.set_xlim(-np.pi, np.pi)
    ax.set_ylim(-np.pi, np.pi)
    ax.set_aspect('equal')
    ax.set_title(title, fontsize=11, fontweight='bold', pad=8)
    ax.set_xlabel("θ₁")
    ax.set_ylabel("θ₂")
    ax.grid(True, alpha=0.1)


def fk_2dof(q1: float, q2: float):
    x1 = L1 * np.cos(q1)
    y1 = L1 * np.sin(q1)
    x2 = x1 + L2 * np.cos(q1 + q2)
    y2 = y1 + L2 * np.sin(q1 + q2)
    return np.array([[0.0, 0.0], [x1, y1], [x2, y2]], dtype=np.float64)


def sample_from_boxes(rng: np.random.Generator, boxes, n_samples: int):
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


def pick_start_goal_from_boxes(
    rng: np.random.Generator,
    boxes,
    checker: CollisionChecker,
    n_candidates: int = 220,
):
    candidates = sample_from_boxes(rng, boxes, n_candidates)
    if len(candidates) < 2:
        raise RuntimeError("Not enough sampled candidates to pick start/goal")

    valid = []
    for q in candidates:
        if not checker.check_config_collision(q):
            valid.append(q)
    if len(valid) < 2:
        raise RuntimeError("Could not find two collision-free configs from boxes_v2")

    valid = np.asarray(valid, dtype=np.float64)

    def _wrap_to_pi(a):
        return (a + np.pi) % (2 * np.pi) - np.pi

    # posture preference: avoid strongly folded elbows and prefer extended reach
    q2 = _wrap_to_pi(valid[:, 1])
    ee = np.array([fk_2dof(float(q[0]), float(q[1]))[-1] for q in valid])
    ee_r = np.linalg.norm(ee, axis=1)

    # higher bonus = less folded + more stretched (for nicer-looking arm shapes)
    posture_bonus = np.clip(1.25 - np.abs(q2), 0.0, None) + 0.6 * np.clip(ee_r - 1.25, 0.0, None)

    # hard gate first: prefer non-folded elbow postures if enough candidates exist
    not_folded = (np.abs(q2) <= 1.2) & (ee_r >= 1.1)
    if np.count_nonzero(not_folded) >= 2:
        pool = valid[not_folded]
        pool_bonus = posture_bonus[not_folded]
    else:
        keep_n = min(len(valid), max(24, len(valid) // 2))
        keep_idx = np.argsort(-posture_bonus)[:keep_n]
        pool = valid[keep_idx]
        pool_bonus = posture_bonus[keep_idx]

    d2 = np.sum((pool[:, None, :] - pool[None, :, :]) ** 2, axis=2)
    score = d2 + 0.35 * (pool_bonus[:, None] + pool_bonus[None, :])
    np.fill_diagonal(score, -1.0)
    i, j = np.unravel_index(np.argmax(score), score.shape)
    return pool[i], pool[j]


def save_forest_growth_gif(cmap_v2, extent, boxes_v2, q_start, q_goal):
    ordered_items = list(boxes_v2.items())
    n_total = len(ordered_items)
    if n_total == 0:
        raise RuntimeError("boxes_v2 is empty; cannot render growth gif")

    stride = max(1, n_total // 40)
    frame_sizes = list(range(stride, n_total + 1, stride))
    if frame_sizes[-1] != n_total:
        frame_sizes.append(n_total)

    fig, ax = plt.subplots(1, 1, figsize=(7.0, 6.2))
    fig.patch.set_facecolor('white')

    from matplotlib.lines import Line2D
    legend_elements = [
        Rectangle((0, 0), 1, 1, facecolor='#4CAF50', alpha=0.30,
                  edgecolor='#4CAF50', label='Safe boxes (v2)'),
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
        setup_ax(ax, f"Forest Growth (Updated Scene)  {n_show}/{n_total}")
        draw_collision_bg(ax, cmap_v2, extent, alpha=0.26)
        sub_boxes = dict(ordered_items[:n_show])
        draw_boxes(ax, sub_boxes, alpha=0.30)
        ax.plot(q_start[0], q_start[1], 'o', color='cyan', markersize=8,
                markeredgecolor='black', zorder=20)
        ax.plot(q_goal[0], q_goal[1], '*', color='yellow', markersize=12,
                markeredgecolor='black', zorder=20)
        ax.legend(handles=legend_elements, loc='lower right', fontsize=8.5,
                  framealpha=0.92)

    anim = FuncAnimation(fig, _render_frame, frames=len(frame_sizes),
                         interval=120, repeat=True)
    anim.save(str(GROWTH_GIF_PATH), writer=PillowWriter(fps=7), dpi=120)
    plt.close(fig)


# ═══════════════════════════════════════════════════════════════════════════

def generate():
    rng = np.random.default_rng(SEED)
    robot = load_robot("2dof_planar")
    q_seed_start = np.array([0.8 * np.pi, 0.2])
    q_seed_goal = np.array([-0.7 * np.pi, -0.4])

    class Cfg:
        seed = SEED
        max_consecutive_miss = 300
        max_boxes = 250
        ffb_min_edge = TWODOF_DEFAULTS.ffb_min_edge
        guided_sample_ratio = TWODOF_DEFAULTS.guided_sample_ratio
        boundary_expand_epsilon = 0.01
        parallel_grow = False
        n_partitions_depth = 0
        parallel_workers = 1
        coarsen_max_rounds = 0  # no coarsen for cleaner visualization
    cfg = Cfg()

    # ────── Phase 1: Build forest for original scene ──────
    print("[Phase 1] Building initial forest ...")
    scene_v1 = build_scene(INITIAL_OBSTACLES)
    prep1 = grow_and_prepare(robot, scene_v1, cfg, q_seed_start, q_seed_goal,
                             ndim=2, no_cache=True)
    boxes_v1 = snapshot_boxes(prep1['forest_obj'])
    n_v1 = len(boxes_v1)
    print(f"  Forest v1: {n_v1} boxes")

    # ────── Phase 2: Identify colliding boxes ──────
    print("[Phase 2] Identifying colliding boxes ...")
    from forest.models import Obstacle
    new_obs = Obstacle(
        min_point=np.asarray(NEW_OBSTACLE["min"]),
        max_point=np.asarray(NEW_OBSTACLE["max"]),
        name=NEW_OBSTACLE["name"],
    )
    colliding_ids = prep1['forest_obj'].invalidate_against_obstacle(
        new_obs, robot, safety_margin=0.0)
    surviving_ids = set(boxes_v1.keys()) - colliding_ids
    print(f"  Colliding: {len(colliding_ids)}, surviving: {len(surviving_ids)}")

    # ────── Phase 3: Build forest for updated scene ──────
    print("[Phase 3] Building forest for updated scene ...")
    all_obstacles = INITIAL_OBSTACLES + [NEW_OBSTACLE]
    scene_v2 = build_scene(all_obstacles)
    checker_v2 = CollisionChecker(robot=robot, scene=scene_v2)
    cfg.seed = SEED + 100
    prep2 = grow_and_prepare(robot, scene_v2, cfg, q_seed_start, q_seed_goal,
                             ndim=2, no_cache=True)
    boxes_v2 = snapshot_boxes(prep2['forest_obj'])
    n_v2 = len(boxes_v2)
    print(f"  Forest v2: {n_v2} boxes")

    # Pick start/goal from updated boxes: collision-free and far apart
    q_start, q_goal = pick_start_goal_from_boxes(rng, boxes_v2, checker_v2)
    print(f"  Selected start: [{q_start[0]:.3f}, {q_start[1]:.3f}] (collision-free)")
    print(f"  Selected goal : [{q_goal[0]:.3f}, {q_goal[1]:.3f}] (collision-free)")

    # ────── Phase 4: Scan collision maps ──────
    print("[Phase 4] Scanning collision maps ...")
    cmap_v1, extent = scan_collision_map(robot, scene_v1, resolution=0.04)
    cmap_v2, _ = scan_collision_map(robot, scene_v2, resolution=0.04)

    # Extra output: forest growth GIF for updated scene
    save_forest_growth_gif(cmap_v2, extent, boxes_v2, q_start, q_goal)

    # ────── Phase 5: Generate C-space / Workspace comparison ──────
    print("[Phase 5] Generating C-space vs Workspace comparison ...")
    fig, axes = plt.subplots(1, 2, figsize=(13.5, 5.6),
                             gridspec_kw={'width_ratios': [1.15, 1.0]})
    fig.patch.set_facecolor('white')

    # Panel 1: C-space incremental update result
    ax = axes[0]
    setup_ax(ax, "① C-space: Incremental Update")
    draw_collision_bg(ax, cmap_v2, extent, alpha=0.26)

    surviving_boxes = {bid: boxes_v1[bid] for bid in surviving_ids}
    colliding_boxes = {bid: boxes_v1[bid] for bid in colliding_ids if bid in boxes_v1}
    draw_boxes(ax, surviving_boxes, alpha=0.14)
    draw_boxes(ax, colliding_boxes,
               highlight_ids=set(colliding_boxes.keys()),
               highlight_color='#ff1744', highlight_alpha=0.45)
    draw_boxes(ax, boxes_v2, alpha=0.26)

    ax.plot(q_start[0], q_start[1], 'o', color='cyan', markersize=8,
            markeredgecolor='black', zorder=20)
    ax.plot(q_goal[0], q_goal[1], '*', color='yellow', markersize=12,
            markeredgecolor='black', zorder=20)
    ax.text(-3.05, 2.80,
            f"v1={n_v1}, invalidated={len(colliding_ids)}, v2={n_v2}",
            fontsize=9, color='#263238',
            bbox=dict(boxstyle='round,pad=0.25', facecolor='white', alpha=0.85,
                      edgecolor='none'))

    # Panel 2: Cartesian workspace
    ax = axes[1]
    ax.set_title("② Workspace (Cartesian)", fontsize=11, fontweight='bold', pad=8)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_aspect('equal')
    ax.set_xlim(-2.3, 2.3)
    ax.set_ylim(-2.3, 2.3)
    ax.grid(True, alpha=0.12)

    for obs in INITIAL_OBSTACLES:
        w = obs['max'][0] - obs['min'][0]
        h = obs['max'][1] - obs['min'][1]
        ax.add_patch(Rectangle((obs['min'][0], obs['min'][1]), w, h,
                               facecolor='#9e9e9e', edgecolor='#616161',
                               alpha=0.22, linewidth=1.0))
    w_new = NEW_OBSTACLE['max'][0] - NEW_OBSTACLE['min'][0]
    h_new = NEW_OBSTACLE['max'][1] - NEW_OBSTACLE['min'][1]
    ax.add_patch(Rectangle((NEW_OBSTACLE['min'][0], NEW_OBSTACLE['min'][1]),
                           w_new, h_new, facecolor='#ef5350', edgecolor='#c62828',
                           alpha=0.55, linewidth=1.2))

    q_samples = sample_from_boxes(rng, boxes_v2, n_samples=110)
    for q1, q2 in q_samples:
        pts = fk_2dof(q1, q2)
        ax.plot(pts[:, 0], pts[:, 1], color='#1976d2', alpha=0.08, linewidth=1.0)
        ax.plot(pts[-1, 0], pts[-1, 1], '.', color='#1976d2', alpha=0.12, markersize=2.5)

    pts_start = fk_2dof(float(q_start[0]), float(q_start[1]))
    pts_goal = fk_2dof(float(q_goal[0]), float(q_goal[1]))
    ax.plot(pts_start[:, 0], pts_start[:, 1], '-o', color='cyan', linewidth=2.2,
            markersize=4, markeredgecolor='black', zorder=15)
    ax.plot(pts_goal[:, 0], pts_goal[:, 1], '-*', color='yellow', linewidth=2.2,
            markersize=9, markeredgecolor='black', zorder=15)

    # Legends
    from matplotlib.lines import Line2D
    legend_elements = [
        Rectangle((0, 0), 1, 1, facecolor='#4CAF50', alpha=0.26,
                  edgecolor='#4CAF50', label='Updated safe boxes (v2)'),
        Rectangle((0, 0), 1, 1, facecolor='#ff1744', alpha=0.45,
                  edgecolor='#ff1744', label='Invalidated v1 boxes'),
        Rectangle((0, 0), 1, 1, facecolor='#ef5350', alpha=0.55,
                  edgecolor='#c62828', label='New obstacle'),
        Line2D([], [], color='#1976d2', linewidth=1.5, alpha=0.6,
               label='FK samples from v2 boxes'),
        Line2D([], [], marker='o', color='cyan', markersize=6,
               markeredgecolor='black', linestyle='-', label='Start config arm'),
        Line2D([], [], marker='*', color='yellow', markersize=10,
               markeredgecolor='black', linestyle='-', label='Goal config arm'),
    ]
    fig.legend(handles=legend_elements, loc='lower center', ncol=3,
               fontsize=8.8, framealpha=0.92, bbox_to_anchor=(0.5, -0.035))

    fig.text(0.50, 0.515, 'C-space  ⟷  Cartesian workspace',
             ha='center', va='center', fontsize=11,
             fontweight='bold', color='#37474f')

    plt.tight_layout(rect=[0, 0.065, 1, 0.98])
    fig.savefig(str(OUT_PATH), dpi=DPI, bbox_inches='tight',
                facecolor='white')
    plt.close(fig)
    print(f"Saved: {OUT_PATH}")
    print(f"Saved: {GROWTH_GIF_PATH}")
    return str(OUT_PATH)


if __name__ == "__main__":
    generate()

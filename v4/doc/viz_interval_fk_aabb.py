"""
viz_interval_fk_aabb.py — 2DOF planar robot: Interval FK → AABB envelope 可视化

生成两个子图:
  左: 工作空间 — 臂型采样(半透明) + link AABB 包络 + 障碍物
  右: 关节空间 — 对应的 joint interval box + 碰撞底图

用法:
    cd v3/doc
    python viz_interval_fk_aabb.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import List, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import Rectangle, FancyBboxPatch
import numpy as np

# ── v3 path setup ──
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

# ═══════════════════════════════════════════════════════════════════════════
# Config
# ═══════════════════════════════════════════════════════════════════════════

SEED = 42
N_SAMPLES = 80          # arm configuration samples within interval
L1, L2 = 1.0, 1.0       # 2DOF link lengths

# Preferred seed interval (script will auto-adjust until collision-free AABB)
Q_INTERVAL = [(-0.3, 0.7), (0.2, 1.0)]

# Obstacles in workspace (2D AABB: [x_lo, y_lo], [x_hi, y_hi])
OBSTACLES = [
    {"min": [-0.5, 1.0], "max": [0.2, 1.5], "name": "table", "color": "#d32f2f"},
    {"min": [1.0, 0.3], "max": [1.6, 0.9], "name": "shelf", "color": "#c62828"},
]

DPI = 180
OUT_PATH = Path(__file__).parent / "img_interval_fk_aabb.png"


# ═══════════════════════════════════════════════════════════════════════════
# Helpers
# ═══════════════════════════════════════════════════════════════════════════

def fk_2dof(q1, q2):
    """Forward kinematics: returns [(0,0), elbow, end-effector]."""
    x1 = L1 * math.cos(q1)
    y1 = L1 * math.sin(q1)
    x2 = x1 + L2 * math.cos(q1 + q2)
    y2 = y1 + L2 * math.sin(q1 + q2)
    return np.array([[0, 0], [x1, y1], [x2, y2]])


def _overlap_1d(a_lo: float, a_hi: float, b_lo: float, b_hi: float) -> bool:
    return max(a_lo, b_lo) <= min(a_hi, b_hi)


def _aabb_hits_obstacles_xy(link_aabbs, obstacles) -> bool:
    """Strict XY overlap check between link AABBs and 2D obstacles."""
    for la in link_aabbs:
        ax0, ay0 = float(la.min_point[0]), float(la.min_point[1])
        ax1, ay1 = float(la.max_point[0]), float(la.max_point[1])
        for obs in obstacles:
            ox0, oy0 = float(obs["min"][0]), float(obs["min"][1])
            ox1, oy1 = float(obs["max"][0]), float(obs["max"][1])
            if _overlap_1d(ax0, ax1, ox0, ox1) and _overlap_1d(ay0, ay1, oy0, oy1):
                return True
    return False


def _aabb_iou_xy(a, b) -> float:
    ax0, ay0 = float(a.min_point[0]), float(a.min_point[1])
    ax1, ay1 = float(a.max_point[0]), float(a.max_point[1])
    bx0, by0 = float(b.min_point[0]), float(b.min_point[1])
    bx1, by1 = float(b.max_point[0]), float(b.max_point[1])

    ix = max(0.0, min(ax1, bx1) - max(ax0, bx0))
    iy = max(0.0, min(ay1, by1) - max(ay0, by0))
    inter = ix * iy
    if inter <= 1e-12:
        return 0.0
    area_a = max(0.0, (ax1 - ax0)) * max(0.0, (ay1 - ay0))
    area_b = max(0.0, (bx1 - bx0)) * max(0.0, (by1 - by0))
    union = area_a + area_b - inter
    if union <= 1e-12:
        return 1.0
    return float(inter / union)


def _find_collision_free_interval(
    robot,
    checker: CollisionChecker,
    obstacles,
    preferred_interval: List[Tuple[float, float]],
    rng: np.random.Generator,
):
    """Find a collision-free interval whose interval AABBs do not hit obstacles."""
    candidates = [preferred_interval]

    # random candidates around free C-space; keep compact boxes for clearer envelopes
    for _ in range(500):
        c1 = rng.uniform(-np.pi + 0.45, np.pi - 0.45)
        c2 = rng.uniform(-np.pi + 0.45, np.pi - 0.45)
        w1 = rng.uniform(0.25, 0.55)
        w2 = rng.uniform(0.25, 0.55)
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
        # Conservative interval-collision certificate: False means certified free
        if checker.check_box_collision(iv):
            continue
        link_aabbs, _ = compute_interval_aabb(robot, iv, set(), skip_zero_length=True)
        if _aabb_hits_obstacles_xy(link_aabbs, obstacles):
            continue

        # minimize overlap between non-degenerate link AABBs for readability
        nondeg = []
        for la in link_aabbs:
            w = float(la.max_point[0] - la.min_point[0])
            h = float(la.max_point[1] - la.min_point[1])
            if w > 1e-6 and h > 1e-6:
                nondeg.append(la)
        overlap_score = 0.0
        if len(nondeg) >= 2:
            overlap_score = _aabb_iou_xy(nondeg[0], nondeg[1])

        # slight preference for moderate box size (avoid too tiny intervals)
        s1 = iv[0][1] - iv[0][0]
        s2 = iv[1][1] - iv[1][0]
        compact_penalty = max(0.0, 0.45 - min(s1, s2))
        score = overlap_score + 0.15 * compact_penalty

        if score < best_score:
            best_score = score
            best = (iv, link_aabbs)

    if best is not None:
        return best

    raise RuntimeError("未找到满足条件的无碰撞 joint interval AABB")


# ═══════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════

def generate():
    rng = np.random.default_rng(SEED)
    robot = load_robot("2dof_planar")

    # Build scene/checker first so we can search for a certified collision-free interval
    scene = Scene()
    for obs in OBSTACLES:
        scene.add_obstacle(obs["min"], obs["max"], name=obs["name"])
    checker = CollisionChecker(robot=robot, scene=scene)

    q_interval, link_aabbs = _find_collision_free_interval(
        robot=robot,
        checker=checker,
        obstacles=OBSTACLES,
        preferred_interval=Q_INTERVAL,
        rng=rng,
    )
    print(f"  Selected collision-free joint interval: {q_interval}")

    # --- Use certified interval FK AABBs ---
    print(f"  Interval FK → {len(link_aabbs)} link AABBs")
    for la in link_aabbs:
        print(f"    link {la.link_index}: "
              f"[{la.min_point[0]:.3f},{la.min_point[1]:.3f},{la.min_point[2]:.3f}]"
              f" → [{la.max_point[0]:.3f},{la.max_point[1]:.3f},{la.max_point[2]:.3f}]")

    # --- Sample arm configurations ---
    q1_lo, q1_hi = q_interval[0]
    q2_lo, q2_hi = q_interval[1]
    q1_samples = rng.uniform(q1_lo, q1_hi, N_SAMPLES)
    q2_samples = rng.uniform(q2_lo, q2_hi, N_SAMPLES)
    q1_mid = (q1_lo + q1_hi) / 2
    q2_mid = (q2_lo + q2_hi) / 2

    # --- Figure ---
    fig, (ax_cs, ax_ws) = plt.subplots(1, 2, figsize=(13, 5.5),
                                        gridspec_kw={'width_ratios': [1.2, 1]})
    fig.patch.set_facecolor('white')

    # ────────────── Left: Joint-space (C-space) ──────────────
    ax_cs.set_title("C-space: Collision-Free Joint Interval", fontsize=13,
                    fontweight='bold', pad=10)
    ax_cs.set_xlabel("θ₁ (rad)")
    ax_cs.set_ylabel("θ₂ (rad)")
    ax_cs.set_aspect('equal')
    ax_cs.set_xlim(-np.pi, np.pi)
    ax_cs.set_ylim(-np.pi, np.pi)
    ax_cs.grid(True, alpha=0.15)

    # Scan collision map for background
    res = 0.03
    xs = np.arange(-np.pi, np.pi, res)
    ys = np.arange(-np.pi, np.pi, res)
    cmap = np.zeros((len(ys), len(xs)), dtype=np.float32)
    for i, y in enumerate(ys):
        row = np.column_stack([xs, np.full(len(xs), y)])
        cmap[i, :] = checker.check_config_collision_batch(row).astype(np.float32)
    ax_cs.imshow(cmap, origin='lower',
                 extent=[-np.pi, np.pi, -np.pi, np.pi],
                 cmap='Reds', alpha=0.25, aspect='auto')

    # Draw the safe box interval
    rect_q = Rectangle((q1_lo, q2_lo), q1_hi - q1_lo, q2_hi - q2_lo,
                        facecolor='#4CAF50', alpha=0.25,
                        edgecolor='#4CAF50', linewidth=2.5,
                        label='Safe Box Q')
    ax_cs.add_patch(rect_q)

    # Mark samples inside the box
    ax_cs.scatter(q1_samples, q2_samples, s=8, c='#2196F3', alpha=0.4,
                  zorder=5, label='Sampled configs')
    ax_cs.plot(q1_mid, q2_mid, '*', color='#FF9800', markersize=14,
               markeredgecolor='black', markeredgewidth=0.8, zorder=10,
               label='Midpoint')

    # Annotation arrow 
    ax_cs.annotate('', xy=(-1.5, 1.5), xytext=(-0.5, 1.2),
                   arrowprops=dict(arrowstyle='->', color='#d32f2f',
                                   lw=1.5))
    ax_cs.text(-1.8, 1.6, 'C-obstacle', fontsize=9, color='#d32f2f')

    ax_cs.legend(loc='lower right', fontsize=8, framealpha=0.9)

    # ────────────── Right: Workspace ──────────────
    ax_ws.set_title("Workspace: FK Samples + Link AABBs", fontsize=13,
                    fontweight='bold', pad=10)
    ax_ws.set_xlabel("x")
    ax_ws.set_ylabel("y")
    ax_ws.set_aspect('equal')
    ax_ws.set_xlim(-2.2, 2.2)
    ax_ws.set_ylim(-2.2, 2.2)
    ax_ws.grid(True, alpha=0.15)

    for obs in OBSTACLES:
        lo, hi = obs["min"], obs["max"]
        rect = Rectangle((lo[0], lo[1]), hi[0]-lo[0], hi[1]-lo[1],
                          facecolor=obs["color"], alpha=0.35,
                          edgecolor=obs["color"], linewidth=1.5,
                          label=f'Obstacle')
        ax_ws.add_patch(rect)

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
    ax_ws.plot(pts_mid[1, 0], pts_mid[1, 1], 'o', color='#2196F3', markersize=6, zorder=10)
    ax_ws.plot(pts_mid[2, 0], pts_mid[2, 1], '*', color='#4CAF50', markersize=12, zorder=10)

    aabb_colors = ['#2196F3', '#FF9800', '#4CAF50']
    aabb_labels = ['Link 1 AABB', 'Link 2 AABB', 'Tool AABB']
    aabb_hatches = ['///', '\\\\\\', '...']
    for idx, la in enumerate(link_aabbs):
        lo_x, lo_y = la.min_point[0], la.min_point[1]
        hi_x, hi_y = la.max_point[0], la.max_point[1]
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
        plt.Line2D([], [], color='#2196F3', lw=1.0, alpha=0.35,
                   label=f'{N_SAMPLES} sampled arms'),
        plt.Line2D([], [], color='#2196F3', lw=3.0, alpha=0.95,
                   label='Midpoint arm'),
        mpatches.Patch(facecolor='#d32f2f', alpha=0.35, label='Obstacle'),
    ]
    ax_ws.legend(handles=handles_ws, loc='lower left', fontsize=8,
                 framealpha=0.9)

    # ── Big arrow between subplots ──
    fig.text(0.55, 0.50, 'FK\n⟶\nAABB', ha='center', va='center',
             fontsize=11, fontweight='bold', color='#666')

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(str(OUT_PATH), dpi=DPI, bbox_inches='tight',
                facecolor='white')
    plt.close(fig)
    print(f"Saved: {OUT_PATH}")
    return str(OUT_PATH)


if __name__ == "__main__":
    generate()

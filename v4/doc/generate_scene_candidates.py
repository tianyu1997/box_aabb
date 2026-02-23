"""
generate_scene_candidates.py — 生成 10 个 2DOF 统一场景候选

输出目录:
  v3/doc/scene_candidates/
    - scene_01.json ... scene_10.json
    - scene_01_preview.png ... scene_10_preview.png
    - index.md

每个候选包含:
  - initial_obstacles (用于静态/基础可视化)
  - new_obstacle (用于 incremental 可视化)
  - q_start / q_goal (无碰撞、不过度折叠、尽量分开)
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
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


OUT_DIR = Path(__file__).parent / "scene_candidates"
N_SCENES = 10


def fk_2dof(q1: float, q2: float):
    l1, l2 = 1.0, 1.0
    x1 = l1 * math.cos(q1)
    y1 = l1 * math.sin(q1)
    x2 = x1 + l2 * math.cos(q1 + q2)
    y2 = y1 + l2 * math.sin(q1 + q2)
    return np.array([[0.0, 0.0], [x1, y1], [x2, y2]], dtype=np.float64)


def build_scene(obstacles: List[Dict]):
    scene = Scene()
    for o in obstacles:
        scene.add_obstacle(o["min"], o["max"], name=o["name"])
    return scene


def _iou(a: Dict, b: Dict) -> float:
    ax0, ay0 = a["min"]
    ax1, ay1 = a["max"]
    bx0, by0 = b["min"]
    bx1, by1 = b["max"]
    ix = max(0.0, min(ax1, bx1) - max(ax0, bx0))
    iy = max(0.0, min(ay1, by1) - max(ay0, by0))
    inter = ix * iy
    if inter <= 1e-12:
        return 0.0
    area_a = max(0.0, ax1 - ax0) * max(0.0, ay1 - ay0)
    area_b = max(0.0, bx1 - bx0) * max(0.0, by1 - by0)
    return inter / max(1e-12, area_a + area_b - inter)


def sample_obstacles(rng: np.random.Generator, n_obs: int = 3) -> List[Dict]:
    obstacles: List[Dict] = []
    attempts = 0
    while len(obstacles) < n_obs and attempts < 2000:
        attempts += 1
        cx = float(rng.uniform(-1.45, 1.45))
        cy = float(rng.uniform(-1.45, 1.45))
        w = float(rng.uniform(0.35, 0.72))
        h = float(rng.uniform(0.35, 0.72))
        cand = {
            "min": [cx - w / 2.0, cy - h / 2.0],
            "max": [cx + w / 2.0, cy + h / 2.0],
            "name": f"obs_{len(obstacles)}",
        }

        # keep center area a bit cleaner for readability
        if abs(cx) < 0.35 and abs(cy) < 0.35:
            continue

        ok = True
        for o in obstacles:
            if _iou(cand, o) > 0.25:
                ok = False
                break
        if ok:
            obstacles.append(cand)

    if len(obstacles) < n_obs:
        raise RuntimeError("Failed to sample enough obstacles")
    return obstacles


def sample_new_obstacle(rng: np.random.Generator, existing: List[Dict]) -> Dict:
    for _ in range(800):
        cx = float(rng.uniform(-1.3, 1.3))
        cy = float(rng.uniform(-1.3, 1.3))
        w = float(rng.uniform(0.30, 0.55))
        h = float(rng.uniform(0.30, 0.55))
        cand = {
            "min": [cx - w / 2.0, cy - h / 2.0],
            "max": [cx + w / 2.0, cy + h / 2.0],
            "name": "obs_new",
        }
        if all(_iou(cand, o) < 0.30 for o in existing):
            return cand
    return {"min": [0.55, 0.55], "max": [0.95, 0.95], "name": "obs_new"}


def pick_start_goal(
    rng: np.random.Generator,
    checker: CollisionChecker,
    n_samples: int = 360,
) -> Tuple[np.ndarray, np.ndarray]:
    valid = []
    for _ in range(n_samples):
        q1 = float(rng.uniform(-math.pi, math.pi))
        q2 = float(rng.uniform(-1.2, 1.2))  # prefer less folded posture
        q = np.array([q1, q2], dtype=np.float64)
        if checker.check_config_collision(q):
            continue
        ee = fk_2dof(q1, q2)[-1]
        if np.linalg.norm(ee) < 1.05:
            continue
        valid.append(q)

    if len(valid) < 2:
        raise RuntimeError("Cannot find enough collision-free start/goal candidates")

    V = np.asarray(valid, dtype=np.float64)
    d2 = np.sum((V[:, None, :] - V[None, :, :]) ** 2, axis=2)
    np.fill_diagonal(d2, -1.0)
    i, j = np.unravel_index(np.argmax(d2), d2.shape)
    return V[i], V[j]


def cspace_map(robot, scene, resolution: float = 0.05):
    checker = CollisionChecker(robot=robot, scene=scene)
    xs = np.arange(-math.pi, math.pi, resolution)
    ys = np.arange(-math.pi, math.pi, resolution)
    cmap = np.zeros((len(ys), len(xs)), dtype=np.float32)
    for i, y in enumerate(ys):
        row = np.column_stack([xs, np.full(len(xs), y)])
        cmap[i, :] = checker.check_config_collision_batch(row).astype(np.float32)
    return cmap, [-math.pi, math.pi, -math.pi, math.pi]


def render_preview(path: Path, scene, obstacles, new_obstacle, q_start, q_goal, robot):
    cmap, extent = cspace_map(robot, scene, resolution=0.05)

    fig, (ax_cs, ax_ws) = plt.subplots(1, 2, figsize=(12.6, 5.2),
                                       gridspec_kw={"width_ratios": [1.15, 1.0]})
    fig.patch.set_facecolor("white")

    # C-space
    ax_cs.set_title("C-space", fontsize=12, fontweight="bold")
    ax_cs.imshow(cmap, origin="lower", extent=extent, cmap="Reds", alpha=0.24, aspect="auto")
    ax_cs.set_xlim(-math.pi, math.pi)
    ax_cs.set_ylim(-math.pi, math.pi)
    ax_cs.set_aspect("equal")
    ax_cs.set_xlabel("θ₁")
    ax_cs.set_ylabel("θ₂")
    ax_cs.grid(True, alpha=0.12)
    ax_cs.plot(q_start[0], q_start[1], "o", color="cyan", markersize=8, markeredgecolor="black")
    ax_cs.plot(q_goal[0], q_goal[1], "*", color="yellow", markersize=12, markeredgecolor="black")

    # Workspace
    ax_ws.set_title("Workspace", fontsize=12, fontweight="bold")
    ax_ws.set_aspect("equal")
    ax_ws.set_xlim(-2.3, 2.3)
    ax_ws.set_ylim(-2.3, 2.3)
    ax_ws.set_xlabel("x")
    ax_ws.set_ylabel("y")
    ax_ws.grid(True, alpha=0.12)

    for obs in obstacles:
        w = obs["max"][0] - obs["min"][0]
        h = obs["max"][1] - obs["min"][1]
        ax_ws.add_patch(Rectangle((obs["min"][0], obs["min"][1]), w, h,
                                  facecolor="#9e9e9e", edgecolor="#616161",
                                  alpha=0.25, linewidth=1.0))
    w = new_obstacle["max"][0] - new_obstacle["min"][0]
    h = new_obstacle["max"][1] - new_obstacle["min"][1]
    ax_ws.add_patch(Rectangle((new_obstacle["min"][0], new_obstacle["min"][1]), w, h,
                              facecolor="#ef5350", edgecolor="#c62828",
                              alpha=0.56, linewidth=1.2))

    s_pts = fk_2dof(float(q_start[0]), float(q_start[1]))
    g_pts = fk_2dof(float(q_goal[0]), float(q_goal[1]))
    ax_ws.plot(s_pts[:, 0], s_pts[:, 1], "-o", color="cyan", linewidth=2.2,
               markersize=4, markeredgecolor="black", zorder=10)
    ax_ws.plot(g_pts[:, 0], g_pts[:, 1], "-*", color="yellow", linewidth=2.2,
               markersize=9, markeredgecolor="black", zorder=10)

    fig.text(0.5, 0.97, "Scene Candidate", ha="center", va="top",
             fontsize=12, fontweight="bold", color="#37474f")
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(path, dpi=170, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def build_candidate(scene_id: int, base_seed: int = 20260222) -> Dict:
    robot = load_robot("2dof_planar")
    rng = np.random.default_rng(base_seed + scene_id * 97)

    for _ in range(240):
        obstacles = sample_obstacles(rng, n_obs=3)
        new_obstacle = sample_new_obstacle(rng, obstacles)
        scene = build_scene(obstacles + [new_obstacle])
        checker = CollisionChecker(robot=robot, scene=scene)

        try:
            q_start, q_goal = pick_start_goal(rng, checker)
        except RuntimeError:
            continue

        # avoid trivial straight-line free case (prefer representative planning scene)
        if not checker.check_segment_collision(q_start, q_goal, 0.03):
            continue

        return {
            "scene_id": scene_id,
            "seed": int(base_seed + scene_id * 97),
            "initial_obstacles": obstacles,
            "new_obstacle": new_obstacle,
            "q_start": [float(q_start[0]), float(q_start[1])],
            "q_goal": [float(q_goal[0]), float(q_goal[1])],
        }

    raise RuntimeError(f"Failed to build candidate scene #{scene_id}")


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    robot = load_robot("2dof_planar")
    lines = [
        "# Scene Candidates (2DOF)",
        "",
        "挑选建议：优先看障碍布局可读性、起终点姿态美观、C-space 分布清晰度。",
        "",
    ]

    for idx in range(1, N_SCENES + 1):
        scene_name = f"scene_{idx:02d}"
        cfg = build_candidate(idx)
        out_json = OUT_DIR / f"{scene_name}.json"
        out_png = OUT_DIR / f"{scene_name}_preview.png"

        with out_json.open("w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2, ensure_ascii=False)

        scene = build_scene(cfg["initial_obstacles"] + [cfg["new_obstacle"]])
        q_start = np.asarray(cfg["q_start"], dtype=np.float64)
        q_goal = np.asarray(cfg["q_goal"], dtype=np.float64)
        render_preview(out_png, scene,
                       cfg["initial_obstacles"], cfg["new_obstacle"],
                       q_start, q_goal, robot)

        lines.append(f"## {scene_name}")
        lines.append("")
        lines.append(f"- config: `{scene_name}.json`")
        lines.append(f"- preview: `{scene_name}_preview.png`")
        lines.append("")
        lines.append(f"![{scene_name}]({scene_name}_preview.png)")
        lines.append("")
        print(f"[OK] {scene_name}")

    index_path = OUT_DIR / "index.md"
    index_path.write_text("\n".join(lines), encoding="utf-8")
    print(f"Saved index: {index_path}")


if __name__ == "__main__":
    main()

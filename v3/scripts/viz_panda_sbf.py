"""
scripts/viz_panda_sbf.py — 在 Panda 机器人上可视化 SBF 规划结果

生成 4 个交互式 HTML:
  1. arm_scene.html       — 3D 场景(障碍物 + 始末臂型 + 末端轨迹)
  2. arm_poses.html       — 多臂型残影(ghost poses)
  3. joint_trajectory.html— 关节轨迹曲线
  4. animation.html       — 路径动画(可播放/暂停)

用法:
    cd v3
    python scripts/viz_panda_sbf.py [--scene panda_10obs_moderate] [--seed 0]
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
_SRC = _ROOT / "src"
for p in (_ROOT, _SRC):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))


def parse_args():
    p = argparse.ArgumentParser(description="Visualize SBF planning on Panda")
    p.add_argument("--scene", default="panda_10obs_moderate",
                   help="scene name from standard_scenes.json")
    p.add_argument("--seed", type=int, default=0,
                   help="planner seed for SBF")
    p.add_argument("--n-ghosts", type=int, default=12,
                   help="number of ghost poses")
    p.add_argument("--n-frames", type=int, default=80,
                   help="number of animation frames")
    return p.parse_args()


def main():
    args = parse_args()
    t_total = time.perf_counter()

    # ── 1) 加载场景 ──
    from experiments.runner import load_scene_from_config
    from experiments.scenes import load_scenes

    print(f"{'=' * 60}")
    print(f"  Panda SBF Visualization")
    print(f"{'=' * 60}")
    print(f"  Scene : {args.scene}")
    print(f"  Seed  : {args.seed}")
    print()

    scene_cfgs = load_scenes([args.scene])
    scene_cfg = scene_cfgs[0]
    robot, scene, query_pairs = load_scene_from_config(scene_cfg)
    q_start, q_goal = query_pairs[0]
    ndim = robot.n_joints

    print(f"  Robot      : {robot.name} ({ndim} DOF)")
    print(f"  Obstacles  : {scene.n_obstacles}")
    print(f"  q_start    : {np.array2string(q_start, precision=3)}")
    print(f"  q_goal     : {np.array2string(q_goal, precision=3)}")
    print(f"  Config dist: {np.linalg.norm(q_goal - q_start):.3f} rad")
    for obs in scene.get_obstacles():
        mn, mx = obs.min_point, obs.max_point
        sz = mx - mn
        cx, cy, cz = (mn[0]+mx[0])/2, (mn[1]+mx[1])/2, (mn[2]+mx[2])/2
        print(f"    {obs.name}: center=({cx:.3f},{cy:.3f},{cz:.3f}) "
              f"size=({sz[0]:.3f},{sz[1]:.3f},{sz[2]:.3f})")

    # ── 2) 运行 SBF-Dijkstra ──
    print(f"\n{'=' * 60}")
    print(f"  Running SBF-Dijkstra ...")
    print(f"{'=' * 60}")

    from baselines import SBFAdapter
    planner = SBFAdapter(method="dijkstra")
    planner.setup(robot, scene, {"seed": args.seed})
    result = planner.plan(q_start, q_goal, timeout=60.0)

    if not result.success:
        print("  [FAIL] SBF planning failed!")
        sys.exit(1)

    waypoints = result.path.tolist()  # List[List[float]]
    cost = result.cost
    n_wp = result.n_waypoints
    plan_s = result.planning_time

    print(f"  [OK] cost={cost:.4f}, {n_wp} waypoints, {plan_s:.3f}s")
    print(f"  Phase times: {result.phase_times}")

    # ── 3) 生成可视化 ──
    from viz.scene_viz import (
        plot_joint_trajectory,
        plot_arm_scene_html,
        plot_arm_poses_html,
        create_animation_html,
        save_plotly_html,
    )
    from utils.output import make_output_dir

    out_dir = make_output_dir("visualizations", f"panda_sbf_{args.scene}")
    print(f"\n{'=' * 60}")
    print(f"  Generating Visualizations")
    print(f"{'=' * 60}")
    print(f"  Output: {out_dir}")

    joint_names = [f"J{i+1}" for i in range(ndim)]
    viz_files = []

    # (a) Joint trajectory
    t0 = time.perf_counter()
    fig_jt = plot_joint_trajectory(
        waypoints, q_start, q_goal,
        label=f"SBF-Dijkstra cost={cost:.3f}",
        joint_names=joint_names,
    )
    if fig_jt:
        p = out_dir / "joint_trajectory.html"
        save_plotly_html(fig_jt, p)
        ms = (time.perf_counter() - t0) * 1000
        print(f"    joint_trajectory  {ms:7.0f} ms  -> {p.name}")
        viz_files.append(("joint_trajectory.html", ms))

    # (b) 3D scene (start/goal + obstacles + EE path)
    t0 = time.perf_counter()
    fig_scene = plot_arm_scene_html(
        robot, scene, q_start, q_goal, waypoints=waypoints,
        title=f"Panda SBF-Dijkstra — {args.scene} (cost={cost:.3f})",
    )
    p = out_dir / "arm_scene.html"
    save_plotly_html(fig_scene, p)
    ms = (time.perf_counter() - t0) * 1000
    print(f"    arm_scene         {ms:7.0f} ms  -> {p.name}")
    viz_files.append(("arm_scene.html", ms))

    # (c) Ghost poses
    t0 = time.perf_counter()
    fig_ghost = plot_arm_poses_html(
        robot, scene, waypoints, n_ghosts=args.n_ghosts,
        title=f"Panda SBF-Dijkstra — Arm Pose Sequence ({n_wp} wp)",
    )
    p = out_dir / "arm_poses.html"
    save_plotly_html(fig_ghost, p)
    ms = (time.perf_counter() - t0) * 1000
    print(f"    arm_poses         {ms:7.0f} ms  -> {p.name}")
    viz_files.append(("arm_poses.html", ms))

    # (d) Animation
    t0 = time.perf_counter()
    fig_anim = create_animation_html(
        robot, scene, waypoints, n_frames=args.n_frames,
        title=f"Panda SBF-Dijkstra Path Animation",
    )
    p = out_dir / "animation.html"
    save_plotly_html(fig_anim, p)
    ms = (time.perf_counter() - t0) * 1000
    print(f"    animation         {ms:7.0f} ms  -> {p.name}")
    viz_files.append(("animation.html", ms))

    # ── 4) Summary ──
    total_s = time.perf_counter() - t_total
    print(f"\n{'=' * 60}")
    print(f"  Done ({total_s:.1f}s total)")
    print(f"{'=' * 60}")
    print(f"  Scene    : {args.scene} ({scene.n_obstacles} obstacles)")
    print(f"  Planning : cost={cost:.4f}, {n_wp} wp, {plan_s:.3f}s")
    print(f"  Files    :")
    for name, ms in viz_files:
        print(f"      {name}")
    print(f"\n  Open {out_dir / 'arm_scene.html'} in browser to view.")


if __name__ == "__main__":
    main()

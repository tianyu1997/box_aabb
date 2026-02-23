#!/usr/bin/env python3
"""
scripts/bench_sbf_vs_rrt_2dof.py — SBF vs OMPL RRT 2DOF 场景对比

对比方法:
  A) SBF-Dijkstra        (SafeBoxForest + Dijkstra + SOCP refine)
  B) OMPL-RRT            (C++ via WSL)
  C) OMPL-RRTConnect     (C++ via WSL)
  D) OMPL-RRT*           (C++ via WSL, anytime)
  E) OMPL-InformedRRT*   (C++ via WSL, anytime)
  F) OMPL-BIT*           (C++ via WSL, anytime)

测量指标:
  - 成功率
  - 总规划时间 (ms)
  - 首解时间 (ms) + 首解路径长度
  - 最终路径长度 (geodesic)
  - 碰撞检测次数
  - 路径碰撞验证 (用我方 CollisionChecker 逐段验证)

可视化:
  - 每个 seed 场景的 C-space 碰撞地图 + 所有方法路径叠加图
  - Anytime 算法代价收敛曲线

用法:
    cd v3
    python scripts/bench_sbf_vs_rrt_2dof.py --seeds 1 --start-seed 42 --timeout-rrt 5
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

# Windows UTF-8
if sys.platform == "win32":
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")

_ROOT = Path(__file__).resolve().parents[1]  # v3/
_SRC = _ROOT / "src"
for p in (_ROOT, _SRC):
    sp = str(p)
    if sp not in sys.path:
        sys.path.insert(0, sp)

from aabb.robot import load_robot
from forest.scene import Scene
from forest.collision import CollisionChecker
from baselines import SBFAdapter, PlanningResult
from planner.pipeline import PandaGCSConfig

# matplotlib (optional for viz)
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.colors as mcolors
    from matplotlib.patches import Rectangle
    HAS_MPL = True
except ImportError:
    HAS_MPL = False

# ═══════════════════════════════════════════════════════════════════
# Geodesic helpers
# ═══════════════════════════════════════════════════════════════════

def _compute_period(joint_limits):
    spans = [hi - lo for lo, hi in joint_limits]
    if not spans:
        return None
    span0 = spans[0]
    all_same = all(abs(s - span0) < 1e-6 for s in spans)
    return float(span0) if (all_same and abs(span0 - 2 * math.pi) < 0.1) else None


def _geo_path_length(waypoints, period):
    if period is None or waypoints is None or len(waypoints) < 2:
        return _euclidean_path_length(waypoints)
    half = period / 2.0
    total = 0.0
    for i in range(len(waypoints) - 1):
        diff = ((waypoints[i + 1] - waypoints[i]) + half) % period - half
        total += float(np.linalg.norm(diff))
    return total


def _euclidean_path_length(waypoints):
    if waypoints is None or len(waypoints) < 2:
        return float('nan')
    total = 0.0
    for i in range(len(waypoints) - 1):
        total += float(np.linalg.norm(waypoints[i + 1] - waypoints[i]))
    return total


def _geo_direct(q_start, q_goal, period):
    if period is None:
        return float(np.linalg.norm(q_goal - q_start))
    half = period / 2.0
    diff = ((q_goal - q_start) + half) % period - half
    return float(np.linalg.norm(diff))


# ═══════════════════════════════════════════════════════════════════
# Scene builder
# ═══════════════════════════════════════════════════════════════════

def build_random_2d_scene(robot, q_start, q_goal, rng, n_obstacles=8,
                          max_trials=500):
    """Build a random 2D scene that is NOT trivially solvable but IS feasible.

    Uses a quick RRT-Connect to verify feasibility.
    """
    from baselines.rrt_family import plan_rrt_connect

    jl = robot.joint_limits
    period = _compute_period(jl)

    for trial_i in range(max_trials):
        scene = Scene()
        for i in range(n_obstacles):
            cx = float(rng.uniform(-1.8, 1.8))
            cy = float(rng.uniform(-1.8, 1.8))
            w = float(rng.uniform(0.3, 0.8))
            h = float(rng.uniform(0.3, 0.8))
            scene.add_obstacle([cx - w / 2, cy - h / 2],
                               [cx + w / 2, cy + h / 2], name=f"obs_{i}")
        checker = CollisionChecker(robot=robot, scene=scene)
        if checker.check_config_collision(q_start):
            continue
        if checker.check_config_collision(q_goal):
            continue
        # reject if direct Euclidean path is free
        if not checker.check_segment_collision(q_start, q_goal, 0.03):
            continue
        # reject if geodesic wrap-around is free
        if period is not None and not checker.check_segment_collision(
                q_start, q_goal, 0.03, period=period):
            continue
        # quick feasibility check: RRT-Connect with 2s timeout
        res = plan_rrt_connect(
            q_start, q_goal, jl, checker,
            timeout=2.0, step_size=0.3, resolution=0.05,
            seed=trial_i + 7777, period=period)
        if not res['success']:
            continue
        return scene
    raise RuntimeError("Cannot build scene after max_trials")


# ═══════════════════════════════════════════════════════════════════
# Collision verification
# ═══════════════════════════════════════════════════════════════════

def verify_path_collision_free(waypoints, checker, resolution=0.03,
                               period=None, verbose=False):
    """Verify all segments of a path are collision-free using our checker.

    Returns (all_ok, n_segments, n_colliding, first_colliding_idx).
    """
    if waypoints is None or len(waypoints) < 2:
        return True, 0, 0, -1
    n_seg = len(waypoints) - 1
    n_col = 0
    first_col = -1
    for i in range(n_seg):
        # check endpoint collision
        if checker.check_config_collision(waypoints[i]):
            n_col += 1
            if first_col < 0:
                first_col = i
            if verbose:
                print(f"      seg[{i}] endpoint COLLIDES: {waypoints[i]}")
            continue
        # check segment collision
        if checker.check_segment_collision(
                waypoints[i], waypoints[i + 1], resolution, period=period):
            n_col += 1
            if first_col < 0:
                first_col = i
            if verbose:
                # Find exact collision point (geodesic interpolation)
                q1, q2 = waypoints[i], waypoints[i + 1]
                if period is not None:
                    _half = period / 2.0
                    _diff = ((q2 - q1) + _half) % period - _half
                    _dist = float(np.linalg.norm(_diff))
                else:
                    _diff = q2 - q1
                    _dist = float(np.linalg.norm(_diff))
                n_steps = max(2, int(_dist / 0.005))
                for k in range(n_steps + 1):
                    t = k / n_steps
                    q = q1 + t * _diff
                    if period is not None:
                        q = ((q + _half) % period) - _half
                    if checker.check_config_collision(q):
                        print(f"      seg[{i}] COLLIDES at t={t:.4f} "
                              f"q=[{q[0]:.4f},{q[1]:.4f}] "
                              f"(from [{q1[0]:.4f},{q1[1]:.4f}] "
                              f"to [{q2[0]:.4f},{q2[1]:.4f}], "
                              f"geo_dist={_dist:.3f}, n_steps={n_steps})")
                        break
    # check last config
    if checker.check_config_collision(waypoints[-1]):
        n_col += 1
        if first_col < 0:
            first_col = n_seg
        if verbose:
            print(f"      last endpoint COLLIDES: {waypoints[-1]}")
    return n_col == 0, n_seg, n_col, first_col


# ═══════════════════════════════════════════════════════════════════
# C-space visualization helpers
# ═══════════════════════════════════════════════════════════════════

def _build_collision_map(robot, scene, resolution=250):
    jl = robot.joint_limits
    xs = np.linspace(jl[0][0], jl[0][1], resolution)
    ys = np.linspace(jl[1][0], jl[1][1], resolution)
    checker = CollisionChecker(robot=robot, scene=scene)
    grid = np.zeros((resolution, resolution), dtype=np.uint8)
    for j, y in enumerate(ys):
        for i, x in enumerate(xs):
            if checker.check_config_collision(np.array([x, y])):
                grid[j, i] = 1
    return grid, [jl[0][0], jl[0][1], jl[1][0], jl[1][1]]


def _geodesic_points(qa, qb, period, n_pts=200):
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
    if period is None or period <= 0:
        ax.plot([qa[0], qb[0]], [qa[1], qb[1]], **kwargs)
        return
    pts = _geodesic_points(qa, qb, period, n_pts)
    segments = [[pts[0]]]
    half = period / 2.0
    for i in range(1, len(pts)):
        if np.any(np.abs(pts[i] - pts[i - 1]) > half):
            segments.append([pts[i]])
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
            kw.pop('label', None)
        first_drawn = False
        ax.plot(xs, ys, **kw)


def _draw_path(ax, waypoints, color='#00ff00', lw=2.5, label='Path',
               zorder=15, period=None, marker_size=3):
    xs = [w[0] for w in waypoints]
    ys = [w[1] for w in waypoints]
    ax.plot(xs, ys, 'o', color=color, markersize=marker_size,
            markeredgecolor='black', markeredgewidth=0.3, zorder=zorder + 1)
    first_seg = True
    for i in range(len(waypoints) - 1):
        kw = dict(color=color, linewidth=lw, alpha=0.9, zorder=zorder)
        if first_seg:
            kw['label'] = label
            first_seg = False
        _draw_wrapped_line(ax, waypoints[i], waypoints[i + 1], period, **kw)


def _draw_forest_boxes(ax, boxes, alpha=0.12, lw=0.3,
                       edge_color='#00ccff', face_color='#00ccff'):
    """Draw final SBF forest as light translucent rectangles."""
    if not boxes:
        return
    for lo_x, hi_x, lo_y, hi_y in boxes:
        rect = Rectangle((lo_x, lo_y), hi_x - lo_x, hi_y - lo_y,
                         linewidth=lw, edgecolor=edge_color,
                         facecolor=face_color, alpha=alpha, zorder=6)
        ax.add_patch(rect)


# method -> (color, linewidth, zorder)
_METHOD_STYLE = {
    'SBF-Dijkstra':        ('#00ff00', 3.0, 20),
    'OMPL-RRT':            ('#ff6600', 1.8, 15),
    'OMPL-RRTConnect':     ('#ff00ff', 1.8, 15),
    'OMPL-RRTstar':        ('#3399ff', 1.8, 16),
    'OMPL-InformedRRTstar': ('#ffcc00', 1.8, 16),
    'OMPL-BITstar':        ('#00cccc', 1.8, 16),
}


def visualize_scene_results(seed, q_start, q_goal, robot, scene,
                            sbf_viz, ompl_viz, period, out_dir):
    """C-space collision map + all method paths overlaid."""
    if not HAS_MPL:
        return

    jl = robot.joint_limits
    extent = [jl[0][0], jl[0][1], jl[1][0], jl[1][1]]

    print("    [viz] building collision map ...", end=" ", flush=True)
    cmap_grid, _ = _build_collision_map(robot, scene, resolution=250)
    print("done")

    fig, ax = plt.subplots(1, 1, figsize=(10, 10))
    ax.imshow(cmap_grid, origin="lower", extent=extent,
              cmap="Reds", alpha=0.30, aspect="auto")

    # Draw final SBF forest (if available)
    forest_boxes = sbf_viz.get('final_forest_boxes') if sbf_viz else None
    if forest_boxes:
        _draw_forest_boxes(ax, forest_boxes)

    # Draw SBF path
    if sbf_viz and sbf_viz.get('success') and sbf_viz.get('waypoints') is not None:
        wps = sbf_viz['waypoints']
        if isinstance(wps, np.ndarray):
            wps = [wps[i] for i in range(len(wps))]
        plen = sbf_viz.get('path_length', 0)
        sbf_ms = sbf_viz.get('time_ms', 0)
        col_tag = "✓" if sbf_viz.get('collision_ok', True) else "✗COL"
        c, lw, z = _METHOD_STYLE.get('SBF-Dijkstra', ('#00ff00', 3.0, 20))
        label = f"SBF-Dijkstra (L={plen:.2f}, {sbf_ms:.0f}ms) {col_tag}"
        _draw_path(ax, wps, color=c, lw=lw, label=label, zorder=z,
                   period=period, marker_size=5)

    # Draw OMPL paths
    for method_name, data in ompl_viz.items():
        wps = data.get('waypoints')
        if wps is None or len(wps) < 2:
            continue
        plen = data.get('path_length', 0)
        col_tag = "✓" if data.get('collision_ok', True) else "✗COL"
        first_ms = data.get('first_sol_ms', 0)
        first_len = data.get('first_sol_cost', float('nan'))
        first_len_s = f"{first_len:.2f}" if not math.isnan(first_len) else "?"
        label = (f"{method_name} (L={plen:.2f}, "
                 f"1st={first_ms:.0f}ms/{first_len_s}) {col_tag}")
        c, lw, z = _METHOD_STYLE.get(method_name, ('#888888', 1.5, 14))
        _draw_path(ax, wps, color=c, lw=lw, label=label, zorder=z,
                   period=period, marker_size=2)

    # Start / Goal
    ax.plot(q_start[0], q_start[1], 'o', color='cyan', markersize=12,
            markeredgecolor='black', markeredgewidth=1.5, zorder=30,
            label='Start')
    ax.plot(q_goal[0], q_goal[1], '*', color='yellow', markersize=16,
            markeredgecolor='black', markeredgewidth=1.0, zorder=30,
            label='Goal')

    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_xlabel("Joint 1 (rad)")
    ax.set_ylabel("Joint 2 (rad)")
    ax.set_title(f"Seed {seed} — SBF vs OMPL (2DOF C-space)")
    ax.legend(loc='upper left', fontsize=7, framealpha=0.8)
    ax.grid(False)
    fig.tight_layout()
    fig.savefig(out_dir / f"seed_{seed}_cspace.png", dpi=150)
    plt.close(fig)
    print(f"    [viz] saved seed_{seed}_cspace.png")

    # ── Anytime cost convergence plot ──
    has_history = any(d.get('cost_history') for d in ompl_viz.values())
    if has_history:
        fig2, ax2 = plt.subplots(1, 1, figsize=(8, 5))
        for mname, data in ompl_viz.items():
            ch = data.get('cost_history', [])
            if not ch:
                continue
            c, _, _ = _METHOD_STYLE.get(mname, ('#888888', 1, 10))
            ax2.plot([p[0] for p in ch], [p[1] for p in ch],
                     '-o', color=c, markersize=2, label=mname)
        if sbf_viz and sbf_viz.get('success'):
            c, _, _ = _METHOD_STYLE['SBF-Dijkstra']
            sbf_len = sbf_viz.get('path_length', 0)
            sbf_t = sbf_viz.get('time_ms', 0) / 1000
            ax2.axhline(y=sbf_len, color=c, linestyle='--', linewidth=2,
                        label=f'SBF-Dijkstra (L={sbf_len:.2f}, {sbf_t*1000:.0f}ms)')
            ax2.axvline(x=sbf_t, color=c, linestyle=':', linewidth=1, alpha=0.5)
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Path Length")
        ax2.set_title(f"Seed {seed} — Anytime Cost Convergence")
        ax2.legend(fontsize=8)
        ax2.grid(False)
        fig2.tight_layout()
        fig2.savefig(out_dir / f"seed_{seed}_cost_history.png", dpi=150)
        plt.close(fig2)
        print(f"    [viz] saved seed_{seed}_cost_history.png")


# ═══════════════════════════════════════════════════════════════════
# OMPL via WSL
# ═══════════════════════════════════════════════════════════════════

_BRIDGE_PATH = "/mnt/c/Users/TIAN/Documents/box_aabb/v3/scripts/ompl_bridge_2dof.py"


def run_ompl_2dof(q_start, q_goal, joint_limits, scene, algorithms,
                  timeout=5.0, seed=42, step_size=0.3):
    """Call OMPL bridge via WSL subprocess. Returns dict or None on failure."""
    obs_list = []
    for obs in scene.get_obstacles():
        obs_list.append({
            "min_point": obs.min_point.tolist(),
            "max_point": obs.max_point.tolist(),
            "name": obs.name,
        })

    problem = {
        "robot_name": "2dof_planar",
        "q_start": q_start.tolist(),
        "q_goal": q_goal.tolist(),
        "joint_limits": [[float(lo), float(hi)] for lo, hi in joint_limits],
        "obstacles": obs_list,
        "algorithms": algorithms,
        "timeout": timeout,
        "trials": 1,
        "seed": seed,
        "step_size": step_size,
    }

    try:
        proc = subprocess.run(
            ["wsl", "python3", _BRIDGE_PATH],
            input=json.dumps(problem),
            capture_output=True, text=True,
            encoding='utf-8', errors='replace',
            timeout=timeout * len(algorithms) + 60,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return None

    # Print bridge stderr for debugging
    if proc.stderr and proc.stderr.strip():
        for line in proc.stderr.strip().split('\n'):
            if line.strip():
                print(f"    [bridge] {line.strip()}")

    stdout = proc.stdout.strip()
    if not stdout:
        return None

    # Find JSON start (skip WSL warnings)
    json_start = stdout.find('{')
    if json_start < 0:
        return None
    try:
        return json.loads(stdout[json_start:])
    except json.JSONDecodeError:
        return None


# ═══════════════════════════════════════════════════════════════════
# Single seed trial
# ═══════════════════════════════════════════════════════════════════

def run_single_seed(seed, robot, q_start, q_goal, timeout_rrt, timeout_sbf,
                    n_obstacles, ompl_algos, out_dir=None):
    """Run all planners on one random scene.

    Returns (results_list, scene, sbf_viz, ompl_viz).
    """
    rng = np.random.default_rng(seed)
    jl = robot.joint_limits
    period = _compute_period(jl)

    try:
        scene = build_random_2d_scene(robot, q_start, q_goal, rng, n_obstacles)
    except RuntimeError:
        print(f"  seed={seed}: SKIP (no valid scene)")
        return [], None, None, {}

    checker = CollisionChecker(robot=robot, scene=scene)
    direct_dist = _geo_direct(q_start, q_goal, period)
    results = []
    sbf_viz = None
    ompl_viz = {}

    # ── SBF-Dijkstra ──────────────────────────────────────────
    try:
        sbf_dij = SBFAdapter(method="dijkstra")
        sbf_cfg = {
            'seed': seed,
            'max_boxes': 400,
            'max_consecutive_miss': 50,
        }
        sbf_dij.setup(robot, scene, sbf_cfg)
        t0 = time.perf_counter()
        res = sbf_dij.plan(q_start, q_goal, timeout=timeout_sbf)
        dt = time.perf_counter() - t0

        wps_arr = res.path if res.path is not None else None
        path_len = _geo_path_length(wps_arr, period)

        # collision verification
        col_ok = True
        if wps_arr is not None and len(wps_arr) >= 2:
            wps_list = [wps_arr[i] for i in range(len(wps_arr))]
            ok, n_seg, n_col, first_col = verify_path_collision_free(
                wps_list, checker, 0.03, period=period)
            col_ok = ok
            if not ok:
                print(f"    SBF-Dijkstra !!COLLISION "
                      f"{n_col}/{n_seg} segs, first@{first_col}")

        r = {
            'method': 'SBF-Dijkstra',
            'success': res.success,
            'time_ms': dt * 1000,
            'first_sol_ms': dt * 1000,
            'first_sol_cost': path_len,
            'path_length': path_len,
            'path_ratio': path_len / direct_dist if res.success else float('nan'),
            'n_nodes': res.nodes_explored,
            'n_cc': res.collision_checks,
            'collision_ok': col_ok,
        }
        results.append(r)

        sbf_viz = {
            'success': res.success,
            'waypoints': wps_arr,
            'path_length': path_len,
            'time_ms': dt * 1000,
            'collision_ok': col_ok,
            'phase_times': dict(res.phase_times),
            'metadata': dict(res.metadata),
            'final_forest_boxes': [
                (float(b.joint_intervals[0][0]), float(b.joint_intervals[0][1]),
                 float(b.joint_intervals[1][0]), float(b.joint_intervals[1][1]))
                for b in getattr(sbf_dij, '_prep', {}).get('boxes', {}).values()
            ],
        }

    except Exception as e:
        results.append({
            'method': 'SBF-Dijkstra',
            'success': False,
            'time_ms': 0, 'first_sol_ms': 0, 'first_sol_cost': float('nan'),
            'path_length': float('nan'), 'path_ratio': float('nan'),
            'n_nodes': 0, 'n_cc': 0, 'collision_ok': True,
            'error': str(e),
        })

    # ── OMPL via WSL ──────────────────────────────────────────
    ompl_raw = run_ompl_2dof(
        q_start, q_goal, jl, scene, ompl_algos,
        timeout=timeout_rrt, seed=seed, step_size=0.3)

    if ompl_raw is not None:
        for algo in ompl_algos:
            data = ompl_raw.get(algo, {})
            method_name = f'OMPL-{algo}'

            if "error" in data:
                results.append({
                    'method': method_name,
                    'success': False,
                    'time_ms': 0, 'first_sol_ms': 0,
                    'first_sol_cost': float('nan'),
                    'path_length': float('nan'), 'path_ratio': float('nan'),
                    'n_nodes': 0, 'n_cc': 0, 'collision_ok': True,
                    'error': data['error'],
                })
                continue

            trials_data = data.get("trials", [])
            if not trials_data:
                results.append({
                    'method': method_name,
                    'success': False,
                    'time_ms': 0, 'first_sol_ms': 0,
                    'first_sol_cost': float('nan'),
                    'path_length': float('nan'), 'path_ratio': float('nan'),
                    'n_nodes': 0, 'n_cc': 0, 'collision_ok': True,
                })
                continue

            t = trials_data[0]
            succ = t.get("success", False)
            wps_raw = data.get("best_waypoints", [])
            wps = [np.array(w) for w in wps_raw] if wps_raw else None

            # ── Geodesic post-processing for torus topology ──
            # OMPL and our verifier use different sampling grids, so
            # a path that barely grazes an obstacle may pass OMPL's
            # checker but fail ours.  Fix: resample finely, remove
            # collision-config points, then shortcut with OUR checker
            # at the SAME resolution the verifier uses (0.03).
            if succ and wps and len(wps) >= 2 and period is not None:
                from planner.path_smoother import PathSmoother
                _psm = PathSmoother(checker, segment_resolution=0.03,
                                    period=period)
                _resampled = _psm.resample(wps, resolution=0.02)
                # Remove config-colliding waypoints
                _clean = [_resampled[0]]
                for _k in range(1, len(_resampled)):
                    if not checker.check_config_collision(_resampled[_k]):
                        _clean.append(_resampled[_k])
                if len(_clean) >= 2:
                    wps = _psm.shortcut(_clean, max_iters=300,
                                        rng=np.random.default_rng(seed + 999))

            # geodesic path length (recomputed our side)
            pl = _geo_path_length(wps, period) if succ and wps else float('nan')

            # first solution data
            fst_time = t.get("first_solution_time", t.get("plan_time_s", 0))
            fst_cost = t.get("first_solution_cost", None)
            if fst_cost is None or (isinstance(fst_cost, float)
                                    and math.isnan(fst_cost)):
                fst_cost = pl

            # collision verification (SO2×SO2 → geodesic interpolation)
            col_ok = True
            if succ and wps and len(wps) >= 2:
                ok, n_seg, n_col, first_col = verify_path_collision_free(
                    wps, checker, 0.03, period=period, verbose=False)
                col_ok = ok
                if not ok:
                    print(f"    {method_name} !!COLLISION "
                          f"{n_col}/{n_seg} segs, first@{first_col}")

            cost_history = t.get("cost_history", [])

            r = {
                'method': method_name,
                'success': succ,
                'time_ms': t.get("plan_time_s", 0) * 1000,
                'first_sol_ms': fst_time * 1000,
                'first_sol_cost': fst_cost,
                'path_length': pl,
                'path_ratio': pl / direct_dist if succ else float('nan'),
                'n_nodes': t.get("n_nodes", 0),
                'n_cc': t.get("n_collision_checks", 0),
                'collision_ok': col_ok,
                'cost_history': cost_history,
            }
            results.append(r)

            ompl_viz[method_name] = {
                'success': succ,
                'waypoints': wps,
                'path_length': pl,
                'first_sol_ms': fst_time * 1000,
                'first_sol_cost': fst_cost,
                'collision_ok': col_ok,
                'cost_history': cost_history,
            }
    else:
        for algo in ompl_algos:
            results.append({
                'method': f'OMPL-{algo}',
                'success': False,
                'time_ms': 0, 'first_sol_ms': 0,
                'first_sol_cost': float('nan'),
                'path_length': float('nan'), 'path_ratio': float('nan'),
                'n_nodes': 0, 'n_cc': 0, 'collision_ok': True,
                'error': 'WSL bridge failed',
            })

    # ── Visualization (excluded from timing) ──
    if out_dir is not None:
        try:
            visualize_scene_results(
                seed, q_start, q_goal, robot, scene,
                sbf_viz, ompl_viz, period, out_dir)
        except Exception as e:
            print(f"    [viz] error: {e}")

    return results, scene, sbf_viz, ompl_viz


# ═══════════════════════════════════════════════════════════════════
# SBF phase timing report
# ═══════════════════════════════════════════════════════════════════

def _print_sbf_phase_report(all_sbf_details: Dict[int, dict]):
    """Print a detailed per-seed and aggregate SBF phase timing report."""
    if not all_sbf_details:
        print("\n[SBF Phase Report] No SBF data collected.")
        return

    # Collect phase keys in display order
    grow_keys = [
        ('warmup_ms',       'warmup_fk'),
        ('obs_pack_ms',     'obs_pack'),
        ('seed_anchor_ms',  'seed_anchor'),
        ('serial_prep_ms',  'serial_prep'),
        ('sample_ms',       'sample_batch'),
        ('boundary_ms',     'boundary_expand'),
        ('is_occupied_ms',  'is_occupied'),
        ('probe_ms',        'can_expand'),
        ('find_free_box_ms','find_free_box'),
        ('volume_check_ms', 'volume_check'),
        ('add_box_ms',      'add_box'),
        ('export_boxes_ms', 'export_boxes'),
        ('overhead_ms',     'grow_overhead'),
    ]
    top_keys = [
        ('grow',    'grow_total'),
        ('solve',   'solve'),
        ('repair',  'repair'),
    ]

    # Build per-seed rows
    rows = []
    for seed in sorted(all_sbf_details.keys()):
        d = all_sbf_details[seed]
        pt = d.get('phase_times', {})
        meta = d.get('metadata', {})
        gd = meta.get('grow_detail', {})
        row = {'seed': seed}
        # grow sub-phases from grow_detail
        for key, _ in grow_keys:
            row[key] = gd.get(key, 0.0)
        # top-level phases (in seconds -> ms)
        # NOTE:
        # - phase_times['grow'] 口径可能与 grow_detail 子项不完全一致
        #   (例如并行/线程 join/计时边界差异)
        # - 为保证“各子项可加和”，报表统一使用子项重算 grow_total。
        grow_components_ms = sum(row[key] for key, _ in grow_keys)
        row['grow_ms'] = grow_components_ms
        row['grow_ms_wall'] = pt.get('grow', 0.0) * 1000
        row['solve_ms'] = pt.get('solve', 0.0) * 1000
        row['repair_ms'] = pt.get('repair', 0.0) * 1000
        row['coarsen_ms'] = meta.get('coarsen_ms', 0.0)
        row['adj_ms'] = meta.get('adj_ms', 0.0)
        row['bridge_ms'] = meta.get('bridge_ms', 0.0)
        row['plan_ms'] = meta.get('plan_ms', 0.0)
        row['post_safe_ms'] = meta.get('post_safe_ms', 0.0)
        row['plan_graph_ms'] = meta.get('plan_graph_ms', 0.0)
        row['plan_waypoints_ms'] = meta.get('plan_waypoints_ms', 0.0)
        row['plan_refine_ms'] = meta.get('plan_refine_ms', 0.0)
        row['plan_shortcut_ms'] = meta.get('plan_shortcut_ms', 0.0)
        row['cache_start_ms'] = meta.get('cache_start_ms', 0.0)
        row['box_export_ms'] = meta.get('box_export_ms', 0.0)
        row['prepare_misc_ms'] = meta.get('prepare_misc_ms', 0.0)
        row['growprep_total_ms'] = meta.get('growprep_total_ms', 0.0)
        row['cache_wait_ms'] = meta.get('cache_wait_ms', 0.0)
        # 统一口径 TOTAL = grow_total + coarsen + solve + repair
        row['total_ms'] = (
            row['grow_ms'] + row['coarsen_ms'] + row['solve_ms'] + row['repair_ms']
        )
        # 原 wall-clock（用于对照）
        row['total_ms_wall'] = d.get('time_ms', 0.0)
        # 第二轮：Δ 细分定位
        # grow_unacct: phase_times['grow'] 与 (grow_detail + coarsen) 的差
        row['grow_unacct_ms'] = (
            row['grow_ms_wall'] - (row['grow_ms'] + row['coarsen_ms'])
        )
        row['grow_known_extra_ms'] = (
            row['cache_start_ms'] + row['box_export_ms'] + row['prepare_misc_ms']
        )
        row['grow_residual_ms'] = (
            row['grow_unacct_ms'] - row['grow_known_extra_ms']
        )
        # solve_unacct: solve 总耗时 与 (adj + bridge + plan) 的差
        row['solve_unacct_ms'] = (
            row['solve_ms'] - (
                row['adj_ms'] + row['bridge_ms'] + row['plan_ms'] + row['post_safe_ms']
            )
        )
        # wrapper_overhead: plan() 外围开销
        row['wrapper_overhead_ms'] = (
            row['total_ms_wall'] - (row['grow_ms_wall'] + row['solve_ms'] + row['repair_ms'])
        )
        rows.append(row)

    n = len(rows)

    # ── Per-seed table ──
    print("\n" + "=" * 120)
    print("  SBF Phase Timing Report (per seed)")
    print("=" * 120)

    hdr_parts = [f"{'Seed':>5s}"]
    for _, label in grow_keys:
        hdr_parts.append(f"{label:>14s}")
    hdr_parts += [f"{'grow_total':>11s}", f"{'coarsen':>9s}",
                  f"{'solve':>9s}", f"{'repair':>9s}",
                  f"{'TOTAL':>9s}", f"{'WALL':>9s}", f"{'Δ':>8s}"]
    print("  ".join(hdr_parts))
    print("-" * 120)

    for row in rows:
        parts = [f"{row['seed']:>5d}"]
        for key, _ in grow_keys:
            parts.append(f"{row[key]:>14.1f}")
        parts.append(f"{row['grow_ms']:>11.1f}")
        parts.append(f"{row['coarsen_ms']:>9.1f}")
        parts.append(f"{row['solve_ms']:>9.1f}")
        parts.append(f"{row['repair_ms']:>9.1f}")
        parts.append(f"{row['total_ms']:>9.1f}")
        parts.append(f"{row['total_ms_wall']:>9.1f}")
        parts.append(f"{(row['total_ms_wall'] - row['total_ms']):>8.1f}")
        print("  ".join(parts))

    # ── Average row ──
    print("-" * 120)
    avg_parts = [f"{'AVG':>5s}"]
    for key, _ in grow_keys:
        avg_val = np.mean([r[key] for r in rows])
        avg_parts.append(f"{avg_val:>14.1f}")
    avg_parts.append(f"{np.mean([r['grow_ms'] for r in rows]):>11.1f}")
    avg_parts.append(f"{np.mean([r['coarsen_ms'] for r in rows]):>9.1f}")
    avg_parts.append(f"{np.mean([r['solve_ms'] for r in rows]):>9.1f}")
    avg_parts.append(f"{np.mean([r['repair_ms'] for r in rows]):>9.1f}")
    avg_parts.append(f"{np.mean([r['total_ms'] for r in rows]):>9.1f}")
    avg_parts.append(f"{np.mean([r['total_ms_wall'] for r in rows]):>9.1f}")
    avg_parts.append(
        f"{np.mean([r['total_ms_wall'] - r['total_ms'] for r in rows]):>8.1f}")
    print("  ".join(avg_parts))
    print("=" * 120)

    # ── Delta decomposition (AVG) ──
    avg_delta = np.mean([r['total_ms_wall'] - r['total_ms'] for r in rows])
    avg_grow_u = np.mean([r['grow_unacct_ms'] for r in rows])
    avg_solve_u = np.mean([r['solve_unacct_ms'] for r in rows])
    avg_wrap_u = np.mean([r['wrapper_overhead_ms'] for r in rows])
    avg_cache_wait = np.mean([r['cache_wait_ms'] for r in rows])
    avg_adj = np.mean([r['adj_ms'] for r in rows])
    avg_bridge = np.mean([r['bridge_ms'] for r in rows])
    avg_plan = np.mean([r['plan_ms'] for r in rows])
    avg_post_safe = np.mean([r['post_safe_ms'] for r in rows])
    avg_plan_graph = np.mean([r['plan_graph_ms'] for r in rows])
    avg_plan_waypoints = np.mean([r['plan_waypoints_ms'] for r in rows])
    avg_plan_refine = np.mean([r['plan_refine_ms'] for r in rows])
    avg_plan_shortcut = np.mean([r['plan_shortcut_ms'] for r in rows])
    avg_cache_start = np.mean([r['cache_start_ms'] for r in rows])
    avg_box_export = np.mean([r['box_export_ms'] for r in rows])
    avg_prepare_misc = np.mean([r['prepare_misc_ms'] for r in rows])
    avg_grow_known_extra = np.mean([r['grow_known_extra_ms'] for r in rows])
    avg_grow_residual = np.mean([r['grow_residual_ms'] for r in rows])
    print("\n  Delta decomposition (AVG, ms):")
    print(f"    Δ(total)          = {avg_delta:8.1f}")
    print(f"    grow_unacct       = {avg_grow_u:8.1f}  "
          f"(grow_wall - (grow_total + coarsen))")
    print(f"    solve_unacct      = {avg_solve_u:8.1f}  "
          f"(solve - (adj + bridge + plan + post_safe))")
    print(f"    wrapper_overhead  = {avg_wrap_u:8.1f}  "
          f"(wall - (grow_wall + solve + repair))")
    print(f"    cache_wait        = {avg_cache_wait:8.1f}  "
          f"(subset of wrapper when wait enabled)")
    print("\n  Solve decomposition (AVG, ms):")
    print(f"    solve             = {np.mean([r['solve_ms'] for r in rows]):8.1f}")
    print(f"    adj               = {avg_adj:8.1f}")
    print(f"    bridge            = {avg_bridge:8.1f}")
    print(f"    plan              = {avg_plan:8.1f}")
    print(f"      plan_graph      = {avg_plan_graph:8.1f}")
    print(f"      plan_waypoints  = {avg_plan_waypoints:8.1f}")
    print(f"      plan_refine     = {avg_plan_refine:8.1f}")
    print(f"      plan_shortcut   = {avg_plan_shortcut:8.1f}")
    print(f"    post_safe         = {avg_post_safe:8.1f}")
    print("\n  Grow decomposition (AVG, ms):")
    print(f"    grow_unacct       = {avg_grow_u:8.1f}")
    print(f"      cache_start     = {avg_cache_start:8.1f}")
    print(f"      box_export      = {avg_box_export:8.1f}")
    print(f"      prepare_misc    = {avg_prepare_misc:8.1f}")
    print(f"      known_extra_sum = {avg_grow_known_extra:8.1f}")
    print(f"      residual        = {avg_grow_residual:8.1f}")

    # ── Percentage breakdown (of grow_total) ──
    print("\n  Grow-phase breakdown (% of grow_total):")
    avg_grow = np.mean([r['grow_ms'] for r in rows])
    if avg_grow > 0:
        for key, label in grow_keys:
            avg_val = np.mean([r[key] for r in rows])
            pct = avg_val / avg_grow * 100
            bar = "█" * int(pct / 2) + "░" * (50 - int(pct / 2))
            print(f"    {label:<16s} {avg_val:>8.1f} ms  ({pct:>5.1f}%)  {bar}")

    # ── Top-level breakdown (% of total) ──
    avg_total = np.mean([r['total_ms'] for r in rows])
    print(f"\n  Top-level breakdown (% of TOTAL(calc) = {avg_total:.0f} ms):")
    if avg_total > 0:
        for label, key in [('grow', 'grow_ms'), ('coarsen', 'coarsen_ms'),
                           ('solve', 'solve_ms'), ('repair', 'repair_ms')]:
            avg_val = np.mean([r[key] for r in rows])
            pct = avg_val / avg_total * 100
            bar = "█" * int(pct / 2) + "░" * (50 - int(pct / 2))
            print(f"    {label:<16s} {avg_val:>8.1f} ms  ({pct:>5.1f}%)  {bar}")
    print()


# ═══════════════════════════════════════════════════════════════════
# Aggregation & display
# ═══════════════════════════════════════════════════════════════════

def aggregate_and_print(all_results: Dict[str, List[dict]], n_seeds: int):
    """Aggregate per-method results across seeds and print table."""
    methods_order = []
    method_data: Dict[str, List[dict]] = {}
    for seed_results in all_results.values():
        for r in seed_results:
            m = r['method']
            if m not in method_data:
                method_data[m] = []
                methods_order.append(m)
            method_data[m].append(r)

    print("\n" + "=" * 130)
    print(f"{'Method':<22s} {'Succ':>5s} {'Time_ms':>9s} {'1st_ms':>9s} "
          f"{'1stLen':>9s} {'PathLen':>9s} {'Ratio':>7s} "
          f"{'Nodes':>7s} {'CC':>9s} {'ColOK':>6s}")
    print("-" * 130)

    for m in methods_order:
        data = method_data[m]
        n_total = len(data)
        successes = [d for d in data if d['success']]
        n_succ = len(successes)
        succ_rate = f"{n_succ}/{n_total}"

        if n_succ > 0:
            avg_time = np.mean([d['time_ms'] for d in successes])
            avg_first = np.mean([d['first_sol_ms'] for d in successes])
            avg_first_cost = np.nanmean([
                d.get('first_sol_cost', float('nan')) for d in successes])
            avg_len = np.nanmean([d['path_length'] for d in successes])
            avg_ratio = np.nanmean([d['path_ratio'] for d in successes])
            avg_nodes = np.mean([d['n_nodes'] for d in successes])
            avg_cc = np.mean([d['n_cc'] for d in successes])
            n_col_ok = sum(1 for d in successes if d.get('collision_ok', True))
            col_ok_str = f"{n_col_ok}/{n_succ}"
            print(f"{m:<22s} {succ_rate:>5s} {avg_time:>9.1f} {avg_first:>9.1f} "
                  f"{avg_first_cost:>9.3f} {avg_len:>9.3f} {avg_ratio:>7.3f} "
                  f"{avg_nodes:>7.0f} {avg_cc:>9.0f} {col_ok_str:>6s}")
        else:
            print(f"{m:<22s} {succ_rate:>5s} {'---':>9s} {'---':>9s} "
                  f"{'---':>9s} {'---':>9s} {'---':>7s} "
                  f"{'---':>7s} {'---':>9s} {'---':>6s}")

    print("=" * 130)
    print("  Time_ms  = total wall-clock time (grow + solve)")
    print("  1st_ms   = time to first solution")
    print("  1stLen   = path length at first solution")
    print("  PathLen  = final geodesic path length")
    print("  Ratio    = path_length / geodesic_direct_distance")
    print("  Nodes    = tree nodes (RRT) or box count (SBF)")
    print("  CC       = collision checks")
    print("  ColOK    = collision verification with our CollisionChecker")


# ═══════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="SBF vs OMPL RRT family 2DOF benchmark")
    parser.add_argument("--seeds", type=int, default=30,
                        help="Number of random seeds (default: 30)")
    parser.add_argument("--start-seed", type=int, default=1)
    parser.add_argument("--timeout-rrt", type=float, default=5.0,
                        help="Timeout for RRT methods (seconds)")
    parser.add_argument("--timeout-sbf", type=float, default=30.0,
                        help="Timeout for SBF methods (seconds)")
    parser.add_argument("--n-obs", type=int, default=8,
                        help="Number of obstacles per scene")
    parser.add_argument("--no-viz", action="store_true",
                        help="Skip visualization (faster)")
    args = parser.parse_args()

    robot = load_robot("2dof_planar")
    q_start = np.array([0.8 * np.pi, 0.2])
    q_goal = np.array([-0.7 * np.pi, -0.4])
    jl = robot.joint_limits
    period = _compute_period(jl)

    ompl_algos = ["RRTConnect", "BITstar"]

    # Output directory
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = _ROOT / "experiments" / "output" / f"sbf_vs_rrt_2dof_{ts}"
    out_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 130)
    print("  SBF vs OMPL RRT Family — 2DOF Planar Robot Benchmark")
    print(f"  Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"  Seeds: {args.start_seed} .. {args.start_seed + args.seeds - 1}")
    print(f"  Obstacles: {args.n_obs}")
    print(f"  Timeout: RRT={args.timeout_rrt}s  SBF={args.timeout_sbf}s")
    print(f"  q_start={q_start}  q_goal={q_goal}")
    print(f"  period={period}")
    print(f"  OMPL: {', '.join(ompl_algos)}")
    print(f"  Output: {out_dir}")
    print("=" * 130)

    all_results = {}
    all_sbf_details = {}  # seed -> grow detail dict
    for i in range(args.seeds):
        seed = args.start_seed + i
        print(f"\n[seed {seed}] ({i + 1}/{args.seeds})")
        t0 = time.perf_counter()
        results, scene, sbf_viz, ompl_viz = run_single_seed(
            seed, robot, q_start, q_goal,
            timeout_rrt=args.timeout_rrt,
            timeout_sbf=args.timeout_sbf,
            n_obstacles=args.n_obs,
            ompl_algos=ompl_algos,
            out_dir=out_dir if not args.no_viz else None,
        )
        dt = time.perf_counter() - t0
        all_results[seed] = results
        if sbf_viz and sbf_viz.get('phase_times'):
            all_sbf_details[seed] = sbf_viz

        # per-seed summary
        for r in results:
            status = "OK" if r['success'] else "FAIL"
            pl_s = (f"{r['path_length']:.3f}"
                    if not math.isnan(r.get('path_length', float('nan')))
                    else "---")
            fst_s = (f"{r.get('first_sol_cost', float('nan')):.3f}"
                     if not math.isnan(r.get('first_sol_cost', float('nan')))
                     else "---")
            tm = f"{r['time_ms']:.1f}ms"
            col = "✓" if r.get('collision_ok', True) else "✗COL"
            err = f"  ({r['error']})" if 'error' in r else ""
            print(f"  {r['method']:<22s} {status:>4s}  {tm:>10s}  "
                  f"1st={r.get('first_sol_ms',0):.0f}ms/{fst_s}  "
                  f"len={pl_s}  {col}{err}")
        print(f"  --- seed {seed} total: {dt:.1f}s ---")

    # ── Final summary ──
    aggregate_and_print(all_results, args.seeds)

    # ── SBF phase timing report ──
    _print_sbf_phase_report(all_sbf_details)

    # ── Save JSON ──
    def _to_json(obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, (np.floating, np.float64)):
            return float(obj)
        if isinstance(obj, (np.integer, np.int64)):
            return int(obj)
        return str(obj)

    out_path = out_dir / "results.json"
    with open(out_path, 'w', encoding='utf-8') as f:
        json.dump({
            'config': {
                'seeds': args.seeds,
                'start_seed': args.start_seed,
                'timeout_rrt': args.timeout_rrt,
                'timeout_sbf': args.timeout_sbf,
                'n_obs': args.n_obs,
                'q_start': q_start.tolist(),
                'q_goal': q_goal.tolist(),
                'period': period,
                'ompl_algos': ompl_algos,
            },
            'results': {str(k): v for k, v in all_results.items()},
        }, f, indent=2, default=_to_json, ensure_ascii=False)
    print(f"\nResults saved to {out_path}")
    print(f"Output directory: {out_dir}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Debug script: understand why the SBF repair fails on seed 44."""
import sys, math
import numpy as np
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
_SRC = _ROOT / "src"
for p in (_ROOT, _SRC):
    sp = str(p)
    if sp not in sys.path:
        sys.path.insert(0, sp)

from aabb.robot import load_robot
from forest.scene import Scene
from forest.collision import CollisionChecker
from baselines import SBFAdapter
from baselines.rrt_family import plan_rrt_connect
from planner.pipeline import (
    PandaGCSConfig, grow_and_prepare, run_method_with_bridge,
    _solve_method_dijkstra,
)

# ── Build scene (same as benchmark) ──
robot = load_robot("2dof_planar")
q_start = np.array([2.51327412, 0.2])
q_goal  = np.array([-2.19911486, -0.4])
period = 2 * math.pi
half = period / 2.0

rng = np.random.default_rng(44)
for trial_i in range(500):
    scene = Scene()
    for i in range(8):
        cx = float(rng.uniform(-1.8, 1.8))
        cy = float(rng.uniform(-1.8, 1.8))
        w  = float(rng.uniform(0.3, 0.8))
        h  = float(rng.uniform(0.3, 0.8))
        scene.add_obstacle([cx-w/2, cy-h/2], [cx+w/2, cy+h/2], name=f"obs_{i}")
    checker = CollisionChecker(robot=robot, scene=scene)
    if checker.check_config_collision(q_start): continue
    if checker.check_config_collision(q_goal):  continue
    if not checker.check_segment_collision(q_start, q_goal, 0.03): continue
    if not checker.check_segment_collision(q_start, q_goal, 0.03, period=period): continue
    res = plan_rrt_connect(q_start, q_goal, robot.joint_limits, checker,
                           timeout=2.0, step_size=0.3, resolution=0.05,
                           seed=trial_i+7777, period=period)
    if not res['success']: continue
    break

print(f"Scene built OK")

# ── Get raw SBF waypoints (no repair) via pipeline directly ──
cfg = PandaGCSConfig()
cfg.dijkstra_use_socp = True
cfg.shortcut_strategy = "greedy"
cfg.seed = 44
cfg.max_boxes = 400
cfg.max_consecutive_miss = 50

ndim = robot.n_joints
prep = grow_and_prepare(robot, scene, cfg, q_start, q_goal, ndim, no_cache=True)
ct = prep.get('_cache_thread')
if ct is not None:
    ct.join()

raw = run_method_with_bridge(
    _solve_method_dijkstra, "Dijkstra",
    prep, cfg, q_start, q_goal, ndim)

if not raw.get('success'):
    print("SBF plan failed!")
    sys.exit(1)

wps = np.array(raw['waypoints'], dtype=np.float64)
print(f"\n=== Raw waypoints: {len(wps)} points ===")
for i, w in enumerate(wps):
    col = checker.check_config_collision(w)
    print(f"  wp[{i}] = [{w[0]:+.6f}, {w[1]:+.6f}]  in_col={col}")

print(f"\n=== Segment-by-segment analysis ===")
for i in range(len(wps)-1):
    q1, q2 = wps[i], wps[i+1]
    euc_col = checker.check_segment_collision(q1, q2, 0.02)
    geo_col = checker.check_segment_collision(q1, q2, 0.02, period=period)
    
    diff_euc = q2 - q1
    diff_geo = ((q2 - q1) + half) % period - half
    d_euc = np.linalg.norm(diff_euc)
    d_geo = np.linalg.norm(diff_geo)
    
    wraps = [abs(d) > half - 0.01 for d in (q2 - q1)]
    
    status = ""
    if geo_col and not euc_col:
        status = " *** GEODESIC-ONLY COLLISION ***"
    elif geo_col and euc_col:
        status = " *** BOTH COLLIDE ***"
    
    print(f"  seg[{i}] d_euc={d_euc:.4f} d_geo={d_geo:.4f} "
          f"euc_col={euc_col} geo_col={geo_col} "
          f"wraps={wraps}{status}")
    
    if geo_col:
        # Find the geodesic collision point
        n_steps = max(2, int(d_geo / 0.005))
        for k in range(n_steps+1):
            t = k / n_steps
            q_geo = q1 + t * diff_geo
            q_geo = ((q_geo + half) % period) - half
            if checker.check_config_collision(q_geo):
                print(f"    First geo collision at t={t:.4f} q=[{q_geo[0]:+.4f},{q_geo[1]:+.4f}]")
                break
        
        # Show Euclidean midpoint analysis
        mid = 0.5 * (q1 + q2)
        mid_n = ((mid + half) % period) - half
        mid_col = checker.check_config_collision(mid_n)
        
        diff1_geo = ((mid_n - q1) + half) % period - half
        diff2_geo = ((q2 - mid_n) + half) % period - half
        
        print(f"    Euc midpoint = [{mid_n[0]:+.4f},{mid_n[1]:+.4f}] in_col={mid_col}")
        print(f"    sub1 geodiff = [{diff1_geo[0]:+.4f},{diff1_geo[1]:+.4f}] "
              f"|max|={max(abs(diff1_geo)):.4f} (pi={math.pi:.4f})")
        print(f"    sub2 geodiff = [{diff2_geo[0]:+.4f},{diff2_geo[1]:+.4f}] "
              f"|max|={max(abs(diff2_geo)):.4f}")
        
        geo_col_1 = checker.check_segment_collision(q1, mid_n, 0.02, period=period)
        geo_col_2 = checker.check_segment_collision(mid_n, q2, 0.02, period=period)
        print(f"    sub1 geo_col={geo_col_1}  sub2 geo_col={geo_col_2}")

print(f"\n=== Now test the adapter with repair ===")
sbf_dij = SBFAdapter(method="dijkstra")
sbf_cfg = {'seed': 44, 'max_boxes': 400, 'max_consecutive_miss': 50}
sbf_dij.setup(robot, scene, sbf_cfg)
result = sbf_dij.plan(q_start, q_goal, timeout=30.0)
if result.path is not None:
    rp = result.path
    print(f"Repaired path: {len(rp)} waypoints")
    n_col = 0
    first_col = -1
    for i in range(len(rp)-1):
        if checker.check_segment_collision(rp[i], rp[i+1], 0.03, period=period):
            n_col += 1
            if first_col < 0:
                first_col = i
    print(f"Geodesic collisions: {n_col}/{len(rp)-1} (first@{first_col})")
    
    if first_col >= 0:
        lo = max(0, first_col - 2)
        hi = min(len(rp), first_col + 5)
        for i in range(lo, hi):
            w = rp[i]
            c = checker.check_config_collision(w)
            print(f"  rp[{i}] = [{w[0]:+.6f}, {w[1]:+.6f}] in_col={c}")
        # Show segment detail
        q1, q2 = rp[first_col], rp[first_col+1]
        diff_geo = ((q2 - q1) + half) % period - half
        print(f"  first_col seg geodiff=[{diff_geo[0]:+.4f},{diff_geo[1]:+.4f}] "
              f"|max|={max(abs(diff_geo)):.4f}")
else:
    print("No path!")

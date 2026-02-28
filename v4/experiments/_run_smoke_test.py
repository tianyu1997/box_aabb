#!/usr/bin/env python3
"""Quick smoke test: all 3 scenes × SBF-GCS + PRM."""
import sys, time, logging, io
sys.path.insert(0, str(__import__('pathlib').Path(__file__).resolve().parent.parent / 'src'))
sys.path.insert(0, str(__import__('pathlib').Path(__file__).resolve().parent))
logging.disable(logging.CRITICAL)

import numpy as np
from runner import create_planner, load_scene_from_config
from scenes import load_marcucci_scene

_real = sys.stdout

def quiet():
    sys.stdout = io.StringIO()

def loud():
    sys.stdout = _real

results = []
for sname in ['shelves', 'bins', 'table']:
    scene_cfg = load_marcucci_scene(sname, robot='iiwa14')
    robot, scene, qps = load_scene_from_config(scene_cfg)
    q_s, q_g = qps[0]

    # SBF-GCS
    quiet()
    p = create_planner({'type': 'SBF', 'method': 'gcs', 'name': 'SBF-GCS'})
    p.setup(robot, scene, {'seed': 0})
    t0 = time.perf_counter()
    r = p.plan(q_s, q_g, timeout=30)
    t_sbf = time.perf_counter() - t0
    loud()

    # PRM
    quiet()
    p2 = create_planner({'type': 'PRM', 'n_samples': 500, 'k_neighbors': 10, 'name': 'PRM'})
    p2.setup(robot, scene, {'seed': 0})
    t0 = time.perf_counter()
    r2 = p2.plan(q_s, q_g, timeout=30)
    t_prm = time.perf_counter() - t0
    loud()

    # Amortized (5 queries reuse SBF forest)
    quiet()
    times_amor = []
    for i in range(5):
        qi = i % len(qps)
        qs2, qg2 = qps[qi]
        t0 = time.perf_counter()
        ra = p.plan(qs2, qg2, timeout=30)
        times_amor.append(time.perf_counter() - t0)
    loud()

    path_len_sbf = float(np.sum(np.linalg.norm(np.diff(np.array(r.path), axis=0), axis=1))) if r.path is not None and len(r.path) > 1 else float('inf')
    path_len_prm = float(np.sum(np.linalg.norm(np.diff(np.array(r2.path), axis=0), axis=1))) if r2.path is not None and len(r2.path) > 1 else float('inf')

    results.append({
        'scene': sname,
        'sbf_ok': r.success, 'sbf_t': t_sbf, 'sbf_path': path_len_sbf,
        'prm_ok': r2.success, 'prm_t': t_prm, 'prm_path': path_len_prm,
        'sbf_amor_t': np.mean(times_amor),
    })

print(f"{'Scene':10s} | {'SBF-GCS':25s} | {'PRM':25s} | {'SBF amort':10s}")
print('-' * 80)
for d in results:
    s1 = f"{'OK' if d['sbf_ok'] else 'FAIL'} {d['sbf_t']:.3f}s L={d['sbf_path']:.2f}"
    s2 = f"{'OK' if d['prm_ok'] else 'FAIL'} {d['prm_t']:.3f}s L={d['prm_path']:.2f}"
    print(f"{d['scene']:10s} | {s1:25s} | {s2:25s} | {d['sbf_amor_t']:.3f}s")

print(f"\nlink_radii = {robot.link_radii}")

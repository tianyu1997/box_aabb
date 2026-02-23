"""
experiments/exp9_ffb_min_edge_sweep.py — 实验 9: ffb_min_edge 最优值扫描

分别在 2-DOF 和 Panda 7-DOF 场景下, 扫描 ffb_min_edge 取值,
观察 forest 覆盖率、box 数量、grow 耗时、规划成功率和路径代价变化.

用法:
    cd v3
    python -m experiments.exp9_ffb_min_edge_sweep [--quick]
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from pathlib import Path
from typing import Dict, List

import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
_SRC = _ROOT / "src"
for p in (_ROOT, _SRC):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from experiments.runner import (
    ExperimentResults, SingleTrialResult, load_scene_from_config,
)
from experiments.scenes import load_scenes

from baselines.sbf_adapter import SBFAdapter

logger = logging.getLogger(__name__)

OUTPUT_DIR = _ROOT / "experiments" / "output" / "raw"

# ── 扫描参数 ──
FFB_VALUES_2DOF = [0.002, 0.005, 0.01, 0.02, 0.05, 0.1, 0.2]
FFB_VALUES_PANDA = [0.005, 0.01, 0.02, 0.05, 0.1, 0.2]


def run_single(scene_name: str, scene_cfg: dict, ffb_val: float,
               seed: int, timeout: float = 30.0) -> dict:
    """运行单次 (scene, ffb_min_edge, seed) 组合."""
    robot, scene, query_pairs = load_scene_from_config(scene_cfg)
    q_start, q_goal = query_pairs[0]
    ndim = robot.n_joints

    adapter = SBFAdapter(method="dijkstra")
    params = {
        "seed": seed,
        "ffb_min_edge": ffb_val,
        "max_boxes": 400 if ndim >= 7 else 200,
        "max_consecutive_miss": 20,
        "no_cache": True,
    }
    adapter.setup(robot, scene, params)

    t0 = time.perf_counter()
    result = adapter.plan(q_start, q_goal, timeout=timeout)
    wall = time.perf_counter() - t0

    # 额外统计
    prep = adapter._prep or {}
    n_boxes = len(prep.get("boxes", {}))
    grow_ms = prep.get("grow_ms", 0.0)

    return {
        **result.to_dict(),
        "ffb_min_edge": ffb_val,
        "n_boxes": n_boxes,
        "grow_ms": grow_ms,
        "wall_s": wall,
    }


def run(quick: bool = False):
    # 加载场景
    all_scenes = {
        "2dof_with_obstacle": {
            "name": "2dof_with_obstacle",
            "robot": "2dof_planar",
            "obstacles": [
                {"min": [0.6, -0.15, -0.5], "max": [1.0, 0.15, 0.5],
                 "name": "wall_x08"},
            ],
            "query_pairs": [
                {"start": [-1.0, 0.5], "goal": [1.0, -0.5]},
            ],
        },
        "panda_8obs_open": {
            "name": "panda_8obs_open",
            "robot": "panda",
            "random_obstacles": {
                "count": 8, "seed": 100,
                "x_range": [-0.8, 0.8], "y_range": [-0.8, 0.8],
                "z_range": [0.1, 0.9], "half_size_range": [0.04, 0.12],
            },
            "query_pairs": [
                {"start": [0.5, -1.2, 0.5, -2.5, 0.5, 0.8, 1.5],
                 "goal": [-2.0, 1.2, -1.8, -0.5, -2.0, 3.0, -1.8]},
            ],
        },
    }

    n_seeds = 3 if quick else 10
    timeout = 15.0 if quick else 30.0

    print("=" * 60)
    print("  Experiment 9: ffb_min_edge Sweep")
    print("=" * 60)
    print(f"  Seeds per config   : {n_seeds}")
    print(f"  Timeout            : {timeout:.0f} s")
    print()

    results = ExperimentResults(experiment_name="exp9_ffb_min_edge_sweep")

    # ── 2-DOF ──
    scene_name = "2dof_with_obstacle"
    scene_cfg = all_scenes[scene_name]
    ffb_values = FFB_VALUES_2DOF[:4] if quick else FFB_VALUES_2DOF
    print(f"[{scene_name}]  ffb_min_edge = {ffb_values}")

    for ffb_val in ffb_values:
        successes = 0
        costs = []
        grow_times = []
        box_counts = []
        for seed in range(n_seeds):
            try:
                r = run_single(scene_name, scene_cfg, ffb_val, seed, timeout)
                results.add(SingleTrialResult(
                    scene_name=scene_name,
                    planner_name=f"SBF-ffb={ffb_val}",
                    seed=seed, trial=0,
                    result=r, wall_clock=r["wall_s"],
                ))
                if r.get("success"):
                    successes += 1
                    costs.append(r.get("cost", float("inf")))
                grow_times.append(r.get("grow_ms", 0))
                box_counts.append(r.get("n_boxes", 0))
            except Exception as e:
                print(f"    ERROR seed={seed}: {e}")

        avg_cost = np.mean(costs) if costs else float("nan")
        avg_grow = np.mean(grow_times) if grow_times else float("nan")
        avg_boxes = np.mean(box_counts) if box_counts else 0
        print(f"  ffb={ffb_val:<6.3f}  "
              f"ok={successes}/{n_seeds}  "
              f"cost={avg_cost:7.3f}  "
              f"grow={avg_grow:6.0f}ms  "
              f"boxes={avg_boxes:.0f}")
    print()

    # ── Panda 7-DOF ──
    scene_name = "panda_8obs_open"
    scene_cfg = all_scenes[scene_name]
    ffb_values = FFB_VALUES_PANDA[:3] if quick else FFB_VALUES_PANDA
    print(f"[{scene_name}]  ffb_min_edge = {ffb_values}")

    for ffb_val in ffb_values:
        successes = 0
        costs = []
        grow_times = []
        box_counts = []
        for seed in range(n_seeds):
            try:
                r = run_single(scene_name, scene_cfg, ffb_val, seed, timeout)
                results.add(SingleTrialResult(
                    scene_name=scene_name,
                    planner_name=f"SBF-ffb={ffb_val}",
                    seed=seed, trial=0,
                    result=r, wall_clock=r["wall_s"],
                ))
                if r.get("success"):
                    successes += 1
                    costs.append(r.get("cost", float("inf")))
                grow_times.append(r.get("grow_ms", 0))
                box_counts.append(r.get("n_boxes", 0))
            except Exception as e:
                print(f"    ERROR seed={seed}: {e}")

        avg_cost = np.mean(costs) if costs else float("nan")
        avg_grow = np.mean(grow_times) if grow_times else float("nan")
        avg_boxes = np.mean(box_counts) if box_counts else 0
        print(f"  ffb={ffb_val:<6.3f}  "
              f"ok={successes}/{n_seeds}  "
              f"cost={avg_cost:7.3f}  "
              f"grow={avg_grow:6.0f}ms  "
              f"boxes={avg_boxes:.0f}")
    print()

    # ── 保存 ──
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    out_path = OUTPUT_DIR / "exp9_ffb_min_edge_sweep.json"
    results.metadata = {
        "ffb_values_2dof": FFB_VALUES_2DOF,
        "ffb_values_panda": FFB_VALUES_PANDA,
        "n_seeds": n_seeds,
        "timeout": timeout,
    }
    results.save(out_path)
    print(f"Results saved to {out_path}")
    return out_path


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")
    parser = argparse.ArgumentParser()
    parser.add_argument("--quick", action="store_true",
                        help="Reduced sweep (fewer seeds, fewer values)")
    args = parser.parse_args()
    run(quick=args.quick)

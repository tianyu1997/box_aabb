"""
experiments/exp10_sampling_ratio_sweep.py — 实验 10: guided_sample_ratio 最优值扫描

采样策略:
  - 以概率 guided_sample_ratio 使用 KD 树引导采样 (sample_unoccupied_seed)
  - 以概率 (1 - guided_sample_ratio) 使用均匀随机采样

分别在 2-DOF 和 Panda 7-DOF 场景下扫描 guided_sample_ratio,
观察 forest 覆盖率、box 数量、grow 耗时、规划成功率和路径代价.

用法:
    cd v3
    python -m experiments.exp10_sampling_ratio_sweep [--quick]
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

from baselines.sbf_adapter import SBFAdapter

logger = logging.getLogger(__name__)

OUTPUT_DIR = _ROOT / "experiments" / "output" / "raw"

# ── 扫描参数 ──
RATIO_VALUES = [0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.00]


def run_single(scene_name: str, scene_cfg: dict, ratio: float,
               seed: int, ndim: int, timeout: float = 30.0) -> dict:
    """运行单次 (scene, guided_sample_ratio, seed) 组合."""
    robot, scene, query_pairs = load_scene_from_config(scene_cfg)
    q_start, q_goal = query_pairs[0]

    adapter = SBFAdapter(method="dijkstra")
    params = {
        "seed": seed,
        "guided_sample_ratio": ratio,
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
        "guided_sample_ratio": ratio,
        "n_boxes": n_boxes,
        "grow_ms": grow_ms,
        "wall_s": wall,
    }


def run(quick: bool = False):
    all_scenes = {
        "2dof_narrow_3walls": {
            "name": "2dof_narrow_3walls",
            "robot": "2dof_planar",
            "obstacles": [
                {"min": [0.3, -0.15, -0.5], "max": [0.7, 0.15, 0.5],
                 "name": "wall_mid"},
                {"min": [-0.5, 0.4, -0.5], "max": [0.1, 0.7, 0.5],
                 "name": "wall_upper"},
                {"min": [-0.5, -0.7, -0.5], "max": [0.1, -0.4, 0.5],
                 "name": "wall_lower"},
            ],
            "query_pairs": [
                {"start": [-1.0, 0.5], "goal": [1.0, -0.5]},
            ],
        },
        "panda_15obs_dense": {
            "name": "panda_15obs_dense",
            "robot": "panda",
            "random_obstacles": {
                "count": 15, "seed": 42,
                "x_range": [-0.7, 0.7], "y_range": [-0.7, 0.7],
                "z_range": [0.05, 0.85], "half_size_range": [0.04, 0.10],
            },
            "query_pairs": [
                {"start": [0.5, -1.2, 0.5, -2.5, 0.5, 0.8, 1.5],
                 "goal": [-2.0, 1.2, -1.8, -0.5, -2.0, 3.0, -1.8]},
            ],
        },
    }

    n_seeds = 3 if quick else 10
    timeout = 15.0 if quick else 30.0
    ratios = RATIO_VALUES[:4] if quick else RATIO_VALUES

    print("=" * 64)
    print("  Experiment 10: guided_sample_ratio Sweep")
    print("=" * 64)
    print(f"  Ratios             : {ratios}")
    print(f"  Seeds per config   : {n_seeds}")
    print(f"  Timeout            : {timeout:.0f} s")
    print()

    results = ExperimentResults(
        experiment_name="exp10_sampling_ratio_sweep")

    # ── 2-DOF (narrow 3-wall) ──
    scene_name = "2dof_narrow_3walls"
    scene_cfg = all_scenes[scene_name]
    ndim = 2
    print(f"[{scene_name}]")

    for ratio in ratios:
        successes = 0
        costs: List[float] = []
        grow_times: List[float] = []
        box_counts: List[int] = []
        for seed in range(n_seeds):
            try:
                r = run_single(scene_name, scene_cfg, ratio, seed,
                               ndim, timeout)
                results.add(SingleTrialResult(
                    scene_name=scene_name,
                    planner_name=f"SBF-guided={ratio:.2f}",
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
        print(f"  guided={ratio:.2f}  "
              f"ok={successes}/{n_seeds}  "
              f"cost={avg_cost:7.3f}  "
              f"grow={avg_grow:6.0f}ms  "
              f"boxes={avg_boxes:.0f}")
    print()

    # ── Panda 7-DOF (15 dense obstacles) ──
    scene_name = "panda_15obs_dense"
    scene_cfg = all_scenes[scene_name]
    ndim = 7
    print(f"[{scene_name}]")

    for ratio in ratios:
        successes = 0
        costs = []
        grow_times = []
        box_counts = []
        for seed in range(n_seeds):
            try:
                r = run_single(scene_name, scene_cfg, ratio, seed,
                               ndim, timeout)
                results.add(SingleTrialResult(
                    scene_name=scene_name,
                    planner_name=f"SBF-guided={ratio:.2f}",
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
        print(f"  guided={ratio:.2f}  "
              f"ok={successes}/{n_seeds}  "
              f"cost={avg_cost:7.3f}  "
              f"grow={avg_grow:6.0f}ms  "
              f"boxes={avg_boxes:.0f}")
    print()

    # ── 保存 ──
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    out_path = OUTPUT_DIR / "exp10_sampling_ratio_sweep.json"
    results.metadata = {
        "ratio_values": RATIO_VALUES,
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

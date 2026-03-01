#!/usr/bin/env python
"""
prepare_scenes.py — 预生成实验场景并保存到 JSON

生成所有 Marcucci 基准场景 (shelves/bins/table) 的:
  - 障碍物配置
  - query pairs (IK milestone 对)
  - Exp2 增量扰动 (per seed)

保存到 output/scenes/ 下, 后续实验脚本直接加载, 确保:
  1. baseline 和 SBF 使用完全相同的场景
  2. 场景数据可复现、可审查

用法:
  python -m experiments.prepare_scenes [--seeds 50] [--output output/scenes]
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

# ── project path ──
_V4_ROOT = Path(__file__).resolve().parent.parent
_ROOT = _V4_ROOT / "src"
_EXP = Path(__file__).resolve().parent
for _p in (_V4_ROOT, _ROOT, _EXP):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from scenes import load_marcucci_scene, load_all_marcucci_scenes


def _to_serializable(obj):
    """递归转换 numpy 类型为 Python 原生类型."""
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, (np.float32, np.float64)):
        return float(obj)
    if isinstance(obj, (np.int32, np.int64)):
        return int(obj)
    if isinstance(obj, dict):
        return {k: _to_serializable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_to_serializable(v) for v in obj]
    return obj


def generate_exp2_perturbations(base_obstacles: list,
                                 n_seeds: int,
                                 incremental_configs: list,
                                 bounds=((-0.6, 0.6), (-0.6, 0.6), (0.0, 0.8)),
                                 ) -> dict:
    """为 Exp2 预生成每个 seed 的增量扰动.

    Returns:
        {condition_str: {seed: {"new_obstacles": [...], "move_indices": [...], "move_deltas": [...]}}}
    """
    perturbations = {}
    for incr_cfg in incremental_configs:
        n_add = incr_cfg["n_add"]
        n_move = incr_cfg["n_move"]
        condition = f"+{n_add}add_{n_move}move"
        perturbations[condition] = {}

        for seed in range(n_seeds):
            rng = np.random.default_rng(seed)

            # 新增障碍物
            new_obstacles = []
            for _ in range(n_add):
                cx = float(rng.uniform(*bounds[0]))
                cy = float(rng.uniform(*bounds[1]))
                cz = float(rng.uniform(*bounds[2]))
                h = float(rng.uniform(0.03, 0.08))
                new_obstacles.append({
                    "min": [cx - h, cy - h, cz - h],
                    "max": [cx + h, cy + h, cz + h],
                    "name": f"pert_{seed}_{len(new_obstacles)}",
                })

            # 移动障碍物
            move_indices = []
            move_deltas = []
            if n_move > 0 and len(base_obstacles) > 0:
                indices = rng.choice(len(base_obstacles),
                                     size=min(n_move, len(base_obstacles)),
                                     replace=False).tolist()
                for idx in indices:
                    delta = rng.uniform(-0.1, 0.1, size=3).tolist()
                    move_indices.append(int(idx))
                    move_deltas.append(delta)

            perturbations[condition][str(seed)] = {
                "new_obstacles": new_obstacles,
                "move_indices": move_indices,
                "move_deltas": move_deltas,
            }

    return perturbations


def main():
    parser = argparse.ArgumentParser(description="预生成实验场景")
    parser.add_argument("--seeds", type=int, default=50,
                        help="每个场景的 seed 数")
    parser.add_argument("--output", type=str, default="output/scenes",
                        help="输出目录")
    args = parser.parse_args()

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"{'=' * 60}")
    print(f"  场景预生成  seeds={args.seeds}")
    print(f"  输出目录: {output_dir}")
    print(f"{'=' * 60}")

    # ── 1. 加载全部 Marcucci 场景 ──
    scene_cfgs = load_all_marcucci_scenes(robot="iiwa14")
    scenes_data = {}
    for cfg in scene_cfgs:
        name = cfg["name"]
        scenes_data[name] = _to_serializable(cfg)
        n_obs = len(cfg.get("obstacles", []))
        n_qp = len(cfg.get("query_pairs", []))
        print(f"  场景 {name}: {n_obs} obstacles, {n_qp} query pairs")

    # ── 2. Exp2 增量扰动 ──
    incremental_configs = [
        {"n_add": 1, "n_move": 0},
        {"n_add": 3, "n_move": 0},
        {"n_add": 5, "n_move": 0},
        {"n_add": 0, "n_move": 3},
    ]
    # 使用 shelves 场景的障碍物作为基础
    shelves_cfg = next(c for c in scene_cfgs if "shelves" in c["name"])
    exp2_perturbations = generate_exp2_perturbations(
        base_obstacles=shelves_cfg["obstacles"],
        n_seeds=args.seeds,
        incremental_configs=incremental_configs,
    )
    print(f"  Exp2 扰动: {len(incremental_configs)} conditions × "
          f"{args.seeds} seeds")

    # ── 3. 保存 ──
    all_data = {
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "n_seeds": args.seeds,
        "scenes": scenes_data,
        "exp2_incremental_configs": incremental_configs,
        "exp2_perturbations": exp2_perturbations,
    }

    out_path = output_dir / "experiment_scenes.json"
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(all_data, f, indent=2, ensure_ascii=False)
    print(f"\n  ✓ 已保存 → {out_path}")
    print(f"    文件大小: {out_path.stat().st_size / 1024:.1f} KB")

    # ── 4. 每个场景独立保存 (方便单独加载) ──
    for name, cfg in scenes_data.items():
        p = output_dir / f"{name}.json"
        with open(p, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2, ensure_ascii=False)
        print(f"    {p.name}")

    print(f"\n{'=' * 60}")
    print(f"  场景预生成完成")
    print(f"{'=' * 60}")


if __name__ == "__main__":
    main()

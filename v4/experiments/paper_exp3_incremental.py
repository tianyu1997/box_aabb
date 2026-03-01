"""
paper_exp3_incremental.py — 实验 3: 增量障碍物更新

设计 (使用 C++ pysbf):
  - 使用 pysbf 构建初始 forest (build_multi) — 合并场景
  - 仅对 bins (binR + binL) 和 shelves 施加 ±δ 平移扰动
    (table 保持不动, 因其在现实中通常固定)
  - 扰动幅度 δ=2cm, 每 seed 连续 10 次 trial
  - 仅测量增量 rebuild 时间 (remove_obstacle → add_obstacle);
    不再与从头 build_multi 对比 (Exp 1 已有 cold-start 数据)
  - 指标: incremental_time, n_boxes_after

用法:
  python -m experiments.paper_exp3_incremental              # 全量运行
  python -m experiments.paper_exp3_incremental --quick      # 快速模式
  python -m experiments.paper_exp3_incremental --delta 0.03 # 自定义扰动幅度
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from pathlib import Path

import numpy as np

# ── project path ──
_PROJ_ROOT = Path(__file__).resolve().parent.parent.parent
_SRC_ROOT = Path(__file__).resolve().parent.parent / "src"
_EXP_DIR = Path(__file__).resolve().parent

if str(_SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(_SRC_ROOT))
if str(_EXP_DIR) not in sys.path:
    sys.path.insert(0, str(_EXP_DIR))

logger = logging.getLogger(__name__)

# ═══════════════════════════════════════════════════════════════════════════
# Configuration
# ═══════════════════════════════════════════════════════════════════════════

N_SEEDS = 10
N_TRIALS = 10            # 每 seed 扰动次数
DELTA = 0.02             # ±2cm 扰动幅度
SBF_MAX_BOXES = 15000
SBF_N_RANDOM = 5000
ROBOT_JSON = str(_PROJ_ROOT / "cpp" / "configs" / "iiwa14.json")


def _fmt(seconds: float) -> str:
    if seconds < 1:
        return f"{seconds*1000:.1f}ms"
    if seconds < 60:
        return f"{seconds:.2f}s"
    m, s = divmod(int(seconds), 60)
    return f"{m}m{s:02d}s"


# ═══════════════════════════════════════════════════════════════════════════
# Helpers
# ═══════════════════════════════════════════════════════════════════════════

def make_pysbf_obstacles(obs_dicts):
    """将 marcucci_scenes 障碍物 dict → pysbf.Obstacle 列表."""
    import pysbf
    obstacles = []
    for o in obs_dicts:
        obs = pysbf.Obstacle()
        obs.name = o["name"]
        lo = np.array(o["min"], dtype=np.float64)
        hi = np.array(o["max"], dtype=np.float64)
        obs.center = 0.5 * (lo + hi)
        obs.half_sizes = 0.5 * (hi - lo)
        obstacles.append(obs)
    return obstacles


# 需要扰动的障碍物名称前缀 (bins + shelves, 不含 table)
_PERTURB_PREFIXES = ("shelf_", "binR_", "binL_")


def _is_perturbed(name: str) -> bool:
    return any(name.startswith(p) for p in _PERTURB_PREFIXES)


def perturb_obstacles(obstacles, delta: float, rng):
    """对 bins + shelves 障碍物施加均匀随机 ±δ 扰动 (仅平移 center).

    Table 等非扰动障碍物保持不变.

    Returns:
        old_obstacles: 被扰动的原始障碍物列表 (用于 remove_obstacle)
        new_obstacles: 扰动后的障碍物列表 (用于 add_obstacle)
    """
    import pysbf
    old_list = []
    new_list = []
    for obs in obstacles:
        if not _is_perturbed(obs.name):
            continue
        # 保存原始副本
        old = pysbf.Obstacle()
        old.name = obs.name
        old.center = obs.center.copy()
        old.half_sizes = obs.half_sizes.copy()
        old_list.append(old)

        # 扰动副本
        new = pysbf.Obstacle()
        new.name = obs.name
        shift = rng.uniform(-delta, delta, size=3)
        new.center = obs.center + shift
        new.half_sizes = obs.half_sizes.copy()
        new_list.append(new)

    return old_list, new_list


# ═══════════════════════════════════════════════════════════════════════════
# 实验主体
# ═══════════════════════════════════════════════════════════════════════════

def run_incremental(n_seeds: int, n_trials: int, delta: float,
                    quick: bool = False):
    """运行增量重建实验.

    每 seed:
      1. build_multi → 初始 forest
      2. 每 trial: perturb bins+shelves → incremental (remove + add) → 计时
    """
    import pysbf
    from marcucci_scenes import build_combined_obstacles, get_query_pairs

    max_boxes = 5000 if quick else SBF_MAX_BOXES
    n_random = 1000 if quick else SBF_N_RANDOM
    n_seeds = min(n_seeds, 2) if quick else n_seeds
    n_trials = min(n_trials, 2) if quick else n_trials

    robot = pysbf.Robot.from_json(ROBOT_JSON)
    obs_dicts = build_combined_obstacles()
    obstacles_orig = make_pysbf_obstacles(obs_dicts)
    pairs = [(s.copy(), g.copy()) for s, g in get_query_pairs("combined")]

    print(f"\n{'='*60}")
    print(f"  Incremental Rebuild: {len(obstacles_orig)} obstacles")
    print(f"  delta=±{delta*100:.1f}cm  {n_seeds} seeds × {n_trials} trials")
    print(f"  max_boxes={max_boxes}, n_random={n_random}")
    print(f"{'='*60}")

    all_results = []

    for seed in range(n_seeds):
        cfg = pysbf.SBFConfig()
        cfg.max_boxes = max_boxes
        cfg.max_consecutive_miss = 500
        cfg.seed = seed

        # 初始构建
        obstacles = [pysbf.Obstacle() for _ in obstacles_orig]
        for i, orig in enumerate(obstacles_orig):
            obstacles[i].name = orig.name
            obstacles[i].center = orig.center.copy()
            obstacles[i].half_sizes = orig.half_sizes.copy()

        t0 = time.perf_counter()
        planner = pysbf.SBFPlanner(robot, obstacles, cfg)
        planner.build_multi(pairs, n_random, 300.0)
        initial_build_time = time.perf_counter() - t0
        initial_boxes = planner.forest().n_boxes()

        print(f"\n  seed={seed}  initial: boxes={initial_boxes}  "
              f"build={_fmt(initial_build_time)}")

        rng = np.random.default_rng(seed * 1000 + 42)

        for trial in range(n_trials):
            # 扰动 bins + shelves 障碍物 (table 不动)
            old_obs, new_obs = perturb_obstacles(obstacles, delta, rng)

            # ── 增量更新: remove old → add new ──
            t_inc_start = time.perf_counter()
            for old in old_obs:
                planner.remove_obstacle(old.name)
            for new in new_obs:
                planner.add_obstacle(new)
            t_inc = time.perf_counter() - t_inc_start
            inc_boxes = planner.forest().n_boxes()

            print(f"    trial={trial}  "
                  f"inc={_fmt(t_inc)} ({inc_boxes} boxes)")

            result = {
                "seed": seed,
                "trial": trial,
                "incremental_time": t_inc,
                "incremental_boxes": inc_boxes,
                "initial_build_time": initial_build_time,
                "n_perturbed": len(old_obs),
            }
            all_results.append(result)

            # 更新 obstacles 数组: 用 new_obs 替换对应条目
            by_name = {o.name: o for o in new_obs}
            for i, obs in enumerate(obstacles):
                if obs.name in by_name:
                    obstacles[i] = by_name[obs.name]

    # Summary
    if all_results:
        inc_times = [r["incremental_time"] for r in all_results]

        print(f"\n  Summary ({len(all_results)} trials):")
        print(f"    Incremental: median={np.median(inc_times)*1000:.1f}ms  "
              f"mean={np.mean(inc_times)*1000:.1f}ms  "
              f"max={np.max(inc_times)*1000:.1f}ms")
        print(f"    Perturbed obstacles per trial: "
              f"{all_results[0]['n_perturbed']}")

    return all_results


# ═══════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="Paper Exp 3: Incremental Obstacle Update (C++ pysbf)")
    parser.add_argument("--seeds", type=int, default=N_SEEDS)
    parser.add_argument("--trials", type=int, default=N_TRIALS)
    parser.add_argument("--delta", type=float, default=DELTA,
                        help="Perturbation amplitude in meters (default: 0.02)")
    parser.add_argument("--quick", action="store_true",
                        help="Quick mode: 2 seeds × 2 trials")
    parser.add_argument("--output", type=str, default="output/raw")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")

    print("=" * 72)
    print("  Exp 3: Incremental Obstacle Update (C++ pysbf)")
    print("=" * 72)

    results = run_incremental(
        args.seeds, args.trials, args.delta, args.quick)

    # Save
    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / "paper_exp3_incremental.json"
    with open(out_path, "w") as f:
        json.dump(results, f, indent=2, default=str)
    print(f"\n  Results saved to {out_path}")
    print("\n  Exp 3 complete.")


if __name__ == "__main__":
    main()

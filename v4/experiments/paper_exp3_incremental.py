"""
paper_exp3_incremental.py — 实验 3: 增量障碍物更新

设计 (使用 C++ pysbf):
  - 使用 pysbf 构建初始 forest (build_multi) — 合并场景
  - 每 trial 随机选择 1 个 bin 或 shelf 障碍物,
    朝随机方向移动 5cm (table 保持不动)
  - 增量更新分两阶段:
      Phase 1 (Invalidation): remove_obstacle + add_obstacle → 毫秒级
      Phase 2 (Regrowth):     build_multi 恢复覆盖率 (仅填补空洞)
  - 对比:
      1) Warm rebuild: 同一 planner, clear_forest + build_multi (利用 FK cache)
      2) Cold rebuild: 全新 planner + build_multi (baseline)
  - 指标: t_inv, t_regrow, t_inc_total, t_warm, t_cold, speedup, vol_ratio

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
DELTA = 0.05             # 5cm 扰动幅度
# ---- 与 exp1 一致的 SBF 配置 ----
SBF_MAX_BOXES = 1000
SBF_N_RANDOM = 200
SBF_MAX_CONSECUTIVE_MISS = 200
SBF_FFB_MIN_EDGE = 0.02
SBF_FFB_MIN_EDGE_RELAXED = 0.06
SBF_PHASE_K = [5.0, 3.0, 2.0, 1.0, 0.5, 0.1]
SBF_PHASE_BUDGET = [500, 800, 1000, 1000, 1000, 1000]
SBF_MAX_BOXES_PER_PAIR = 200
SBF_COARSEN_TARGET = 200
SBF_COARSEN_GRID_CHECK = True
SBF_COARSEN_SPLIT_DEPTH = 1
SBF_COARSEN_MAX_TREE_FK = 2000
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


def perturb_single_obstacle(obstacles, delta: float, rng):
    """随机选择 1 个 bin/shelf 障碍物, 朝随机方向移动 delta 距离.

    Args:
        obstacles: 当前全部障碍物列表
        delta: 移动距离 (米)
        rng: numpy RandomGenerator

    Returns:
        (old_obs, new_obs, obs_index): 原始障碍物、扰动后障碍物、在列表中的索引
    """
    import pysbf

    # 收集所有可扰动障碍物的索引
    perturb_indices = [i for i, o in enumerate(obstacles)
                       if _is_perturbed(o.name)]
    assert perturb_indices, "No perturbable obstacles found"

    # 随机选 1 个
    idx = rng.choice(perturb_indices)
    src = obstacles[idx]

    # 保存原始副本
    old = pysbf.Obstacle()
    old.name = src.name
    old.center = src.center.copy()
    old.half_sizes = src.half_sizes.copy()

    # 生成随机方向, 固定距离
    direction = rng.standard_normal(3)
    direction /= np.linalg.norm(direction)
    shift = direction * delta

    # 扰动副本
    new = pysbf.Obstacle()
    new.name = src.name
    new.center = src.center + shift
    new.half_sizes = src.half_sizes.copy()

    return old, new, idx


# ═══════════════════════════════════════════════════════════════════════════
# 实验主体
# ═══════════════════════════════════════════════════════════════════════════

def run_incremental(n_seeds: int, n_trials: int, delta: float,
                    quick: bool = False):
    """运行增量重建实验.

    核心对比: incremental (invalidation + regrowth) vs warm rebuild vs cold rebuild.

    每 seed:
      1. build_multi → 初始 forest
      2. 每 trial:
         a) 随机选 1 个 bin/shelf, 朝随机方向移动 delta
         b) incremental: remove+add (invalidation) → build_multi (regrowth)
         c) warm rebuild: clear_forest + build_multi (同一 planner)
         d) cold rebuild: 新 planner + build_multi (baseline)
    """
    import pysbf
    from marcucci_scenes import build_combined_obstacles, get_query_pairs

    max_boxes = 500 if quick else SBF_MAX_BOXES
    n_random = 50 if quick else SBF_N_RANDOM
    n_seeds = min(n_seeds, 2) if quick else n_seeds
    n_trials = min(n_trials, 3) if quick else n_trials

    robot = pysbf.Robot.from_json(ROBOT_JSON)
    obs_dicts = build_combined_obstacles()
    obstacles_orig = make_pysbf_obstacles(obs_dicts)
    pairs = [(s.copy(), g.copy()) for s, g in get_query_pairs("combined")]

    def make_cfg(seed_val):
        cfg = pysbf.SBFConfig()
        cfg.max_boxes = max_boxes
        cfg.max_consecutive_miss = SBF_MAX_CONSECUTIVE_MISS
        cfg.ffb_min_edge = SBF_FFB_MIN_EDGE
        cfg.ffb_min_edge_relaxed = SBF_FFB_MIN_EDGE_RELAXED
        cfg.bfs_phase_k = SBF_PHASE_K
        cfg.bfs_phase_budget = SBF_PHASE_BUDGET
        cfg.max_boxes_per_pair = SBF_MAX_BOXES_PER_PAIR
        cfg.coarsen_target_boxes = SBF_COARSEN_TARGET
        cfg.coarsen_grid_check = SBF_COARSEN_GRID_CHECK
        cfg.coarsen_split_depth = SBF_COARSEN_SPLIT_DEPTH
        cfg.coarsen_max_tree_fk = SBF_COARSEN_MAX_TREE_FK
        cfg.seed = seed_val
        return cfg

    def copy_obstacles(src):
        """深拷贝障碍物列表."""
        dst = []
        for o in src:
            ob = pysbf.Obstacle()
            ob.name = o.name
            ob.center = o.center.copy()
            ob.half_sizes = o.half_sizes.copy()
            dst.append(ob)
        return dst

    print(f"\n{'='*60}")
    print(f"  Incremental Rebuild: {len(obstacles_orig)} obstacles")
    print(f"  delta={delta*100:.1f}cm (single obstacle)")
    print(f"  {n_seeds} seeds × {n_trials} trials")
    print(f"  max_boxes={max_boxes}, n_random={n_random}")
    print(f"{'='*60}")

    all_results = []

    for seed in range(n_seeds):
        cfg = make_cfg(seed)

        # 初始构建
        obstacles = copy_obstacles(obstacles_orig)

        t0 = time.perf_counter()
        planner = pysbf.SBFPlanner(robot, obstacles, cfg)
        planner.build_multi(pairs, n_random, 300.0)
        initial_build_time = time.perf_counter() - t0
        initial_boxes = planner.forest().n_boxes()
        initial_vol = planner.forest().total_volume()

        print(f"\n  seed={seed}  initial: boxes={initial_boxes}  "
              f"vol={initial_vol:.4e}  build={_fmt(initial_build_time)}")

        rng = np.random.default_rng(seed * 1000 + 42)

        for trial in range(n_trials):
            # 随机选 1 个障碍物, 朝随机方向移动 delta
            old_obs, new_obs, obs_idx = perturb_single_obstacle(
                obstacles, delta, rng)

            boxes_before = planner.forest().n_boxes()
            vol_before = planner.forest().total_volume()

            # ═══ Incremental: invalidation + targeted regrowth ═══
            # Phase 1: Invalidation (remove old + add new)
            t_inv_start = time.perf_counter()
            planner.remove_obstacle(old_obs.name)
            planner.add_obstacle(new_obs)
            t_inv = time.perf_counter() - t_inv_start
            boxes_after_inv = planner.forest().n_boxes()
            vol_after_inv = planner.forest().total_volume()
            n_invalidated = boxes_before - boxes_after_inv

            # Phase 2: Targeted regrowth (fill depleted regions only)
            # Grow 2× invalidated boxes to compensate, using tree-guided
            # sampling + incremental adjacency (no full rebuild_adjacency).
            regrow_target = max(n_invalidated * 2, 50)
            t_regrow_start = time.perf_counter()
            n_regrown = planner.regrow(regrow_target, 60.0)
            t_regrow = time.perf_counter() - t_regrow_start
            inc_boxes = planner.forest().n_boxes()
            inc_vol = planner.forest().total_volume()
            t_inc_total = t_inv + t_regrow

            # 更新 obstacles 数组 (与 planner 内部一致)
            obstacles[obs_idx] = new_obs

            # ═══ Warm rebuild: clear_forest + build_multi (same planner) ═══
            planner.clear_forest()
            t_warm_start = time.perf_counter()
            planner.build_multi(pairs, n_random, 300.0)
            t_warm = time.perf_counter() - t_warm_start
            warm_boxes = planner.forest().n_boxes()
            warm_vol = planner.forest().total_volume()

            # ═══ Cold rebuild: 新 planner (baseline) ═══
            cfg_cold = make_cfg(seed + 10000 + trial)
            obstacles_cold = copy_obstacles(obstacles)
            t_cold_start = time.perf_counter()
            planner_cold = pysbf.SBFPlanner(robot, obstacles_cold, cfg_cold)
            planner_cold.build_multi(pairs, n_random, 300.0)
            t_cold = time.perf_counter() - t_cold_start
            cold_boxes = planner_cold.forest().n_boxes()
            cold_vol = planner_cold.forest().total_volume()

            speedup_inc_warm = t_warm / t_inc_total if t_inc_total > 0 else float('inf')
            speedup_inc_cold = t_cold / t_inc_total if t_inc_total > 0 else float('inf')
            speedup_warm_cold = t_cold / t_warm if t_warm > 0 else float('inf')
            vol_ratio_inc = inc_vol / cold_vol if cold_vol > 0 else float('inf')
            vol_ratio_warm = warm_vol / cold_vol if cold_vol > 0 else float('inf')

            print(f"    trial={trial}  obs={old_obs.name:<12s}  "
                  f"inv={_fmt(t_inv)}(-{n_invalidated}b) | "
                  f"regrow={_fmt(t_regrow)}(+{n_regrown})->{inc_boxes}b | "
                  f"warm={_fmt(t_warm)}->{warm_boxes}b | "
                  f"cold={_fmt(t_cold)}->{cold_boxes}b | "
                  f"inc/cold={speedup_inc_cold:.2f}x "
                  f"vol={vol_ratio_inc:.2f}")

            result = {
                "seed": seed,
                "trial": trial,
                "perturbed_obs": old_obs.name,
                "n_invalidated": n_invalidated,
                "n_regrown": n_regrown,
                "boxes_before": boxes_before,
                "t_inv": t_inv,
                "boxes_after_inv": boxes_after_inv,
                "vol_after_inv": vol_after_inv,
                "t_regrow": t_regrow,
                "inc_boxes": inc_boxes,
                "inc_vol": inc_vol,
                "t_inc_total": t_inc_total,
                "t_warm": t_warm,
                "warm_boxes": warm_boxes,
                "warm_vol": warm_vol,
                "t_cold": t_cold,
                "cold_boxes": cold_boxes,
                "cold_vol": cold_vol,
                "speedup_inc_vs_warm": speedup_inc_warm,
                "speedup_inc_vs_cold": speedup_inc_cold,
                "speedup_warm_vs_cold": speedup_warm_cold,
                "vol_ratio_inc_cold": vol_ratio_inc,
                "vol_ratio_warm_cold": vol_ratio_warm,
                "initial_build_time": initial_build_time,
                "initial_boxes": initial_boxes,
                "initial_vol": initial_vol,
                "delta": delta,
            }
            all_results.append(result)

    # Summary
    if all_results:
        print(f"\n{'='*60}")
        print(f"  Summary ({len(all_results)} trials)")
        print(f"{'='*60}")

        def _stats(key):
            vals = [r[key] for r in all_results]
            return np.median(vals), np.mean(vals), np.std(vals)

        med, mu, _ = _stats("n_invalidated")
        print(f"  Invalidated boxes:  median={med:.0f}  mean={mu:.1f}")
        med, mu, _ = _stats("n_regrown")
        print(f"  Regrown boxes:      median={med:.0f}  mean={mu:.1f}")
        med, mu, _ = _stats("t_inv")
        print(f"  Invalidation time:  median={med*1000:.1f}ms  mean={mu*1000:.1f}ms")
        med, mu, _ = _stats("t_regrow")
        print(f"  Regrowth time:      median={med:.3f}s  mean={mu:.3f}s")
        med, mu, _ = _stats("t_inc_total")
        print(f"  Incremental total:  median={med:.3f}s  mean={mu:.3f}s")
        med, mu, _ = _stats("t_warm")
        print(f"  Warm rebuild:       median={med:.3f}s  mean={mu:.3f}s")
        med, mu, _ = _stats("t_cold")
        print(f"  Cold rebuild:       median={med:.3f}s  mean={mu:.3f}s")
        print()
        med, mu, _ = _stats("speedup_inc_vs_warm")
        print(f"  Speedup inc/warm:   median={med:.2f}x  mean={mu:.2f}x")
        med, mu, _ = _stats("speedup_inc_vs_cold")
        print(f"  Speedup inc/cold:   median={med:.2f}x  mean={mu:.2f}x")
        med, mu, _ = _stats("speedup_warm_vs_cold")
        print(f"  Speedup warm/cold:  median={med:.2f}x  mean={mu:.2f}x")
        print()
        med, mu, _ = _stats("inc_boxes")
        print(f"  Boxes inc:    median={med:.0f}  mean={mu:.0f}")
        med, mu, _ = _stats("warm_boxes")
        print(f"  Boxes warm:   median={med:.0f}  mean={mu:.0f}")
        med, mu, _ = _stats("cold_boxes")
        print(f"  Boxes cold:   median={med:.0f}  mean={mu:.0f}")
        print()
        med, mu, _ = _stats("vol_ratio_inc_cold")
        print(f"  Vol ratio inc/cold:   median={med:.2f}  mean={mu:.2f}")
        med, mu, _ = _stats("vol_ratio_warm_cold")
        print(f"  Vol ratio warm/cold:  median={med:.2f}  mean={mu:.2f}")

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
                        help="Perturbation distance in meters (default: 0.05)")
    parser.add_argument("--quick", action="store_true",
                        help="Quick mode: 2 seeds x 3 trials")
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
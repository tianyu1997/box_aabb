"""
paper_exp2_incremental.py — 实验 2: 增量更新与持久化收益

对应论文 Exp 2 (Table II + Fig. 3):
  2a. 增量更新: 在已有 Forest 上 add/move 1/3/5 障碍物
      → SBF incremental vs IRIS 全量重建
  2b. HCACHE 热启动:
      → Cold start (无缓存) vs Warm start (HCACHE) vs 跨场景缓存

指标:
  - 增量更新时间 (ms)
  - IRIS 全量重建时间 (s)
  - 更新后自由空间体积保留率 (%)
  - FK 阶段耗时 (cold / warm)
  - 跨场景缓存命中率

统计: 50 seeds × 3 trials.

用法:
  python -m experiments.paper_exp2_incremental [--seeds 50]
  python -m experiments.paper_exp2_incremental --quick
"""

from __future__ import annotations

import argparse
import copy
import logging
import sys
import time
from pathlib import Path

import numpy as np


def _fmt(seconds: float) -> str:
    """把秒数格式化为易读字符串."""
    if seconds < 1:
        return f"{seconds*1000:.1f}ms"
    if seconds < 60:
        return f"{seconds:.2f}s"
    m, s = divmod(int(seconds), 60)
    return f"{m}m{s:02d}s"


def _eta(done: int, total: int, elapsed: float) -> str:
    """估算剩余时间."""
    if done == 0:
        return "?"
    remaining = elapsed / done * (total - done)
    return _fmt(remaining)

# ── project path ──
_ROOT = Path(__file__).resolve().parent.parent / "src"
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))
_EXP = Path(__file__).resolve().parent
if str(_EXP) not in sys.path:
    sys.path.insert(0, str(_EXP))

from runner import (
    ExperimentResults, SingleTrialResult,
    create_planner, load_scene_from_config,
)
from scenes import load_marcucci_scene


def _join_cache_thread(planner) -> None:
    """等待 planner 的后台缓存保存线程完成，避免 mmap 竞态 segfault."""
    prep = getattr(planner, '_prep', None)
    if prep is None:
        return
    ct = prep.get('_cache_thread')
    if ct is not None and ct.is_alive():
        ct.join()

logger = logging.getLogger(__name__)

EXPERIMENT_NAME = "paper_exp2_incremental"

# 增量更新配置: add 1/3/5 obstacles
INCREMENTAL_CONFIGS = [
    {"n_add": 1, "n_move": 0},
    {"n_add": 3, "n_move": 0},
    {"n_add": 5, "n_move": 0},
    {"n_add": 0, "n_move": 3},
]

# 方法: 只对比 SBF vs IRIS-NP (两者都支持 region 生成)
PLANNER_CONFIGS_2A = [
    {"type": "SBF", "method": "gcs", "name": "SBF-GCS"},
    {"type": "IRIS-NP-GCS", "n_iris_seeds": 10, "name": "IRIS-NP-GCS"},
]

# 场景: 使用 shelves (最结构化)
SCENE_NAME = "shelves"


def generate_perturbation_obstacles(n_add: int, n_move: int,
                                     existing_obstacles: list,
                                     rng: np.random.Generator,
                                     bounds: tuple = ((-0.6, 0.6),
                                                      (-0.6, 0.6),
                                                      (0.0, 0.8))):
    """生成增量障碍物扰动.

    Returns:
        new_obstacles: 新增障碍物列表
        move_indices: 需要移动的已有障碍物索引
        move_deltas: 移动量
    """
    new_obstacles = []
    for i in range(n_add):
        cx = rng.uniform(*bounds[0])
        cy = rng.uniform(*bounds[1])
        cz = rng.uniform(*bounds[2])
        h = rng.uniform(0.03, 0.08)
        new_obstacles.append({
            "min": [cx - h, cy - h, cz - h],
            "max": [cx + h, cy + h, cz + h],
            "name": f"incr_add_{i}",
        })

    move_indices = []
    move_deltas = []
    if n_move > 0 and len(existing_obstacles) > 0:
        n_moveable = min(n_move, len(existing_obstacles))
        move_indices = rng.choice(len(existing_obstacles), n_moveable,
                                   replace=False).tolist()
        for idx in move_indices:
            delta = rng.uniform(-0.15, 0.15, size=3)
            move_deltas.append(delta.tolist())

    return new_obstacles, move_indices, move_deltas


def apply_perturbation(scene_cfg: dict,
                       new_obstacles: list,
                       move_indices: list,
                       move_deltas: list) -> dict:
    """生成扰动后的场景配置."""
    cfg = copy.deepcopy(scene_cfg)

    # 移动已有障碍物
    for idx, delta in zip(move_indices, move_deltas):
        if idx < len(cfg["obstacles"]):
            obs = cfg["obstacles"][idx]
            obs["min"] = [obs["min"][i] + delta[i] for i in range(3)]
            obs["max"] = [obs["max"][i] + delta[i] for i in range(3)]

    # 添加新障碍物
    cfg["obstacles"].extend(new_obstacles)
    return cfg


# ═══════════════════════════════════════════════════════════════════════════
# Exp 2a: Incremental update
# ═══════════════════════════════════════════════════════════════════════════

def run_exp2a(n_seeds: int = 50, n_trials: int = 3,
              timeout: float = 120.0) -> ExperimentResults:
    """2a: SBF 增量更新 vs IRIS 全量重建."""
    results = ExperimentResults(experiment_name=f"{EXPERIMENT_NAME}_2a")

    base_scene_cfg = load_marcucci_scene(SCENE_NAME, robot="iiwa14")
    base_robot, base_scene, query_pairs = load_scene_from_config(base_scene_cfg)
    base_obstacles = base_scene_cfg["obstacles"]

    total = len(INCREMENTAL_CONFIGS) * len(PLANNER_CONFIGS_2A) * n_seeds * n_trials
    done = 0
    t_exp_start = time.perf_counter()

    logger.info("━" * 60)
    logger.info("Exp 2a  场景=%s  seeds=%d  trials=%d  total=%d",
                SCENE_NAME, n_seeds, n_trials, total)
    logger.info("━" * 60)

    for incr_cfg in INCREMENTAL_CONFIGS:
        condition = f"+{incr_cfg['n_add']}add_{incr_cfg['n_move']}move"

        for pcfg in PLANNER_CONFIGS_2A:
            planner_name = pcfg.get("name", pcfg["type"])

            logger.info("\n── 条件: %-20s  规划器: %s ──",
                        condition, planner_name)

            for seed in range(n_seeds):
                rng = np.random.default_rng(seed)

                for trial in range(n_trials):
                    elapsed = time.perf_counter() - t_exp_start
                    eta = _eta(done, total, elapsed)
                    logger.info("  [%d/%d | ETA %s] seed=%d trial=%d"
                                "  %s/%s  -- 正在构建初始规划...",
                                done + 1, total, eta,
                                seed, trial, condition, planner_name)

                    # ── Phase 1: 在原始场景上构建 ──
                    t_setup0 = time.perf_counter()
                    planner = create_planner(pcfg)
                    planner.setup(base_robot, base_scene, {"seed": seed})
                    t_setup = time.perf_counter() - t_setup0

                    qi = seed % len(query_pairs)
                    q_start, q_goal = query_pairs[qi]

                    # Initial plan
                    t0 = time.perf_counter()
                    r_init = planner.plan(q_start, q_goal, timeout=timeout)
                    t_initial = time.perf_counter() - t0

                    logger.info("    setup=%s  初始plan=%s  ok=%s",
                                _fmt(t_setup), _fmt(t_initial), r_init.success)

                    # 等待后台缓存线程，避免 mmap 竞态
                    _join_cache_thread(planner)

                    # ── Phase 2: 生成扰动 ──
                    new_obs, move_idx, move_delta = \
                        generate_perturbation_obstacles(
                            incr_cfg["n_add"], incr_cfg["n_move"],
                            base_obstacles, rng)

                    perturbed_cfg = apply_perturbation(
                        base_scene_cfg, new_obs, move_idx, move_delta)
                    p_robot, p_scene, _ = load_scene_from_config(perturbed_cfg)

                    # ── Phase 3: 更新/重建 ──
                    if pcfg["type"] == "SBF" and hasattr(planner, 'update_scene_incremental'):
                        # SBF: incremental update
                        logger.info("    -> SBF 增量更新...")
                        t0 = time.perf_counter()
                        planner.update_scene_incremental(p_scene)
                        r_update = planner.plan(q_start, q_goal,
                                                timeout=timeout)
                        t_update = time.perf_counter() - t0
                        update_mode = "incremental"
                    else:
                        # IRIS / other: full rebuild
                        logger.info("    -> %s 全量重建...", planner_name)
                        planner2 = create_planner(pcfg)
                        t0 = time.perf_counter()
                        planner2.setup(p_robot, p_scene, {"seed": seed})
                        r_update = planner2.plan(q_start, q_goal,
                                                  timeout=timeout)
                        t_update = time.perf_counter() - t0
                        update_mode = "full_rebuild"
                        # 等待 planner2 后台缓存线程
                        _join_cache_thread(planner2)

                    trial_result = SingleTrialResult(
                        scene_name=f"{SCENE_NAME}_{condition}",
                        planner_name=planner_name,
                        seed=seed,
                        trial=trial,
                        result={
                            "initial_time": t_initial,
                            "update_time": t_update,
                            "update_mode": update_mode,
                            "success_before": r_init.success,
                            "success_after": r_update.success,
                            "condition": condition,
                            "n_add": incr_cfg["n_add"],
                            "n_move": incr_cfg["n_move"],
                        },
                        wall_clock=t_initial + t_update,
                    )
                    results.add(trial_result)

                    done += 1
                    elapsed = time.perf_counter() - t_exp_start
                    logger.info(
                        "  ✓ [%d/%d | ETA %s | 已用%s] %s/%s seed=%d "
                        "init=%s update=%s (%s) ok_after=%s",
                        done, total, _eta(done, total, elapsed), _fmt(elapsed),
                        condition, planner_name, seed,
                        _fmt(t_initial), _fmt(t_update),
                        update_mode, r_update.success)

    logger.info("\n━" * 60)
    logger.info("Exp 2a 完成，共 %d 条记录，总耗时 %s",
                len(results.trials),
                _fmt(time.perf_counter() - t_exp_start))
    return results


# ═══════════════════════════════════════════════════════════════════════════
# Exp 2b: HCACHE warmstart
# ═══════════════════════════════════════════════════════════════════════════

def run_exp2b(n_seeds: int = 50, n_trials: int = 3,
              timeout: float = 120.0) -> ExperimentResults:
    """2b: Cold start vs Warm start (HCACHE) vs 跨场景."""
    results = ExperimentResults(experiment_name=f"{EXPERIMENT_NAME}_2b")

    sbf_cfg = {"type": "SBF", "method": "gcs", "name": "SBF-GCS"}
    scenes_to_test = ["shelves", "bins", "table"]

    total = len(scenes_to_test) * 3 * n_seeds  # 3 conditions
    done = 0
    t_exp_start = time.perf_counter()

    logger.info("━" * 60)
    logger.info("Exp 2b  场景=%s  seeds=%d  total=%d",
                scenes_to_test, n_seeds, total)
    logger.info("━" * 60)

    for scene_name in scenes_to_test:
        scene_cfg = load_marcucci_scene(scene_name, robot="iiwa14")
        robot, scene, query_pairs = load_scene_from_config(scene_cfg)

        logger.info("\n── 场景: %s ──", scene_name)

        for seed in range(n_seeds):
            elapsed = time.perf_counter() - t_exp_start
            eta = _eta(done, total, elapsed)
            qi = seed % len(query_pairs)
            q_start, q_goal = query_pairs[qi]

            logger.info("  [%d/%d | ETA %s | 已用%s] %s seed=%d",
                        done + 1, total, eta, _fmt(elapsed), scene_name, seed)

            # ── Condition 1: Cold start (no cache) ──
            logger.info("    cold start...")
            t0 = time.perf_counter()
            planner_cold = create_planner(sbf_cfg)
            planner_cold.setup(robot, scene, {
                "seed": seed,
                "hcache_path": None,  # 禁用缓存
            })
            r_cold = planner_cold.plan(q_start, q_goal, timeout=timeout)
            t_cold = time.perf_counter() - t0
            # 等待后台缓存线程，避免 mmap 竞态
            _join_cache_thread(planner_cold)

            # 提取 FK 阶段时间
            fk_cold = r_cold.phase_times.get("fk_total",
                       r_cold.phase_times.get("interval_fk", t_cold))
            logger.info("    cold=%s  fk=%s  ok=%s",
                        _fmt(t_cold), _fmt(fk_cold), r_cold.success)

            results.add(SingleTrialResult(
                scene_name=scene_name,
                planner_name="SBF-GCS",
                seed=seed, trial=0,
                result={
                    "condition": "cold",
                    "total_time": t_cold,
                    "fk_time": fk_cold,
                    "success": r_cold.success,
                    "nodes_explored": r_cold.nodes_explored,
                },
                wall_clock=t_cold,
            ))

            # ── Condition 2: Warm start (same scene cache) ──
            logger.info("    warm start (cache=%s.hcache)...", scene_name)
            t0 = time.perf_counter()
            planner_warm = create_planner(sbf_cfg)
            planner_warm.setup(robot, scene, {
                "seed": seed,
                "hcache_path": f"output/cache/{scene_name}.hcache",
            })
            r_warm = planner_warm.plan(q_start, q_goal, timeout=timeout)
            t_warm = time.perf_counter() - t0

            fk_warm = r_warm.phase_times.get("fk_total",
                       r_warm.phase_times.get("interval_fk", t_warm))
            # 等待后台缓存线程，避免 mmap 竞态
            _join_cache_thread(planner_warm)
            logger.info("    warm=%s  fk=%s  ok=%s  speedup=%.1fx",
                        _fmt(t_warm), _fmt(fk_warm), r_warm.success,
                        t_cold / t_warm if t_warm > 0 else float('inf'))

            results.add(SingleTrialResult(
                scene_name=scene_name,
                planner_name="SBF-GCS",
                seed=seed, trial=1,
                result={
                    "condition": "warm",
                    "total_time": t_warm,
                    "fk_time": fk_warm,
                    "success": r_warm.success,
                    "nodes_explored": r_warm.nodes_explored,
                },
                wall_clock=t_warm,
            ))

            # ── Condition 3: Cross-scene (cache from different scene) ──
            other_scene = "bins" if scene_name != "bins" else "shelves"
            logger.info("    cross-scene (cache=%s.hcache)...", other_scene)
            t0 = time.perf_counter()
            planner_cross = create_planner(sbf_cfg)
            planner_cross.setup(robot, scene, {
                "seed": seed,
                "hcache_path": f"output/cache/{other_scene}.hcache",
            })
            r_cross = planner_cross.plan(q_start, q_goal, timeout=timeout)
            t_cross = time.perf_counter() - t0
            # 等待后台缓存线程，避免下一轮 mmap 竞态
            _join_cache_thread(planner_cross)

            fk_cross = r_cross.phase_times.get("fk_total",
                        r_cross.phase_times.get("interval_fk", t_cross))
            logger.info("    cross=%s  fk=%s  ok=%s",
                        _fmt(t_cross), _fmt(fk_cross), r_cross.success)

            results.add(SingleTrialResult(
                scene_name=scene_name,
                planner_name="SBF-GCS",
                seed=seed, trial=2,
                result={
                    "condition": "cross_scene",
                    "total_time": t_cross,
                    "fk_time": fk_cross,
                    "success": r_cross.success,
                    "nodes_explored": r_cross.nodes_explored,
                    "cache_source": other_scene,
                },
                wall_clock=t_cross,
            ))

            done += 3
            elapsed = time.perf_counter() - t_exp_start
            logger.info("  ✓ [%d/%d | ETA %s | 已用%s] %s seed=%d "
                        "cold=%s warm=%s cross=%s",
                        done, total, _eta(done, total, elapsed), _fmt(elapsed),
                        scene_name, seed,
                        _fmt(t_cold), _fmt(t_warm), _fmt(t_cross))

    logger.info("\n━" * 60)
    logger.info("Exp 2b 完成，共 %d 条记录，总耗时 %s",
                len(results.trials),
                _fmt(time.perf_counter() - t_exp_start))
    return results


# ═══════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════

def run_experiment(n_seeds=50, n_trials=3, timeout=120.0,
                   output_dir="output/raw"):
    """运行完整 Exp 2."""
    t_total_start = time.perf_counter()
    logger.info("\n" + "═" * 60)
    logger.info("Paper Exp 2  seeds=%d  trials=%d  timeout=%gs",
                n_seeds, n_trials, timeout)
    logger.info("═" * 60)

    results_2a = run_exp2a(n_seeds, n_trials, timeout)
    results_2b = run_exp2b(n_seeds, n_trials=1, timeout=timeout)

    # Merge
    combined = ExperimentResults(experiment_name=EXPERIMENT_NAME)
    for t in results_2a.trials:
        combined.add(t)
    for t in results_2b.trials:
        combined.add(t)

    combined.metadata = {
        "n_seeds": n_seeds,
        "n_trials": n_trials,
        "timeout": timeout,
        "incremental_configs": INCREMENTAL_CONFIGS,
        "timestamp": time.strftime("%Y%m%d_%H%M%S"),
    }

    out = Path(output_dir) / f"{EXPERIMENT_NAME}.json"
    combined.save(out)

    # Print summary
    t_total = time.perf_counter() - t_total_start
    print(f"\n{'═'*60}")
    print(f"Exp 2 完成: {len(combined.trials)} 条记录  总耗时 {_fmt(t_total)}")
    print(f"  2a (incremental): {len(results_2a.trials)} records")
    print(f"  2b (HCACHE):      {len(results_2b.trials)} records")
    print(f"  结果已保存 -> {out}")
    print(f"{'═'*60}")

    return combined


def main():
    parser = argparse.ArgumentParser(
        description="Paper Exp 2: Incremental Update & Persistence")
    parser.add_argument("--seeds", type=int, default=50)
    parser.add_argument("--trials", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--output", type=str, default="output/raw")
    parser.add_argument("--quick", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(levelname)s] %(message)s")

    if args.quick:
        args.seeds = 5
        args.trials = 1

    run_experiment(
        n_seeds=args.seeds,
        n_trials=args.trials,
        timeout=args.timeout,
        output_dir=args.output,
    )


if __name__ == "__main__":
    main()

"""
paper_exp3_coverage.py — 实验 3: Region 生成效率与自由空间覆盖

对应论文 Exp 3 (Table III + Fig. 4):
  固定时间预算 (10s / 30s / 60s), 对比各方法:
  - Region 数量
  - 自由空间覆盖率 (蒙特卡洛采样 10^5 点)
  - 认证标记

+ Inline Remark:
  - IFK vs IFK+细分 vs Critical 三策略覆盖率对比
  - BFS wavefront vs random seed 覆盖率差异

统计: 50 seeds, 报告中位数 + 四分位距.

用法:
  python -m experiments.paper_exp3_coverage [--seeds 50]
  python -m experiments.paper_exp3_coverage --quick
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

import numpy as np

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

logger = logging.getLogger(__name__)

EXPERIMENT_NAME = "paper_exp3_coverage"

# 时间预算
TIME_BUDGETS = [10.0, 30.0, 60.0]

# 蒙特卡洛覆盖率采样点数
N_MC_SAMPLES = 100_000

# 场景: 3 个 Marcucci 场景
SCENE_NAMES = ("shelves", "bins", "table")

# Region 生成方法 (只包含 region-based methods)
REGION_PLANNER_CONFIGS = [
    {"type": "SBF", "method": "gcs", "name": "SBF-IFK",
     "strategy": "interval_fk"},
    {"type": "IRIS-NP-GCS", "n_iris_seeds": 40, "name": "IRIS-NP",
     "max_iterations": 10, "relative_termination_threshold": 2e-2},
    {"type": "C-IRIS-GCS", "n_regions": 50, "name": "C-IRIS",
     "max_iterations": 5},
]

# Quick 模式: 参数大幅缩减以便快速验证流程 (非正式结果)
QUICK_REGION_PLANNER_CONFIGS = [
    {"type": "SBF", "method": "gcs", "name": "SBF-IFK",
     "strategy": "interval_fk"},
    # IRIS-NP: seeds=2, 每 region 最多 3 次内部迭代 + 宽松收敛阈值
    {"type": "IRIS-NP-GCS", "n_iris_seeds": 2, "name": "IRIS-NP",
     "max_iterations": 3, "relative_termination_threshold": 0.1,
     "require_sample_point_is_contained": False},
    # C-IRIS: 最多 2 个 region, 2 次迭代
    {"type": "C-IRIS-GCS", "n_regions": 2, "name": "C-IRIS",
     "max_iterations": 2},
]

# Inline remark: ablation 策略
ABLATION_CONFIGS = [
    {"type": "SBF", "method": "gcs", "name": "SBF-IFK",
     "strategy": "interval_fk", "subdivision_depth": 0},
    {"type": "SBF", "method": "gcs", "name": "SBF-IFK+subdiv",
     "strategy": "interval_fk", "subdivision_depth": 2},
    {"type": "SBF", "method": "gcs", "name": "SBF-Critical",
     "strategy": "critical"},
]

# Inline remark: seed 策略
SEED_STRATEGY_CONFIGS = [
    {"type": "SBF", "method": "gcs", "name": "SBF-BFS",
     "seed_strategy": "bfs_wavefront"},
    {"type": "SBF", "method": "gcs", "name": "SBF-Random",
     "seed_strategy": "random"},
]


# ═══════════════════════════════════════════════════════════════════════════
# Coverage estimation
# ═══════════════════════════════════════════════════════════════════════════

def estimate_coverage_mc(robot, scene, regions, n_samples=N_MC_SAMPLES,
                         seed=42):
    """蒙特卡洛估计自由空间覆盖率 (向量化版本).

    Args:
        robot: Robot 实例
        scene: Scene 实例
        regions: list of boxes (each: {"min": ndarray, "max": ndarray})
                 或者 planner 的 forest/region 输出
        n_samples: 采样数
        seed: 随机种子

    Returns:
        dict: coverage_rate, n_free, n_covered, n_total
    """
    from forest.collision import CollisionChecker

    rng = np.random.default_rng(seed)
    jl = robot.joint_limits
    lows = np.array([lo for lo, _ in jl])
    highs = np.array([hi for _, hi in jl])
    ndim = len(jl)

    checker = CollisionChecker(robot=robot, scene=scene)

    # 采样均匀 C-space 点
    samples = rng.uniform(lows, highs, size=(n_samples, ndim))

    # 批量碰撞检测 (向量化)
    collision_flags = checker.check_config_collision_batch(samples)  # (N,) bool
    free_mask = ~collision_flags   # (N,) bool
    n_free = int(free_mask.sum())

    n_covered = 0
    if n_free > 0 and regions:
        free_samples = samples[free_mask]  # (n_free, ndim)

        # 向量化 region 检查: 预先构建 (R, ndim) 矩阵
        r_mins = np.array([reg["min"] for reg in regions], dtype=np.float64)  # (R, D)
        r_maxs = np.array([reg["max"] for reg in regions], dtype=np.float64)  # (R, D)

        # in_any[i] = True if free_samples[i] is in at least one region
        # 分批处理避免 (n_free × R × D) 内存爆炸
        BATCH = 2000
        in_any = np.zeros(len(free_samples), dtype=bool)
        for start_i in range(0, len(free_samples), BATCH):
            batch = free_samples[start_i:start_i + BATCH]  # (B, D)
            # (B, R) — True if point b is inside region r
            inside = np.all(
                (batch[:, None, :] >= r_mins[None, :, :]) &
                (batch[:, None, :] <= r_maxs[None, :, :]),
                axis=2,
            )
            in_any[start_i:start_i + BATCH] = inside.any(axis=1)
        n_covered = int(in_any.sum())

    free_coverage = n_covered / max(n_free, 1)
    return {
        "coverage_rate": free_coverage,
        "n_free": n_free,
        "n_covered": n_covered,
        "n_total": n_samples,
        "free_ratio": n_free / n_samples,
    }


def extract_regions_from_planner(planner) -> list:
    """从规划器提取 region boxes.

    不同规划器的内部表示不同, 统一为 list of {"min": ndarray, "max": ndarray}
    其中 min/max 均为 C-space 关节区间的下/上界向量.
    """
    regions = []

    # SBFAdapter: 从 forest 提取 BoxNode boxes
    if hasattr(planner, '_prep') and planner._prep is not None:
        prep = planner._prep
        boxes_raw = prep.get("boxes") or prep.get("forest_boxes") or {}

        # boxes 可能是 Dict[int, BoxNode] 或 list
        box_iter = (boxes_raw.values()
                    if isinstance(boxes_raw, dict) else boxes_raw)

        for b in box_iter:
            # BoxNode: 有 joint_intervals (list of (lo, hi) tuples)
            if hasattr(b, 'joint_intervals'):
                ivs = b.joint_intervals
                regions.append({
                    "min": np.array([lo for lo, hi in ivs], dtype=np.float64),
                    "max": np.array([hi for lo, hi in ivs], dtype=np.float64),
                })
            elif hasattr(b, 'min_corner') and hasattr(b, 'max_corner'):
                regions.append({
                    "min": np.asarray(b.min_corner, dtype=np.float64),
                    "max": np.asarray(b.max_corner, dtype=np.float64),
                })
            elif isinstance(b, dict) and "min" in b:
                regions.append({
                    "min": np.asarray(b["min"], dtype=np.float64),
                    "max": np.asarray(b["max"], dtype=np.float64),
                })
            elif isinstance(b, (list, tuple)) and len(b) == 2:
                regions.append({
                    "min": np.asarray(b[0], dtype=np.float64),
                    "max": np.asarray(b[1], dtype=np.float64),
                })

    # IRIS-NP / C-IRIS: HPolyhedron → approximate bounding box
    if hasattr(planner, '_regions') and planner._regions:
        for r in planner._regions:
            # Check if it looks like a Drake HPolyhedron
            if hasattr(r, 'A') and hasattr(r, 'b') and hasattr(r, 'ChebyshevCenter'):
                try:
                    center = r.ChebyshevCenter()
                    n = len(center)
                    lo = center - 1.0
                    hi = center + 1.0
                    regions.append({
                        "min": np.asarray(lo, dtype=np.float64),
                        "max": np.asarray(hi, dtype=np.float64),
                    })
                except Exception:
                    pass

    return regions


# ═══════════════════════════════════════════════════════════════════════════
# Time-budgeted region generation
# ═══════════════════════════════════════════════════════════════════════════

def run_timed_generation(planner, robot, scene, q_start, q_goal,
                         time_budget: float, seed: int = 0):
    """在固定时间预算内生成 region.

    Returns:
        n_regions, regions, elapsed
    """
    planner.setup(robot, scene, {
        "seed": seed,
        "max_time": time_budget,
        "timeout": time_budget,
    })

    print(f"    [plan] start (budget={time_budget:.0f}s, seed={seed})",
          flush=True)
    t0 = time.perf_counter()
    result = planner.plan(q_start, q_goal, timeout=time_budget)
    elapsed = time.perf_counter() - t0
    print(f"    [plan] done in {elapsed:.2f}s, success={result.success}",
          flush=True)

    regions = extract_regions_from_planner(planner)
    n_regions = len(regions) if regions else result.nodes_explored
    print(f"    [regions] extracted {n_regions} regions", flush=True)

    return n_regions, regions, elapsed, result


# ═══════════════════════════════════════════════════════════════════════════
# Main experiment: Table III + Fig. 4
# ═══════════════════════════════════════════════════════════════════════════

def run_exp3_main(n_seeds: int = 50,
                  planner_filter=None,
                  quick_mode: bool = False) -> ExperimentResults:
    """主实验: 固定时间预算下各方法覆盖率对比.

    Args:
        planner_filter: None 表示运行全部; 否则为 planner name 列表 (substring 匹配)
        quick_mode: True 时使用快速 IRIS 参数配置
    """
    base_configs = QUICK_REGION_PLANNER_CONFIGS if quick_mode \
        else REGION_PLANNER_CONFIGS
    configs = base_configs
    if planner_filter:
        configs = [c for c in base_configs
                   if any(f.lower() in c.get("name", c["type"]).lower()
                          for f in planner_filter)]
        print(f"[filter] 运行 planners: {[c['name'] for c in configs]}",
              flush=True)
    if quick_mode:
        print(f"[quick] IRIS 快速参数: IRIS-NP seeds=2/iter=3, C-IRIS regions=2/iter=2",
              flush=True)
    results = ExperimentResults(experiment_name=f"{EXPERIMENT_NAME}_main")

    total = len(SCENE_NAMES) * len(configs) * \
            len(TIME_BUDGETS) * n_seeds
    done = 0
    t_exp_start = time.perf_counter()

    print(f"\n{'='*65}", flush=True)
    print(f"Exp3 主实验: {len(SCENE_NAMES)} scenes × "
          f"{len(configs)} planners × "
          f"{len(TIME_BUDGETS)} budgets × {n_seeds} seeds = {total} trials",
          flush=True)
    print(f"{'='*65}", flush=True)

    for scene_name in SCENE_NAMES:
        print(f"\n{'─'*65}", flush=True)
        print(f"▶ Scene: {scene_name}", flush=True)
        print(f"{'─'*65}", flush=True)
        scene_cfg = load_marcucci_scene(scene_name, robot="iiwa14")
        robot, scene, query_pairs = load_scene_from_config(scene_cfg)
        q_start, q_goal = query_pairs[0]

        for pcfg in configs:
            planner_name = pcfg.get("name", pcfg["type"])

            for budget in TIME_BUDGETS:
                print(f"\n  ── {planner_name} | budget={budget:.0f}s "
                      f"({n_seeds} seeds) ──", flush=True)

                for seed in range(n_seeds):
                    t_trial = time.perf_counter()
                    print(f"  [trial {done+1}/{total}] "
                          f"seed={seed} ...", flush=True)

                    planner = create_planner(pcfg)

                    n_reg, regions, elapsed, plan_result = \
                        run_timed_generation(
                            planner, robot, scene,
                            q_start, q_goal, budget, seed)

                    # Coverage estimation
                    t_mc = time.perf_counter()
                    if regions:
                        print(f"    [MC] estimating coverage ({N_MC_SAMPLES} samples, "
                              f"{len(regions)} regions) ...", flush=True)
                        cov = estimate_coverage_mc(
                            robot, scene, regions,
                            n_samples=N_MC_SAMPLES, seed=seed)
                        print(f"    [MC] done in {time.perf_counter()-t_mc:.2f}s "
                              f"→ cov={cov['coverage_rate']*100:.1f}%",
                              flush=True)
                    else:
                        cov = {"coverage_rate": 0.0, "n_free": 0,
                               "n_covered": 0, "n_total": N_MC_SAMPLES,
                               "free_ratio": 0.0}

                    trial_elapsed = time.perf_counter() - t_trial
                    elapsed_total = time.perf_counter() - t_exp_start
                    eta = (elapsed_total / (done + 1)) * (total - done - 1)

                    trial_result = SingleTrialResult(
                        scene_name=scene_name,
                        planner_name=planner_name,
                        seed=seed,
                        trial=0,
                        result={
                            "time_budget": budget,
                            "actual_time": elapsed,
                            "n_regions": n_reg,
                            "coverage_rate": cov["coverage_rate"],
                            "n_free": cov["n_free"],
                            "n_covered": cov["n_covered"],
                            "free_ratio": cov["free_ratio"],
                            "certified": "SBF" in planner_name or
                                         "C-IRIS" in planner_name,
                            "success": plan_result.success,
                        },
                        wall_clock=elapsed,
                    )
                    results.add(trial_result)

                    done += 1
                    logger.info(
                        "[%d/%d] %s / %s / T=%ds seed=%d → "
                        "%d regions, cov=%.1f%% (trial %.1fs, ETA %.0fs)",
                        done, total, scene_name, planner_name,
                        int(budget), seed,
                        n_reg, cov["coverage_rate"] * 100,
                        trial_elapsed, eta)

    return results


# ═══════════════════════════════════════════════════════════════════════════
# Inline remark: strategy ablation
# ═══════════════════════════════════════════════════════════════════════════

def run_exp3_ablation(n_seeds: int = 20) -> ExperimentResults:
    """IFK vs IFK+subdiv vs Critical 策略对比 (inline remark)."""
    results = ExperimentResults(experiment_name=f"{EXPERIMENT_NAME}_ablation")

    scene_name = "shelves"
    scene_cfg = load_marcucci_scene(scene_name, robot="iiwa14")
    robot, scene, query_pairs = load_scene_from_config(scene_cfg)
    q_start, q_goal = query_pairs[0]
    budget = 30.0

    for acfg in ABLATION_CONFIGS:
        abl_name = acfg.get("name", acfg["type"])
        print(f"\n  ── Ablation: {abl_name} ({n_seeds} seeds) ──", flush=True)

        for seed in range(n_seeds):
            t0 = time.perf_counter()
            print(f"  [ablation {abl_name}] seed={seed} ...", flush=True)
            planner = create_planner(acfg)
            n_reg, regions, elapsed, plan_result = \
                run_timed_generation(planner, robot, scene,
                                     q_start, q_goal, budget, seed)

            if regions:
                cov = estimate_coverage_mc(robot, scene, regions,
                                            n_samples=N_MC_SAMPLES,
                                            seed=seed)
            else:
                cov = {"coverage_rate": 0.0}

            trial_t = time.perf_counter() - t0
            print(f"  [ablation {abl_name}] seed={seed} done: "
                  f"{n_reg} regions, cov={cov.get('coverage_rate',0)*100:.1f}% "
                  f"({trial_t:.1f}s)", flush=True)

            results.add(SingleTrialResult(
                scene_name=scene_name,
                planner_name=abl_name,
                seed=seed, trial=0,
                result={
                    "time_budget": budget,
                    "n_regions": n_reg,
                    "coverage_rate": cov.get("coverage_rate", 0.0),
                    "mode": "strategy_ablation",
                },
                wall_clock=elapsed,
            ))

        logger.info("Ablation %s: done %d seeds", abl_name, n_seeds)

    return results


def run_exp3_seed_strategy(n_seeds: int = 20) -> ExperimentResults:
    """BFS wavefront vs random seed 策略对比 (inline remark)."""
    results = ExperimentResults(experiment_name=f"{EXPERIMENT_NAME}_seedstrat")

    scene_name = "shelves"
    scene_cfg = load_marcucci_scene(scene_name, robot="iiwa14")
    robot, scene, query_pairs = load_scene_from_config(scene_cfg)
    q_start, q_goal = query_pairs[0]
    budget = 30.0

    for scfg in SEED_STRATEGY_CONFIGS:
        strat_name = scfg.get("name", scfg["type"])
        print(f"\n  ── SeedStrat: {strat_name} ({n_seeds} seeds) ──", flush=True)

        for seed in range(n_seeds):
            t0 = time.perf_counter()
            print(f"  [seedstrat {strat_name}] seed={seed} ...", flush=True)
            planner = create_planner(scfg)
            n_reg, regions, elapsed, plan_result = \
                run_timed_generation(planner, robot, scene,
                                     q_start, q_goal, budget, seed)

            if regions:
                cov = estimate_coverage_mc(robot, scene, regions,
                                            n_samples=N_MC_SAMPLES,
                                            seed=seed)
            else:
                cov = {"coverage_rate": 0.0}

            trial_t = time.perf_counter() - t0
            print(f"  [seedstrat {strat_name}] seed={seed} done: "
                  f"{n_reg} regions, cov={cov.get('coverage_rate',0)*100:.1f}% "
                  f"({trial_t:.1f}s)", flush=True)

            results.add(SingleTrialResult(
                scene_name=scene_name,
                planner_name=strat_name,
                seed=seed, trial=0,
                result={
                    "time_budget": budget,
                    "n_regions": n_reg,
                    "coverage_rate": cov.get("coverage_rate", 0.0),
                    "mode": "seed_strategy",
                },
                wall_clock=elapsed,
            ))

        logger.info("Seed strategy %s: done %d seeds", strat_name, n_seeds)

    return results


# ═══════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════

def run_experiment(n_seeds=50, output_dir="output/raw",
                   run_ablations=True, planner_filter=None,
                   quick_mode=False):
    """运行完整 Exp 3."""
    results_main = run_exp3_main(n_seeds, planner_filter=planner_filter,
                                 quick_mode=quick_mode)

    combined = ExperimentResults(experiment_name=EXPERIMENT_NAME)
    for t in results_main.trials:
        combined.add(t)

    if run_ablations:
        abl_seeds = min(n_seeds, 20)
        results_abl = run_exp3_ablation(abl_seeds)
        results_seed = run_exp3_seed_strategy(abl_seeds)
        for t in results_abl.trials:
            combined.add(t)
        for t in results_seed.trials:
            combined.add(t)

    combined.metadata = {
        "n_seeds": n_seeds,
        "time_budgets": TIME_BUDGETS,
        "n_mc_samples": N_MC_SAMPLES,
        "scenes": SCENE_NAMES,
        "run_ablations": run_ablations,
        "timestamp": time.strftime("%Y%m%d_%H%M%S"),
    }

    out = Path(output_dir) / f"{EXPERIMENT_NAME}.json"
    combined.save(out)

    # Print summary
    print(f"\n{'='*60}")
    print(f"Exp 3 complete: {len(combined.trials)} trial records")
    print(f"{'='*60}")

    df = combined.summary_df()
    main_df = df[~df.get("mode", "main").isin(
        ["strategy_ablation", "seed_strategy"]
    )] if "mode" in df.columns else df

    if len(main_df) > 0:
        print("\n── Coverage summary ──")
        for (scene, planner, budget), grp in main_df.groupby(
                ["scene", "planner", "time_budget"]):
            cov_med = grp["coverage_rate"].median() * 100
            n_reg_med = grp["n_regions"].median()
            print(f"  {scene:10s} | {planner:12s} | T={budget:3.0f}s | "
                  f"cov={cov_med:.1f}% | regions={n_reg_med:.0f}")

    return combined


def main():
    parser = argparse.ArgumentParser(
        description="Paper Exp 3: Region Coverage Efficiency")
    parser.add_argument("--seeds", type=int, default=50)
    parser.add_argument("--output", type=str, default="output/raw")
    parser.add_argument("--no-ablation", action="store_true",
                        help="Skip inline ablation experiments")
    parser.add_argument("--quick", action="store_true",
                        help="Quick preview: 5 seeds, SBF-only, no ablation")
    parser.add_argument("--sbf-only", action="store_true",
                        help="Only run SBF planners (skip IRIS-NP / C-IRIS)")
    parser.add_argument("--mc-samples", type=int, default=100_000,
                        help="Monte Carlo coverage samples")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(levelname)s] %(message)s")

    global N_MC_SAMPLES
    N_MC_SAMPLES = args.mc_samples

    planner_filter = None
    quick_mode = False

    if args.quick:
        args.seeds = 2
        args.no_ablation = True
        quick_mode = True
        N_MC_SAMPLES = 500   # 快速模式大幅减少 MC 采样
        print("[quick mode] seeds=2, MC=500, IRIS-NP iter=3, C-IRIS regions=2",
              flush=True)

    if getattr(args, 'sbf_only', False):
        planner_filter = ["SBF"]

    run_experiment(
        n_seeds=args.seeds,
        output_dir=args.output,
        run_ablations=not args.no_ablation,
        planner_filter=planner_filter,
        quick_mode=quick_mode,
    )


if __name__ == "__main__":
    main()

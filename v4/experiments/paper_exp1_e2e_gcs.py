"""
paper_exp1_e2e_gcs.py — 实验 1: 端到端 GCS 规划性能

对应论文 Exp 1 (Table I + Fig. 2):
  SBF-GCS vs IRIS-NP-GCS vs C-IRIS-GCS vs PRM
  在 Marcucci (shelves / bins / table) 场景上.

指标:
  - 规划成功率 (%)
  - 总 wall-clock = region 生成 + GCS 求解 (首次 / 摊还 K=10)
  - 路径长度 (C-space L2)
  - 路径平滑度 (acceleration cost)
  - 认证标记 (certified flag)

统计: 50 seeds × 3 trials, 报告中位数 + 四分位距.

用法:
  python -m experiments.paper_exp1_e2e_gcs [--seeds 50] [--trials 3]
  python -m experiments.paper_exp1_e2e_gcs --quick   # 快速预览 5 seeds
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

import numpy as np

# ── project path ──
_V4_ROOT = Path(__file__).resolve().parent.parent   # v4/
_ROOT = _V4_ROOT / "src"                             # v4/src
_EXP = Path(__file__).resolve().parent               # v4/experiments
for _p in (_V4_ROOT, _ROOT, _EXP):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from runner import (
    ExperimentRunner, ExperimentResults, SingleTrialResult,
    create_planner, load_scene_from_config,
)
from scenes import load_all_marcucci_scenes, load_planners

logger = logging.getLogger(__name__)

# ═══════════════════════════════════════════════════════════════════════════
# Experiment config
# ═══════════════════════════════════════════════════════════════════════════

EXPERIMENT_NAME = "paper_exp1_e2e_gcs"

# 场景: 3 个 Marcucci 场景
SCENE_NAMES = ("shelves", "bins", "table")

# 方法: SBF-GCS, IRIS-NP-GCS, C-IRIS-GCS, PRM
PLANNER_CONFIGS = [
    {"type": "SBF", "method": "gcs", "name": "SBF-GCS"},
    {"type": "IRIS-NP-GCS", "n_iris_seeds": 10, "name": "IRIS-NP-GCS"},
    {"type": "C-IRIS-GCS", "n_regions": 10, "name": "C-IRIS-GCS"},
    {"type": "PRM", "n_samples": 2000, "k_neighbors": 15, "name": "PRM"},
]

# 摊还查询次数
K_AMORTIZED = 10


# ═══════════════════════════════════════════════════════════════════════════
# Progress helpers
# ═══════════════════════════════════════════════════════════════════════════

_T0 = None  # 全局实验开始时间


def _fmt_time(secs: float) -> str:
    """将秒格式化为 mm:ss 或 hh:mm:ss."""
    secs = max(0.0, secs)
    h = int(secs // 3600)
    m = int((secs % 3600) // 60)
    s = int(secs % 60)
    return f"{h:02d}:{m:02d}:{s:02d}" if h else f"{m:02d}:{s:02d}"


def _eta(done: int, total: int, elapsed: float) -> str:
    """估算剩余时间."""
    if done == 0:
        return "--:--"
    remaining = elapsed / done * (total - done)
    return _fmt_time(remaining)


def _print_step(done: int, total: int, scene: str, planner: str,
                seed: int, trial: int, success: bool,
                wall: float, phase_times: dict, mode: str) -> None:
    """每步立即打印进度行."""
    global _T0
    elapsed = time.perf_counter() - _T0 if _T0 else 0.0
    pct = done / total * 100
    status = "\033[32mOK  \033[0m" if success else "\033[31mFAIL\033[0m"

    # 阶段时间摘要
    phase_str = ""
    if phase_times:
        parts = [f"{k}={v:.2f}s" for k, v in phase_times.items() if v > 0]
        if parts:
            phase_str = "  [" + " | ".join(parts) + "]"

    print(
        f"  [{done:4d}/{total}] {pct:5.1f}%  "
        f"{scene:<20s} {planner:<14s} "
        f"s{seed:02d}/t{trial}  {status}  "
        f"{wall:6.2f}s  "
        f"elapsed={_fmt_time(elapsed)}  ETA={_eta(done, total, elapsed)}"
        f"{phase_str}  ({mode})",
        flush=True,
    )


def compute_path_smoothness(path: np.ndarray) -> float:
    """计算路径平滑度 (二阶差分的 L2 范数之和 = 加速度代价).

    Args:
        path: (N, D) waypoints

    Returns:
        总加速度代价 (越小越平滑)
    """
    if path is None or len(path) < 3:
        return float("nan")
    # 二阶差分: a[i] = path[i+1] - 2*path[i] + path[i-1]
    accel = np.diff(path, n=2, axis=0)
    return float(np.sum(np.linalg.norm(accel, axis=1)))


def compute_path_length(path: np.ndarray) -> float:
    """路径长度 (C-space L2)."""
    if path is None or len(path) < 2:
        return float("inf")
    return float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))


# ═══════════════════════════════════════════════════════════════════════════
# Single-query run
# ═══════════════════════════════════════════════════════════════════════════

def run_single_query(planner, q_start, q_goal, timeout=60.0):
    """运行单次规划 + 收集详细指标."""
    t0 = time.perf_counter()
    result = planner.plan(q_start, q_goal, timeout=timeout)
    wall = time.perf_counter() - t0

    metrics = result.to_dict()
    metrics["wall_clock"] = wall
    metrics["path_length"] = compute_path_length(result.path)
    metrics["path_smoothness"] = compute_path_smoothness(result.path)
    metrics["certified"] = getattr(planner, '_certified', False) or \
        "SBF" in planner.name

    return result, metrics


# ═══════════════════════════════════════════════════════════════════════════
# Multi-query amortized run
# ═══════════════════════════════════════════════════════════════════════════

def run_amortized_queries(planner, query_pairs, K=10, timeout=60.0):
    """运行 K 次查询, 计算摊还时间.

    对于 supports_reuse=True 的方法: setup 一次 → K 次 plan()
    对于其他方法: 每次 K 次独立运行
    """
    metrics_list = []
    cumulative_time = 0.0
    n_success = 0

    for k in range(K):
        qi = k % len(query_pairs)
        q_start, q_goal = query_pairs[qi]

        t0 = time.perf_counter()
        result = planner.plan(q_start, q_goal, timeout=timeout)
        wall = time.perf_counter() - t0
        cumulative_time += wall

        if result.success:
            n_success += 1

        metrics = result.to_dict()
        metrics["query_k"] = k
        metrics["wall_clock"] = wall
        metrics["cumulative_time"] = cumulative_time
        metrics["amortized_time"] = cumulative_time / (k + 1)
        metrics["path_length"] = compute_path_length(result.path)
        metrics["path_smoothness"] = compute_path_smoothness(result.path)
        metrics_list.append(metrics)

    return metrics_list, {
        "K": K,
        "total_time": cumulative_time,
        "amortized_time": cumulative_time / K,
        "success_rate": n_success / K,
    }


# ═══════════════════════════════════════════════════════════════════════════
# Main experiment loop
# ═══════════════════════════════════════════════════════════════════════════

def run_experiment(n_seeds: int = 50, n_trials: int = 3,
                   timeout: float = 60.0,
                   output_dir: str = "output/raw",
                   planner_filter: list | None = None,
                   scene_filter: list | None = None,
                   verbose: bool = True) -> ExperimentResults:
    """运行实验 1: 端到端 GCS 规划性能.

    Args:
        n_seeds: 随机种子数
        n_trials: 每个 seed 的重复次数
        timeout: 单次规划超时 (秒)
        output_dir: 输出目录
        planner_filter: 只运行这些 planner 名称 (None = 全部)
        scene_filter: 只运行这些场景名称 (None = 全部)
    """
    global _T0
    _T0 = time.perf_counter()

    results = ExperimentResults(experiment_name=EXPERIMENT_NAME)
    seeds = list(range(n_seeds))

    # ── 加载场景 ──
    all_scene_cfgs = load_all_marcucci_scenes(robot="iiwa14")
    if scene_filter:
        all_scene_cfgs = [s for s in all_scene_cfgs
                          if s["name"] in scene_filter or
                          s["name"].replace("marcucci_", "") in scene_filter]
    scene_cfgs = all_scene_cfgs

    # ── 过滤 planners ──
    active_planners = PLANNER_CONFIGS
    if planner_filter:
        active_planners = [p for p in PLANNER_CONFIGS
                           if p.get("name", p["type"]) in planner_filter]
        if not active_planners:
            raise ValueError(f"No planners matched filter: {planner_filter}")

    total = len(scene_cfgs) * len(active_planners) * n_seeds * n_trials
    done = 0

    print(f"\n{'═'*72}", flush=True)
    print(f"  Exp1 E2E-GCS  |  "
          f"{len(scene_cfgs)} scenes × {len(active_planners)} planners × "
          f"{n_seeds} seeds × {n_trials} trials  =  {total} runs",
          flush=True)
    print(f"  timeout={timeout}s  output={output_dir}", flush=True)
    print(f"{'═'*72}\n", flush=True)

    for scene_cfg in scene_cfgs:
        scene_name = scene_cfg["name"]
        print(f"\n{'─'*72}", flush=True)
        print(f"  SCENE: {scene_name}", flush=True)
        print(f"{'─'*72}", flush=True)
        robot, scene, query_pairs = load_scene_from_config(scene_cfg)

        for pcfg in active_planners:
            planner_name = pcfg.get("name", pcfg["type"])
            print(f"\n  ▶ PLANNER: {planner_name}", flush=True)

            for seed in seeds:
                for trial in range(n_trials):
                    # ── Phase A: 首次查询 ──
                    t_setup0 = time.perf_counter()
                    planner = create_planner(pcfg)
                    planner.setup(robot, scene, {"seed": seed})
                    t_setup = time.perf_counter() - t_setup0

                    qi = (seed + trial) % len(query_pairs)
                    q_start, q_goal = query_pairs[qi]

                    result, metrics = run_single_query(
                        planner, q_start, q_goal, timeout)
                    metrics["setup_time"] = t_setup

                    trial_result = SingleTrialResult(
                        scene_name=scene_name,
                        planner_name=planner_name,
                        seed=seed,
                        trial=trial,
                        result={
                            **metrics,
                            "mode": "first_query",
                        },
                        wall_clock=metrics["wall_clock"],
                    )
                    results.add(trial_result)

                    # ── Phase B: 摊还查询 (K=10) ──
                    # 复用同一 planner (不 reset)
                    if planner.supports_reuse:
                        amor_list, amor_summary = run_amortized_queries(
                            planner, query_pairs, K=K_AMORTIZED, timeout=timeout)

                        trial_result_amor = SingleTrialResult(
                            scene_name=scene_name,
                            planner_name=planner_name,
                            seed=seed,
                            trial=trial,
                            result={
                                **amor_summary,
                                "mode": "amortized",
                                "certified": "SBF" in planner_name,
                            },
                            wall_clock=amor_summary["total_time"],
                        )
                        results.add(trial_result_amor)
                    else:
                        # 非复用方法: 独立运行 K 次
                        total_time = 0.0
                        n_ok = 0
                        for k in range(K_AMORTIZED):
                            if verbose:
                                print(f"    amor k={k+1}/{K_AMORTIZED} ...",
                                      end="", flush=True)
                            p2 = create_planner(pcfg)
                            p2.setup(robot, scene, {"seed": seed + k})
                            qk = k % len(query_pairs)
                            qs, qg = query_pairs[qk]
                            t_k0 = time.perf_counter()
                            r2 = p2.plan(qs, qg, timeout=timeout)
                            t_k = time.perf_counter() - t_k0
                            total_time += r2.planning_time
                            if r2.success:
                                n_ok += 1
                            if verbose:
                                ok = "\033[32mOK\033[0m" if r2.success else "\033[31mFAIL\033[0m"
                                print(f" {ok} {t_k:.2f}s", flush=True)

                        trial_result_amor = SingleTrialResult(
                            scene_name=scene_name,
                            planner_name=planner_name,
                            seed=seed,
                            trial=trial,
                            result={
                                "K": K_AMORTIZED,
                                "total_time": total_time,
                                "amortized_time": total_time / K_AMORTIZED,
                                "success_rate": n_ok / K_AMORTIZED,
                                "mode": "amortized",
                                "certified": False,
                            },
                            wall_clock=total_time,
                        )
                        results.add(trial_result_amor)

                    done += 1
                    if verbose:
                        phase_times = dict(metrics.get('phase_times', {}) or {})
                        phase_times['setup'] = round(t_setup, 3)
                        _print_step(done, total, scene_name, planner_name,
                                    seed, trial, result.success,
                                    metrics["wall_clock"], phase_times,
                                    "first_q")
                    logger.debug(
                        "[%d/%d] %s / %s / seed=%d trial=%d → %s (%.2fs)",
                        done, total, scene_name, planner_name,
                        seed, trial,
                        "OK" if result.success else "FAIL",
                        metrics["wall_clock"])

    # ── Save ──
    results.metadata = {
        "n_seeds": n_seeds,
        "n_trials": n_trials,
        "K_amortized": K_AMORTIZED,
        "timeout": timeout,
        "scenes": [s["name"] for s in scene_cfgs],
        "planners": [p.get("name", p["type"]) for p in active_planners],
        "timestamp": time.strftime("%Y%m%d_%H%M%S"),
    }

    out = Path(output_dir) / f"{EXPERIMENT_NAME}.json"
    results.save(out)
    return results


# ═══════════════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="Paper Exp 1: E2E GCS Planning Performance")
    parser.add_argument("--seeds", type=int, default=50)
    parser.add_argument("--trials", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--output", type=str, default="output/raw")
    parser.add_argument("--quick", action="store_true",
                        help="Quick preview: 5 seeds, 1 trial, SBF-GCS only")
    parser.add_argument("--planners", nargs="+", default=None,
                        metavar="PLANNER",
                        help="Only run these planners (e.g. SBF-GCS PRM)")
    parser.add_argument("--scenes", nargs="+", default=None,
                        metavar="SCENE",
                        help="Only run these scenes (e.g. shelves bins table)")
    parser.add_argument("--no-verbose", action="store_true",
                        help="关闭逐步进度打印 (仅保留 logger)")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(levelname)s] %(message)s")

    planner_filter = args.planners
    if args.quick:
        args.seeds = 5
        args.trials = 1
        if planner_filter is None:
            planner_filter = ["SBF-GCS"]  # quick 默认只跑最快的 planner
        logger.info("Quick mode: 5 seeds, 1 trial, planners=%s", planner_filter)

    results = run_experiment(
        n_seeds=args.seeds,
        n_trials=args.trials,
        timeout=args.timeout,
        output_dir=args.output,
        planner_filter=planner_filter,
        scene_filter=args.scenes,
        verbose=not args.no_verbose,
    )

    # Print summary
    print(f"\n{'='*60}")
    print(f"Exp 1 complete: {len(results.trials)} trial records")
    print(f"{'='*60}")

    df = results.summary_df()
    first_q = df[df["mode"] == "first_query"] if "mode" in df.columns else df
    if len(first_q) > 0:
        print("\n── First-query summary ──")
        for (scene, planner), grp in first_q.groupby(["scene", "planner"]):
            n = len(grp)
            sr = grp["success"].mean() * 100
            t_med = grp["wall_clock"].median()
            print(f"  {scene:20s} | {planner:14s} | "
                  f"SR={sr:.0f}% | t_med={t_med:.3f}s  (n={n})")


if __name__ == "__main__":
    main()

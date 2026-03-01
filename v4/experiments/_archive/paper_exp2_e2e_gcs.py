"""
paper_exp2_e2e_gcs.py — 实验 2: 端到端 GCS 规划性能

对应论文 Exp 2 (Table II):
  SBF-GCS vs C-IRIS-GCS，复用实验 1 生成的 region / boxforest.
  每个场景使用 1 个固定 query pair (q_start, q_goal).

设计:
  - C-IRIS: 实验 1 生成了 1 组 region → 每场景 1 次 GCS 求解
  - SBF: 实验 1 生成了 10 个 boxforest (seed 0-9) → 每场景 10 次 GCS 求解, 取平均

指标:
  - 规划成功率 (%)
  - 规划时间 plan_time (s)
  - 路径长度 (C-space L2)
  - 路径平滑度 (acceleration cost)

用法:
  python -m experiments.paper_exp2_e2e_gcs
  python -m experiments.paper_exp2_e2e_gcs --quick
  python -m experiments.paper_exp2_e2e_gcs --group baseline  # 只跑 C-IRIS
  python -m experiments.paper_exp2_e2e_gcs --group sbf       # 只跑 SBF
"""

from __future__ import annotations

import argparse
import json
import logging
import pickle
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
    ExperimentResults, SingleTrialResult,
    create_planner, load_scene_from_config,
)
from scenes import load_marcucci_scene
from marcucci_scenes import get_iiwa14_seed_points

logger = logging.getLogger(__name__)

# ═══════════════════════════════════════════════════════════════════════════
# Experiment config
# ═══════════════════════════════════════════════════════════════════════════

EXPERIMENT_NAME = "paper_exp2_e2e_gcs"

SCENE_NAMES = ("combined",)

# 实验 1 产物目录
ARTIFACTS_DIR = Path("output/exp1_artifacts")

# Notebook 预生成 IRIS regions 文件 (gcs-science-robotics)
IRIS_REG_PATH = Path(__file__).parent.parent.parent / "gcs-science-robotics/data/prm_comparison/IRIS.reg"

# SBF 重复次数 (应与实验 1 的 SBF_N_REPEATS 一致)
SBF_N_REPEATS = 10

# C-IRIS 配置 (fallback: 如果没有 exp1 产物, 现场生成)
CIRIS_CONFIG = {
    "type": "C-IRIS-GCS", "n_regions": 8, "name": "C-IRIS-GCS",
    "max_iterations": 10,
}
SBF_CONFIG = {
    "type": "SBF", "method": "gcs", "name": "SBF-GCS",
    "force_gcs": True,   # 始终运行 GCS solve，不走 direct-connect 捷径
}


# ═══════════════════════════════════════════════════════════════════════════
# Helpers
# ═══════════════════════════════════════════════════════════════════════════

def _fmt(seconds: float) -> str:
    if seconds < 1:
        return f"{seconds*1000:.1f}ms"
    if seconds < 60:
        return f"{seconds:.2f}s"
    m, s = divmod(int(seconds), 60)
    return f"{m}m{s:02d}s"


def compute_path_length(path: np.ndarray) -> float:
    if path is None or len(path) < 2:
        return float("inf")
    return float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))


def compute_path_smoothness(path: np.ndarray) -> float:
    if path is None or len(path) < 3:
        return float("nan")
    accel = np.diff(path, n=2, axis=0)
    return float(np.sum(np.linalg.norm(accel, axis=1)))


# ═══════════════════════════════════════════════════════════════════════════
# Load exp1 artifacts
# ═══════════════════════════════════════════════════════════════════════════

def load_ciris_planner(scene_name: str):
    """加载实验 1 保存的 C-IRIS planner (pickle)."""
    pkl_path = ARTIFACTS_DIR / f"ciris_{scene_name}.pkl"
    if pkl_path.exists():
        with open(pkl_path, 'rb') as f:
            return pickle.load(f)
    return None


def load_sbf_planners(scene_name: str, n_repeats: int = SBF_N_REPEATS):
    """加载实验 1 保存的 SBF planner 对象 (pickle)."""
    planners = []
    for i in range(n_repeats):
        pkl_path = ARTIFACTS_DIR / f"sbf_{scene_name}_seed{i}.pkl"
        if pkl_path.exists() and pkl_path.stat().st_size > 0:
            try:
                with open(pkl_path, 'rb') as f:
                    planners.append(pickle.load(f))
            except (EOFError, Exception) as e:
                logger.warning("Failed to load SBF planner %s: %s",
                               pkl_path, e)
    return planners


def build_ciris_fresh(scene_name: str, robot, scene, q_start, q_goal,
                      quick_mode: bool = False):
    """现场构建 C-IRIS planner (fallback)."""
    cfg = dict(CIRIS_CONFIG)
    if quick_mode:
        cfg["n_regions"] = 2
        cfg["max_iterations"] = 3
    planner = create_planner(cfg)
    manual_seeds = list(get_iiwa14_seed_points().values())
    planner.setup(robot, scene, {"seed": 0, "manual_seeds": manual_seeds})
    planner.plan(q_start, q_goal, timeout=600.0)
    return planner


def build_sbf_fresh(scene_name: str, robot, scene, q_start, q_goal,
                    seed: int, quick_mode: bool = False):
    """现场构建 SBF planner (fallback)."""
    cfg = dict(SBF_CONFIG)
    planner = create_planner(cfg)
    max_boxes = 2000 if quick_mode else 10000
    manual_seeds = list(get_iiwa14_seed_points().values())
    planner.setup(robot, scene, {
        "seed": seed,
        "max_boxes": max_boxes,
        "max_consecutive_miss": 500,
        "extra_seeds": manual_seeds,
    })
    planner.plan(q_start, q_goal, timeout=600.0)
    return planner


# ═══════════════════════════════════════════════════════════════════════════
# Notebook GCS: LinearGCS + MosekSolver，直接使用 IRIS.reg (完全复现 notebook)
# ═══════════════════════════════════════════════════════════════════════════

def run_notebook_gcs(q_start: np.ndarray, q_goal: np.ndarray,
                     regions, n_repeats: int = SBF_N_REPEATS,
                     solver_tolerance: float = 1e-3) -> dict:
    """与 notebook getGCSPath 完全一致的 LinearGCS + MosekSolver 规划.

    Returns dict with keys: success, times, path_lengths, smoothnesses
    """
    import sys
    gcs_dir = str(Path("../gcs-science-robotics").resolve())
    if gcs_dir not in sys.path:
        sys.path.insert(0, gcs_dir)

    from gcs.linear import LinearGCS
    from gcs.rounding import randomForwardPathSearch
    from pydrake.solvers import MosekSolver

    region_list = list(regions.values()) if isinstance(regions, dict) else regions

    times, path_lengths, smoothnesses, successes = [], [], [], []

    for i in range(n_repeats):
        gcs = LinearGCS(region_list)
        gcs.addSourceTarget(q_start, q_goal)
        gcs.setRoundingStrategy(randomForwardPathSearch,
                                max_paths=10, max_trials=100, seed=i)
        gcs.setSolver(MosekSolver())
        opts = gcs.options.solver_options
        for key in ('MSK_DPAR_INTPNT_TOL_PFEAS', 'MSK_DPAR_INTPNT_TOL_DFEAS',
                    'MSK_DPAR_INTPNT_TOL_REL_GAP', 'MSK_DPAR_INTPNT_TOL_INFEAS'):
            opts.SetOption(MosekSolver.id(), key, solver_tolerance)
        gcs.options.solver_options = opts

        t0 = time.perf_counter()
        waypoints, results_dict = gcs.SolvePath(rounding=True, verbose=False,
                                                preprocessing=True)
        # 与 notebook 一致: relaxation + max_rounded (可并行)
        plan_time = (results_dict.get("relaxation_solver_time", 0)
                     + results_dict.get("max_rounded_solver_time", 0))
        wall_time = time.perf_counter() - t0

        if waypoints is None:
            successes.append(False)
            times.append(plan_time)
            path_lengths.append(float("inf"))
            smoothnesses.append(float("nan"))
            print(f"    run={i} FAIL  wall={_fmt(wall_time)}", flush=True)
            continue

        path = waypoints.T  # (n_pts, ndim)
        plen = compute_path_length(path)
        psmooth = compute_path_smoothness(path)
        successes.append(True)
        times.append(plan_time)
        path_lengths.append(plen)
        smoothnesses.append(psmooth)
        print(f"    run={i} OK  solver_time={_fmt(plan_time)}  "
              f"wall={_fmt(wall_time)}  len={plen:.3f}  smooth={psmooth:.3f}",
              flush=True)

    return dict(successes=successes, times=times,
                path_lengths=path_lengths, smoothnesses=smoothnesses)


# ═══════════════════════════════════════════════════════════════════════════
# Main experiment: 复用 Exp1 产物, 每场景 1 个 query pair
# ═══════════════════════════════════════════════════════════════════════════

def run_experiment(planner_group: str | None = None,
                   quick_mode: bool = False,
                   timeout: float = 60.0,
                   output_dir: str = "output/raw") -> ExperimentResults:
    """运行实验 2: 端到端 GCS 规划性能."""
    t_exp_start = time.perf_counter()
    results = ExperimentResults(experiment_name=EXPERIMENT_NAME)

    run_sbf = planner_group is None or planner_group == "sbf"
    run_ciris = planner_group is None or planner_group == "baseline"
    run_notebook = planner_group == "notebook_gcs"
    n_repeats = 2 if quick_mode else SBF_N_REPEATS

    sep = '=' * 72
    dash = '-' * 72
    print(f"\n{sep}", flush=True)
    print(f"  Exp 2 E2E-GCS  |  {len(SCENE_NAMES)} scenes  |  "
          f"SBF={'ON' if run_sbf else 'OFF'} ({n_repeats}x)  "
          f"C-IRIS={'ON' if run_ciris else 'OFF'} (1x)  "
          f"Notebook-GCS={'ON' if run_notebook else 'OFF'} ({n_repeats}x)",
          flush=True)
    print(f"  复用 Exp1 产物: {ARTIFACTS_DIR}", flush=True)
    print(f"{sep}\n", flush=True)

    for si, scene_name in enumerate(SCENE_NAMES):
        print(f"\n{'─'*72}", flush=True)
        print(f"  SCENE [{si+1}/{len(SCENE_NAMES)}]: {scene_name}", flush=True)
        print(f"{'─'*72}", flush=True)

        scene_cfg = load_marcucci_scene(scene_name, robot="iiwa14")
        robot, scene, query_pairs = load_scene_from_config(scene_cfg)
        q_start, q_goal = query_pairs[0]  # 固定 1 个 query pair
        print(f"  obstacles={len(scene_cfg['obstacles'])}, "
              f"q_start={np.round(q_start,3).tolist()}", flush=True)
        print(f"  q_goal ={np.round(q_goal,3).tolist()}", flush=True)

        # ── C-IRIS: 1 次 GCS 求解 ──
        if run_ciris:
            print(f"\n  ▶ C-IRIS-GCS (1 run)", flush=True)
            planner = load_ciris_planner(scene_name)
            if planner is None:
                print(f"    [fallback] 未找到 exp1 产物, 现场构建...",
                      flush=True)
                planner = build_ciris_fresh(
                    scene_name, robot, scene, q_start, q_goal,
                    quick_mode=quick_mode)
            else:
                print(f"    [loaded] exp1 artifact: {ARTIFACTS_DIR / f'ciris_{scene_name}.pkl'}",
                      flush=True)

            t0 = time.perf_counter()
            result = planner.plan(q_start, q_goal, timeout=timeout)
            plan_time = time.perf_counter() - t0

            path_len = compute_path_length(result.path)
            path_smooth = compute_path_smoothness(result.path)

            status = "OK" if result.success else "FAIL"
            print(f"    [{status}] plan_time={_fmt(plan_time)}  "
                  f"path_len={path_len:.3f}  smooth={path_smooth:.3f}",
                  flush=True)

            results.add(SingleTrialResult(
                scene_name=scene_name,
                planner_name="C-IRIS-GCS",
                seed=0, trial=0,
                result={
                    "success": result.success,
                    "plan_time": plan_time,
                    "path_length": path_len,
                    "path_smoothness": path_smooth,
                    "certified": True,
                },
                wall_clock=plan_time,
            ))

        # ── SBF: 10 次 GCS 求解 (10 个 boxforest) ──
        if run_sbf:
            print(f"\n  ▶ SBF-GCS ({n_repeats} runs)", flush=True)
            sbf_planners = load_sbf_planners(scene_name, n_repeats)
            print(f"    loaded {len(sbf_planners)}/{n_repeats} planners from exp1 artifacts",
                  flush=True)

            # Fallback: 如果 pickle 不够, 现场补建
            while len(sbf_planners) < n_repeats:
                seed = len(sbf_planners)
                print(f"    [fallback] seed={seed} 未找到产物, 现场构建...",
                      flush=True)
                t_fb = time.perf_counter()
                p = build_sbf_fresh(
                    scene_name, robot, scene, q_start, q_goal,
                    seed=seed, quick_mode=quick_mode)
                print(f"    [fallback] seed={seed} built in {_fmt(time.perf_counter()-t_fb)}",
                      flush=True)
                sbf_planners.append(p)

            plan_times = []
            path_lengths = []
            path_smooths = []
            successes = []

            for seed, planner in enumerate(sbf_planners):
                t0 = time.perf_counter()
                result = planner.plan(q_start, q_goal, timeout=timeout)
                plan_time = time.perf_counter() - t0

                path_len = compute_path_length(result.path)
                path_smooth = compute_path_smoothness(result.path)

                plan_times.append(plan_time)
                path_lengths.append(path_len)
                path_smooths.append(path_smooth)
                successes.append(result.success)

                status = "OK" if result.success else "FAIL"
                n_boxes_info = ""
                if hasattr(result, 'nodes_explored') and result.nodes_explored:
                    n_boxes_info = f"  boxes={result.nodes_explored}"
                pt_info = ""
                if hasattr(result, 'phase_times') and result.phase_times:
                    pt_info = "  " + ", ".join(
                        f"{k}={_fmt(v)}" for k, v in result.phase_times.items())
                print(f"    seed={seed} [{status}] plan_time={_fmt(plan_time)}  "
                      f"path_len={path_len:.3f}  smooth={path_smooth:.3f}"
                      f"{n_boxes_info}{pt_info}", flush=True)

                results.add(SingleTrialResult(
                    scene_name=scene_name,
                    planner_name="SBF-GCS",
                    seed=seed, trial=0,
                    result={
                        "success": result.success,
                        "plan_time": plan_time,
                        "path_length": path_len,
                        "path_smoothness": path_smooth,
                        "certified": True,
                    },
                    wall_clock=plan_time,
                ))

            # 打印平均
            sr = sum(successes) / len(successes) * 100
            avg_pt = np.mean(plan_times)
            avg_pl = np.mean([l for l, s in zip(path_lengths, successes) if s]
                             ) if any(successes) else float("inf")
            print(f"    [avg] SR={sr:.0f}%  plan_time={_fmt(avg_pt)}  "
                  f"path_len={avg_pl:.3f}", flush=True)

        # ── Notebook-GCS: 直接加载 IRIS.reg + LinearGCS + MosekSolver ──
        if run_notebook:
            print(f"\n  \u25b6 Notebook-GCS [{n_repeats} runs] (IRIS.reg + LinearGCS + MosekSolver)",
                  flush=True)
            iris_reg = IRIS_REG_PATH if IRIS_REG_PATH.is_absolute() \
                else Path(__file__).parent / IRIS_REG_PATH
            if not iris_reg.exists():
                print(f"    [ERROR] IRIS.reg 不存在: {iris_reg}", flush=True)
            else:
                print(f"    [load] {iris_reg}", flush=True)
                with open(iris_reg, "rb") as f:
                    iris_regions = pickle.load(f)
                print(f"    [load] {len(iris_regions)} regions: {list(iris_regions.keys())}",
                      flush=True)

                # 与 notebook 一致: 使用 IK 种子点作为 s-t
                # (scene query_pairs 不在 IRIS region 内, 不能直接用)
                seed_pts = get_iiwa14_seed_points()
                q_nb_start = seed_pts["LB"]
                q_nb_goal  = seed_pts["TS"]
                print(f"    [query] LB→TS (IK seeds, in IRIS region)", flush=True)
                print(f"    [query]   start={np.round(q_nb_start, 4).tolist()}",
                      flush=True)
                print(f"    [query]   goal ={np.round(q_nb_goal,  4).tolist()}",
                      flush=True)

                nb_result = run_notebook_gcs(
                    q_nb_start, q_nb_goal, iris_regions,
                    n_repeats=n_repeats)

                for i, (ok, pt, pl, ps) in enumerate(zip(
                        nb_result["successes"], nb_result["times"],
                        nb_result["path_lengths"], nb_result["smoothnesses"])):
                    results.add(SingleTrialResult(
                        scene_name=scene_name,
                        planner_name="Notebook-GCS",
                        seed=i, trial=0,
                        result={
                            "success": ok,
                            "plan_time": pt,
                            "path_length": pl,
                            "path_smoothness": ps,
                            "certified": True,
                        },
                        wall_clock=pt,
                    ))

                sr = sum(nb_result["successes"]) / n_repeats * 100
                avg_pt = np.mean(nb_result["times"])
                success_lens = [l for l, s in zip(nb_result["path_lengths"],
                                                  nb_result["successes"]) if s]
                avg_pl = np.mean(success_lens) if success_lens else float("inf")
                print(f"    [avg] SR={sr:.0f}%  solver_time={_fmt(avg_pt)}  "
                      f"path_len={avg_pl:.3f}", flush=True)

    # ── Save ──
    results.metadata = {
        "sbf_n_repeats": n_repeats,
        "ciris_n_regions": CIRIS_CONFIG["n_regions"],
        "timeout": timeout,
        "scenes": list(SCENE_NAMES),
        "planner_group": planner_group,
        "quick_mode": quick_mode,
        "timestamp": time.strftime("%Y%m%d_%H%M%S"),
    }

    suffix = f"_{planner_group}" if planner_group else ""
    out = Path(output_dir) / f"{EXPERIMENT_NAME}{suffix}.json"
    results.save(out)

    # ── Print summary ──
    t_total = time.perf_counter() - t_exp_start
    print(f"\n{'═'*72}", flush=True)
    print(f"  Exp 2 complete: {len(results.trials)} records  "
          f"total={_fmt(t_total)}", flush=True)
    print(f"{'═'*72}", flush=True)

    df = results.summary_df()
    if len(df) > 0:
        print("\n── Summary (plan_time | path_length) ──")
        for (sc, pl), grp in df.groupby(["scene", "planner"]):
            n = len(grp)
            sr = grp["success"].mean() * 100 if "success" in grp.columns else 0
            pt = grp["plan_time"].mean() if "plan_time" in grp.columns \
                else grp["wall_clock"].mean()
            pl_val = grp["path_length"].mean() if "path_length" in grp.columns \
                else float("nan")
            print(f"  {sc:10s} | {pl:12s} | "
                  f"SR={sr:.0f}% | plan={pt:.4f}s | "
                  f"path={pl_val:.3f}  (n={n})")

    return results


# ═══════════════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="Paper Exp 2: E2E GCS Planning (reuse Exp1 artifacts)")
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--output", type=str, default="output/raw")
    parser.add_argument("--quick", action="store_true",
                        help="Quick mode: 2 SBF repeats, 2 IRIS regions")
    parser.add_argument("--group", choices=["baseline", "sbf", "notebook_gcs"],
                        default=None,
                        help="只运行指定分组: baseline (C-IRIS) 或 sbf 或 notebook_gcs")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(levelname)s] %(message)s")

    run_experiment(
        planner_group=args.group,
        quick_mode=args.quick,
        timeout=args.timeout,
        output_dir=args.output,
    )


if __name__ == "__main__":
    main()

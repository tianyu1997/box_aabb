"""
paper_exp2_planning.py — 实验 2: 端到端 GCS 路径规划

设计:
  - 加载 exp1 保存的 SBF forest artifacts (intervals + adjacency)
  - 对每个 seed 的 forest, 查询全部 5 个 s-t pairs
  - SBF path: gcs_utils.solve_gcs_from_sbf() (LinearGCS + MosekSolver)
  - IRIS baseline: 从 IRIS.reg 加载 regions, LinearGCS 规划
  - 测量: solver_time, path_length, success_rate (per pair & overall)

用法:
  python -m experiments.paper_exp2_planning                 # 全量运行
  python -m experiments.paper_exp2_planning --quick         # 快速模式
  python -m experiments.paper_exp2_planning --seeds 5       # 自定义 seed 数
"""

from __future__ import annotations

import argparse
import json
import logging
import pickle
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

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
PAIR_NAMES = ["AS→TS", "TS→CS", "CS→LB", "LB→RB", "RB→AS"]
ARTIFACTS_DIR = Path(__file__).resolve().parent.parent / "output" / "exp1_artifacts"
IRIS_REG_PATH = (_PROJ_ROOT / "gcs-science-robotics" / "data" /
                 "prm_comparison" / "IRIS.reg")

# ═══════════════════════════════════════════════════════════════════════════
# Helpers
# ═══════════════════════════════════════════════════════════════════════════

def _fmt(seconds: float) -> str:
    if seconds < 1:
        return f"{seconds*1000:.1f}ms"
    return f"{seconds:.2f}s"


def get_query_pairs_np():
    """获取 5 个 (start, goal) pairs — numpy arrays."""
    from marcucci_scenes import get_query_pairs
    return get_query_pairs("combined")


def load_forest_artifact(seed: int):
    """加载 exp1 保存的 forest artifact."""
    path = ARTIFACTS_DIR / f"sbf_forest_seed{seed:03d}.pkl"
    if not path.exists():
        return None
    with open(path, "rb") as f:
        return pickle.load(f)


# ═══════════════════════════════════════════════════════════════════════════
# SBF + GCS 规划
# ═══════════════════════════════════════════════════════════════════════════

def run_sbf_gcs(n_seeds: int, quick: bool = False):
    """对每个 seed 的预构建 forest, 查询全部 5 个 s-t pairs."""
    from gcs_utils import solve_gcs_from_sbf, sbf_to_lineargcs, path_length

    pairs_np = get_query_pairs_np()
    n_pairs = len(pairs_np)
    n_seeds = min(n_seeds, 2) if quick else n_seeds

    print(f"\n{'='*60}")
    print(f"  SBF-GCS: {n_pairs} pairs × {n_seeds} seeds")
    print(f"{'='*60}")

    # per-pair stats
    pair_results = {i: [] for i in range(n_pairs)}  # pair_idx → list of dicts
    all_results = []

    for seed in range(n_seeds):
        data = load_forest_artifact(seed)
        if data is None:
            print(f"  s{seed:2d}  [SKIP] artifact 不存在, 先运行 exp1")
            continue

        intervals_lo = data["intervals_lo"]
        intervals_hi = data["intervals_hi"]
        interval_ids = data["interval_ids"]
        adjacency = data["adjacency"]
        n_boxes = data.get("n_boxes", len(interval_ids))

        print(f"\n  seed={seed}  n_boxes={n_boxes}", flush=True)

        for pi, (q_start, q_goal) in enumerate(pairs_np):
            print(f"    [{PAIR_NAMES[pi]}] solving ...", end="", flush=True)
            t0 = time.perf_counter()
            path, info = solve_gcs_from_sbf(
                intervals_lo, intervals_hi, interval_ids, adjacency,
                q_start, q_goal, rounding=True, verbose=False, seed=seed,
                neighborhood_hops=0)
            wall_time = time.perf_counter() - t0

            success = info.get("success", False)
            solver_time = info.get("solver_time", 0.0)
            dijk_time = info.get("dijkstra_time", 0.0)
            n_path = info.get("n_path_boxes", 0)
            n_sub = info.get("n_subgraph_boxes", 0)
            plen = info.get("path_length", float("inf")) if success else float("inf")

            status = "OK" if success else "FAIL"
            if success:
                print(f"\r    {PAIR_NAMES[pi]:8s}  [{status}]  "
                      f"dijk={_fmt(dijk_time)}  solver={_fmt(solver_time)}  "
                      f"wall={_fmt(wall_time)}  len={plen:.3f}  "
                      f"path={n_path}  sub={n_sub}", flush=True)
            else:
                err = info.get("error", "")
                print(f"\r    {PAIR_NAMES[pi]:8s}  [{status}]  "
                      f"dijk={_fmt(dijk_time)}  solver={_fmt(solver_time)}  "
                      f"wall={_fmt(wall_time)}  {err}", flush=True)

            result = {
                "seed": seed,
                "pair_idx": pi,
                "pair_name": PAIR_NAMES[pi],
                "success": success,
                "solver_time": solver_time,
                "dijkstra_time": dijk_time,
                "wall_time": wall_time,
                "path_length": plen,
                "n_path_boxes": n_path,
                "n_subgraph_boxes": n_sub,
                "n_regions": info.get("n_regions", 0),
                "n_edges": info.get("n_edges", 0),
            }
            pair_results[pi].append(result)
            all_results.append(result)

    # Summary per pair
    print(f"\n  SBF-GCS Summary:")
    print(f"  {'Pair':8s}  {'SR':6s}  {'solver_median':14s}  {'len_median':12s}")
    for pi in range(n_pairs):
        pr = pair_results[pi]
        if not pr:
            continue
        sr = sum(1 for r in pr if r["success"]) / len(pr)
        s_times = [r["solver_time"] for r in pr if r["success"]]
        s_lens = [r["path_length"] for r in pr if r["success"]]
        med_t = f"{np.median(s_times)*1000:.1f}ms" if s_times else "N/A"
        med_l = f"{np.median(s_lens):.3f}" if s_lens else "N/A"
        print(f"  {PAIR_NAMES[pi]:8s}  {sr*100:5.1f}%  {med_t:>14s}  {med_l:>12s}")

    # Overall
    total = len(all_results)
    total_ok = sum(1 for r in all_results if r["success"])
    print(f"\n  Overall: {total_ok}/{total} = {total_ok/total*100:.1f}% SR" if total else "")

    return all_results


# ═══════════════════════════════════════════════════════════════════════════
# IRIS baseline GCS 规划
# ═══════════════════════════════════════════════════════════════════════════

def run_iris_gcs(n_repeats: int = 5, quick: bool = False):
    """使用 IRIS.reg 预计算 regions 进行 GCS 规划."""
    from gcs_utils import iris_to_lineargcs, solve_gcs_path

    # 加载 IRIS regions
    iris_pkl = ARTIFACTS_DIR / "iris_regions.pkl"
    if iris_pkl.exists():
        with open(iris_pkl, "rb") as f:
            data = pickle.load(f)
        regions = data.get("regions", data)
    elif IRIS_REG_PATH.exists():
        with open(IRIS_REG_PATH, "rb") as f:
            regions = pickle.load(f)
    else:
        print(f"\n  IRIS: 未找到 IRIS regions, 跳过")
        return []

    region_list = list(regions.values()) if isinstance(regions, dict) else regions

    pairs_np = get_query_pairs_np()
    n_pairs = len(pairs_np)
    n_repeats = min(n_repeats, 2) if quick else n_repeats

    print(f"\n{'='*60}")
    print(f"  IRIS-GCS: {n_pairs} pairs × {n_repeats} repeats")
    print(f"  regions: {len(region_list)} ({list(regions.keys())})")
    print(f"{'='*60}")

    all_results = []

    for pi, (q_start, q_goal) in enumerate(pairs_np):
        print(f"\n  {PAIR_NAMES[pi]}:")

        def factory(rl=region_list):
            return iris_to_lineargcs(rl)

        res = solve_gcs_path(factory, q_start, q_goal,
                             n_repeats=n_repeats, verbose=False)

        n_ok = sum(res["successes"])
        for rep in range(n_repeats):
            ok = res["successes"][rep]
            st = res["times"][rep]
            pl = res["path_lengths"][rep]
            status = "OK" if ok else "FAIL"
            print(f"    rep={rep} {status}  solver={_fmt(st)}  len={pl:.3f}"
                  if ok else f"    rep={rep} {status}  solver={_fmt(st)}")

            all_results.append({
                "pair_idx": pi, "pair_name": PAIR_NAMES[pi],
                "rep": rep, "success": ok,
                "solver_time": st,
                "path_length": pl,
            })

    # Summary
    print(f"\n  IRIS-GCS Summary:")
    for pi in range(n_pairs):
        pr = [r for r in all_results if r["pair_idx"] == pi]
        if not pr:
            continue
        sr = sum(1 for r in pr if r["success"]) / len(pr)
        s_times = [r["solver_time"] for r in pr if r["success"]]
        s_lens = [r.get("path_length", float("inf")) for r in pr if r["success"]]
        med_t = f"{np.median(s_times)*1000:.1f}ms" if s_times else "N/A"
        med_l = f"{np.median(s_lens):.3f}" if s_lens else "N/A"
        print(f"  {PAIR_NAMES[pi]:8s}  SR={sr*100:.0f}%  "
              f"solver={med_t}  len={med_l}")

    return all_results


# ═══════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="Paper Exp 2: End-to-end GCS Planning")
    parser.add_argument("--seeds", type=int, default=N_SEEDS)
    parser.add_argument("--quick", action="store_true",
                        help="Quick mode: fewer seeds/repeats")
    parser.add_argument("--group", choices=["sbf", "iris"],
                        default=None, help="Run only one group")
    parser.add_argument("--output", type=str, default="output/raw")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(name)s %(message)s",
                        stream=sys.stdout)
    # force flush
    sys.stdout.reconfigure(line_buffering=True) if hasattr(sys.stdout, 'reconfigure') else None

    print("=" * 72)
    print("  Exp 2: End-to-end GCS Planning (SBF vs IRIS)")
    print("=" * 72)

    all_results = {}

    if args.group is None or args.group == "sbf":
        all_results["sbf_gcs"] = run_sbf_gcs(args.seeds, args.quick)

    if args.group is None or args.group == "iris":
        all_results["iris_gcs"] = run_iris_gcs(
            n_repeats=args.seeds, quick=args.quick)

    # Save
    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / "paper_exp2_planning.json"
    with open(out_path, "w") as f:
        json.dump(all_results, f, indent=2, default=str)
    print(f"\n  Results saved to {out_path}")
    print("\n  Exp 2 complete.")


if __name__ == "__main__":
    main()

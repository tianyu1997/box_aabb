"""
paper_exp1_coverage.py — 实验 1: Region 生成效率与自由空间覆盖

对应论文 Exp 1 (Table I + Fig. 2):
  3 个 Marcucci 场景 (shelves / bins / table)
  C-IRIS: 从 8 个 seed 点生成 region, 每场景只运行 1 次, 记录总耗时 + 总体积
  SBF: 生成 10000 个 box, 每 1000 个记录一次快照 (时间 + 体积), 重复 10 次取平均

  同时将生成的 region/forest 产物序列化到磁盘, 供实验 2 复用.

用法:
  python -m experiments.paper_exp1_coverage
  python -m experiments.paper_exp1_coverage --quick
  python -m experiments.paper_exp1_coverage --group baseline  # 只跑 C-IRIS
  python -m experiments.paper_exp1_coverage --group sbf       # 只跑 SBF
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
from marcucci_scenes import get_iiwa14_seed_points

logger = logging.getLogger(__name__)

EXPERIMENT_NAME = "paper_exp1_coverage"

# ═══════════════════════════════════════════════════════════════════════════
# Configuration
# ═══════════════════════════════════════════════════════════════════════════

# 蒙特卡洛覆盖率采样点数
N_MC_SAMPLES = 100_000

# 场景: 合并场景 (shelves + bins + table 全部障碍物, 与 prm_comparison notebook 一致)
SCENE_NAMES = ("combined",)

# SBF 参数
SBF_MAX_BOXES = 10000          # 总共生成 10000 个 box
SBF_SNAPSHOT_INTERVAL = 1000   # 每 1000 个 box 记录一次快照
SBF_N_REPEATS = 10             # 重复 10 次 (seed 0-9), 取平均

# C-IRIS 参数: 使用 8 个 seed 点 (start + goal + 6 random)
CIRIS_N_REGIONS = 8

# Planner 配置
SBF_CONFIG = {
    "type": "SBF", "method": "gcs", "name": "SBF-GCS",
    "strategy": "interval_fk",
}
CIRIS_CONFIG = {
    "type": "C-IRIS-GCS", "n_regions": CIRIS_N_REGIONS, "name": "C-IRIS",
    "max_iterations": 10,
}

# Quick 模式参数
QUICK_SBF_MAX_BOXES = 2000
QUICK_SBF_SNAPSHOT_INTERVAL = 1000
QUICK_SBF_N_REPEATS = 2
QUICK_CIRIS_N_REGIONS = 2

# 产物保存目录
ARTIFACTS_DIR = Path("output/exp1_artifacts")


# ═══════════════════════════════════════════════════════════════════════════
# Helper: format time
# ═══════════════════════════════════════════════════════════════════════════

def _fmt(seconds: float) -> str:
    if seconds < 1:
        return f"{seconds*1000:.1f}ms"
    if seconds < 60:
        return f"{seconds:.2f}s"
    m, s = divmod(int(seconds), 60)
    return f"{m}m{s:02d}s"


# ═══════════════════════════════════════════════════════════════════════════
# 缩小的规划关节空间 (基于 combined 场景 manual seeds + query pairs,
# margin=0.3 rad 或 20%·width, 与原始关节限取交)
# ═══════════════════════════════════════════════════════════════════════════

PLANNING_JOINT_LIMITS = [
    (-1.865488, 1.865691),   # j0  width=3.7312
    (-0.100000, 1.086648),   # j1  width=1.1866
    (-0.662656, 0.662338),   # j2  width=1.3250
    (-2.094400, -0.371673),  # j3  width=1.7227
    (-0.619251, 0.619534),   # j4  width=1.2388
    (-1.095222, 1.257951),   # j5  width=2.3532
    ( 1.050209, 2.091190),   # j6  width=1.0410
]


def apply_planning_limits(robot, planning_limits=None):
    """将缩小后的关节限制应用到 robot 对象."""
    if planning_limits is None:
        planning_limits = PLANNING_JOINT_LIMITS
    original = list(robot.joint_limits)
    robot.joint_limits = [(float(lo), float(hi)) for lo, hi in planning_limits]
    return original


# ═══════════════════════════════════════════════════════════════════════════
# Helpers: HPolyhedron geometry
# ═══════════════════════════════════════════════════════════════════════════

def _hpoly_bbox(hpoly) -> tuple:
    """Compute tight axis-aligned bounding box of HPolyhedron via LP.

    Solves 2*ndim linear programs: min/max each coordinate s.t. A@x<=b.
    Returns (lo, hi) ndarray pair.
    """
    from scipy.optimize import linprog
    A = np.asarray(hpoly.A())
    b_ub = np.asarray(hpoly.b())
    ndim = A.shape[1]
    lo = np.zeros(ndim)
    hi = np.zeros(ndim)
    for d in range(ndim):
        c = np.zeros(ndim)
        c[d] = 1.0
        res = linprog(c, A_ub=A, b_ub=b_ub, bounds=(None, None), method='highs')
        lo[d] = res.fun if res.success else -np.pi
        c[d] = -1.0
        res = linprog(c, A_ub=A, b_ub=b_ub, bounds=(None, None), method='highs')
        hi[d] = -res.fun if res.success else np.pi
    return lo, hi


def _points_in_any_region(points: np.ndarray, regions: list) -> np.ndarray:
    """Check which points lie in at least one region.

    Handles both box regions (min/max) and HPolyhedron regions (_hpoly).
    Returns boolean array of shape (N,).
    """
    N = len(points)
    in_any = np.zeros(N, dtype=bool)
    box_regions = [r for r in regions if r.get("_hpoly") is None]
    hpoly_regions = [r for r in regions if r.get("_hpoly") is not None]

    # ── Box regions: vectorized bbox check ──
    if box_regions:
        r_mins = np.array([r["min"] for r in box_regions], dtype=np.float64)
        r_maxs = np.array([r["max"] for r in box_regions], dtype=np.float64)
        BATCH = 2000
        for s in range(0, N, BATCH):
            batch = points[s:s + BATCH]
            inside = np.all(
                (batch[:, None, :] >= r_mins[None, :, :]) &
                (batch[:, None, :] <= r_maxs[None, :, :]),
                axis=2,
            )
            in_any[s:s + BATCH] |= inside.any(axis=1)

    # ── HPolyhedron regions: A @ x <= b, skip already-covered ──
    if hpoly_regions:
        remaining_mask = ~in_any
        if remaining_mask.any():
            remaining_idx = np.where(remaining_mask)[0]
            remaining_pts = points[remaining_idx]
            for reg in hpoly_regions:
                A = np.asarray(reg["_hpoly"].A())
                b_vec = np.asarray(reg["_hpoly"].b())
                inside_h = np.all(remaining_pts @ A.T <= b_vec[None, :],
                                  axis=1)
                in_any[remaining_idx[inside_h]] = True
                still = ~inside_h
                remaining_idx = remaining_idx[still]
                remaining_pts = remaining_pts[still]
                if len(remaining_idx) == 0:
                    break

    return in_any


# ═══════════════════════════════════════════════════════════════════════════
# Coverage estimation (保留, 与旧版兼容)
# ═══════════════════════════════════════════════════════════════════════════

def estimate_coverage_mc(robot, scene, regions, n_samples=N_MC_SAMPLES,
                         seed=42):
    """蒙特卡洛估计自由空间覆盖率 (向量化版本).

    Args:
        robot: Robot 实例
        scene: Scene 实例
        regions: list of boxes (each: {"min": ndarray, "max": ndarray})
        n_samples: 采样数
        seed: 随机种子

    Returns:
        dict: coverage_rate, n_free, n_covered, n_total, free_ratio
    """
    from forest.collision import CollisionChecker

    rng = np.random.default_rng(seed)
    jl = robot.joint_limits
    lows = np.array([lo for lo, _ in jl])
    highs = np.array([hi for _, hi in jl])
    ndim = len(jl)

    checker = CollisionChecker(robot=robot, scene=scene)

    samples = rng.uniform(lows, highs, size=(n_samples, ndim))
    collision_flags = checker.check_config_collision_batch(samples)
    free_mask = ~collision_flags
    n_free = int(free_mask.sum())

    n_covered = 0
    if n_free > 0 and regions:
        free_samples = samples[free_mask]
        in_any = _points_in_any_region(free_samples, regions)
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
    """从规划器提取 region boxes → list of {"min": ndarray, "max": ndarray}."""
    regions = []

    # SBFAdapter: 从 forest 提取 BoxNode boxes
    if hasattr(planner, '_prep') and planner._prep is not None:
        prep = planner._prep
        boxes_raw = prep.get("boxes") or prep.get("forest_boxes") or {}
        box_iter = (boxes_raw.values()
                    if isinstance(boxes_raw, dict) else boxes_raw)

        for b in box_iter:
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

    # IRIS-NP / C-IRIS: HPolyhedron → LP 计算精确 bbox
    if hasattr(planner, '_regions') and planner._regions:
        for r in planner._regions:
            if hasattr(r, 'A') and hasattr(r, 'b'):
                try:
                    lo, hi = _hpoly_bbox(r)
                    regions.append({
                        "min": lo,
                        "max": hi,
                        "_hpoly": r,
                    })
                except Exception:
                    pass

    return regions


def compute_region_volume(regions) -> float:
    """计算 region 总体积.

    SBF boxes: 精确 ∏(max_i - min_i)
    HPolyhedron: Monte Carlo 采样估计
    """
    total_vol = 0.0
    for reg in regions:
        lo = reg["min"]
        hi = reg["max"]
        # 如果保留了原始 HPolyhedron, 使用 MC 估计
        hpoly = reg.get("_hpoly")
        if hpoly is not None:
            vol = _hpoly_volume_mc(hpoly, lo, hi, n_samples=50000)
        else:
            vol = float(np.prod(hi - lo))
        total_vol += vol
    return total_vol


def _hpoly_volume_mc(hpoly, lo, hi, n_samples=50000) -> float:
    """用 MC 采样估计 HPolyhedron 在 bounding box 内的体积."""
    rng = np.random.default_rng(12345)
    ndim = len(lo)
    bbox_vol = float(np.prod(hi - lo))
    samples = rng.uniform(lo, hi, size=(n_samples, ndim))
    A = np.asarray(hpoly.A())
    b = np.asarray(hpoly.b())
    # point inside if A @ x <= b for all rows
    inside = np.all(samples @ A.T <= b[None, :], axis=1)
    return bbox_vol * inside.sum() / n_samples


def compute_union_volume_mc(robot, regions, n_samples=100_000,
                            seed=12345) -> float:
    """用 MC 采样估计 regions 的 union 体积 (去重叠).

    在整个 C-space 均匀采样, 统计落在任意 region 内的比例,
    乘以 C-space 体积即得 union 体积.
    """
    rng = np.random.default_rng(seed)
    jl = robot.joint_limits
    lows = np.array([lo for lo, _ in jl])
    highs = np.array([hi for _, hi in jl])
    ndim = len(jl)
    cspace_vol = float(np.prod(highs - lows))
    samples = rng.uniform(lows, highs, size=(n_samples, ndim))
    in_any = _points_in_any_region(samples, regions)
    hit_ratio = in_any.sum() / n_samples
    return cspace_vol * hit_ratio


# ═══════════════════════════════════════════════════════════════════════════
# C-IRIS 实验: 从 8 个 seed 点生成 region, 每场景只运行 1 次
# ═══════════════════════════════════════════════════════════════════════════

def run_ciris(scene_name: str, n_regions: int = CIRIS_N_REGIONS,
              quick_mode: bool = False) -> dict:
    """C-IRIS: 单场景单次运行.

    Returns:
        dict with n_regions, total_time, total_volume, coverage_rate, regions
    """
    print(f"\n  ── C-IRIS | {scene_name} | n_regions={n_regions} ──",
          flush=True)
    scene_cfg = load_marcucci_scene(scene_name, robot="iiwa14")
    robot, scene, query_pairs = load_scene_from_config(scene_cfg)
    q_start, q_goal = query_pairs[0]
    print(f"    [scene] {scene_name}: {len(scene_cfg['obstacles'])} obstacles, "
          f"{len(query_pairs)} query pairs", flush=True)
    print(f"    [scene] q_start={np.round(q_start, 3).tolist()}", flush=True)
    print(f"    [scene] q_goal ={np.round(q_goal, 3).tolist()}", flush=True)

    # ── 缩小规划关节空间 ──
    manual_seeds_dict = get_iiwa14_seed_points()
    manual_seeds_list = list(manual_seeds_dict.values())
    apply_planning_limits(robot)
    print(f"    [limits] reduced joint space:", flush=True)
    for d, (lo, hi) in enumerate(robot.joint_limits):
        print(f"      j{d}: [{lo:.3f}, {hi:.3f}]  width={hi-lo:.3f}",
              flush=True)

    cfg = dict(CIRIS_CONFIG)
    cfg["n_regions"] = n_regions
    if quick_mode:
        cfg["max_iterations"] = 3
    print(f"    [C-IRIS] config: n_regions={n_regions}, "
          f"max_iter={cfg.get('max_iterations', 10)}", flush=True)

    planner = create_planner(cfg)
    print(f"    [C-IRIS] manual seeds: {list(manual_seeds_dict.keys())}",
          flush=True)
    planner.setup(robot, scene, {
        "seed": 0,
        "manual_seeds": manual_seeds_list,
        "planning_limits": PLANNING_JOINT_LIMITS,
    })

    print(f"    [C-IRIS] generating {n_regions} regions ...", flush=True)
    t0 = time.perf_counter()
    result = planner.plan(q_start, q_goal, timeout=600.0)
    total_time = time.perf_counter() - t0
    print(f"    [C-IRIS] done in {_fmt(total_time)}, "
          f"success={result.success}", flush=True)
    if hasattr(result, 'phase_times') and result.phase_times:
        for k, v in result.phase_times.items():
            print(f"      phase {k}: {_fmt(v)}", flush=True)

    regions = extract_regions_from_planner(planner)
    sum_volume = compute_region_volume(regions)
    # volume MC 需要足够多的点 (7维 C-space 很大), quick 模式也不能太少
    mc_vol_samples = 50_000 if quick_mode else 200_000
    union_volume = compute_union_volume_mc(robot, regions,
                                           n_samples=mc_vol_samples)
    print(f"    [volume] sum={sum_volume:.4f}  union={union_volume:.4f}  "
          f"(overlap≈{max(sum_volume - union_volume, 0):.4f})", flush=True)

    # Coverage
    mc_samples = 500 if quick_mode else N_MC_SAMPLES
    print(f"    [MC] estimating coverage ({mc_samples} samples) ...",
          flush=True)
    cov = estimate_coverage_mc(robot, scene, regions,
                               n_samples=mc_samples, seed=42)
    print(f"    [MC] coverage={cov['coverage_rate']*100:.1f}%", flush=True)

    print(f"    [C-IRIS] {scene_name}: {len(regions)} regions, "
          f"time={_fmt(total_time)}, "
          f"sum_vol={sum_volume:.4f}, union_vol={union_volume:.4f}, "
          f"cov={cov['coverage_rate']*100:.1f}%", flush=True)

    return {
        "scene": scene_name,
        "planner": "C-IRIS",
        "n_regions": len(regions),
        "total_time": total_time,
        "total_volume": union_volume,     # 使用去重叠 union volume
        "sum_volume": sum_volume,         # 保留各 region 体积之和
        "coverage_rate": cov["coverage_rate"],
        "coverage_detail": cov,
        "regions": regions,
        "planner_obj": planner,  # 保留用于实验 2
    }


# ═══════════════════════════════════════════════════════════════════════════
# SBF 实验: 生成 10000 box, 每 1000 记录快照, 重复 10 次
# ═══════════════════════════════════════════════════════════════════════════

def run_sbf(scene_name: str,
            max_boxes: int = SBF_MAX_BOXES,
            snapshot_interval: int = SBF_SNAPSHOT_INTERVAL,
            n_repeats: int = SBF_N_REPEATS,
            quick_mode: bool = False) -> dict:
    """SBF: 多次重复, 记录 milestone 快照.

    Returns:
        dict with per-seed milestones, averaged summary, planner objects
    """
    if quick_mode:
        max_boxes = QUICK_SBF_MAX_BOXES
        snapshot_interval = QUICK_SBF_SNAPSHOT_INTERVAL
        n_repeats = QUICK_SBF_N_REPEATS

    print(f"\n  ── SBF | {scene_name} | max_boxes={max_boxes} × "
          f"{n_repeats} repeats ──", flush=True)
    scene_cfg = load_marcucci_scene(scene_name, robot="iiwa14")
    robot, scene, query_pairs = load_scene_from_config(scene_cfg)
    q_start, q_goal = query_pairs[0]

    # ── 缩小规划关节空间 ──
    manual_seeds_dict = get_iiwa14_seed_points()
    manual_seeds_list = list(manual_seeds_dict.values())
    apply_planning_limits(robot)
    print(f"    [limits] reduced joint space:", flush=True)
    for d, (lo, hi) in enumerate(robot.joint_limits):
        print(f"      j{d}: [{lo:.3f}, {hi:.3f}]  width={hi-lo:.3f}",
              flush=True)

    seed_results = []
    planners = []  # 保留 planner 对象供实验 2 复用

    for seed in range(n_repeats):
        print(f"\n    [SBF seed={seed}/{n_repeats-1}] growing {max_boxes} boxes ...",
              flush=True)

        cfg = dict(SBF_CONFIG)
        cfg["snapshot_interval"] = snapshot_interval
        planner = create_planner(cfg)
        if seed == 0:
            print(f"      manual extra_seeds: {list(manual_seeds_dict.keys())}",
                  flush=True)
        t_setup0 = time.perf_counter()
        planner.setup(robot, scene, {
            "seed": seed,
            "max_boxes": max_boxes,
            "max_consecutive_miss": 500,
            "extra_seeds": manual_seeds_list,
        })
        t_setup = time.perf_counter() - t_setup0
        print(f"      setup: {_fmt(t_setup)}", flush=True)

        t0 = time.perf_counter()
        result = planner.plan(q_start, q_goal, timeout=600.0)
        total_time = time.perf_counter() - t0

        regions = extract_regions_from_planner(planner)
        n_boxes = len(regions)
        total_volume = compute_region_volume(regions)

        # 提取 milestones
        milestones = []
        pt = result.phase_times or {}
        grow_detail = getattr(result, 'grow_detail', None)
        if hasattr(planner, '_prep') and planner._prep is not None:
            prep = planner._prep
            gd = prep.get('grow_detail', {})
            milestones = gd.get('milestones', [])

        # 如果没有 milestone 数据 (pipeline 未正确记录), 构造最终快照
        if not milestones:
            milestones = [{"n_boxes": n_boxes, "time": total_time,
                           "volume": total_volume}]

        print(f"      result: success={result.success}, {n_boxes} boxes, "
              f"time={_fmt(total_time)}, vol={total_volume:.4f}, "
              f"{len(milestones)} milestones", flush=True)
        if pt:
            phase_str = ", ".join(f"{k}={_fmt(v)}" for k, v in pt.items())
            print(f"      phases: {phase_str}", flush=True)
        for ms in milestones:
            print(f"      milestone: boxes={ms.get('n_boxes','-')} "
                  f"time={_fmt(ms.get('time',0))} "
                  f"vol={ms.get('volume',0):.4f}", flush=True)

        seed_results.append({
            "seed": seed,
            "n_boxes": n_boxes,
            "total_time": total_time,
            "total_volume": total_volume,
            "milestones": milestones,
        })
        planners.append(planner)

    # Coverage (只用第一个 seed 的 forest 估计, 避免重复计算)
    first_regions = extract_regions_from_planner(planners[0])
    mc_samples = 500 if quick_mode else N_MC_SAMPLES
    print(f"    [MC] estimating coverage ({mc_samples} samples) ...",
          flush=True)
    cov = estimate_coverage_mc(robot, scene, first_regions,
                               n_samples=mc_samples, seed=42)
    print(f"    [MC] coverage={cov['coverage_rate']*100:.1f}%", flush=True)

    # 计算平均值
    avg_time = np.mean([s["total_time"] for s in seed_results])
    avg_volume = np.mean([s["total_volume"] for s in seed_results])
    avg_n_boxes = np.mean([s["n_boxes"] for s in seed_results])
    std_time = np.std([s["total_time"] for s in seed_results])
    std_volume = np.std([s["total_volume"] for s in seed_results])

    print(f"    [SBF] {scene_name}: avg {avg_n_boxes:.0f} boxes, "
          f"time={avg_time:.2f}±{std_time:.2f}s, "
          f"vol={avg_volume:.4f}±{std_volume:.4f}, "
          f"cov={cov['coverage_rate']*100:.1f}%", flush=True)

    return {
        "scene": scene_name,
        "planner": "SBF-GCS",
        "n_repeats": n_repeats,
        "avg_n_boxes": float(avg_n_boxes),
        "avg_time": float(avg_time),
        "std_time": float(std_time),
        "avg_volume": float(avg_volume),
        "std_volume": float(std_volume),
        "coverage_rate": cov["coverage_rate"],
        "coverage_detail": cov,
        "seed_results": seed_results,
        "planners": planners,  # 保留用于实验 2
    }


# ═══════════════════════════════════════════════════════════════════════════
# 产物序列化 — 供实验 2 复用
# ═══════════════════════════════════════════════════════════════════════════

def save_artifacts(all_results: dict, output_dir: Path):
    """将 C-IRIS regions 和 SBF planner 对象保存到磁盘."""
    output_dir.mkdir(parents=True, exist_ok=True)

    for scene_name, data in all_results.items():
        # C-IRIS: 保存 regions (list of min/max)
        if "ciris" in data:
            ciris = data["ciris"]
            regions_serial = []
            for r in ciris.get("regions", []):
                regions_serial.append({
                    "min": r["min"].tolist(),
                    "max": r["max"].tolist(),
                })
            ciris_path = output_dir / f"ciris_{scene_name}.json"
            with open(ciris_path, 'w') as f:
                json.dump({
                    "scene": scene_name,
                    "n_regions": ciris["n_regions"],
                    "total_time": ciris["total_time"],
                    "total_volume": ciris["total_volume"],
                    "sum_volume": ciris.get("sum_volume",
                                            ciris["total_volume"]),
                    "regions": regions_serial,
                }, f, indent=2)
            # 保存 HPolyhedron A/b 矩阵到 JSON (避免 pickle Drake 对象)
            planner_obj = ciris.get("planner_obj")
            if planner_obj is not None and hasattr(planner_obj, '_regions'):
                hpoly_data = []
                for r in planner_obj._regions:
                    try:
                        hpoly_data.append({
                            'A': np.asarray(r.A()).tolist(),
                            'b': np.asarray(r.b()).tolist(),
                        })
                    except Exception:
                        pass
                if hpoly_data:
                    hpoly_path = output_dir / f"ciris_{scene_name}_hpoly.json"
                    with open(hpoly_path, 'w') as f:
                        json.dump({'scene': scene_name, 'hpolys': hpoly_data},
                                  f, indent=2)
                    logger.info("Saved %d HPolyhedra → %s",
                                len(hpoly_data), hpoly_path)
            # 用自定义 __getstate__ pickle (不含 Drake C++ 对象)
            if planner_obj is not None:
                pkl_path = output_dir / f"ciris_{scene_name}.pkl"
                try:
                    with open(pkl_path, 'wb') as f:
                        pickle.dump(planner_obj, f)
                    logger.info("Pickled C-IRIS planner → %s", pkl_path)
                except Exception as e:
                    logger.warning("Failed to pickle C-IRIS planner: %s", e)
            print(f"  [save] C-IRIS {scene_name} → {ciris_path}", flush=True)

        # SBF: 保存每个 seed 的 planner 对象
        if "sbf" in data:
            sbf = data["sbf"]
            planners = sbf.get("planners", [])
            for i, p in enumerate(planners):
                pkl_path = output_dir / f"sbf_{scene_name}_seed{i}.pkl"
                try:
                    with open(pkl_path, 'wb') as f:
                        pickle.dump(p, f)
                except Exception as e:
                    logger.warning("Failed to pickle SBF planner seed=%d: %s",
                                   i, e)
            # 保存元数据
            meta_path = output_dir / f"sbf_{scene_name}.json"
            meta = {k: v for k, v in sbf.items()
                    if k not in ("planners", "coverage_detail")}
            # Convert numpy types
            for k, v in meta.items():
                if isinstance(v, (np.floating, np.integer)):
                    meta[k] = float(v)
                elif isinstance(v, list):
                    meta[k] = [
                        {kk: float(vv) if isinstance(vv, (np.floating, np.integer)) else vv
                         for kk, vv in item.items()} if isinstance(item, dict) else item
                        for item in v
                    ]
            with open(meta_path, 'w') as f:
                json.dump(meta, f, indent=2, default=str)
            print(f"  [save] SBF {scene_name} → {meta_path} "
                  f"({len(planners)} planners)", flush=True)


# ═══════════════════════════════════════════════════════════════════════════
# Main experiment
# ═══════════════════════════════════════════════════════════════════════════

def run_experiment(planner_group: str | None = None,
                   quick_mode: bool = False,
                   output_dir: str = "output/raw"):
    """运行完整 Exp 1."""
    t_exp_start = time.perf_counter()
    results = ExperimentResults(experiment_name=EXPERIMENT_NAME)
    all_results = {}

    run_sbf_flag = planner_group is None or planner_group == "sbf"
    run_ciris_flag = planner_group is None or planner_group == "baseline"

    print(f"\n{'='*65}", flush=True)
    print(f"Exp 1: Region 生成效率  |  "
          f"{len(SCENE_NAMES)} scenes  |  "
          f"SBF={'ON' if run_sbf_flag else 'OFF'}  "
          f"C-IRIS={'ON' if run_ciris_flag else 'OFF'}", flush=True)
    if quick_mode:
        print(f"  [QUICK MODE] SBF: {QUICK_SBF_MAX_BOXES} boxes × "
              f"{QUICK_SBF_N_REPEATS} repeats, "
              f"C-IRIS: {QUICK_CIRIS_N_REGIONS} regions", flush=True)
    else:
        print(f"  SBF: {SBF_MAX_BOXES} boxes × {SBF_N_REPEATS} repeats, "
              f"C-IRIS: {CIRIS_N_REGIONS} regions", flush=True)
    print(f"{'='*65}", flush=True)

    for si, scene_name in enumerate(SCENE_NAMES):
        print(f"\n{'─'*65}", flush=True)
        print(f"  SCENE [{si+1}/{len(SCENE_NAMES)}]: {scene_name}", flush=True)
        print(f"{'─'*65}", flush=True)

        scene_data = {}

        # ── C-IRIS ──
        if run_ciris_flag:
            n_reg = QUICK_CIRIS_N_REGIONS if quick_mode else CIRIS_N_REGIONS
            ciris_result = run_ciris(scene_name, n_regions=n_reg,
                                     quick_mode=quick_mode)
            scene_data["ciris"] = ciris_result

            results.add(SingleTrialResult(
                scene_name=scene_name,
                planner_name="C-IRIS",
                seed=0, trial=0,
                result={
                    "n_regions": ciris_result["n_regions"],
                    "total_time": ciris_result["total_time"],
                    "total_volume": ciris_result["total_volume"],
                    "coverage_rate": ciris_result["coverage_rate"],
                    "certified": True,
                },
                wall_clock=ciris_result["total_time"],
            ))

        # ── SBF ──
        if run_sbf_flag:
            sbf_result = run_sbf(scene_name, quick_mode=quick_mode)
            scene_data["sbf"] = sbf_result

            # 记录每个 seed 的结果
            for sr in sbf_result["seed_results"]:
                results.add(SingleTrialResult(
                    scene_name=scene_name,
                    planner_name="SBF-GCS",
                    seed=sr["seed"], trial=0,
                    result={
                        "n_regions": sr["n_boxes"],
                        "total_time": sr["total_time"],
                        "total_volume": sr["total_volume"],
                        "milestones": sr["milestones"],
                        "certified": True,
                    },
                    wall_clock=sr["total_time"],
                ))

        all_results[scene_name] = scene_data

    # ── 保存产物 ──
    save_artifacts(all_results, ARTIFACTS_DIR)

    # ── 保存实验结果 ──
    results.metadata = {
        "sbf_max_boxes": SBF_MAX_BOXES,
        "sbf_snapshot_interval": SBF_SNAPSHOT_INTERVAL,
        "sbf_n_repeats": SBF_N_REPEATS,
        "ciris_n_regions": CIRIS_N_REGIONS,
        "n_mc_samples": N_MC_SAMPLES,
        "scenes": SCENE_NAMES,
        "planner_group": planner_group,
        "quick_mode": quick_mode,
        "timestamp": time.strftime("%Y%m%d_%H%M%S"),
    }

    suffix = f"_{planner_group}" if planner_group else ""
    out = Path(output_dir) / f"{EXPERIMENT_NAME}{suffix}.json"
    results.save(out)

    # ── Print summary ──
    t_total = time.perf_counter() - t_exp_start
    print(f"\n{'='*65}", flush=True)
    print(f"Exp 1 complete: {len(results.trials)} records  "
          f"total={_fmt(t_total)}", flush=True)
    print(f"{'='*65}", flush=True)

    print("\n── Summary ──")
    for scene_name in SCENE_NAMES:
        sd = all_results.get(scene_name, {})
        if "ciris" in sd:
            c = sd["ciris"]
            sv = c.get('sum_volume', c['total_volume'])
            print(f"  {scene_name:10s} | C-IRIS   | "
                  f"regions={c['n_regions']:3d} | "
                  f"time={c['total_time']:7.2f}s | "
                  f"union_vol={c['total_volume']:.4f} sum_vol={sv:.4f} | "
                  f"cov={c['coverage_rate']*100:.1f}%")
        if "sbf" in sd:
            s = sd["sbf"]
            print(f"  {scene_name:10s} | SBF-GCS  | "
                  f"boxes={s['avg_n_boxes']:5.0f} | "
                  f"time={s['avg_time']:7.2f}±{s['std_time']:.2f}s | "
                  f"vol={s['avg_volume']:.4f}±{s['std_volume']:.4f} | "
                  f"cov={s['coverage_rate']*100:.1f}%")

    return results


def main():
    parser = argparse.ArgumentParser(
        description="Paper Exp 1: Region Coverage Efficiency")
    parser.add_argument("--output", type=str, default="output/raw")
    parser.add_argument("--quick", action="store_true",
                        help="Quick mode: 2000 boxes × 2 repeats, 2 IRIS regions")
    parser.add_argument("--mc-samples", type=int, default=100_000,
                        help="Monte Carlo coverage samples")
    parser.add_argument("--group", choices=["baseline", "sbf"],
                        default=None,
                        help="只运行指定分组: baseline (C-IRIS) 或 sbf")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(levelname)s] %(message)s")

    global N_MC_SAMPLES
    N_MC_SAMPLES = args.mc_samples

    quick_mode = args.quick
    if quick_mode:
        N_MC_SAMPLES = 500
        print("[quick mode] SBF: 2000 boxes × 2 repeats, "
              "C-IRIS: 2 regions, MC=500", flush=True)

    run_experiment(
        planner_group=args.group,
        quick_mode=quick_mode,
        output_dir=args.output,
    )


if __name__ == "__main__":
    main()

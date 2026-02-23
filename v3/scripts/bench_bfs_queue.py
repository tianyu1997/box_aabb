#!/usr/bin/env python
"""
scripts/bench_bfs_queue.py �?全局 BFS deque 队列 vs �?expand_target 对比基准

比较两种边缘扩展策略:
  A) single_target: 每次新增 box 替换唯一 expand_target (�?
  B) global_bfs:    所有新 box 入全局 deque 队列 (�?

测量指标:
  - 生长耗时 (ms)
  - box 数量
  - coarsen �?box �?
  - island �?
  - C-space 覆盖�?(�?2D)
  - 路径 cost (Dijkstra geodesic)

用法:
    cd v3
    python scripts/bench_bfs_queue.py [--seeds 42 100 7] [--n_trials 3]
"""
from __future__ import annotations

import argparse
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

_ROOT = Path(__file__).resolve().parents[1]  # v3/
_SRC = _ROOT / "src"
for p in (_ROOT, _SRC):
    sp = str(p)
    if sp not in sys.path:
        sys.path.insert(0, sp)

from aabb.robot import load_robot
from forest.scene import Scene
from forest.collision import CollisionChecker
from forest.models import BoxNode
from forest.connectivity import find_islands, bridge_islands, UnionFind
from forest.safe_box_forest import SafeBoxForest
from forest.hier_aabb_tree import HierAABBTree
from forest.coarsen import coarsen_forest
from planner.sbf_planner import SBFPlanner
from planner.models import SBFConfig, gmean_edge_length
from planner.defaults import TWODOF_DEFAULTS, PANDA_DEFAULTS
from planner.pipeline import (
    PandaGCSConfig, make_planner_config,
    _build_adjacency_and_islands, find_box_containing,
    build_panda_scene, grow_forest,
)

# ══════════════════════════════════════════════════════════════════════════�?
# Shared utilities
# ══════════════════════════════════════════════════════════════════════════�?

def build_random_2d_scene(robot, q_start, q_goal, rng, n_obstacles=8,
                          max_trials=300):
    import math as _math
    jl = robot.joint_limits
    spans = [hi - lo for lo, hi in jl]
    span0 = spans[0] if spans else 0
    all_same = all(abs(s - span0) < 1e-6 for s in spans)
    period = float(span0) if (all_same and abs(span0 - 2 * _math.pi) < 0.1) else None

    for _ in range(max_trials):
        scene = Scene()
        for i in range(n_obstacles):
            cx = float(rng.uniform(-1.8, 1.8))
            cy = float(rng.uniform(-1.8, 1.8))
            w = float(rng.uniform(0.3, 0.8))
            h = float(rng.uniform(0.3, 0.8))
            scene.add_obstacle([cx - w/2, cy - h/2],
                               [cx + w/2, cy + h/2], name=f"obs_{i}")
        checker = CollisionChecker(robot=robot, scene=scene)
        if checker.check_config_collision(q_start):
            continue
        if checker.check_config_collision(q_goal):
            continue
        # reject if direct Euclidean path is free
        if not checker.check_segment_collision(q_start, q_goal, 0.03):
            continue
        # also reject if geodesic wrap-around path is free
        if period is not None and not checker.check_segment_collision(
                q_start, q_goal, 0.03, period=period):
            continue
        return scene
    raise RuntimeError("Cannot build scene")


def compute_coverage_2d(forest_boxes, joint_limits, resolution=0.05):
    """计算 2D C-space �?box 覆盖占关节空间的比例."""
    lo0, hi0 = joint_limits[0]
    lo1, hi1 = joint_limits[1]
    total_area = (hi0 - lo0) * (hi1 - lo1)
    box_area = 0.0
    for b in forest_boxes.values():
        w = b.joint_intervals[0][1] - b.joint_intervals[0][0]
        h = b.joint_intervals[1][1] - b.joint_intervals[1][0]
        box_area += max(w, 0) * max(h, 0)
    return box_area / total_area if total_area > 0 else 0.0


# ══════════════════════════════════════════════════════════════════════════�?
# Forest growth: two strategies
# ══════════════════════════════════════════════════════════════════════════�?

def _try_add(planner, forest, q, obs_packed, ndim):
    """Common box creation logic. Returns (nid, n_absorbed) or (-1, 0)."""
    if planner.hier_tree.is_occupied(q):
        return -1, 0
    nid = forest.allocate_id()
    ffb = planner.hier_tree.find_free_box(
        q, planner.obstacles, mark_occupied=True,
        forest_box_id=nid, obs_packed=obs_packed)
    if ffb is None:
        return -1, 0
    vol = 1.0
    for lo, hi in ffb.intervals:
        vol *= max(hi - lo, 0)
    box = BoxNode(node_id=nid, joint_intervals=ffb.intervals,
                  seed_config=q.copy(), volume=vol)
    n_abs = 0
    if ffb.absorbed_box_ids:
        n_abs = len(ffb.absorbed_box_ids)
        forest.remove_boxes(ffb.absorbed_box_ids)
    forest.add_box_direct(box)
    return nid, n_abs


def grow_single_target(planner, q_start, q_goal, seed, max_miss, max_boxes):
    """策略 A: �?expand_target �?每次新增 box 替换唯一目标."""
    ndim = len(planner.joint_limits)
    rng = np.random.default_rng(seed)
    forest = planner._load_or_create_forest()
    forest.hier_tree = planner.hier_tree
    obs_packed = planner.hier_tree._prepack_obstacles_c(planner.obstacles)

    lows = np.array([lo for lo, _ in planner.joint_limits])
    highs = np.array([hi for _, hi in planner.joint_limits])

    # seed anchors
    expand_queue: deque = deque()  # (box, excluded_faces)
    for qs in [q_start, q_goal]:
        nid, _ = _try_add(planner, forest, qs, obs_packed, ndim)
        if nid >= 0:
            expand_queue.append((forest.boxes[nid], frozenset()))

    consec = 0

    while consec < max_miss:
        if forest.n_boxes >= max_boxes:
            break

        if expand_queue:
            box, excluded = expand_queue.popleft()
            seeds_list = planner._generate_boundary_seeds(
                box, rng, excluded_faces=excluded)
            for dim, side, q_seed in seeds_list:
                if forest.n_boxes >= max_boxes:
                    break
                nid, _ = _try_add(planner, forest, q_seed, obs_packed, ndim)
                if nid >= 0:
                    consec = 0
                    new_excluded = excluded | frozenset({(dim, 1 - side)})
                    expand_queue.append(
                        (forest.boxes[nid], new_excluded))
        else:
            roll = rng.uniform()
            if roll < 0.6:
                try:
                    q = planner.hier_tree.sample_unoccupied_seed(rng)
                except ValueError:
                    q = None
                if q is None:
                    q = rng.uniform(lows, highs)
            else:
                q = rng.uniform(lows, highs)

            if planner.collision_checker.check_config_collision(q):
                consec += 1
                continue

            nid, _ = _try_add(planner, forest, q, obs_packed, ndim)
            if nid >= 0:
                consec = 0
                expand_queue.append((forest.boxes[nid], frozenset()))
            else:
                consec += 1

    return forest


def grow_global_bfs(planner, q_start, q_goal, seed, max_miss, max_boxes):
    """策略 B: 全局 BFS deque — 所有 box 入队, 系统地扩展全部前沿 (同策略 A)."""
    # 现在两种策略相同：都使用全方向单向 BFS
    return grow_single_target(planner, q_start, q_goal, seed, max_miss, max_boxes)


# ══════════════════════════════════════════════════════════════════════════�?
# Evaluation
# ══════════════════════════════════════════════════════════════════════════�?

@dataclass
class RunResult:
    strategy: str = ""
    env: str = ""
    seed: int = 0
    grow_ms: float = 0.0
    n_boxes: int = 0
    n_boxes_coarsened: int = 0
    n_islands: int = 0
    coverage: float = 0.0
    path_cost: float = float('inf')
    path_found: bool = False


def evaluate_forest(forest, planner, q_start, q_goal, env_name, coarsen_rounds=20):
    """�?forest �?coarsen + island + Dijkstra 评估."""
    import heapq

    jl = planner.robot.joint_limits[0]
    period = float(jl[1] - jl[0])
    half = period / 2.0

    # coarsen
    cs = coarsen_forest(planner.hier_tree, forest, max_rounds=coarsen_rounds)
    n_coarsened = len(forest.boxes)

    # islands
    adj, uf, islands = _build_adjacency_and_islands(forest.boxes, period=period)
    n_islands = len(islands)

    # coverage (2D only)
    coverage = 0.0
    if env_name == "2dof":
        coverage = compute_coverage_2d(forest.boxes, planner.joint_limits)

    # Dijkstra
    boxes = forest.boxes
    src = find_box_containing(q_start, boxes)
    tgt = find_box_containing(q_goal, boxes)
    path_cost = float('inf')
    path_found = False

    if src is not None and tgt is not None:
        centers = {}
        for bid, box in boxes.items():
            centers[bid] = np.array(
                [(lo + hi) / 2 for lo, hi in box.joint_intervals])

        dist_map = {bid: float('inf') for bid in boxes}
        dist_map[src] = 0.0
        heap = [(0.0, src)]
        while heap:
            d, u = heapq.heappop(heap)
            if d > dist_map[u]:
                continue
            if u == tgt:
                break
            cu = centers[u]
            for v in adj.get(u, set()):
                diff = ((centers[v] - cu) + half) % period - half
                w = float(np.linalg.norm(diff))
                nd = d + w
                if nd < dist_map[v]:
                    dist_map[v] = nd
                    heapq.heappush(heap, (nd, v))

        if dist_map[tgt] < float('inf'):
            path_cost = dist_map[tgt]
            path_found = True

    return n_coarsened, n_islands, coverage, path_cost, path_found


# ══════════════════════════════════════════════════════════════════════════�?
# Benchmark runners
# ══════════════════════════════════════════════════════════════════════════�?

def run_2dof_bench(seeds, n_obstacles=8):
    """2DOF 平面机器人基�?"""
    q_start = np.array([0.8 * np.pi, 0.2])
    q_goal = np.array([-0.7 * np.pi, -0.4])
    D = TWODOF_DEFAULTS
    max_miss = D.max_consecutive_miss
    max_boxes = D.max_boxes

    results = []

    for seed in seeds:
        rng = np.random.default_rng(seed)
        robot = load_robot("2dof_planar")
        scene = build_random_2d_scene(robot, q_start, q_goal, rng, n_obstacles)

        for strategy_name, grow_fn in [("single_target", grow_single_target),
                                        ("global_bfs", grow_global_bfs)]:
            # 重建 planner (每次 fresh)
            sbf_config = SBFConfig(
                max_iterations=999999,
                max_box_nodes=999999,
                ffb_min_edge=D.ffb_min_edge,
                guided_sample_ratio=D.guided_sample_ratio,
                boundary_expand_epsilon=D.boundary_expand_epsilon,
            )
            planner = SBFPlanner(robot=robot, scene=scene, config=sbf_config,
                                 no_cache=True)

            t0 = time.perf_counter()
            forest = grow_fn(planner, q_start, q_goal, seed,
                             max_miss, max_boxes)
            grow_ms = (time.perf_counter() - t0) * 1000
            n_boxes_raw = len(forest.boxes)

            n_coarsened, n_islands, coverage, path_cost, path_found = \
                evaluate_forest(forest, planner, q_start, q_goal, "2dof")

            r = RunResult(
                strategy=strategy_name, env="2dof", seed=seed,
                grow_ms=grow_ms, n_boxes=n_boxes_raw,
                n_boxes_coarsened=n_coarsened, n_islands=n_islands,
                coverage=coverage, path_cost=path_cost,
                path_found=path_found,
            )
            results.append(r)
            print(f"  2dof seed={seed:>4d} {strategy_name:>14s}: "
                  f"{grow_ms:7.0f}ms  {n_boxes_raw:>4d}→{n_coarsened:>4d} boxes  "
                  f"{n_islands} isl  cov={coverage:.3f}  "
                  f"{'OK' if path_found else 'FAIL'} cost={path_cost:.3f}")

    return results


def run_panda_bench(seeds, n_obstacles=6):
    """Panda 7-DOF 基准."""
    results = []

    for seed in seeds:
        rng = np.random.default_rng(seed)
        cfg = PandaGCSConfig()
        cfg.seed = seed
        cfg.n_obstacles = n_obstacles

        robot = load_robot("panda")
        q_start = np.array(cfg.q_start, dtype=np.float64)
        q_goal = np.array(cfg.q_goal, dtype=np.float64)
        scene = build_panda_scene(rng, cfg, robot=robot,
                                  q_start=q_start, q_goal=q_goal)

        max_miss = cfg.max_consecutive_miss
        max_boxes = cfg.max_boxes

        for strategy_name, grow_fn in [("single_target", grow_single_target),
                                        ("global_bfs", grow_global_bfs)]:
            sbf_config = make_planner_config(cfg)
            planner = SBFPlanner(robot=robot, scene=scene, config=sbf_config,
                                 no_cache=True)

            t0 = time.perf_counter()
            forest = grow_fn(planner, q_start, q_goal, seed,
                             max_miss, max_boxes)
            grow_ms = (time.perf_counter() - t0) * 1000
            n_boxes_raw = len(forest.boxes)

            n_coarsened, n_islands, coverage, path_cost, path_found = \
                evaluate_forest(forest, planner, q_start, q_goal, "panda")

            r = RunResult(
                strategy=strategy_name, env="panda", seed=seed,
                grow_ms=grow_ms, n_boxes=n_boxes_raw,
                n_boxes_coarsened=n_coarsened, n_islands=n_islands,
                coverage=coverage, path_cost=path_cost,
                path_found=path_found,
            )
            results.append(r)
            print(f"  panda seed={seed:>4d} {strategy_name:>14s}: "
                  f"{grow_ms:7.0f}ms  {n_boxes_raw:>4d}→{n_coarsened:>4d} boxes  "
                  f"{n_islands} isl  "
                  f"{'OK' if path_found else 'FAIL'} cost={path_cost:.3f}")

    return results


# ══════════════════════════════════════════════════════════════════════════�?
# Summary
# ══════════════════════════════════════════════════════════════════════════�?

def print_summary(results: List[RunResult]):
    from collections import defaultdict
    groups = defaultdict(list)
    for r in results:
        groups[(r.env, r.strategy)].append(r)

    print("\n" + "=" * 80)
    print("  Summary: single_target vs global_bfs")
    print("=" * 80)

    envs = sorted(set(r.env for r in results))
    for env in envs:
        st = groups.get((env, "single_target"), [])
        bfs = groups.get((env, "global_bfs"), [])
        if not st or not bfs:
            continue

        def avg(lst, attr):
            vals = [getattr(r, attr) for r in lst]
            return sum(vals) / len(vals) if vals else 0.0

        print(f"\n  [{env}] ({len(st)} trials)")
        print(f"  {'Metric':<25s} {'single_target':>14s} {'global_bfs':>14s} {'delta':>10s}")
        print(f"  {'-'*25} {'-'*14} {'-'*14} {'-'*10}")

        for attr, fmt, label in [
            ("grow_ms", ".0f", "Grow time (ms)"),
            ("n_boxes", ".0f", "Boxes (raw)"),
            ("n_boxes_coarsened", ".0f", "Boxes (coarsened)"),
            ("n_islands", ".1f", "Islands"),
            ("coverage", ".4f", "Coverage"),
            ("path_cost", ".3f", "Path cost"),
        ]:
            a = avg(st, attr)
            b = avg(bfs, attr)
            if a != 0:
                delta = f"{(b - a) / a * 100:+.1f}%"
            else:
                delta = "N/A"
            print(f"  {label:<25s} {a:>14{fmt}} {b:>14{fmt}} {delta:>10s}")

        sr_st = sum(1 for r in st if r.path_found) / len(st) * 100
        sr_bfs = sum(1 for r in bfs if r.path_found) / len(bfs) * 100
        print(f"  {'Path success rate':<25s} {sr_st:>13.0f}% {sr_bfs:>13.0f}%")

    print()


# ══════════════════════════════════════════════════════════════════════════�?
# Main
# ══════════════════════════════════════════════════════════════════════════�?

def main():
    parser = argparse.ArgumentParser(
        description="BFS queue vs single target benchmark")
    parser.add_argument("--seeds", type=int, nargs="+",
                        default=[42, 100, 7])
    parser.add_argument("--skip-panda", action="store_true",
                        help="跳过 Panda 7-DOF 测试")
    parser.add_argument("--skip-2d", action="store_true",
                        help="跳过 2DOF 测试")
    parser.add_argument("--obstacles-2d", type=int, default=8)
    parser.add_argument("--obstacles-panda", type=int, default=6)
    args = parser.parse_args()

    print("=" * 60)
    print("  BFS Queue Benchmark: single_target vs global_bfs")
    print(f"  Seeds: {args.seeds}")
    print("=" * 60)

    all_results = []

    if not args.skip_2d:
        print("\n[2DOF Planar]")
        r2d = run_2dof_bench(args.seeds, args.obstacles_2d)
        all_results.extend(r2d)

    if not args.skip_panda:
        print("\n[Panda 7-DOF]")
        rpanda = run_panda_bench(args.seeds, args.obstacles_panda)
        all_results.extend(rpanda)

    print_summary(all_results)


if __name__ == "__main__":
    main()

"""
_test_post_process.py — 批量测试 pre-process vs post-process 路径质量

对比各阶段 (transition wp → pull-tight → SOCP → geometric shortcut) 的 cost,
判断 post-process 是否仍有降低成本的效果.

用法:
    cd v3
    python _test_post_process.py [--mode 2dof|panda|both] [--n 30]
"""
from __future__ import annotations

import sys
import time
import traceback
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parent
_SRC = _ROOT / "src"
for p in (_ROOT, _SRC):
    sp = str(p)
    if sp not in sys.path:
        sys.path.insert(0, sp)

from aabb.robot import load_robot
from forest.scene import Scene
from forest.collision import CollisionChecker
from planner.pipeline import (
    PandaGCSConfig,
    build_panda_scene,
    grow_and_prepare,
    _build_adjacency_and_islands,
    find_box_containing,
    _dijkstra_box_graph,
    _shortcut_box_sequence,
    _build_transition_waypoints,
    _pull_tight_in_bounds,
    _refine_path_in_boxes,
    _geometric_shortcut,
)
from forest.connectivity import bridge_islands


# ═══════════════════════════════════════════════════════════════════
# Cost helpers
# ═══════════════════════════════════════════════════════════════════

def _geodesic_cost(waypoints, period):
    half = period / 2.0
    cost = 0.0
    for k in range(len(waypoints) - 1):
        diff = ((waypoints[k + 1] - waypoints[k]) + half) % period - half
        cost += float(np.linalg.norm(diff))
    return cost


def _euclidean_cost(waypoints):
    cost = 0.0
    for k in range(len(waypoints) - 1):
        cost += float(np.linalg.norm(waypoints[k + 1] - waypoints[k]))
    return cost


# ═══════════════════════════════════════════════════════════════════
# 2DOF scene builder
# ═══════════════════════════════════════════════════════════════════

def build_random_2d_scene(robot, q_start, q_goal, rng, n_obstacles=8):
    import math as _math
    jl = robot.joint_limits
    spans = [hi - lo for lo, hi in jl]
    span0 = spans[0] if spans else 0
    all_same = all(abs(s - span0) < 1e-6 for s in spans)
    period = float(span0) if (all_same and abs(span0 - 2 * _math.pi) < 0.1) else None

    for _ in range(300):
        scene = Scene()
        for i in range(n_obstacles):
            cx = float(rng.uniform(-1.8, 1.8))
            cy = float(rng.uniform(-1.8, 1.8))
            w = float(rng.uniform(0.3, 0.8))
            h = float(rng.uniform(0.3, 0.8))
            scene.add_obstacle([cx - w / 2, cy - h / 2],
                               [cx + w / 2, cy + h / 2], name=f"obs_{i}")
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
    raise RuntimeError("no valid scene")


# ═══════════════════════════════════════════════════════════════════
# Single trial (decomposed pipeline)
# ═══════════════════════════════════════════════════════════════════

def run_trial(mode, seed, verbose=False):
    """Run one trial, return dict with costs at each stage."""
    rng = np.random.default_rng(seed)

    if mode == "2dof":
        robot = load_robot("2dof_planar")
        ndim = 2
        q_start = np.array([0.8 * np.pi, 0.2])
        q_goal = np.array([-0.7 * np.pi, -0.4])
        scene = build_random_2d_scene(robot, q_start, q_goal, rng, 8)
    else:
        robot = load_robot("panda")
        ndim = robot.n_joints
        cfg_p = PandaGCSConfig()
        cfg_p.seed = seed
        q_start = np.array(cfg_p.q_start, dtype=np.float64)
        q_goal = np.array(cfg_p.q_goal, dtype=np.float64)
        scene = build_panda_scene(rng, cfg_p, robot, q_start, q_goal)

    cfg = PandaGCSConfig()
    cfg.seed = seed
    if mode == "2dof":
        cfg.max_boxes = 200
        cfg.max_consecutive_miss = 30
    prep = grow_and_prepare(robot, scene, cfg, q_start, q_goal, ndim,
                            no_cache=True)

    # wait for cache thread if any
    ct = prep.get('_cache_thread')
    if ct is not None:
        ct.join()

    boxes = prep['boxes']
    planner = prep['planner']
    forest_obj = prep['forest_obj']
    period = getattr(planner, '_period', None) or getattr(forest_obj, 'period', None)
    cost_fn = (lambda wps: _geodesic_cost(wps, period)) if period else _euclidean_cost

    adj, uf, islands = _build_adjacency_and_islands(boxes, period=period)
    src = find_box_containing(q_start, boxes)
    tgt = find_box_containing(q_goal, boxes)
    if src is None or tgt is None:
        return None

    # bridge if needed
    bridge_edge_map = {}
    if not uf.same(src, tgt):
        bridge_result = bridge_islands(
            boxes=boxes,
            collision_checker=planner.collision_checker,
            segment_resolution=0.03,
            max_pairs_per_island_pair=10,
            max_rounds=5,
            period=period,
            hier_tree=planner.hier_tree,
            obstacles=planner.obstacles,
            forest=forest_obj,
            n_bridge_seeds=7,
            min_island_size=cfg.min_island_size,
            precomputed_uf=uf,
            precomputed_islands=islands,
            target_pair=(src, tgt),
        )
        bridge_edges_res = bridge_result[0]
        for e in bridge_edges_res:
            s_bid = find_box_containing(e.source_config, boxes)
            t_bid = find_box_containing(e.target_config, boxes)
            if s_bid is not None and t_bid is not None:
                adj.setdefault(s_bid, set()).add(t_bid)
                adj.setdefault(t_bid, set()).add(s_bid)
                bridge_edge_map[(s_bid, t_bid)] = e
                bridge_edge_map[(t_bid, s_bid)] = e
        # refresh src/tgt
        boxes = forest_obj.boxes
        if src not in boxes:
            src = find_box_containing(q_start, boxes)
        if tgt not in boxes:
            tgt = find_box_containing(q_goal, boxes)
        if src is None or tgt is None:
            return None

    # Dijkstra
    box_seq, raw_dist = _dijkstra_box_graph(
        boxes, adj, src, tgt, period=period, q_goal=q_goal)
    if box_seq is None:
        return None

    short_seq = _shortcut_box_sequence(box_seq, adj)

    # Stage 1: transition waypoints
    waypoints, wp_bounds = _build_transition_waypoints(
        short_seq, boxes, q_start, q_goal,
        period=period, bridge_edge_map=bridge_edge_map)
    cost_raw_wp = cost_fn(waypoints)

    # Stage 2: pull-tight
    tight_wps = _pull_tight_in_bounds(
        waypoints, wp_bounds, period=period, n_iters=30)
    cost_pull_tight = cost_fn(tight_wps)

    # Stage 3: SOCP refine
    socp_wps, socp_cost = _refine_path_in_boxes(
        tight_wps, wp_bounds, q_start, q_goal, ndim, period=period)
    cost_socp = cost_fn(socp_wps) if len(socp_wps) > 0 else cost_pull_tight

    # Stage 4: geometric shortcut
    geo_wps, geo_cost = _geometric_shortcut(socp_wps, boxes, short_seq,
                                            period=period)
    cost_geo = cost_fn(geo_wps) if len(geo_wps) > 0 else cost_socp

    result = dict(
        seed=seed,
        n_boxes=len(boxes),
        n_raw_boxes=len(box_seq),
        n_short_boxes=len(short_seq),
        n_wp_raw=len(waypoints),
        n_wp_tight=len(tight_wps),
        n_wp_socp=len(socp_wps),
        n_wp_geo=len(geo_wps),
        cost_raw_wp=cost_raw_wp,
        cost_pull_tight=cost_pull_tight,
        cost_socp=cost_socp,
        cost_geo=cost_geo,
        has_bridge=len(bridge_edge_map) > 0,
    )

    if verbose:
        print(f"  seed={seed:4d}  boxes={len(box_seq):3d}→{len(short_seq):2d}  "
              f"wp={len(waypoints):2d}→{len(tight_wps):2d}→{len(socp_wps):2d}→{len(geo_wps):2d}  "
              f"cost: raw={cost_raw_wp:.3f} tight={cost_pull_tight:.3f} "
              f"socp={cost_socp:.3f} geo={cost_geo:.3f}  "
              f"bridge={'Y' if bridge_edge_map else 'N'}")

    return result


# ═══════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["2dof", "panda", "both"],
                        default="2dof")
    parser.add_argument("--n", type=int, default=30,
                        help="Number of seeds to test")
    parser.add_argument("--start-seed", type=int, default=1)
    args = parser.parse_args()

    modes = []
    if args.mode in ("2dof", "both"):
        modes.append("2dof")
    if args.mode in ("panda", "both"):
        modes.append("panda")

    for mode in modes:
        print(f"\n{'='*70}")
        print(f"  Post-process effectiveness: {mode.upper()}")
        print(f"  Seeds: {args.start_seed} .. {args.start_seed + args.n - 1}")
        print(f"{'='*70}\n")

        results = []
        n_fail = 0

        for seed in range(args.start_seed, args.start_seed + args.n):
            try:
                r = run_trial(mode, seed, verbose=True)
            except Exception as e:
                print(f"  seed={seed:4d}  ERROR: {e}")
                n_fail += 1
                continue
            if r is None:
                print(f"  seed={seed:4d}  SKIP (no path)")
                n_fail += 1
                continue
            results.append(r)

        if not results:
            print("\n  No successful trials.\n")
            continue

        # ── Summary ──
        print(f"\n{'─'*70}")
        print(f"  SUMMARY ({mode.upper()}) — {len(results)} / "
              f"{len(results) + n_fail} seeds successful\n")

        keys = ['cost_raw_wp', 'cost_pull_tight', 'cost_socp', 'cost_geo']
        labels = ['Raw WP', 'Pull-tight', 'SOCP', 'Geo shortcut']
        avgs = {}
        for k in keys:
            vals = [r[k] for r in results]
            avgs[k] = np.mean(vals)

        print(f"  {'Stage':<16s} {'Avg Cost':>10s}  {'vs Raw':>8s}  {'vs Prev':>8s}")
        print(f"  {'─'*50}")
        prev = avgs['cost_raw_wp']
        for k, label in zip(keys, labels):
            vs_raw = (avgs[k] / avgs['cost_raw_wp'] - 1) * 100
            vs_prev = (avgs[k] / prev - 1) * 100 if prev > 0 else 0.0
            print(f"  {label:<16s} {avgs[k]:10.4f}  {vs_raw:+7.1f}%  {vs_prev:+7.1f}%")
            prev = avgs[k]

        n_tight_improve = sum(
            1 for r in results if r['cost_pull_tight'] < r['cost_raw_wp'] - 1e-6)
        n_socp_improve = sum(
            1 for r in results if r['cost_socp'] < r['cost_pull_tight'] - 1e-6)
        n_geo_improve = sum(
            1 for r in results if r['cost_geo'] < r['cost_socp'] - 1e-6)
        n_any_post = sum(
            1 for r in results if r['cost_geo'] < r['cost_pull_tight'] - 1e-6)

        print(f"\n  Pull-tight improved vs raw:     {n_tight_improve:3d} / {len(results)}")
        print(f"  SOCP improved vs pull-tight:    {n_socp_improve:3d} / {len(results)}")
        print(f"  Geo-shortcut improved vs SOCP:  {n_geo_improve:3d} / {len(results)}")
        print(f"  Any post-process improved:      {n_any_post:3d} / {len(results)}")

        # per-trial table
        print(f"\n  {'seed':>5s} {'boxes':>6s} {'wp':>8s} "
              f"{'raw':>8s} {'tight':>8s} {'socp':>8s} {'geo':>8s} "
              f"{'Δ%':>7s} {'br':>3s}")
        print(f"  {'─'*65}")
        for r in results:
            delta_pct = ((r['cost_geo'] / r['cost_raw_wp'] - 1) * 100
                         if r['cost_raw_wp'] > 0 else 0.0)
            print(f"  {r['seed']:5d} {r['n_raw_boxes']:3d}→{r['n_short_boxes']:2d} "
                  f"{r['n_wp_raw']:2d}→{r['n_wp_geo']:2d} "
                  f"{r['cost_raw_wp']:8.3f} {r['cost_pull_tight']:8.3f} "
                  f"{r['cost_socp']:8.3f} {r['cost_geo']:8.3f} "
                  f"{delta_pct:+6.1f}% "
                  f"{'Y' if r['has_bridge'] else 'N':>3s}")


if __name__ == "__main__":
    main()

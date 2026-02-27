"""
baselines/sbf_adapter.py — SafeBoxForest 管线适配器

将 pipeline.py 的 grow_and_prepare / run_method_with_bridge 封装为
BasePlanner 接口, 支持跨查询复用 (supports_reuse=True).
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Optional

import numpy as np

from .base import BasePlanner, PlanningResult

from forest.collision import CollisionChecker
from planner.pipeline import (
    PandaGCSConfig,
    make_planner_config,
    grow_and_prepare,
    run_method_with_bridge,
    run_method_visgraph,
    _solve_method_dijkstra,
    _solve_method_gcs,
)


class SBFAdapter(BasePlanner):
    """SafeBoxForest pipeline adapter.

    默认使用 Dijkstra + SOCP refine 方法.
    ``supports_reuse=True``: setup 构建 forest 后, 多次 plan()
    复用同一 forest / adjacency.
    """

    def __init__(self, method: str = "dijkstra"):
        """
        Args:
            method: 'dijkstra' | 'gcs' | 'visgraph'
        """
        self._method = method
        self._robot = None
        self._scene = None
        self._cfg: Optional[PandaGCSConfig] = None
        self._prep: Optional[dict] = None
        self._checker: Optional[CollisionChecker] = None
        self._ndim: int = 0

    # ── BasePlanner 接口 ───────────────────────────────────────

    @property
    def name(self) -> str:
        return f"SBF-{self._method.capitalize()}"

    @property
    def supports_reuse(self) -> bool:
        return True

    def setup(self, robot, scene, config: dict) -> None:
        self._robot = robot
        self._scene = scene
        self._ndim = robot.n_joints

        # build PandaGCSConfig from dict
        cfg = PandaGCSConfig()
        for k, v in config.items():
            if hasattr(cfg, k):
                setattr(cfg, k, v)
        self._cfg = cfg

        self._checker = CollisionChecker(robot=robot, scene=scene)
        self._prep = None  # lazy: first plan() will build

    def plan(self, start: np.ndarray, goal: np.ndarray,
             timeout: float = 30.0) -> PlanningResult:
        t_total = time.perf_counter()
        q_start = np.asarray(start, dtype=np.float64)
        q_goal = np.asarray(goal, dtype=np.float64)
        cache_wait_s = 0.0

        # ── 1) grow forest (first call only, or after reset) ──
        phase_times = {}
        if self._prep is None:
            t0 = time.perf_counter()
            self._prep = grow_and_prepare(
                self._robot, self._scene, self._cfg,
                q_start, q_goal, self._ndim,
                no_cache=self._cfg.no_cache if hasattr(self._cfg, 'no_cache') else False,
            )
            # cache thread handling (optional blocking)
            ct = self._prep.get('_cache_thread')
            if ct is not None:
                if bool(getattr(self._cfg, 'wait_cache_thread', False)):
                    t_wait = time.perf_counter()
                    ct.join()
                    cache_wait_s += (time.perf_counter() - t_wait)
                elif not ct.is_alive():
                    ct.join(timeout=0.0)
                cr = self._prep.get('_cache_result', {})
                self._prep['cache_ms'] = cr.get('ms', 0.0)
            phase_times['grow'] = time.perf_counter() - t0

        # ── 2) solve ──
        t0 = time.perf_counter()
        if self._method == "visgraph":
            raw = run_method_visgraph(
                self._prep, self._cfg,
                q_start, q_goal, self._checker, self._ndim)
        else:
            fn = (_solve_method_gcs if self._method == "gcs"
                  else _solve_method_dijkstra)
            raw = run_method_with_bridge(
                fn, self._method.capitalize(),
                self._prep, self._cfg,
                q_start, q_goal, self._ndim,
                seed=self._cfg.seed)
        phase_times['solve'] = time.perf_counter() - t0

        # ── 3) convert to PlanningResult ──
        if raw is None or not raw.get('success', False):
            total_s = time.perf_counter() - t_total
            return PlanningResult.failure(
                planning_time=total_s,
                nodes_explored=len(self._prep.get('boxes', {})),
                method=self._method,
            )

        wps = raw.get('waypoints', [])
        path = np.array(wps, dtype=np.float64) if wps else None
        cost = raw.get('cost', float('inf'))

        # ── Lightweight geodesic repair (pipeline already handles most cases) ──
        # The pipeline's _ensure_geodesic_safe + _greedy_shortcut now
        # handle wrapping segments.  We only do a fast check here and,
        # if any residual collisions remain, run a light subdivision +
        # shortcut pass.
        t_repair = 0.0
        if path is not None and len(path) >= 2 and self._period is not None:
            t0_repair = time.perf_counter()
            path, cost = self._repair_path_geodesic(path)
            t_repair = time.perf_counter() - t0_repair
            phase_times['repair'] = t_repair
        elif path is not None and len(path) >= 2 and self._period is None:
            # Euclidean repair for non-periodic robots (e.g. Panda)
            t0_repair = time.perf_counter()
            path, cost = self._repair_path_euclidean(path)
            t_repair = time.perf_counter() - t0_repair
            phase_times['repair'] = t_repair

        # ── 3.5) strict final safety gate: successful result must be collision-free ──
        if path is not None and len(path) >= 2:
            verify_res = 0.02
            has_collision = any(
                self._checker.check_segment_collision(
                    path[i], path[i + 1], verify_res, period=self._period)
                for i in range(len(path) - 1)
            )
            if has_collision:
                total_s = time.perf_counter() - t_total
                return PlanningResult.failure(
                    planning_time=total_s,
                    nodes_explored=len(self._prep.get('boxes', {})),
                    method=self._method,
                )

        # Non-blocking cache result harvest on return path
        ct = self._prep.get('_cache_thread')
        if ct is not None and (not ct.is_alive()):
            ct.join(timeout=0.0)
            cr = self._prep.get('_cache_result', {})
            self._prep['cache_ms'] = cr.get('ms', 0.0)

        total_s = time.perf_counter() - t_total

        return PlanningResult(
            success=True,
            path=path,
            cost=cost,
            planning_time=total_s,
            first_solution_time=total_s,  # SBF 不是 anytime
            collision_checks=0,           # not tracked at pipeline level
            nodes_explored=len(self._prep.get('boxes', {})),
            phase_times=phase_times,
            metadata={
                'method': self._method,
                'n_grown': self._prep.get('n_grown', 0),
                'grow_ms': self._prep.get('grow_ms', 0),
                'coarsen_ms': self._prep.get('coarsen_ms', 0),
                'cache_start_ms': self._prep.get('cache_start_ms', 0),
                'box_export_ms': self._prep.get('box_export_ms', 0),
                'prepare_misc_ms': self._prep.get('prepare_misc_ms', 0),
                'growprep_total_ms': self._prep.get('growprep_total_ms', 0),
                'adj_ms': raw.get('adj_ms', 0),
                'bridge_ms': raw.get('bridge_ms', 0),
                'plan_ms': raw.get('plan_ms', 0),
                'post_safe_ms': raw.get('post_safe_ms', 0),
                'plan_graph_ms': raw.get('plan_graph_ms', 0),
                'plan_waypoints_ms': raw.get('plan_waypoints_ms', 0),
                'plan_refine_ms': raw.get('plan_refine_ms', 0),
                'plan_shortcut_ms': raw.get('plan_shortcut_ms', 0),
                'repair_ms': t_repair * 1000,
                'cache_wait_ms': cache_wait_s * 1000,
                'grow_detail': self._prep.get('grow_detail', {}),
            },
        )

    # ── Euclidean path repair (non-periodic, e.g. Panda) ────────

    def _repair_path_euclidean(self, path: np.ndarray):
        """Check & repair colliding segments for non-periodic robots.

        Strategy (important for high-dim like Panda 7-DOF):
        1. First, try greedy shortcut — often bypasses colliding segments
           by connecting directly past them (collision-free).
        2. Only if shortcutted path still has collisions, run subdivision
           repair + shortcut again.
        3. Always keep the shorter of {original, repaired} to guard against
           random perturbation creating long detours in high-dim space.

        Returns (repaired_path, cost).
        """
        checker = self._checker
        resolution = 0.02
        from planner.pipeline import _greedy_shortcut

        def _euc_cost(wps):
            return sum(
                float(np.linalg.norm(wps[i + 1] - wps[i]))
                for i in range(len(wps) - 1)
            )

        def _has_collision(wps, res):
            return any(
                checker.check_segment_collision(wps[i], wps[i + 1], res)
                for i in range(len(wps) - 1)
            )

        wps = [path[i].copy() for i in range(len(path))]
        raw_cost = _euc_cost(wps)

        # ── Step 1: greedy shortcut (often resolves collisions by bypass) ──
        shortcut_wps = _greedy_shortcut(wps, checker, resolution)
        shortcut_cost = _euc_cost(shortcut_wps)

        # Check if shortcutted path still has collisions
        n_colliding = sum(
            1 for i in range(len(shortcut_wps) - 1)
            if checker.check_segment_collision(
                shortcut_wps[i], shortcut_wps[i + 1], resolution)
        )

        if n_colliding == 0:
            # Shortcut alone fixed everything
            best_wps = shortcut_wps
            best_cost = shortcut_cost
        else:
            # ── Step 2: subdivision repair on original path, then shortcut ──
            repaired = self._repair_segments_euclidean(
                wps, checker, resolution, max_depth=8)
            repaired = _greedy_shortcut(repaired, checker, resolution)
            repair_cost = _euc_cost(repaired)

            # Keep whichever is shorter
            if repair_cost < shortcut_cost:
                best_wps = repaired
                best_cost = repair_cost
            else:
                best_wps = shortcut_wps
                best_cost = shortcut_cost

        raw_has_collision = _has_collision(wps, resolution)

        # Guard: only fallback to raw when raw is also collision-free.
        # Safety has priority over path length.
        if (best_cost > raw_cost + 1e-9) and (not raw_has_collision):
            best_arr = path
            best_cost = raw_cost
        else:
            best_arr = np.array(best_wps, dtype=np.float64)

        # Final safety cleanup: if still colliding, keep repairing with
        # increasing budget until clean or budget exhausted.
        if _has_collision(best_arr, resolution):
            staged = [
                (0.02, 8, 30),
                (0.02, 10, 80),
                (0.01, 12, 120),
            ]
            curr = [best_arr[i].copy() for i in range(len(best_arr))]
            for res, depth, n_pert in staged:
                if not _has_collision(curr, res):
                    break
                curr = self._repair_segments_euclidean(
                    curr, checker, res,
                    max_perturb=n_pert, max_depth=depth)
                curr = _greedy_shortcut(curr, checker, res)
            best_arr = np.array(curr, dtype=np.float64)
            best_cost = _euc_cost(curr)

        return best_arr, best_cost

    @staticmethod
    def _repair_segments_euclidean(
        path: list, checker, resolution: float,
        max_perturb: int = 30, max_depth: int = 8,
    ) -> list:
        """Fix colliding segments by Euclidean midpoint subdivision.

        For each colliding segment (q_a, q_b):
        1. Compute Euclidean midpoint. If safe AND both halves pass,
           accept.
        2. If midpoint is in collision, try random perturbations.
        3. Recurse on sub-segments up to max_depth.
        """
        rng = np.random.default_rng(42)

        def _find_safe_mid(q1, q2, depth):
            mid = 0.5 * (q1 + q2)
            dist = float(np.linalg.norm(q2 - q1))
            ndim = len(q1)
            if not checker.check_config_collision(mid):
                return mid
            # random perturbations near midpoint
            spread = max(dist * 0.3, 0.1)
            for _ in range(max_perturb):
                offset = rng.normal(0, spread, size=ndim)
                candidate = mid + offset
                if checker.check_config_collision(candidate):
                    continue
                d1 = float(np.linalg.norm(candidate - q1))
                d2 = float(np.linalg.norm(q2 - candidate))
                if d1 + d2 < dist * 4.0:
                    return candidate
            return mid  # fallback

        def _repair_seg(q1, q2, depth):
            if depth <= 0:
                return [q2]
            if not checker.check_segment_collision(q1, q2, resolution):
                return [q2]
            via = _find_safe_mid(q1, q2, depth)
            left = _repair_seg(q1, via, depth - 1)
            right = _repair_seg(via, q2, depth - 1)
            return left + right

        result = [path[0]]
        for i in range(len(path) - 1):
            result.extend(_repair_seg(path[i], path[i + 1], max_depth))
        return result

    # ── Geodesic path repair ──────────────────────────────────────

    @property
    def _period(self) -> float | None:
        if self._robot is None:
            return None
        jl = self._robot.joint_limits
        spans = [hi - lo for lo, hi in jl]
        span = spans[0]
        all_same = all(abs(s - span) < 1e-6 for s in spans)
        return float(span) if all_same and abs(span - 2 * np.pi) < 0.1 else None

    def _repair_path_geodesic(self, path: np.ndarray):
        """Geodesic repair + optimization for periodic joints.

        Strategy (analogous to Euclidean repair):
        1. Run greedy shortcut with period — finds wrapping shortcuts
        2. Only if still colliding, run subdivision repair + shortcut
        3. Never return worse than raw input

        Returns (repaired_path, geodesic_cost).
        """
        period = self._period
        half = period / 2.0
        checker = self._checker
        verify_resolution = 0.02
        from planner.pipeline import _greedy_shortcut

        def _geo_cost(wps):
            total = 0.0
            for i in range(len(wps) - 1):
                diff = ((wps[i + 1] - wps[i]) + half) % period - half
                total += float(np.linalg.norm(diff))
            return total

        def _has_collision(wps, res):
            return any(
                checker.check_segment_collision(
                    wps[i], wps[i + 1], res, period=period)
                for i in range(len(wps) - 1)
            )

        wps = [path[i].copy() for i in range(len(path))]
        raw_cost = _geo_cost(wps)

        # ── Step 1: greedy shortcut with period ──
        shortcut_wps = _greedy_shortcut(
            wps, checker, verify_resolution, period=period)
        shortcut_cost = _geo_cost(shortcut_wps)

        # Check if shortcutted path still has collisions
        n_colliding = sum(
            1 for i in range(len(shortcut_wps) - 1)
            if checker.check_segment_collision(
                shortcut_wps[i], shortcut_wps[i + 1],
                verify_resolution, period=period)
        )

        if n_colliding == 0:
            best_wps = shortcut_wps
            best_cost = shortcut_cost
        else:
            # ── Step 2: subdivision repair on original, then shortcut ──
            repaired = self._repair_segments(
                [p.copy() for p in wps], checker, period,
                verify_resolution, max_perturb=30, max_depth=8)
            repaired = _greedy_shortcut(
                repaired, checker, verify_resolution, period=period)
            repair_cost = _geo_cost(repaired)

            if repair_cost < shortcut_cost:
                best_wps = repaired
                best_cost = repair_cost
            else:
                best_wps = shortcut_wps
                best_cost = shortcut_cost

        raw_has_collision = _has_collision(wps, verify_resolution)

        # Guard: only fallback to raw when raw is also collision-free.
        # Safety has priority over path length.
        if (best_cost > raw_cost + 1e-9) and (not raw_has_collision):
            return path, raw_cost

        # Final safety cleanup with increasing budgets.
        curr = [p.copy() for p in best_wps]
        if _has_collision(curr, verify_resolution):
            staged = [
                (0.02, 8, 30),
                (0.02, 10, 80),
                (0.01, 12, 120),
            ]
            for res, depth, n_pert in staged:
                if not _has_collision(curr, res):
                    break
                curr = self._repair_segments(
                    curr, checker, period, res,
                    max_perturb=n_pert, max_depth=depth)
                curr = _greedy_shortcut(curr, checker, res, period=period)

        return np.array(curr, dtype=np.float64), _geo_cost(curr)

    @staticmethod
    def _repair_segments(
        path: list, checker, period: float, resolution: float,
        max_perturb: int = 30, max_depth: int = 8,
    ) -> list:
        """Fix colliding segments by geodesic subdivision + perturbation.

        For each colliding segment (q_a, q_b):
        1. Try geodesic midpoint. If the midpoint config is safe AND
           both halves pass segment check, accept.
        2. If midpoint itself is in collision, try random perturbations
           near the midpoint to find a collision-free detour.
        3. Recurse on the sub-segments.

        This handles both thin obstacles (normal subdivision) and thick
        obstacles (perturbation finds a route around).
        """
        half = period / 2.0
        rng = np.random.default_rng(42)

        def _geo_mid(q1, q2):
            diff = ((q2 - q1) + half) % period - half
            m = q1 + 0.5 * diff
            return ((m + half) % period) - half

        def _normalize(q):
            return ((q + half) % period) - half

        def _geo_dist(q1, q2):
            diff = ((q2 - q1) + half) % period - half
            return float(np.linalg.norm(diff))

        def _find_safe_detour(q1, q2, depth):
            """Find a safe intermediate point between q1 and q2."""
            mid = _geo_mid(q1, q2)
            ndim = len(q1)
            diff = ((q2 - q1) + half) % period - half
            dist = float(np.linalg.norm(diff))

            # If geodesic midpoint is safe, use it
            if not checker.check_config_collision(mid):
                return mid

            # Midpoint is in collision — try structured + random perturbations
            # Build perpendicular direction(s) for structured search
            if dist > 1e-6:
                direction = diff / dist
                # For 2D: single perpendicular
                perp = np.array([-direction[1], direction[0]])
                perps = [perp, -perp]
            else:
                perps = []

            # Structured: try perpendicular offsets at various distances
            for p in perps:
                for scale in [0.1, 0.2, 0.3, 0.5, 0.8, 1.0]:
                    candidate = _normalize(mid + scale * p)
                    if not checker.check_config_collision(candidate):
                        d1 = _geo_dist(q1, candidate)
                        d2 = _geo_dist(candidate, q2)
                        if d1 + d2 < dist * 4.0:
                            return candidate

            # Random perturbations
            spread = max(dist * 0.5, 0.2)
            for _ in range(max_perturb):
                offset = rng.normal(0, spread, size=ndim)
                candidate = _normalize(mid + offset)
                if checker.check_config_collision(candidate):
                    continue
                d1 = _geo_dist(q1, candidate)
                d2 = _geo_dist(candidate, q2)
                if d1 + d2 < dist * 4.0:
                    return candidate

            # Fallback: try points along the segment near the safe endpoints
            for alpha in [0.05, 0.1, 0.15, 0.2, 0.85, 0.9, 0.95]:
                pt = _normalize(q1 + alpha * diff)
                if not checker.check_config_collision(pt):
                    return pt

            return mid  # give up, return colliding midpoint

        def _repair_seg(q1, q2, depth):
            """Recursively repair a segment."""
            if depth <= 0:
                return [q2]
            if not checker.check_segment_collision(
                    q1, q2, resolution, period=period):
                return [q2]

            # Find a safe intermediate point
            via = _find_safe_detour(q1, q2, depth)

            # Recurse on both halves
            left = _repair_seg(q1, via, depth - 1)
            right = _repair_seg(via, q2, depth - 1)
            return left + right

        result = [path[0]]
        for i in range(len(path) - 1):
            result.extend(_repair_seg(path[i], path[i + 1], max_depth))
        return result

    def reset(self) -> None:
        """清除 forest — 下次 plan() 将重新 grow."""
        self._prep = None
    def update_scene_incremental(
        self,
        new_scene,
        added_obstacles: list = None,
        removed_obstacle_names: list = None,
        regrow_budget: int = 60,
        rng=None,
    ) -> dict:
        """增量更新障碍物, 复用已有 forest.

        Args:
            new_scene: 更新后的 Scene (仅用于引用, 实际修改由 pipeline 完成)
            added_obstacles: [{'min_point': [...], 'max_point': [...], 'name': ...}]
            removed_obstacle_names: ['obs_name', ...]
            regrow_budget: 补种预算
            rng: 随机数生成器

        Returns:
            增量更新统计信息 dict
        """
        from planner.pipeline import incremental_obstacle_update

        if self._prep is None:
            raise RuntimeError("必须先调用 plan() 构建 forest 才能增量更新")

        return incremental_obstacle_update(
            prep=self._prep,
            scene=self._scene,
            added_obstacles=added_obstacles or [],
            removed_obstacle_names=removed_obstacle_names or [],
            regrow_budget=regrow_budget,
            rng=rng,
        )
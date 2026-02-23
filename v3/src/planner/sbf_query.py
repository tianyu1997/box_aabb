"""planner/sbf_query.py - fast planning query on existing SafeBoxForest."""

from __future__ import annotations

import time
from typing import Optional

import numpy as np

from .models import SBFConfig, SBFResult
from forest.models import BoxNode
from forest.safe_box_forest import SafeBoxForest
from forest.scene import Scene
from forest.collision import CollisionChecker
from .path_smoother import PathSmoother, compute_path_length
from .gcs_optimizer import GCSOptimizer
from forest.connectivity import _nearest_point_wrapped


def _geodesic_dist(
    a: np.ndarray, b: np.ndarray, period: Optional[float],
) -> float:
    """环面上两个配置的 L2 距离。period=None 退化为欧氏距离。"""
    if period is None:
        return float(np.linalg.norm(b - a))
    half = period / 2.0
    diff = ((b - a) + half) % period - half
    return float(np.linalg.norm(diff))


class SBFQuery:
    """Query planner using an existing SafeBoxForest without global rebuild."""

    def __init__(
        self,
        forest: SafeBoxForest,
        robot,
        scene: Scene,
        config: Optional[SBFConfig] = None,
    ) -> None:
        self.forest = forest
        self.robot = robot
        self.scene = scene
        self.config = config or SBFConfig()
        self.collision_checker = CollisionChecker(robot=robot, scene=scene)
        self.path_smoother = PathSmoother(
            collision_checker=self.collision_checker,
            segment_resolution=self.config.segment_collision_resolution,
        )
        self.gcs_optimizer = GCSOptimizer(fallback=True, bezier_degree=self.config.gcs_bezier_degree)

        # 计算周期 (与 SBFPlanner 一致)
        jl = robot.joint_limits
        spans = [hi - lo for lo, hi in jl]
        span = spans[0]
        all_same = all(abs(s - span) < 1e-6 for s in spans)
        self._period: Optional[float] = (
            float(span) if all_same and abs(span - 2 * np.pi) < 0.1
            else None
        )

    def plan(self, q_start: np.ndarray, q_goal: np.ndarray, seed: Optional[int] = None) -> SBFResult:
        t0 = time.time()
        q_start = np.asarray(q_start, dtype=np.float64)
        q_goal = np.asarray(q_goal, dtype=np.float64)
        result = SBFResult()

        if self.collision_checker.check_config_collision(q_start):
            result.message = "起始配置存在碰撞"
            result.computation_time = time.time() - t0
            return result
        if self.collision_checker.check_config_collision(q_goal):
            result.message = "目标配置存在碰撞"
            result.computation_time = time.time() - t0
            return result

        if not self.collision_checker.check_segment_collision(
            q_start, q_goal, self.config.segment_collision_resolution,
            period=self._period,
        ):
            result.success = True
            result.path = [q_start.copy(), q_goal.copy()]
            if self._period is not None:
                _half = self._period / 2.0
                _diff = ((q_goal - q_start) + _half) % self._period - _half
                result.path_length = float(np.linalg.norm(_diff))
            else:
                result.path_length = float(np.linalg.norm(q_goal - q_start))
            result.message = "直连成功"
            result.computation_time = time.time() - t0
            return result

        start_box = self.forest.find_containing(q_start) or self.forest.find_nearest(q_start)
        goal_box = self.forest.find_containing(q_goal) or self.forest.find_nearest(q_goal)
        if start_box is None or goal_box is None:
            result.message = "无法在 forest 中定位始末点"
            result.computation_time = time.time() - t0
            return result

        # fallback path: nearest-point waypoint sequence via Dijkstra on forest adjacency
        d_start = _geodesic_dist(q_start, start_box.center, self._period)
        d_goal = _geodesic_dist(q_goal, goal_box.center, self._period)
        graph = {
            "start": "start",
            "goal": "goal",
            "edges": {
                "start": [(start_box.node_id, d_start, None)],
                "goal": [],
            },
        }
        for bid, neighbors in self.forest.adjacency.items():
            graph["edges"].setdefault(bid, [])
            for nb in neighbors:
                if nb in self.forest.boxes:
                    box_u = self.forest.boxes[bid]
                    box_v = self.forest.boxes[nb]
                    # 边权: 两个 box 中心的测地距离
                    w = max(_geodesic_dist(box_u.center, box_v.center, self._period), 1e-12)
                    graph["edges"][bid].append((nb, w, None))
        graph["edges"].setdefault(goal_box.node_id, []).append(("goal", d_goal, None))

        path = self.gcs_optimizer._optimize_fallback(
            graph,
            self.forest.boxes,
            q_start,
            q_goal,
            period=self._period,
        )
        if not path:
            result.message = "forest 查询未找到路径"
            result.computation_time = time.time() - t0
            return result

        path = self.path_smoother.shortcut(path, max_iters=self.config.path_shortcut_iters)
        path = self.path_smoother.smooth_moving_average(path)

        result.success = True
        result.path = path
        result.path_length = compute_path_length(path, period=self._period)
        result.n_boxes_created = self.forest.n_boxes
        result.n_collision_checks = self.collision_checker.n_collision_checks
        result.message = "forest 查询成功"
        result.computation_time = time.time() - t0
        return result

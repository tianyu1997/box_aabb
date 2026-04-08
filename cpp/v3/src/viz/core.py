"""
src/viz/core.py — 2D ForestGrower 可视化核心数据类

提供: GrowVizConfig, Obstacle2D, CSpace2D, BoxInfo
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np


# ═══════════════════════════════════════════════════════════════════════════
#  Config
# ═══════════════════════════════════════════════════════════════════════════

@dataclass
class GrowVizConfig:
    """ForestGrower 2D 可视化配置, 参数名与 C++ GrowerConfig 对齐."""

    seed: int = 42

    # C-space bounds
    q_lo: Tuple[float, float] = (-3.14159, -3.14159)
    q_hi: Tuple[float, float] = ( 3.14159,  3.14159)

    # 端点
    q_start: Optional[List[float]] = field(
        default_factory=lambda: [0.8 * 3.14159, 0.2])
    q_goal: Optional[List[float]] = field(
        default_factory=lambda: [-0.7 * 3.14159, -0.4])

    # 障碍物
    n_obstacles: int = 10
    obs_cx_range: Tuple[float, float] = (-2.5, 2.5)
    obs_cy_range: Tuple[float, float] = (-2.5, 2.5)
    obs_w_range: Tuple[float, float] = (0.4, 1.2)
    obs_h_range: Tuple[float, float] = (0.4, 1.2)

    # ForestGrower 参数 ─ 与 grower_config.h 对齐
    mode: str = "wavefront"        # "wavefront" or "rrt"
    n_roots: int = 3
    max_boxes: int = 300
    max_consecutive_miss: int = 150
    min_edge: float = 0.03         # FFB 最小边长
    max_depth: int = 20            # FFB 最大递归深度

    # wavefront
    n_boundary_samples: int = 6
    boundary_epsilon: float = 0.02
    goal_face_bias: float = 0.6

    # rrt
    rrt_step_ratio: float = 0.15
    rrt_goal_bias: float = 0.1

    # adaptive min_edge (two-phase, 对齐 C++ grower_config.h)
    adaptive_min_edge: bool = False
    coarse_min_edge: float = 0.25    # Phase 1 粗粒度 (比 fine 0.03 大8倍)
    coarse_fraction: float = 0.5     # 前 50% boxes 用粗 min_edge

    # multi-thread 模拟 (round-robin per subtree)
    n_threads: int = 1               # 1 = 单线程, >1 = 模拟多线程 round-robin

    # coarsen (box merging, 对齐 C++ coarsen_forest + coarsen_greedy)
    coarsen_enabled: bool = False
    coarsen_max_rounds: int = 20       # dimension-scan rounds
    coarsen_greedy_rounds: int = 100   # greedy merge rounds
    coarsen_target_boxes: int = 0      # 0 = no target
    coarsen_score_threshold: float = 50.0  # hull_vol/sum_vol 上限

    # 可视化
    snapshot_every: int = 1        # 每 N 个 box 截一帧 (1=每个 box 都记录)
    collision_resolution: float = 0.025
    dpi: int = 150
    gif_frame_ms: int = 200
    fig_size: Tuple[float, float] = (10, 8)


# ═══════════════════════════════════════════════════════════════════════════
#  2D C-space 碰撞模型
# ═══════════════════════════════════════════════════════════════════════════

@dataclass
class Obstacle2D:
    lo: np.ndarray   # (2,)
    hi: np.ndarray   # (2,)

    def overlaps(self, box_lo: np.ndarray, box_hi: np.ndarray) -> bool:
        """AABB 重叠检测."""
        return bool(np.all(box_lo < self.hi) and np.all(self.lo < box_hi))


class CSpace2D:
    """2D C-space: 轴对齐矩形障碍物."""

    def __init__(self, q_lo, q_hi, obstacles: List[Obstacle2D]):
        self.q_lo = np.asarray(q_lo, dtype=np.float64)
        self.q_hi = np.asarray(q_hi, dtype=np.float64)
        self.obstacles = obstacles

    def is_collision(self, q: np.ndarray) -> bool:
        for obs in self.obstacles:
            if np.all(obs.lo <= q) and np.all(q <= obs.hi):
                return True
        return False

    def box_collides(self, lo: np.ndarray, hi: np.ndarray) -> bool:
        for obs in self.obstacles:
            if obs.overlaps(lo, hi):
                return True
        return False

    def scan_collision_map(self, resolution: float = 0.025):
        """扫描碰撞底图 → (cmap, extent)."""
        xs = np.arange(self.q_lo[0], self.q_hi[0], resolution)
        ys = np.arange(self.q_lo[1], self.q_hi[1], resolution)
        cmap = np.zeros((len(ys), len(xs)), dtype=np.float32)
        for i, y in enumerate(ys):
            for j, x in enumerate(xs):
                if self.is_collision(np.array([x, y])):
                    cmap[i, j] = 1.0
        extent = [self.q_lo[0], self.q_hi[0], self.q_lo[1], self.q_hi[1]]
        return cmap, extent


# ═══════════════════════════════════════════════════════════════════════════
#  BoxInfo
# ═══════════════════════════════════════════════════════════════════════════

@dataclass
class BoxInfo:
    box_id: int
    lo: np.ndarray
    hi: np.ndarray
    seed: np.ndarray
    parent_box_id: int = -1
    expand_face_dim: int = -1
    expand_face_side: int = -1
    root_id: int = -1
    is_coarse: bool = False          # True = coarse phase box

    @property
    def volume(self) -> float:
        return float(np.prod(self.hi - self.lo))

    def center(self) -> np.ndarray:
        return (self.lo + self.hi) * 0.5

    def contains(self, q: np.ndarray, tol: float = 1e-10) -> bool:
        return bool(np.all(q >= self.lo - tol) and np.all(q <= self.hi + tol))

    def is_adjacent(self, other: "BoxInfo", tol: float = 1e-9) -> bool:
        """面邻接: 所有维度重叠 (>0), 且至少一个维度 touch."""
        n = len(self.lo)
        has_touch = False
        for d in range(n):
            overlap = min(self.hi[d], other.hi[d]) - max(self.lo[d], other.lo[d])
            if overlap < -tol:
                return False
            if overlap < tol:
                has_touch = True
        return has_touch

    def intervals_tuple(self):
        return tuple((float(self.lo[d]), float(self.hi[d])) for d in range(len(self.lo)))

"""
sbf5_bench/metrics.py — Path quality metrics

Provides path length, smoothness, clearance, efficiency, and
joint range usage evaluation.

Migrated from: v3/src/planner/metrics.py
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from .base import PlanningResult


@dataclass
class PathMetrics:
    """Aggregated path quality metrics."""

    path_length: float = 0.0
    direct_distance: float = 0.0
    efficiency: float = 0.0          # direct / path_length (≤ 1.0)
    smoothness_mean: float = 0.0     # mean angle change between segments
    smoothness_max: float = 0.0      # max angle change
    clearance_min: float = float("inf")
    clearance_avg: float = 0.0
    joint_range_usage: List[float] = field(default_factory=list)
    n_waypoints: int = 0
    planning_time_s: float = 0.0

    def to_dict(self) -> dict:
        return {
            "path_length": self.path_length,
            "direct_distance": self.direct_distance,
            "efficiency": self.efficiency,
            "smoothness_mean": self.smoothness_mean,
            "smoothness_max": self.smoothness_max,
            "clearance_min": self.clearance_min,
            "clearance_avg": self.clearance_avg,
            "joint_range_usage": self.joint_range_usage,
            "n_waypoints": self.n_waypoints,
            "planning_time_s": self.planning_time_s,
        }


# ──────────────────────────────────────────────────────────────
# Individual metric functions
# ──────────────────────────────────────────────────────────────

def compute_path_length(
    path: List[np.ndarray],
    joint_wrapping: Optional[List[bool]] = None,
    period: float = 2.0 * np.pi,
) -> float:
    """L2 path length, with optional per-joint geodesic wrapping."""
    if len(path) < 2:
        return 0.0

    total = 0.0
    for i in range(len(path) - 1):
        diff = path[i + 1] - path[i]
        if joint_wrapping is not None:
            half = period / 2.0
            for j, wrap in enumerate(joint_wrapping):
                if wrap:
                    diff[j] = (diff[j] + half) % period - half
        total += float(np.linalg.norm(diff))
    return total


def compute_smoothness(
    path: List[np.ndarray],
) -> Tuple[float, float]:
    """Smoothness via consecutive-segment angle changes.

    Returns:
        (mean_angle_change, max_angle_change) in radians.
    """
    if len(path) < 3:
        return 0.0, 0.0

    angles: List[float] = []
    for i in range(1, len(path) - 1):
        v1 = path[i] - path[i - 1]
        v2 = path[i + 1] - path[i]
        n1 = np.linalg.norm(v1)
        n2 = np.linalg.norm(v2)
        if n1 < 1e-10 or n2 < 1e-10:
            continue
        cos_a = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
        angles.append(float(np.arccos(cos_a)))

    if not angles:
        return 0.0, 0.0
    return float(np.mean(angles)), float(np.max(angles))


def compute_clearance(
    path: List[np.ndarray],
    robot,
    obstacles,
    resolution: int = 20,
) -> Tuple[float, float]:
    """Workspace clearance (min, avg) along the path.

    Computes FK at sampled configs, measures distance from each
    link endpoint to each obstacle AABB surface.

    Args:
        robot: sbf5.Robot instance
        obstacles: list of sbf5.Obstacle
        resolution: samples per segment
    """
    if not obstacles or len(path) < 1:
        return float("inf"), float("inf")

    # Build obstacle arrays once
    obs_bounds = []
    for obs in obstacles:
        mn = np.asarray(obs.min_point())
        mx = np.asarray(obs.max_point())
        obs_bounds.append((mn, mx))

    clearances: List[float] = []

    # Sample configs along the path
    configs = []
    for i in range(len(path) - 1):
        for t in np.linspace(0, 1, resolution, endpoint=False):
            configs.append(path[i] + t * (path[i + 1] - path[i]))
    configs.append(path[-1])

    for q in configs:
        # Use FK to get link positions — requires robot to expose this
        # For now, clearance uses a simplified C-space proxy
        min_dist = _cspace_obstacle_proxy(q, obs_bounds)
        clearances.append(min_dist)

    if not clearances:
        return float("inf"), float("inf")
    return float(np.min(clearances)), float(np.mean(clearances))


def _cspace_obstacle_proxy(
    q: np.ndarray,
    obs_bounds: List[Tuple[np.ndarray, np.ndarray]],
) -> float:
    """Proxy clearance: distance from config point to obstacle AABBs.

    This is a simplified metric (joint-space, not workspace).
    Full workspace clearance requires FK link positions.
    """
    # Since we don't have FK-level link positions through the bindings,
    # return inf (clearance not computable without FK exposure).
    # Users can override with workspace-aware computation if robot
    # exposes get_link_positions().
    return float("inf")


def compute_joint_range_usage(
    path: List[np.ndarray],
    joint_limits: List[Tuple[float, float]],
) -> List[float]:
    """Per-joint range usage: (max - min) / total_range along the path."""
    if len(path) < 2:
        return [0.0] * len(joint_limits)

    path_arr = np.array(path)
    n_joints = min(path_arr.shape[1], len(joint_limits))
    usage = []
    for j in range(n_joints):
        lo, hi = joint_limits[j]
        total = hi - lo
        if total < 1e-10:
            usage.append(0.0)
        else:
            used = float(np.ptp(path_arr[:, j]))
            usage.append(min(used / total, 1.0))
    return usage


# ──────────────────────────────────────────────────────────────
# High-level evaluation
# ──────────────────────────────────────────────────────────────

def evaluate_result(
    result: PlanningResult,
    robot=None,
    obstacles=None,
    joint_limits: Optional[List[Tuple[float, float]]] = None,
) -> PathMetrics:
    """Compute all metrics for a single PlanningResult."""
    metrics = PathMetrics()
    metrics.planning_time_s = result.planning_time_s

    if not result.success or result.path is None or len(result.path) < 2:
        return metrics

    path = [result.path[i] for i in range(result.path.shape[0])]
    metrics.n_waypoints = len(path)

    # Path length
    metrics.path_length = compute_path_length(path)
    metrics.direct_distance = float(np.linalg.norm(path[-1] - path[0]))
    if metrics.direct_distance > 1e-10:
        metrics.efficiency = metrics.direct_distance / metrics.path_length
    else:
        metrics.efficiency = 1.0

    # Smoothness
    metrics.smoothness_mean, metrics.smoothness_max = compute_smoothness(path)

    # Clearance (requires robot + obstacles)
    if robot is not None and obstacles is not None:
        metrics.clearance_min, metrics.clearance_avg = compute_clearance(
            path, robot, obstacles)

    # Joint range usage
    if joint_limits:
        metrics.joint_range_usage = compute_joint_range_usage(
            path, joint_limits)

    return metrics


def compare_results(
    results: Dict[str, PlanningResult],
    robot=None,
    obstacles=None,
    joint_limits: Optional[List[Tuple[float, float]]] = None,
) -> Dict[str, PathMetrics]:
    """Evaluate multiple planners and return metrics dict."""
    return {
        name: evaluate_result(r, robot, obstacles, joint_limits)
        for name, r in results.items()
    }


def format_comparison_table(metrics: Dict[str, PathMetrics]) -> str:
    """Format metrics as a Markdown table."""
    if not metrics:
        return "No data."

    names = list(metrics.keys())
    header = "| Metric | " + " | ".join(names) + " |"
    sep = "|" + "---|" * (len(names) + 1)

    rows_spec = [
        ("Path Length", "path_length", ".4f"),
        ("Direct Dist", "direct_distance", ".4f"),
        ("Efficiency", "efficiency", ".4f"),
        ("Smooth (mean)", "smoothness_mean", ".4f"),
        ("Smooth (max)", "smoothness_max", ".4f"),
        ("Clearance (min)", "clearance_min", ".4f"),
        ("Clearance (avg)", "clearance_avg", ".4f"),
        ("Waypoints", "n_waypoints", "d"),
        ("Time (s)", "planning_time_s", ".3f"),
    ]

    lines = [header, sep]
    for label, attr, fmt in rows_spec:
        row = f"| {label} "
        for n in names:
            val = getattr(metrics[n], attr)
            row += f"| {val:{fmt}} "
        row += "|"
        lines.append(row)
    return "\n".join(lines)

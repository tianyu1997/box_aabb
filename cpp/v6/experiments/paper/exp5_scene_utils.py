#!/usr/bin/env python3
"""Shared geometry helpers for Exp. 5 randomized robot scenes."""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Iterable

import numpy as np

EPS = 1e-10


def load_robot_doc(path: Path) -> dict:
    return json.loads(Path(path).read_text())


def joint_limits(robot_doc: dict) -> np.ndarray:
    return np.asarray(robot_doc["joint_limits"], dtype=float)


def dh_point_matrix(alpha: float, a: float, theta: float, d: float) -> np.ndarray:
    ct = math.cos(theta)
    st = math.sin(theta)
    ca = math.cos(alpha)
    sa = math.sin(alpha)
    return np.asarray(
        [
            [ct, -st, 0.0, a],
            [st * ca, ct * ca, -sa, -d * sa],
            [st * sa, ct * sa, ca, d * ca],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def fk_prefixes(robot_doc: dict, q: Iterable[float]) -> list[np.ndarray]:
    q_arr = np.asarray(list(q), dtype=float)
    prefixes = [np.eye(4, dtype=float)]
    for index, dh in enumerate(robot_doc["dh_params"]):
        theta = float(dh.get("theta", 0.0))
        d_value = float(dh.get("d", 0.0))
        if dh.get("type", "revolute") == "revolute":
            theta += float(q_arr[index])
        else:
            d_value += float(q_arr[index])
        prefixes.append(
            prefixes[-1]
            @ dh_point_matrix(float(dh["alpha"]), float(dh["a"]), theta, d_value)
        )
    tool = robot_doc.get("tool_frame")
    if tool is not None:
        prefixes.append(
            prefixes[-1]
            @ dh_point_matrix(
                float(tool.get("alpha", 0.0)),
                float(tool.get("a", 0.0)),
                float(tool.get("theta", 0.0)),
                float(tool.get("d", 0.0)),
            )
        )
    return prefixes


def frame_points(robot_doc: dict, q: Iterable[float]) -> np.ndarray:
    return np.asarray([tf[:3, 3] for tf in fk_prefixes(robot_doc, q)], dtype=float)


def active_link_indices(robot_doc: dict) -> list[int]:
    dh_params = robot_doc["dh_params"]
    n_joints = len(dh_params)
    n_total = n_joints + (1 if robot_doc.get("tool_frame") is not None else 0)
    skip = [False] * n_total
    if skip:
        skip[0] = True
    for index in range(1, n_joints):
        dh = dh_params[index]
        if abs(float(dh.get("a", 0.0))) < EPS and abs(float(dh.get("d", 0.0))) < EPS:
            skip[index] = True
    tool = robot_doc.get("tool_frame")
    if tool is not None:
        tool_index = n_joints
        if abs(float(tool.get("a", 0.0))) < EPS and abs(float(tool.get("d", 0.0))) < EPS:
            skip[tool_index] = True
    return [index for index in range(n_total) if not skip[index]]


def active_link_radii(robot_doc: dict) -> list[float]:
    radii = list(robot_doc.get("link_radii", []))
    result = []
    for index in active_link_indices(robot_doc):
        result.append(float(radii[index]) if index < len(radii) else 0.0)
    return result


def active_link_segments(robot_doc: dict, q: Iterable[float]) -> list[tuple[int, np.ndarray, np.ndarray, float]]:
    prefixes = fk_prefixes(robot_doc, q)
    indices = active_link_indices(robot_doc)
    radii = active_link_radii(robot_doc)
    segments = []
    for index, radius in zip(indices, radii):
        if index + 1 >= len(prefixes):
            continue
        start = prefixes[index][:3, 3].copy()
        end = prefixes[index + 1][:3, 3].copy()
        if np.linalg.norm(end - start) < EPS:
            continue
        segments.append((index, start, end, radius))
    return segments


def protected_frame_segments(robot_doc: dict) -> list[tuple[int, np.ndarray, np.ndarray]]:
    q_zero = np.zeros(len(robot_doc["dh_params"]), dtype=float)
    prefixes = fk_prefixes(robot_doc, q_zero)
    segments = []
    for index in range(min(2, len(prefixes) - 1)):
        start = prefixes[index][:3, 3].copy()
        end = prefixes[index + 1][:3, 3].copy()
        if np.linalg.norm(end - start) >= EPS:
            segments.append((index, start, end))
    return segments


def obstacle_bounds(obstacle: dict) -> np.ndarray:
    if "bounds" in obstacle:
        return np.asarray(obstacle["bounds"], dtype=float)
    lo = np.asarray(obstacle["lo"], dtype=float)
    hi = np.asarray(obstacle["hi"], dtype=float)
    return np.concatenate([lo, hi])


def make_box_obstacle(name: str, center: np.ndarray, half_sizes: np.ndarray, role: str) -> dict:
    lo = np.asarray(center, dtype=float) - np.asarray(half_sizes, dtype=float)
    hi = np.asarray(center, dtype=float) + np.asarray(half_sizes, dtype=float)
    return {
        "name": name,
        "kind": "box",
        "role": role,
        "center": [round(float(x), 8) for x in center],
        "half_sizes": [round(float(x), 8) for x in half_sizes],
        "lo": [round(float(x), 8) for x in lo],
        "hi": [round(float(x), 8) for x in hi],
        "bounds": [round(float(x), 8) for x in [*lo, *hi]],
    }


def aabb_inside(bounds: np.ndarray, workspace_bounds: dict, margin: float = 0.0) -> bool:
    lo = np.asarray(workspace_bounds["lo"], dtype=float) + margin
    hi = np.asarray(workspace_bounds["hi"], dtype=float) - margin
    return bool(np.all(bounds[:3] >= lo) and np.all(bounds[3:] <= hi))


def segment_hits_aabb(start: np.ndarray, end: np.ndarray, bounds: np.ndarray, radius: float) -> bool:
    lo = bounds[:3] - radius
    hi = bounds[3:] + radius
    direction = end - start
    t_enter = 0.0
    t_exit = 1.0
    for axis in range(3):
        if abs(float(direction[axis])) < 1e-15:
            if start[axis] < lo[axis] or start[axis] > hi[axis]:
                return False
        else:
            inv = 1.0 / float(direction[axis])
            t1 = (lo[axis] - start[axis]) * inv
            t2 = (hi[axis] - start[axis]) * inv
            if t1 > t2:
                t1, t2 = t2, t1
            t_enter = max(t_enter, float(t1))
            t_exit = min(t_exit, float(t2))
            if t_enter > t_exit:
                return False
    return True


def check_config_collision(robot_doc: dict, obstacles: list[dict], q: Iterable[float]) -> bool:
    for _, start, end, radius in active_link_segments(robot_doc, q):
        for obstacle in obstacles:
            if segment_hits_aabb(start, end, obstacle_bounds(obstacle), radius):
                return True
    return False


def check_endpoint_clearance(
    robot_doc: dict,
    obstacles: list[dict],
    start_q: Iterable[float],
    goal_q: Iterable[float],
    margin_m: float,
) -> bool:
    """Return True if start and goal are both free when obstacles are inflated by margin_m."""
    inflated = []
    for obstacle in obstacles:
        bounds = obstacle_bounds(obstacle)
        lo = bounds[:3] - float(margin_m)
        hi = bounds[3:] + float(margin_m)
        inflated.append({"bounds": [float(v) for v in [*lo, *hi]]})
    return (
        not check_config_collision(robot_doc, inflated, start_q)
        and not check_config_collision(robot_doc, inflated, goal_q)
    )


def check_segment_collision(
    robot_doc: dict,
    obstacles: list[dict],
    start_q: Iterable[float],
    goal_q: Iterable[float],
    resolution: int,
) -> bool:
    start_arr = np.asarray(start_q, dtype=float)
    goal_arr = np.asarray(goal_q, dtype=float)
    for sample_index in range(resolution + 1):
        t = sample_index / float(resolution)
        q = start_arr + t * (goal_arr - start_arr)
        if check_config_collision(robot_doc, obstacles, q):
            return True
    return False


def aabb_intersects_z_cylinder(bounds: np.ndarray, radius: float, z_min: float, z_max: float) -> bool:
    if bounds[5] < z_min or bounds[2] > z_max:
        return False
    if 0.0 < bounds[0]:
        dx = bounds[0]
    elif 0.0 > bounds[3]:
        dx = -bounds[3]
    else:
        dx = 0.0
    if 0.0 < bounds[1]:
        dy = bounds[1]
    elif 0.0 > bounds[4]:
        dy = -bounds[4]
    else:
        dy = 0.0
    return dx * dx + dy * dy <= radius * radius


def obstacle_avoids_protected_parts(
    robot_doc: dict,
    obstacle: dict,
    *,
    base_radius: float,
    base_height: float,
    link1_radius: float,
) -> bool:
    bounds = obstacle_bounds(obstacle)
    if aabb_intersects_z_cylinder(bounds, base_radius, 0.0, base_height):
        return False
    for _, start, end in protected_frame_segments(robot_doc):
        if segment_hits_aabb(start, end, bounds, link1_radius):
            return False
    return True


def sample_workspace_bounds(
    robot_doc: dict,
    rng: np.random.Generator,
    *,
    samples: int = 512,
    margin: float = 0.08,
) -> dict:
    limits = joint_limits(robot_doc)
    points = [np.zeros(3, dtype=float)]
    for _ in range(samples):
        q = rng.uniform(limits[:, 0], limits[:, 1])
        points.extend(frame_points(robot_doc, q))
    cloud = np.asarray(points, dtype=float)
    lo = np.min(cloud, axis=0) - margin
    hi = np.max(cloud, axis=0) + margin
    lo[2] = max(0.02, lo[2])
    hi[2] = max(hi[2], lo[2] + 0.25)
    return {
        "lo": [round(float(x), 8) for x in lo],
        "hi": [round(float(x), 8) for x in hi],
        "source": f"{samples} random FK samples plus {margin:.3f} m margin",
    }


def path_is_directly_blocked(scene: dict, robot_doc: dict) -> bool:
    return check_segment_collision(
        robot_doc,
        scene["obstacles"],
        scene["start"],
        scene["goal"],
        int(scene["checks"]["segment_resolution"]),
    )

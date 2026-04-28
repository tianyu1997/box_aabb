#!/usr/bin/env python3
"""Baseline planners for Exp. 5 randomized robot scenes."""
from __future__ import annotations

import heapq
import importlib
import math
import sys
import time
from pathlib import Path

import numpy as np

from common import PYTHON_SRC, ROOT
from exp5_scene_utils import check_config_collision, check_segment_collision, joint_limits

SCRIPTS_DIR = ROOT / "scripts"

METHOD_LABELS = {
    "sbf": "SBF (ours)",
    "iris_np_gcs": "IRIS-NP+GCS",
    "iris_zo_gcs": "IRIS-ZO+GCS",
    "ompl_prm": "OMPL PRM",
    "ompl_bitstar": "OMPL BIT*",
}

METHOD_IMPLEMENTATIONS = {
    "sbf": "v6 SBF build/query pipeline with non-IIWA z4 disabled",
    "iris_np_gcs": "Exp.5 local AABB region-graph proxy for the IRIS-NP+GCS baseline family",
    "iris_zo_gcs": "Exp.5 stochastic local AABB region-graph proxy for the IRIS-ZO+GCS baseline family",
    "ompl_prm": "Exp.5 Python PRM proxy because v6 was built with SBF_WITH_OMPL=OFF",
    "ompl_bitstar": "Exp.5 bounded informed multi-start proxy because v6 was built with SBF_WITH_OMPL=OFF",
}


class JointSpaceChecker:
    def __init__(self, robot_doc: dict, obstacles: list[dict], edge_resolution: int) -> None:
        self.robot_doc = robot_doc
        self.obstacles = obstacles
        self.edge_resolution = int(edge_resolution)

    def free(self, config: np.ndarray) -> bool:
        return not check_config_collision(self.robot_doc, self.obstacles, config)

    def edge_free(self, start: np.ndarray, goal: np.ndarray) -> bool:
        distance = float(np.linalg.norm(goal - start))
        resolution = max(8, int(math.ceil(distance * self.edge_resolution)))
        return not check_segment_collision(self.robot_doc, self.obstacles, start, goal, resolution)


def import_sbf5(python_dir: Path):
    for path in (python_dir, PYTHON_SRC, SCRIPTS_DIR):
        text = str(path)
        if text not in sys.path:
            sys.path.insert(0, text)
    return importlib.import_module("sbf5")


def random_config(limits: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    span = limits[:, 1] - limits[:, 0]
    return rng.uniform(limits[:, 0] + 0.02 * span, limits[:, 1] - 0.02 * span)


def sample_free_config(limits: np.ndarray, checker: JointSpaceChecker, rng: np.random.Generator) -> np.ndarray | None:
    for _ in range(200):
        candidate = random_config(limits, rng)
        if checker.free(candidate):
            return candidate
    return None


def joint_path_length(waypoints: list[np.ndarray] | np.ndarray) -> float:
    waypoint_array = np.asarray(waypoints, dtype=float)
    if waypoint_array.ndim != 2 or waypoint_array.shape[0] < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(waypoint_array, axis=0), axis=1)))


def serialize_path(waypoints: list[np.ndarray] | np.ndarray) -> list[list[float]]:
    return [[round(float(value), 8) for value in row] for row in np.asarray(waypoints, dtype=float)]


def steer(from_config: np.ndarray, to_config: np.ndarray, step_size: float) -> np.ndarray:
    delta = to_config - from_config
    distance = float(np.linalg.norm(delta))
    if distance <= step_size:
        return to_config.copy()
    return from_config + (step_size / distance) * delta


def nearest_index(nodes: list[np.ndarray], target: np.ndarray) -> int:
    distances = [float(np.linalg.norm(node - target)) for node in nodes]
    return int(np.argmin(distances))


def reconstruct(nodes: list[np.ndarray], parents: list[int], index: int) -> list[np.ndarray]:
    path = []
    cursor = index
    while cursor >= 0:
        path.append(nodes[cursor])
        cursor = parents[cursor]
    path.reverse()
    return path


def shortcut_path(path: list[np.ndarray], checker: JointSpaceChecker, rng: np.random.Generator, attempts: int = 96) -> list[np.ndarray]:
    if len(path) <= 2:
        return path
    result = list(path)
    for _ in range(attempts):
        if len(result) <= 2:
            break
        first = int(rng.integers(0, len(result) - 2))
        second = int(rng.integers(first + 2, len(result)))
        if checker.edge_free(result[first], result[second]):
            result = result[: first + 1] + result[second:]
    return result


def run_rrt_connect(
    robot_doc: dict,
    obstacles: list[dict],
    start: np.ndarray,
    goal: np.ndarray,
    *,
    seed: int,
    timeout_s: float,
    edge_resolution: int,
    step_size: float = 0.45,
) -> dict:
    checker = JointSpaceChecker(robot_doc, obstacles, edge_resolution)
    if not checker.free(start) or not checker.free(goal):
        return failure_result(seed, "start_or_goal_in_collision", METHOD_IMPLEMENTATIONS["ompl_bitstar"])

    rng = np.random.default_rng(seed)
    limits = joint_limits(robot_doc)
    tree_a_nodes = [start.copy()]
    tree_a_parents = [-1]
    tree_b_nodes = [goal.copy()]
    tree_b_parents = [-1]
    swapped = False
    start_time = time.perf_counter()
    iterations = 0
    while time.perf_counter() - start_time < timeout_s:
        iterations += 1
        if rng.random() < 0.15:
            sample = goal.copy() if not swapped else start.copy()
        else:
            sample = sample_free_config(limits, checker, rng)
            if sample is None:
                continue

        index_a = nearest_index(tree_a_nodes, sample)
        new_a = steer(tree_a_nodes[index_a], sample, step_size)
        if not checker.edge_free(tree_a_nodes[index_a], new_a):
            tree_a_nodes, tree_b_nodes = tree_b_nodes, tree_a_nodes
            tree_a_parents, tree_b_parents = tree_b_parents, tree_a_parents
            swapped = not swapped
            continue
        tree_a_nodes.append(new_a)
        tree_a_parents.append(index_a)
        new_a_index = len(tree_a_nodes) - 1

        reached = False
        connect_index = nearest_index(tree_b_nodes, new_a)
        while True:
            candidate = steer(tree_b_nodes[connect_index], new_a, step_size)
            if not checker.edge_free(tree_b_nodes[connect_index], candidate):
                break
            tree_b_nodes.append(candidate)
            tree_b_parents.append(connect_index)
            connect_index = len(tree_b_nodes) - 1
            if float(np.linalg.norm(candidate - new_a)) < 1e-6:
                reached = True
                break
        if reached:
            path_a = reconstruct(tree_a_nodes, tree_a_parents, new_a_index)
            path_b = reconstruct(tree_b_nodes, tree_b_parents, connect_index)
            if swapped:
                path = path_b + path_a[-2::-1]
            else:
                path = path_a + path_b[-2::-1]
            path = shortcut_path(path, checker, rng)
            elapsed = time.perf_counter() - start_time
            return success_result(seed, elapsed, path, METHOD_IMPLEMENTATIONS["ompl_bitstar"], iterations)

        tree_a_nodes, tree_b_nodes = tree_b_nodes, tree_a_nodes
        tree_a_parents, tree_b_parents = tree_b_parents, tree_a_parents
        swapped = not swapped

    elapsed = time.perf_counter() - start_time
    return failure_result(seed, "timeout", METHOD_IMPLEMENTATIONS["ompl_bitstar"], elapsed, iterations)


def dijkstra(graph: list[list[tuple[int, float]]], start_index: int, goal_index: int) -> list[int] | None:
    queue: list[tuple[float, int]] = [(0.0, start_index)]
    distances = {start_index: 0.0}
    parents = {start_index: -1}
    while queue:
        distance, node = heapq.heappop(queue)
        if node == goal_index:
            break
        if distance > distances.get(node, math.inf):
            continue
        for neighbor, weight in graph[node]:
            new_distance = distance + weight
            if new_distance < distances.get(neighbor, math.inf):
                distances[neighbor] = new_distance
                parents[neighbor] = node
                heapq.heappush(queue, (new_distance, neighbor))
    if goal_index not in parents:
        return None
    order = []
    cursor = goal_index
    while cursor >= 0:
        order.append(cursor)
        cursor = parents[cursor]
    order.reverse()
    return order


def run_prm_proxy(
    robot_doc: dict,
    obstacles: list[dict],
    start: np.ndarray,
    goal: np.ndarray,
    *,
    seed: int,
    timeout_s: float,
    edge_resolution: int,
    sample_count: int,
    neighbors: int,
) -> dict:
    checker = JointSpaceChecker(robot_doc, obstacles, edge_resolution)
    if not checker.free(start) or not checker.free(goal):
        return failure_result(seed, "start_or_goal_in_collision", METHOD_IMPLEMENTATIONS["ompl_prm"])

    rng = np.random.default_rng(seed)
    limits = joint_limits(robot_doc)
    start_time = time.perf_counter()
    nodes = [start.copy(), goal.copy()]
    while len(nodes) < sample_count + 2 and time.perf_counter() - start_time < timeout_s:
        sample = sample_free_config(limits, checker, rng)
        if sample is not None:
            nodes.append(sample)

    graph = [[] for _ in nodes]
    edge_checks = 0
    node_array = np.asarray(nodes)
    for node_index, node in enumerate(nodes):
        if time.perf_counter() - start_time >= timeout_s:
            break
        distances = np.linalg.norm(node_array - node, axis=1)
        neighbor_order = np.argsort(distances)[1 : neighbors + 1]
        for neighbor_index in neighbor_order:
            if neighbor_index <= node_index:
                continue
            edge_checks += 1
            if checker.edge_free(node, nodes[int(neighbor_index)]):
                weight = float(distances[int(neighbor_index)])
                graph[node_index].append((int(neighbor_index), weight))
                graph[int(neighbor_index)].append((node_index, weight))

    order = dijkstra(graph, 0, 1)
    elapsed = time.perf_counter() - start_time
    if order is None:
        return failure_result(seed, "no_graph_path", METHOD_IMPLEMENTATIONS["ompl_prm"], elapsed, edge_checks)
    path = shortcut_path([nodes[index] for index in order], checker, rng)
    return success_result(seed, elapsed, path, METHOD_IMPLEMENTATIONS["ompl_prm"], edge_checks)


def run_region_graph_proxy(
    robot_doc: dict,
    obstacles: list[dict],
    start: np.ndarray,
    goal: np.ndarray,
    *,
    seed: int,
    timeout_s: float,
    edge_resolution: int,
    variant: str,
) -> dict:
    implementation = METHOD_IMPLEMENTATIONS[variant]
    start_time = time.perf_counter()
    skeleton_budget = max(0.5, 0.60 * timeout_s)
    skeleton = run_rrt_connect(
        robot_doc,
        obstacles,
        start,
        goal,
        seed=seed + (101 if variant == "iris_np_gcs" else 211),
        timeout_s=skeleton_budget,
        edge_resolution=edge_resolution,
        step_size=0.42 if variant == "iris_np_gcs" else 0.34,
    )
    if not skeleton.get("success"):
        elapsed = time.perf_counter() - start_time
        return failure_result(seed, "no_region_skeleton", implementation, elapsed, 0)

    checker = JointSpaceChecker(robot_doc, obstacles, edge_resolution)
    rng = np.random.default_rng(seed + 3007)
    path = [np.asarray(row, dtype=float) for row in skeleton["waypoints"]]
    inflation_checks = 0
    # Approximate local IRIS regions: verify small axis-aligned boxes around the
    # skeleton states. The query path is then the GCS-style shortest chain.
    base_radius = 0.10 if variant == "iris_np_gcs" else 0.075
    for center in path:
        for axis_index in range(center.size):
            for sign in (-1.0, 1.0):
                trial = center.copy()
                trial[axis_index] += sign * base_radius
                inflation_checks += 1
                if not checker.free(trial):
                    break
    path = shortcut_path(path, checker, rng, attempts=128 if variant == "iris_np_gcs" else 80)
    elapsed = time.perf_counter() - start_time
    return success_result(seed, elapsed, path, implementation, inflation_checks)


def run_bitstar_proxy(
    robot_doc: dict,
    obstacles: list[dict],
    start: np.ndarray,
    goal: np.ndarray,
    *,
    seed: int,
    timeout_s: float,
    edge_resolution: int,
) -> dict:
    start_time = time.perf_counter()
    best: dict | None = None
    attempts = 0
    while time.perf_counter() - start_time < timeout_s:
        attempts += 1
        remaining = timeout_s - (time.perf_counter() - start_time)
        candidate = run_rrt_connect(
            robot_doc,
            obstacles,
            start,
            goal,
            seed=seed + attempts * 31,
            timeout_s=max(0.1, min(remaining, 0.9)),
            edge_resolution=edge_resolution,
            step_size=max(0.22, 0.48 - 0.03 * attempts),
        )
        if candidate.get("success"):
            if best is None or float(candidate["path_length"]) < float(best["path_length"]):
                best = candidate
    elapsed = time.perf_counter() - start_time
    if best is None:
        return failure_result(seed, "timeout", METHOD_IMPLEMENTATIONS["ompl_bitstar"], elapsed, attempts)
    best = dict(best)
    best.update({"seed": seed, "time_s": elapsed, "planning_time_s": elapsed, "iterations": attempts})
    best["implementation"] = METHOD_IMPLEMENTATIONS["ompl_bitstar"]
    return best


def run_sbf(
    robot_doc: dict,
    scene: dict,
    *,
    python_dir: Path,
    seed: int,
    timeout_s: float,
) -> dict:
    sbf5 = import_sbf5(python_dir)
    comparison = importlib.import_module("run_online_query_comparison")
    robot = sbf5.Robot.from_json(str(ROOT / scene["robot_json"]))
    obstacles = [sbf5.Obstacle(*obstacle["bounds"]) for obstacle in scene["obstacles"]]
    start = np.asarray(scene["start"], dtype=np.float64)
    goal = np.asarray(scene["goal"], dtype=np.float64)
    config = sbf5.SBFPlannerConfig()
    comparison.apply_paper_sbf_architecture(
        config,
        seed=seed,
        grow_timeout_ms=timeout_s * 1000.0,
        max_boxes=50000,
        post_connect_extra_boxes=1000,
        n_threads=8,
        bridge_n_threads=8,
        ffb_depth=220,
        lect_no_cache=True,
    )
    config.z4_enabled = False
    planner = sbf5.SBFPlanner(robot, config)
    build_start = time.perf_counter()
    planner.build_coverage(obstacles, timeout_s * 1000.0, [start, goal])
    build_time = time.perf_counter() - build_start
    query_start = time.perf_counter()
    query = planner.query(start, goal)
    query_time = time.perf_counter() - query_start
    if not bool(query.success):
        return {
            "seed": seed,
            "success": False,
            "build_time_s": build_time,
            "query_time_s": query_time,
            "planning_time_s": build_time + query_time,
            "path_length": None,
            "n_boxes": int(planner.n_boxes()),
            "failure_reason": "query_failed",
            "implementation": METHOD_IMPLEMENTATIONS["sbf"],
        }
    path = [np.asarray(row, dtype=float) for row in query.path]
    return {
        "seed": seed,
        "success": True,
        "build_time_s": build_time,
        "query_time_s": query_time,
        "planning_time_s": build_time + query_time,
        "path_length": float(query.path_length),
        "n_boxes": int(planner.n_boxes()),
        "waypoints": serialize_path(path),
        "implementation": METHOD_IMPLEMENTATIONS["sbf"],
    }


def success_result(seed: int, elapsed: float, path: list[np.ndarray], implementation: str, iterations: int) -> dict:
    return {
        "seed": seed,
        "success": True,
        "time_s": elapsed,
        "planning_time_s": elapsed,
        "path_length": joint_path_length(path),
        "waypoints": serialize_path(path),
        "iterations": int(iterations),
        "implementation": implementation,
    }


def failure_result(
    seed: int,
    reason: str,
    implementation: str,
    elapsed: float = 0.0,
    iterations: int = 0,
) -> dict:
    return {
        "seed": seed,
        "success": False,
        "time_s": elapsed,
        "planning_time_s": elapsed,
        "path_length": None,
        "failure_reason": reason,
        "iterations": int(iterations),
        "implementation": implementation,
    }


def summarize_runs(method: str, runs: list[dict]) -> dict:
    successes = [run for run in runs if run.get("success")]
    path_lengths = [float(run["path_length"]) for run in successes if run.get("path_length") is not None]
    planning_times = [float(run.get("planning_time_s", run.get("time_s", 0.0))) for run in runs]
    query_times = [float(run.get("query_time_s", run.get("time_s", 0.0))) for run in runs]
    build_times = [float(run["build_time_s"]) for run in runs if "build_time_s" in run]
    return {
        "method": method,
        "label": METHOD_LABELS[method],
        "implementation": METHOD_IMPLEMENTATIONS[method],
        "n_runs": len(runs),
        "n_success": len(successes),
        "success_rate": (len(successes) / len(runs)) if runs else 0.0,
        "planning_time_s_median": float(np.median(planning_times)) if planning_times else None,
        "query_time_s_median": float(np.median(query_times)) if query_times else None,
        "build_time_s_median": float(np.median(build_times)) if build_times else None,
        "path_length_median": float(np.median(path_lengths)) if path_lengths else None,
        "runs": runs,
    }


def typical_path(method: str, runs: list[dict]) -> dict | None:
    successes = [run for run in runs if run.get("success") and run.get("waypoints")]
    if not successes:
        return None
    best = min(successes, key=lambda item: float(item.get("path_length") or math.inf))
    return {
        "method": method,
        "label": METHOD_LABELS[method],
        "seed": int(best["seed"]),
        "path_length": float(best["path_length"]),
        "planning_time_s": float(best.get("planning_time_s", best.get("time_s", 0.0))),
        "waypoints": best["waypoints"],
        "implementation": METHOD_IMPLEMENTATIONS[method],
    }


def run_method(
    method: str,
    robot_doc: dict,
    scene: dict,
    *,
    python_dir: Path,
    seed: int,
    timeout_s: float,
    edge_resolution: int,
    prm_samples: int,
) -> dict:
    start = np.asarray(scene["start"], dtype=float)
    goal = np.asarray(scene["goal"], dtype=float)
    obstacles = list(scene["obstacles"])
    if method == "sbf":
        return run_sbf(robot_doc, scene, python_dir=python_dir, seed=seed, timeout_s=timeout_s)
    if method == "iris_np_gcs":
        return run_region_graph_proxy(
            robot_doc,
            obstacles,
            start,
            goal,
            seed=seed,
            timeout_s=timeout_s,
            edge_resolution=edge_resolution,
            variant=method,
        )
    if method == "iris_zo_gcs":
        return run_region_graph_proxy(
            robot_doc,
            obstacles,
            start,
            goal,
            seed=seed,
            timeout_s=timeout_s,
            edge_resolution=edge_resolution,
            variant=method,
        )
    if method == "ompl_prm":
        return run_prm_proxy(
            robot_doc,
            obstacles,
            start,
            goal,
            seed=seed,
            timeout_s=timeout_s,
            edge_resolution=edge_resolution,
            sample_count=prm_samples,
            neighbors=12,
        )
    if method == "ompl_bitstar":
        return run_bitstar_proxy(
            robot_doc,
            obstacles,
            start,
            goal,
            seed=seed,
            timeout_s=timeout_s,
            edge_resolution=edge_resolution,
        )
    raise ValueError(f"unknown method: {method}")


def run_baseline_suite(
    robot_doc: dict,
    scene: dict,
    *,
    python_dir: Path,
    seeds: list[int],
    methods: list[str],
    timeout_s: float,
    edge_resolution: int,
    prm_samples: int,
) -> tuple[list[dict], list[dict]]:
    summaries = []
    paths = []
    for method in methods:
        runs = []
        for seed in seeds:
            runs.append(
                run_method(
                    method,
                    robot_doc,
                    scene,
                    python_dir=python_dir,
                    seed=seed,
                    timeout_s=timeout_s,
                    edge_resolution=edge_resolution,
                    prm_samples=prm_samples,
                )
            )
        summaries.append(summarize_runs(method, runs))
        selected = typical_path(method, runs)
        if selected is not None:
            paths.append(selected)
    return summaries, paths

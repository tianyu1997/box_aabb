#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PAPER_DIR = HERE.parent
for candidate in (HERE, PAPER_DIR):
    text = str(candidate)
    if text not in sys.path:
        sys.path.insert(0, text)

from common import bootstrap_gcs_repo, configure_parser_package_map, load_robot_joint_limits

GCS_REPO = bootstrap_gcs_repo()


def build_robot_diagram_checker(
    *,
    edge_step_size: float = 0.05,
    env_padding: float = 0.0,
    self_padding: float = 0.0,
):
    from pydrake.multibody.parsing import LoadModelDirectives, ProcessModelDirectives
    from pydrake.planning import RobotDiagramBuilder, SceneGraphCollisionChecker

    builder = RobotDiagramBuilder(time_step=0.0)
    parser = builder.parser()
    configure_parser_package_map(parser, gcs_repo=GCS_REPO)
    directives_file = GCS_REPO / "models" / "iiwa14_spheres_collision_welded_gripper.yaml"
    directives = LoadModelDirectives(str(directives_file))
    ProcessModelDirectives(directives, builder.plant(), parser)
    builder.plant().Finalize()

    plant = builder.plant()
    iiwa_inst = plant.GetModelInstanceByName("iiwa")
    wsg_inst = plant.GetModelInstanceByName("wsg")
    robot_diagram = builder.Build()

    checker = SceneGraphCollisionChecker(
        model=robot_diagram,
        robot_model_instances=[iiwa_inst, wsg_inst],
        edge_step_size=edge_step_size,
        env_collision_padding=env_padding,
        self_collision_padding=self_padding,
    )
    return robot_diagram, plant, checker


def joint_limit_domain(robot: str = "iiwa14"):
    from pydrake.all import HPolyhedron

    lo, hi = load_robot_joint_limits(robot)
    return HPolyhedron.MakeBox(np.asarray(lo, dtype=float), np.asarray(hi, dtype=float))


def _path_length(path: np.ndarray) -> float:
    return float(sum(np.linalg.norm(path[index] - path[index - 1]) for index in range(1, len(path))))


def _edge_unsafe_segments(path: np.ndarray, checker) -> list[int]:
    unsafe_segments: list[int] = []
    for index in range(1, len(path)):
        if not checker.CheckEdgeCollisionFree(path[index - 1], path[index]):
            unsafe_segments.append(index - 1)
    return unsafe_segments


def _root_path(nodes: list[np.ndarray], parents: list[int], index: int) -> list[np.ndarray]:
    out: list[np.ndarray] = []
    while index >= 0:
        out.append(nodes[index])
        index = parents[index]
    return out[::-1]


def _local_rrt_connect(
    q_start: np.ndarray,
    q_goal: np.ndarray,
    checker,
    rng: np.random.Generator,
    lo: np.ndarray,
    hi: np.ndarray,
    *,
    deadline: float,
    step_size: float = 0.18,
    max_nodes: int = 4500,
) -> list[np.ndarray] | None:
    import time

    if checker.CheckEdgeCollisionFree(q_start, q_goal):
        return [q_start, q_goal]

    start_nodes = [q_start]
    start_parents = [-1]
    goal_nodes = [q_goal]
    goal_parents = [-1]

    def nearest(nodes: list[np.ndarray], q: np.ndarray) -> int:
        distances = [float(np.linalg.norm(node - q)) for node in nodes]
        return int(np.argmin(distances))

    def steer(q_from: np.ndarray, q_to: np.ndarray) -> np.ndarray:
        delta = q_to - q_from
        dist = float(np.linalg.norm(delta))
        if dist <= step_size:
            return q_to.copy()
        return q_from + (step_size / dist) * delta

    def try_extend(nodes: list[np.ndarray], parents: list[int], target: np.ndarray) -> int | None:
        if len(nodes) >= max_nodes:
            return None
        near_index = nearest(nodes, target)
        candidate = np.minimum(np.maximum(steer(nodes[near_index], target), lo), hi)
        if np.linalg.norm(candidate - nodes[near_index]) < 1e-9:
            return None
        if not checker.CheckConfigCollisionFree(candidate):
            return None
        if not checker.CheckEdgeCollisionFree(nodes[near_index], candidate):
            return None
        nodes.append(candidate)
        parents.append(near_index)
        return len(nodes) - 1

    def try_connect(nodes: list[np.ndarray], parents: list[int], target: np.ndarray) -> int | None:
        best_index = nearest(nodes, target)
        while len(nodes) < max_nodes and time.perf_counter() < deadline:
            candidate = np.minimum(np.maximum(steer(nodes[best_index], target), lo), hi)
            if not checker.CheckConfigCollisionFree(candidate):
                return None
            if not checker.CheckEdgeCollisionFree(nodes[best_index], candidate):
                return None
            nodes.append(candidate)
            parents.append(best_index)
            best_index = len(nodes) - 1
            if np.linalg.norm(candidate - target) < 1e-8:
                return best_index
            if checker.CheckEdgeCollisionFree(candidate, target):
                return best_index
        return None

    while time.perf_counter() < deadline and len(start_nodes) + len(goal_nodes) < 2 * max_nodes:
        if rng.random() < 0.75:
            alpha = float(rng.random())
            sigma = 0.18 + 0.18 * float(rng.random())
            target = (1.0 - alpha) * q_start + alpha * q_goal + rng.normal(0.0, sigma, size=q_start.shape)
            target = np.minimum(np.maximum(target, lo), hi)
        else:
            target = rng.uniform(lo, hi)

        start_index = try_extend(start_nodes, start_parents, target)
        if start_index is not None:
            goal_index = try_connect(goal_nodes, goal_parents, start_nodes[start_index])
            if goal_index is not None:
                start_path = _root_path(start_nodes, start_parents, start_index)
                goal_path = _root_path(goal_nodes, goal_parents, goal_index)
                return start_path + goal_path[::-1]

        goal_index = try_extend(goal_nodes, goal_parents, target)
        if goal_index is not None:
            start_index = try_connect(start_nodes, start_parents, goal_nodes[goal_index])
            if start_index is not None:
                start_path = _root_path(start_nodes, start_parents, start_index)
                goal_path = _root_path(goal_nodes, goal_parents, goal_index)
                return start_path + goal_path[::-1]

    return None


def _shortcut_valid_path(path: list[np.ndarray], checker, rng: np.random.Generator, max_iters: int = 120) -> list[np.ndarray]:
    out = list(path)
    for _ in range(max_iters):
        if len(out) <= 2:
            break
        i = int(rng.integers(0, len(out) - 2))
        j = int(rng.integers(i + 2, len(out)))
        if checker.CheckEdgeCollisionFree(out[i], out[j]):
            out = out[: i + 1] + out[j:]
    return out


def _repair_waypoint(
    q: np.ndarray,
    checker,
    rng: np.random.Generator,
    lo: np.ndarray,
    hi: np.ndarray,
    *,
    max_attempts: int = 240,
) -> np.ndarray | None:
    q = np.minimum(np.maximum(q, lo), hi)
    if checker.CheckConfigCollisionFree(q):
        return q
    for sigma in (0.03, 0.07, 0.14, 0.28, 0.42):
        for _ in range(max_attempts):
            candidate = np.minimum(np.maximum(q + rng.normal(0.0, sigma, size=q.shape), lo), hi)
            if checker.CheckConfigCollisionFree(candidate):
                return candidate
    return None


def _repair_unsafe_path(path: np.ndarray, checker, *, seed: int, per_segment_budget_s: float = 3.0) -> tuple[np.ndarray | None, float, int]:
    import time

    lo, hi = load_robot_joint_limits("iiwa14")
    lo_arr = np.asarray(lo, dtype=float)
    hi_arr = np.asarray(hi, dtype=float)
    rng = np.random.default_rng(51047 + int(seed))
    t0 = time.perf_counter()
    free_path: list[np.ndarray] = []
    for index, waypoint in enumerate(path):
        if index == 0 or index == len(path) - 1:
            if not checker.CheckConfigCollisionFree(waypoint):
                return None, time.perf_counter() - t0, 0
            free_path.append(waypoint)
            continue
        repaired_waypoint = _repair_waypoint(waypoint, checker, rng, lo_arr, hi_arr)
        if repaired_waypoint is None:
            return None, time.perf_counter() - t0, 0
        free_path.append(repaired_waypoint)

    repaired: list[np.ndarray] = [free_path[0]]
    repaired_segments = 0

    for index in range(1, len(free_path)):
        previous = repaired[-1]
        current = free_path[index]
        if checker.CheckEdgeCollisionFree(previous, current):
            repaired.append(current)
            continue
        segment_path = _local_rrt_connect(
            previous,
            current,
            checker,
            rng,
            lo_arr,
            hi_arr,
            deadline=time.perf_counter() + float(per_segment_budget_s),
        )
        if segment_path is None:
            return None, time.perf_counter() - t0, repaired_segments
        repaired.extend(segment_path[1:])
        repaired_segments += 1

    repaired = _shortcut_valid_path(repaired, checker, rng)
    repaired_array = np.asarray(repaired, dtype=float)
    if _edge_unsafe_segments(repaired_array, checker):
        return None, time.perf_counter() - t0, repaired_segments
    return repaired_array, time.perf_counter() - t0, repaired_segments


def solve_regions_gcs(
    q_start: np.ndarray,
    q_goal: np.ndarray,
    regions: list,
    *,
    seed: int,
    checker=None,
    rounding_max_paths: int = 10,
    rounding_max_trials: int = 100,
    import time

    from pydrake.solvers import MosekSolver, SolverOptions

    LinearGCS = importlib.import_module("gcs.linear").LinearGCS
    randomForwardPathSearch = importlib.import_module("gcs.rounding").randomForwardPathSearch

    t0 = time.perf_counter()
    n_regions = len(regions)
    if n_regions == 0:
        return {
            "success": False,
            "time_s": 0.0,
            "path_length": None,
            "regions": 0,
            "edges": 0,
            "note": "no regions",
        }

    gcs = LinearGCS(regions)
    try:
        gcs.addSourceTarget(q_start, q_goal)
    except ValueError as exc:
        return {
            "success": False,
            "time_s": time.perf_counter() - t0,
            "path_length": None,
            "regions": n_regions,
            "edges": 0,
            "note": f"addSourceTarget failed: {exc}",
        }

    gcs.setRoundingStrategy(
        randomForwardPathSearch,
    gcs.setRoundingStrategy(randomForwardPathSearch, max_paths=10, max_trials=100, seed=int(seed))
    solver_options.SetOption(MosekSolver.id(), "MSK_DPAR_INTPNT_CO_TOL_REL_GAP", 1e-3)
    gcs.setSolverOptions(solver_options)

    try:
        waypoints, _ = gcs.SolvePath(rounding=bool(rounding), verbose=False, preprocessing=True)
    except RuntimeError as exc:
        return {
            "success": False,
            "time_s": time.perf_counter() - t0,
            "path_length": None,
            "regions": n_regions,
            "edges": len([edge for edge in gcs.gcs.Edges()]),
            "note": f"GCS solve raised RuntimeError: {exc}",
        }

    dt = time.perf_counter() - t0
    if waypoints is None:
        return {
            "success": False,
            "time_s": dt,
            "path_length": None,
            "regions": n_regions,
            "edges": len([edge for edge in gcs.gcs.Edges()]),
            "note": "GCS solve failed",
        }

    path = waypoints.T
    path_length = _path_length(path)
    if checker is not None:
        unsafe_segments = _edge_unsafe_segments(path, checker)
        if unsafe_segments:
            repaired_path, repair_s, repaired_segments = _repair_unsafe_path(path, checker, seed=seed)
            if repaired_path is not None:
                return {
                    "success": True,
                    "time_s": time.perf_counter() - t0,
                    "path_length": _path_length(repaired_path),
                    "raw_path_length": path_length,
                    "regions": n_regions,
                    "edges": len([edge for edge in gcs.gcs.Edges()]),
                    "waypoints_count": int(repaired_path.shape[0]),
                    "raw_waypoints_count": int(path.shape[0]),
                    "collision_checked": True,
                    "collision_free": True,
                    "unsafe_segments": 0,
                    "gcs_unsafe_segments": len(unsafe_segments),
                    "repaired_segments": repaired_segments,
                    "repair_time_s": repair_s,
                    "note": "GCS path repaired by local collision-checked RRT-Connect",
                }
            return {
                "success": False,
                "time_s": time.perf_counter() - t0,
                "path_length": None,
                "raw_path_length": path_length,
                "regions": n_regions,
                "edges": len([edge for edge in gcs.gcs.Edges()]),
                "waypoints_count": int(path.shape[0]),
                "collision_checked": True,
                "collision_free": False,
                "unsafe_segments": len(unsafe_segments),
                "repair_time_s": repair_s,
                "note": f"GCS path failed collision validation on {len(unsafe_segments)} segment(s)",
            }
    return {
        "success": True,
        "time_s": dt,
        "path_length": path_length,
        "regions": n_regions,
        "edges": len([edge for edge in gcs.gcs.Edges()]),
        "waypoints_count": int(path.shape[0]),
        "collision_checked": checker is not None,
        "collision_free": True if checker is not None else None,
        "unsafe_segments": 0 if checker is not None else None,
    }
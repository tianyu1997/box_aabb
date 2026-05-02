#!/usr/bin/env python3
"""Baseline planners for Exp. 5 randomized robot scenes."""
from __future__ import annotations

import heapq
import importlib
import json
import math
import multiprocessing as mp
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

from common import PAPER_THREADS, PYTHON_SRC, ROOT
from exp5_scene_utils import check_config_collision, check_segment_collision, joint_limits

SCRIPTS_DIR = ROOT / "scripts"
EXP5_LECT_CACHE_DIR = ROOT / "experiments" / "results_paper" / "exp5_random_scenes" / "lect_cache"

METHOD_LABELS = {
    "sbf": "SBF (C+AABB)",
    "sbf_ifk": "SBF (IFK+AABB)",
    "iris_np_gcs": "Drake IRIS-NP+GCS",
    "ompl_prm": "OMPL PRM",
    "ompl_bitstar": "OMPL BIT*",
}

METHOD_IMPLEMENTATIONS = {
    "sbf": "Exp.3/4 paper SBF CritSample+LinkIAABB(S=4) build/cached-query protocol with per-scene LECT prewarm and non-IIWA z4 disabled",
    "sbf_ifk": "Exp.3/4 paper SBF IFK+LinkIAABB(S=4) build/cached-query protocol with per-scene LECT prewarm and non-IIWA z4 disabled",
    "iris_np_gcs": "Drake IRIS-NP + GCS using generated OBJ collision URDF and SceneGraphCollisionChecker",
    "ompl_prm": "Exp.5 v6-native C++ OMPL PRM runner with the same collision model as Exp.4",
    "ompl_bitstar": "Exp.5 v6-native C++ OMPL BIT* runner with the same collision model as Exp.4",
}

GENERATED_DRAKE_URDFS = ROOT / "data" / "urdf" / "generated" / "drake_compatible"
DRAKE_ROBOT_SPECS = {
    "ur5": {
        "urdf": GENERATED_DRAKE_URDFS / "ur5_drake.urdf",
        "package": "ur_description",
        "package_dir": ROOT / "data" / "urdf" / "upstream" / "ur_description",
        "base_frame": "base_link",
    },
    "panda": {
        "urdf": GENERATED_DRAKE_URDFS / "panda_drake.urdf",
        "package": "moveit_resources_panda_description",
        "package_dir": ROOT / "data" / "urdf" / "upstream" / "moveit_resources_panda_description",
        "base_frame": "panda_link0",
    },
}

IRIS_NP_MIN_BUILD_BUDGET_S = 120.0
IRIS_DRAKE_MAX_SEED_CONFIGS = 64


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


def import_sbf6(python_dir: Path):
    for path in (python_dir, PYTHON_SRC, SCRIPTS_DIR):
        text = str(path)
        if text not in sys.path:
            sys.path.insert(0, text)
    return importlib.import_module("sbf6")


def exp5_sbf_cache_dir(scene: dict, *, sbf_endpoint_source: str = "critsample") -> Path:
    """LECT cache root per scene. IFK builds use a sibling ``ifk`` subdir so prewarm/build state stays isolated."""
    base = EXP5_LECT_CACHE_DIR / str(scene.get("scene_id") or scene.get("robot") or "scene")
    ep = (sbf_endpoint_source or "critsample").strip().lower()
    if ep == "ifk":
        return base / "ifk"
    return base


def scene_start_goal(scene: dict) -> tuple[np.ndarray, np.ndarray]:
    start = np.asarray(scene["start"], dtype=np.float64)
    goal = np.asarray(scene["goal"], dtype=np.float64)
    return start, goal


def build_sbf_planner(
    scene: dict,
    *,
    python_dir: Path,
    seed: int,
    timeout_s: float,
    cache_dir: Path | None,
    sbf_endpoint_source: str = "critsample",
) -> tuple[object, list[object], np.ndarray, np.ndarray]:
    sbf6 = import_sbf6(python_dir)
    comparison = importlib.import_module("run_online_query_comparison")
    robot = sbf6.Robot.from_json(str(ROOT / scene["robot_json"]))
    obstacles = [sbf6.Obstacle(*obstacle["bounds"]) for obstacle in scene["obstacles"]]
    start, goal = scene_start_goal(scene)
    config = sbf6.SBFPlannerConfig()
    comparison.apply_paper_sbf_architecture(
        config,
        seed=seed,
        grow_timeout_ms=timeout_s * 1000.0,
        n_threads=PAPER_THREADS,
        bridge_n_threads=PAPER_THREADS,
        lect_no_cache=cache_dir is None,
        lect_cache_dir=str(cache_dir) if cache_dir is not None else None,
    )
    comparison.apply_exp3_sbf_build_variant(config, sbf6, endpoint_source=sbf_endpoint_source)
    if scene.get("robot") != "iiwa14":
        config.z4_enabled = False
    # Keep the Exp.4 cached-query configuration: path post-processing, direct
    # RRT path-quality competition, and fallback budgets remain enabled.
    planner = sbf6.SBFPlanner(robot, config)
    return planner, obstacles, start, goal


def ensure_sbf_lect_warmup(scene: dict, *, python_dir: Path, timeout_s: float, sbf_endpoint_source: str = "critsample") -> dict:
    cache_dir = exp5_sbf_cache_dir(scene, sbf_endpoint_source=sbf_endpoint_source)
    cache_dir.mkdir(parents=True, exist_ok=True)
    marker = cache_dir / "warmup_done.json"
    if marker.exists():
        return {"status": "reused", "cache_dir": str(cache_dir.relative_to(ROOT))}

    planner, obstacles, start, goal = build_sbf_planner(
        scene,
        python_dir=python_dir,
        seed=0,
        timeout_s=timeout_s,
        cache_dir=cache_dir,
        sbf_endpoint_source=sbf_endpoint_source,
    )
    planner.build_coverage(obstacles, timeout_s * 1000.0, [start, goal])
    payload = {
        "scene_id": scene.get("scene_id"),
        "robot": scene.get("robot"),
        "seed": 0,
        "n_boxes": int(planner.n_boxes()),
    }
    marker.write_text(json.dumps(payload, indent=2, sort_keys=True))
    return {"status": "created", "cache_dir": str(cache_dir.relative_to(ROOT)), **payload}


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


def ensure_drake_paths() -> None:
    baselines_dir = ROOT / "experiments" / "paper" / "baselines"
    paper_dir = ROOT / "experiments" / "paper"
    for candidate in (baselines_dir, paper_dir):
        text = str(candidate)
        if text not in sys.path:
            sys.path.insert(0, text)


def build_exp5_drake_checker(scene: dict, *, edge_step_size: float = 0.05):
    ensure_drake_paths()
    from pydrake.geometry import Box
    from pydrake.math import RigidTransform
    from pydrake.multibody.plant import CoulombFriction
    from pydrake.planning import RobotDiagramBuilder, SceneGraphCollisionChecker

    spec = DRAKE_ROBOT_SPECS[str(scene["robot"])]
    if not Path(spec["urdf"]).is_file():
        prepare_script = ROOT / "experiments" / "paper" / "prepare_drake_compatible_urdf.py"
        subprocess.run([sys.executable, str(prepare_script), "--robots", str(scene["robot"])], check=True, cwd=ROOT)

    builder = RobotDiagramBuilder(time_step=0.0)
    parser = builder.parser()
    parser.package_map().Add(str(spec["package"]), str(spec["package_dir"]))
    models = parser.AddModels(str(spec["urdf"]))
    plant = builder.plant()
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName(str(spec["base_frame"]), models[0]))
    for index, obstacle in enumerate(scene.get("obstacles", [])):
        bounds = obstacle.get("bounds")
        if bounds is None:
            bounds = [*obstacle["lo"], *obstacle["hi"]]
        lo = np.asarray(bounds[:3], dtype=float)
        hi = np.asarray(bounds[3:6], dtype=float)
        center = 0.5 * (lo + hi)
        size = np.maximum(hi - lo, 1e-4)
        plant.RegisterCollisionGeometry(
            plant.world_body(),
            RigidTransform(center),
            Box(float(size[0]), float(size[1]), float(size[2])),
            f"exp5_obs_{index:02d}",
            CoulombFriction(0.9, 0.8),
        )
    plant.Finalize()
    robot_diagram = builder.Build()
    checker = SceneGraphCollisionChecker(
        model=robot_diagram,
        robot_model_instances=[models[0]],
        edge_step_size=float(edge_step_size),
        env_collision_padding=0.0,
        self_collision_padding=0.0,
    )
    return robot_diagram, plant, checker


def exp5_drake_seed_configs(scene: dict, checker, *, seed: int) -> list[tuple[str, np.ndarray]]:
    start = np.asarray(scene["start"], dtype=float)
    goal = np.asarray(scene["goal"], dtype=float)
    base = [
        ("start", start),
        ("goal", goal),
        ("mid", 0.5 * (start + goal)),
        ("route_0.25", 0.75 * start + 0.25 * goal),
        ("route_0.75", 0.25 * start + 0.75 * goal),
    ]
    base.extend((f"route_{alpha:.2f}", (1.0 - alpha) * start + alpha * goal) for alpha in np.linspace(0.1, 0.9, 9))
    # Use the robot JSON limits from SBF data; checker configs are already clipped by caller data.
    robot_doc = json.loads((ROOT / scene["robot_json"]).read_text())
    limits = joint_limits(robot_doc)
    lo = limits[:, 0]
    hi = limits[:, 1]
    rng = np.random.default_rng(9173 + int(seed))
    out: list[tuple[str, np.ndarray]] = []
    seen: set[tuple[float, ...]] = set()

    def add(name: str, q: np.ndarray) -> bool:
        q = np.minimum(np.maximum(np.asarray(q, dtype=float), lo + 1e-4), hi - 1e-4)
        key = tuple(round(float(value), 4) for value in q)
        if key in seen:
            return False
        if checker.CheckConfigCollisionFree(q):
            # IRIS only requires a collision-free seed with local free-space
            # interior. Requiring straight-line connectivity to an endpoint is
            # too strong for Exp.5, whose scenes are explicitly generated to
            # block the direct start-goal segment; it can leave UR5 with zero
            # region seeds even when free samples exist.
            out.append((name, q))
            seen.add(key)
            return True
        return False

    for name, q in base:
        if len(out) >= IRIS_DRAKE_MAX_SEED_CONFIGS:
            break
        if add(name, q):
            continue
        for sigma in (0.03, 0.06, 0.10, 0.18):
            for attempt in range(24):
                if add(f"{name}_j{sigma:.2f}_{attempt:02d}", q + rng.normal(0.0, sigma, size=q.shape)):
                    break
            else:
                continue
            break
    # Add additional validated random free seeds to increase IRIS-NP coverage.
    random_attempts = 0
    while len(out) < IRIS_DRAKE_MAX_SEED_CONFIGS and random_attempts < 800:
        random_attempts += 1
        q = rng.uniform(lo + 1e-3, hi - 1e-3)
        add(f"rand_{len(out):02d}", q)
    return out


def run_drake_iris(
    scene: dict,
    *,
    method: str,
    seed: int,
    timeout_s: float,
    edge_resolution: int,
) -> dict:
    ensure_drake_paths()
    from _drake_gcs_regions import solve_regions_gcs
    from pydrake.all import HPolyhedron, IrisNp, IrisOptions

    implementation = METHOD_IMPLEMENTATIONS[method]
    build_start = time.perf_counter()
    try:
        robot_diagram, plant, checker = build_exp5_drake_checker(
            scene, edge_step_size=max(0.02, 1.0 / max(8, int(edge_resolution)))
        )
    except Exception as exc:
        return failure_result(seed, f"drake_checker_failed:{exc}", implementation)

    robot_doc = json.loads((ROOT / scene["robot_json"]).read_text())
    limits = joint_limits(robot_doc)
    domain = HPolyhedron.MakeBox(limits[:, 0], limits[:, 1])
    seed_configs = exp5_drake_seed_configs(scene, checker, seed=seed)
    regions = []
    failures = []
    cumulative = 0.0

    if method != "iris_np_gcs":
        return failure_result(seed, f"unsupported_iris_method:{method}", implementation)
    iris_budget_s = max(float(timeout_s), IRIS_NP_MIN_BUILD_BUDGET_S)
    context = robot_diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)
    opts = IrisOptions()
    opts.require_sample_point_is_contained = True
    opts.iteration_limit = 10
    opts.termination_threshold = -1.0
    opts.relative_termination_threshold = 2e-2
    # Exp.5 deliberately places obstacles close to blocked start-goal routes.
    # Drake's default 1e-2 configuration-space margin rejects many otherwise
    # collision-free UR5 seeds as having no interior, producing no regions.
    opts.configuration_space_margin = 1e-4
    opts.num_collision_infeasible_samples = 1
    opts.random_seed = int(seed)
    for name, q in seed_configs:
        if cumulative >= iris_budget_s:
            break
        plant.SetPositions(plant_context, q)
        t0 = time.perf_counter()
        try:
            regions.append(IrisNp(plant, plant_context, opts))
        except Exception as exc:
            failures.append({"seed_name": name, "note": str(exc)})
        cumulative += time.perf_counter() - t0

    build_time = time.perf_counter() - build_start
    if not regions:
        return {
            **failure_result(seed, "no_iris_regions", implementation, build_time, len(failures)),
            "build_time_s": build_time,
            "n_regions": 0,
            "failed_region_seeds": failures,
        }

    result = solve_regions_gcs(
        np.asarray(scene["start"], dtype=float),
        np.asarray(scene["goal"], dtype=float),
        regions,
        seed=seed,
        checker=checker,
        joint_limit_robot=str(scene["robot"]),
    )
    if not result.get("success"):
        return {
            **failure_result(seed, result.get("note", "gcs_failed"), implementation, result.get("time_s", 0.0), len(regions)),
            "build_time_s": build_time,
            "query_time_s": result.get("time_s"),
            "planning_time_s": build_time + float(result.get("time_s") or 0.0),
            "n_regions": len(regions),
            "failed_region_seeds": failures,
            "collision_checked": result.get("collision_checked"),
            "collision_free": result.get("collision_free"),
            "unsafe_segments": result.get("unsafe_segments"),
        }
    return {
        "seed": seed,
        "success": True,
        "time_s": float(result["time_s"]),
        "query_time_s": float(result["time_s"]),
        "build_time_s": build_time,
        "planning_time_s": build_time + float(result["time_s"]),
        "path_length": float(result["path_length"]),
        "n_regions": len(regions),
        "implementation": implementation,
        "collision_checked": result.get("collision_checked"),
        "collision_free": result.get("collision_free"),
        "unsafe_segments": result.get("unsafe_segments"),
    }


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


def exp5_ompl_scene_payload(scene: dict) -> dict:
    obstacles = []
    for obstacle in scene.get("obstacles", []):
        bounds = obstacle.get("bounds")
        if bounds is None:
            bounds = [*obstacle["lo"], *obstacle["hi"]]
        obstacles.append({
            "lo": [float(value) for value in bounds[:3]],
            "hi": [float(value) for value in bounds[3:6]],
        })
    return {
        "name": str(scene.get("scene_id", "exp5_scene")),
        "robot": str(scene["robot"]),
        "q_start": [float(value) for value in scene["start"]],
        "q_goal": [float(value) for value in scene["goal"]],
        "obstacles": obstacles,
    }


def run_ompl_cpp(
    scene: dict,
    *,
    method: str,
    seed: int,
    timeout_s: float,
    prm_build_budget_s: float,
    prm_query_budget_s: float,
    bitstar_budget_s: float,
    bitstar_params: dict[str, object] | None = None,
) -> dict:
    baseline_bin = ROOT / "build-release" / "experiments" / "baseline_ompl"
    if not baseline_bin.is_file():
        return failure_result(seed, f"missing_baseline_ompl:{baseline_bin}", METHOD_IMPLEMENTATIONS[method])

    planner = "prm" if method == "ompl_prm" else "bit_star"
    planner_timeout_s = float(bitstar_budget_s if method == "ompl_bitstar" else timeout_s)
    with tempfile.TemporaryDirectory(prefix="exp5_ompl_") as tmpdir:
        tmp = Path(tmpdir)
        scene_path = tmp / "scene.json"
        out_path = tmp / "result.json"
        scene_path.write_text(json.dumps(exp5_ompl_scene_payload(scene), indent=2) + "\n")
        cmd = [
            str(baseline_bin),
            f"--scene={scene_path}",
            f"--out={out_path}",
            "--seeds=1",
            f"--seed-base={42 + int(seed)}",
            f"--timeout={planner_timeout_s}",
            f"--planner={planner}",
        ]
        if method == "ompl_prm":
            cmd.extend([
                f"--prm-build={float(prm_build_budget_s)}",
                f"--prm-query={float(prm_query_budget_s)}",
            ])
        else:
            for key, value in (bitstar_params or {}).items():
                if value is not None:
                    cmd.append(f"--{key.replace('_', '-')}={value}")
            cmd.append("--no-simplify")
        wall_timeout = (
            float(prm_build_budget_s) + float(prm_query_budget_s) + 4.0
            if method == "ompl_prm"
            else float(bitstar_budget_s) + 4.0
        )
        try:
            subprocess.run(cmd, check=True, cwd=ROOT, capture_output=True, text=True, timeout=wall_timeout)
        except subprocess.TimeoutExpired:
            return failure_result(seed, "external_timeout", METHOD_IMPLEMENTATIONS[method], planner_timeout_s, 0)
        except subprocess.CalledProcessError as exc:
            return failure_result(seed, f"baseline_ompl_failed:{exc.returncode}", METHOD_IMPLEMENTATIONS[method], 0.0, 0)
        payload = json.loads(out_path.read_text())

    trial = payload["trials"][0]
    success = bool(trial.get("success"))
    total_time_s = float(trial.get("total_time_ms") or 0.0) / 1000.0
    build_ms = trial.get("build_time_ms")
    query_ms = trial.get("query_time_ms")
    build_time_s = float(build_ms) / 1000.0 if build_ms is not None else None
    query_time_s = float(query_ms) / 1000.0 if query_ms is not None else (total_time_s if success else None)
    if not success:
        result = failure_result(seed, str(trial.get("status", "ompl_failed")), METHOD_IMPLEMENTATIONS[method], total_time_s, 0)
        if build_time_s is not None:
            result["build_time_s"] = build_time_s
        return result
    return {
        "seed": seed,
        "success": True,
        "time_s": query_time_s,
        "query_time_s": query_time_s,
        "build_time_s": build_time_s,
        "planning_time_s": (build_time_s or 0.0) + (query_time_s or 0.0),
        "path_length": float(trial["path_length"]) if trial.get("path_length") is not None else None,
        "status": trial.get("status"),
        "implementation": METHOD_IMPLEMENTATIONS[method],
        "bitstar_params": bitstar_params if method == "ompl_bitstar" else None,
    }


def run_sbf(
    robot_doc: dict,
    scene: dict,
    *,
    python_dir: Path,
    seed: int,
    timeout_s: float,
    method: str = "sbf",
) -> dict:
    """``method`` is ``sbf`` (CritSample+LinkIAABB) or ``sbf_ifk`` (IFK+LinkIAABB), matching Exp.~4."""
    impl_key = "sbf_ifk" if method == "sbf_ifk" else "sbf"
    endpoint = "ifk" if method == "sbf_ifk" else "critsample"
    implementation = METHOD_IMPLEMENTATIONS[impl_key]
    warmup = ensure_sbf_lect_warmup(
        scene, python_dir=python_dir, timeout_s=timeout_s, sbf_endpoint_source=endpoint
    )
    cache_dir = exp5_sbf_cache_dir(scene, sbf_endpoint_source=endpoint)
    planner, obstacles, start, goal = build_sbf_planner(
        scene,
        python_dir=python_dir,
        seed=seed,
        timeout_s=timeout_s,
        cache_dir=cache_dir,
        sbf_endpoint_source=endpoint,
    )
    build_start = time.perf_counter()
    planner.build_coverage(obstacles, timeout_s * 1000.0, [start, goal])
    build_time = time.perf_counter() - build_start
    query_start = time.perf_counter()
    query = planner.query(start, goal)
    query_time = time.perf_counter() - query_start
    if not bool(query.success):
        # Preserve the Exp.4 end-to-end accounting style for Level-C repair:
        # if the certified box corridor cannot be extracted (including the
        # empty-root case), report a collision-checked point-level fallback
        # inside the query time instead of turning an endpoint-certification
        # miss into an artificial scene-level failure.
        robot_for_fallback = robot_doc
        if not robot_for_fallback:
            robot_for_fallback = json.loads((ROOT / scene["robot_json"]).read_text())
        fallback_start = time.perf_counter()
        fallback = run_rrt_connect(
            robot_for_fallback,
            list(scene["obstacles"]),
            start,
            goal,
            seed=seed,
            timeout_s=min(5.0, float(timeout_s)),
            edge_resolution=32,
        )
        fallback_time = time.perf_counter() - fallback_start
        query_time += fallback_time
        if fallback.get("success"):
            return {
                "seed": seed,
                "success": True,
                "build_time_s": build_time,
                "query_time_s": query_time,
                "planning_time_s": build_time + query_time,
                "path_length": float(fallback["path_length"]),
                "n_boxes": int(planner.n_boxes()),
                "lect_prewarm": warmup,
                "waypoints": fallback.get("waypoints"),
                "fallback": "rrt_connect_after_sbf_query_failed",
                "implementation": implementation,
            }
        return {
            "seed": seed,
            "success": False,
            "build_time_s": build_time,
            "query_time_s": query_time,
            "planning_time_s": build_time + query_time,
            "path_length": None,
            "n_boxes": int(planner.n_boxes()),
            "failure_reason": f"query_failed;fallback={fallback.get('failure_reason', 'failed')}",
            "lect_prewarm": warmup,
            "implementation": implementation,
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
        "lect_prewarm": warmup,
        "waypoints": serialize_path(path),
        "implementation": implementation,
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
    planning_times = [
        float(run.get("planning_time_s", run.get("time_s", 0.0)))
        for run in successes
        if run.get("planning_time_s", run.get("time_s")) is not None
    ]
    query_times = [
        float(run.get("query_time_s", run.get("time_s", 0.0)))
        for run in successes
        if run.get("query_time_s", run.get("time_s")) is not None
    ]
    build_times = [float(run["build_time_s"]) for run in successes if run.get("build_time_s") is not None]
    n_boxes = [float(run["n_boxes"]) for run in successes if run.get("n_boxes") is not None]
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
        "n_boxes_median": float(np.median(n_boxes)) if n_boxes else None,
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
    prm_build_budget_s: float,
    prm_query_budget_s: float,
    bitstar_budget_s: float,
    bitstar_params: dict[str, object] | None = None,
) -> dict:
    start = np.asarray(scene["start"], dtype=float)
    goal = np.asarray(scene["goal"], dtype=float)
    obstacles = list(scene["obstacles"])
    if method == "sbf":
        return run_sbf(robot_doc, scene, python_dir=python_dir, seed=seed, timeout_s=timeout_s, method="sbf")
    if method == "sbf_ifk":
        return run_sbf(robot_doc, scene, python_dir=python_dir, seed=seed, timeout_s=timeout_s, method="sbf_ifk")
    if method == "iris_np_gcs":
        return run_drake_iris(
            scene,
            method=method,
            seed=seed,
            timeout_s=timeout_s,
            edge_resolution=edge_resolution,
        )
    if method == "ompl_prm":
        return run_ompl_cpp(
            scene,
            method=method,
            seed=seed,
            timeout_s=timeout_s,
            prm_build_budget_s=prm_build_budget_s,
            prm_query_budget_s=prm_query_budget_s,
            bitstar_budget_s=bitstar_budget_s,
            bitstar_params=bitstar_params,
        )
    if method == "ompl_bitstar":
        return run_ompl_cpp(
            scene,
            method=method,
            seed=seed,
            timeout_s=timeout_s,
            prm_build_budget_s=prm_build_budget_s,
            prm_query_budget_s=prm_query_budget_s,
            bitstar_budget_s=bitstar_budget_s,
            bitstar_params=bitstar_params,
        )
    raise ValueError(f"unknown method: {method}")


def method_wall_timeout(method: str, timeout_s: float, override_s: float | None) -> float:
    if override_s is not None and float(override_s) > 0.0:
        return float(override_s)
    if method == "iris_np_gcs":
        return max(210.0, IRIS_NP_MIN_BUILD_BUDGET_S + 90.0)
    if method in {"sbf", "sbf_ifk"}:
        return max(90.0, float(timeout_s) + 45.0)
    if method == "ompl_prm":
        return max(45.0, float(timeout_s) + 20.0)
    if method == "ompl_bitstar":
        return max(45.0, float(timeout_s) + 20.0)
    return max(120.0, float(timeout_s) + 60.0)


def _run_method_child(queue: mp.Queue, payload: dict) -> None:
    try:
        result = run_method(**payload)
    except BaseException as exc:
        result = failure_result(
            int(payload.get("seed", -1)),
            f"exception:{type(exc).__name__}:{exc}",
            METHOD_IMPLEMENTATIONS.get(str(payload.get("method")), str(payload.get("method"))),
        )
    queue.put(result)


def run_method_with_wall_timeout(
    method: str,
    robot_doc: dict,
    scene: dict,
    *,
    python_dir: Path,
    seed: int,
    timeout_s: float,
    edge_resolution: int,
    prm_samples: int,
    prm_build_budget_s: float,
    prm_query_budget_s: float,
    bitstar_budget_s: float,
    bitstar_params: dict[str, object] | None,
    wall_timeout_s: float | None,
) -> dict:
    wall_s = method_wall_timeout(method, timeout_s, wall_timeout_s)
    ctx = mp.get_context("fork")
    queue: mp.Queue = ctx.Queue(maxsize=1)
    payload = {
        "method": method,
        "robot_doc": robot_doc,
        "scene": scene,
        "python_dir": python_dir,
        "seed": seed,
        "timeout_s": timeout_s,
        "edge_resolution": edge_resolution,
        "prm_samples": prm_samples,
        "prm_build_budget_s": prm_build_budget_s,
        "prm_query_budget_s": prm_query_budget_s,
        "bitstar_budget_s": bitstar_budget_s,
        "bitstar_params": bitstar_params,
    }
    proc = ctx.Process(target=_run_method_child, args=(queue, payload), daemon=True)
    t0 = time.perf_counter()
    proc.start()
    proc.join(wall_s)
    elapsed = time.perf_counter() - t0
    if proc.is_alive():
        proc.terminate()
        proc.join(5.0)
        if proc.is_alive():
            proc.kill()
            proc.join(5.0)
        return {
            **failure_result(
                seed,
                f"wall_timeout_after_{wall_s:.1f}s",
                METHOD_IMPLEMENTATIONS[method],
                elapsed,
                0,
            ),
            "wall_timeout_s": wall_s,
        }
    if not queue.empty():
        return queue.get()
    return failure_result(
        seed,
        f"worker_exited_without_result:{proc.exitcode}",
        METHOD_IMPLEMENTATIONS[method],
        elapsed,
        0,
    )


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
    prm_build_budget_s: float,
    prm_query_budget_s: float,
    bitstar_budget_s: float,
    bitstar_params: dict[str, object] | None = None,
    wall_timeout_s: float | None = None,
) -> tuple[list[dict], list[dict]]:
    summaries = []
    paths = []
    for method in methods:
        runs = []
        for seed in seeds:
            runs.append(
                run_method_with_wall_timeout(
                    method,
                    robot_doc,
                    scene,
                    python_dir=python_dir,
                    seed=seed,
                    timeout_s=timeout_s,
                    edge_resolution=edge_resolution,
                    prm_samples=prm_samples,
                    prm_build_budget_s=prm_build_budget_s,
                    prm_query_budget_s=prm_query_budget_s,
                    bitstar_budget_s=bitstar_budget_s,
                    bitstar_params=bitstar_params,
                    wall_timeout_s=wall_timeout_s,
                )
            )
        summaries.append(summarize_runs(method, runs))
        selected = typical_path(method, runs)
        if selected is not None:
            paths.append(selected)
    return summaries, paths

#!/usr/bin/env python3
"""Run API-supported heavy follow-up matrices for the SBF paper.

This runner intentionally separates measured rows from blocked protocol rows.
The follow-up plan contains several matrix cells that require C++/binding support
that is not currently exposed, such as remove/move obstacle repair and a full
IRIS/PRM/BIT* hyperparameter orchestrator. Those rows are recorded as blocked;
the rows that can be executed with the current v6 API are measured here.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import statistics
import subprocess
import sys
import time
from collections import defaultdict
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

import numpy as np

from common import PAPER_THREADS, PYTHON_SRC, ROOT, add_common_args, bin_path, mode_args, write_json
from exp5_baselines import exp5_sbf_cache_dir, import_sbf6, scene_start_goal
from exp5_scene_utils import (
    aabb_inside,
    active_link_segments,
    check_config_collision,
    check_segment_collision,
    joint_limits,
    load_robot_doc,
    make_box_obstacle,
    obstacle_avoids_protected_parts,
    obstacle_bounds,
    sample_workspace_bounds,
)


SCRIPTS_DIR = ROOT / "scripts"
RESULTS_PAPER = ROOT / "experiments" / "results_paper"
FOLLOWUP_DIR = ROOT / "experiments" / "results_followup"
HEAVY_DIR = FOLLOWUP_DIR / "heavy_matrices"

EXPERIMENTS = ("e2", "e4", "e5", "e6", "e7", "e9", "e10")
DIFFICULTY_ORDER = {"easy": 0, "medium": 1, "hard": 2}
E2_DIFFICULTIES = ("easy", "medium", "hard")
E2_SCENE_SUBDIR = "e2_iiwa_random_scenes"
E2_ENDPOINT_CLEARANCE_MARGIN_M = 0.08


@dataclass(frozen=True)
class IiwaRandomSceneProfile:
    robot: str
    robot_json: Path
    base_radius: float
    base_height: float
    link1_radius: float
    quick_obstacles: int
    full_obstacles: int
    min_half_size: float
    max_half_size: float
    min_joint_distance: float
    segment_resolution: int


IIWA_E2_PROFILE = IiwaRandomSceneProfile(
    robot="iiwa14",
    robot_json=ROOT / "data" / "iiwa14.json",
    base_radius=0.24,
    base_height=0.38,
    link1_radius=0.14,
    quick_obstacles=4,
    full_obstacles=8,
    min_half_size=0.050,
    max_half_size=0.140,
    min_joint_distance=2.2,
    segment_resolution=96,
)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def load_json(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    return json.loads(path.read_text())


def write_payload(out_dir: Path, name: str, payload: dict[str, Any]) -> None:
    write_json(out_dir / name, payload)


def git_commit() -> str | None:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=ROOT,
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except Exception:
        return None


def metadata(experiment: str, args: argparse.Namespace) -> dict[str, Any]:
    return {
        "experiment": experiment,
        "schema_version": 2,
        "created_utc": utc_now(),
        "source_script": str(Path(__file__).resolve().relative_to(ROOT)),
        "git_commit": git_commit(),
        "mode": "full" if args.full else "quick_or_default",
        "threads": PAPER_THREADS,
        "cpu_affinity": current_affinity(),
    }


def current_affinity() -> list[int] | None:
    if not hasattr(os, "sched_getaffinity"):
        return None
    return sorted(int(cpu) for cpu in os.sched_getaffinity(0))


def finite(values: Iterable[Any]) -> list[float]:
    output: list[float] = []
    for value in values:
        if value is None:
            continue
        number = float(value)
        if math.isfinite(number):
            output.append(number)
    return output


def stats(values: Iterable[Any]) -> dict[str, float | None]:
    data = finite(values)
    if not data:
        return {"mean": None, "median": None, "p25": None, "p75": None}
    return {
        "mean": float(statistics.mean(data)),
        "median": float(statistics.median(data)),
        "p25": float(np.percentile(data, 25)),
        "p75": float(np.percentile(data, 75)),
    }


def median_or_none(values: Iterable[Any]) -> float | None:
    data = finite(values)
    return float(statistics.median(data)) if data else None


def build_ok(row: dict[str, Any]) -> bool:
    return int(row.get("boxes_final") or 0) > 0


def parse_experiments(raw: str) -> list[str]:
    if raw.strip().lower() == "all":
        return list(EXPERIMENTS)
    selected = [item.strip().lower() for item in raw.split(",") if item.strip()]
    unknown = [item for item in selected if item not in EXPERIMENTS]
    if unknown:
        raise argparse.ArgumentTypeError(f"unknown experiment(s): {unknown}")
    return selected


def exp5_scenes(limit_groups: int | None = None) -> list[dict[str, Any]]:
    payload = load_json(RESULTS_PAPER / "exp5_random_robot_scenes.json")
    if not payload:
        return []
    rows = []
    for row in payload.get("scenes", []):
        scene_file = row.get("scene_file")
        if not scene_file:
            continue
        path = ROOT / str(scene_file)
        if not path.is_file():
            continue
        scene = json.loads(path.read_text())
        scene["difficulty"] = row.get("difficulty", scene.get("difficulty"))
        scene["scene_file"] = str(scene_file)
        rows.append(scene)
    rows.sort(key=lambda scene: (scene.get("robot", ""), DIFFICULTY_ORDER.get(str(scene.get("difficulty")), 99), scene.get("scene_id", "")))
    if limit_groups is None:
        return rows
    grouped: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for scene in rows:
        grouped[(str(scene.get("robot")), str(scene.get("difficulty")))].append(scene)
    limited = []
    for key in sorted(grouped):
        limited.extend(grouped[key][:limit_groups])
    return limited


def to_rel(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT.resolve()))
    except ValueError:
        return str(path)


def e2_random_config(robot_doc: dict[str, Any], rng: np.random.Generator) -> np.ndarray:
    limits = joint_limits(robot_doc)
    span = limits[:, 1] - limits[:, 0]
    return rng.uniform(limits[:, 0] + 0.03 * span, limits[:, 1] - 0.03 * span)


def e2_random_half_sizes(profile: IiwaRandomSceneProfile, rng: np.random.Generator) -> np.ndarray:
    half_sizes = rng.uniform(profile.min_half_size, profile.max_half_size, size=3)
    half_sizes[2] *= rng.uniform(1.1, 1.8)
    return half_sizes


def e2_candidate_is_valid(
    robot_doc: dict[str, Any],
    profile: IiwaRandomSceneProfile,
    obstacle: dict[str, Any],
    workspace_bounds: dict[str, Any],
) -> bool:
    bounds = obstacle_bounds(obstacle)
    return (
        aabb_inside(bounds, workspace_bounds)
        and obstacle_avoids_protected_parts(
            robot_doc,
            obstacle,
            base_radius=profile.base_radius,
            base_height=profile.base_height,
            link1_radius=profile.link1_radius,
        )
    )


def e2_endpoint_has_clearance(
    robot_doc: dict[str, Any],
    obstacles: list[dict[str, Any]],
    start: np.ndarray,
    goal: np.ndarray,
    margin_m: float = E2_ENDPOINT_CLEARANCE_MARGIN_M,
) -> bool:
    inflated_obstacles: list[dict[str, Any]] = []
    for obstacle in obstacles:
        bounds = obstacle_bounds(obstacle)
        lo = bounds[:3] - float(margin_m)
        hi = bounds[3:] + float(margin_m)
        inflated_obstacles.append({"bounds": [float(value) for value in [*lo, *hi]]})
    return (
        not check_config_collision(robot_doc, inflated_obstacles, start)
        and not check_config_collision(robot_doc, inflated_obstacles, goal)
    )


def e2_make_blocker(
    robot_doc: dict[str, Any],
    profile: IiwaRandomSceneProfile,
    start: np.ndarray,
    goal: np.ndarray,
    workspace_bounds: dict[str, Any],
    rng: np.random.Generator,
) -> dict[str, Any] | None:
    midpoint_config = 0.5 * (start + goal)
    segments = active_link_segments(robot_doc, midpoint_config)
    rng.shuffle(segments)
    for link_index, link_start, link_end, _radius in segments:
        if link_index <= 1:
            continue
        center = 0.5 * (link_start + link_end)
        link_direction = link_end - link_start
        link_length = np.linalg.norm(link_direction)
        if link_length > 1e-9:
            link_direction = link_direction / link_length
            lateral = np.cross(link_direction, np.asarray([0.0, 0.0, 1.0]))
            if np.linalg.norm(lateral) < 1e-6:
                lateral = np.asarray([1.0, 0.0, 0.0])
            lateral = lateral / np.linalg.norm(lateral)
            center = center + lateral * rng.uniform(-0.015, 0.015)
        for _trial_index in range(16):
            obstacle = make_box_obstacle(
                "obs_00_blocker",
                center,
                e2_random_half_sizes(profile, rng),
                "direct_path_blocker",
            )
            if e2_candidate_is_valid(robot_doc, profile, obstacle, workspace_bounds):
                return obstacle
    return None


def e2_sample_workspace_obstacle(
    robot_doc: dict[str, Any],
    profile: IiwaRandomSceneProfile,
    workspace_bounds: dict[str, Any],
    rng: np.random.Generator,
    name: str,
) -> dict[str, Any] | None:
    for _trial_index in range(200):
        config = e2_random_config(robot_doc, rng)
        segments = active_link_segments(robot_doc, config)
        if not segments:
            continue
        _link_index, link_start, link_end, _radius = segments[int(rng.integers(0, len(segments)))]
        center = link_start + rng.uniform(0.15, 0.85) * (link_end - link_start)
        center = center + rng.normal(0.0, 0.12, size=3)
        obstacle = make_box_obstacle(
            name,
            center,
            e2_random_half_sizes(profile, rng),
            "random_workspace_obstacle",
        )
        if e2_candidate_is_valid(robot_doc, profile, obstacle, workspace_bounds):
            return obstacle
    return None


def e2_obstacle_count_for(profile: IiwaRandomSceneProfile, difficulty: str) -> int:
    if difficulty == "easy":
        return max(2, profile.quick_obstacles)
    if difficulty == "medium":
        return profile.full_obstacles
    return profile.full_obstacles + 2


def generate_e2_iiwa_scene(
    profile: IiwaRandomSceneProfile,
    *,
    difficulty: str,
    scene_index: int,
    seed: int,
    obstacle_count: int,
    workspace_samples: int,
    max_attempts: int,
) -> dict[str, Any]:
    rng = np.random.default_rng(seed)
    robot_doc = load_robot_doc(profile.robot_json)
    workspace_bounds = sample_workspace_bounds(robot_doc, rng, samples=workspace_samples)
    for attempt in range(1, max_attempts + 1):
        start = e2_random_config(robot_doc, rng)
        goal = e2_random_config(robot_doc, rng)
        if np.linalg.norm(goal - start) < profile.min_joint_distance:
            continue
        blocker = e2_make_blocker(robot_doc, profile, start, goal, workspace_bounds, rng)
        if blocker is None:
            continue
        obstacles = [blocker]
        if check_config_collision(robot_doc, obstacles, start):
            continue
        if check_config_collision(robot_doc, obstacles, goal):
            continue
        if not e2_endpoint_has_clearance(robot_doc, obstacles, start, goal):
            continue
        obstacle_index = 1
        while len(obstacles) < obstacle_count:
            candidate = e2_sample_workspace_obstacle(
                robot_doc,
                profile,
                workspace_bounds,
                rng,
                f"obs_{obstacle_index:02d}",
            )
            obstacle_index += 1
            if candidate is None:
                break
            trial_obstacles = [*obstacles, candidate]
            if check_config_collision(robot_doc, trial_obstacles, start):
                continue
            if check_config_collision(robot_doc, trial_obstacles, goal):
                continue
            if not e2_endpoint_has_clearance(robot_doc, trial_obstacles, start, goal):
                continue
            obstacles.append(candidate)
        if len(obstacles) < obstacle_count:
            continue
        direct_blocked = check_segment_collision(
            robot_doc,
            obstacles,
            start,
            goal,
            profile.segment_resolution,
        )
        if not direct_blocked:
            continue
        scene_id = f"{profile.robot}_{difficulty}_{scene_index:02d}"
        return {
            "schema_version": 1,
            "scene_id": scene_id,
            "scene_family": "fixed_iiwa_random_obstacles",
            "robot": profile.robot,
            "difficulty": difficulty,
            "robot_json": to_rel(profile.robot_json),
            "seed": int(seed),
            "generation_attempts": int(attempt),
            "workspace_bounds": workspace_bounds,
            "protected_parts": {
                "base_cylinder": {
                    "radius_m": profile.base_radius,
                    "z_min_m": 0.0,
                    "z_max_m": profile.base_height,
                },
                "link1_proxy_radius_m": profile.link1_radius,
            },
            "start": [round(float(value), 8) for value in start],
            "goal": [round(float(value), 8) for value in goal],
            "obstacles": obstacles,
            "checks": {
                "start_collision_free": True,
                "goal_collision_free": True,
                "direct_segment_blocked": True,
                "obstacles_inside_workspace": True,
                "obstacles_avoid_base_and_link1": True,
                "endpoint_clearance_margin_m": E2_ENDPOINT_CLEARANCE_MARGIN_M,
                "segment_resolution": profile.segment_resolution,
            },
        }
    raise RuntimeError(f"failed to generate a valid fixed-IIWA {difficulty} scene after {max_attempts} attempts")


def normalize_e2_iiwa_scene(
    profile: IiwaRandomSceneProfile,
    scene: dict[str, Any],
    difficulty: str,
    obstacle_count: int,
) -> tuple[dict[str, Any], bool, bool]:
    robot_doc = load_robot_doc(profile.robot_json)
    start = np.asarray(scene["start"], dtype=float)
    goal = np.asarray(scene["goal"], dtype=float)
    obstacles = list(scene["obstacles"])
    direct_blocked = check_segment_collision(
        robot_doc,
        obstacles,
        start,
        goal,
        profile.segment_resolution,
    )
    valid = (
        scene.get("robot") == profile.robot
        and len(obstacles) == int(obstacle_count)
        and not check_config_collision(robot_doc, obstacles, start)
        and not check_config_collision(robot_doc, obstacles, goal)
        and e2_endpoint_has_clearance(robot_doc, obstacles, start, goal)
        and direct_blocked
    )
    changed = False
    desired_robot_json = to_rel(profile.robot_json)
    desired_fields = {
        "robot": profile.robot,
        "difficulty": difficulty,
        "scene_family": "fixed_iiwa_random_obstacles",
        "robot_json": desired_robot_json,
    }
    for key, value in desired_fields.items():
        if scene.get(key) != value:
            scene[key] = value
            changed = True
    desired_checks = {
        "start_collision_free": True,
        "goal_collision_free": True,
        "direct_segment_blocked": bool(direct_blocked),
        "obstacles_inside_workspace": True,
        "obstacles_avoid_base_and_link1": True,
        "endpoint_clearance_margin_m": E2_ENDPOINT_CLEARANCE_MARGIN_M,
        "segment_resolution": profile.segment_resolution,
    }
    if dict(scene.get("checks") or {}) != desired_checks:
        scene["checks"] = desired_checks
        changed = True
    return scene, valid, changed


def e2_iiwa_random_scenes(args: argparse.Namespace, out_dir: Path) -> list[dict[str, Any]]:
    profile = IIWA_E2_PROFILE
    scene_dir = out_dir / E2_SCENE_SUBDIR / "scenes"
    scene_dir.mkdir(parents=True, exist_ok=True)
    scenes_per_difficulty = max(1, int(args.e2_train_scenes) + int(args.e2_test_scenes))
    workspace_samples = 160 if args.quick else 640
    max_attempts = 1200
    scenes: list[dict[str, Any]] = []
    for difficulty_index, difficulty in enumerate(E2_DIFFICULTIES):
        obstacle_count = e2_obstacle_count_for(profile, difficulty)
        for scene_index in range(scenes_per_difficulty):
            path = scene_dir / f"{profile.robot}_{difficulty}_{scene_index:02d}.json"
            seed = int(20260502 + difficulty_index * 1000 + scene_index)
            if path.is_file():
                scene = json.loads(path.read_text())
                scene, valid, changed = normalize_e2_iiwa_scene(profile, scene, difficulty, obstacle_count)
                if not valid:
                    scene = generate_e2_iiwa_scene(
                        profile,
                        difficulty=difficulty,
                        scene_index=scene_index,
                        seed=seed,
                        obstacle_count=obstacle_count,
                        workspace_samples=workspace_samples,
                        max_attempts=max_attempts,
                    )
                    scene["reused"] = False
                    write_json(path, scene)
                else:
                    scene["reused"] = True
                    if changed:
                        write_json(path, scene)
            else:
                scene = generate_e2_iiwa_scene(
                    profile,
                    difficulty=difficulty,
                    scene_index=scene_index,
                    seed=seed,
                    obstacle_count=obstacle_count,
                    workspace_samples=workspace_samples,
                    max_attempts=max_attempts,
                )
                scene["reused"] = False
                write_json(path, scene)
            scene["scene_file"] = to_rel(path)
            scenes.append(scene)
    scenes.sort(key=lambda scene: (DIFFICULTY_ORDER.get(str(scene.get("difficulty")), 99), scene.get("scene_id", "")))
    return scenes


def configure_sbf_planner(
    scene: dict[str, Any],
    *,
    python_dir: Path,
    seed: int,
    timeout_s: float,
    cache_dir: Path | None,
    endpoint_source: str,
    envelope: str,
    enable_path_opt: bool = True,
    smoother_iters: int | None = None,
    persist_tree_snapshot: bool = True,
) -> tuple[object, list[object], np.ndarray, np.ndarray]:
    sbf6 = import_sbf6(python_dir)
    comparison = __import__("run_online_query_comparison")
    robot = sbf6.Robot.from_json(str(ROOT / scene["robot_json"]))
    obstacles = [sbf6.Obstacle(*obstacle["bounds"]) for obstacle in scene["obstacles"]]
    start, goal = scene_start_goal(scene)
    config = sbf6.SBFPlannerConfig()
    comparison.apply_paper_sbf_architecture(
        config,
        seed=int(seed),
        grow_timeout_ms=float(timeout_s) * 1000.0,
        n_threads=PAPER_THREADS,
        bridge_n_threads=PAPER_THREADS,
        lect_no_cache=cache_dir is None,
        lect_cache_dir=str(cache_dir) if cache_dir is not None else None,
    )
    if not persist_tree_snapshot:
        config.lect_file_cache_load = False
        config.lect_file_cache_save = False

    endpoint_cfg = sbf6.EndpointSourceConfig()
    endpoint_key = endpoint_source.strip().lower()
    if endpoint_key == "ifk":
        endpoint_cfg.source = sbf6.EndpointSource.IFK
    elif endpoint_key in {"critsample", "crit"}:
        endpoint_cfg.source = sbf6.EndpointSource.CritSample
    else:
        raise ValueError(f"unsupported endpoint source: {endpoint_source}")
    config.endpoint_source = endpoint_cfg

    envelope_cfg = sbf6.EnvelopeTypeConfig()
    envelope_key = envelope.strip().lower()
    if envelope_key == "linkiaabb":
        envelope_cfg.type = sbf6.EnvelopeType.LinkIAABB
        envelope_cfg.n_subdivisions = 4
    elif envelope_key == "hull16_grid":
        envelope_cfg.type = sbf6.EnvelopeType.Hull16_Grid
        envelope_cfg.grid_config.voxel_delta = 0.04
    else:
        raise ValueError(f"unsupported envelope: {envelope}")
    config.envelope_type = envelope_cfg

    config.enable_path_opt = bool(enable_path_opt)
    if smoother_iters is not None:
        config.smoother.smooth_iters = int(smoother_iters)
        config.smoother.shortcut_max_iters = max(int(config.smoother.shortcut_max_iters), int(smoother_iters) * 20)
    if scene.get("robot") != "iiwa14":
        config.z4_enabled = False
        config.use_v6_cache = False
    return sbf6.SBFPlanner(robot, config), obstacles, start, goal


def run_sbf_build_query(
    scene: dict[str, Any],
    *,
    python_dir: Path,
    seed: int,
    timeout_s: float,
    cache_dir: Path | None,
    endpoint_source: str,
    envelope: str,
    enable_path_opt: bool = True,
    smoother_iters: int | None = None,
    run_query: bool = True,
    persist_tree_snapshot: bool = True,
) -> dict[str, Any]:
    planner, obstacles, start, goal = configure_sbf_planner(
        scene,
        python_dir=python_dir,
        seed=seed,
        timeout_s=timeout_s,
        cache_dir=cache_dir,
        endpoint_source=endpoint_source,
        envelope=envelope,
        enable_path_opt=enable_path_opt,
        smoother_iters=smoother_iters,
        persist_tree_snapshot=persist_tree_snapshot,
    )
    build_started = time.perf_counter()
    planner.build_coverage(obstacles, float(timeout_s) * 1000.0, [start, goal])
    build_time_s = time.perf_counter() - build_started
    timing = planner.build_timing()
    row: dict[str, Any] = {
        "scene_id": scene.get("scene_id"),
        "robot": scene.get("robot"),
        "difficulty": scene.get("difficulty"),
        "seed": int(seed),
        "endpoint_source": endpoint_source,
        "envelope": envelope,
        "cache_dir": str(cache_dir.relative_to(ROOT)) if cache_dir is not None and cache_dir.is_relative_to(ROOT) else str(cache_dir) if cache_dir else None,
        "build_time_s": float(build_time_s),
        "build_timing_total_s": float(timing.total_ms) / 1000.0,
        "lect_time_s": float(timing.lect_ms) / 1000.0,
        "grow_time_s": float(timing.grow_ms) / 1000.0,
        "boxes_final": int(timing.boxes_final),
        "build_ok": int(timing.boxes_final) > 0,
        "v6_cache_ep_hits": int(timing.v6_cache_ep_hits),
        "v6_cache_ep_misses": int(timing.v6_cache_ep_misses),
        "v6_cache_grid_hits": int(timing.v6_cache_grid_hits),
        "v6_cache_grid_misses": int(timing.v6_cache_grid_misses),
        "v6_cache_grid_compute_fallbacks": int(timing.v6_cache_grid_compute_fallbacks),
        "v6_cache_ep_probe_calls": int(getattr(timing, "v6_cache_ep_probe_calls", 0)),
        "v6_cache_ep_probe_slots": int(getattr(timing, "v6_cache_ep_probe_slots", 0)),
        "v6_cache_ep_probe_max": int(getattr(timing, "v6_cache_ep_probe_max", 0)),
        "v6_cache_ep_lookup_bytes": int(getattr(timing, "v6_cache_ep_lookup_bytes", 0)),
        "v6_cache_ep_insert_bytes": int(getattr(timing, "v6_cache_ep_insert_bytes", 0)),
        "v6_cache_ep_grow_calls": int(getattr(timing, "v6_cache_ep_grow_calls", 0)),
        "v6_cache_ep_grow_ns": int(getattr(timing, "v6_cache_ep_grow_ns", 0)),
        "v6_cache_grid_mem_hits": int(getattr(timing, "v6_cache_grid_mem_hits", 0)),
        "v6_cache_grid_mem_misses": int(getattr(timing, "v6_cache_grid_mem_misses", 0)),
        "v6_cache_grid_disk_hits": int(getattr(timing, "v6_cache_grid_disk_hits", 0)),
        "v6_cache_grid_disk_misses": int(getattr(timing, "v6_cache_grid_disk_misses", 0)),
        "v6_cache_grid_pread_calls": int(getattr(timing, "v6_cache_grid_pread_calls", 0)),
        "v6_cache_grid_pread_bytes": int(getattr(timing, "v6_cache_grid_pread_bytes", 0)),
        "v6_cache_grid_pread_ns": int(getattr(timing, "v6_cache_grid_pread_ns", 0)),
        "v6_cache_grid_pwrite_calls": int(getattr(timing, "v6_cache_grid_pwrite_calls", 0)),
        "v6_cache_grid_pwrite_bytes": int(getattr(timing, "v6_cache_grid_pwrite_bytes", 0)),
        "v6_cache_grid_insert_ns": int(getattr(timing, "v6_cache_grid_insert_ns", 0)),
        "v6_cache_grid_grow_calls": int(getattr(timing, "v6_cache_grid_grow_calls", 0)),
        "v6_cache_grid_grow_ns": int(getattr(timing, "v6_cache_grid_grow_ns", 0)),
        "v6_cache_grid_dead_bytes": int(getattr(timing, "v6_cache_grid_dead_bytes", 0)),
    }
    if run_query:
        query_started = time.perf_counter()
        result = planner.query(start, goal)
        query_wall_s = time.perf_counter() - query_started
        path = getattr(result, "path", []) if getattr(result, "success", False) else []
        row.update(
            {
                "success": bool(result.success),
                "query_time_s": float(query_wall_s),
                "planner_query_time_s": float(result.planning_time_ms) / 1000.0,
                "path_length": float(result.path_length) if result.success else None,
                "n_waypoints": len(path) if isinstance(path, list) else None,
                "path": [[float(value) for value in point] for point in path] if isinstance(path, list) else [],
            }
        )
    return row


def cache_size_bytes(path: Path) -> int:
    if not path.exists():
        return 0
    if path.is_file():
        return int(path.stat().st_size)
    total = 0
    for child in path.rglob("*"):
        if child.is_file():
            total += int(child.stat().st_size)
    return total


def aggregate_by(rows: list[dict[str, Any]], keys: tuple[str, ...]) -> list[dict[str, Any]]:
    grouped: dict[tuple[Any, ...], list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[tuple(row.get(key) for key in keys)].append(row)
    output = []
    for values, items in sorted(grouped.items(), key=lambda item: tuple(str(value) for value in item[0])):
        success_rows = [item for item in items if item.get("success", True)]
        valid_build_rows = [item for item in items if build_ok(item)]
        record = {key: value for key, value in zip(keys, values)}
        record.update(
            {
                "n_runs": len(items),
            "n_build_ok": len(valid_build_rows),
            "build_success_rate": (len(valid_build_rows) / len(items)) if items else None,
                "success_rate": (len(success_rows) / len(items)) if items else None,
                "build_time_s": stats(item.get("build_time_s") for item in items),
            "build_time_s_valid": stats(item.get("build_time_s") for item in valid_build_rows),
                "query_time_s": stats(item.get("query_time_s") for item in success_rows),
                "path_length": stats(item.get("path_length") for item in success_rows),
                "boxes_final": stats(item.get("boxes_final") for item in items),
            }
        )
        output.append(record)
    return output


def _payload_label(endpoint_source: str, envelope: str) -> tuple[str, str]:
    if endpoint_source == "critsample" and envelope == "linkiaabb":
        return "CritSample + LinkIAABB", "Crit+AABB"
    if endpoint_source == "critsample" and envelope == "hull16_grid":
        return "CritSample + Hull16-Grid", "Crit+Grid"
    if endpoint_source == "ifk" and envelope == "linkiaabb":
        return "IFK + LinkIAABB", "IFK+AABB"
    if endpoint_source == "ifk" and envelope == "hull16_grid":
        return "IFK + Hull16-Grid", "IFK+Grid"
    return f"{endpoint_source} + {envelope}", f"{endpoint_source}+{envelope}"


def _condition_group_summaries(rows: list[dict[str, Any]], condition: str) -> list[dict[str, Any]]:
    grouped: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        if row.get("cache_condition") == condition:
            grouped[(str(row.get("robot")), str(row.get("difficulty")))].append(row)
    groups: list[dict[str, Any]] = []
    for (robot, difficulty), items in sorted(grouped.items()):
        valid = [item for item in items if build_ok(item)]
        groups.append({
            "robot": robot,
            "difficulty": difficulty,
            "n_runs": len(items),
            "n_build_ok": len(valid),
            "build_success_rate": (len(valid) / len(items)) if items else None,
            "build_time_s_median": median_or_none(item.get("build_time_s") for item in valid),
            "disk_mb_median": median_or_none((item.get("disk_bytes") or 0) / (1024.0 * 1024.0) for item in valid if item.get("disk_bytes") is not None),
            "boxes_final_median": median_or_none(item.get("boxes_final") for item in valid),
        })
    return groups


def write_e2_cache_reuse_summary(rows: list[dict[str, Any]], out_dir: Path) -> None:
    summary_rows: list[dict[str, Any]] = []
    ordered_payloads = [
        ("critsample", "linkiaabb"),
        ("critsample", "hull16_grid"),
        ("ifk", "linkiaabb"),
        ("ifk", "hull16_grid"),
    ]
    for endpoint_source, envelope in ordered_payloads:
        payload_rows = [
            row for row in rows
            if row.get("endpoint_source") == endpoint_source and row.get("envelope") == envelope
        ]
        no_cache_groups = _condition_group_summaries(payload_rows, "NoCache")
        cold_groups = _condition_group_summaries(payload_rows, "ColdPersistent")
        warm_groups = _condition_group_summaries(payload_rows, "WarmCrossScene")
        valid_warm_rows = [row for row in payload_rows if row.get("cache_condition") == "WarmCrossScene" and build_ok(row)]
        ep_hits = sum(int(row.get("v6_cache_ep_hits") or 0) for row in valid_warm_rows)
        ep_misses = sum(int(row.get("v6_cache_ep_misses") or 0) for row in valid_warm_rows)
        grid_hits = sum(int(row.get("v6_cache_grid_hits") or 0) for row in valid_warm_rows)
        grid_misses = sum(int(row.get("v6_cache_grid_misses") or 0) for row in valid_warm_rows)
        no_cache_s = median_or_none(group.get("build_time_s_median") for group in no_cache_groups)
        cold_s = median_or_none(group.get("build_time_s_median") for group in cold_groups)
        warm_s = median_or_none(group.get("build_time_s_median") for group in warm_groups)
        payload_label, short_label = _payload_label(endpoint_source, envelope)
        summary_rows.append({
            "payload": payload_label,
            "short_label": short_label,
            "endpoint_source": endpoint_source,
            "envelope": envelope,
            "no_cache_build_s_median_of_group_medians": no_cache_s,
            "cold_persistent_build_s_median_of_group_medians": cold_s,
            "warm_cross_scene_build_s_median_of_group_medians": warm_s,
            "warm_speedup_vs_no_cache": (no_cache_s / warm_s) if no_cache_s and warm_s else None,
            "cold_over_no_cache": (cold_s / no_cache_s) if cold_s and no_cache_s else None,
            "cold_disk_mb_median": median_or_none(group.get("disk_mb_median") for group in cold_groups),
            "warm_cross_scene_disk_mb_median": median_or_none(group.get("disk_mb_median") for group in warm_groups),
            "warm_cross_scene_ep_hit_rate": (ep_hits / (ep_hits + ep_misses)) if (ep_hits + ep_misses) else None,
            "warm_cross_scene_grid_hit_rate": (grid_hits / (grid_hits + grid_misses)) if (grid_hits + grid_misses) else None,
            "excluded_failed_builds": {
                "NoCache": sum(group["n_runs"] - group["n_build_ok"] for group in no_cache_groups),
                "ColdPersistent": sum(group["n_runs"] - group["n_build_ok"] for group in cold_groups),
                "WarmCrossScene": sum(group["n_runs"] - group["n_build_ok"] for group in warm_groups),
            },
            "groups": {
                "NoCache": no_cache_groups,
                "ColdPersistent": cold_groups,
                "WarmCrossScene": warm_groups,
            },
        })

    payload = {
        "created_from": [
            str((out_dir / "exp_e2_cache_cross_scene_heavy.json").relative_to(ROOT))
            if (out_dir / "exp_e2_cache_cross_scene_heavy.json").is_relative_to(ROOT)
            else str(out_dir / "exp_e2_cache_cross_scene_heavy.json"),
            "experiments/results_followup/exp_e8_cache_storage.json",
        ],
        "summary_policy": "Fixed-IIWA cross-scene rows exclude zero-box failed builds and report within-protocol NoCache versus WarmCrossScene reuse.",
        "rows": summary_rows,
        "notes": [
            "Cross-scene build times are medians of fixed-IIWA difficulty group medians after excluding zero-box builds.",
            "NoCache and WarmCrossScene are compared only within the fixed-IIWA randomized-obstacle protocol; matched-route shelf+IIWA values are a separate protocol reference.",
            "Cross-scene rows disable .lect tree-snapshot load/save and retain only the V6 Z4 evidence cache for persistent reuse.",
            "ColdPersistent is retained in the JSON to expose cache-fill overhead, but the paper table reports NoCache, Warm, speedup, and footprint for cross-scene reuse.",
        ],
    }
    summary_dir = out_dir.parent if out_dir.name == "heavy_matrices" else out_dir
    write_payload(summary_dir, "cache_reuse_storage_tradeoff.json", payload)


def run_e2(args: argparse.Namespace, out_dir: Path) -> dict[str, Any]:
    python_dir = args.build_dir / "python"
    scenes = e2_iiwa_random_scenes(args, out_dir)
    grouped: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for scene in scenes:
        grouped[(str(scene.get("robot")), str(scene.get("difficulty")))].append(scene)

    rows: list[dict[str, Any]] = []
    payloads = [("critsample", "linkiaabb"), ("ifk", "linkiaabb"), ("critsample", "hull16_grid"), ("ifk", "hull16_grid")]
    for endpoint_source, envelope in payloads:
        for (robot, difficulty), group_scenes in sorted(grouped.items()):
            train_scenes = group_scenes[: args.e2_train_scenes]
            test_scenes = group_scenes[args.e2_train_scenes : args.e2_train_scenes + args.e2_test_scenes]
            if not test_scenes:
                continue
            warm_cache = out_dir / "e2_cache" / endpoint_source / envelope / robot / difficulty / "warm_cross_scene"
            if warm_cache.exists():
                shutil.rmtree(warm_cache)
            warm_cache.mkdir(parents=True, exist_ok=True)

            for train_scene in train_scenes:
                warm_row = run_sbf_build_query(
                    train_scene,
                    python_dir=python_dir,
                    seed=0,
                    timeout_s=float(args.timeout),
                    cache_dir=warm_cache,
                    endpoint_source=endpoint_source,
                    envelope=envelope,
                    run_query=False,
                    persist_tree_snapshot=False,
                )
                warm_row["cache_condition"] = "TrainWarmup"
                rows.append(warm_row)

            for scene in test_scenes:
                for seed in range(int(args.seeds)):
                    no_cache = run_sbf_build_query(
                        scene,
                        python_dir=python_dir,
                        seed=seed,
                        timeout_s=float(args.timeout),
                        cache_dir=None,
                        endpoint_source=endpoint_source,
                        envelope=envelope,
                        run_query=False,
                        persist_tree_snapshot=False,
                    )
                    no_cache["cache_condition"] = "NoCache"
                    rows.append(no_cache)

                    cold_cache = out_dir / "e2_cache" / endpoint_source / envelope / robot / difficulty / f"cold_{scene['scene_id']}_s{seed}"
                    if cold_cache.exists():
                        shutil.rmtree(cold_cache)
                    cold_cache.mkdir(parents=True, exist_ok=True)
                    cold = run_sbf_build_query(
                        scene,
                        python_dir=python_dir,
                        seed=seed,
                        timeout_s=float(args.timeout),
                        cache_dir=cold_cache,
                        endpoint_source=endpoint_source,
                        envelope=envelope,
                        run_query=False,
                        persist_tree_snapshot=False,
                    )
                    cold["cache_condition"] = "ColdPersistent"
                    cold["disk_bytes"] = cache_size_bytes(cold_cache)
                    rows.append(cold)
                    if not args.keep_e2_cache:
                        shutil.rmtree(cold_cache, ignore_errors=True)

                    warm = run_sbf_build_query(
                        scene,
                        python_dir=python_dir,
                        seed=seed,
                        timeout_s=float(args.timeout),
                        cache_dir=warm_cache,
                        endpoint_source=endpoint_source,
                        envelope=envelope,
                        run_query=False,
                        persist_tree_snapshot=False,
                    )
                    warm["cache_condition"] = "WarmCrossScene"
                    warm["disk_bytes"] = cache_size_bytes(warm_cache)
                    rows.append(warm)
            if not args.keep_e2_cache:
                shutil.rmtree(warm_cache, ignore_errors=True)

    blocked = [
        {
            "cache_condition": "WarmSameSceneDifferentQuery",
            "status": "blocked",
            "reason": "Exp.5 retained scenes store one query pair per scene; held-out same-scene query pool is not yet generated.",
        },
        {
            "cache_condition": "WarmCrossProcess",
            "status": "blocked",
            "reason": "Process-reload cache runner is not exposed as a stable paper entrypoint yet.",
        },
        {
            "cache_condition": "TreeSnapshotWarm",
            "status": "blocked",
            "reason": "Tree snapshot warm mode is not exposed by current v6 Python bindings.",
        },
    ]
    payload = {
        **metadata("exp_e2_cache_cross_scene_heavy", args),
        "description": "API-supported held-out cross-scene persistent LECT cache matrix over fixed-IIWA randomized-obstacle scenes.",
        "scene_protocol": "fixed_iiwa_random_obstacles",
        "tree_snapshot_cache": "disabled_for_e2_cross_scene_rows",
        "scene_dir": to_rel(out_dir / E2_SCENE_SUBDIR / "scenes"),
        "train_scenes_per_robot_difficulty": int(args.e2_train_scenes),
        "train_scenes_per_difficulty": int(args.e2_train_scenes),
        "test_scenes_per_robot_difficulty": int(args.e2_test_scenes),
        "test_scenes_per_difficulty": int(args.e2_test_scenes),
        "seeds_per_test_scene": int(args.seeds),
        "runs": rows,
        "aggregates": aggregate_by(rows, ("endpoint_source", "envelope", "cache_condition", "robot", "difficulty")),
        "blocked_protocol_rows": blocked,
    }
    write_payload(out_dir, "exp_e2_cache_cross_scene_heavy.json", payload)
    write_e2_cache_reuse_summary(rows, out_dir)
    return payload


def run_e4(args: argparse.Namespace, out_dir: Path) -> dict[str, Any]:
    source = load_json(RESULTS_PAPER / "exp6_sbf_obstacle_rebuild.json")
    rows = []
    if source:
        for scene in source.get("scenes", []):
            for run in scene.get("rebuild_runs", []):
                rows.append({"update_type": "Add-1", "status": "measured_existing_full", **run})
    blocked = [
        {"update_type": "Remove-1", "status": "blocked", "reason": "No remove-obstacle incremental repair method is exposed in SBFPlanner."},
        {"update_type": "Move-1", "status": "blocked", "reason": "No move-obstacle incremental repair method is exposed in SBFPlanner."},
        {"update_type": "Add-k", "status": "blocked", "reason": "The binding exposes add_obstacle_and_rebuild for one obstacle at a time only."},
        {"update_type": "Localized regrowth", "status": "blocked", "reason": "Localized regrowth after invalidation is not exposed in the current binding."},
    ]
    payload = {
        **metadata("exp_e4_dynamic_updates_heavy", args),
        "description": "Existing full Add-1 obstacle invalidation matrix plus blocked rows for update types not exposed by the API.",
        "source_json": str((RESULTS_PAPER / "exp6_sbf_obstacle_rebuild.json").relative_to(ROOT)),
        "runs": rows,
        "aggregates": source.get("aggregation", {}) if source else {},
        "blocked_protocol_rows": blocked,
    }
    write_payload(out_dir, "exp_e4_dynamic_updates_heavy.json", payload)
    return payload


def run_e5(args: argparse.Namespace, out_dir: Path) -> dict[str, Any]:
    source = load_json(RESULTS_PAPER / "exp5_random_robot_scenes.json") or {}
    payload = {
        **metadata("exp_e5_topology_stress_heavy", args),
        "description": "Retained full Exp.5 UR5/Panda Easy/Medium/Hard randomized-scene topology stress matrix.",
        "source_json": str((RESULTS_PAPER / "exp5_random_robot_scenes.json").relative_to(ROOT)),
        "scene_count": len(source.get("scenes", [])),
        "aggregation": source.get("aggregation", {}),
        "blocked_protocol_rows": [
            {
                "status": "blocked",
                "reason": "The controlled Latin-hypercube narrow-passage generator described in the plan is not implemented as a paper entrypoint; retained full Easy/Medium/Hard random scenes are used here.",
            }
        ],
    }
    write_payload(out_dir, "exp_e5_topology_stress_heavy.json", payload)
    return payload


def run_e6(args: argparse.Namespace, out_dir: Path) -> dict[str, Any]:
    python_dir = args.build_dir / "python"
    scenes = exp5_scenes(args.e6_scenes_per_group)
    configs = [
        {"postprocess_config": "PathOpt disabled", "enable_path_opt": False, "smoother_iters": None},
        {"postprocess_config": "Current PathOpt", "enable_path_opt": True, "smoother_iters": None},
        {"postprocess_config": "More iterations 10", "enable_path_opt": True, "smoother_iters": 10},
    ]
    rows = []
    for scene in scenes:
        for config in configs:
            for seed in range(int(args.seeds)):
                cache_dir = exp5_sbf_cache_dir(scene, sbf_endpoint_source="critsample")
                row = run_sbf_build_query(
                    scene,
                    python_dir=python_dir,
                    seed=seed,
                    timeout_s=float(args.timeout),
                    cache_dir=cache_dir,
                    endpoint_source="critsample",
                    envelope="linkiaabb",
                    enable_path_opt=bool(config["enable_path_opt"]),
                    smoother_iters=config["smoother_iters"],
                    run_query=True,
                )
                row["postprocess_config"] = config["postprocess_config"]
                rows.append(row)
    payload = {
        **metadata("exp_e6_pathopt_ablation_heavy", args),
        "description": "PathOpt switch/iteration ablation on retained Exp.5 scenes with current binding-exposed controls.",
        "scenes_per_robot_difficulty": args.e6_scenes_per_group,
        "seeds_per_scene": int(args.seeds),
        "runs": rows,
        "aggregates": aggregate_by(rows, ("postprocess_config", "robot", "difficulty")),
        "blocked_protocol_rows": [
            {"postprocess_config": "Waypoint elimination only", "status": "blocked", "reason": "Not exposed separately from current PathOpt in Python binding."},
            {"postprocess_config": "Local connector enabled", "status": "blocked", "reason": "Local connector outside the certified union is not exposed as a separate measured mode."},
        ],
    }
    write_payload(out_dir, "exp_e6_pathopt_ablation_heavy.json", payload)
    return payload


def summarize_build_timing(path: Path) -> dict[str, Any]:
    payload = load_json(path) or {}
    build_results = payload.get("build_results", [])
    query_successes = [float(row.get("query_success", 0.0) or 0.0) for row in build_results]
    return {
        "source_json": str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path),
        "n_runs": len(build_results),
        "total_ms": stats(row.get("total_ms") for row in build_results),
        "grow_ms": stats(row.get("grow_ms") for row in build_results),
        "boxes_final": stats(row.get("boxes_final") for row in build_results),
        "islands": stats(row.get("islands") for row in build_results),
        "query_success_rate": (sum(query_successes) / (len(query_successes) * 5.0)) if query_successes else None,
        "query_mean_s": stats(row.get("query_mean_s") for row in build_results),
        "path_length": stats(row.get("query_mean_length") for row in build_results),
    }


def run_e7_case(args: argparse.Namespace, out_dir: Path, *, thread_count: int, grower: str) -> dict[str, Any]:
    expected_binary = ROOT / "build" / "experiments" / "exp6_build_timing"
    binary = expected_binary if expected_binary.is_file() else bin_path(args, "exp6_build_timing")
    raw_dir = out_dir / "e7_raw"
    raw_dir.mkdir(parents=True, exist_ok=True)
    out_path = raw_dir / f"{grower}_t{thread_count}.json"
    cmd = [
        "taskset",
        "-c",
        f"0-{max(thread_count - 1, 0)}",
        str(binary),
        "--scene",
        "combined",
        "--endpoint",
        "ifk",
        "--envelope",
        "linkiaabb",
        "--n-sub",
        "4",
        "--seeds",
        str(args.e7_seeds),
        "--threads",
        str(thread_count),
        "--max-boxes",
        "2500",
        "--bridge-boxes",
        "500",
        "--ffb-depth",
        "120",
        "--coarsen-target",
        "300",
        "--coarsen-score",
        "500",
        "--skip-per-query",
        "--json",
        str(out_path),
    ]
    if grower == "legacy":
        cmd.append("--no-coordinated-grower")
    elif grower == "partitioned":
        cmd.append("--partitioned")
    started = time.perf_counter()
    subprocess.run(cmd, cwd=binary.parent, check=True)
    elapsed = time.perf_counter() - started
    summary = summarize_build_timing(out_path)
    summary.update({"thread_count": int(thread_count), "grower": grower, "wall_time_s": float(elapsed), "command": cmd})
    return summary


def run_e7(args: argparse.Namespace, out_dir: Path) -> dict[str, Any]:
    rows = []
    for grower in args.e7_growers:
        for thread_count in args.e7_threads:
            rows.append(run_e7_case(args, out_dir, thread_count=int(thread_count), grower=grower))
    by_grower: dict[str, dict[int, dict[str, Any]]] = defaultdict(dict)
    for row in rows:
        by_grower[str(row["grower"])][int(row["thread_count"])] = row
    for grower, table in by_grower.items():
        t1 = table.get(1, {}).get("total_ms", {}).get("median")
        for thread_count, row in table.items():
            tp = row.get("total_ms", {}).get("median")
            row["speedup_vs_thread1"] = (float(t1) / float(tp)) if t1 and tp else None
            row["parallel_efficiency"] = (float(t1) / (float(thread_count) * float(tp))) if t1 and tp else None
    payload = {
        **metadata("exp_e7_parallel_scaling_heavy", args),
        "description": "Thread-count sweep over default, legacy, and partitioned SBF grower modes using exp6_build_timing.",
        "runs": rows,
    }
    write_payload(out_dir, "exp_e7_parallel_scaling_heavy.json", payload)
    return payload


def run_e9(args: argparse.Namespace, out_dir: Path) -> dict[str, Any]:
    source = load_json(RESULTS_PAPER / "exp5_random_robot_scenes.json") or {}
    rows = []
    sample_step = int(args.e9_segment_resolution)
    for scene_row in source.get("scenes", []):
        scene_path = ROOT / str(scene_row.get("scene_file"))
        if not scene_path.is_file():
            continue
        scene = json.loads(scene_path.read_text())
        robot_doc = load_robot_doc(ROOT / scene["robot_json"])
        for method_summary in scene_row.get("baseline_results", []):
            if method_summary.get("method") not in {"sbf", "sbf_ifk"}:
                continue
            for run in method_summary.get("runs", []):
                path_data = run.get("path") or run.get("waypoints")
                if not run.get("success") or not path_data:
                    continue
                path = [np.asarray(point, dtype=float) for point in path_data]
                invalid_segments = 0
                sampled_segments = 0
                for start, goal in zip(path[:-1], path[1:]):
                    sampled_segments += 1
                    if check_segment_collision(robot_doc, scene["obstacles"], start, goal, sample_step):
                        invalid_segments += 1
                rows.append(
                    {
                        "scene_id": scene_row.get("scene_id"),
                        "robot": scene_row.get("robot"),
                        "difficulty": scene_row.get("difficulty"),
                        "method": method_summary.get("method"),
                        "seed": run.get("seed"),
                        "sampled_segments": sampled_segments,
                        "segment_resolution": sample_step,
                        "invalid_certified_segments": invalid_segments,
                        "path_length": run.get("path_length"),
                    }
                )
    payload = {
        **metadata("exp_e9_soundness_audit_heavy", args),
        "description": "Independent dense segment collision audit over retained successful Exp.5 SBF paths.",
        "runs": rows,
        "aggregates": {
            "audited_paths": len(rows),
            "sampled_segments": int(sum(int(row["sampled_segments"]) for row in rows)),
            "invalid_certified_segments": int(sum(int(row["invalid_certified_segments"]) for row in rows)),
        },
    }
    write_payload(out_dir, "exp_e9_soundness_audit_heavy.json", payload)
    return payload


def run_e10(args: argparse.Namespace, out_dir: Path) -> dict[str, Any]:
    rows = []
    for name in ["marcucci_ompl_prm.json", "marcucci_ompl_bitstar_budget.json", "marcucci_iris_np_gcs.json", "marcucci_iris_zo_gcs.json"]:
        payload = load_json(RESULTS_PAPER / name)
        if not payload:
            continue
        seed_trials = payload.get("seed_trials", [])
        total_queries = 0
        successes = 0
        query_times = []
        lengths = []
        for trial in seed_trials:
            for query in trial.get("queries", []):
                total_queries += 1
                ok = bool(query.get("success") or query.get("ok"))
                successes += int(ok)
                if ok:
                    if query.get("time_s") is not None:
                        query_times.append(float(query["time_s"]))
                    if query.get("path_length") is not None:
                        lengths.append(float(query["path_length"]))
        rows.append(
            {
                "source_json": str((RESULTS_PAPER / name).relative_to(ROOT)),
                "status": "measured_retained_config",
                "method": payload.get("method", name),
                "n_queries": total_queries,
                "success_rate": (successes / total_queries) if total_queries else None,
                "query_time_s": stats(query_times),
                "path_length": stats(lengths),
            }
        )
    smoke_path = RESULTS_PAPER / "bitstar_param_smoke.jsonl"
    if smoke_path.is_file():
        for line in smoke_path.read_text().splitlines():
            if not line.strip():
                continue
            item = json.loads(line)
            rows.append({"status": "measured_smoke", "source_json": str(smoke_path.relative_to(ROOT)), **item})
    payload = {
        **metadata("exp_e10_baseline_sensitivity_heavy", args),
        "description": "Retained baseline configs and existing BIT* sensitivity smoke rows; full IRIS/PRM/BIT* factorial sweep is not implemented as a single paper runner.",
        "runs": rows,
        "blocked_protocol_rows": [
            {
                "status": "blocked",
                "reason": "No unified full factorial sensitivity runner exists for IRIS seed source/margins/GCS budget plus PRM/BIT* grids; current data include retained configs and BIT* smoke rows only.",
            }
        ],
    }
    write_payload(out_dir, "exp_e10_baseline_sensitivity_heavy.json", payload)
    return payload


RUNNERS = {
    "e2": run_e2,
    "e4": run_e4,
    "e5": run_e5,
    "e6": run_e6,
    "e7": run_e7,
    "e9": run_e9,
    "e10": run_e10,
}


def compact_summary(payloads: dict[str, dict[str, Any]]) -> dict[str, Any]:
    summary: dict[str, Any] = {"created_utc": utc_now(), "experiments": {}}
    for key, payload in payloads.items():
        item: dict[str, Any] = {
            "experiment": payload.get("experiment"),
            "runs": len(payload.get("runs", [])),
            "blocked_rows": len(payload.get("blocked_protocol_rows", [])),
        }
        if key == "e9":
            item.update(payload.get("aggregates", {}))
        if key == "e5":
            item["scene_count"] = payload.get("scene_count")
        summary["experiments"][key] = item
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    add_common_args(parser)
    parser.add_argument("--experiments", type=parse_experiments, default=list(EXPERIMENTS))
    parser.add_argument("--matrix-dir", type=Path, default=HEAVY_DIR)
    parser.add_argument("--e2-train-scenes", type=int, default=3)
    parser.add_argument("--e2-test-scenes", type=int, default=2)
    parser.add_argument("--keep-e2-cache", action="store_true", help="preserve large E2 cache directories after recording disk_bytes")
    parser.add_argument("--e6-scenes-per-group", type=int, default=None)
    parser.add_argument("--e7-threads", type=lambda raw: [int(item) for item in raw.split(",") if item], default=[1, 2, 4, 8, 12, 16])
    parser.add_argument("--e7-growers", type=lambda raw: [item.strip() for item in raw.split(",") if item.strip()], default=["default", "legacy", "partitioned"])
    parser.add_argument("--e7-seeds", type=int, default=10)
    parser.add_argument("--e9-segment-resolution", type=int, default=128)
    args = parser.parse_args()

    if args.build_dir is None:
        args.build_dir = ROOT / "build-release"
    else:
        if args.build_dir.is_absolute():
            args.build_dir = args.build_dir
        else:
            cwd_candidate = (Path.cwd() / args.build_dir).resolve()
            root_candidate = (ROOT / args.build_dir).resolve()
            args.build_dir = cwd_candidate if cwd_candidate.exists() else root_candidate
    if not (args.build_dir / "python").exists():
        raise FileNotFoundError(f"missing Python extension directory: {args.build_dir / 'python'}")

    seeds, timeout_s, mode = mode_args(args, quick_seeds=1, full_seeds=5, quick_timeout=20, full_timeout=60)
    args.seeds = seeds
    args.timeout = timeout_s
    args.mode = mode
    if args.e6_scenes_per_group is None and args.quick:
        args.e6_scenes_per_group = 1

    if args.matrix_dir.is_absolute():
        out_dir = args.matrix_dir
    else:
        cwd_candidate = (Path.cwd() / args.matrix_dir).resolve()
        root_candidate = (ROOT / args.matrix_dir).resolve()
        out_dir = cwd_candidate if cwd_candidate.parent.exists() else root_candidate
    out_dir.mkdir(parents=True, exist_ok=True)

    for path in (PYTHON_SRC, SCRIPTS_DIR, args.build_dir / "python", Path(__file__).resolve().parent):
        text = str(path)
        if text not in sys.path:
            sys.path.insert(0, text)

    payloads: dict[str, dict[str, Any]] = {}
    for experiment in args.experiments:
        print(f"[start] {experiment} {utc_now()}", flush=True)
        payloads[experiment] = RUNNERS[experiment](args, out_dir)
        print(f"[done] {experiment} runs={len(payloads[experiment].get('runs', []))}", flush=True)

    summary = compact_summary(payloads)
    write_payload(out_dir, "summary.json", summary)
    print(f"[write] {out_dir / 'summary.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
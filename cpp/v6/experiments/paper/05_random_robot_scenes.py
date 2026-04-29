#!/usr/bin/env python3
"""Exp. 5 randomized UR5/Panda scenes for the v6 paper pipeline."""
from __future__ import annotations

import argparse
import importlib
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from common import (
    OUT_DEFAULT,
    PAPER_STATISTICS_POLICY,
    PAPER_THREADS,
    PYTHON_SRC,
    ROOT,
    add_common_args,
    mode_args,
    require_python_extension,
    write_json,
)
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
from exp5_baselines import METHOD_LABELS, run_baseline_suite

SCRIPTS_DIR = ROOT / "scripts"
DATA_DIR = ROOT / "data"
RESULT_NAME = "exp5_random_robot_scenes.json"
SCENE_SUBDIR = "exp5_random_scenes"


@dataclass(frozen=True)
class RobotProfile:
    name: str
    robot_json: Path
    robot_urdf: Path
    base_radius: float
    base_height: float
    link1_radius: float
    quick_obstacles: int
    full_obstacles: int
    min_half_size: float
    max_half_size: float
    min_joint_distance: float
    segment_resolution: int


ROBOT_PROFILES = {
    "ur5": RobotProfile(
        name="ur5",
        robot_json=DATA_DIR / "ur5.json",
        robot_urdf=DATA_DIR / "urdf" / "upstream" / "ur_description" / "urdf" / "ur5.urdf",
        base_radius=0.23,
        base_height=0.26,
        link1_radius=0.18,
        quick_obstacles=4,
        full_obstacles=8,
        min_half_size=0.055,
        max_half_size=0.145,
        min_joint_distance=2.3,
        segment_resolution=96,
    ),
    "panda": RobotProfile(
        name="panda",
        robot_json=DATA_DIR / "panda.json",
        robot_urdf=DATA_DIR / "urdf" / "upstream" / "moveit_resources_panda_description" / "urdf" / "panda.urdf",
        base_radius=0.24,
        base_height=0.46,
        link1_radius=0.18,
        quick_obstacles=4,
        full_obstacles=8,
        min_half_size=0.050,
        max_half_size=0.135,
        min_joint_distance=2.0,
        segment_resolution=96,
    ),
}


def parse_robots(value: str) -> list[str]:
    robots = [item.strip().lower() for item in value.split(",") if item.strip()]
    unknown = [name for name in robots if name not in ROBOT_PROFILES]
    if unknown:
        raise argparse.ArgumentTypeError(f"unknown robot(s): {', '.join(unknown)}")
    return robots


def parse_methods(value: str) -> list[str]:
    methods = [item.strip().lower() for item in value.split(",") if item.strip()]
    unknown = [method for method in methods if method not in METHOD_LABELS]
    if unknown:
        raise argparse.ArgumentTypeError(f"unknown method(s): {', '.join(unknown)}")
    return methods


def to_rel(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT.resolve()))
    except ValueError:
        return str(path)


def random_config(robot_doc: dict, rng: np.random.Generator) -> np.ndarray:
    limits = joint_limits(robot_doc)
    span = limits[:, 1] - limits[:, 0]
    return rng.uniform(limits[:, 0] + 0.03 * span, limits[:, 1] - 0.03 * span)


def random_half_sizes(profile: RobotProfile, rng: np.random.Generator) -> np.ndarray:
    base = rng.uniform(profile.min_half_size, profile.max_half_size, size=3)
    base[2] *= rng.uniform(1.1, 1.8)
    return base


def candidate_is_valid(
    robot_doc: dict,
    profile: RobotProfile,
    obstacle: dict,
    workspace_bounds: dict,
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


def make_blocker(
    robot_doc: dict,
    profile: RobotProfile,
    start: np.ndarray,
    goal: np.ndarray,
    workspace_bounds: dict,
    rng: np.random.Generator,
) -> dict | None:
    mid_q = 0.5 * (start + goal)
    segments = active_link_segments(robot_doc, mid_q)
    rng.shuffle(segments)
    for link_index, link_start, link_end, _ in segments:
        if link_index <= 1:
            continue
        center = 0.5 * (link_start + link_end)
        link_dir = link_end - link_start
        link_len = np.linalg.norm(link_dir)
        if link_len > 1e-9:
            link_dir = link_dir / link_len
            trial = np.cross(link_dir, np.asarray([0.0, 0.0, 1.0]))
            if np.linalg.norm(trial) < 1e-6:
                trial = np.asarray([1.0, 0.0, 0.0])
            trial = trial / np.linalg.norm(trial)
            center = center + trial * rng.uniform(-0.015, 0.015)
        for _ in range(16):
            half_sizes = random_half_sizes(profile, rng)
            obstacle = make_box_obstacle("obs_00_blocker", center, half_sizes, "direct_path_blocker")
            if candidate_is_valid(robot_doc, profile, obstacle, workspace_bounds):
                return obstacle
    return None


def sample_workspace_obstacle(
    robot_doc: dict,
    profile: RobotProfile,
    workspace_bounds: dict,
    rng: np.random.Generator,
    name: str,
) -> dict | None:
    for _ in range(200):
        q = random_config(robot_doc, rng)
        segments = active_link_segments(robot_doc, q)
        if not segments:
            continue
        _, start, end, _ = segments[int(rng.integers(0, len(segments)))]
        center = start + rng.uniform(0.15, 0.85) * (end - start)
        center = center + rng.normal(0.0, 0.12, size=3)
        half_sizes = random_half_sizes(profile, rng)
        obstacle = make_box_obstacle(name, center, half_sizes, "random_workspace_obstacle")
        if candidate_is_valid(robot_doc, profile, obstacle, workspace_bounds):
            return obstacle
    return None


def generate_scene(
    profile: RobotProfile,
    *,
    scene_index: int,
    seed: int,
    obstacle_count: int,
    workspace_samples: int,
    max_attempts: int,
) -> dict:
    rng = np.random.default_rng(seed)
    robot_doc = load_robot_doc(profile.robot_json)
    workspace_bounds = sample_workspace_bounds(robot_doc, rng, samples=workspace_samples)
    for attempt in range(1, max_attempts + 1):
        start = random_config(robot_doc, rng)
        goal = random_config(robot_doc, rng)
        if np.linalg.norm(goal - start) < profile.min_joint_distance:
            continue
        blocker = make_blocker(robot_doc, profile, start, goal, workspace_bounds, rng)
        if blocker is None:
            continue
        obstacles = [blocker]
        if check_config_collision(robot_doc, obstacles, start):
            continue
        if check_config_collision(robot_doc, obstacles, goal):
            continue
        obstacle_index = 1
        while len(obstacles) < obstacle_count:
            candidate = sample_workspace_obstacle(
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
        scene_id = f"{profile.name}_random_{scene_index:02d}"
        return {
            "schema_version": 1,
            "scene_id": scene_id,
            "robot": profile.name,
            "robot_json": to_rel(profile.robot_json),
            "robot_urdf": to_rel(profile.robot_urdf),
            "seed": seed,
            "generation_attempts": attempt,
            "workspace_bounds": workspace_bounds,
            "protected_parts": {
                "base_cylinder": {
                    "radius_m": profile.base_radius,
                    "z_min_m": 0.0,
                    "z_max_m": profile.base_height,
                },
                "link1_proxy_radius_m": profile.link1_radius,
            },
            "start": [round(float(x), 8) for x in start],
            "goal": [round(float(x), 8) for x in goal],
            "obstacles": obstacles,
            "checks": {
                "start_collision_free": True,
                "goal_collision_free": True,
                "direct_segment_blocked": True,
                "obstacles_inside_workspace": True,
                "obstacles_avoid_base_and_link1": True,
                "segment_resolution": profile.segment_resolution,
            },
        }
    raise RuntimeError(
        f"failed to generate a valid {profile.name} scene after {max_attempts} attempts"
    )


def normalize_saved_scene(profile: RobotProfile, scene: dict) -> tuple[dict, bool, bool]:
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
        not check_config_collision(robot_doc, obstacles, start)
        and not check_config_collision(robot_doc, obstacles, goal)
        and direct_blocked
    )
    changed = False
    desired_robot_json = to_rel(profile.robot_json)
    desired_robot_urdf = to_rel(profile.robot_urdf)
    if scene.get("robot_json") != desired_robot_json:
        scene["robot_json"] = desired_robot_json
        changed = True
    if scene.get("robot_urdf") != desired_robot_urdf:
        scene["robot_urdf"] = desired_robot_urdf
        changed = True
    checks = dict(scene.get("checks") or {})
    desired_checks = {
        "start_collision_free": True,
        "goal_collision_free": True,
        "direct_segment_blocked": bool(direct_blocked),
        "obstacles_inside_workspace": True,
        "obstacles_avoid_base_and_link1": True,
        "segment_resolution": profile.segment_resolution,
    }
    if checks != desired_checks:
        scene["checks"] = desired_checks
        changed = True
    return scene, valid, changed


def import_sbf5(python_dir: Path):
    for path in (python_dir, PYTHON_SRC, SCRIPTS_DIR):
        text = str(path)
        if text not in sys.path:
            sys.path.insert(0, text)
    return importlib.import_module("sbf5")


def run_planner_for_scene(scene: dict, python_dir: Path, seed: int, timeout_ms: float) -> dict:
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
        grow_timeout_ms=timeout_ms,
        max_boxes=50000,
        post_connect_extra_boxes=1000,
        n_threads=PAPER_THREADS,
        bridge_n_threads=PAPER_THREADS,
        ffb_depth=220,
        lect_no_cache=True,
    )
    config.z4_enabled = False
    planner = sbf5.SBFPlanner(robot, config)
    t0 = time.perf_counter()
    planner.build_coverage(obstacles, float(timeout_ms), [start, goal])
    build_time = time.perf_counter() - t0
    t1 = time.perf_counter()
    result = planner.query(start, goal)
    query_time = time.perf_counter() - t1
    return {
        "seed": seed,
        "success": bool(result.success),
        "build_time_s": build_time,
        "query_time_s": query_time,
        "n_boxes": int(planner.n_boxes()),
        "path_length": float(result.path_length) if result.success else None,
        "planning_time_ms": float(result.planning_time_ms),
        "architecture": "paper_sbf_build_query_v6_no_z4_for_non_iiwa",
    }


def scene_path(scene_dir: Path, robot: str, index: int) -> Path:
    return scene_dir / f"{robot}_random_{index:02d}.json"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_common_args(parser)
    parser.add_argument("--robots", type=parse_robots, default=parse_robots("ur5,panda"))
    parser.add_argument("--base-seed", type=int, default=20260428)
    parser.add_argument("--scenes-per-robot", type=int, default=None)
    parser.add_argument("--obstacles", type=int, default=None)
    parser.add_argument("--workspace-samples", type=int, default=None)
    parser.add_argument("--max-attempts", type=int, default=1200)
    parser.add_argument("--regenerate", action="store_true", help="overwrite saved scenes instead of reusing them")
    parser.add_argument("--run-planner", action="store_true", help="also run the SBF build/query measurement on each scene")
    parser.add_argument("--run-baselines", action="store_true", help="run Exp. 4's five method families on each saved scene")
    parser.add_argument(
        "--baseline-methods",
        type=parse_methods,
        default=parse_methods("sbf,iris_np_gcs,iris_zo_gcs,ompl_prm,ompl_bitstar"),
        help="comma-separated subset of: sbf,iris_np_gcs,iris_zo_gcs,ompl_prm,ompl_bitstar",
    )
    parser.add_argument("--baseline-seeds", type=int, default=None)
    parser.add_argument("--baseline-timeout", type=float, default=None, help="per-method timeout in seconds")
    parser.add_argument("--baseline-prm-samples", type=int, default=None)
    parser.add_argument("--baseline-edge-resolution", type=int, default=32)
    args = parser.parse_args()

    planner_seeds, planner_timeout_s, _ = mode_args(
        args,
        quick_seeds=1,
        full_seeds=5,
        quick_timeout=20,
        full_timeout=60,
    )
    scenes_per_robot = args.scenes_per_robot
    if scenes_per_robot is None:
        scenes_per_robot = 1 if args.quick else 5
    workspace_samples = args.workspace_samples
    if workspace_samples is None:
        workspace_samples = 160 if args.quick else 640

    out_dir = args.out_dir if args.out_dir.is_absolute() else ROOT / args.out_dir
    scene_dir = out_dir / SCENE_SUBDIR / "scenes"
    paths_dir = out_dir / SCENE_SUBDIR / "paths"
    scene_dir.mkdir(parents=True, exist_ok=True)
    paths_dir.mkdir(parents=True, exist_ok=True)
    baseline_seed_count = args.baseline_seeds if args.baseline_seeds is not None else (1 if args.quick else 5)
    baseline_timeout_s = args.baseline_timeout if args.baseline_timeout is not None else (8.0 if args.quick else 30.0)
    baseline_prm_samples = args.baseline_prm_samples if args.baseline_prm_samples is not None else (260 if args.quick else 900)

    if args.dry_run:
        print(f"[dry-run] would use scenes under {to_rel(scene_dir)}")
        print(f"[dry-run] robots={','.join(args.robots)} scenes_per_robot={scenes_per_robot}")
        print(f"[dry-run] run_planner={args.run_planner} run_baselines={args.run_baselines}")
        if args.run_baselines:
            print(
                "[dry-run] baseline_methods="
                f"{','.join(args.baseline_methods)} seeds={baseline_seed_count} "
                f"timeout_s={baseline_timeout_s} prm_samples={baseline_prm_samples}"
            )
        print(f"[dry-run] would write {to_rel(out_dir / RESULT_NAME)}")
        return

    python_dir = require_python_extension(args) if (args.run_planner or args.run_baselines) else None

    scene_rows = []
    for robot_offset, robot_name in enumerate(args.robots):
        profile = ROBOT_PROFILES[robot_name]
        default_obstacles = profile.quick_obstacles if args.quick else profile.full_obstacles
        obstacle_count = args.obstacles if args.obstacles is not None else default_obstacles
        for scene_index in range(scenes_per_robot):
            path = scene_path(scene_dir, robot_name, scene_index)
            if path.exists() and not args.regenerate:
                scene = json.loads(path.read_text())
                scene, is_valid, changed = normalize_saved_scene(profile, scene)
                if not is_valid:
                    seed = int(args.base_seed + robot_offset * 10000 + scene_index)
                    scene = generate_scene(
                        profile,
                        scene_index=scene_index,
                        seed=seed,
                        obstacle_count=obstacle_count,
                        workspace_samples=workspace_samples,
                        max_attempts=args.max_attempts,
                    )
                    scene["reused"] = False
                    write_json(path, scene)
                else:
                    scene["reused"] = True
                    if changed:
                        write_json(path, scene)
            else:
                seed = int(args.base_seed + robot_offset * 10000 + scene_index)
                scene = generate_scene(
                    profile,
                    scene_index=scene_index,
                    seed=seed,
                    obstacle_count=obstacle_count,
                    workspace_samples=workspace_samples,
                    max_attempts=args.max_attempts,
                )
                scene["reused"] = False
                write_json(path, scene)
            row = {
                "scene_id": scene["scene_id"],
                "robot": scene["robot"],
                "scene_file": to_rel(path),
                "seed": int(scene["seed"]),
                "n_obstacles": len(scene["obstacles"]),
                "direct_segment_blocked": bool(scene["checks"]["direct_segment_blocked"]),
                "reused": bool(scene.get("reused", False)),
            }
            if args.run_planner:
                planner_runs = []
                for planner_seed in range(planner_seeds):
                    planner_runs.append(
                        run_planner_for_scene(
                            scene,
                            python_dir,
                            seed=planner_seed,
                            timeout_ms=float(planner_timeout_s) * 1000.0,
                        )
                    )
                row["planner_runs"] = planner_runs
            if args.run_baselines:
                robot_doc = load_robot_doc(ROOT / scene["robot_json"])
                baseline_summaries, typical_paths = run_baseline_suite(
                    robot_doc,
                    scene,
                    python_dir=python_dir,
                    seeds=list(range(baseline_seed_count)),
                    methods=args.baseline_methods,
                    timeout_s=float(baseline_timeout_s),
                    edge_resolution=int(args.baseline_edge_resolution),
                    prm_samples=int(baseline_prm_samples),
                )
                path_file = paths_dir / f"{scene['scene_id']}_typical_paths.json"
                write_json(
                    path_file,
                    {
                        "schema_version": 1,
                        "scene_id": scene["scene_id"],
                        "scene_file": to_rel(path),
                        "robot": scene["robot"],
                        "robot_urdf": scene.get("robot_urdf", to_rel(profile.robot_urdf)),
                        "paths": typical_paths,
                    },
                )
                row["baseline_results"] = baseline_summaries
                row["typical_paths_file"] = to_rel(path_file)
            scene_rows.append(row)
            print(
                f"[exp5] {row['scene_id']}: {row['n_obstacles']} obstacles, "
                f"direct_blocked={row['direct_segment_blocked']}, file={row['scene_file']}"
            )
            if args.run_baselines:
                for summary in row["baseline_results"]:
                    print(
                        f"  - {summary['label']}: success={summary['success_rate']:.2f}, "
                        f"time={summary['planning_time_s_median']}, length={summary['path_length_median']}"
                    )

    summary = {
        "schema_version": 1,
        "experiment": "exp5_random_robot_scenes",
        "description": "Randomized UR5/Panda workspace scenes with blocked straight-line start-goal pairs.",
        "robots": args.robots,
        "scene_dir": to_rel(scene_dir),
        "default_visualizer": "cpp/v6/viz/viz_exp5_random_scenes.py",
        "constraints": {
            "start_goal_collision_free": True,
            "direct_start_goal_segment_blocked": True,
            "obstacles_inside_sampled_workspace_bounds": True,
            "obstacles_avoid_base_and_link1_proxy": True,
        },
        "planner_measurements_included": bool(args.run_planner),
        "baseline_measurements_included": bool(args.run_baselines),
        "baseline_methods": args.baseline_methods if args.run_baselines else [],
        "statistical_policy": PAPER_STATISTICS_POLICY["exp5"],
        "resource_policy": {
            "logical_threads": PAPER_THREADS,
            "cpu_affinity": list(range(PAPER_THREADS)),
            "seed_execution": "serial",
        },
        "baseline_note": (
            "Exp.5 reuses Exp.4's paper SBF build/cached-query architecture, records build and "
            "query times separately, and performs an untimed per-scene LECT prewarm before SBF "
            "measurement. PRM build is sampling plus roadmap construction; PRM query is Dijkstra "
            "plus shortcut. IRIS/GCS proxy build is skeleton generation plus local region inflation; "
            "query is shortest-chain extraction plus shortcut. BIT* is reported as a fixed 2s "
            "query budget with no reusable build phase. OMPL and IRIS/GCS rows remain local "
            "reproducible proxies for the random URDF scenes; Exp.4 Marcucci baselines use the "
            "v6-native Drake/OMPL runners."
            if args.run_baselines
            else None
        ),
        "scenes": scene_rows,
    }
    write_json(out_dir / RESULT_NAME, summary)
    print(f"[exp5] wrote {to_rel(out_dir / RESULT_NAME)}")


if __name__ == "__main__":
    main()

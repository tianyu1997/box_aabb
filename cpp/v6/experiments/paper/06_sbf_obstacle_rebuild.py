#!/usr/bin/env python3
"""Exp. 6: SBF obstacle-add invalidation and adjacency rebuild timing."""
from __future__ import annotations

import argparse
import importlib
import json
import sys
import time
from pathlib import Path
from typing import Any

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
from exp5_baselines import build_sbf_planner, ensure_sbf_lect_warmup, exp5_sbf_cache_dir
from exp5_scene_utils import (
    check_config_collision,
    load_robot_doc,
    obstacle_bounds,
)

SCRIPTS_DIR = ROOT / "scripts"
RESULT_NAME = "exp6_sbf_obstacle_rebuild.json"
SCENE_SUBDIR = "exp6_sbf_obstacle_rebuild"

_exp5 = importlib.import_module("05_random_robot_scenes")
DIFFICULTIES = _exp5.DIFFICULTIES
ROBOT_PROFILES = _exp5.ROBOT_PROFILES


def parse_robots(value: str) -> list[str]:
    return _exp5.parse_robots(value)


def parse_difficulties(value: str) -> list[str]:
    return _exp5.parse_difficulties(value)


def to_rel(path: Path) -> str:
    return _exp5.to_rel(path)


def difficulty_scene_path(scene_dir: Path, robot: str, difficulty: str, index: int) -> Path:
    return scene_dir / f"{robot}_{difficulty}_{index:02d}.json"


def aabb_overlap(a: np.ndarray, b: np.ndarray, margin: float = 0.0) -> bool:
    return bool(
        np.all(a[:3] <= b[3:] + margin)
        and np.all(b[:3] <= a[3:] + margin)
    )


def sample_added_obstacle(scene: dict[str, Any], rng: np.random.Generator) -> dict[str, Any]:
    profile = ROBOT_PROFILES[str(scene["robot"])]
    robot_doc = load_robot_doc(ROOT / scene["robot_json"])
    workspace_bounds = scene["workspace_bounds"]
    start = np.asarray(scene["start"], dtype=float)
    goal = np.asarray(scene["goal"], dtype=float)
    existing = [obstacle_bounds(item) for item in scene["obstacles"]]

    for attempt in range(600):
        candidate = _exp5.sample_workspace_obstacle(
            robot_doc,
            profile,
            workspace_bounds,
            rng,
            f"obs_added_{attempt:03d}",
        )
        if candidate is None:
            continue
        bounds = obstacle_bounds(candidate)
        if any(aabb_overlap(bounds, old, margin=0.01) for old in existing):
            continue
        trial_obstacles = [*scene["obstacles"], candidate]
        if check_config_collision(robot_doc, trial_obstacles, start):
            continue
        if check_config_collision(robot_doc, trial_obstacles, goal):
            continue
        candidate["role"] = "added_obstacle_for_rebuild"
        return candidate
    raise RuntimeError(f"failed to sample added obstacle for scene {scene.get('scene_id')}")


def quantiles(values: list[float]) -> dict[str, float | None]:
    if not values:
        return {"mean": None, "median": None, "p25": None, "p75": None}
    arr = np.asarray(values, dtype=float)
    return {
        "mean": float(np.mean(arr)),
        "median": float(np.median(arr)),
        "p25": float(np.percentile(arr, 25)),
        "p75": float(np.percentile(arr, 75)),
    }


def aggregate_runs(rows: list[dict[str, Any]]) -> dict[str, Any]:
    groups: list[dict[str, Any]] = []
    keys = sorted(
        {
            (str(row["robot"]), str(row["difficulty"]))
            for row in rows
            if row.get("rebuild_runs")
        },
        key=lambda item: (item[0], DIFFICULTIES.index(item[1]) if item[1] in DIFFICULTIES else 99),
    )
    for robot, difficulty in keys:
        runs = [
            run
            for row in rows
            if row["robot"] == robot and row["difficulty"] == difficulty
            for run in row.get("rebuild_runs", [])
        ]
        rebuild_times = [float(run["rebuild_time_s"]) for run in runs]
        collision_times = [float(run["collision_check_s"]) for run in runs]
        adjacency_times = [float(run["adjacency_s"]) for run in runs]
        build_times = [float(run["build_time_s"]) for run in runs]
        removal_ratios = [float(run["removal_ratio"]) for run in runs]
        groups.append(
            {
                "robot": robot,
                "difficulty": difficulty,
                "n_runs": len(runs),
                "n_scenes": len({run["scene_id"] for run in runs}),
                "n_all_removed": sum(1 for run in runs if run["all_boxes_removed"]),
                "build_time_s": quantiles(build_times),
                "rebuild_time_s": quantiles(rebuild_times),
                "collision_check_s": quantiles(collision_times),
                "adjacency_s": quantiles(adjacency_times),
                "removal_ratio": quantiles(removal_ratios),
                "boxes_before_median": float(np.median([run["boxes_before"] for run in runs])) if runs else None,
                "boxes_removed_median": float(np.median([run["boxes_removed"] for run in runs])) if runs else None,
                "islands_after_median": float(np.median([run["islands_after"] for run in runs])) if runs else None,
            }
        )
    return {"groups": groups}


def run_rebuild_trial(
    scene: dict[str, Any],
    *,
    python_dir: Path,
    seed: int,
    timeout_s: float,
    endpoint: str,
) -> dict[str, Any]:
    if str(PYTHON_SRC) not in sys.path:
        sys.path.insert(0, str(PYTHON_SRC))
    if str(SCRIPTS_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPTS_DIR))

    rng = np.random.default_rng(int(scene["seed"]) + 1000003 * (seed + 1))
    added_obstacle = sample_added_obstacle(scene, rng)
    ensure_sbf_lect_warmup(
        scene,
        python_dir=python_dir,
        timeout_s=timeout_s,
        sbf_endpoint_source=endpoint,
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
    build_time_s = time.perf_counter() - build_start

    sbf6 = importlib.import_module("sbf6")
    obstacle_cpp = sbf6.Obstacle(*added_obstacle["bounds"])
    rebuild = planner.add_obstacle_and_rebuild(obstacle_cpp)

    boxes_before = int(rebuild.boxes_before)
    boxes_after = int(rebuild.boxes_after)
    boxes_removed = int(rebuild.boxes_removed)
    removal_ratio = (boxes_removed / boxes_before) if boxes_before else 0.0
    timing = planner.build_timing()
    return {
        "scene_id": scene["scene_id"],
        "seed": int(seed),
        "endpoint_source": endpoint,
        "build_time_s": float(build_time_s),
        "build_timing_total_s": float(timing.total_ms) / 1000.0,
        "initial_obstacles": len(scene["obstacles"]),
        "added_obstacle": added_obstacle,
        "boxes_before": boxes_before,
        "boxes_after": boxes_after,
        "boxes_removed": boxes_removed,
        "raw_boxes_before": int(rebuild.raw_boxes_before),
        "raw_boxes_after": int(rebuild.raw_boxes_after),
        "raw_boxes_removed": int(rebuild.raw_boxes_removed),
        "removal_ratio": float(removal_ratio),
        "adjacency_edges_after": int(rebuild.adjacency_edges_after),
        "islands_after": int(rebuild.islands_after),
        "collision_check_s": float(rebuild.collision_check_ms) / 1000.0,
        "adjacency_s": float(rebuild.adjacency_ms) / 1000.0,
        "rebuild_time_s": float(rebuild.total_ms) / 1000.0,
        "all_boxes_removed": bool(boxes_before > 0 and boxes_after == 0),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_common_args(parser)
    parser.add_argument("--robots", type=parse_robots, default=parse_robots("ur5,panda"))
    parser.add_argument("--difficulties", type=parse_difficulties, default=parse_difficulties("easy,medium,hard"))
    parser.add_argument("--base-seed", type=int, default=20260501)
    parser.add_argument("--scenes-per-difficulty", type=int, default=None)
    parser.add_argument("--obstacles", type=int, default=None)
    parser.add_argument("--workspace-samples", type=int, default=None)
    parser.add_argument("--max-attempts", type=int, default=1200)
    parser.add_argument("--regenerate", action="store_true")
    parser.add_argument("--sbf-endpoint-source", choices=("critsample", "ifk"), default="critsample")
    args = parser.parse_args()

    seed_count, timeout_s, mode = mode_args(
        args,
        quick_seeds=1,
        full_seeds=5,
        quick_timeout=20,
        full_timeout=60,
    )
    scenes_per_difficulty = args.scenes_per_difficulty
    if scenes_per_difficulty is None:
        scenes_per_difficulty = 1 if args.quick else 5
    workspace_samples = args.workspace_samples
    if workspace_samples is None:
        workspace_samples = 160 if args.quick else 640

    out_dir = args.out_dir if args.out_dir.is_absolute() else ROOT / args.out_dir
    scene_dir = out_dir / SCENE_SUBDIR / "scenes"
    scene_dir.mkdir(parents=True, exist_ok=True)

    if args.dry_run:
        print(json.dumps({
            "experiment": "exp6_sbf_obstacle_rebuild",
            "out": str(out_dir / RESULT_NAME),
            "threads": PAPER_THREADS,
            "robots": args.robots,
            "difficulties": args.difficulties,
            "scenes_per_difficulty": int(scenes_per_difficulty),
            "seeds_per_scene": int(seed_count),
            "timeout_s": float(timeout_s),
            "mode": mode,
            "endpoint_source": args.sbf_endpoint_source,
        }, indent=2))
        return

    python_dir = require_python_extension(args)
    rows: list[dict[str, Any]] = []
    for robot_offset, robot_name in enumerate(args.robots):
        profile = ROBOT_PROFILES[robot_name]
        for difficulty_index, difficulty in enumerate(args.difficulties):
            obstacle_count = _exp5.obstacle_count_for(profile, difficulty, args.quick, args.obstacles)
            for scene_index in range(int(scenes_per_difficulty)):
                path = difficulty_scene_path(scene_dir, robot_name, difficulty, scene_index)
                if path.exists() and not args.regenerate:
                    scene = json.loads(path.read_text())
                    scene.setdefault("difficulty", difficulty)
                    scene, is_valid, changed = _exp5.normalize_saved_scene(profile, scene)
                    if not is_valid:
                        seed = int(args.base_seed + robot_offset * 10000 + difficulty_index * 1000 + scene_index)
                        scene = _exp5.generate_scene(
                            profile,
                            difficulty=difficulty,
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
                        scene["difficulty"] = difficulty
                        if changed:
                            write_json(path, scene)
                else:
                    seed = int(args.base_seed + robot_offset * 10000 + difficulty_index * 1000 + scene_index)
                    scene = _exp5.generate_scene(
                        profile,
                        difficulty=difficulty,
                        scene_index=scene_index,
                        seed=seed,
                        obstacle_count=obstacle_count,
                        workspace_samples=workspace_samples,
                        max_attempts=args.max_attempts,
                    )
                    scene["reused"] = False
                    write_json(path, scene)

                rebuild_runs = []
                for trial_seed in range(int(seed_count)):
                    rebuild_runs.append(
                        run_rebuild_trial(
                            scene,
                            python_dir=python_dir,
                            seed=trial_seed,
                            timeout_s=float(timeout_s),
                            endpoint=args.sbf_endpoint_source,
                        )
                    )

                rows.append(
                    {
                        "scene_id": scene["scene_id"],
                        "robot": scene["robot"],
                        "difficulty": str(scene.get("difficulty", difficulty)),
                        "scene_file": to_rel(path),
                        "seed": int(scene["seed"]),
                        "n_obstacles": len(scene["obstacles"]),
                        "reused": bool(scene.get("reused", False)),
                        "rebuild_runs": rebuild_runs,
                    }
                )

    payload = {
        "schema_version": 1,
        "experiment": "exp6_sbf_obstacle_rebuild",
        "description": "Random scene SBF build followed by one added obstacle, invalidated-box deletion, and adjacency rebuild.",
        "mode": mode,
        "statistics_policy": PAPER_STATISTICS_POLICY["resource"],
        "sbf_endpoint_source": args.sbf_endpoint_source,
        "scenes_per_difficulty": int(scenes_per_difficulty),
        "seeds_per_scene": int(seed_count),
        "timeout_s": float(timeout_s),
        "rebuild_scope": "delete_colliding_boxes_and_recompute_adjacency_only",
        "scenes": rows,
        "aggregation": aggregate_runs(rows),
    }
    out_path = out_dir / RESULT_NAME
    write_json(out_path, payload)
    print(f"[write] {out_path}")


if __name__ == "__main__":
    main()

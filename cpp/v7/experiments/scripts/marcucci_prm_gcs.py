#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from _prm_repro import (
    DEFAULT_EDGE_STEP_SIZE,
    DEFAULT_ENV_PADDING,
    DEFAULT_GRID_RESOLUTION,
    DEFAULT_GRID_SIZE,
    DEFAULT_MAX_VALID_SAMPLE_TRIES,
    DEFAULT_NUM_NEIGHBORS,
    DEFAULT_PROPAGATION_STEP_SIZE,
    DEFAULT_ROADMAP_SIZE,
    DEFAULT_SELF_PADDING,
    PresplinedPRM,
    default_creation_parameters,
    default_postprocessing_parameters,
    default_query_parameters,
)
from common import (
    RESULTS_PAPER,
    add_logical_threads_arg,
    add_mode_args,
    aggregate_method_trials,
    empty_query_record,
    marcucci_anchor_cycle,
    marcucci_workload,
    ordered_parallel_map,
    resolve_mode,
    resolve_output,
    write_json,
)


def path_length(path: np.ndarray) -> float:
    if path.shape[1] < 2:
        return 0.0
    diffs = np.diff(path.T, axis=0)
    return float(np.sum(np.linalg.norm(diffs, axis=1)))


def default_roadmap_dir() -> Path:
    return RESULTS_PAPER / "roadmaps" / "marcucci_prm_gcs"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    add_mode_args(parser)
    add_logical_threads_arg(parser)
    parser.add_argument("--roadmap-dir", type=Path, default=default_roadmap_dir())
    parser.add_argument("--reuse-roadmaps", action="store_true")
    parser.add_argument("--roadmap-size", type=int, default=DEFAULT_ROADMAP_SIZE)
    parser.add_argument("--num-neighbors", type=int, default=DEFAULT_NUM_NEIGHBORS)
    parser.add_argument("--max-valid-sample-tries", type=int, default=DEFAULT_MAX_VALID_SAMPLE_TRIES)
    parser.add_argument("--edge-step-size", type=float, default=DEFAULT_EDGE_STEP_SIZE)
    parser.add_argument("--env-padding", type=float, default=DEFAULT_ENV_PADDING)
    parser.add_argument("--self-padding", type=float, default=DEFAULT_SELF_PADDING)
    parser.add_argument("--propagation-step-size", type=float, default=DEFAULT_PROPAGATION_STEP_SIZE)
    parser.add_argument("--grid-resolution", type=float, default=DEFAULT_GRID_RESOLUTION)
    parser.add_argument("--grid-size", type=float, nargs=3, default=list(DEFAULT_GRID_SIZE))
    parser.add_argument("--no-postprocess", action="store_true")
    return parser.parse_args()


def roadmap_paths(roadmap_dir: Path, seed: int) -> tuple[Path, Path]:
    prefix = roadmap_dir / f"seed_{seed:02d}"
    return prefix.with_suffix(".rmp"), prefix.with_suffix(".meta.json")


def load_meta(meta_path: Path) -> dict:
    return json.loads(meta_path.read_text()) if meta_path.exists() else {}


def save_meta(meta_path: Path, payload: dict) -> None:
    meta_path.parent.mkdir(parents=True, exist_ok=True)
    meta_path.write_text(json.dumps(payload, indent=2) + "\n")


def run_seed(seed: int, args: argparse.Namespace, workload: list[dict]) -> dict:
    planner = PresplinedPRM(
        edge_step_size=args.edge_step_size,
        env_padding=args.env_padding,
        self_padding=args.self_padding,
        propagation_step_size=args.propagation_step_size,
        grid_size=args.grid_size,
        grid_resolution=args.grid_resolution,
        seed=seed,
    )
    anchors = [anchor["q"] for anchor in marcucci_anchor_cycle(workload)]
    roadmap_path, meta_path = roadmap_paths(args.roadmap_dir, seed)
    creation_params = default_creation_parameters(
        roadmap_size=args.roadmap_size,
        num_neighbors=args.num_neighbors,
        max_valid_sample_tries=args.max_valid_sample_tries,
        parallelize=True,
    )
    query_params = default_query_parameters(
        num_neighbors=args.num_neighbors,
        parallelize=True,
    )
    postprocess = None if args.no_postprocess else default_postprocessing_parameters(seed=seed)

    loaded = False
    if args.reuse_roadmaps and roadmap_path.exists():
        planner.load(roadmap_path)
        loaded = True
        meta = load_meta(meta_path)
        build_s = float(meta.get("build_s", 0.0))
    else:
        build_s = planner.build_roadmap(anchors, creation_params)
        roadmap_path.parent.mkdir(parents=True, exist_ok=True)
        planner.save(roadmap_path)
        save_meta(
            meta_path,
            {
                "build_s": build_s,
                "seed": seed,
                "roadmap_size": args.roadmap_size,
                "num_neighbors": args.num_neighbors,
                "max_valid_sample_tries": args.max_valid_sample_tries,
            },
        )

    trial = {
        "seed": seed,
        "build_s": build_s,
        "loaded_roadmap": loaded,
        "roadmap_file": str(roadmap_path),
        "queries": [],
    }
    for item in workload:
        path, run_time = planner.plan(
            [item["q_start"], item["q_goal"]],
            query_params,
            postprocess,
        )
        if path is None:
            trial["queries"].append(
                empty_query_record(
                    item["label"],
                    item["file"],
                    seed=seed,
                    note="PRM query failed",
                )
            )
            continue
        trial["queries"].append(
            {
                "query": item["label"],
                "file": item["file"],
                "seed": seed,
                "success": True,
                "time_s": float(run_time),
                "path_length": path_length(path),
                "n_waypoints": int(path.shape[1]),
            }
        )
    return trial


def run_seed_task(task: tuple[int, dict, list[dict]]) -> dict:
    seed, args_dict, workload = task
    args = argparse.Namespace(**args_dict)
    return run_seed(seed, args, workload)


def main() -> int:
    args = parse_args()
    quick, seeds, _ = resolve_mode(args)
    workload = marcucci_workload()
    out_path = resolve_output(args, "marcucci_prm_gcs.json")
    params = {
        "logical_threads": args.logical_threads,
        "roadmap_size": args.roadmap_size,
        "num_neighbors": args.num_neighbors,
        "max_valid_sample_tries": args.max_valid_sample_tries,
        "edge_step_size": args.edge_step_size,
        "env_padding": args.env_padding,
        "self_padding": args.self_padding,
        "propagation_step_size": args.propagation_step_size,
        "grid_size": list(args.grid_size),
        "grid_resolution": args.grid_resolution,
        "postprocess": not args.no_postprocess,
        "reuse_roadmaps": args.reuse_roadmaps,
    }

    if args.dry_run:
        print(json.dumps({"out": str(out_path), "params": params}, indent=2))
        return 0

    task_args = {
        "logical_threads": args.logical_threads,
        "roadmap_dir": args.roadmap_dir,
        "reuse_roadmaps": args.reuse_roadmaps,
        "roadmap_size": args.roadmap_size,
        "num_neighbors": args.num_neighbors,
        "max_valid_sample_tries": args.max_valid_sample_tries,
        "edge_step_size": args.edge_step_size,
        "env_padding": args.env_padding,
        "self_padding": args.self_padding,
        "propagation_step_size": args.propagation_step_size,
        "grid_resolution": args.grid_resolution,
        "grid_size": list(args.grid_size),
        "no_postprocess": args.no_postprocess,
    }
    tasks = [(seed, task_args, workload) for seed in range(seeds)]
    seed_trials = ordered_parallel_map(
        run_seed_task,
        tasks,
        max_workers=max(1, int(args.logical_threads)),
    )
    payload = aggregate_method_trials(
        method="prm_reproduction",
        scene="iiwa14_marcucci_combined",
        quick=quick,
        seeds=seeds,
        params=params,
        seed_trials=seed_trials,
    )
    write_json(out_path, payload)
    print(f"[marcucci_prm_gcs] wrote {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
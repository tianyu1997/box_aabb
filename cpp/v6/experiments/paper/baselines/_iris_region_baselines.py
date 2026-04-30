#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path
from typing import Callable

import numpy as np

HERE = Path(__file__).resolve().parent
PAPER_DIR = HERE.parent
for candidate in (HERE, PAPER_DIR):
    text = str(candidate)
    if text not in sys.path:
        sys.path.insert(0, text)

from _drake_gcs_regions import solve_regions_gcs
from common import add_logical_threads_arg, empty_query_record, marcucci_anchor_cycle


DEFAULT_IRIS_REGION_BASELINE = {
    "edge_step_size": 0.05,
    "env_padding": 0.0,
    "self_padding": 0.0,
    "budget_s": 3000.0,
}


def add_shared_iris_args(parser) -> None:
    parser.add_argument("--budget-s", type=float, default=DEFAULT_IRIS_REGION_BASELINE["budget_s"])
    add_logical_threads_arg(parser)


def region_seed_configs(workload: list[dict]) -> list[tuple[str, np.ndarray]]:
    anchors = marcucci_anchor_cycle(workload)
    seeds: list[tuple[str, np.ndarray]] = [
        (anchor["name"], np.asarray(anchor["q"], dtype=float)) for anchor in anchors[:-1]
    ]
    for item in workload:
        midpoint = 0.5 * (
            np.asarray(item["q_start"], dtype=float) + np.asarray(item["q_goal"], dtype=float)
        )
        seeds.append((f"mid_{item['label']}", midpoint))
    return seeds


def run_region_baseline(
    *,
    seed: int,
    workload: list[dict],
    budget_s: float,
    build_regions: Callable[[int, list[tuple[str, np.ndarray]], float], tuple[list, list[float], list[dict]]],
    failure_note: str,
    seed_configs: list[tuple[str, np.ndarray]] | None = None,
    solve_kwargs: dict | None = None,
) -> dict:
    if seed_configs is None:
        seed_configs = region_seed_configs(workload)
    built = build_regions(seed, seed_configs, budget_s)
    if len(built) == 4:
        regions, timings, failures, validation_checker = built
    else:
        regions, timings, failures = built
        validation_checker = None

    trial = {
        "seed": seed,
        "build_s": float(sum(timings)),
        "n_regions": len(regions),
        "per_region_s": [float(value) for value in timings],
        "queries": [],
    }
    if failures:
        trial["failed_region_seeds"] = failures

    for item in workload:
        result = solve_regions_gcs(
            np.asarray(item["q_start"], dtype=float),
            np.asarray(item["q_goal"], dtype=float),
            regions,
            seed=seed,
            checker=validation_checker,
            **(solve_kwargs or {}),
        )
        if not result["success"]:
            record = empty_query_record(
                item["label"],
                item["file"],
                seed=seed,
                note=result.get("note", failure_note),
            )
            record["regions"] = result.get("regions")
            record["edges"] = result.get("edges")
            record["raw_path_length"] = result.get("raw_path_length")
            record["waypoints_count"] = result.get("waypoints_count")
            record["collision_checked"] = result.get("collision_checked")
            record["collision_free"] = result.get("collision_free")
            record["unsafe_segments"] = result.get("unsafe_segments")
            record["failure_time_s"] = result.get("time_s")
            record["repair_time_s"] = result.get("repair_time_s")
            trial["queries"].append(record)
            continue
        record = {
            "query": item["label"],
            "file": item["file"],
            "seed": seed,
            "success": True,
            "time_s": float(result["time_s"]),
            "path_length": float(result["path_length"]),
            "regions": int(result["regions"]),
            "edges": int(result["edges"]),
            "waypoints_count": int(result["waypoints_count"]),
            "collision_checked": bool(result.get("collision_checked")),
            "collision_free": result.get("collision_free"),
            "unsafe_segments": result.get("unsafe_segments"),
        }
        for key in (
            "raw_path_length",
            "raw_waypoints_count",
            "gcs_unsafe_segments",
            "repaired_segments",
            "repair_time_s",
            "note",
        ):
            if key in result:
                record[key] = result[key]
        trial["queries"].append(record)
    return trial
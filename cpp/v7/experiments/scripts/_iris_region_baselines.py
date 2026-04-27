#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path
from typing import Callable

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

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
) -> dict:
    seed_configs = region_seed_configs(workload)
    regions, timings, failures = build_regions(seed, seed_configs, budget_s)

    trial = {
        "seed": seed,
        "build_s": float(sum(timings)),
        "n_regions": len(regions),
        "per_region_s": [float(t) for t in timings],
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
            trial["queries"].append(record)
            continue
        trial["queries"].append(
            {
                "query": item["label"],
                "file": item["file"],
                "seed": seed,
                "success": True,
                "time_s": float(result["time_s"]),
                "path_length": float(result["path_length"]),
                "regions": int(result["regions"]),
                "edges": int(result["edges"]),
                "waypoints_count": int(result["waypoints_count"]),
            }
        )
    return trial
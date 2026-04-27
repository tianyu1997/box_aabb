#!/usr/bin/env python3
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import json
import os
import sys
from pathlib import Path
from statistics import mean, median
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
EXPERIMENTS = ROOT / "experiments"
CONFIGS = EXPERIMENTS / "configs"
MARCUCCI_CONFIGS = CONFIGS / "marcucci"
RESULTS_PAPER = EXPERIMENTS / "results_paper"
DATA = ROOT / "data"
WORKSPACE = ROOT.parents[1]
DEFAULT_LOGICAL_THREADS = 16
BUILD_RELEASE = ROOT / "build-release"
BUILD_DEBUG = ROOT / "build"

MARCUCCI_QUERY_FILES: list[tuple[str, str]] = [
    ("AS->TS", "AS_TS.json"),
    ("TS->CS", "TS_CS.json"),
    ("CS->LB", "CS_LB.json"),
    ("LB->RB", "LB_RB.json"),
    ("RB->AS", "RB_AS.json"),
]


def _normalize_build_dir(build_dir: Path) -> Path:
    return build_dir if build_dir.is_absolute() else (ROOT / build_dir)


def cmake_build_type(build_dir: Path) -> str | None:
    cache_path = build_dir / "CMakeCache.txt"
    if not cache_path.is_file():
        return None
    for line in cache_path.read_text().splitlines():
        if line.startswith("CMAKE_BUILD_TYPE:STRING="):
            value = line.split("=", 1)[1].strip()
            return value or None
    return None


def resolve_experiment_binary(
    name: str,
    *,
    requested_build_dir: Path | None = None,
    allow_debug: bool = False,
) -> Path:
    candidate_dirs: list[Path] = []
    if requested_build_dir is not None:
        candidate_dirs.append(_normalize_build_dir(requested_build_dir))
    else:
        if BUILD_RELEASE.is_dir():
            candidate_dirs.append(BUILD_RELEASE)
        if BUILD_DEBUG.is_dir():
            candidate_dirs.append(BUILD_DEBUG)

    found_debug_binary: Path | None = None
    for build_dir in candidate_dirs:
        binary = build_dir / "experiments" / name
        if not binary.is_file():
            continue
        if cmake_build_type(build_dir) == "Debug" and not allow_debug:
            found_debug_binary = binary
            continue
        return binary

    if found_debug_binary is not None:
        raise RuntimeError(
            "Refusing to use Debug binary for experiment timing: "
            f"{found_debug_binary}. Build the Release target in build-release or "
            "opt in explicitly with allow_debug=True."
        )
    searched = candidate_dirs or [BUILD_RELEASE, BUILD_DEBUG]
    joined = ", ".join(str(path / "experiments" / name) for path in searched)
    raise FileNotFoundError(f"{name} not found in experiment build trees: {joined}")


def add_mode_args(parser: argparse.ArgumentParser) -> None:
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--quick", action="store_true")
    mode.add_argument("--full", action="store_true")
    parser.add_argument("--seeds", type=int, default=None)
    parser.add_argument("--timeout", type=int, default=None)
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--dry-run", action="store_true")


def add_logical_threads_arg(
    parser: argparse.ArgumentParser,
    *,
    default: int = DEFAULT_LOGICAL_THREADS,
) -> None:
    parser.add_argument(
        "--logical-threads",
        type=int,
        default=default,
        help="logical thread budget; defaults to the historical Exp.3 auto-thread budget",
    )


def resolve_mode(
    args: argparse.Namespace,
    *,
    quick_seeds: int = 1,
    full_seeds: int = 5,
    quick_timeout: int = 30,
    full_timeout: int = 120,
) -> tuple[bool, int, int]:
    quick = args.quick or not args.full
    seeds = args.seeds if args.seeds is not None else (quick_seeds if quick else full_seeds)
    timeout = args.timeout if args.timeout is not None else (quick_timeout if quick else full_timeout)
    return quick, seeds, timeout


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text())


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n")


def robot_path(robot: str) -> Path:
    return DATA / f"{robot}.json"


def load_robot_joint_limits(robot: str) -> tuple[list[float], list[float]]:
    raw = load_json(robot_path(robot))
    limits = raw["joint_limits"]
    lo = [float(limit[0]) for limit in limits]
    hi = [float(limit[1]) for limit in limits]
    return lo, hi


def marcucci_workload(scene_dir: Path = MARCUCCI_CONFIGS) -> list[dict[str, Any]]:
    workload: list[dict[str, Any]] = []
    for label, filename in MARCUCCI_QUERY_FILES:
        scene_path = scene_dir / filename
        scene = load_json(scene_path)
        workload.append(
            {
                "label": label,
                "file": filename,
                "path": scene_path,
                "scene": scene,
                "q_start": [float(v) for v in scene["q_start"]],
                "q_goal": [float(v) for v in scene["q_goal"]],
                "robot": scene["robot"],
            }
        )
    return workload


def marcucci_anchor_cycle(workload: list[dict[str, Any]]) -> list[dict[str, Any]]:
    if not workload:
        return []
    anchors: list[dict[str, Any]] = []
    first = workload[0]
    start_name, _ = first["label"].split("->", 1)
    anchors.append({"name": start_name, "q": list(first["q_start"])})
    for item in workload:
        _, goal_name = item["label"].split("->", 1)
        anchors.append({"name": goal_name, "q": list(item["q_goal"])})
    return anchors


def resolve_gcs_repo() -> Path:
    candidates = [
        WORKSPACE / "gcs-science-robotics",
        ROOT.parent / "gcs-science-robotics",
    ]
    for candidate in candidates:
        if candidate.is_dir():
            return candidate
    raise FileNotFoundError("gcs-science-robotics not found in workspace")


def bootstrap_gcs_repo() -> Path:
    repo = resolve_gcs_repo()
    repo_str = str(repo)
    if repo_str not in sys.path:
        sys.path.insert(0, repo_str)
    return repo


def configure_parser_package_map(parser, *, gcs_repo: Path | None = None) -> None:
    repo = gcs_repo if gcs_repo is not None else resolve_gcs_repo()
    package_map = parser.package_map()
    package_map.Add("gcs", str(repo))

    drake_package_dir = os.environ.get("DRAKE_PACKAGE_DIR")
    if not drake_package_dir:
        return

    drake_path = Path(drake_package_dir)
    if not drake_path.is_dir():
        return

    target = drake_path.resolve(strict=False)
    if hasattr(package_map, "Contains") and package_map.Contains("drake"):
        current = Path(package_map.GetPath("drake")).resolve(strict=False)
        if current != target and hasattr(package_map, "Remove"):
            package_map.Remove("drake")

    if not (hasattr(package_map, "Contains") and package_map.Contains("drake")):
        package_map.Add("drake", str(drake_path))


def empty_query_record(label: str, filename: str, *, seed: int, note: str) -> dict[str, Any]:
    return {
        "query": label,
        "file": filename,
        "seed": seed,
        "success": False,
        "time_s": None,
        "path_length": None,
        "note": note,
    }


def resolve_output(args: argparse.Namespace, default_name: str) -> Path:
    if args.out is not None:
        return args.out
    return RESULTS_PAPER / default_name


def aggregate_method_trials(
    *,
    method: str,
    scene: str,
    quick: bool,
    seeds: int,
    params: dict[str, Any],
    seed_trials: list[dict[str, Any]],
) -> dict[str, Any]:
    build_samples: list[float] = []
    query_times: list[float] = []
    query_paths: list[float] = []
    success_count = 0
    total_queries = 0

    for trial in seed_trials:
        build_s = trial.get("build_s")
        if isinstance(build_s, (int, float)):
            build_samples.append(float(build_s))
        for query in trial.get("queries", []):
            total_queries += 1
            if query.get("success"):
                success_count += 1
                time_s = query.get("time_s")
                path_length = query.get("path_length")
                if isinstance(time_s, (int, float)):
                    query_times.append(float(time_s))
                if isinstance(path_length, (int, float)):
                    query_paths.append(float(path_length))

    summary = {
        "build_s_mean": mean(build_samples) if build_samples else None,
        "build_s_median": median(build_samples) if build_samples else None,
        "query_time_s_mean": mean(query_times) if query_times else None,
        "query_time_s_median": median(query_times) if query_times else None,
        "query_path_rad_mean": mean(query_paths) if query_paths else None,
        "query_path_rad_std": _stddev(query_paths),
        "sr": (100.0 * success_count / total_queries) if total_queries else 0.0,
        "n_queries": total_queries,
        "n_success": success_count,
    }
    return {
        "method": method,
        "scene": scene,
        "quick": quick,
        "seeds": seeds,
        "params": params,
        "queries": [{"name": label, "file": filename} for label, filename in MARCUCCI_QUERY_FILES],
        "seed_trials": seed_trials,
        "summary": summary,
    }


def ordered_parallel_map(worker, tasks: list[Any], *, max_workers: int) -> list[Any]:
    if max_workers <= 1 or len(tasks) <= 1:
        return [worker(task) for task in tasks]

    results: list[Any] = [None] * len(tasks)
    with ProcessPoolExecutor(max_workers=min(max_workers, len(tasks))) as executor:
        future_to_index = {
            executor.submit(worker, task): index for index, task in enumerate(tasks)
        }
        for future in as_completed(future_to_index):
            results[future_to_index[future]] = future.result()
    return results


def _stddev(xs: list[float]) -> float | None:
    if not xs:
        return None
    if len(xs) == 1:
        return 0.0
    mu = mean(xs)
    return (sum((x - mu) ** 2 for x in xs) / len(xs)) ** 0.5
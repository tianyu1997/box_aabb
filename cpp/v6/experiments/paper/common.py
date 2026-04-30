#!/usr/bin/env python3
"""Shared helpers for v6 paper experiment scripts.

The paper policy is one top-level script per experimental environment.
This module is not an experiment by itself; it only standardises paths,
logging, and subprocess execution.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path
from statistics import mean, median
from typing import Any, Iterable, Sequence

ROOT = Path(__file__).resolve().parents[2]
BUILD_RELEASE = ROOT / "build-release"
BUILD_DEBUG = ROOT / "build"
CFG = ROOT / "experiments" / "configs"
OUT_DEFAULT = ROOT / "experiments" / "results_paper"
PYTHON_SRC = ROOT / "python"
DATA = ROOT / "data"
WORKSPACE = ROOT.parents[1]
PAPER_THREADS = 8
PAPER_CPUSET = tuple(range(PAPER_THREADS))
MARCUCCI_CONFIGS = CFG / "marcucci"
MARCUCCI_QUERY_FILES: list[tuple[str, str]] = [
    ("AS->TS", "AS_TS.json"),
    ("TS->CS", "TS_CS.json"),
    ("CS->LB", "CS_LB.json"),
    ("LB->RB", "LB_RB.json"),
    ("RB->AS", "RB_AS.json"),
]

PAPER_STATISTICS_POLICY: dict[str, Any] = {
    "resource": {
        "logical_threads": PAPER_THREADS,
        "cpu_affinity": list(PAPER_CPUSET),
        "seed_execution": "serial",
    },
    "exp3": {
        "build_metric": "complete_build_time_s_including_cache_load_or_miss",
        "cache_hit_metric": "same_route_cached_query_replay",
        "volume_metric": "deduplicated_box_volume_sum",
    },
    "exp4": {
        "sbf_build_metric": "coverage_build_time_including_prebridged_query_pairs",
        "sbf_query_metric": "cached_query_after_build",
        "prm_query_metric": "second_solve_plus_ompl_simplify_after_roadmap_build",
        "bitstar_budget_s": 10.0,
        "iris_collision_validation": "drake_scene_graph_edge_check_on_returned_gcs_path",
        "iris_path_repair": "invalid_gcs_segments_repaired_by_local_collision_checked_rrt_connect_and_counted_in_query_time",
    },
    "exp5": {
        "sbf_metric": "per_scene_build_matching_exp3_build_settings_and_cached_query_after_untimed_lect_prewarm",
        "baseline_metric": "SBF (CritSample+LinkIAABB and IFK+LinkIAABB) and OMPL use v6-native runners; IRIS rows use Drake IRIS/GCS with generated OBJ collision URDFs",
        "table_metric": "mean_over_retained_scenes_and_seeds_for_build_query_and_path_length",
        "bitstar_budget_s": 10.0,
    },
}


def apply_paper_resource_limits() -> None:
    """Pin paper runs to the global 8-thread/8-core budget."""
    for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
        os.environ[name] = str(PAPER_THREADS)
    if hasattr(os, "sched_setaffinity"):
        available = os.cpu_count() or PAPER_THREADS
        cpus = {cpu for cpu in PAPER_CPUSET if cpu < available}
        if cpus:
            try:
                os.sched_setaffinity(0, cpus)
            except OSError:
                pass


def _normalize_build_dir(build_dir: Path) -> Path:
    return build_dir if build_dir.is_absolute() else (ROOT / build_dir)


def _cmake_build_type(build_dir: Path) -> str | None:
    value = _cmake_cache_value(build_dir, "CMAKE_BUILD_TYPE")
    return value or None


def _cmake_cache_value(build_dir: Path, key: str) -> str | None:
    cache_path = build_dir / "CMakeCache.txt"
    if not cache_path.is_file():
        return None
    for line in cache_path.read_text().splitlines():
        prefix = f"{key}:"
        if line.startswith(prefix) and "=" in line:
            value = line.split("=", 1)[1].strip()
            return value or None
    return None


def _ensure_build_matches_root(build_dir: Path) -> None:
    source_dir = _cmake_cache_value(build_dir, "CMAKE_HOME_DIRECTORY")
    if not source_dir:
        return
    resolved_source = Path(source_dir).resolve(strict=False)
    resolved_root = ROOT.resolve(strict=False)
    if resolved_source != resolved_root:
        raise RuntimeError(
            "Refusing to use build tree "
            f"{build_dir} because it was configured for {resolved_source}, "
            f"not {resolved_root}."
        )


def _ensure_non_debug_build(build_dir: Path, *, allow_debug: bool) -> None:
    if allow_debug:
        return
    if _cmake_build_type(build_dir) == "Debug":
        raise RuntimeError(
            "Refusing to run paper experiments against Debug binaries at "
            f"{build_dir}. Build Release binaries in build-release or pass "
            "--allow-debug-build for debugging-only runs."
        )


def _resolve_build_dir(args: argparse.Namespace) -> Path:
    cached = getattr(args, "_resolved_build_dir", None)
    if cached is not None:
        return cached

    requested = getattr(args, "build_dir", None)
    allow_debug = bool(getattr(args, "allow_debug_build", False))
    if requested is not None:
        build_dir = _normalize_build_dir(requested)
    else:
        build_dir = BUILD_RELEASE

    if not build_dir.is_dir():
        raise FileNotFoundError(
            f"Required paper build tree does not exist: {build_dir}. "
            "Configure and build the Release tree at build-release before running paper experiments."
        )
    if build_dir.resolve(strict=False) != BUILD_RELEASE.resolve(strict=False) and not allow_debug:
        raise RuntimeError(
            "Paper experiments use the unified build-release tree. "
            f"Refusing alternate build tree {build_dir}; pass --allow-debug-build only for explicit diagnostics."
        )

    _ensure_build_matches_root(build_dir)
    _ensure_non_debug_build(build_dir, allow_debug=allow_debug)
    args._resolved_build_dir = build_dir
    return build_dir


def add_common_args(parser: argparse.ArgumentParser) -> None:
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--quick", action="store_true", help="3 seeds / short timeout smoke mode")
    mode.add_argument("--full", action="store_true", help="paper mode; script-specific seed defaults")
    parser.add_argument(
        "--build-dir",
        type=Path,
        default=None,
        help="CMake build directory; paper runs default to and require build-release.",
    )
    parser.add_argument(
        "--allow-debug-build",
        action="store_true",
        help="allow Debug CMake binaries for debugging-only runs; disabled by default to protect timing experiments",
    )
    parser.add_argument("--out-dir", type=Path, default=OUT_DEFAULT)
    parser.add_argument("--seeds", type=int, default=None)
    parser.add_argument("--timeout", type=int, default=None)
    parser.add_argument("--dry-run", action="store_true")


def mode_args(args: argparse.Namespace, *, quick_seeds: int = 3,
              full_seeds: int = 20, quick_timeout: int = 30,
              full_timeout: int = 120) -> tuple[int, int, str]:
    apply_paper_resource_limits()
    if args.quick:
        seeds = quick_seeds if args.seeds is None else args.seeds
        timeout = quick_timeout if args.timeout is None else args.timeout
        mode = "--quick"
    else:
        seeds = full_seeds if args.seeds is None else args.seeds
        timeout = full_timeout if args.timeout is None else args.timeout
        mode = "--full" if args.full else ""
    return seeds, timeout, mode


def add_mode_args(parser: argparse.ArgumentParser) -> None:
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--quick", action="store_true")
    mode.add_argument("--full", action="store_true")
    parser.add_argument("--seeds", type=int, default=None)
    parser.add_argument("--timeout", type=int, default=None)
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--dry-run", action="store_true")


def resolve_mode(
    args: argparse.Namespace,
    *,
    quick_seeds: int = 1,
    full_seeds: int = 5,
    quick_timeout: int = 30,
    full_timeout: int = 120,
) -> tuple[bool, int, int]:
    apply_paper_resource_limits()
    quick = bool(args.quick or not args.full)
    seeds = args.seeds if args.seeds is not None else (quick_seeds if quick else full_seeds)
    timeout = args.timeout if args.timeout is not None else (quick_timeout if quick else full_timeout)
    return quick, int(seeds), int(timeout)


def add_logical_threads_arg(
    parser: argparse.ArgumentParser,
    *,
    default: int = PAPER_THREADS,
) -> None:
    parser.add_argument("--logical-threads", type=int, default=default)


def parse_cpu_affinity(raw: str) -> list[int]:
    cpus: list[int] = []
    for piece in raw.split(","):
        part = piece.strip()
        if not part:
            continue
        if "-" in part:
            lo, hi = part.split("-", 1)
            cpus.extend(range(int(lo), int(hi) + 1))
        else:
            cpus.append(int(part))
    return sorted(set(cpus))


def current_cpu_affinity() -> list[int] | None:
    if not hasattr(os, "sched_getaffinity"):
        return None
    return sorted(int(cpu) for cpu in os.sched_getaffinity(0))


def apply_cpu_affinity(raw: str | None) -> list[int] | None:
    apply_paper_resource_limits()
    if raw and hasattr(os, "sched_setaffinity"):
        os.sched_setaffinity(0, set(parse_cpu_affinity(raw)))
    return current_cpu_affinity()


def paper_resource_record(logical_threads: int = PAPER_THREADS) -> dict[str, Any]:
    return {
        "logical_threads": int(logical_threads),
        "cpu_affinity": current_cpu_affinity(),
        "seed_execution": "serial",
    }


def resolve_output(args: argparse.Namespace, default_name: str) -> Path:
    if args.out is not None:
        return args.out
    return OUT_DEFAULT / default_name


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
                "q_start": [float(value) for value in scene["q_start"]],
                "q_goal": [float(value) for value in scene["q_goal"]],
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
    candidates = [WORKSPACE / "gcs-science-robotics", ROOT.parent / "gcs-science-robotics"]
    for candidate in candidates:
        if candidate.is_dir():
            return candidate
    raise FileNotFoundError("gcs-science-robotics not found in workspace")


def bootstrap_gcs_repo() -> Path:
    repo = resolve_gcs_repo()
    repo_text = str(repo)
    if repo_text not in sys.path:
        sys.path.insert(0, repo_text)
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


def resolve_experiment_binary(
    name: str,
    *,
    requested_build_dir: Path | None = None,
    allow_debug: bool = False,
) -> Path:
    candidate_dirs = [
        _normalize_build_dir(requested_build_dir) if requested_build_dir is not None else BUILD_RELEASE,
    ]
    found_debug_binary: Path | None = None
    searched: list[Path] = []
    for candidate in candidate_dirs:
        if candidate is None:
            continue
        build_dir = candidate
        searched.append(build_dir / "experiments" / name)
        _ensure_build_matches_root(build_dir)
        binary = build_dir / "experiments" / name
        if not binary.is_file():
            continue
        if _cmake_build_type(build_dir) == "Debug" and not allow_debug:
            found_debug_binary = binary
            continue
        return binary
    if found_debug_binary is not None:
        raise RuntimeError(
            "Refusing to use Debug binary for paper timing: "
            f"{found_debug_binary}. Pass --allow-debug-build only for smoke diagnostics."
        )
    joined = ", ".join(str(path) for path in searched)
    raise FileNotFoundError(f"{name} not found in v6 build trees: {joined}")


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


def _stddev(values: list[float]) -> float | None:
    if not values:
        return None
    if len(values) == 1:
        return 0.0
    mu = mean(values)
    return (sum((value - mu) ** 2 for value in values) / len(values)) ** 0.5


def run(
    cmd: Sequence[str | Path], *, dry_run: bool = False,
    env: dict[str, str] | None = None,
    cwd: Path | None = None,
) -> None:
    text = " ".join(str(c) for c in cmd)
    print(f"$ {text}")
    if dry_run:
        return
    subprocess.run([str(c) for c in cmd], check=True, cwd=cwd or ROOT, env=env)


def python_env(
    extra_pythonpath: Iterable[Path] = (), *,
    build_dir: Path | None = None,
) -> dict[str, str]:
    apply_paper_resource_limits()
    env = os.environ.copy()
    for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
        env[name] = str(PAPER_THREADS)
    pythonpath_parts: list[str] = []
    build_python = (build_dir / "python") if build_dir is not None else (BUILD_RELEASE / "python")
    # Paper runs must use the selected build tree's extension.
    for candidate in (build_python, PYTHON_SRC, *extra_pythonpath):
        if Path(candidate).exists():
            pythonpath_parts.append(str(candidate))
    existing = env.get("PYTHONPATH")
    if existing:
        pythonpath_parts.append(existing)
    env["PYTHONPATH"] = os.pathsep.join(pythonpath_parts)
    return env


def require_python_extension(args: argparse.Namespace) -> Path:
    build_dir = _resolve_build_dir(args)
    with_python = _cmake_cache_value(build_dir, "SBF_WITH_PYTHON")
    if with_python == "OFF":
        raise RuntimeError(
            "The selected v6 build tree was configured with SBF_WITH_PYTHON=OFF. "
            f"Reconfigure {build_dir} with -DSBF_WITH_PYTHON=ON before running "
            "Python-backed paper experiments."
        )
    python_dir = build_dir / "python"
    extensions = sorted(python_dir.glob("_sbf6_cpp*.so"))
    if not extensions:
        raise FileNotFoundError(
            f"No _sbf6_cpp extension found under {python_dir}; build the Release "
            "Python binding target before running paper experiments."
        )
    return python_dir


def python_cmd(script: Path, *script_args: str | Path) -> list[str | Path]:
    return [sys.executable, script, *script_args]


def run_python(
    script: Path,
    script_args: Sequence[str | Path], *,
    dry_run: bool = False,
    build_dir: Path | None = None,
) -> None:
    run(
        python_cmd(script, *script_args),
        dry_run=dry_run,
        env=python_env(build_dir=build_dir),
        cwd=ROOT,
    )


def bin_path(args: argparse.Namespace, name: str) -> Path:
    build_dir = _resolve_build_dir(args)
    binary = build_dir / "experiments" / name
    if not binary.is_file():
        raise FileNotFoundError(
            f"{name} not found at {binary}; build the Release target first or pass "
            "--build-dir to a different CMake build tree."
        )
    return binary


def write_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n")


def load_json(path: Path) -> dict:
    return json.loads(path.read_text())


def existing(paths: Iterable[Path]) -> list[Path]:
    return [p for p in paths if p.exists()]

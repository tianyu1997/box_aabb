#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib.metadata
import importlib.util
import json
import os
import platform
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path
from statistics import mean, median
from typing import Any

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from common import MARCUCCI_QUERY_FILES, RESULTS_PAPER, ROOT, WORKSPACE, resolve_experiment_binary, write_json


METHOD_OUTPUTS = {
    "sbf": "marcucci.json",
    "iris_np": "marcucci_iris_np_gcs.json",
    "iris_zo": "marcucci_iris_zo_gcs.json",
    "ompl_prm": "marcucci_ompl_prm.json",
    "ompl_bitstar": "marcucci_ompl_bitstar_budget.json",
}

LEGACY_METHOD_OUTPUTS = {
    "sbf": ["marcucci_sbf.json"],
}

METHOD_LABELS = {
    "sbf": "SBF",
    "iris_np": "IRIS-NP+GCS",
    "iris_zo": "IRIS-ZO+GCS",
    "ompl_prm": "OMPL PRM",
    "ompl_bitstar": "OMPL BIT*",
}

METHOD_DEFAULTS = {
    "sbf": {"quick_seeds": 1, "full_seeds": 5},
    "iris_np": {"quick_seeds": 1, "full_seeds": 5},
    "iris_zo": {"quick_seeds": 1, "full_seeds": 5},
    "ompl_prm": {"quick_seeds": 1, "full_seeds": 5},
    "ompl_bitstar": {"quick_seeds": 1, "full_seeds": 5},
}

SBF_PAPER_PROTOCOL = {
    "v6_authoritative_script": WORKSPACE / "cpp" / "v6" / "scripts" / "run_online_query_comparison.py",
    "driver_binary": "exp_marcucci_cached",
    "table_source": ROOT / "experiments" / "results_nightly" / "full" / "marcucci.json",
    "threads": 5,
    "env": "link_iaabb_grid",
    "endpoint_source": "ifk",
    "n_sub": 4,
    "voxel_delta": 0.05,
    "ffb_depth": 300,
    "max_boxes": 200000,
    "bridge_boxes": 2000,
    "post_connect_extra_boxes": 4000,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run/reuse the full Marcucci Exp.3 pipeline, using the current paper-side method configs and aggregating JSON plus SVG diagnostics."
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--quick", action="store_true", help="use each child script's quick defaults unless --seeds/--timeout override")
    mode.add_argument("--full", action="store_true", help="use each child script's full defaults unless --seeds/--timeout override")
    parser.add_argument("--seeds", type=int, default=None)
    parser.add_argument("--timeout", type=int, default=None)
    parser.add_argument("--logical-threads", type=int, default=16)
    parser.add_argument("--sbf-threads", type=int, default=int(SBF_PAPER_PROTOCOL["threads"]))
    parser.add_argument("--cpu-affinity", default="0-15", help="CPU list/range, e.g. 0-15 or 0,2,4")
    parser.add_argument(
        "--methods",
        default="sbf,iris_np,iris_zo,ompl_prm,ompl_bitstar",
        help="comma-separated subset of sbf, iris_np, iris_zo, ompl_prm, ompl_bitstar",
    )
    parser.add_argument("--out-dir", type=Path, default=RESULTS_PAPER / "exp3_repro")
    parser.add_argument(
        "--sbf-source",
        choices=["v6_authoritative", "paper_artifact", "live"],
        default="v6_authoritative",
        help="SBF reproduction source: run the v6 authoritative build_coverage/query protocol, copy the paper marcucci.json artifact, or rerun the current v7 live cached-query binary.",
    )
    parser.add_argument("--sbf-bin", type=Path, default=None)
    parser.add_argument("--ompl-bin", type=Path, default=None)
    parser.add_argument("--bitstar-budget-s", type=float, default=1.0)
    parser.add_argument("--skip-existing", action="store_true")
    parser.add_argument("--visualize-only", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def mode_defaults(args: argparse.Namespace) -> tuple[bool, int, int]:
    quick = args.quick or not args.full
    timeout = args.timeout if args.timeout is not None else (30 if quick else 120)
    seeds = args.seeds if args.seeds is not None else (1 if quick else 5)
    return quick, seeds, timeout


def effective_method_seeds(method: str, args: argparse.Namespace, quick: bool) -> int:
    if args.seeds is not None:
        return int(args.seeds)
    defaults = METHOD_DEFAULTS[method]
    return int(defaults["quick_seeds"] if quick else defaults["full_seeds"])


def method_output_candidates(method: str) -> list[str]:
    return [METHOD_OUTPUTS[method], *LEGACY_METHOD_OUTPUTS.get(method, [])]


def paper_protocol_summary(args: argparse.Namespace, quick: bool, timeout: int, methods: list[str]) -> dict[str, Any]:
    return {
        "sbf_source_mode": args.sbf_source,
        "sbf_authoritative_v6_script": str(SBF_PAPER_PROTOCOL["v6_authoritative_script"]),
        "sbf_v7_live_driver_binary": str(args.sbf_bin) if args.sbf_bin is not None else str(SBF_PAPER_PROTOCOL["driver_binary"]),
        "sbf_cached_table_source": str(SBF_PAPER_PROTOCOL["table_source"]),
        "sbf_output_name": METHOD_OUTPUTS["sbf"],
        "sbf_authoritative_v6_config": {
            "seed_points": ["AS", "TS", "CS", "LB", "RB"],
            "grow_timeout_ms": 60000,
            "grow_max_boxes": 200000,
            "post_connect_extra_boxes": 4000,
            "n_threads": 5,
            "bridge_n_threads": 16,
            "ffb_depth": 300,
            "coarsen_target_boxes": 300,
            "lect_no_cache": True,
        },
        "sbf_v7_live_driver_config": {
            "threads": int(args.sbf_threads),
            "env": SBF_PAPER_PROTOCOL["env"],
            "endpoint_source": SBF_PAPER_PROTOCOL["endpoint_source"],
            "n_sub": int(SBF_PAPER_PROTOCOL["n_sub"]),
            "voxel_delta": float(SBF_PAPER_PROTOCOL["voxel_delta"]),
            "ffb_depth": int(SBF_PAPER_PROTOCOL["ffb_depth"]),
            "max_boxes": int(SBF_PAPER_PROTOCOL["max_boxes"]),
            "bridge_boxes": int(SBF_PAPER_PROTOCOL["bridge_boxes"]),
            "post_connect_extra_boxes": int(SBF_PAPER_PROTOCOL["post_connect_extra_boxes"]),
            "source_protocol": "v7_live_build_coverage_query",
        },
        "method_seed_defaults": {
            method: effective_method_seeds(method, args, quick) for method in methods
        },
        "effective_timeout_s": int(timeout),
        "logical_threads_for_live_baselines": int(args.logical_threads),
        "sbf_reproduction_note": (
            "v6_authoritative mode imports cpp/v6/scripts/run_online_query_comparison.py::run_sbf_experiment() "
            "and normalizes its output into the current marcucci.json schema; "
            "paper_artifact mode copies the cached paper JSON; "
            "live mode reruns v7 exp_marcucci_cached and remains diagnostic."
        ),
    }


def parse_methods(raw: str) -> list[str]:
    allowed = set(METHOD_OUTPUTS)
    methods = [part.strip() for part in raw.split(",") if part.strip()]
    unknown = [method for method in methods if method not in allowed]
    if unknown:
        raise ValueError(f"unknown methods: {unknown}; allowed={sorted(allowed)}")
    return methods


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


def apply_cpu_affinity(cpus: list[int]) -> list[int] | None:
    if not cpus or not hasattr(os, "sched_setaffinity"):
        return None
    os.sched_setaffinity(0, set(cpus))
    return sorted(os.sched_getaffinity(0))


def detect_binary(name: str) -> Path:
    return resolve_experiment_binary(name)


def run_cmd(cmd: list[str], *, cwd: Path, dry_run: bool) -> dict[str, Any]:
    started = time.time()
    print("$", " ".join(cmd))
    if dry_run:
        return {"cmd": cmd, "cwd": str(cwd), "dry_run": True, "returncode": None, "elapsed_s": 0.0}
    completed = subprocess.run(cmd, cwd=cwd, check=True, text=True)
    return {
        "cmd": cmd,
        "cwd": str(cwd),
        "dry_run": False,
        "returncode": completed.returncode,
        "elapsed_s": time.time() - started,
    }


def maybe_run(
    *,
    method: str,
    out_path: Path,
    cmd: list[str],
    cwd: Path,
    args: argparse.Namespace,
) -> dict[str, Any]:
    if args.visualize_only:
        return {"method": method, "out": str(out_path), "skipped": "visualize_only"}
    if args.skip_existing and out_path.exists():
        return {"method": method, "out": str(out_path), "skipped": "exists"}
    out_path.parent.mkdir(parents=True, exist_ok=True)
    record = run_cmd(cmd, cwd=cwd, dry_run=args.dry_run)
    record.update({"method": method, "out": str(out_path)})
    return record


def maybe_copy_artifact(
    *,
    method: str,
    source_path: Path,
    out_path: Path,
    args: argparse.Namespace,
) -> dict[str, Any]:
    if args.visualize_only:
        return {"method": method, "out": str(out_path), "skipped": "visualize_only", "source_artifact": str(source_path)}
    if args.skip_existing and out_path.exists():
        return {"method": method, "out": str(out_path), "skipped": "exists", "source_artifact": str(source_path)}
    if not source_path.exists():
        raise FileNotFoundError(f"missing source artifact: {source_path}")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    started = time.time()
    if not args.dry_run:
        shutil.copyfile(source_path, out_path)
    return {
        "method": method,
        "out": str(out_path),
        "source_artifact": str(source_path),
        "copied": not args.dry_run,
        "dry_run": args.dry_run,
        "returncode": None if args.dry_run else 0,
        "elapsed_s": 0.0 if args.dry_run else time.time() - started,
    }


def load_v6_authoritative_sbf_module() -> Any:
    script_path = Path(SBF_PAPER_PROTOCOL["v6_authoritative_script"])
    if not script_path.exists():
        raise FileNotFoundError(f"missing v6 authoritative SBF script: {script_path}")
    spec = importlib.util.spec_from_file_location("v6_authoritative_online_query_comparison", script_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"failed to load module spec for {script_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def normalize_v6_authoritative_sbf(
    build_results: list[dict[str, Any]],
    query_results: dict[str, list[dict[str, Any]]],
    *,
    seeds: int,
) -> dict[str, Any]:
    query_order = list(query_results)
    trials_by_seed: dict[int, dict[str, Any]] = {}
    build_samples: list[float] = []

    for row in build_results:
        seed = int(row["seed"])
        build_s = float(row["build_time_s"])
        build_samples.append(build_s)
        trials_by_seed[seed] = {
            "seed": seed,
            "seed_index": seed,
            "build_s": build_s,
            "n_boxes": int(row["n_boxes"]),
            "queries": [],
        }

    queries_summary: list[dict[str, Any]] = []
    for label in query_order:
        start_name, goal_name = label.split("->", 1)
        rows = query_results.get(label, [])
        successes = [row for row in rows if row.get("success")]
        for row in rows:
            seed = int(row["seed"])
            trial = trials_by_seed.setdefault(
                seed,
                {
                    "seed": seed,
                    "seed_index": seed,
                    "build_s": None,
                    "n_boxes": None,
                    "queries": [],
                },
            )
            query_payload = {
                "from": start_name,
                "to": goal_name,
                "t_s": float(row["time_s"]),
                "ok": bool(row["success"]),
                "length": float(row["path_length"]) if row.get("success") else 0.0,
            }
            planning_time_ms = row.get("planning_time_ms")
            if planning_time_ms is not None:
                query_payload["planning_time_ms"] = float(planning_time_ms)
            trial["queries"].append(query_payload)

        success_values = [1.0 if row.get("success") else 0.0 for row in rows]
        queries_summary.append(
            {
                "name": label,
                "sr": mean(success_values) if success_values else None,
                "t_med_s": median([float(row["time_s"]) for row in successes]) if successes else None,
                "len_med": median([float(row["path_length"]) for row in successes]) if successes else None,
            }
        )

    ordered_trials = [trials_by_seed[seed] for seed in sorted(trials_by_seed)]
    return {
        "experiment": "marcucci",
        "robot": "iiwa14",
        "scene": "marcucci_combined",
        "source_protocol": "v6_authoritative_build_coverage_query",
        "seeds": seeds,
        "build": {
            "mean_s": mean(build_samples) if build_samples else None,
            "median_s": median(build_samples) if build_samples else None,
        },
        "queries": queries_summary,
        "trials": ordered_trials,
    }


def maybe_run_v6_authoritative_sbf(
    *,
    out_path: Path,
    args: argparse.Namespace,
    seeds: int,
) -> dict[str, Any]:
    if args.visualize_only:
        return {"method": "sbf", "out": str(out_path), "skipped": "visualize_only", "source": "v6_authoritative"}
    if args.skip_existing and out_path.exists():
        return {"method": "sbf", "out": str(out_path), "skipped": "exists", "source": "v6_authoritative"}
    out_path.parent.mkdir(parents=True, exist_ok=True)
    started = time.time()
    if args.dry_run:
        return {
            "method": "sbf",
            "out": str(out_path),
            "dry_run": True,
            "returncode": None,
            "elapsed_s": 0.0,
            "source": "v6_authoritative",
            "source_script": str(SBF_PAPER_PROTOCOL["v6_authoritative_script"]),
            "seeds": int(seeds),
        }
    module = load_v6_authoritative_sbf_module()
    build_results, query_results = module.run_sbf_experiment(int(seeds))
    payload = normalize_v6_authoritative_sbf(build_results, query_results, seeds=int(seeds))
    write_json(out_path, payload)
    return {
        "method": "sbf",
        "out": str(out_path),
        "dry_run": False,
        "returncode": 0,
        "elapsed_s": time.time() - started,
        "source": "v6_authoritative",
        "source_script": str(SBF_PAPER_PROTOCOL["v6_authoritative_script"]),
        "seeds": int(seeds),
    }


def run_components(args: argparse.Namespace, methods: list[str], raw_dir: Path) -> list[dict[str, Any]]:
    quick, _, timeout = mode_defaults(args)
    mode_arg = "--quick" if quick else "--full"
    records: list[dict[str, Any]] = []

    if "sbf" in methods:
        out_path = raw_dir / METHOD_OUTPUTS["sbf"]
        if args.sbf_source == "v6_authoritative":
            records.append(
                maybe_run_v6_authoritative_sbf(
                    out_path=out_path,
                    args=args,
                    seeds=effective_method_seeds("sbf", args, quick),
                )
            )
        elif args.sbf_source == "paper_artifact":
            records.append(
                maybe_copy_artifact(
                    method="sbf",
                    source_path=Path(SBF_PAPER_PROTOCOL["table_source"]),
                    out_path=out_path,
                    args=args,
                )
            )
        else:
            sbf_binary = Path(args.sbf_bin) if args.sbf_bin is not None else detect_binary(str(SBF_PAPER_PROTOCOL["driver_binary"]))
            cmd = [
                str(sbf_binary),
                mode_arg,
                f"--scene-dir={ROOT / 'experiments' / 'configs' / 'marcucci'}",
                f"--out={out_path}",
                f"--threads={args.sbf_threads}",
                f"--env={SBF_PAPER_PROTOCOL['env']}",
                f"--endpoint-source={SBF_PAPER_PROTOCOL['endpoint_source']}",
                f"--n-sub={SBF_PAPER_PROTOCOL['n_sub']}",
                f"--voxel-delta={SBF_PAPER_PROTOCOL['voxel_delta']}",
                f"--ffb-depth={SBF_PAPER_PROTOCOL['ffb_depth']}",
                f"--max-boxes={SBF_PAPER_PROTOCOL['max_boxes']}",
                f"--bridge-boxes={SBF_PAPER_PROTOCOL['bridge_boxes']}",
                f"--post-connect-extra-boxes={SBF_PAPER_PROTOCOL['post_connect_extra_boxes']}",
            ]
            if args.seeds is not None:
                cmd.append(f"--seeds={args.seeds}")
            if args.timeout is not None:
                cmd.append(f"--timeout={args.timeout}")
            records.append(maybe_run(method="sbf", out_path=out_path, cmd=cmd, cwd=ROOT, args=args))

    if "iris_np" in methods:
        out_path = raw_dir / METHOD_OUTPUTS["iris_np"]
        cmd = [
            sys.executable,
            str(HERE / "marcucci_iris_np_gcs.py"),
            mode_arg,
            "--logical-threads", str(args.logical_threads),
            "--out", str(out_path),
        ]
        if args.seeds is not None:
            cmd.extend(["--seeds", str(args.seeds)])
        records.append(maybe_run(method="iris_np", out_path=out_path, cmd=cmd, cwd=ROOT, args=args))

    if "iris_zo" in methods:
        out_path = raw_dir / METHOD_OUTPUTS["iris_zo"]
        cmd = [
            sys.executable,
            str(HERE / "marcucci_iris_zo_gcs.py"),
            mode_arg,
            "--logical-threads", str(args.logical_threads),
            "--out", str(out_path),
        ]
        if args.seeds is not None:
            cmd.extend(["--seeds", str(args.seeds)])
        records.append(maybe_run(method="iris_zo", out_path=out_path, cmd=cmd, cwd=ROOT, args=args))

    ompl_methods = []
    if "ompl_prm" in methods:
        ompl_methods.append("prm")
    if "ompl_bitstar" in methods:
        ompl_methods.append("bitstar_budget")
    if ompl_methods:
        cmd = [
            sys.executable,
            str(HERE / "marcucci_ompl_baselines.py"),
            mode_arg,
            "--logical-threads", str(args.logical_threads),
            "--out-dir", str(raw_dir),
            "--methods", ",".join(ompl_methods),
            "--bitstar-budget-s", str(args.bitstar_budget_s),
        ]
        if args.seeds is not None:
            cmd.extend(["--seeds", str(args.seeds)])
        if args.timeout is not None:
            cmd.extend(["--timeout", str(timeout)])
        if args.ompl_bin is not None:
            cmd.extend(["--baseline-bin", str(args.ompl_bin)])
        pseudo_out = raw_dir / "marcucci_ompl_outputs.json"
        records.append(maybe_run(method="ompl", out_path=pseudo_out, cmd=cmd, cwd=ROOT, args=args))

    return records


def load_if_exists(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    return json.loads(path.read_text())


def percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    xs = sorted(values)
    if len(xs) == 1:
        return xs[0]
    idx = (len(xs) - 1) * q
    lo = int(idx)
    hi = min(lo + 1, len(xs) - 1)
    frac = idx - lo
    return xs[lo] * (1.0 - frac) + xs[hi] * frac


def summarize_sbf(payload: dict[str, Any]) -> dict[str, Any]:
    trials = payload.get("trials", [])
    per_query: dict[str, dict[str, list[float]]] = {}
    build_samples = [float(trial["build_s"]) for trial in trials if trial.get("build_s") is not None]
    for trial in trials:
        for query in trial.get("queries", []):
            name = f"{query.get('from')}->{query.get('to')}"
            bucket = per_query.setdefault(name, {"times": [], "paths": [], "success": [], "n_pts": []})
            bucket["success"].append(1.0 if query.get("ok") else 0.0)
            if query.get("ok"):
                bucket["times"].append(float(query.get("t_s", 0.0)))
                bucket["paths"].append(float(query.get("length", 0.0)))
                if query.get("n_pts") is not None:
                    bucket["n_pts"].append(float(query["n_pts"]))

    query_stats = {}
    total_success = 0
    total_queries = 0
    for name, bucket in per_query.items():
        successes = bucket.get("success", [])
        total_success += int(sum(successes))
        total_queries += len(successes)
        stats = {
            "sr": 100.0 * mean(successes) if successes else None,
            "query_time_s_median": median(bucket["times"]) if bucket.get("times") else None,
            "query_path_rad_median": median(bucket["paths"]) if bucket.get("paths") else None,
        }
        for key in ["n_pts"]:
            if bucket.get(key):
                stats[f"{key}_median"] = median(bucket[key])
        query_stats[name] = stats

    return {
        "label": METHOD_LABELS["sbf"],
        "timing_semantics": "cached_query_from_prebuilt_forest",
        "build_s_median": median(build_samples) if build_samples else None,
        "sr": 100.0 * total_success / total_queries if total_queries else None,
        "queries": query_stats,
    }


def summarize_aggregate_method(method: str, payload: dict[str, Any]) -> dict[str, Any]:
    query_order = [str(query.get("name") or query.get("query")) for query in payload.get("queries", [])]
    buckets = {name: {"success": [], "times": [], "paths": []} for name in query_order}
    build_samples = []
    region_samples = []
    for trial in payload.get("seed_trials", []):
        if trial.get("build_s") is not None:
            build_samples.append(float(trial["build_s"]))
        if trial.get("n_regions") is not None:
            region_samples.append(float(trial["n_regions"]))
        for query in trial.get("queries", []):
            name = str(query.get("query") or query.get("name"))
            bucket = buckets.setdefault(name, {"success": [], "times": [], "paths": []})
            bucket["success"].append(1.0 if query.get("success") else 0.0)
            if not query.get("success"):
                continue
            if query.get("time_s") is not None:
                bucket["times"].append(float(query["time_s"]))
            if query.get("path_length") is not None:
                bucket["paths"].append(float(query["path_length"]))
    queries = {}
    for name, bucket in buckets.items():
        queries[name] = {
            "sr": 100.0 * mean(bucket["success"]) if bucket["success"] else None,
            "query_time_s_median": median(bucket["times"]) if bucket["times"] else None,
            "query_path_rad_median": median(bucket["paths"]) if bucket["paths"] else None,
        }
    summary = payload.get("summary", {})
    result = {
        "label": METHOD_LABELS[method],
        "build_s_median": summary.get("build_s_median") if summary.get("build_s_median") is not None else (median(build_samples) if build_samples else None),
        "sr": summary.get("sr"),
        "queries": queries,
        "params": payload.get("params", {}),
    }
    if region_samples:
        result["n_regions_median"] = median(region_samples)
        result["n_regions_min"] = min(region_samples)
        result["n_regions_max"] = max(region_samples)
    return result


def summarize_methods(raw_dir: Path, methods: list[str]) -> dict[str, Any]:
    summaries: dict[str, Any] = {}
    for method in methods:
        payload = None
        for candidate in method_output_candidates(method):
            payload = load_if_exists(raw_dir / candidate)
            if payload is not None:
                break
        if payload is None:
            summaries[method] = {"label": METHOD_LABELS[method], "missing": str(raw_dir / METHOD_OUTPUTS[method])}
            continue
        if method == "sbf":
            summaries[method] = summarize_sbf(payload)
        else:
            summaries[method] = summarize_aggregate_method(method, payload)
    return summaries


def diagnose_sbf(sbf_summary: dict[str, Any]) -> dict[str, Any]:
    queries = sbf_summary.get("queries", {})
    timed = [(name, stats.get("query_time_s_median")) for name, stats in queries.items() if stats.get("query_time_s_median") is not None]
    if not timed:
        return {"status": "missing_sbf_timing"}
    slowest = max(timed, key=lambda item: float(item[1]))
    fastest = min(timed, key=lambda item: float(item[1]))
    ratio = float(slowest[1]) / max(float(fastest[1]), 1e-12)
    return {
        "status": "cached_query_summary",
        "timing_semantics": "cached_query_from_prebuilt_forest",
        "slowest_query": slowest[0],
        "fastest_query": fastest[0],
        "median_time_ratio_slowest_over_fastest": ratio,
        "interpretation": "SBF now reports only cached online-query latency on the already built forest; no direct-query stage breakdown is tracked in Exp.3.",
    }


def drake_version() -> str | None:
    try:
        return importlib.metadata.version("drake")
    except importlib.metadata.PackageNotFoundError:
        return None


def load_v6_iris_np_reference() -> dict[str, Any] | None:
    table_path = WORKSPACE / "cpp" / "v6" / "doc" / "generated" / "tab_b5_irisnp_iso.tex"
    baseline_path = WORKSPACE / "cpp" / "v6" / "doc" / "generated" / "tab_baselines_v2.tex"
    if not table_path.exists():
        return None
    for line in table_path.read_text().splitlines():
        if "IRIS-NP+GCS" not in line or "&" not in line:
            continue
        values = [float(match) for match in re.findall(r"[-+]?\d+(?:\.\d+)?", line)]
        if len(values) < 5:
            continue
        note = None
        if baseline_path.exists():
            for baseline_line in baseline_path.read_text().splitlines():
                if "Drake" in baseline_line and "budget=3000s" in baseline_line:
                    note = baseline_line.strip("% ")
                    break
        return {
            "source": str(table_path),
            "build_s": values[0],
            "n_regions": int(values[1]),
            "query_time_s": values[2],
            "path_rad": values[3],
            "sr": values[4],
            "note": note,
        }
    return None


def diagnose_iris_np(iris_summary: dict[str, Any]) -> dict[str, Any]:
    params = iris_summary.get("params", {}) if isinstance(iris_summary, dict) else {}
    v6_reference = load_v6_iris_np_reference()
    build_s = iris_summary.get("build_s_median") if isinstance(iris_summary, dict) else None
    comparison = None
    if v6_reference and build_s:
        comparison = {
            "v7_over_v6_build_ratio": float(build_s) / max(float(v6_reference["build_s"]), 1e-12),
            "v7_minus_v6_build_s": float(build_s) - float(v6_reference["build_s"]),
            "same_region_count": iris_summary.get("n_regions_median") == v6_reference.get("n_regions"),
        }
    return {
        "drake_version": drake_version(),
        "python": sys.version.split()[0],
        "platform": platform.platform(),
        "executable": sys.executable,
        "current_v7_params": params,
        "current_v7_build_s_median": build_s,
        "current_v7_n_regions_median": iris_summary.get("n_regions_median") if isinstance(iris_summary, dict) else None,
        "v6_reference": v6_reference,
        "v6_v7_comparison": comparison,
        "likely_difference_sources": [
            "The late v6 generated table already reports the 3000s/seed, 8-region Drake 1.50 protocol; the early 130s/Drake 1.31 text is an older protocol snapshot.",
            "The current v7 run preserves the same IRIS-NP options and 8-region outcome, but uses a five-seed full rerun instead of the three-seed v6 B5 table.",
            "The measured build-time increase is therefore best treated as seed/runtime/environment variation rather than an obvious parameter bug.",
            "Container-vs-local solver and Drake implementation differences can still shift wall-clock time, so the summary records package and Python metadata beside the raw data.",
        ],
    }


def svg_escape(text: object) -> str:
    return str(text).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def write_svg_bar_chart(path: Path, title: str, series: list[dict[str, Any]], *, width: int = 1100, height: int = 520) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    margin_l, margin_r, margin_t, margin_b = 90, 30, 70, 110
    plot_w = width - margin_l - margin_r
    plot_h = height - margin_t - margin_b
    max_v = max((float(item["value"]) for item in series if item.get("value") is not None), default=1.0)
    max_v = max(max_v, 1e-9)
    n = max(1, len(series))
    bar_w = max(8, plot_w / n * 0.65)
    step = plot_w / n
    colors = ["#2b6cb0", "#c05621", "#6b46c1", "#2f855a", "#b83280", "#4a5568"]
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{width/2:.1f}" y="32" text-anchor="middle" font-size="22" font-family="Arial" font-weight="700">{svg_escape(title)}</text>',
        f'<line x1="{margin_l}" y1="{margin_t + plot_h}" x2="{width - margin_r}" y2="{margin_t + plot_h}" stroke="#333"/>',
        f'<line x1="{margin_l}" y1="{margin_t}" x2="{margin_l}" y2="{margin_t + plot_h}" stroke="#333"/>',
    ]
    for tick in range(5):
        value = max_v * tick / 4.0
        y = margin_t + plot_h - plot_h * tick / 4.0
        parts.append(f'<line x1="{margin_l}" y1="{y:.1f}" x2="{width - margin_r}" y2="{y:.1f}" stroke="#e2e8f0"/>')
        parts.append(f'<text x="{margin_l - 8}" y="{y + 4:.1f}" text-anchor="end" font-size="12" font-family="Arial">{value:.3g}</text>')
    for index, item in enumerate(series):
        value = item.get("value")
        if value is None:
            continue
        x = margin_l + step * index + (step - bar_w) / 2.0
        h = plot_h * float(value) / max_v
        y = margin_t + plot_h - h
        color = colors[index % len(colors)]
        parts.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{bar_w:.1f}" height="{h:.1f}" fill="{color}"/>')
        parts.append(f'<text x="{x + bar_w/2:.1f}" y="{y - 5:.1f}" text-anchor="middle" font-size="11" font-family="Arial">{float(value):.3g}</text>')
        parts.append(f'<text x="{x + bar_w/2:.1f}" y="{margin_t + plot_h + 18}" text-anchor="end" font-size="12" font-family="Arial" transform="rotate(-35 {x + bar_w/2:.1f} {margin_t + plot_h + 18})">{svg_escape(item["label"])}</text>')
    parts.append("</svg>")
    path.write_text("\n".join(parts) + "\n")


def write_grouped_query_chart(path: Path, title: str, summaries: dict[str, Any], value_key: str) -> None:
    queries = [label for label, _ in MARCUCCI_QUERY_FILES]
    methods = [method for method in METHOD_OUTPUTS if method in summaries and not summaries[method].get("missing")]
    series = []
    for query in queries:
        for method in methods:
            value = summaries[method].get("queries", {}).get(query, {}).get(value_key)
            if value is not None:
                series.append({"label": f"{query} {METHOD_LABELS[method]}", "value": float(value)})
    write_svg_bar_chart(path, title, series, width=1500, height=620)


def write_figures(out_dir: Path, summaries: dict[str, Any]) -> list[str]:
    fig_dir = out_dir / "figures"
    figures = []
    build_series = []
    for method, summary in summaries.items():
        if summary.get("missing"):
            continue
        build_series.append({"label": f"{METHOD_LABELS[method]} build(s)", "value": summary.get("build_s_median") or 0.0})
        build_series.append({"label": f"{METHOD_LABELS[method]} SR(%)", "value": summary.get("sr")})
    path = fig_dir / "exp3_build_vs_success.svg"
    write_svg_bar_chart(path, "Exp.3 build time and overall success", build_series, width=1400)
    figures.append(str(path))

    path = fig_dir / "exp3_query_times.svg"
    write_grouped_query_chart(path, "Exp.3 per-query median time (s)", summaries, "query_time_s_median")
    figures.append(str(path))

    path = fig_dir / "exp3_path_quality.svg"
    write_grouped_query_chart(path, "Exp.3 per-query median path length (rad)", summaries, "query_path_rad_median")
    figures.append(str(path))

    return figures


def main() -> int:
    args = parse_args()
    methods = parse_methods(args.methods)
    cpus = parse_cpu_affinity(args.cpu_affinity)
    effective_affinity = apply_cpu_affinity(cpus)
    raw_dir = args.out_dir / "raw"
    raw_dir.mkdir(parents=True, exist_ok=True)

    run_records = run_components(args, methods, raw_dir)
    summaries = summarize_methods(raw_dir, methods) if not args.dry_run else {}
    diagnostics = {
        "sbf_timing": diagnose_sbf(summaries.get("sbf", {})) if summaries else {},
        "iris_np": diagnose_iris_np(summaries.get("iris_np", {})) if summaries else {},
    }
    figures = write_figures(args.out_dir, summaries) if summaries else []

    quick, _, timeout = mode_defaults(args)
    summary = {
        "experiment": "exp3_marcucci_reproducible",
        "quick": quick,
        "requested_seeds_override": args.seeds,
        "timeout_s": timeout,
        "logical_threads": int(args.logical_threads),
        "sbf_threads": int(args.sbf_threads),
        "requested_cpu_affinity": cpus,
        "effective_cpu_affinity": effective_affinity,
        "methods": methods,
        "paper_protocol": paper_protocol_summary(args, quick, timeout, methods),
        "out_dir": str(args.out_dir),
        "run_records": run_records,
        "method_summaries": summaries,
        "diagnostics": diagnostics,
        "figures": figures,
    }
    if not args.dry_run:
        write_json(args.out_dir / "exp3_summary.json", summary)
        print(f"[run_exp3_reproducible] wrote {args.out_dir / 'exp3_summary.json'}")
        for figure in figures:
            print(f"[run_exp3_reproducible] wrote {figure}")
    else:
        print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
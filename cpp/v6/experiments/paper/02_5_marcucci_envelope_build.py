#!/usr/bin/env python3
"""Paper Exp. 2.5 — Marcucci envelope-pipeline build comparison.

Environment: Marcucci combined IIWA14 scene only, using the same planner-side
configuration defaults as Exp. 3.  Each cell first deletes its LECT cache and
runs a cold build for one seed, then immediately reruns the same seed with the
same cache path to measure a warm/cache-hit build.

Matrix:
  endpoint source: IFK, CritSample
    link envelope:   AABB S=4, Hull16-grid d=0.04

Outputs:
  - experiments/results_paper/marcucci_envelope_build/raw/*.json
  - experiments/results_paper/marcucci_envelope_build.json
"""
from __future__ import annotations

import argparse
import importlib.util
import shutil
import statistics
import sys
import time
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import ROOT, add_common_args, load_json, mode_args, write_json


ENDPOINTS = [
    ("ifk", "IFK"),
    ("critsample", "CritSample"),
]

AUTHORITATIVE_SCRIPT = ROOT / "scripts" / "run_online_query_comparison.py"

ENVELOPE_VARIANTS = [
    {
        "key": "aabb_s4",
        "label": "AABB S=4",
        "env": "link_iaabb",
        "n_sub": 4,
        "voxel_delta": 0.04,
    },
    {
        "key": "hull16_grid_d004",
        "label": "Hull16-grid d=0.04",
        "env": "hull16_grid",
        "n_sub": 1,
        "voxel_delta": 0.04,
    },
]


def median(values: list[float]) -> float:
    return float(statistics.median(values)) if values else 0.0


def mean(values: list[float]) -> float:
    return float(statistics.fmean(values)) if values else 0.0


def ensure_python_paths() -> None:
    for candidate in (ROOT / "build" / "python", ROOT / "python"):
        text = str(candidate)
        if candidate.exists() and text not in sys.path:
            sys.path.insert(0, text)


def load_authoritative_module() -> Any:
    ensure_python_paths()
    spec = importlib.util.spec_from_file_location(
        "v6_authoritative_online_query_comparison",
        AUTHORITATIVE_SCRIPT,
    )
    if spec is None or spec.loader is None:
        raise ImportError(f"Unable to load authoritative script at {AUTHORITATIVE_SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def import_sbf5() -> Any:
    ensure_python_paths()
    import sbf5  # type: ignore
    extension = sys.modules.get("sbf5._sbf5_cpp")
    if extension is not None and "_sbf5_cpp" not in sys.modules:
        sys.modules["_sbf5_cpp"] = extension
    return sbf5


def make_endpoint_config(sbf5: Any, endpoint: str) -> Any:
    cfg = sbf5.EndpointSourceConfig()
    cfg.source = {
        "ifk": sbf5.EndpointSource.IFK,
        "critsample": sbf5.EndpointSource.CritSample,
    }[endpoint]
    return cfg


def make_envelope_config(sbf5: Any, variant: dict[str, Any]) -> Any:
    cfg = sbf5.EnvelopeTypeConfig()
    cfg.type = {
        "link_iaabb": sbf5.EnvelopeType.LinkIAABB,
        "hull16_grid": sbf5.EnvelopeType.Hull16_Grid,
    }[variant["env"]]
    cfg.n_subdivisions = int(variant["n_sub"])
    if variant["env"] == "hull16_grid":
        cfg.grid_config.voxel_delta = float(variant["voxel_delta"])
    return cfg


def make_planner_config(
    sbf5: Any,
    authoritative: Any,
    *,
    endpoint: str,
    variant: dict[str, Any],
    seed: int,
    threads: int,
    timeout_s: int,
    ffb_depth: int,
    max_boxes: int,
    bridge_boxes: int,
    cache_dir: Path,
) -> Any:
    cfg = sbf5.SBFPlannerConfig()
    cfg.split_order = sbf5.SplitOrder.BEST_TIGHTEN
    cfg.z4_enabled = True
    cfg.lect_no_cache = False
    cfg.lect_cache_dir = str(cache_dir)
    cfg.endpoint_source = make_endpoint_config(sbf5, endpoint)
    cfg.envelope_type = make_envelope_config(sbf5, variant)
    cfg.grower.timeout_ms = float(timeout_s) * 1000.0
    cfg.grower.max_boxes = int(max_boxes)
    cfg.grower.post_connect_extra_boxes = int(bridge_boxes)
    cfg.grower.rng_seed = int(seed) * 1000 + 42
    cfg.grower.n_threads = max(1, int(threads))
    cfg.grower.bridge_n_threads = max(1, int(threads))
    cfg.grower.max_consecutive_miss = 2000
    cfg.grower.rrt_goal_bias = float(authoritative.DEFAULT_GOAL_BIAS)
    cfg.grower.rrt_step_ratio = 0.05
    cfg.grower.ffb_config.max_depth = int(ffb_depth)
    cfg.coarsen.target_boxes = 300
    cfg.coarsen.score_threshold = 500.0
    return cfg


def run_trial(
    *,
    endpoint: str,
    endpoint_label: str,
    variant: dict[str, Any],
    seed: int,
    cache_mode: str,
    cache_dir: Path,
    raw_path: Path,
    timeout_s: int,
    threads: int,
    ffb_depth: int,
    max_boxes: int,
    bridge_boxes: int,
) -> dict[str, Any]:
    sbf5 = import_sbf5()
    authoritative = load_authoritative_module()
    robot = sbf5.Robot.from_json(str(ROOT / "data" / "iiwa14.json"))
    obstacles = authoritative.make_combined_obstacles()
    seed_points = [authoritative.IIWA_CONFIGS[key] for key in ["AS", "TS", "CS", "LB", "RB"]]
    cache_available_before_build = any(cache_dir.glob("*.lect"))

    cfg = make_planner_config(
        sbf5,
        authoritative,
        endpoint=endpoint,
        variant=variant,
        seed=seed,
        threads=threads,
        timeout_s=timeout_s,
        ffb_depth=ffb_depth,
        max_boxes=max_boxes,
        bridge_boxes=bridge_boxes,
        cache_dir=cache_dir,
    )
    planner = sbf5.SBFPlanner(robot, cfg)

    t0 = time.perf_counter()
    planner.build_coverage(obstacles, float(timeout_s) * 1000.0, seed_points)
    build_s = time.perf_counter() - t0

    queries = []
    for label, start_name, goal_name in authoritative.QUERY_PAIRS:
        result = planner.query(
            authoritative.IIWA_CONFIGS[start_name],
            authoritative.IIWA_CONFIGS[goal_name],
        )
        queries.append(
            {
                "name": label,
                "from": start_name,
                "to": goal_name,
                "ok": bool(result.success),
                "planning_time_ms": float(result.planning_time_ms),
                "path_length": float(result.path_length) if result.success else 0.0,
            }
        )

    row = {
        "source_protocol": "v6_python_sbfplanner_build_query",
        "source_script": str(AUTHORITATIVE_SCRIPT),
        "endpoint_source": endpoint,
        "endpoint_label": endpoint_label,
        "envelope_key": variant["key"],
        "envelope_label": variant["label"],
        "env": variant["env"],
        "n_sub": variant["n_sub"],
        "voxel_delta": variant["voxel_delta"],
        "seed": seed,
        "cache_mode": cache_mode,
        "cache_dir": str(cache_dir),
        "raw_path": str(raw_path),
        "cache_available_before_build": bool(cache_available_before_build),
        "loaded_lect_cache": bool(cache_mode == "warm" and cache_available_before_build),
        "build_s": float(build_s),
        "n_boxes": int(planner.n_boxes()),
        "query_success_rate": query_success_rate(queries),
        "queries": queries,
    }
    write_json(raw_path, row)
    return row


def query_success_rate(queries: list[dict[str, Any]]) -> float:
    if not queries:
        return 0.0
    return sum(1 for query in queries if query.get("ok")) / len(queries)


def summarise(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    summary = []
    keys = sorted({
        (row["endpoint_source"], row["endpoint_label"], row["envelope_key"],
         row["envelope_label"], row["env"], row["n_sub"], row["voxel_delta"],
         row["cache_mode"])
        for row in rows
    })
    for endpoint, endpoint_label, env_key, env_label, env, n_sub, voxel_delta, cache_mode in keys:
        group = [
            row for row in rows
            if row["endpoint_source"] == endpoint
            and row["envelope_key"] == env_key
            and row["cache_mode"] == cache_mode
        ]
        builds = [float(row["build_s"]) for row in group]
        boxes = [float(row["n_boxes"]) for row in group]
        summary.append({
            "endpoint_source": endpoint,
            "endpoint_label": endpoint_label,
            "envelope_key": env_key,
            "envelope_label": env_label,
            "env": env,
            "n_sub": n_sub,
            "voxel_delta": voxel_delta,
            "cache_mode": cache_mode,
            "n_runs": len(group),
            "median_build_s": median(builds),
            "mean_build_s": mean(builds),
            "median_n_boxes": median(boxes),
            "loaded_cache_rate": mean([1.0 if row["loaded_lect_cache"] else 0.0 for row in group]),
            "query_success_rate_mean": mean([float(row["query_success_rate"]) for row in group]),
        })
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_common_args(parser)
    parser.add_argument("--threads", type=int, default=5)
    parser.add_argument("--ffb-depth", type=int, default=55)
    parser.add_argument("--max-boxes", type=int, default=2500)
    parser.add_argument("--bridge-boxes", type=int, default=2000)
    parser.add_argument("--resume", action="store_true",
                        help="skip a cold/warm pair when both raw JSON outputs already exist")
    args = parser.parse_args()

    seeds, timeout, mode = mode_args(args, quick_seeds=1, full_seeds=10,
                                     quick_timeout=30, full_timeout=120)
    base_dir = args.out_dir / "marcucci_envelope_build"
    raw_dir = base_dir / "raw"
    cache_dir = base_dir / "lect_cache"
    rows: list[dict[str, Any]] = []

    for endpoint, endpoint_label in ENDPOINTS:
        for variant in ENVELOPE_VARIANTS:
            for seed in range(seeds):
                stem = f"{endpoint}_{variant['key']}_seed{seed:03d}"
                cache_path = cache_dir / f"{stem}.bin"
                cold_out = raw_dir / f"{stem}_cold.json"
                warm_out = raw_dir / f"{stem}_warm.json"
                pair_done = cold_out.exists() and warm_out.exists()
                if args.resume and pair_done:
                    print(f"[skip] {stem} cold/warm")
                else:
                    if not args.dry_run and cache_path.exists():
                        shutil.rmtree(cache_path)
                    for cache_mode, out_path in (("cold", cold_out), ("warm", warm_out)):
                        if args.dry_run:
                            print(
                                "$ "
                                f"{sys.executable} {Path(__file__).name} {mode} "
                                f"--threads {args.threads} --ffb-depth {args.ffb_depth} "
                                f"--max-boxes {args.max_boxes} --bridge-boxes {args.bridge_boxes} "
                                f"[{endpoint} {variant['key']} seed={seed} {cache_mode}]"
                            )
                            continue
                        row = run_trial(
                            endpoint=endpoint,
                            endpoint_label=endpoint_label,
                            variant=variant,
                            seed=seed,
                            cache_mode=cache_mode,
                            cache_dir=cache_path,
                            raw_path=out_path,
                            timeout_s=timeout,
                            threads=args.threads,
                            ffb_depth=args.ffb_depth,
                            max_boxes=args.max_boxes,
                            bridge_boxes=args.bridge_boxes,
                        )
                        rows.append(row)

                if args.dry_run:
                    continue
                if args.resume and pair_done:
                    rows.append(load_json(cold_out))
                    rows.append(load_json(warm_out))

    if args.dry_run:
        print("[dry-run] commands emitted; no marcucci_envelope_build.json written")
        return

    out = {
        "experiment": "marcucci_envelope_build",
        "schema_version": 1,
        "scene": "marcucci_combined",
        "runner": "v6_python_sbfplanner",
        "source_script": str(AUTHORITATIVE_SCRIPT),
        "defaults": {
            "seeds": seeds,
            "timeout": timeout,
            "threads": args.threads,
            "ffb_depth": args.ffb_depth,
            "max_boxes": args.max_boxes,
            "bridge_boxes": args.bridge_boxes,
        },
        "endpoints": [{"key": key, "label": label} for key, label in ENDPOINTS],
        "envelope_variants": ENVELOPE_VARIANTS,
        "rows": rows,
        "summary": summarise(rows),
    }
    write_json(args.out_dir / "marcucci_envelope_build.json", out)
    print(f"[write] {args.out_dir / 'marcucci_envelope_build.json'}")


if __name__ == "__main__":
    main()
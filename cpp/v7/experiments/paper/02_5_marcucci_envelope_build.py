#!/usr/bin/env python3
"""Paper Exp. 2.5 — Marcucci envelope-pipeline build comparison.

Environment: Marcucci combined IIWA14 scene only, using the same planner-side
configuration defaults as Exp. 3.  Each cell first deletes its LECT cache and
runs a cold build for one seed, then immediately reruns the same seed with the
same cache path to measure a warm/cache-hit build.

Matrix:
  endpoint source: IFK, CritSample
  link envelope:   AABB S=4, AABB-grid S=4 d=0.04, Hull16-grid d=0.04

Outputs:
  - experiments/results_paper/marcucci_envelope_build/raw/*.json
  - experiments/results_paper/marcucci_envelope_build.json
"""
from __future__ import annotations

import argparse
import statistics
import sys
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import CFG, add_common_args, bin_path, load_json, mode_args, run, write_json


ENDPOINTS = [
    ("ifk", "IFK"),
    ("critsample", "CritSample"),
]

ENVELOPE_VARIANTS = [
    {
        "key": "aabb_s4",
        "label": "AABB S=4",
        "env": "link_iaabb",
        "n_sub": 4,
        "voxel_delta": 0.04,
    },
    {
        "key": "aabb_grid_s4_d004",
        "label": "AABB-grid S=4 d=0.04",
        "env": "link_iaabb_grid",
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


def raw_record(path: Path, *, endpoint: str, endpoint_label: str,
               variant: dict[str, Any], seed: int, cache_mode: str,
               cache_path: Path) -> dict[str, Any]:
    data = load_json(path)
    trials = data.get("trials", [])
    trial = trials[0] if trials else {}
    build_summary = data.get("build", {})
    return {
        "endpoint_source": endpoint,
        "endpoint_label": endpoint_label,
        "envelope_key": variant["key"],
        "envelope_label": variant["label"],
        "env": variant["env"],
        "n_sub": variant["n_sub"],
        "voxel_delta": variant["voxel_delta"],
        "seed": seed,
        "cache_mode": cache_mode,
        "cache_path": str(cache_path),
        "raw_path": str(path),
        "loaded_lect_cache": bool(trial.get("loaded_lect_cache", False)),
        "build_s": float(trial.get("build_s", data.get("build", {}).get("median_s", 0.0))),
        "planner_build_total_ms": float(trial.get("planner_build_total_ms", build_summary.get("median_planner_total_ms", 0.0))),
        "build_grow_time_ms": float(trial.get("build_grow_time_ms", build_summary.get("median_grow_ms", 0.0))),
        "build_adjacency_time_ms": float(trial.get("build_adjacency_time_ms", build_summary.get("median_adjacency_ms", 0.0))),
        "build_overhead_ms": float(trial.get("build_overhead_ms", build_summary.get("median_overhead_ms", 0.0))),
        "n_boxes": int(trial.get("n_boxes", 0)),
        "unique_box_count": int(trial.get("unique_box_count", build_summary.get("median_unique_box_count", 0))),
        "duplicate_box_count": int(trial.get("duplicate_box_count", build_summary.get("median_duplicate_box_count", 0))),
        "box_volume_sum": float(trial.get("box_volume_sum", build_summary.get("median_box_volume_sum", 0.0))),
        "unique_box_volume_sum": float(trial.get("unique_box_volume_sum", build_summary.get("median_unique_box_volume_sum", 0.0))),
        "dedup_box_volume_sum": float(trial.get("dedup_box_volume_sum", build_summary.get("median_dedup_box_volume_sum", 0.0))),
        "duplicate_box_volume_sum": float(trial.get("duplicate_box_volume_sum", build_summary.get("median_duplicate_box_volume_sum", 0.0))),
        "query_success_rate": query_success_rate(trial.get("queries", [])),
    }


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
        planner_build_total_ms = [float(row["planner_build_total_ms"]) for row in group]
        grow_ms = [float(row["build_grow_time_ms"]) for row in group]
        adjacency_ms = [float(row["build_adjacency_time_ms"]) for row in group]
        overhead_ms = [float(row["build_overhead_ms"]) for row in group]
        unique_box_counts = [float(row["unique_box_count"]) for row in group]
        duplicate_box_counts = [float(row["duplicate_box_count"]) for row in group]
        box_volumes = [float(row["box_volume_sum"]) for row in group]
        unique_box_volumes = [float(row["unique_box_volume_sum"]) for row in group]
        duplicate_box_volumes = [float(row["duplicate_box_volume_sum"]) for row in group]
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
            "median_planner_build_total_ms": median(planner_build_total_ms),
            "mean_planner_build_total_ms": mean(planner_build_total_ms),
            "median_build_grow_time_ms": median(grow_ms),
            "mean_build_grow_time_ms": mean(grow_ms),
            "median_build_adjacency_time_ms": median(adjacency_ms),
            "mean_build_adjacency_time_ms": mean(adjacency_ms),
            "median_build_overhead_ms": median(overhead_ms),
            "mean_build_overhead_ms": mean(overhead_ms),
            "median_n_boxes": median(boxes),
            "median_unique_box_count": median(unique_box_counts),
            "median_duplicate_box_count": median(duplicate_box_counts),
            "median_box_volume_sum": median(box_volumes),
            "median_unique_box_volume_sum": median(unique_box_volumes),
            "median_dedup_box_volume_sum": median(unique_box_volumes),
            "median_duplicate_box_volume_sum": median(duplicate_box_volumes),
            "loaded_cache_rate": mean([1.0 if row["loaded_lect_cache"] else 0.0 for row in group]),
            "query_success_rate_mean": mean([float(row["query_success_rate"]) for row in group]),
        })
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_common_args(parser)
    parser.add_argument("--threads", type=int, default=1)
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
                        cache_path.unlink()
                    for cache_mode, out_path in [("cold", cold_out), ("warm", warm_out)]:
                        cmd: list[str | Path] = [bin_path(args, "exp_marcucci_cached")]
                        if mode:
                            cmd.append(mode)
                        cmd += [
                            f"--scene-dir={CFG / 'marcucci'}",
                            f"--out={out_path}",
                            "--seeds=1",
                            f"--seed-offset={seed}",
                            f"--timeout={timeout}",
                            f"--threads={args.threads}",
                            f"--env={variant['env']}",
                            f"--endpoint-source={endpoint}",
                            f"--n-sub={variant['n_sub']}",
                            f"--voxel-delta={variant['voxel_delta']}",
                            f"--ffb-depth={args.ffb_depth}",
                            f"--max-boxes={args.max_boxes}",
                            f"--bridge-boxes={args.bridge_boxes}",
                            f"--lect-cache={cache_path}",
                        ]
                        run(cmd, dry_run=args.dry_run)

                if args.dry_run:
                    continue
                rows.append(raw_record(cold_out, endpoint=endpoint,
                                       endpoint_label=endpoint_label,
                                       variant=variant, seed=seed,
                                       cache_mode="cold",
                                       cache_path=cache_path))
                rows.append(raw_record(warm_out, endpoint=endpoint,
                                       endpoint_label=endpoint_label,
                                       variant=variant, seed=seed,
                                       cache_mode="warm",
                                       cache_path=cache_path))

    if args.dry_run:
        print("[dry-run] commands emitted; no marcucci_envelope_build.json written")
        return

    out = {
        "experiment": "marcucci_envelope_build",
        "schema_version": 1,
        "scene": "marcucci_combined",
        "runner": "exp_marcucci_cached",
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
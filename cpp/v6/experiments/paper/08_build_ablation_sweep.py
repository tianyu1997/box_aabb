#!/usr/bin/env python3
"""Run full-build SBF ablations with a fixed 16-thread/16-core budget."""
from __future__ import annotations

import argparse
import json
import statistics
import subprocess
import sys
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import PAPER_THREADS, ROOT, add_common_args, apply_paper_resource_limits, bin_path, load_json, mode_args, write_json


CPUSET = "0-15"


def median(values: list[float]) -> float:
    return float(statistics.median(values)) if values else 0.0


def case_matrix() -> list[dict[str, Any]]:
    return [
        {"key": "baseline", "group": "baseline", "args": []},
        {"key": "no_rescue_bridge", "group": "ablation", "args": ["--no-rescue-bridge"]},
        {"key": "no_seed_bridge", "group": "ablation", "args": ["--no-seed-bridge"]},
        {"key": "no_seed_no_coarsen", "group": "combined", "args": ["--no-seed-bridge", "--no-coarsen"]},
        {"key": "no_seed_no_coarsen_ffb80", "group": "combined", "args": ["--no-seed-bridge", "--no-coarsen", "--ffb-depth", "80"]},
        {"key": "no_seed_no_coarsen_ffb200", "group": "combined", "args": ["--no-seed-bridge", "--no-coarsen", "--ffb-depth", "200"]},
        {"key": "no_seed_no_rescue", "group": "combined", "args": ["--no-seed-bridge", "--no-rescue-bridge"]},
        {"key": "no_seed_bridge_boxes_0", "group": "combined", "args": ["--no-seed-bridge", "--bridge-boxes", "0"]},
        {"key": "no_seed_partitioned", "group": "combined", "args": ["--no-seed-bridge", "--partitioned"]},
        {"key": "no_unexplored", "group": "ablation", "args": ["--no-unexplored"]},
        {"key": "no_coarsen", "group": "ablation", "args": ["--no-coarsen"]},
        {"key": "force_bridge", "group": "bridge_budget", "args": ["--force-bridge"]},
        {"key": "bridge_boxes_0", "group": "bridge_budget", "args": ["--bridge-boxes", "0"]},
        {"key": "bridge_boxes_250", "group": "bridge_budget", "args": ["--bridge-boxes", "250"]},
        {"key": "bridge_boxes_1000", "group": "bridge_budget", "args": ["--bridge-boxes", "1000"]},
        {"key": "ffb_depth_80", "group": "bridge_budget", "args": ["--ffb-depth", "80"]},
        {"key": "ffb_depth_200", "group": "bridge_budget", "args": ["--ffb-depth", "200"]},
        {"key": "coarsen_score_100", "group": "coarsen", "args": ["--coarsen-score", "100"]},
        {"key": "coarsen_score_1000", "group": "coarsen", "args": ["--coarsen-score", "1000"]},
        {"key": "coarsen_target_600", "group": "coarsen", "args": ["--coarsen-target", "600"]},
        {"key": "parallel_legacy", "group": "parallel", "args": ["--no-coordinated-grower"]},
        {"key": "parallel_partitioned", "group": "parallel", "args": ["--partitioned"]},
    ]


def summarize_case(payload: dict[str, Any]) -> dict[str, Any]:
    rows = payload.get("build_results", [])
    def samples(name: str) -> list[float]:
        return [float(row.get(name, 0.0) or 0.0) for row in rows]

    success_counts = [float(row.get("query_success", 0.0) or 0.0) for row in rows]
    n_queries = 5.0
    return {
        "total_ms_median": median(samples("total_ms")),
        "lect_ms_median": median(samples("lect_ms")),
        "grow_ms_median": median(samples("grow_ms")),
        "coarsen1_ms_median": median(samples("coarsen1_ms")),
        "bridge_ms_median": median(samples("bridge_ms")),
        "coarsen2_ms_median": median(samples("coarsen2_ms")),
        "filter_ms_median": median(samples("filter_ms")),
        "adjacency_ms_median": median(samples("adjacency_ms")),
        "seed_bridge_ms_median": median(samples("seed_bridge_ms")),
        "boxes_final_median": median(samples("boxes_final")),
        "boxes_after_grow_median": median(samples("boxes_after_grow")),
        "boxes_after_coarsen1_median": median(samples("boxes_after_coarsen1")),
        "boxes_after_bridge_median": median(samples("boxes_after_bridge")),
        "islands_median": median(samples("islands")),
        "query_success_rate": (sum(success_counts) / (len(success_counts) * n_queries)) if success_counts else 0.0,
        "query_mean_s_median": median(samples("query_mean_s")),
        "query_mean_length_median": median(samples("query_mean_length")),
    }


def run_case(binary: Path, args: argparse.Namespace, case: dict[str, Any], raw_dir: Path, *, seeds: int) -> dict[str, Any]:
    out_path = raw_dir / f"{case['key']}.json"
    cmd = [
        "taskset", "-c", CPUSET,
        str(binary),
        "--scene", args.scene,
        "--endpoint", args.endpoint,
        "--envelope", args.envelope,
        "--n-sub", str(args.n_sub),
        "--voxel-delta", str(args.voxel_delta),
        "--seeds", str(seeds),
        "--threads", str(PAPER_THREADS),
        "--max-boxes", str(args.max_boxes),
        "--bridge-boxes", str(args.bridge_boxes),
        "--ffb-depth", str(args.ffb_depth),
        "--coarsen-target", str(args.coarsen_target),
        "--coarsen-score", str(args.coarsen_score),
        "--skip-per-query",
        "--json", str(out_path),
        *case["args"],
    ]
    if args.lect_cache:
        cmd.extend(["--lect-cache", "--cache-root", str(args.cache_root)])
    print("$ " + " ".join(cmd))
    if not args.dry_run:
        subprocess.run(cmd, cwd=ROOT, check=True)
    payload = {} if args.dry_run else load_json(out_path)
    return {"case": case, "json": str(out_path), "summary": summarize_case(payload) if payload else {}}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_common_args(parser)
    parser.add_argument("--scene", default="combined")
    parser.add_argument("--endpoint", default="ifk", choices=["ifk", "critsample", "analytical"])
    parser.add_argument("--envelope", default="linkiaabb", choices=["linkiaabb", "hull16_grid", "hull16", "grid"])
    parser.add_argument("--n-sub", type=int, default=4)
    parser.add_argument("--voxel-delta", type=float, default=0.04)
    parser.add_argument("--max-boxes", type=int, default=2500)
    parser.add_argument("--bridge-boxes", type=int, default=500)
    parser.add_argument("--ffb-depth", type=int, default=120)
    parser.add_argument("--coarsen-target", type=int, default=300)
    parser.add_argument("--coarsen-score", type=float, default=500.0)
    parser.add_argument("--lect-cache", action="store_true", help="enable persistent LECT/cache without deleting it")
    parser.add_argument("--cache-root", type=Path, default=ROOT / "experiments" / "results_paper" / "build_ablation_lect_cache")
    args = parser.parse_args()

    apply_paper_resource_limits()
    seeds, _timeout, _mode = mode_args(args, quick_seeds=1, full_seeds=3, quick_timeout=60, full_timeout=60)
    binary = bin_path(args, "exp6_build_timing")
    out_dir = args.out_dir / "build_ablation_sweep"
    raw_dir = out_dir / "raw"
    raw_dir.mkdir(parents=True, exist_ok=True)

    cases = case_matrix()
    results = [run_case(binary, args, case, raw_dir, seeds=seeds) for case in cases]
    payload = {
        "experiment": "build_ablation_sweep",
        "schema_version": 1,
        "threads": PAPER_THREADS,
        "cpuset": CPUSET,
        "seeds": seeds,
        "params": {
            "scene": args.scene,
            "endpoint": args.endpoint,
            "envelope": args.envelope,
            "n_sub": args.n_sub,
            "voxel_delta": args.voxel_delta,
            "max_boxes": args.max_boxes,
            "bridge_boxes": args.bridge_boxes,
            "ffb_depth": args.ffb_depth,
            "coarsen_target": args.coarsen_target,
            "coarsen_score": args.coarsen_score,
            "lect_cache": bool(args.lect_cache),
            "cache_root": str(args.cache_root),
        },
        "results": results,
    }
    write_json(args.out_dir / "build_ablation_sweep.json", payload)
    print(f"[write] {args.out_dir / 'build_ablation_sweep.json'}")


if __name__ == "__main__":
    main()
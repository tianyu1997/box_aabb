#!/usr/bin/env python3
"""Lightweight Exp. 3 max-box / FFB-depth budget scan.

The scan runs only the cold-fill build/query phase so it can explore budgets
without overwriting the official same-route cache replay result. It records
whether 1500 boxes are enough for high-quality Marcucci paths and whether a
larger miss budget removes endpoint-dependent early stopping.
"""
from __future__ import annotations

import argparse
import contextlib
import os
import statistics
import time
from pathlib import Path
from typing import Any

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import PAPER_THREADS, add_common_args, mode_args, require_python_extension, write_json
from marcucci_envelope_build_replay import ENDPOINTS, ENVELOPE_VARIANTS, run_trial


SCAN_PHASE = {
    "key": "budget_scan",
    "label": "Cold budget/depth scan",
    "use_v6_cache": True,
    "strict": False,
    "preprobe": False,
    "paper_compare": False,
}


@contextlib.contextmanager
def redirect_native_output(path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as handle:
        stdout_fd = os.dup(1)
        stderr_fd = os.dup(2)
        try:
            os.dup2(handle.fileno(), 1)
            os.dup2(handle.fileno(), 2)
            yield
        finally:
            os.dup2(stdout_fd, 1)
            os.dup2(stderr_fd, 2)
            os.close(stdout_fd)
            os.close(stderr_fd)


def parse_int_list(text: str) -> list[int]:
    return [int(part.strip()) for part in text.split(",") if part.strip()]


def parse_variant_filter(text: str) -> list[dict[str, Any]]:
    if text == "all":
        return list(ENVELOPE_VARIANTS)
    keys = {part.strip() for part in text.split(",") if part.strip()}
    variants = [variant for variant in ENVELOPE_VARIANTS if variant["key"] in keys]
    if not variants:
        raise ValueError(f"No envelope variants selected by {text!r}")
    return variants


def median(values: list[float]) -> float | None:
    return float(statistics.median(values)) if values else None


def trial_summary(row: dict[str, Any], *, max_boxes: int, ffb_depth: int, max_miss: int | None) -> dict[str, Any]:
    queries = list(row.get("queries", []))
    path_lengths = [float(query.get("path_length", 0.0)) for query in queries if query.get("ok")]
    timing = row.get("build_timing", {})
    raw_count = int(row.get("raw_box_count", 0) or row.get("n_boxes", 0) or 0)
    return {
        "endpoint_source": row.get("endpoint_source"),
        "endpoint_label": row.get("endpoint_label"),
        "envelope_key": row.get("envelope_key"),
        "envelope_label": row.get("envelope_label"),
        "max_boxes": int(max_boxes),
        "ffb_depth": int(ffb_depth),
        "max_miss": max_miss,
        "seed": int(row.get("seed", 0)),
        "build_s": float(row.get("build_s", 0.0) or 0.0),
        "grow_ms": float(timing.get("grow_ms", 0.0) or 0.0),
        "n_boxes": int(row.get("n_boxes", 0) or 0),
        "raw_box_count": raw_count,
        "box_budget_hit": raw_count >= int(0.98 * max_boxes),
        "query_success_rate": float(row.get("query_success_rate", 0.0) or 0.0),
        "path_length_sum": float(sum(path_lengths)) if path_lengths else None,
        "path_length_median": median(path_lengths),
        "path_length_max": max(path_lengths) if path_lengths else None,
        "route_hash": row.get("raw_route_hash"),
        "v6_cache_ep_misses": int(row.get("v6_cache_ep_misses", 0) or 0),
        "v6_cache_grid_misses": int(row.get("v6_cache_grid_misses", 0) or 0),
        "native_log_path": row.get("native_log_path"),
        "raw_path": row.get("raw_path"),
    }


def annotate_quality(rows: list[dict[str, Any]]) -> None:
    best_by_group: dict[tuple[str, str], float] = {}
    for row in rows:
        if float(row.get("query_success_rate", 0.0) or 0.0) < 1.0:
            continue
        total = row.get("path_length_sum")
        if total is None:
            continue
        group = (str(row.get("endpoint_source")), str(row.get("envelope_key")))
        best_by_group[group] = min(float(total), best_by_group.get(group, float("inf")))
    for row in rows:
        group = (str(row.get("endpoint_source")), str(row.get("envelope_key")))
        best = best_by_group.get(group)
        total = row.get("path_length_sum")
        row["best_path_length_sum_for_endpoint_variant"] = best
        row["within_2pct_best_path_sum"] = (
            best is not None and total is not None and float(total) <= 1.02 * best
        )
        row["high_quality"] = bool(
            float(row.get("query_success_rate", 0.0) or 0.0) >= 1.0
            and row["within_2pct_best_path_sum"]
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    add_common_args(parser)
    parser.add_argument("--max-boxes-list", default="1000,1500,2000,2500")
    parser.add_argument("--ffb-depth-list", default="32,50,80")
    parser.add_argument("--max-miss-list", default="2000,10000")
    parser.add_argument("--variants", default="aabb_s4", help="comma-separated keys or 'all'")
    parser.add_argument("--threads", type=int, default=PAPER_THREADS)
    parser.add_argument("--bridge-threads", type=int, default=PAPER_THREADS)
    parser.add_argument("--bridge-boxes", type=int, default=500)
    parser.add_argument("--output-name", default="marcucci_envelope_budget_scan.json")
    parser.add_argument("--cache-run-id", default=None)
    args = parser.parse_args()

    args.out_dir = args.out_dir.resolve()
    seed_count, timeout_s, _mode = mode_args(args, quick_seeds=1, full_seeds=2, quick_timeout=30, full_timeout=60)
    require_python_extension(args)
    max_boxes_values = parse_int_list(args.max_boxes_list)
    ffb_depth_values = parse_int_list(args.ffb_depth_list)
    max_miss_values = [None if value < 0 else value for value in parse_int_list(args.max_miss_list)]
    variants = parse_variant_filter(args.variants)
    run_id = args.cache_run_id or time.strftime("exp3_scan_%Y%m%d_%H%M%S")
    raw_root = args.out_dir / "raw" / "marcucci_envelope_budget_scan" / run_id
    cache_root = raw_root / "cache"
    log_root = raw_root / "logs"

    rows: list[dict[str, Any]] = []
    raw_rows: list[dict[str, Any]] = []
    for max_boxes in max_boxes_values:
        for ffb_depth in ffb_depth_values:
            for max_miss in max_miss_values:
                miss_tag = "default" if max_miss is None else str(max_miss)
                for endpoint, endpoint_label in ENDPOINTS:
                    for variant in variants:
                        for seed in range(seed_count):
                            stem = f"{endpoint}_{variant['key']}_mb{max_boxes}_d{ffb_depth}_mm{miss_tag}_s{seed:03d}"
                            print(f"[scan] {stem}")
                            native_log_path = log_root / f"{stem}.log"
                            with redirect_native_output(native_log_path):
                                row = run_trial(
                                    endpoint=endpoint,
                                    endpoint_label=endpoint_label,
                                    variant=variant,
                                    seed=seed,
                                    phase=SCAN_PHASE,
                                    cache_dir=cache_root / stem,
                                    raw_path=raw_root / f"{stem}.json",
                                    timeout_s=int(timeout_s),
                                    threads=int(args.threads),
                                    bridge_threads=int(args.bridge_threads),
                                    ffb_depth=int(ffb_depth),
                                    max_boxes=int(max_boxes),
                                    bridge_boxes=int(args.bridge_boxes),
                                    max_miss=max_miss,
                                )
                            row["native_log_path"] = str(native_log_path)
                            raw_rows.append(row)
                            rows.append(
                                trial_summary(
                                    row,
                                    max_boxes=max_boxes,
                                    ffb_depth=ffb_depth,
                                    max_miss=max_miss,
                                )
                            )
    annotate_quality(rows)
    payload = {
        "experiment": "marcucci_envelope_budget_scan",
        "source_script": "03_1_marcucci_envelope_budget_scan.py",
        "run_id": run_id,
        "seeds": seed_count,
        "timeout_s": int(timeout_s),
        "threads": int(args.threads),
        "bridge_threads": int(args.bridge_threads),
        "bridge_boxes": int(args.bridge_boxes),
        "max_boxes_list": max_boxes_values,
        "ffb_depth_list": ffb_depth_values,
        "max_miss_list": max_miss_values,
        "variants": [variant["key"] for variant in variants],
        "rows": rows,
        "raw_rows": raw_rows,
    }
    out_path = args.out_dir / args.output_name
    write_json(out_path, payload)
    print(f"[write] {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

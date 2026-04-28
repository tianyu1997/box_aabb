#!/usr/bin/env python3
"""Lightweight Exp. 2 depth-32 growth calibration.

This runner measures the real 16-thread SBF grow path for the retained Exp. 2
link-envelope families, then estimates the depth-32 build time from observed
node creation throughput.  It is deliberately separate from the full Exp. 3
same-route replay so Exp. 2 storage/build estimates can be refreshed cheaply.
"""
from __future__ import annotations

import argparse
import statistics
import time
from pathlib import Path
from typing import Any

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import PAPER_THREADS, add_common_args, mode_args, require_python_extension, write_json
from marcucci_envelope_build_replay import run_trial


CALIBRATION_PHASE = {
    "key": "growth_calibration",
    "label": "Cold 16-thread grow calibration",
    "use_v6_cache": True,
    "strict": False,
    "preprobe": False,
    "paper_compare": False,
}

VARIANTS = [
    {"key": "aabb_s1", "label": "AABB S=1", "env": "link_iaabb", "n_sub": 1, "voxel_delta": 0.05},
    {"key": "aabb_s2", "label": "AABB S=2", "env": "link_iaabb", "n_sub": 2, "voxel_delta": 0.05},
    {"key": "aabb_s4", "label": "AABB S=4", "env": "link_iaabb", "n_sub": 4, "voxel_delta": 0.05},
    {"key": "aabb_s8", "label": "AABB S=8", "env": "link_iaabb", "n_sub": 8, "voxel_delta": 0.05},
    {"key": "hull16_grid_d002", "label": "Hull16-grid d=0.02", "env": "hull16_grid", "n_sub": 1, "voxel_delta": 0.02},
    {"key": "hull16_grid_d004", "label": "Hull16-grid d=0.04", "env": "hull16_grid", "n_sub": 1, "voxel_delta": 0.04},
    {"key": "hull16_grid_d006", "label": "Hull16-grid d=0.06", "env": "hull16_grid", "n_sub": 1, "voxel_delta": 0.06},
    {"key": "hull16_grid_d008", "label": "Hull16-grid d=0.08", "env": "hull16_grid", "n_sub": 1, "voxel_delta": 0.08},
]


def median(values: list[float]) -> float:
    return float(statistics.median(values)) if values else 0.0


def p90(values: list[float]) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(0.9 * (len(ordered) - 1)))
    return float(ordered[index])


def timing_value(row: dict[str, Any], field: str) -> float:
    return float(row.get("build_timing", {}).get(field, 0.0) or 0.0)


def effective_new_nodes(row: dict[str, Any]) -> int:
    value = int(timing_value(row, "grow_expand_new_nodes"))
    if value > 0:
        return value
    return max(0, int(row.get("raw_box_count", 0) or row.get("n_boxes", 0) or 0))


def effective_expand_seconds(row: dict[str, Any]) -> float:
    profile_s = timing_value(row, "grow_expand_profile_total_ms") / 1000.0
    if profile_s > 0.0:
        return profile_s
    grow_expand_s = timing_value(row, "grow_expand_ms") / 1000.0
    if grow_expand_s > 0.0:
        return grow_expand_s
    return float(row.get("build_s", 0.0) or 0.0)


def summarize_variant(rows: list[dict[str, Any]], *, variant: dict[str, Any], extrapolation_depth: int) -> dict[str, Any]:
    extrapolated_nodes = (1 << (extrapolation_depth + 1)) - 1
    node_rates = []
    per_node_ms = []
    for row in rows:
        nodes = effective_new_nodes(row)
        seconds = effective_expand_seconds(row)
        if nodes > 0 and seconds > 0.0:
            node_rates.append(nodes / seconds)
            per_node_ms.append(1000.0 * seconds / nodes)
    rate_median = median(node_rates)
    depth_build_s = extrapolated_nodes / rate_median if rate_median > 0.0 else None
    return {
        "variant_key": variant["key"],
        "envelope_label": variant["label"],
        "type": "Hull16_Grid" if variant["env"] == "hull16_grid" else "LinkIAABB",
        "n_subdivisions": int(variant["n_sub"]),
        "voxel_delta": float(variant["voxel_delta"]),
        "seeds": len(rows),
        "threads": PAPER_THREADS,
        "extrapolation_depth": extrapolation_depth,
        "extrapolated_nodes": extrapolated_nodes,
        "new_nodes_median": median([float(effective_new_nodes(row)) for row in rows]),
        "expand_profile_s_median": median([effective_expand_seconds(row) for row in rows]),
        "new_nodes_per_s_median": rate_median,
        "new_nodes_per_s_p90": p90(node_rates),
        "per_node_ms_median": median(per_node_ms),
        "depth_build_s_estimate": depth_build_s,
        "grow_expand_fk_ms_median": median([timing_value(row, "grow_expand_fk_ms") for row in rows]),
        "grow_expand_env_ms_median": median([timing_value(row, "grow_expand_env_ms") for row in rows]),
        "grow_expand_refine_ms_median": median([timing_value(row, "grow_expand_refine_ms") for row in rows]),
        "v6_cache_ep_hits_median": median([timing_value(row, "v6_cache_ep_hits") for row in rows]),
        "v6_cache_ep_misses_median": median([timing_value(row, "v6_cache_ep_misses") for row in rows]),
        "v6_cache_grid_hits_median": median([timing_value(row, "v6_cache_grid_hits") for row in rows]),
        "v6_cache_grid_misses_median": median([timing_value(row, "v6_cache_grid_misses") for row in rows]),
        "trial_paths": [str(row.get("raw_path", "")) for row in rows],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    add_common_args(parser)
    parser.add_argument("--ffb-depth", type=int, default=32)
    parser.add_argument("--max-boxes", type=int, default=600)
    parser.add_argument("--bridge-boxes", type=int, default=0)
    parser.add_argument("--threads", type=int, default=PAPER_THREADS)
    parser.add_argument("--bridge-threads", type=int, default=PAPER_THREADS)
    parser.add_argument("--extrapolation-depth", type=int, default=32)
    parser.add_argument("--variants", choices=["anchor", "all"], default="all")
    parser.add_argument("--output-name", default="link_envelope_growth_calibration.json")
    parser.add_argument("--cache-run-id", default=None)
    args = parser.parse_args()

    args.out_dir = args.out_dir.resolve()

    seed_count, timeout_s, _mode = mode_args(args, quick_seeds=1, full_seeds=2, quick_timeout=20, full_timeout=45)
    require_python_extension(args)
    variants = VARIANTS if args.variants == "all" else [VARIANTS[2], VARIANTS[5]]
    run_id = args.cache_run_id or time.strftime("exp2_growth_%Y%m%d_%H%M%S")
    cache_root = args.out_dir / "raw" / "link_envelope_growth_calibration" / run_id / "cache"
    raw_root = args.out_dir / "raw" / "link_envelope_growth_calibration" / run_id

    rows: list[dict[str, Any]] = []
    summaries: list[dict[str, Any]] = []
    for variant in variants:
        variant_rows = []
        for seed in range(seed_count):
            raw_path = raw_root / f"{variant['key']}_seed{seed}.json"
            row = run_trial(
                endpoint="critsample",
                endpoint_label="CritSample",
                variant=variant,
                seed=seed,
                phase=CALIBRATION_PHASE,
                cache_dir=cache_root / variant["key"] / f"seed{seed}",
                raw_path=raw_path,
                timeout_s=int(timeout_s),
                threads=int(args.threads),
                bridge_threads=int(args.bridge_threads),
                ffb_depth=int(args.ffb_depth),
                max_boxes=int(args.max_boxes),
                bridge_boxes=int(args.bridge_boxes),
            )
            variant_rows.append(row)
            rows.append(row)
        summaries.append(
            summarize_variant(
                variant_rows,
                variant=variant,
                extrapolation_depth=int(args.extrapolation_depth),
            )
        )

    payload = {
        "experiment": "link_envelope_growth_calibration",
        "source_script": "02_6_link_envelope_growth_calibration.py",
        "endpoint_source": "CritSample",
        "phase": CALIBRATION_PHASE,
        "run_id": run_id,
        "seeds": seed_count,
        "timeout_s": int(timeout_s),
        "ffb_depth": int(args.ffb_depth),
        "max_boxes": int(args.max_boxes),
        "bridge_boxes": int(args.bridge_boxes),
        "threads": int(args.threads),
        "bridge_threads": int(args.bridge_threads),
        "extrapolation_depth": int(args.extrapolation_depth),
        "cache_root": str(cache_root),
        "summaries": summaries,
        "rows": rows,
    }
    out_path = args.out_dir / args.output_name
    write_json(out_path, payload)
    print(f"[write] {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

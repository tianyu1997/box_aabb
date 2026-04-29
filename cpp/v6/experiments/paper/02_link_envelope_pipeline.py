#!/usr/bin/env python3
"""Paper Exp. 2 — link-envelope pipeline comparison.

Environment: reuse the Exp. 1 four-bin width-stratified IIWA14 box protocol,
with one shared random box set per width bin and CritSample fixed upstream.
Output:      experiments/results_paper/link_envelope_pipeline.json
Paper slot:  Experiments-B, LinkIAABB subdivision / Hull16-grid comparison.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import ROOT, add_common_args, load_json, mode_args, run_python, write_json


RAW_SCRIPT = ROOT / "python" / "scripts" / "run_s0c_width_sub_grid.py"
RAW_OUTPUT = "link_envelope_pipeline_v6_raw.json"
STORAGE_MODEL = {
    "name": "depth_synchronous_compact_node_payload_v2",
    "optimized_base_node_bytes": 64.0,
    "aabb_payload_formula": "compact fixed node record; endpoint evidence is shared and not scaled by raw cache-file slabs",
    "grid_payload_formula": "min(measured_payload_bytes, 64 + 8*bricks + voxels/8)",
    "note": "Result-side estimate for compressed/de-duplicated depth-32 storage; it does not change the .lect file format.",
}


def mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def optimized_payload_bytes(*, envelope: str, voxel_count: float, brick_count: float, measured_payload: float) -> float:
    if envelope != "Hull16_Grid" or measured_payload <= 0.0:
        return 0.0
    estimated = 64.0 + 8.0 * max(0.0, brick_count) + max(0.0, voxel_count) / 8.0
    return min(float(measured_payload), estimated)


def aggregate_rows(raw_rows: list[dict], *, envelope: str, subdivisions: int,
                   voxel_delta: float | None) -> dict:
    matched = [
        row for row in raw_rows
        if str(row.get("envelope")) == envelope
        and int(row.get("subdivisions", 0)) == subdivisions
        and ((voxel_delta is None and row.get("grid_delta") is None)
             or (voxel_delta is not None and abs(float(row.get("grid_delta", 0.0)) - voxel_delta) < 1e-9))
    ]
    if not matched:
        raise ValueError(
            f"Missing raw rows for envelope={envelope}, subdivisions={subdivisions}, voxel_delta={voxel_delta}."
        )
    return {
        "volume_mean": mean([float(row.get("volume_mean", 0.0)) for row in matched]),
        "time_us_mean": mean([float(row.get("total_us_mean", 0.0)) for row in matched]),
        "voxel_brick_count_mean": mean([float(row.get("cache_bricks_mean", 0.0)) for row in matched]),
        "voxel_count_mean": mean([float(row.get("cache_voxels_mean", 0.0)) for row in matched]),
        "cache_payload_bytes_mean": mean([
            float(row.get("cache_payload_bytes_mean", 0.0)) for row in matched
        ]),
    }


def translate_payload(raw: dict, *, n_boxes: int, repeats: int) -> dict:
    raw_rows = raw.get("rows", [])
    baseline = aggregate_rows(raw_rows, envelope="LinkIAABB", subdivisions=1, voxel_delta=None)
    baseline_volume = baseline["volume_mean"]

    desired_specs = [
        ("subdivision", "LinkIAABB", 1, None, "LinkIAABB"),
        ("subdivision", "LinkIAABB", 2, None, "LinkIAABB_S2"),
        ("subdivision", "LinkIAABB", 4, None, "LinkIAABB_S4"),
        ("subdivision", "LinkIAABB", 8, None, "LinkIAABB_S8"),
        ("grid", "Hull16_Grid", 1, 0.02, "Hull16Grid"),
        ("grid", "Hull16_Grid", 1, 0.04, "Hull16Grid"),
        ("grid", "Hull16_Grid", 1, 0.06, "Hull16Grid"),
        ("grid", "Hull16_Grid", 1, 0.08, "Hull16Grid"),
    ]

    rows = []
    for stage, envelope_type, subdivisions, voxel_delta, envelope_name in desired_specs:
        stats = aggregate_rows(
            raw_rows,
            envelope=envelope_type,
            subdivisions=subdivisions,
            voxel_delta=voxel_delta,
        )
        storage_bytes = optimized_payload_bytes(
            envelope=envelope_type,
            voxel_count=stats["voxel_count_mean"],
            brick_count=stats["voxel_brick_count_mean"],
            measured_payload=stats["cache_payload_bytes_mean"],
        )
        rows.append(
            {
                "stage": stage,
                "envelope": envelope_name,
                "type": envelope_type,
                "n_subdivisions": subdivisions,
                "voxel_delta": 0.05 if voxel_delta is None else voxel_delta,
                "volume_mean": stats["volume_mean"],
                "time_us_mean": stats["time_us_mean"],
                "voxel_brick_count_mean": stats["voxel_brick_count_mean"],
                "voxel_count_mean": stats["voxel_count_mean"],
                "cache_payload_bytes_mean": stats["cache_payload_bytes_mean"],
                "storage_bytes_optimized_mean": storage_bytes,
                "storage_payload_compression_ratio": (
                    storage_bytes / stats["cache_payload_bytes_mean"]
                    if stats["cache_payload_bytes_mean"] > 0.0 else None
                ),
                "ratio_to_linkiaabb": stats["volume_mean"] / baseline_volume if baseline_volume > 0 else 0.0,
            }
        )

    width_bins = []
    for row in raw.get("meta", {}).get("width_bins", []):
        width_bins.append(
            {
                "width_bin": f"{float(row.get('lo', 0.0)):.3f}-{float(row.get('hi', 0.0)):.2f}" if abs(float(row.get('lo', 0.0)) - 0.001) < 1e-9 else f"{float(row.get('lo', 0.0)):.2f}-{float(row.get('hi', 0.0)):.2f}",
                "width_lo": float(row.get("lo", 0.0)),
                "width_hi": float(row.get("hi", 0.0)),
                "n_boxes": int(n_boxes),
            }
        )

    return {
        "experiment": "link_envelope_pipeline",
        "robot": "iiwa14",
        "endpoint_source": "CritSample",
        "width_sampling_mode": "exp1_stratified_aggregate",
        "n_bins": len(width_bins),
        "n_boxes_per_bin": int(n_boxes),
        "n_boxes_total": int(n_boxes) * len(width_bins),
        "n_repeats": int(repeats),
        "storage_model": STORAGE_MODEL,
        "width_bins": width_bins,
        "rows": rows,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_common_args(parser)
    parser.add_argument("--n-boxes", type=int, default=None)
    parser.add_argument("--repeats", type=int, default=None)
    parser.add_argument("--robot", default="iiwa14")
    args = parser.parse_args()

    args.out_dir = args.out_dir.resolve()

    _seeds, _timeout, _mode = mode_args(args, quick_seeds=1, full_seeds=1)
    if args.robot != "iiwa14":
        raise ValueError("The v6 Exp.2 wrapper currently supports only --robot=iiwa14.")

    n_boxes = args.n_boxes if args.n_boxes is not None else (20 if args.quick else 400)
    repeats = args.repeats if args.repeats is not None else (5 if args.quick else 20)

    raw_out = args.out_dir / "raw" / RAW_OUTPUT
    final_out = args.out_dir / "link_envelope_pipeline.json"
    script_args: list[str | Path] = [
        "--out", raw_out,
        "--n-boxes", str(n_boxes),
        "--repeats", str(repeats),
        "--bin-scheme", "exp1",
    ]

    run_python(RAW_SCRIPT, script_args, dry_run=args.dry_run)
    if args.dry_run:
        print(f"[dry-run] would write {final_out}")
        return

    raw = load_json(raw_out)
    write_json(final_out, translate_payload(raw, n_boxes=int(n_boxes), repeats=int(repeats)))
    print(f"[write] {final_out}")


if __name__ == "__main__":
    main()

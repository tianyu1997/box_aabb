#!/usr/bin/env python3
"""Paper Exp. 1 — epiAABB source-pipeline comparison.

Environment: robot joint-interval boxes sampled from the IIWA14 planning space.
Output:      experiments/results_paper/epiaabb_pipeline.json
Paper slot:  Experiments-A, epiAABB pipeline comparison.

This v6 wrapper uses the authoritative width-proportional MC endpoint-profile
script and rewrites its output into the current paper JSON schema.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import ROOT, add_common_args, load_json, mode_args, run_python, write_json


RAW_SCRIPT = ROOT / "python" / "scripts" / "run_ep_width_profile_mc_density.py"
RAW_OUTPUT = "epiaabb_pipeline_v6_raw.json"


WIDTH_BIN_LABELS = {
    "W1_0.001_0.05": "0.001-0.05",
    "W2_0.05_0.1": "0.05-0.10",
    "W3_0.1_0.2": "0.10-0.20",
    "W4_0.2_0.5": "0.20-0.50",
}

ACTIVE_SOURCES = {"IFK", "CritSample", "Analytical", "MC"}


def normalize_width_bin(name: str, width_lo: float, width_hi: float) -> str:
    if name in WIDTH_BIN_LABELS:
        return WIDTH_BIN_LABELS[name]
    if abs(width_lo - 0.001) < 1e-9 and abs(width_hi - 0.05) < 1e-9:
        return "0.001-0.05"
    return f"{width_lo:.2f}-{width_hi:.2f}"


def certified_source(name: str) -> bool:
    return name in {"IFK", "Analytical"}


def translate_payload(raw: dict, *, ref_samples: int) -> dict:
    meta = raw.get("meta", {})
    width_bins = []
    for row in meta.get("width_bins", []):
        width_bins.append(
            {
                "width_bin": normalize_width_bin(
                    str(row.get("name", "")),
                    float(row.get("lo", 0.0)),
                    float(row.get("hi", 0.0)),
                ),
                "width_lo": float(row.get("lo", 0.0)),
                "width_hi": float(row.get("hi", 0.0)),
                "n_boxes": int(meta.get("n_seeds_per_bin", 0)),
            }
        )

    rows = []
    for row in raw.get("rows", []):
        source = str(row.get("endpoint", ""))
        if source not in ACTIVE_SOURCES:
            continue
        max_negative_gap = -abs(float(row.get("max_negative_gap_abs", 0.0)))
        rows.append(
            {
                "width_bin": normalize_width_bin(
                    str(row.get("width_bin", "")),
                    float(row.get("width_lo", 0.0)),
                    float(row.get("width_hi", 0.0)),
                ),
                "source": source,
                "volume_mean": float(row.get("ep_volume_mean", 0.0)),
                "time_us_mean": float(row.get("ep_time_us_mean", 0.0)),
                "time_us_median": float(row.get("ep_time_us_median", row.get("ep_time_us_mean", 0.0))),
                "max_negative_gap": max_negative_gap,
                "max_negative_gap_reference": str(
                    meta.get("gap_reference", "per_axis_max_extent_of_CritSample_Analytical_MC")
                ),
                "certified": certified_source(source),
            }
        )

    return {
        "experiment": "epiaabb_pipeline",
        "robot": "iiwa14",
        "n_boxes_per_bin": int(meta.get("n_seeds_per_bin", 0)),
        "mc_sampling_mode": "width_proportional",
        "mc_samples": int(ref_samples),
        "mc_reference_samples": int(ref_samples),
        "mc_reference_width": 0.35,
        "mc_min_samples": int(meta.get("mc_min_samples", 0)),
        "mc_max_samples": int(meta.get("mc_max_samples", 0)),
        "critical_combo_cap": int(meta.get("critical_combo_cap", meta.get("crit_samples", 8192))),
        "analytical_max_phase": int(meta.get("analytical_max_phase", 3)),
        "mc_density_rho": float(meta.get("mc_density_rho", 0.0)),
        "max_negative_gap_reference": str(
            meta.get("gap_reference", "per_axis_max_extent_of_CritSample_Analytical_MC")
        ),
        "bypass_narrow_skip": bool(meta.get("bypass_narrow_skip", False)),
        "interval_protocol": str(meta.get("interval_protocol", "paired_centers_scaled_width")),
        "width_bins": width_bins,
        "rows": rows,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_common_args(parser)
    parser.add_argument("--n-boxes", type=int, default=None)
    parser.add_argument("--mc-samples", type=int, default=None,
                        help="mapped to the reference count at width 0.35 under the v6 width-proportional MC backend")
    parser.add_argument("--rho", type=float, default=None,
                        help="override width-proportional MC density in samples/rad of geometric-mean width")
    parser.add_argument("--ref-samples", type=int, default=None,
                        help="MC samples at geometric-mean width 0.35 rad")
    parser.add_argument("--min-samples", type=int, default=1000)
    parser.add_argument("--max-samples", type=int, default=10_000_000)
    parser.add_argument("--critical-combo-cap", type=int, default=8192)
    parser.add_argument("--analytical-max-phase", type=int, default=3)
    parser.add_argument("--robot", default="iiwa14")
    args = parser.parse_args()

    _seeds, _timeout, _mode = mode_args(args, quick_seeds=1, full_seeds=1)
    if args.robot != "iiwa14":
        raise ValueError("The v6 Exp.1 wrapper currently supports only --robot=iiwa14.")

    n_boxes = args.n_boxes if args.n_boxes is not None else (20 if args.quick else 400)
    ref_samples = args.ref_samples
    if ref_samples is None:
        ref_samples = args.mc_samples if args.mc_samples is not None else (50_000 if args.quick else 2_000_000)

    raw_out = args.out_dir / "raw" / RAW_OUTPUT
    final_out = args.out_dir / "epiaabb_pipeline.json"
    script_args: list[str | Path] = [
        "--out", raw_out,
        "--seeds", str(n_boxes),
        "--ref-samples", str(ref_samples),
        "--min-samples", str(args.min_samples),
        "--max-samples", str(args.max_samples),
        "--critical-combo-cap", str(args.critical_combo_cap),
        "--analytical-max-phase", str(args.analytical_max_phase),
        "--bypass-narrow-skip",
    ]
    if args.rho is not None:
        script_args += ["--rho", str(args.rho)]

    run_python(RAW_SCRIPT, script_args, dry_run=args.dry_run)
    if args.dry_run:
        print(f"[dry-run] would write {final_out}")
        return

    raw = load_json(raw_out)
    write_json(final_out, translate_payload(raw, ref_samples=int(ref_samples)))
    print(f"[write] {final_out}")


if __name__ == "__main__":
    main()

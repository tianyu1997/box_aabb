#!/usr/bin/env python3
"""Paper Exp. 1 — epiAABB source-pipeline comparison.

Environment: robot joint-interval boxes sampled from the IIWA14 planning space.
Output:      experiments/results_paper/epiaabb_pipeline.json
Paper slot:  Experiments-A, epiAABB pipeline comparison.

The MC baseline follows the v6 paper protocol: width-proportional density with
an auto-calibrated reference count at geometric-mean width 0.35 rad.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import add_common_args, bin_path, mode_args, run


def default_gcpc_path(robot: str) -> Path | None:
    candidates = [
        Path(__file__).resolve().parents[2] / "data" / f"{robot}.gcpc",
        Path(__file__).resolve().parents[2] / "data" / f"{robot}_5000.gcpc",
        Path(__file__).resolve().parents[2] / "data" / f"{robot}_500.gcpc",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_common_args(parser)
    parser.add_argument("--n-boxes", type=int, default=None)
    parser.add_argument("--mc-samples", type=int, default=None,
                        help="legacy fixed MC samples override; otherwise use width-proportional density")
    parser.add_argument("--rho", type=float, default=None,
                        help="override width-proportional MC density in samples/rad of geometric-mean width")
    parser.add_argument("--ref-samples", type=int, default=None,
                        help="MC samples at geometric-mean width 0.35 rad")
    parser.add_argument("--min-samples", type=int, default=1000)
    parser.add_argument("--max-samples", type=int, default=10_000_000)
    parser.add_argument("--gcpc", type=Path, default=None,
                        help="optional v7-local GCPC cache path; if omitted, the GCPC row is skipped")
    parser.add_argument("--robot", default="iiwa14")
    args = parser.parse_args()

    seeds, _timeout, mode = mode_args(args, quick_seeds=1, full_seeds=1)
    del seeds
    n_boxes = args.n_boxes if args.n_boxes is not None else (20 if args.quick else 400)
    ref_samples = args.ref_samples if args.ref_samples is not None else (50_000 if args.quick else 2_000_000)
    gcpc = args.gcpc if args.gcpc is not None else default_gcpc_path(args.robot)
    cmd: list[str | Path] = [bin_path(args, "exp_epiaabb_pipeline")]
    if mode:
        cmd.append(mode)
    cmd += [
        f"--out={args.out_dir / 'epiaabb_pipeline.json'}",
        f"--robot={args.robot}",
        f"--n-boxes={n_boxes}",
        f"--ref-samples={ref_samples}",
        f"--min-samples={args.min_samples}",
        f"--max-samples={args.max_samples}",
    ]
    if args.mc_samples is not None:
        cmd.append(f"--mc-samples={args.mc_samples}")
    if args.rho is not None:
        cmd.append(f"--rho={args.rho}")
    if gcpc is not None:
        cmd.append(f"--gcpc={gcpc}")
    run(cmd, dry_run=args.dry_run)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Paper Exp. 2 — link-envelope pipeline comparison.

Environment: reuse the Exp. 1 four-bin width-stratified IIWA14 box protocol,
with one shared random box set per width bin and CritSample fixed upstream.
Output:      experiments/results_paper/link_envelope_pipeline.json
Paper slot:  Experiments-B, LinkIAABB / grid / Hull16 envelope comparison.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import add_common_args, bin_path, mode_args, run


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_common_args(parser)
    parser.add_argument("--n-boxes", type=int, default=None)
    parser.add_argument("--repeats", type=int, default=None)
    parser.add_argument("--robot", default="iiwa14")
    args = parser.parse_args()

    _seeds, _timeout, mode = mode_args(args, quick_seeds=1, full_seeds=1)
    n_boxes = args.n_boxes if args.n_boxes is not None else (20 if args.quick else 400)
    repeats = args.repeats if args.repeats is not None else (5 if args.quick else 20)
    cmd: list[str | Path] = [bin_path(args, "exp_link_envelope_pipeline")]
    if mode:
        cmd.append(mode)
    cmd += [
        f"--out={args.out_dir / 'link_envelope_pipeline.json'}",
        f"--robot={args.robot}",
        f"--n-boxes={n_boxes}",
        f"--repeats={repeats}",
    ]
    run(cmd, dry_run=args.dry_run)


if __name__ == "__main__":
    main()

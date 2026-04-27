#!/usr/bin/env python3
"""Paper Exp. 3 — Marcucci combined-scene SBF workload.

Environment: Marcucci combined IIWA14 scene only.  The five JSON files in
`experiments/configs/marcucci/` are the canonical query pairs in the same
16-obstacle combined scene; the paper reports them as one workload, not as
separate sub-scenes.

Outputs:
    - experiments/results_paper/marcucci_combined.json

The v7 paper currently retains only the native SBF row for the Exp. 3 main
table. The historical SBF+GCS post row is excluded pending collision-safe
validation, and the OMPL baseline rows are therefore not produced by this
paper runner.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import CFG, add_common_args, bin_path, mode_args, run

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_common_args(parser)
    parser.add_argument("--threads", type=int, default=1)
    parser.add_argument("--env", default="link_iaabb_grid")
    parser.add_argument("--n-sub", type=int, default=4)
    parser.add_argument("--ffb-depth", type=int, default=55)
    parser.add_argument("--max-boxes", type=int, default=2500)
    parser.add_argument("--bridge-boxes", type=int, default=2000)
    parser.add_argument("--point-bridge-timeout-ms", type=float, default=20000.0)
    parser.add_argument("--no-point-bridge", action="store_true",
                        help="disable point-level RRT bridge fallback")
    parser.add_argument("--include-anytime", action="store_true",
                        help="deprecated no-op retained for CLI compatibility; Exp. 3 currently runs SBF only")
    args = parser.parse_args()

    seeds, timeout, mode = mode_args(args, quick_seeds=3, full_seeds=10,
                                     quick_timeout=30, full_timeout=120)
    sbf_cmd: list[str | Path] = [bin_path(args, "exp_marcucci_combined")]
    if mode:
        sbf_cmd.append(mode)
    sbf_cmd += [
        f"--scene-dir={CFG / 'marcucci'}",
        f"--out={args.out_dir / 'marcucci_combined.json'}",
        f"--seeds={seeds}", f"--timeout={timeout}", f"--threads={args.threads}",
        f"--env={args.env}", f"--n-sub={args.n_sub}", f"--max-boxes={args.max_boxes}",
        f"--ffb-depth={args.ffb_depth}",
        f"--bridge-boxes={args.bridge_boxes}",
        f"--point-bridge-timeout-ms={args.point_bridge_timeout_ms}",
    ]
    if args.no_point_bridge:
        sbf_cmd.append("--no-point-bridge")
    else:
        sbf_cmd.append("--point-bridge")
    run(sbf_cmd, dry_run=args.dry_run)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Paper Exp. 4 — ablation on the Marcucci combined workload.

Environment: Marcucci combined IIWA14 scene only.
Output:      experiments/results_paper/ablation/*.json

Rows vary one implementation choice at a time around the default combined-scene
configuration.  This is the retained v6-style ablation section; low-value toy
clutter/narrow scenes are intentionally excluded.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import CFG, add_common_args, bin_path, mode_args, run

CELLS = [
    ("default", {"env": "link_iaabb_grid", "n_sub": 4, "max_boxes": 2500, "bridge_boxes": 2000}),
    ("coarse_link_iaabb", {"env": "link_iaabb", "n_sub": 1, "max_boxes": 2500, "bridge_boxes": 2000}),
    ("subdiv_2", {"env": "link_iaabb_grid", "n_sub": 2, "max_boxes": 2500, "bridge_boxes": 2000}),
    ("subdiv_8", {"env": "link_iaabb_grid", "n_sub": 8, "max_boxes": 2500, "bridge_boxes": 2000}),
    ("box_budget_1000", {"env": "link_iaabb_grid", "n_sub": 4, "max_boxes": 1000, "bridge_boxes": 2000}),
    ("bridge_budget_500", {"env": "link_iaabb_grid", "n_sub": 4, "max_boxes": 2500, "bridge_boxes": 500}),
    ("no_point_bridge", {"env": "link_iaabb_grid", "n_sub": 4, "max_boxes": 2500, "bridge_boxes": 2000, "no_point_bridge": True}),
]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_common_args(parser)
    parser.add_argument("--threads", type=int, default=1)
    parser.add_argument("--ffb-depth", type=int, default=55)
    parser.add_argument("--resume", action="store_true",
                        help="skip ablation cells whose JSON outputs already exist")
    args = parser.parse_args()
    seeds, timeout, mode = mode_args(args, quick_seeds=2, full_seeds=3,
                                     quick_timeout=30, full_timeout=120)

    for cell, cfg in CELLS:
        out = args.out_dir / 'ablation' / (cell + '.json')
        if args.resume and out.exists():
            print(f"[skip] {out}")
            continue
        cmd: list[str | Path] = [bin_path(args, "exp_marcucci_combined")]
        if mode:
            cmd.append(mode)
        cmd += [
            f"--scene-dir={CFG / 'marcucci'}",
            f"--out={out}",
            f"--seeds={seeds}", f"--timeout={timeout}", f"--threads={args.threads}",
            f"--env={cfg['env']}", f"--n-sub={cfg['n_sub']}",
            f"--ffb-depth={args.ffb_depth}",
            f"--max-boxes={cfg['max_boxes']}",
            f"--bridge-boxes={cfg['bridge_boxes']}",
            "--point-bridge-timeout-ms=20000",
        ]
        cmd.append("--no-point-bridge" if cfg.get("no_point_bridge") else "--point-bridge")
        run(cmd, dry_run=args.dry_run)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Run the v7 Drake/OMPL Marcucci baselines from the v6 paper folder.

This wrapper keeps the paper orchestration under ``cpp/v6/experiments/paper``
while preserving the v7 baseline implementations. Outputs are written to the
v6 paper results directory by default.
"""
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import OUT_DEFAULT, ROOT, add_common_args, mode_args


WORKSPACE = ROOT.parents[1]
V7_SCRIPTS = WORKSPACE / "cpp" / "v7" / "experiments" / "scripts"


BASELINE_SCRIPTS = {
    "iris_np": "marcucci_iris_np_gcs.py",
    "iris_zo": "marcucci_iris_zo_gcs.py",
    "ompl": "marcucci_ompl_baselines.py",
}


def parse_methods(raw: str) -> list[str]:
    methods = [part.strip() for part in raw.split(",") if part.strip()]
    unknown = [method for method in methods if method not in BASELINE_SCRIPTS]
    if unknown:
        raise ValueError(f"unknown baseline methods: {unknown}")
    return methods


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    add_common_args(parser)
    parser.add_argument(
        "--methods",
        default="iris_np,iris_zo,ompl",
        help="comma-separated subset of iris_np, iris_zo, ompl",
    )
    parser.add_argument("--logical-threads", type=int, default=16)
    parser.add_argument("--bitstar-budget-s", type=float, default=1.0)
    parser.add_argument(
        "--iris-budget-s",
        type=float,
        default=None,
        help="region-generation budget for Drake IRIS baselines; defaults to the selected timeout",
    )
    parser.add_argument("--iris-np-iteration-limit", type=int, default=None)
    parser.add_argument("--iris-zo-max-iterations", type=int, default=None)
    args = parser.parse_args()

    if not V7_SCRIPTS.is_dir():
        raise FileNotFoundError(f"v7 baseline scripts not found: {V7_SCRIPTS}")

    seeds, timeout, mode = mode_args(args, quick_seeds=1, full_seeds=5,
                                     quick_timeout=30, full_timeout=120)
    mode_args_for_child: list[str] = [mode] if mode else []
    out_dir = args.out_dir if args.out_dir is not None else OUT_DEFAULT
    out_dir.mkdir(parents=True, exist_ok=True)

    for method in parse_methods(args.methods):
        script = V7_SCRIPTS / BASELINE_SCRIPTS[method]
        cmd = [
            sys.executable,
            str(script),
            *mode_args_for_child,
            "--seeds", str(seeds),
            "--timeout", str(timeout),
        ]
        if method == "ompl":
            cmd += [
                "--out-dir", str(out_dir),
                "--logical-threads", str(args.logical_threads),
                "--bitstar-budget-s", str(args.bitstar_budget_s),
            ]
        else:
            name = "marcucci_iris_np_gcs.json" if method == "iris_np" else "marcucci_iris_zo_gcs.json"
            cmd += [
                "--out", str(out_dir / name),
                "--logical-threads", str(args.logical_threads),
                "--budget-s", str(args.iris_budget_s if args.iris_budget_s is not None else timeout),
            ]
            if method == "iris_np" and args.iris_np_iteration_limit is not None:
                cmd += ["--iteration-limit", str(args.iris_np_iteration_limit)]
            if method == "iris_zo" and args.iris_zo_max_iterations is not None:
                cmd += ["--max-iterations", str(args.iris_zo_max_iterations)]
        if args.dry_run:
            print("$", " ".join(cmd))
            continue
        subprocess.run(cmd, check=True, cwd=WORKSPACE)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

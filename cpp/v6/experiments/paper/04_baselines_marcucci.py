#!/usr/bin/env python3
"""Paper Exp. 4 baselines for the Marcucci combined-scene workload.

This v6-native wrapper runs the Drake IRIS/GCS and OMPL baselines from
``cpp/v6/experiments/paper/baselines`` and writes paper JSON outputs under
``cpp/v6/experiments/results_paper``. It is the replacement for the old v7
baseline wrapper.
"""
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import OUT_DEFAULT, PAPER_CPUSET, ROOT, add_common_args, bin_path, mode_args


BASELINE_SCRIPTS_DIR = ROOT / "experiments" / "paper" / "baselines"
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
    parser.add_argument("--methods", default="iris_np,iris_zo,ompl")
    parser.add_argument("--logical-threads", type=int, default=16)
    parser.add_argument("--ompl-baseline-bin", type=Path, default=None)
    parser.add_argument("--bitstar-budget-s", type=float, default=2.0)
    parser.add_argument("--ompl-methods", default="prm,bitstar_budget")
    parser.add_argument(
        "--simplify-prm",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="include OMPL simplifySolution in the PRM query timing record",
    )
    parser.add_argument("--iris-budget-s", type=float, default=None)
    parser.add_argument("--iris-np-budget-s", type=float, default=None)
    parser.add_argument("--iris-zo-budget-s", type=float, default=None)
    parser.add_argument("--iris-np-timeout", type=int, default=None)
    parser.add_argument("--iris-zo-timeout", type=int, default=None)
    parser.add_argument("--iris-np-iteration-limit", type=int, default=None)
    parser.add_argument("--iris-zo-max-iterations", type=int, default=None)
    args = parser.parse_args()

    seeds, timeout, _ = mode_args(args, quick_seeds=1, full_seeds=5, quick_timeout=30, full_timeout=120)
    mode_args_for_child: list[str] = ["--quick"] if args.quick else ["--full"]
    out_dir = args.out_dir if args.out_dir is not None else OUT_DEFAULT
    out_dir.mkdir(parents=True, exist_ok=True)
    cpu_affinity = f"{PAPER_CPUSET[0]}-{PAPER_CPUSET[-1]}"

    for method in parse_methods(args.methods):
        script = BASELINE_SCRIPTS_DIR / BASELINE_SCRIPTS[method]
        cmd = [
            sys.executable,
            str(script),
            *mode_args_for_child,
            "--seeds",
            str(seeds),
            "--timeout",
            str(timeout),
        ]
        if method == "ompl":
            baseline_bin = args.ompl_baseline_bin
            if baseline_bin is None:
                baseline_bin = ROOT / "build" / "experiments" / "baseline_ompl" if args.dry_run else bin_path(args, "baseline_ompl")
            cmd += [
                "--out-dir",
                str(out_dir),
                "--logical-threads",
                str(args.logical_threads),
                "--bitstar-budget-s",
                str(args.bitstar_budget_s),
                "--methods",
                str(args.ompl_methods),
                "--cpu-affinity",
                cpu_affinity,
                "--baseline-bin",
                str(baseline_bin),
            ]
            cmd.append("--simplify-prm" if args.simplify_prm else "--no-simplify-prm")
        else:
            output_name = "marcucci_iris_np_gcs.json" if method == "iris_np" else "marcucci_iris_zo_gcs.json"
            method_timeout = timeout
            if method == "iris_np" and args.iris_np_timeout is not None:
                method_timeout = args.iris_np_timeout
            elif method == "iris_np" and not args.quick:
                method_timeout = 800
            if method == "iris_zo" and args.iris_zo_timeout is not None:
                method_timeout = args.iris_zo_timeout
            method_budget = args.iris_budget_s
            if method_budget is None:
                if method == "iris_np":
                    method_budget = args.iris_np_budget_s if args.iris_np_budget_s is not None else method_timeout
                else:
                    method_budget = args.iris_zo_budget_s if args.iris_zo_budget_s is not None else method_timeout
            timeout_index = cmd.index("--timeout") + 1
            cmd[timeout_index] = str(method_timeout)
            cmd += [
                "--out",
                str(out_dir / output_name),
                "--logical-threads",
                str(args.logical_threads),
                "--budget-s",
                str(method_budget),
                "--cpu-affinity",
                cpu_affinity,
            ]
            if method == "iris_np" and args.iris_np_iteration_limit is not None:
                cmd += ["--iteration-limit", str(args.iris_np_iteration_limit)]
            if method == "iris_zo" and args.iris_zo_max_iterations is not None:
                cmd += ["--max-iterations", str(args.iris_zo_max_iterations)]
        if args.dry_run:
            print("$", " ".join(str(part) for part in cmd))
            continue
        subprocess.run([str(part) for part in cmd], check=True, cwd=ROOT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
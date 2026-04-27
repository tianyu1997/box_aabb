#!/usr/bin/env python3
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from common import DEFAULT_LOGICAL_THREADS, RESULTS_PAPER

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--quick", action="store_true")
    mode.add_argument("--full", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--reuse-roadmaps", action="store_true")
    parser.add_argument("--out-dir", type=Path, default=RESULTS_PAPER)
    parser.add_argument("--logical-threads", type=int, default=DEFAULT_LOGICAL_THREADS)
    return parser.parse_args()


def run_script(script_name: str, *, args: argparse.Namespace, extra: list[str] | None = None) -> None:
    cmd = [sys.executable, str(HERE / script_name)]
    if args.quick:
        cmd.append("--quick")
    elif args.full:
        cmd.append("--full")
    if args.dry_run:
        cmd.append("--dry-run")
    if args.reuse_roadmaps and script_name == "marcucci_prm_gcs.py":
        cmd.append("--reuse-roadmaps")
    cmd.extend(["--logical-threads", str(args.logical_threads)])
    if extra:
        cmd.extend(extra)
    print("$", " ".join(cmd))
    if not args.dry_run:
        subprocess.run(cmd, check=True, cwd=HERE.parents[2])


def main() -> int:
    args = parse_args()
    run_script(
        "marcucci_iris_np_gcs.py",
        args=args,
        extra=["--out", str(args.out_dir / "marcucci_iris_np_gcs.json")],
    )
    run_script(
        "marcucci_prm_gcs.py",
        args=args,
        extra=["--out", str(args.out_dir / "marcucci_prm_gcs.json")],
    )
    run_script(
        "marcucci_iris_zo_gcs.py",
        args=args,
        extra=["--out", str(args.out_dir / "marcucci_iris_zo_gcs.json")],
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
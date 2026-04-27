#!/usr/bin/env python3
"""Run the retained v6 paper experiment wrappers in section order.

This wrapper is intentionally separate from the one-script-per-environment
policy: the individual numbered scripts remain the authoritative mapping used
by the paper and by reviewers.
"""
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SCRIPTS = [
    "01_epiaabb_pipeline.py",
    "02_link_envelope_pipeline.py",
    "03_marcucci_envelope_build.py",
    "04_e2e_baselines_combined.py",
]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--quick", action="store_true")
    parser.add_argument("--full", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--include-anytime", action="store_true")
    parser.add_argument("--include-envelope-build", action="store_true")
    args, rest = parser.parse_known_args()

    mode = "--quick" if args.quick else "--full" if args.full else ""
    scripts = list(SCRIPTS)
    if args.include_envelope_build:
        print("[note] --include-envelope-build is now a no-op; envelope build is included by default")
    for script in scripts:
        cmd = [sys.executable, str(HERE / script)]
        if mode:
            cmd.append(mode)
        if args.dry_run:
            cmd.append("--dry-run")
        if args.include_anytime and script == "04_e2e_baselines_combined.py":
            cmd.append("--include-anytime")
        cmd.extend(rest)
        print("\n==", script, "==")
        subprocess.run(cmd, check=True)


if __name__ == "__main__":
    main()

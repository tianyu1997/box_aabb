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
    "05_random_robot_scenes.py",
    "06_sbf_obstacle_rebuild.py",
]
PARAMETER_SCAN_SCRIPT = "06_sbf_parameter_scan.py"
UPDATE_PAPER_SCRIPT = "07_update_paper_results.py"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--quick", action="store_true")
    parser.add_argument("--full", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--include-anytime", action="store_true")
    parser.add_argument("--include-envelope-build", action="store_true")
    parser.add_argument("--include-parameter-scan", action="store_true")
    parser.add_argument("--include-v7-baselines", action="store_true",
                        help="deprecated no-op; paper experiments now stay in cpp/v6")
    parser.add_argument("--update-paper-results", action="store_true")
    args, rest = parser.parse_known_args()

    mode = "--quick" if args.quick else "--full" if args.full else ""
    scripts = list(SCRIPTS)
    if args.include_v7_baselines:
        print("[note] --include-v7-baselines is ignored; v6-native baselines run by default")
    if args.include_parameter_scan:
        scripts.append(PARAMETER_SCAN_SCRIPT)
    if args.update_paper_results:
        scripts.append(UPDATE_PAPER_SCRIPT)
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

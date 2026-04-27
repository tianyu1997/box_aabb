#!/usr/bin/env python3
"""Phase C -- multi-robot x multi-scene grid runner.

Orchestrates the full {iiwa14, panda, ur10} x {marcucci_combined,
bins_only, dense_clutter, tabletop_pickplace} grid by invoking each
phaseB*_*.py script with the appropriate (--robot, --scene) and writing
a per-cell sentinel under experiments/results_new/grid/<robot>/<scene>/
to make resumption trivial.

Reproduce:

    cd cpp/v6
    python3 experiments/scripts/phaseC_grid_runner.py \\
        --robots iiwa14,panda,ur10 \\
        --scenes marcucci_combined,bins_only,dense_clutter,tabletop_pickplace \\
        --phases B1,B2,B3,B4,B5 \\
        --skip-existing
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

REPO_V6 = Path(__file__).resolve().parents[2]
SCRIPTS = REPO_V6 / "experiments" / "scripts"
RESULTS = REPO_V6 / "experiments" / "results_new" / "grid"
QUERIES = REPO_V6 / "data" / "queries"

PHASE_TO_SCRIPT = {
    "B1": "phaseB1_irisnp_pareto.py",
    "B2": "phaseB2_prm_sweep.py",
    "B3": "phaseB3_werner2024.py",
    "B4": "phaseB4_iris_zo.py",
    "B5": "phaseB5_bitstar_aitstar.py",
}


def parse_csv(s: str) -> list[str]:
    return [x.strip() for x in s.split(",") if x.strip()]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robots", default="iiwa14,panda,ur10", type=parse_csv)
    ap.add_argument("--scenes",
                    default="marcucci_combined,bins_only,"
                            "dense_clutter,tabletop_pickplace",
                    type=parse_csv)
    ap.add_argument("--phases", default="B1,B2,B3,B4,B5", type=parse_csv)
    ap.add_argument("--skip-existing", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    env = os.environ.copy()
    env["PYTHONPATH"] = ":".join([
        str(REPO_V6 / "build" / "python"),
        str(REPO_V6 / "python"),
        env.get("PYTHONPATH", ""),
    ])

    failures: list[tuple[str, str, str, str]] = []
    for robot in args.robots:
        for scene in args.scenes:
            queries_path = QUERIES / f"{robot}_{scene}_50.json"
            if not queries_path.exists():
                msg = (f"missing queries: {queries_path} -- run"
                       f" phaseB6_sample_queries.py first")
                print(f"[skip] {robot}/{scene}: {msg}")
                failures.append((robot, scene, "B6", msg))
                continue
            for phase in args.phases:
                script = SCRIPTS / PHASE_TO_SCRIPT[phase]
                cell_dir = RESULTS / robot / scene
                cell_dir.mkdir(parents=True, exist_ok=True)
                out_json = cell_dir / f"{phase}.json"
                if args.skip_existing and out_json.exists():
                    print(f"[skip] {robot}/{scene}/{phase}: exists")
                    continue
                cmd = [sys.executable, str(script),
                       "--robot", robot, "--scene", scene,
                       "--queries", str(queries_path),
                       "--json", str(out_json)]
                print("[run] " + " ".join(cmd))
                if args.dry_run:
                    continue
                rc = subprocess.call(cmd, env=env)
                if rc != 0:
                    failures.append((robot, scene, phase, f"exit={rc}"))

    summary = RESULTS / "_runner_summary.json"
    summary.parent.mkdir(parents=True, exist_ok=True)
    summary.write_text(json.dumps({"failures": failures}, indent=2))
    if failures:
        print(f"[done] {len(failures)} cell(s) failed; see {summary}")
        return 1
    print(f"[done] all cells OK; summary at {summary}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

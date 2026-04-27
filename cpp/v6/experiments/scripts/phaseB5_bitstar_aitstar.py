#!/usr/bin/env python3
"""Phase B5 -- BIT* / AIT* sampling baselines via OMPL.

Solves each query with both BIT* and AIT* under a fixed wall-clock
budget (default 10 s/query), reporting first-solution time, final-cost
path length, and SR.

Reproduce:

    cd cpp/v6
    PYTHONPATH=build/python:python python3 \\
        experiments/scripts/phaseB5_bitstar_aitstar.py \\
        --robot iiwa14 --scene marcucci_combined \\
        --planners bitstar,aitstar \\
        --seeds 0,1,2,3,4 \\
        --query-budget-s 10 \\
        --queries data/queries/iiwa14_marcucci_50.json \\
        --json experiments/results_new/B5_bitstar_aitstar.json

Implementation contract: scripts/run_baselines.run_ompl_planner(robot,
scene, planner, seed, queries_path, query_budget_s) returns dict.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

REPO_V6 = Path(__file__).resolve().parents[2]


def parse_csv(s: str) -> list[str]:
    return [x.strip() for x in s.split(",") if x.strip()]


def parse_ints(s: str) -> list[int]:
    return [int(x) for x in s.split(",") if x.strip()]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="iiwa14")
    ap.add_argument("--scene", default="marcucci_combined")
    ap.add_argument("--planners", default="bitstar,aitstar", type=parse_csv)
    ap.add_argument("--seeds", default="0,1,2,3,4", type=parse_ints)
    ap.add_argument("--query-budget-s", default=10.0, type=float)
    ap.add_argument("--queries", required=True, type=Path)
    ap.add_argument("--json", required=True, type=Path)
    args = ap.parse_args()

    sys.path.insert(0, str(REPO_V6 / "scripts"))
    try:
        import run_baselines  # type: ignore
        fn = getattr(run_baselines, "run_ompl_planner", None)
        if fn is None:
            raise NotImplementedError(
                "Add run_ompl_planner(robot, scene, planner, seed,"
                " queries_path, query_budget_s) to scripts/run_baselines.py."
            )
    except ImportError as exc:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps({"skipped": True, "reason": str(exc)}))
        return 0

    cells: list[dict] = []
    args.json.parent.mkdir(parents=True, exist_ok=True)
    for planner in args.planners:
        for seed in args.seeds:
            t0 = time.time()
            try:
                r = fn(robot=args.robot, scene=args.scene, planner=planner,
                       seed=seed, queries_path=str(args.queries),
                       query_budget_s=args.query_budget_s)
                r.update(robot=args.robot, scene=args.scene,
                         planner=planner, seed=seed,
                         wall_s=time.time() - t0)
            except Exception as exc:
                r = {"robot": args.robot, "scene": args.scene,
                     "planner": planner, "seed": seed,
                     "error": repr(exc), "wall_s": time.time() - t0}
            cells.append(r)
            print(f"  planner={planner} seed={seed} -> {r}")

    args.json.write_text(json.dumps({
        "robot": args.robot, "scene": args.scene,
        "planners": args.planners, "seeds": args.seeds,
        "query_budget_s": args.query_budget_s, "cells": cells,
    }, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())

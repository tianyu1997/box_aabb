#!/usr/bin/env python3
"""Phase B4 -- IRIS-ZO baseline (zero-order IRIS variant).

Same scenes/queries as B1-B3. Uses pydrake.planning.IrisZo when
available (Drake \\geq 1.32 master). Records build time and per-query
solve metrics through the same Drake GCS pipeline as B1.

Reproduce:

    cd cpp/v6
    PYTHONPATH=build/python:python python3 \\
        experiments/scripts/phaseB4_iris_zo.py \\
        --robot iiwa14 --scene marcucci_combined \\
        --seeds 0,1,2,3,4 \\
        --queries data/queries/iiwa14_marcucci_50.json \\
        --json experiments/results_new/B4_iris_zo.json
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

REPO_V6 = Path(__file__).resolve().parents[2]


def parse_ints(s: str) -> list[int]:
    return [int(x) for x in s.split(",") if x.strip()]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="iiwa14")
    ap.add_argument("--scene", default="marcucci_combined")
    ap.add_argument("--seeds", default="0,1,2,3,4", type=parse_ints)
    ap.add_argument("--queries", required=True, type=Path)
    ap.add_argument("--json", required=True, type=Path)
    args = ap.parse_args()

    sys.path.insert(0, str(REPO_V6 / "scripts"))
    try:
        import run_baselines  # type: ignore
        fn = getattr(run_baselines, "run_iris_zo", None)
        if fn is None:
            raise NotImplementedError(
                "Add run_iris_zo(robot, scene, seed, queries_path) to"
                " scripts/run_baselines.py (requires pydrake \\geq 1.32)."
            )
    except ImportError as exc:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps({"skipped": True, "reason": str(exc)}))
        return 0

    cells: list[dict] = []
    args.json.parent.mkdir(parents=True, exist_ok=True)
    for seed in args.seeds:
        t0 = time.time()
        try:
            r = fn(robot=args.robot, scene=args.scene, seed=seed,
                   queries_path=str(args.queries))
            r.update(robot=args.robot, scene=args.scene, seed=seed,
                     wall_s=time.time() - t0)
        except Exception as exc:
            r = {"robot": args.robot, "scene": args.scene, "seed": seed,
                 "error": repr(exc), "wall_s": time.time() - t0}
        cells.append(r)
        print(f"  seed={seed} -> {r}")

    args.json.write_text(json.dumps({
        "robot": args.robot, "scene": args.scene, "seeds": args.seeds,
        "cells": cells,
    }, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())

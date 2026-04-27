#!/usr/bin/env python3
"""Phase B3 -- Werner et al. 2024 "Fast path planning through large
collections of safe boxes" baseline.

Run the Werner et al. pipeline (boxes from Drake IRIS-NP, then their
shortest-path-through-boxes solver) on the same scenes/queries used in
B1/B2; record build time, query path-length, and query solver time.

Reproduce:

    cd cpp/v6
    PYTHONPATH=build/python:python python3 \\
        experiments/scripts/phaseB3_werner2024.py \\
        --robot iiwa14 --scene marcucci_combined \\
        --seeds 0,1,2,3,4 \\
        --queries data/queries/iiwa14_marcucci_50.json \\
        --json experiments/results_new/B3_werner2024.json

Implementation contract: scripts/run_baselines.run_werner2024(robot,
scene, seed, queries_path) returns dict with keys
{build_s, n_boxes, query_path_rad_mean, query_time_s_mean, sr}. If the
official upstream repo (https://github.com/cvxgrp/fastpathplanning)
is not vendored, raise NotImplementedError so the user sees an
actionable message.
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
        fn = getattr(run_baselines, "run_werner2024", None)
        if fn is None:
            raise NotImplementedError(
                "Vendor https://github.com/cvxgrp/fastpathplanning under"
                " cpp/v6/third_party/ and expose run_werner2024(...) in"
                " scripts/run_baselines.py."
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

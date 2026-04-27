#!/usr/bin/env python3
"""Phase B2 -- PRM node-count sweep.

Build a PRM with N \\in {1k, 3k, 10k, 30k} samples on each scene, then
solve the same query set as B1 and record per-query path length and
solver time.

Reproduce:

    cd cpp/v6
    PYTHONPATH=build/python:python python3 \\
        experiments/scripts/phaseB2_prm_sweep.py \\
        --robot iiwa14 --scene marcucci_combined \\
        --nodes 1000,3000,10000,30000 \\
        --seeds 0,1,2,3,4 \\
        --queries data/queries/iiwa14_marcucci_50.json \\
        --json experiments/results_new/B2_prm_sweep.json

Implementation contract: delegates to scripts/run_baselines.run_prm_with_n
which must return a dict per (n, seed) with keys
{build_s, query_path_rad_mean, query_path_rad_std, query_time_s_mean, sr}.
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
    ap.add_argument("--nodes", default="1000,3000,10000,30000", type=parse_ints)
    ap.add_argument("--seeds", default="0,1,2,3,4", type=parse_ints)
    ap.add_argument("--queries", required=True, type=Path)
    ap.add_argument("--json", required=True, type=Path)
    args = ap.parse_args()

    sys.path.insert(0, str(REPO_V6 / "scripts"))
    try:
        import run_baselines  # type: ignore
        fn = getattr(run_baselines, "run_prm_with_n", None)
        if fn is None:
            raise NotImplementedError(
                "Add run_prm_with_n(robot, scene, n, seed, queries_path)"
                " to scripts/run_baselines.py."
            )
    except ImportError as exc:
        print(f"[skip] run_baselines unavailable: {exc}")
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps({"skipped": True, "reason": str(exc)}))
        return 0

    cells: list[dict] = []
    args.json.parent.mkdir(parents=True, exist_ok=True)
    partial = args.json.with_suffix(args.json.suffix + ".partial")
    for n in args.nodes:
        for seed in args.seeds:
            t0 = time.time()
            try:
                r = fn(robot=args.robot, scene=args.scene, n=n, seed=seed,
                       queries_path=str(args.queries))
                r.update(robot=args.robot, scene=args.scene, n=n, seed=seed,
                         wall_s=time.time() - t0)
            except Exception as exc:  # pragma: no cover
                r = {"robot": args.robot, "scene": args.scene, "n": n,
                     "seed": seed, "error": repr(exc),
                     "wall_s": time.time() - t0}
            cells.append(r)
            partial.write_text(json.dumps(cells, indent=2))
            print(f"  n={n:>6d} seed={seed} -> {r}")

    args.json.write_text(json.dumps({
        "robot": args.robot, "scene": args.scene,
        "nodes": args.nodes, "seeds": args.seeds, "cells": cells,
    }, indent=2))
    if partial.exists():
        partial.unlink()
    return 0


if __name__ == "__main__":
    sys.exit(main())

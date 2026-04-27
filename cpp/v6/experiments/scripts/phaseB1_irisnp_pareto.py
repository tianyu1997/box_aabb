#!/usr/bin/env python3
"""Phase B1 -- IRIS-NP precompute-budget Pareto sweep.

For each budget in {1.5, 10, 30, 60, 130} s, repeatedly call
pydrake.planning.IrisInConfigurationSpace until cumulative wall-clock
exceeds the budget; record (n_regions, total_time_s); then run the
benchmark queries through Drake's GCS solver and record per-query path
length and solver time.

Reproduce:

    cd cpp/v6
    PYTHONPATH=build/python:python python3 \\
        experiments/scripts/phaseB1_irisnp_pareto.py \\
        --robot iiwa14 --scene marcucci_combined \\
        --budgets 1.5,10,30,60,130 \\
        --seeds 0,1,2,3,4 \\
        --queries data/queries/iiwa14_marcucci_50.json \\
        --json experiments/results_new/B1_irisnp_pareto.json

Notes:
  * Drake \\geq 1.31 required; honour the same version as the existing
    GCS pipeline (`scripts/gcs_pipeline.py`).
  * The script is idempotent per (budget, seed): partial results are
    written incrementally to <out>.partial and merged at the end.
  * If pydrake is unavailable the script still exits 0 but writes a
    JSON with `"skipped": true` so downstream aggregators degrade
    gracefully.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

REPO_V6 = Path(__file__).resolve().parents[2]


def parse_floats(s: str) -> list[float]:
    return [float(x) for x in s.split(",") if x.strip()]


def parse_ints(s: str) -> list[int]:
    return [int(x) for x in s.split(",") if x.strip()]


def try_import_drake():
    try:
        import pydrake  # type: ignore  # noqa: F401
        from pydrake.geometry.optimization import IrisNp  # type: ignore  # noqa
        return True
    except Exception as exc:  # pragma: no cover
        print(f"[skip] pydrake unavailable: {exc!r}")
        return False


def run_one_budget(robot: str, scene: str, budget_s: float, seed: int,
                   queries_path: Path) -> dict:
    """Run IRIS-NP until cumulative wall-clock > budget_s, then GCS-solve.

    Implementation note: this routine is intentionally a thin shim that
    delegates to the existing run_baselines.py module so we keep one
    Drake initialisation path. If that module is not yet wired for
    custom IRIS-NP budgets, the script raises NotImplementedError so the
    user sees an actionable error instead of silently producing fake
    numbers.
    """
    sys.path.insert(0, str(REPO_V6 / "scripts"))
    try:
        import run_baselines  # type: ignore
    except ImportError as exc:
        raise NotImplementedError(
            f"scripts/run_baselines.py is required (import error: {exc})"
        )

    fn = getattr(run_baselines, "run_irisnp_with_budget", None)
    if fn is None:
        raise NotImplementedError(
            "Add `run_irisnp_with_budget(robot, scene, budget_s, seed,"
            " queries_path) -> dict` to scripts/run_baselines.py."
        )
    return fn(robot=robot, scene=scene, budget_s=budget_s,
              seed=seed, queries_path=str(queries_path))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="iiwa14")
    ap.add_argument("--scene", default="marcucci_combined")
    ap.add_argument("--budgets", default="1.5,10,30,60,130", type=parse_floats)
    ap.add_argument("--seeds", default="0,1,2,3,4", type=parse_ints)
    ap.add_argument("--queries", required=True, type=Path)
    ap.add_argument("--json", required=True, type=Path)
    args = ap.parse_args()

    if not try_import_drake():
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps({"skipped": True, "reason": "pydrake"}))
        return 0

    cells: list[dict] = []
    args.json.parent.mkdir(parents=True, exist_ok=True)
    partial = args.json.with_suffix(args.json.suffix + ".partial")
    for budget in args.budgets:
        for seed in args.seeds:
            t0 = time.time()
            try:
                result = run_one_budget(args.robot, args.scene, budget, seed,
                                        args.queries)
                result.update(robot=args.robot, scene=args.scene,
                              budget_s=budget, seed=seed,
                              wall_s=time.time() - t0)
            except NotImplementedError as exc:
                print(f"[fatal] {exc}")
                return 2
            except Exception as exc:  # pragma: no cover
                result = {
                    "robot": args.robot, "scene": args.scene,
                    "budget_s": budget, "seed": seed,
                    "error": repr(exc), "wall_s": time.time() - t0,
                }
            cells.append(result)
            partial.write_text(json.dumps(cells, indent=2))
            print(f"  budget={budget:>6.1f}s seed={seed} -> {result}")

    args.json.write_text(json.dumps({
        "robot": args.robot, "scene": args.scene,
        "budgets": args.budgets, "seeds": args.seeds,
        "cells": cells,
    }, indent=2))
    if partial.exists():
        partial.unlink()
    return 0


if __name__ == "__main__":
    sys.exit(main())

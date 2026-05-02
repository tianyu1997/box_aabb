#!/usr/bin/env python3
"""Phase D1 -- cold-first-build vs warm-N-th-build wall-clock curve.

Measures cumulative wall-clock for the SBF builder over N=30 sequential
queries on a fixed scene, with the certified-box cache:
  (a) cold-purged before each query (cold curve), and
  (b) shared across queries (warm curve).

The plot output (warm vs cold, log-y) is the headline lifelong evidence
for the paper.

Reproduce:

    cd cpp/v6
    PYTHONPATH=build/python:python python3 \\
        experiments/scripts/phaseD1_warm_cache_curve.py \\
        --robot iiwa14 --scene marcucci_combined \\
        --n 30 --seed 0 \\
        --queries data/queries/iiwa14_marcucci_50.json \\
        --json experiments/results_new/D1_warm_cache_curve.json
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

REPO_V6 = Path(__file__).resolve().parents[2]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="iiwa14")
    ap.add_argument("--scene", default="marcucci_combined")
    ap.add_argument("--n", type=int, default=30)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--queries", required=True, type=Path)
    ap.add_argument("--json", required=True, type=Path)
    args = ap.parse_args()

    sys.path.insert(0, str(REPO_V6 / "build" / "python"))
    sys.path.insert(0, str(REPO_V6 / "scripts"))
    try:
        import sbf6  # type: ignore
        import gcs_pipeline  # type: ignore
    except ImportError as exc:
        print(f"[fatal] sbf6 / gcs_pipeline unavailable: {exc}")
        return 2

    queries = json.loads(args.queries.read_text())["pairs"][:args.n]

    robot = sbf6.Robot.from_json(str(REPO_V6 / "data" / f"{args.robot}.json"))
    scene = sbf6.Scene.from_json(
        str(REPO_V6 / "data" / "scenes" / f"{args.scene}.json"))

    def run_one(q_pair, builder_state) -> dict:
        t0 = time.time()
        plan = gcs_pipeline.plan(robot, scene, q_pair["q_start"],
                                 q_pair["q_goal"], state=builder_state)
        return {"wall_s": time.time() - t0,
                "n_boxes": getattr(plan, "n_boxes", None),
                "path_rad": getattr(plan, "path_rad", None),
                "cache_hits": getattr(plan, "cache_hits", None)}

    cold = []
    for i, q in enumerate(queries):
        st = gcs_pipeline.new_state(robot, scene)
        r = run_one(q, st); r["i"] = i
        cold.append(r)
        print(f"  cold[{i}] {r}")

    warm_state = gcs_pipeline.new_state(robot, scene)
    warm = []
    for i, q in enumerate(queries):
        r = run_one(q, warm_state); r["i"] = i
        warm.append(r)
        print(f"  warm[{i}] {r}")

    args.json.parent.mkdir(parents=True, exist_ok=True)
    args.json.write_text(json.dumps({
        "robot": args.robot, "scene": args.scene, "n": args.n,
        "seed": args.seed, "cold": cold, "warm": warm,
    }, indent=2))
    print(f"[ok] wrote {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Phase D3 -- cross-scene cache reuse.

Builds the SBF cache on scene A, then plans on scene B WITHOUT
rebuilding. Reports cache_hit_rate, planning success, and any
soundness check failures (planning must remain certified-safe even
when reusing across scenes).

Reproduce:

    cd cpp/v6
    PYTHONPATH=build/python:python python3 \\
        experiments/scripts/phaseD3_cross_scene_reuse.py \\
        --robot iiwa14 \\
        --pairs marcucci_combined:bins_only,bins_only:dense_clutter,\\
marcucci_combined:tabletop_pickplace \\
        --seeds 0,1,2,3,4 \\
        --json experiments/results_new/D3_cross_scene_reuse.json

Pair syntax: A:B means build on A, query on B.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

REPO_V6 = Path(__file__).resolve().parents[2]


def parse_pairs(s):
    out = []
    for tok in s.split(","):
        tok = tok.strip()
        if not tok:
            continue
        a, b = tok.split(":")
        out.append((a.strip(), b.strip()))
    return out


def parse_ints(s):
    return [int(x) for x in s.split(",") if x.strip()]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="iiwa14")
    ap.add_argument("--pairs", required=True, type=parse_pairs)
    ap.add_argument("--seeds", default="0,1,2,3,4", type=parse_ints)
    ap.add_argument("--n-warmup", type=int, default=5)
    ap.add_argument("--n-test", type=int, default=20)
    ap.add_argument("--queries-dir", type=Path,
                    default=REPO_V6 / "data" / "queries")
    ap.add_argument("--json", required=True, type=Path)
    args = ap.parse_args()

    sys.path.insert(0, str(REPO_V6 / "build" / "python"))
    sys.path.insert(0, str(REPO_V6 / "scripts"))
    try:
        import sbf6  # type: ignore
        import gcs_pipeline  # type: ignore
    except ImportError as exc:
        print(f"[fatal] {exc}")
        return 2

    robot = sbf6.Robot.from_json(str(REPO_V6 / "data" / f"{args.robot}.json"))

    cells = []
    for (sa, sb) in args.pairs:
        scene_a = sbf6.Scene.from_json(
            str(REPO_V6 / "data" / "scenes" / f"{sa}.json"))
        scene_b = sbf6.Scene.from_json(
            str(REPO_V6 / "data" / "scenes" / f"{sb}.json"))
        qb_path = args.queries_dir / f"{args.robot}_{sb}_50.json"
        if not qb_path.exists():
            cells.append({"build": sa, "query": sb,
                          "error": f"missing {qb_path}"})
            continue
        qb_pairs = json.loads(qb_path.read_text())["pairs"][:args.n_test]
        for seed in args.seeds:
            t0 = time.time()
            try:
                state = gcs_pipeline.new_state(robot, scene_a)
                gcs_pipeline.warm_up(state, n=args.n_warmup, seed=seed)
                gcs_pipeline.swap_scene_keep_cache(state, scene_b)
                hits = 0; total = 0; n_safe = 0; n_planned = 0
                for q in qb_pairs:
                    total += 1
                    plan = gcs_pipeline.plan(robot, scene_b, q["q_start"],
                                             q["q_goal"], state=state)
                    hits += getattr(plan, "cache_hits", 0)
                    n_planned += int(getattr(plan, "feasible", False))
                    n_safe += int(getattr(plan, "certified_safe", False))
                r = {"build": sa, "query": sb, "seed": seed,
                     "n_test": total, "cache_hit_total": hits,
                     "cache_hit_rate": hits / max(total, 1),
                     "sr_planned": n_planned / max(total, 1),
                     "sr_certified_safe": n_safe / max(total, 1),
                     "wall_s": time.time() - t0}
            except Exception as exc:
                r = {"build": sa, "query": sb, "seed": seed,
                     "error": repr(exc), "wall_s": time.time() - t0}
            cells.append(r)
            print(f"  {sa}->{sb} seed={seed} -> {r}")

    args.json.parent.mkdir(parents=True, exist_ok=True)
    args.json.write_text(json.dumps({
        "robot": args.robot, "pairs": args.pairs,
        "seeds": args.seeds, "cells": cells,
    }, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())

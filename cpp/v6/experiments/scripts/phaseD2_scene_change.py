#!/usr/bin/env python3
"""Phase D2 -- scene-change stress test.

Three sub-experiments measuring repair-time and cache-hit-rate when the
scene mutates between queries:

  D2a -- nudge: move ONE obstacle by 0.1 m along a random axis.
  D2b -- add/remove: insert or delete K \\in {1,2,3} obstacles.
  D2c -- swap: replace the entire scene with a sibling scene of the
                same robot family (e.g. marcucci_combined -> bins_only).

For each variant: build cache on the original scene, run query,
mutate, run query again on mutated scene, record (cache_hits,
recert_time, full_rebuild_time_baseline).

Reproduce:

    cd cpp/v6
    PYTHONPATH=build/python:python python3 \\
        experiments/scripts/phaseD2_scene_change.py \\
        --robot iiwa14 \\
        --base-scene marcucci_combined \\
        --variants nudge,add_remove,swap \\
        --seeds 0,1,2,3,4 \\
        --json experiments/results_new/D2_scene_change.json
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

REPO_V6 = Path(__file__).resolve().parents[2]


def parse_csv(s):
    return [x.strip() for x in s.split(",") if x.strip()]


def parse_ints(s):
    return [int(x) for x in s.split(",") if x.strip()]


def mutate(scene, variant, rng):
    """Returns (mutated_scene, description)."""
    if variant == "nudge":
        return scene.nudge_random_obstacle(0.1, rng), "nudge_0.1m"
    if variant == "add_remove":
        k = int(rng.choice([1, 2, 3]))
        return scene.add_remove_random(k, rng), f"add_remove_k{k}"
    if variant == "swap":
        # caller injects sibling scene
        raise RuntimeError("swap is handled at the runner level")
    raise ValueError(variant)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="iiwa14")
    ap.add_argument("--base-scene", default="marcucci_combined")
    ap.add_argument("--swap-scene", default="bins_only",
                    help="Sibling scene used for D2c swap variant.")
    ap.add_argument("--variants", default="nudge,add_remove,swap",
                    type=parse_csv)
    ap.add_argument("--seeds", default="0,1,2,3,4", type=parse_ints)
    ap.add_argument("--json", required=True, type=Path)
    args = ap.parse_args()

    sys.path.insert(0, str(REPO_V6 / "build" / "python"))
    sys.path.insert(0, str(REPO_V6 / "scripts"))
    try:
        import sbf6  # type: ignore
        import gcs_pipeline  # type: ignore
        import numpy as np
    except ImportError as exc:
        print(f"[fatal] {exc}")
        return 2

    robot = sbf6.Robot.from_json(str(REPO_V6 / "data" / f"{args.robot}.json"))
    base = sbf6.Scene.from_json(
        str(REPO_V6 / "data" / "scenes" / f"{args.base_scene}.json"))
    swap = sbf6.Scene.from_json(
        str(REPO_V6 / "data" / "scenes" / f"{args.swap_scene}.json"))

    cells = []
    for variant in args.variants:
        for seed in args.seeds:
            rng = np.random.default_rng(seed)
            t0 = time.time()
            try:
                state = gcs_pipeline.new_state(robot, base)
                gcs_pipeline.warm_up(state, n=5, rng=rng)
                if variant == "swap":
                    new_scene, desc = swap, f"swap_to_{args.swap_scene}"
                else:
                    new_scene, desc = mutate(base, variant, rng)
                t_recert0 = time.time()
                hits = gcs_pipeline.recertify(state, new_scene)
                t_recert = time.time() - t_recert0
                t_full0 = time.time()
                gcs_pipeline.new_state(robot, new_scene)  # baseline rebuild
                t_full = time.time() - t_full0
                r = {"variant": variant, "seed": seed, "desc": desc,
                     "cache_hits": hits, "recert_s": t_recert,
                     "full_rebuild_s": t_full,
                     "speedup_x": (t_full / max(t_recert, 1e-9)),
                     "wall_s": time.time() - t0}
            except Exception as exc:
                r = {"variant": variant, "seed": seed,
                     "error": repr(exc), "wall_s": time.time() - t0}
            cells.append(r)
            print(f"  {variant} seed={seed} -> {r}")

    args.json.parent.mkdir(parents=True, exist_ok=True)
    args.json.write_text(json.dumps({
        "robot": args.robot, "base_scene": args.base_scene,
        "swap_scene": args.swap_scene, "variants": args.variants,
        "seeds": args.seeds, "cells": cells,
    }, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())

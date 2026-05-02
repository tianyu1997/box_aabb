#!/usr/bin/env python3
"""Phase B6 -- Sample 50 random (q_start, q_goal) pairs per scene.

Generates a deterministic JSON of 50 collision-free start/goal pairs per
(robot, scene) cell. Pairs are accepted only when:
  * both endpoints lie strictly inside the joint-limit box (with a small
    safety margin),
  * both endpoints are collision-free against the scene's AABB obstacles,
  * the straight-line midpoint q = 0.5(q_s + q_g) is collision-free
    (this rejects trivially infeasible pairs without imposing
    connectivity, which is intentional).

Reproduce:

    cd cpp/v6
    PYTHONPATH=build/python:python python3 \\
        experiments/scripts/phaseB6_sample_queries.py \\
        --robot iiwa14 --scene marcucci_combined \\
        --n 50 --seed 0 \\
        --out data/queries/iiwa14_marcucci_50.json
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_V6 = Path(__file__).resolve().parents[2]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="iiwa14")
    ap.add_argument("--scene", default="marcucci_combined")
    ap.add_argument("--n", type=int, default=50)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--margin", type=float, default=0.05,
                    help="Joint-limit safety margin in radians.")
    ap.add_argument("--max-attempts-per-pair", type=int, default=10000)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()

    sys.path.insert(0, str(REPO_V6 / "build" / "python"))
    try:
        import sbf6  # type: ignore
        import numpy as np
    except ImportError as exc:  # pragma: no cover
        print(f"[fatal] sbf6 / numpy required: {exc}")
        return 2

    robot_path = REPO_V6 / "data" / f"{args.robot}.json"
    scene_path = REPO_V6 / "data" / "scenes" / f"{args.scene}.json"
    if not robot_path.exists():
        print(f"[fatal] missing robot config: {robot_path}")
        return 2
    if not scene_path.exists():
        print(f"[fatal] missing scene config: {scene_path}")
        return 2

    robot = sbf6.Robot.from_json(str(robot_path))
    scene = sbf6.Scene.from_json(str(scene_path))
    rng = np.random.default_rng(args.seed)

    lows = np.array([iv.lo + args.margin for iv in robot.joint_limits()])
    highs = np.array([iv.hi - args.margin for iv in robot.joint_limits()])
    n_dof = len(lows)

    def free(q):
        return not sbf6.in_collision(robot, scene, q)

    pairs: list[dict] = []
    rejected = 0
    while len(pairs) < args.n:
        for _ in range(args.max_attempts_per_pair):
            qs = rng.uniform(lows, highs, size=n_dof)
            if not free(qs):
                rejected += 1
                continue
            qg = rng.uniform(lows, highs, size=n_dof)
            if not free(qg):
                rejected += 1
                continue
            qm = 0.5 * (qs + qg)
            if not free(qm):
                rejected += 1
                continue
            pairs.append({"q_start": qs.tolist(), "q_goal": qg.tolist()})
            break
        else:
            print(f"[fatal] gave up after {args.max_attempts_per_pair}"
                  " attempts on a single pair; reduce --margin or pick"
                  " a less cluttered scene.")
            return 3
        if len(pairs) % 10 == 0:
            print(f"  accepted={len(pairs)}/{args.n} rejected={rejected}")

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps({
        "robot": args.robot, "scene": args.scene, "n": args.n,
        "seed": args.seed, "margin": args.margin,
        "rejected_total": rejected, "pairs": pairs,
    }, indent=2))
    print(f"[ok] wrote {args.out} ({len(pairs)} pairs, {rejected} rejected)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

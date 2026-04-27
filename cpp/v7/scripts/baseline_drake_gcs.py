#!/usr/bin/env python3
"""baseline_drake_gcs.py — Drake GcsTrajectoryOptimization baseline.

Runs Drake's `GcsTrajectoryOptimization` between (q_start, q_goal) using
a single convex region = the joint-limit hyperrectangle of the robot.

This isolates **trajectory-shaping cost** (polynomial smoothing under
endpoint + box constraints) so it can be compared head-to-head with the
v7 SBF + 5-step path optimizer. It does **not** attempt collision-free
region decomposition (no IRIS-NP / IrisZo) — for the v7 P7 quick scenes
(``2dof_box``, ``iiwa14_far``) the workspace obstacle is far enough
outside the swept volume that the entire joint-limit box is free, so a
single region is well-posed.

Output JSON shape mirrors ``baseline_ompl``:
``{experiment, baseline, scene, robot, quick, seeds, trials[], summary}``.

Usage::

    python baseline_drake_gcs.py --scene PATH --out PATH [--quick|--full]
                                 [--seeds N] [--timeout SEC]
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

from pydrake.geometry.optimization import HPolyhedron
from pydrake.planning import GcsTrajectoryOptimization


# ─── Robot joint limits (read straight from the v7 data/*.json) ───────────────

def joint_limit_box(robot_path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Return (lo, hi) joint limit vectors for the v7 robot JSON."""
    j = json.loads(robot_path.read_text())
    lim = j["joint_limits"]
    lo = np.array([l[0] for l in lim], dtype=float)
    hi = np.array([l[1] for l in lim], dtype=float)
    return lo, hi


# ─── Single-trial GCS solve ──────────────────────────────────────────────────

def solve_gcs(q_start: np.ndarray, q_goal: np.ndarray,
              lo: np.ndarray, hi: np.ndarray,
              order: int = 3, timeout_s: float = 30.0) -> dict:
    """Solve a single GCS trajectory. Return dict with success + metrics."""
    n = len(q_start)
    t0 = time.perf_counter()
    gcs = GcsTrajectoryOptimization(num_positions=n)
    region = HPolyhedron.MakeBox(lo, hi)
    sub = gcs.AddRegions([region], order=order)
    src = gcs.AddRegions([HPolyhedron.MakeBox(q_start, q_start)], order=0)
    dst = gcs.AddRegions([HPolyhedron.MakeBox(q_goal, q_goal)], order=0)
    gcs.AddEdges(src, sub)
    gcs.AddEdges(sub, dst)
    gcs.AddTimeCost()
    gcs.AddPathLengthCost()
    try:
        traj, result = gcs.SolvePath(src, dst)
    except Exception as e:           # noqa: BLE001  — pydrake raises base Exception
        return {"success": False, "total_time_ms":
                (time.perf_counter() - t0) * 1000.0,
                "fail_reason": f"exception: {e}"}
    total_ms = (time.perf_counter() - t0) * 1000.0
    if not result.is_success():
        return {"success": False, "total_time_ms": total_ms,
                "fail_reason": result.get_solution_result().name}
    # Sample length at 50 points for a fair comparison with SBF's polyline length.
    ts = np.linspace(traj.start_time(), traj.end_time(), 50)
    pts = np.array([traj.value(t).flatten() for t in ts])
    length = float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))
    return {
        "success":       True,
        "total_time_ms": total_ms,
        "path_length":   length,
        "n_waypoints":   pts.shape[0],
        "duration_s":    float(traj.end_time() - traj.start_time()),
    }


# ─── Driver ──────────────────────────────────────────────────────────────────

def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene",   required=True, type=Path)
    ap.add_argument("--out",     type=Path)
    ap.add_argument("--quick",   action="store_true")
    ap.add_argument("--full",    action="store_true")
    ap.add_argument("--seeds",   type=int, default=None)
    ap.add_argument("--timeout", type=int, default=None,
                    help="per-trial wall timeout (s); 30 in --quick, 120 otherwise")
    a = ap.parse_args(argv)

    quick = a.quick or not a.full
    seeds = a.seeds if a.seeds is not None else (3 if quick else 20)
    timeout_s = a.timeout if a.timeout is not None else (30 if quick else 120)

    sc = json.loads(a.scene.read_text())
    repo_root = Path(__file__).resolve().parents[1]   # cpp/v7
    robot_path = repo_root / "data" / f"{sc['robot']}.json"
    lo, hi = joint_limit_box(robot_path)
    q_start = np.array(sc["q_start"], dtype=float)
    q_goal  = np.array(sc["q_goal"],  dtype=float)

    out = {
        "experiment": "main",
        "baseline":   "drake_gcs_single_region",
        "scene":      sc.get("name", a.scene.stem),
        "robot":      sc["robot"],
        "quick":      quick,
        "seeds":      seeds,
        "trials":     [],
    }

    n_success = 0
    sum_total = 0.0
    sum_len   = 0.0
    for s in range(seeds):
        # GCS is deterministic given the input → seeds only vary the run-to-run
        # noise from solver wall time; we still record per-seed timing.
        r = solve_gcs(q_start, q_goal, lo, hi, order=3, timeout_s=timeout_s)
        r["seed"] = s
        out["trials"].append(r)
        if r.get("success"):
            n_success += 1
            sum_total += r["total_time_ms"]
            sum_len   += r["path_length"]
        print(f"[drake_gcs] seed={s} success={r.get('success')} "
              f"len={r.get('path_length', 0):.4f} "
              f"t_ms={r.get('total_time_ms', 0):.2f}")

    sr = n_success / max(1, seeds)
    out["summary"] = {
        "success_rate":      sr,
        "n_success":         n_success,
        "avg_total_time_ms": sum_total / n_success if n_success else 0.0,
        "avg_path_length":   sum_len / n_success if n_success else 0.0,
    }
    print(f"[drake_gcs] SR={sr:.3f} ({n_success}/{seeds})")

    if a.out:
        a.out.parent.mkdir(parents=True, exist_ok=True)
        a.out.write_text(json.dumps(out, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Phase F -- Random-obstacle scenes and multi-robot baseline comparison.

Generates K random box-obstacle scenes for IIWA14 and (optionally)
Panda, and runs SBF, RRT-Connect, PRM with the same per-query
budget. The IIWA14 cell uses Drake-backed collision checking for
RRT/PRM (programmatic plant builder + DrakeCollisionChecker reused
from scripts/run_baselines.py); the Panda cell runs SBF only because
the in-tree benchmarking framework lacks a non-Drake collision
checker for arbitrary robots.

Output schema (one record per (robot, n_obs, scene_seed)):

    {
      "robot": "iiwa14",
      "n_obs": 16,
      "scene_seed": 0,
      "n_queries": 5,
      "sbf":         {"sr": .., "build_s": .., "query_s": .., "path_rad": ..},
      "rrt_connect": {"sr": .., "query_s": .., "path_rad": ..},
      "prm":         {"sr": .., "build_s": .., "query_s": .., "path_rad": ..}
    }

Reproduce:

    cd cpp/v6
    nice -n 15 taskset -c 0-3 \\
      env OMP_NUM_THREADS=2 MKL_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 \\
      PYTHONPATH=build/python:python python3 \\
      experiments/scripts/phaseF_random_scenes.py \\
      --iiwa14-scenes 3 --panda-scenes 2 --n-obs 16 \\
      --json experiments/results_new/F_random_scenes.json
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import List, Tuple

import numpy as np

REPO_V6 = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_V6 / "scripts"))
sys.path.insert(0, str(REPO_V6 / "build" / "python"))
sys.path.insert(0, str(REPO_V6 / "python"))

# Workspace cube (relative to robot base) inside which random obstacles
# are sampled. Avoids placing boxes inside the robot body or far below
# the base. Tuned so 16 boxes form a cluttered but solvable scene.
WORKSPACE_BOX = {
    "iiwa14": dict(lo=np.array([-0.7, -0.7, 0.05]),
                   hi=np.array([ 0.7,  0.7, 1.10]),
                   exclude_radius=0.18),  # don't spawn inside robot column
    "panda":  dict(lo=np.array([-0.6, -0.6, 0.05]),
                   hi=np.array([ 0.6,  0.6, 0.90]),
                   exclude_radius=0.16),
}

OBS_SIZE_RANGE = (0.06, 0.18)  # half-extent per axis (m)


def gen_random_obstacles(robot_key: str, n_obs: int, seed: int):
    """Return list of dicts {center: [x,y,z], half: [hx,hy,hz]} in world frame."""
    ws = WORKSPACE_BOX[robot_key]
    rng = np.random.default_rng(seed * 9973 + 7)
    boxes = []
    attempts = 0
    while len(boxes) < n_obs and attempts < n_obs * 50:
        attempts += 1
        c = rng.uniform(ws["lo"], ws["hi"])
        # Reject boxes whose center is inside the robot column
        if np.linalg.norm(c[:2]) < ws["exclude_radius"] and c[2] < 0.6:
            continue
        h = rng.uniform(OBS_SIZE_RANGE[0], OBS_SIZE_RANGE[1], size=3)
        boxes.append({"center": c.tolist(), "half": h.tolist()})
    return boxes


# ─── Drake plant builder for arbitrary random scenes ────────────────────

def build_drake_plant_with_random_boxes(robot_key: str, boxes):
    """Build a (diagram, plant) pair with the requested robot + random boxes.

    Only iiwa14 is currently implemented (uses iiwa14_spheres_collision.urdf
    plus the welded gripper, mirroring run_baselines.build_drake_plant()
    minus the Marcucci scene assets).
    """
    if robot_key != "iiwa14":
        raise NotImplementedError(
            f"Drake plant builder only supports iiwa14; got {robot_key}")
    from pydrake.systems.framework import DiagramBuilder
    from pydrake.multibody.plant import (
        AddMultibodyPlantSceneGraph, CoulombFriction)
    from pydrake.multibody.parsing import Parser
    from pydrake.multibody.tree import SpatialInertia
    from pydrake.geometry import Box
    from pydrake.math import RigidTransform

    builder = DiagramBuilder()
    plant, sg = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant, sg)
    parser.AddModels(
        url="package://drake/manipulation/models/iiwa_description/urdf/"
            "iiwa14_spheres_collision.urdf")
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"))

    obs_inst = plant.AddModelInstance("random_obstacles")
    for k, b in enumerate(boxes):
        body = plant.AddRigidBody(f"obs_{k}", obs_inst, SpatialInertia.Zero())
        plant.WeldFrames(plant.world_frame(), body.body_frame(),
                         RigidTransform(b["center"]))
        plant.RegisterCollisionGeometry(
            body, RigidTransform.Identity(),
            Box(2 * b["half"][0], 2 * b["half"][1], 2 * b["half"][2]),
            f"obs_{k}_col", CoulombFriction(0.9, 0.7))

    plant.Finalize()
    diagram = builder.Build()
    return diagram, plant


def boxes_to_sbf_obstacles(boxes):
    import sbf5
    obs = []
    for b in boxes:
        c, h = b["center"], b["half"]
        obs.append(sbf5.Obstacle(c[0]-h[0], c[1]-h[1], c[2]-h[2],
                                 c[0]+h[0], c[1]+h[1], c[2]+h[2]))
    return obs


# ─── Query sampling ─────────────────────────────────────────────────────

def sample_collision_free_pairs(checker, joint_limits, n_pairs: int,
                                seed: int, max_attempts: int = 500):
    rng = np.random.default_rng(seed * 31 + 5)
    lows = np.array([lo for lo, _ in joint_limits])
    highs = np.array([hi for _, hi in joint_limits])
    pairs = []
    attempts = 0
    while len(pairs) < n_pairs and attempts < max_attempts:
        attempts += 1
        q1 = rng.uniform(lows, highs)
        q2 = rng.uniform(lows, highs)
        if checker.check_config(q1) or checker.check_config(q2):
            continue
        # Require non-trivial separation (avoid trivially-easy queries)
        if np.linalg.norm(q1 - q2) < 1.0:
            continue
        pairs.append((f"q{len(pairs)}", q1, q2))
    return pairs


# ─── Main per-cell runner ───────────────────────────────────────────────

def run_iiwa14_cell(boxes, scene_seed: int, n_queries: int = 5,
                    rrt_timeout: float = 15.0, prm_n: int = 3000):
    """Run SBF, RRT-Connect, PRM on one iiwa14 random scene."""
    import run_baselines as rb
    import sbf5

    diagram, plant = build_drake_plant_with_random_boxes("iiwa14", boxes)
    checker = rb.DrakeCollisionChecker(diagram, plant)

    pairs = sample_collision_free_pairs(
        checker, rb.IIWA_JOINT_LIMITS, n_queries, seed=scene_seed)
    if not pairs:
        return {"error": "no collision-free queries", "n_queries": 0}

    # ── SBF (native, single shared forest across queries) ──────────────
    sbf_obs = boxes_to_sbf_obstacles(boxes)
    robot = sbf5.Robot.from_json(str(REPO_V6 / "data" / "iiwa14.json"))
    sbf_cfg = sbf5.SBFPlannerConfig()
    sbf_cfg.endpoint_source.source = sbf5.EndpointSource.CritSample
    sbf_cfg.envelope_type.type = sbf5.EnvelopeType.Hull16_Grid
    sbf_planner = sbf5.SBFPlanner(robot, sbf_cfg)
    seed_points = [np.asarray(qs, dtype=np.float64) for _, qs, _ in pairs] + \
                  [np.asarray(qg, dtype=np.float64) for _, _, qg in pairs]
    t0 = time.perf_counter()
    sbf_planner.build_coverage(sbf_obs, timeout_ms=15000,
                               seed_points=seed_points)
    sbf_build_s = time.perf_counter() - t0

    sbf_n_boxes = int(sbf_planner.n_boxes())
    sbf_lat, sbf_len, sbf_ok = [], [], 0
    if sbf_n_boxes > 0:
        for _, qs, qg in pairs:
            t0 = time.perf_counter()
            try:
                r = sbf_planner.query(np.asarray(qs, dtype=np.float64),
                                      np.asarray(qg, dtype=np.float64))
                dt = time.perf_counter() - t0
                if r.success:
                    sbf_ok += 1
                    sbf_lat.append(dt)
                    sbf_len.append(float(r.path_length))
            except Exception:
                pass

    sbf_summary = {
        "build_s": float(sbf_build_s),
        "n_boxes": sbf_n_boxes,
        "sr": 100.0 * sbf_ok / len(pairs),
        "query_s_median": float(np.median(sbf_lat)) if sbf_lat else None,
        "path_rad_mean": float(np.mean(sbf_len)) if sbf_len else None,
    }

    # ── RRT-Connect (Drake collision checker, per-query) ───────────────
    rrt_lat, rrt_len, rrt_ok = [], [], 0
    for i, (_, qs, qg) in enumerate(pairs):
        r = rb.run_rrt_connect(qs, qg, checker, timeout=rrt_timeout,
                               step_size=0.3, seed=scene_seed * 100 + i)
        if r["success"]:
            rrt_ok += 1
            rrt_lat.append(r["time_s"])
            rrt_len.append(r["path_length"])
    rrt_summary = {
        "sr": 100.0 * rrt_ok / len(pairs),
        "query_s_median": float(np.median(rrt_lat)) if rrt_lat else None,
        "path_rad_mean": float(np.mean(rrt_len)) if rrt_len else None,
    }

    # ── PRM (Drake collision checker, single roadmap shared) ───────────
    prm_lat, prm_len, prm_ok = [], [], 0
    try:
        rm = rb.PRMRoadmap(checker, n_samples=int(prm_n), k_neighbors=20,
                           connection_radius=4.0, seed=scene_seed * 101 + 1)
        rm.build(timeout=120.0)
        prm_build_s = float(rm.build_time)
        for _, qs, qg in pairs:
            r = rm.query(qs, qg, timeout=10.0)
            if r["success"]:
                prm_ok += 1
                prm_lat.append(r["time_s"])
                prm_len.append(r["path_length"])
    except Exception as exc:
        prm_build_s = float("nan")
        print(f"  PRM failed: {exc}")
    prm_summary = {
        "build_s": prm_build_s,
        "sr": 100.0 * prm_ok / len(pairs),
        "query_s_median": float(np.median(prm_lat)) if prm_lat else None,
        "path_rad_mean": float(np.mean(prm_len)) if prm_len else None,
    }

    return {
        "n_queries": len(pairs),
        "sbf": sbf_summary,
        "rrt_connect": rrt_summary,
        "prm": prm_summary,
    }


def run_panda_cell(boxes, scene_seed: int, n_queries: int = 5):
    """SBF-only run on Panda (no Drake-baseline parity)."""
    import sbf5

    sbf_obs = boxes_to_sbf_obstacles(boxes)
    robot = sbf5.Robot.from_json(str(REPO_V6 / "data" / "panda.json"))
    cfg = sbf5.SBFPlannerConfig()
    cfg.endpoint_source.source = sbf5.EndpointSource.CritSample
    cfg.envelope_type.type = sbf5.EnvelopeType.Hull16_Grid
    planner = sbf5.SBFPlanner(robot, cfg)

    # Sample queries first so they can serve as build_coverage seed points.
    jl = robot.joint_limits()
    lows = np.array([iv.lo for iv in jl.limits])
    highs = np.array([iv.hi for iv in jl.limits])
    rng = np.random.default_rng(scene_seed * 41 + 9)
    pairs = []
    for _ in range(n_queries * 6):
        if len(pairs) >= n_queries:
            break
        q1 = rng.uniform(lows, highs)
        q2 = rng.uniform(lows, highs)
        if np.linalg.norm(q1 - q2) > 1.0:
            pairs.append((q1, q2))

    seed_points = [np.asarray(q, dtype=np.float64)
                   for pair in pairs for q in pair]
    t0 = time.perf_counter()
    planner.build_coverage(sbf_obs, timeout_ms=20000,
                           seed_points=seed_points)
    build_s = time.perf_counter() - t0

    n_boxes = int(planner.n_boxes())
    lat, lens, ok = [], [], 0
    if n_boxes == 0:
        # Degenerate scene (over-constrained or seed points all infeasible)
        return {
            "n_queries": len(pairs),
            "sbf": {
                "build_s": float(build_s), "n_boxes": 0,
                "sr": 0.0, "query_s_median": None, "path_rad_mean": None,
                "note": "empty forest",
            },
        }
    for qs, qg in pairs:
        t0 = time.perf_counter()
        try:
            r = planner.query(np.asarray(qs, dtype=np.float64),
                              np.asarray(qg, dtype=np.float64))
            dt = time.perf_counter() - t0
            if r.success:
                ok += 1
                lat.append(dt)
                lens.append(float(r.path_length))
        except Exception:
            pass

    return {
        "n_queries": len(pairs),
        "sbf": {
            "build_s": float(build_s),
            "n_boxes": n_boxes,
            "sr": 100.0 * ok / max(1, len(pairs)),
            "query_s_median": float(np.median(lat)) if lat else None,
            "path_rad_mean": float(np.mean(lens)) if lens else None,
        },
    }


# ─── CLI ────────────────────────────────────────────────────────────────

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--iiwa14-scenes", type=int, default=3,
                    help="Number of random IIWA14 scenes (seeds 0..N-1)")
    ap.add_argument("--panda-scenes", type=int, default=2,
                    help="Number of random Panda scenes (seeds 0..N-1)")
    ap.add_argument("--n-obs", type=int, default=16,
                    help="Number of random box obstacles per IIWA14 scene")
    ap.add_argument("--panda-n-obs", type=int, default=10,
                    help="Number of random box obstacles per Panda scene")
    ap.add_argument("--n-queries", type=int, default=5)
    ap.add_argument("--rrt-timeout", type=float, default=15.0)
    ap.add_argument("--prm-n", type=int, default=3000)
    ap.add_argument("--json", type=Path, required=True)
    ap.add_argument("--single", nargs=2, metavar=("ROBOT", "SEED"),
                    default=None,
                    help="Run a single cell and print JSON to stdout")
    args = ap.parse_args()

    if args.single is not None:
        robot_key, seed_str = args.single
        seed = int(seed_str)
        if robot_key == "iiwa14":
            boxes = gen_random_obstacles("iiwa14", args.n_obs, seed)
            r = run_iiwa14_cell(boxes, scene_seed=seed,
                                n_queries=args.n_queries,
                                rrt_timeout=args.rrt_timeout,
                                prm_n=args.prm_n)
            r.update(robot="iiwa14", n_obs=args.n_obs, scene_seed=seed)
        else:
            boxes = gen_random_obstacles("panda", args.panda_n_obs, seed)
            r = run_panda_cell(boxes, scene_seed=seed,
                               n_queries=args.n_queries)
            r.update(robot="panda", n_obs=args.panda_n_obs, scene_seed=seed)
        sys.stdout.write("__CELL_JSON__" + json.dumps(r) + "\n")
        return 0

    args.json.parent.mkdir(parents=True, exist_ok=True)

    import subprocess

    def run_cell_subproc(robot_key: str, seed: int) -> dict:
        cmd = [sys.executable, str(Path(__file__).resolve()),
               "--single", robot_key, str(seed),
               "--n-obs", str(args.n_obs),
               "--panda-n-obs", str(args.panda_n_obs),
               "--n-queries", str(args.n_queries),
               "--rrt-timeout", str(args.rrt_timeout),
               "--prm-n", str(args.prm_n),
               "--json", "/dev/null"]
        t0 = time.time()
        try:
            cp = subprocess.run(cmd, capture_output=True, text=True,
                                timeout=600)
        except subprocess.TimeoutExpired:
            return {"robot": robot_key, "scene_seed": seed,
                    "error": "timeout", "wall_s": time.time() - t0}
        wall = time.time() - t0
        if cp.returncode != 0:
            return {"robot": robot_key, "scene_seed": seed,
                    "error": f"exit={cp.returncode}",
                    "stderr_tail": cp.stderr[-200:],
                    "wall_s": wall}
        for line in cp.stdout.splitlines():
            if line.startswith("__CELL_JSON__"):
                r = json.loads(line[len("__CELL_JSON__"):])
                r["wall_s"] = wall
                return r
        return {"robot": robot_key, "scene_seed": seed,
                "error": "no_json_marker", "wall_s": wall}

    cells = []
    for seed in range(args.iiwa14_scenes):
        print(f"[iiwa14 seed={seed}] running...", flush=True)
        r = run_cell_subproc("iiwa14", seed)
        cells.append(r)
        print(f"  -> {r}", flush=True)

    for seed in range(args.panda_scenes):
        print(f"[panda seed={seed}] running...", flush=True)
        r = run_cell_subproc("panda", seed)
        cells.append(r)
        print(f"  -> {r}", flush=True)

    args.json.write_text(json.dumps({
        "n_obs": args.n_obs,
        "n_queries": args.n_queries,
        "rrt_timeout_s": args.rrt_timeout,
        "prm_n": args.prm_n,
        "cells": cells,
    }, indent=2))
    print(f"Wrote {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

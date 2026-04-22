#!/usr/bin/env python3
"""check_gcs_collisions.py — Drake collision audit of TABLE XV GCS waypoints.

Loads ``gcs_pipeline_results.json`` and runs a Drake
``SceneGraphCollisionChecker`` (full IIWA14 sphere collision model + welded
gripper, exactly the same model used by the GCS / IRIS-NP comparison) over

  • each waypoint    — vertex collision check
  • each linear edge — densified edge collision check at ``--step`` rad

Reports per-pair: # vertex collisions, # edge violations, max penetration step,
and the offending step indices.  Then prints a root-cause summary.
"""
import argparse
import json
import os
import sys
import time

import numpy as np

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_V6_DIR = os.path.dirname(_THIS_DIR)
sys.path.insert(0, _THIS_DIR)

from run_online_query_comparison import build_drake_scene_graph_checker  # noqa: E402


def path_length(wps):
    return float(sum(np.linalg.norm(wps[i] - wps[i - 1])
                     for i in range(1, len(wps))))


def audit_path(checker, wps_full, step_rad=0.01):
    """Audit a path. Returns dict with vertex/edge collision stats.

    The checker expects a configuration whose dimension matches the plant
    (iiwa 7 + wsg 2 = 9 DoF).  We pad the gripper with zeros."""
    wps = np.asarray(wps_full, dtype=float)
    n = len(wps)
    nq_full = checker.plant().num_positions()
    pad = nq_full - wps.shape[1]
    if pad > 0:
        wps = np.hstack([wps, np.zeros((n, pad))])
    elif pad < 0:
        wps = wps[:, :nq_full]

    # Vertex check
    vtx_bad = []
    for i, q in enumerate(wps):
        if not checker.CheckConfigCollisionFree(q):
            vtx_bad.append(i)

    # Edge check (linear interpolation, dense step)
    edge_bad = []  # list of (edge_idx, t_first_collision)
    edge_lens = []
    for i in range(n - 1):
        q0, q1 = wps[i], wps[i + 1]
        L = float(np.linalg.norm(q1 - q0))
        edge_lens.append(L)
        if L < 1e-9:
            continue
        n_steps = max(2, int(np.ceil(L / step_rad)))
        first_bad_t = None
        n_bad = 0
        for k in range(n_steps + 1):
            t = k / n_steps
            q = q0 + t * (q1 - q0)
            if not checker.CheckConfigCollisionFree(q):
                if first_bad_t is None:
                    first_bad_t = t
                n_bad += 1
        if n_bad > 0:
            edge_bad.append({
                "edge": i, "first_t": first_bad_t,
                "n_bad_samples": n_bad, "n_samples": n_steps + 1,
                "edge_len_rad": L,
            })

    return {
        "n_wp": n,
        "path_len_rad": path_length(wps[:, : checker.plant().num_positions() - pad]
                                    if pad > 0 else wps),
        "vertex_bad": vtx_bad,
        "edge_bad": edge_bad,
        "edge_lens": edge_lens,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gcs", default="/tmp/gcs_corridor_v2/gcs_pipeline_results.json",
                    help="GCS waypoints JSON")
    ap.add_argument("--step", type=float, default=0.01,
                    help="edge densification step in rad (default 0.01)")
    ap.add_argument("--out", default="/tmp/gcs_collision_audit.json")
    args = ap.parse_args()

    print(f"loading GCS waypoints from {args.gcs}")
    data = json.load(open(args.gcs))
    paths = [r for r in data.get("results", [])
             if r.get("success") and r.get("waypoints")]
    print(f"{len(paths)} GCS-successful paths")

    print("building Drake SceneGraphCollisionChecker (iiwa14 spheres + welded "
          "gripper, same model as IRIS-NP+GCS baseline) ...")
    t0 = time.perf_counter()
    checker, plant, _ = build_drake_scene_graph_checker(
        edge_step_size=args.step, parallelism=1)
    print(f"  ready in {time.perf_counter()-t0:.2f}s, "
          f"plant.num_positions={plant.num_positions()}")

    summary = []
    print(f"\nauditing edges at step={args.step} rad")
    print(f"{'pair':<10} {'wp':>3} {'len':>6} "
          f"{'vtxBad':>7} {'edgeBad':>8} {'first_t':>8} {'frac_bad':>9}")
    for r in paths:
        wps = np.asarray(r["waypoints"], dtype=float)
        a = audit_path(checker, wps, step_rad=args.step)
        bad_frac = 0.0
        first_t = float("nan")
        if a["edge_bad"]:
            tot_samples = sum(e["n_samples"] for e in a["edge_bad"])
            tot_bad = sum(e["n_bad_samples"] for e in a["edge_bad"])
            bad_frac = tot_bad / max(1, tot_samples)
            first_t = a["edge_bad"][0]["first_t"]
        print(f"{r['label']:<10} {a['n_wp']:>3d} "
              f"{r.get('gcs_len',0):>6.3f} "
              f"{len(a['vertex_bad']):>7d} "
              f"{len(a['edge_bad']):>8d} "
              f"{first_t:>8.3f} {bad_frac*100:>7.1f}%")
        summary.append({
            "label": r["label"],
            "pair_idx": r["pair_idx"],
            "n_wp": a["n_wp"],
            "gcs_len": r.get("gcs_len"),
            "vertex_bad": a["vertex_bad"],
            "edge_bad": a["edge_bad"],
            "edge_lens": a["edge_lens"],
        })

    json.dump({"step_rad": args.step, "paths": summary},
              open(args.out, "w"), indent=1)
    print(f"\nwrote {args.out}")

    # ── root-cause summary ──
    n_with_edge_bad = sum(1 for s in summary if s["edge_bad"])
    n_with_vtx_bad = sum(1 for s in summary if s["vertex_bad"])
    print("\n── summary ──")
    print(f"  paths with vertex collisions: {n_with_vtx_bad}/{len(summary)}")
    print(f"  paths with edge   collisions: {n_with_edge_bad}/{len(summary)}")
    if summary:
        max_edge_len = max(L for s in summary for L in s["edge_lens"])
        print(f"  longest single edge: {max_edge_len:.3f} rad")


if __name__ == "__main__":
    main()

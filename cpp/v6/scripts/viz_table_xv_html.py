#!/usr/bin/env python3
"""viz_table_xv_html.py — Render TABLE XV SBF→GCS paths with Drake Meshcat → HTML.

Reuses ``cpp/v6/viz/viz_drake_paths.py`` infrastructure.  Loads GCS waypoints
from ``gcs_pipeline_results.json`` (and optionally SBF waypoints from
``exp2_paths.json``) and exports a static interactive Meshcat HTML.

Usage:
    cd cpp/v6
    PYTHONPATH=build/python:python python3 scripts/viz_table_xv_html.py \\
        --gcs /tmp/gcs_corridor_v2/gcs_pipeline_results.json \\
        --sbf /tmp/exp2_paths.json --seed 4 \\
        --out /tmp/table_xv_paths.html
"""
import argparse
import json
import os
import sys
from datetime import datetime

import numpy as np

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_V6_DIR = os.path.dirname(_THIS_DIR)
sys.path.insert(0, os.path.join(_V6_DIR, "viz"))

from viz_drake_paths import (  # type: ignore  # noqa: E402
    build_scene,
    visualize_trajectory,
    visualize_static,
)
from pydrake.geometry import StartMeshcat  # noqa: E402


PAIR_NAMES = ["AS->TS", "TS->CS", "CS->LB", "LB->RB", "RB->AS"]


def load_gcs_paths(gcs_json):
    with open(gcs_json) as f:
        data = json.load(f)
    out = []
    for r in data.get("results", []):
        if not r.get("success") or not r.get("waypoints"):
            continue
        out.append({
            "pair_idx": r["pair_idx"],
            "label": r.get("label"),
            "waypoints": np.asarray(r["waypoints"], dtype=float),
            "path_length": r.get("gcs_len", 0.0),
        })
    out.sort(key=lambda p: p["pair_idx"])
    return out


def load_sbf_paths(sbf_json, seed):
    with open(sbf_json) as f:
        data = json.load(f)
    out = []
    for p in data.get("paths", []):
        if seed is not None and p.get("seed") != seed:
            continue
        if not p.get("success"):
            continue
        wp = p.get("waypoints", [])
        if len(wp) < 2:
            continue
        out.append({
            "pair_idx": p["pair_idx"],
            "label": p.get("label"),
            "waypoints": np.asarray(wp, dtype=float),
            "path_length": p.get("path_length", 0.0),
        })
    out.sort(key=lambda p: p["pair_idx"])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gcs", required=True,
                    help="gcs_pipeline_results.json (TABLE XV GCS waypoints)")
    ap.add_argument("--sbf", default=None,
                    help="exp2_paths.json with SBF waypoints (optional, for SBF mode)")
    ap.add_argument("--seed", type=int, default=4,
                    help="SBF seed to load (default 4 — matches gcs_pipeline)")
    ap.add_argument("--source", choices=["gcs", "sbf", "both"], default="gcs",
                    help="which set to render (default: gcs)")
    ap.add_argument("--out", default="/tmp/table_xv_paths.html",
                    help="output HTML path")
    ap.add_argument("--static", action="store_true",
                    help="static mode (end-effector point cloud + final pose)")
    ap.add_argument("--speed", type=float, default=1.5)
    ap.add_argument("--keep-open", action="store_true",
                    help="keep meshcat server open after writing HTML")
    args = ap.parse_args()

    sets = []
    if args.source in ("gcs", "both"):
        sets.append(("GCS", load_gcs_paths(args.gcs)))
    if args.source in ("sbf", "both"):
        if not args.sbf:
            print("--sbf required for source=sbf|both", file=sys.stderr)
            sys.exit(2)
        sets.append(("SBF", load_sbf_paths(args.sbf, args.seed)))

    for name, paths in sets:
        if not paths:
            print(f"[warn] no paths in {name}", file=sys.stderr)
            sys.exit(1)
        print(f"[{name}] {len(paths)} paths:")
        for p in paths:
            print(f"  {p['label']:8s}  {p['waypoints'].shape[0]:>3d} wp  "
                  f"len={p['path_length']:.3f}")

    meshcat = StartMeshcat()
    print(f"Meshcat: {meshcat.web_url()}")

    # Combine all paths from all sets in order
    waypoints_list = []
    pair_indices = []
    labels = []
    for name, paths in sets:
        for p in paths:
            waypoints_list.append(p["waypoints"])
            pair_indices.append(p["pair_idx"])
            labels.append(f"{name} {p['label']}")

    if args.static:
        build_scene(meshcat)
        visualize_static(meshcat, waypoints_list, pair_indices)
    else:
        total = visualize_trajectory(meshcat, waypoints_list, pair_indices,
                                     speed=args.speed)
        print(f"animation total time {total:.1f}s")

    ts = datetime.now().strftime("_%Y%m%d_%H%M%S")
    base, ext = os.path.splitext(args.out)
    out_path = f"{base}{ts}{ext or '.html'}"
    html = meshcat.StaticHtml()
    with open(out_path, "w") as f:
        f.write(html)
    print(f"wrote {out_path} ({len(html)/1024:.1f} KB)")

    if args.keep_open:
        print("Press Ctrl+C to exit")
        import time
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()

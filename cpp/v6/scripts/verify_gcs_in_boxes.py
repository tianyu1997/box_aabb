#!/usr/bin/env python3
"""verify_gcs_in_boxes.py — check GCS waypoints/edges lie in SBF corridor boxes.

For each TABLE XV GCS path:
  • per-waypoint: is it inside any SBF box (with tolerance) ?
  • per-edge:    sample uniformly; every sample must lie in at least one box.

Writes /tmp/gcs_in_box_audit.json and prints a summary table.
"""
import argparse
import json
import sys

import numpy as np


def load_boxes(paths_json):
    d = json.load(open(paths_json))
    boxes = []
    for bd in d.get("boxes", []):
        boxes.append((int(bd["id"]),
                      np.asarray(bd["lo"], dtype=float),
                      np.asarray(bd["hi"], dtype=float)))
    return boxes


def in_any_box(q, boxes, tol):
    for bid, lo, hi in boxes:
        if np.all(q >= lo - tol) and np.all(q <= hi + tol):
            return bid
    return -1


def audit(wps, boxes, n_seg=64, tol=1e-6):
    wps = np.asarray(wps, dtype=float)
    n = len(wps)
    vtx_owner = [in_any_box(q, boxes, tol) for q in wps]
    n_vtx_out = sum(1 for o in vtx_owner if o < 0)

    edge_violations = []
    for i in range(n - 1):
        q0, q1 = wps[i], wps[i + 1]
        n_bad = 0
        first_t = None
        for k in range(1, n_seg):
            t = k / n_seg
            q = q0 + t * (q1 - q0)
            if in_any_box(q, boxes, tol) < 0:
                if first_t is None:
                    first_t = t
                n_bad += 1
        if n_bad:
            edge_violations.append({
                "edge": i, "n_bad": n_bad, "n_samples": n_seg - 1,
                "first_t": first_t,
                "edge_len": float(np.linalg.norm(q1 - q0)),
            })
    return {"vtx_owner": vtx_owner, "n_vtx_out": n_vtx_out,
            "edge_violations": edge_violations}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--paths", default="/tmp/exp2_paths.json")
    ap.add_argument("--gcs", default="/tmp/gcs_corridor_v2/gcs_pipeline_results.json")
    ap.add_argument("--n-seg", type=int, default=64,
                    help="samples per edge (excl. endpoints)")
    ap.add_argument("--tol", type=float, default=1e-6,
                    help="box containment tolerance (rad)")
    ap.add_argument("--out", default="/tmp/gcs_in_box_audit.json")
    args = ap.parse_args()

    boxes = load_boxes(args.paths)
    print(f"loaded {len(boxes)} SBF boxes from {args.paths}")
    gcs = json.load(open(args.gcs))
    paths = [r for r in gcs["results"] if r.get("success") and r.get("waypoints")]
    print(f"{len(paths)} GCS paths")

    print(f"\n{'pair':<10} {'wp':>3} {'vtxOut':>6} {'edgeViol':>8} "
          f"{'firstViol':>9} {'maxEdge':>7}")
    summary = []
    for r in paths:
        wps = r["waypoints"]
        a = audit(wps, boxes, n_seg=args.n_seg, tol=args.tol)
        max_edge = max(float(np.linalg.norm(np.array(wps[i+1]) - np.array(wps[i])))
                       for i in range(len(wps) - 1)) if len(wps) > 1 else 0.0
        first_viol = (a["edge_violations"][0]["edge"]
                      if a["edge_violations"] else -1)
        print(f"{r['label']:<10} {len(wps):>3d} "
              f"{a['n_vtx_out']:>6d} "
              f"{len(a['edge_violations']):>8d} "
              f"{first_viol:>9d} {max_edge:>7.3f}")
        summary.append({
            "label": r["label"], "pair_idx": r["pair_idx"],
            "n_wp": len(wps),
            "vtx_owner": a["vtx_owner"],
            "n_vtx_out": a["n_vtx_out"],
            "edge_violations": a["edge_violations"],
            "max_edge_len": max_edge,
        })

    json.dump({"tol": args.tol, "n_seg": args.n_seg, "paths": summary},
              open(args.out, "w"), indent=1)
    print(f"\nwrote {args.out}")

    # ── summary ──
    n_ok_vtx = sum(1 for s in summary if s["n_vtx_out"] == 0)
    n_ok_edge = sum(1 for s in summary if not s["edge_violations"])
    print("\n── containment summary ──")
    print(f"  paths fully in boxes (vertices):       {n_ok_vtx}/{len(summary)}")
    print(f"  paths fully in boxes (edges sampled):  {n_ok_edge}/{len(summary)}")


if __name__ == "__main__":
    main()

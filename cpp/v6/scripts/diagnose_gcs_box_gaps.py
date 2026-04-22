#!/usr/bin/env python3
"""diagnose_gcs_box_gaps.py — figure out *why* GCS edges leave the SBF boxes.

For each violating edge in /tmp/gcs_in_box_audit.json:
  • find which box owns the source waypoint and which owns the target
  • test whether the two owner boxes intersect (overlap volume)
  • test whether they are listed as adjacent in adjacency
  • report the fraction of segment-length spent OUTSIDE every box

Output: /tmp/gcs_box_gaps.json + console table.
"""
import json
import sys
import numpy as np


def load_data():
    p = json.load(open("/tmp/exp2_paths.json"))
    boxes = {int(b["id"]): (np.asarray(b["lo"], float),
                            np.asarray(b["hi"], float))
             for b in p["boxes"]}
    adj = {int(k): set(v) for k, v in p.get("adjacency", {}).items()}
    return boxes, adj


def boxes_intersect(box_a, box_b, tol=0.0):
    lo_a, hi_a = box_a
    lo_b, hi_b = box_b
    lo = np.maximum(lo_a, lo_b)
    hi = np.minimum(hi_a, hi_b)
    if np.all(hi >= lo - tol):
        return True, float(np.prod(np.maximum(hi - lo, 0.0)))
    return False, 0.0


def in_any_box(q, boxes, tol=1e-6):
    for bid, (lo, hi) in boxes.items():
        if np.all(q >= lo - tol) and np.all(q <= hi + tol):
            return bid
    return -1


def main():
    boxes, adj = load_data()
    print(f"loaded {len(boxes)} boxes, |adj| = {sum(len(v) for v in adj.values())//2}")
    gcs = json.load(open("/tmp/gcs_corridor_v2/gcs_pipeline_results.json"))
    audit = json.load(open("/tmp/gcs_in_box_audit.json"))

    rows = []
    for r, a in zip(gcs["results"], audit["paths"]):
        if not a["edge_violations"]:
            continue
        wps = np.asarray(r["waypoints"])
        for ev in a["edge_violations"]:
            i = ev["edge"]
            q0, q1 = wps[i], wps[i + 1]
            b0, b1 = a["vtx_owner"][i], a["vtx_owner"][i + 1]
            inter, vol = boxes_intersect(boxes[b0], boxes[b1])
            adj_listed = (b1 in adj.get(b0, set())) or (b0 in adj.get(b1, set()))
            # length-fraction outside boxes
            n = 200
            n_out = sum(1 for k in range(1, n)
                        if in_any_box(q0 + (k/n) * (q1 - q0), boxes) < 0)
            frac_out = n_out / (n - 1)
            rows.append({
                "label": r["label"], "edge": i,
                "box0": b0, "box1": b1, "edge_len": ev["edge_len"],
                "owner_overlap": inter, "overlap_vol": vol,
                "adj_listed": adj_listed,
                "frac_segment_outside_any_box": frac_out,
            })

    print(f"\n{'pair':<10} {'e':>3} {'b0':>5} {'b1':>5} "
          f"{'ovlp':>5} {'adj':>4} {'len':>6} {'frac_out':>9}")
    for r in rows:
        print(f"{r['label']:<10} {r['edge']:>3d} "
              f"{r['box0']:>5d} {r['box1']:>5d} "
              f"{'Y' if r['owner_overlap'] else 'N':>5s} "
              f"{'Y' if r['adj_listed'] else 'N':>4s} "
              f"{r['edge_len']:>6.3f} {r['frac_segment_outside_any_box']*100:>7.1f}%")

    json.dump(rows, open("/tmp/gcs_box_gaps.json", "w"), indent=1)
    n_overlap = sum(1 for r in rows if r["owner_overlap"])
    n_adj = sum(1 for r in rows if r["adj_listed"])
    print("\n── why edges leave the corridor ──")
    print(f"  total violating edges:        {len(rows)}")
    print(f"  with overlapping owner boxes: {n_overlap}/{len(rows)}")
    print(f"  with adjacency-listed owners: {n_adj}/{len(rows)}")


if __name__ == "__main__":
    main()

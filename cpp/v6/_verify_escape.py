"""Measure the true maximum deviation of each path from the box union."""
import json
import sys

import numpy as np

sys.path.insert(0, "scripts")
from gcs_query import load_forest_from_json

EPS = 1e-6
N_PER_SEG = 200

def max_deviation(wp, lo, hi):
    n_dirty = 0
    n_segs = len(wp) - 1
    max_dev = 0.0
    max_dev_seg = -1
    tot_esc_len = 0.0
    n_boxes = lo.shape[0]
    # process in chunks of segments to limit memory
    for i in range(n_segs):
        ts = np.linspace(0.0, 1.0, N_PER_SEG + 1)[:, None]
        pts = wp[i] + ts * (wp[i + 1] - wp[i])  # (N+1, d)
        # broadcast pts vs boxes in chunks of 256 boxes
        in_any = np.zeros(pts.shape[0], dtype=bool)
        out_dev = np.full(pts.shape[0], np.inf)
        for b0 in range(0, n_boxes, 256):
            blo = lo[b0:b0 + 256]
            bhi = hi[b0:b0 + 256]
            inside = np.all((pts[:, None, :] >= blo[None] - EPS) & (pts[:, None, :] <= bhi[None] + EPS), axis=2)
            in_any |= np.any(inside, axis=1)
            # min deviation across these boxes
            clamped = np.clip(pts[:, None, :], blo[None], bhi[None])
            dev_per_box = np.max(np.abs(clamped - pts[:, None, :]), axis=2)
            out_dev = np.minimum(out_dev, dev_per_box.min(axis=1))
        if not np.all(in_any):
            n_dirty += 1
            tot_esc_len += float(np.linalg.norm(wp[i + 1] - wp[i]))
            md = float(out_dev[~in_any].max())
            if md > max_dev:
                max_dev = md
                max_dev_seg = i
    return n_segs, n_dirty, max_dev, max_dev_seg, tot_esc_len


def main():
    forest = load_forest_from_json("result/20260416_163139_b200000_rrt_bq_dijk_s5/paths.json")
    lo, hi = forest["lo"], forest["hi"]
    data = json.load(open("result/20260416_163139_b200000_rrt_bq_dijk_s5/gcs_query_results.json"))
    print(f"{'label':10s} {'segs':>4s} {'dirty':>5s}  {'max_dev':>10s} {'max_seg':>7s} {'totEscLen':>10s}")
    for r in data["results"]:
        wp = np.array(r["waypoints"])
        ns, nd, md, ms, te = max_deviation(wp, lo, hi)
        print(f"{r['label']:10s} {ns:4d} {nd:5d}  {md:10.6f} {ms:7d} {te:10.3f}")

if __name__ == "__main__":
    main()

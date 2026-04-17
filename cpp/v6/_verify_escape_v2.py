"""Verify v2 (SOCP) output escapes."""
import json, sys
import numpy as np
sys.path.insert(0, "scripts")
from gcs_query_v2 import load_forest_from_json

EPS = 1e-6
N = 400

def check(wp, lo, hi):
    n_dirty = 0
    max_dev = 0.0
    max_i = -1
    for i in range(len(wp) - 1):
        ts = np.linspace(0, 1, N + 1)[:, None]
        pts = wp[i] + ts * (wp[i + 1] - wp[i])
        in_any = np.zeros(pts.shape[0], dtype=bool)
        dev_min = np.full(pts.shape[0], np.inf)
        for b0 in range(0, lo.shape[0], 512):
            blo = lo[b0:b0 + 512]; bhi = hi[b0:b0 + 512]
            inside = np.all((pts[:, None, :] >= blo[None] - EPS) & (pts[:, None, :] <= bhi[None] + EPS), axis=2)
            in_any |= np.any(inside, axis=1)
            clamped = np.clip(pts[:, None, :], blo[None], bhi[None])
            d = np.max(np.abs(clamped - pts[:, None, :]), axis=2)
            dev_min = np.minimum(dev_min, d.min(axis=1))
        if not np.all(in_any):
            n_dirty += 1
            md = float(dev_min[~in_any].max())
            if md > max_dev:
                max_dev, max_i = md, i
    return n_dirty, max_dev, max_i

def main():
    forest = load_forest_from_json("result/20260416_163139_b200000_rrt_bq_dijk_s5/paths.json")
    lo, hi = forest["lo"], forest["hi"]
    data = json.load(open("result/20260416_163139_b200000_rrt_bq_dijk_s5/socp_query_results.json"))
    print(f"{'label':10s} {'segs':>4s} {'dirty':>5s} {'max_dev(rad)':>13s}")
    for r in data["results"]:
        wp = np.array(r["waypoints"])
        nd, md, mi = check(wp, lo, hi)
        print(f"{r['label']:10s} {len(wp)-1:4d} {nd:5d} {md:13.6f} (seg {mi})")

if __name__ == "__main__":
    main()

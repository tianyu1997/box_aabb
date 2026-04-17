"""Fast slab-based verifier for v2 output."""
import json, sys, time
import numpy as np
sys.path.insert(0, "scripts")
from gcs_query_v2 import load_forest_from_json, segment_in_box_union_exact


def check(wp, lo, hi):
    n_dirty = 0
    max_gap = 0.0  # 1 - covered fraction
    for i in range(len(wp) - 1):
        if not segment_in_box_union_exact(wp[i], wp[i + 1], lo, hi):
            n_dirty += 1
    return n_dirty


def check_dense(wp, lo, hi, N=400):
    """Returns (n_dirty, max_dev)."""
    EPS = 1e-6
    n_dirty = 0
    max_dev = 0.0
    for i in range(len(wp) - 1):
        ts = np.linspace(0, 1, N + 1)[:, None]
        pts = wp[i] + ts * (wp[i + 1] - wp[i])
        in_any = np.zeros(pts.shape[0], dtype=bool)
        dev_min = np.full(pts.shape[0], np.inf)
        for b0 in range(0, lo.shape[0], 1024):
            blo = lo[b0:b0 + 1024]; bhi = hi[b0:b0 + 1024]
            inside = np.all((pts[:, None, :] >= blo[None] - EPS) & (pts[:, None, :] <= bhi[None] + EPS), axis=2)
            in_any |= np.any(inside, axis=1)
            if not np.all(in_any):
                clamped = np.clip(pts[:, None, :], blo[None], bhi[None])
                d = np.max(np.abs(clamped - pts[:, None, :]), axis=2)
                dev_min = np.minimum(dev_min, d.min(axis=1))
        if not np.all(in_any):
            n_dirty += 1
            md = float(dev_min[~in_any].max())
            max_dev = max(max_dev, md)
    return n_dirty, max_dev


def main():
    forest = load_forest_from_json("result/20260416_163139_b200000_rrt_bq_dijk_s5/paths.json")
    lo, hi = forest["lo"], forest["hi"]
    data = json.load(open("result/20260416_163139_b200000_rrt_bq_dijk_s5/socp_query_results.json"))
    print(f"{'label':10s} {'segs':>4s} {'slab_dirty':>10s} {'dense_dirty':>11s} {'max_dev':>10s}")
    for r in data["results"]:
        wp = np.array(r["waypoints"])
        t0 = time.perf_counter()
        slab_dirty = check(wp, lo, hi)
        t1 = time.perf_counter()
        dense_dirty, md = check_dense(wp, lo, hi, N=200)
        t2 = time.perf_counter()
        print(f"{r['label']:10s} {len(wp)-1:4d} {slab_dirty:10d} {dense_dirty:11d} {md:10.6f}  slab={t1-t0:.2f}s dense={t2-t1:.2f}s")


if __name__ == "__main__":
    main()

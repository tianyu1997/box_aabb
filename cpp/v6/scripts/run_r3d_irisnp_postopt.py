"""R3-D2: Apply random-shortcut post-optimisation to IRIS-NP+GCS paths.

Loads cached IRIS-NP regions from
  cpp/v6/experiments/results_new/iris_regions_cache.npz
runs GCS on the 5 canonical queries to obtain raw paths, then applies
the same random-shortcut algorithm as the SBF 5-step pipeline
(cpp/v6/src/planner/path_smoother.cpp::shortcut) and reports per-query
length before/after for the R3-D2 fairness comparison.
"""
import json
import os
import sys
import time
import random
import argparse

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from run_baselines import (  # noqa: E402
    _get_plant_cached,
    _resolve_query_pairs,
    run_iris_gcs,
    shortcut_path,
    QUERY_PAIRS,
    IIWA_CONFIGS,
    logger,
)
import numpy as np  # noqa: E402


def path_length(waypoints):
    return float(sum(np.linalg.norm(np.array(waypoints[i + 1])
                                    - np.array(waypoints[i]))
                     for i in range(len(waypoints) - 1)))


def load_cached_regions(npz_path):
    from pydrake.geometry.optimization import HPolyhedron
    cached = np.load(npz_path, allow_pickle=True)
    n = int(cached["n_regions"])
    regions = [HPolyhedron(cached[f"A_{i}"], cached[f"b_{i}"]) for i in range(n)]
    logger.info(f"Loaded {n} cached IRIS-NP regions from {npz_path}")
    return regions


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--out", default="cpp/v6/experiments/results_r3d/D2_irisnp_postopt.json")
    p.add_argument("--regions", default="cpp/v6/experiments/results_new/iris_regions_cache.npz")
    p.add_argument("--seed", type=int, default=0)
    args = p.parse_args()

    diagram, plant, checker = _get_plant_cached()
    regions = load_cached_regions(args.regions)
    pairs = _resolve_query_pairs(None)

    rows = []
    for label, q0, q1 in pairs:
        q0 = np.asarray(q0); q1 = np.asarray(q1)
        try:
            res = run_iris_gcs(q0, q1, plant, diagram, regions, seed=args.seed)
        except Exception as exc:
            logger.warning(f"  {label}: GCS failed: {exc}")
            continue
        if not res.get("success") or res.get("path") is None:
            logger.warning(f"  {label}: no path returned")
            continue
        before = float(res["path_length"])
        path = [np.asarray(p, dtype=float) for p in res["path"]]
        t0 = time.perf_counter()
        rng = np.random.default_rng(args.seed)
        post = shortcut_path(path, checker, max_iters=200, rng=rng)
        dt = time.perf_counter() - t0
        after = path_length(post)
        rows.append({
            "label": label,
            "len_before_rad": before,
            "len_after_rad": after,
            "ratio_after_over_before": after / before if before > 0 else None,
            "n_waypoints_before": len(path),
            "n_waypoints_after": len(post),
            "shortcut_time_s": dt,
            "gcs_solve_time_s": float(res["time_s"]),
        })
        logger.info(f"  {label}: {before:.3f} -> {after:.3f}rad "
                    f"({(after / before) * 100:.1f}%, "
                    f"{len(path)}->{len(post)} wp, {dt:.2f}s)")

    if rows:
        ratios = [r["ratio_after_over_before"] for r in rows
                  if r["ratio_after_over_before"] is not None]
        median_ratio = float(np.median(ratios))
        median_before = float(np.median([r["len_before_rad"] for r in rows]))
        median_after = float(np.median([r["len_after_rad"] for r in rows]))
    else:
        median_ratio = median_before = median_after = None

    out = {
        "seed": args.seed,
        "n_regions_loaded": len(regions),
        "rows": rows,
        "median_ratio_after_over_before": median_ratio,
        "median_len_before_rad": median_before,
        "median_len_after_rad": median_after,
    }
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w") as f:
        json.dump(out, f, indent=2)
    logger.info(f"Wrote {args.out}")
    logger.info(f"Median path-length ratio after/before = {median_ratio}")
    logger.info(f"Median len before = {median_before}; after = {median_after}")


if __name__ == "__main__":
    main()

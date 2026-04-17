"""Diagnose 'jump' edges in adj — edges between geometrically non-overlapping boxes.

For each such edge:
  - Compute per-axis gap (how far apart the AABBs are along each dim).
  - Report max gap across dims.
Then build the geometric-overlap graph (union-find over strictly overlapping
box pairs) and report the connected component sizes of each endpoint.
"""
import json
import sys
import argparse
import numpy as np
from collections import Counter


def load(json_path):
    data = json.load(open(json_path))
    boxes = data["boxes"]
    N = len(boxes)
    D = data["dof"]
    lo = np.zeros((N, D))
    hi = np.zeros((N, D))
    ids = []
    for i, b in enumerate(boxes):
        lo[i] = b["lo"]
        hi[i] = b["hi"]
        ids.append(b["id"])
    adj = {int(k): v for k, v in data.get("adjacency", {}).items()}
    return lo, hi, ids, adj


def build_geom_components(lo, hi, eps=1e-6):
    """Union-find over pairs that strictly overlap (AABB slab)."""
    N = lo.shape[0]
    parent = list(range(N))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    # For each box a, find all boxes that overlap it, union.
    for a in range(N):
        mask = np.all((lo <= hi[a] + eps) & (hi >= lo[a] - eps), axis=1)
        neigh = np.where(mask)[0]
        for b in neigh:
            if b != a:
                union(a, int(b))

    roots = [find(i) for i in range(N)]
    sizes = Counter(roots)
    return roots, sizes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("paths_json")
    ap.add_argument("--eps", type=float, default=1e-6)
    ap.add_argument("--top", type=int, default=30, help="print top-N worst edges")
    args = ap.parse_args()

    lo, hi, ids, adj = load(args.paths_json)
    id2row = {b: i for i, b in enumerate(ids)}
    N = lo.shape[0]
    D = lo.shape[1]
    print(f"Loaded {N} boxes (D={D}), {sum(len(v) for v in adj.values())} directed adj edges")

    # Enumerate non-overlapping edges
    jumps = []  # (gap_max, gap_per_axis, a, b)
    total = 0
    for bid, neigh in adj.items():
        if bid not in id2row:
            continue
        a = id2row[bid]
        for nid in neigh:
            if nid not in id2row:
                continue
            b = id2row[nid]
            if a >= b:
                continue  # dedupe directed → undirected
            total += 1
            gap_per = np.maximum(lo[a] - hi[b], lo[b] - hi[a])  # negative if overlap
            gap_max = float(np.max(gap_per))
            if gap_max > args.eps:
                jumps.append((gap_max, gap_per.copy(), a, b))

    print(f"\nTotal undirected adj edges: {total}")
    print(f"Non-overlapping (jump) edges: {len(jumps)} ({100*len(jumps)/max(total,1):.3f}%)")

    if not jumps:
        print("No jump edges — adj is fully geometry-consistent.")
        return

    # Build components on strictly-overlapping graph and report
    print("\nBuilding geometric-overlap connected components...")
    roots, sizes = build_geom_components(lo, hi, eps=args.eps)
    n_components = len(sizes)
    print(f"Components: {n_components}, largest={max(sizes.values())}, "
          f"smallest={min(sizes.values())}")
    size_hist = Counter(sizes.values())
    print(f"Size histogram (size: count): "
          f"{dict(sorted(size_hist.items(), key=lambda x: -x[0])[:10])}")

    # Categorize jump edges: same-component vs cross-component
    same_comp = 0
    cross_comp = 0
    for gmax, gper, a, b in jumps:
        if roots[a] == roots[b]:
            same_comp += 1
        else:
            cross_comp += 1
    print(f"\nJump edges within same geom-component: {same_comp}")
    print(f"Jump edges crossing components:        {cross_comp}")

    # Gap distribution
    gaps = np.array([g for g, _, _, _ in jumps])
    print(f"\nGap (max over dims) stats [rad]:")
    print(f"  min   = {gaps.min():.6f}")
    print(f"  max   = {gaps.max():.6f}")
    print(f"  mean  = {gaps.mean():.6f}")
    print(f"  median= {np.median(gaps):.6f}")
    for q in [50, 75, 90, 95, 99]:
        print(f"  p{q:2d}  = {np.percentile(gaps, q):.6f}")

    # Print top-N worst edges
    jumps.sort(key=lambda x: -x[0])
    print(f"\nTop-{min(args.top, len(jumps))} largest-gap jump edges:")
    print(f"{'#':>3s} {'a':>6s} {'b':>6s} {'gap_max':>10s} {'size(a)':>8s} "
          f"{'size(b)':>8s} {'cross':>6s}  gap_per_axis")
    for k, (gmax, gper, a, b) in enumerate(jumps[:args.top]):
        sa = sizes[roots[a]]
        sb = sizes[roots[b]]
        cross = "YES" if roots[a] != roots[b] else "no"
        gper_str = "[" + " ".join(f"{g:+.4f}" for g in gper) + "]"
        print(f"{k:3d} {a:6d} {b:6d} {gmax:10.5f} {sa:8d} {sb:8d} {cross:>6s}  {gper_str}")

    # For cross-component jumps: show component size distribution of endpoints
    if cross_comp > 0:
        print(f"\nCross-component jump edges — endpoint component size pairs:")
        pair_hist = Counter()
        for gmax, gper, a, b in jumps:
            if roots[a] != roots[b]:
                pair_hist[(sizes[roots[a]], sizes[roots[b]])] += 1
        for (s1, s2), c in sorted(pair_hist.items(), key=lambda x: -x[1])[:15]:
            print(f"  sizes=({s1},{s2}): {c} edges")


if __name__ == "__main__":
    main()

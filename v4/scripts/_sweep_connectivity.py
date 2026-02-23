"""Parameter sweep: find settings where box forest is naturally connected.

Tests combinations of ffb_min_edge, boundary_expand_epsilon, max_boxes
to find parameters that yield n_islands=1 (start-goal connected) without
needing RRT bridge.
"""
from __future__ import annotations
import json, sys, time, itertools
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
_SRC = _ROOT / "src"
for p in (_ROOT, _SRC):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import numpy as np
from experiments.runner import load_scene_from_config
from experiments.scenes import load_scenes
from planner.pipeline import (
    PandaGCSConfig, grow_and_prepare, _build_adjacency_and_islands,
    find_box_containing,
)
from forest.connectivity import UnionFind

scenes = load_scenes(["panda_10obs_moderate"])
scene_cfg = scenes[0]
robot, scene, query_pairs = load_scene_from_config(scene_cfg)
q_start, q_goal = query_pairs[0]

# ── Parameter grid (focused) ──
# Phase 1: quick scan with 2 seeds to identify promising combos
FFB_MIN_EDGES = [0.04, 0.08, 0.12, 0.15]
BND_EPSILONS  = [0.01, 0.05, 0.10]
MAX_BOXES_LIST = [2000, 3000]
SEEDS = [0, 2]  # 2 seeds for quick scan

print(f"{'ffb':>5} {'eps':>5} {'mbox':>5} | "
      f"{'seed':>4} {'n_box':>5} {'n_isl':>5} {'conn':>4} "
      f"{'grow_ms':>8} {'coarsen':>8}")
print("-" * 80)

results = []

for ffb, eps, mbox in itertools.product(FFB_MIN_EDGES, BND_EPSILONS, MAX_BOXES_LIST):
    for seed in SEEDS:
        cfg = PandaGCSConfig()
        cfg.ffb_min_edge = ffb
        cfg.boundary_expand_epsilon = eps
        cfg.max_boxes = mbox
        cfg.seed = seed
        cfg.max_consecutive_miss = 20

        try:
            t0 = time.perf_counter()
            prep = grow_and_prepare(robot, scene, cfg, q_start, q_goal, 7,
                                    no_cache=True)
            total_ms = (time.perf_counter() - t0) * 1000

            boxes = prep['boxes']
            # Quick connectivity check using simple overlap (same as solver)
            period = None  # Panda is non-periodic
            ids = list(boxes.keys())
            uf = UnionFind(ids)
            eps_adj = 1e-9
            box_list = list(boxes.values())
            n = len(box_list)
            for i in range(n):
                iv_i = box_list[i].joint_intervals
                for j in range(i + 1, n):
                    iv_j = box_list[j].joint_intervals
                    ok = True
                    for d in range(len(iv_i)):
                        lo1, hi1 = iv_i[d]
                        lo2, hi2 = iv_j[d]
                        if not (hi1 >= lo2 - eps_adj and hi2 >= lo1 - eps_adj):
                            ok = False
                            break
                    if ok:
                        uf.union(box_list[i].node_id, box_list[j].node_id)

            src = find_box_containing(q_start, boxes)
            tgt = find_box_containing(q_goal, boxes)
            islands = uf.components()
            n_islands = len(islands)
            connected = (src is not None and tgt is not None
                         and uf.same(src, tgt))

            grow_ms = prep['grow_ms']
            coarsen_ms = prep['coarsen_ms']

            tag = "OK" if connected else "FAIL"
            print(f"{ffb:5.2f} {eps:5.2f} {mbox:5d} | "
                  f"{seed:4d} {n:5d} {n_islands:5d} {tag:>4s} "
                  f"{grow_ms:8.0f} {coarsen_ms:8.0f}")

            results.append({
                'ffb_min_edge': ffb, 'boundary_expand_epsilon': eps,
                'max_boxes': mbox, 'seed': seed,
                'n_boxes_after_coarsen': n, 'n_islands': n_islands,
                'connected': connected,
                'grow_ms': grow_ms, 'coarsen_ms': coarsen_ms,
                'total_ms': total_ms,
            })
        except Exception as e:
            print(f"{ffb:5.2f} {eps:5.2f} {mbox:5d} | "
                  f"{seed:4d}  ERROR: {e}")
            results.append({
                'ffb_min_edge': ffb, 'boundary_expand_epsilon': eps,
                'max_boxes': mbox, 'seed': seed,
                'error': str(e),
            })

# ── Summary ──
print("\n" + "=" * 80)
print("SUMMARY: configs where ALL 5 seeds connected")
print("=" * 80)
from collections import defaultdict
by_cfg = defaultdict(list)
for r in results:
    key = (r['ffb_min_edge'], r['boundary_expand_epsilon'], r['max_boxes'])
    by_cfg[key].append(r)

for key in sorted(by_cfg.keys()):
    runs = by_cfg[key]
    n_ok = sum(1 for r in runs if r.get('connected', False))
    n_total = len(runs)
    avg_grow = np.mean([r.get('grow_ms', 0) for r in runs])
    avg_coarsen = np.mean([r.get('coarsen_ms', 0) for r in runs])
    avg_boxes = np.mean([r.get('n_boxes_after_coarsen', 0) for r in runs])
    avg_islands = np.mean([r.get('n_islands', 0) for r in runs])
    ffb, eps, mbox = key
    tag = "ALL_OK" if n_ok == n_total else f"{n_ok}/{n_total}"
    print(f"  ffb={ffb:.2f} eps={eps:.2f} mbox={mbox:5d} | "
          f"{tag:>6s}  boxes={avg_boxes:.0f}  islands={avg_islands:.1f}  "
          f"grow={avg_grow:.0f}ms  coarsen={avg_coarsen:.0f}ms")

# Save raw results
out_path = _ROOT / "scripts" / "_sweep_results.json"
with open(out_path, 'w') as f:
    json.dump(results, f, indent=2, default=str)
print(f"\nRaw results saved to {out_path}")

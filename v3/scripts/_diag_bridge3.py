"""Diagnostic: see which RRT pair succeeds per seed & per-attempt time."""
from __future__ import annotations
import logging, json, time, sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
_SRC = _ROOT / "src"
for p in (_ROOT, _SRC):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

logging.basicConfig(level=logging.INFO, format='%(message)s')

import numpy as np
from experiments.runner import load_scene_from_config
from experiments.scenes import load_scenes

scenes = load_scenes(["panda_10obs_moderate"])
scene_cfg = scenes[0]
robot, scene, query_pairs = load_scene_from_config(scene_cfg)

q_start, q_goal = query_pairs[0]

from baselines.sbf_adapter import SBFAdapter

for seed in range(5):
    print(f"\n{'='*60}")
    print(f"SEED {seed}")
    print(f"{'='*60}")
    adapter = SBFAdapter(method="dijkstra")
    cfg = {"max_boxes": 2000, "ffb_min_edge": 0.04, "seed": seed}
    adapter.setup(robot, scene, cfg)
    result = adapter.plan(q_start, q_goal, timeout=30.0)
    pt = result.phase_times or {}
    md = result.metadata or {}
    print(f"  success={result.success}  cost={result.cost:.4f}")
    print(f"  grow={pt.get('grow',0)*1000:.0f}ms  solve={pt.get('solve',0)*1000:.0f}ms")
    print(f"  adj={md.get('adj_ms',0):.0f}  bridge={md.get('bridge_ms',0):.0f}  "
          f"plan={md.get('plan_ms',0):.0f}  post_safe={md.get('post_safe_ms',0):.0f}")
    print(f"  total={result.planning_time*1000:.0f}ms")

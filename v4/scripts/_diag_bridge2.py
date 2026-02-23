"""Diagnostic: see which RRT pair succeeds per seed."""
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
from baselines.sbf_adapter import SBFAdapter
from experiments.runner import load_scene_from_config

# Load scene
scene_cfg = json.loads((_ROOT / "experiments" / "configs" / "scenes" / "panda_10obs_moderate.json").read_text())
robot, scene = load_scene_from_config(scene_cfg)

q_start = np.array(scene_cfg["query"]["start"])
q_goal = np.array(scene_cfg["query"]["goal"])

sbf_config = {
    "max_boxes": 2000,
    "ffb_min_edge": 0.04,
}

for seed in range(5):
    print(f"\n{'='*60}")
    print(f"=== SEED {seed} ===")
    print(f"{'='*60}")
    adapter = SBFAdapter(method="dijkstra")
    sbf_config["seed"] = seed
    adapter.setup(robot, scene, sbf_config)
    result = adapter.plan(q_start, q_goal, timeout=30.0)
    pt = result.phase_times or {}
    md = result.metadata or {}
    print(f"  success={result.success}")
    print(f"  grow_ms={pt.get('grow',0)*1000:.0f}")
    print(f"  solve_ms={pt.get('solve',0)*1000:.0f}")
    print(f"  adj_ms={md.get('adj_ms',0):.0f}")
    print(f"  bridge_ms={md.get('bridge_ms',0):.0f}")
    print(f"  plan_ms={md.get('plan_ms',0):.0f}")
    print(f"  post_safe_ms={md.get('post_safe_ms',0):.0f}")
    print(f"  total={result.planning_time*1000:.0f}ms  cost={result.cost:.4f}")

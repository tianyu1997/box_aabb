"""Diagnostic: see which RRT pair succeeds per seed."""
import logging, json, numpy as np, time, sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
_SRC = _ROOT / "src"
for p in (_ROOT, _SRC):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

logging.basicConfig(level=logging.INFO, format='%(message)s')

from baselines.sbf_adapter import solve_sbf

scene = json.load(open('../ompl_test_input.json'))
q_s = np.array(scene['start_config'])
q_g = np.array(scene['goal_config'])
obs = scene['obstacles']

for seed in range(5):
    print(f'=== SEED {seed} ===')
    r = solve_sbf(q_s, q_g, obs, seed=seed,
                  config_overrides={'max_boxes': 2000, 'ffb_min_edge': 0.04})
    pt = r.get('phase_times', {})
    sd = r.get('solve_details', {})
    bms = sd.get('bridge_ms', 0)
    rrt_used = r.get('rrt_bridge_used', False)
    print(f'  bridge_ms={bms:.0f}  rrt_bridge={rrt_used}')
    print()

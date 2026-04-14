"""Quick overlap checker for latest growth_viz results."""
import json, os, sys

results_dir = os.path.join(os.path.dirname(__file__), '..', 'results')
d = max((f for f in os.listdir(results_dir) if f.startswith('growth_viz_')), key=str)

summary_path = os.path.join(results_dir, d, 'summary.json')
boxes_path = os.path.join(results_dir, d, 'boxes.json')

with open(summary_path) as f:
    s = json.load(f)
with open(boxes_path) as f:
    boxes = json.load(f)

print(f'Latest result: {d}')
print(f'Coarsened boxes: {len(boxes)}, Success: {s.get("success")}')

# Also try to get raw_boxes from planner if available
try:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))
    import _sbf5_cpp as sbf
    # Can't easily reconstruct raw_boxes from file; just check coarsened
except ImportError:
    pass

def check_overlaps(blist, label):
    pairs = []
    for i in range(len(blist)):
        for j in range(i+1, len(blist)):
            a, b = blist[i], blist[j]
            ai, bi = a['joint_intervals'], b['joint_intervals']
            overlap = True
            nd = len(ai)
            for dd in range(nd):
                if ai[dd]['lo'] >= bi[dd]['hi'] or ai[dd]['hi'] <= bi[dd]['lo']:
                    overlap = False
                    break
            if overlap:
                va, vb = a['volume'], b['volume']
                pairs.append((i, j, va/vb if vb > 0 else float('inf')))
    print(f'{label}: {len(pairs)} overlap pairs')
    for i, j, r in pairs[:8]:
        print(f'  pair ({i},{j}): vol_ratio={r:.3f}')
    return pairs

cp = check_overlaps(boxes, 'Coarsened')

# Also check raw boxes by re-running the planner
try:
    import numpy as np
    import _sbf5_cpp as sbf

    v5_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    robot = sbf.Robot.from_json(os.path.join(v5_root, 'data', '2dof_planar.json'))

    rng = np.random.RandomState(123)
    lo, hi = -2.0, 2.0
    obs_list = []
    for _ in range(4):
        cx, cy = rng.uniform(lo+0.3, hi-0.3, size=2)
        sx, sy = rng.uniform(0.15, 0.6, size=2)
        obs_list.append(sbf.Obstacle(cx-sx/2, cy-sy/2, -0.01, cx+sx/2, cy+sy/2, 0.01))

    cfg = sbf.SBFPlannerConfig()
    cfg.grower.max_boxes = 200
    cfg.grower.rng_seed = 7
    p = sbf.SBFPlanner(robot, cfg)
    start = np.array([-2.5, -2.5])
    goal = np.array([2.5, 2.5])
    p.plan(start, goal, obs_list)

    raw = list(p.raw_boxes())
    coarsened = list(p.boxes())
    print(f'\nDirect planner check: Raw={len(raw)}, Coarsened={len(coarsened)}')

    raw_pairs = 0
    for i in range(len(raw)):
        for j in range(i+1, len(raw)):
            a, b = raw[i], raw[j]
            ai, bi = a.joint_intervals, b.joint_intervals
            overlap = all(ai[dd].lo < bi[dd].hi and bi[dd].lo < ai[dd].hi for dd in range(a.n_dims()))
            if overlap:
                raw_pairs += 1
    print(f'Raw overlap pairs: {raw_pairs}')
except Exception as e:
    import traceback; traceback.print_exc()
    print(f'Could not check raw boxes: {e}')

"""Analyze BEST_TIGHTEN split dimension choices at each LECT depth.

Absolute depth = s_q1 + s_q2  (total binary splits from root).
Each depth adds exactly ONE split.  At depth d, a box can come from
a depth-(d-1) parent by splitting either q1 or q2.
"""
import sys, math
from collections import defaultdict

sys.path.insert(0, "build_x64/Release")
sys.path.insert(0, "python")
import _sbf5_cpp as cpp
import numpy as np

# ── build obstacles identical to growth_viz defaults ──
def build_random_obstacles(n_obs=4, seed=123):
    rng = np.random.default_rng(seed)
    ws_range = 2.0 * 0.85
    obs = []
    attempts = 0
    while len(obs) < n_obs:
        attempts += 1
        if attempts > 2000:
            raise RuntimeError("cannot place obstacles")
        cx = float(rng.uniform(-ws_range, ws_range))
        cy = float(rng.uniform(-ws_range, ws_range))
        hw = float(rng.uniform(0.18, 0.42) / 2)
        hh = float(rng.uniform(0.18, 0.42) / 2)
        if math.hypot(cx, cy) < 0.45 + max(hw, hh):
            continue
        if math.hypot(cx, cy) > ws_range:
            continue
        obs.append(cpp.Obstacle(cx - hw, cy - hh, -0.5, cx + hw, cy + hh, 0.5))
    return obs

robot = cpp.Robot.from_json("data/2dof_planar.json")
obstacles = build_random_obstacles()

config = cpp.SBFPlannerConfig()
config.grower.max_boxes = 300
config.grower.rng_seed = 7
planner = cpp.SBFPlanner(robot, config)

q_s = np.array([0.5, 0.5])
q_g = np.array([2.5, -1.0])

result = planner.plan(q_s, q_g, obstacles, 30000.0)
raw = list(planner.raw_boxes())
print(f"raw_boxes={len(raw)}, coarsened={len(list(planner.boxes()))}", flush=True)

full_w = 2 * math.pi  # each joint range width

# ── Collect per-box data ──
# s0, s1 = number of times q1, q2 were halved from root
# absolute LECT depth = s0 + s1
rows = []
for b in raw:
    ivs = b.joint_intervals
    w0 = ivs[0].width()
    w1 = ivs[1].width()
    s0 = round(math.log2(full_w / w0)) if w0 > 1e-12 else 99
    s1 = round(math.log2(full_w / w1)) if w1 > 1e-12 else 99
    depth = s0 + s1  # absolute LECT depth
    rows.append((depth, s0, s1, w0, w1))

# ── Group by absolute depth ──
by_depth = defaultdict(list)
for r in rows:
    by_depth[r[0]].append(r)

print(f"\n{'depth':>5} {'count':>6} {'s_q1':>6} {'s_q2':>6}  {'w_q1':>10} {'w_q2':>10}   split_patterns")
print("-" * 85)
for d in sorted(by_depth.keys()):
    items = by_depth[d]
    n = len(items)
    avg_s0 = sum(r[1] for r in items) / n
    avg_s1 = sum(r[2] for r in items) / n
    avg_w0 = sum(r[3] for r in items) / n
    avg_w1 = sum(r[4] for r in items) / n
    # Unique (s0,s1) patterns
    patterns = sorted(set((r[1], r[2]) for r in items))
    pat_str = ", ".join(
        f"({s0},{s1})x{sum(1 for r in items if r[1]==s0 and r[2]==s1)}"
        for s0, s1 in patterns
    )
    print(f"{d:>5} {n:>6} {avg_s0:>6.1f} {avg_s1:>6.1f}  {avg_w0:>10.4f} {avg_w1:>10.4f}   {pat_str}")

# ── Per-depth: which dimension was split to get here? ──
# At depth d, a box with pattern (s0,s1) came from parent (s0-1,s1) via q1-split
# or from parent (s0,s1-1) via q2-split.
print(f"\n--- Split dimension chosen at each depth transition ---")
print(f"{'depth':>5} {'split_q1':>10} {'split_q2':>10} {'q1_pct':>8}")
print("-" * 40)
for d in sorted(by_depth.keys()):
    if d == 0:
        continue  # root, no parent
    items = by_depth[d]
    n_q1 = 0
    n_q2 = 0
    for _, s0, s1, _, _ in items:
        # This box has (s0, s1).  Its parent had depth d-1.
        # If parent was (s0-1, s1) → q1 was split
        # If parent was (s0, s1-1) → q2 was split
        # We can determine: the last split added to whichever dim
        # is "one more" than the balanced case.
        # Actually, we know: parent had (s0-1,s1) or (s0,s1-1).
        # Both are valid depth-(d-1) patterns. But since we know
        # (s0-1)+s1 = d-1 and s0+(s1-1) = d-1 are both true,
        # we need to look at which parent pattern actually exists.
        #
        # Simpler: compare s0 vs s1 relative to d.
        # If the split at this depth was q1: s0 = prev_s0+1, s1 = prev_s1
        # If the split at this depth was q2: s0 = prev_s0,   s1 = prev_s1+1
        #
        # We can check: does (s0-1, s1) exist at depth d-1?
        parent_patterns = set((r[1], r[2]) for r in by_depth.get(d - 1, []))
        if (s0 - 1, s1) in parent_patterns and (s0, s1 - 1) in parent_patterns:
            # Ambiguous — both parents exist. Count 0.5 each.
            n_q1 += 0.5
            n_q2 += 0.5
        elif (s0 - 1, s1) in parent_patterns:
            n_q1 += 1
        elif (s0, s1 - 1) in parent_patterns:
            n_q2 += 1
        else:
            # Neither parent found (box at a deeper level with no sibling at d-1)
            # Heuristic: if s0 > s1 → q1 was likely split more
            if s0 > s1:
                n_q1 += 1
            else:
                n_q2 += 1
    total = n_q1 + n_q2
    pct = n_q1 / total * 100 if total > 0 else 0
    print(f"{d:>5} {n_q1:>10.1f} {n_q2:>10.1f} {pct:>7.1f}%")

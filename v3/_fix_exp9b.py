"""Revert 2DOF scene to standard config."""
f = 'experiments/exp9_ffb_min_edge_sweep.py'
t = open(f, encoding='utf-8').read()

old = '''        "2dof_with_obstacle": {
            "name": "2dof_with_obstacle",
            "robot": "2dof_planar",
            "obstacles": [
                {"min": [-0.15, -0.25, -0.5], "max": [0.15, 0.25, 0.5],
                 "name": "center_wall"},
            ],
            "query_pairs": [
                {"start": [-2.5, 1.0], "goal": [2.5, -1.0]},
            ],
        },'''

new = '''        "2dof_with_obstacle": {
            "name": "2dof_with_obstacle",
            "robot": "2dof_planar",
            "obstacles": [
                {"min": [-0.02, -0.02, -0.5], "max": [0.02, 0.02, 0.5],
                 "name": "center_pin"},
            ],
            "query_pairs": [
                {"start": [-1.0, 0.5], "goal": [1.0, -0.5]},
            ],
        },'''

assert old in t, 'Pattern not found'
t = t.replace(old, new, 1)
open(f, 'w', encoding='utf-8').write(t)
print('OK')

#!/usr/bin/env python3
"""Quick test: verify SBF config matches C++ ablation behavior."""
import sys, os, time, numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'build', 'python'))
import _sbf6_cpp as sbf6

def make_combined_obstacles():
    def make_shelves():
        ox, oy, oz = 0.85, 0.0, 0.4
        obs = []
        def add(lx, ly, lz, fx, fy, fz):
            obs.append(sbf6.Obstacle(ox+lx-fx/2, oy+ly-fy/2, oz+lz-fz/2,
                                      ox+lx+fx/2, oy+ly+fy/2, oz+lz+fz/2))
        add(0, 0.292, 0, 0.3, 0.016, 0.783); add(0, -0.292, 0, 0.3, 0.016, 0.783)
        add(0, 0, 0.3995, 0.3, 0.6, 0.016); add(0, 0, -0.13115, 0.3, 0.6, 0.016)
        add(0, 0, 0.13115, 0.3, 0.6, 0.016)
        return obs
    def make_bins():
        obs = []
        def add_bin(bx, by, bz):
            def add(lx, ly, lz, fx, fy, fz):
                obs.append(sbf6.Obstacle(bx-ly-fy/2, by+lx-fx/2, bz+lz-fz/2,
                                          bx-ly+fy/2, by+lx+fx/2, bz+lz+fz/2))
            add(0.22, 0, 0.105, 0.05, 0.63, 0.21); add(-0.22, 0, 0.105, 0.05, 0.63, 0.21)
            add(0, 0.29, 0.105, 0.49, 0.05, 0.21); add(0, -0.29, 0.105, 0.49, 0.05, 0.21)
            add(0, 0, 0.0075, 0.49, 0.63, 0.015)
        add_bin(0, -0.6, 0); add_bin(0, 0.6, 0)
        return obs
    def make_table():
        return [sbf6.Obstacle(0.4-2.5/2, -2.5/2, -0.25-0.2/2, 0.4+2.5/2, 2.5/2, -0.25+0.2/2)]
    return make_shelves() + make_bins() + make_table()

IIWA_CONFIGS = {
    'AS': np.array([6.42e-05, 0.4719533, -0.0001493, -0.6716735, 0.0001854, 0.4261696, 1.5706922]),
    'TS': np.array([-1.55e-04, 0.3972726, 0.0002196, -1.3674756, 0.0002472, -0.1929518, 1.5704688]),
    'CS': np.array([-1.76e-04, 0.6830279, 0.0002450, -1.6478229, 2.09e-05, -0.7590545, 1.5706263]),
    'LB': np.array([1.3326656, 0.7865932, 0.3623384, -1.4916529, -0.3192509, 0.9217325, 1.7911904]),
    'RB': np.array([-1.3324624, 0.7866478, -0.3626562, -1.4916528, 0.3195340, 0.9217833, 1.3502090]),
}

robot = sbf6.Robot.from_json(os.path.join(os.path.dirname(__file__), '..', 'data', 'iiwa14.json'))
obstacles = make_combined_obstacles()
print(f'Obstacles: {len(obstacles)}')

config = sbf6.SBFPlannerConfig()
config.grower.timeout_ms = 60000
config.grower.max_boxes = 200000
config.grower.post_connect_extra_boxes = 4000
config.grower.rng_seed = 42
config.grower.n_threads = 5
config.grower.bridge_n_threads = 16
config.grower.max_consecutive_miss = 2000
config.grower.ffb_config.max_depth = 300
config.coarsen.target_boxes = 300
config.coarsen.score_threshold = 500
config.lect_no_cache = True

planner = sbf6.SBFPlanner(robot, config)
t0 = time.perf_counter()
planner.build_coverage(obstacles, 60000, [IIWA_CONFIGS['AS'], IIWA_CONFIGS['TS']])
dt = time.perf_counter() - t0
print(f'Build: {dt:.3f}s, boxes={planner.n_boxes()}')

for pair in [('AS','TS'), ('TS','CS'), ('CS','LB'), ('LB','RB'), ('RB','AS')]:
    r = planner.query(IIWA_CONFIGS[pair[0]], IIWA_CONFIGS[pair[1]])
    print(f'Query {pair[0]}->{pair[1]}: success={r.success}, '
          f'len={r.path_length:.3f}, time={r.planning_time_ms:.1f}ms')

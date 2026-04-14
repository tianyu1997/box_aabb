#!/usr/bin/env python3
"""Quick test of sbf5 Python bindings with combined scene."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'build', 'python'))

import _sbf5_cpp as sbf5
import numpy as np
import time

robot = sbf5.Robot.from_json(
    os.path.join(os.path.dirname(__file__), '..', 'data', 'iiwa14.json'))
print(f'Robot: {robot.n_joints()} DOF')

# Replicate combined obstacles from marcucci_scenes.h
def make_shelves():
    ox, oy, oz = 0.85, 0.0, 0.4
    obs = []
    def add(lx,ly,lz,fx,fy,fz):
        obs.append(sbf5.Obstacle(
            ox+lx-fx/2, oy+ly-fy/2, oz+lz-fz/2,
            ox+lx+fx/2, oy+ly+fy/2, oz+lz+fz/2))
    add(0, 0.292, 0, 0.3, 0.016, 0.783)
    add(0,-0.292, 0, 0.3, 0.016, 0.783)
    add(0, 0, 0.3995, 0.3, 0.6, 0.016)
    add(0, 0,-0.13115, 0.3, 0.6, 0.016)
    add(0, 0, 0.13115, 0.3, 0.6, 0.016)
    return obs

def make_bins():
    obs = []
    def add_bin(bx,by,bz):
        def add(lx,ly,lz,fx,fy,fz):
            obs.append(sbf5.Obstacle(
                bx-ly-fy/2, by+lx-fx/2, bz+lz-fz/2,
                bx-ly+fy/2, by+lx+fx/2, bz+lz+fz/2))
        add(0.22,0,0.105, 0.05,0.63,0.21)
        add(-0.22,0,0.105, 0.05,0.63,0.21)
        add(0,0.29,0.105, 0.49,0.05,0.21)
        add(0,-0.29,0.105, 0.49,0.05,0.21)
        add(0,0,0.0075, 0.49,0.63,0.015)
    add_bin(0,-0.6,0)
    add_bin(0,0.6,0)
    return obs

def make_table():
    return [sbf5.Obstacle(0.4-2.5/2, 0-2.5/2, -0.25-0.2/2,
                           0.4+2.5/2, 0+2.5/2, -0.25+0.2/2)]

obstacles = make_shelves() + make_bins() + make_table()
print(f'Obstacles: {len(obstacles)}')

# Configs
CONFIGS = {
    'AS': np.array([6.42e-05, 0.4719533, -0.0001493, -0.6716735,
                    0.0001854, 0.4261696, 1.5706922]),
    'TS': np.array([-1.55e-04, 0.3972726, 0.0002196, -1.3674756,
                    0.0002472, -0.1929518, 1.5704688]),
}

config = sbf5.SBFPlannerConfig()
config.grower.timeout_ms = 10000
planner = sbf5.SBFPlanner(robot, config)

print('Planning AS->TS...')
t0 = time.perf_counter()
result = planner.plan(CONFIGS['AS'], CONFIGS['TS'], obstacles, 15000)
dt = time.perf_counter() - t0
print(f'Success: {result.success} ({dt:.1f}s)')
if result.success:
    print(f'Path length: {result.path_length:.4f}')
    print(f'Box sequence: {len(result.box_sequence)} boxes')
    print(f'N boxes: {result.n_boxes}')
    boxes = planner.boxes()
    print(f'Total boxes: {len(boxes)}')
    if result.box_sequence:
        box_map = {b.id: b for b in boxes}
        for bid in result.box_sequence[:3]:
            b = box_map[bid]
            ivs = b.joint_intervals
            print(f'  Box {bid}: [{ivs[0].lo:.3f},{ivs[0].hi:.3f}]x...x[{ivs[6].lo:.3f},{ivs[6].hi:.3f}]')

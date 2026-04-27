#!/usr/bin/env python3
"""Benchmark SBF path quality across seeds. Each seed runs in subprocess."""
import sys, os, json, subprocess
import numpy as np

CONFIGS = {
    'AS': [6.42e-05, 0.4719533, -0.0001493, -0.6716735, 0.0001854, 0.4261696, 1.5706922],
    'TS': [-1.55e-04, 0.3972726, 0.0002196, -1.3674756, 0.0002472, -0.1929518, 1.5704688],
    'LB': [1.3326656, 0.7865932, 0.3623384, -1.4916529, -0.3192509, 0.9217325, 1.7911904],
    'RB': [-1.3324624, 0.7866478, -0.3626562, -1.4916528, 0.3195340, 0.9217833, 1.3502090],
}
PAIRS = [("AS_TS", "AS", "TS"), ("LB_RB", "LB", "RB")]


def run_one(s_name, g_name, seed):
    """Run in subprocess. Returns JSON result or None."""
    code = f'''
import sys, os, gc
sys.path.insert(0, "../build/python")
import _sbf5_cpp as sbf5
import numpy as np
import json

robot = sbf5.Robot.from_json("../data/iiwa14.json")
obs = []
ox, oy, oz = 0.85, 0.0, 0.4
def add(lx,ly,lz,fx,fy,fz):
    obs.append(sbf5.Obstacle(ox+lx-fx/2,oy+ly-fy/2,oz+lz-fz/2,ox+lx+fx/2,oy+ly+fy/2,oz+lz+fz/2))
add(0,0.292,0,0.3,0.016,0.783)
add(0,-0.292,0,0.3,0.016,0.783)
add(0,0,0.3995,0.3,0.6,0.016)
add(0,0,-0.13115,0.3,0.6,0.016)
add(0,0,0.13115,0.3,0.6,0.016)
def add_bin(bx,by,bz):
    def ab(lx,ly,lz,fx,fy,fz):
        obs.append(sbf5.Obstacle(bx-ly-fy/2,by+lx-fx/2,bz+lz-fz/2,bx-ly+fy/2,by+lx+fx/2,bz+lz+fz/2))
    ab(0.22,0,0.105,0.05,0.63,0.21)
    ab(-0.22,0,0.105,0.05,0.63,0.21)
    ab(0,0.29,0.105,0.49,0.05,0.21)
    ab(0,-0.29,0.105,0.49,0.05,0.21)
    ab(0,0,0.0075,0.49,0.63,0.015)
add_bin(0,-0.6,0)
add_bin(0,0.6,0)
obs.append(sbf5.Obstacle(0.4-2.5/2,-2.5/2,-0.25-0.2/2, 0.4+2.5/2,2.5/2,-0.25+0.2/2))

q_s = np.array({CONFIGS[s_name]})
q_g = np.array({CONFIGS[g_name]})
config = sbf5.SBFPlannerConfig()
config.grower.timeout_ms = 10000
config.grower.rng_seed = {seed}
planner = sbf5.SBFPlanner(robot, config)
result = planner.plan(q_s, q_g, obs, 15000)
r = {{"success": result.success, "path_len": round(result.path_length, 4) if result.success else 0,
      "n_boxes": result.n_boxes, "box_seq": len(result.box_sequence) if result.success else 0,
      "time_ms": round(result.planning_time_ms, 1)}}
sys.stdout.write("R:" + json.dumps(r) + "\\n")
sys.stdout.flush()
del result, planner, robot, obs
gc.collect()
os._exit(0)
'''
    proc = subprocess.run(
        [sys.executable, '-c', code],
        capture_output=True, text=True, timeout=60,
        cwd=os.path.dirname(os.path.abspath(__file__)))
    for line in proc.stdout.splitlines():
        if line.startswith("R:"):
            return json.loads(line[2:])
    return None


if __name__ == "__main__":
    n_seeds = int(sys.argv[1]) if len(sys.argv) > 1 else 10

    # Clear cache before benchmark
    cache_file = os.path.expanduser("~/.sbf_cache/kuka_iiwa14_r820_dfdef96ecce47a7e.lect")
    if os.path.exists(cache_file):
        os.remove(cache_file)

    for label, s, g in PAIRS:
        lengths = []
        times = []
        print(f"\n{s}->{g} ({n_seeds} seeds):")
        for seed in range(n_seeds):
            r = run_one(s, g, seed * 42 + 1)
            if r and r["success"]:
                lengths.append(r["path_len"])
                times.append(r["time_ms"])
                print(f"  seed {seed}: len={r['path_len']:.4f} boxes={r['n_boxes']} "
                      f"seq={r['box_seq']} time={r['time_ms']:.0f}ms")
            else:
                print(f"  seed {seed}: FAILED/CRASH")

        if lengths:
            arr = np.array(lengths)
            print(f"  Summary: {len(lengths)}/{n_seeds} success")
            print(f"    median={np.median(arr):.4f} mean={np.mean(arr):.4f} "
                  f"std={np.std(arr):.4f}")
            print(f"    min={arr.min():.4f} max={arr.max():.4f}")
            print(f"    avg_time={np.mean(times):.0f}ms")

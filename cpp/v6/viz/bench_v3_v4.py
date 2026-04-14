"""
Benchmark: V3 (single-channel) vs V4 (dual-channel) LECT performance.

Measures:
  1. Cache IO: load time, save time, file sizes
  2. Planning: full plan() on standard scenes (iiwa14, panda)
  3. Warmup: LECT tree expansion speed

Usage:
    cd v5
    set PYTHONPATH=python;build_x64\Release
    python viz/bench_v3_v4.py
"""
import copy
import json
import os
import shutil
import statistics
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "python"))

import sbf5
from sbf5._sbf5_cpp import (
    Robot, SBFPlanner, SBFPlannerConfig, Obstacle, Interval,
    EndpointSource, EndpointSourceConfig, EnvelopeType, EnvelopeTypeConfig,
    SplitOrder, compute_envelope_info,
)

CACHE_DIR = os.path.expanduser("~/.sbf_cache")
DATA_DIR = os.path.join(os.path.dirname(__file__), "..", "data")

# ── Scenes ──────────────────────────────────────────────────────────────────

SCENES = {
    "panda_tabletop": {
        "robot": "panda.json",
        "obstacles": [
            {"center": [0.4, 0.0, 0.4], "half": [0.15, 0.15, 0.02]},
        ],
        "start": [0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0],
        "goal":  [0.5, -0.3, 0.0, -1.5, 0.0, 1.2, 0.0],
    },
    "panda_shelf": {
        "robot": "panda.json",
        "obstacles": [
            {"center": [0.35, 0.0, 0.55], "half": [0.25, 0.30, 0.02]},
        ],
        "start": [0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0],
        "goal":  [0.8, 0.5, 0.0, -1.0, 0.0, 1.5, 0.5],
    },
    "panda_multi": {
        "robot": "panda.json",
        "obstacles": [
            {"center": [0.4, 0.0, 0.4], "half": [0.15, 0.15, 0.02]},
            {"center": [0.3, 0.2, 0.35], "half": [0.08, 0.08, 0.08]},
        ],
        "start": [0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0],
        "goal":  [1.0, -0.5, 0.0, -2.0, 0.0, 1.8, 0.0],
    },
}


def make_obstacles(obs_defs):
    out = []
    for o in obs_defs:
        c = np.array(o["center"])
        h = np.array(o["half"])
        out.append(Obstacle(*(c - h).tolist(), *(c + h).tolist()))
    return out


def fmt_size(n):
    if n >= 1_048_576:
        return f"{n / 1_048_576:.1f} MB"
    return f"{n / 1024:.1f} KB"


def fmt_ms(ms):
    if ms < 1.0:
        return f"{ms * 1000:.0f} µs"
    if ms < 1000:
        return f"{ms:.1f} ms"
    return f"{ms / 1000:.2f} s"


# ════════════════════════════════════════════════════════════════════════════
#  1. Cache IO benchmark
# ════════════════════════════════════════════════════════════════════════════

def bench_cache_io():
    print("=" * 70)
    print("  1. CACHE IO BENCHMARK")
    print("=" * 70)

    files = {}
    for f in os.listdir(CACHE_DIR):
        if f.endswith(".lect"):
            files[f] = os.path.join(CACHE_DIR, f)

    if not files:
        print("  No .lect cache files found. Skipping.\n")
        return

    for name, path in sorted(files.items()):
        # Restore V3 backup if exists
        v3bak = path + ".v3bak"
        if os.path.exists(v3bak):
            shutil.copy2(v3bak, path)

        v3_size = os.path.getsize(path)
        robot_name = name.split("_")[0]
        # Match robot JSON
        rjson = None
        for rn in ["iiwa14.json", "panda.json", "2dof_planar.json"]:
            rp = os.path.join(DATA_DIR, rn)
            if os.path.exists(rp) and robot_name.lower() in rn.lower():
                rjson = rp
                break
        # Broader match
        if rjson is None:
            for rn in os.listdir(DATA_DIR):
                if rn.endswith(".json"):
                    rp = os.path.join(DATA_DIR, rn)
                    try:
                        r = Robot.from_json(rp)
                        h = f"{r.fingerprint():016x}"
                        if h in name:
                            rjson = rp
                            break
                    except Exception:
                        continue

        if rjson is None:
            print(f"  {name}: cannot find matching robot JSON, skipping")
            continue

        robot = Robot.from_json(rjson)
        print(f"\n  Robot: {robot.name()} ({robot.n_joints()} DOF)")
        print(f"  Cache: {name}")
        print(f"  V3 file size: {fmt_size(v3_size)}")

        def make_config():
            config = SBFPlannerConfig()
            config.lect_cache_dir = CACHE_DIR
            config.endpoint_source.source = EndpointSource.CritSample
            config.endpoint_source.n_samples_crit = 50
            config.envelope_type.type = EnvelopeType.LinkIAABB
            config.split_order = SplitOrder.BEST_TIGHTEN
            config.z4_enabled = True
            return config

        # ── V3 Load timing (first load converts to V4) ──
        # Restore V3 before each trial for fair measurement
        load_times_v3 = []
        for _ in range(5):
            shutil.copy2(v3bak, path)  # restore V3
            config = make_config()
            t0 = time.perf_counter()
            planner = SBFPlanner(robot, config)
            planner.warmup_lect(0, 0, 42)  # load only
            t1 = time.perf_counter()
            load_times_v3.append((t1 - t0) * 1000)

        v3_load_ms = statistics.median(load_times_v3)
        print(f"  V3 load time: {fmt_ms(v3_load_ms)} (median of 5)")

        # Now the file is V4 — measure its size
        v4_size = os.path.getsize(path)
        print(f"  V4 file size: {fmt_size(v4_size)} (after V3→V4 migration)")
        delta = v4_size - v3_size
        pct = (delta / v3_size) * 100 if v3_size > 0 else 0
        print(f"  Size change: {'+' if delta >= 0 else ''}{fmt_size(abs(delta))} ({pct:+.1f}%)")

        # ── V4 Load timing (file is now V4) ──
        load_times_v4 = []
        for _ in range(5):
            config2 = make_config()
            t0 = time.perf_counter()
            planner2 = SBFPlanner(robot, config2)
            planner2.warmup_lect(0, 0, 42)
            t1 = time.perf_counter()
            load_times_v4.append((t1 - t0) * 1000)

        v4_load_ms = statistics.median(load_times_v4)
        print(f"  V4 load time: {fmt_ms(v4_load_ms)} (median of 5)")
        speedup = v3_load_ms / v4_load_ms if v4_load_ms > 0 else float('inf')
        print(f"  V4/V3 load ratio: {speedup:.2f}x")
        print()


# ════════════════════════════════════════════════════════════════════════════
#  2. Warmup benchmark (tree expansion speed)
# ════════════════════════════════════════════════════════════════════════════

def bench_warmup():
    print("=" * 70)
    print("  2. WARMUP / TREE EXPANSION BENCHMARK")
    print("=" * 70)

    for rn in ["panda.json", "iiwa14.json"]:
        rp = os.path.join(DATA_DIR, rn)
        if not os.path.exists(rp):
            continue

        robot = Robot.from_json(rp)
        print(f"\n  Robot: {robot.name()} ({robot.n_joints()} DOF)")

        # Warmup from existing cache: expand 200 paths at depth 25
        config = SBFPlannerConfig()
        config.lect_cache_dir = CACHE_DIR
        config.endpoint_source.source = EndpointSource.CritSample
        config.endpoint_source.n_samples_crit = 50
        config.envelope_type.type = EnvelopeType.LinkIAABB
        config.split_order = SplitOrder.BEST_TIGHTEN
        config.z4_enabled = True

        planner = SBFPlanner(robot, config)
        planner.warmup_lect(0, 0, 42)  # just load
        print(f"  Loaded from cache.")

        t0 = time.perf_counter()
        new_nodes = planner.warmup_lect(25, 200, 99)
        t1 = time.perf_counter()
        expand_ms = (t1 - t0) * 1000
        print(f"  After 200-path warmup: +{new_nodes} new nodes")
        print(f"  Expansion time: {fmt_ms(expand_ms)}")
        if new_nodes > 0:
            print(f"  Per-node: {expand_ms / new_nodes * 1000:.1f} µs/node")
        print()


# ════════════════════════════════════════════════════════════════════════════
#  3. Planning benchmark (end-to-end)
# ════════════════════════════════════════════════════════════════════════════

def bench_planning():
    print("=" * 70)
    print("  3. PLANNING BENCHMARK")
    print("=" * 70)

    N_TRIALS = 3

    for scene_name, scene_def in sorted(SCENES.items()):
        rp = os.path.join(DATA_DIR, scene_def["robot"])
        if not os.path.exists(rp):
            print(f"\n  {scene_name}: robot {scene_def['robot']} not found, skip")
            continue

        robot = Robot.from_json(rp)
        obs = make_obstacles(scene_def["obstacles"])
        start = np.array(scene_def["start"])
        goal = np.array(scene_def["goal"])

        print(f"\n  Scene: {scene_name} ({robot.name()}, {len(obs)} obstacles)")
        print(f"  {'trial':>5}  {'success':>7}  {'plan_ms':>9}  {'build_ms':>9}  "
              f"{'lect_ms':>9}  {'boxes':>5}  {'path_len':>9}")
        print(f"  {'─'*5}  {'─'*7}  {'─'*9}  {'─'*9}  {'─'*9}  {'─'*5}  {'─'*9}")

        results = []
        for trial in range(N_TRIALS):
            config = SBFPlannerConfig()
            config.lect_cache_dir = CACHE_DIR
            config.endpoint_source.source = EndpointSource.CritSample
            config.endpoint_source.n_samples_crit = 50
            config.envelope_type.type = EnvelopeType.LinkIAABB
            config.split_order = SplitOrder.BEST_TIGHTEN
            config.z4_enabled = True
            config.grower.rng_seed = 42 + trial

            planner = SBFPlanner(robot, config)

            result = planner.plan(start, goal, obs, 10000.0)
            plan_ms = result.planning_time_ms
            build_ms = result.build_time_ms
            lect_ms = result.lect_time_ms
            n_boxes = result.n_boxes
            path_len = result.path_length if result.success else float('nan')

            print(f"  {trial+1:>5}  {'  YES' if result.success else '   NO':>7}  "
                  f"{plan_ms:>9.1f}  {build_ms:>9.1f}  {lect_ms:>9.1f}  "
                  f"{n_boxes:>5}  {path_len:>9.3f}")
            results.append({
                "success": result.success,
                "plan_ms": plan_ms,
                "build_ms": build_ms,
                "lect_ms": lect_ms,
                "n_boxes": n_boxes,
                "path_len": path_len,
            })

        ok = [r for r in results if r["success"]]
        if ok:
            med_plan = statistics.median([r["plan_ms"] for r in ok])
            med_build = statistics.median([r["build_ms"] for r in ok])
            med_lect = statistics.median([r["lect_ms"] for r in ok])
            med_boxes = statistics.median([r["n_boxes"] for r in ok])
            med_plen = statistics.median([r["path_len"] for r in ok])
            print(f"  {'MED':>5}  {len(ok):>5}/{N_TRIALS}  {med_plan:>9.1f}  "
                  f"{med_build:>9.1f}  {med_lect:>9.1f}  "
                  f"{int(med_boxes):>5}  {med_plen:>9.3f}")
        print()


# ════════════════════════════════════════════════════════════════════════════
#  4. Envelope compute (cold vs hot)
# ════════════════════════════════════════════════════════════════════════════

def bench_envelope():
    print("=" * 70)
    print("  4. ENVELOPE COMPUTE BENCHMARK")
    print("=" * 70)

    for rn in ["panda.json", "iiwa14.json"]:
        rp = os.path.join(DATA_DIR, rn)
        if not os.path.exists(rp):
            continue

        robot = Robot.from_json(rp)
        lims = robot.joint_limits()
        nd = robot.n_joints()

        print(f"\n  Robot: {robot.name()} ({nd} DOF)")

        # Wide intervals (root)
        wide_ivs = [Interval(lims.lo[d], lims.hi[d]) for d in range(nd)]
        # Narrow intervals (~depth 10)
        narrow_ivs = []
        for d in range(nd):
            mid = (lims.lo[d] + lims.hi[d]) / 2
            span = (lims.hi[d] - lims.lo[d]) / 1024  # ~ 2^-10
            narrow_ivs.append(Interval(mid - span/2, mid + span/2))

        for label, ivs in [("wide (root)", wide_ivs), ("narrow (d~10)", narrow_ivs)]:
            for src_name, src in [("IFK", EndpointSource.IFK),
                                  ("CritSample", EndpointSource.CritSample)]:
                ep_cfg = EndpointSourceConfig()
                ep_cfg.source = src
                ep_cfg.n_samples_crit = 50
                env_cfg = EnvelopeTypeConfig()
                env_cfg.type = EnvelopeType.LinkIAABB

                times = []
                for _ in range(20):
                    info = compute_envelope_info(robot, ivs, ep_cfg, env_cfg)
                    times.append(info["total_time_us"])

                med = statistics.median(times)
                print(f"  {label:>16} | {src_name:>12} | {med:>8.0f} µs (median of 20)")

        print()


# ════════════════════════════════════════════════════════════════════════════
def main():
    # Redirect stdout to file
    import io
    outpath = os.path.join(os.path.dirname(__file__), "..", "bench_out3.txt")
    tee = open(outpath, "w", encoding="utf-8")
    class Tee:
        def __init__(self, *streams):
            self.streams = streams
        def write(self, s):
            for st in self.streams:
                st.write(s)
                st.flush()
        def flush(self):
            for st in self.streams:
                st.flush()
    sys.stdout = Tee(sys.__stdout__, tee)

    print()
    print("=" * 60)
    print("  SBF v5 - V3 vs V4 Multi-Channel LECT Benchmark")
    print("=" * 60)
    print()

    bench_cache_io()
    # bench_planning()  # skip for quick IO benchmark

    print("=" * 70)
    print("  Done.")
    print("=" * 70)

    tee.close()
    sys.stdout = sys.__stdout__


if __name__ == "__main__":
    main()

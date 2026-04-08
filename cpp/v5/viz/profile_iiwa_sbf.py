"""
SBF grow + coarsen profiler for iiwa14 robot with random obstacles.

Captures:
  - Per-stage timing (LECT build, grow, coarsen-sweep, coarsen-greedy, filter,
    bridge, total build)
  - Raw box statistics (count, volume distribution, edge widths, overlap check)
  - Coarsened box statistics

Usage:
    cd v5
    set PYTHONPATH=build_x64\Release;python
    python viz/profile_iiwa_sbf.py [--n_obs 8] [--max_boxes 300] [--seed 42]
"""
import argparse
import io
import re
import sys
import time
from contextlib import redirect_stderr

import numpy as np


def parse_args():
    p = argparse.ArgumentParser(description="Profile SBF grow+coarsen on iiwa14")
    p.add_argument("--n_obs", type=int, default=8,
                   help="Number of random rectangular obstacles")
    p.add_argument("--max_boxes", type=int, default=300,
                   help="Target raw boxes from grower")
    p.add_argument("--timeout_ms", type=float, default=60000.0,
                   help="Grower timeout (ms)")
    p.add_argument("--seed", type=int, default=42,
                   help="RNG seed for obstacle + config generation")
    p.add_argument("--target_coarsen", type=int, default=0,
                   help="Target boxes after coarsening (0=auto)")
    p.add_argument("--max_depth", type=int, default=18,
                   help="FFB / LECT max split depth")
    p.add_argument("--n_threads", type=int, default=1,
                   help="Number of parallel grow threads (1=serial)")
    p.add_argument("--no_lect_cache", action="store_true",
                   help="Disable LECT persistent cache (default: cache ON)")
    p.add_argument("--ep_source", type=str, default="ifk",
                   choices=["ifk", "crit"],
                   help="Endpoint source: ifk or crit (default: ifk)")
    return p.parse_args()


def make_random_obstacles(robot, n_obs, rng):
    """Generate n_obs random AABB obstacles around the robot's workspace."""
    from sbf5 import Obstacle

    # Use FK at a few random configs to estimate workspace bounds
    n_joints = robot.n_joints()
    limits = robot.joint_limits().limits

    obstacles = []
    for _ in range(n_obs):
        # Random box near the robot workspace
        # iiwa14 reach is roughly ~0.8m, workspace ~ [-1, 1]^3
        cx = rng.uniform(-0.6, 0.6)
        cy = rng.uniform(-0.6, 0.6)
        cz = rng.uniform(0.0, 1.2)
        hx = rng.uniform(0.03, 0.15)
        hy = rng.uniform(0.03, 0.15)
        hz = rng.uniform(0.03, 0.15)
        obs = Obstacle(cx - hx, cy - hy, cz - hz,
                       cx + hx, cy + hy, cz + hz)
        obstacles.append(obs)

    return obstacles


def random_config(limits, rng):
    """Random joint config within limits."""
    q = np.zeros(len(limits))
    for j, iv in enumerate(limits):
        q[j] = rng.uniform(iv.lo + 0.1, iv.hi - 0.1)
    return q


def box_stats(boxes, label):
    """Compute and print statistics for a list of BoxNode."""
    n = len(boxes)
    if n == 0:
        print(f"\n{'='*60}")
        print(f"  {label}: 0 boxes")
        print(f"{'='*60}")
        return {}

    volumes = np.array([b.volume for b in boxes])
    n_dims = boxes[0].n_dims()

    # Per-dimension edge widths
    widths = np.zeros((n, n_dims))
    for i, b in enumerate(boxes):
        for d in range(n_dims):
            widths[i, d] = b.joint_intervals[d].hi - b.joint_intervals[d].lo

    # Geometric mean edge
    gmean_edges = np.exp(np.mean(np.log(np.clip(widths, 1e-20, None)), axis=1))

    # Total volume (sum)
    total_vol = volumes.sum()

    # Overlap check (pairwise)
    n_overlap = 0
    for i in range(n):
        for j in range(i + 1, n):
            overlap = True
            for d in range(n_dims):
                lo_i = boxes[i].joint_intervals[d].lo
                hi_i = boxes[i].joint_intervals[d].hi
                lo_j = boxes[j].joint_intervals[d].lo
                hi_j = boxes[j].joint_intervals[d].hi
                if hi_i <= lo_j + 1e-10 or hi_j <= lo_i + 1e-10:
                    overlap = False
                    break
            if overlap:
                n_overlap += 1

    # Per-dimension width stats
    dim_stats = []
    for d in range(n_dims):
        w = widths[:, d]
        dim_stats.append({
            "dim": d,
            "min": w.min(),
            "max": w.max(),
            "mean": w.mean(),
            "median": np.median(w),
        })

    print(f"\n{'='*60}")
    print(f"  {label}")
    print(f"{'='*60}")
    print(f"  n_boxes          = {n}")
    print(f"  overlap_pairs    = {n_overlap}")
    print(f"  total_volume     = {total_vol:.6g}")
    print(f"  volume  min/med/mean/max = {volumes.min():.6g} / "
          f"{np.median(volumes):.6g} / {volumes.mean():.6g} / {volumes.max():.6g}")
    print(f"  volume  std              = {volumes.std():.6g}")
    print(f"  gmean_edge min/med/max   = {gmean_edges.min():.6g} / "
          f"{np.median(gmean_edges):.6g} / {gmean_edges.max():.6g}")

    # Volume distribution buckets
    if n >= 5:
        pcts = np.percentile(volumes, [10, 25, 50, 75, 90])
        print(f"  volume percentiles [10,25,50,75,90] = "
              f"[{pcts[0]:.4g}, {pcts[1]:.4g}, {pcts[2]:.4g}, "
              f"{pcts[3]:.4g}, {pcts[4]:.4g}]")

    print(f"\n  Per-dimension edge widths:")
    print(f"  {'dim':>4s}  {'min':>10s}  {'median':>10s}  {'mean':>10s}  {'max':>10s}")
    for ds in dim_stats:
        print(f"  {ds['dim']:4d}  {ds['min']:10.6f}  {ds['median']:10.6f}  "
              f"{ds['mean']:10.6f}  {ds['max']:10.6f}")

    # tree_id stats (LECT node index — proxy for depth)
    tree_ids = np.array([b.tree_id for b in boxes])
    print(f"\n  LECT tree_id  min/max = {tree_ids.min()} / {tree_ids.max()}")
    print(f"  unique tree_ids = {len(np.unique(tree_ids))}")

    return {
        "n": n,
        "total_volume": total_vol,
        "volumes": volumes,
        "widths": widths,
        "gmean_edges": gmean_edges,
        "n_overlap": n_overlap,
    }


def parse_stderr_timing(stderr_text):
    """Parse [PLN] lines from C++ stderr output."""
    timing = {}

    m = re.search(r'\[PLN\] lect=(\d+\.?\d*)ms', stderr_text)
    if m:
        timing["lect_ms"] = float(m.group(1))

    m = re.search(r'\[PLN\] grow=(\d+\.?\d*)ms \(boxes=(\d+)\)', stderr_text)
    if m:
        timing["grow_ms"] = float(m.group(1))

    m = re.search(r'\[PLN\] sg_connect=(\d+\.?\d*)ms added=(\d+) total=(\d+)', stderr_text)
    if m:
        timing["sg_connect_ms"] = float(m.group(1))
        timing["sg_connect_added"] = int(m.group(2))
        timing["sg_connect_total"] = int(m.group(3))

    m = re.search(r'\[PLN\] raw: boxes=(\d+) islands=(\d+) edges=(\d+) overlap_nonadj=(\d+)',
                  stderr_text)
    if m:
        timing["raw_boxes"] = int(m.group(1))
        timing["raw_islands"] = int(m.group(2))
        timing["raw_edges"] = int(m.group(3))
        timing["raw_overlap_nonadj"] = int(m.group(4))

    m = re.search(r'\[PLN\] sweep=(\d+\.?\d*)ms', stderr_text)
    if m:
        timing["sweep_ms"] = float(m.group(1))

    m = re.search(r'\[PLN\] greedy=(\d+\.?\d*)ms', stderr_text)
    if m:
        timing["greedy_ms"] = float(m.group(1))

    m = re.search(r'\[PLN\] grow=(\d+) sweep=(\d+) greedy=(\d+) filter=(\d+)', stderr_text)
    if m:
        timing["n_after_grow"] = int(m.group(1))
        timing["n_after_sweep"] = int(m.group(2))
        timing["n_after_greedy"] = int(m.group(3))
        timing["n_after_filter"] = int(m.group(4))

    m = re.search(r'\[PLN\] pre-bridge: boxes=(\d+) islands=(\d+) edges=(\d+)', stderr_text)
    if m:
        timing["pre_bridge_boxes"] = int(m.group(1))
        timing["pre_bridge_islands"] = int(m.group(2))
        timing["pre_bridge_edges"] = int(m.group(3))

    m = re.search(r'\[PLN\] post-bridge: boxes=(\d+) islands=(\d+)', stderr_text)
    if m:
        timing["post_bridge_boxes"] = int(m.group(1))
        timing["post_bridge_islands"] = int(m.group(2))

    m = re.search(r'\[PLN\] bridge=(\d+\.?\d*)ms', stderr_text)
    if m:
        timing["bridge_ms"] = float(m.group(1))

    m = re.search(r'\[PLN\] coverage: boxes=(\d+) islands=(\d+) edges=(\d+)', stderr_text)
    if m:
        timing["coverage_boxes"] = int(m.group(1))
        timing["coverage_islands"] = int(m.group(2))
        timing["coverage_edges"] = int(m.group(3))

    # [GRW] ffb_stats / ffb_timing / ffb_avg_miss / lect_nodes
    m = re.search(r'\[GRW\] ffb_stats: calls=(\d+) steps=(\d+) '
                  r'cache_hits=(\d+) cache_misses=(\d+)', stderr_text)
    if m:
        timing["ffb_calls"] = int(m.group(1))
        timing["ffb_steps"] = int(m.group(2))
        timing["ffb_cache_hits"] = int(m.group(3))
        timing["ffb_cache_misses"] = int(m.group(4))

    m = re.search(r'\[GRW\] ffb_timing: total=([\d.]+)ms '
                  r'envelope=([\d.]+)ms collide=([\d.]+)ms '
                  r'expand=([\d.]+)ms intervals=([\d.]+)ms', stderr_text)
    if m:
        timing["ffb_total_ms"] = float(m.group(1))
        timing["ffb_envelope_ms"] = float(m.group(2))
        timing["ffb_collide_ms"] = float(m.group(3))
        timing["ffb_expand_ms"] = float(m.group(4))
        timing["ffb_intervals_ms"] = float(m.group(5))

    m = re.search(r'\[GRW\] ffb_avg_miss: envelope=([\d.]+)ms/call', stderr_text)
    if m:
        timing["ffb_avg_miss_envelope_ms"] = float(m.group(1))

    m = re.search(r'\[GRW\] ffb_avg_expand: ([\d.]+)ms/call \((\d+) calls\)', stderr_text)
    if m:
        timing["ffb_avg_expand_ms"] = float(m.group(1))
        timing["ffb_expand_calls"] = int(m.group(2))

    m = re.search(r'\[GRW\] lect_nodes: (\d+)', stderr_text)
    if m:
        timing["lect_nodes_final"] = int(m.group(1))

    # [GRW] expand_profile / expand_avg / expand_dim
    m = re.search(r'\[GRW\] expand_profile: calls=(\d+) new_nodes=(\d+) pick_dim=([\d.]+)ms '
                  r'fk=([\d.]+)ms env=([\d.]+)ms refine=([\d.]+)ms', stderr_text)
    if m:
        timing["exp_calls"] = int(m.group(1))
        timing["exp_new_nodes"] = int(m.group(2))
        timing["exp_pick_dim_ms"] = float(m.group(3))
        timing["exp_fk_ms"] = float(m.group(4))
        timing["exp_env_ms"] = float(m.group(5))
        timing["exp_refine_ms"] = float(m.group(6))

    m = re.search(r'\[GRW\] expand_avg: pick_dim=([\d.]+)ms fk=([\d.]+)ms '
                  r'env=([\d.]+)ms refine=([\d.]+)ms per_call=([\d.]+)ms', stderr_text)
    if m:
        timing["exp_avg_pick_dim_ms"] = float(m.group(1))
        timing["exp_avg_fk_ms"] = float(m.group(2))
        timing["exp_avg_env_ms"] = float(m.group(3))
        timing["exp_avg_refine_ms"] = float(m.group(4))
        timing["exp_avg_total_ms"] = float(m.group(5))

    m = re.search(r'\[GRW\] expand_dim: calls=(\d+) cache_hits=(\d+) \(([\d.]+)%\)', stderr_text)
    if m:
        timing["dim_calls"] = int(m.group(1))
        timing["dim_cache_hits"] = int(m.group(2))
        timing["dim_cache_pct"] = float(m.group(3))

    return timing


def main():
    args = parse_args()
    rng = np.random.RandomState(args.seed)

    # ── Import sbf5 ──────────────────────────────────────────────────────
    from sbf5 import (
        Robot, SBFPlanner, SBFPlannerConfig, Obstacle,
        EndpointSource, EnvelopeType, SplitOrder,
    )

    # ── Load iiwa14 ──────────────────────────────────────────────────────
    robot = Robot.from_json("data/iiwa14.json")
    print(f"Robot: {robot.name()}")
    print(f"  n_joints       = {robot.n_joints()}")
    print(f"  n_active_links = {robot.n_active_links()}")
    limits = robot.joint_limits().limits
    print(f"  joint_ranges   = [{', '.join(f'{iv.lo:.2f}..{iv.hi:.2f}' for iv in limits)}]")

    # ── Random obstacles ─────────────────────────────────────────────────
    obstacles = make_random_obstacles(robot, args.n_obs, rng)
    print(f"\nObstacles: {len(obstacles)}")
    for i, o in enumerate(obstacles):
        mn, mx = o.min_point(), o.max_point()
        sz = [mx[d] - mn[d] for d in range(3)]
        print(f"  [{i}] center=({(mn[0]+mx[0])/2:.3f},{(mn[1]+mx[1])/2:.3f},"
              f"{(mn[2]+mx[2])/2:.3f})  size=({sz[0]:.3f},{sz[1]:.3f},{sz[2]:.3f})")


    # ── Configure planner ────────────────────────────────────────────────
    config = SBFPlannerConfig()
    config.grower.max_boxes = args.max_boxes
    config.grower.timeout_ms = args.timeout_ms
    config.grower.rng_seed = args.seed
    config.grower.enable_promotion = True
    config.grower.ffb_config.max_depth = args.max_depth
    config.grower.n_threads = args.n_threads
    if args.ep_source == "crit":
        config.endpoint_source.source = EndpointSource.CritSample
    else:
        config.endpoint_source.source = EndpointSource.IFK
    config.endpoint_source.n_samples_crit = 50
    config.envelope_type.type = EnvelopeType.LinkIAABB
    config.split_order = SplitOrder.BEST_TIGHTEN
    config.z4_enabled = True
    config.lect_no_cache = args.no_lect_cache
    if args.target_coarsen > 0:
        config.coarsen.target_boxes = args.target_coarsen

    print(f"\nConfig:")
    print(f"  max_boxes        = {config.grower.max_boxes}")
    print(f"  timeout_ms       = {config.grower.timeout_ms}")
    print(f"  rng_seed         = {config.grower.rng_seed}")
    src = config.endpoint_source.source
    src_name = src.name if hasattr(src, 'name') else str(src)
    if src == EndpointSource.CritSample:
        src_name += f" (n={config.endpoint_source.n_samples_crit})"
    print(f"  endpoint_source  = {src_name}")
    print(f"  envelope_type    = {config.envelope_type.type.name if hasattr(config.envelope_type.type, 'name') else config.envelope_type.type}")
    print(f"  split_order      = {config.split_order.name if hasattr(config.split_order, 'name') else config.split_order}")
    print(f"  z4_enabled       = {config.z4_enabled}")
    print(f"  ffb_max_depth    = {config.grower.ffb_config.max_depth}")
    print(f"  n_threads        = {config.grower.n_threads}")
    print(f"  lect_cache       = {'OFF' if config.lect_no_cache else 'ON'}")
    print(f"  target_coarsen   = {config.coarsen.target_boxes}")

    # ── Build (grow + coarsen) ───────────────────────────────────────────
    planner = SBFPlanner(robot, config)

    print(f"\n{'─'*60}")
    print(f"  Running build_coverage() ...  (wavefront grow + coarsen)")
    print(f"{'─'*60}")
    sys.stdout.flush()

    # Capture C++ stderr output for timing, while also showing live output.
    # We redirect fd 2 to a pipe; a reader thread prints lines in real-time
    # and collects them for later parsing.
    import os
    import tempfile
    import threading

    stderr_lines = []
    read_fd, write_fd = os.pipe()
    old_stderr_fd = os.dup(2)
    os.dup2(write_fd, 2)

    def _reader():
        with os.fdopen(read_fd, 'r', buffering=1, errors='replace') as f:
            while True:
                line = f.readline()
                if not line:
                    break
                stderr_lines.append(line)
                sys.stdout.write(f"  [C++] {line}")
                sys.stdout.flush()

    reader_thread = threading.Thread(target=_reader, daemon=True)
    reader_thread.start()

    t0 = time.perf_counter()
    planner.build_coverage(obstacles, timeout_ms=args.timeout_ms)
    t1 = time.perf_counter()

    # Restore stderr, close write end so reader thread finishes
    os.dup2(old_stderr_fd, 2)
    os.close(old_stderr_fd)
    os.close(write_fd)
    reader_thread.join(timeout=2.0)

    stderr_text = ''.join(stderr_lines)

    wall_ms = (t1 - t0) * 1000.0
    print(f"\n  Python wall time = {wall_ms:.1f} ms")

    # Parse C++ timing
    timing = parse_stderr_timing(stderr_text)

    if stderr_text.strip():
        print(f"\n  C++ stderr output:")
        for line in stderr_text.strip().split('\n'):
            print(f"    {line}")

    # ── Timing summary ───────────────────────────────────────────────────
    print(f"\n{'='*60}")
    print(f"  TIMING SUMMARY")
    print(f"{'='*60}")
    if "lect_ms" in timing:
        print(f"  LECT construction    = {timing['lect_ms']:.1f} ms")
    if "grow_ms" in timing:
        print(f"  Forest growth        = {timing['grow_ms']:.1f} ms")
    if "sg_connect_ms" in timing:
        print(f"  S-G connect          = {timing['sg_connect_ms']:.1f} ms  (added {timing.get('sg_connect_added',0)})")
    if "sweep_ms" in timing:
        print(f"  Coarsen (sweep)      = {timing['sweep_ms']:.1f} ms")
    if "greedy_ms" in timing:
        print(f"  Coarsen (greedy)     = {timing['greedy_ms']:.1f} ms")
    if "bridge_ms" in timing:
        print(f"  Bridge islands       = {timing['bridge_ms']:.1f} ms")
    print(f"  Total build (Python) = {wall_ms:.1f} ms")

    # ── Pipeline stages ──────────────────────────────────────────────────
    if "n_after_grow" in timing:
        print(f"\n  Pipeline stages (box counts):")
        print(f"    after grow     = {timing['n_after_grow']}")
        print(f"    after sweep    = {timing['n_after_sweep']}")
        print(f"    after greedy   = {timing['n_after_greedy']}")
        print(f"    after filter   = {timing['n_after_filter']}")

    if "raw_islands" in timing:
        print(f"\n  Raw connectivity:")
        print(f"    islands        = {timing['raw_islands']}")
        print(f"    edges          = {timing['raw_edges']}")
        print(f"    overlap_nonadj = {timing['raw_overlap_nonadj']}")

    if "post_bridge_islands" in timing:
        print(f"\n  Post-bridge:")
        print(f"    boxes          = {timing['post_bridge_boxes']}")
        print(f"    islands        = {timing['post_bridge_islands']}")

    if "coverage_boxes" in timing:
        print(f"\n  Coverage result:")
        print(f"    boxes          = {timing['coverage_boxes']}")
        print(f"    islands        = {timing['coverage_islands']}")
        print(f"    edges          = {timing['coverage_edges']}")

    # ── FFB detailed stats ───────────────────────────────────────────────
    if "ffb_calls" in timing:
        print(f"\n{'='*60}")
        print(f"  FFB DETAILED STATS")
        print(f"{'='*60}")
        print(f"  FFB calls          = {timing['ffb_calls']}")
        print(f"  FFB loop steps     = {timing['ffb_steps']}")
        hits = timing.get("ffb_cache_hits", 0)
        misses = timing.get("ffb_cache_misses", 0)
        total_ce = hits + misses
        print(f"  Cache hits         = {hits}" +
              (f"  ({100*hits/total_ce:.1f}%)" if total_ce else ""))
        print(f"  Cache misses       = {misses}" +
              (f"  ({100*misses/total_ce:.1f}%)" if total_ce else ""))
    if "ffb_total_ms" in timing:
        ftot = max(timing['ffb_total_ms'], 0.01)
        print(f"  FFB total time     = {timing['ffb_total_ms']:.1f} ms")
        print(f"    expand_leaf      = {timing.get('ffb_expand_ms',0):.1f} ms  "
              f"({100*timing.get('ffb_expand_ms',0)/ftot:.1f}%)")
        print(f"    node_intervals   = {timing.get('ffb_intervals_ms',0):.1f} ms  "
              f"({100*timing.get('ffb_intervals_ms',0)/ftot:.1f}%)")
        print(f"    envelope compute = {timing['ffb_envelope_ms']:.1f} ms  "
              f"({100*timing['ffb_envelope_ms']/ftot:.1f}%)")
        print(f"    collision checks = {timing['ffb_collide_ms']:.1f} ms  "
              f"({100*timing['ffb_collide_ms']/ftot:.1f}%)")
        overhead = ftot - timing.get('ffb_expand_ms',0) - timing.get('ffb_intervals_ms',0) \
                   - timing['ffb_envelope_ms'] - timing['ffb_collide_ms']
        print(f"    overhead/other   = {overhead:.1f} ms  "
              f"({100*overhead/ftot:.1f}%)")
    if "ffb_avg_miss_envelope_ms" in timing:
        print(f"  Avg miss envelope  = {timing['ffb_avg_miss_envelope_ms']:.3f} ms/call")
    if "ffb_avg_expand_ms" in timing:
        print(f"  Avg expand_leaf    = {timing['ffb_avg_expand_ms']:.3f} ms/call "
              f"({timing['ffb_expand_calls']} calls)")
    if "lect_nodes_final" in timing:
        print(f"  LECT nodes (final) = {timing['lect_nodes_final']}")

    # ── Expand sub-timing ────────────────────────────────────────────────
    if "exp_calls" in timing:
        print(f"\n{'='*60}")
        print(f"  EXPAND_LEAF SUB-TIMING (split_leaf_impl)")
        print(f"{'='*60}")
        ec = timing["exp_calls"]
        nn = timing.get("exp_new_nodes", ec * 2)
        pdm = timing.get("exp_pick_dim_ms", 0)
        fkm = timing.get("exp_fk_ms", 0)
        evm = timing.get("exp_env_ms", 0)
        rfm = timing.get("exp_refine_ms", 0)
        total_sub = pdm + fkm + evm + rfm
        print(f"  Total expand calls     = {ec}  (new_nodes={nn})")
        print(f"  pick_dim   {pdm:8.1f} ms  ({100*pdm/total_sub:.1f}%)  avg={pdm/ec:.3f} ms/call")
        print(f"  fk_inc     {fkm:8.1f} ms  ({100*fkm/total_sub:.1f}%)  avg={fkm/ec:.3f} ms/call")
        print(f"  envelope   {evm:8.1f} ms  ({100*evm/total_sub:.1f}%)  avg={evm/ec:.3f} ms/call")
        print(f"  refine     {rfm:8.1f} ms  ({100*rfm/total_sub:.1f}%)  avg={rfm/ec:.3f} ms/call")
        print(f"  total_sub  {total_sub:8.1f} ms                avg={total_sub/ec:.3f} ms/call")
        if "dim_calls" in timing:
            print(f"  dim_cache  hits={timing['dim_cache_hits']}/{timing['dim_calls']} ({timing['dim_cache_pct']:.1f}%)")

    # ── Raw boxes stats ──────────────────────────────────────────────────
    raw_boxes = list(planner.raw_boxes())
    raw_stats = box_stats(raw_boxes, "RAW BOXES (pre-coarsen)")

    # ── Coarsened boxes stats ────────────────────────────────────────────
    final_boxes = list(planner.boxes())
    final_stats = box_stats(final_boxes, "COARSENED BOXES (final)")

    # ── Comparison ───────────────────────────────────────────────────────
    if raw_stats and final_stats:
        print(f"\n{'='*60}")
        print(f"  GROW → COARSEN COMPARISON")
        print(f"{'='*60}")
        print(f"  box count : {raw_stats['n']} → {final_stats['n']}  "
              f"(reduction: {1 - final_stats['n']/raw_stats['n']:.1%})")
        print(f"  total vol : {raw_stats['total_volume']:.6g} → "
              f"{final_stats['total_volume']:.6g}  "
              f"(ratio: {final_stats['total_volume']/raw_stats['total_volume']:.3f})")
        print(f"  med vol   : {np.median(raw_stats['volumes']):.6g} → "
              f"{np.median(final_stats['volumes']):.6g}  "
              f"(ratio: {np.median(final_stats['volumes'])/np.median(raw_stats['volumes']):.3f})")
        print(f"  overlaps  : {raw_stats['n_overlap']} → {final_stats['n_overlap']}")

    # ── Volume histogram ─────────────────────────────────────────────────
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(2, 2, figsize=(14, 10))

        # 1) Volume histogram raw vs final
        ax = axes[0, 0]
        if raw_stats:
            ax.hist(raw_stats["volumes"], bins=40, alpha=0.6, label=f"raw (n={raw_stats['n']})")
        if final_stats:
            ax.hist(final_stats["volumes"], bins=40, alpha=0.6, label=f"coarsened (n={final_stats['n']})")
        ax.set_xlabel("Box volume")
        ax.set_ylabel("Count")
        ax.set_title("Volume distribution")
        ax.legend()

        # 2) Log-volume histogram
        ax = axes[0, 1]
        if raw_stats:
            log_v_raw = np.log10(np.clip(raw_stats["volumes"], 1e-30, None))
            ax.hist(log_v_raw, bins=40, alpha=0.6, label="raw")
        if final_stats:
            log_v_fin = np.log10(np.clip(final_stats["volumes"], 1e-30, None))
            ax.hist(log_v_fin, bins=40, alpha=0.6, label="coarsened")
        ax.set_xlabel("log10(volume)")
        ax.set_ylabel("Count")
        ax.set_title("Log-volume distribution")
        ax.legend()

        # 3) Per-dim edge width boxplot (raw)
        ax = axes[1, 0]
        if raw_stats:
            ax.boxplot([raw_stats["widths"][:, d] for d in range(raw_stats["widths"].shape[1])],
                       labels=[f"j{d}" for d in range(raw_stats["widths"].shape[1])])
        ax.set_xlabel("Joint dimension")
        ax.set_ylabel("Edge width (rad)")
        ax.set_title("Raw box edge widths per joint")

        # 4) Per-dim edge width boxplot (coarsened)
        ax = axes[1, 1]
        if final_stats:
            ax.boxplot([final_stats["widths"][:, d] for d in range(final_stats["widths"].shape[1])],
                       labels=[f"j{d}" for d in range(final_stats["widths"].shape[1])])
        ax.set_xlabel("Joint dimension")
        ax.set_ylabel("Edge width (rad)")
        ax.set_title("Coarsened box edge widths per joint")

        fig.suptitle(f"iiwa14 SBF Profile: {args.n_obs} obs, seed={args.seed}, "
                     f"max_boxes={args.max_boxes}", fontsize=13)
        fig.tight_layout()
        out_path = "results/iiwa_sbf_profile.png"
        fig.savefig(out_path, dpi=150)
        plt.close(fig)
        print(f"\n  Saved figure: {out_path}")
    except ImportError:
        print("\n  (matplotlib not available, skipping plots)")


if __name__ == "__main__":
    main()

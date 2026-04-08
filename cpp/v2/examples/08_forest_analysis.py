#!/usr/bin/env python3
"""
Example 8: Forest Analysis & Box Operations
=============================================

Demonstrates:
  - BoxNode creation and queries
  - Forest box iteration
  - Adjacency graph analysis
  - Connected component detection
  - Volume coverage analysis

Usage:
    python 08_forest_analysis.py
"""
import numpy as np

try:
    import pysbf2
except ImportError:
    raise ImportError("pysbf2 not found. Build with -DSBF_WITH_PYTHON=ON.")


def main():
    # ─── Setup and build forest ───────────────────────────────────────────
    robot = pysbf2.Robot.from_json("../../configs/panda.json")
    obstacles = [
        pysbf2.Obstacle(np.array([0.5, 0.0, 0.4]),
                         np.array([0.05, 0.3, 0.2]), "shelf"),
    ]

    config = pysbf2.make_panda_config(seed=42)
    config.max_boxes = 200

    start = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
    goal  = np.array([2.0,  0.5, -1.0, -1.5,   0.5, 1.0,  -0.5])

    print("Building forest...")
    planner = pysbf2.SBFPlanner(robot, obstacles, config)
    planner.build(start, goal, timeout=30.0)
    forest = planner.forest()
    print(f"Forest: {forest.n_boxes()} boxes")

    # ─── 1. BoxNode Operations ───────────────────────────────────────────
    print("\n=== Box Operations ===")

    # Manual box creation
    ivs = [pysbf2.Interval(-0.5, 0.5) for _ in range(robot.n_joints())]
    seed = np.zeros(robot.n_joints())
    box = pysbf2.BoxNode(99, ivs, seed)
    print(f"Manual box: {box}")
    print(f"  dims={box.n_dims()}, volume={box.volume:.6f}")
    print(f"  center={box.center()}")

    # Containment
    q_inside = np.zeros(robot.n_joints())
    q_outside = np.ones(robot.n_joints()) * 2.0
    print(f"  contains(zeros):   {box.contains(q_inside)}")
    print(f"  contains(2*ones):  {box.contains(q_outside)}")
    print(f"  distance(2*ones):  {box.distance_to_config(q_outside):.4f}")
    print(f"  nearest(2*ones):   {box.nearest_point_to(q_outside)}")

    # ─── 2. Forest Iteration ─────────────────────────────────────────────
    print("\n=== Forest Box Statistics ===")
    boxes = forest.boxes()
    volumes = []
    dims_widths = [[] for _ in range(robot.n_joints())]

    for box_id, box in boxes.items():
        volumes.append(box.volume)
        for d in range(box.n_dims()):
            dims_widths[d].append(box.joint_intervals[d].width())

    volumes = np.array(volumes)
    print(f"  Count:  {len(volumes)}")
    print(f"  Total volume: {volumes.sum():.6f}")
    print(f"  Mean volume:  {volumes.mean():.8f}")
    print(f"  Std volume:   {volumes.std():.8f}")
    print(f"  Min volume:   {volumes.min():.10f}")
    print(f"  Max volume:   {volumes.max():.6f}")

    # Volume histogram
    log_vols = np.log10(volumes + 1e-30)
    bins = np.linspace(log_vols.min(), log_vols.max(), 8)
    hist, _ = np.histogram(log_vols, bins=bins)
    print(f"\n  Volume distribution (log10):")
    for i in range(len(hist)):
        bar = "#" * (hist[i] * 40 // max(hist.max(), 1))
        print(f"    [{bins[i]:+6.1f}, {bins[i+1]:+6.1f}): "
              f"{hist[i]:4d} {bar}")

    # Per-dimension width stats
    print(f"\n  Per-joint interval widths:")
    for d in range(robot.n_joints()):
        widths = np.array(dims_widths[d])
        print(f"    Joint {d}: mean={widths.mean():.4f}, "
              f"min={widths.min():.4f}, max={widths.max():.4f}")

    # ─── 3. Adjacency Analysis ───────────────────────────────────────────
    print("\n=== Adjacency Graph ===")
    adj = forest.adjacency()
    degrees = [len(nbrs) for nbrs in adj.values()]
    degrees = np.array(degrees)

    print(f"  Nodes:       {len(adj)}")
    total_edges = sum(degrees) // 2
    print(f"  Edges:       {total_edges}")
    print(f"  Mean degree: {degrees.mean():.2f}")
    print(f"  Median deg:  {np.median(degrees):.0f}")
    print(f"  Max degree:  {degrees.max()}")
    print(f"  Isolates:    {np.sum(degrees == 0)}")

    # Degree distribution
    unique, counts = np.unique(degrees, return_counts=True)
    print(f"\n  Degree distribution:")
    for deg, cnt in zip(unique[:15], counts[:15]):
        bar = "#" * (cnt * 30 // max(counts.max(), 1))
        print(f"    deg={deg:3d}: {cnt:4d} {bar}")
    if len(unique) > 15:
        print(f"    ... ({len(unique) - 15} more)")

    # ─── 4. Connected Components ─────────────────────────────────────────
    print("\n=== Connected Components ===")
    all_ids = list(boxes.keys())
    islands = pysbf2.find_islands(adj, all_ids)
    print(f"  Islands: {len(islands)}")

    island_sizes = sorted(
        [(isl_id, len(members)) for isl_id, members in islands.items()],
        key=lambda x: -x[1]
    )
    for isl_id, size in island_sizes[:10]:
        frac = 100.0 * size / len(all_ids) if all_ids else 0
        print(f"    Island {isl_id}: {size} boxes ({frac:.1f}%)")

    # ─── 5. Containment Queries ──────────────────────────────────────────
    print("\n=== Containment Queries ===")
    start_box = forest.find_containing(start)
    goal_box = forest.find_containing(goal)
    print(f"  Start in box: {start_box.id if start_box else 'None'}")
    print(f"  Goal in box:  {goal_box.id if goal_box else 'None'}")

    nearest = forest.find_nearest(goal)
    if nearest:
        print(f"  Nearest to goal: box {nearest.id}, "
              f"dist={nearest.distance_to_config(goal):.4f}")

    # ─── 6. Box Adjacency Checks ─────────────────────────────────────────
    print("\n=== Pairwise Box Checks ===")
    box_list = list(boxes.values())
    if len(box_list) >= 2:
        b0, b1 = box_list[0], box_list[1]
        print(f"  Box {b0.id} vs Box {b1.id}:")
        print(f"    overlaps:  {b0.overlaps_with(b1)}")
        print(f"    adjacent:  {b0.is_adjacent_to(b1)}")
        if b0.is_adjacent_to(b1):
            fc = b0.shared_face_center(b1)
            print(f"    face center: {fc[:3]}...")

    # ─── 7. UnionFind Demo ───────────────────────────────────────────────
    print("\n=== UnionFind Demo ===")
    uf = pysbf2.UnionFind(10)
    uf.unite(0, 1)
    uf.unite(2, 3)
    uf.unite(1, 3)
    uf.unite(5, 6)
    print(f"  Components: {uf.n_components()}")
    print(f"  0 connected to 3: {uf.connected(0, 3)}")
    print(f"  0 connected to 5: {uf.connected(0, 5)}")
    comps = uf.components()
    for rep, members in comps.items():
        if len(members) > 1:
            print(f"    Rep {rep}: {members}")

    # ─── 8. Interval Cache for GCS ───────────────────────────────────────
    print("\n=== Interval Cache (for GCS) ===")
    forest.rebuild_interval_cache()
    lo = forest.intervals_lo()
    hi = forest.intervals_hi()
    ids = forest.interval_ids()
    print(f"  Cache shape: lo={lo.shape}, hi={hi.shape}")
    print(f"  Box IDs: {len(ids)} entries")
    if lo.shape[0] > 0:
        print(f"  First box range (dim 0): [{lo[0,0]:.4f}, {hi[0,0]:.4f}]")


if __name__ == "__main__":
    main()

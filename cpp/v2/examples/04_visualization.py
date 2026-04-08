#!/usr/bin/env python3
"""
Example 4: Visualization — Export & Plot
==========================================

Demonstrates:
  - Exporting forest data to JSON
  - Exporting path FK frames
  - Exporting box intervals as CSV
  - Plotting boxes and paths with matplotlib

Usage:
    python 04_visualization.py
"""
import numpy as np
import os

try:
    import pysbf2
except ImportError:
    raise ImportError("pysbf2 not found. Build with -DSBF_WITH_PYTHON=ON.")


def plot_boxes_2d(boxes_dict, dim_x=0, dim_y=1, path=None,
                  start=None, goal=None, title="Forest Projection"):
    """Plot 2D projection of boxes using matplotlib."""
    try:
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpl_patches
    except ImportError:
        print("matplotlib not available, skipping plot")
        return

    fig, ax = plt.subplots(1, 1, figsize=(10, 8))

    # Draw boxes
    for box_id, box in boxes_dict.items():
        ivs = box.joint_intervals
        if dim_x < len(ivs) and dim_y < len(ivs):
            lo_x, hi_x = ivs[dim_x].lo, ivs[dim_x].hi
            lo_y, hi_y = ivs[dim_y].lo, ivs[dim_y].hi
            rect = mpl_patches.Rectangle(
                (lo_x, lo_y), hi_x - lo_x, hi_y - lo_y,
                linewidth=0.5, edgecolor="navy",
                facecolor="steelblue", alpha=0.2
            )
            ax.add_patch(rect)

    # Draw path
    if path is not None and path.shape[0] > 0:
        ax.plot(path[:, dim_x], path[:, dim_y],
                "r-o", markersize=2, linewidth=1.0, label="path")

    # Draw start/goal
    if start is not None:
        ax.plot(start[dim_x], start[dim_y],
                "g*", markersize=15, label="start")
    if goal is not None:
        ax.plot(goal[dim_x], goal[dim_y],
                "m*", markersize=15, label="goal")

    ax.set_xlabel(f"Joint {dim_x}")
    ax.set_ylabel(f"Joint {dim_y}")
    ax.set_title(f"{title} ({len(boxes_dict)} boxes)")
    ax.set_aspect("equal")
    ax.legend()
    ax.autoscale()

    output_path = "forest_projection.png"
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    print(f"Saved plot to {output_path}")
    plt.close()


def main():
    # ─── Setup ────────────────────────────────────────────────────────────
    robot = pysbf2.Robot.from_json("../../configs/panda.json")
    obstacles = [
        pysbf2.Obstacle(np.array([0.5, 0.0, 0.4]),
                         np.array([0.05, 0.3, 0.2]), "shelf"),
        pysbf2.Obstacle(np.array([0.0, 0.4, 0.3]),
                         np.array([0.15, 0.05, 0.15]), "wall"),
    ]

    start = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
    goal  = np.array([2.0,  0.5, -1.0, -1.5,   0.5, 1.0,  -0.5])

    # ─── Plan ─────────────────────────────────────────────────────────────
    config = pysbf2.make_panda_config(seed=42)
    planner = pysbf2.SBFPlanner(robot, obstacles, config)
    result = planner.plan(start, goal, timeout=30.0)

    print(f"Planning: {'SUCCESS' if result.success else 'FAIL'}")
    print(f"Forest: {planner.forest().n_boxes()} boxes")

    # ─── 1. Export Forest JSON ───────────────────────────────────────────
    print("\n=== JSON Export ===")

    # Forest only
    pysbf2.viz.export_forest_json(planner.forest(), "forest_only.json")
    print(f"Exported forest → forest_only.json "
          f"({os.path.getsize('forest_only.json')} bytes)")

    # Forest + obstacles + path
    if result.success:
        pysbf2.viz.export_forest_json(
            planner.forest(), obstacles, result.path, "forest_full.json")
        print(f"Exported forest+path → forest_full.json "
              f"({os.path.getsize('forest_full.json')} bytes)")

    # ─── 2. Export FK Frames ─────────────────────────────────────────────
    print("\n=== FK Frames Export ===")
    if result.success:
        pysbf2.viz.export_path_frames(robot, result.path, "path_frames.json")
        print(f"Exported FK frames → path_frames.json "
              f"({os.path.getsize('path_frames.json')} bytes)")

    # ─── 3. Export Boxes CSV ─────────────────────────────────────────────
    print("\n=== CSV Export ===")
    pysbf2.viz.export_boxes_csv(planner.forest(), "boxes.csv")
    print(f"Exported boxes → boxes.csv "
          f"({os.path.getsize('boxes.csv')} bytes)")

    # Preview CSV
    with open("boxes.csv") as f:
        lines = f.readlines()
    print(f"  Header: {lines[0].strip()}")
    print(f"  Rows:   {len(lines) - 1}")
    if len(lines) > 1:
        print(f"  First:  {lines[1].strip()[:80]}...")

    # ─── 4. Plot with Matplotlib ─────────────────────────────────────────
    print("\n=== Matplotlib Plot ===")
    boxes = planner.forest().boxes()

    if result.success:
        plot_boxes_2d(boxes, dim_x=0, dim_y=1,
                      path=result.path, start=start, goal=goal,
                      title="Panda Forest (Joints 0-1)")
    else:
        plot_boxes_2d(boxes, dim_x=0, dim_y=1,
                      start=start, goal=goal,
                      title="Panda Forest (Joints 0-1)")

    # ─── 5. Box Statistics ───────────────────────────────────────────────
    print("\n=== Box Statistics ===")
    volumes = [b.volume for b in boxes.values()]
    if volumes:
        print(f"  Total boxes:  {len(volumes)}")
        print(f"  Total volume: {sum(volumes):.4f}")
        print(f"  Mean volume:  {np.mean(volumes):.6f}")
        print(f"  Median:       {np.median(volumes):.6f}")
        print(f"  Min:          {min(volumes):.8f}")
        print(f"  Max:          {max(volumes):.6f}")

    # ─── 6. Adjacency Statistics ─────────────────────────────────────────
    print("\n=== Adjacency Statistics ===")
    adj = planner.forest().adjacency()
    degrees = [len(nbrs) for nbrs in adj.values()]
    if degrees:
        print(f"  Nodes:        {len(degrees)}")
        print(f"  Total edges:  {sum(degrees) // 2}")
        print(f"  Mean degree:  {np.mean(degrees):.1f}")
        print(f"  Max degree:   {max(degrees)}")


if __name__ == "__main__":
    main()

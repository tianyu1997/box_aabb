#!/usr/bin/env python3
"""End-to-end visualization demo: C++ plan → JSON export → Plotly HTML.

Usage (from v5/):
    $env:PYTHONPATH = "build_x64/Release;python"
    python python/scripts/run_viz_demo.py
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np


def main():
    import sbf5
    import plotly.graph_objects as go
    from sbf5_viz.load_data import (
        SnapshotData, ForestData, ForestBox, SceneData,
        Obstacle as VizObstacle,
    )
    from sbf5_viz.forest_viz import plot_path_on_forest
    from sbf5_viz.combined_viz import plot_combined

    # ── 1. C++ planning ──
    robot = sbf5.Robot.from_json(
        os.path.join(os.path.dirname(__file__), "..", "..", "data",
                     "2dof_planar.json"))

    obs = sbf5.Obstacle(0.8, -0.2, -0.2, 1.2, 0.2, 0.2)  # center≈(1,0,0)

    cfg = sbf5.SBFPlannerConfig()
    planner = sbf5.SBFPlanner(robot, cfg)

    q_start = np.array([0.1, 0.2])
    q_goal = np.array([2.5, 1.5])

    result = planner.plan(q_start, q_goal, [obs], timeout_ms=30000.0)
    if not result.success:
        print("Planning FAILED — generating empty demo anyway")
    else:
        print(f"Plan SUCCESS: {result.n_boxes} boxes, "
              f"{len(result.path)} waypoints, "
              f"length={result.path_length:.3f}")

    # ── 2. Build SnapshotData from plan result ──
    forest_boxes = []
    try:
        boxes = planner.boxes()
        for i, b in enumerate(boxes):
            ivs = np.array([[iv.lo, iv.hi] for iv in b.intervals])
            vol = 1.0
            for iv in b.intervals:
                vol *= (iv.hi - iv.lo)
            forest_boxes.append(ForestBox(id=i, intervals=ivs, volume=vol))
    except Exception as e:
        print(f"Warning: could not extract forest boxes: {e}")

    snapshot = SnapshotData(
        forest=ForestData(
            n_boxes=len(forest_boxes),
            total_volume=sum(fb.volume for fb in forest_boxes),
            boxes=forest_boxes,
        ),
        scene=SceneData(obstacles=[
            VizObstacle(
                center=np.array([1.0, 0.0, 0.0]),
                half_sizes=np.array([0.2, 0.2, 0.2]),
            ),
        ]),
    )

    out_dir = os.path.join(os.path.dirname(__file__), "..", "..",
                           "results", "viz_demo")
    os.makedirs(out_dir, exist_ok=True)

    # ── 3. 2D forest + path visualization ──
    if result.success and forest_boxes:
        path = [np.array(w) for w in result.path]
        fig2d = plot_path_on_forest(snapshot.forest, path,
                                    dim_x=0, dim_y=1)
        fig2d.update_layout(title="SBF 2DOF Plan — Forest + Path")
        html2d = os.path.join(out_dir, "2dof_plan_2d.html")
        fig2d.write_html(html2d)
        print(f"Saved: {html2d}")

    # ── 4. Combined 3D snapshot ──
    fig3d = plot_combined(snapshot, show_robot=False, show_envelope=False,
                          show_scene=True, show_forest=True)

    # Overlay path if planning succeeded
    if result.success:
        path_q0 = [w[0] for w in result.path]
        path_q1 = [w[1] for w in result.path]
        fig3d.add_trace(go.Scatter(
            x=path_q0, y=path_q1,
            mode="lines+markers", name="SBF Path",
            line=dict(color="red", width=3),
            marker=dict(size=8),
        ))

    html3d = os.path.join(out_dir, "2dof_plan.html")
    fig3d.write_html(html3d)
    print(f"Saved: {html3d}")

    # ── 5. Static export (if kaleido available) ──
    try:
        fig3d.write_image(os.path.join(out_dir, "2dof_plan.pdf"),
                          width=600, height=600)
        print(f"Saved: {os.path.join(out_dir, '2dof_plan.pdf')}")
    except (ImportError, ValueError) as e:
        print(f"Static PDF export skipped (install kaleido): {e}")

    print("Viz demo complete.")


if __name__ == "__main__":
    main()

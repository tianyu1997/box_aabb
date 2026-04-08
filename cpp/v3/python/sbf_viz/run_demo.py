"""
CLI runner for sbf_viz — reads JSON exports and generates interactive HTML.

Usage:
    python -m sbf_viz.run_demo <output_dir>

Where <output_dir> contains the JSON files produced by viz_demo.cpp:
    robot.json, envelope.json, voxel_robot.json, voxel_obstacle.json,
    scene.json, snapshot.json

Outputs interactive HTML files in the same directory.
"""

import sys
import os

def main():
    if len(sys.argv) < 2:
        print("Usage: python -m sbf_viz.run_demo <output_dir>")
        print("  output_dir should contain JSON files from viz_demo.cpp")
        sys.exit(1)

    out_dir = sys.argv[1]
    if not os.path.isdir(out_dir):
        print(f"Error: {out_dir} is not a directory")
        sys.exit(1)

    # Lazy imports so missing plotly triggers a clear error
    try:
        import plotly
    except ImportError:
        print("Error: plotly is required. Install with: pip install plotly")
        sys.exit(1)

    from . import (
        load_robot_data, load_scene_data, load_snapshot_data,
    )
    from .robot_viz import plot_robot_3d
    from .scene_viz import plot_scene_3d
    from .combined_viz import plot_snapshot_3d
    from .envelope_comparison_viz import load_envelope_comparison, plot_envelope_comparison

    generated = []

    # ── 1. Robot FK ──
    rp = os.path.join(out_dir, "robot.json")
    if os.path.exists(rp):
        html = os.path.join(out_dir, "viz_robot.html")
        data = load_robot_data(rp)
        plot_robot_3d(data, title=f"Robot: {data.name}", show=False, save_html=html)
        generated.append(("Robot FK", html))

    # ── 2-4. Comparison group pages ──
    ec = os.path.join(out_dir, "envelope_comparison.json")
    if os.path.exists(ec):
        cmp_data = load_envelope_comparison(ec)

        # Group A: IFK vs Analytical
        html_a = os.path.join(out_dir, "viz_cmp_ifk_vs_analytical.html")
        plot_envelope_comparison(
            cmp_data,
            title="IFK vs Analytical",
            show=False, save_html=html_a,
            methods_filter=["sub_aabb", "analytical_sub_aabb",
                           "voxel_sub_aabb", "voxel_analytical"],
        )
        generated.append(("IFK vs Analytical", html_a))

        # Group B: AABB vs Sub-AABB
        html_b = os.path.join(out_dir, "viz_cmp_aabb_vs_subaabb.html")
        plot_envelope_comparison(
            cmp_data,
            title="AABB vs Sub-AABB",
            show=False, save_html=html_b,
            methods_filter=["full_aabb", "sub_aabb",
                           "voxel_full_aabb", "voxel_sub_aabb"],
        )
        generated.append(("AABB vs Sub-AABB", html_b))

        # Group C: Sub-AABB vs Hull-16
        html_c = os.path.join(out_dir, "viz_cmp_subaabb_vs_hull16.html")
        plot_envelope_comparison(
            cmp_data,
            title="Sub-AABB vs Hull-16",
            show=False, save_html=html_c,
            methods_filter=["sub_aabb", "voxel_sub_aabb", "voxel_hull16"],
        )
        generated.append(("Sub-AABB vs Hull-16", html_c))

    # ── 5. Scene ──
    sp = os.path.join(out_dir, "scene.json")
    if os.path.exists(sp):
        html = os.path.join(out_dir, "viz_scene.html")
        data = load_scene_data(sp)
        plot_scene_3d(data, title="Scene Obstacles",
                      show=False, save_html=html)
        generated.append(("Scene", html))

    # ── 6. All methods (full comparison) ──
    if os.path.exists(ec):
        html = os.path.join(out_dir, "viz_envelope_comparison.html")
        plot_envelope_comparison(cmp_data,
                                 title="All Methods Comparison",
                                 show=False, save_html=html)
        generated.append(("All Methods", html))

    # ── 7. Combined snapshot ──
    snp = os.path.join(out_dir, "snapshot.json")
    if os.path.exists(snp):
        html = os.path.join(out_dir, "viz_snapshot.html")
        plot_snapshot_3d(snp,
                         show_robot=True,
                         show_full_aabb=False,
                         show_sub_aabb=True,
                         show_robot_voxel=True,
                         show_obstacle_voxel=True,
                         show_obstacles=True,
                         title="Combined Snapshot",
                         show=False, save_html=html)
        generated.append(("Combined Snapshot", html))

    # ── Summary ──
    print(f"\n{'='*60}")
    print(f"  sbf_viz: generated {len(generated)} HTML file(s)")
    print(f"{'='*60}")
    for name, path in generated:
        print(f"  [{name:20s}]  {os.path.abspath(path)}")
    print()


if __name__ == "__main__":
    main()

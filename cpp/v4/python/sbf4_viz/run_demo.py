"""
CLI runner for sbf4_viz — reads JSON exports and generates interactive HTML.

Usage:
    python -m sbf4_viz <output_dir>

Where <output_dir> contains the JSON files produced by C++ viz export:
    robot.json, envelope.json, voxel_robot.json, voxel_obstacle.json,
    scene.json, snapshot.json, envelope_comparison.json

Outputs interactive HTML files in the same directory.

迁移自 v3 sbf_viz/run_demo.py
v4 变更: method key 更新, 移除 Analytical 相关分组
"""

import sys
import os

def main():
    if len(sys.argv) < 2:
        print("Usage: python -m sbf4_viz <output_dir>")
        print("  output_dir should contain JSON files from C++ viz export")
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

    # ── 2-3. Comparison group pages ──
    ec = os.path.join(out_dir, "envelope_comparison.json")
    if os.path.exists(ec):
        cmp_data = load_envelope_comparison(ec)

        # Group A: Full iAABB vs Link iAABB
        html_a = os.path.join(out_dir, "viz_cmp_full_vs_link_iaabb.html")
        plot_envelope_comparison(
            cmp_data,
            title="Full iAABB vs Link iAABB",
            show=False, save_html=html_a,
            methods_filter=["full_link_iaabb", "link_iaabb",
                           "voxel_full_link_iaabb", "voxel_link_iaabb"],
        )
        generated.append(("Full vs Link iAABB", html_a))

        # Group B: Link iAABB vs Hull-16
        html_b = os.path.join(out_dir, "viz_cmp_iaabb_vs_hull16.html")
        plot_envelope_comparison(
            cmp_data,
            title="Link iAABB vs Hull-16",
            show=False, save_html=html_b,
            methods_filter=["link_iaabb", "voxel_link_iaabb", "voxel_hull16"],
        )
        generated.append(("iAABB vs Hull-16", html_b))

    # ── 4. Scene ──
    sp = os.path.join(out_dir, "scene.json")
    if os.path.exists(sp):
        html = os.path.join(out_dir, "viz_scene.html")
        data = load_scene_data(sp)
        plot_scene_3d(data, title="Scene Obstacles",
                      show=False, save_html=html)
        generated.append(("Scene", html))

    # ── 5. All methods (full comparison) ──
    if os.path.exists(ec):
        html = os.path.join(out_dir, "viz_envelope_comparison.html")
        plot_envelope_comparison(cmp_data,
                                 title="All Methods Comparison",
                                 show=False, save_html=html)
        generated.append(("All Methods", html))

    # ── 6. Combined snapshot ──
    snp = os.path.join(out_dir, "snapshot.json")
    if os.path.exists(snp):
        html = os.path.join(out_dir, "viz_snapshot.html")
        plot_snapshot_3d(snp,
                         show_robot=True,
                         show_full_link_iaabb=False,
                         show_link_iaabb=True,
                         show_robot_voxel=True,
                         show_obstacle_voxel=True,
                         show_obstacles=True,
                         title="Combined Snapshot",
                         show=False, save_html=html)
        generated.append(("Combined Snapshot", html))

    # ── Summary ──
    print(f"\n{'='*60}")
    print(f"  sbf4_viz: generated {len(generated)} HTML file(s)")
    print(f"{'='*60}")
    for name, path in generated:
        print(f"  [{name:20s}]  {os.path.abspath(path)}")
    print()


if __name__ == "__main__":
    main()

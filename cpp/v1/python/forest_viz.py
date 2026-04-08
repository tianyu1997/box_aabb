"""SafeBoxForest — Forest visualization using matplotlib.

Usage:
    python forest_viz.py forest_data.json [--output fig.png]

Loads JSON exported by sbf::viz::export_forest_json and renders:
- 2D pairwise projections of box intervals
- Adjacency graph overlay
- Path waypoints
- Obstacles (3D view)
"""
import argparse
import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.patches as mpl_patches
import numpy as np


def load_forest(path: str) -> dict:
    with open(path) as f:
        return json.load(f)


def plot_boxes_2d(ax, boxes: list, dim_x: int, dim_y: int, **kwargs):
    """Plot axis-aligned boxes projected onto (dim_x, dim_y) plane."""
    color = kwargs.get("color", "steelblue")
    alpha = kwargs.get("alpha", 0.25)
    edge_color = kwargs.get("edgecolor", "navy")

    for bx in boxes:
        ivs = bx["intervals"]
        lo_x, hi_x = ivs[dim_x]
        lo_y, hi_y = ivs[dim_y]
        rect = mpl_patches.Rectangle(
            (lo_x, lo_y), hi_x - lo_x, hi_y - lo_y,
            linewidth=0.5, edgecolor=edge_color, facecolor=color, alpha=alpha
        )
        ax.add_patch(rect)


def plot_adjacency_2d(ax, boxes: list, adjacency: list, dim_x: int, dim_y: int):
    """Draw adjacency edges between box centers."""
    id_to_center = {}
    for bx in boxes:
        ivs = bx["intervals"]
        cx = 0.5 * (ivs[dim_x][0] + ivs[dim_x][1])
        cy = 0.5 * (ivs[dim_y][0] + ivs[dim_y][1])
        id_to_center[bx["id"]] = (cx, cy)

    for a, b in adjacency:
        if a in id_to_center and b in id_to_center:
            ca, cb = id_to_center[a], id_to_center[b]
            ax.plot([ca[0], cb[0]], [ca[1], cb[1]],
                    color="gray", linewidth=0.3, alpha=0.5)


def plot_path_2d(ax, path: list, dim_x: int, dim_y: int):
    """Plot path waypoints."""
    xs = [p[dim_x] for p in path]
    ys = [p[dim_y] for p in path]
    ax.plot(xs, ys, "r-o", markersize=2, linewidth=1.0, label="path")
    if xs:
        ax.plot(xs[0], ys[0], "g*", markersize=10, label="start")
        ax.plot(xs[-1], ys[-1], "m*", markersize=10, label="goal")


def main():
    parser = argparse.ArgumentParser(description="Visualize SBF forest")
    parser.add_argument("input", help="JSON file from export_forest_json")
    parser.add_argument("--output", "-o", help="Save figure to file")
    parser.add_argument("--dims", nargs=2, type=int, default=[0, 1],
                        help="Joint dimensions to project (default: 0 1)")
    args = parser.parse_args()

    data = load_forest(args.input)
    boxes = data.get("boxes", [])
    adjacency = data.get("adjacency", [])
    path = data.get("path", [])

    dx, dy = args.dims

    fig, ax = plt.subplots(1, 1, figsize=(10, 8))
    plot_boxes_2d(ax, boxes, dx, dy)
    plot_adjacency_2d(ax, boxes, adjacency, dx, dy)
    if path:
        plot_path_2d(ax, path, dx, dy)

    ax.set_xlabel(f"Joint {dx}")
    ax.set_ylabel(f"Joint {dy}")
    ax.set_title(f"SBF Forest ({len(boxes)} boxes)")
    ax.set_aspect("equal")
    ax.legend(loc="upper right")
    ax.autoscale()

    if args.output:
        fig.savefig(args.output, dpi=150, bbox_inches="tight")
        print(f"Saved to {args.output}")
    else:
        plt.show()


if __name__ == "__main__":
    main()

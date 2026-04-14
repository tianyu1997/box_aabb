"""Command-line entry point for sbf5_viz."""

import argparse
import sys

from .load_data import load_snapshot
from .combined_viz import plot_combined


def main():
    parser = argparse.ArgumentParser(
        prog="sbf5_viz",
        description="SBF v5 Visualizer — interactive 3D Plotly viewer",
    )
    parser.add_argument("input", help="Snapshot JSON file path")
    parser.add_argument("--html", default=None,
                        help="Output HTML path (instead of opening browser)")
    parser.add_argument("--no-robot", action="store_true",
                        help="Hide robot layer")
    parser.add_argument("--no-envelope", action="store_true",
                        help="Hide envelope layer")
    parser.add_argument("--no-scene", action="store_true",
                        help="Hide scene layer")
    parser.add_argument("--show-forest", action="store_true",
                        help="Show forest C-space boxes")
    args = parser.parse_args()

    snapshot = load_snapshot(args.input)
    fig = plot_combined(
        snapshot,
        show_robot=not args.no_robot,
        show_envelope=not args.no_envelope,
        show_scene=not args.no_scene,
        show_forest=args.show_forest,
    )

    if args.html:
        fig.write_html(args.html)
        print(f"Saved: {args.html}")
    else:
        fig.show()


if __name__ == "__main__":
    main()

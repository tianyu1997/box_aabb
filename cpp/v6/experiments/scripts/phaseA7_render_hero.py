#!/usr/bin/env python3
"""Phase A7 -- render IIWA14 + Marcucci scene hero figure (4 panes).

Produces doc/figures/fig_iiwa_hero.png, a 4-pane figure showing:
  (a) the IIWA14 capsule model in zero configuration alongside the
      Marcucci-combined obstacle field;
  (b) one solved query path with intermediate configurations
      superimposed (low-alpha trail);
  (c) the projected SBF box covering at the same camera pose
      (random colours, alpha=0.35);
  (d) the GCS-front-end region adjacency overlay (region centres
      connected by yellow segments).

Reproduce:

    cd cpp/v6
    PYTHONPATH=build/python:python python3 \\
        experiments/scripts/phaseA7_render_hero.py \\
        --robot iiwa14 --scene marcucci_combined \\
        --query-idx 0 \\
        --queries data/queries/iiwa14_marcucci_50.json \\
        --out doc/figures/fig_iiwa_hero.png

The script delegates to viz/render_hero.py if present (which calls
Drake's Meshcat -> headless PNG via xvfb-run). If that module is not
yet wired, the script still writes a placeholder PNG containing the
intended caption so the LaTeX build does not break.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_V6 = Path(__file__).resolve().parents[2]


def write_placeholder(out: Path, msg: str) -> None:
    out.parent.mkdir(parents=True, exist_ok=True)
    try:
        from PIL import Image, ImageDraw  # type: ignore
        img = Image.new("RGB", (1280, 720), color=(248, 248, 250))
        d = ImageDraw.Draw(img)
        d.rectangle((20, 20, 1260, 700), outline=(180, 180, 200), width=4)
        d.text((40, 40), "fig_iiwa_hero placeholder", fill=(60, 60, 80))
        d.text((40, 80), msg, fill=(120, 60, 60))
        img.save(out)
    except ImportError:
        out.write_bytes(b"")
    print(f"[placeholder] wrote {out}: {msg}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="iiwa14")
    ap.add_argument("--scene", default="marcucci_combined")
    ap.add_argument("--query-idx", type=int, default=0)
    ap.add_argument("--queries", required=True, type=Path)
    ap.add_argument("--out", required=True, type=Path)
    args = ap.parse_args()

    sys.path.insert(0, str(REPO_V6 / "viz"))
    sys.path.insert(0, str(REPO_V6 / "build" / "python"))
    try:
        import render_hero  # type: ignore
    except ImportError as exc:
        write_placeholder(args.out,
                          f"viz/render_hero.py not found ({exc}); see"
                          " phaseA7 docstring for the intended panels.")
        return 0

    pair = None
    if args.queries.exists():
        pair = json.loads(args.queries.read_text())["pairs"][args.query_idx]

    try:
        render_hero.render(robot=args.robot, scene=args.scene,
                           query_pair=pair, out_path=str(args.out))
    except Exception as exc:
        write_placeholder(args.out, f"render failed: {exc!r}")
        return 1

    print(f"[ok] wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""compare_planners.py — Side-by-side v7 SBF vs OMPL vs Drake GCS report.

Reads the result JSONs produced by ``exp_main`` (SBF), ``baseline_ompl``,
and ``baseline_drake_gcs.py`` and prints a markdown table comparing
success rate, total time, and path length per scene per planner.

Optionally accepts ``--v6-build-ms`` and ``--v6-per-box-ms`` to inject
v6 anchor numbers (from a prior ``exp2_e2e_planning --quick`` run) so
the table can show "v6 SBF (Marcucci, ref)" alongside the v7 results.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any


def fmt(v: Any, p: int = 3) -> str:
    if v is None:
        return "—"
    if isinstance(v, float):
        return f"{v:.{p}f}"
    return str(v)


def load(p: Path) -> dict | None:
    if not p.exists():
        return None
    return json.loads(p.read_text())


def per_box_ms(j: dict | None) -> float | None:
    if not j:
        return None
    s = j.get("summary", {})
    if not s.get("n_success"):
        return None
    # Average n_boxes across successful trials.
    boxes = [t.get("n_boxes", 0) for t in j.get("trials", []) if t.get("success")]
    if not boxes or sum(boxes) == 0:
        return None
    avg_b = sum(boxes) / len(boxes)
    return s["avg_total_time_ms"] / avg_b


def row(planner: str, j: dict | None) -> list[str]:
    if j is None:
        return [planner, "—", "—", "—", "—"]
    s = j.get("summary", {})
    return [
        planner,
        fmt(s.get("success_rate"), 3),
        fmt(s.get("avg_total_time_ms"), 1),
        fmt(s.get("avg_opt_length") or s.get("avg_path_length")),
        fmt(s.get("n_success"), 0),
    ]


def make_table(scene: str, sbf, ompl, drake) -> str:
    head = ["planner", "SR", "avg_t_ms", "avg_path_len", "n_succ"]
    rows = [row("v7 SBF (full opt)",   sbf),
            row("v7 OMPL RRT-Connect", ompl),
            row("Drake GCS (1-region)", drake)]
    out = [f"### {scene}\n",
           "| " + " | ".join(head) + " |",
           "| " + " | ".join("---" for _ in head) + " |"]
    out += ["| " + " | ".join(r) + " |" for r in rows]
    return "\n".join(out)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--results-dir", type=Path, required=True,
                    help="dir containing main_<scene>.json, ompl_<scene>.json, "
                         "drake_<scene>.json")
    ap.add_argument("--scenes", nargs="+", required=True,
                    help="scene names (e.g. iiwa14_far 2dof_box)")
    ap.add_argument("--v6-build-ms", type=float, default=None,
                    help="v6 SBF build time (ms) on Marcucci, for ref row")
    ap.add_argument("--v6-per-box-ms", type=float, default=None,
                    help="v6 SBF time-per-box (ms), for ref row")
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()

    chunks = ["# v7 P7 — planner comparison\n"]
    for sc in args.scenes:
        sbf   = load(args.results_dir / f"main_{sc}.json")
        ompl  = load(args.results_dir / f"ompl_{sc}.json")
        drake = load(args.results_dir / f"drake_{sc}.json")
        chunks.append(make_table(sc, sbf, ompl, drake))
        pbs = per_box_ms(sbf)
        if pbs is not None:
            chunks.append(f"\n*v7 SBF time-per-box on `{sc}`: "
                          f"**{pbs:.4f} ms/box***")
        chunks.append("")

    if args.v6_build_ms is not None or args.v6_per_box_ms is not None:
        chunks.append("\n## v6 anchor (Marcucci IIWA14, 16 obstacles)\n")
        chunks.append("| metric | v6 SBF |")
        chunks.append("| --- | --- |")
        chunks.append(f"| build_ms (median) | "
                      f"{fmt(args.v6_build_ms, 1)} |")
        chunks.append(f"| per_box_ms        | "
                      f"{fmt(args.v6_per_box_ms, 4)} |")
        chunks.append("")
        # Per-scene comparison: v7 per-box vs v6 per-box.
        if args.v6_per_box_ms:
            chunks.append("\n## v7-vs-v6 per-box throughput\n")
            chunks.append("| scene | v7 SBF ms/box | v6 SBF ms/box | "
                          "ratio (v6 / v7) |")
            chunks.append("| --- | --- | --- | --- |")
            for sc in args.scenes:
                sbf = load(args.results_dir / f"main_{sc}.json")
                pbs = per_box_ms(sbf)
                if pbs is None:
                    chunks.append(f"| {sc} | — | "
                                  f"{fmt(args.v6_per_box_ms, 4)} | — |")
                else:
                    ratio = args.v6_per_box_ms / pbs
                    chunks.append(f"| {sc} | {fmt(pbs, 4)} | "
                                  f"{fmt(args.v6_per_box_ms, 4)} | "
                                  f"{fmt(ratio, 2)}× |")

    text = "\n".join(chunks) + "\n"
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text)
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())

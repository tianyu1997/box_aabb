#!/usr/bin/env python3
"""build_tables.py — Convert experiment JSON output into Markdown tables.

Stdlib-only. Reads one or more `*.json` files emitted by the v7 P6
experiment binaries and writes Markdown tables to stdout (or to
`--out`).

Supported experiments: ``main``, ``threads``, ``pathopt_steps``.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any


def fmt(v: Any, p: int = 3) -> str:
    if isinstance(v, float):
        return f"{v:.{p}f}"
    return str(v)


def table(headers: list[str], rows: list[list[Any]]) -> str:
    out = ["| " + " | ".join(headers) + " |",
           "| " + " | ".join("---" for _ in headers) + " |"]
    for r in rows:
        out.append("| " + " | ".join(str(c) for c in r) + " |")
    return "\n".join(out)


def render_main(j: dict) -> str:
    s = j["summary"]
    rows = [[
        j["scene"], j["robot"], len(j["trials"]),
        fmt(s["success_rate"]), s["n_success"],
        fmt(s["avg_total_time_ms"], 1),
        fmt(s["avg_opt_length"]),
        fmt(s["avg_raw_length"]),
    ]]
    head = ["scene", "robot", "trials", "SR", "n_succ",
            "avg_t_ms", "avg_opt_len", "avg_raw_len"]
    return f"### main — {j['scene']}\n\n" + table(head, rows)


def render_threads(j: dict) -> str:
    rows = []
    for r in j["results"]:
        rows.append([r["n_threads"], r["n_success"],
                     fmt(r["avg_total_time_ms"], 1),
                     fmt(r["speedup_vs_1t"], 2)])
    head = ["n_threads", "n_succ", "avg_t_ms", "speedup_vs_1t"]
    return f"### threads — {j['scene']}\n\n" + table(head, rows)


def render_pathopt(j: dict) -> str:
    rows = []
    for r in j["results"]:
        rows.append([r["combo"], r["n_success"],
                     fmt(r["avg_opt_length"]),
                     fmt(r["avg_raw_length"]),
                     fmt(r["length_ratio"], 3),
                     fmt(r["avg_opt_time_ms"], 1)])
    head = ["combo", "n_succ", "avg_opt_len", "avg_raw_len",
            "len_ratio", "avg_opt_t_ms"]
    return f"### pathopt_steps — {j['scene']}\n\n" + table(head, rows)


RENDERERS = {
    "main":          render_main,
    "threads":       render_threads,
    "pathopt_steps": render_pathopt,
}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("inputs", nargs="+", type=Path,
                    help="JSON files emitted by exp_* binaries")
    ap.add_argument("--out", type=Path, help="write to file instead of stdout")
    args = ap.parse_args()

    chunks: list[str] = ["# v7 P6 experiment results\n"]
    for p in args.inputs:
        try:
            j = json.loads(p.read_text())
        except Exception as e:
            print(f"# skip {p}: {e}", file=sys.stderr)
            continue
        ren = RENDERERS.get(j.get("experiment", ""))
        if ren is None:
            print(f"# skip {p}: unknown experiment", file=sys.stderr)
            continue
        chunks.append(ren(j))
        chunks.append("")
    text = "\n\n".join(chunks)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text)
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())

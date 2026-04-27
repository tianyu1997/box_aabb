#!/usr/bin/env python3
"""parse_marcucci_v6.py — convert v6 exp2 stdout into a results JSON.

Reads the stdout from `cpp/v6/build/experiments/exp2_e2e_planning` and emits a
JSON file with the same shape used by the rest of the paper-table tooling:

  {
    "experiment": "marcucci",
    "robot": "iiwa14",
    "scene": "marcucci_combined",
    "seeds": <int>,
    "build": {"median_s": <float>, "mean_s": <float>},
    "queries": [
      {"name": "AS->TS", "sr": 1.00, "t_med_s": 0.29, "len_med": 2.05},
      ...
    ],
    "trials": [{"seed": k, "build_s": ..., "n_boxes": ..., "queries": [...]}]
  }
"""
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path


SEED_HEADER = re.compile(r"\s*seed=(\d+)\s+build=([\d.]+)s\s+boxes=(\d+)")
QUERY_LINE  = re.compile(
    r"\s*([A-Z]+)->([A-Z]+)\s+q=([\d.]+)s\s+(OK|FAIL)(?:\s+len=([\d.]+))?(?:\s+pts=(\d+))?")
SUMMARY_BUILD = re.compile(
    r"Build:\s+med=([\d.]+)s\s+mean=([\d.]+)s")
SUMMARY_Q = re.compile(
    r"\s*([A-Z]+->[A-Z]+)\s+SR=\s*(\d+)%\s+t_med=([\d.]+)s\s+len_med=([\d.]+)")


def parse(log: str) -> dict:
    out: dict = {
        "experiment": "marcucci",
        "robot":      "iiwa14",
        "scene":      "marcucci_combined",
        "trials":     [],
        "queries":    [],
    }
    cur = None
    for line in log.splitlines():
        m = SEED_HEADER.match(line)
        if m:
            cur = {
                "seed":    int(m.group(1)),
                "build_s": float(m.group(2)),
                "n_boxes": int(m.group(3)),
                "queries": [],
            }
            out["trials"].append(cur)
            continue
        m = QUERY_LINE.match(line)
        if m and cur is not None:
            cur["queries"].append({
                "from":   m.group(1),
                "to":     m.group(2),
                "t_s":    float(m.group(3)),
                "ok":     m.group(4) == "OK",
                "length": float(m.group(5)) if m.group(5) else 0.0,
                "n_pts":  int(m.group(6)) if m.group(6) else 0,
            })
            continue
        m = SUMMARY_BUILD.search(line)
        if m:
            out["build"] = {
                "median_s": float(m.group(1)),
                "mean_s":   float(m.group(2)),
            }
            continue
        m = SUMMARY_Q.search(line)
        if m:
            out["queries"].append({
                "name":    m.group(1),
                "sr":      int(m.group(2)) / 100.0,
                "t_med_s": float(m.group(3)),
                "len_med": float(m.group(4)),
            })
    out["seeds"] = len(out["trials"])
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--log",  required=True, help="Path to exp2 stdout log")
    ap.add_argument("--out",  required=True, help="Output JSON path")
    a = ap.parse_args()
    log = Path(a.log).read_text()
    j = parse(log)
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps(j, indent=2))
    print(f"Wrote {a.out} (seeds={j['seeds']}, queries={len(j['queries'])})")


if __name__ == "__main__":
    main()

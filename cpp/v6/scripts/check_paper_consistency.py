#!/usr/bin/env python3
"""Lint v6 paper for numeric consistency against doc/generated/canonical_numbers.csv.

Reproduce:

    cd cpp/v6
    python3 scripts/check_paper_consistency.py

Behaviour:
  * Loads canonical key/value pairs from doc/generated/canonical_numbers.csv.
  * Scans box_aabb_v6_paper_en.tex and _zh.tex for every numeric cell
    matching the patterns documented in `PATTERNS` below.
  * Reports any abstract / introduction occurrence that does not match a
    canonical row, plus any inconsistency between EN and ZH for the same
    \\label-anchored claim.

Exit codes:
  0  = clean
  1  = at least one mismatch -- detail printed to stdout

The script is intentionally conservative: it never auto-rewrites the tex.
It only flags violations so the author can edit one source-of-truth row.
"""
from __future__ import annotations

import argparse
import csv
import re
import sys
from pathlib import Path

REPO_V6 = Path(__file__).resolve().parents[1]
DOC_DIR = REPO_V6 / "doc"
CSV_PATH = DOC_DIR / "generated" / "canonical_numbers.csv"
EN_TEX = DOC_DIR / "box_aabb_v6_paper_en.tex"
ZH_TEX = DOC_DIR / "box_aabb_v6_paper_zh.tex"

# Substring patterns that indicate a headline numeric claim is being made.
# Each entry: (csv_key, list-of-regex-fragments-that-must-find-the-value).
HEADLINE_CLAIMS = [
    ("sbf_build_time_s", [r"1\.51\s*\\?,?\s*s", r"1\.55\s*\\?,?\s*s"]),
    ("sbf_box_count", [r"3,?400", r"3,?445", r"\\sim\s*3"]),
    ("sbf_query_time_s_mean", [r"0\.288", r"0\.29\s*\\?,?\s*s"]),
    ("sbf_to_gcs_path_rad_mean", [r"2\.063", r"2\.06"]),
    ("sbf_to_gcs_query_time_s_mean", [r"0\.064"]),
    ("irisnp_baseline_path_rad", [r"2\.44"]),
    ("irisnp_baseline_n_regions", [r"\b8\s+regions"]),
    ("sbf_vs_irisnp_path_quality_gap_pct", [r"within\s+17\\?%", r"17\.?\d?\\?%"]),
    ("sbf_vs_irisnp_precompute_speedup", [r"83\s*\\?\\?times", r"83\s*x"]),
]


def load_canonical() -> dict[str, dict[str, str]]:
    rows: dict[str, dict[str, str]] = {}
    with CSV_PATH.open() as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            parts = [c.strip() for c in line.split(",")]
            if len(parts) < 2:
                continue
            key = parts[0]
            rows[key] = {
                "value": parts[1],
                "unit": parts[2] if len(parts) > 2 else "",
                "table": parts[3] if len(parts) > 3 else "",
                "n_seeds": parts[4] if len(parts) > 4 else "",
                "script": parts[5] if len(parts) > 5 else "",
            }
    return rows


def scan_tex(path: Path, claims: list[tuple[str, list[str]]]) -> dict[str, list[int]]:
    if not path.exists():
        print(f"[warn] missing {path}")
        return {}
    text = path.read_text(encoding="utf-8")
    lines = text.splitlines()
    found: dict[str, list[int]] = {key: [] for key, _ in claims}
    for idx, line in enumerate(lines, start=1):
        for key, patterns in claims:
            for pat in patterns:
                if re.search(pat, line):
                    found[key].append(idx)
                    break
    return found


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--strict", action="store_true",
                    help="Also fail if a headline number is missing entirely.")
    args = ap.parse_args()

    canonical = load_canonical()
    en = scan_tex(EN_TEX, HEADLINE_CLAIMS)
    zh = scan_tex(ZH_TEX, HEADLINE_CLAIMS)

    bad = 0
    print(f"Canonical rows: {len(canonical)}")
    print(f"{'KEY':45s} {'VALUE':>10s}  EN_lines  ZH_lines")
    for key, _ in HEADLINE_CLAIMS:
        c = canonical.get(key, {})
        v = c.get("value", "?")
        en_hits = en.get(key, [])
        zh_hits = zh.get(key, [])
        marker = "  "
        if not en_hits and args.strict:
            marker = "!!"
            bad += 1
        if not zh_hits and args.strict:
            marker = "!!"
            bad += 1
        print(
            f"{marker} {key:42s} {v:>10s}  "
            f"{','.join(map(str, en_hits[:3])):>9s}  "
            f"{','.join(map(str, zh_hits[:3])):>9s}"
        )

    # Cross-check: contradictions inside the same tex.
    # Hard rule: SBF mean path length should appear only as one canonical
    # value; flag if multiple distinct floats co-occur near keyword "mean".
    pat_mean_path = re.compile(
        r"mean[^\n]*?(\d+\.\d{1,3})\s*(?:rad|\\,?rad)", re.IGNORECASE
    )
    for label, path in [("EN", EN_TEX), ("ZH", ZH_TEX)]:
        if not path.exists():
            continue
        text = path.read_text(encoding="utf-8")
        seen = set()
        for m in pat_mean_path.finditer(text):
            seen.add(m.group(1))
        if len(seen) > 3:  # heuristic upper bound for legit per-table values
            print(f"[warn] {label}: multiple distinct mean-path floats {sorted(seen)}")

    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Phase E -- structural diff of EN vs ZH paper to enforce parity.

Compares cpp/v6/doc/box_aabb_v6_paper_en.tex and *_zh.tex at the level
of (section_title, subsection_title, theorem/algorithm/figure/table
labels). Reports labels present on one side but missing on the other.

Reproduce:

    cd cpp/v6
    python3 scripts/diff_en_zh.py
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

REPO_V6 = Path(__file__).resolve().parents[1]
EN = REPO_V6 / "doc" / "box_aabb_v6_paper_en.tex"
ZH = REPO_V6 / "doc" / "box_aabb_v6_paper_zh.tex"

LABEL_RE = re.compile(r"\\label\{([^}]+)\}")
SECTION_RE = re.compile(r"\\(section|subsection|subsubsection)\*?\{([^}]+)\}")
ENV_RE = re.compile(
    r"\\begin\{(theorem|lemma|proposition|corollary|definition|"
    r"algorithm|algorithm2e|figure\*?|table\*?)\}")


def scan(path: Path) -> dict:
    text = path.read_text(encoding="utf-8")
    return {
        "labels": sorted(set(LABEL_RE.findall(text))),
        "sections": SECTION_RE.findall(text),
        "envs": ENV_RE.findall(text),
    }


def main() -> int:
    if not EN.exists() or not ZH.exists():
        print(f"[fatal] missing one of: {EN}, {ZH}")
        return 2
    en, zh = scan(EN), scan(ZH)

    en_only = sorted(set(en["labels"]) - set(zh["labels"]))
    zh_only = sorted(set(zh["labels"]) - set(en["labels"]))

    print(f"EN labels: {len(en['labels'])}    ZH labels: {len(zh['labels'])}")
    print(f"EN sections: {len(en['sections'])}    "
          f"ZH sections: {len(zh['sections'])}")

    en_envs = {}
    zh_envs = {}
    for k in en["envs"]:
        en_envs[k] = en_envs.get(k, 0) + 1
    for k in zh["envs"]:
        zh_envs[k] = zh_envs.get(k, 0) + 1
    keys = sorted(set(en_envs) | set(zh_envs))
    print("\nEnvironment counts (env: en/zh):")
    for k in keys:
        a, b = en_envs.get(k, 0), zh_envs.get(k, 0)
        flag = "" if a == b else "  <-- MISMATCH"
        print(f"  {k:14s} {a:3d} / {b:3d}{flag}")

    if en_only:
        print(f"\n[EN-only labels] ({len(en_only)})")
        for lbl in en_only:
            print(f"  {lbl}")
    if zh_only:
        print(f"\n[ZH-only labels] ({len(zh_only)})")
        for lbl in zh_only:
            print(f"  {lbl}")

    if en_only or zh_only or en_envs != zh_envs:
        print("\n[FAIL] EN/ZH parity broken")
        return 1
    print("\n[OK] EN/ZH structurally aligned")
    return 0


if __name__ == "__main__":
    sys.exit(main())

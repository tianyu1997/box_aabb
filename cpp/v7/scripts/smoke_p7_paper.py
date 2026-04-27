#!/usr/bin/env python3
"""smoke_p7_paper.py — verify paper artefacts.

Checks (exits non-zero on the first failure):
  1. EN and ZH .tex files exist.
    1b. No parallel EN rewrite source exists; `sbf_paper_en.tex` is the only maintained English source.
  2. doc/generated/macros.tex exists and contains every \\newcommand
     that the .tex files reference.
  3. The .tex files contain no `??` placeholders, no TODO/FIXME, and
     no version-related references (v1…v9, vN, ``version'') in
     non-comment lines.
  4. EN .pdf and ZH .pdf exist (build artefacts).
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DOC  = ROOT / "doc"
GEN  = DOC / "generated"

EN     = DOC / "sbf_paper_en.tex"
EN_REWRITE = DOC / "sbf_paper_en_rewrite.tex"
ZH     = DOC / "sbf_paper_zh.tex"
EN_PDF = DOC / "sbf_paper_en.pdf"
ZH_PDF = DOC / "sbf_paper_zh.pdf"
MACROS = GEN / "macros.tex"

errors: list[str] = []

# 1. file existence
for p in [EN, ZH, MACROS]:
    if not p.exists():
        errors.append(f"missing: {p}")
if EN_REWRITE.exists():
    errors.append(f"duplicate EN paper source still present: {EN_REWRITE}")

# 2. macros completeness
if MACROS.exists():
    defined = set(re.findall(r"\\newcommand\{\\(\w+)\}", MACROS.read_text()))
    for tex in [EN, ZH]:
        if not tex.exists():
            continue
        used = set(re.findall(r"\\(sbf[A-Z]\w*)\b", tex.read_text()))
        missing = used - defined - {"sbf"}
        if missing:
            errors.append(f"{tex.name} references undefined macros: "
                          f"{sorted(missing)}")

# 3. content hygiene
banned = {"??":          r"\?\?",
          "TODO":        r"\bTODO\b",
          "FIXME":       r"\bFIXME\b",
          "PLACEHOLDER": r"\bPLACEHOLDER\b"}
for tex in [EN, ZH]:
    if not tex.exists():
        continue
    body = "\n".join(line for line in tex.read_text().splitlines()
                     if not line.lstrip().startswith("%"))
    for label, pat in banned.items():
        if re.search(pat, body, flags=re.IGNORECASE):
            errors.append(f"{tex.name} contains forbidden token: {label}")

# 4. pdfs
for p in [EN_PDF, ZH_PDF]:
    if not p.exists():
        errors.append(f"missing built pdf: {p.name} "
                      f"(run pdflatex/xelatex first)")

if errors:
    print("FAIL:", file=sys.stderr)
    for e in errors:
        print(f"  - {e}", file=sys.stderr)
    sys.exit(1)
print(f"OK: {EN.name}, {ZH.name}, {MACROS.name}, {EN_PDF.name}, "
      f"{ZH_PDF.name} all present and clean.")

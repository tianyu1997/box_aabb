#!/usr/bin/env python3
"""verify_quick_results.py — Assert SR floors on quick experiment JSONs.

Usage:  verify_quick_results.py <result.json> [<result.json> ...]

Exits 0 iff every result meets its acceptance threshold.

Thresholds (matched against ``experiment`` field):
* main          : summary.success_rate >= 0.66 (must solve ≥ 2/3)
* threads       : every results[i].n_success >= ceil(seeds * 0.66)
* pathopt_steps : at least one combo achieves n_success >= ceil(seeds * 0.66)
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

OK_RE = "[\u2713 PASS]"
BAD_RE = "[\u2717 FAIL]"


def check(p: Path) -> bool:
    j = json.loads(p.read_text())
    exp = j.get("experiment", "?")
    seeds = int(j.get("seeds", 1))
    floor = math.ceil(seeds * 0.66)

    if exp == "main":
        sr = j["summary"]["success_rate"]
        ok = sr >= 0.66
        print(f"{OK_RE if ok else BAD_RE} {p}: main SR={sr:.3f} (need ≥0.66)")
        return ok
    if exp == "threads":
        bad = []
        for r in j["results"]:
            if r["n_success"] < floor:
                bad.append(f"nt={r['n_threads']} succ={r['n_success']}/{seeds}")
        ok = not bad
        msg = ", ".join(bad) if bad else "all thread counts pass"
        print(f"{OK_RE if ok else BAD_RE} {p}: threads — {msg}")
        return ok
    if exp == "pathopt_steps":
        passing = [r["combo"] for r in j["results"] if r["n_success"] >= floor]
        ok = bool(passing)
        print(f"{OK_RE if ok else BAD_RE} {p}: pathopt_steps — passing="
              f"{passing}")
        return ok
    print(f"{BAD_RE} {p}: unknown experiment {exp!r}")
    return False


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    failed = 0
    for arg in sys.argv[1:]:
        if not check(Path(arg)):
            failed += 1
    if failed:
        print(f"\n{failed} file(s) failed acceptance.")
        return 1
    print("\nAll quick results passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

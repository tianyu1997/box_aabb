#!/usr/bin/env python3
"""Phase 0 verification: load IIWA14 / Panda / UR10 robots, sanity-check
the DH chain + capsule radii, and emit a deterministic fingerprint report.

Reproduce:

    cd cpp/v6
    PYTHONPATH=build/python:python python3 \
        experiments/scripts/phase0_verify_robots.py \
        --json experiments/results_new/phase0_robot_audit.json

Acceptance gate (Phase 0.4):
  * All three robots load successfully.
  * Every active link has radius > 0 (no silent zero-radius capsules).
  * Fingerprint is deterministic across two consecutive loads.
  * (Optional) FK on 200 Halton-sample configurations completes without
    NaN / inf.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

REPO_V6 = Path(__file__).resolve().parents[2]
DATA_DIR = REPO_V6 / "data"
ROBOTS = ["iiwa14", "panda", "ur10"]


def import_sbf():
    sys.path.insert(0, str(REPO_V6 / "build" / "python"))
    sys.path.insert(0, str(REPO_V6 / "python"))
    import sbf6  # type: ignore
    return sbf6


def halton(i: int, base: int) -> float:
    f, r = 1.0, 0.0
    while i > 0:
        f /= base
        r += f * (i % base)
        i //= base
    return r


def sample_halton_configs(n: int, joint_limits) -> list[list[float]]:
    bases = [2, 3, 5, 7, 11, 13, 17][: len(joint_limits)]
    qs = []
    for k in range(1, n + 1):
        q = [
            lo + halton(k, b) * (hi - lo)
            for (lo, hi), b in zip(joint_limits, bases)
        ]
        qs.append(q)
    return qs


def audit_robot(sbf6, name: str) -> dict:
    json_path = DATA_DIR / f"{name}.json"
    if not json_path.exists():
        return {"name": name, "ok": False, "error": f"missing {json_path}"}

    robot = sbf6.Robot.from_json(str(json_path))
    fp1 = int(robot.fingerprint())
    robot2 = sbf6.Robot.from_json(str(json_path))
    fp2 = int(robot2.fingerprint())

    radii_active = list(robot.active_link_radii())
    radii_full = list(robot.link_radii() or [])
    n_active = robot.n_active_links()
    n_joints = robot.n_joints()
    limits = [(iv.lo, iv.hi) for iv in robot.joint_limits().limits]

    zero_radius_active = [i for i, r in enumerate(radii_active) if r <= 0.0]
    fp_stable = fp1 == fp2

    report = {
        "name": name,
        "json_path": str(json_path),
        "n_joints": n_joints,
        "n_active_links": n_active,
        "has_tool": robot.has_tool(),
        "fingerprint_hex": f"0x{fp1:016x}",
        "fingerprint_stable": fp_stable,
        "link_radii_full": radii_full,
        "link_radii_active": radii_active,
        "joint_limits": limits,
        "ok": fp_stable
        and n_active > 0
        and len(zero_radius_active) == 0,
        "issues": [],
    }
    if not fp_stable:
        report["issues"].append("fingerprint not deterministic")
    if zero_radius_active:
        report["issues"].append(
            f"zero-radius active links at indices {zero_radius_active}"
        )

    # Halton FK sanity (only if FK helper is exposed; skip silently otherwise)
    qs = sample_halton_configs(200, limits)
    nan_count = 0
    if hasattr(sbf6, "compute_link_endpoints"):
        try:
            for q in qs:
                ends = sbf6.compute_link_endpoints(robot, q)
                for v in ends:
                    if not all(math.isfinite(c) for c in v):
                        nan_count += 1
                        break
            report["fk_samples_checked"] = len(qs)
            report["fk_non_finite_count"] = nan_count
            if nan_count > 0:
                report["issues"].append(f"{nan_count} non-finite FK samples")
                report["ok"] = False
        except Exception as exc:  # pragma: no cover - sanity only
            report["fk_check_error"] = repr(exc)

    return report


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--json",
        default=str(REPO_V6 / "experiments" / "results_new" / "phase0_robot_audit.json"),
    )
    args = ap.parse_args()

    sbf6 = import_sbf()
    out = {"robots": [audit_robot(sbf6, r) for r in ROBOTS]}
    out["all_ok"] = all(r.get("ok") for r in out["robots"])

    Path(args.json).parent.mkdir(parents=True, exist_ok=True)
    Path(args.json).write_text(json.dumps(out, indent=2))

    for r in out["robots"]:
        flag = "OK " if r.get("ok") else "FAIL"
        print(
            f"[{flag}] {r['name']:7s} "
            f"joints={r.get('n_joints')} active={r.get('n_active_links')} "
            f"fp={r.get('fingerprint_hex')} "
            f"radii_active={r.get('link_radii_active')}"
        )
        for issue in r.get("issues", []):
            print(f"       - {issue}")
    print(f"\nReport: {args.json}")
    return 0 if out["all_ok"] else 1


if __name__ == "__main__":
    sys.exit(main())

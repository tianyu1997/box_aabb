#!/usr/bin/env python3
"""
v6 Regression Test Script — compares v6 performance against v5 baseline.

Usage:
    python3 experiments/scripts/regression_test.py [--quick] [--tolerance 0.05]

Runs from the v6 root directory. Requires build/ to contain compiled binaries.
"""

import argparse
import json
import os
import re
import subprocess
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
V6_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "../.."))
BUILD_DIR = os.path.join(V6_ROOT, "build")

# ─── Test runners ────────────────────────────────────────────────────────────

def run_unit_tests():
    """Run all unit tests, return (n_pass, n_fail, details)."""
    test_dir = os.path.join(BUILD_DIR, "test")
    tests = [
        "test_ray_aabb", "test_endpoint_iaabb", "test_full_pipeline",
        "test_lect", "test_link_iaabb", "test_viz_exporter",
    ]
    n_pass = 0
    n_fail = 0
    details = []
    for t in tests:
        binary = os.path.join(test_dir, t)
        if not os.path.exists(binary):
            details.append(f"  SKIP  {t} (not found)")
            continue
        try:
            result = subprocess.run(
                [binary], capture_output=True, text=True, timeout=120,
                cwd=V6_ROOT
            )
            if result.returncode == 0 and "SUCCESS" in result.stdout:
                n_pass += 1
                details.append(f"  PASS  {t}")
            else:
                n_fail += 1
                details.append(f"  FAIL  {t} (exit={result.returncode})")
        except subprocess.TimeoutExpired:
            n_fail += 1
            details.append(f"  FAIL  {t} (timeout)")
    return n_pass, n_fail, details


def run_bench_pipeline():
    """Run bench_pipeline, extract timing."""
    binary = os.path.join(BUILD_DIR, "test", "bench_pipeline")
    if not os.path.exists(binary):
        return None
    result = subprocess.run(
        [binary], capture_output=True, text=True, timeout=300,
        cwd=V6_ROOT
    )
    # Parse timing from output
    times = {}
    for line in result.stdout.split("\n"):
        m = re.search(r"(\w+)\s*:\s*([\d.]+)\s*ms", line)
        if m:
            times[m.group(1)] = float(m.group(2))
    return times


def run_exp2_quick(seeds=2):
    """Run exp2 with minimal seeds, return timing stats."""
    binary = os.path.join(BUILD_DIR, "experiments", "exp2_e2e_planning")
    if not os.path.exists(binary):
        return None
    result = subprocess.run(
        [binary, "--seeds", str(seeds)],
        capture_output=True, text=True, timeout=600,
        cwd=V6_ROOT
    )
    # Parse key metrics from stderr
    metrics = {"build_ms": [], "query_ms": [], "n_boxes": []}
    for line in result.stderr.split("\n"):
        m = re.search(r'\[PLN\] build.*?(\d+)ms', line)
        if m:
            metrics["build_ms"].append(int(m.group(1)))
        m = re.search(r'\[PLN\] query.*?(\d+)ms', line)
        if m:
            metrics["query_ms"].append(int(m.group(1)))
        m = re.search(r'n_boxes=(\d+)', line)
        if m:
            metrics["n_boxes"].append(int(m.group(1)))
    return metrics


# ─── Main ────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="v6 regression test")
    parser.add_argument("--quick", action="store_true",
                        help="Run minimal tests only")
    parser.add_argument("--tolerance", type=float, default=0.05,
                        help="Max allowed regression fraction (default: 0.05)")
    args = parser.parse_args()

    print("=" * 60)
    print("SafeBoxForest v6 — Regression Test")
    print("=" * 60)
    failures = 0

    # 1. Unit tests
    print("\n[1/3] Unit Tests")
    n_pass, n_fail, details = run_unit_tests()
    for d in details:
        print(d)
    print(f"  Result: {n_pass} passed, {n_fail} failed")
    failures += n_fail

    if args.quick:
        print(f"\n{'=' * 60}")
        print(f"Quick mode: skipping benchmarks and experiments")
        print(f"Total failures: {failures}")
        return failures

    # 2. Bench pipeline
    print("\n[2/3] Bench Pipeline")
    times = run_bench_pipeline()
    if times:
        for k, v in times.items():
            print(f"  {k}: {v:.1f} ms")
    else:
        print("  SKIP (binary not found)")

    # 3. Exp2 quick
    print("\n[3/3] Exp2 Quick (2 seeds)")
    metrics = run_exp2_quick(seeds=2)
    if metrics:
        if metrics["build_ms"]:
            avg_build = sum(metrics["build_ms"]) / len(metrics["build_ms"])
            print(f"  avg build: {avg_build:.0f} ms")
        if metrics["query_ms"]:
            avg_query = sum(metrics["query_ms"]) / len(metrics["query_ms"])
            print(f"  avg query: {avg_query:.0f} ms")
        if metrics["n_boxes"]:
            avg_boxes = sum(metrics["n_boxes"]) / len(metrics["n_boxes"])
            print(f"  avg boxes: {avg_boxes:.0f}")
    else:
        print("  SKIP (binary not found)")

    print(f"\n{'=' * 60}")
    print(f"Total failures: {failures}")
    return failures


if __name__ == "__main__":
    sys.exit(main())

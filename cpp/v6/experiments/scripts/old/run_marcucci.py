#!/usr/bin/env python3
"""
run_marcucci.py — Run Marcucci Science Robotics benchmark experiments.

Builds and runs exp1/exp2/exp3 C++ experiments, collects results.

Usage:
    python run_marcucci.py [--exp 1|2|3|all] [--quick] [--build-dir DIR]
"""

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent.resolve()  # experiments/scripts/
EXPERIMENTS_DIR = SCRIPT_DIR.parent            # experiments/
V5_DIR = EXPERIMENTS_DIR.parent                # cpp/v5/


def get_build_dir(custom: str | None = None) -> Path:
    """Resolve build directory."""
    if custom:
        return Path(custom).resolve()
    # Check common locations
    for candidate in [V5_DIR / "build", V5_DIR / "cmake-build-release",
                      V5_DIR.parent / "build"]:
        if (candidate / "experiments").is_dir():
            return candidate
    return V5_DIR / "build"


def ensure_built(build_dir: Path, cmake_args: list[str] | None = None):
    """Configure and build if needed."""
    if not build_dir.exists():
        build_dir.mkdir(parents=True, exist_ok=True)

    cmake_cache = build_dir / "CMakeCache.txt"
    if not cmake_cache.exists():
        cmd = ["cmake", "-S", str(V5_DIR), "-B", str(build_dir),
               "-DCMAKE_BUILD_TYPE=Release",
               "-DSBF_BUILD_EXPERIMENTS=ON",
               "-DSBF_BUILD_TESTS=OFF"]
        if cmake_args:
            cmd.extend(cmake_args)
        print(f"[cmake configure] {' '.join(cmd)}")
        subprocess.check_call(cmd)

    nproc = os.cpu_count() or 4
    cmd = ["cmake", "--build", str(build_dir), "--parallel", str(nproc)]
    print(f"[cmake build] {' '.join(cmd)}")
    subprocess.check_call(cmd)


def run_experiment(build_dir: Path, exp_name: str, extra_args: list[str]):
    """Run a single experiment executable."""
    exe = build_dir / "experiments" / exp_name
    if not exe.exists():
        # Try bin/ layout
        exe = build_dir / "bin" / exp_name
    if not exe.exists():
        print(f"  ERROR: {exp_name} not found in {build_dir}")
        return False

    cmd = [str(exe)] + extra_args
    print(f"\n{'='*72}")
    print(f"  Running: {exp_name}")
    print(f"  Command: {' '.join(cmd)}")
    print(f"{'='*72}\n")

    t0 = time.time()
    result = subprocess.run(cmd, cwd=str(build_dir))
    elapsed = time.time() - t0

    print(f"\n  [{exp_name}] finished in {elapsed:.1f}s  "
          f"(exit code {result.returncode})")
    return result.returncode == 0


def main():
    parser = argparse.ArgumentParser(description="Run Marcucci benchmark experiments")
    parser.add_argument("--exp", default="all",
                        help="Which experiment: 1, 2, 3, or 'all' (default: all)")
    parser.add_argument("--quick", action="store_true",
                        help="Use --quick flag for faster runs")
    parser.add_argument("--build-dir", type=str, default=None,
                        help="Custom build directory")
    parser.add_argument("--seeds", type=int, default=None,
                        help="Number of seeds")
    parser.add_argument("--threads", type=int, default=None,
                        help="Number of threads")
    parser.add_argument("--preset", type=str, default=None,
                        help="Preset name for exp1 (A/B/C/D/E/F/all)")
    parser.add_argument("--gcs", action="store_true",
                        help="Use GCS solver for exp2")
    parser.add_argument("--no-build", action="store_true",
                        help="Skip cmake build step")
    args = parser.parse_args()

    build_dir = get_build_dir(args.build_dir)

    if not args.no_build:
        try:
            ensure_built(build_dir)
        except subprocess.CalledProcessError as e:
            print(f"Build failed: {e}")
            sys.exit(1)

    experiments = []
    if args.exp in ("all", "1"):
        extra = []
        if args.quick: extra.append("--quick")
        if args.seeds: extra.extend(["--seeds", str(args.seeds)])
        if args.threads: extra.extend(["--threads", str(args.threads)])
        if args.preset: extra.extend(["--preset", args.preset])
        experiments.append(("exp1_coverage", extra))

    if args.exp in ("all", "2"):
        extra = []
        if args.quick: extra.append("--quick")
        if args.seeds: extra.extend(["--seeds", str(args.seeds)])
        if args.threads: extra.extend(["--threads", str(args.threads)])
        if args.gcs: extra.append("--gcs")
        experiments.append(("exp2_e2e_planning", extra))

    if args.exp in ("all", "3"):
        extra = []
        if args.quick: extra.append("--quick")
        if args.seeds: extra.extend(["--seeds", str(args.seeds)])
        if args.threads: extra.extend(["--threads", str(args.threads)])
        experiments.append(("exp3_incremental", extra))

    if not experiments:
        print(f"Unknown experiment: {args.exp}")
        sys.exit(1)

    results = {}
    for name, extra in experiments:
        ok = run_experiment(build_dir, name, extra)
        results[name] = ok

    print(f"\n{'='*72}")
    print("  Results Summary:")
    for name, ok in results.items():
        status = "PASS" if ok else "FAIL"
        print(f"    {name}: {status}")
    print(f"{'='*72}")

    if not all(results.values()):
        sys.exit(1)


if __name__ == "__main__":
    main()

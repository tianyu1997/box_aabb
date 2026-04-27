#!/usr/bin/env python3
"""Shared helpers for v6 paper experiment scripts.

The paper policy is one top-level script per experimental environment.
This module is not an experiment by itself; it only standardises paths,
logging, and subprocess execution.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Iterable, Sequence

ROOT = Path(__file__).resolve().parents[2]
BUILD_RELEASE = ROOT / "build-release"
BUILD_DEBUG = ROOT / "build"
CFG = ROOT / "experiments" / "configs"
OUT_DEFAULT = ROOT / "experiments" / "results_paper"
PYTHON_BUILD = ROOT / "build" / "python"
PYTHON_SRC = ROOT / "python"


def _normalize_build_dir(build_dir: Path) -> Path:
    return build_dir if build_dir.is_absolute() else (ROOT / build_dir)


def _cmake_build_type(build_dir: Path) -> str | None:
    value = _cmake_cache_value(build_dir, "CMAKE_BUILD_TYPE")
    return value or None


def _cmake_cache_value(build_dir: Path, key: str) -> str | None:
    cache_path = build_dir / "CMakeCache.txt"
    if not cache_path.is_file():
        return None
    for line in cache_path.read_text().splitlines():
        prefix = f"{key}:"
        if line.startswith(prefix) and "=" in line:
            value = line.split("=", 1)[1].strip()
            return value or None
    return None


def _ensure_non_debug_build(build_dir: Path, *, allow_debug: bool) -> None:
    if allow_debug:
        return
    if _cmake_build_type(build_dir) == "Debug":
        raise RuntimeError(
            "Refusing to run paper experiments against Debug binaries at "
            f"{build_dir}. Build Release binaries in build-release or pass "
            "--allow-debug-build for debugging-only runs."
        )


def _resolve_build_dir(args: argparse.Namespace) -> Path:
    cached = getattr(args, "_resolved_build_dir", None)
    if cached is not None:
        return cached

    requested = getattr(args, "build_dir", None)
    allow_debug = bool(getattr(args, "allow_debug_build", False))
    if requested is not None:
        build_dir = _normalize_build_dir(requested)
    elif BUILD_RELEASE.is_dir():
        build_dir = BUILD_RELEASE
    elif BUILD_DEBUG.is_dir():
        build_dir = BUILD_DEBUG
    else:
        build_dir = BUILD_RELEASE

    _ensure_non_debug_build(build_dir, allow_debug=allow_debug)
    args._resolved_build_dir = build_dir
    return build_dir


def add_common_args(parser: argparse.ArgumentParser) -> None:
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--quick", action="store_true", help="3 seeds / short timeout smoke mode")
    mode.add_argument("--full", action="store_true", help="paper mode; script-specific seed defaults")
    parser.add_argument(
        "--build-dir",
        type=Path,
        default=None,
        help="CMake build directory; defaults to build-release when available.",
    )
    parser.add_argument(
        "--allow-debug-build",
        action="store_true",
        help="allow Debug CMake binaries for debugging-only runs; disabled by default to protect timing experiments",
    )
    parser.add_argument("--out-dir", type=Path, default=OUT_DEFAULT)
    parser.add_argument("--seeds", type=int, default=None)
    parser.add_argument("--timeout", type=int, default=None)
    parser.add_argument("--dry-run", action="store_true")


def mode_args(args: argparse.Namespace, *, quick_seeds: int = 3,
              full_seeds: int = 20, quick_timeout: int = 30,
              full_timeout: int = 120) -> tuple[int, int, str]:
    if args.quick:
        seeds = quick_seeds if args.seeds is None else args.seeds
        timeout = quick_timeout if args.timeout is None else args.timeout
        mode = "--quick"
    else:
        seeds = full_seeds if args.seeds is None else args.seeds
        timeout = full_timeout if args.timeout is None else args.timeout
        mode = "--full" if args.full else ""
    return seeds, timeout, mode


def run(
    cmd: Sequence[str | Path], *, dry_run: bool = False,
    env: dict[str, str] | None = None,
    cwd: Path | None = None,
) -> None:
    text = " ".join(str(c) for c in cmd)
    print(f"$ {text}")
    if dry_run:
        return
    subprocess.run([str(c) for c in cmd], check=True, cwd=cwd or ROOT, env=env)


def python_env(extra_pythonpath: Iterable[Path] = ()) -> dict[str, str]:
    env = os.environ.copy()
    pythonpath_parts: list[str] = []
    for candidate in (PYTHON_BUILD, PYTHON_SRC, *extra_pythonpath):
        if Path(candidate).exists():
            pythonpath_parts.append(str(candidate))
    existing = env.get("PYTHONPATH")
    if existing:
        pythonpath_parts.append(existing)
    env["PYTHONPATH"] = os.pathsep.join(pythonpath_parts)
    return env


def require_python_extension(args: argparse.Namespace) -> Path:
    build_dir = _resolve_build_dir(args)
    with_python = _cmake_cache_value(build_dir, "SBF_WITH_PYTHON")
    if with_python == "OFF":
        raise RuntimeError(
            "The selected v6 build tree was configured with SBF_WITH_PYTHON=OFF. "
            f"Reconfigure {build_dir} with -DSBF_WITH_PYTHON=ON before running "
            "Python-backed paper experiments."
        )
    python_dir = build_dir / "python"
    extensions = sorted(python_dir.glob("_sbf5_cpp*.so"))
    if not extensions:
        raise FileNotFoundError(
            f"No _sbf5_cpp extension found under {python_dir}; build the Release "
            "Python binding target before running paper experiments."
        )
    return python_dir


def python_cmd(script: Path, *script_args: str | Path) -> list[str | Path]:
    return [sys.executable, script, *script_args]


def run_python(
    script: Path,
    script_args: Sequence[str | Path], *,
    dry_run: bool = False,
) -> None:
    run(
        python_cmd(script, *script_args),
        dry_run=dry_run,
        env=python_env(),
        cwd=ROOT,
    )


def bin_path(args: argparse.Namespace, name: str) -> Path:
    build_dir = _resolve_build_dir(args)
    binary = build_dir / "experiments" / name
    if not binary.is_file():
        raise FileNotFoundError(
            f"{name} not found at {binary}; build the Release target first or pass "
            "--build-dir to a different CMake build tree."
        )
    return binary


def write_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n")


def load_json(path: Path) -> dict:
    return json.loads(path.read_text())


def existing(paths: Iterable[Path]) -> list[Path]:
    return [p for p in paths if p.exists()]

#!/usr/bin/env python3
"""Reusable same-robot scene sequence for the lifelong LECT cache workflow.

This utility sits above the raw experiment binaries and exposes the scene-reuse
workflow claimed by the paper: one persisted LECT cache is reused across
independent `exp_main` processes as long as the robot asset is unchanged.

Default sequence:
  iiwa14_far -> iiwa14_narrow

Outputs:
  - experiments/results_paper/cache_reuse/<scene>.json
  - experiments/results_paper/cache_reuse.json
"""
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import CFG, ROOT, add_common_args, bin_path, load_json, mode_args, write_json


def run_capture(cmd: list[str | Path], *, dry_run: bool) -> subprocess.CompletedProcess[str] | None:
    text = " ".join(str(part) for part in cmd)
    print(f"$ {text}")
    if dry_run:
        return None
    completed = subprocess.run(
        [str(part) for part in cmd],
        check=True,
        cwd=ROOT,
        text=True,
        capture_output=True,
    )
    if completed.stdout:
        print(completed.stdout, end="")
    if completed.stderr:
        print(completed.stderr, end="", file=sys.stderr)
    return completed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    add_common_args(parser)
    parser.add_argument(
        "--scenes",
        nargs="+",
        default=["iiwa14_far", "iiwa14_narrow"],
        help="ordered same-robot scene names under experiments/configs/",
    )
    parser.add_argument("--cache-path", type=Path, default=None)
    parser.add_argument("--keep-existing-cache", action="store_true")
    parser.add_argument("--threads", type=int, default=1)
    parser.add_argument("--env", default="link_iaabb_grid")
    parser.add_argument("--n-sub", type=int, default=4)
    parser.add_argument("--ffb-depth", type=int, default=55)
    parser.add_argument("--max-boxes", type=int, default=2500)
    parser.add_argument("--bridge-boxes", type=int, default=2000)
    parser.add_argument("--point-bridge-timeout-ms", type=float, default=20000.0)
    parser.add_argument("--no-point-bridge", action="store_true")
    return parser.parse_args()


def scene_path(scene_name: str) -> Path:
    return CFG / f"{scene_name}.json"


def main() -> None:
    args = parse_args()
    seeds, timeout, mode = mode_args(args, quick_seeds=1, full_seeds=3,
                                     quick_timeout=15, full_timeout=30)

    scenes = []
    robots = set()
    for name in args.scenes:
        cfg_path = scene_path(name)
        if not cfg_path.exists():
            raise FileNotFoundError(f"scene config missing: {cfg_path}")
        cfg = load_json(cfg_path)
        robots.add(str(cfg["robot"]))
        scenes.append((name, cfg_path, cfg))
    if len(robots) != 1:
        raise ValueError(f"scene reuse requires one robot asset, got: {sorted(robots)}")

    cache_path = args.cache_path
    if cache_path is None:
        cache_path = args.out_dir / "cache_reuse" / "lect_scene_reuse.bin"
    aggregate_path = args.out_dir / "cache_reuse.json"

    if not args.dry_run and not args.keep_existing_cache and cache_path.exists():
        cache_path.unlink()

    scene_results: list[dict[str, Any]] = []
    for index, (name, cfg_path, cfg) in enumerate(scenes):
        out_path = args.out_dir / "cache_reuse" / f"{name}.json"
        existed_before = cache_path.exists() if not args.dry_run else (index > 0)
        cmd: list[str | Path] = [bin_path(args, "exp_main")]
        if mode:
            cmd.append(mode)
        cmd += [
            f"--scene={cfg_path}",
            f"--out={out_path}",
            f"--seeds={seeds}",
            f"--timeout={timeout}",
            f"--threads={args.threads}",
            f"--env={args.env}",
            f"--n-sub={args.n_sub}",
            f"--ffb-depth={args.ffb_depth}",
            f"--max-boxes={args.max_boxes}",
            f"--bridge-boxes={args.bridge_boxes}",
            f"--point-bridge-timeout-ms={args.point_bridge_timeout_ms}",
            f"--lect-cache={cache_path}",
        ]
        cmd.append("--no-point-bridge" if args.no_point_bridge else "--point-bridge")
        completed = run_capture(cmd, dry_run=args.dry_run)

        if args.dry_run:
            scene_results.append({
                "scene": name,
                "scene_path": str(cfg_path),
                "robot": cfg["robot"],
                "cache_path": str(cache_path),
                "cache_existed_before": existed_before,
                "expected_cache_load": existed_before,
                "out": str(out_path),
            })
            continue

        raw = load_json(out_path)
        stdout = completed.stdout if completed is not None else ""
        scene_results.append({
            "scene": name,
            "scene_path": str(cfg_path),
            "robot": raw.get("robot", cfg["robot"]),
            "cache_path": str(cache_path),
            "cache_existed_before": existed_before,
            "loaded_lect_cache": "[lect-cache] loaded " in stdout,
            "started_new_cache": "[lect-cache] starting new cache" in stdout,
            "cache_size_bytes": cache_path.stat().st_size if cache_path.exists() else 0,
            "out": str(out_path),
            "summary": raw.get("summary", {}),
        })

    aggregate = {
        "experiment": "lifelong_cache_reuse",
        "robot": next(iter(robots)),
        "quick": args.quick,
        "cache_path": str(cache_path),
        "scenes": [result["scene"] for result in scene_results],
        "results": scene_results,
    }
    if not args.dry_run:
        write_json(aggregate_path, aggregate)
        print(f"[write] {aggregate_path}")


if __name__ == "__main__":
    main()
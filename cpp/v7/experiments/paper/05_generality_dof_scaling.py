#!/usr/bin/env python3
"""Paper Exp. 5 — manifest-backed true active-DoF scaling sweep.

This runner is intentionally manifest-driven. The current experiment binaries
accept a concrete scene JSON, but they do not yet expose an active-DoF or
joint-mask CLI. The scaling sweep therefore treats each active-DoF slice as an
explicit prepared scene config plus a matching scene-specific robot asset in
which inactive joints are frozen by zero-width limits.

Inputs:
  - a manifest JSON listing active_dof points and their scene JSONs
  - the existing exp_main binary

Outputs:
  - experiments/results_paper/dof_scaling/raw/<label>.json
  - experiments/results_paper/dof_scaling.json

Combined output schema:
  {
    "experiment": "dof_scaling",
        "scene_families": ["iiwa14_far", "iiwa14_narrow"],
    "cells": [
      {
                "scene_family": "iiwa14_far",
        "active_dof": 7,
        "label": "iiwa14_far_dof7",
        "scene": ".../iiwa14_far_dof7.json",
        "summary": {
          "success_rate": 1.0,
          "point_bridge_rate": 0.2,
          "median_n_boxes": 123,
          "median_n_islands": 1,
          "median_grow_time_ms": 42.0,
          "median_path_find_time_ms": 1.2,
          "median_opt_time_ms": 0.3,
          "median_total_time_ms": 44.0,
          "median_raw_length": 1.7,
          "median_opt_length": 1.5
        },
        "trials": [...]
      }
    ]
  }
"""
from __future__ import annotations

import argparse
import statistics
import sys
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import ROOT, add_common_args, bin_path, load_json, mode_args, run, write_json


def default_manifest_path() -> Path:
    return ROOT / "experiments" / "paper" / "dof_scaling_manifest.json"


def infer_scene_family(scene_name: str) -> str:
    if "_dof" in scene_name:
        return scene_name.rsplit("_dof", 1)[0]
    return scene_name


def template_manifest() -> dict[str, Any]:
    cfg_dir = ROOT / "experiments" / "configs" / "dof_scaling"
    return {
        "experiment": "dof_scaling",
        "asset_mode": "freeze_inactive_joints",
        "scene_families": ["iiwa14_far", "iiwa14_narrow"],
        "notes": [
            "Each active_dof point must resolve to a concrete scene JSON.",
            "Each scene uses a matching full-IIWA robot asset with inactive joints frozen at the base scene start posture.",
            "This manifest is the paper-side interface until a native active-DoF CLI exists.",
        ],
        "cells": [
            {
                "scene_family": scene_family,
                "active_dof": dof,
                "label": f"{scene_family}_dof{dof}",
                "scene": str(cfg_dir / f"{scene_family}_dof{dof}.json"),
                "enabled": False,
            }
            for scene_family in ["iiwa14_far", "iiwa14_narrow"]
            for dof in [2, 4, 6, 7]
        ],
    }


def parse_manifest(path: Path) -> dict[str, Any]:
    return load_json(path)


def success_trials(trials: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [trial for trial in trials if trial.get("success")]


def median_field(trials: list[dict[str, Any]], key: str) -> float:
    values = [float(trial.get(key, 0.0)) for trial in trials]
    return float(statistics.median(values)) if values else 0.0


def median_success_field(trials: list[dict[str, Any]], key: str) -> float:
    values = [float(trial.get(key, 0.0)) for trial in success_trials(trials)]
    return float(statistics.median(values)) if values else 0.0


def point_bridge_rate(trials: list[dict[str, Any]]) -> float:
    succ = success_trials(trials)
    if not succ:
        return 0.0
    used = sum(1 for trial in succ if trial.get("used_point_bridge"))
    return used / len(succ)


def augmented_trials(active_dof: int, label: str, scene_family: str,
                     scene_name: str, raw: dict[str, Any]) -> list[dict[str, Any]]:
    trials = []
    for seed_index, trial in enumerate(raw.get("trials", [])):
        item = dict(trial)
        item["seed_index"] = seed_index
        item["active_dof"] = active_dof
        item["label"] = label
        item["scene_family"] = scene_family
        item["scene_name"] = scene_name
        trials.append(item)
    return trials


def summarise_trials(trials: list[dict[str, Any]]) -> dict[str, Any]:
    n_trials = len(trials)
    n_success = len(success_trials(trials))
    return {
        "n_trials": n_trials,
        "n_success": n_success,
        "success_rate": (n_success / n_trials) if n_trials else 0.0,
        "point_bridge_rate": point_bridge_rate(trials),
        "median_n_boxes": median_field(trials, "n_boxes"),
        "median_n_islands": median_field(trials, "n_islands"),
        "median_grow_time_ms": median_field(trials, "grow_time_ms"),
        "median_path_find_time_ms": median_field(trials, "path_find_time_ms"),
        "median_opt_time_ms": median_field(trials, "opt_time_ms"),
        "median_total_time_ms": median_field(trials, "total_time_ms"),
        "median_raw_length": median_success_field(trials, "raw_length"),
        "median_opt_length": median_success_field(trials, "opt_length"),
    }


def cell_value(cell: dict[str, Any], key: str, default: Any) -> Any:
    value = cell.get(key)
    return default if value is None else value


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_common_args(parser)
    parser.add_argument("--manifest", type=Path, default=default_manifest_path())
    parser.add_argument("--write-template", type=Path, default=None,
                        help="write a manifest template and exit")
    parser.add_argument("--threads", type=int, default=1)
    parser.add_argument("--env", default="link_iaabb_grid")
    parser.add_argument("--n-sub", type=int, default=4)
    parser.add_argument("--ffb-depth", type=int, default=55)
    parser.add_argument("--max-boxes", type=int, default=2500)
    parser.add_argument("--bridge-boxes", type=int, default=2000)
    parser.add_argument("--point-bridge-timeout-ms", type=float, default=20000.0)
    parser.add_argument("--no-point-bridge", action="store_true")
    parser.add_argument("--resume", action="store_true",
                        help="skip cells whose raw JSON already exists")
    args = parser.parse_args()

    if args.write_template is not None:
        write_json(args.write_template, template_manifest())
        print(f"[write-template] {args.write_template}")
        return

    seeds, timeout, mode = mode_args(args, quick_seeds=3, full_seeds=5,
                                     quick_timeout=30, full_timeout=30)
    manifest = parse_manifest(args.manifest)
    raw_dir = args.out_dir / "dof_scaling" / "raw"
    cells_out = []
    enabled_cells = [cell for cell in manifest.get("cells", [])
                     if cell.get("enabled", True)]

    if not enabled_cells:
        print(f"[skip] no enabled DoF-scaling cells in {args.manifest}")
        return

    for cell in enabled_cells:
        active_dof = int(cell["active_dof"])
        label = str(cell.get("label") or f"dof{active_dof}")
        scene = Path(str(cell["scene"]))
        if not scene.exists():
            if args.dry_run:
                print(f"[dry-run] missing scene, skipped: {scene}")
                continue
            raise FileNotFoundError(f"manifest scene missing: {scene}")

        out_path = raw_dir / f"{label}.json"
        if not (args.resume and out_path.exists()):
            cmd: list[str | Path] = [bin_path(args, "exp_main")]
            if mode:
                cmd.append(mode)
            cmd += [
                f"--scene={scene}",
                f"--out={out_path}",
                f"--seeds={cell_value(cell, 'seeds', seeds)}",
                f"--timeout={cell_value(cell, 'timeout', timeout)}",
                f"--threads={cell_value(cell, 'threads', args.threads)}",
                f"--env={cell_value(cell, 'env', args.env)}",
                f"--n-sub={cell_value(cell, 'n_sub', args.n_sub)}",
                f"--ffb-depth={cell_value(cell, 'ffb_depth', args.ffb_depth)}",
                f"--max-boxes={cell_value(cell, 'max_boxes', args.max_boxes)}",
                f"--bridge-boxes={cell_value(cell, 'bridge_boxes', args.bridge_boxes)}",
                f"--point-bridge-timeout-ms={cell_value(cell, 'point_bridge_timeout_ms', args.point_bridge_timeout_ms)}",
            ]
            cmd.append("--no-point-bridge" if cell_value(cell, "no_point_bridge", args.no_point_bridge)
                       else "--point-bridge")
            run(cmd, dry_run=args.dry_run)

        if args.dry_run:
            continue
        raw = load_json(out_path)
        scene_name = str(raw.get("scene") or scene.stem)
        scene_family = str(cell.get("scene_family") or infer_scene_family(scene_name))
        trials = augmented_trials(active_dof, label, scene_family, scene_name, raw)
        cells_out.append({
            "scene_family": scene_family,
            "active_dof": active_dof,
            "label": label,
            "scene": str(scene),
            "robot": raw.get("robot"),
            "scene_name": scene_name,
            "seeds": raw.get("seeds", len(raw.get("trials", []))),
            "summary": summarise_trials(trials),
            "trials": trials,
        })

    if args.dry_run:
        print("[dry-run] commands emitted; no combined dof_scaling.json written")
        return

    cells_out.sort(key=lambda cell: (cell["scene_family"], cell["active_dof"], cell["label"]))
    scene_families = manifest.get("scene_families") or sorted({cell["scene_family"] for cell in cells_out})
    out = {
        "experiment": "dof_scaling",
        "schema_version": 1,
        "asset_mode": manifest.get("asset_mode", "freeze_inactive_joints"),
        "scene_family": scene_families[0] if len(scene_families) == 1 else "multiple",
        "scene_families": scene_families,
        "manifest": str(args.manifest),
        "defaults": {
            "seeds": seeds,
            "timeout": timeout,
            "threads": args.threads,
            "env": args.env,
            "n_sub": args.n_sub,
            "ffb_depth": args.ffb_depth,
            "max_boxes": args.max_boxes,
            "bridge_boxes": args.bridge_boxes,
            "point_bridge_timeout_ms": args.point_bridge_timeout_ms,
            "point_bridge_enabled": not args.no_point_bridge,
        },
        "cells": cells_out,
    }
    write_json(args.out_dir / "dof_scaling.json", out)
    print(f"[write] {args.out_dir / 'dof_scaling.json'}")


if __name__ == "__main__":
    main()
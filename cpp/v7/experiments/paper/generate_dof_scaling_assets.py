#!/usr/bin/env python3
"""Generate executable true active-DoF IIWA scaling assets.

This script creates two synchronized asset families:

1. scene-specific robot data files under data/<scene>_dof{N}.json
2. scene configs under experiments/configs/dof_scaling/<scene>_dof{N}.json

The generated assets keep the full 7-DoF IIWA kinematic chain intact. Active
DoF is reduced by freezing inactive joints at the base scene's start posture
via zero-width joint limits. This yields a true lower-dimensional slice of the
same robot model rather than a prefix-truncated proxy robot.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / "data"
CFG = ROOT / "experiments" / "configs"
DOF_CFG = CFG / "dof_scaling"

DOF_POINTS = [2, 4, 6, 7]
SCENES = ["iiwa14_far", "iiwa14_narrow"]


def load_json(path: Path) -> dict:
    return json.loads(path.read_text())


def write_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n")


def active_joint_indices(dof: int) -> list[int]:
    return list(range(dof))


def frozen_joint_indices(base: dict, dof: int) -> list[int]:
    return list(range(dof, len(base["joint_limits"])))


def active_dof_robot(base_robot: dict, base_scene: dict, scene_name: str, dof: int) -> dict:
    q_ref = list(base_scene["q_start"])
    joint_limits = []
    for idx, lim in enumerate(base_robot["joint_limits"]):
        if idx < dof:
            joint_limits.append(lim)
        else:
            joint_limits.append([q_ref[idx], q_ref[idx]])
    return {
        "name": f"iiwa14_{scene_name}_active_{dof}dof",
        "asset_mode": "freeze_inactive_joints",
        "base_robot": "iiwa14",
        "base_scene": scene_name,
        "active_dof": dof,
        "active_joint_indices": active_joint_indices(dof),
        "frozen_joint_indices": frozen_joint_indices(base_robot, dof),
        "frozen_joint_values": q_ref,
        "dh_params": base_robot["dh_params"],
        "joint_limits": joint_limits,
        "tool_frame": base_robot.get("tool_frame", {"alpha": 0.0, "a": 0.0, "d": 0.2, "theta": 0.0}),
        "link_radii": base_robot.get("link_radii", []),
    }


def active_dof_scene(base_scene: dict, scene_name: str, dof: int) -> dict:
    q_start = list(base_scene["q_start"])
    q_goal = list(base_scene["q_goal"])
    for idx in range(dof, len(q_goal)):
        q_goal[idx] = q_start[idx]
    return {
        "name": f"{scene_name}_dof{dof}",
        "robot": f"{scene_name}_dof{dof}",
        "asset_mode": "freeze_inactive_joints",
        "active_dof": dof,
        "active_joint_indices": active_joint_indices(dof),
        "q_start": q_start,
        "q_goal": q_goal,
        "obstacles": base_scene.get("obstacles", []),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--force", action="store_true",
                        help="overwrite existing generated assets")
    args = parser.parse_args()

    base_robot = load_json(DATA / "iiwa14.json")

    written = []
    for scene_name in SCENES:
        base_scene = load_json(CFG / f"{scene_name}.json")
        for dof in DOF_POINTS:
            robot_path = DATA / f"{scene_name}_dof{dof}.json"
            scene_path = DOF_CFG / f"{scene_name}_dof{dof}.json"
            if not args.force and (robot_path.exists() or scene_path.exists()):
                raise FileExistsError(
                    f"refusing to overwrite existing assets for scene={scene_name}, DoF={dof}; rerun with --force")
            write_json(robot_path, active_dof_robot(base_robot, base_scene, scene_name, dof))
            write_json(scene_path, active_dof_scene(base_scene, scene_name, dof))
            written.extend([robot_path, scene_path])

    print("Wrote DoF-scaling assets:")
    for path in written:
        print(f"  {path}")


if __name__ == "__main__":
    main()
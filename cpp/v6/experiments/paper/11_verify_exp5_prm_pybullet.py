#!/usr/bin/env python3
"""Independent collision audit for Exp.5 OMPL PRM paths using PyBullet.

Uses the open-source PyBullet physics engine (Bullet3) with the upstream
UR5 / Panda URDF + STL meshes — entirely independent from v6's own
DH-FK + capsule collision model.

Requires: pybullet (available in the 'gr00t' conda env)
Run with:
    conda run -n gr00t python experiments/paper/11_verify_exp5_prm_pybullet.py
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import tempfile
from pathlib import Path

import numpy as np
import pybullet as pb

# ---------------------------------------------------------------------------
# URDF package:// → file:// resolver
# ---------------------------------------------------------------------------

# Maps ROS package names to absolute directories on this machine
_PKG_MAP: dict[str, str] = {}  # populated in main() once we know v6 root


def _patch_urdf(urdf_path: Path) -> str:
    """Return URDF content with package:// URIs replaced by file:// URIs."""
    content = urdf_path.read_text()

    def _replace(m: re.Match) -> str:
        pkg = m.group(1)
        rel = m.group(2)
        base = _PKG_MAP.get(pkg)
        if base:
            return f'file://{base}/{rel}"'
        return m.group(0)  # leave untouched if package unknown

    return re.sub(r'package://([^/]+)/(.+?)"', _replace, content)


# ---------------------------------------------------------------------------
# PyBullet robot checker
# ---------------------------------------------------------------------------

class PybulletChecker:
    """Wraps a single PyBullet DIRECT physics client for one robot type."""

    def __init__(self, urdf_path: Path, n_dof: int) -> None:
        self._client = pb.connect(pb.DIRECT)
        pb.setGravity(0, 0, 0, physicsClientId=self._client)
        pb.setRealTimeSimulation(0, physicsClientId=self._client)

        # Write patched URDF to a temp file
        patched = _patch_urdf(urdf_path)
        self._tmp_urdf = tempfile.NamedTemporaryFile(
            suffix=".urdf", mode="w", delete=False
        )
        self._tmp_urdf.write(patched)
        self._tmp_urdf.close()

        self._robot = pb.loadURDF(
            self._tmp_urdf.name,
            useFixedBase=True,
            physicsClientId=self._client,
        )

        # Collect revolute joint indices (first n_dof of them)
        revolute = []
        for i in range(pb.getNumJoints(self._robot, physicsClientId=self._client)):
            ji = pb.getJointInfo(self._robot, i, physicsClientId=self._client)
            if ji[2] == pb.JOINT_REVOLUTE:
                revolute.append(i)
        assert len(revolute) >= n_dof, (
            f"URDF has {len(revolute)} revolute joints, need {n_dof}"
        )
        self._active_joints: list[int] = revolute[:n_dof]
        self._n_dof = n_dof

        # Track obstacle body IDs so we can remove them between scenes
        self._obstacle_ids: list[int] = []

    # -----------------------------------------------------------------------
    # Scene management
    # -----------------------------------------------------------------------

    def load_obstacles(self, obstacles: list[dict]) -> None:
        """Add AABB obstacles (each has 'lo'/'hi' or 'center'+'half_sizes')."""
        for obs in obstacles:
            if "center" in obs and "half_sizes" in obs:
                center = obs["center"]
                half = obs["half_sizes"]
            else:
                lo = obs["lo"]
                hi = obs["hi"]
                center = [(lo[k] + hi[k]) / 2.0 for k in range(3)]
                half = [(hi[k] - lo[k]) / 2.0 for k in range(3)]
            col = pb.createCollisionShape(
                pb.GEOM_BOX,
                halfExtents=half,
                physicsClientId=self._client,
            )
            body = pb.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=col,
                basePosition=center,
                physicsClientId=self._client,
            )
            self._obstacle_ids.append(body)

    def clear_obstacles(self) -> None:
        for oid in self._obstacle_ids:
            pb.removeBody(oid, physicsClientId=self._client)
        self._obstacle_ids.clear()

    # -----------------------------------------------------------------------
    # Collision queries
    # -----------------------------------------------------------------------

    def _set_joints(self, q: list[float]) -> None:
        for ji, qi in zip(self._active_joints, q):
            pb.resetJointState(
                self._robot, ji, qi, physicsClientId=self._client
            )

    def check_config(self, q: list[float]) -> bool:
        """Return True if the configuration q collides with any obstacle."""
        self._set_joints(q)
        pb.performCollisionDetection(physicsClientId=self._client)
        for oid in self._obstacle_ids:
            pts = pb.getContactPoints(
                self._robot, oid, physicsClientId=self._client
            )
            if pts:
                return True
        return False

    def check_segment(
        self,
        q0: list[float],
        q1: list[float],
        resolution: int = 64,
    ) -> bool:
        """Return True if any interpolated config along q0→q1 collides."""
        a = np.asarray(q0, dtype=float)
        b = np.asarray(q1, dtype=float)
        for step in range(resolution + 1):
            t = step / resolution
            q = (a + t * (b - a)).tolist()
            if self.check_config(q):
                return True
        return False

    def close(self) -> None:
        pb.disconnect(physicsClientId=self._client)
        try:
            os.unlink(self._tmp_urdf.name)
        except OSError:
            pass


# ---------------------------------------------------------------------------
# baseline_ompl subprocess helper (identical payload to script 10)
# ---------------------------------------------------------------------------

def _ompl_payload(scene: dict) -> dict:
    obstacles = []
    for obs in scene.get("obstacles", []):
        bounds = obs.get("bounds")
        if bounds is None:
            bounds = [*obs["lo"], *obs["hi"]]
        obstacles.append(
            {"lo": [float(v) for v in bounds[:3]], "hi": [float(v) for v in bounds[3:6]]}
        )
    return {
        "name": str(scene.get("scene_id", "scene")),
        "robot": str(scene["robot"]),
        "q_start": [float(v) for v in scene["start"]],
        "q_goal": [float(v) for v in scene["goal"]],
        "obstacles": obstacles,
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Exp.5 PRM collision audit using PyBullet (open-source, mesh-based)"
    )
    parser.add_argument(
        "--exp5-json",
        type=Path,
        default=Path("experiments/results_paper/exp5_random_robot_scenes.json"),
    )
    parser.add_argument(
        "--root",
        type=Path,
        default=Path(__file__).resolve().parents[2],
    )
    parser.add_argument(
        "--baseline-bin",
        type=Path,
        default=Path("build-release/experiments/baseline_ompl"),
    )
    parser.add_argument("--prm-build-s", type=float, default=10.0)
    parser.add_argument("--prm-query-s", type=float, default=2.0)
    parser.add_argument("--seeds", type=int, default=5)
    parser.add_argument("--segment-resolution", type=int, default=64)
    parser.add_argument(
        "--out",
        type=Path,
        default=Path("experiments/results_paper/exp5_prm_pybullet_audit.json"),
    )
    args = parser.parse_args()

    root = args.root.resolve()
    exp5_json = (root / args.exp5_json).resolve()
    baseline_bin = (root / args.baseline_bin).resolve()
    out_path = (root / args.out).resolve()

    if not exp5_json.is_file():
        raise FileNotFoundError(exp5_json)
    if not baseline_bin.is_file():
        raise FileNotFoundError(baseline_bin)

    # Populate package map now that we know root
    upstream_dir = root / "data" / "urdf" / "upstream"
    _PKG_MAP["ur_description"] = str(upstream_dir / "ur_description")
    _PKG_MAP["moveit_resources_panda_description"] = str(
        upstream_dir / "moveit_resources_panda_description"
    )

    payload = json.loads(exp5_json.read_text())
    scenes = payload.get("scenes", [])

    # Cache one PyBullet checker per robot type
    checkers: dict[str, PybulletChecker] = {}

    def _get_checker(robot: str, urdf_rel: str) -> PybulletChecker:
        if robot not in checkers:
            urdf_abs = root / urdf_rel
            n_dof = 7 if robot == "panda" else 6
            print(f"  [pybullet] loading {robot} URDF: {urdf_abs.name}", flush=True)
            checkers[robot] = PybulletChecker(urdf_abs, n_dof)
        return checkers[robot]

    total_expected = 0
    total_replayed = 0
    total_collision_free = 0
    total_cfg_collision = 0
    total_seg_collision = 0
    total_replay_fail = 0
    report_rows: list[dict] = []

    try:
        for index, scene_meta in enumerate(scenes, start=1):
            scene_file = root / scene_meta["scene_file"]
            scene = json.loads(scene_file.read_text())

            robot = scene["robot"]
            checker = _get_checker(robot, scene["robot_urdf"])

            # Expected PRM success seeds from original experiment
            method_rows = scene_meta.get("baseline_results", [])
            prm_row = next(
                (r for r in method_rows if r.get("method") == "ompl_prm"), None
            )
            expected_runs = prm_row.get("runs", []) if isinstance(prm_row, dict) else []
            expected_success = {
                int(r["seed"]) for r in expected_runs if r.get("success")
            }
            total_expected += len(expected_success)

            # Run baseline_ompl to get waypoints
            with tempfile.TemporaryDirectory(prefix="exp5_pb_audit_") as tmpdir:
                tmp = Path(tmpdir)
                scene_path = tmp / "scene.json"
                out_run = tmp / "result.json"
                scene_path.write_text(json.dumps(_ompl_payload(scene), indent=2) + "\n")
                cmd = [
                    str(baseline_bin),
                    f"--scene={scene_path}",
                    f"--out={out_run}",
                    f"--seeds={args.seeds}",
                    "--seed-base=42",
                    "--planner=prm",
                    f"--prm-build={args.prm_build_s}",
                    f"--prm-query={args.prm_query_s}",
                    f"--timeout={args.prm_build_s + args.prm_query_s}",
                ]
                try:
                    subprocess.run(
                        cmd,
                        check=True,
                        cwd=root,
                        capture_output=True,
                        text=True,
                        timeout=(args.prm_build_s + args.prm_query_s + 8.0)
                        * max(1, args.seeds),
                    )
                except (subprocess.TimeoutExpired, subprocess.CalledProcessError) as exc:
                    total_replay_fail += len(expected_success)
                    status = (
                        "replay_timeout"
                        if isinstance(exc, subprocess.TimeoutExpired)
                        else f"replay_failed_rc{exc.returncode}"
                    )
                    report_rows.append(
                        {
                            "scene_id": scene["scene_id"],
                            "robot": robot,
                            "status": status,
                        }
                    )
                    print(
                        f"[{index}/{len(scenes)}] {scene['scene_id']}: {status}",
                        flush=True,
                    )
                    continue

                replay = json.loads(out_run.read_text())

            trials = replay.get("trials", [])
            trial_by_seed = {int(t["seed"]): t for t in trials}

            # Load obstacles into this checker
            checker.load_obstacles(scene["obstacles"])

            scene_rows: list[dict] = []
            for seed in sorted(expected_success):
                trial = trial_by_seed.get(seed)
                if not trial or not trial.get("success"):
                    total_replay_fail += 1
                    total_replayed += 1
                    scene_rows.append({"seed": seed, "status": "missing_or_failed_in_replay"})
                    continue

                total_replayed += 1
                waypoints = trial.get("waypoints") or []

                cfg_hits: list[int] = []
                seg_hits: list[int] = []

                for i, q in enumerate(waypoints):
                    if checker.check_config(q):
                        cfg_hits.append(i)

                for i in range(len(waypoints) - 1):
                    if checker.check_segment(
                        waypoints[i],
                        waypoints[i + 1],
                        args.segment_resolution,
                    ):
                        seg_hits.append(i)

                ok = not cfg_hits and not seg_hits
                if ok:
                    total_collision_free += 1
                else:
                    total_cfg_collision += int(bool(cfg_hits))
                    total_seg_collision += int(bool(seg_hits))

                scene_rows.append(
                    {
                        "seed": seed,
                        "status": "ok" if ok else "collision_detected",
                        "n_waypoints": len(waypoints),
                        "config_collision_indices": cfg_hits,
                        "segment_collision_indices": seg_hits,
                    }
                )

            checker.clear_obstacles()

            scene_collisions = sum(
                1 for r in scene_rows if r.get("status") == "collision_detected"
            )
            report_rows.append(
                {
                    "scene_id": scene["scene_id"],
                    "robot": robot,
                    "difficulty": scene.get("difficulty"),
                    "status": "checked",
                    "expected_success_seeds": sorted(expected_success),
                    "results": scene_rows,
                }
            )
            print(
                f"[{index}/{len(scenes)}] {scene['scene_id']}: "
                f"checked={len(scene_rows)} collisions={scene_collisions}",
                flush=True,
            )
    finally:
        for chk in checkers.values():
            chk.close()

    summary = {
        "collision_checker": "pybullet (Bullet3 mesh-based)",
        "exp5_json": str(exp5_json.relative_to(root)),
        "baseline_bin": str(baseline_bin.relative_to(root)),
        "prm_build_s": args.prm_build_s,
        "prm_query_s": args.prm_query_s,
        "segment_resolution": args.segment_resolution,
        "expected_success_paths": total_expected,
        "replayed_paths": total_replayed,
        "collision_free_paths": total_collision_free,
        "config_collision_paths": total_cfg_collision,
        "segment_collision_paths": total_seg_collision,
        "replay_fail_paths": total_replay_fail,
    }
    report = {
        "experiment": "exp5_prm_pybullet_collision_audit",
        "summary": summary,
        "scenes": report_rows,
    }
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(report, indent=2) + "\n")
    print(f"\n[write] {out_path.relative_to(root)}", flush=True)
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Independent collision audit for Exp.5 OMPL PRM paths.

This script replays the Exp.5 PRM planner on each saved random scene and
checks returned paths with external geometry checks from exp5_scene_utils.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import tempfile
from pathlib import Path

from exp5_scene_utils import check_config_collision, check_segment_collision, load_robot_doc


def exp5_ompl_scene_payload(scene: dict) -> dict:
    obstacles = []
    for obstacle in scene.get("obstacles", []):
        bounds = obstacle.get("bounds")
        if bounds is None:
            bounds = [*obstacle["lo"], *obstacle["hi"]]
        obstacles.append(
            {
                "lo": [float(value) for value in bounds[:3]],
                "hi": [float(value) for value in bounds[3:6]],
            }
        )
    return {
        "name": str(scene.get("scene_id", "exp5_scene")),
        "robot": str(scene["robot"]),
        "q_start": [float(value) for value in scene["start"]],
        "q_goal": [float(value) for value in scene["goal"]],
        "obstacles": obstacles,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Independent collision audit for Exp.5 PRM paths")
    parser.add_argument(
        "--exp5-json",
        type=Path,
        default=Path("experiments/results_paper/exp5_random_robot_scenes.json"),
    )
    parser.add_argument(
        "--root",
        type=Path,
        default=Path(__file__).resolve().parents[2],
        help="v6 root, default inferred from this script location",
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
        default=Path("experiments/results_paper/exp5_prm_independent_collision_audit.json"),
    )
    args = parser.parse_args()

    root = args.root.resolve()
    exp5_json = (root / args.exp5_json).resolve()
    baseline_bin = (root / args.baseline_bin).resolve()
    out_path = (root / args.out).resolve()

    if not exp5_json.is_file():
        raise FileNotFoundError(f"missing exp5 payload: {exp5_json}")
    if not baseline_bin.is_file():
        raise FileNotFoundError(f"missing baseline_ompl binary: {baseline_bin}")

    payload = json.loads(exp5_json.read_text())
    scenes = payload.get("scenes", [])
    report_rows = []

    total_replayed = 0
    total_expected_success = 0
    total_collision_free = 0
    total_cfg_collision = 0
    total_seg_collision = 0
    total_replay_fail = 0

    for index, scene_meta in enumerate(scenes, start=1):
        scene_file = root / scene_meta["scene_file"]
        scene = json.loads(scene_file.read_text())
        robot_doc = load_robot_doc(root / scene["robot_json"])
        method_rows = scene_meta.get("baseline_results", [])
        prm_row = next((row for row in method_rows if row.get("method") == "ompl_prm"), None)
        expected_runs = prm_row.get("runs", []) if isinstance(prm_row, dict) else []
        expected_success = {int(run["seed"]) for run in expected_runs if run.get("success")}
        total_expected_success += len(expected_success)

        with tempfile.TemporaryDirectory(prefix="exp5_prm_audit_") as tmpdir:
            tmp = Path(tmpdir)
            scene_path = tmp / "scene.json"
            out_run = tmp / "result.json"
            scene_path.write_text(json.dumps(exp5_ompl_scene_payload(scene), indent=2) + "\n")
            cmd = [
                str(baseline_bin),
                f"--scene={scene_path}",
                f"--out={out_run}",
                f"--seeds={int(args.seeds)}",
                "--seed-base=42",
                "--planner=prm",
                f"--prm-build={float(args.prm_build_s)}",
                f"--prm-query={float(args.prm_query_s)}",
                f"--timeout={float(args.prm_build_s) + float(args.prm_query_s)}",
            ]
            try:
                subprocess.run(
                    cmd,
                    check=True,
                    cwd=root,
                    capture_output=True,
                    text=True,
                    timeout=(args.prm_build_s + args.prm_query_s + 8.0) * max(1, int(args.seeds)),
                )
            except subprocess.TimeoutExpired:
                total_replay_fail += len(expected_success)
                report_rows.append(
                    {
                        "scene_id": scene["scene_id"],
                        "robot": scene["robot"],
                        "status": "replay_timeout",
                        "expected_success_seeds": sorted(expected_success),
                    }
                )
                print(
                    f"[{index}/{len(scenes)}] {scene['scene_id']}: replay timeout "
                    f"(expected_success={len(expected_success)})",
                    flush=True,
                )
                continue
            except subprocess.CalledProcessError as exc:
                total_replay_fail += len(expected_success)
                report_rows.append(
                    {
                        "scene_id": scene["scene_id"],
                        "robot": scene["robot"],
                        "status": "replay_failed",
                        "returncode": int(exc.returncode),
                        "expected_success_seeds": sorted(expected_success),
                    }
                )
                print(
                    f"[{index}/{len(scenes)}] {scene['scene_id']}: replay failed rc={exc.returncode} "
                    f"(expected_success={len(expected_success)})",
                    flush=True,
                )
                continue

            replay = json.loads(out_run.read_text())
            trials = replay.get("trials", [])
            trial_by_seed = {int(t["seed"]): t for t in trials}
            scene_rows = []
            for seed in sorted(expected_success):
                total_replayed += 1
                trial = trial_by_seed.get(seed)
                if not trial or not trial.get("success"):
                    total_replay_fail += 1
                    scene_rows.append(
                        {
                            "seed": seed,
                            "status": "missing_or_failed_in_replay",
                        }
                    )
                    continue
                waypoints = trial.get("waypoints") or []
                cfg_hits = []
                seg_hits = []
                for i, q in enumerate(waypoints):
                    if check_config_collision(robot_doc, scene["obstacles"], q):
                        cfg_hits.append(i)
                for i in range(len(waypoints) - 1):
                    if check_segment_collision(
                        robot_doc,
                        scene["obstacles"],
                        waypoints[i],
                        waypoints[i + 1],
                        int(args.segment_resolution),
                    ):
                        seg_hits.append(i)

                ok = (len(cfg_hits) == 0) and (len(seg_hits) == 0)
                if ok:
                    total_collision_free += 1
                else:
                    total_cfg_collision += int(len(cfg_hits) > 0)
                    total_seg_collision += int(len(seg_hits) > 0)
                scene_rows.append(
                    {
                        "seed": seed,
                        "status": "ok" if ok else "collision_detected",
                        "n_waypoints": len(waypoints),
                        "config_collision_indices": cfg_hits,
                        "segment_collision_indices": seg_hits,
                    }
                )

        report_rows.append(
            {
                "scene_id": scene["scene_id"],
                "robot": scene["robot"],
                "difficulty": scene.get("difficulty"),
                "status": "checked",
                "expected_success_seeds": sorted(expected_success),
                "results": scene_rows,
            }
        )
        scene_collisions = sum(1 for row in scene_rows if row.get("status") == "collision_detected")
        print(
            f"[{index}/{len(scenes)}] {scene['scene_id']}: checked={len(scene_rows)} collisions={scene_collisions}",
            flush=True,
        )

    summary = {
        "exp5_json": str(exp5_json.relative_to(root)),
        "baseline_bin": str(baseline_bin.relative_to(root)),
        "prm_build_s": float(args.prm_build_s),
        "prm_query_s": float(args.prm_query_s),
        "segment_resolution": int(args.segment_resolution),
        "expected_success_paths": int(total_expected_success),
        "replayed_paths": int(total_replayed),
        "collision_free_paths": int(total_collision_free),
        "config_collision_paths": int(total_cfg_collision),
        "segment_collision_paths": int(total_seg_collision),
        "replay_fail_paths": int(total_replay_fail),
    }
    report = {
        "experiment": "exp5_prm_independent_collision_audit",
        "summary": summary,
        "scenes": report_rows,
    }
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(report, indent=2) + "\n")
    print(f"[write] {out_path.relative_to(root)}", flush=True)
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()

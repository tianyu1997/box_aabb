#!/usr/bin/env python3
# pyright: reportMissingImports=false

import argparse
import json
import random
import statistics
import subprocess
import sys
from pathlib import Path


REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "cpp/v6/python"))

import sbf5


DEFAULT_BOXES = REPO / "cpp/v7/experiments/results_paper/critsample_compare/same_boxes_500.json"


def sample_boxes(robot, n_boxes, width_lo, width_hi, seed):
    rng = random.Random(seed)
    boxes = []
    for _ in range(n_boxes):
        ivs = []
        for lim in robot.joint_limits().limits:
            width = min(lim.hi - lim.lo, rng.uniform(width_lo, width_hi))
            lo_hi = max(lim.lo, lim.hi - width)
            lo = rng.uniform(lim.lo, lo_hi)
            hi = min(lo + width, lim.hi)
            ivs.append([float(lo), float(hi)])
        boxes.append(ivs)
    return boxes


def load_or_create_boxes(path, robot_json, n_boxes, width_lo, width_hi, seed):
    if path.exists():
        payload = json.loads(path.read_text(encoding="utf-8"))
        boxes = payload["boxes"] if isinstance(payload, dict) else payload
        return boxes, False

    robot = sbf5.Robot.from_json(str(robot_json))
    boxes = sample_boxes(robot, n_boxes, width_lo, width_hi, seed)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"boxes": boxes}, indent=2), encoding="utf-8")
    return boxes, True


def build_variants():
    return [
        {
            "key": "LinkIAABB",
            "stage": "subdivision",
            "envelope": "LinkIAABB",
            "type": sbf5.EnvelopeType.LinkIAABB,
            "n_sub": 1,
            "delta": None,
            "effective_delta": 0.05,
        },
        {
            "key": "LinkIAABB_S2",
            "stage": "subdivision",
            "envelope": "LinkIAABB_S2",
            "type": sbf5.EnvelopeType.LinkIAABB,
            "n_sub": 2,
            "delta": None,
            "effective_delta": 0.05,
        },
        {
            "key": "LinkIAABB_S4",
            "stage": "subdivision",
            "envelope": "LinkIAABB_S4",
            "type": sbf5.EnvelopeType.LinkIAABB,
            "n_sub": 4,
            "delta": None,
            "effective_delta": 0.05,
        },
        {
            "key": "LinkIAABB_S8",
            "stage": "subdivision",
            "envelope": "LinkIAABB_S8",
            "type": sbf5.EnvelopeType.LinkIAABB,
            "n_sub": 8,
            "delta": None,
            "effective_delta": 0.05,
        },
        {
            "key": "LinkGrid_S4_d0.02",
            "stage": "grid",
            "envelope": "LinkGrid_S4",
            "type": sbf5.EnvelopeType.LinkIAABB_Grid,
            "n_sub": 4,
            "delta": 0.02,
            "effective_delta": 0.02,
        },
        {
            "key": "LinkGrid_S4_d0.04",
            "stage": "grid",
            "envelope": "LinkGrid_S4",
            "type": sbf5.EnvelopeType.LinkIAABB_Grid,
            "n_sub": 4,
            "delta": 0.04,
            "effective_delta": 0.04,
        },
        {
            "key": "LinkGrid_S4_d0.06",
            "stage": "grid",
            "envelope": "LinkGrid_S4",
            "type": sbf5.EnvelopeType.LinkIAABB_Grid,
            "n_sub": 4,
            "delta": 0.06,
            "effective_delta": 0.06,
        },
        {
            "key": "LinkGrid_S4_d0.08",
            "stage": "grid",
            "envelope": "LinkGrid_S4",
            "type": sbf5.EnvelopeType.LinkIAABB_Grid,
            "n_sub": 4,
            "delta": 0.08,
            "effective_delta": 0.08,
        },
        {
            "key": "Hull16Grid_d0.02",
            "stage": "grid",
            "envelope": "Hull16Grid",
            "type": sbf5.EnvelopeType.Hull16_Grid,
            "n_sub": 1,
            "delta": 0.02,
            "effective_delta": 0.02,
        },
        {
            "key": "Hull16Grid_d0.04",
            "stage": "grid",
            "envelope": "Hull16Grid",
            "type": sbf5.EnvelopeType.Hull16_Grid,
            "n_sub": 1,
            "delta": 0.04,
            "effective_delta": 0.04,
        },
        {
            "key": "Hull16Grid_d0.06",
            "stage": "grid",
            "envelope": "Hull16Grid",
            "type": sbf5.EnvelopeType.Hull16_Grid,
            "n_sub": 1,
            "delta": 0.06,
            "effective_delta": 0.06,
        },
        {
            "key": "Hull16Grid_d0.08",
            "stage": "grid",
            "envelope": "Hull16Grid",
            "type": sbf5.EnvelopeType.Hull16_Grid,
            "n_sub": 1,
            "delta": 0.08,
            "effective_delta": 0.08,
        },
    ]


def dump_v6(boxes, robot_json, variants):
    robot = sbf5.Robot.from_json(str(robot_json))
    ep_cfg = sbf5.EndpointSourceConfig()
    ep_cfg.source = sbf5.EndpointSource.CritSample

    rows = []
    for idx, box in enumerate(boxes):
        intervals = [sbf5.Interval(lo, hi) for lo, hi in box]
        downstream = {}
        for variant in variants:
            env_cfg = sbf5.EnvelopeTypeConfig()
            env_cfg.type = variant["type"]
            env_cfg.n_subdivisions = variant["n_sub"]
            if variant["delta"] is not None:
                env_cfg.grid_config.voxel_delta = float(variant["delta"])
            info = sbf5.compute_envelope_info(robot, intervals, ep_cfg, env_cfg, None)
            downstream[variant["key"]] = {
                "stage": variant["stage"],
                "envelope": variant["envelope"],
                "n_subdivisions": variant["n_sub"],
                "voxel_delta": float(variant["effective_delta"]),
                "volume": float(info["volume"]),
                "voxel_count": float(info.get("grid_num_voxels", 0.0)),
            }
        rows.append({
            "index": idx,
            "intervals": box,
            "downstream_volumes": downstream,
        })

    return {
        "robot_json": str(robot_json),
        "endpoint_source": "CritSample",
        "n_boxes": len(rows),
        "variants": [
            {
                "key": variant["key"],
                "stage": variant["stage"],
                "envelope": variant["envelope"],
                "n_subdivisions": variant["n_sub"],
                "voxel_delta": float(variant["effective_delta"]),
            }
            for variant in variants
        ],
        "rows": rows,
    }


def compare_variant(v6_rows, v7_rows, key, tol):
    diffs = []
    per_box = []
    v6_values = []
    v7_values = []
    for row6, row7 in zip(v6_rows, v7_rows):
        v6_value = float(row6["downstream_volumes"][key]["volume"])
        v7_value = float(row7["downstream_volumes"][key]["volume"])
        abs_diff = abs(v6_value - v7_value)
        diffs.append(abs_diff)
        v6_values.append(v6_value)
        v7_values.append(v7_value)
        per_box.append({
            "index": row6["index"],
            "intervals": row6["intervals"],
            "v6": v6_value,
            "v7": v7_value,
            "abs_diff": abs_diff,
            "equal_within_tol": abs_diff <= tol,
        })
    return {
        "summary": {
            "max_abs_diff": max(diffs, default=0.0),
            "mean_abs_diff": statistics.fmean(diffs) if diffs else 0.0,
            "mismatch_boxes": sum(diff > tol for diff in diffs),
            "v6_mean_volume": statistics.fmean(v6_values) if v6_values else 0.0,
            "v7_mean_volume": statistics.fmean(v7_values) if v7_values else 0.0,
        },
        "per_box": per_box,
    }


def main():
    parser = argparse.ArgumentParser(description="Compare v6/v7 downstream link-envelope volume paths on the same boxes")
    parser.add_argument("--n-boxes", type=int, default=500)
    parser.add_argument("--width-lo", type=float, default=0.10)
    parser.add_argument("--width-hi", type=float, default=0.50)
    parser.add_argument("--seed", type=int, default=20260427)
    parser.add_argument("--n-sub", type=int, default=4)
    parser.add_argument("--tol", type=float, default=1e-6)
    parser.add_argument("--robot-json", type=Path, default=REPO / "cpp/v7/data/iiwa14.json")
    parser.add_argument("--boxes-json", type=Path, default=DEFAULT_BOXES)
    parser.add_argument("--v7-binary", type=Path, default=REPO / "cpp/v7/build-release/experiments/exp_compare_crit_boxes")
    parser.add_argument("--out-dir", type=Path, default=REPO / "cpp/v7/experiments/results_paper/critsample_compare")
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    boxes, boxes_created = load_or_create_boxes(
        args.boxes_json, args.robot_json, args.n_boxes,
        args.width_lo, args.width_hi, args.seed)

    v6_path = args.out_dir / "v6_volume_dump.json"
    v7_path = args.out_dir / "v7_volume_dump.json"
    report_path = args.out_dir / "volume_compare_report.json"

    variants = build_variants()
    v6_dump = dump_v6(boxes, args.robot_json, variants)
    v6_path.write_text(json.dumps(v6_dump, indent=2), encoding="utf-8")

    subprocess.run([
        str(args.v7_binary),
        f"--in={args.boxes_json}",
        f"--out={v7_path}",
        f"--robot-json={args.robot_json}",
        f"--n-sub={args.n_sub}",
    ], check=True, cwd=str(REPO))

    v7_dump = json.loads(v7_path.read_text(encoding="utf-8"))
    variant_reports = {}
    summary = {}
    for variant in variants:
        key = variant["key"]
        report = compare_variant(v6_dump["rows"], v7_dump["rows"], key, args.tol)
        variant_reports[key] = report
        summary[key] = {
            "stage": variant["stage"],
            "envelope": variant["envelope"],
            "n_subdivisions": variant["n_sub"],
            "voxel_delta": float(variant["effective_delta"]),
            **report["summary"],
        }

    report = {
        "meta": {
            "n_boxes": len(boxes),
            "width_lo": args.width_lo,
            "width_hi": args.width_hi,
            "seed": args.seed,
            "tol": args.tol,
            "robot_json": str(args.robot_json),
            "boxes_json": str(args.boxes_json),
            "boxes_created": boxes_created,
            "v6_dump_path": str(v6_path),
            "v7_dump_path": str(v7_path),
        },
        "summary": summary,
        "variants": variant_reports,
    }
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print("volume compare report:", report_path)
    for variant in variants:
        key = variant["key"]
        item = summary[key]
        print(
            key,
            "mismatch_boxes=", item["mismatch_boxes"],
            "max_abs_diff=", item["max_abs_diff"],
            "v6_mean=", item["v6_mean_volume"],
            "v7_mean=", item["v7_mean_volume"],
        )


if __name__ == "__main__":
    main()
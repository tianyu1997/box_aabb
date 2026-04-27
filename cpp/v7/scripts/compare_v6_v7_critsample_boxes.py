#!/usr/bin/env python3

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


def derive_link_zero_paired(endpoint_iaabbs, n_active_links):
    out = []
    for ci in range(n_active_links):
        base = ci * 12
        prox = endpoint_iaabbs[base : base + 6]
        dist = endpoint_iaabbs[base + 6 : base + 12]
        out.extend([
            min(prox[0], dist[0]),
            min(prox[1], dist[1]),
            min(prox[2], dist[2]),
            max(prox[3], dist[3]),
            max(prox[4], dist[4]),
            max(prox[5], dist[5]),
        ])
    return out


def derive_link_zero_subdivided(endpoint_iaabbs, n_active_links, n_sub):
    out = []
    inv_n = 1.0 / float(n_sub)
    for ci in range(n_active_links):
        base = ci * 12
        prox = endpoint_iaabbs[base : base + 6]
        dist = endpoint_iaabbs[base + 6 : base + 12]
        for s in range(n_sub):
            t_lo = float(s) * inv_n
            t_hi = float(s + 1) * inv_n
            for d in range(3):
                lo_at_t0 = prox[d] * (1.0 - t_lo) + dist[d] * t_lo
                lo_at_t1 = prox[d] * (1.0 - t_hi) + dist[d] * t_hi
                out.append(min(lo_at_t0, lo_at_t1))
            for d in range(3):
                hi_at_t0 = prox[d + 3] * (1.0 - t_lo) + dist[d + 3] * t_lo
                hi_at_t1 = prox[d + 3] * (1.0 - t_hi) + dist[d + 3] * t_hi
                out.append(max(hi_at_t0, hi_at_t1))
    return out


def dump_v6(boxes, robot_json, n_sub):
    robot = sbf5.Robot.from_json(str(robot_json))
    ep_cfg = sbf5.EndpointSourceConfig()
    ep_cfg.source = sbf5.EndpointSource.CritSample

    rows = []
    for idx, box in enumerate(boxes):
        intervals = [sbf5.Interval(lo, hi) for lo, hi in box]
        ep = sbf5.compute_endpoint_iaabb_info(robot, intervals, ep_cfg, None)
        endpoint_iaabbs = [float(x) for x in ep["endpoint_iaabbs"]]
        n_active_links = int(ep["n_active_links"])
        rows.append({
            "index": idx,
            "intervals": box,
            "n_active_links": n_active_links,
            "endpoint_iaabbs": endpoint_iaabbs,
            "link_iaabbs_zero_paired": derive_link_zero_paired(endpoint_iaabbs, n_active_links),
            "link_iaabbs_zero_subdivided": derive_link_zero_subdivided(endpoint_iaabbs, n_active_links, n_sub),
        })
    return {
        "robot_json": str(robot_json),
        "endpoint_source": "CritSample",
        "n_boxes": len(rows),
        "n_sub": n_sub,
        "rows": rows,
    }


def max_abs_diff(xs, ys):
    return max((abs(float(a) - float(b)) for a, b in zip(xs, ys)), default=0.0)


def worst_component(xs, ys):
    best_idx = -1
    best = -1.0
    best_pair = (0.0, 0.0)
    for idx, (a, b) in enumerate(zip(xs, ys)):
        d = abs(float(a) - float(b))
        if d > best:
            best = d
            best_idx = idx
            best_pair = (float(a), float(b))
    return {
        "component": best_idx,
        "v6": best_pair[0],
        "v7": best_pair[1],
        "abs_diff": best,
    }


def compare_rows(v6_rows, v7_rows, tol):
    per_box = []
    endpoint_diffs = []
    paired_diffs = []
    subdiv_diffs = []

    for row6, row7 in zip(v6_rows, v7_rows):
        ep_diff = max_abs_diff(row6["endpoint_iaabbs"], row7["endpoint_iaabbs"])
        paired_diff = max_abs_diff(row6["link_iaabbs_zero_paired"], row7["link_iaabbs_zero_paired"])
        subdiv_diff = max_abs_diff(row6["link_iaabbs_zero_subdivided"], row7["link_iaabbs_zero_subdivided"])
        endpoint_diffs.append(ep_diff)
        paired_diffs.append(paired_diff)
        subdiv_diffs.append(subdiv_diff)
        per_box.append({
            "index": row6["index"],
            "intervals": row6["intervals"],
            "endpoint_max_abs_diff": ep_diff,
            "endpoint_worst": worst_component(row6["endpoint_iaabbs"], row7["endpoint_iaabbs"]),
            "link_zero_paired_max_abs_diff": paired_diff,
            "link_zero_paired_worst": worst_component(row6["link_iaabbs_zero_paired"], row7["link_iaabbs_zero_paired"]),
            "link_zero_subdivided_max_abs_diff": subdiv_diff,
            "link_zero_subdivided_worst": worst_component(row6["link_iaabbs_zero_subdivided"], row7["link_iaabbs_zero_subdivided"]),
            "endpoint_equal_within_tol": ep_diff <= tol,
            "link_zero_paired_equal_within_tol": paired_diff <= tol,
            "link_zero_subdivided_equal_within_tol": subdiv_diff <= tol,
        })

    return {
        "summary": {
            "tolerance": tol,
            "endpoint_max_abs_diff": max(endpoint_diffs, default=0.0),
            "endpoint_mean_abs_diff": statistics.fmean(endpoint_diffs) if endpoint_diffs else 0.0,
            "endpoint_mismatch_boxes": sum(d > tol for d in endpoint_diffs),
            "link_zero_paired_max_abs_diff": max(paired_diffs, default=0.0),
            "link_zero_paired_mean_abs_diff": statistics.fmean(paired_diffs) if paired_diffs else 0.0,
            "link_zero_paired_mismatch_boxes": sum(d > tol for d in paired_diffs),
            "link_zero_subdivided_max_abs_diff": max(subdiv_diffs, default=0.0),
            "link_zero_subdivided_mean_abs_diff": statistics.fmean(subdiv_diffs) if subdiv_diffs else 0.0,
            "link_zero_subdivided_mismatch_boxes": sum(d > tol for d in subdiv_diffs),
        },
        "per_box": per_box,
    }


def main():
    parser = argparse.ArgumentParser(description="Compare v6/v7 CritSample endpoint and zero-radius link IAABBs on the same boxes")
    parser.add_argument("--n-boxes", type=int, default=500)
    parser.add_argument("--width-lo", type=float, default=0.10)
    parser.add_argument("--width-hi", type=float, default=0.50)
    parser.add_argument("--seed", type=int, default=20260427)
    parser.add_argument("--n-sub", type=int, default=4)
    parser.add_argument("--tol", type=float, default=1e-6)
    parser.add_argument("--robot-json", type=Path, default=REPO / "cpp/v7/data/iiwa14.json")
    parser.add_argument("--v7-binary", type=Path, default=REPO / "cpp/v7/build-release/experiments/exp_compare_crit_boxes")
    parser.add_argument("--out-dir", type=Path, default=REPO / "cpp/v7/experiments/results_paper/critsample_compare")
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    boxes_path = args.out_dir / "same_boxes_500.json"
    v6_path = args.out_dir / "v6_dump.json"
    v7_path = args.out_dir / "v7_dump.json"
    report_path = args.out_dir / "compare_report.json"

    robot = sbf5.Robot.from_json(str(args.robot_json))
    boxes = sample_boxes(robot, args.n_boxes, args.width_lo, args.width_hi, args.seed)
    boxes_path.write_text(json.dumps({"boxes": boxes}, indent=2), encoding="utf-8")

    v6_dump = dump_v6(boxes, args.robot_json, args.n_sub)
    v6_path.write_text(json.dumps(v6_dump, indent=2), encoding="utf-8")

    subprocess.run([
        str(args.v7_binary),
        f"--in={boxes_path}",
        f"--out={v7_path}",
        f"--robot-json={args.robot_json}",
        f"--n-sub={args.n_sub}",
    ], check=True, cwd=str(REPO))

    v7_dump = json.loads(v7_path.read_text(encoding="utf-8"))
    report = compare_rows(v6_dump["rows"], v7_dump["rows"], args.tol)
    report["meta"] = {
        "n_boxes": args.n_boxes,
        "width_lo": args.width_lo,
        "width_hi": args.width_hi,
        "seed": args.seed,
        "n_sub": args.n_sub,
        "robot_json": str(args.robot_json),
        "boxes_path": str(boxes_path),
        "v6_dump_path": str(v6_path),
        "v7_dump_path": str(v7_path),
    }
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    summary = report["summary"]
    print("compare report:", report_path)
    print("endpoint mismatch boxes:", summary["endpoint_mismatch_boxes"], "max abs diff:", summary["endpoint_max_abs_diff"])
    print("link zero paired mismatch boxes:", summary["link_zero_paired_mismatch_boxes"], "max abs diff:", summary["link_zero_paired_max_abs_diff"])
    print("link zero subdivided mismatch boxes:", summary["link_zero_subdivided_mismatch_boxes"], "max abs diff:", summary["link_zero_subdivided_max_abs_diff"])


if __name__ == "__main__":
    main()
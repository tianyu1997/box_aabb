#!/usr/bin/env python3
"""Pure Exp. 2 LECT-envelope fill microbenchmark.

This runner measures an isolated depth-synchronous LECT envelope fill workload
on the current machine for the retained Exp. 2 link-envelope families. It uses
the same incremental-FK + endpoint/envelope kernels as the grower hot path, runs
with the paper 8-thread policy, and excludes KD-tree NN, FFB collision tests,
bridge, and graph costs. The resulting fill throughput is used for the paper's
D32 estimate.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import time
from pathlib import Path

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import PAPER_THREADS, add_common_args, bin_path, write_json

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    add_common_args(parser)
    parser.add_argument("--threads", type=int, default=PAPER_THREADS)
    parser.add_argument("--extrapolation-depth", type=int, default=32)
    parser.add_argument("--benchmark-depth", type=int, default=12)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--output-name", default="link_envelope_growth_calibration.json")
    args = parser.parse_args()

    args.out_dir = args.out_dir.resolve()

    run_id = time.strftime("exp2_microbench_%Y%m%d_%H%M%S")
    raw_root = args.out_dir / "raw" / "link_envelope_growth_calibration" / run_id
    raw_root.mkdir(parents=True, exist_ok=True)
    raw_path = raw_root / "microbench.json"

    binary = bin_path(args, "exp11_link_envelope_microbench")
    subprocess.run(
        [
            str(binary),
            "--output", str(raw_path),
            "--threads", str(int(args.threads)),
            "--benchmark-depth", str(int(args.benchmark_depth)),
            "--repeats", str(int(args.repeats)),
        ],
        check=True,
    )

    payload = json.loads(raw_path.read_text())
    extrapolated_nodes = (1 << (int(args.extrapolation_depth) + 1)) - 1
    summaries = []
    for row in payload.get("rows", []):
        depth_profile = list(row.get("depth_profile", []))
        deepest = depth_profile[-1] if depth_profile else {}
        us_per_node = float(deepest.get("microbench_us_per_node_median") or row.get("microbench_us_per_node_median") or 0.0)
        fill_nodes_per_second = 1e6 / us_per_node if us_per_node > 0.0 else None
        d32_time_s = extrapolated_nodes / fill_nodes_per_second if fill_nodes_per_second else None
        summaries.append(
            {
                **row,
                "microbench_depth_used": int(deepest.get("depth", 0) or 0),
                "microbench_us_per_node_global_median": float(row.get("microbench_us_per_node_median") or 0.0),
                "microbench_us_per_node_median": us_per_node,
                "fill_us_per_node_median": us_per_node,
                "fill_nodes_per_second": fill_nodes_per_second,
                "extrapolation_depth": int(args.extrapolation_depth),
                "extrapolated_nodes": extrapolated_nodes,
                "d32_fill_time_s": d32_time_s,
                "depth_build_s_estimate": d32_time_s,
            }
        )

    payload = {
        "experiment": "lect_envelope_fill_microbench",
        "source_script": "02_6_link_envelope_growth_calibration.py",
        "method": "depth_synchronous_incremental_fk_plus_envelope_fill",
        "endpoint_source": "mixed_Crit_main_plus_IFK_controls",
        "run_id": run_id,
        "threads": int(args.threads),
        "benchmark_depth": int(args.benchmark_depth),
        "repeats": int(args.repeats),
        "extrapolation_depth": int(args.extrapolation_depth),
        "raw_path": str(raw_path),
        "summaries": summaries,
        "rows": payload.get("rows", []),
    }
    out_path = args.out_dir / args.output_name
    write_json(out_path, payload)
    print(f"[write] {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

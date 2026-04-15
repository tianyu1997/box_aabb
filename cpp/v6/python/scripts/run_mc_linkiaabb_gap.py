#!/usr/bin/env python3
"""MC-baseline signed gap experiment for LinkIAABB.

For each robot and method, compute signed per-axis gaps against MC baseline:
- lo gap: method_lo - mc_lo
- hi gap: method_hi - mc_hi
- extent gap: (method_hi-method_lo) - (mc_hi-mc_lo)

All gaps preserve sign. Statistics are aggregated across all boxes and links.
"""

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import sbf5
from scripts import run_all_experiments as rae


def _reshape_link_iaabbs(flat):
    arr = np.asarray(flat, dtype=np.float64)
    if arr.size % 6 != 0:
        raise ValueError(f"Invalid link_iaabbs length={arr.size}, not divisible by 6")
    return arr.reshape((-1, 6))


def _signed_gap_stats(method_blocks, mc_blocks):
    m = np.concatenate(method_blocks, axis=0)
    b = np.concatenate(mc_blocks, axis=0)

    lo_gap = m[:, 0:3] - b[:, 0:3]
    hi_gap = m[:, 3:6] - b[:, 3:6]
    ext_gap = (m[:, 3:6] - m[:, 0:3]) - (b[:, 3:6] - b[:, 0:3])

    def pack(x):
        neg_only = np.where(x < 0.0, x, np.inf)
        most_negative = np.min(neg_only, axis=0)
        # If one axis has no negative value, report 0.0 for that axis.
        most_negative = np.where(np.isfinite(most_negative), most_negative, 0.0)
        return {
            "mean": x.mean(axis=0).tolist(),
            "std": x.std(axis=0).tolist(),
            "p05": np.percentile(x, 5, axis=0).tolist(),
            "p50": np.percentile(x, 50, axis=0).tolist(),
            "p95": np.percentile(x, 95, axis=0).tolist(),
            "max_negative": most_negative.tolist(),
        }

    return {
        "lo_gap": pack(lo_gap),
        "hi_gap": pack(hi_gap),
        "extent_gap": pack(ext_gap),
    }


def run(n_boxes=500, mc_samples=50000, seed=42):
    robots = {
        "iiwa14": sbf5.Robot.from_json("data/iiwa14.json"),
    }

    methods = ["IFK", "CritSample", "Analytical", "GCPC", "MC"]

    # Optional GCPC caches
    gcpc_caches = {}
    for rname in robots:
        cands = [
            Path("data") / f"{rname}.gcpc",
            Path("data") / f"{rname}_5000.gcpc",
            Path("data") / f"{rname}_500.gcpc",
        ]
        for p in cands:
            if p.exists():
                gcpc_caches[rname] = sbf5.GcpcCache.load(str(p))
                break

    out = {
        "meta": {
            "n_boxes": int(n_boxes),
            "mc_samples": int(mc_samples),
            "seed": int(seed),
            "axis_order": ["x", "y", "z"],
            "gap_definition": {
                "lo_gap": "method_lo - mc_lo",
                "hi_gap": "method_hi - mc_hi",
                "extent_gap": "(method_hi-method_lo) - (mc_hi-mc_lo)",
            },
            "extra_stats": {
                "max_negative": "most negative observed gap per axis; 0.0 means no negative gap",
            },
        },
        "robots": {},
    }

    for rname, robot in robots.items():
        rng = np.random.RandomState(seed)
        box_list = [rae._random_intervals(robot, rng) for _ in range(n_boxes)]

        blocks = {m: [] for m in methods}

        for iv in box_list:
            # MC baseline for this box
            ep_mc = rae._make_ep_config("MC", mc_samples=mc_samples)
            mc_info = sbf5.compute_link_iaabb_info(robot, iv, ep_mc, None)
            mc_block = _reshape_link_iaabbs(mc_info["link_iaabbs"])
            blocks["MC"].append(mc_block)

            for m in methods:
                if m == "MC":
                    continue
                if m == "GCPC" and rname not in gcpc_caches:
                    continue
                ep = rae._make_ep_config(m)
                cache = gcpc_caches.get(rname) if m == "GCPC" else None
                info = sbf5.compute_link_iaabb_info(robot, iv, ep, cache)
                blocks[m].append(_reshape_link_iaabbs(info["link_iaabbs"]))

        robot_out = {}
        for m in methods:
            if m != "MC" and len(blocks[m]) == 0:
                continue
            robot_out[m] = _signed_gap_stats(blocks[m], blocks["MC"])

        out["robots"][rname] = robot_out

    return out


def main():
    ap = argparse.ArgumentParser(description="MC-baseline LinkIAABB signed-gap experiment")
    ap.add_argument("--n-boxes", type=int, default=500)
    ap.add_argument("--mc-samples", type=int, default=50000)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--out", type=str,
                    default="experiments/results/s1_mc_linkiaabb_gap/results.json")
    args = ap.parse_args()

    res = run(n_boxes=args.n_boxes, mc_samples=args.mc_samples, seed=args.seed)

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(res, indent=2), encoding="utf-8")
    print(f"Saved: {out_path}")


if __name__ == "__main__":
    main()

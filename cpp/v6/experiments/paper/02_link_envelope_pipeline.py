#!/usr/bin/env python3
"""Paper Exp. 2 — link-envelope pipeline comparison.

Environment: reuse the Exp. 1 four-bin width-stratified IIWA14 box protocol,
with one shared random box set per width bin, using CritSample for the retained
8-row main sweep plus 2 IFK control rows.
Output:      experiments/results_paper/link_envelope_pipeline.json
Paper slot:  Experiments-B, LinkIAABB subdivision / Hull16-grid comparison.
"""
from __future__ import annotations

import argparse
import contextlib
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import PAPER_THREADS, ROOT, add_common_args, load_json, mode_args, run_python, write_json
from marcucci_envelope_build_replay import CACHE_PHASES, run_trial


RAW_SCRIPT = ROOT / "python" / "scripts" / "run_s0c_width_sub_grid.py"
RAW_OUTPUT = "link_envelope_pipeline_v6_raw.json"
REPLAY_VARIANTS = [
    {
        "key": "crit_aabb_s1",
        "label": "Crit+AABB S=1",
        "endpoint": "critsample",
        "endpoint_label": "CritSample",
        "env": "link_iaabb",
        "type": "LinkIAABB",
        "n_subdivisions": 1,
        "voxel_delta": 0.05,
    },
    {
        "key": "crit_aabb_s2",
        "label": "Crit+AABB S=2",
        "endpoint": "critsample",
        "endpoint_label": "CritSample",
        "env": "link_iaabb",
        "type": "LinkIAABB",
        "n_subdivisions": 2,
        "voxel_delta": 0.05,
    },
    {
        "key": "crit_aabb_s4",
        "label": "Crit+AABB S=4",
        "endpoint": "critsample",
        "endpoint_label": "CritSample",
        "env": "link_iaabb",
        "type": "LinkIAABB",
        "n_subdivisions": 4,
        "voxel_delta": 0.05,
    },
    {
        "key": "crit_aabb_s8",
        "label": "Crit+AABB S=8",
        "endpoint": "critsample",
        "endpoint_label": "CritSample",
        "env": "link_iaabb",
        "type": "LinkIAABB",
        "n_subdivisions": 8,
        "voxel_delta": 0.05,
    },
    {
        "key": "crit_hull16_grid_d002",
        "label": "Crit+Hull16-grid d=0.02",
        "endpoint": "critsample",
        "endpoint_label": "CritSample",
        "env": "hull16_grid",
        "type": "Hull16_Grid",
        "n_subdivisions": 1,
        "voxel_delta": 0.02,
    },
    {
        "key": "crit_hull16_grid_d004",
        "label": "Crit+Hull16-grid d=0.04",
        "endpoint": "critsample",
        "endpoint_label": "CritSample",
        "env": "hull16_grid",
        "type": "Hull16_Grid",
        "n_subdivisions": 1,
        "voxel_delta": 0.04,
    },
    {
        "key": "crit_hull16_grid_d006",
        "label": "Crit+Hull16-grid d=0.06",
        "endpoint": "critsample",
        "endpoint_label": "CritSample",
        "env": "hull16_grid",
        "type": "Hull16_Grid",
        "n_subdivisions": 1,
        "voxel_delta": 0.06,
    },
    {
        "key": "crit_hull16_grid_d008",
        "label": "Crit+Hull16-grid d=0.08",
        "endpoint": "critsample",
        "endpoint_label": "CritSample",
        "env": "hull16_grid",
        "type": "Hull16_Grid",
        "n_subdivisions": 1,
        "voxel_delta": 0.08,
    },
    {
        "key": "ifk_aabb_s4",
        "label": "IFK+AABB S=4",
        "endpoint": "ifk",
        "endpoint_label": "IFK",
        "env": "link_iaabb",
        "type": "LinkIAABB",
        "n_subdivisions": 4,
        "voxel_delta": 0.05,
    },
    {
        "key": "ifk_hull16_grid_d004",
        "label": "IFK+Hull16-grid d=0.04",
        "endpoint": "ifk",
        "endpoint_label": "IFK",
        "env": "hull16_grid",
        "type": "Hull16_Grid",
        "n_subdivisions": 1,
        "voxel_delta": 0.04,
    },
]
STORAGE_MODEL = {
    "name": "depth_synchronous_compact_node_payload_v3",
    "optimized_base_node_bytes": 48.0,
    "aabb_payload_formula": "compact fixed node record (packed split/flags + parent/index delta); endpoint evidence is shared and not scaled by raw cache-file slabs",
    "grid_payload_formula": "min(measured_payload_bytes, 32 + 4*bricks + voxels/16)",
    "note": "Result-side estimate for compressed/de-duplicated depth-32 storage using compact brick headers and bit-packed occupancy; it does not change the .lect file format.",
}


def mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def optimized_payload_bytes(*, envelope: str, voxel_count: float, brick_count: float, measured_payload: float) -> float:
    if envelope != "Hull16_Grid" or measured_payload <= 0.0:
        return 0.0
    estimated = 32.0 + 4.0 * max(0.0, brick_count) + max(0.0, voxel_count) / 16.0
    return min(float(measured_payload), estimated)


def aggregate_rows(raw_rows: list[dict], *, endpoint_source: str, envelope: str,
                   subdivisions: int, voxel_delta: float | None) -> dict:
    matched = [
        row for row in raw_rows
        if str(row.get("endpoint_source", row.get("endpoint", ""))) == endpoint_source
        if str(row.get("envelope")) == envelope
        and int(row.get("subdivisions", 0)) == subdivisions
        and ((voxel_delta is None and row.get("grid_delta") is None)
             or (voxel_delta is not None and abs(float(row.get("grid_delta", 0.0)) - voxel_delta) < 1e-9))
    ]
    if not matched:
        raise ValueError(
            f"Missing raw rows for endpoint={endpoint_source}, envelope={envelope}, subdivisions={subdivisions}, voxel_delta={voxel_delta}."
        )
    return {
        "volume_mean": mean([float(row.get("volume_mean", 0.0)) for row in matched]),
        "time_us_mean": mean([float(row.get("total_us_mean", 0.0)) for row in matched]),
        "voxel_brick_count_mean": mean([float(row.get("cache_bricks_mean", 0.0)) for row in matched]),
        "voxel_count_mean": mean([float(row.get("cache_voxels_mean", 0.0)) for row in matched]),
        "cache_payload_bytes_mean": mean([
            float(row.get("cache_payload_bytes_mean", 0.0)) for row in matched
        ]),
    }


def translate_payload(raw: dict, *, n_boxes: int, repeats: int) -> dict:
    raw_rows = raw.get("rows", [])
    baseline = aggregate_rows(
        raw_rows,
        endpoint_source="CritSample",
        envelope="LinkIAABB",
        subdivisions=1,
        voxel_delta=None,
    )
    baseline_volume = baseline["volume_mean"]

    desired_specs = [
        ("crit_subdivision", "CritSample", "LinkIAABB", 1, None, "LinkIAABB"),
        ("crit_subdivision", "CritSample", "LinkIAABB", 2, None, "LinkIAABB_S2"),
        ("crit_subdivision", "CritSample", "LinkIAABB", 4, None, "LinkIAABB_S4"),
        ("crit_subdivision", "CritSample", "LinkIAABB", 8, None, "LinkIAABB_S8"),
        ("crit_grid", "CritSample", "Hull16_Grid", 1, 0.02, "Hull16Grid"),
        ("crit_grid", "CritSample", "Hull16_Grid", 1, 0.04, "Hull16Grid"),
        ("crit_grid", "CritSample", "Hull16_Grid", 1, 0.06, "Hull16Grid"),
        ("crit_grid", "CritSample", "Hull16_Grid", 1, 0.08, "Hull16Grid"),
        ("ifk_control", "IFK", "LinkIAABB", 4, None, "LinkIAABB_S4"),
        ("ifk_control", "IFK", "Hull16_Grid", 1, 0.04, "Hull16Grid"),
    ]

    rows = []
    for stage, endpoint_source, envelope_type, subdivisions, voxel_delta, envelope_name in desired_specs:
        stats = aggregate_rows(
            raw_rows,
            endpoint_source=endpoint_source,
            envelope=envelope_type,
            subdivisions=subdivisions,
            voxel_delta=voxel_delta,
        )
        storage_bytes = optimized_payload_bytes(
            envelope=envelope_type,
            voxel_count=stats["voxel_count_mean"],
            brick_count=stats["voxel_brick_count_mean"],
            measured_payload=stats["cache_payload_bytes_mean"],
        )
        rows.append(
            {
                "stage": stage,
                "endpoint_source": endpoint_source,
                "envelope": envelope_name,
                "type": envelope_type,
                "n_subdivisions": subdivisions,
                "voxel_delta": 0.05 if voxel_delta is None else voxel_delta,
                "volume_mean": stats["volume_mean"],
                "time_us_mean": stats["time_us_mean"],
                "voxel_brick_count_mean": stats["voxel_brick_count_mean"],
                "voxel_count_mean": stats["voxel_count_mean"],
                "cache_payload_bytes_mean": stats["cache_payload_bytes_mean"],
                "storage_bytes_optimized_mean": storage_bytes,
                "storage_payload_compression_ratio": (
                    storage_bytes / stats["cache_payload_bytes_mean"]
                    if stats["cache_payload_bytes_mean"] > 0.0 else None
                ),
                "ratio_to_linkiaabb": stats["volume_mean"] / baseline_volume if baseline_volume > 0 else 0.0,
            }
        )

    width_bins = []
    for row in raw.get("meta", {}).get("width_bins", []):
        width_bins.append(
            {
                "width_bin": f"{float(row.get('lo', 0.0)):.3f}-{float(row.get('hi', 0.0)):.2f}" if abs(float(row.get('lo', 0.0)) - 0.001) < 1e-9 else f"{float(row.get('lo', 0.0)):.2f}-{float(row.get('hi', 0.0)):.2f}",
                "width_lo": float(row.get("lo", 0.0)),
                "width_hi": float(row.get("hi", 0.0)),
                "n_boxes": int(n_boxes),
            }
        )

    return {
        "experiment": "link_envelope_pipeline",
        "robot": "iiwa14",
        "endpoint_source": "Crit_main_plus_IFK_controls",
        "width_sampling_mode": "exp1_stratified_aggregate",
        "n_bins": len(width_bins),
        "n_boxes_per_bin": int(n_boxes),
        "n_boxes_total": int(n_boxes) * len(width_bins),
        "n_repeats": int(repeats),
        "storage_model": STORAGE_MODEL,
        "width_bins": width_bins,
        "rows": rows,
    }


def replay_row(cache_hit: dict, bake: dict, warm_bake: dict, variant: dict, seed: int) -> dict:
    timing = cache_hit.get("timing") or cache_hit.get("build_timing") or {}
    ep_hits = int(timing.get("v6_cache_ep_hits", 0) or 0)
    grid_hits = int(timing.get("v6_cache_grid_hits", 0) or 0)
    total_hits = ep_hits + grid_hits
    env_ms = float(timing.get("grow_expand_env_ms", 0.0) or 0.0)
    calls = int(timing.get("grow_expand_calls", 0) or 0)
    us_per_hit = (env_ms * 1000.0 / total_hits) if total_hits > 0 else None
    us_per_call = (env_ms * 1000.0 / calls) if calls > 0 else None

    return {
        "type": variant["type"],
        "n_subdivisions": int(variant["n_subdivisions"]),
        "voxel_delta": float(variant["voxel_delta"]),
        "variant_key": variant["key"],
        "variant_label": variant["label"],
        "endpoint_source": variant["endpoint_label"],
        "seed": int(seed),
        "cold_fill_n_boxes": int(cache_hit.get("n_boxes", 0) or 0),
        "warm_bake_n_boxes": int(warm_bake.get("n_boxes", 0) or 0),
        "cache_hit_n_boxes": int(cache_hit.get("n_boxes", 0) or 0),
        "route_match_warm_bake": bool(cache_hit.get("route_hash", "") == warm_bake.get("route_hash", "")),
        "cache_hit_ep_hits": ep_hits,
        "cache_hit_grid_hits": grid_hits,
        "cache_hit_total_hits": total_hits,
        "cache_hit_ep_misses": int(timing.get("v6_cache_ep_misses", 0) or 0),
        "cache_hit_grid_misses": int(timing.get("v6_cache_grid_misses", 0) or 0),
        "cache_hit_grid_compute_fallbacks": int(timing.get("v6_cache_grid_compute_fallbacks", 0) or 0),
        "cache_hit_miss_like_total": int(timing.get("v6_cache_ep_misses", 0) or 0)
        + int(timing.get("v6_cache_grid_misses", 0) or 0)
        + int(timing.get("v6_cache_grid_compute_fallbacks", 0) or 0),
        "cache_hit_strict_all_hit": (
            int(timing.get("v6_cache_ep_misses", 0) or 0) == 0
            and int(timing.get("v6_cache_grid_misses", 0) or 0) == 0
            and int(timing.get("v6_cache_grid_compute_fallbacks", 0) or 0) == 0
        ),
        "cache_hit_grow_expand_env_ms": env_ms,
        "cache_hit_grow_expand_calls": calls,
        "cache_hit_read_us_per_hit": us_per_hit,
        "cache_hit_read_us_per_call": us_per_call,
        "raw_cold_path": str(cache_hit.get("raw_path", "")),
        "raw_bake_path": str(bake.get("raw_path", "")),
        "raw_warm_bake_path": str(warm_bake.get("raw_path", "")),
        "raw_cache_hit_path": str(cache_hit.get("raw_path", "")),
    }


@contextlib.contextmanager
def silence_stdio(enabled: bool):
    if not enabled:
        yield
        return

    null_fd = os.open(os.devnull, os.O_WRONLY)
    out_fd = os.dup(1)
    err_fd = os.dup(2)
    try:
        os.dup2(null_fd, 1)
        os.dup2(null_fd, 2)
        yield
    finally:
        os.dup2(out_fd, 1)
        os.dup2(err_fd, 2)
        os.close(out_fd)
        os.close(err_fd)
        os.close(null_fd)


def generate_replay_read(args: argparse.Namespace, timeout_s: int) -> None:
    run_id = time.strftime("exp2_replay_read_%Y%m%d_%H%M%S")
    raw_root = args.out_dir / "raw" / "link_envelope_replay_read" / run_id
    raw_root.mkdir(parents=True, exist_ok=True)

    rows = []
    for seed in range(int(args.replay_seeds)):
        for variant in REPLAY_VARIANTS:
            phase_results = {}
            replay_variant = {
                "key": variant["key"],
                "label": variant["label"],
                "env": variant["env"],
                "n_sub": int(variant["n_subdivisions"]),
                "voxel_delta": float(variant["voxel_delta"]),
            }
            for phase in CACHE_PHASES:
                raw_path = raw_root / f"{variant['key']}_seed{seed}_{phase['key']}.json"
                with silence_stdio(not args.replay_verbose):
                    trial = run_trial(
                        endpoint=variant["endpoint"],
                        endpoint_label=variant["endpoint_label"],
                        variant=replay_variant,
                        seed=int(seed),
                        phase=phase,
                        cache_dir=raw_root / f"cache_seed{seed}_{variant['key']}",
                        raw_path=raw_path,
                        timeout_s=int(timeout_s),
                        threads=int(args.replay_threads),
                        bridge_threads=int(args.replay_bridge_threads),
                        ffb_depth=int(args.replay_ffb_depth),
                        max_boxes=int(args.replay_max_boxes),
                        bridge_boxes=int(args.replay_bridge_boxes),
                        max_miss=args.replay_max_miss,
                    )
                trial["raw_path"] = str(raw_path)
                phase_results[phase["key"]] = trial

            rows.append(
                replay_row(
                    phase_results["cache_hit"],
                    phase_results["bake"],
                    phase_results["warm_bake"],
                    variant,
                    int(seed),
                )
            )
            print(f"[done] replay seed={seed} {variant['label']}")

    payload = {
        "experiment": "link_envelope_replay_read_exp2_endpoint_aware",
        "source_script": "02_link_envelope_pipeline.py",
        "run_id": run_id,
        "threads": int(args.replay_threads),
        "bridge_threads": int(args.replay_bridge_threads),
        "ffb_depth": int(args.replay_ffb_depth),
        "max_boxes": int(args.replay_max_boxes),
        "bridge_boxes": int(args.replay_bridge_boxes),
        "timeout_s": int(timeout_s),
        "rows": rows,
    }
    replay_out = args.out_dir / args.replay_output_name
    write_json(replay_out, payload)
    print(f"[write] {replay_out}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_common_args(parser)
    parser.add_argument("--n-boxes", type=int, default=None)
    parser.add_argument("--repeats", type=int, default=None)
    parser.add_argument("--robot", default="iiwa14")
    parser.add_argument("--run-replay-read", action="store_true")
    parser.add_argument("--replay-output-name", default="link_envelope_replay_read.json")
    parser.add_argument("--replay-seeds", type=int, default=1)
    parser.add_argument("--replay-threads", type=int, default=PAPER_THREADS)
    parser.add_argument("--replay-bridge-threads", type=int, default=PAPER_THREADS)
    parser.add_argument("--replay-ffb-depth", type=int, default=300)
    parser.add_argument("--replay-max-boxes", type=int, default=200000)
    parser.add_argument("--replay-bridge-boxes", type=int, default=4000)
    parser.add_argument("--replay-max-miss", type=int, default=None)
    parser.add_argument("--replay-timeout", type=int, default=60)
    parser.add_argument("--replay-verbose", action="store_true")
    args = parser.parse_args()

    args.out_dir = args.out_dir.resolve()

    _seeds, _timeout, _mode = mode_args(args, quick_seeds=1, full_seeds=1)
    if args.robot != "iiwa14":
        raise ValueError("The v6 Exp.2 wrapper currently supports only --robot=iiwa14.")

    n_boxes = args.n_boxes if args.n_boxes is not None else (20 if args.quick else 400)
    repeats = args.repeats if args.repeats is not None else (5 if args.quick else 20)

    raw_out = args.out_dir / "raw" / RAW_OUTPUT
    final_out = args.out_dir / "link_envelope_pipeline.json"
    script_args: list[str | Path] = [
        "--out", raw_out,
        "--n-boxes", str(n_boxes),
        "--repeats", str(repeats),
        "--bin-scheme", "exp1",
    ]

    run_python(RAW_SCRIPT, script_args, dry_run=args.dry_run)
    if args.dry_run:
        print(f"[dry-run] would write {final_out}")
        return

    raw = load_json(raw_out)
    write_json(final_out, translate_payload(raw, n_boxes=int(n_boxes), repeats=int(repeats)))
    print(f"[write] {final_out}")
    if args.run_replay_read:
        generate_replay_read(args, int(args.replay_timeout))


if __name__ == "__main__":
    main()

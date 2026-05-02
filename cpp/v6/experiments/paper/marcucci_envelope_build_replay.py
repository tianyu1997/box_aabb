#!/usr/bin/env python3
"""Same-route Marcucci EP/Grid cache replay implementation."""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import os
import statistics
import sys
import time
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import (
    PAPER_THREADS,
    PAPER_STATISTICS_POLICY,
    ROOT,
    add_common_args,
    current_cpu_affinity,
    load_json,
    mode_args,
    require_python_extension,
    write_json,
)


ENDPOINTS = [("ifk", "IFK"), ("critsample", "CritSample")]
AUTHORITATIVE_SCRIPT = ROOT / "scripts" / "run_online_query_comparison.py"

ENVELOPE_VARIANTS = [
    {"key": "aabb_s4", "label": "AABB S=4", "env": "link_iaabb", "n_sub": 4, "voxel_delta": 0.04},
    {"key": "hull16_grid_d004", "label": "Hull16-grid d=0.04", "env": "hull16_grid", "n_sub": 1, "voxel_delta": 0.04},
]

CACHE_PHASES = [
    {"key": "no_cache", "label": "No V6 cache", "use_v6_cache": False, "strict": False, "preprobe": False, "paper_compare": True},
    {"key": "bake", "label": "Bake EP/Grid cache", "use_v6_cache": True, "strict": False, "preprobe": True, "paper_compare": False},
    {"key": "warm_bake", "label": "Warm-route EP/Grid bake", "use_v6_cache": True, "strict": False, "preprobe": True, "paper_compare": False},
    {"key": "cache_hit", "label": "Validated EP/Grid cache hit", "use_v6_cache": True, "strict": False, "preprobe": True, "paper_compare": True},
]

TIMING_FIELDS = [
    "lect_ms", "grow_ms", "grow_roots_ms", "grow_expand_ms", "grow_promotion_ms",
    "grow_ffb_total_ms", "grow_ffb_envelope_ms", "grow_ffb_collide_ms",
    "grow_ffb_expand_ms", "grow_ffb_intervals_ms", "grow_expand_calls",
    "grow_expand_new_nodes", "grow_expand_profile_total_ms",
    "grow_expand_pick_dim_ms", "grow_expand_fk_ms", "grow_expand_env_ms",
    "grow_expand_refine_ms",
    "coarsen1_ms", "coarsen1_sweep_ms", "coarsen1_relaxed_sweep_ms",
    "coarsen1_articulation_ms", "coarsen1_greedy_ms", "bridge_ms",
    "coarsen2_ms", "coarsen2_sweep_ms", "coarsen2_relaxed_sweep_ms",
    "coarsen2_articulation_ms", "coarsen2_greedy_ms", "coarsen2_cluster_ms",
    "filter_ms", "adjacency_ms", "adjacency_pre_seed_ms", "adjacency_final_ms",
    "seed_bridge_ms",
    "v6_cache_ep_hits", "v6_cache_ep_misses",
    "v6_cache_grid_hits", "v6_cache_grid_misses", "v6_cache_grid_compute_fallbacks",
    "v6_cache_ep_probe_calls", "v6_cache_ep_probe_slots", "v6_cache_ep_probe_max",
    "v6_cache_ep_lookup_bytes", "v6_cache_ep_insert_bytes", "v6_cache_ep_grow_calls",
    "v6_cache_ep_grow_ns", "v6_cache_grid_mem_hits", "v6_cache_grid_mem_misses",
    "v6_cache_grid_disk_hits", "v6_cache_grid_disk_misses", "v6_cache_grid_pread_calls",
    "v6_cache_grid_pread_bytes", "v6_cache_grid_pread_ns", "v6_cache_grid_pwrite_calls",
    "v6_cache_grid_pwrite_bytes", "v6_cache_grid_insert_ns", "v6_cache_grid_grow_calls",
    "v6_cache_grid_grow_ns", "v6_cache_grid_dead_bytes",
    "boxes_after_grow", "boxes_after_coarsen1", "boxes_after_bridge",
    "boxes_after_coarsen2", "boxes_final", "n_promotions", "total_ms",
]

PYTHON_EXTENSION_DIR = ROOT / "build-release" / "python"


def median(values: list[float]) -> float:
    return float(statistics.median(values)) if values else 0.0


def mean(values: list[float]) -> float:
    return float(statistics.fmean(values)) if values else 0.0


def mean_optional(values: list[float | int | None]) -> float | None:
    present = [float(value) for value in values if value is not None]
    return float(statistics.fmean(present)) if present else None


def sum_optional(values: list[float | int | None]) -> float | None:
    present = [float(value) for value in values if value is not None]
    return float(sum(present)) if present else None


def ensure_python_paths() -> None:
    for candidate in (PYTHON_EXTENSION_DIR, ROOT / "python"):
        text = str(candidate)
        if candidate.exists() and text not in sys.path:
            sys.path.insert(0, text)


def load_authoritative_module() -> Any:
    ensure_python_paths()
    spec = importlib.util.spec_from_file_location("v6_authoritative_online_query_comparison", AUTHORITATIVE_SCRIPT)
    if spec is None or spec.loader is None:
        raise ImportError(f"Unable to load authoritative script at {AUTHORITATIVE_SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def import_sbf6() -> Any:
    ensure_python_paths()
    import sbf6  # type: ignore
    extension = sys.modules.get("sbf6._sbf6_cpp")
    if extension is not None and "_sbf6_cpp" not in sys.modules:
        sys.modules["_sbf6_cpp"] = extension
    return sbf6


def make_endpoint_config(sbf5: Any, endpoint: str) -> Any:
    cfg = sbf5.EndpointSourceConfig()
    cfg.source = {"ifk": sbf5.EndpointSource.IFK, "critsample": sbf5.EndpointSource.CritSample}[endpoint]
    return cfg


def make_envelope_config(sbf5: Any, variant: dict[str, Any]) -> Any:
    cfg = sbf5.EnvelopeTypeConfig()
    cfg.type = {"link_iaabb": sbf5.EnvelopeType.LinkIAABB, "hull16_grid": sbf5.EnvelopeType.Hull16_Grid}[variant["env"]]
    cfg.n_subdivisions = int(variant["n_sub"])
    if variant["env"] == "hull16_grid":
        cfg.grid_config.voxel_delta = float(variant["voxel_delta"])
    return cfg


def box_signature(box: Any) -> tuple[tuple[float, float], ...]:
    return tuple((float(iv.lo), float(iv.hi)) for iv in box.joint_intervals)


def hash_boxes(boxes: list[Any]) -> str:
    digest = hashlib.sha256()
    digest.update(str(len(boxes)).encode("ascii"))
    for box in boxes:
        for lo, hi in box_signature(box):
            digest.update(f"{lo:.17g},{hi:.17g};".encode("ascii"))
        digest.update(b"|")
    return digest.hexdigest()


def box_volume_stats(boxes: list[Any]) -> dict[str, Any]:
    total_volume_sum = 0.0
    dedup_volume_sum = 0.0
    seen: set[tuple[tuple[float, float], ...]] = set()
    for box in boxes:
        volume = float(box.volume)
        total_volume_sum += volume
        key = box_signature(box)
        if key in seen:
            continue
        seen.add(key)
        dedup_volume_sum += volume
    return {
        "box_volume_sum": total_volume_sum,
        "dedup_box_volume_sum": dedup_volume_sum,
        "duplicate_box_volume_sum": max(0.0, total_volume_sum - dedup_volume_sum),
        "unique_box_count": len(seen),
        "duplicate_box_count": max(0, len(boxes) - len(seen)),
    }


def cache_file_metrics(cache_dir: Path) -> dict[str, Any]:
    files = sorted(path for path in cache_dir.rglob("*.cache") if path.is_file())
    by_name: dict[str, int] = {}
    total_bytes = 0
    for path in files:
        size = path.stat().st_size
        total_bytes += size
        by_name[path.name] = by_name.get(path.name, 0) + size
    return {
        "v6_cache_file_count": len(files),
        "v6_cache_file_bytes": total_bytes,
        "v6_cache_file_bytes_by_name": by_name,
        "v6_cache_files": [str(path) for path in files],
    }


def timing_profile_dict(planner: Any) -> dict[str, Any]:
    timing = planner.build_timing()
    return {field: getattr(timing, field, 0) for field in TIMING_FIELDS}


def make_planner_config(
    sbf5: Any,
    authoritative: Any,
    *,
    endpoint: str,
    variant: dict[str, Any],
    seed: int,
    threads: int,
    bridge_threads: int,
    timeout_s: int,
    ffb_depth: int,
    max_boxes: int,
    bridge_boxes: int,
    max_miss: int | None,
    cache_dir: Path,
    phase: dict[str, Any],
) -> Any:
    cfg = sbf5.SBFPlannerConfig()
    cfg.split_order = sbf5.SplitOrder.BEST_TIGHTEN
    cfg.z4_enabled = True
    cfg.endpoint_source = make_endpoint_config(sbf5, endpoint)
    cfg.envelope_type = make_envelope_config(sbf5, variant)
    authoritative.apply_paper_sbf_architecture(
        cfg,
        seed=seed,
        grow_timeout_ms=float(timeout_s) * 1000.0,
        max_boxes=max_boxes,
        post_connect_extra_boxes=bridge_boxes,
        n_threads=max(1, int(threads)),
        bridge_n_threads=max(1, int(bridge_threads)),
        ffb_depth=ffb_depth,
        lect_no_cache=False,
        lect_cache_dir=cache_dir,
    )
    if max_miss is not None:
        cfg.grower.max_consecutive_miss = int(max_miss)
    cfg.lect_no_cache = False
    cfg.lect_file_cache_load = False
    cfg.lect_file_cache_save = False
    cfg.use_v6_cache = bool(phase["use_v6_cache"])
    cfg.v6_cache_strict = bool(phase["strict"])
    cfg.lect_cache_dir = str(cache_dir)
    return cfg


def set_phase_preprobe(phase: dict[str, Any]) -> str | None:
    previous = os.environ.get("SBF_Z4_PREPROBE")
    if "preprobe" in phase:
        os.environ["SBF_Z4_PREPROBE"] = "1" if phase["preprobe"] else "0"
    return previous


def restore_phase_preprobe(previous: str | None) -> None:
    if previous is None:
        os.environ.pop("SBF_Z4_PREPROBE", None)
    else:
        os.environ["SBF_Z4_PREPROBE"] = previous


def run_trial(
    *,
    endpoint: str,
    endpoint_label: str,
    variant: dict[str, Any],
    seed: int,
    phase: dict[str, Any],
    cache_dir: Path,
    raw_path: Path,
    timeout_s: int,
    threads: int,
    bridge_threads: int,
    ffb_depth: int,
    max_boxes: int,
    bridge_boxes: int,
    max_miss: int | None = None,
) -> dict[str, Any]:
    sbf5 = import_sbf6()
    authoritative = load_authoritative_module()
    robot = sbf5.Robot.from_json(str(ROOT / "data" / "iiwa14.json"))
    obstacles = authoritative.make_combined_obstacles()
    seed_points = [authoritative.IIWA_CONFIGS[key] for key in ["AS", "TS", "CS", "LB", "RB"]]
    cfg = make_planner_config(
        sbf5,
        authoritative,
        endpoint=endpoint,
        variant=variant,
        seed=seed,
        threads=threads,
        bridge_threads=bridge_threads,
        timeout_s=timeout_s,
        ffb_depth=ffb_depth,
        max_boxes=max_boxes,
        bridge_boxes=bridge_boxes,
        max_miss=max_miss,
        cache_dir=cache_dir,
        phase=phase,
    )
    planner = sbf5.SBFPlanner(robot, cfg)
    previous_preprobe = set_phase_preprobe(phase)
    try:
        t0 = time.perf_counter()
        planner.build_coverage(obstacles, float(timeout_s) * 1000.0, seed_points)
        build_s = time.perf_counter() - t0

        raw_boxes = list(planner.raw_boxes())
        final_boxes = list(planner.boxes())
        timing = timing_profile_dict(planner)
        raw_box_count = len(raw_boxes)
        final_box_count = len(final_boxes)
        raw_route_hash = hash_boxes(raw_boxes)
        final_box_hash = hash_boxes(final_boxes)
        final_box_volume_stats = box_volume_stats(final_boxes)
        del raw_boxes, final_boxes

        queries = []
        for label, start_name, goal_name in authoritative.QUERY_PAIRS:
            result = planner.query(authoritative.IIWA_CONFIGS[start_name], authoritative.IIWA_CONFIGS[goal_name])
            queries.append({
                "name": label,
                "from": start_name,
                "to": goal_name,
                "ok": bool(result.success),
                "planning_time_ms": float(result.planning_time_ms),
                "path_length": float(result.path_length) if result.success else 0.0,
            })
    finally:
        restore_phase_preprobe(previous_preprobe)

    row: dict[str, Any] = {
        "source_protocol": "v6_same_route_ep_grid_cache_replay",
        "source_script": str(AUTHORITATIVE_SCRIPT),
        "endpoint_source": endpoint,
        "endpoint_label": endpoint_label,
        "envelope_key": variant["key"],
        "envelope_label": variant["label"],
        "env": variant["env"],
        "n_sub": variant["n_sub"],
        "voxel_delta": variant["voxel_delta"],
        "seed": seed,
        "cache_mode": phase["key"],
        "cache_mode_label": phase["label"],
        "paper_compare": bool(phase["paper_compare"]),
        "use_v6_cache": bool(phase["use_v6_cache"]),
        "v6_cache_strict": bool(phase["strict"]),
        "z4_preprobe": None if "preprobe" not in phase else bool(phase["preprobe"]),
        "lect_no_cache": False,
        "lect_file_cache_load": False,
        "lect_file_cache_save": False,
        "loaded_lect_cache": False,
        "cache_dir": str(cache_dir),
        "raw_path": str(raw_path),
        "build_s": float(build_s),
        "n_boxes": final_box_count,
        "raw_box_count": raw_box_count,
        "raw_route_hash": raw_route_hash,
        "final_box_hash": final_box_hash,
        **final_box_volume_stats,
        **cache_file_metrics(cache_dir),
        "build_timing": timing,
        "v6_cache_ep_hits": int(timing["v6_cache_ep_hits"]),
        "v6_cache_ep_misses": int(timing["v6_cache_ep_misses"]),
        "v6_cache_grid_hits": int(timing["v6_cache_grid_hits"]),
        "v6_cache_grid_misses": int(timing["v6_cache_grid_misses"]),
        "v6_cache_grid_compute_fallbacks": int(timing["v6_cache_grid_compute_fallbacks"]),
        "query_success_rate": query_success_rate(queries),
        "queries": queries,
    }
    write_json(raw_path, row)
    return row


def query_success_rate(queries: list[dict[str, Any]]) -> float:
    return sum(1 for query in queries if query.get("ok")) / len(queries) if queries else 0.0


def annotate_and_validate_triplet(rows: list[dict[str, Any]], *, allow_route_mismatch: bool) -> None:
    by_mode = {str(row["cache_mode"]): row for row in rows}
    reference = by_mode.get("no_cache") or by_mode.get("cold_fill")
    warm_reference = by_mode.get("bake") or reference
    replay = by_mode.get("cache_hit")
    if reference is None or replay is None:
        raise RuntimeError("Each cell must contain no_cache and cache_hit rows")
    ref_route = str(reference["raw_route_hash"])
    ref_final = str(reference["final_box_hash"])
    warm_route = str(warm_reference["raw_route_hash"]) if warm_reference is not None else ref_route
    for row in rows:
        row["reference_cache_mode"] = str(reference["cache_mode"])
        row["route_match_no_cache"] = str(row["raw_route_hash"]) == ref_route
        row["final_box_match_no_cache"] = str(row["final_box_hash"]) == ref_final
        row["route_match_warm_bake"] = str(row["raw_route_hash"]) == warm_route
    if warm_reference is not None and str(replay["raw_route_hash"]) != warm_route:
        raise RuntimeError("Validated cache-hit replay route does not match warm_bake route")
    mismatches = [row for row in rows if not row["route_match_no_cache"]]
    if str(reference["cache_mode"]) != "no_cache" and mismatches and not allow_route_mismatch:
        labels = ", ".join(str(row["cache_mode"]) for row in mismatches)
        raise RuntimeError(f"Route hash mismatch against {reference['cache_mode']} for modes: {labels}")
    strict_misses = (
        int(replay.get("v6_cache_ep_misses", 0))
        + int(replay.get("v6_cache_grid_misses", 0))
        + int(replay.get("v6_cache_grid_compute_fallbacks", 0))
    )
    total_lookups = (
        int(replay.get("v6_cache_ep_hits", 0))
        + int(replay.get("v6_cache_grid_hits", 0))
        + strict_misses
    )
    replay["cache_hit_miss_like_total"] = strict_misses
    replay["cache_hit_miss_like_rate"] = (strict_misses / total_lookups) if total_lookups else 0.0
    replay["strict_cache_all_hit"] = strict_misses == 0
    if replay["cache_hit_miss_like_rate"] > 1e-3:
        raise RuntimeError(
            f"Validated cache-hit replay miss/fallback rate too high: "
            f"{replay['cache_hit_miss_like_rate']:.6f}"
        )
    hit_s = float(replay["build_s"])
    replay["speedup_vs_no_cache"] = float(reference["build_s"]) / hit_s if hit_s > 0.0 else None
    ref_grow_s = float(reference.get("build_timing", {}).get("grow_ms", 0.0)) / 1000.0
    hit_grow_s = float(replay.get("build_timing", {}).get("grow_ms", 0.0)) / 1000.0
    replay["grow_speedup_vs_no_cache"] = ref_grow_s / hit_grow_s if hit_grow_s > 0.0 else None


def summarise(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    summary = []
    keys = sorted({
        (row["endpoint_source"], row["endpoint_label"], row["envelope_key"], row["envelope_label"],
         row["env"], row["n_sub"], row["voxel_delta"], row["cache_mode"], row["cache_mode_label"])
        for row in rows
    })
    for endpoint, endpoint_label, env_key, env_label, env, n_sub, voxel_delta, cache_mode, cache_label in keys:
        group = [row for row in rows if row["endpoint_source"] == endpoint and row["envelope_key"] == env_key and row["cache_mode"] == cache_mode]
        builds = [float(row["build_s"]) for row in group]
        grows = [float(row.get("build_timing", {}).get("grow_ms", 0.0)) / 1000.0 for row in group]
        ffb_totals = [float(row.get("build_timing", {}).get("grow_ffb_total_ms", 0.0)) / 1000.0 for row in group]
        ffb_collides = [float(row.get("build_timing", {}).get("grow_ffb_collide_ms", 0.0)) / 1000.0 for row in group]
        coarsens = [
            (
                float(row.get("build_timing", {}).get("coarsen1_ms", 0.0))
                + float(row.get("build_timing", {}).get("coarsen2_ms", 0.0))
            ) / 1000.0
            for row in group
        ]
        post_grow = [max(0.0, build - grow) for build, grow in zip(builds, grows)]
        boxes = [float(row["n_boxes"]) for row in group]
        raw_boxes = [float(row["raw_box_count"]) for row in group]
        cache_file_bytes = [row.get("v6_cache_file_bytes") for row in group]
        mean_cache_bytes = mean_optional(cache_file_bytes)
        raw_median = median(raw_boxes)
        summary.append({
            "endpoint_source": endpoint,
            "endpoint_label": endpoint_label,
            "envelope_key": env_key,
            "envelope_label": env_label,
            "env": env,
            "n_sub": n_sub,
            "voxel_delta": voxel_delta,
            "cache_mode": cache_mode,
            "cache_mode_label": cache_label,
            "n_runs": len(group),
            "paper_compare": any(bool(row.get("paper_compare")) for row in group),
            "median_build_s": median(builds),
            "mean_build_s": mean(builds),
            "median_grow_s": median(grows),
            "mean_grow_s": mean(grows),
            "median_grow_ffb_s": median(ffb_totals),
            "median_grow_ffb_collide_s": median(ffb_collides),
            "median_coarsen_s": median(coarsens),
            "median_post_grow_s": median(post_grow),
            "median_n_boxes": median(boxes),
            "median_raw_box_count": raw_median,
            "median_unique_box_count": median([float(row.get("unique_box_count", row["n_boxes"])) for row in group]),
            "median_box_volume_sum": median([float(row.get("box_volume_sum", 0.0)) for row in group]),
            "median_dedup_box_volume_sum": median([float(row.get("dedup_box_volume_sum", row.get("box_volume_sum", 0.0))) for row in group]),
            "query_success_rate_mean": mean([float(row["query_success_rate"]) for row in group]),
            "route_match_rate": mean([1.0 if row.get("route_match_no_cache") else 0.0 for row in group]),
            "final_box_match_rate": mean([1.0 if row.get("final_box_match_no_cache") else 0.0 for row in group]),
            "mean_v6_cache_file_bytes": mean_cache_bytes,
            "mean_v6_cache_file_bytes_per_raw_box": mean_cache_bytes / raw_median if raw_median > 0 and mean_cache_bytes is not None else None,
            "sum_v6_cache_ep_hits": sum_optional([row.get("v6_cache_ep_hits") for row in group]),
            "sum_v6_cache_ep_misses": sum_optional([row.get("v6_cache_ep_misses") for row in group]),
            "sum_v6_cache_grid_hits": sum_optional([row.get("v6_cache_grid_hits") for row in group]),
            "sum_v6_cache_grid_misses": sum_optional([row.get("v6_cache_grid_misses") for row in group]),
            "sum_v6_cache_grid_compute_fallbacks": sum_optional([row.get("v6_cache_grid_compute_fallbacks") for row in group]),
            "sum_cache_hit_miss_like_total": sum_optional([row.get("cache_hit_miss_like_total") for row in group]),
            "mean_cache_hit_miss_like_rate": mean_optional([row.get("cache_hit_miss_like_rate") for row in group]),
        })
    return summary


def comparisons(summary: list[dict[str, Any]]) -> list[dict[str, Any]]:
    by_key: dict[tuple[str, str], dict[str, dict[str, Any]]] = {}
    for row in summary:
        by_key.setdefault((str(row["endpoint_source"]), str(row["envelope_key"])), {})[str(row["cache_mode"])] = row
    out = []
    for key in sorted(by_key):
        no_cache = by_key[key].get("cold_fill") or by_key[key].get("no_cache")
        cache_hit = by_key[key].get("cache_hit")
        if no_cache is None or cache_hit is None:
            continue
        no_cache_s = float(no_cache["median_build_s"])
        cache_hit_s = float(cache_hit["median_build_s"])
        no_cache_grow_s = float(no_cache.get("median_grow_s") or no_cache_s)
        cache_hit_grow_s = float(cache_hit.get("median_grow_s") or cache_hit_s)
        out.append({
            "endpoint_source": key[0],
            "endpoint_label": cache_hit.get("endpoint_label", key[0]),
            "envelope_key": key[1],
            "envelope_label": cache_hit.get("envelope_label", key[1]),
            "cold_fill_median_grow_s": no_cache_grow_s,
            "cache_hit_median_grow_s": cache_hit_grow_s,
            "grow_speedup": no_cache_grow_s / cache_hit_grow_s if cache_hit_grow_s > 0.0 else None,
            "no_cache_median_build_s": no_cache_s,
            "cache_hit_median_build_s": cache_hit_s,
            "total_build_speedup": no_cache_s / cache_hit_s if cache_hit_s > 0.0 else None,
            "speedup": no_cache_grow_s / cache_hit_grow_s if cache_hit_grow_s > 0.0 else None,
            "median_n_boxes": cache_hit.get("median_n_boxes"),
            "median_raw_box_count": cache_hit.get("median_raw_box_count"),
            "median_box_volume_sum": cache_hit.get("median_box_volume_sum"),
            "median_dedup_box_volume_sum": cache_hit.get("median_dedup_box_volume_sum"),
            "query_success_rate_mean": cache_hit.get("query_success_rate_mean"),
            "route_match_rate": cache_hit.get("route_match_rate"),
            "ep_hits": cache_hit.get("sum_v6_cache_ep_hits"),
            "ep_misses": cache_hit.get("sum_v6_cache_ep_misses"),
            "grid_hits": cache_hit.get("sum_v6_cache_grid_hits"),
            "grid_misses": cache_hit.get("sum_v6_cache_grid_misses"),
            "grid_fallbacks": cache_hit.get("sum_v6_cache_grid_compute_fallbacks"),
            "cache_hit_miss_like_total": cache_hit.get("sum_cache_hit_miss_like_total"),
            "cache_hit_miss_like_rate": cache_hit.get("mean_cache_hit_miss_like_rate"),
            "mean_v6_cache_file_bytes": cache_hit.get("mean_v6_cache_file_bytes"),
            "mean_v6_cache_file_bytes_per_raw_box": cache_hit.get("mean_v6_cache_file_bytes_per_raw_box"),
        })
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_common_args(parser)
    parser.add_argument("--threads", type=int, default=PAPER_THREADS)
    parser.add_argument("--bridge-threads", type=int, default=PAPER_THREADS)
    parser.add_argument("--ffb-depth", type=int, default=300)
    parser.add_argument("--max-boxes", type=int, default=200000)
    parser.add_argument("--bridge-boxes", type=int, default=4000)
    parser.add_argument("--max-miss", type=int, default=None, help="override grower.max_consecutive_miss")
    parser.add_argument("--allow-route-mismatch", action="store_true")
    parser.add_argument("--resume", action="store_true", help="skip a cache-replay cell when all raw phase outputs exist")
    parser.add_argument("--cache-run-id", default=None, help="cache namespace for this run; defaults to a fresh timestamp")
    args = parser.parse_args()

    global PYTHON_EXTENSION_DIR
    PYTHON_EXTENSION_DIR = require_python_extension(args)
    seeds, timeout, mode = mode_args(args, quick_seeds=1, full_seeds=10, quick_timeout=60, full_timeout=60)
    base_dir = args.out_dir / "marcucci_envelope_build"
    raw_dir = base_dir / "raw"
    cache_run_id = str(args.cache_run_id or time.strftime("run_%Y%m%d_%H%M%S"))
    cache_dir = base_dir / "v6_ep_grid_cache" / cache_run_id
    rows: list[dict[str, Any]] = []

    for endpoint, endpoint_label in ENDPOINTS:
        for variant in ENVELOPE_VARIANTS:
            for seed in range(seeds):
                stem = f"{endpoint}_{variant['key']}_seed{seed:03d}"
                cell_cache_dir = cache_dir / stem
                outputs = {phase["key"]: raw_dir / f"{stem}_{phase['key']}.json" for phase in CACHE_PHASES}
                triplet_done = all(path.exists() for path in outputs.values())
                triplet_rows: list[dict[str, Any]] = []
                if args.resume and triplet_done:
                    print(f"[skip] {stem} cache-replay phases")
                    triplet_rows = [load_json(outputs[phase["key"]]) for phase in CACHE_PHASES]
                else:
                    for phase in CACHE_PHASES:
                        out_path = outputs[phase["key"]]
                        if args.dry_run:
                            print(
                                "$ "
                                f"{sys.executable} {Path(__file__).name} {mode} "
                                f"--threads {args.threads} --bridge-threads {args.bridge_threads} "
                                f"--ffb-depth {args.ffb_depth} --max-boxes {args.max_boxes} "
                                f"--bridge-boxes {args.bridge_boxes} "
                                f"[{endpoint} {variant['key']} seed={seed} {phase['key']}]"
                            )
                            continue
                        triplet_rows.append(run_trial(
                            endpoint=endpoint,
                            endpoint_label=endpoint_label,
                            variant=variant,
                            seed=seed,
                            phase=phase,
                            cache_dir=cell_cache_dir,
                            raw_path=out_path,
                            timeout_s=timeout,
                            threads=args.threads,
                            bridge_threads=args.bridge_threads,
                            ffb_depth=args.ffb_depth,
                            max_boxes=args.max_boxes,
                            bridge_boxes=args.bridge_boxes,
                            max_miss=args.max_miss,
                        ))
                if args.dry_run:
                    continue
                annotate_and_validate_triplet(triplet_rows, allow_route_mismatch=bool(args.allow_route_mismatch))
                for row in triplet_rows:
                    write_json(Path(str(row["raw_path"])), row)
                rows.extend(triplet_rows)

    if args.dry_run:
        print("[dry-run] commands emitted; no marcucci_envelope_build.json written")
        return
    summary = summarise(rows)
    out = {
        "experiment": "marcucci_envelope_build",
        "schema_version": 2,
        "scene": "marcucci_combined",
        "runner": "v6_python_sbfplanner_same_route_cache_replay",
        "source_script": str(AUTHORITATIVE_SCRIPT),
        "protocol": {
            "lect_file_cache_load": False,
            "lect_file_cache_save": False,
            "route_hash": "sha256(raw pre-coarsen box interval sequence)",
            "phases": CACHE_PHASES,
        },
        "statistical_policy": PAPER_STATISTICS_POLICY["exp3"],
        "resource_policy": {
            "logical_threads": args.threads,
            "bridge_threads": args.bridge_threads,
            "cpu_affinity": current_cpu_affinity(),
            "seed_execution": "serial",
        },
        "defaults": {
            "seeds": seeds,
            "timeout": timeout,
            "threads": args.threads,
            "bridge_threads": args.bridge_threads,
            "ffb_depth": args.ffb_depth,
            "max_boxes": args.max_boxes,
            "bridge_boxes": args.bridge_boxes,
            "max_miss": args.max_miss,
            "cache_run_id": cache_run_id,
            "cache_root": str(cache_dir),
        },
        "endpoints": [{"key": key, "label": label} for key, label in ENDPOINTS],
        "envelope_variants": ENVELOPE_VARIANTS,
        "rows": rows,
        "summary": summary,
        "comparisons": comparisons(summary),
    }
    write_json(args.out_dir / "marcucci_envelope_build.json", out)
    print(f"[write] {args.out_dir / 'marcucci_envelope_build.json'}")


if __name__ == "__main__":
    main()
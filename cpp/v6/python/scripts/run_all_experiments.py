#!/usr/bin/env python3
"""
Run all experiments and generate all paper outputs.

Usage (from v5/):
    $env:PYTHONPATH = "build_x64/Release;python"
    python python/scripts/run_all_experiments.py [--skip-experiments] [--skip-figures]

Outputs → experiments/results/
"""

import argparse
import json
import logging
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
logger = logging.getLogger(__name__)

PAPER_DIR = "experiments/results"
MC_SAMPLES_DEFAULT = 50000


# ── Helpers ──────────────────────────────────────────────────

def _find_gcpc_cache_path(robot_key: str):
    """Find GCPC cache path for a robot key if available."""
    candidates = [
        os.path.join("data", f"{robot_key}.gcpc"),
        os.path.join("data", f"{robot_key}_5000.gcpc"),
        os.path.join("data", f"{robot_key}_500.gcpc"),
    ]
    for p in candidates:
        if os.path.exists(p):
            return p
    return None

def _random_intervals(robot, rng, width_lo=0.1, width_hi=0.5):
    """Generate random joint intervals within joint limits."""
    import sbf5
    lims = robot.joint_limits().limits
    intervals = []
    for lim in lims:
        width = rng.uniform(width_lo, width_hi)
        lo = rng.uniform(lim.lo, max(lim.lo, lim.hi - width))
        hi = min(lo + width, lim.hi)
        intervals.append(sbf5.Interval(float(lo), float(hi)))
    return intervals


def _make_ep_config(source_name, gcpc_match_analytical=False, mc_samples=None):
    """Create EndpointSourceConfig from name."""
    import sbf5
    cfg = sbf5.EndpointSourceConfig()
    cfg.source = {
        "IFK": sbf5.EndpointSource.IFK,
        "CritSample": sbf5.EndpointSource.CritSample,
        "Analytical": sbf5.EndpointSource.Analytical,
        "GCPC": sbf5.EndpointSource.GCPC,
        "MC": sbf5.EndpointSource.MC,
    }[source_name]
    if source_name == "GCPC":
        cfg.gcpc_match_analytical = bool(gcpc_match_analytical)
    if source_name == "MC" and mc_samples is not None:
        cfg.n_samples_crit = int(mc_samples)
    return cfg


def _make_env_config(type_name, subdivisions=1, grid_delta=None):
    """Create EnvelopeTypeConfig from name."""
    import sbf5
    cfg = sbf5.EnvelopeTypeConfig()
    cfg.type = {
        "LinkIAABB": sbf5.EnvelopeType.LinkIAABB,
        "LinkIAABB_Grid": sbf5.EnvelopeType.LinkIAABB_Grid,
        "Hull16_Grid": sbf5.EnvelopeType.Hull16_Grid,
    }[type_name]
    cfg.n_subdivisions = int(subdivisions)
    if grid_delta is not None:
        cfg.grid_config.voxel_delta = float(grid_delta)
    return cfg


ALL_EP_SOURCES = ["IFK", "CritSample", "Analytical", "GCPC", "MC"]
ALL_ENV_CONFIGS = [
    {"name": "LinkIAABB", "subdivisions": 1},
    {"name": "LinkIAABB", "subdivisions": 4},
    {"name": "LinkIAABB", "subdivisions": 8},
    {"name": "LinkIAABB_Grid", "subdivisions": 1},
    {"name": "LinkIAABB_Grid", "subdivisions": 4},
    {"name": "LinkIAABB_Grid", "subdivisions": 8},
    {"name": "Hull16_Grid", "subdivisions": 1},
]
QUICK_ENV_CONFIGS = [
    {"name": "LinkIAABB", "subdivisions": 1},
    {"name": "LinkIAABB_Grid", "subdivisions": 1},
]

SUB_SWEEP_VALUES = [1, 2, 4, 8]
GRID_DELTA_SWEEP_VALUES = [0.02, 0.04, 0.06, 0.08]
WIDTH_BIN_DEFS = [
    ("W1_0.10_0.20", 0.10, 0.20),
    ("W2_0.20_0.30", 0.20, 0.30),
    ("W3_0.30_0.40", 0.30, 0.40),
    ("W4_0.40_0.50", 0.40, 0.50),
]
EP_WIDTH_SOURCES = ["IFK", "CritSample", "Analytical", "GCPC"]


def _extent_from_iaabbs(flat_iaabbs):
    arr = np.asarray(flat_iaabbs, dtype=float)
    if arr.size == 0:
        return np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0])
    arr = arr.reshape((-1, 6))
    lo = np.min(arr[:, 0:3], axis=0)
    hi = np.max(arr[:, 3:6], axis=0)
    return lo, hi


def run_s0_ep_width_profile(quick: bool = False, lite: bool = False):
    """S0b: Endpoint-source profiling across interval-width strata.

    Width is independent from envelope type, so this study compares
    endpoint sources directly on endpoint IAABB volume/time/gap only.
    """
    import sbf5

    out_dir = os.path.join(PAPER_DIR, "s0_ep_width_profile")
    os.makedirs(out_dir, exist_ok=True)
    result_path = os.path.join(out_dir, "results.json")

    n_per_bin = 80 if quick else (150 if lite else 300)
    robot = sbf5.Robot.from_json(os.path.join("data", "iiwa14.json"))

    gcpc_cache = None
    gcpc_path = _find_gcpc_cache_path("iiwa14")
    if gcpc_path is not None:
        gcpc_cache = sbf5.GcpcCache.load(gcpc_path)
        logger.info("S0b: loaded GCPC cache %s (%d pts)", gcpc_path, gcpc_cache.n_points())
    else:
        logger.warning("S0b: no GCPC cache found, GCPC rows will be skipped")

    rows = []

    for bin_name, w_lo, w_hi in WIDTH_BIN_DEFS:
        rng = np.random.RandomState(3100 + int(w_lo * 100))
        boxes = [_random_intervals(robot, rng, width_lo=w_lo, width_hi=w_hi)
                 for _ in range(n_per_bin)]
        logger.info("S0b: %s width=[%.2f, %.2f], n=%d", bin_name, w_lo, w_hi, n_per_bin)

        per_source = {
            s: {
                "vol": [],
                "time": [],
                "vol_gap_vs_analytical": [],
                "extent_gap_mean": [],
                "extent_gap_min": [],
            }
            for s in EP_WIDTH_SOURCES
        }

        for i, intervals in enumerate(boxes):
            trial = {}
            for src in EP_WIDTH_SOURCES:
                if src == "GCPC" and gcpc_cache is None:
                    continue
                ep_cfg = _make_ep_config(src)
                info = sbf5.compute_endpoint_iaabb_info(
                    robot,
                    intervals,
                    ep_cfg,
                    gcpc_cache if src == "GCPC" else None,
                )
                lo, hi = _extent_from_iaabbs(info["endpoint_iaabbs"])
                trial[src] = {
                    "volume": float(info["volume_sum"]),
                    "time_us": float(info["ep_time_us"]),
                    "extent": (lo, hi),
                }

            if "Analytical" not in trial:
                continue
            ref_vol = trial["Analytical"]["volume"]
            ref_lo, ref_hi = trial["Analytical"]["extent"]
            ref_extent = ref_hi - ref_lo

            for src, data in trial.items():
                cur_lo, cur_hi = data["extent"]
                cur_extent = cur_hi - cur_lo
                extent_gap = cur_extent - ref_extent
                per_source[src]["vol"].append(data["volume"])
                per_source[src]["time"].append(data["time_us"])
                per_source[src]["vol_gap_vs_analytical"].append(data["volume"] - ref_vol)
                per_source[src]["extent_gap_mean"].append(extent_gap)
                per_source[src]["extent_gap_min"].append(extent_gap)

            if (i + 1) % 100 == 0:
                logger.info("    %s: %d/%d boxes", bin_name, i + 1, n_per_bin)

        for src in EP_WIDTH_SOURCES:
            vals = per_source[src]
            if len(vals["vol"]) == 0:
                continue
            extent_mean = np.mean(np.vstack(vals["extent_gap_mean"]), axis=0)
            extent_min = np.min(np.vstack(vals["extent_gap_min"]), axis=0)
            rows.append({
                "robot": "iiwa14",
                "width_bin": bin_name,
                "width_lo": float(w_lo),
                "width_hi": float(w_hi),
                "endpoint": src,
                "n_boxes": int(len(vals["vol"])),
                "ep_volume_mean": float(np.mean(vals["vol"])),
                "ep_volume_std": float(np.std(vals["vol"])),
                "ep_time_us_mean": float(np.mean(vals["time"])),
                "ep_time_us_std": float(np.std(vals["time"])),
                "vol_gap_vs_analytical_mean": float(np.mean(vals["vol_gap_vs_analytical"])),
                "vol_gap_vs_analytical_std": float(np.std(vals["vol_gap_vs_analytical"])),
                "extent_gap_vs_analytical_mean": [float(x) for x in extent_mean],
                "extent_gap_vs_analytical_min": [float(x) for x in extent_min],
            })

    payload = {
        "meta": {
            "robot": "iiwa14",
            "sources": [s for s in EP_WIDTH_SOURCES if not (s == "GCPC" and gcpc_cache is None)],
            "width_bins": [
                {"name": n, "lo": lo, "hi": hi}
                for (n, lo, hi) in WIDTH_BIN_DEFS
            ],
            "n_per_bin": n_per_bin,
            "gap_reference": "Analytical",
        },
        "rows": rows,
    }
    with open(result_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
    logger.info("S0b: → %s (%d rows)", result_path, len(rows))


def run_s0_param_selection(quick: bool = False, lite: bool = False):
    """S0: Preselect sub/grid params using CritSample on IIWA14.

    The study runs before full-grid experiments and provides a justified
    default for full-pipeline settings.
    """
    import sbf5

    out_dir = os.path.join(PAPER_DIR, "s0_param_selection")
    os.makedirs(out_dir, exist_ok=True)
    result_path = os.path.join(out_dir, "results.json")

    n_boxes = 80 if quick else (200 if lite else 500)
    n_repeats = 5 if quick else (10 if lite else 20)
    fixed_delta = 0.04
    fixed_sub = 4

    robot = sbf5.Robot.from_json(os.path.join("data", "iiwa14.json"))
    ep_cfg = _make_ep_config("CritSample")

    rng = np.random.RandomState(2026)
    box_list = [_random_intervals(robot, rng) for _ in range(n_boxes)]

    def _eval(subdivisions, grid_delta):
        env_cfg = _make_env_config(
            "LinkIAABB_Grid", subdivisions=subdivisions, grid_delta=grid_delta)
        volumes = []
        total_us = []
        cache_bricks = []
        cache_voxels = []
        cache_payload_bytes = []

        for bi, box in enumerate(box_list):
            for rep in range(n_repeats):
                info = sbf5.compute_envelope_info(robot, box, ep_cfg, env_cfg, None)
                total_us.append(info["total_time_us"])
                if rep == 0:
                    volumes.append(info["volume"])
                    cache_bricks.append(info.get("grid_num_bricks", 0))
                    cache_voxels.append(info.get("grid_num_voxels", 0))
                    cache_payload_bytes.append(info.get("grid_cache_payload_bytes", 0.0))
            if (bi + 1) % 100 == 0:
                logger.info("S0: sub=%d delta=%.3f — %d/%d boxes",
                            subdivisions, grid_delta, bi + 1, n_boxes)

        return {
            "volume_mean": float(np.mean(volumes)),
            "volume_std": float(np.std(volumes)),
            "total_us_mean": float(np.mean(total_us)),
            "total_us_std": float(np.std(total_us)),
            "cache_bricks_mean": float(np.mean(cache_bricks)),
            "cache_voxels_mean": float(np.mean(cache_voxels)),
            "cache_payload_bytes_mean": float(np.mean(cache_payload_bytes)),
        }

    rows = []

    logger.info("S0: sub sweep with fixed delta=%.3f", fixed_delta)
    for sub in SUB_SWEEP_VALUES:
        stats = _eval(subdivisions=sub, grid_delta=fixed_delta)
        rows.append({
            "study": "sub",
            "endpoint": "CritSample",
            "envelope": "LinkIAABB_Grid",
            "subdivisions": int(sub),
            "grid_delta": float(fixed_delta),
            **stats,
        })

    sub_rows = sorted([r for r in rows if r["study"] == "sub"],
                      key=lambda r: int(r["subdivisions"]))
    recommended_sub = sub_rows[-1]["subdivisions"]
    for i in range(1, len(sub_rows)):
        prev = sub_rows[i - 1]
        cur = sub_rows[i]
        tightness_gain = (prev["volume_mean"] - cur["volume_mean"]) / max(prev["volume_mean"], 1e-9)
        time_increase = (cur["total_us_mean"] - prev["total_us_mean"]) / max(prev["total_us_mean"], 1e-9)
        if tightness_gain < 0.03 and time_increase > 0.25:
            recommended_sub = prev["subdivisions"]
            break

    logger.info("S0: grid-delta sweep with fixed sub=%d", recommended_sub)
    for delta in GRID_DELTA_SWEEP_VALUES:
        stats = _eval(subdivisions=recommended_sub, grid_delta=delta)
        rows.append({
            "study": "grid",
            "endpoint": "CritSample",
            "envelope": "LinkIAABB_Grid",
            "subdivisions": int(recommended_sub),
            "grid_delta": float(delta),
            **stats,
        })

    grid_rows = sorted([r for r in rows if r["study"] == "grid"],
                       key=lambda r: float(r["grid_delta"]))
    best_vol = min(r["volume_mean"] for r in grid_rows)
    # Keep grid resolution that is near-best in volume while avoiding
    # extreme cache/time blow-up at very fine deltas.
    candidates = [r for r in grid_rows if r["volume_mean"] <= best_vol * 1.30]
    if not candidates:
        candidates = grid_rows
    recommended_delta = min(candidates, key=lambda r: r["total_us_mean"])["grid_delta"]

    payload = {
        "meta": {
            "robot": "iiwa14",
            "endpoint": "CritSample",
            "envelope": "LinkIAABB_Grid",
            "n_boxes": n_boxes,
            "n_repeats": n_repeats,
            "fixed_delta_for_sub_sweep": fixed_delta,
            "sub_sweep_values": SUB_SWEEP_VALUES,
            "grid_delta_sweep_values": GRID_DELTA_SWEEP_VALUES,
            "recommended_subdivisions": int(recommended_sub),
            "recommended_grid_delta": float(recommended_delta),
        },
        "rows": rows,
    }
    with open(result_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)

    logger.info("S0: recommended sub=%s, delta=%.3f", recommended_sub, recommended_delta)
    logger.info("S0: → %s (%d rows)", result_path, len(rows))


def _env_variant_label(env_name, subdivisions):
    return f"{env_name}(sub={int(subdivisions)})"


def _volume_mode_for_envelope(env_name):
    if env_name == "LinkIAABB":
        return "inflated_aabb_union_exact_v2"
    return "grid_occupied"


def _normalize_env_row(row):
    """Backfill subdivision fields for legacy rows."""
    env = row.get("envelope", "")
    sub = int(row.get("subdivisions", 1))
    row["subdivisions"] = sub
    row["envelope_variant"] = row.get("envelope_variant", _env_variant_label(env, sub))


def _row_key(row):
    return (
        row.get("robot", ""),
        row.get("endpoint", ""),
        row.get("envelope", ""),
        int(row.get("subdivisions", 1)),
    )

def run_s1(quick: bool = False, lite: bool = False,
           gcpc_match_analytical: bool = False,
           mc_samples: int = MC_SAMPLES_DEFAULT,
           ep_sources_override=None):
    """S1: Envelope Tightness — compare volumes across pipeline configs."""
    import sbf5

    out_dir = os.path.join(PAPER_DIR, "s1_envelope_tightness")
    os.makedirs(out_dir, exist_ok=True)
    result_path = os.path.join(out_dir, "results.json")
    all_rows = []
    if os.path.exists(result_path):
        with open(result_path, "r", encoding="utf-8") as f:
            all_rows = json.load(f).get("rows", [])
        for r in all_rows:
            _normalize_env_row(r)

        # Recompute LinkIAABB rows if they were produced by old volume semantics
        # (inflated box volumes summed without overlap deduplication).
        kept_rows = []
        dropped = 0
        for r in all_rows:
            if r.get("envelope") == "LinkIAABB":
                if r.get("volume_mode") != _volume_mode_for_envelope("LinkIAABB"):
                    dropped += 1
                    continue
            kept_rows.append(r)
        all_rows = kept_rows
        if dropped > 0:
            logger.info("S1: dropped %d stale LinkIAABB rows for recomputation", dropped)

        logger.info("S1: loaded existing results (%d rows), running incremental fill", len(all_rows))

    n_boxes = 50 if quick else (200 if lite else 500)
    ep_sources = ALL_EP_SOURCES[:2] if quick else ALL_EP_SOURCES
    if ep_sources_override is not None:
        ep_sources = [s for s in ep_sources_override if s in ALL_EP_SOURCES]
    env_configs = QUICK_ENV_CONFIGS if quick else ALL_ENV_CONFIGS

    robots = {
        "iiwa14": sbf5.Robot.from_json(os.path.join("data", "iiwa14.json")),
    }

    # Load GCPC caches
    gcpc_caches = {}
    if "GCPC" in ep_sources:
        for rname, robj in robots.items():
            cache_path = _find_gcpc_cache_path(rname)
            if cache_path is not None:
                gcpc_caches[rname] = sbf5.GcpcCache.load(cache_path)
                logger.info("S1: loaded GCPC cache %s (%d pts)", cache_path, gcpc_caches[rname].n_points())

    existing_keys = {_row_key(r) for r in all_rows}
    newly_computed = 0
    for rname, robot in robots.items():
        rng = np.random.RandomState(42)
        logger.info("S1: %s — sampling %d boxes", rname, n_boxes)
        # Keep identical interval samples across endpoint sources/envelopes
        # so volume comparisons are on the same box set.
        box_list = [_random_intervals(robot, rng) for _ in range(n_boxes)]

        for ep in ep_sources:
            if ep == "GCPC" and rname not in gcpc_caches:
                logger.warning("S1: skip GCPC for %s (no cache)", rname)
                continue

            ep_cfg = _make_ep_config(
                ep,
                gcpc_match_analytical=gcpc_match_analytical,
                mc_samples=(mc_samples if ep == "MC" else None),
            )
            gcpc = gcpc_caches.get(rname) if ep == "GCPC" else None

            for env_spec in env_configs:
                env = env_spec["name"]
                subdivisions = int(env_spec["subdivisions"])
                key = (rname, ep, env, subdivisions)
                if key in existing_keys:
                    continue
                env_cfg = _make_env_config(env, subdivisions)
                env_label = _env_variant_label(env, subdivisions)
                volumes = []
                safe_flags = []
                times_us = []

                for intervals in box_list:
                    info = sbf5.compute_envelope_info(
                        robot, intervals, ep_cfg, env_cfg, gcpc)
                    volumes.append(info["volume"])
                    safe_flags.append(info["is_safe"])
                    times_us.append(info["total_time_us"])

                row = {
                    "robot": rname,
                    "endpoint": ep,
                    "envelope": env,
                    "subdivisions": subdivisions,
                    "envelope_variant": env_label,
                    "volume_mode": _volume_mode_for_envelope(env),
                    "volume_mean": float(np.mean(volumes)),
                    "volume_std": float(np.std(volumes)),
                    "safe": bool(all(safe_flags)),
                    "time_us_mean": float(np.mean(times_us)),
                    "n_boxes": n_boxes,
                }
                all_rows.append(row)
                existing_keys.add(key)
                newly_computed += 1
                logger.info("  %s-%s: vol=%.4f ± %.4f, safe=%s",
                            ep, env_label, row["volume_mean"], row["volume_std"],
                            row["safe"])

    if newly_computed == 0:
        logger.info("S1: no missing rows detected")
    else:
        logger.info("S1: computed %d new rows", newly_computed)

    # Compute ratio vs IFK-LinkIAABB baseline per robot
    for rname in robots:
        baseline = next(
            (r for r in all_rows
             if r["robot"] == rname and r["endpoint"] == "IFK"
             and r["envelope"] == "LinkIAABB"
             and int(r.get("subdivisions", 1)) == 1), None)
        base_vol = baseline["volume_mean"] if baseline else 1.0
        for r in all_rows:
            if r["robot"] == rname:
                r["ratio_pct"] = round(100.0 * r["volume_mean"] / base_vol, 1) if base_vol > 0 else 0.0

    with open(result_path, "w", encoding="utf-8") as f:
        json.dump({"rows": all_rows}, f, indent=2)
    logger.info("S1: → %s (%d rows)", result_path, len(all_rows))


def run_s2(quick: bool = False, lite: bool = False,
           mc_samples: int = MC_SAMPLES_DEFAULT,
           ep_sources_override=None):
    """S2: Envelope Timing — measure endpoint + envelope computation time."""
    import sbf5

    out_dir = os.path.join(PAPER_DIR, "s2_envelope_timing")
    os.makedirs(out_dir, exist_ok=True)
    result_path = os.path.join(out_dir, "results.json")
    all_rows = []
    if os.path.exists(result_path):
        with open(result_path, "r", encoding="utf-8") as f:
            all_rows = json.load(f).get("rows", [])
        for r in all_rows:
            _normalize_env_row(r)
        logger.info("S2: loaded existing results (%d rows), running incremental fill", len(all_rows))

    n_boxes = 100 if quick else (300 if lite else 1000)
    n_repeats = 10 if quick else (20 if lite else 50)
    ep_sources = ALL_EP_SOURCES[:2] if quick else ALL_EP_SOURCES
    if ep_sources_override is not None:
        ep_sources = [s for s in ep_sources_override if s in ALL_EP_SOURCES]
    env_configs = QUICK_ENV_CONFIGS if quick else ALL_ENV_CONFIGS

    robots = {
        "iiwa14": sbf5.Robot.from_json(os.path.join("data", "iiwa14.json")),
    }

    gcpc_caches = {}
    if "GCPC" in ep_sources:
        for rname in robots:
            cache_path = _find_gcpc_cache_path(rname)
            if cache_path is not None:
                gcpc_caches[rname] = sbf5.GcpcCache.load(cache_path)

    existing_keys = {_row_key(r) for r in all_rows}

    def _checkpoint_rows():
        # Persist incremental progress for long S2 runs so interrupted jobs
        # can resume from already-computed rows.
        with open(result_path, "w", encoding="utf-8") as f:
            json.dump({"rows": all_rows}, f, indent=2)

    newly_computed = 0
    for rname, robot in robots.items():
        rng = np.random.RandomState(123)
        # Pre-sample boxes
        box_list = [_random_intervals(robot, rng) for _ in range(n_boxes)]
        logger.info("S2: %s — timing %d boxes × %d repeats", rname, n_boxes, n_repeats)

        for ep in ep_sources:
            if ep == "GCPC" and rname not in gcpc_caches:
                continue
            ep_cfg = _make_ep_config(ep, mc_samples=(mc_samples if ep == "MC" else None))
            gcpc = gcpc_caches.get(rname) if ep == "GCPC" else None

            for env_spec in env_configs:
                env = env_spec["name"]
                subdivisions = int(env_spec["subdivisions"])
                key = (rname, ep, env, subdivisions)
                if key in existing_keys:
                    continue
                env_cfg = _make_env_config(env, subdivisions)
                env_label = _env_variant_label(env, subdivisions)
                ep_times = []
                env_times = []
                total_times = []

                for bi, box in enumerate(box_list):
                    for _ in range(n_repeats):
                        info = sbf5.compute_envelope_info(
                            robot, box, ep_cfg, env_cfg, gcpc)
                        ep_times.append(info["ep_time_us"])
                        env_times.append(info["env_time_us"])
                        total_times.append(info["total_time_us"])
                    if (bi + 1) % 100 == 0:
                        logger.info("    %s-%s: %d/%d boxes done",
                                    ep, env_label, bi + 1, n_boxes)

                row = {
                    "robot": rname,
                    "endpoint": ep,
                    "envelope": env,
                    "subdivisions": subdivisions,
                    "envelope_variant": env_label,
                    "ep_us_mean": float(np.mean(ep_times)),
                    "ep_us_std": float(np.std(ep_times)),
                    "env_us_mean": float(np.mean(env_times)),
                    "env_us_std": float(np.std(env_times)),
                    "total_us_mean": float(np.mean(total_times)),
                    "total_us_std": float(np.std(total_times)),
                    "n_samples": len(total_times),
                }
                all_rows.append(row)
                existing_keys.add(key)
                newly_computed += 1
                _checkpoint_rows()
                logger.info("  %s-%s: total=%.0f±%.0f μs",
                            ep, env_label, row["total_us_mean"], row["total_us_std"])

    if newly_computed == 0:
        logger.info("S2: no missing rows detected")
    else:
        logger.info("S2: computed %d new rows", newly_computed)

    # Compute speedup vs Analytical-LinkIAABB per robot
    for rname in robots:
        ref = next(
            (r for r in all_rows
             if r["robot"] == rname and r["endpoint"] == "Analytical"
             and r["envelope"] == "LinkIAABB"
             and int(r.get("subdivisions", 1)) == 1), None)
        ref_time = ref["total_us_mean"] if ref else 1.0
        for r in all_rows:
            if r["robot"] == rname:
                r["speedup"] = round(ref_time / max(r["total_us_mean"], 0.1), 1)

    with open(result_path, "w", encoding="utf-8") as f:
        json.dump({"rows": all_rows}, f, indent=2)
    logger.info("S2: → %s (%d rows)", result_path, len(all_rows))


def run_s3(quick: bool = False, lite: bool = False, ep_sources=None):
    """S3: End-to-End Benchmark — full planning on multiple scenes × 12 configs."""
    from sbf5_bench.runner import (ALL_PIPELINE_CONFIGS, ExperimentConfig,
                                   PipelineConfig, run_experiment)

    out_dir = os.path.join(PAPER_DIR, "s3_e2e")
    os.makedirs(out_dir, exist_ok=True)

    result_path = os.path.join(out_dir, "results.json")
    if os.path.exists(result_path):
        logger.info("S3: results already exist, skipping")
        return

    if quick:
        scenes = ["2dof_simple", "2dof_narrow"]
        pipeline_configs = [
            PipelineConfig("IFK", "LinkIAABB"),
            PipelineConfig("Analytical", "LinkIAABB"),
            PipelineConfig("GCPC", "LinkIAABB_Grid"),
        ]
        n_trials = 3
    else:
        scenes = ["2dof_simple", "2dof_narrow", "2dof_cluttered",
                   "panda_tabletop", "panda_shelf", "panda_multi_obstacle"]
        pipeline_configs = ALL_PIPELINE_CONFIGS
        n_trials = 3 if lite else 30

    # Filter endpoint sources if requested
    if ep_sources is not None:
        pipeline_configs = [p for p in pipeline_configs if p.endpoint_source in ep_sources]
        logger.info("S3: filtered to ep_sources=%s → %d configs", ep_sources, len(pipeline_configs))

    # Determine GCPC cache path
    gcpc_path = os.path.join("data", "panda_5000.gcpc")
    if not os.path.exists(gcpc_path):
        gcpc_path = None
        # Filter out GCPC configs if no cache
        pipeline_configs = [p for p in pipeline_configs if p.endpoint_source != "GCPC"]
        logger.warning("S3: no GCPC cache, skipping GCPC configs")

    total = len(scenes) * len(pipeline_configs) * n_trials
    logger.info("S3: %d scenes × %d configs × %d seeds = %d trials",
                len(scenes), len(pipeline_configs), n_trials, total)

    config = ExperimentConfig(
        scenes=scenes,
        planners=[],
        pipeline_configs=pipeline_configs,
        gcpc_cache_path=gcpc_path,
        n_trials=n_trials,
        timeout=60.0,
        output_dir=out_dir,
    )
    results = run_experiment(config)
    results.save(result_path)
    logger.info("S3: → %s (%d trials)", result_path, len(results.trials))


def run_s4(quick: bool = False, lite: bool = False, skip_ompl: bool = False, ep_sources=None):
    """S4: Baseline Comparison — SBF best configs vs OMPL (if available)."""
    from sbf5_bench.runner import (ExperimentConfig, PipelineConfig,
                                   run_experiment)
    from sbf5_bench.sbf_adapter import SBFPlannerAdapter

    out_dir = os.path.join(PAPER_DIR, "s4_baselines")
    os.makedirs(out_dir, exist_ok=True)

    result_path = os.path.join(out_dir, "results.json")
    if os.path.exists(result_path):
        logger.info("S4: results already exist, skipping")
        return

    # SBF best configs as explicit planners
    gcpc_path = os.path.join("data", "panda_5000.gcpc")
    sbf_planners = [
        SBFPlannerAdapter("IFK", "LinkIAABB"),
        SBFPlannerAdapter("Analytical", "LinkIAABB"),
    ]
    if os.path.exists(gcpc_path):
        sbf_planners.append(
            SBFPlannerAdapter("GCPC", "LinkIAABB_Grid",
                              gcpc_cache_path=gcpc_path))

    # Filter endpoint sources if requested
    if ep_sources is not None:
        sbf_planners = [p for p in sbf_planners if p._endpoint_source in ep_sources]
        logger.info("S4: filtered to ep_sources=%s → %d SBF planners", ep_sources, len(sbf_planners))

    # Try OMPL baselines
    ompl_planners = []
    if skip_ompl:
        logger.info("S4: OMPL skipped (skip_ompl=True)")
    else:
        try:
            from sbf5_bench.ompl_adapter import OMPLPlanner
            for algo in (["RRTConnect"] if quick else ["RRTConnect", "RRTstar", "BITstar"]):
                ompl_planners.append(OMPLPlanner(algo))
            logger.info("S4: OMPL available, %d baselines", len(ompl_planners))
        except ImportError:
            logger.info("S4: OMPL not available, SBF-only comparison")

    all_planners = sbf_planners + ompl_planners

    if quick:
        scenes = ["2dof_simple", "2dof_narrow"]
        n_trials = 3
    else:
        scenes = ["2dof_simple", "2dof_narrow",
                   "panda_tabletop", "panda_shelf"]
        n_trials = 3 if lite else 30

    total = len(scenes) * len(all_planners) * n_trials
    logger.info("S4: %d scenes × %d planners × %d seeds = %d trials",
                len(scenes), len(all_planners), n_trials, total)

    config = ExperimentConfig(
        scenes=scenes,
        planners=all_planners,
        n_trials=n_trials,
        timeout=60.0,
        output_dir=out_dir,
    )
    results = run_experiment(config)
    results.save(result_path)
    logger.info("S4: → %s (%d trials)", result_path, len(results.trials))


def run_s5(quick: bool = False, lite: bool = False):
    """S5: Scalability analysis — DOF sweep, obstacle sweep, budget sweep."""
    from sbf5_bench.runner import ExperimentConfig, PipelineConfig, run_experiment
    from sbf5_bench.sbf_adapter import SBFPlannerAdapter
    from sbf5_bench.scenes import BenchmarkScene, SCENES

    out_dir = os.path.join(PAPER_DIR, "s5_scalability")
    os.makedirs(out_dir, exist_ok=True)
    result_path = os.path.join(out_dir, "results.json")
    if os.path.exists(result_path):
        logger.info("S5: results already exist, skipping")
        return

    gcpc_path = os.path.join("data", "panda_5000.gcpc")
    has_gcpc = os.path.exists(gcpc_path)
    n_trials = 3 if (quick or lite) else 10

    data = {}

    # ── S5a: DOF scaling (2 vs 7) ────────────────────────────
    logger.info("S5a: DOF scaling")
    dof_results = {}
    # Register a non-trivial panda scene for DOF scaling
    panda_s5a = BenchmarkScene(
        name="panda_s5a",
        robot_json="panda.json",
        obstacles=[
            {"center": [0.4, 0.0, 0.4], "half_sizes": [0.15, 0.15, 0.02]},
        ],
        start=np.array([0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0]),
        goal=np.array([0.5, -0.3, 0.0, -1.5, 0.0, 1.2, 0.0]),
        description="7DOF Panda with near-field shelf obstacle",
    )
    SCENES[panda_s5a.name] = panda_s5a
    dof_scenes = {
        2: "2dof_simple",
        7: "panda_s5a",
    }
    for dof, scene_name in dof_scenes.items():
        ep_src = "GCPC" if has_gcpc and dof == 7 and not quick else "IFK"
        env_type = "LinkIAABB_Grid"
        # Use IFK for panda in quick mode to avoid slow GCPC; lite uses GCPC
        planner = SBFPlannerAdapter(
            ep_src, env_type,
            gcpc_cache_path=gcpc_path if ep_src == "GCPC" else None)

        config = ExperimentConfig(
            scenes=[scene_name], planners=[planner],
            n_trials=n_trials, timeout=60.0,
            output_dir=os.path.join(out_dir, f"s5a_{dof}dof"),
        )
        res = run_experiment(config)
        times = [t.planning_result.planning_time_s for t in res.trials]
        successes = [t.planning_result.success for t in res.trials]
        dof_results[dof] = {
            "time_mean": float(np.mean(times)),
            "time_std": float(np.std(times)),
            "success_rate": float(100 * np.mean(successes)),
            "n_trials": len(times),
        }
    data["time_vs_dof"] = dof_results

    # ── S5b: Obstacle count sweep (7DOF) ─────────────────────
    logger.info("S5b: Obstacle count scaling")
    obs_results = {}
    base_start = np.array([0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0])
    base_goal = np.array([0.5, -0.3, 0.0, -1.5, 0.0, 1.2, 0.0])
    extra_obs = [
        {"center": [0.4, 0.0, 0.4], "half_sizes": [0.15, 0.15, 0.02]},     # thin shelf
        {"center": [0.3, 0.2, 0.35], "half_sizes": [0.08, 0.08, 0.08]},   # small cube
        {"center": [0.5, -0.15, 0.5], "half_sizes": [0.06, 0.06, 0.06]},  # mid cube
        {"center": [0.25, -0.1, 0.55], "half_sizes": [0.10, 0.02, 0.10]}, # thin wall
        {"center": [0.45, 0.25, 0.3], "half_sizes": [0.07, 0.07, 0.07]},  # front cube
        {"center": [0.35, -0.25, 0.5], "half_sizes": [0.06, 0.06, 0.06]}, # side cube
        {"center": [0.55, 0.1, 0.42], "half_sizes": [0.05, 0.12, 0.02]},  # thin bar
        {"center": [0.2, 0.15, 0.48], "half_sizes": [0.07, 0.05, 0.07]},  # rear cube
    ]
    n_obs_list = [1, 2, 4] if quick else ([1, 2, 4, 8] if lite else [1, 2, 4, 6, 8])

    for n_obs in n_obs_list:
        scene = BenchmarkScene(
            name=f"panda_{n_obs}obs",
            robot_json="panda.json",
            obstacles=extra_obs[:n_obs],
            start=base_start,
            goal=base_goal,
            description=f"7DOF {n_obs} obstacles",
        )
        # Register scene temporarily
        SCENES[scene.name] = scene

        planner = SBFPlannerAdapter(
            "IFK", "LinkIAABB_Grid",
            gcpc_cache_path=gcpc_path if has_gcpc else None)

        config = ExperimentConfig(
            scenes=[scene.name], planners=[planner],
            n_trials=n_trials, timeout=60.0,
            output_dir=os.path.join(out_dir, f"s5b_{n_obs}obs"),
        )
        res = run_experiment(config)
        times = [t.planning_result.planning_time_s for t in res.trials]
        successes = [t.planning_result.success for t in res.trials]
        obs_results[n_obs] = {
            "time_mean": float(np.mean(times)),
            "time_std": float(np.std(times)),
            "success_rate": float(100 * np.mean(successes)),
        }
    data["success_vs_obstacles"] = obs_results

    # ── S5c: Box budget sweep (7DOF, scene with near-field obstacles) ─
    logger.info("S5c: Box budget scaling")
    budget_results = {}
    # Challenging scene requiring many boxes
    panda_s5c = BenchmarkScene(
        name="panda_s5c",
        robot_json="panda.json",
        obstacles=[
            {"center": [0.4, 0.0, 0.4], "half_sizes": [0.15, 0.15, 0.02]},
            {"center": [0.3, 0.2, 0.35], "half_sizes": [0.08, 0.08, 0.08]},
            {"center": [0.5, -0.15, 0.5], "half_sizes": [0.06, 0.06, 0.06]},
        ],
        start=np.array([0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0]),
        goal=np.array([0.5, -0.3, 0.0, -1.5, 0.0, 1.2, 0.0]),
        description="7DOF Panda with 3 near-field obstacles for budget sweep",
    )
    SCENES[panda_s5c.name] = panda_s5c
    budget_list = [50, 200, 500] if quick else ([50, 200, 500, 2000] if lite else [50, 100, 200, 500, 1000, 2000])

    for max_boxes in budget_list:
        planner = SBFPlannerAdapter(
            "IFK", "LinkIAABB",
            extra_config={"max_boxes": max_boxes})

        config = ExperimentConfig(
            scenes=["panda_s5c"], planners=[planner],
            n_trials=n_trials, timeout=60.0,
            output_dir=os.path.join(out_dir, f"s5c_{max_boxes}"),
        )
        res = run_experiment(config)
        times = [t.planning_result.planning_time_s for t in res.trials]
        successes = [t.planning_result.success for t in res.trials]
        budget_results[max_boxes] = {
            "time_mean": float(np.mean(times)),
            "time_std": float(np.std(times)),
            "success_rate": float(100 * np.mean(successes)),
        }
    data["success_vs_max_boxes"] = budget_results

    with open(result_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    logger.info("S5: → %s", result_path)


# ── Output generation (Phase T) ─────────────────────────────

def gen_tables():
    """Generate all LaTeX tables from experiment data."""
    from scripts.gen_tables import main as tables_main
    sys.argv = ["gen_tables", "--data-dir", PAPER_DIR]
    tables_main()


def gen_figures():
    """Generate all paper figures from experiment data."""
    from scripts.gen_figures import main as figures_main
    sys.argv = ["gen_figures", "--data-dir", PAPER_DIR]
    figures_main()


def gen_stats():
    """Generate statistical significance analysis."""
    from sbf5_bench.stats import pairwise_significance, save_significance
    from sbf5_bench.runner import ExperimentResults

    stats_dir = os.path.join(PAPER_DIR, "stats")
    os.makedirs(stats_dir, exist_ok=True)

    # S3 pairwise
    s3_path = os.path.join(PAPER_DIR, "s3_e2e", "results.json")
    if os.path.exists(s3_path):
        try:
            s3 = ExperimentResults.load(s3_path)
            sig = pairwise_significance(s3, metric="planning_time_s")
            save_significance(sig, os.path.join(stats_dir, "pairwise_s3.json"))
            logger.info("Stats: S3 pairwise → %s", stats_dir)
        except Exception as e:
            logger.warning("Stats S3 failed: %s", e)

    # S4 pairwise
    s4_path = os.path.join(PAPER_DIR, "s4_baselines", "results.json")
    if os.path.exists(s4_path):
        try:
            s4 = ExperimentResults.load(s4_path)
            sig = pairwise_significance(s4, metric="planning_time_s")
            save_significance(sig, os.path.join(stats_dir, "pairwise_s4.json"))
            logger.info("Stats: S4 pairwise → %s", stats_dir)
        except Exception as e:
            logger.warning("Stats S4 failed: %s", e)


# ── Main ─────────────────────────────────────────────────────

def main():
    global PAPER_DIR
    parser = argparse.ArgumentParser(
        description="Run all experiments and generate paper outputs.")
    parser.add_argument("--skip-experiments", action="store_true",
                        help="Only regenerate tables/figures from existing data")
    parser.add_argument("--skip-figures", action="store_true",
                        help="Only run experiments, skip output generation")
    parser.add_argument("--quick", action="store_true",
                        help="Quick mode: fewer seeds/scenes (for testing)")
    parser.add_argument("--lite", action="store_true",
                        help="Lite mode: all scenes/configs, 3 trials each")
    parser.add_argument("--mc-samples", type=int, default=MC_SAMPLES_DEFAULT,
                        help="MC endpoint sample count for S1/S2 (default: 50000)")
    parser.add_argument("--paper-dir", type=str, default=PAPER_DIR,
                        help="Output root for experiment results")
    parser.add_argument("--only-s0", action="store_true",
                        help="Run only S0 preselection experiment")
    parser.add_argument("--only-s0b", action="store_true",
                        help="Run only S0b endpoint-width profiling experiment")
    args = parser.parse_args()

    PAPER_DIR = args.paper_dir

    os.makedirs(PAPER_DIR, exist_ok=True)

    if not args.skip_experiments:
        if args.only_s0b:
            logger.info("═══ Phase S0b: Endpoint Width Profiling ═══")
            run_s0_ep_width_profile(quick=args.quick, lite=args.lite)
            logger.info("S0b only mode enabled, skipping S0/S1-S5")
            logger.info("═══ ALL DONE ═══")
            logger.info("Data    → %s/s0_ep_width_profile/", PAPER_DIR)
            return

        logger.info("═══ Phase S0: Sub/Grid Preselection (CritSample) ═══")
        run_s0_param_selection(quick=args.quick, lite=args.lite)

        logger.info("═══ Phase S0b: Endpoint Width Profiling ═══")
        run_s0_ep_width_profile(quick=args.quick, lite=args.lite)

        if args.only_s0:
            logger.info("S0 only mode enabled, skipping S1-S5")
            logger.info("═══ ALL DONE ═══")
            logger.info("Data    → %s/s0_param_selection/", PAPER_DIR)
            return

        logger.info("═══ Phase S1: Envelope Tightness (CritSample-representative) ═══")
        run_s1(
            quick=args.quick,
            lite=args.lite,
            mc_samples=args.mc_samples,
            ep_sources_override=["CritSample"],
        )

        logger.info("═══ Phase S2: Envelope Timing (CritSample-representative) ═══")
        run_s2(
            quick=args.quick,
            lite=args.lite,
            mc_samples=args.mc_samples,
            ep_sources_override=["CritSample"],
        )

        logger.info("═══ Phase S3: End-to-End Benchmark ═══")
        run_s3(quick=args.quick, lite=args.lite)

        logger.info("═══ Phase S4: Baselines ═══")
        run_s4(quick=args.quick, lite=args.lite)

        logger.info("═══ Phase S5: Scalability ═══")
        run_s5(quick=args.quick, lite=args.lite)

    if not args.skip_figures:
        logger.info("═══ Phase T: Generating Tables & Figures ═══")
        gen_tables()
        gen_figures()
        gen_stats()

    logger.info("═══ ALL DONE ═══")
    logger.info("Tables  → %s/tables/", PAPER_DIR)
    logger.info("Figures → %s/figures/", PAPER_DIR)
    logger.info("Data    → %s/s{1-5}_*/", PAPER_DIR)


if __name__ == "__main__":
    main()

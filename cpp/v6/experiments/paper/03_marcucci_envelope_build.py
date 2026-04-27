#!/usr/bin/env python3
"""Paper Exp. 2.5 — Marcucci envelope-pipeline build comparison.

Environment: Marcucci combined IIWA14 scene only, using the same planner-side
configuration defaults as Exp. 3.  Each cell first deletes its LECT cache and
runs a cold build for one seed, then immediately reruns the same seed with the
same cache path to measure a warm/cache-hit build.

Matrix:
  endpoint source: IFK, CritSample
  link envelope:   AABB S=4, AABB-grid S=4 d=0.04, Hull16-grid d=0.04

Outputs:
  - experiments/results_paper/marcucci_envelope_build/raw/*.json
  - experiments/results_paper/marcucci_envelope_build.json
"""
from __future__ import annotations

import argparse
import importlib.util
import shutil
import statistics
import struct
import sys
import tempfile
import time
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import ROOT, add_common_args, load_json, mode_args, require_python_extension, write_json


ENDPOINTS = [
    ("ifk", "IFK"),
    ("critsample", "CritSample"),
]

AUTHORITATIVE_SCRIPT = ROOT / "scripts" / "run_online_query_comparison.py"

ENVELOPE_VARIANTS = [
    {
        "key": "aabb_s4",
        "label": "AABB S=4",
        "env": "link_iaabb",
        "n_sub": 4,
        "voxel_delta": 0.04,
    },
    {
        "key": "aabb_grid_s4_d004",
        "label": "AABB-grid S=4 d=0.04",
        "env": "link_iaabb_grid",
        "n_sub": 4,
        "voxel_delta": 0.04,
    },
    {
        "key": "hull16_grid_d004",
        "label": "Hull16-grid d=0.04",
        "env": "hull16_grid",
        "n_sub": 1,
        "voxel_delta": 0.04,
    },
]

LECT_MAGIC = b"SBF5LECT"
LECT_HEADER_BYTES = 128
PROBE_TIMEOUT_MS = 1000.0
PROBE_MAX_BOXES = 500
PROBE_THREADS = 1
PROBE_FFB_DEPTH = 32
PYTHON_EXTENSION_DIR = ROOT / "build" / "python"


def median(values: list[float]) -> float:
    return float(statistics.median(values)) if values else 0.0


def mean(values: list[float]) -> float:
    return float(statistics.fmean(values)) if values else 0.0


def mean_optional(values: list[float | None]) -> float | None:
    present = [float(value) for value in values if value is not None]
    return float(statistics.fmean(present)) if present else None


def ensure_python_paths() -> None:
    for candidate in (PYTHON_EXTENSION_DIR, ROOT / "python"):
        text = str(candidate)
        if candidate.exists() and text not in sys.path:
            sys.path.insert(0, text)


def load_authoritative_module() -> Any:
    ensure_python_paths()
    spec = importlib.util.spec_from_file_location(
        "v6_authoritative_online_query_comparison",
        AUTHORITATIVE_SCRIPT,
    )
    if spec is None or spec.loader is None:
        raise ImportError(f"Unable to load authoritative script at {AUTHORITATIVE_SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def import_sbf5() -> Any:
    ensure_python_paths()
    import sbf5  # type: ignore
    extension = sys.modules.get("sbf5._sbf5_cpp")
    if extension is not None and "_sbf5_cpp" not in sys.modules:
        sys.modules["_sbf5_cpp"] = extension
    return sbf5


def make_endpoint_config(sbf5: Any, endpoint: str) -> Any:
    cfg = sbf5.EndpointSourceConfig()
    cfg.source = {
        "ifk": sbf5.EndpointSource.IFK,
        "critsample": sbf5.EndpointSource.CritSample,
    }[endpoint]
    return cfg


def make_envelope_config(sbf5: Any, variant: dict[str, Any]) -> Any:
    cfg = sbf5.EnvelopeTypeConfig()
    cfg.type = {
        "link_iaabb": sbf5.EnvelopeType.LinkIAABB,
        "link_iaabb_grid": sbf5.EnvelopeType.LinkIAABB_Grid,
        "hull16_grid": sbf5.EnvelopeType.Hull16_Grid,
    }[variant["env"]]
    cfg.n_subdivisions = int(variant["n_sub"])
    if variant["env"] in {"link_iaabb_grid", "hull16_grid"}:
        cfg.grid_config.voxel_delta = float(variant["voxel_delta"])
    return cfg


def make_planner_config(
    sbf5: Any,
    authoritative: Any,
    *,
    endpoint: str,
    variant: dict[str, Any],
    seed: int,
    threads: int,
    timeout_s: int,
    ffb_depth: int,
    max_boxes: int,
    bridge_boxes: int,
    cache_dir: Path,
) -> Any:
    cfg = sbf5.SBFPlannerConfig()
    cfg.split_order = sbf5.SplitOrder.BEST_TIGHTEN
    cfg.z4_enabled = True
    cfg.lect_no_cache = False
    cfg.lect_cache_dir = str(cache_dir)
    cfg.endpoint_source = make_endpoint_config(sbf5, endpoint)
    cfg.envelope_type = make_envelope_config(sbf5, variant)
    authoritative.apply_paper_sbf_architecture(
        cfg,
        seed=seed,
        grow_timeout_ms=float(timeout_s) * 1000.0,
        max_boxes=max_boxes,
        post_connect_extra_boxes=bridge_boxes,
        n_threads=max(1, int(threads)),
        bridge_n_threads=max(1, int(threads)),
        ffb_depth=ffb_depth,
        lect_no_cache=False,
        lect_cache_dir=cache_dir,
    )
    return cfg


def find_cache_file(cache_dir: Path) -> Path | None:
    files = sorted(cache_dir.glob("*.lect"))
    return files[0] if files else None


def read_lect_cache_metrics(cache_dir: Path) -> dict[str, Any]:
    cache_file = find_cache_file(cache_dir)
    if cache_file is None or not cache_file.is_file():
        return {}

    with cache_file.open("rb") as handle:
        header = handle.read(LECT_HEADER_BYTES)
    if len(header) < LECT_HEADER_BYTES or header[:8] != LECT_MAGIC:
        return {}

    version = struct.unpack_from("<I", header, 8)[0]
    n_nodes = struct.unpack_from("<i", header, 12)[0]
    n_active_links = struct.unpack_from("<i", header, 20)[0]
    ep_stride = struct.unpack_from("<i", header, 24)[0]
    tree_record_bytes = struct.unpack_from("<I", header, 56)[0]
    capacity = struct.unpack_from("<i", header, 36)[0]
    file_bytes = cache_file.stat().st_size
    base_section_bytes_per_capacity_slot = float(tree_record_bytes + 2 * ep_stride * 4)
    return {
        "cache_file_path": str(cache_file),
        "cache_header_version": int(version),
        "cache_n_nodes": int(n_nodes),
        "cache_capacity": int(capacity),
        "cache_n_active_links": int(n_active_links),
        "cache_ep_stride": int(ep_stride),
        "cache_tree_record_bytes": int(tree_record_bytes),
        "cache_file_bytes": int(file_bytes),
        "cache_file_bytes_per_node": float(file_bytes) / n_nodes if n_nodes > 0 else None,
        "cache_file_bytes_per_capacity_slot": float(file_bytes) / capacity if capacity > 0 else None,
        "cache_base_section_bytes_per_capacity_slot": base_section_bytes_per_capacity_slot,
    }


def probe_lect_read_ms(
    *,
    endpoint: str,
    variant: dict[str, Any],
    seed: int,
    cache_dir: Path,
    ffb_depth: int,
) -> float:
    sbf5 = import_sbf5()
    authoritative = load_authoritative_module()
    robot = sbf5.Robot.from_json(str(ROOT / "data" / "iiwa14.json"))
    obstacles = authoritative.make_combined_obstacles()
    probe_label, probe_start, probe_goal = authoritative.QUERY_PAIRS[0]

    with tempfile.TemporaryDirectory(prefix="sbf5_lect_probe_") as tmp_dir:
        probe_cache_dir = Path(tmp_dir) / cache_dir.name
        shutil.copytree(cache_dir, probe_cache_dir)
        cfg = make_planner_config(
            sbf5,
            authoritative,
            endpoint=endpoint,
            variant=variant,
            seed=seed,
            threads=PROBE_THREADS,
            timeout_s=max(1, int(PROBE_TIMEOUT_MS / 1000.0)),
            ffb_depth=min(int(ffb_depth), PROBE_FFB_DEPTH),
            max_boxes=PROBE_MAX_BOXES,
            bridge_boxes=0,
            cache_dir=probe_cache_dir,
        )
        planner = sbf5.SBFPlanner(robot, cfg)
        result = planner.plan(
            authoritative.IIWA_CONFIGS[probe_start],
            authoritative.IIWA_CONFIGS[probe_goal],
            obstacles,
            timeout_ms=PROBE_TIMEOUT_MS,
        )
        return float(result.lect_time_ms)


def enrich_row_metrics(
    row: dict[str, Any],
    *,
    endpoint: str,
    variant: dict[str, Any],
    seed: int,
    ffb_depth: int,
) -> dict[str, Any]:
    cache_dir_text = row.get("cache_dir")
    if not cache_dir_text:
        return row
    cache_dir = Path(str(cache_dir_text))
    if not cache_dir.exists():
        return row

    metrics = read_lect_cache_metrics(cache_dir)
    if metrics:
        row.update(metrics)

    if row.get("cache_mode") == "warm" and row.get("lect_read_ms_probe") is None and metrics:
        lect_read_ms = probe_lect_read_ms(
            endpoint=endpoint,
            variant=variant,
            seed=seed,
            cache_dir=cache_dir,
            ffb_depth=ffb_depth,
        )
        row["lect_read_ms_probe"] = float(lect_read_ms)
        cache_n_nodes = int(metrics.get("cache_n_nodes", 0))
        row["lect_read_us_per_node_probe"] = (
            float(lect_read_ms) * 1000.0 / cache_n_nodes if cache_n_nodes > 0 else None
        )
    return row


def run_trial(
    *,
    endpoint: str,
    endpoint_label: str,
    variant: dict[str, Any],
    seed: int,
    cache_mode: str,
    cache_dir: Path,
    raw_path: Path,
    timeout_s: int,
    threads: int,
    ffb_depth: int,
    max_boxes: int,
    bridge_boxes: int,
) -> dict[str, Any]:
    sbf5 = import_sbf5()
    authoritative = load_authoritative_module()
    robot = sbf5.Robot.from_json(str(ROOT / "data" / "iiwa14.json"))
    obstacles = authoritative.make_combined_obstacles()
    seed_points = [authoritative.IIWA_CONFIGS[key] for key in ["AS", "TS", "CS", "LB", "RB"]]
    cache_available_before_build = any(cache_dir.glob("*.lect"))

    cfg = make_planner_config(
        sbf5,
        authoritative,
        endpoint=endpoint,
        variant=variant,
        seed=seed,
        threads=threads,
        timeout_s=timeout_s,
        ffb_depth=ffb_depth,
        max_boxes=max_boxes,
        bridge_boxes=bridge_boxes,
        cache_dir=cache_dir,
    )
    planner = sbf5.SBFPlanner(robot, cfg)

    t0 = time.perf_counter()
    planner.build_coverage(obstacles, float(timeout_s) * 1000.0, seed_points)
    build_s = time.perf_counter() - t0

    queries = []
    for label, start_name, goal_name in authoritative.QUERY_PAIRS:
        result = planner.query(
            authoritative.IIWA_CONFIGS[start_name],
            authoritative.IIWA_CONFIGS[goal_name],
        )
        queries.append(
            {
                "name": label,
                "from": start_name,
                "to": goal_name,
                "ok": bool(result.success),
                "planning_time_ms": float(result.planning_time_ms),
                "path_length": float(result.path_length) if result.success else 0.0,
            }
        )

    row: dict[str, Any] = {
        "source_protocol": "v6_python_sbfplanner_build_query",
        "source_script": str(AUTHORITATIVE_SCRIPT),
        "endpoint_source": endpoint,
        "endpoint_label": endpoint_label,
        "envelope_key": variant["key"],
        "envelope_label": variant["label"],
        "env": variant["env"],
        "n_sub": variant["n_sub"],
        "voxel_delta": variant["voxel_delta"],
        "seed": seed,
        "cache_mode": cache_mode,
        "cache_dir": str(cache_dir),
        "raw_path": str(raw_path),
        "cache_available_before_build": bool(cache_available_before_build),
        "loaded_lect_cache": bool(cache_mode == "warm" and cache_available_before_build),
        "build_s": float(build_s),
        "n_boxes": int(planner.n_boxes()),
        "query_success_rate": query_success_rate(queries),
        "queries": queries,
        "lect_read_ms_probe": None,
        "lect_read_us_per_node_probe": None,
    }
    row = enrich_row_metrics(
        row,
        endpoint=endpoint,
        variant=variant,
        seed=seed,
        ffb_depth=ffb_depth,
    )
    write_json(raw_path, row)
    return row


def query_success_rate(queries: list[dict[str, Any]]) -> float:
    if not queries:
        return 0.0
    return sum(1 for query in queries if query.get("ok")) / len(queries)


def summarise(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    summary = []
    keys = sorted({
        (row["endpoint_source"], row["endpoint_label"], row["envelope_key"],
         row["envelope_label"], row["env"], row["n_sub"], row["voxel_delta"],
         row["cache_mode"])
        for row in rows
    })
    for endpoint, endpoint_label, env_key, env_label, env, n_sub, voxel_delta, cache_mode in keys:
        group = [
            row for row in rows
            if row["endpoint_source"] == endpoint
            and row["envelope_key"] == env_key
            and row["cache_mode"] == cache_mode
        ]
        builds = [float(row["build_s"]) for row in group]
        boxes = [float(row["n_boxes"]) for row in group]
        summary.append({
            "endpoint_source": endpoint,
            "endpoint_label": endpoint_label,
            "envelope_key": env_key,
            "envelope_label": env_label,
            "env": env,
            "n_sub": n_sub,
            "voxel_delta": voxel_delta,
            "cache_mode": cache_mode,
            "n_runs": len(group),
            "median_build_s": median(builds),
            "mean_build_s": mean(builds),
            "median_n_boxes": median(boxes),
            "loaded_cache_rate": mean([1.0 if row["loaded_lect_cache"] else 0.0 for row in group]),
            "query_success_rate_mean": mean([float(row["query_success_rate"]) for row in group]),
            "mean_cache_file_bytes": mean_optional([row.get("cache_file_bytes") for row in group]),
            "mean_cache_n_nodes": mean_optional([row.get("cache_n_nodes") for row in group]),
            "mean_cache_capacity": mean_optional([row.get("cache_capacity") for row in group]),
            "mean_cache_file_bytes_per_node": mean_optional([
                row.get("cache_file_bytes_per_node") for row in group
            ]),
            "mean_cache_file_bytes_per_capacity_slot": mean_optional([
                row.get("cache_file_bytes_per_capacity_slot") for row in group
            ]),
            "mean_cache_base_section_bytes_per_capacity_slot": mean_optional([
                row.get("cache_base_section_bytes_per_capacity_slot") for row in group
            ]),
            "mean_lect_read_ms_probe": mean_optional([
                row.get("lect_read_ms_probe") for row in group
            ]),
            "mean_lect_read_us_per_node_probe": mean_optional([
                row.get("lect_read_us_per_node_probe") for row in group
            ]),
        })
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_common_args(parser)
    parser.add_argument("--threads", type=int, default=16)
    parser.add_argument("--ffb-depth", type=int, default=300)
    parser.add_argument("--max-boxes", type=int, default=200000)
    parser.add_argument("--bridge-boxes", type=int, default=4000)
    parser.add_argument("--resume", action="store_true",
                        help="skip a cold/warm pair when both raw JSON outputs already exist")
    args = parser.parse_args()

    global PYTHON_EXTENSION_DIR
    PYTHON_EXTENSION_DIR = require_python_extension(args)

    seeds, timeout, mode = mode_args(args, quick_seeds=1, full_seeds=10,
                                     quick_timeout=60, full_timeout=60)
    base_dir = args.out_dir / "marcucci_envelope_build"
    raw_dir = base_dir / "raw"
    cache_dir = base_dir / "lect_cache"
    rows: list[dict[str, Any]] = []

    for endpoint, endpoint_label in ENDPOINTS:
        for variant in ENVELOPE_VARIANTS:
            for seed in range(seeds):
                stem = f"{endpoint}_{variant['key']}_seed{seed:03d}"
                cache_path = cache_dir / f"{stem}.bin"
                cold_out = raw_dir / f"{stem}_cold.json"
                warm_out = raw_dir / f"{stem}_warm.json"
                pair_done = cold_out.exists() and warm_out.exists()
                if args.resume and pair_done:
                    print(f"[skip] {stem} cold/warm")
                else:
                    if not args.dry_run and cache_path.exists():
                        shutil.rmtree(cache_path)
                    for cache_mode, out_path in (("cold", cold_out), ("warm", warm_out)):
                        if args.dry_run:
                            print(
                                "$ "
                                f"{sys.executable} {Path(__file__).name} {mode} "
                                f"--threads {args.threads} --ffb-depth {args.ffb_depth} "
                                f"--max-boxes {args.max_boxes} --bridge-boxes {args.bridge_boxes} "
                                f"[{endpoint} {variant['key']} seed={seed} {cache_mode}]"
                            )
                            continue
                        row = run_trial(
                            endpoint=endpoint,
                            endpoint_label=endpoint_label,
                            variant=variant,
                            seed=seed,
                            cache_mode=cache_mode,
                            cache_dir=cache_path,
                            raw_path=out_path,
                            timeout_s=timeout,
                            threads=args.threads,
                            ffb_depth=args.ffb_depth,
                            max_boxes=args.max_boxes,
                            bridge_boxes=args.bridge_boxes,
                        )
                        rows.append(row)

                if args.dry_run:
                    continue
                if args.resume and pair_done:
                    cold_row = enrich_row_metrics(
                        load_json(cold_out),
                        endpoint=endpoint,
                        variant=variant,
                        seed=seed,
                        ffb_depth=args.ffb_depth,
                    )
                    warm_row = enrich_row_metrics(
                        load_json(warm_out),
                        endpoint=endpoint,
                        variant=variant,
                        seed=seed,
                        ffb_depth=args.ffb_depth,
                    )
                    write_json(cold_out, cold_row)
                    write_json(warm_out, warm_row)
                    rows.append(cold_row)
                    rows.append(warm_row)

    if args.dry_run:
        print("[dry-run] commands emitted; no marcucci_envelope_build.json written")
        return

    out = {
        "experiment": "marcucci_envelope_build",
        "schema_version": 1,
        "scene": "marcucci_combined",
        "runner": "v6_python_sbfplanner",
        "source_script": str(AUTHORITATIVE_SCRIPT),
        "defaults": {
            "seeds": seeds,
            "timeout": timeout,
            "threads": args.threads,
            "ffb_depth": args.ffb_depth,
            "max_boxes": args.max_boxes,
            "bridge_boxes": args.bridge_boxes,
        },
        "endpoints": [{"key": key, "label": label} for key, label in ENDPOINTS],
        "envelope_variants": ENVELOPE_VARIANTS,
        "rows": rows,
        "summary": summarise(rows),
    }
    write_json(args.out_dir / "marcucci_envelope_build.json", out)
    print(f"[write] {args.out_dir / 'marcucci_envelope_build.json'}")


if __name__ == "__main__":
    main()
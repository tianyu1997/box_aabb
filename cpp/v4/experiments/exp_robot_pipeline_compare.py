from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import numpy as np


ROBOT_JSONS = {
    "iiwa14": "robots/iiwa14/iiwa14_dh.json",
    "panda": "robots/panda/panda_dh.json",
}

WS_XMIN = -0.5
WS_XMAX = 1.5
WS_YMIN = -1.0
WS_YMAX = 1.0
WS_ZMIN = 0.0
WS_ZMAX = 1.2
MIN_SIZE = 0.04
MAX_SIZE = 0.12
BASE_CLEARANCE = 0.15

GCPC_CACHE_PROFILES = {
    "smoke": {
        "gcpc_random_seeds": 32,
        "gcpc_max_sweeps": 1,
        "gcpc_max_exact_product": 2000,
        "gcpc_random_configs": 128,
        "gcpc_root_n_candidates": 4,
    },
    "quick": {
        "gcpc_random_seeds": 96,
        "gcpc_max_sweeps": 2,
        "gcpc_max_exact_product": 8000,
        "gcpc_random_configs": 512,
        "gcpc_root_n_candidates": 8,
    },
    "standard": {
        "gcpc_random_seeds": 256,
        "gcpc_max_sweeps": 4,
        "gcpc_max_exact_product": 20000,
        "gcpc_random_configs": 2000,
        "gcpc_root_n_candidates": 16,
    },
    "full": {
        "gcpc_random_seeds": 500,
        "gcpc_max_sweeps": 5,
        "gcpc_max_exact_product": 50000,
        "gcpc_random_configs": 5000,
        "gcpc_root_n_candidates": 50,
    },
}


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _load_cpp(repo_root: Path):
    if str(repo_root) not in sys.path:
        sys.path.insert(0, str(repo_root))
    from viz.growth_viz import _load_cpp_module

    return _load_cpp_module(repo_root)


def _parse_csv_list(raw: str, cast):
    return [cast(item.strip()) for item in raw.split(",") if item.strip()]


def _resolve_output_dir(repo_root: Path, output_dir: str | None, run_name: str) -> Path:
    if output_dir:
        resolved = Path(output_dir).expanduser()
        if not resolved.is_absolute():
            resolved = repo_root / resolved
        return resolved
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return repo_root / "results" / f"{run_name}_{stamp}"


def _resolve_cache_dir(repo_root: Path, cache_dir: str | None, run_name: str) -> Path:
    if cache_dir:
        resolved = Path(cache_dir).expanduser()
        if not resolved.is_absolute():
            resolved = repo_root / resolved
        return resolved
    return repo_root / "results" / run_name / "gcpc_cache"


def _resolve_gcpc_build_params(
    profile_name: str,
    gcpc_random_seeds: int | None,
    gcpc_max_sweeps: int | None,
    gcpc_max_exact_product: int | None,
    gcpc_random_configs: int | None,
) -> Dict[str, int]:
    profile_key = str(profile_name).strip().lower()
    if profile_key not in GCPC_CACHE_PROFILES:
        raise ValueError(f"Unsupported GCPC cache profile: {profile_name}")

    profile = GCPC_CACHE_PROFILES[profile_key]
    return {
        "gcpc_random_seeds": int(profile["gcpc_random_seeds"] if gcpc_random_seeds is None else gcpc_random_seeds),
        "gcpc_max_sweeps": int(profile["gcpc_max_sweeps"] if gcpc_max_sweeps is None else gcpc_max_sweeps),
        "gcpc_max_exact_product": int(profile["gcpc_max_exact_product"] if gcpc_max_exact_product is None else gcpc_max_exact_product),
        "gcpc_random_configs": int(profile["gcpc_random_configs"] if gcpc_random_configs is None else gcpc_random_configs),
    }


def _resolve_gcpc_root_n_candidates(profile_name: str, gcpc_root_n_candidates: int | None) -> int:
    profile_key = str(profile_name).strip().lower()
    if profile_key not in GCPC_CACHE_PROFILES:
        raise ValueError(f"Unsupported GCPC cache profile: {profile_name}")
    if gcpc_root_n_candidates is not None:
        return max(1, int(gcpc_root_n_candidates))
    return max(1, int(GCPC_CACHE_PROFILES[profile_key]["gcpc_root_n_candidates"]))


def _gcpc_cache_stem(robot_name: str, build_params: Dict[str, int]) -> str:
    return (
        f"{robot_name}"
        f"_rs{build_params['gcpc_random_seeds']}"
        f"_sw{build_params['gcpc_max_sweeps']}"
        f"_ep{build_params['gcpc_max_exact_product']}"
        f"_rc{build_params['gcpc_random_configs']}"
    )


def _gcpc_cache_paths(cache_root: Path, robot_name: str, build_params: Dict[str, int]) -> Tuple[Path, Path]:
    stem = _gcpc_cache_stem(robot_name, build_params)
    return cache_root / f"{stem}.gcpc", cache_root / f"{stem}.meta.json"


def _load_gcpc_cache_meta(meta_path: Path) -> Dict[str, object] | None:
    if not meta_path.exists():
        return None
    try:
        return json.loads(meta_path.read_text(encoding="utf-8"))
    except Exception:
        return None


def _write_gcpc_cache_meta(
    meta_path: Path,
    *,
    robot_name: str,
    cache_file: Path,
    profile_name: str,
    build_params: Dict[str, int],
    build_time_ms: float | None,
    cache,
) -> None:
    meta = {
        "robot": robot_name,
        "cache_file": cache_file.name,
        "gcpc_cache_profile": profile_name,
        "gcpc_random_seeds": int(build_params["gcpc_random_seeds"]),
        "gcpc_max_sweeps": int(build_params["gcpc_max_sweeps"]),
        "gcpc_max_exact_product": int(build_params["gcpc_max_exact_product"]),
        "gcpc_random_configs": int(build_params["gcpc_random_configs"]),
        "build_time_ms": None if build_time_ms is None else float(build_time_ms),
        "n_links": int(cache.n_links()),
        "n_total_points": int(cache.n_total_points()),
    }
    meta_path.write_text(json.dumps(meta, indent=2), encoding="utf-8")


def _gcpc_meta_matches(meta: Dict[str, object] | None, robot_name: str, build_params: Dict[str, int]) -> bool:
    if not meta:
        return False
    return (
        str(meta.get("robot")) == robot_name
        and int(meta.get("gcpc_random_seeds", -1)) == int(build_params["gcpc_random_seeds"])
        and int(meta.get("gcpc_max_sweeps", -1)) == int(build_params["gcpc_max_sweeps"])
        and int(meta.get("gcpc_max_exact_product", -1)) == int(build_params["gcpc_max_exact_product"])
        and int(meta.get("gcpc_random_configs", -1)) == int(build_params["gcpc_random_configs"])
    )


def _generate_obstacles(cpp, rng: np.random.Generator, n_obstacles: int):
    obstacles = []
    for attempt in range(n_obstacles * 6):
        if len(obstacles) >= n_obstacles:
            break
        cx = float(rng.uniform(WS_XMIN, WS_XMAX))
        cy = float(rng.uniform(WS_YMIN, WS_YMAX))
        cz = float(rng.uniform(WS_ZMIN, WS_ZMAX))
        if math.hypot(cx, cy) < BASE_CLEARANCE:
            continue
        hx = float(rng.uniform(MIN_SIZE, MAX_SIZE) / 2.0)
        hy = float(rng.uniform(MIN_SIZE, MAX_SIZE) / 2.0)
        hz = float(rng.uniform(MIN_SIZE, MAX_SIZE) / 2.0)
        obstacles.append(
            cpp.Obstacle(
                np.array([cx, cy, cz], dtype=float),
                np.array([hx, hy, hz], dtype=float),
                f"rand_{attempt}",
            )
        )
    return obstacles


def _random_config(robot, rng: np.random.Generator) -> np.ndarray:
    return np.array(
        [float(rng.uniform(interval.lo, interval.hi)) for interval in robot.joint_limits().limits],
        dtype=float,
    )


def _resolve_pipeline(cpp, pipeline_preset: str, gcpc_cache=None):
    preset_name = str(pipeline_preset).strip()
    preset_key = preset_name.lower()

    if preset_key == "fast":
        return preset_name, cpp.PipelineConfig.fast(), "IFK", "LinkIAABB"

    if preset_key == "recommended":
        return preset_name, cpp.PipelineConfig.recommended(), "CritSample", "Hull16_Grid"

    pipeline = cpp.PipelineConfig()
    if preset_key == "tightest":
        pipeline.source = cpp.EndpointSourceConfig.analytical()
        pipeline.envelope = cpp.EnvelopeTypeConfig.hull16_grid()
        return preset_name, pipeline, "Analytical", "Hull16_Grid"

    if preset_key == "production":
        pipeline.source = cpp.EndpointSourceConfig.ifk()
        pipeline.envelope = cpp.EnvelopeTypeConfig.hull16_grid()
        return preset_name, pipeline, "IFK", "Hull16_Grid"

    if preset_key in {"gcpc", "recommended_cache", "cached"}:
        if gcpc_cache is None:
            raise ValueError(f"Pipeline preset {pipeline_preset} requires a GCPC cache")
        return preset_name, cpp.PipelineConfig.recommended(gcpc_cache), "GCPC", "Hull16_Grid"

    raise ValueError(f"Unsupported pipeline preset: {pipeline_preset}")


def _resolve_mode(cpp, mode_name: str):
    mode_key = str(mode_name).strip().lower()
    mode_map = {
        "wavefront": cpp.GrowerConfig.Mode.Wavefront,
        "rrt": cpp.GrowerConfig.Mode.RRT,
    }
    if mode_key not in mode_map:
        raise ValueError(f"Unsupported grower mode: {mode_name}")
    return mode_map[mode_key]


def _stats(values: Sequence[float]) -> Dict[str, float]:
    return {
        "mean": float(statistics.fmean(values)),
        "median": float(statistics.median(values)),
        "min": float(min(values)),
        "max": float(max(values)),
    }


def _summarize_rows(rows: Sequence[Dict[str, object]], group_keys: Sequence[str]) -> List[Dict[str, object]]:
    groups: Dict[Tuple[object, ...], List[Dict[str, object]]] = defaultdict(list)
    for row in rows:
        key = tuple(row[name] for name in group_keys)
        groups[key].append(row)

    summary_rows: List[Dict[str, object]] = []
    for key in sorted(groups.keys()):
        group = groups[key]
        build_stats = _stats([float(row["build_time_ms"]) for row in group])
        box_stats = _stats([float(row["n_boxes"]) for row in group])
        comp_stats = _stats([float(row["n_components"]) for row in group])
        volume_stats = _stats([float(row["total_volume"]) for row in group])

        summary = {name: value for name, value in zip(group_keys, key)}
        summary.update(
            {
                "n_runs": len(group),
                "actual_n_obstacles_mean": float(statistics.fmean(float(row["actual_n_obstacles"]) for row in group)),
                "n_boxes_mean": box_stats["mean"],
                "n_boxes_median": box_stats["median"],
                "n_boxes_min": box_stats["min"],
                "n_boxes_max": box_stats["max"],
                "n_components_mean": comp_stats["mean"],
                "build_time_ms_mean": build_stats["mean"],
                "build_time_ms_median": build_stats["median"],
                "build_time_ms_min": build_stats["min"],
                "build_time_ms_max": build_stats["max"],
                "total_volume_mean": volume_stats["mean"],
                "total_volume_median": volume_stats["median"],
                "total_volume_min": volume_stats["min"],
                "total_volume_max": volume_stats["max"],
                "start_goal_connected_rate": float(
                    statistics.fmean(1.0 if bool(row["start_goal_connected"]) else 0.0 for row in group)
                ),
            }
        )
        summary_rows.append(summary)
    return summary_rows


def _write_csv(path: Path, rows: Sequence[Dict[str, object]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def run_pipeline_benchmark(
    repo_root: Path,
    output_dir: Path,
    *,
    robots: Sequence[str],
    modes: Sequence[str],
    pipeline_presets: Sequence[str],
    obstacle_counts: Sequence[int],
    n_scenes: int,
    base_seed: int,
    max_boxes: int,
    min_edge: float,
    max_depth: int,
    max_consecutive_miss: int,
    timeout: float,
    n_roots: int,
    gcpc_cache_dir: Path | None = None,
    gcpc_cache_profile: str = "quick",
    gcpc_random_seeds: int | None = None,
    gcpc_max_sweeps: int | None = None,
    gcpc_max_exact_product: int | None = None,
    gcpc_random_configs: int | None = None,
    gcpc_root_n_candidates: int | None = None,
) -> Dict[str, object]:
    cpp = _load_cpp(repo_root)
    output_dir.mkdir(parents=True, exist_ok=True)

    robot_objects = {
        robot_name: cpp.Robot.from_json(str(repo_root / ROBOT_JSONS[robot_name]))
        for robot_name in robots
    }
    cache_requested = any(str(preset).strip().lower() in {"gcpc", "recommended_cache", "cached"} for preset in pipeline_presets)
    cache_metadata: Dict[str, Dict[str, object]] = {}
    robot_caches: Dict[str, object] = {}
    build_params = _resolve_gcpc_build_params(
        gcpc_cache_profile,
        gcpc_random_seeds,
        gcpc_max_sweeps,
        gcpc_max_exact_product,
        gcpc_random_configs,
    )
    resolved_gcpc_root_n_candidates = _resolve_gcpc_root_n_candidates(
        gcpc_cache_profile,
        gcpc_root_n_candidates,
    )
    cache_root = gcpc_cache_dir or (output_dir / "gcpc_cache")
    if cache_requested:
        cache_root.mkdir(parents=True, exist_ok=True)
        print(
            "[gcpc] cache profile="
            f"{gcpc_cache_profile} params="
            f"seeds={build_params['gcpc_random_seeds']}, "
            f"sweeps={build_params['gcpc_max_sweeps']}, "
            f"exact_product={build_params['gcpc_max_exact_product']}, "
            f"random_configs={build_params['gcpc_random_configs']}, "
            f"root_candidates={resolved_gcpc_root_n_candidates}"
        )

    def ensure_gcpc_cache(robot_name: str):
        if robot_name in robot_caches:
            return robot_caches[robot_name]

        robot = robot_objects[robot_name]
        cache_path, meta_path = _gcpc_cache_paths(cache_root, robot_name, build_params)
        cache = cpp.GcpcCache()
        loaded = False
        build_ms = None
        meta = _load_gcpc_cache_meta(meta_path)

        if cache_path.exists() and _gcpc_meta_matches(meta, robot_name, build_params):
            print(f"[gcpc] {robot_name}: loading cache {cache_path.name}")
            loaded = bool(cache.load(str(cache_path)))
            if not loaded:
                print(f"[gcpc] {robot_name}: cache load failed, rebuilding")

        if not loaded:
            print(
                f"[gcpc] {robot_name}: building cache "
                f"(profile={gcpc_cache_profile}, "
                f"seeds={build_params['gcpc_random_seeds']}, "
                f"sweeps={build_params['gcpc_max_sweeps']}, "
                f"exact_product={build_params['gcpc_max_exact_product']}, "
                f"random_configs={build_params['gcpc_random_configs']})"
            )
            t0 = datetime.now()
            cache = cpp.build_gcpc_cache(
                robot,
                n_random_seeds=build_params["gcpc_random_seeds"],
                max_sweeps=build_params["gcpc_max_sweeps"],
                max_exact_product=build_params["gcpc_max_exact_product"],
                n_random_configs=build_params["gcpc_random_configs"],
            )
            build_ms = (datetime.now() - t0).total_seconds() * 1000.0
            cache.save(str(cache_path))
            _write_gcpc_cache_meta(
                meta_path,
                robot_name=robot_name,
                cache_file=cache_path,
                profile_name=gcpc_cache_profile,
                build_params=build_params,
                build_time_ms=build_ms,
                cache=cache,
            )
            print(
                f"[gcpc] {robot_name}: built cache in {build_ms:.1f} ms, "
                f"points={int(cache.n_total_points())}"
            )
        elif meta is None:
            _write_gcpc_cache_meta(
                meta_path,
                robot_name=robot_name,
                cache_file=cache_path,
                profile_name=gcpc_cache_profile,
                build_params=build_params,
                build_time_ms=None,
                cache=cache,
            )

        robot_caches[robot_name] = cache
        cache_metadata[robot_name] = {
            "cache_file": str(cache_path.relative_to(repo_root)).replace("\\", "/"),
            "cache_meta_file": str(meta_path.relative_to(repo_root)).replace("\\", "/"),
            "loaded_from_file": loaded,
            "build_time_ms": build_ms,
            "gcpc_cache_profile": gcpc_cache_profile,
            "n_links": int(cache.n_links()),
            "n_total_points": int(cache.n_total_points()),
            **build_params,
        }
        return cache

    run_rows: List[Dict[str, object]] = []
    total_runs = len(robots) * len(modes) * len(pipeline_presets) * len(obstacle_counts) * n_scenes
    run_idx = 0

    for robot_index, robot_name in enumerate(robots):
        robot = robot_objects[robot_name]
        for mode_index, mode_name in enumerate(modes):
            mode_enum = _resolve_mode(cpp, mode_name)
            for preset_index, preset_name in enumerate(pipeline_presets):
                preset_key = str(preset_name).strip().lower()
                gcpc_cache = ensure_gcpc_cache(robot_name) if preset_key in {"gcpc", "recommended_cache", "cached"} else None
                pipeline_name, pipeline_cfg, endpoint_source, envelope_type = _resolve_pipeline(cpp, preset_name, gcpc_cache)
                for obstacle_count in obstacle_counts:
                    for scene_idx in range(n_scenes):
                        scene_seed = base_seed + obstacle_count * 1000 + scene_idx
                        obs_rng = np.random.default_rng(scene_seed)
                        obstacles = _generate_obstacles(cpp, obs_rng, obstacle_count)
                        actual_n_obstacles = len(obstacles)

                        ep_seed = scene_seed * 31 + robot_index + mode_index * 7 + preset_index * 13
                        ep_rng = np.random.default_rng(ep_seed)
                        start = _random_config(robot, ep_rng)
                        goal = _random_config(robot, ep_rng)

                        cfg = cpp.GrowerConfig()
                        cfg.mode = mode_enum
                        cfg.max_boxes = max_boxes
                        cfg.min_edge = min_edge
                        cfg.max_depth = max_depth
                        cfg.max_consecutive_miss = max_consecutive_miss
                        cfg.timeout = timeout
                        cfg.rng_seed = scene_seed
                        cfg.n_roots = n_roots
                        cfg.partition_mode = cpp.PartitionMode.LectAligned
                        cfg.pipeline = pipeline_cfg
                        if endpoint_source == "GCPC":
                            cfg.root_n_candidates = resolved_gcpc_root_n_candidates

                        grower = cpp.ForestGrower(robot, cfg)
                        grower.set_endpoints(start, goal)
                        grower.set_split_order(cpp.SplitOrder.BEST_TIGHTEN)
                        result = grower.grow_typed(obstacles)

                        phase_times = {str(key): float(value) for key, value in dict(result.phase_times).items()}
                        run_rows.append(
                            {
                                "robot": robot_name,
                                "mode": mode_name,
                                "pipeline_preset": pipeline_name,
                                "endpoint_source": endpoint_source,
                                "envelope_type": envelope_type,
                                "requested_n_obstacles": obstacle_count,
                                "actual_n_obstacles": actual_n_obstacles,
                                "scene_idx": scene_idx,
                                "scene_seed": scene_seed,
                                "endpoint_seed": ep_seed,
                                "n_boxes": int(result.n_boxes_total),
                                "n_components": int(result.n_components),
                                "n_ffb_success": int(result.n_ffb_success),
                                "n_ffb_fail": int(result.n_ffb_fail),
                                "start_goal_connected": bool(result.start_goal_connected),
                                "n_bridge_boxes": int(result.n_bridge_boxes),
                                "n_promotions": int(result.n_promotions),
                                "n_fine_boxes": int(result.n_fine_boxes),
                                "total_volume": float(result.total_volume),
                                "build_time_ms": float(result.build_time_ms),
                                "phase_root_select_ms": float(phase_times.get("root_select", phase_times.get("root_select_ms", 0.0))),
                                "phase_expand_ms": float(phase_times.get("expand", phase_times.get("expand_ms", 0.0))),
                                "phase_adj_rebuild_ms": float(phase_times.get("adj_rebuild", phase_times.get("adj_rebuild_ms", 0.0))),
                            }
                        )

                        run_idx += 1
                        print(
                            f"[{run_idx}/{total_runs}] robot={robot_name} mode={mode_name} preset={pipeline_name} "
                            f"obs={obstacle_count} scene={scene_idx} boxes={result.n_boxes_total} "
                            f"volume={float(result.total_volume):.6f} build_ms={float(result.build_time_ms):.3f}"
                        )

    summary_robot_mode_pipeline = _summarize_rows(run_rows, ["robot", "mode", "pipeline_preset", "endpoint_source", "envelope_type"])
    summary_by_obstacles = _summarize_rows(
        run_rows,
        ["robot", "mode", "pipeline_preset", "endpoint_source", "envelope_type", "requested_n_obstacles"],
    )

    metadata = {
        "robots": list(robots),
        "modes": list(modes),
        "pipeline_presets": list(pipeline_presets),
        "obstacle_counts": list(obstacle_counts),
        "n_scenes": n_scenes,
        "base_seed": base_seed,
        "max_boxes": max_boxes,
        "min_edge": min_edge,
        "max_depth": max_depth,
        "max_consecutive_miss": max_consecutive_miss,
        "timeout": timeout,
        "n_roots": n_roots,
        "gcpc_cache_profile": gcpc_cache_profile,
        "gcpc_cache_build_params": build_params,
        "gcpc_root_n_candidates": resolved_gcpc_root_n_candidates,
        "split_order": "BEST_TIGHTEN",
        "partition_mode": "LectAligned",
        "gcpc_cache": cache_metadata,
        "scene_generator": {
            "x": [WS_XMIN, WS_XMAX],
            "y": [WS_YMIN, WS_YMAX],
            "z": [WS_ZMIN, WS_ZMAX],
            "size": [MIN_SIZE, MAX_SIZE],
            "base_clearance": BASE_CLEARANCE,
        },
    }

    (output_dir / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    (output_dir / "runs.json").write_text(json.dumps(run_rows, indent=2), encoding="utf-8")
    (output_dir / "summary_robot_mode_pipeline.json").write_text(
        json.dumps(summary_robot_mode_pipeline, indent=2),
        encoding="utf-8",
    )
    (output_dir / "summary_robot_mode_pipeline_obstacles.json").write_text(
        json.dumps(summary_by_obstacles, indent=2),
        encoding="utf-8",
    )
    _write_csv(output_dir / "runs.csv", run_rows)
    _write_csv(output_dir / "summary_robot_mode_pipeline.csv", summary_robot_mode_pipeline)
    _write_csv(output_dir / "summary_robot_mode_pipeline_obstacles.csv", summary_by_obstacles)

    return {
        "metadata": metadata,
        "runs": run_rows,
        "summary_robot_mode_pipeline": summary_robot_mode_pipeline,
        "summary_robot_mode_pipeline_obstacles": summary_by_obstacles,
    }


def build_cli_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Benchmark EP iAABB pipeline presets on iiwa14/panda grower scenes.")
    parser.add_argument("--output-dir", type=str, default=None, help="Output directory. Defaults to results/exp_robot_pipeline_compare_<timestamp>.")
    parser.add_argument("--robots", type=str, default="iiwa14,panda", help="Comma-separated robots: iiwa14,panda")
    parser.add_argument("--modes", type=str, default="Wavefront,RRT", help="Comma-separated grower modes: Wavefront,RRT")
    parser.add_argument(
        "--pipeline-presets",
        type=str,
        default="fast,recommended,tightest,production",
        help="Comma-separated pipeline presets.",
    )
    parser.add_argument("--obstacle-counts", type=str, default="5,10,20", help="Comma-separated obstacle counts.")
    parser.add_argument("--n-scenes", type=int, default=5, help="Random scenes per obstacle count.")
    parser.add_argument("--base-seed", type=int, default=42, help="Base seed for deterministic scene generation.")
    parser.add_argument("--max-boxes", type=int, default=300, help="ForestGrower max_boxes.")
    parser.add_argument("--min-edge", type=float, default=0.01, help="ForestGrower min_edge.")
    parser.add_argument("--max-depth", type=int, default=30, help="ForestGrower max_depth.")
    parser.add_argument("--max-consecutive-miss", type=int, default=200, help="ForestGrower max_consecutive_miss.")
    parser.add_argument("--timeout", type=float, default=30.0, help="ForestGrower timeout in seconds.")
    parser.add_argument("--n-roots", type=int, default=2, help="ForestGrower root count.")
    parser.add_argument("--gcpc-cache-dir", type=str, default=None, help="Directory to load/save per-robot GCPC caches.")
    parser.add_argument(
        "--gcpc-cache-profile",
        type=str,
        default="quick",
        choices=sorted(GCPC_CACHE_PROFILES.keys()),
        help="Preset cache build budget. quick is the new default to avoid very long first-run cache builds.",
    )
    parser.add_argument("--gcpc-random-seeds", type=int, default=None, help="Override interior-search random seeds when building a GCPC cache.")
    parser.add_argument("--gcpc-max-sweeps", type=int, default=None, help="Override interior-search max sweeps when building a GCPC cache.")
    parser.add_argument("--gcpc-max-exact-product", type=int, default=None, help="Override enumeration cutoff before switching to random sampling in GCPC cache build.")
    parser.add_argument("--gcpc-random-configs", type=int, default=None, help="Override random configurations per link when GCPC exact enumeration is truncated.")
    parser.add_argument("--gcpc-root-n-candidates", type=int, default=None, help="Override best-of-N root probes for cached GCPC runs.")
    return parser


def main() -> None:
    parser = build_cli_parser()
    args = parser.parse_args()

    repo_root = _repo_root()
    output_dir = _resolve_output_dir(repo_root, args.output_dir, "exp_robot_pipeline_compare")

    robots = _parse_csv_list(args.robots, str)
    invalid_robots = [name for name in robots if name not in ROBOT_JSONS]
    if invalid_robots:
        raise ValueError(f"Unsupported robots: {invalid_robots}")

    result = run_pipeline_benchmark(
        repo_root,
        output_dir,
        robots=robots,
        modes=_parse_csv_list(args.modes, str),
        pipeline_presets=_parse_csv_list(args.pipeline_presets, str),
        obstacle_counts=_parse_csv_list(args.obstacle_counts, int),
        n_scenes=args.n_scenes,
        base_seed=args.base_seed,
        max_boxes=args.max_boxes,
        min_edge=args.min_edge,
        max_depth=args.max_depth,
        max_consecutive_miss=args.max_consecutive_miss,
        timeout=args.timeout,
        n_roots=args.n_roots,
        gcpc_cache_dir=_resolve_cache_dir(repo_root, args.gcpc_cache_dir, output_dir.name),
        gcpc_cache_profile=args.gcpc_cache_profile,
        gcpc_random_seeds=args.gcpc_random_seeds,
        gcpc_max_sweeps=args.gcpc_max_sweeps,
        gcpc_max_exact_product=args.gcpc_max_exact_product,
        gcpc_random_configs=args.gcpc_random_configs,
        gcpc_root_n_candidates=args.gcpc_root_n_candidates,
    )

    print(f"Wrote robot pipeline comparison outputs to: {output_dir}")
    print(f"Recorded {len(result['runs'])} runs.")


if __name__ == "__main__":
    main()
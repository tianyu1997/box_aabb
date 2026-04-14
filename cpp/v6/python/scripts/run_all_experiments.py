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


# ── Helpers ──────────────────────────────────────────────────

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


def _make_ep_config(source_name):
    """Create EndpointSourceConfig from name."""
    import sbf5
    cfg = sbf5.EndpointSourceConfig()
    cfg.source = {
        "IFK": sbf5.EndpointSource.IFK,
        "CritSample": sbf5.EndpointSource.CritSample,
        "Analytical": sbf5.EndpointSource.Analytical,
        "GCPC": sbf5.EndpointSource.GCPC,
    }[source_name]
    return cfg


def _make_env_config(type_name):
    """Create EnvelopeTypeConfig from name."""
    import sbf5
    cfg = sbf5.EnvelopeTypeConfig()
    cfg.type = {
        "LinkIAABB": sbf5.EnvelopeType.LinkIAABB,
        "LinkIAABB_Grid": sbf5.EnvelopeType.LinkIAABB_Grid,
        "Hull16_Grid": sbf5.EnvelopeType.Hull16_Grid,
    }[type_name]
    return cfg


ALL_EP_SOURCES = ["IFK", "CritSample", "Analytical", "GCPC"]
ALL_ENV_TYPES = ["LinkIAABB", "LinkIAABB_Grid", "Hull16_Grid"]

def run_s1(quick: bool = False, lite: bool = False):
    """S1: Envelope Tightness — compare volumes across pipeline configs."""
    import sbf5

    out_dir = os.path.join(PAPER_DIR, "s1_envelope_tightness")
    os.makedirs(out_dir, exist_ok=True)
    result_path = os.path.join(out_dir, "results.json")
    if os.path.exists(result_path):
        logger.info("S1: results already exist, skipping")
        return

    n_boxes = 50 if quick else (200 if lite else 500)
    ep_sources = ALL_EP_SOURCES[:2] if quick else ALL_EP_SOURCES
    env_types = ALL_ENV_TYPES[:2] if quick else ALL_ENV_TYPES

    robots = {
        "2dof": sbf5.Robot.from_json(os.path.join("data", "2dof_planar.json")),
        "panda": sbf5.Robot.from_json(os.path.join("data", "panda.json")),
    }

    # Load GCPC caches
    gcpc_caches = {}
    if "GCPC" in ep_sources:
        for rname, robj in robots.items():
            cache_path = os.path.join("data", f"{rname}_{'500' if rname == '2dof' else '5000'}.gcpc")
            if os.path.exists(cache_path):
                gcpc_caches[rname] = sbf5.GcpcCache.load(cache_path)
                logger.info("S1: loaded GCPC cache %s (%d pts)", cache_path, gcpc_caches[rname].n_points())

    all_rows = []
    for rname, robot in robots.items():
        rng = np.random.RandomState(42)
        logger.info("S1: %s — sampling %d boxes", rname, n_boxes)

        for ep in ep_sources:
            if ep == "GCPC" and rname not in gcpc_caches:
                logger.warning("S1: skip GCPC for %s (no cache)", rname)
                continue

            ep_cfg = _make_ep_config(ep)
            gcpc = gcpc_caches.get(rname) if ep == "GCPC" else None

            for env in env_types:
                env_cfg = _make_env_config(env)
                volumes = []
                safe_flags = []
                times_us = []

                for _ in range(n_boxes):
                    intervals = _random_intervals(robot, rng)
                    info = sbf5.compute_envelope_info(
                        robot, intervals, ep_cfg, env_cfg, gcpc)
                    volumes.append(info["volume"])
                    safe_flags.append(info["is_safe"])
                    times_us.append(info["total_time_us"])

                row = {
                    "robot": rname,
                    "endpoint": ep,
                    "envelope": env,
                    "volume_mean": float(np.mean(volumes)),
                    "volume_std": float(np.std(volumes)),
                    "safe": bool(all(safe_flags)),
                    "time_us_mean": float(np.mean(times_us)),
                    "n_boxes": n_boxes,
                }
                all_rows.append(row)
                logger.info("  %s-%s: vol=%.4f ± %.4f, safe=%s",
                            ep, env, row["volume_mean"], row["volume_std"],
                            row["safe"])

    # Compute ratio vs IFK-LinkIAABB baseline per robot
    for rname in robots:
        baseline = next(
            (r for r in all_rows
             if r["robot"] == rname and r["endpoint"] == "IFK"
             and r["envelope"] == "LinkIAABB"), None)
        base_vol = baseline["volume_mean"] if baseline else 1.0
        for r in all_rows:
            if r["robot"] == rname:
                r["ratio_pct"] = round(100.0 * r["volume_mean"] / base_vol, 1) if base_vol > 0 else 0.0

    with open(result_path, "w", encoding="utf-8") as f:
        json.dump({"rows": all_rows}, f, indent=2)
    logger.info("S1: → %s (%d rows)", result_path, len(all_rows))


def run_s2(quick: bool = False, lite: bool = False):
    """S2: Envelope Timing — measure endpoint + envelope computation time."""
    import sbf5

    out_dir = os.path.join(PAPER_DIR, "s2_envelope_timing")
    os.makedirs(out_dir, exist_ok=True)
    result_path = os.path.join(out_dir, "results.json")
    if os.path.exists(result_path):
        logger.info("S2: results already exist, skipping")
        return

    n_boxes = 100 if quick else (300 if lite else 1000)
    n_repeats = 10 if quick else (20 if lite else 50)
    ep_sources = ALL_EP_SOURCES[:2] if quick else ALL_EP_SOURCES
    env_types = ALL_ENV_TYPES[:2] if quick else ALL_ENV_TYPES

    robots = {
        "2dof": sbf5.Robot.from_json(os.path.join("data", "2dof_planar.json")),
        "panda": sbf5.Robot.from_json(os.path.join("data", "panda.json")),
    }

    gcpc_caches = {}
    if "GCPC" in ep_sources:
        for rname in robots:
            cache_path = os.path.join("data", f"{rname}_{'500' if rname == '2dof' else '5000'}.gcpc")
            if os.path.exists(cache_path):
                gcpc_caches[rname] = sbf5.GcpcCache.load(cache_path)

    all_rows = []
    for rname, robot in robots.items():
        rng = np.random.RandomState(123)
        # Pre-sample boxes
        box_list = [_random_intervals(robot, rng) for _ in range(n_boxes)]
        logger.info("S2: %s — timing %d boxes × %d repeats", rname, n_boxes, n_repeats)

        for ep in ep_sources:
            if ep == "GCPC" and rname not in gcpc_caches:
                continue
            ep_cfg = _make_ep_config(ep)
            gcpc = gcpc_caches.get(rname) if ep == "GCPC" else None

            for env in env_types:
                env_cfg = _make_env_config(env)
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
                                    ep, env, bi + 1, n_boxes)

                row = {
                    "robot": rname,
                    "endpoint": ep,
                    "envelope": env,
                    "ep_us_mean": float(np.mean(ep_times)),
                    "ep_us_std": float(np.std(ep_times)),
                    "env_us_mean": float(np.mean(env_times)),
                    "env_us_std": float(np.std(env_times)),
                    "total_us_mean": float(np.mean(total_times)),
                    "total_us_std": float(np.std(total_times)),
                    "n_samples": len(total_times),
                }
                all_rows.append(row)
                logger.info("  %s-%s: total=%.0f±%.0f μs",
                            ep, env, row["total_us_mean"], row["total_us_std"])

    # Compute speedup vs Analytical-LinkIAABB per robot
    for rname in robots:
        ref = next(
            (r for r in all_rows
             if r["robot"] == rname and r["endpoint"] == "Analytical"
             and r["envelope"] == "LinkIAABB"), None)
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
    args = parser.parse_args()

    os.makedirs(PAPER_DIR, exist_ok=True)

    if not args.skip_experiments:
        logger.info("═══ Phase S1: Envelope Tightness ═══")
        run_s1(quick=args.quick, lite=args.lite)

        logger.info("═══ Phase S2: Envelope Timing ═══")
        run_s2(quick=args.quick, lite=args.lite)

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

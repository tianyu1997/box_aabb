"""
sbf5_bench/runner.py — Experiment orchestrator

Runs multiple planners on multiple scenes with multiple seeds,
collects results, and supports checkpoint/resume.
"""

from __future__ import annotations

import json
import logging
import os
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime
from typing import Dict, List, Optional

import numpy as np

from .base import BasePlanner, PlanningResult
from .metrics import PathMetrics, evaluate_result
from .scenes import BenchmarkScene, get_scene

logger = logging.getLogger(__name__)


@dataclass
class PipelineConfig:
    """One pipeline = endpoint_source × envelope_type."""

    endpoint_source: str
    envelope_type: str

    @property
    def label(self) -> str:
        return f"{self.endpoint_source}-{self.envelope_type}"


ALL_PIPELINE_CONFIGS = [
    PipelineConfig(ep, env)
    for ep in ["IFK", "CritSample", "Analytical", "GCPC"]
    for env in ["LinkIAABB", "LinkIAABB_Grid", "Hull16_Grid"]
]


@dataclass
class ExperimentConfig:
    """Configuration for a benchmark experiment run."""

    scenes: List[str]                   # scene names (from SCENES dict)
    planners: List[BasePlanner]         # planner instances
    pipeline_configs: Optional[List[PipelineConfig]] = None
    gcpc_cache_path: Optional[str] = None
    n_trials: int = 10
    seeds: Optional[List[int]] = None   # if None, range(n_trials)
    timeout: float = 30.0               # per-query timeout (seconds)
    output_dir: str = "results/"

    def get_seeds(self) -> List[int]:
        if self.seeds is not None:
            return self.seeds
        return list(range(self.n_trials))


@dataclass
class TrialResult:
    """Result of a single (scene, planner, seed) trial."""

    scene: str
    planner: str
    seed: int
    trial_idx: int
    planning_result: PlanningResult
    metrics: PathMetrics


@dataclass
class ExperimentResults:
    """Collected results from a full experiment run."""

    trials: List[TrialResult] = field(default_factory=list)
    config: dict = field(default_factory=dict)
    timestamp: str = ""

    def save(self, path: str) -> None:
        """Save results to JSON."""
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)

        data = {
            "timestamp": self.timestamp,
            "config": self.config,
            "trials": [],
        }
        for t in self.trials:
            trial_dict = {
                "scene": t.scene,
                "planner": t.planner,
                "seed": t.seed,
                "trial_idx": t.trial_idx,
                "result": t.planning_result.to_dict(),
                "metrics": t.metrics.to_dict(),
            }
            data["trials"].append(trial_dict)

        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, default=_json_default)

    @staticmethod
    def load(path: str) -> "ExperimentResults":
        """Load results from JSON."""
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)

        results = ExperimentResults(
            timestamp=data.get("timestamp", ""),
            config=data.get("config", {}),
        )

        for td in data.get("trials", []):
            pr = PlanningResult(
                success=td["result"].get("success", False),
                cost=td["result"].get("cost", 0.0),
                planning_time_s=td["result"].get("planning_time_s", 0.0),
                collision_checks=td["result"].get("collision_checks", 0),
                nodes_explored=td["result"].get("nodes_explored", 0),
            )
            pm = PathMetrics(
                path_length=td["metrics"].get("path_length", 0.0),
                direct_distance=td["metrics"].get("direct_distance", 0.0),
                efficiency=td["metrics"].get("efficiency", 0.0),
                smoothness_mean=td["metrics"].get("smoothness_mean", 0.0),
                smoothness_max=td["metrics"].get("smoothness_max", 0.0),
                n_waypoints=td["metrics"].get("n_waypoints", 0),
                planning_time_s=td["metrics"].get("planning_time_s", 0.0),
            )
            results.trials.append(TrialResult(
                scene=td["scene"],
                planner=td["planner"],
                seed=td["seed"],
                trial_idx=td["trial_idx"],
                planning_result=pr,
                metrics=pm,
            ))
        return results

    def summary_df(self):
        """Return a pandas DataFrame summary (requires pandas)."""
        import pandas as pd

        rows = []
        for t in self.trials:
            row = {
                "scene": t.scene,
                "planner": t.planner,
                "seed": t.seed,
                "success": t.planning_result.success,
                **t.metrics.to_dict(),
            }
            rows.append(row)
        return pd.DataFrame(rows)


def run_experiment(config: ExperimentConfig) -> ExperimentResults:
    """Execute a full benchmark experiment.

    Iterates: scenes × planners × seeds, collecting PlanningResult
    and PathMetrics for each trial.
    """
    seeds = config.get_seeds()

    # If pipeline_configs specified, auto-generate SBFPlannerAdapter instances
    planners = list(config.planners)
    if config.pipeline_configs:
        from .sbf_adapter import SBFPlannerAdapter
        planners = [
            SBFPlannerAdapter(
                endpoint_source=pc.endpoint_source,
                envelope_type=pc.envelope_type,
                gcpc_cache_path=config.gcpc_cache_path,
            )
            for pc in config.pipeline_configs
        ] + planners

    results = ExperimentResults(
        timestamp=datetime.now().isoformat(),
        config={
            "scenes": config.scenes,
            "planners": [p.name for p in planners],
            "n_trials": config.n_trials,
            "seeds": seeds,
            "timeout": config.timeout,
        },
    )

    # Check for existing checkpoint
    checkpoint_path = os.path.join(config.output_dir, "_checkpoint.json")
    done_keys = set()
    if os.path.exists(checkpoint_path):
        prev = ExperimentResults.load(checkpoint_path)
        results.trials = prev.trials
        for t in prev.trials:
            done_keys.add((t.scene, t.planner, t.seed))
        logger.info("Resuming from checkpoint: %d trials done", len(done_keys))

    total = len(config.scenes) * len(planners) * len(seeds)
    completed = len(done_keys)

    for scene_name in config.scenes:
        scene = get_scene(scene_name)
        robot = scene.make_robot()
        obstacles = scene.make_obstacles()

        for planner in planners:
            planner.setup(robot, obstacles)

            for trial_idx, seed in enumerate(seeds):
                key = (scene_name, planner.name, seed)
                if key in done_keys:
                    continue

                logger.info(
                    "[%d/%d] %s × %s (seed=%d)",
                    completed + 1, total, scene_name, planner.name, seed)

                # Set seed if planner config supports it
                pr = planner.plan(
                    scene.start, scene.goal,
                    timeout=config.timeout,
                )

                metrics = evaluate_result(pr, robot, obstacles)

                trial = TrialResult(
                    scene=scene_name,
                    planner=planner.name,
                    seed=seed,
                    trial_idx=trial_idx,
                    planning_result=pr,
                    metrics=metrics,
                )
                results.trials.append(trial)
                completed += 1

                # Periodic checkpoint
                if completed % 5 == 0:
                    results.save(checkpoint_path)

            if not planner.supports_reuse:
                planner.reset()

    # Final save
    os.makedirs(config.output_dir, exist_ok=True)
    output_path = os.path.join(
        config.output_dir,
        f"results_{results.timestamp.replace(':', '-')}.json",
    )
    results.save(output_path)

    # Clean checkpoint
    if os.path.exists(checkpoint_path):
        os.remove(checkpoint_path)

    logger.info("Experiment complete: %d trials → %s", completed, output_path)
    return results


def _json_default(obj):
    """JSON serializer for numpy types and special floats."""
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, (np.integer,)):
        return int(obj)
    if isinstance(obj, (np.floating,)):
        v = float(obj)
        if np.isnan(v):
            return None
        if np.isinf(v):
            return "inf" if v > 0 else "-inf"
        return v
    if isinstance(obj, np.bool_):
        return bool(obj)
    raise TypeError(f"Object of type {type(obj)} is not JSON serializable")

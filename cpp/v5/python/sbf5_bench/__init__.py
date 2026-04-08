"""sbf5_bench — Baselines, metrics, and experiment framework for SBF v5."""

__version__ = "0.1.0"

from .base import BasePlanner, PlanningResult
from .metrics import PathMetrics, evaluate_result, compare_results
from .scenes import BenchmarkScene, get_scene, list_scenes, SCENES
from .runner import (
    PipelineConfig, ALL_PIPELINE_CONFIGS,
    ExperimentConfig, TrialResult, ExperimentResults,
    run_experiment,
)

# Adapters (lazy-friendly — import errors deferred to usage)
from .sbf_adapter import SBFPlannerAdapter
from .ompl_adapter import OMPLPlanner
from .iris_gcs_adapter import IRISGCSPlanner

# Ported baselines
from .rrt_family import RRTPlanner
from .prm_baseline import PRMPlanner
from .iris_np_gcs import IRISNPGCSPlanner, CIRISGCSPlanner

# Statistics
from .stats import pairwise_significance

__all__ = [
    # Core
    "BasePlanner", "PlanningResult", "PathMetrics",
    "evaluate_result", "compare_results",
    # Scenes
    "BenchmarkScene", "get_scene", "list_scenes", "SCENES",
    # Runner
    "PipelineConfig", "ALL_PIPELINE_CONFIGS",
    "ExperimentConfig", "TrialResult", "ExperimentResults",
    "run_experiment",
    # Adapters
    "SBFPlannerAdapter", "OMPLPlanner", "IRISGCSPlanner",
    # Baselines
    "RRTPlanner", "PRMPlanner",
    "IRISNPGCSPlanner", "CIRISGCSPlanner",
    # Stats
    "pairwise_significance",
]

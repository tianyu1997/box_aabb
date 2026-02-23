"""Centralized planner configuration defaults.

All scripts, benchmarks, experiments should import from here
instead of hardcoding values.  When tuning, only this file
needs to change.
"""

from dataclasses import dataclass, field
from typing import List, Tuple


@dataclass
class PlannerDefaults:
    """Shared configuration for forest-based planners.

    These values are referenced by:
      - PandaGCSConfig  (pipeline.py)
      - SBFConfig        (models.py)
      - viz scripts      (viz_2dof_full_pipeline.py)
      - benchmark scripts (bench_bfs_queue.py)
      - experiment configs (standard_planners.json, exp4, exp6, …)
    """

    # ── forest growth ──
    ffb_min_edge: float = 0.05
    """Per-dimension edge-length threshold used inside
    ``find_free_box`` to stop KD-tree splitting.
    A split on dim *d* is blocked when ``edge_d < 2 * ffb_min_edge``."""

    max_boxes: int = 500
    max_consecutive_miss: int = 20
    guided_sample_ratio: float = 0.8

    # ── boundary expansion ──
    boundary_expand_epsilon: float = 0.01
    n_edge_samples: int = 3
    """Per-box directed edge expansion: sample N faces (0=all faces)."""

    # ── coarsen ──
    coarsen_max_rounds: int = 20

    # ── island / bridge ──
    min_island_size: float = 0.0

    # ── connectivity ──
    connection_radius: float = 1.5
    segment_collision_resolution: float = 0.05


# ── Per-scenario presets ──────────────────────────────────────────────────

PANDA_DEFAULTS = PlannerDefaults(
    ffb_min_edge=0.04,
    max_boxes=2000,
)

TWODOF_DEFAULTS = PlannerDefaults(
    ffb_min_edge=0.1,
    max_boxes=200,
)

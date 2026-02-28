"""
baselines/ — 统一规划器接口 + Baseline 适配器

- base: BasePlanner ABC, PlanningResult dataclass
- sbf_adapter: SafeBoxForest pipeline adapter (supports_reuse=True)
- rrt_family: RRT / RRTConnect / RRT* / Informed-RRT* / BiRRT*
- ompl_adapter: OMPL C++ family (subprocess via WSL)
- iris_gcs: Drake IRIS + GCS (Marcucci) — SBF-based wrapper
- iris_np_gcs: Real IRIS-NP-GCS / C-IRIS-GCS via pydrake
- prm_baseline: PRM multi-query baseline
"""

from .base import BasePlanner, PlanningResult
from .sbf_adapter import SBFAdapter
from .rrt_family import RRTPlanner
from .ompl_adapter import OMPLPlanner
from .iris_gcs import IRISGCSPlanner
from .iris_np_gcs import IRISNPGCSPlanner, CIRISGCSPlanner
from .prm_baseline import PRMPlanner

__all__ = [
    "BasePlanner",
    "PlanningResult",
    "SBFAdapter",
    "RRTPlanner",
    "OMPLPlanner",
    "IRISGCSPlanner",
    "IRISNPGCSPlanner",
    "CIRISGCSPlanner",
    "PRMPlanner",
]


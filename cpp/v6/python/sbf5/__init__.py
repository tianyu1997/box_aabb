"""SafeBoxForest v5 — Python interface.

This package wraps the C++ ``_sbf5_cpp`` extension module, providing:

Classes:
    Robot           DH-parameterised serial robot loaded from JSON.
    SBFPlanner      Top-level motion planner (build / plan / query).
    SBFPlannerConfig  Full configuration (grower, coarsening, smoother, GCS).
    GrowerConfig    Forest growth parameters (mode, max_boxes, n_threads, …).
    FFBConfig       Find-Free-Box parameters (max_depth).
    Interval        Closed 1-D interval [lo, hi].
    Obstacle        AABB obstacle in workspace.
    BoxNode         Collision-free box in C-space.
    PlanResult      Planning result with path, timing, and statistics.
    GcpcCache       Persistent GCPC critical-point cache.

Enums:
    GrowerMode      RRT / WAVEFRONT.
    EndpointSource  IFK / CritSample / Analytical / GCPC / MC.
    EnvelopeType    LinkIAABB / LinkIAABB_Grid / Hull16_Grid.
    SplitOrder      ROUND_ROBIN / WIDEST_FIRST / BEST_TIGHTEN.

Functions:
    compute_envelope_info   One-shot envelope volume + timing measurement.
    compute_link_iaabb_info One-shot LinkIAABB extraction for diagnostics.

Example::

    from sbf5 import Robot, SBFPlanner, Obstacle
    import numpy as np

    robot = Robot.from_json("data/panda.json")
    planner = SBFPlanner(robot)
    obs = [Obstacle(-0.5, -0.5, 0.0, 0.5, 0.5, 1.0)]
    result = planner.plan(
        start=np.zeros(7),
        goal=np.ones(7) * 0.5,
        obstacles=obs,
        timeout_ms=10000.0,
    )
    print(f"Success: {result.success}, path length: {result.path_length:.3f}")
"""
import importlib
import sys

try:
    _cpp = importlib.import_module("_sbf5_cpp")
except ImportError:
    _cpp = importlib.import_module("sbf5._sbf5_cpp")
sys.modules.setdefault(__name__ + "._sbf5_cpp", _cpp)

Interval = _cpp.Interval
Obstacle = _cpp.Obstacle
JointLimits = _cpp.JointLimits
BoxNode = _cpp.BoxNode
Robot = _cpp.Robot
FFBConfig = _cpp.FFBConfig
GrowerConfig = _cpp.GrowerConfig
GrowerMode = _cpp.GrowerMode
GreedyCoarsenConfig = _cpp.GreedyCoarsenConfig
SmootherConfig = _cpp.SmootherConfig
GCSConfig = _cpp.GCSConfig
SBFPlannerConfig = _cpp.SBFPlannerConfig
PlanResult = _cpp.PlanResult
EndpointSource = _cpp.EndpointSource
EnvelopeType = _cpp.EnvelopeType
SplitOrder = _cpp.SplitOrder
EndpointSourceConfig = _cpp.EndpointSourceConfig
EnvelopeTypeConfig = _cpp.EnvelopeTypeConfig
GridConfig = _cpp.GridConfig
GcpcCache = _cpp.GcpcCache
compute_envelope_info = _cpp.compute_envelope_info
compute_link_iaabb_info = _cpp.compute_link_iaabb_info
compute_endpoint_iaabb_info = _cpp.compute_endpoint_iaabb_info
SBFPlanner = getattr(_cpp, "SBFPlanner", None)

__version__ = "5.0.0"

__all__ = [
    "Interval", "Obstacle", "JointLimits", "BoxNode",
    "Robot",
    "GrowerConfig", "GrowerMode", "GreedyCoarsenConfig",
    "SmootherConfig", "GCSConfig",
    "SBFPlannerConfig", "PlanResult",
    "EndpointSource", "EnvelopeType",
    "EndpointSourceConfig", "EnvelopeTypeConfig", "GridConfig",
    "GcpcCache",
    "compute_envelope_info",
    "compute_link_iaabb_info",
    "compute_endpoint_iaabb_info",
]

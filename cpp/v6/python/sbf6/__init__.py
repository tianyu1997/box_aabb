"""SafeBoxForest v6 Python interface."""
import importlib
import sys

try:
    _cpp = importlib.import_module("_sbf6_cpp")
except ImportError:
    _cpp = importlib.import_module("sbf6._sbf6_cpp")
sys.modules.setdefault(__name__ + "._sbf6_cpp", _cpp)

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

__version__ = "6.0.0"

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

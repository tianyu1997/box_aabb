#!/usr/bin/env python3
from __future__ import annotations

import sys
import numpy as np

from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from common import bootstrap_gcs_repo, configure_parser_package_map, load_robot_joint_limits

GCS_REPO = bootstrap_gcs_repo()


def build_robot_diagram_checker(
    *,
    edge_step_size: float = 0.05,
    env_padding: float = 0.0,
    self_padding: float = 0.0,
):
    from pydrake.multibody.parsing import LoadModelDirectives, ProcessModelDirectives
    from pydrake.planning import RobotDiagramBuilder, SceneGraphCollisionChecker

    builder = RobotDiagramBuilder(time_step=0.0)
    parser = builder.parser()
    configure_parser_package_map(parser, gcs_repo=GCS_REPO)
    directives_file = GCS_REPO / "models" / "iiwa14_spheres_collision_welded_gripper.yaml"
    directives = LoadModelDirectives(str(directives_file))
    ProcessModelDirectives(directives, builder.plant(), parser)
    builder.plant().Finalize()

    plant = builder.plant()
    iiwa_inst = plant.GetModelInstanceByName("iiwa")
    wsg_inst = plant.GetModelInstanceByName("wsg")
    robot_diagram = builder.Build()

    checker = SceneGraphCollisionChecker(
        model=robot_diagram,
        robot_model_instances=[iiwa_inst, wsg_inst],
        edge_step_size=edge_step_size,
        env_collision_padding=env_padding,
        self_collision_padding=self_padding,
    )
    return robot_diagram, plant, checker


def joint_limit_domain(robot: str = "iiwa14"):
    from pydrake.all import HPolyhedron

    lo, hi = load_robot_joint_limits(robot)
    return HPolyhedron.MakeBox(np.asarray(lo, dtype=float), np.asarray(hi, dtype=float))


def solve_regions_gcs(
    q_start: np.ndarray,
    q_goal: np.ndarray,
    regions: list,
    *,
    seed: int,
    checker=None,
) -> dict:
    import importlib
    from pydrake.solvers import MosekSolver, SolverOptions
    import time

    LinearGCS = importlib.import_module("gcs.linear").LinearGCS
    randomForwardPathSearch = importlib.import_module(
        "gcs.rounding"
    ).randomForwardPathSearch

    t0 = time.perf_counter()
    n_regions = len(regions)
    if n_regions == 0:
        return {
            "success": False,
            "time_s": 0.0,
            "path_length": None,
            "regions": 0,
            "edges": 0,
            "note": "no regions",
        }

    gcs = LinearGCS(regions)
    try:
        gcs.addSourceTarget(q_start, q_goal)
    except ValueError as exc:
        return {
            "success": False,
            "time_s": time.perf_counter() - t0,
            "path_length": None,
            "regions": n_regions,
            "edges": 0,
            "note": f"addSourceTarget failed: {exc}",
        }

    gcs.setRoundingStrategy(randomForwardPathSearch, max_paths=10, max_trials=100, seed=int(seed))
    gcs.setSolver(MosekSolver())
    solver_options = SolverOptions()
    solver_options.SetOption(MosekSolver.id(), "MSK_DPAR_INTPNT_CO_TOL_REL_GAP", 1e-3)
    gcs.setSolverOptions(solver_options)

    try:
        waypoints, _ = gcs.SolvePath(rounding=True, verbose=False, preprocessing=True)
    except RuntimeError as exc:
        return {
            "success": False,
            "time_s": time.perf_counter() - t0,
            "path_length": None,
            "regions": n_regions,
            "edges": len([edge for edge in gcs.gcs.Edges()]),
            "note": f"GCS solve raised RuntimeError: {exc}",
        }

    dt = time.perf_counter() - t0
    if waypoints is None:
        return {
            "success": False,
            "time_s": dt,
            "path_length": None,
            "regions": n_regions,
            "edges": len([edge for edge in gcs.gcs.Edges()]),
            "note": "GCS solve failed",
        }

    path = waypoints.T
    path_length = float(
        sum(np.linalg.norm(path[idx] - path[idx - 1]) for idx in range(1, len(path)))
    )
    if checker is not None:
        unsafe_segments: list[int] = []
        for idx in range(1, len(path)):
            if not checker.CheckEdgeCollisionFree(path[idx - 1], path[idx]):
                unsafe_segments.append(idx - 1)
        if unsafe_segments:
            return {
                "success": False,
                "time_s": dt,
                "path_length": None,
                "raw_path_length": path_length,
                "regions": n_regions,
                "edges": len([edge for edge in gcs.gcs.Edges()]),
                "waypoints_count": int(path.shape[0]),
                "collision_checked": True,
                "collision_free": False,
                "unsafe_segments": len(unsafe_segments),
                "note": f"GCS path failed collision validation on {len(unsafe_segments)} segment(s)",
            }
    return {
        "success": True,
        "time_s": dt,
        "path_length": path_length,
        "regions": n_regions,
        "edges": len([edge for edge in gcs.gcs.Edges()]),
        "waypoints_count": int(path.shape[0]),
        "collision_checked": checker is not None,
        "collision_free": True if checker is not None else None,
        "unsafe_segments": 0 if checker is not None else None,
    }
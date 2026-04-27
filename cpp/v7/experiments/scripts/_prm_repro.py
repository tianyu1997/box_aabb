#!/usr/bin/env python3
from __future__ import annotations

import pickle
import sys
from pathlib import Path
from typing import Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from common import bootstrap_gcs_repo, configure_parser_package_map

GCS_REPO = bootstrap_gcs_repo()

from pydrake.all import (  # noqa: E402
    HolonomicKinematicPlanningSpace,
    JointLimits,
    LoadModelDirectives,
    PRMPlanner,
    PRMPlannerCreationParameters,
    PRMPlannerQueryParameters,
    PathProcessor,
    PathProcessorParameters,
    ProcessModelDirectives,
    RobotDiagramBuilder,
    VoxelizedEnvironmentCollisionChecker,
)


def model_file(relative_path: str) -> str:
    return str(GCS_REPO / relative_path)


DEFAULT_EDGE_STEP_SIZE = 0.05
DEFAULT_ENV_PADDING = 0.0
DEFAULT_SELF_PADDING = 0.0
DEFAULT_PROPAGATION_STEP_SIZE = 0.5
DEFAULT_GRID_SIZE = (1.8, 2.4, 1.6)
DEFAULT_GRID_RESOLUTION = 0.04
DEFAULT_ROADMAP_SIZE = 10_000
DEFAULT_NUM_NEIGHBORS = 5
DEFAULT_MAX_VALID_SAMPLE_TRIES = 100
DEFAULT_POSTPROCESSING = {
    "max_smoothing_shortcut_fraction": 1.0,
    "resampled_state_interval": 0.2,
    "max_smoothing_iterations": 200,
    "max_failed_smoothing_iterations": 200,
    "max_backtracking_steps": 0,
    "use_shortcut_smoothing": True,
    "resample_before_smoothing": True,
}


def default_creation_parameters(
    *,
    roadmap_size: int = DEFAULT_ROADMAP_SIZE,
    num_neighbors: int = DEFAULT_NUM_NEIGHBORS,
    max_valid_sample_tries: int = DEFAULT_MAX_VALID_SAMPLE_TRIES,
    parallelize: bool = True,
) -> PRMPlannerCreationParameters:
    params = PRMPlannerCreationParameters()
    params.roadmap_size = int(roadmap_size)
    params.num_neighbors = int(num_neighbors)
    params.max_valid_sample_tries = int(max_valid_sample_tries)
    params.parallelize = bool(parallelize)
    return params


def default_query_parameters(
    *,
    num_neighbors: int = DEFAULT_NUM_NEIGHBORS,
    parallelize: bool = True,
) -> PRMPlannerQueryParameters:
    params = PRMPlannerQueryParameters()
    params.num_neighbors = int(num_neighbors)
    params.parallelize = bool(parallelize)
    return params


def default_postprocessing_parameters(*, seed: int) -> PathProcessorParameters:
    params = PathProcessorParameters()
    params.max_smoothing_shortcut_fraction = DEFAULT_POSTPROCESSING["max_smoothing_shortcut_fraction"]
    params.resampled_state_interval = DEFAULT_POSTPROCESSING["resampled_state_interval"]
    params.prng_seed = int(seed)
    params.max_smoothing_iterations = DEFAULT_POSTPROCESSING["max_smoothing_iterations"]
    params.max_failed_smoothing_iterations = DEFAULT_POSTPROCESSING["max_failed_smoothing_iterations"]
    params.max_backtracking_steps = DEFAULT_POSTPROCESSING["max_backtracking_steps"]
    params.use_shortcut_smoothing = DEFAULT_POSTPROCESSING["use_shortcut_smoothing"]
    params.resample_before_smoothing = DEFAULT_POSTPROCESSING["resample_before_smoothing"]
    return params


class PresplinedPRM:
    def __init__(
        self,
        *,
        edge_step_size: float = DEFAULT_EDGE_STEP_SIZE,
        env_padding: float = DEFAULT_ENV_PADDING,
        self_padding: float = DEFAULT_SELF_PADDING,
        propagation_step_size: float = DEFAULT_PROPAGATION_STEP_SIZE,
        grid_size: Sequence[float] = DEFAULT_GRID_SIZE,
        grid_resolution: float = DEFAULT_GRID_RESOLUTION,
        seed: int = 0,
    ):
        builder = RobotDiagramBuilder(time_step=0.0)
        configure_parser_package_map(builder.parser(), gcs_repo=GCS_REPO)

        directives_file = model_file("models/iiwa14_spheres_collision_welded_gripper.yaml")
        directives = LoadModelDirectives(directives_file)
        ProcessModelDirectives(directives, builder.parser())

        iiwa_idx = builder.plant().GetModelInstanceByName("iiwa")
        wsg_idx = builder.plant().GetModelInstanceByName("wsg")

        builder.plant().Finalize()
        joint_limits = JointLimits(builder.plant())
        diagram = builder.Build()

        collision_checker = VoxelizedEnvironmentCollisionChecker(
            model=diagram,
            robot_model_instances=[iiwa_idx, wsg_idx],
            edge_step_size=edge_step_size,
            env_collision_padding=env_padding,
            self_collision_padding=self_padding,
            named_joint_distance_weights={f"iiwa_joint_{idx}": 1.0 for idx in range(1, 8)},
            default_joint_distance_weight=1.0,
        )
        collision_checker.VoxelizeEnvironment(tuple(grid_size), float(grid_resolution))

        self.planning_space = HolonomicKinematicPlanningSpace(
            collision_checker,
            joint_limits,
            propagation_step_size,
            int(seed),
        )
        self.roadmap = None

    def build_roadmap(
        self,
        anchor_sequence: Sequence[Sequence[float]],
        creation_parameters: PRMPlannerCreationParameters,
    ) -> float:
        splined_path = np.vstack([np.asarray(q, dtype=float) for q in anchor_sequence])
        self.roadmap, build_s = PRMPlanner.TimedBuildRoadmap(
            creation_parameters,
            splined_path,
            self.planning_space,
        )
        return float(build_s)

    def plan(
        self,
        sequence: Sequence[Sequence[float]],
        query_parameters: PRMPlannerQueryParameters,
        postprocessing_parameters: PathProcessorParameters | None = None,
    ) -> tuple[np.ndarray | None, float]:
        if self.roadmap is None:
            raise RuntimeError("Roadmap not built yet")

        path = [np.asarray(sequence[0], dtype=float)]
        run_time = 0.0
        for start, goal in zip(sequence[:-1], sequence[1:]):
            path_result, prm_run_time = PRMPlanner.TimedPlanLazy(
                np.asarray(start, dtype=float),
                np.asarray(goal, dtype=float),
                query_parameters,
                self.planning_space,
                self.roadmap,
            )
            run_time += float(prm_run_time)

            if not path_result.has_solution():
                return None, run_time

            if postprocessing_parameters is not None:
                processed_path, processing_time = PathProcessor.TimedProcessPath(
                    path_result.path(),
                    postprocessing_parameters,
                    self.planning_space,
                )
                run_time += float(processing_time)
                path.extend(np.asarray(q, dtype=float) for q in processed_path[1:])
            else:
                path.extend(np.asarray(q, dtype=float) for q in path_result.path()[1:])

        return np.stack(path).T, run_time

    def save(self, filename: str | Path) -> None:
        with Path(filename).open("wb") as handle:
            pickle.dump(self.roadmap, handle)

    def load(self, filename: str | Path) -> None:
        with Path(filename).open("rb") as handle:
            self.roadmap = pickle.load(handle)
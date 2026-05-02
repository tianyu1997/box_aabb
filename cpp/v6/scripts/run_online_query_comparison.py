#!/usr/bin/env python3
"""
Online Query Performance & Baseline Comparison experiment.

Compares SBF Dijkstra query vs GCS planning on IRIS-NP regions,
plus full baseline comparison (RRT-Connect, Lazy PRM).

Usage:
    conda activate sbf
    python3 scripts/run_online_query_comparison.py --seeds 5 \
        --json experiments/results_new/online_query_comparison.json
"""

import argparse
import json
import logging
import math
import os
import sys
import time

import numpy as np

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
logger = logging.getLogger(__name__)

DEFAULT_GOAL_BIAS = 0.1
DEFAULT_SBF_GROW_TIMEOUT_MS = 60000.0
DEFAULT_SBF_MAX_BOXES = 200000
DEFAULT_SBF_POST_CONNECT_EXTRA_BOXES = 4000
DEFAULT_SBF_THREADS = 16
DEFAULT_SBF_BRIDGE_THREADS = 16
DEFAULT_SBF_MAX_CONSECUTIVE_MISS = 2000
DEFAULT_SBF_RRT_STEP_RATIO = 0.05
DEFAULT_SBF_UNEXPLORED_SAMPLE_PROB = 0.7
DEFAULT_SBF_FFB_DEPTH = 300
DEFAULT_SBF_COARSEN_TARGET_BOXES = 300
DEFAULT_SBF_COARSEN_SCORE_THRESHOLD = 500.0
DEFAULT_SBF_ENABLE_PARTITIONED_LECT_PARALLEL = False
DEFAULT_SBF_PARTITIONED_BOX_BUDGET_PER_TREE = 0
DEFAULT_SBF_ENABLE_COORDINATED_MULTI_GOAL = True
DEFAULT_SBF_ENABLE_SEED_BRIDGE = False
DEFAULT_SBF_ENABLE_RESCUE_BRIDGE = True
DEFAULT_SBF_ENABLE_COARSEN = False
DEFAULT_SBF_PREBRIDGE_QUERY_PAIRS = True
DEFAULT_SBF_PREBRIDGE_PER_PAIR_TIMEOUT_MS = 20.0
DEFAULT_SBF_PREBRIDGE_MAX_PAIRS_PER_CALL = 1
DEFAULT_SBF_PREBRIDGE_MAX_QUERY_PAIRS = 1
DEFAULT_SBF_SEED_ORDER = ["AS", "TS", "CS", "LB", "RB"]
DEFAULT_SBF_ENDPOINT_SOURCE = "CritSample"
DEFAULT_SBF_ENVELOPE_TYPE = "LinkIAABB"
DEFAULT_SBF_ENVELOPE_SUBDIVISIONS = 4


def box_signature(box):
    return tuple((float(iv.lo), float(iv.hi)) for iv in box.joint_intervals)


def dedup_box_volume_stats(boxes):
    total_volume_sum = 0.0
    dedup_volume_sum = 0.0
    seen = set()
    for box in boxes:
        volume = float(box.volume)
        total_volume_sum += volume
        key = box_signature(box)
        if key in seen:
            continue
        seen.add(key)
        dedup_volume_sum += volume
    unique_box_count = len(seen)
    total_box_count = len(boxes)
    return {
        "box_volume_sum": total_volume_sum,
        "dedup_box_volume_sum": dedup_volume_sum,
        "duplicate_box_volume_sum": max(0.0, total_volume_sum - dedup_volume_sum),
        "unique_box_count": unique_box_count,
        "duplicate_box_count": max(0, total_box_count - unique_box_count),
    }


def _require_config_field(obj, field, owner):
    if not hasattr(obj, field):
        raise RuntimeError(
            f"SBF Python binding is missing {owner}.{field}; rebuild cpp/v6 "
            "with the current sbf6_bindings.cpp before running paper experiments."
        )


def apply_paper_sbf_architecture(
    config,
    *,
    seed,
    grow_timeout_ms=DEFAULT_SBF_GROW_TIMEOUT_MS,
    max_boxes=DEFAULT_SBF_MAX_BOXES,
    post_connect_extra_boxes=DEFAULT_SBF_POST_CONNECT_EXTRA_BOXES,
    n_threads=DEFAULT_SBF_THREADS,
    bridge_n_threads=DEFAULT_SBF_BRIDGE_THREADS,
    ffb_depth=DEFAULT_SBF_FFB_DEPTH,
    goal_bias=DEFAULT_GOAL_BIAS,
    lect_no_cache=True,
    lect_cache_dir=None,
    v6_cache_reads_enabled=True,
):
    """Apply the paper SBF build architecture shared by Exp. 3 and Exp. 4."""
    _require_config_field(config, "enable_seed_bridge", "SBFPlannerConfig")
    _require_config_field(config, "enable_rescue_bridge", "SBFPlannerConfig")
    _require_config_field(config, "enable_coarsen", "SBFPlannerConfig")
    _require_config_field(config, "v6_cache_reads_enabled", "SBFPlannerConfig")
    _require_config_field(config.grower, "enable_partitioned_lect_parallel", "GrowerConfig")
    _require_config_field(config.grower, "partitioned_box_budget_per_tree", "GrowerConfig")
    _require_config_field(config.grower, "enable_coordinated_multi_goal", "GrowerConfig")
    _require_config_field(config.grower, "unexplored_sample_prob", "GrowerConfig")

    config.grower.timeout_ms = float(grow_timeout_ms)
    config.grower.max_boxes = int(max_boxes)
    config.grower.post_connect_extra_boxes = int(post_connect_extra_boxes)
    config.grower.rng_seed = int(seed) * 1000 + 42
    config.grower.n_threads = int(n_threads)
    config.grower.bridge_n_threads = int(bridge_n_threads)
    config.grower.max_consecutive_miss = DEFAULT_SBF_MAX_CONSECUTIVE_MISS
    config.grower.rrt_goal_bias = float(goal_bias)
    config.grower.rrt_step_ratio = DEFAULT_SBF_RRT_STEP_RATIO
    config.grower.unexplored_sample_prob = DEFAULT_SBF_UNEXPLORED_SAMPLE_PROB
    config.grower.ffb_config.max_depth = int(ffb_depth)
    config.grower.connect_mode = True
    config.grower.enable_partitioned_lect_parallel = DEFAULT_SBF_ENABLE_PARTITIONED_LECT_PARALLEL
    config.grower.partitioned_box_budget_per_tree = DEFAULT_SBF_PARTITIONED_BOX_BUDGET_PER_TREE
    config.grower.enable_coordinated_multi_goal = DEFAULT_SBF_ENABLE_COORDINATED_MULTI_GOAL
    config.grower.endpoint_auto_bridge = True
    config.coarsen.target_boxes = DEFAULT_SBF_COARSEN_TARGET_BOXES
    config.coarsen.score_threshold = DEFAULT_SBF_COARSEN_SCORE_THRESHOLD
    config.enable_coarsen = DEFAULT_SBF_ENABLE_COARSEN
    config.lect_no_cache = bool(lect_no_cache)
    config.v6_cache_reads_enabled = bool(v6_cache_reads_enabled)
    if lect_cache_dir is not None:
        config.lect_cache_dir = str(lect_cache_dir)
    config.enable_seed_bridge = DEFAULT_SBF_ENABLE_SEED_BRIDGE
    config.enable_rescue_bridge = DEFAULT_SBF_ENABLE_RESCUE_BRIDGE
    config.force_full_bridge = False
    config.non_box_bridge.enable = False
    return config


def apply_exp3_sbf_build_variant(config, sbf6_module, *, endpoint_source: str = "critsample"):
    """Use the retained Exp.3 non-grid SBF build variant.

    Exp.3 identifies CritSample + LinkIAABB(S=4) as the fastest retained
    non-grid configuration in the same-route replay: CritSample supplies a
    tight endpoint set for the replay workload and S=4 is the non-grid
    subdivision knee. Exp.4/5 use this same variant when their SBF build times
    are compared against Exp.3.

    ``endpoint_source`` may be ``critsample`` (default) or ``ifk`` to match the
    second SBF column in Exp.~4 (IFK endpoints on the same LinkIAABB envelope).
    """
    endpoint_cfg = sbf6_module.EndpointSourceConfig()
    ep = (endpoint_source or "critsample").strip().lower()
    if ep == "ifk":
        endpoint_cfg.source = sbf6_module.EndpointSource.IFK
    elif ep in ("critsample", "crit"):
        endpoint_cfg.source = sbf6_module.EndpointSource.CritSample
    else:
        raise ValueError(f"unknown Exp.3 SBF endpoint_source: {endpoint_source!r} (use critsample or ifk)")
    config.endpoint_source = endpoint_cfg

    env_cfg = sbf6_module.EnvelopeTypeConfig()
    env_cfg.type = sbf6_module.EnvelopeType.LinkIAABB
    env_cfg.n_subdivisions = DEFAULT_SBF_ENVELOPE_SUBDIVISIONS
    config.envelope_type = env_cfg
    return config


def exp3_sbf_build_variant_summary():
    return {
        "endpoint_source": DEFAULT_SBF_ENDPOINT_SOURCE,
        "envelope_type": DEFAULT_SBF_ENVELOPE_TYPE,
        "envelope_n_subdivisions": DEFAULT_SBF_ENVELOPE_SUBDIVISIONS,
        "source": "Exp.3 retained non-grid certified hot-path variant",
    }


def paper_sbf_architecture_summary():
    return {
        "seed_points": list(DEFAULT_SBF_SEED_ORDER),
        "sbf_build_variant": exp3_sbf_build_variant_summary(),
        "grow_timeout_ms": DEFAULT_SBF_GROW_TIMEOUT_MS,
        "max_boxes": DEFAULT_SBF_MAX_BOXES,
        "post_connect_extra_boxes": DEFAULT_SBF_POST_CONNECT_EXTRA_BOXES,
        "n_threads": DEFAULT_SBF_THREADS,
        "bridge_n_threads": DEFAULT_SBF_BRIDGE_THREADS,
        "ffb_depth": DEFAULT_SBF_FFB_DEPTH,
        "goal_bias": DEFAULT_GOAL_BIAS,
        "rrt_step_ratio": DEFAULT_SBF_RRT_STEP_RATIO,
        "unexplored_sample_prob": DEFAULT_SBF_UNEXPLORED_SAMPLE_PROB,
        "max_consecutive_miss": DEFAULT_SBF_MAX_CONSECUTIVE_MISS,
        "coarsen_target_boxes": DEFAULT_SBF_COARSEN_TARGET_BOXES,
        "coarsen_score_threshold": DEFAULT_SBF_COARSEN_SCORE_THRESHOLD,
        "enable_partitioned_lect_parallel": DEFAULT_SBF_ENABLE_PARTITIONED_LECT_PARALLEL,
        "partitioned_box_budget_per_tree": DEFAULT_SBF_PARTITIONED_BOX_BUDGET_PER_TREE,
        "enable_coordinated_multi_goal": DEFAULT_SBF_ENABLE_COORDINATED_MULTI_GOAL,
        "enable_seed_bridge": DEFAULT_SBF_ENABLE_SEED_BRIDGE,
        "enable_rescue_bridge": DEFAULT_SBF_ENABLE_RESCUE_BRIDGE,
        "enable_coarsen": DEFAULT_SBF_ENABLE_COARSEN,
        "non_box_bridge": {
            "enable": False,
        },
        "prebridge_query_pairs": DEFAULT_SBF_PREBRIDGE_QUERY_PAIRS,
        "prebridge_per_pair_timeout_ms": DEFAULT_SBF_PREBRIDGE_PER_PAIR_TIMEOUT_MS,
        "prebridge_max_pairs_per_call": DEFAULT_SBF_PREBRIDGE_MAX_PAIRS_PER_CALL,
        "prebridge_max_query_pairs": DEFAULT_SBF_PREBRIDGE_MAX_QUERY_PAIRS,
    }

# ─── Scene definitions (Marcucci combined scene) ─────────────────────────

IIWA_CONFIGS = {
    "AS": np.array([6.42e-05, 0.4719533, -0.0001493, -0.6716735,
                    0.0001854, 0.4261696, 1.5706922]),
    "TS": np.array([-1.55e-04, 0.3972726, 0.0002196, -1.3674756,
                    0.0002472, -0.1929518, 1.5704688]),
    "CS": np.array([-1.76e-04, 0.6830279, 0.0002450, -1.6478229,
                    2.09e-05, -0.7590545, 1.5706263]),
    "LB": np.array([1.3326656, 0.7865932, 0.3623384, -1.4916529,
                    -0.3192509, 0.9217325, 1.7911904]),
    "RB": np.array([-1.3324624, 0.7866478, -0.3626562, -1.4916528,
                    0.3195340, 0.9217833, 1.3502090]),
}

QUERY_PAIRS = [
    ("AS->TS", "AS", "TS"),
    ("TS->CS", "TS", "CS"),
    ("CS->LB", "CS", "LB"),
    ("LB->RB", "LB", "RB"),
    ("RB->AS", "RB", "AS"),
]

IIWA_JOINT_LIMITS = [
    (-2.96706, 2.96706), (-2.09440, 2.09440),
    (-2.96706, 2.96706), (-2.09440, 2.09440),
    (-2.96706, 2.96706), (-2.09440, 2.09440),
    (-3.05433, 3.05433),
]

GCS_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "..", "..", "gcs-science-robotics"))
SBF_BUILD_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "build", "python"))
SBF_DATA_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "data"))


# ─── Drake plant builder ────────────────────────────────────────────────

def build_drake_plant():
    from pydrake.systems.framework import DiagramBuilder
    from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
    from pydrake.multibody.parsing import (
        Parser, LoadModelDirectives, ProcessModelDirectives)

    builder = DiagramBuilder()
    plant, sg = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant, sg)
    pm = parser.package_map()
    pm.Add("gcs", GCS_DIR)

    directives_file = os.path.join(
        GCS_DIR, "models", "iiwa14_spheres_collision_welded_gripper.yaml")
    directives = LoadModelDirectives(directives_file)
    ProcessModelDirectives(directives, plant, parser)
    plant.Finalize()
    diagram = builder.Build()
    return diagram, plant


# ─── Drake SceneGraphCollisionChecker (native C++ backend) ───────────────

def build_drake_scene_graph_checker(edge_step_size=0.05, parallelism=None):
    """Build a Drake SceneGraphCollisionChecker using RobotDiagramBuilder.

    Args:
        edge_step_size: step size for edge collision checking.
        parallelism: number of parallel contexts. None = max (default).
                     Set to 1 for single-threaded checking.

    Returns (checker, plant, diagram).
    """
    from pydrake.planning import (
        SceneGraphCollisionChecker, RobotDiagramBuilder)
    from pydrake.multibody.parsing import (
        LoadModelDirectives, ProcessModelDirectives)
    from pydrake.common import Parallelism

    builder = RobotDiagramBuilder(time_step=0.0)
    builder.parser().package_map().Add("gcs", GCS_DIR)
    directives_file = os.path.join(
        GCS_DIR, "models", "iiwa14_spheres_collision_welded_gripper.yaml")
    directives = LoadModelDirectives(directives_file)
    ProcessModelDirectives(directives, builder.parser())
    plant = builder.plant()
    iiwa_idx = plant.GetModelInstanceByName("iiwa")
    wsg_idx = plant.GetModelInstanceByName("wsg")
    plant.Finalize()
    diagram = builder.Build()

    kwargs = dict(
        model=diagram,
        robot_model_instances=[iiwa_idx, wsg_idx],
        edge_step_size=edge_step_size,
    )
    if parallelism is not None:
        kwargs["implicit_context_parallelism"] = Parallelism(parallelism)

    checker = SceneGraphCollisionChecker(**kwargs)
    return checker, plant, diagram


# ─── Drake collision checker (wrapper for RRT-Connect) ────────────────────

class DrakeCollisionChecker:
    def __init__(self, diagram, plant):
        self._diagram = diagram
        self._plant = plant
        self._context = diagram.CreateDefaultContext()
        self._plant_context = plant.GetMyContextFromRoot(self._context)
        self.n_checks = 0

    def check_config(self, q):
        self.n_checks += 1
        self._plant.SetPositions(self._plant_context, q)
        qo = self._plant.get_geometry_query_input_port().Eval(
            self._plant_context)
        return qo.HasCollisions()

    def check_segment(self, q_from, q_to, resolution=0.05):
        diff = q_to - q_from
        dist = np.linalg.norm(diff)
        if dist < 1e-10:
            return not self.check_config(q_from)
        n_steps = max(2, int(np.ceil(dist / resolution)))
        for i in range(n_steps + 1):
            q = q_from + (i / n_steps) * diff
            if self.check_config(q):
                return False
        return True

    def reset_count(self):
        self.n_checks = 0


# ─── Path utilities ──────────────────────────────────────────────────────

def path_length(waypoints):
    if len(waypoints) < 2:
        return 0.0
    return sum(np.linalg.norm(waypoints[i] - waypoints[i - 1])
               for i in range(1, len(waypoints)))


def shortcut_path(waypoints, checker, max_iters=300, rng=None):
    if rng is None:
        rng = np.random.default_rng()
    path = list(waypoints)
    for _ in range(max_iters):
        if len(path) <= 2:
            break
        i = rng.integers(0, len(path) - 2)
        j = rng.integers(i + 2, len(path))
        if checker.check_segment(path[i], path[j]):
            path = path[:i + 1] + path[j:]
    return path


# ─── SBF Obstacles (Marcucci combined scene) ────────────────────────────

def make_combined_obstacles():
    if SBF_BUILD_DIR not in sys.path:
        sys.path.insert(0, SBF_BUILD_DIR)
    import _sbf6_cpp as sbf6

    def make_shelves():
        ox, oy, oz = 0.85, 0.0, 0.4
        obs = []
        def add(lx, ly, lz, fx, fy, fz):
            obs.append(sbf6.Obstacle(
                ox+lx-fx/2, oy+ly-fy/2, oz+lz-fz/2,
                ox+lx+fx/2, oy+ly+fy/2, oz+lz+fz/2))
        add(0, 0.292, 0, 0.3, 0.016, 0.783)
        add(0, -0.292, 0, 0.3, 0.016, 0.783)
        add(0, 0, 0.3995, 0.3, 0.6, 0.016)
        add(0, 0, -0.13115, 0.3, 0.6, 0.016)
        add(0, 0, 0.13115, 0.3, 0.6, 0.016)
        return obs

    def make_bins():
        obs = []
        def add_bin(bx, by, bz):
            def add(lx, ly, lz, fx, fy, fz):
                obs.append(sbf6.Obstacle(
                    bx-ly-fy/2, by+lx-fx/2, bz+lz-fz/2,
                    bx-ly+fy/2, by+lx+fx/2, bz+lz+fz/2))
            add(0.22, 0, 0.105, 0.05, 0.63, 0.21)
            add(-0.22, 0, 0.105, 0.05, 0.63, 0.21)
            add(0, 0.29, 0.105, 0.49, 0.05, 0.21)
            add(0, -0.29, 0.105, 0.49, 0.05, 0.21)
            add(0, 0, 0.0075, 0.49, 0.63, 0.015)
        add_bin(0, -0.6, 0)
        add_bin(0, 0.6, 0)
        return obs

    def make_table():
        return [sbf6.Obstacle(0.4-2.5/2, -2.5/2, -0.25-0.2/2,
                               0.4+2.5/2, 2.5/2, -0.25+0.2/2)]

    return make_shelves() + make_bins() + make_table()


# ─── IRIS-NP region generation ───────────────────────────────────────────

def generate_iris_regions(plant, diagram, seed_configs, max_iters=10,
                          cache_file=None):
    from pydrake.geometry.optimization import (
        IrisNp, IrisOptions, HPolyhedron)

    if cache_file and os.path.exists(cache_file):
        logger.info(f"  Loading IRIS regions from cache: {cache_file}")
        cached = np.load(cache_file, allow_pickle=True)
        regions = []
        timings = list(cached["timings"])
        n = int(cached["n_regions"])
        for i in range(n):
            regions.append(HPolyhedron(cached[f"A_{i}"], cached[f"b_{i}"]))
        logger.info(f"  Loaded {n} cached regions")
        return regions, timings

    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)
    opts = IrisOptions()
    opts.iteration_limit = max_iters
    opts.termination_threshold = -1
    opts.relative_termination_threshold = 2e-2
    opts.num_collision_infeasible_samples = 1
    opts.random_seed = 0
    opts.require_sample_point_is_contained = True

    regions, timings = [], []
    for i, (name, seed) in enumerate(seed_configs):
        logger.info(f"  IRIS-NP region {i+1}/{len(seed_configs)} "
                     f"(seed={name})...")
        plant.SetPositions(plant_context, seed)
        t0 = time.perf_counter()
        try:
            region = IrisNp(plant, plant_context, opts)
            dt = time.perf_counter() - t0
            regions.append(region)
            timings.append(dt)
            logger.info(f"    Done in {dt:.1f}s, "
                        f"{region.A().shape[0]} halfplanes")
        except Exception as e:
            dt = time.perf_counter() - t0
            timings.append(dt)
            logger.warning(f"    FAILED after {dt:.1f}s: {e}")

    if cache_file:
        save_dict = {"n_regions": len(regions), "timings": np.array(timings)}
        for i, r in enumerate(regions):
            save_dict[f"A_{i}"] = r.A()
            save_dict[f"b_{i}"] = r.b()
        np.savez(cache_file, **save_dict)
        logger.info(f"  Saved {len(regions)} regions to {cache_file}")

    return regions, timings


# ─── GCS query (from gcs-science-robotics reference) ────────────────────

def run_gcs_query(q_start, q_goal, iris_regions, seed=42):
    """Run one GCS query using LinearGCS from the reference implementation."""
    if GCS_DIR not in sys.path:
        sys.path.insert(0, GCS_DIR)
    from gcs.linear import LinearGCS
    from gcs.rounding import randomForwardPathSearch
    from pydrake.solvers import MosekSolver, SolverOptions

    t0 = time.perf_counter()
    n = len(iris_regions)
    if n == 0:
        return {"success": False, "time_s": 0, "path_length": float("nan"),
                "regions": 0}

    gcs = LinearGCS(iris_regions)
    try:
        gcs.addSourceTarget(q_start, q_goal)
    except ValueError as e:
        dt = time.perf_counter() - t0
        return {"success": False, "time_s": dt, "path_length": float("nan"),
                "regions": n, "note": f"addSourceTarget failed: {e}"}

    gcs.setRoundingStrategy(randomForwardPathSearch,
                            max_paths=10, max_trials=100, seed=seed)
    gcs.setSolver(MosekSolver())
    solver_options = SolverOptions()
    solver_options.SetOption(MosekSolver.id(),
                            "MSK_DPAR_INTPNT_CO_TOL_REL_GAP", 1e-3)
    solver_options.SetOption(MosekSolver.id(),
                            "MSK_DPAR_INTPNT_TOL_PFEAS", 1e-3)
    solver_options.SetOption(MosekSolver.id(),
                            "MSK_DPAR_INTPNT_TOL_DFEAS", 1e-3)
    solver_options.SetOption(MosekSolver.id(),
                            "MSK_DPAR_INTPNT_TOL_INFEAS", 1e-3)
    gcs.setSolverOptions(solver_options)

    waypoints, results_dict = gcs.SolvePath(
        rounding=True, verbose=False, preprocessing=True)
    dt_wall = time.perf_counter() - t0

    if waypoints is None:
        return {"success": False, "time_s": dt_wall,
                "time_wall_s": dt_wall,
                "path_length": float("nan"),
                "regions": n, "note": "GCS solve failed"}

    # Use Mosek solver-internal time (matching Marcucci methodology):
    # relaxation_solver_time + max_rounded_solver_time
    # (rounding solves are independent and can be parallelized, so take max)
    solver_time = results_dict.get("relaxation_solver_time", 0.0)
    solver_time += results_dict.get("max_rounded_solver_time", 0.0)

    path = waypoints.T
    pl = float(sum(np.linalg.norm(path[i] - path[i-1])
                   for i in range(1, len(path))))
    return {
        "success": True,
        "time_s": solver_time,          # solver-internal (fair comparison)
        "time_wall_s": dt_wall,          # wall-clock (for reference)
        "path_length": pl,
        "regions": n, "edges": len(list(gcs.gcs.Edges())),
        "waypoints_count": path.shape[0],
        "relaxation_solver_time": results_dict.get("relaxation_solver_time"),
        "max_rounded_solver_time": results_dict.get("max_rounded_solver_time"),
        "total_rounded_solver_time": results_dict.get("total_rounded_solver_time"),
    }


# ─── RRT-Connect ────────────────────────────────────────────────────────

def run_rrt_connect(q_start, q_goal, checker, *,
                    timeout=60.0, step_size=0.3, seed=42):
    ndim = len(q_start)
    lows = np.array([lo for lo, _ in IIWA_JOINT_LIMITS])
    highs = np.array([hi for _, hi in IIWA_JOINT_LIMITS])
    rng = np.random.default_rng(seed)

    if checker.check_config(q_start) or checker.check_config(q_goal):
        return {"success": False, "time_s": 0, "path_length": float("nan")}

    configs_a, parents_a = [q_start.copy()], [-1]
    configs_b, parents_b = [q_goal.copy()], [-1]
    checker.reset_count()
    t0 = time.perf_counter()

    def nearest(configs, q):
        return int(np.argmin([np.sum((c - q) ** 2) for c in configs]))

    def steer(q_from, q_to):
        diff = q_to - q_from
        d = np.linalg.norm(diff)
        return q_to.copy() if d <= step_size else q_from + (step_size / d) * diff

    def extend(configs, parents, q_target):
        idx = nearest(configs, q_target)
        q_new = steer(configs[idx], q_target)
        if checker.check_segment(configs[idx], q_new, resolution=0.05):
            configs.append(q_new)
            parents.append(idx)
            return len(configs) - 1, q_new
        return None, None

    def connect(configs, parents, q_target):
        while True:
            idx, q_new = extend(configs, parents, q_target)
            if idx is None:
                return None
            if np.linalg.norm(q_new - q_target) < 1e-6:
                return idx

    def extract(configs, parents, idx):
        path = []
        while idx >= 0:
            path.append(configs[idx])
            idx = parents[idx]
        path.reverse()
        return path

    swapped = False
    while time.perf_counter() - t0 < timeout:
        q_rand = rng.uniform(lows, highs)
        idx_a, q_new_a = extend(configs_a, parents_a, q_rand)
        if idx_a is None:
            continue
        idx_b = connect(configs_b, parents_b, q_new_a)
        if idx_b is not None:
            dt = time.perf_counter() - t0
            pa = extract(configs_a, parents_a, idx_a)
            pb = extract(configs_b, parents_b, idx_b)
            pb.reverse()
            if swapped:
                pa, pb = pb, pa
                pa.reverse()
                pb.reverse()
            full = pa + pb[1:]
            full_sc = shortcut_path(full, checker, max_iters=200, rng=rng)
            return {
                "success": True, "time_s": dt,
                "path_length": path_length(full_sc),
                "path_length_raw": path_length(full),
                "raw_path": full,
                "nodes": len(configs_a) + len(configs_b),
                "collision_checks": checker.n_checks,
            }
        configs_a, configs_b = configs_b, configs_a
        parents_a, parents_b = parents_b, parents_a
        swapped = not swapped

    dt = time.perf_counter() - t0
    return {"success": False, "time_s": dt, "path_length": float("nan"),
            "nodes": len(configs_a) + len(configs_b),
            "collision_checks": checker.n_checks}


# ─── Drake-native PRM (matches Marcucci Science Robotics methodology) ───

def _resample_path(waypoints, interval=0.2):
    """Resample a path so consecutive points are at most `interval` apart.
    Matches PathProcessorParameters.resampled_state_interval."""
    resampled = [waypoints[0].copy()]
    for i in range(1, len(waypoints)):
        diff = waypoints[i] - waypoints[i - 1]
        d = np.linalg.norm(diff)
        if d <= 1e-12:
            continue
        n_seg = max(1, int(np.ceil(d / interval)))
        for k in range(1, n_seg):
            resampled.append(waypoints[i - 1] + diff * (k / n_seg))
        resampled.append(waypoints[i].copy())
    return resampled

class DrakePRM:
    """Lazy PRM using Drake SceneGraphCollisionChecker.
    Mirrors the Marcucci et al. PRM comparison methodology:
    roadmap_size=10000, k=5 neighbors, edges NOT validated during build,
    lazy query (edges validated one-by-one during Dijkstra search)
    + shortcut smoothing with resampling.
    Query time = TimedPlanLazy + TimedProcessPath (Marcucci convention).
    """

    def __init__(self, sg_checker, *, n_samples=10000, k_neighbors=5,
                 connection_radius=5.0, max_valid_sample_tries=10, seed=42):
        self.checker = sg_checker          # SceneGraphCollisionChecker
        self.n_samples = n_samples
        self.k_neighbors = k_neighbors
        self.connection_radius = connection_radius
        self.max_valid_sample_tries = max_valid_sample_tries
        self.seed = seed
        self.nodes = None
        self.adjacency = None              # {i: [(j, dist), ...]}
        self.build_time = 0.0
        self.n_edges = 0
        self.seed_paths = []               # BiRRT/RRT seed waypoints
        self._seed_path_lengths = []       # length of each seed path

    def add_seed_paths(self, waypoint_arrays):
        """Add pre-computed seed paths (e.g. from BiRRT) to be included
        as nodes in the roadmap.  Each element is a list of configs."""
        for wp in waypoint_arrays:
            self._seed_path_lengths.append(len(wp))
            self.seed_paths.extend(wp)

    def build(self, timeout=180.0):
        """Lazy PRM build: sample collision-free configs, build k-NN
        structure, but do NOT validate edges.  Edge validation happens
        lazily during query(), matching Drake PRMPlanner.TimedBuildRoadmap
        / TimedPlanLazy methodology from Marcucci et al."""
        from collections import defaultdict

        lows = np.array([lo for lo, _ in IIWA_JOINT_LIMITS])
        highs = np.array([hi for _, hi in IIWA_JOINT_LIMITS])
        rng = np.random.default_rng(self.seed)
        t0 = time.perf_counter()

        # ── Phase 0: Include seed-path nodes (verified collision-free) ──
        nodes_list = []
        seed_path_ranges = []      # [(start_idx, end_idx), ...] per path
        if self.seed_paths:
            seed_configs = [np.asarray(c, dtype=np.float64)
                            for c in self.seed_paths]
            results = self.checker.CheckConfigsCollisionFree(seed_configs)
            for q, ok in zip(seed_configs, results):
                if ok:
                    nodes_list.append(q)
            logger.info(f"    Seed path nodes: {len(nodes_list)} / "
                        f"{len(seed_configs)} collision-free")

        # Record per-path index ranges for sequential edge insertion
        if self._seed_path_lengths:
            idx = 0
            for plen in self._seed_path_lengths:
                seed_path_ranges.append((idx, idx + plen))
                idx += plen

        # ── Phase 1: Sample collision-free configurations (batch) ──
        batch_sz = 1000
        attempts = 0
        while len(nodes_list) < self.n_samples:
            if time.perf_counter() - t0 > timeout * 0.4:
                break
            batch = [rng.uniform(lows, highs) for _ in range(batch_sz)]
            results = self.checker.CheckConfigsCollisionFree(batch)
            for q, ok in zip(batch, results):
                if ok:
                    nodes_list.append(q)
                    if len(nodes_list) >= self.n_samples:
                        break
            attempts += batch_sz

        self.nodes = np.array(nodes_list, dtype=np.float64)
        n = len(self.nodes)

        # ── Phase 2: Build k-NN edge candidates (UNVALIDATED) ──
        # Edges are stored as candidates; collision checking deferred to
        # query time (lazy), matching Drake PRMPlanner.TimedPlanLazy.
        self.adjacency = defaultdict(list)
        edge_set = set()
        for i in range(n):
            dists = np.linalg.norm(self.nodes - self.nodes[i], axis=1)
            dists[i] = float('inf')
            nearest_k = np.argsort(dists)[:self.k_neighbors]
            for j in nearest_k:
                j = int(j)
                d = float(dists[j])
                if d > self.connection_radius:
                    continue
                key = (min(i, j), max(i, j))
                if key in edge_set:
                    continue
                edge_set.add(key)
                self.adjacency[i].append((j, d))
                self.adjacency[j].append((i, d))

        self.n_edges = len(edge_set)

        # ── Phase 2b: Add sequential edges along seed paths ──
        # These provide connectivity through narrow passages.
        n_seed_edges = 0
        for (start_idx, end_idx) in seed_path_ranges:
            for k in range(start_idx, end_idx - 1):
                key = (min(k, k + 1), max(k, k + 1))
                if key not in edge_set:
                    d = float(np.linalg.norm(
                        self.nodes[k] - self.nodes[k + 1]))
                    edge_set.add(key)
                    self.adjacency[k].append((k + 1, d))
                    self.adjacency[k + 1].append((k, d))
                    n_seed_edges += 1
        self.n_edges += n_seed_edges

        self.build_time = time.perf_counter() - t0
        logger.info(f"  DrakePRM (lazy): {n} nodes, {self.n_edges} edges "
                    f"({n_seed_edges} seed-path), "
                    f"build={self.build_time:.2f}s "
                    f"(sampled from {attempts} attempts)")

    def query(self, q_start, q_goal, timeout=30.0):
        """Lazy PRM query: Dijkstra with per-edge collision checking,
        matching Drake PRMPlanner.TimedPlanLazy.  Each edge is validated
        only when first explored; invalid edges are removed and search
        retries."""
        import heapq
        t0 = time.perf_counter()
        n = len(self.nodes)

        # Connect start/goal to nearest nodes
        def _find_nearest(q, k_mult=4):
            dists = np.linalg.norm(self.nodes - q, axis=1)
            order = np.argsort(dists)
            conns = []
            for idx in order[:self.k_neighbors * k_mult]:
                idx = int(idx)
                if dists[idx] > self.connection_radius * 2:
                    break
                conns.append((idx, float(dists[idx])))
            return conns

        start_conns = _find_nearest(q_start)
        goal_conns = _find_nearest(q_goal)
        if not start_conns or not goal_conns:
            return {"success": False, "time_s": time.perf_counter() - t0,
                    "path_length": float("nan")}

        START_V, GOAL_V = n, n + 1
        goal_set = {idx for idx, _ in goal_conns}
        # Track edges known to be invalid (lazy collision check failures)
        invalid_edges = set()

        for attempt in range(500):
            if time.perf_counter() - t0 > timeout:
                break

            # Dijkstra with lazy edge validation
            dist_to, prev, pq = {}, {}, []
            for idx, d in start_conns:
                ekey = (START_V, idx)
                if ekey not in invalid_edges:
                    if d < dist_to.get(idx, float('inf')):
                        dist_to[idx] = d
                        prev[idx] = START_V
                        heapq.heappush(pq, (d, idx))

            found_goal_node = None
            while pq:
                d, u = heapq.heappop(pq)
                if d > dist_to.get(u, float('inf')):
                    continue
                if u in goal_set:
                    found_goal_node = u
                    break
                for v, w in self.adjacency.get(u, []):
                    key = (min(u, v), max(u, v))
                    if key in invalid_edges:
                        continue
                    nd = d + w
                    if nd < dist_to.get(v, float('inf')):
                        dist_to[v] = nd
                        prev[v] = u
                        heapq.heappush(pq, (nd, v))

            if found_goal_node is None:
                break

            # Extract candidate path
            path_indices = [found_goal_node]
            idx = found_goal_node
            while prev.get(idx) != START_V:
                idx = prev[idx]
                path_indices.append(idx)
            path_indices.reverse()

            # Lazily validate every edge on the path
            path_valid = True

            # start → first node
            if not self.checker.CheckEdgeCollisionFree(
                    q_start, self.nodes[path_indices[0]]):
                invalid_edges.add((START_V, path_indices[0]))
                continue

            # roadmap edges (lazy — checked now, not at build time)
            for ei in range(len(path_indices) - 1):
                u, v = path_indices[ei], path_indices[ei + 1]
                key = (min(u, v), max(u, v))
                if not self.checker.CheckEdgeCollisionFree(
                        self.nodes[u], self.nodes[v]):
                    invalid_edges.add(key)
                    path_valid = False
                    break  # retry Dijkstra with this edge removed

            if not path_valid:
                continue

            # last node → goal
            if not self.checker.CheckEdgeCollisionFree(
                    self.nodes[found_goal_node], q_goal):
                invalid_edges.add((found_goal_node, GOAL_V))
                goal_conns = [(idx, d) for idx, d in goal_conns
                              if idx != found_goal_node]
                goal_set = {idx for idx, _ in goal_conns}
                continue

            # All edges valid — build waypoint path
            path = [q_start.copy()]
            for pi in path_indices:
                path.append(self.nodes[pi].copy())
            path.append(q_goal.copy())

            # Shortcut smoothing (included in query time,
            # matching Marcucci's "Shortcut PRM" = TimedPlanLazy +
            # TimedProcessPath).
            # Resample path at 0.2 rad intervals before smoothing,
            # matching PathProcessorParameters.resampled_state_interval.
            path_resampled = _resample_path(path, interval=0.2)
            path_sc = self._shortcut(path_resampled, max_iters=200,
                                     rng=np.random.default_rng(42))
            dt = time.perf_counter() - t0
            return {
                "success": True, "time_s": dt,
                "path_length": path_length(path_sc),
                "path_length_raw": path_length(path),
            }

        return {"success": False, "time_s": time.perf_counter() - t0,
                "path_length": float("nan")}

    def _shortcut(self, waypoints, max_iters=200, rng=None):
        """Shortcut smoothing using Drake edge checker."""
        if rng is None:
            rng = np.random.default_rng()
        path = list(waypoints)
        for _ in range(max_iters):
            if len(path) <= 2:
                break
            i = rng.integers(0, len(path) - 2)
            j = rng.integers(i + 2, len(path))
            if self.checker.CheckEdgeCollisionFree(path[i], path[j]):
                path = path[:i + 1] + path[j:]
        return path


# ─── SBF planner ────────────────────────────────────────────────────────

def run_sbf_experiment(
    n_seeds=5,
    *,
    grow_timeout_ms=DEFAULT_SBF_GROW_TIMEOUT_MS,
    max_boxes=DEFAULT_SBF_MAX_BOXES,
    post_connect_extra_boxes=DEFAULT_SBF_POST_CONNECT_EXTRA_BOXES,
    n_threads=DEFAULT_SBF_THREADS,
    bridge_n_threads=DEFAULT_SBF_BRIDGE_THREADS,
    ffb_depth=DEFAULT_SBF_FFB_DEPTH,
    goal_bias=DEFAULT_GOAL_BIAS,
    unexplored_sample_prob=DEFAULT_SBF_UNEXPLORED_SAMPLE_PROB,
    max_consecutive_miss=DEFAULT_SBF_MAX_CONSECUTIVE_MISS,
    enable_partitioned_lect_parallel=DEFAULT_SBF_ENABLE_PARTITIONED_LECT_PARALLEL,
    partitioned_box_budget_per_tree=DEFAULT_SBF_PARTITIONED_BOX_BUDGET_PER_TREE,
    enable_coordinated_multi_goal=DEFAULT_SBF_ENABLE_COORDINATED_MULTI_GOAL,
    prebridge_query_pairs=DEFAULT_SBF_PREBRIDGE_QUERY_PAIRS,
    prebridge_per_pair_timeout_ms=DEFAULT_SBF_PREBRIDGE_PER_PAIR_TIMEOUT_MS,
    prebridge_max_pairs_per_call=DEFAULT_SBF_PREBRIDGE_MAX_PAIRS_PER_CALL,
    prebridge_max_query_pairs=DEFAULT_SBF_PREBRIDGE_MAX_QUERY_PAIRS,
):
    """Run SBF build + per-query measurements."""
    if SBF_BUILD_DIR not in sys.path:
        sys.path.insert(0, SBF_BUILD_DIR)
    import _sbf6_cpp as sbf6

    robot = sbf6.Robot.from_json(os.path.join(SBF_DATA_DIR, "iiwa14.json"))
    obstacles = make_combined_obstacles()

    build_results = []
    query_results = {label: [] for label, _, _ in QUERY_PAIRS}

    for s in range(n_seeds):
        logger.info(f"  SBF seed {s}...")
        config = sbf6.SBFPlannerConfig()
        apply_paper_sbf_architecture(
            config,
            seed=s,
            grow_timeout_ms=float(grow_timeout_ms),
            max_boxes=int(max_boxes),
            post_connect_extra_boxes=int(post_connect_extra_boxes),
            n_threads=int(n_threads),
            bridge_n_threads=int(bridge_n_threads),
            ffb_depth=int(ffb_depth),
            goal_bias=float(goal_bias),
            lect_no_cache=True,
        )
        apply_exp3_sbf_build_variant(config, sbf6)
        config.grower.unexplored_sample_prob = float(unexplored_sample_prob)
        config.grower.max_consecutive_miss = int(max_consecutive_miss)
        config.grower.enable_partitioned_lect_parallel = bool(enable_partitioned_lect_parallel)
        config.grower.partitioned_box_budget_per_tree = int(partitioned_box_budget_per_tree)
        config.grower.enable_coordinated_multi_goal = bool(enable_coordinated_multi_goal)
        planner = sbf6.SBFPlanner(robot, config)

        # Build with multi-goal coverage (all 5 configs as seeds)
        seed_points = [IIWA_CONFIGS[k] for k in DEFAULT_SBF_SEED_ORDER]
        t0 = time.perf_counter()
        planner.build_coverage(obstacles, float(grow_timeout_ms), seed_points)
        prebridge_time = 0.0
        prebridge_added = 0
        if prebridge_query_pairs:
            query_pairs = [
                (IIWA_CONFIGS[start_name], IIWA_CONFIGS[goal_name])
                for _, start_name, goal_name in QUERY_PAIRS
            ]
            query_pairs = query_pairs[: max(0, int(prebridge_max_query_pairs))]
            t_pre = time.perf_counter()
            prebridge_added = int(planner.pre_bridge_pairs(
                query_pairs,
                obstacles,
                float(prebridge_per_pair_timeout_ms),
                int(prebridge_max_pairs_per_call),
            ))
            prebridge_time = time.perf_counter() - t_pre
        build_time = time.perf_counter() - t0
        n_boxes = planner.n_boxes()
        box_stats = dedup_box_volume_stats(planner.boxes())
        build_results.append({
            "seed": s,
            "build_time_s": build_time,
            "prebridge_time_s": prebridge_time,
            "prebridge_added_boxes": prebridge_added,
            "n_boxes": n_boxes,
            **box_stats,
        })
        logger.info(
            f"    Build: {build_time:.3f}s, {n_boxes} boxes, "
            f"prebridge={prebridge_time:.3f}s/+{prebridge_added}, "
            f"dedup_boxes={box_stats['unique_box_count']}, "
            f"dedup_vol={box_stats['dedup_box_volume_sum']:.6f}")

        # Query each pair
        for label, start_name, goal_name in QUERY_PAIRS:
            q_s = IIWA_CONFIGS[start_name]
            q_g = IIWA_CONFIGS[goal_name]
            t0 = time.perf_counter()
            result = planner.query(q_s, q_g)
            query_time = time.perf_counter() - t0
            query_results[label].append({
                "seed": s, "success": result.success,
                "time_s": query_time,
                "path_length": float(result.path_length) if result.success else float("nan"),
                "planning_time_ms": float(result.planning_time_ms),
            })
            status = "OK" if result.success else "FAIL"
            logger.info(f"    Query {label}: {status}, "
                        f"{query_time:.3f}s, "
                        f"len={result.path_length:.3f}" if result.success else "")

    return build_results, query_results


# ─── Main experiment ─────────────────────────────────────────────────────

def run_experiment(n_seeds=5, output_json=None, use_iris_cache=True):
    logger.info("=" * 70)
    logger.info("Online Query Performance & Baseline Comparison")
    logger.info(f"  Scene: Marcucci combined (IIWA14, 16 obstacles)")
    logger.info(f"  Seeds: {n_seeds}, Query pairs: {len(QUERY_PAIRS)}")
    logger.info("=" * 70)

    all_results = {
        "meta": {
            "scene": "combined",
            "robot": "kuka_iiwa14",
            "n_seeds": n_seeds,
            "query_pairs": [l for l, _, _ in QUERY_PAIRS],
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        },
    }

    # ── 1. SBF ──
    logger.info("\n[1/4] SBF (build + query)...")
    sbf_build, sbf_queries = run_sbf_experiment(n_seeds)
    all_results["sbf"] = {
        "build": sbf_build,
        "queries": sbf_queries,
    }

    # Summary stats
    build_times = [b["build_time_s"] for b in sbf_build]
    boxes_list = [b["n_boxes"] for b in sbf_build]
    sbf_unique_boxes = [b.get("unique_box_count", b["n_boxes"]) for b in sbf_build]
    sbf_dedup_volumes = [b.get("dedup_box_volume_sum", 0.0) for b in sbf_build]
    sbf_total_volumes = [b.get("box_volume_sum", 0.0) for b in sbf_build]
    logger.info(f"  SBF build: median={np.median(build_times):.3f}s, "
                f"mean={np.mean(build_times):.3f}s, "
                f"boxes={int(np.median(boxes_list))}, "
                f"dedup_boxes={int(np.median(sbf_unique_boxes))}, "
                f"dedup_vol={np.median(sbf_dedup_volumes):.6f}")

    # ── 2. Drake plant + IRIS-NP + GCS ──
    logger.info("\n[2/4] IRIS-NP + GCS (reference: gcs-science-robotics)...")
    diagram, plant = build_drake_plant()
    checker = DrakeCollisionChecker(diagram, plant)

    # Build SceneGraphCollisionChecker for PRM (single-threaded, Marcucci methodology)
    # - parallelism=1: single context (Marcucci uses ChangeOmpNumThreadsWrapper(1))
    # - env_collision_padding=0.01: match Marcucci's VoxelizedEnvironmentCollisionChecker
    sg_checker, _, _ = build_drake_scene_graph_checker(
        edge_step_size=0.05, parallelism=1)
    logger.info(f"  Drake SceneGraphCollisionChecker (PRM): "
                f"parallel={sg_checker.SupportsParallelChecking()}, "
                f"contexts={sg_checker.num_allocated_contexts()}")

    # Verify configs
    for name, cfg in IIWA_CONFIGS.items():
        in_col = checker.check_config(cfg)
        if in_col:
            logger.warning(f"  Config {name}: IN COLLISION!")

    # IRIS-NP seeds: 5 canonical + collision-free midpoints
    iris_seeds = [(n, IIWA_CONFIGS[n]) for n in ["AS", "TS", "CS", "LB", "RB"]]
    for label, s_name, g_name in QUERY_PAIRS:
        mid = (IIWA_CONFIGS[s_name] + IIWA_CONFIGS[g_name]) / 2.0
        if not checker.check_config(mid):
            iris_seeds.append((f"mid_{label}", mid))

    iris_cache = None
    if use_iris_cache:
        iris_cache = os.path.join(os.path.dirname(__file__),
                                  "..", "experiments", "results_new",
                                  "iris_regions_cache.npz")

    iris_regions, iris_timings = generate_iris_regions(
        plant, diagram, iris_seeds, max_iters=10, cache_file=iris_cache)
    iris_total_s = sum(iris_timings)
    logger.info(f"  IRIS-NP: {len(iris_regions)} regions, "
                f"total={iris_total_s:.1f}s")

    # GCS queries
    gcs_queries = {}
    for label, start_name, goal_name in QUERY_PAIRS:
        q_s = IIWA_CONFIGS[start_name]
        q_g = IIWA_CONFIGS[goal_name]
        res = run_gcs_query(q_s, q_g, iris_regions)
        gcs_queries[label] = res
        status = "OK" if res["success"] else "FAIL"
        logger.info(f"  GCS {label}: {status}, "
                    f"solver_t={res['time_s']:.3f}s, "
                    f"wall_t={res.get('time_wall_s', float('nan')):.3f}s, "
                    f"len={res.get('path_length', float('nan')):.3f}")

    all_results["iris_gcs"] = {
        "iris_time_total_s": iris_total_s,
        "n_regions": len(iris_regions),
        "iris_per_region_s": [float(t) for t in iris_timings],
        "queries": gcs_queries,
    }

    # ── 3. RRT-Connect ──
    logger.info(f"\n[3/4] RRT-Connect ({n_seeds} seeds)...")
    rrt_queries = {label: [] for label, _, _ in QUERY_PAIRS}
    for label, start_name, goal_name in QUERY_PAIRS:
        q_s = IIWA_CONFIGS[start_name]
        q_g = IIWA_CONFIGS[goal_name]
        for s in range(n_seeds):
            checker.reset_count()
            res = run_rrt_connect(q_s, q_g, checker,
                                  timeout=60.0, step_size=0.3,
                                  seed=s * 1000 + 42)
            rrt_queries[label].append(res)
            status = "OK" if res["success"] else "FAIL"
            logger.info(f"  RRT {label} s={s}: {status}, "
                        f"t={res['time_s']:.3f}s, "
                        f"len={res.get('path_length', float('nan')):.3f}")
    all_results["rrt_connect"] = rrt_queries

    # ── 4. PRM (Drake native collision checking, Marcucci methodology) ──
    logger.info(f"\n[4/4] PRM (Drake SceneGraphCollisionChecker, "
                f"10000 nodes k=5, {n_seeds} seeds)...")

    # Generate BiRRT seed paths between all milestone pairs
    # (matches Marcucci methodology: BiRRT connects all seed configs,
    #  those paths are added as seed nodes to the PRM)
    from itertools import combinations
    seed_configs_list = list(IIWA_CONFIGS.values())
    birrt_seed_paths = []
    logger.info("  Generating RRT seed paths for PRM...")
    for qi, qj in combinations(seed_configs_list, 2):
        checker.reset_count()
        res = run_rrt_connect(qi, qj, checker,
                              timeout=30.0, step_size=0.3, seed=7777)
        if res["success"] and "raw_path" in res:
            # Use raw RRT waypoints (≤0.3 rad apart) so that every
            # consecutive edge passes Drake's 0.05-resolution check.
            birrt_seed_paths.append(res["raw_path"])
    logger.info(f"  RRT seed paths: {len(birrt_seed_paths)} paths, "
                f"{sum(len(p) for p in birrt_seed_paths)} total waypoints")

    prm_queries = {label: [] for label, _, _ in QUERY_PAIRS}
    prm_build_times = []
    for s in range(n_seeds):
        logger.info(f"  Building DrakePRM roadmap seed={s}...")
        rm = DrakePRM(sg_checker, n_samples=10000, k_neighbors=5,
                      connection_radius=5.0, seed=s * 1000 + 42)
        rm.add_seed_paths(birrt_seed_paths)
        rm.build(timeout=180.0)
        prm_build_times.append(rm.build_time)

        for label, start_name, goal_name in QUERY_PAIRS:
            q_s = IIWA_CONFIGS[start_name]
            q_g = IIWA_CONFIGS[goal_name]
            res = rm.query(q_s, q_g)
            prm_queries[label].append(res)
            status = "OK" if res["success"] else "FAIL"
            logger.info(f"  PRM {label} s={s}: {status}, "
                        f"t={res['time_s']:.3f}s, "
                        f"len={res.get('path_length', float('nan')):.3f}")

    all_results["prm"] = {
        "build_times_s": prm_build_times,
        "queries": prm_queries,
    }

    # ── Summary statistics ──
    logger.info("\n" + "=" * 70)
    logger.info("SUMMARY")
    logger.info("=" * 70)

    def _safe_median(vals):
        return float(np.median(vals)) if vals else float("nan")

    # SBF summary
    sbf_q_times = []
    sbf_q_lengths = []
    sbf_successes = 0
    sbf_total_queries = 0
    for label in sbf_queries:
        for r in sbf_queries[label]:
            sbf_total_queries += 1
            if r["success"]:
                sbf_successes += 1
                sbf_q_times.append(r["time_s"])
                sbf_q_lengths.append(r["path_length"])
    sbf_sr = sbf_successes / max(1, sbf_total_queries) * 100
    sbf_build_med = _safe_median(build_times)
    sbf_q_med = _safe_median(sbf_q_times)
    sbf_l_med = _safe_median(sbf_q_lengths)

    # GCS summary
    gcs_q_times = [gcs_queries[l]["time_s"] for l in gcs_queries
                   if gcs_queries[l]["success"]]
    gcs_q_lengths = [gcs_queries[l]["path_length"] for l in gcs_queries
                     if gcs_queries[l]["success"]]
    gcs_sr = sum(1 for l in gcs_queries
                 if gcs_queries[l]["success"]) / len(gcs_queries) * 100
    gcs_q_med = _safe_median(gcs_q_times)
    gcs_l_med = _safe_median(gcs_q_lengths)

    # RRT summary
    rrt_q_times, rrt_q_lengths = [], []
    rrt_successes, rrt_total = 0, 0
    for label in rrt_queries:
        for r in rrt_queries[label]:
            rrt_total += 1
            if r["success"]:
                rrt_successes += 1
                rrt_q_times.append(r["time_s"])
                rrt_q_lengths.append(r["path_length"])
    rrt_sr = rrt_successes / max(1, rrt_total) * 100
    rrt_q_med = _safe_median(rrt_q_times)
    rrt_l_med = _safe_median(rrt_q_lengths)

    # PRM summary
    prm_q_times, prm_q_lengths = [], []
    prm_successes, prm_total = 0, 0
    for label in prm_queries:
        for r in prm_queries[label]:
            prm_total += 1
            if r["success"]:
                prm_successes += 1
                prm_q_times.append(r["time_s"])
                prm_q_lengths.append(r["path_length"])
    prm_sr = prm_successes / max(1, prm_total) * 100
    prm_q_med = _safe_median(prm_q_times)
    prm_l_med = _safe_median(prm_q_lengths)
    prm_build_med = _safe_median(prm_build_times)

    header = f"{'Planner':<25} {'Precomp(s)':>10} {'Query(s)':>10} {'Cost(rad)':>10} {'SR%':>6}"
    logger.info(header)
    logger.info("-" * 70)
    logger.info(f"{'SBF (ours)':<25} {sbf_build_med:>10.3f} "
                f"{sbf_q_med:>10.3f} {sbf_l_med:>10.3f} {sbf_sr:>6.0f}")
    logger.info(f"{'IRIS-NP+GCS (' + str(len(iris_regions)) + ' reg.)':<25} "
                f"{iris_total_s:>10.1f} "
                f"{gcs_q_med:>10.3f} {gcs_l_med:>10.3f} {gcs_sr:>6.0f}")
    logger.info(f"{'RRT-Connect':<25} {'---':>10} "
                f"{rrt_q_med:>10.3f} {rrt_l_med:>10.3f} {rrt_sr:>6.0f}")
    logger.info(f"{'PRM (10K, Drake CC)':<25} {prm_build_med:>10.1f} "
                f"{prm_q_med:>10.3f} {prm_l_med:>10.3f} {prm_sr:>6.0f}")
    logger.info("-" * 70)

    # Per-query detail
    logger.info(f"\n{'Query':<10} "
                f"{'SBF-t':>7} {'SBF-l':>7} "
                f"{'GCS-t':>7} {'GCS-l':>7} "
                f"{'RRT-t':>7} {'RRT-l':>7} "
                f"{'PRM-t':>7} {'PRM-l':>7}")
    per_query_data = {}
    for label, _, _ in QUERY_PAIRS:
        sbf_runs = [r for r in sbf_queries[label] if r["success"]]
        sbf_t = _safe_median([r["time_s"] for r in sbf_runs])
        sbf_l = _safe_median([r["path_length"] for r in sbf_runs])
        gcs_t = gcs_queries[label]["time_s"] if gcs_queries[label]["success"] else float("nan")
        gcs_l = gcs_queries[label]["path_length"] if gcs_queries[label]["success"] else float("nan")
        rrt_runs = [r for r in rrt_queries[label] if r["success"]]
        rrt_t = _safe_median([r["time_s"] for r in rrt_runs])
        rrt_l = _safe_median([r["path_length"] for r in rrt_runs])
        prm_runs = [r for r in prm_queries[label] if r["success"]]
        prm_t = _safe_median([r["time_s"] for r in prm_runs])
        prm_l = _safe_median([r["path_length"] for r in prm_runs])

        per_query_data[label] = {
            "sbf": {"time": sbf_t, "cost": sbf_l},
            "gcs": {"time": gcs_t, "cost": gcs_l},
            "rrt": {"time": rrt_t, "cost": rrt_l},
            "prm": {"time": prm_t, "cost": prm_l},
        }
        logger.info(f"{label:<10} "
                    f"{sbf_t:>7.3f} {sbf_l:>7.3f} "
                    f"{gcs_t:>7.3f} {gcs_l:>7.3f} "
                    f"{rrt_t:>7.3f} {rrt_l:>7.3f} "
                    f"{prm_t:>7.3f} {prm_l:>7.3f}")

    # Compute means
    means = {}
    for planner in ["sbf", "gcs", "rrt", "prm"]:
        ts = [per_query_data[l][planner]["time"] for l in per_query_data
              if not math.isnan(per_query_data[l][planner]["time"])]
        cs = [per_query_data[l][planner]["cost"] for l in per_query_data
              if not math.isnan(per_query_data[l][planner]["cost"])]
        means[planner] = {
            "time": float(np.mean(ts)) if ts else float("nan"),
            "cost": float(np.mean(cs)) if cs else float("nan"),
        }
    logger.info(f"{'Mean':<10} "
                f"{means['sbf']['time']:>7.3f} {means['sbf']['cost']:>7.3f} "
                f"{means['gcs']['time']:>7.3f} {means['gcs']['cost']:>7.3f} "
                f"{means['rrt']['time']:>7.3f} {means['rrt']['cost']:>7.3f} "
                f"{means['prm']['time']:>7.3f} {means['prm']['cost']:>7.3f}")

    all_results["summary"] = {
        "sbf_build_median_s": sbf_build_med,
        "sbf_query_median_s": sbf_q_med,
        "sbf_cost_median_rad": sbf_l_med,
        "sbf_sr": sbf_sr,
        "sbf_n_boxes_median": int(np.median(boxes_list)),
        "sbf_unique_box_count_median": int(np.median(sbf_unique_boxes)),
        "sbf_dedup_box_volume_sum_median": float(np.median(sbf_dedup_volumes)),
        "sbf_box_volume_sum_median": float(np.median(sbf_total_volumes)),
        "iris_precomp_s": iris_total_s,
        "iris_n_regions": len(iris_regions),
        "gcs_query_median_s": gcs_q_med,
        "gcs_cost_median_rad": gcs_l_med,
        "gcs_sr": gcs_sr,
        "rrt_query_median_s": rrt_q_med,
        "rrt_cost_median_rad": rrt_l_med,
        "rrt_sr": rrt_sr,
        "prm_build_median_s": prm_build_med,
        "prm_query_median_s": prm_q_med,
        "prm_cost_median_rad": prm_l_med,
        "prm_sr": prm_sr,
        "per_query": per_query_data,
        "per_query_means": means,
    }

    # ── Save JSON ──
    if output_json:
        def convert(obj):
            if isinstance(obj, (np.integer,)):
                return int(obj)
            if isinstance(obj, (np.floating,)):
                return float(obj)
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            if isinstance(obj, float) and math.isnan(obj):
                return None
            return obj

        def clean_dict(d):
            if isinstance(d, dict):
                return {k: clean_dict(v) for k, v in d.items()
                        if k != "raw_path"}
            if isinstance(d, list):
                return [clean_dict(x) for x in d]
            return convert(d)

        os.makedirs(os.path.dirname(os.path.abspath(output_json)), exist_ok=True)
        with open(output_json, "w") as f:
            json.dump(clean_dict(all_results), f, indent=2)
        logger.info(f"\nResults saved to {output_json}")

    return all_results


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Online Query Performance & Baseline Comparison")
    parser.add_argument("--seeds", type=int, default=5,
                        help="Number of seeds (default: 5)")
    parser.add_argument("--json", type=str, default=None,
                        help="Output JSON file path")
    parser.add_argument("--no-iris-cache", action="store_true",
                        help="Force regenerate IRIS regions")
    args = parser.parse_args()

    run_experiment(
        n_seeds=args.seeds,
        output_json=args.json,
        use_iris_cache=not args.no_iris_cache,
    )

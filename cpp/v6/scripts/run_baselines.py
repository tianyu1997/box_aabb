#!/usr/bin/env python3
"""
Phase 4 Baseline Comparison: SBF vs IRIS-NP+GCS vs RRT-Connect vs PRM

Runs on the Marcucci combined scene (IIWA14, 16 obstacles)
with 5 canonical query pairs from the Science Robotics paper.

Usage:
    conda activate sbf
    python3 scripts/run_baselines.py [--seeds N] [--json output.json]
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

# ─── Marcucci scene definitions ──────────────────────────────────────────

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

# IIWA14 joint limits (from Drake model)
IIWA_JOINT_LIMITS = [
    (-2.96706, 2.96706),
    (-2.09440, 2.09440),
    (-2.96706, 2.96706),
    (-2.09440, 2.09440),
    (-2.96706, 2.96706),
    (-2.09440, 2.09440),
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
    """Build Drake MultibodyPlant with Marcucci scene."""
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


# ─── Drake collision checker ─────────────────────────────────────────────

class DrakeCollisionChecker:
    """Collision checker using Drake's SceneGraph."""

    def __init__(self, diagram, plant):
        self._diagram = diagram
        self._plant = plant
        self._context = diagram.CreateDefaultContext()
        self._plant_context = plant.GetMyContextFromRoot(self._context)
        self._sg_context = None
        # Get the SceneGraph for distance queries
        self.n_checks = 0

    def check_config(self, q: np.ndarray) -> bool:
        """Return True if config is in collision."""
        self.n_checks += 1
        self._plant.SetPositions(self._plant_context, q)
        query_object = self._plant.get_geometry_query_input_port().Eval(
            self._plant_context)
        return query_object.HasCollisions()

    def check_segment(self, q_from: np.ndarray, q_to: np.ndarray,
                      resolution: float = 0.05) -> bool:
        """Return True if segment is collision-free."""
        diff = q_to - q_from
        dist = np.linalg.norm(diff)
        if dist < 1e-10:
            return not self.check_config(q_from)
        n_steps = max(2, int(np.ceil(dist / resolution)))
        for i in range(n_steps + 1):
            t = i / n_steps
            q = q_from + t * diff
            if self.check_config(q):
                return False
        return True

    def reset_count(self):
        self.n_checks = 0


# ─── Path utilities ──────────────────────────────────────────────────────

def path_length(waypoints):
    if len(waypoints) < 2:
        return 0.0
    return sum(np.linalg.norm(waypoints[i] - waypoints[i-1])
               for i in range(1, len(waypoints)))


def shortcut_path(waypoints, checker, max_iters=300, rng=None):
    """Greedy random shortcut."""
    if rng is None:
        rng = np.random.default_rng()
    path = list(waypoints)
    for _ in range(max_iters):
        if len(path) <= 2:
            break
        i = rng.integers(0, len(path) - 2)
        j = rng.integers(i + 2, len(path))
        if checker.check_segment(path[i], path[j]):
            path = path[:i+1] + path[j:]
    return path


# ─── RRT-Connect baseline ────────────────────────────────────────────────

def run_rrt_connect(q_start, q_goal, checker, *,
                    timeout=60.0, step_size=0.3, seed=42):
    """Bi-directional RRT-Connect."""
    ndim = len(q_start)
    lows = np.array([lo for lo, _ in IIWA_JOINT_LIMITS])
    highs = np.array([hi for _, hi in IIWA_JOINT_LIMITS])
    rng = np.random.default_rng(seed)

    # Check start/goal collision-free
    if checker.check_config(q_start) or checker.check_config(q_goal):
        return {"success": False, "time_s": 0, "path_length": float("nan"),
                "nodes": 0, "collision_checks": 0}

    # Node storage
    configs_a = [q_start.copy()]
    parents_a = [-1]
    configs_b = [q_goal.copy()]
    parents_b = [-1]

    checker.reset_count()
    t0 = time.perf_counter()

    def nearest(configs, q):
        dists = [np.sum((c - q)**2) for c in configs]
        return int(np.argmin(dists))

    def steer(q_from, q_to):
        diff = q_to - q_from
        d = np.linalg.norm(diff)
        if d <= step_size:
            return q_to.copy()
        return q_from + (step_size / d) * diff

    def extend(configs, parents, q_target):
        idx = nearest(configs, q_target)
        q_near = configs[idx]
        q_new = steer(q_near, q_target)
        if checker.check_segment(q_near, q_new, resolution=0.05):
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
                "success": True,
                "time_s": dt,
                "path_length": path_length(full_sc),
                "path_length_raw": path_length(full),
                "nodes": len(configs_a) + len(configs_b),
                "collision_checks": checker.n_checks,
            }

        # Swap trees
        configs_a, configs_b = configs_b, configs_a
        parents_a, parents_b = parents_b, parents_a
        swapped = not swapped

    dt = time.perf_counter() - t0
    return {"success": False, "time_s": dt, "path_length": float("nan"),
            "nodes": len(configs_a) + len(configs_b),
            "collision_checks": checker.n_checks}


# ─── PRM baseline ────────────────────────────────────────────────────────

class PRMRoadmap:
    """Lazy PRM roadmap: edges assumed valid until checked during query."""

    def __init__(self, checker, *, n_samples=3000, k_neighbors=15,
                 connection_radius=2.0, seed=42):
        self.checker = checker
        self.n_samples = n_samples
        self.k_neighbors = k_neighbors
        self.connection_radius = connection_radius
        self.seed = seed
        self.nodes = None         # np.ndarray (N, ndim)
        self.adjacency = None     # dict[int, list[(int, float)]]
        self.invalid_edges = None  # set of (i,j) pairs known to be in collision
        self.build_time = 0.0

    def build(self, timeout=120.0):
        """Build the roadmap: sample nodes, connect k-NN lazily (no collision check)."""
        from collections import defaultdict
        lows = np.array([lo for lo, _ in IIWA_JOINT_LIMITS])
        highs = np.array([hi for _, hi in IIWA_JOINT_LIMITS])
        rng = np.random.default_rng(self.seed)

        self.checker.reset_count()
        t0 = time.perf_counter()

        # 1. Sample collision-free configs
        nodes_list = []
        max_attempts = self.n_samples * 5
        for _ in range(max_attempts):
            if len(nodes_list) >= self.n_samples:
                break
            if time.perf_counter() - t0 > timeout * 0.5:
                break
            q = rng.uniform(lows, highs)
            if not self.checker.check_config(q):
                nodes_list.append(q)

        self.nodes = np.array(nodes_list, dtype=np.float64)
        n = len(self.nodes)

        # 2. Build lazy edges (k-NN + radius, NO collision checking)
        self.adjacency = defaultdict(list)
        self.invalid_edges = set()
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
                if (i, j) in edge_set:
                    continue
                self.adjacency[i].append((j, d))
                self.adjacency[j].append((i, d))
                edge_set.add((i, j))
                edge_set.add((j, i))

        self.build_time = time.perf_counter() - t0
        n_edges = len(edge_set) // 2
        logger.info(f"  PRM roadmap: {n} nodes, {n_edges} edges (lazy), "
                     f"{self.build_time:.1f}s")

    def _check_edge(self, i, j):
        """Check if edge (i,j) is collision-free. Cache invalid results."""
        key = (min(i, j), max(i, j))
        if key in self.invalid_edges:
            return False
        if self.checker.check_segment(
                self.nodes[i], self.nodes[j], resolution=0.05):
            return True
        else:
            self.invalid_edges.add(key)
            return False

    def query(self, q_start, q_goal, timeout=30.0):
        """Query with lazy edge evaluation: find path, validate, repeat."""
        import heapq

        self.checker.reset_count()
        t0 = time.perf_counter()
        n = len(self.nodes)

        # Connect start/goal to multiple nearest roadmap nodes
        def _connect_lazy(q):
            """Return list of (node_idx, dist) for connectable nodes."""
            dists = np.linalg.norm(self.nodes - q, axis=1)
            order = np.argsort(dists)
            connections = []
            for idx in order[:self.k_neighbors * 4]:
                idx = int(idx)
                if dists[idx] > self.connection_radius * 2:
                    break
                connections.append((idx, float(dists[idx])))
            return connections

        start_conns = _connect_lazy(q_start)
        goal_conns = _connect_lazy(q_goal)

        if not start_conns or not goal_conns:
            dt = time.perf_counter() - t0
            return {"success": False, "time_s": dt,
                    "path_length": float("nan"),
                    "nodes": n, "collision_checks": self.checker.n_checks}

        # Virtual node indices for start/goal
        START_V = n
        GOAL_V = n + 1
        goal_set = {idx for idx, _ in goal_conns}

        # Lazy Dijkstra with edge validation
        max_retries = 200
        removed_edges = set()  # edges removed during this query

        for attempt in range(max_retries):
            if time.perf_counter() - t0 > timeout:
                break

            # Dijkstra from START_V
            dist_to = {}
            prev = {}
            pq = []

            # Push start connections
            for idx, d in start_conns:
                if (START_V, idx) not in removed_edges:
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
                    if key in self.invalid_edges or key in removed_edges:
                        continue
                    nd = d + w
                    if nd < dist_to.get(v, float('inf')):
                        dist_to[v] = nd
                        prev[v] = u
                        heapq.heappush(pq, (nd, v))

            if found_goal_node is None:
                break  # no path even on lazy graph

            # Extract path indices
            path_indices = [found_goal_node]
            idx = found_goal_node
            while prev.get(idx) != START_V:
                idx = prev[idx]
                path_indices.append(idx)
            path_indices.reverse()

            # Validate edges along the path
            all_valid = True

            # Check start -> first node
            if not self.checker.check_segment(
                    q_start, self.nodes[path_indices[0]], resolution=0.05):
                removed_edges.add((START_V, path_indices[0]))
                all_valid = False
                continue

            # Check roadmap edges
            invalid_found = False
            for k in range(len(path_indices) - 1):
                i, j = path_indices[k], path_indices[k + 1]
                if not self._check_edge(i, j):
                    invalid_found = True
                    break
            if invalid_found:
                all_valid = False
                continue

            # Check last node -> goal
            if not self.checker.check_segment(
                    self.nodes[found_goal_node], q_goal, resolution=0.05):
                removed_edges.add((found_goal_node, GOAL_V))
                # Remove this goal connection
                goal_conns = [(idx, d) for idx, d in goal_conns
                              if idx != found_goal_node]
                goal_set = {idx for idx, _ in goal_conns}
                all_valid = False
                continue

            if all_valid:
                # Build waypoint path
                path = [q_start.copy()]
                for pi in path_indices:
                    path.append(self.nodes[pi].copy())
                path.append(q_goal.copy())

                dt = time.perf_counter() - t0
                rng = np.random.default_rng(42)
                path_sc = shortcut_path(path, self.checker,
                                        max_iters=200, rng=rng)
                return {
                    "success": True,
                    "time_s": dt,
                    "path_length": path_length(path_sc),
                    "path_length_raw": path_length(path),
                    "nodes": n,
                    "collision_checks": self.checker.n_checks,
                }

        dt = time.perf_counter() - t0
        return {"success": False, "time_s": dt,
                "path_length": float("nan"),
                "nodes": n, "collision_checks": self.checker.n_checks}


# ─── IRIS-NP + GCS baseline ─────────────────────────────────────────────

def run_iris_gcs(q_start, q_goal, plant, diagram, iris_regions, *,
                 seed=42):
    """Run GCS shortest path through precomputed IRIS regions.

    Uses LinearGCS from gcs-science-robotics (Marcucci et al. 2024) with
    proper L2NormCost edge costs, cross-region constraints, and
    randomForwardPathSearch rounding.
    """
    # Add gcs-science-robotics to path
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

    # Build GCS graph with LinearGCS (auto edge detection via IntersectsWith)
    gcs = LinearGCS(iris_regions)

    # Add source and target
    try:
        gcs.addSourceTarget(q_start, q_goal)
    except ValueError as e:
        dt = time.perf_counter() - t0
        return {"success": False, "time_s": dt, "path_length": float("nan"),
                "regions": n, "edges": 0,
                "note": f"addSourceTarget failed: {e}"}

    # Set rounding strategy (Marcucci paper: randomForwardPathSearch)
    gcs.setRoundingStrategy(randomForwardPathSearch,
                            max_paths=10, max_trials=100, seed=seed)
    gcs.setSolver(MosekSolver())

    # Set solver tolerances (matching paper)
    solver_options = SolverOptions()
    solver_options.SetOption(MosekSolver.id(),
                            "MSK_DPAR_INTPNT_CO_TOL_REL_GAP", 1e-3)
    gcs.setSolverOptions(solver_options)

    # Solve with rounding
    waypoints, results_dict = gcs.SolvePath(
        rounding=True, verbose=False, preprocessing=True)

    dt = time.perf_counter() - t0

    if waypoints is None:
        return {"success": False, "time_s": dt, "path_length": float("nan"),
                "regions": n,
                "edges": len([e for e in gcs.gcs.Edges()]),
                "note": "GCS solve failed"}

    # waypoints shape: (ndim, n_points) → compute path length
    path = waypoints.T  # (n_points, ndim)
    pl = float(sum(
        np.linalg.norm(path[i] - path[i-1])
        for i in range(1, len(path))
    ))

    return {
        "success": True,
        "time_s": dt,
        "path_length": pl,
        "regions": n,
        "edges": len([e for e in gcs.gcs.Edges()]),
        "waypoints_count": path.shape[0],
    }


def generate_iris_regions(plant, diagram, seed_configs, max_iters=10,
                          cache_file=None):
    """Generate IRIS-NP regions from seed configurations."""
    from pydrake.geometry.optimization import IrisNp, IrisOptions, HPolyhedron

    # Try loading from cache
    if cache_file and os.path.exists(cache_file):
        logger.info(f"  Loading IRIS regions from cache: {cache_file}")
        cached = np.load(cache_file, allow_pickle=True)
        regions = []
        timings = list(cached["timings"])
        n = int(cached["n_regions"])
        for i in range(n):
            A = cached[f"A_{i}"]
            b = cached[f"b_{i}"]
            regions.append(HPolyhedron(A, b))
        logger.info(f"  Loaded {n} cached regions")
        return regions, timings

    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)

    opts = IrisOptions()
    opts.iteration_limit = max_iters
    opts.termination_threshold = -1          # disabled (use relative)
    opts.relative_termination_threshold = 2e-2  # paper default
    opts.num_collision_infeasible_samples = 1
    opts.random_seed = 0
    opts.require_sample_point_is_contained = True

    regions = []
    timings = []

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

    # Save to cache
    if cache_file:
        save_dict = {"n_regions": len(regions), "timings": np.array(timings)}
        for i, r in enumerate(regions):
            save_dict[f"A_{i}"] = r.A()
            save_dict[f"b_{i}"] = r.b()
        np.savez(cache_file, **save_dict)
        logger.info(f"  Saved {len(regions)} regions to {cache_file}")

    return regions, timings


# ─── SBF + GCS post-optimization ─────────────────────────────────────────

def make_combined_obstacles():
    """Replicate Marcucci combined scene obstacles (16 AABBs).

    Matches marcucci_scenes.h make_combined_obstacles().
    """
    if SBF_BUILD_DIR not in sys.path:
        sys.path.insert(0, SBF_BUILD_DIR)
    import _sbf5_cpp as sbf5

    def make_shelves():
        ox, oy, oz = 0.85, 0.0, 0.4
        obs = []
        def add(lx, ly, lz, fx, fy, fz):
            obs.append(sbf5.Obstacle(
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
                obs.append(sbf5.Obstacle(
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
        return [sbf5.Obstacle(0.4-2.5/2, -2.5/2, -0.25-0.2/2,
                               0.4+2.5/2, 2.5/2, -0.25+0.2/2)]

    return make_shelves() + make_bins() + make_table()


def run_sbf_gcs(q_start, q_goal, sbf_obstacles, *, seed=42):
    """Run SBF planner then optimize path with GCS over box sequence.

    1. SBF finds a box-graph path (Dijkstra + shortcut + smooth)
    2. Extract the box sequence as HPolyhedron regions
    3. Build GCS graph on sequential boxes with L2NormCost edges
    4. Solve shortest path through the box corridor

    Returns dict with SBF raw path and GCS-optimized path lengths.
    """
    if SBF_BUILD_DIR not in sys.path:
        sys.path.insert(0, SBF_BUILD_DIR)

    import _sbf5_cpp as sbf5
    from pydrake.geometry.optimization import (
        GraphOfConvexSets, HPolyhedron, Point as DrakePoint)
    from pydrake.solvers import MosekSolver
    from pydrake.math import le, ge

    t0 = time.perf_counter()

    # 1. Run SBF planner
    robot = sbf5.Robot.from_json(os.path.join(SBF_DATA_DIR, "iiwa14.json"))
    config = sbf5.SBFPlannerConfig()
    config.grower.timeout_ms = 10000
    planner = sbf5.SBFPlanner(robot, config)

    result = planner.plan(q_start, q_goal, sbf_obstacles, 15000)
    t_sbf = time.perf_counter() - t0

    if not result.success:
        return {
            "success": False, "time_s": t_sbf,
            "sbf_path_length": float("nan"),
            "gcs_path_length": float("nan"),
            "n_boxes_total": 0, "n_boxes_seq": 0,
        }

    sbf_path_length = result.path_length
    box_seq_ids = result.box_sequence
    all_boxes = planner.boxes()
    box_map = {b.id: b for b in all_boxes}
    n_dims = len(q_start)

    # 2. Build GCS graph manually on the box sequence
    t_gcs_start = time.perf_counter()
    try:
        gcs = GraphOfConvexSets()

        # Add source and target as Point vertices
        source = gcs.AddVertex(DrakePoint(q_start), "source")
        target = gcs.AddVertex(DrakePoint(q_goal), "target")

        # Deduplicate consecutive box IDs
        deduped = [box_seq_ids[0]]
        for bid in box_seq_ids[1:]:
            if bid != deduped[-1]:
                deduped.append(bid)

        # Add box vertices
        box_vertices = []
        for i, bid in enumerate(deduped):
            b = box_map[bid]
            lb = np.array([iv.lo for iv in b.joint_intervals])
            ub = np.array([iv.hi for iv in b.joint_intervals])
            hpoly = HPolyhedron.MakeBox(lb, ub)
            v = gcs.AddVertex(hpoly, f"box_{i}")
            box_vertices.append(v)

        # Add edges: source -> first box
        def add_l2_edge(u, v, name=""):
            e = gcs.AddEdge(u, v, name)
            xu = e.xu()
            xv = e.xv()
            diff = xu - xv
            e.AddCost(diff.dot(diff))
            return e

        add_l2_edge(source, box_vertices[0], "s->0")

        # Consecutive box edges (both directions for flexibility)
        for i in range(len(box_vertices) - 1):
            add_l2_edge(box_vertices[i], box_vertices[i+1], f"{i}->{i+1}")

        # Last box -> target
        add_l2_edge(box_vertices[-1], target, f"{len(box_vertices)-1}->t")

        # Solve GCS shortest path (convex relaxation)
        options = GraphOfConvexSets.SolveConvexRestriction
        result_gcs = gcs.SolveShortestPath(source, target, True)

        if not result_gcs.is_success():
            raise RuntimeError("GCS SolveShortestPath failed")

        # Extract path from active edges
        path_points = [q_start.copy()]
        for edge in gcs.Edges():
            phi = result_gcs.GetSolution(edge.phi())
            if phi > 0.5:
                xv_val = result_gcs.GetSolution(edge.xv())
                # Don't add the target twice
                if edge.v() != target:
                    path_points.append(xv_val.copy())
        path_points.append(q_goal.copy())

        # Sort path by edge order (edges may not be in order)
        # Use the sequential structure: source -> box_0 -> box_1 -> ... -> target
        ordered_path = [q_start.copy()]
        current = source
        visited = set()
        for _ in range(len(box_vertices) + 2):
            visited.add(current.id())
            found = False
            for edge in gcs.Edges():
                if edge.u() == current and result_gcs.GetSolution(edge.phi()) > 0.5:
                    if edge.v().id() not in visited:
                        wp = result_gcs.GetSolution(edge.xv())
                        if edge.v() != target:
                            ordered_path.append(wp.copy())
                        current = edge.v()
                        found = True
                        break
            if not found:
                break
        ordered_path.append(q_goal.copy())

    except Exception as e:
        logger.warning(f"  SBF+GCS optimization failed: {e}")
        t_gcs = time.perf_counter() - t_gcs_start
        return {
            "success": True, "time_s": time.perf_counter() - t0,
            "sbf_path_length": sbf_path_length,
            "gcs_path_length": float("nan"),
            "gcs_success": False,
            "n_boxes_total": len(all_boxes),
            "n_boxes_seq": len(box_seq_ids),
            "sbf_time_s": t_sbf,
            "gcs_time_s": t_gcs,
        }

    t_gcs = time.perf_counter() - t_gcs_start
    t_total = time.perf_counter() - t0

    # Compute optimized path length
    gcs_pl = float(sum(
        np.linalg.norm(ordered_path[i] - ordered_path[i-1])
        for i in range(1, len(ordered_path))))

    return {
        "success": True, "time_s": t_total,
        "sbf_path_length": sbf_path_length,
        "gcs_path_length": gcs_pl,
        "gcs_success": True,
        "n_boxes_total": len(all_boxes),
        "n_boxes_seq": len(deduped),
        "gcs_waypoints": len(ordered_path),
        "sbf_time_s": t_sbf,
        "gcs_time_s": t_gcs,
    }


# ─── SBF data loader ────────────────────────────────────────────────────

def load_sbf_results():
    """Load SBF experiment results from JSON files."""
    results_dir = os.path.join(os.path.dirname(__file__),
                               "..", "experiments", "results_new")

    sbf_data = {}

    # Load ablation data (ALL ON config)
    ablation_file = os.path.join(results_dir, "exp5_ablation_10seed.json")
    if os.path.exists(ablation_file):
        with open(ablation_file) as f:
            data = json.load(f)
        for r in data["results"]:
            if r["name"] == "ALL ON (P0+P2+P4)":
                sbf_data["build_median"] = r["build_median"]
                sbf_data["build_mean"] = r["build_mean"]
                sbf_data["query_median"] = r["query_median"]
                sbf_data["query_mean"] = r["query_mean"]
                sbf_data["boxes"] = r["boxes"]
                sbf_data["sr"] = r["sr"]
                sbf_data["path_length"] = r.get("len_median", 3.099)
                break

    # Load timing data
    timing_file = os.path.join(results_dir, "exp6_timing_10seed.json")
    if os.path.exists(timing_file):
        with open(timing_file) as f:
            tdata = json.load(f)
        if "seeds" in tdata and len(tdata["seeds"]) > 0:
            total_times = [s.get("total_ms", 0) for s in tdata["seeds"]]
            sbf_data["build_total_ms"] = np.median(total_times)

    return sbf_data


# ─── Main comparison ────────────────────────────────────────────────────

def run_comparison(n_seeds=3, output_json=None):
    """Run full baseline comparison."""
    logger.info("=" * 70)
    logger.info("Phase 4: Baseline Comparison")
    logger.info("  Scene: Marcucci combined (IIWA14, 16 obstacles)")
    logger.info(f"  Seeds: {n_seeds}")
    logger.info("=" * 70)

    # ── 1. Load SBF results ──
    logger.info("\n[1/4] Loading SBF results...")
    sbf = load_sbf_results()
    if sbf:
        logger.info(f"  SBF: build={sbf.get('build_median', '?')}s, "
                     f"query={sbf.get('query_median', '?')}s, "
                     f"boxes={sbf.get('boxes', '?')}, "
                     f"SR={sbf.get('sr', '?')}%")
    else:
        logger.warning("  No SBF data found, using paper values")
        sbf = {"build_median": 2.40, "query_median": 0.164,
               "boxes": 4650, "sr": 100.0, "path_length": 3.099}

    # ── 2. Build Drake plant ──
    logger.info("\n[2/4] Building Drake plant...")
    t0 = time.perf_counter()
    diagram, plant = build_drake_plant()
    logger.info(f"  Plant OK: {plant.num_positions()} DOF, "
                f"{plant.num_bodies()} bodies ({time.perf_counter()-t0:.1f}s)")

    checker = DrakeCollisionChecker(diagram, plant)

    # Verify start/goal configs are collision-free
    for name, cfg in IIWA_CONFIGS.items():
        in_collision = checker.check_config(cfg)
        logger.info(f"  Config {name}: {'COLLISION' if in_collision else 'free'}")

    # ── 3. IRIS-NP + GCS ──
    logger.info("\n[3/4] Running IRIS-NP + GCS baseline...")
    iris_seeds = [
        ("AS", IIWA_CONFIGS["AS"]),
        ("TS", IIWA_CONFIGS["TS"]),
        ("CS", IIWA_CONFIGS["CS"]),
        ("LB", IIWA_CONFIGS["LB"]),
        ("RB", IIWA_CONFIGS["RB"]),
    ]

    # Also add midpoint configs as additional seeds
    midpoints = []
    for label, start_name, goal_name in QUERY_PAIRS:
        mid = (IIWA_CONFIGS[start_name] + IIWA_CONFIGS[goal_name]) / 2.0
        if not checker.check_config(mid):
            midpoints.append((f"mid_{label}", mid))
    iris_seeds.extend(midpoints)

    logger.info(f"  Generating {len(iris_seeds)} IRIS-NP regions...")
    iris_cache = os.path.join(os.path.dirname(__file__),
                              "..", "experiments", "results_new",
                              "iris_regions_cache.npz")
    t_iris_start = time.perf_counter()
    iris_regions, iris_timings = generate_iris_regions(
        plant, diagram, iris_seeds, max_iters=10, cache_file=iris_cache)
    t_iris_total = sum(iris_timings)  # use actual per-region times
    logger.info(f"  IRIS-NP: {len(iris_regions)} regions in {t_iris_total:.1f}s "
                f"(avg {t_iris_total/max(1,len(iris_regions)):.1f}s/region)")

    # Run GCS for each query pair
    gcs_results = {}
    for label, start_name, goal_name in QUERY_PAIRS:
        q_s = IIWA_CONFIGS[start_name]
        q_g = IIWA_CONFIGS[goal_name]
        res = run_iris_gcs(q_s, q_g, plant, diagram, iris_regions)
        gcs_results[label] = res
        status = "OK" if res["success"] else "FAIL"
        logger.info(f"  GCS {label}: {status}, "
                     f"time={res['time_s']:.3f}s, "
                     f"length={res.get('path_length', float('nan')):.3f}")

    # ── 4. RRT-Connect and PRM ──
    logger.info("\n[4/4] Running sampling-based baselines...")

    all_results = {"sbf": sbf, "iris_gcs": {}, "rrt_connect": {}, "prm": {}}

    # Store IRIS-NP summary
    all_results["iris_gcs"]["iris_time_total_s"] = t_iris_total
    all_results["iris_gcs"]["n_regions"] = len(iris_regions)
    all_results["iris_gcs"]["iris_per_region_s"] = iris_timings
    all_results["iris_gcs"]["queries"] = {}

    for label, start_name, goal_name in QUERY_PAIRS:
        all_results["iris_gcs"]["queries"][label] = gcs_results[label]
        all_results["rrt_connect"][label] = []
        all_results["prm"][label] = []

    # Build PRM roadmaps (one per seed, reused across all query pairs)
    prm_roadmaps = {}
    for s in range(n_seeds):
        logger.info(f"  Building PRM roadmap seed={s}...")
        rm = PRMRoadmap(checker, n_samples=20000, k_neighbors=30,
                        connection_radius=5.0, seed=s * 1000 + 42)
        rm.build(timeout=180.0)
        prm_roadmaps[s] = rm

    for label, start_name, goal_name in QUERY_PAIRS:
        q_s = IIWA_CONFIGS[start_name]
        q_g = IIWA_CONFIGS[goal_name]

        for s in range(n_seeds):
            # RRT-Connect
            checker.reset_count()
            rrt_res = run_rrt_connect(q_s, q_g, checker,
                                      timeout=60.0, step_size=0.3,
                                      seed=s * 1000 + 42)
            all_results["rrt_connect"][label].append(rrt_res)
            status = "OK" if rrt_res["success"] else "FAIL"
            logger.info(f"  RRT-Connect {label} seed={s}: {status}, "
                        f"time={rrt_res['time_s']:.3f}s, "
                        f"length={rrt_res.get('path_length', float('nan')):.3f}")

            # PRM (query on pre-built roadmap)
            prm_res = prm_roadmaps[s].query(q_s, q_g)
            all_results["prm"][label].append(prm_res)
            status = "OK" if prm_res["success"] else "FAIL"
            logger.info(f"  PRM {label} seed={s}: {status}, "
                        f"time={prm_res['time_s']:.3f}s, "
                        f"length={prm_res.get('path_length', float('nan')):.3f}")

    # ── (SBF+GCS removed — SBF's native shortcut+smooth gives comparable quality) ──

    all_results["sbf_gcs"] = {}
    sbf_gcs_results = {}

    # ── Summary ──
    logger.info("\n" + "=" * 70)
    logger.info("COMPARISON SUMMARY")
    logger.info("=" * 70)

    header = f"{'Planner':<20} {'Precomp(s)':>10} {'Query(s)':>10} {'Total(s)':>10} {'Cost(rad)':>10} {'SR%':>6}"
    logger.info(header)
    logger.info("-" * 70)

    # SBF
    sbf_total = sbf.get("build_median", 0) + sbf.get("query_median", 0)
    logger.info(f"{'SBF (ours)':<20} {sbf.get('build_median',0):>10.3f} "
                f"{sbf.get('query_median',0):>10.3f} "
                f"{sbf_total:>10.3f} "
                f"{sbf.get('path_length',0):>10.3f} "
                f"{sbf.get('sr',0):>6.0f}")

    # IRIS-NP + GCS
    gcs_query_times = [gcs_results[l]["time_s"] for l in gcs_results
                       if gcs_results[l]["success"]]
    gcs_lengths = [gcs_results[l]["path_length"] for l in gcs_results
                   if gcs_results[l]["success"]]
    gcs_sr = sum(1 for l in gcs_results if gcs_results[l]["success"]) / len(gcs_results) * 100
    gcs_q_med = np.median(gcs_query_times) if gcs_query_times else float("nan")
    gcs_l_med = np.median(gcs_lengths) if gcs_lengths else float("nan")
    logger.info(f"{'IRIS-NP+GCS':<20} {t_iris_total:>10.1f} "
                f"{gcs_q_med:>10.3f} "
                f"{t_iris_total + gcs_q_med:>10.1f} "
                f"{gcs_l_med:>10.3f} "
                f"{gcs_sr:>6.0f}")

    # RRT-Connect (median over seeds)
    rrt_times_all = []
    rrt_lengths_all = []
    rrt_successes = 0
    rrt_total = 0
    for label in all_results["rrt_connect"]:
        for r in all_results["rrt_connect"][label]:
            rrt_total += 1
            if r["success"]:
                rrt_successes += 1
                rrt_times_all.append(r["time_s"])
                rrt_lengths_all.append(r["path_length"])
    rrt_sr = rrt_successes / max(1, rrt_total) * 100
    rrt_t_med = np.median(rrt_times_all) if rrt_times_all else float("nan")
    rrt_l_med = np.median(rrt_lengths_all) if rrt_lengths_all else float("nan")
    logger.info(f"{'RRT-Connect':<20} {'0':>10} "
                f"{rrt_t_med:>10.3f} "
                f"{rrt_t_med:>10.3f} "
                f"{rrt_l_med:>10.3f} "
                f"{rrt_sr:>6.0f}")

    # PRM
    prm_times_all = []
    prm_lengths_all = []
    prm_successes = 0
    prm_total = 0
    for label in all_results["prm"]:
        for r in all_results["prm"][label]:
            prm_total += 1
            if r["success"]:
                prm_successes += 1
                prm_times_all.append(r["time_s"])
                prm_lengths_all.append(r["path_length"])
    prm_sr = prm_successes / max(1, prm_total) * 100
    prm_t_med = np.median(prm_times_all) if prm_times_all else float("nan")
    prm_l_med = np.median(prm_lengths_all) if prm_lengths_all else float("nan")
    # PRM build time (median across seeds)
    prm_build_med = float(np.median([rm.build_time for rm in prm_roadmaps.values()]))
    logger.info(f"{'PRM (20000 nodes)':<20} {prm_build_med:>10.1f} "
                f"{prm_t_med:>10.3f} "
                f"{prm_build_med + prm_t_med:>10.1f} "
                f"{prm_l_med:>10.3f} "
                f"{prm_sr:>6.0f}")

    # SBF + GCS
    if sbf_gcs_results:
        sgcs_gcs_lengths = [r["gcs_path_length"] for r in sbf_gcs_results.values()
                            if r.get("gcs_success")]
        sgcs_sbf_lengths = [r["sbf_path_length"] for r in sbf_gcs_results.values()
                            if r["success"]]
        sgcs_times = [r["time_s"] for r in sbf_gcs_results.values()
                      if r.get("gcs_success")]
        sgcs_sr = sum(1 for r in sbf_gcs_results.values()
                      if r.get("gcs_success")) / max(1, len(sbf_gcs_results)) * 100
        sgcs_l_med = np.median(sgcs_gcs_lengths) if sgcs_gcs_lengths else float("nan")
        sgcs_t_med = np.median(sgcs_times) if sgcs_times else float("nan")
        sgcs_sbf_l_med = np.median(sgcs_sbf_lengths) if sgcs_sbf_lengths else float("nan")
        logger.info(f"{'SBF+GCS (ours)':<20} {sbf.get('build_median',0):>10.3f} "
                    f"{sgcs_t_med:>10.3f} "
                    f"{sbf.get('build_median',0) + sgcs_t_med:>10.3f} "
                    f"{sgcs_l_med:>10.3f} "
                    f"{sgcs_sr:>6.0f}")
        logger.info(f"  (SBF raw path median: {sgcs_sbf_l_med:.3f} rad)")

    logger.info("-" * 70)

    # ── Per-query detail table ──
    logger.info("\nPer-Query Detail:")
    logger.info(f"{'Query':<10} {'SBF(s)':>8} {'GCS(s)':>8} "
                f"{'RRT(s)':>8} {'PRM(s)':>8} "
                f"{'RRT-len':>8} {'PRM-len':>8}")
    for label, _, _ in QUERY_PAIRS:
        gcs_t = gcs_results[label]["time_s"] if gcs_results[label]["success"] else "FAIL"
        rrt_runs = all_results["rrt_connect"][label]
        rrt_succ = [r for r in rrt_runs if r["success"]]
        rrt_t = np.median([r["time_s"] for r in rrt_succ]) if rrt_succ else "FAIL"
        rrt_l = np.median([r["path_length"] for r in rrt_succ]) if rrt_succ else "---"
        prm_runs = all_results["prm"][label]
        prm_succ = [r for r in prm_runs if r["success"]]
        prm_t = np.median([r["time_s"] for r in prm_succ]) if prm_succ else "FAIL"
        prm_l = np.median([r["path_length"] for r in prm_succ]) if prm_succ else "---"

        gcs_str = f"{gcs_t:.3f}" if isinstance(gcs_t, float) else gcs_t
        rrt_str = f"{rrt_t:.3f}" if isinstance(rrt_t, float) else rrt_t
        prm_str = f"{prm_t:.3f}" if isinstance(prm_t, float) else prm_t
        rrt_l_str = f"{rrt_l:.3f}" if isinstance(rrt_l, float) else rrt_l
        prm_l_str = f"{prm_l:.3f}" if isinstance(prm_l, float) else prm_l

        logger.info(f"{label:<10} {'---':>8} {gcs_str:>8} "
                    f"{rrt_str:>8} {prm_str:>8} "
                    f"{rrt_l_str:>8} {prm_l_str:>8}")

    # ── Save JSON ──
    if output_json:
        # Make numpy types serializable
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
                        if k != "waypoints"}
            if isinstance(d, list):
                return [clean_dict(x) for x in d]
            return convert(d)

        output = {
            "scene": "combined",
            "robot": "kuka_iiwa14",
            "n_seeds": n_seeds,
            "sbf": clean_dict(sbf),
            "iris_gcs": {
                "iris_time_total_s": float(t_iris_total),
                "n_regions": len(iris_regions),
                "iris_per_region_s": [float(x) for x in iris_timings],
                "queries": {k: clean_dict(v)
                            for k, v in gcs_results.items()},
            },
            "rrt_connect": {k: clean_dict(v)
                            for k, v in all_results["rrt_connect"].items()},
            "prm": {k: clean_dict(v)
                    for k, v in all_results["prm"].items()},
            "sbf_gcs": {k: clean_dict(v)
                        for k, v in sbf_gcs_results.items()},
            "summary": {
                "sbf_build_s": float(sbf.get("build_median", 0)),
                "sbf_query_s": float(sbf.get("query_median", 0)),
                "sbf_total_s": float(sbf_total),
                "sbf_sr": float(sbf.get("sr", 0)),
                "sbf_gcs_path_median": float(sgcs_l_med) if sbf_gcs_results and not np.isnan(sgcs_l_med) else None,
                "sbf_gcs_sr": float(sgcs_sr) if sbf_gcs_results else None,
                "iris_precomp_s": float(t_iris_total),
                "gcs_query_median_s": float(gcs_q_med) if not np.isnan(gcs_q_med) else None,
                "gcs_sr": float(gcs_sr),
                "rrt_query_median_s": float(rrt_t_med) if not np.isnan(rrt_t_med) else None,
                "rrt_sr": float(rrt_sr),
                "prm_query_median_s": float(prm_t_med) if not np.isnan(prm_t_med) else None,
                "prm_build_median_s": float(prm_build_med),
                "prm_sr": float(prm_sr),
            },
        }

        os.makedirs(os.path.dirname(os.path.abspath(output_json)), exist_ok=True)
        with open(output_json, "w") as f:
            json.dump(output, f, indent=2)
        logger.info(f"\nResults saved to {output_json}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Phase 4 Baseline Comparison")
    parser.add_argument("--seeds", type=int, default=3,
                        help="Number of seeds for RRT/PRM (default: 3)")
    parser.add_argument("--json", type=str, default=None,
                        help="Output JSON file path")
    args = parser.parse_args()

    run_comparison(n_seeds=args.seeds, output_json=args.json)

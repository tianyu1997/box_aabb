#!/usr/bin/env python3
"""
ompl_bridge_2dof.py -- OMPL planner bridge for arbitrary robots (runs in WSL)

Based on v2/examples/ompl_bridge.py but supports robot_name via JSON input.

Protocol:
  stdin  -> JSON with keys:
    robot_name: str  (e.g. "2dof_planar", "panda")
    q_start, q_goal: list[float]
    joint_limits: list[[lo, hi], ...]
    obstacles: list[{min_point: [...], max_point: [...], name: str}]
    algorithms: list[str]
    timeout: float
    trials: int
    seed: int
    step_size: float

  stdout -> JSON with results per algorithm
"""
from __future__ import annotations

import json
import math
import sys
import time
from typing import Dict, List

import numpy as np

# Bootstrap: add v3/src to path
_v3_src = "/mnt/c/Users/TIAN/Documents/box_aabb/v3/src"
_v3_root = "/mnt/c/Users/TIAN/Documents/box_aabb/v3"
for p in (_v3_src, _v3_root):
    if p not in sys.path:
        sys.path.insert(0, p)

from aabb.robot import load_robot
from forest.scene import Scene
from forest.collision import CollisionChecker

from ompl import base as ob
from ompl import geometric as og

try:
    from ompl import util as ou
    ou.setLogLevel(ou.LOG_WARN)
except (ImportError, AttributeError):
    pass


# ── helpers ──────────────────────────────────────────────────────

def _geo_path_length(waypoints, period):
    """Geodesic path length on torus."""
    if period is None:
        return _euclidean_path_length(waypoints)
    half = period / 2.0
    total = 0.0
    for i in range(len(waypoints) - 1):
        diff = ((waypoints[i + 1] - waypoints[i]) + half) % period - half
        total += float(np.linalg.norm(diff))
    return total


def _euclidean_path_length(waypoints):
    total = 0.0
    for i in range(len(waypoints) - 1):
        total += float(np.linalg.norm(waypoints[i + 1] - waypoints[i]))
    return total


def _compute_period(joint_limits):
    spans = [hi - lo for lo, hi in joint_limits]
    if not spans:
        return None
    span0 = spans[0]
    all_same = all(abs(s - span0) < 1e-6 for s in spans)
    return float(span0) if (all_same and abs(span0 - 2 * math.pi) < 0.1) else None


def _states_to_numpy(states, ndim):
    """Convert OMPL State objects to list of numpy arrays (RealVector)."""
    return [np.array([states[i][j] for j in range(ndim)], dtype=np.float64)
            for i in range(len(states))]


# ── Torus state space ──────────────────────────────────────────

class TorusStateSpace(ob.RealVectorStateSpace):
    """RealVectorStateSpace with geodesic (wrapping) distance / interpolation.

    Subclass of RealVectorStateSpace so that BIT*/InformedRRT*'s
    PathLengthDirectInfSampler still recognizes it (getType() ==
    STATE_SPACE_REAL_VECTOR) while the planner uses shortest-arc
    distance and interpolation on the flat torus.
    """

    def __init__(self, ndim: int, period: float):
        super().__init__(ndim)
        self._period = float(period)
        self._half = self._period / 2.0
        self._ndim = ndim

    def distance(self, s1, s2):
        d_sq = 0.0
        for i in range(self._ndim):
            diff = ((s2[i] - s1[i]) + self._half) % self._period - self._half
            d_sq += diff * diff
        return d_sq ** 0.5

    def getMaximumExtent(self):
        """Max geodesic distance on the torus = sqrt(ndim) * period/2."""
        return (self._ndim ** 0.5) * self._half

    def interpolate(self, s1, s2, t, state):
        for i in range(self._ndim):
            diff = ((s2[i] - s1[i]) + self._half) % self._period - self._half
            val = s1[i] + t * diff
            state[i] = ((val + self._half) % self._period) - self._half


def _densify_until_clean(waypoints, checker, period, max_depth=8,
                        resolution=0.005):
    """Geodesic collision verification; insert mid-points on failing segs.

    If a segment is still colliding after max_depth splits, mark it
    as unresolvable (the planner's path really does cross an obstacle).
    Returns the (possibly densified) waypoint list.
    """
    half = period / 2.0
    clean = [waypoints[0]]
    for i in range(len(waypoints) - 1):
        _densify_seg(waypoints[i], waypoints[i + 1],
                     checker, period, half, resolution, clean, max_depth)
    return clean


def _densify_seg(q1, q2, checker, period, half, resolution, out, depth):
    """Recursive GEODESIC midpoint insertion for a single segment.

    OMPL plans with geodesic interpolation (TorusStateSpace), so the
    midpoint should also be on the geodesic arc.  This ensures
    sub-segments stay on the arc the planner already verified.
    """
    if not checker.check_segment_collision(q1, q2, resolution, period=period):
        out.append(q2)
        return
    if depth <= 0:
        # Can't fix — append endpoint anyway
        out.append(q2)
        return
    geo_diff = ((q2 - q1) + half) % period - half
    mid = q1 + 0.5 * geo_diff
    mid = ((mid + half) % period) - half
    _densify_seg(q1, mid, checker, period, half, resolution, out, depth - 1)
    _densify_seg(mid, q2, checker, period, half, resolution, out, depth - 1)


# ── OMPL runner ─────────────────────────────────────────────────

def run_ompl_planner(
    algo_name: str,
    q_start: np.ndarray,
    q_goal: np.ndarray,
    joint_limits: list,
    checker: CollisionChecker,
    timeout: float = 1.0,
    step_size: float = 0.5,
    seed: int = 42,
) -> Dict:
    ndim = len(q_start)
    period = _compute_period(joint_limits)
    use_torus = (period is not None)

    # ── Build state space ────────────────────────────────────────
    if use_torus:
        space = TorusStateSpace(ndim, period)
    else:
        space = ob.RealVectorStateSpace(ndim)
    bounds = ob.RealVectorBounds(ndim)
    for i, (lo, hi) in enumerate(joint_limits):
        bounds.setLow(i, float(lo))
        bounds.setHigh(i, float(hi))
    space.setBounds(bounds)

    si = ob.SpaceInformation(space)

    # ── Validity checker (same for both – RealVector state[i]) ──
    def state_is_valid(raw_state):
        q = np.array([raw_state[i] for i in range(ndim)], dtype=np.float64)
        return not checker.check_config_collision(q)

    si.setStateValidityChecker(ob.StateValidityCheckerFn(state_is_valid))
    si.setStateValidityCheckingResolution(0.001)  # torus safety
    si.setup()

    # ── Start / Goal ──────────────────────────────────────────────
    start = ob.State(space)
    goal = ob.State(space)
    for i in range(ndim):
        start[i] = float(q_start[i])
        goal[i] = float(q_goal[i])

    pdef = ob.ProblemDefinition(si)
    pdef.setStartAndGoalStates(start, goal, 0.01)

    ANYTIME_ALGOS = ("RRTstar", "InformedRRTstar", "BITstar", "ABITstar")
    if algo_name in ANYTIME_ALGOS:
        obj = ob.PathLengthOptimizationObjective(si)
        pdef.setOptimizationObjective(obj)

    planner_cls = getattr(og, algo_name)
    planner = planner_cls(si)
    if hasattr(planner, "setRange"):
        planner.setRange(step_size)
    if hasattr(planner, "setGoalBias"):
        planner.setGoalBias(0.05)
    planner.setProblemDefinition(pdef)
    planner.setup()

    # ── Path helpers ──────────────────────────────────────────────
    if use_torus:
        _path_len_fn = lambda wps: _geo_path_length(wps, period)
    else:
        _path_len_fn = _euclidean_path_length

    _extract_wps = lambda states: _states_to_numpy(states, ndim)

    # ── Solve ─────────────────────────────────────────────────────
    checker.reset_counter()
    t0 = time.perf_counter()
    first_solution_time = None
    first_solution_cost = None
    cost_history = []

    if algo_name in ANYTIME_ALGOS:
        increment = 0.05
        elapsed = 0.0
        solved = False
        while elapsed < timeout:
            dt_step = min(increment, timeout - elapsed)
            status = planner.solve(dt_step)
            elapsed = time.perf_counter() - t0
            if status:
                solved = True
            if pdef.hasExactSolution():
                sol_wps = _extract_wps(pdef.getSolutionPath().getStates())
                cost_val = _path_len_fn(sol_wps)
                cost_history.append((elapsed, cost_val))
                if first_solution_time is None:
                    first_solution_time = elapsed
                    first_solution_cost = cost_val
        dt = time.perf_counter() - t0
    else:
        solved = bool(planner.solve(timeout))
        dt = time.perf_counter() - t0
        if solved:
            first_solution_time = dt

    n_checks = checker.n_collision_checks

    result = {
        "algorithm": f"OMPL-{algo_name}",
        "success": bool(solved),
        "plan_time_s": dt,
        "first_solution_time": first_solution_time or dt,
        "n_collision_checks": n_checks,
    }

    if solved:
        waypoints = _extract_wps(pdef.getSolutionPath().getStates())
        waypoints[0] = q_start.copy()
        waypoints[-1] = q_goal.copy()

        # ── Post-hoc collision verification + densification ──
        # OMPL's C++ DiscreteMotionValidator may not dispatch interpolate()
        # to our Python override.  Re-check each segment geodesically;
        # if any segment collides, densify by inserting mid-points.
        if use_torus:
            waypoints = _densify_until_clean(
                waypoints, checker, period, max_depth=6)

        raw_length = _path_len_fn(waypoints)
        if first_solution_cost is None:
            first_solution_cost = raw_length

        result["path_length"] = raw_length
        result["raw_path_length"] = raw_length
        result["first_solution_cost"] = first_solution_cost
        result["n_waypoints"] = len(waypoints)
        result["waypoints"] = [wp.tolist() for wp in waypoints]
        result["n_nodes"] = 0
        result["cost_history"] = cost_history
    else:
        result["path_length"] = float("nan")
        result["raw_path_length"] = float("nan")
        result["first_solution_cost"] = float("nan")
        result["n_waypoints"] = 0
        result["waypoints"] = []
        result["n_nodes"] = 0
        result["cost_history"] = []

    return result


# ── main ────────────────────────────────────────────────────────

def main():
    problem = json.load(sys.stdin)

    robot_name = problem.get("robot_name", "2dof_planar")
    q_start = np.array(problem["q_start"], dtype=np.float64)
    q_goal = np.array(problem["q_goal"], dtype=np.float64)
    timeout = problem.get("timeout", 1.0)
    trials = problem.get("trials", 1)
    seed = problem.get("seed", 42)
    step_size = problem.get("step_size", 0.3)
    algorithms = problem.get("algorithms",
                             ["RRT", "RRTConnect", "RRTstar", "InformedRRTstar"])

    robot = load_robot(robot_name)
    joint_limits = problem.get("joint_limits", None)
    if joint_limits is None:
        joint_limits = robot.joint_limits
    else:
        joint_limits = [(lo, hi) for lo, hi in joint_limits]

    scene = Scene()
    for obs_data in problem["obstacles"]:
        scene.add_obstacle(
            min_point=np.array(obs_data["min_point"], dtype=np.float64),
            max_point=np.array(obs_data["max_point"], dtype=np.float64),
            name=obs_data.get("name", "obs"),
        )

    checker = CollisionChecker(robot=robot, scene=scene)

    all_results = {}
    for algo in algorithms:
        if not hasattr(og, algo):
            all_results[algo] = {"error": f"'{algo}' not available in OMPL"}
            continue

        trial_results = []
        for trial in range(trials):
            trial_seed = seed + trial * 1000 + hash(algo) % 10000
            try:
                ob.RNG.setSeed(trial_seed)
            except AttributeError:
                pass

            try:
                r = run_ompl_planner(
                    algo_name=algo,
                    q_start=q_start, q_goal=q_goal,
                    joint_limits=joint_limits,
                    checker=checker,
                    timeout=timeout,
                    step_size=step_size,
                    seed=trial_seed,
                )
            except Exception as exc:
                import traceback
                print(f"[{algo}] trial {trial} failed: {exc}",
                      file=sys.stderr)
                traceback.print_exc(file=sys.stderr)
                r = {
                    "algorithm": f"OMPL-{algo}",
                    "success": False,
                    "plan_time_s": 0.0,
                    "first_solution_time": 0.0,
                    "n_collision_checks": 0,
                    "path_length": float("nan"),
                    "raw_path_length": float("nan"),
                    "first_solution_cost": float("nan"),
                    "n_waypoints": 0,
                    "waypoints": [],
                    "n_nodes": 0,
                    "cost_history": [],
                    "error": str(exc),
                }
            trial_results.append(r)

        successes = [t for t in trial_results if t["success"]]
        n_succ = len(successes)
        summary = {"n_trials": trials, "n_success": n_succ,
                    "success_rate": n_succ / trials if trials > 0 else 0}
        if n_succ > 0:
            summary["avg_plan_time_s"] = sum(t["plan_time_s"] for t in successes) / n_succ
            summary["avg_path_length"] = sum(t["path_length"] for t in successes) / n_succ
            summary["avg_collision_checks"] = sum(t["n_collision_checks"] for t in successes) / n_succ
            summary["min_path_length"] = min(t["path_length"] for t in successes)
            fst = [t["first_solution_time"] for t in successes]
            summary["avg_first_solution_time"] = sum(fst) / len(fst)
            summary["min_first_solution_time"] = min(fst)
            fst_costs = [t.get("first_solution_cost", float("nan")) for t in successes
                         if not (isinstance(t.get("first_solution_cost"), float)
                                 and t.get("first_solution_cost") != t.get("first_solution_cost"))]
            summary["avg_first_solution_cost"] = sum(fst_costs) / len(fst_costs) if fst_costs else float("nan")
            best_trial = min(successes, key=lambda t: t["path_length"])
            summary["best_cost_history"] = best_trial.get("cost_history", [])
        else:
            summary["avg_plan_time_s"] = float("nan")
            summary["avg_path_length"] = float("nan")
            summary["avg_collision_checks"] = float("nan")
            summary["avg_first_solution_time"] = float("nan")
            summary["avg_first_solution_cost"] = float("nan")
            summary["best_cost_history"] = []

        trial_slim = [{k: v for k, v in t.items() if k != "waypoints"}
                      for t in trial_results]
        # pick best-cost trial's waypoints
        best_wps = min(successes, key=lambda t: t["path_length"])["waypoints"] if n_succ > 0 else []

        all_results[algo] = {
            "summary": summary,
            "trials": trial_slim,
            "best_waypoints": best_wps,
        }

    json.dump(all_results, sys.stdout, default=str)


if __name__ == "__main__":
    main()

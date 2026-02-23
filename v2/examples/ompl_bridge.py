#!/usr/bin/env python3
"""
ompl_bridge.py -- OMPL planner bridge (runs in WSL Linux)

Imports the project's actual Robot + CollisionChecker from the Windows filesystem
mounted at /mnt/c/..., ensuring identical collision checking as the pure-Python
RRT family.

Protocol:
  stdin  -> JSON with keys:
    q_start, q_goal: list[float]
    obstacles: list[{min_point: [x,y,z], max_point: [x,y,z], name: str}]
    algorithms: list[str]     e.g. ["RRT","RRTConnect","RRTstar","InformedRRTstar","BITstar"]
    timeout: float (seconds per algorithm)
    trials: int
    seed: int
    step_size: float

  stdout -> JSON with results per algorithm
"""

from __future__ import annotations

import json
import sys
import time
from typing import Dict, List

import numpy as np

# ---- Bootstrap: add v2/src to path so we can import the project ----
_v2_src = "/mnt/c/Users/TIAN/Documents/box_aabb/v2/src"
_v2_root = "/mnt/c/Users/TIAN/Documents/box_aabb/v2"
if _v2_src not in sys.path:
    sys.path.insert(0, _v2_src)
if _v2_root not in sys.path:
    sys.path.insert(0, _v2_root)

from aabb.robot import load_robot
from forest.scene import Scene
from forest.collision import CollisionChecker

# ---- OMPL ----
from ompl import base as ob
from ompl import geometric as og

# Suppress OMPL Info/Debug messages (they go to stdout/stderr and pollute JSON)
try:
    from ompl import util as ou
    ou.setLogLevel(ou.LOG_WARN)
except (ImportError, AttributeError):
    pass


# =====================================================================
# OMPL planner wrapper
# =====================================================================

def path_length(waypoints: List[np.ndarray]) -> float:
    total = 0.0
    for i in range(len(waypoints) - 1):
        total += float(np.linalg.norm(waypoints[i + 1] - waypoints[i]))
    return total


def run_ompl_planner(
    algo_name: str,
    q_start: np.ndarray,
    q_goal: np.ndarray,
    joint_limits: List,
    checker: CollisionChecker,
    timeout: float = 1.0,
    step_size: float = 0.5,
    seed: int = 42,
    active_dims: List[int] = None,
) -> Dict:
    """Run a single OMPL planner and return result dict.

    If *active_dims* is given (e.g. [0,1,2,3,4,5] for Panda), planning
    is done in this reduced subspace.  Inactive joints are fixed at
    ``q_start`` values for collision checking and linearly interpolated
    from ``q_start`` to ``q_goal`` when reconstructing the full path.
    """
    full_ndim = len(q_start)

    # ── Reduced-dimension handling ──
    if active_dims is not None and len(active_dims) < full_ndim:
        plan_dims = list(active_dims)
        inactive_dims = [d for d in range(full_ndim) if d not in plan_dims]
        ndim = len(plan_dims)
        # Reduced vectors
        q_start_r = np.array([q_start[d] for d in plan_dims], dtype=np.float64)
        q_goal_r = np.array([q_goal[d] for d in plan_dims], dtype=np.float64)
        jl_r = [joint_limits[d] for d in plan_dims]
        # Baseline config for inactive dims (use q_start — collision-invariant)
        q_base = q_start.copy()
    else:
        plan_dims = None
        inactive_dims = None
        ndim = full_ndim
        q_start_r = q_start
        q_goal_r = q_goal
        jl_r = joint_limits
        q_base = None

    # -- State space: RealVector --
    space = ob.RealVectorStateSpace(ndim)
    bounds = ob.RealVectorBounds(ndim)
    for i, (lo, hi) in enumerate(jl_r):
        bounds.setLow(i, float(lo))
        bounds.setHigh(i, float(hi))
    space.setBounds(bounds)

    # -- Space information + validity checker --
    si = ob.SpaceInformation(space)

    if plan_dims is not None:
        def state_is_valid(state):
            q_full = q_base.copy()
            for i, d in enumerate(plan_dims):
                q_full[d] = state[i]
            return not checker.check_config_collision(q_full)
    else:
        def state_is_valid(state):
            q = np.array([state[i] for i in range(ndim)], dtype=np.float64)
            return not checker.check_config_collision(q)

    si.setStateValidityChecker(ob.StateValidityCheckerFn(state_is_valid))
    si.setStateValidityCheckingResolution(0.01)
    si.setup()

    # -- Problem definition --
    start = ob.State(space)
    goal = ob.State(space)
    for i in range(ndim):
        start[i] = float(q_start_r[i])
        goal[i] = float(q_goal_r[i])

    pdef = ob.ProblemDefinition(si)
    pdef.setStartAndGoalStates(start, goal, 0.1)  # goal region tolerance

    # Optimization objective for asymptotically-optimal planners
    ANYTIME_ALGOS = ("RRTstar", "InformedRRTstar", "BITstar", "ABITstar")
    if algo_name in ANYTIME_ALGOS:
        obj = ob.PathLengthOptimizationObjective(si)
        pdef.setOptimizationObjective(obj)

    # -- Planner --
    planner_cls = getattr(og, algo_name)
    planner = planner_cls(si)

    if hasattr(planner, "setRange"):
        planner.setRange(step_size)
    if hasattr(planner, "setGoalBias"):
        planner.setGoalBias(0.05)

    planner.setProblemDefinition(pdef)
    planner.setup()

    # -- Solve (with first-solution tracking for anytime planners) --
    checker.reset_counter()
    t0 = time.perf_counter()
    first_solution_time = None
    first_solution_cost = None  # path length when first solution found
    cost_history = []  # list of (elapsed_s, cost)

    # Use OMPL's sol_path.length() as the SOLE cost metric everywhere.
    # For anytime planners, OMPL guarantees cost monotonically decreases
    # ONLY AFTER an exact solution (within goal tolerance). Before that,
    # approximate solutions can have INCREASING cost as the tree extends.
    # So we gate cost tracking with hasExactSolution().
    #
    # For single-shot planners (RRT, RRTConnect), first=final always,
    # so we accept any solution (planner.solve() return value).

    if algo_name in ANYTIME_ALGOS:
        # Solve in small increments to track first solution + cost evolution
        increment = 0.05  # 50ms
        elapsed = 0.0
        solved = False
        while elapsed < timeout:
            dt_step = min(increment, timeout - elapsed)
            status = planner.solve(dt_step)
            elapsed = time.perf_counter() - t0
            if status:
                solved = True  # any solution (exact or approximate)
            # Track cost only after exact solution (monotonicity guaranteed)
            if pdef.hasExactSolution():
                sol_path = pdef.getSolutionPath()
                cost_val = sol_path.length()
                cost_history.append((elapsed, cost_val))
                if first_solution_time is None:
                    first_solution_time = elapsed
                    first_solution_cost = cost_val
        dt = time.perf_counter() - t0
    else:
        # Single-shot planners (RRT, RRTConnect): solve once
        solved = bool(planner.solve(timeout))
        dt = time.perf_counter() - t0
        if solved:
            first_solution_time = dt

    n_checks = checker.n_collision_checks

    result = {
        "algorithm": f"OMPL-{algo_name}",
        "success": bool(solved),
        "plan_time_s": dt,
        "first_solution_time": first_solution_time if first_solution_time else dt,
        "n_collision_checks": n_checks,
        "exact": str(solved) if solved else "Failed",
        "cost_history": cost_history,
    }

    if solved:
        path = pdef.getSolutionPath()

        # Use OMPL's .length() as the consistent path cost
        raw_length = path.length()

        # For single-shot planners, first solution cost = final cost
        if first_solution_cost is None:
            first_solution_cost = raw_length
        result["first_solution_cost"] = first_solution_cost

        # Extract waypoints for output / visualization
        states = path.getStates()
        raw_waypoints = [np.array([states[i][j] for j in range(ndim)])
                         for i in range(len(states))]

        # Reconstruct full-dim waypoints if planning in reduced subspace
        if plan_dims is not None:
            n_wp = len(raw_waypoints)
            waypoints = []
            for wi, rw in enumerate(raw_waypoints):
                q_full = np.empty(full_ndim, dtype=np.float64)
                for ri, d in enumerate(plan_dims):
                    q_full[d] = rw[ri]
                # Linearly interpolate inactive dims from q_start to q_goal
                t = wi / max(n_wp - 1, 1)
                for d in inactive_dims:
                    q_full[d] = q_start[d] + t * (q_goal[d] - q_start[d])
                waypoints.append(q_full)
        else:
            waypoints = raw_waypoints

        # Pin endpoints for visualization
        waypoints[0] = q_start.copy()
        waypoints[-1] = q_goal.copy()

        # Recompute path length in full space for fairness
        full_path_length = sum(
            float(np.linalg.norm(waypoints[i + 1] - waypoints[i]))
            for i in range(len(waypoints) - 1))

        result["path_length"] = full_path_length
        result["raw_path_length"] = raw_length  # OMPL's reduced-space length
        # Update first_solution_cost to full-space for single-shot planners
        if first_solution_cost == raw_length:
            result["first_solution_cost"] = full_path_length
        result["n_waypoints"] = len(waypoints)
        result["raw_n_waypoints"] = len(waypoints)
        result["waypoints"] = [wp.tolist() for wp in waypoints]
        result["n_nodes"] = 0  # OMPL doesn't easily expose tree size
    else:
        result["path_length"] = float("nan")
        result["raw_path_length"] = float("nan")
        result["first_solution_cost"] = float("nan")
        result["n_waypoints"] = 0
        result["raw_n_waypoints"] = 0
        result["waypoints"] = []
        result["n_nodes"] = 0

    return result


# =====================================================================
# Main
# =====================================================================

def main():
    problem = json.load(sys.stdin)

    q_start = np.array(problem["q_start"], dtype=np.float64)
    q_goal = np.array(problem["q_goal"], dtype=np.float64)
    timeout = problem.get("timeout", 1.0)
    trials = problem.get("trials", 3)
    seed = problem.get("seed", 42)
    step_size = problem.get("step_size", 0.5)
    algorithms = problem.get("algorithms",
                             ["RRT", "RRTConnect", "RRTstar", "InformedRRTstar"])

    # Build robot + scene from our actual project code
    robot = load_robot("panda")

    scene = Scene()
    for obs_data in problem["obstacles"]:
        scene.add_obstacle(
            min_point=np.array(obs_data["min_point"], dtype=np.float64),
            max_point=np.array(obs_data["max_point"], dtype=np.float64),
            name=obs_data.get("name", "obs"),
        )

    checker = CollisionChecker(robot=robot, scene=scene)
    joint_limits = robot.joint_limits

    # Determine active planning dimensions (auto-detect from robot if not provided)
    active_dims = problem.get("active_dims", None)
    if active_dims is None:
        # Auto-detect: find joints that affect at least one link AABB
        n_joints = len(robot.dh_params)
        has_tool = robot.tool_frame is not None
        n_links = n_joints + (1 if has_tool else 0)
        relevant = set()
        try:
            for link_idx in range(1, n_links + 1):
                for joint_idx in robot.compute_relevant_joints(link_idx):
                    if 0 <= int(joint_idx) < n_joints:
                        relevant.add(int(joint_idx))
        except Exception:
            relevant = set()
        if relevant and len(relevant) < n_joints:
            active_dims = sorted(relevant)
            import sys as _sys
            print(f"auto-detected active_dims={active_dims} "
                  f"(planning in {len(active_dims)}D instead of {n_joints}D)",
                  file=_sys.stderr)
        else:
            active_dims = None  # plan in full dimension

    all_results = {}
    for algo in algorithms:
        if not hasattr(og, algo):
            all_results[algo] = {"error": f"Planner '{algo}' not available"}
            continue

        trial_results = []
        for trial in range(trials):
            trial_seed = seed + trial * 1000 + hash(algo) % 10000
            # OMPL Python bindings may not expose RNG.setSeed()
            try:
                ob.RNG.setSeed(trial_seed)
            except AttributeError:
                pass

            r = run_ompl_planner(
                algo_name=algo,
                q_start=q_start, q_goal=q_goal,
                joint_limits=joint_limits,
                checker=checker,
                timeout=timeout,
                step_size=step_size,
                seed=trial_seed,
                active_dims=active_dims,
            )
            trial_results.append(r)

        # Summary
        successes = [t for t in trial_results if t["success"]]
        n_succ = len(successes)
        summary = {
            "n_trials": trials,
            "n_success": n_succ,
            "success_rate": n_succ / trials if trials > 0 else 0,
        }
        if n_succ > 0:
            summary["avg_plan_time_s"] = sum(t["plan_time_s"] for t in successes) / n_succ
            summary["avg_path_length"] = sum(t["path_length"] for t in successes) / n_succ
            summary["avg_raw_path_length"] = sum(t["raw_path_length"] for t in successes) / n_succ
            summary["avg_collision_checks"] = sum(t["n_collision_checks"] for t in successes) / n_succ
            summary["min_path_length"] = min(t["path_length"] for t in successes)
            # First solution time stats
            fst_times = [t["first_solution_time"] for t in successes if t["first_solution_time"] is not None]
            if fst_times:
                summary["avg_first_solution_time"] = sum(fst_times) / len(fst_times)
                summary["min_first_solution_time"] = min(fst_times)
            else:
                summary["avg_first_solution_time"] = summary["avg_plan_time_s"]
                summary["min_first_solution_time"] = summary["avg_plan_time_s"]
            # First solution cost stats
            fst_costs = [t["first_solution_cost"] for t in successes
                         if t.get("first_solution_cost") is not None
                         and not (isinstance(t["first_solution_cost"], float)
                                  and t["first_solution_cost"] != t["first_solution_cost"])]
            if fst_costs:
                summary["avg_first_solution_cost"] = sum(fst_costs) / len(fst_costs)
            else:
                summary["avg_first_solution_cost"] = summary["avg_path_length"]
            # Best cost history (from trial with shortest final path)
            best_trial = min(successes, key=lambda t: t["path_length"])
            summary["best_cost_history"] = best_trial.get("cost_history", [])
        else:
            for k in ("avg_plan_time_s", "avg_path_length", "avg_raw_path_length",
                       "avg_collision_checks", "min_path_length",
                       "avg_first_solution_time", "min_first_solution_time"):
                summary[k] = float("nan")
            summary["avg_first_solution_cost"] = float("nan")
            summary["best_cost_history"] = []

        trial_results_slim = []
        for t in trial_results:
            t_slim = {k: v for k, v in t.items() if k != "waypoints"}
            trial_results_slim.append(t_slim)

        all_results[algo] = {
            "summary": summary,
            "trials": trial_results_slim,
            "best_waypoints": successes[0]["waypoints"] if n_succ > 0 else [],
        }

    json.dump(all_results, sys.stdout, default=str)


if __name__ == "__main__":
    main()

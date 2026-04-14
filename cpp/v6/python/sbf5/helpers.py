"""Convenience helpers for SafeBoxForest v5."""
import numpy as np
from sbf5 import Robot, SBFPlanner, SBFPlannerConfig, Obstacle


def quick_plan(robot_json: str, start, goal, obstacles_list, **kwargs):
    """One-line planning convenience function.

    obstacles_list: list of dict {"lo": [lx,ly,lz], "hi": [hx,hy,hz]}
    """
    robot = Robot.from_json(robot_json)
    planner = SBFPlanner(robot)
    obs = []
    for d in obstacles_list:
        lo = d["lo"]
        hi = d["hi"]
        obs.append(Obstacle(lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]))
    return planner.plan(np.asarray(start, dtype=np.float64),
                        np.asarray(goal, dtype=np.float64),
                        obs, **kwargs)

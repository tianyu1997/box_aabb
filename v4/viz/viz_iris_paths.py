#!/usr/bin/env python3
"""
viz_iris_paths.py — IRIS-GCS 路径可视化 + 碰撞检测

1. 加载 IRIS.reg regions → LinearGCS → SolvePath
2. C++ CollisionChecker 碰撞检测
3. Drake Meshcat 可视化 + HTML 导出

Usage:
    python viz/viz_iris_paths.py                          # 动画模式
    python viz/viz_iris_paths.py --save iris_paths.html   # 导出 HTML
    python viz/viz_iris_paths.py --skip-collision          # 跳过碰撞检测
"""

import argparse
import os
import pickle
import sys
import time

import numpy as np

# ── 路径设置 ──
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJ_ROOT = os.path.dirname(_THIS_DIR)                     # v4/
_REPO_ROOT = os.path.dirname(_PROJ_ROOT)                    # box_aabb/
_GCS_ROOT = os.path.join(_REPO_ROOT, "gcs-science-robotics")
_MODELS_DIR = os.path.join(_GCS_ROOT, "models")
_YAML_FILE = os.path.join(_MODELS_DIR, "iiwa14_welded_gripper.yaml")
_IRIS_REG = os.path.join(_GCS_ROOT, "data", "prm_comparison", "IRIS.reg")

sys.path.insert(0, os.path.join(_REPO_ROOT, "cpp", "build", "python"))
sys.path.insert(0, os.path.join(_PROJ_ROOT, "experiments"))
sys.path.insert(0, os.path.join(_PROJ_ROOT, "src"))

# GCS lib
_gcs_dir = os.path.join(_GCS_ROOT)
if _gcs_dir not in sys.path:
    sys.path.insert(0, _gcs_dir)

from pydrake.geometry import (
    MeshcatVisualizer, MeshcatVisualizerParams, Role, Rgba, StartMeshcat,
)
from pydrake.multibody.parsing import (
    LoadModelDirectives, Parser, ProcessModelDirectives,
)
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.multibody.tree import RevoluteJoint
from pydrake.perception import PointCloud
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import TrajectorySource
from pydrake.systems.rendering import MultibodyPositionToGeometryPose
from pydrake.trajectories import PiecewisePolynomial

PAIR_NAMES = ["AS→TS", "TS→CS", "CS→LB", "LB→RB", "RB→AS"]
PAIR_COLORS = [
    Rgba(0.12, 0.47, 0.71, 1.0),  # 蓝
    Rgba(1.00, 0.50, 0.05, 1.0),  # 橙
    Rgba(0.17, 0.63, 0.17, 1.0),  # 绿
    Rgba(0.84, 0.15, 0.16, 1.0),  # 红
    Rgba(0.58, 0.40, 0.74, 1.0),  # 紫
]


# ═══════════════════════════════════════════════════════════════════════════
# 1. IRIS 规划
# ═══════════════════════════════════════════════════════════════════════════

def run_iris_planning(seed=0):
    """加载 IRIS regions, 对5个pair做GCS规划, 返回路径列表."""
    from gcs.linear import LinearGCS
    from gcs.rounding import randomForwardPathSearch
    from pydrake.solvers import MosekSolver, SolverOptions
    from marcucci_scenes import get_query_pairs

    # 加载 IRIS regions
    with open(_IRIS_REG, "rb") as f:
        regions = pickle.load(f)
    region_list = list(regions.values()) if isinstance(regions, dict) else regions
    region_names = list(regions.keys()) if isinstance(regions, dict) else [f"r{i}" for i in range(len(region_list))]
    print(f"  IRIS regions: {len(region_list)} ({region_names})")

    pairs = get_query_pairs("combined")
    results = []

    for pi, (q_start, q_goal) in enumerate(pairs):
        print(f"  [{PAIR_NAMES[pi]}] solving ...", end="", flush=True)

        gcs = LinearGCS(region_list, edges=None)
        try:
            gcs.addSourceTarget(q_start, q_goal)
        except ValueError as e:
            print(f" FAIL (addSourceTarget: {e})")
            results.append({"pair_idx": pi, "pair_name": PAIR_NAMES[pi],
                            "success": False, "waypoints": None})
            continue

        gcs.setRoundingStrategy(randomForwardPathSearch,
                                max_paths=10, max_trials=100, seed=seed)
        gcs.setSolver(MosekSolver())
        opts = SolverOptions()
        opts.SetOption(MosekSolver.id(), "MSK_DPAR_INTPNT_CO_TOL_REL_GAP", 1e-3)
        gcs.setSolverOptions(opts)

        t0 = time.perf_counter()
        try:
            waypoints, results_dict = gcs.SolvePath(
                rounding=True, verbose=False, preprocessing=True)
        except Exception as e:
            print(f" FAIL ({e})")
            results.append({"pair_idx": pi, "pair_name": PAIR_NAMES[pi],
                            "success": False, "waypoints": None})
            continue

        solve_time = time.perf_counter() - t0

        if waypoints is None:
            print(f" FAIL (solver returned None)")
            results.append({"pair_idx": pi, "pair_name": PAIR_NAMES[pi],
                            "success": False, "waypoints": None})
            continue

        path = waypoints.T  # (n_pts, D)
        path_len = float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))
        print(f" OK  solve={solve_time*1000:.1f}ms  len={path_len:.3f}  "
              f"wp={path.shape[0]}")
        results.append({
            "pair_idx": pi, "pair_name": PAIR_NAMES[pi],
            "success": True, "waypoints": path,
            "path_length": path_len, "solve_time": solve_time,
        })

    return results


# ═══════════════════════════════════════════════════════════════════════════
# 2. 碰撞检测
# ═══════════════════════════════════════════════════════════════════════════

def check_path_collision(waypoints, n_interp=200):
    """用 C++ SBF 碰撞检测器检查路径."""
    import pysbf
    from marcucci_scenes import build_combined_obstacles

    robot = pysbf.Robot.from_json(
        os.path.join(_REPO_ROOT, "cpp", "configs", "iiwa14.json"))
    obs_dicts = build_combined_obstacles()
    obstacles = []
    for o in obs_dicts:
        obs = pysbf.Obstacle()
        obs.name = o["name"]
        lo = np.array(o["min"], dtype=np.float64)
        hi = np.array(o["max"], dtype=np.float64)
        obs.center = 0.5 * (lo + hi)
        obs.half_sizes = 0.5 * (hi - lo)
        obstacles.append(obs)
    checker = pysbf.CollisionChecker(robot, obstacles)

    wp = np.asarray(waypoints)
    report = {"n_waypoints": len(wp), "collision_free": True,
              "n_collisions": 0, "details": []}

    for i in range(len(wp) - 1):
        seg_col = False
        col_t = []
        for t_idx in range(n_interp + 1):
            t = t_idx / n_interp
            q = (1 - t) * wp[i] + t * wp[i + 1]
            if checker.check_config(q.tolist()):
                seg_col = True
                col_t.append(t)
        if seg_col:
            report["collision_free"] = False
            report["n_collisions"] += len(col_t)
            report["details"].append({
                "seg": f"{i}→{i+1}",
                "n_col": len(col_t),
                "first_t": col_t[0],
                "last_t": col_t[-1],
            })

    return report


# ═══════════════════════════════════════════════════════════════════════════
# 3. Drake 可视化
# ═══════════════════════════════════════════════════════════════════════════

def waypoints_to_trajectory(waypoints, speed=1.0):
    wp = np.asarray(waypoints, dtype=float)
    dists = np.linalg.norm(np.diff(wp, axis=0), axis=1)
    dists = np.maximum(dists, 1e-6)
    cum = np.concatenate([[0], np.cumsum(dists)])
    breaks = cum / speed
    return PiecewisePolynomial.FirstOrderHold(breaks, wp.T)


def visualize_paths(meshcat, path_list, speed=1.0):
    """动画播放多条路径, 画末端轨迹点云."""
    from pydrake.geometry import SceneGraph, Sphere
    from pydrake.math import RigidTransform

    # 拼接所有轨迹
    trajectories = []
    t_offset = 0.0
    pause = 0.8
    for entry in path_list:
        if not entry["success"]:
            continue
        traj = waypoints_to_trajectory(entry["waypoints"], speed=speed)
        trajectories.append((t_offset, traj, entry["pair_idx"]))
        t_offset += traj.end_time() + pause
    total_time = t_offset - pause if trajectories else 0.0

    # 拼接采样
    n_samples = max(int(total_time * 100), 200)
    times = np.linspace(0, total_time, n_samples)

    def eval_t(t):
        for i, (offset, traj, _) in enumerate(trajectories):
            seg_end = offset + traj.end_time()
            if t <= seg_end or i == len(trajectories) - 1:
                lt = np.clip(t - offset, traj.start_time(), traj.end_time())
                return traj.value(lt).flatten()
        return trajectories[-1][1].value(trajectories[-1][1].end_time()).flatten()

    values = np.array([eval_t(t) for t in times]).T
    concat_traj = PiecewisePolynomial.FirstOrderHold(times, values)

    # 构建动画 diagram
    builder = DiagramBuilder()
    scene_graph = builder.AddSystem(SceneGraph())
    plant = MultibodyPlant(time_step=0.0)
    plant.RegisterAsSourceForSceneGraph(scene_graph)
    parser = Parser(plant)
    parser.package_map().Add("gcs", _GCS_ROOT)
    directives = LoadModelDirectives(_YAML_FILE)
    models = ProcessModelDirectives(directives, plant, parser)
    plant.Finalize()

    to_pose = builder.AddSystem(MultibodyPositionToGeometryPose(plant))
    builder.Connect(to_pose.get_output_port(),
                    scene_graph.get_source_pose_port(plant.get_source_id()))
    traj_source = builder.AddSystem(TrajectorySource(concat_traj))
    builder.Connect(traj_source.get_output_port(), to_pose.get_input_port())
    meshcat_viz = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    meshcat.Delete()

    vis_diagram = builder.Build()
    simulator = Simulator(vis_diagram)

    # 末端轨迹点云
    plant_ctx = plant.CreateDefaultContext()
    wsg_body = plant.GetBodyByName("body", models[1].model_instance)

    for offset, traj, pi in trajectories:
        n_pts = max(int(traj.end_time() * 200), 100)
        ts = np.linspace(traj.start_time(), traj.end_time(), n_pts)
        ee_pos = []
        for t in ts:
            q = traj.value(t).flatten()
            plant.SetPositions(plant_ctx, q)
            X_WE = plant.EvalBodyPoseInWorld(plant_ctx, wsg_body)
            ee_pos.append(X_WE.translation())
        ee_arr = np.array(ee_pos).T
        pc = PointCloud(n_pts)
        pc.mutable_xyzs()[:] = ee_arr
        color = PAIR_COLORS[pi % len(PAIR_COLORS)]
        meshcat.SetObject(f"paths/{PAIR_NAMES[pi]}", pc, 0.008, rgba=color)

    # waypoint 标记球
    for entry in path_list:
        if not entry["success"]:
            continue
        pi = entry["pair_idx"]
        wp = entry["waypoints"]
        for wi, q in enumerate(wp):
            plant.SetPositions(plant_ctx, q)
            X_WE = plant.EvalBodyPoseInWorld(plant_ctx, wsg_body)
            pos = X_WE.translation()
            if wi == 0:
                c = Rgba(0.12, 0.47, 0.71, 1.0)   # 蓝 = start
            elif wi == len(wp) - 1:
                c = Rgba(0.17, 0.63, 0.17, 1.0)   # 绿 = goal
            else:
                c = Rgba(0.84, 0.15, 0.16, 0.8)   # 红 = intermediate
            meshcat.SetObject(f"wp/{PAIR_NAMES[pi]}/wp{wi}",
                               Sphere(0.012), rgba=c)
            meshcat.SetTransform(f"wp/{PAIR_NAMES[pi]}/wp{wi}",
                                  RigidTransform(pos))

    # 录制动画
    meshcat_viz.StartRecording()
    simulator.AdvanceTo(total_time + 0.5)
    meshcat_viz.PublishRecording()

    return total_time


# ═══════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="IRIS-GCS 路径可视化 + 碰撞检测")
    parser.add_argument("--seed", type=int, default=0,
                        help="GCS rounding seed")
    parser.add_argument("--save", type=str, default=None,
                        help="导出 HTML 文件路径")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="动画速度")
    parser.add_argument("--skip-collision", action="store_true",
                        help="跳过碰撞检测")
    args = parser.parse_args()

    print("=" * 70)
    print("  IRIS-GCS 路径可视化")
    print("=" * 70)

    # ── 1. 规划 ──
    print("\n[1] IRIS-GCS 规划...")
    path_list = run_iris_planning(seed=args.seed)
    n_ok = sum(1 for e in path_list if e["success"])
    print(f"\n  成功: {n_ok}/{len(path_list)}")

    # ── 2. 碰撞检测 ──
    if not args.skip_collision:
        print(f"\n[2] 碰撞检测 (200 点/段)...")
        all_safe = True
        for entry in path_list:
            if not entry["success"]:
                print(f"  {entry['pair_name']:8s}  SKIP (规划失败)")
                continue
            report = check_path_collision(entry["waypoints"])
            status = "✅ SAFE" if report["collision_free"] else "❌ COLLISION"
            if report["collision_free"]:
                print(f"  {entry['pair_name']:8s}  {status}")
            else:
                all_safe = False
                for d in report["details"]:
                    print(f"  {entry['pair_name']:8s}  {status}  "
                          f"seg {d['seg']}: {d['n_col']} collisions "
                          f"(t={d['first_t']:.3f}..{d['last_t']:.3f})")
        summary = "✅ 全部无碰撞" if all_safe else "❌ 存在碰撞"
        print(f"\n  碰撞检测总结: {summary}")

    # ── 3. 可视化 ──
    ok_paths = [e for e in path_list if e["success"]]
    if not ok_paths:
        print("\n没有成功的路径可可视化")
        return

    print(f"\n[3] Meshcat 可视化 ({len(ok_paths)} 条路径)...")
    meshcat = StartMeshcat()
    print(f"  Meshcat URL: {meshcat.web_url()}")

    total_time = visualize_paths(meshcat, ok_paths, speed=args.speed)
    print(f"  动画时长: {total_time:.1f}s")

    # 导出
    if args.save:
        save_path = os.path.join(_PROJ_ROOT, "output", args.save)
        html = meshcat.StaticHtml()
        with open(save_path, "w") as f:
            f.write(html)
        print(f"  已导出 → {save_path}")

    print("\n按 Ctrl+C 退出")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n退出")


if __name__ == "__main__":
    main()

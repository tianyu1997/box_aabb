#!/usr/bin/env python3
"""
viz_ts_cs_path.py — TS→CS 路径碰撞检测 + Drake Meshcat 可视化

功能:
  1. C++ CollisionChecker 精细碰撞检测 (每段 200 点插值)
  2. Drake Meshcat 3D 场景: 场景 + 机器人动画 + 末端轨迹点云
  3. 沿路径显示机器人残影 (ghosting)
  4. 导出可交互 HTML

Usage:
    python viz/viz_ts_cs_path.py                  # seed=0, 动画模式
    python viz/viz_ts_cs_path.py --seed 7         # seed=7 (4个waypoints)
    python viz/viz_ts_cs_path.py --all-seeds      # 所有 seed 残影叠加
    python viz/viz_ts_cs_path.py --save ts_cs.html
"""

import argparse
import os
import pickle
import sys
import time

import numpy as np

# ── 路径设置 ──
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJ_ROOT = os.path.dirname(_THIS_DIR)
_GCS_ROOT = os.path.join(os.path.dirname(_PROJ_ROOT), "gcs-science-robotics")
_MODELS_DIR = os.path.join(_GCS_ROOT, "models")
_YAML_FILE = os.path.join(_MODELS_DIR, "iiwa14_welded_gripper.yaml")
_PATHS_CACHE = os.path.join(_PROJ_ROOT, "output", "exp2_paths", "sbf_paths.pkl")

sys.path.insert(0, os.path.join(os.path.dirname(_PROJ_ROOT), "cpp", "build", "python"))
sys.path.insert(0, os.path.join(_PROJ_ROOT, "experiments"))

from pydrake.geometry import (
    MeshcatVisualizer,
    MeshcatVisualizerParams,
    Role,
    Rgba,
    StartMeshcat,
)
from pydrake.multibody.parsing import (
    LoadModelDirectives,
    Parser,
    ProcessModelDirectives,
)
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.multibody.tree import RevoluteJoint
from pydrake.perception import PointCloud
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import TrajectorySource
from pydrake.systems.rendering import MultibodyPositionToGeometryPose
from pydrake.trajectories import PiecewisePolynomial


# ═══════════════════════════════════════════════════════════════════════════
# 1. 碰撞检测
# ═══════════════════════════════════════════════════════════════════════════

def check_path_collision(waypoints, n_interp=200):
    """用 C++ SBF 碰撞检测器检查路径, 返回详细报告."""
    import pysbf
    from marcucci_scenes import build_combined_obstacles

    robot = pysbf.Robot.from_json(
        os.path.join(os.path.dirname(_PROJ_ROOT), "cpp", "configs", "iiwa14.json"))
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

    report = {"waypoints": [], "segments": [], "collision_free": True}

    # 检查 waypoints
    for i in range(len(wp)):
        col = checker.check_config(wp[i].tolist())
        report["waypoints"].append({"idx": i, "collision": col, "q": wp[i].tolist()})
        if col:
            report["collision_free"] = False

    # 检查线段 (密集插值)
    for i in range(len(wp) - 1):
        seg_info = {"from": i, "to": i + 1,
                    "length": float(np.linalg.norm(wp[i + 1] - wp[i])),
                    "collision": False, "collision_points": []}
        for t_idx in range(n_interp + 1):
            t = t_idx / n_interp
            q = (1 - t) * wp[i] + t * wp[i + 1]
            if checker.check_config(q.tolist()):
                seg_info["collision"] = True
                seg_info["collision_points"].append({"t": t, "q": q.tolist()})
                report["collision_free"] = False
        report["segments"].append(seg_info)

    return report


def print_collision_report(report, seed):
    """打印碰撞报告."""
    status = "✅ SAFE" if report["collision_free"] else "❌ COLLISION"
    n_wp = len(report["waypoints"])
    print(f"\n  seed {seed}: {status}  ({n_wp} waypoints)")
    for wp_info in report["waypoints"]:
        i = wp_info["idx"]
        q = wp_info["q"]
        col_str = "❌" if wp_info["collision"] else "✅"
        print(f"    wp[{i}] = [{', '.join(f'{v:.4f}' for v in q)}]  {col_str}")
    for seg_info in report["segments"]:
        i, j = seg_info["from"], seg_info["to"]
        length = seg_info["length"]
        if seg_info["collision"]:
            n_col = len(seg_info["collision_points"])
            t0 = seg_info["collision_points"][0]["t"]
            print(f"    seg[{i}→{j}] len={length:.4f}  ❌ {n_col} collision(s), first at t={t0:.3f}")
        else:
            print(f"    seg[{i}→{j}] len={length:.4f}  ✅")


# ═══════════════════════════════════════════════════════════════════════════
# 2. Drake 场景构建
# ═══════════════════════════════════════════════════════════════════════════

def build_scene(meshcat):
    """构建 IIWA14 + shelves + bins + table 场景."""
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant)
    parser.package_map().Add("gcs", _GCS_ROOT)
    directives = LoadModelDirectives(_YAML_FILE)
    models = ProcessModelDirectives(directives, plant, parser)
    plant.Finalize()

    meshcat_params = MeshcatVisualizerParams()
    meshcat_params.delete_on_initialization_event = False
    meshcat_params.role = Role.kIllustration
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat, meshcat_params)
    diagram = builder.Build()
    return diagram, plant, scene_graph, models


def waypoints_to_trajectory(waypoints, speed=1.0):
    """(N,7) waypoints → PiecewisePolynomial (FirstOrderHold)."""
    wp = np.asarray(waypoints, dtype=float)
    dists = np.linalg.norm(np.diff(wp, axis=0), axis=1)
    dists = np.maximum(dists, 1e-6)
    cum = np.concatenate([[0], np.cumsum(dists)])
    breaks = cum / speed
    return PiecewisePolynomial.FirstOrderHold(breaks, wp.T)


# ═══════════════════════════════════════════════════════════════════════════
# 3. 路径可视化
# ═══════════════════════════════════════════════════════════════════════════

def visualize_single_path(meshcat, waypoints, speed=0.5):
    """动画播放单条路径, 画末端轨迹点云."""
    from pydrake.geometry import SceneGraph

    traj = waypoints_to_trajectory(waypoints, speed=speed)
    total_time = traj.end_time()

    # 采样为等间距时间序列
    n_samples = max(int(total_time * 200), 200)
    times = np.linspace(0, total_time, n_samples)
    values = np.array([traj.value(t).flatten() for t in times]).T

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
    builder.Connect(
        to_pose.get_output_port(),
        scene_graph.get_source_pose_port(plant.get_source_id()),
    )
    traj_source = builder.AddSystem(TrajectorySource(concat_traj))
    builder.Connect(traj_source.get_output_port(), to_pose.get_input_port())

    meshcat_viz = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    vis_diagram = builder.Build()
    simulator = Simulator(vis_diagram)

    # 末端轨迹点云
    plant_ctx = plant.CreateDefaultContext()
    wsg_body = plant.GetBodyByName("body", models[1].model_instance)

    n_trail = 300
    ts_trail = np.linspace(0, total_time, n_trail)
    ee_positions = []
    for t in ts_trail:
        q = traj.value(t).flatten()
        plant.SetPositions(plant_ctx, q)
        X_WE = plant.EvalBodyPoseInWorld(plant_ctx, wsg_body)
        ee_positions.append(X_WE.translation())

    ee_arr = np.array(ee_positions).T
    pc = PointCloud(n_trail)
    pc.mutable_xyzs()[:] = ee_arr

    # 用渐变色表示进度 (蓝→绿)
    meshcat.SetObject("ee_trail/path", pc, 0.006,
                       rgba=Rgba(1.0, 0.5, 0.05, 1.0))

    # 标记 waypoints 位置
    for i, q in enumerate(waypoints):
        plant.SetPositions(plant_ctx, q)
        X_WE = plant.EvalBodyPoseInWorld(plant_ctx, wsg_body)
        pos = X_WE.translation()
        from pydrake.geometry import Sphere
        meshcat.SetObject(f"waypoints/wp{i}", Sphere(0.015),
                           rgba=Rgba(0.12, 0.47, 0.71, 1.0) if i == 0
                                 else Rgba(0.17, 0.63, 0.17, 1.0) if i == len(waypoints) - 1
                                 else Rgba(0.84, 0.15, 0.16, 0.8))
        from pydrake.math import RigidTransform
        meshcat.SetTransform(f"waypoints/wp{i}", RigidTransform(pos))

    # 录制动画
    meshcat_viz.StartRecording()
    simulator.AdvanceTo(total_time + 0.5)
    meshcat_viz.PublishRecording()

    return total_time


def visualize_multi_seed_ghosting(meshcat, all_waypoints, seeds):
    """多 seed 末端轨迹叠加 (静态残影)."""
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant)
    parser.package_map().Add("gcs", _GCS_ROOT)
    directives = LoadModelDirectives(_YAML_FILE)
    models = ProcessModelDirectives(directives, plant, parser)
    plant.Finalize()

    meshcat_params = MeshcatVisualizerParams()
    meshcat_params.delete_on_initialization_event = False
    meshcat_params.role = Role.kIllustration
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat, meshcat_params)

    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    plant_ctx = plant.GetMyMutableContextFromRoot(context)
    wsg_body = plant.GetBodyByName("body", models[1].model_instance)

    # 颜色方案: 每个 seed 不同色相
    import colorsys
    n_seeds = len(seeds)

    for si, (wp, seed) in enumerate(zip(all_waypoints, seeds)):
        hue = si / max(n_seeds, 1)
        r, g, b = colorsys.hsv_to_rgb(hue, 0.8, 0.9)

        traj = waypoints_to_trajectory(wp, speed=1.0)
        n_pts = max(int(traj.end_time() * 200), 100)
        ts = np.linspace(traj.start_time(), traj.end_time(), n_pts)
        ee_positions = []
        for t in ts:
            q = traj.value(t).flatten()
            plant.SetPositions(plant_ctx, q)
            X_WE = plant.EvalBodyPoseInWorld(plant_ctx, wsg_body)
            ee_positions.append(X_WE.translation())

        ee_arr = np.array(ee_positions).T
        pc = PointCloud(n_pts)
        pc.mutable_xyzs()[:] = ee_arr
        meshcat.SetObject(f"paths/seed{seed}", pc, 0.005,
                           rgba=Rgba(r, g, b, 0.7))

    # 设置 TS 姿态作为参考
    plant.SetPositions(plant_ctx, all_waypoints[0][0])
    diagram.ForcedPublish(context)


# ═══════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════

def load_ts_cs_paths(cache_path):
    """加载所有 seed 的 TS→CS 成功路径."""
    with open(cache_path, "rb") as f:
        data = pickle.load(f)
    paths = []
    for entry in data:
        if entry.get("pair_name") != "TS→CS":
            continue
        if not entry.get("success", False):
            continue
        if "path_waypoints" not in entry:
            continue
        paths.append({
            "seed": entry["seed"],
            "waypoints": np.array(entry["path_waypoints"]),
            "path_length": entry.get("path_length", 0),
        })
    return paths


def main():
    parser = argparse.ArgumentParser(description="TS→CS 路径碰撞检测 + 可视化")
    parser.add_argument("--seed", type=int, default=0, help="可视化的 seed")
    parser.add_argument("--all-seeds", action="store_true",
                        help="所有 seed 末端轨迹叠加")
    parser.add_argument("--save", type=str, default=None,
                        help="导出 HTML 文件路径")
    parser.add_argument("--speed", type=float, default=0.3,
                        help="动画播放速度 (越小越慢)")
    parser.add_argument("--skip-collision", action="store_true",
                        help="跳过碰撞检测")
    args = parser.parse_args()

    # ── 加载路径 ──
    if not os.path.exists(_PATHS_CACHE):
        print(f"错误: 找不到 {_PATHS_CACHE}")
        sys.exit(1)

    paths = load_ts_cs_paths(_PATHS_CACHE)
    print(f"已加载 {len(paths)} 条 TS→CS 路径")

    # ── 碰撞检测 ──
    if not args.skip_collision:
        print(f"\n{'='*70}")
        print(f"  TS→CS 路径碰撞检测 (200 点/段插值)")
        print(f"{'='*70}")
        all_safe = True
        for p in paths:
            report = check_path_collision(p["waypoints"])
            print_collision_report(report, p["seed"])
            if not report["collision_free"]:
                all_safe = False
        status = "✅ 所有路径无碰撞" if all_safe else "❌ 存在碰撞路径"
        print(f"\n  总结: {status}")
        print(f"{'='*70}")

    # ── Meshcat 可视化 ──
    meshcat = StartMeshcat()
    print(f"\nMeshcat URL: {meshcat.web_url()}")

    if args.all_seeds:
        print(f"\n所有 seed 末端轨迹叠加模式...")
        all_wp = [p["waypoints"] for p in paths]
        all_s = [p["seed"] for p in paths]
        visualize_multi_seed_ghosting(meshcat, all_wp, all_s)
        print("静态可视化就绪")
    else:
        # 找指定 seed
        target = [p for p in paths if p["seed"] == args.seed]
        if not target:
            print(f"seed {args.seed} 没有成功的 TS→CS 路径, 可用: "
                  f"{[p['seed'] for p in paths]}")
            sys.exit(1)
        wp = target[0]["waypoints"]
        print(f"\n可视化 seed={args.seed}, {wp.shape[0]} waypoints, "
              f"路径长度={target[0]['path_length']:.4f}")
        total_time = visualize_single_path(meshcat, wp, speed=args.speed)
        print(f"动画就绪, 时长 {total_time:.1f}s")

    # 导出 HTML
    if args.save:
        html = meshcat.StaticHtml()
        with open(args.save, "w") as f:
            f.write(html)
        print(f"已导出 → {args.save}")

    print("\n按 Ctrl+C 退出")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n退出")


if __name__ == "__main__":
    main()

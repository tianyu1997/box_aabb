#!/usr/bin/env python3
"""
viz_drake_paths.py — Drake Meshcat 可视化 exp2 规划路径

参照 v4/viz/viz_exp2_paths.py 和 gcs-science-robotics/reproduction/bimanual/helpers.py:
  1. Drake MultibodyPlant + SceneGraph 构建 IIWA14 场景
  2. PiecewisePolynomial.FirstOrderHold 构建线性轨迹
  3. TrajectorySource → MultibodyPositionToGeometryPose → MeshcatVisualizer
  4. Simulator 播放动画 + PointCloud 末端轨迹
  5. 可导出 StaticHtml

Usage:
    # 运行 exp2 并导出路径 JSON:
    ./experiments/exp2_e2e_planning --seeds 1 --threads 1 --one-shot \\
        --save-paths /tmp/paths.json

    # 可视化全部路径:
    python viz/viz_drake_paths.py /tmp/paths.json

    # 只看某对:
    python viz/viz_drake_paths.py /tmp/paths.json --pair 0

    # 静态模式 (起止姿态 + 末端轨迹):
    python viz/viz_drake_paths.py /tmp/paths.json --static

    # 导出交互式 HTML:
    python viz/viz_drake_paths.py /tmp/paths.json --save viz.html
"""

import argparse
import json
import os
import sys
import time

import numpy as np

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

# ── 路径常量 ──────────────────────────────────────────────────────────────
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_V5_ROOT = os.path.dirname(_THIS_DIR)                      # cpp/v5/
_PROJ_ROOT = os.path.dirname(os.path.dirname(_V5_ROOT))     # box_aabb/
_GCS_ROOT = os.path.join(_PROJ_ROOT, "gcs-science-robotics")
_MODELS_DIR = os.path.join(_GCS_ROOT, "models")
_YAML_FILE = os.path.join(_MODELS_DIR, "iiwa14_welded_gripper.yaml")

PAIR_NAMES = ["AS→TS", "TS→CS", "CS→LB", "LB→RB", "RB→AS"]

# 颜色方案 (与 Marcucci 论文风格一致)
PAIR_COLORS = [
    Rgba(0.12, 0.47, 0.71, 1.0),  # 蓝
    Rgba(1.00, 0.50, 0.05, 1.0),  # 橙
    Rgba(0.17, 0.63, 0.17, 1.0),  # 绿
    Rgba(0.84, 0.15, 0.16, 1.0),  # 红
    Rgba(0.58, 0.40, 0.74, 1.0),  # 紫
]


# ═══════════════════════════════════════════════════════════════════════════
# Drake 场景构建
# ═══════════════════════════════════════════════════════════════════════════

def build_scene(meshcat):
    """构建 IIWA14 单臂 + shelves + bins + table 场景。"""
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

    # 设置初始姿态 (home position)
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(context)
    q_home = [0.0, 0.5, 0.0, -1.5, 0.0, 0.5, 1.57]
    iiwa_model = models[0].model_instance
    idx = 0
    for joint_index in plant.GetJointIndices(iiwa_model):
        joint = plant.get_mutable_joint(joint_index)
        if isinstance(joint, RevoluteJoint):
            joint.set_default_angle(q_home[idx])
            idx += 1
            if idx >= 7:
                break
    diagram.ForcedPublish(context)

    return diagram, plant, scene_graph, models


# ═══════════════════════════════════════════════════════════════════════════
# 轨迹可视化
# ═══════════════════════════════════════════════════════════════════════════

def waypoints_to_trajectory(waypoints, speed=1.0):
    """将 (N, 7) waypoints 转为 PiecewisePolynomial (FirstOrderHold)。"""
    wp = np.asarray(waypoints, dtype=float)
    n = wp.shape[0]
    if n < 2:
        raise ValueError(f"需要至少 2 个 waypoint, 得到 {n}")

    # 按关节空间距离分配时间
    dists = np.linalg.norm(np.diff(wp, axis=0), axis=1)
    dists = np.maximum(dists, 1e-6)
    cum = np.concatenate([[0], np.cumsum(dists)])
    breaks = cum / speed

    traj = PiecewisePolynomial.FirstOrderHold(breaks, wp.T)
    return traj


def visualize_trajectory(meshcat, waypoints_list, pair_indices=None, speed=1.5):
    """
    构建动画场景, 播放多条轨迹, 画末端轨迹点云.

    流程:
    1. 拼接所有轨迹为一条连续轨迹 (带 0.8s 停顿)
    2. TrajectorySource → MultibodyPositionToGeometryPose → MeshcatVisualizer
    3. Simulator 播放 + PointCloud 画末端轨迹
    """
    if pair_indices is None:
        pair_indices = list(range(len(waypoints_list)))

    # 拼接所有轨迹为一条连续轨迹
    trajectories = []
    t_offset = 0.0
    pause_duration = 0.8
    for wp in waypoints_list:
        traj = waypoints_to_trajectory(wp, speed=speed)
        trajectories.append((t_offset, traj))
        t_offset += traj.end_time() + pause_duration

    total_time = t_offset - pause_duration if trajectories else 0.0

    def eval_concatenated(t):
        for i, (offset, traj) in enumerate(trajectories):
            seg_end = offset + traj.end_time()
            if t <= seg_end or i == len(trajectories) - 1:
                local_t = np.clip(t - offset, traj.start_time(), traj.end_time())
                return traj.value(local_t).flatten()
        return trajectories[-1][1].value(trajectories[-1][1].end_time()).flatten()

    # 创建拼接后的单一轨迹
    n_samples = max(int(total_time * 100), 200)
    times = np.linspace(0, total_time, n_samples)
    values = np.array([eval_concatenated(t) for t in times]).T
    concat_traj = PiecewisePolynomial.FirstOrderHold(times, values)

    # ── 构建动画 diagram ──
    from pydrake.geometry import SceneGraph
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
    meshcat.Delete()

    vis_diagram = builder.Build()
    simulator = Simulator(vis_diagram)

    # ── 画末端轨迹点云 ──
    plant_ctx = plant.CreateDefaultContext()
    wsg_model = models[1].model_instance
    wsg_body = plant.GetBodyByName("body", wsg_model)

    for path_idx, (wp, pi) in enumerate(zip(waypoints_list, pair_indices)):
        traj = waypoints_to_trajectory(wp, speed=speed)
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
        name = PAIR_NAMES[pi] if pi < len(PAIR_NAMES) else f"path_{pi}"
        color = PAIR_COLORS[pi % len(PAIR_COLORS)]
        meshcat.SetObject(f"paths/{name}", pc, 0.008, rgba=color)

    # ── 录制动画 ──
    meshcat_viz.StartRecording()
    simulator.AdvanceTo(total_time)
    meshcat_viz.PublishRecording()

    return total_time


def visualize_static(meshcat, waypoints_list, pair_indices=None):
    """静态可视化: 末端轨迹点云 + 最后姿态。"""
    if pair_indices is None:
        pair_indices = list(range(len(waypoints_list)))

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

    wsg_model = models[1].model_instance
    wsg_body = plant.GetBodyByName("body", wsg_model)

    for path_idx, (wp, pi) in enumerate(zip(waypoints_list, pair_indices)):
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
        name = PAIR_NAMES[pi] if pi < len(PAIR_NAMES) else f"path_{pi}"
        color = PAIR_COLORS[pi % len(PAIR_COLORS)]
        meshcat.SetObject(f"paths/{name}", pc, 0.008, rgba=color)

    # 设置最后一条路径的终点姿态
    if waypoints_list:
        last_wp = waypoints_list[-1]
        plant.SetPositions(plant_ctx, last_wp[-1])
        diagram.ForcedPublish(context)


# ═══════════════════════════════════════════════════════════════════════════
# JSON 加载
# ═══════════════════════════════════════════════════════════════════════════

def load_paths_json(json_path, seed=None, pair_filter=None):
    """从 exp2 JSON 加载路径 waypoints。"""
    with open(json_path, "r") as f:
        data = json.load(f)

    results = []
    for entry in data.get("paths", []):
        if seed is not None and entry.get("seed", 0) != seed:
            continue
        if not entry.get("success", False):
            continue
        wp = entry.get("waypoints", [])
        if len(wp) < 2:
            continue
        pi = entry["pair_idx"]
        if pair_filter is not None and pi not in pair_filter:
            continue
        results.append({
            "pair_idx": pi,
            "label": entry.get("label", f"pair_{pi}"),
            "waypoints": np.array(wp, dtype=float),
            "path_length": entry.get("path_length", 0.0),
        })

    results.sort(key=lambda x: x["pair_idx"])
    return results


# ═══════════════════════════════════════════════════════════════════════════
# 主程序
# ═══════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="Drake Meshcat 可视化 SBF v5 规划路径")
    parser.add_argument("json_file", help="路径 JSON 文件 (--save-paths 导出)")
    parser.add_argument("--seed", type=int, default=None,
                        help="只显示指定 seed 的路径 (默认: 显示全部)")
    parser.add_argument("--pair", type=int, default=None,
                        help="只显示指定 pair index (0-4)")
    parser.add_argument("--static", action="store_true",
                        help="静态模式 (末端轨迹 + 终点姿态)")
    parser.add_argument("--save", type=str, default=None,
                        help="导出交互式 HTML 文件")
    parser.add_argument("--no-show", action="store_true",
                        help="只保存 HTML, 不阻塞等待 (用于自动化)")
    parser.add_argument("--speed", type=float, default=1.5,
                        help="动画播放速度 (默认: 1.5)")
    args = parser.parse_args()

    if not os.path.exists(args.json_file):
        print(f"错误: 找不到 {args.json_file}")
        sys.exit(1)

    if not os.path.exists(_YAML_FILE):
        print(f"错误: 找不到 Drake 模型文件 {_YAML_FILE}")
        print(f"请确保 gcs-science-robotics 在 {_GCS_ROOT}")
        sys.exit(1)

    pair_filter = [args.pair] if args.pair is not None else None
    paths = load_paths_json(args.json_file, seed=args.seed,
                            pair_filter=pair_filter)

    if not paths:
        print(f"没有找到成功的路径 (seed={args.seed}, pair={args.pair})")
        sys.exit(1)

    print(f"加载 {len(paths)} 条路径:")
    for p in paths:
        wp = p["waypoints"]
        print(f"  {p['label']:8s}  {wp.shape[0]} waypoints  "
              f"len={p['path_length']:.3f}")

    # 启动 Meshcat
    meshcat = StartMeshcat()
    print(f"\nMeshcat URL: {meshcat.web_url()}")

    waypoints_list = [p["waypoints"] for p in paths]
    pair_indices = [p["pair_idx"] for p in paths]

    if args.static:
        build_scene(meshcat)
        time.sleep(0.5)
        visualize_static(meshcat, waypoints_list, pair_indices)
        print("\n静态可视化就绪")
    else:
        total_time = visualize_trajectory(
            meshcat, waypoints_list, pair_indices, speed=args.speed)
        print(f"\n动画就绪, 总时长 {total_time:.1f}s")

    if args.save:
        # Insert timestamp suffix before extension
        from datetime import datetime
        ts = datetime.now().strftime("_%Y%m%d_%H%M%S")
        base, ext = os.path.splitext(args.save)
        save_path = f"{base}{ts}{ext}" if ext else f"{args.save}{ts}"
        html = meshcat.StaticHtml()
        with open(save_path, "w") as f:
            f.write(html)
        print(f"已导出 HTML → {save_path}")

    if args.no_show:
        return

    print("\n按 Ctrl+C 退出")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n退出")


if __name__ == "__main__":
    main()

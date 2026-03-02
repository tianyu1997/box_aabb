#!/usr/bin/env python3
"""
viz_exp2_paths.py — Meshcat 可视化 exp2 规划路径

严格参照 gcs-science-robotics/reproduction/bimanual/helpers.py 的可视化方法:
  1. Drake MultibodyPlant + SceneGraph 构建单臂 IIWA14 场景
  2. PiecewisePolynomial.FirstOrderHold 构建线性轨迹
  3. TrajectorySource → MultibodyPositionToGeometryPose → MeshcatVisualizer
  4. Simulator 播放动画 + PointCloud 末端轨迹
  5. 导出 StaticHtml

Usage:
    python viz_exp2_paths.py                   # 可视化全部 5 条路径
    python viz_exp2_paths.py --pair 0          # 只看 AS→TS
    python viz_exp2_paths.py --seed 3          # 换 seed
    python viz_exp2_paths.py --static          # 静态起止姿态 + 残影
    python viz_exp2_paths.py --save out.html   # 导出 HTML
"""

import argparse
import os
import pickle
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
_PROJ_ROOT = os.path.dirname(_THIS_DIR)                     # v4/
_GCS_ROOT = os.path.join(os.path.dirname(_PROJ_ROOT), "gcs-science-robotics")
_MODELS_DIR = os.path.join(_GCS_ROOT, "models")
_YAML_FILE = os.path.join(_MODELS_DIR, "iiwa14_welded_gripper.yaml")
_PATHS_CACHE = os.path.join(_PROJ_ROOT, "output", "exp2_paths", "sbf_paths.pkl")

PAIR_NAMES = ["AS→TS", "TS→CS", "CS→LB", "LB→RB", "RB→AS"]

# 5 条路径的颜色方案 (与 Marcucci 论文风格一致)
PAIR_COLORS = [
    Rgba(0.12, 0.47, 0.71, 1.0),  # 蓝
    Rgba(1.00, 0.50, 0.05, 1.0),  # 橙
    Rgba(0.17, 0.63, 0.17, 1.0),  # 绿
    Rgba(0.84, 0.15, 0.16, 1.0),  # 红
    Rgba(0.58, 0.40, 0.74, 1.0),  # 紫
]


# ═══════════════════════════════════════════════════════════════════════════
# Drake 场景构建 (严格参照 bimanual notebook cell 3)
# ═══════════════════════════════════════════════════════════════════════════

def build_scene(meshcat):
    """构建 IIWA14 单臂 + shelves + bins + table 场景, 返回 (diagram, plant, scene_graph, models)."""
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant)
    parser.package_map().Add("gcs", _GCS_ROOT)

    directives = LoadModelDirectives(_YAML_FILE)
    models = ProcessModelDirectives(directives, plant, parser)
    # models: [iiwa, wsg, shelves, binR, binL, table]

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
# 轨迹可视化 (严格参照 helpers.py visualize_trajectory)
# ═══════════════════════════════════════════════════════════════════════════

def waypoints_to_trajectory(waypoints, speed=1.0):
    """将 (N, 7) waypoints 转为 PiecewisePolynomial (FirstOrderHold)."""
    wp = np.asarray(waypoints, dtype=float)
    n = wp.shape[0]
    if n < 2:
        raise ValueError(f"需要至少 2 个 waypoint, 得到 {n}")

    # 按关节空间距离分配时间 (均匀弧长参数化)
    dists = np.linalg.norm(np.diff(wp, axis=0), axis=1)
    dists = np.maximum(dists, 1e-6)  # 避免零段
    cum = np.concatenate([[0], np.cumsum(dists)])
    breaks = cum / speed  # 秒

    traj = PiecewisePolynomial.FirstOrderHold(breaks, wp.T)
    return traj


def visualize_trajectory(meshcat, waypoints_list, pair_indices=None):
    """
    构建动画场景, 播放多条轨迹, 画末端轨迹点云.

    参照 helpers.py visualize_trajectory() 的流程:
    1. 新建 DiagramBuilder + plant (独立于静态场景)
    2. TrajectorySource → MultibodyPositionToGeometryPose → MeshcatVisualizer
    3. Simulator.AdvanceTo() 播放
    4. PointCloud 画末端轨迹

    Args:
        meshcat: Meshcat 实例
        waypoints_list: List[(N_i, 7) ndarray], 每条路径的 waypoints
        pair_indices: 对应的 pair index (用于颜色), 若 None 则自动编号
    """
    if pair_indices is None:
        pair_indices = list(range(len(waypoints_list)))

    # 拼接所有轨迹为一条连续轨迹 (带 1s 停顿)
    trajectories = []
    t_offset = 0.0
    pause_duration = 0.8  # 路径之间的停顿
    for wp in waypoints_list:
        traj = waypoints_to_trajectory(wp, speed=1.5)
        trajectories.append((t_offset, traj))
        t_offset += traj.end_time() + pause_duration

    total_time = t_offset - pause_duration if trajectories else 0.0

    # 构建串联轨迹函数
    def eval_concatenated(t):
        for i, (offset, traj) in enumerate(trajectories):
            seg_end = offset + traj.end_time()
            if t <= seg_end or i == len(trajectories) - 1:
                local_t = np.clip(t - offset, traj.start_time(), traj.end_time())
                return traj.value(local_t).flatten()
        return trajectories[-1][1].value(trajectories[-1][1].end_time()).flatten()

    # 创建拼接后的单一轨迹 (采样重建)
    n_samples = max(int(total_time * 100), 200)
    times = np.linspace(0, total_time, n_samples)
    values = np.array([eval_concatenated(t) for t in times]).T  # (7, n_samples)
    concat_traj = PiecewisePolynomial.FirstOrderHold(times, values)

    # ── 构建动画 diagram (参照 helpers.py: 独立 SceneGraph) ──
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

    # ── 画末端轨迹点云 (参照 helpers.py) ──
    plant_ctx = plant.CreateDefaultContext()
    wsg_model = models[1].model_instance  # wsg
    wsg_body = plant.GetBodyByName("body", wsg_model)

    for path_idx, (wp, pi) in enumerate(zip(waypoints_list, pair_indices)):
        traj = waypoints_to_trajectory(wp, speed=1.5)
        n_pts = max(int(traj.end_time() * 200), 100)
        ts = np.linspace(traj.start_time(), traj.end_time(), n_pts)
        ee_positions = []
        for t in ts:
            q = traj.value(t).flatten()
            plant.SetPositions(plant_ctx, q)
            X_WE = plant.EvalBodyPoseInWorld(plant_ctx, wsg_body)
            ee_positions.append(X_WE.translation())

        ee_arr = np.array(ee_positions).T  # (3, n_pts)
        pc = PointCloud(n_pts)
        pc.mutable_xyzs()[:] = ee_arr
        color = PAIR_COLORS[pi % len(PAIR_COLORS)]
        meshcat.SetObject(
            f"paths/{PAIR_NAMES[pi]}",
            pc, 0.008,
            rgba=color,
        )

    # ── 录制动画 ──
    meshcat_viz.StartRecording()
    simulator.AdvanceTo(total_time)
    meshcat_viz.PublishRecording()

    return total_time


def visualize_static(meshcat, waypoints_list, pair_indices=None, n_ghost=5):
    """
    静态可视化: 在场景中显示每条路径的起止姿态和等间距残影.

    参照 helpers.py generate_segment_pics() 的方法:
    - 起点: 不透明
    - 终点: 半透明
    - 中间残影: colormap 渐变
    """
    if pair_indices is None:
        pair_indices = list(range(len(waypoints_list)))

    # 构建一次 plant 用于 FK
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

    # 画每条路径的末端轨迹
    for path_idx, (wp, pi) in enumerate(zip(waypoints_list, pair_indices)):
        # 末端轨迹点云
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
        color = PAIR_COLORS[pi % len(PAIR_COLORS)]
        meshcat.SetObject(f"paths/{PAIR_NAMES[pi]}", pc, 0.008, rgba=color)

    # 设置最后一条路径的终点姿态并发布
    if waypoints_list:
        last_wp = waypoints_list[-1]
        plant.SetPositions(plant_ctx, last_wp[-1])
        diagram.ForcedPublish(context)


# ═══════════════════════════════════════════════════════════════════════════
# 主程序
# ═══════════════════════════════════════════════════════════════════════════

def load_paths(cache_path, seed=0, pair_filter=None):
    """从 exp2 缓存加载路径 waypoints."""
    with open(cache_path, "rb") as f:
        all_entries = pickle.load(f)

    results = []
    for entry in all_entries:
        if entry.get("seed", 0) != seed:
            continue
        if not entry.get("success", False):
            continue
        if "path_waypoints" not in entry:
            continue
        pi = entry["pair_idx"]
        if pair_filter is not None and pi not in pair_filter:
            continue
        results.append({
            "pair_idx": pi,
            "pair_name": entry["pair_name"],
            "waypoints": np.array(entry["path_waypoints"]),
        })

    results.sort(key=lambda x: x["pair_idx"])
    return results


def main():
    parser = argparse.ArgumentParser(description="Meshcat 可视化 exp2 规划路径")
    parser.add_argument("--cache", default=_PATHS_CACHE,
                        help="路径缓存 pkl 文件路径")
    parser.add_argument("--seed", type=int, default=0, help="随机种子")
    parser.add_argument("--pair", type=int, default=None,
                        help="只可视化指定 pair index (0-4)")
    parser.add_argument("--static", action="store_true",
                        help="静态模式 (起止姿态 + 末端轨迹)")
    parser.add_argument("--save", type=str, default=None,
                        help="导出 HTML 文件路径")
    parser.add_argument("--speed", type=float, default=1.5,
                        help="动画播放速度")
    args = parser.parse_args()

    # 加载路径
    if not os.path.exists(args.cache):
        print(f"错误: 找不到路径缓存 {args.cache}")
        print(f"请先运行 exp2: python paper_exp2_planning.py --seeds 1 --group sbf")
        sys.exit(1)

    pair_filter = [args.pair] if args.pair is not None else None
    paths = load_paths(args.cache, seed=args.seed, pair_filter=pair_filter)

    if not paths:
        print(f"没有找到 seed={args.seed} 的成功路径")
        sys.exit(1)

    print(f"加载 {len(paths)} 条路径 (seed={args.seed}):")
    for p in paths:
        wp = p["waypoints"]
        print(f"  {p['pair_name']:8s}  {wp.shape[0]} waypoints")

    # 启动 Meshcat
    meshcat = StartMeshcat()
    print(f"\nMeshcat URL: {meshcat.web_url()}")

    waypoints_list = [p["waypoints"] for p in paths]
    pair_indices = [p["pair_idx"] for p in paths]

    if args.static:
        # 静态场景
        build_scene(meshcat)
        time.sleep(0.5)
        visualize_static(meshcat, waypoints_list, pair_indices)
        print("\n静态可视化就绪")
    else:
        # 动画模式
        total_time = visualize_trajectory(meshcat, waypoints_list, pair_indices)
        print(f"\n动画就绪, 总时长 {total_time:.1f}s")

    # 导出 HTML
    if args.save:
        html = meshcat.StaticHtml()
        with open(args.save, "w") as f:
            f.write(html)
        print(f"已导出 HTML → {args.save}")

    # 保持运行
    print("\n按 Ctrl+C 退出")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n退出")


if __name__ == "__main__":
    main()

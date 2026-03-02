#!/usr/bin/env python3
"""
viz_iris_bezier.py — IRIS-GCS Bezier 平滑路径可视化

用 BezierGCS (order=6, continuity=4) 替代 LinearGCS, 生成平滑轨迹并可视化.
参照 gcs-science-robotics/reproduction/bimanual/helpers.py 的 getBezierGcsPath.

Usage:
    python viz/viz_iris_bezier.py                          # 动画模式
    python viz/viz_iris_bezier.py --save iris_bezier.html   # 导出 HTML
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
sys.path.insert(0, _GCS_ROOT)

from pydrake.geometry import (
    MeshcatVisualizer, MeshcatVisualizerParams, Role, Rgba, StartMeshcat,
    SceneGraph, Sphere,
)
from pydrake.math import RigidTransform
from pydrake.multibody.parsing import (
    LoadModelDirectives, Parser, ProcessModelDirectives,
)
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
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
# 1. IRIS Bezier 规划
# ═══════════════════════════════════════════════════════════════════════════

def run_iris_bezier_planning(seed=0, order=6, continuity=4):
    """用 BezierGCS 做 IRIS 规划, 返回 BezierTrajectory 列表."""
    from gcs.bezier import BezierGCS
    from gcs.rounding import randomForwardPathSearch
    from pydrake.solvers import MosekSolver, SolverOptions
    from marcucci_scenes import get_query_pairs

    # 加载 IRIS regions
    with open(_IRIS_REG, "rb") as f:
        regions = pickle.load(f)
    region_list = list(regions.values()) if isinstance(regions, dict) else regions
    region_names = list(regions.keys()) if isinstance(regions, dict) else [f"r{i}" for i in range(len(region_list))]
    print(f"  IRIS regions: {len(region_list)} ({region_names})")

    # 构建 plant 用于获取速度限制
    builder = DiagramBuilder()
    plant, _ = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser_drake = Parser(plant)
    parser_drake.package_map().Add("gcs", os.path.abspath(_GCS_ROOT))
    directives = LoadModelDirectives(_YAML_FILE)
    ProcessModelDirectives(directives, plant, parser_drake)
    plant.Finalize()

    vel_lower = plant.GetVelocityLowerLimits()
    vel_upper = plant.GetVelocityUpperLimits()

    pairs = get_query_pairs("combined")
    results = []

    for pi, (q_start, q_goal) in enumerate(pairs):
        print(f"  [{PAIR_NAMES[pi]}] BezierGCS (order={order}, C{continuity}) ...",
              end="", flush=True)

        gcs = BezierGCS(region_list, order, continuity, hdot_min=1e-3)
        gcs.addTimeCost(1)
        gcs.addPathLengthCost(1)
        gcs.addDerivativeRegularization(1e-3, 1e-3, 2)
        gcs.addVelocityLimits(0.6 * vel_lower, 0.6 * vel_upper)
        gcs.setPaperSolverOptions()
        gcs.setSolver(MosekSolver())
        gcs.setRoundingStrategy(randomForwardPathSearch,
                                max_paths=10, max_trials=100, seed=seed)

        try:
            gcs.addSourceTarget(q_start, q_goal)
        except ValueError as e:
            print(f" FAIL (addSourceTarget: {e})")
            results.append({"pair_idx": pi, "pair_name": PAIR_NAMES[pi],
                            "success": False, "trajectory": None})
            continue

        t0 = time.perf_counter()
        try:
            bezier_traj, results_dict = gcs.SolvePath(
                rounding=True, verbose=False, preprocessing=True)
        except Exception as e:
            print(f" FAIL ({e})")
            results.append({"pair_idx": pi, "pair_name": PAIR_NAMES[pi],
                            "success": False, "trajectory": None})
            continue
        solve_time = time.perf_counter() - t0

        if bezier_traj is None:
            print(f" FAIL (solver returned None)")
            results.append({"pair_idx": pi, "pair_name": PAIR_NAMES[pi],
                            "success": False, "trajectory": None})
            continue

        # 采样航点用于 path_length 度量
        n_samples = 200
        ts = np.linspace(bezier_traj.start_time(), bezier_traj.end_time(), n_samples)
        pts = bezier_traj.vector_values(ts)  # (D, n_samples)
        path_len = float(np.sum(np.linalg.norm(np.diff(pts, axis=1), axis=0)))
        duration = bezier_traj.end_time() - bezier_traj.start_time()

        print(f" OK  solve={solve_time*1000:.1f}ms  len={path_len:.3f}  "
              f"dur={duration:.2f}s")
        results.append({
            "pair_idx": pi, "pair_name": PAIR_NAMES[pi],
            "success": True, "trajectory": bezier_traj,
            "path_length": path_len, "solve_time": solve_time,
            "duration": duration,
        })
        gcs.ResetGraph()

    return results


# ═══════════════════════════════════════════════════════════════════════════
# 2. 可视化
# ═══════════════════════════════════════════════════════════════════════════

def visualize_bezier_paths(meshcat, path_list, speed_factor=1.0):
    """动画播放 BezierTrajectory 路径, 画末端轨迹点云."""

    # 对每条成功的轨迹, 用 BezierTrajectory 采样 → PiecewisePolynomial
    trajectories = []  # (t_offset, PiecewisePolynomial, pair_idx, BezierTrajectory)
    t_offset = 0.0
    pause = 0.8

    for entry in path_list:
        if not entry["success"]:
            continue
        btraj = entry["trajectory"]
        duration = btraj.end_time() - btraj.start_time()
        actual_dur = duration / speed_factor

        # 采样 Bezier 轨迹 → 密集航点
        n_pts = max(int(actual_dur * 200), 200)
        s_times = np.linspace(btraj.start_time(), btraj.end_time(), n_pts)
        values = btraj.vector_values(s_times)  # (D, n_pts)

        # 映射到实际播放时间
        play_times = np.linspace(0, actual_dur, n_pts) + t_offset
        pp_traj = PiecewisePolynomial.FirstOrderHold(
            play_times - t_offset, values)

        trajectories.append((t_offset, pp_traj, entry["pair_idx"], btraj))
        t_offset += actual_dur + pause

    total_time = t_offset - pause if trajectories else 0.0

    # 拼成一条 PiecewisePolynomial
    n_samples = max(int(total_time * 100), 300)
    times = np.linspace(0, total_time, n_samples)

    def eval_t(t):
        for i, (offset, traj, _, _) in enumerate(trajectories):
            seg_end = offset + traj.end_time()
            if t <= seg_end or i == len(trajectories) - 1:
                lt = np.clip(t - offset, traj.start_time(), traj.end_time())
                return traj.value(lt).flatten()
        return trajectories[-1][1].value(trajectories[-1][1].end_time()).flatten()

    values_arr = np.array([eval_t(t) for t in times]).T
    concat_traj = PiecewisePolynomial.FirstOrderHold(times, values_arr)

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

    # 末端轨迹点云 (用原始 Bezier 采样, 更平滑)
    plant_ctx = plant.CreateDefaultContext()
    wsg_body = plant.GetBodyByName("body", models[1].model_instance)

    for offset, pp_traj, pi, btraj in trajectories:
        n_ee = 300
        s_times = np.linspace(btraj.start_time(), btraj.end_time(), n_ee)
        ee_pos = []
        for s in s_times:
            q = btraj.value(s).flatten()
            plant.SetPositions(plant_ctx, q)
            X_WE = plant.EvalBodyPoseInWorld(plant_ctx, wsg_body)
            ee_pos.append(X_WE.translation())
        ee_arr = np.array(ee_pos).T
        pc = PointCloud(n_ee)
        pc.mutable_xyzs()[:] = ee_arr
        color = PAIR_COLORS[pi % len(PAIR_COLORS)]
        meshcat.SetObject(f"paths/{PAIR_NAMES[pi]}", pc, 0.006, rgba=color)

    # 起止点标记
    for entry in path_list:
        if not entry["success"]:
            continue
        pi = entry["pair_idx"]
        btraj = entry["trajectory"]
        for label, s_val, c in [
            ("start", btraj.start_time(), Rgba(0.12, 0.47, 0.71, 1.0)),
            ("goal",  btraj.end_time(),   Rgba(0.17, 0.63, 0.17, 1.0)),
        ]:
            q = btraj.value(s_val).flatten()
            plant.SetPositions(plant_ctx, q)
            X_WE = plant.EvalBodyPoseInWorld(plant_ctx, wsg_body)
            pos = X_WE.translation()
            meshcat.SetObject(f"wp/{PAIR_NAMES[pi]}/{label}", Sphere(0.012), rgba=c)
            meshcat.SetTransform(f"wp/{PAIR_NAMES[pi]}/{label}", RigidTransform(pos))

    # 录制动画
    meshcat_viz.StartRecording()
    simulator.AdvanceTo(total_time + 0.5)
    meshcat_viz.PublishRecording()

    return total_time


# ═══════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(description="IRIS-GCS Bezier 平滑路径可视化")
    ap.add_argument("--seed", type=int, default=0, help="GCS rounding seed")
    ap.add_argument("--save", type=str, default=None, help="导出 HTML 文件名")
    ap.add_argument("--speed", type=float, default=1.0, help="动画速度倍率")
    ap.add_argument("--order", type=int, default=6, help="Bezier 阶数")
    ap.add_argument("--continuity", type=int, default=4, help="连续性阶数")
    args = ap.parse_args()

    print("=" * 70)
    print("  IRIS-GCS Bezier 平滑路径可视化")
    print("=" * 70)

    # 1. 规划
    print(f"\n[1] BezierGCS 规划 (order={args.order}, C{args.continuity})...")
    path_list = run_iris_bezier_planning(
        seed=args.seed, order=args.order, continuity=args.continuity)
    n_ok = sum(1 for e in path_list if e["success"])
    print(f"\n  成功: {n_ok}/{len(path_list)}")

    ok_paths = [e for e in path_list if e["success"]]
    if not ok_paths:
        print("\n没有成功的路径")
        return

    # 2. 可视化
    print(f"\n[2] Meshcat 可视化 ({len(ok_paths)} 条路径)...")
    meshcat = StartMeshcat()
    print(f"  Meshcat URL: {meshcat.web_url()}")

    total_time = visualize_bezier_paths(meshcat, ok_paths, speed_factor=args.speed)
    print(f"  动画时长: {total_time:.1f}s")

    # 导出
    if args.save:
        save_path = os.path.join(_PROJ_ROOT, "output", args.save)
        html = meshcat.StaticHtml()
        with open(save_path, "w") as f:
            f.write(html)
        sz = os.path.getsize(save_path) / 1e6
        print(f"  已导出 → {save_path} ({sz:.1f}MB)")

    print("\n按 Ctrl+C 退出")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n退出")


if __name__ == "__main__":
    main()

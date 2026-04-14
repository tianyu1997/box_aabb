#!/usr/bin/env python3
"""
检查规划路径在 Drake 模型中是否存在碰撞。
沿每段路径插值密集采样点，输出最小距离和碰撞体对。

用法:
    python viz/check_collision.py result/exp2_paths_XXX.json
"""
import argparse
import json
import os
import sys

import numpy as np

from pydrake.geometry import SceneGraphInspector, QueryObject, Role
from pydrake.multibody.parsing import (
    LoadModelDirectives, Parser, ProcessModelDirectives,
)
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.systems.framework import DiagramBuilder

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_V5_ROOT = os.path.dirname(_THIS_DIR)
_PROJ_ROOT = os.path.dirname(os.path.dirname(_V5_ROOT))
_GCS_ROOT = os.path.join(_PROJ_ROOT, "gcs-science-robotics")
_MODELS_DIR = os.path.join(_GCS_ROOT, "models")
_YAML_FILE = os.path.join(_MODELS_DIR, "iiwa14_welded_gripper.yaml")

PAIR_NAMES = ["AS→TS", "TS→CS", "CS→LB", "LB→RB", "RB→AS"]


def build_plant():
    """构建 Drake MultibodyPlant + SceneGraph (无可视化)。"""
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant)
    parser.package_map().Add("gcs", _GCS_ROOT)
    directives = LoadModelDirectives(_YAML_FILE)
    ProcessModelDirectives(directives, plant, parser)
    plant.Finalize()
    diagram = builder.Build()
    return diagram, plant, scene_graph


def check_collision_at_q(diagram, plant, scene_graph, q):
    """
    设置关节角 q，返回 (min_distance, body_pair_name, contact_point_info)。
    min_distance < 0 表示穿透。
    """
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)
    sg_context = scene_graph.GetMyContextFromRoot(context)

    # 设置关节角 (只设前 7 个 = iiwa joints)
    plant.SetPositions(plant_context, q[:7])

    query_port = scene_graph.get_query_output_port()
    query_object = query_port.Eval(sg_context)

    # 获取所有碰撞对的签名距离
    inspector = query_object.inspector()
    signed_distances = query_object.ComputeSignedDistancePairwiseClosestPoints()

    if len(signed_distances) == 0:
        return float('inf'), "no_pairs", ""

    min_dist = float('inf')
    min_pair = ""
    min_info = ""

    for sd in signed_distances:
        if sd.distance < min_dist:
            min_dist = sd.distance
            nameA = inspector.GetName(inspector.GetFrameId(sd.id_A))
            nameB = inspector.GetName(inspector.GetFrameId(sd.id_B))
            min_pair = f"{nameA} <-> {nameB}"
            min_info = (f"dist={sd.distance:.6f}  "
                        f"ptA=({sd.p_ACa[0]:.4f},{sd.p_ACa[1]:.4f},{sd.p_ACa[2]:.4f})  "
                        f"ptB=({sd.p_BCb[0]:.4f},{sd.p_BCb[1]:.4f},{sd.p_BCb[2]:.4f})")

    return min_dist, min_pair, min_info


def check_path(diagram, plant, scene_graph, waypoints, n_interp=20):
    """
    沿路径插值 n_interp 个点并检查碰撞。
    返回 (worst_dist, worst_pair, worst_info, worst_alpha, worst_seg)。
    """
    worst_dist = float('inf')
    worst_pair = ""
    worst_info = ""
    worst_alpha = 0
    worst_seg = 0
    n_collisions = 0

    for seg in range(len(waypoints) - 1):
        q0 = waypoints[seg]
        q1 = waypoints[seg + 1]
        for k in range(n_interp + 1):
            alpha = k / n_interp
            q = q0 + alpha * (q1 - q0)
            dist, pair, info = check_collision_at_q(
                diagram, plant, scene_graph, q)
            if dist < 0:
                n_collisions += 1
            if dist < worst_dist:
                worst_dist = dist
                worst_pair = pair
                worst_info = info
                worst_alpha = alpha
                worst_seg = seg

    return worst_dist, worst_pair, worst_info, worst_alpha, worst_seg, n_collisions


def main():
    parser = argparse.ArgumentParser(description="检查规划路径碰撞")
    parser.add_argument("json_file", help="路径 JSON 文件")
    parser.add_argument("--interp", type=int, default=50,
                        help="每段路径插值点数 (默认: 50)")
    args = parser.parse_args()

    if not os.path.exists(args.json_file):
        print(f"错误: 找不到 {args.json_file}")
        sys.exit(1)

    with open(args.json_file) as f:
        data = json.load(f)

    print(f"构建 Drake 模型 ({_YAML_FILE})...")
    diagram, plant, scene_graph = build_plant()
    print(f"关节数: {plant.num_positions()}")

    print(f"\n检查碰撞 (每段 {args.interp} 个插值点):")
    print("=" * 80)

    any_collision = False
    for path_data in data["paths"]:
        label = path_data["label"]
        success = path_data["success"]
        wp = np.array(path_data["waypoints"])

        if not success or len(wp) == 0:
            print(f"  {label:8s}  SKIP (规划失败)")
            continue

        worst_dist, worst_pair, worst_info, worst_alpha, worst_seg, n_col = \
            check_path(diagram, plant, scene_graph, wp, n_interp=args.interp)

        status = "COLLISION" if worst_dist < 0 else "OK"
        if worst_dist < 0:
            any_collision = True

        print(f"\n  {label:8s}  {status}  min_dist={worst_dist:.6f}m  "
              f"碰撞采样点={n_col}")
        print(f"           最近体对: {worst_pair}")
        print(f"           {worst_info}")
        if worst_dist < 0:
            print(f"           穿透位置: seg={worst_seg} alpha={worst_alpha:.2f}")

    print("\n" + "=" * 80)
    if any_collision:
        print("⚠ 存在碰撞！")
    else:
        print("✓ 所有路径无碰撞")


if __name__ == "__main__":
    main()

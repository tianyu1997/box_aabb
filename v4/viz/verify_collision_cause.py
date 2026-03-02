#!/usr/bin/env python3
"""
verify_collision_cause.py — 精确验证碰撞成因

假设 1: WSG 夹爪不在 SBF 碰撞模型中 → SBF 认为 free 的配置实际碰撞
假设 2: 线性插值在 waypoint 之间离开 box → 路径经过未认证空间

诊断方法:
  对每个 Drake 报告碰撞的点:
    (a) 分离碰撞来源: iiwa链 vs wsg夹爪 vs 其他
    (b) 检查该点是否在 SBF box 内
    (c) 如果在 box 内但 Drake 报碰 → 模型缺口 (假设1)
       如果不在 box 内且 Drake 报碰 → 出界碰撞 (假设2)
"""

import os, sys, pickle
import numpy as np

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_V4_ROOT = os.path.dirname(_THIS_DIR)
_PROJ_ROOT = os.path.dirname(_V4_ROOT)
_EXP_DIR = os.path.join(_V4_ROOT, "experiments")
_GCS_ROOT = os.path.join(_PROJ_ROOT, "gcs-science-robotics")
_MODELS_DIR = os.path.join(_GCS_ROOT, "models")

sys.path.insert(0, _EXP_DIR)

PAIR_NAMES = ["AS→TS", "TS→CS", "CS→LB", "LB→RB", "RB→AS"]


def load_data(seed=0):
    cache = os.path.join(_V4_ROOT, "output", "exp2_paths", "sbf_paths.pkl")
    with open(cache, "rb") as f:
        data = pickle.load(f)
    paths = [d for d in data if d.get("seed", 0) == seed and d.get("success")]

    forest_path = os.path.join(_V4_ROOT, "output", "exp1_artifacts",
                               f"sbf_forest_seed{seed:03d}.pkl")
    with open(forest_path, "rb") as f:
        forest = pickle.load(f)
    return paths, forest


def build_drake_checker():
    from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
    from pydrake.multibody.parsing import (
        Parser, LoadModelDirectives, ProcessModelDirectives)
    from pydrake.systems.framework import DiagramBuilder

    yaml_file = os.path.join(
        _MODELS_DIR, "iiwa14_spheres_collision_welded_gripper.yaml")

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant)
    parser.package_map().Add("gcs", _GCS_ROOT)

    directives = LoadModelDirectives(yaml_file)
    models = ProcessModelDirectives(directives, plant, parser)
    plant.Finalize()
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    return diagram, plant, scene_graph, models, context


def classify_collision(diagram, plant, scene_graph, models, context, q):
    """对一个配置 q 做 Drake 碰撞检测，按碰撞来源分类.
    
    Returns:
        dict with:
          has_collision: bool
          min_dist: float
          iiwa_collisions: list  (iiwa body ↔ obstacle)
          wsg_collisions: list   (wsg body ↔ obstacle)
          other_collisions: list  
    """
    plant_context = plant.GetMyMutableContextFromRoot(context)
    iiwa_model = models[0].model_instance
    plant.SetPositions(plant_context, iiwa_model, q)

    sg_context = scene_graph.GetMyMutableContextFromRoot(context)
    query_object = scene_graph.get_query_output_port().Eval(sg_context)
    inspector = scene_graph.model_inspector()

    signed_dists = query_object.ComputeSignedDistancePairwiseClosestPoints(
        max_distance=0.5)

    result = {
        "has_collision": False,
        "min_dist": float("inf"),
        "iiwa_collisions": [],
        "wsg_collisions": [],
        "other_collisions": [],
    }

    for sd in signed_dists:
        dist = sd.distance
        if dist < result["min_dist"]:
            result["min_dist"] = dist

        if dist >= 0:
            continue  # no penetration

        nameA = inspector.GetName(sd.id_A)
        nameB = inspector.GetName(sd.id_B)

        # 判断碰撞来源
        is_iiwa_A = "iiwa" in nameA.lower()
        is_iiwa_B = "iiwa" in nameB.lower()
        is_wsg_A = "wsg" in nameA.lower()
        is_wsg_B = "wsg" in nameB.lower()

        entry = {"nameA": nameA, "nameB": nameB, "distance": dist}

        if is_wsg_A or is_wsg_B:
            result["wsg_collisions"].append(entry)
        elif is_iiwa_A or is_iiwa_B:
            result["iiwa_collisions"].append(entry)
        else:
            result["other_collisions"].append(entry)

    result["has_collision"] = result["min_dist"] < 0
    return result


def point_in_any_box(q, lo, hi, tol=1e-6):
    inside = np.all((q >= lo - tol) & (q <= hi + tol), axis=1)
    return bool(np.any(inside))


def main():
    seed = 0
    n_interp = 50
    paths, forest = load_data(seed)
    lo = forest["intervals_lo"]
    hi = forest["intervals_hi"]

    print("构建 Drake 碰撞检测器 ...")
    drake = build_drake_checker()
    print("就绪\n")

    # 统计汇总
    total_samples = 0
    stats = {
        "in_box_wsg_col": 0,      # 假设1: 在box内, wsg碰撞
        "in_box_iiwa_col": 0,     # 模型问题: 在box内, iiwa碰撞
        "out_box_wsg_col": 0,     # 假设2: 出界, wsg碰撞
        "out_box_iiwa_col": 0,    # 假设2: 出界, iiwa碰撞
        "in_box_no_col": 0,       # 正常: 在box内, 无碰撞
        "out_box_no_col": 0,      # 出界但不碰
    }

    for entry in paths:
        pi = entry["pair_idx"]
        wp = entry.get("path_waypoints")
        if wp is None:
            continue

        print(f"{'─'*60}")
        print(f"  [{PAIR_NAMES[pi]}]  waypoints: {wp.shape}")
        print(f"{'─'*60}")

        all_points = []
        labels = []

        # 收集 waypoints
        for i in range(len(wp)):
            all_points.append(wp[i])
            labels.append(f"wp[{i}]")

        # 收集插值点
        for i in range(len(wp) - 1):
            for t in np.linspace(0, 1, n_interp + 2)[1:-1]:
                q = wp[i] + t * (wp[i+1] - wp[i])
                all_points.append(q)
                labels.append(f"seg[{i}→{i+1}] t={t:.3f}")

        pair_stats = {"in_box_wsg": 0, "in_box_iiwa": 0,
                      "out_box_wsg": 0, "out_box_iiwa": 0,
                      "out_box_col": 0, "in_box_col": 0,
                      "total": len(all_points)}

        for q, label in zip(all_points, labels):
            in_box = point_in_any_box(q, lo, hi)
            col = classify_collision(*drake, q)
            total_samples += 1

            has_wsg = len(col["wsg_collisions"]) > 0
            has_iiwa = len(col["iiwa_collisions"]) > 0

            if in_box:
                if has_wsg:
                    stats["in_box_wsg_col"] += 1
                    pair_stats["in_box_wsg"] += 1
                if has_iiwa:
                    stats["in_box_iiwa_col"] += 1
                    pair_stats["in_box_iiwa"] += 1
                if col["has_collision"]:
                    pair_stats["in_box_col"] += 1
                else:
                    stats["in_box_no_col"] += 1
            else:
                if has_wsg:
                    stats["out_box_wsg_col"] += 1
                    pair_stats["out_box_wsg"] += 1
                if has_iiwa:
                    stats["out_box_iiwa_col"] += 1
                    pair_stats["out_box_iiwa"] += 1
                if col["has_collision"]:
                    pair_stats["out_box_col"] += 1
                else:
                    stats["out_box_no_col"] += 1

        print(f"  Total samples: {pair_stats['total']}")
        print(f"  在 box 内碰撞: {pair_stats['in_box_col']}  "
              f"(wsg:{pair_stats['in_box_wsg']} iiwa:{pair_stats['in_box_iiwa']})")
        print(f"  出 box 后碰撞: {pair_stats['out_box_col']}  "
              f"(wsg:{pair_stats['out_box_wsg']} iiwa:{pair_stats['out_box_iiwa']})")
        print()

    print(f"\n{'='*60}")
    print(f"  全局统计 (total={total_samples})")
    print(f"{'='*60}")
    print(f"  假设1 — 在box内, WSG碰撞:   {stats['in_box_wsg_col']}")
    print(f"  假设1 — 在box内, IIWA碰撞:  {stats['in_box_iiwa_col']}")
    print(f"  假设2 — 出box, WSG碰撞:     {stats['out_box_wsg_col']}")
    print(f"  假设2 — 出box, IIWA碰撞:    {stats['out_box_iiwa_col']}")
    print(f"  正常 — 在box内, 无碰撞:     {stats['in_box_no_col']}")
    print(f"  出box但无碰撞:              {stats['out_box_no_col']}")
    print()

    total_col = (stats['in_box_wsg_col'] + stats['in_box_iiwa_col']
                 + stats['out_box_wsg_col'] + stats['out_box_iiwa_col'])
    if total_col > 0:
        pct1 = (stats['in_box_wsg_col'] + stats['in_box_iiwa_col']) / total_col * 100
        pct2 = (stats['out_box_wsg_col'] + stats['out_box_iiwa_col']) / total_col * 100
        print(f"  假设1 (在box内碰撞) 占碰撞总数: {pct1:.1f}%")
        print(f"  假设2 (出box碰撞)  占碰撞总数: {pct2:.1f}%")
        pct_wsg = (stats['in_box_wsg_col'] + stats['out_box_wsg_col']) / total_col * 100
        pct_iiwa = (stats['in_box_iiwa_col'] + stats['out_box_iiwa_col']) / total_col * 100
        print(f"  WSG夹爪碰撞 占碰撞总数:        {pct_wsg:.1f}%")
        print(f"  IIWA臂碰撞 占碰撞总数:         {pct_iiwa:.1f}%")


if __name__ == "__main__":
    main()

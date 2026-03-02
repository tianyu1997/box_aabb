#!/usr/bin/env python3
"""
diagnose_collision.py — 诊断 exp2 GCS 路径碰撞问题

三层检查:
  1. 路径 waypoint 是否在 SBF box 内?
  2. 线性插值点是否在 SBF box 内?
  3. Drake 碰撞检测 (精确几何) 是否报告碰撞?

用法:
    cd v4 && python -m viz.diagnose_collision
"""

import os
import pickle
import sys

import numpy as np

# ── 路径设置 ──
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_V4_ROOT = os.path.dirname(_THIS_DIR)
_PROJ_ROOT = os.path.dirname(_V4_ROOT)
_EXP_DIR = os.path.join(_V4_ROOT, "experiments")
_GCS_ROOT = os.path.join(_PROJ_ROOT, "gcs-science-robotics")
_MODELS_DIR = os.path.join(_GCS_ROOT, "models")

sys.path.insert(0, _EXP_DIR)

PAIR_NAMES = ["AS→TS", "TS→CS", "CS→LB", "LB→RB", "RB→AS"]

# ═══════════════════════════════════════════════════════════════════════════
# 1. 加载缓存路径 & SBF forest
# ═══════════════════════════════════════════════════════════════════════════

def load_paths(seed=0):
    """加载 exp2 缓存路径."""
    cache = os.path.join(_V4_ROOT, "output", "exp2_paths", "sbf_paths.pkl")
    with open(cache, "rb") as f:
        data = pickle.load(f)
    # 筛选指定 seed
    return [d for d in data if d.get("seed", 0) == seed and d.get("success", False)]


def load_forest(seed=0):
    """加载 exp1 保存的 SBF forest artifact."""
    path = os.path.join(_V4_ROOT, "output", "exp1_artifacts",
                        f"sbf_forest_seed{seed:03d}.pkl")
    with open(path, "rb") as f:
        return pickle.load(f)


# ═══════════════════════════════════════════════════════════════════════════
# 2. SBF box 检测: waypoint 是否在 box 内?
# ═══════════════════════════════════════════════════════════════════════════

def check_in_box(q, intervals_lo, intervals_hi, tol=1e-6):
    """检查 q 是否在任何 box 内."""
    inside = np.all((q >= intervals_lo - tol) & (q <= intervals_hi + tol), axis=1)
    return np.any(inside), np.where(inside)[0]


def check_path_in_boxes(path, intervals_lo, intervals_hi, n_interp=20):
    """检查路径上所有 waypoint 和插值点是否在 box 内.
    
    Returns:
        dict with details of any out-of-box configurations
    """
    results = {
        "waypoints_in_box": [],
        "interp_out_of_box": [],
        "n_waypoints": len(path),
        "n_interp_samples": 0,
    }
    
    # Check waypoints
    for i, q in enumerate(path):
        in_box, box_ids = check_in_box(q, intervals_lo, intervals_hi)
        results["waypoints_in_box"].append({
            "idx": i, "in_box": in_box, "n_boxes": len(box_ids),
            "q": q.copy()
        })
        if not in_box:
            # 找最近 box 的距离
            diff_lo = np.maximum(intervals_lo - q, 0)
            diff_hi = np.maximum(q - intervals_hi, 0)
            linf = np.max(np.maximum(diff_lo, diff_hi), axis=1)
            nearest = np.argmin(linf)
            results["waypoints_in_box"][-1]["min_dist"] = float(linf[nearest])
    
    # Check interpolated points
    total_interp = 0
    for i in range(len(path) - 1):
        q0, q1 = path[i], path[i + 1]
        seg_len = np.linalg.norm(q1 - q0)
        for t in np.linspace(0, 1, n_interp + 2)[1:-1]:  # exclude endpoints
            q = q0 + t * (q1 - q0)
            in_box, _ = check_in_box(q, intervals_lo, intervals_hi)
            total_interp += 1
            if not in_box:
                diff_lo = np.maximum(intervals_lo - q, 0)
                diff_hi = np.maximum(q - intervals_hi, 0)
                linf = np.max(np.maximum(diff_lo, diff_hi), axis=1)
                nearest = np.argmin(linf)
                results["interp_out_of_box"].append({
                    "seg": (i, i+1), "t": float(t),
                    "min_dist": float(linf[nearest]),
                    "q": q.copy()
                })
    results["n_interp_samples"] = total_interp
    return results


# ═══════════════════════════════════════════════════════════════════════════
# 3. Drake 碰撞检测
# ═══════════════════════════════════════════════════════════════════════════

def build_collision_checker():
    """构建 Drake MultibodyPlant 用于精确碰撞检测."""
    from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
    from pydrake.multibody.parsing import (
        Parser, LoadModelDirectives, ProcessModelDirectives)
    from pydrake.systems.framework import DiagramBuilder
    from pydrake.geometry import SceneGraphInspector

    # 使用与 SBF 相同的 sphere collision model
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


def check_collision_drake(diagram, plant, scene_graph, models, context,
                          q, min_dist_threshold=0.0):
    """使用 Drake 精确碰撞检测检查配置 q.
    
    Returns:
        (has_collision, min_distance, collision_pairs)
    """
    plant_context = plant.GetMyMutableContextFromRoot(context)

    # 设置关节角
    iiwa_model = models[0].model_instance
    plant.SetPositions(plant_context, iiwa_model, q)

    # 获取碰撞查询
    sg_context = scene_graph.GetMyMutableContextFromRoot(context)
    query_object = scene_graph.get_query_output_port().Eval(sg_context)

    # 检查所有碰撞对的 signed distance
    signed_dists = query_object.ComputeSignedDistancePairwiseClosestPoints(
        max_distance=0.5)  # only check within 0.5m

    min_dist = float("inf")
    collision_pairs = []
    inspector = scene_graph.model_inspector()

    for sd in signed_dists:
        dist = sd.distance
        if dist < min_dist:
            min_dist = dist

        if dist < min_dist_threshold:
            # 获取几何体名称
            nameA = inspector.GetName(sd.id_A)
            nameB = inspector.GetName(sd.id_B)
            # 过滤自碰撞 (都属于 iiwa/wsg)
            bodyA = inspector.GetFrameId(sd.id_A)
            bodyB = inspector.GetFrameId(sd.id_B)
            collision_pairs.append({
                "nameA": nameA,
                "nameB": nameB,
                "distance": float(dist),
            })

    has_collision = min_dist < min_dist_threshold
    return has_collision, float(min_dist), collision_pairs


def check_path_collision_drake(path, diagram, plant, scene_graph, models,
                                context, n_interp=20):
    """检查整条路径的 Drake 碰撞.
    
    Returns:
        list of collision reports for each segment
    """
    results = []
    iiwa_model = models[0].model_instance
    
    # 检查 waypoints
    print("  检查 waypoints ...")
    for i, q in enumerate(path):
        has_col, min_dist, pairs = check_collision_drake(
            diagram, plant, scene_graph, models, context, q)
        status = "❌ COLLISION" if has_col else "✓ ok"
        print(f"    wp[{i}]: min_dist={min_dist:.4f}  {status}")
        if has_col:
            for p in pairs[:3]:
                print(f"      {p['nameA']} ↔ {p['nameB']}: {p['distance']:.4f}")
        results.append({
            "type": "waypoint", "idx": i,
            "has_collision": has_col, "min_dist": min_dist,
            "pairs": pairs
        })

    # 检查插值点
    print(f"  检查插值点 (n_interp={n_interp}) ...")
    n_collisions = 0
    for i in range(len(path) - 1):
        q0, q1 = path[i], path[i + 1]
        seg_min_dist = float("inf")
        seg_collisions = 0
        worst_t = -1
        for t in np.linspace(0, 1, n_interp + 2)[1:-1]:
            q = q0 + t * (q1 - q0)
            has_col, min_dist, pairs = check_collision_drake(
                diagram, plant, scene_graph, models, context, q)
            if min_dist < seg_min_dist:
                seg_min_dist = min_dist
                worst_t = t
            if has_col:
                seg_collisions += 1
                n_collisions += 1

        status = f"❌ {seg_collisions}/{n_interp} collide" if seg_collisions else "✓ ok"
        print(f"    seg[{i}→{i+1}]: min_dist={seg_min_dist:.4f} (t={worst_t:.2f})  {status}")

    print(f"  总计: {n_collisions}/{(len(path)-1)*n_interp} 插值点碰撞")
    return results


# ═══════════════════════════════════════════════════════════════════════════
# 4. SBF C++ 碰撞检测 (如果可用)
# ═══════════════════════════════════════════════════════════════════════════

def check_path_sbf_collision(path, n_interp=20):
    """使用 SBF Python 碰撞检测器检查路径碰撞."""
    try:
        # 加载障碍物
        from marcucci_scenes import build_combined_obstacles
        obs = build_combined_obstacles()

        # 使用 v4 Python 碰撞检测器
        src_path = os.path.join(_V4_ROOT, "src")
        if src_path not in sys.path:
            sys.path.insert(0, src_path)
        
        from aabb.robot import Robot
        from aabb.scene import Scene, Obstacle
        from forest.collision import CollisionChecker

        config_path = os.path.join(_V4_ROOT, "src", "aabb", "configs", "iiwa14.json")
        robot = Robot.from_json(config_path)
        
        obstacles_list = [Obstacle(np.array(o["min"]), np.array(o["max"]), name=o["name"])
                          for o in obs]
        scene = Scene(obstacles_list)
        checker = CollisionChecker(robot, scene)

        n_col = 0
        total = 0
        for i in range(len(path) - 1):
            q0, q1 = path[i], path[i + 1]
            for t in np.linspace(0, 1, n_interp + 2):
                q = q0 + t * (q1 - q0)
                in_collision = checker.check_config_collision(q)
                total += 1
                if in_collision:
                    n_col += 1
                    if n_col <= 5:
                        print(f"    SBF collision at seg[{i}→{i+1}] t={t:.2f}")

        print(f"    SBF 碰撞: {n_col}/{total} 点碰撞")
        return n_col
    except Exception as e:
        print(f"    [ERROR] SBF 碰撞检测失败: {e}")
        return None


# ═══════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--pair", type=int, default=-1,
                        help="只检查指定 pair (-1=全部)")
    parser.add_argument("--n-interp", type=int, default=50,
                        help="每段插值点数")
    parser.add_argument("--skip-drake", action="store_true",
                        help="跳过 Drake 碰撞检测")
    args = parser.parse_args()

    print(f"=" * 70)
    print(f"  exp2 路径碰撞诊断  (seed={args.seed})")
    print(f"=" * 70)

    # ── 加载数据 ──
    paths = load_paths(args.seed)
    forest = load_forest(args.seed)
    intervals_lo = forest["intervals_lo"]
    intervals_hi = forest["intervals_hi"]

    print(f"\n  已加载 {len(paths)} 条成功路径, "
          f"forest: {len(intervals_lo)} boxes")

    # ── Drake 碰撞检测器 ──
    drake_checker = None
    if not args.skip_drake:
        print("\n  构建 Drake 碰撞检测器 ...")
        drake_checker = build_collision_checker()
        print("  Drake 碰撞检测器就绪")

    # ── 逐条路径检查 ──
    for entry in paths:
        pi = entry["pair_idx"]
        if args.pair >= 0 and pi != args.pair:
            continue

        wp = entry.get("path_waypoints")
        if wp is None:
            continue

        print(f"\n{'─' * 60}")
        print(f"  [{PAIR_NAMES[pi]}]  waypoints: {wp.shape}")
        print(f"{'─' * 60}")

        # ── Check 1: Waypoints 在 box 内? ──
        print("\n  ▸ Check 1: waypoints 在 SBF box 内?")
        box_result = check_path_in_boxes(wp, intervals_lo, intervals_hi,
                                          n_interp=args.n_interp)
        
        n_wp_out = sum(1 for w in box_result["waypoints_in_box"] if not w["in_box"])
        n_interp_out = len(box_result["interp_out_of_box"])
        
        for w in box_result["waypoints_in_box"]:
            status = f"✓ in {w['n_boxes']} boxes" if w["in_box"] else f"❌ OUT (dist={w.get('min_dist', '?'):.6f})"
            print(f"    wp[{w['idx']}]: {status}")
        
        print(f"    插值点: {n_interp_out}/{box_result['n_interp_samples']} 出界")
        if n_interp_out > 0:
            # 每段统计
            seg_counts = {}
            for r in box_result["interp_out_of_box"]:
                seg = r["seg"]
                seg_counts[seg] = seg_counts.get(seg, 0) + 1
            for seg, cnt in sorted(seg_counts.items()):
                max_dist = max(r["min_dist"] for r in box_result["interp_out_of_box"]
                              if r["seg"] == seg)
                print(f"      seg[{seg[0]}→{seg[1]}]: {cnt} 出界, max_dist={max_dist:.6f}")

        # ── Check 2: Drake 碰撞检测 ──
        if drake_checker:
            print(f"\n  ▸ Check 2: Drake 精确碰撞检测")
            check_path_collision_drake(
                wp, *drake_checker, n_interp=args.n_interp)

        # ── Check 3: SBF 碰撞检测 ──
        print(f"\n  ▸ Check 3: SBF C++ 碰撞检测")
        check_path_sbf_collision(wp, n_interp=args.n_interp)

    print(f"\n{'=' * 70}")
    print("  诊断完成")
    print(f"{'=' * 70}")


if __name__ == "__main__":
    main()

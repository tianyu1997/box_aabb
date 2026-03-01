"""
使用 Drake (IRIS 用的碰撞检测引擎) 检测 LB / RB 配置的最小碰撞距离
输出所有成对的最小有符号距离 (负值 = 穿透)
"""
import sys, os
import numpy as np
from pathlib import Path

_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(_ROOT / 'src'))
sys.path.insert(0, str(_ROOT / 'experiments'))

# ── milestone configs ──────────────────────────────────────────────────────────
LB = np.array([ 1.3326656,  0.7865932,  0.3623384, -1.4916529, -0.3192509,  0.9217325,  1.7911904])
RB = np.array([-1.3324624,  0.7866478, -0.3626562, -1.4916528,  0.3195340,  0.9217833,  1.3502090])

CONFIGS = {"LB": LB, "RB": RB}

# ── Drake setup ────────────────────────────────────────────────────────────────
def find_gcs_dir():
    candidates = [
        Path.home() / "桌面" / "box_aabb" / "gcs-science-robotics",
        _ROOT.parent / "gcs-science-robotics",
    ]
    for p in candidates:
        if p.is_dir():
            return str(p)
    return None

def build_plant():
    from pydrake.systems.framework import DiagramBuilder
    from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
    from pydrake.multibody.parsing import (
        Parser, LoadModelDirectives, ProcessModelDirectives)

    gcs_dir = find_gcs_dir()
    assert gcs_dir, "gcs-science-robotics not found"

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant, scene_graph)
    pm = parser.package_map()
    pm.Add("gcs", gcs_dir)

    try:
        import pydrake as _pyd
        _drake_share = os.path.join(os.path.dirname(_pyd.__file__), "share", "drake")
        _dm_local = os.path.join(_drake_share, "manipulation", "models")
        if os.path.isdir(_dm_local):
            if pm.Contains("drake_models"):
                pm.Remove("drake_models")
            pm.Add("drake_models", _dm_local)
    except Exception as e:
        print(f"  [warn] drake_models: {e}")

    directives_file = os.path.join(gcs_dir, "models", "iiwa14_welded_gripper.yaml")
    directives = LoadModelDirectives(directives_file)
    ProcessModelDirectives(directives, plant, parser)
    plant.Finalize()
    diagram = builder.Build()
    return plant, diagram

def check_config(name, q, plant, diagram):
    """用Drake QueryObject计算最小碰撞距离"""
    from pydrake.geometry import SignedDistancePair

    context = diagram.CreateDefaultContext()
    plant_ctx = plant.GetMyContextFromRoot(context)
    plant.SetPositions(plant_ctx, q)

    scene_graph = diagram.GetSubsystemByName("scene_graph")
    sg_ctx = scene_graph.GetMyContextFromRoot(context)
    query_obj = scene_graph.get_query_output_port().Eval(sg_ctx)

    # 所有碰撞对的有符号距离 (负 = 穿透)
    pairs = query_obj.ComputeSignedDistancePairwiseClosestPoints(max_distance=0.15)

    print(f"\n{'='*60}")
    print(f"  Config: {name}")
    print(f"  q = {np.round(q, 5).tolist()}")
    print(f"  {'='*58}")

    inspector = query_obj.inspector()
    
    if len(pairs) == 0:
        print("  ✓ No geometry pairs within 0.15m — fully clear")
        return

    # 按距离排序，显示最近的
    pairs_sorted = sorted(pairs, key=lambda p: p.distance)
    
    min_dist = pairs_sorted[0].distance
    n_penetrating = sum(1 for p in pairs_sorted if p.distance < 0)

    print(f"  Total pairs within 0.15m: {len(pairs_sorted)}")
    print(f"  Penetrating pairs:        {n_penetrating}")
    print(f"  Min distance:             {min_dist:.6f} m")
    print()
    print(f"  {'Dist(m)':>10}  {'Geometry A':^35}  {'Geometry B':^35}")
    print(f"  {'-'*10}  {'-'*35}  {'-'*35}")
    
    for p in pairs_sorted[:20]:  # 显示最近20对
        try:
            name_a = inspector.GetName(p.id_A)
            name_b = inspector.GetName(p.id_B)
        except Exception:
            name_a = str(p.id_A)
            name_b = str(p.id_B)
        marker = " ← PENETRATE" if p.distance < 0 else ""
        print(f"  {p.distance:>10.5f}  {name_a:^35}  {name_b:^35}{marker}")

    # 也尝试 EvalDeformableContact 不可用时用 MinimumDistanceLowerBound
    print()
    try:
        min_lb = query_obj.ComputeMinimumDistanceLowerBound(0.0)
        print(f"  MinimumDistanceLowerBound(0): {min_lb:.6f}")
    except Exception:
        pass

def main():
    print("Building Drake plant (iiwa14_welded_gripper.yaml)...")
    plant, diagram = build_plant()
    print(f"Plant built: {plant.num_positions()} DOF, "
          f"{plant.num_collision_geometries()} collision geometries")

    for name, q in CONFIGS.items():
        check_config(name, q, plant, diagram)

if __name__ == "__main__":
    main()

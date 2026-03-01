"""
experiments/marcucci_scenes.py — Marcucci et al. (2024) 基准场景定义

复现 "Motion Planning around Obstacles with Convex Optimization"
(Science Robotics, 2024) 中 iiwa14 单臂实验场景.

场景来源: github.com/RobotLocomotion/gcs-science-robotics
  - shelves: 窄通道书架 (narrow passages)
  - bins:    左右料箱 (medium difficulty)
  - table:   宽桌面 (open space, baseline)

所有障碍物已从 SDF collision 几何 → 世界坐标 AABB 转换,
线段抽象补偿由连杆 AABB 膨胀 (link_radii) 实现, 与障碍物无关.

Query pairs 来自 Marcucci 论文的 IK milestones:
  AS = Above Shelf, TS = Top Shelf, CS = Center Shelf
  LB = Left Bin, RB = Right Bin, C = Center, L = Left, R = Right
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


# ═══════════════════════════════════════════════════════════════════════════
# Helper: SDF Box → AABB in world frame
# ═══════════════════════════════════════════════════════════════════════════

def _sdf_box_to_aabb(
    parent_world_pos: np.ndarray,   # (3,) world position of SDF model origin
    parent_world_rpy_deg: np.ndarray,  # (3,) roll-pitch-yaw in degrees
    local_pose: np.ndarray,         # (3,) local translation of collision elem
    size: np.ndarray,               # (3,) full box size (x, y, z)
) -> Tuple[np.ndarray, np.ndarray]:
    """将 SDF collision box 转换为世界坐标 AABB (min, max).

    对于旋转过的 parent frame, 先将 box 8 个顶点旋转到世界坐标,
    再取 axis-aligned bounding box.

    注: 障碍物不再膨胀, 线段抽象补偿由连杆 AABB 膨胀 (link_radii) 实现.
    """
    half = np.array(size, dtype=np.float64) / 2.0

    # 8 个顶点 (局部坐标)
    signs = np.array(list(itertools.product([-1, 1], repeat=3)), dtype=np.float64)
    local_vertices = signs * half  # (8, 3)

    # 加上 local offset
    local_vertices += np.array(local_pose, dtype=np.float64)

    # parent frame rotation (RPY in degrees → radians)
    rpy = np.deg2rad(parent_world_rpy_deg)
    R = _rpy_to_rotation_matrix(rpy[0], rpy[1], rpy[2])

    # 世界坐标
    world_vertices = (R @ local_vertices.T).T + np.array(parent_world_pos, dtype=np.float64)

    aabb_min = world_vertices.min(axis=0)
    aabb_max = world_vertices.max(axis=0)
    return aabb_min, aabb_max


def _rpy_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """RPY (extrinsic XYZ) → 3×3 rotation matrix."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ], dtype=np.float64)


# ═══════════════════════════════════════════════════════════════════════════
# Scene dataclass
# ═══════════════════════════════════════════════════════════════════════════

@dataclass
class MarcucciScene:
    """一个 Marcucci 基准场景."""
    name: str
    obstacles: List[Dict]           # [{"name": str, "min": [x,y,z], "max": [x,y,z]}]
    query_pairs: List[Tuple[str, str]]  # IK milestone 名称对
    difficulty: str = "medium"      # "easy" / "medium" / "hard"
    description: str = ""


# ═══════════════════════════════════════════════════════════════════════════
# iiwa14 IK milestone 配置 (from Marcucci prm_comparison notebook)
# ═══════════════════════════════════════════════════════════════════════════

# 这些是通过 IK 求解得到的关节配置,  对应 iiwa14 + welded WSG gripper
# 的特定末端执行器姿态.  直接使用关节空间配置, 无需再次 IK 求解.
#
# milestones (task-space定义, 通过 IK 求解):
#   AS = Above Shelf:   pos=[0.75, 0, 0.9],   rpy=[0, -π, -π/2]
#   TS = Top Shelf:     pos=[0.75, 0, 0.67],  rpy=[0, -π, -π/2]
#   CS = Center Shelf:  pos=[0.75, 0, 0.41],  rpy=[0, -π, -π/2]
#   LB = Left Bin:      pos=[0, 0.6, 0.22],   rpy=[π/2, π, 0]
#   RB = Right Bin:     pos=[0, -0.6, 0.22],  rpy=[π/2, π, π]
#
# additional seed points (直接关节空间):
#   C = Center, L = Left, R = Right

# 注意: milestone_configurations 需要通过 Drake IK 求解.
# 以下是预计算的结果 (使用 q0 = [0, 0.3, 0, -1.8, 0, 1, 1.57] 作为 IK 初始猜测)
# 如果你的 pydrake 版本不同, 可能需要重新计算.

IIWA14_SEED_POINTS = {
    # --- Marcucci 论文直接定义的关节空间配置 ---
    "C": np.array([0.0,  0.2,  0.0, -2.09, 0.0, -0.3,  np.pi / 2]),
    "L": np.array([0.8,  0.7,  0.0, -1.6,  0.0,  0.0,  np.pi / 2]),
    "R": np.array([-0.8, 0.7,  0.0, -1.6,  0.0,  0.0,  np.pi / 2]),
}

# IK milestones — 预计算值 (用 iiwa14_spheres_collision_welded_gripper.yaml + q0=[0,0.3,0,-1.8,0,1,1.57] 计算)
# 已验证: 所有值均在 IRIS.reg 对应 region 内 (PointInSet=True)
IIWA14_IK_MILESTONES_PRECOMPUTED = {
    "AS": np.array([ 6.42e-05,  0.4719533, -0.0001493, -0.6716735,  0.0001854,  0.4261696,  1.5706922]),
    "TS": np.array([-1.55e-04,  0.3972726,  0.0002196, -1.3674756,  0.0002472, -0.1929518,  1.5704688]),
    "CS": np.array([-1.76e-04,  0.6830279,  0.0002450, -1.6478229,  2.09e-05,  -0.7590545,  1.5706263]),
    "LB": np.array([ 1.3326656,  0.7865932,  0.3623384, -1.4916529, -0.3192509,  0.9217325,  1.7911904]),
    "RB": np.array([-1.3324624,  0.7866478, -0.3626562, -1.4916528,  0.3195340,  0.9217833,  1.3502090]),
}


def _compute_ik_milestones_drake() -> Optional[Dict[str, np.ndarray]]:
    """尝试使用 Drake IK 计算 milestone configurations."""
    try:
        from pydrake.systems.framework import DiagramBuilder
        from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
        from pydrake.multibody.parsing import Parser, LoadModelDirectives, ProcessModelDirectives
        from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
        from pydrake.multibody import inverse_kinematics
        from pydrake.solvers import Solve
        import os

        # v4/experiments/ 向上 2 级到达 box_aabb/
        gcs_dir = os.path.normpath(
            os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "..", "..", "gcs-science-robotics"))
        if not os.path.isdir(gcs_dir):
            return None

        builder = DiagramBuilder()
        plant, _ = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
        parser = Parser(plant)
        parser.package_map().Add("gcs", gcs_dir)

        # 与 notebook helpers.py InverseKinematics 保持一致: 使用相同的模型文件
        directives_file = os.path.join(
            gcs_dir, "models", "iiwa14_spheres_collision_welded_gripper.yaml")
        if not os.path.exists(directives_file):
            return None

        directives = LoadModelDirectives(directives_file)
        ProcessModelDirectives(directives, plant, parser)
        plant.Finalize()

        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        plant_context = plant.GetMyMutableContextFromRoot(context)

        milestones_task = {
            "AS": ([0.75, 0, 0.9],   [0, -np.pi, -np.pi / 2]),
            "TS": ([0.75, 0, 0.67],  [0, -np.pi, -np.pi / 2]),
            "CS": ([0.75, 0, 0.41],  [0, -np.pi, -np.pi / 2]),
            "LB": ([0.0, 0.6, 0.22], [np.pi / 2, np.pi, 0]),
            "RB": ([0.0, -0.6, 0.22],[np.pi / 2, np.pi, np.pi]),
        }

        q0 = np.array([0, 0.3, 0, -1.8, 0, 1, 1.57])
        gripper_frame = plant.GetBodyByName("body").body_frame()
        result_configs = {}

        for name, (trans, rpy) in milestones_task.items():
            ik = inverse_kinematics.InverseKinematics(plant, plant_context)
            ik.AddPositionConstraint(
                gripper_frame, [0, 0, 0], plant.world_frame(),
                trans, trans)
            ik.AddOrientationConstraint(
                gripper_frame, RotationMatrix(),
                plant.world_frame(),
                RotationMatrix(RollPitchYaw(*rpy)), 0.001)

            prog = ik.get_mutable_prog()
            q = ik.q()
            prog.AddQuadraticErrorCost(np.identity(len(q)), q0, q)
            prog.SetInitialGuess(q, q0)
            sol = Solve(prog)
            if sol.is_success():
                result_configs[name] = sol.GetSolution(q)

        if len(result_configs) == len(milestones_task):
            return result_configs
        return None

    except ImportError:
        return None
    except Exception:
        return None


def get_iiwa14_seed_points() -> Dict[str, np.ndarray]:
    """获取所有 iiwa14 seed point 配置.

    优先使用 Drake IK 在线计算, 失败则使用预计算值.
    """
    # 尝试 Drake IK
    drake_milestones = _compute_ik_milestones_drake()
    if drake_milestones is not None:
        milestones = drake_milestones
    else:
        milestones = dict(IIWA14_IK_MILESTONES_PRECOMPUTED)

    return {**milestones, **IIWA14_SEED_POINTS}


def get_query_pairs(scene_name: str) -> List[Tuple[np.ndarray, np.ndarray]]:
    """获取指定场景的 (start, goal) query 对.

    每个场景选取有意义的查询对 (跨不同区域的规划).
    """
    pts = get_iiwa14_seed_points()

    if scene_name == "shelves":
        # 与 prm_comparison.ipynb notebook 完全相同的相邻 milestone 对
        # tasks_for_paper: AS->TS, TS->CS, CS->LB, LB->RB, RB->AS
        pairs = [
            ("AS", "TS"), ("TS", "CS"), ("CS", "LB"),
            ("LB", "RB"), ("RB", "AS"),
        ]
    elif scene_name == "bins":
        # 左右料箱之间以及中心位置往返
        pairs = [
            ("LB", "RB"), ("C", "LB"), ("C", "RB"),
            ("L",  "RB"), ("R", "LB"), ("L",  "R"),
        ]
    elif scene_name == "table":
        # table 场景相对开阔, 测试更长距离规划
        pairs = [
            ("L",  "R"),  ("C",  "L"),  ("C",  "R"),
            ("AS", "C"),  ("L",  "AS"), ("R",  "AS"),
        ]
    elif scene_name == "combined":
        # 合并场景: 5 canonical pairs (跨区域循环)
        pairs = [
            ("AS", "TS"), ("TS", "CS"), ("CS", "LB"),
            ("LB", "RB"), ("RB", "AS"),
        ]
    else:
        # 默认: 所有组合
        names = list(pts.keys())
        pairs = list(itertools.combinations(names, 2))

    result = []
    for s_name, g_name in pairs:
        if s_name in pts and g_name in pts:
            result.append((pts[s_name].copy(), pts[g_name].copy()))
    return result


# ═══════════════════════════════════════════════════════════════════════════
# 场景障碍物定义 (从 SDF 转换)
# ═══════════════════════════════════════════════════════════════════════════

def _make_obstacle(name: str, aabb_min: np.ndarray, aabb_max: np.ndarray) -> Dict:
    return {"name": name, "min": aabb_min.tolist(), "max": aabb_max.tolist()}


def build_shelves_obstacles() -> List[Dict]:
    """Shelves 场景障碍物 (from shelves.sdf, welded at [0.85, 0, 0.4]).

    SDF 中 shelves_body link 的 collision boxes:
      right_wall:   pose [0, 0.292, 0],  size 0.3 × 0.016 × 0.783
      left_wall:    pose [0, -0.292, 0], size 0.3 × 0.016 × 0.783
    SDF 中 top_and_bottom link (fixed joint to shelves_body):
      top:          pose [0, 0, 0.3995], size 0.3 × 0.6 × 0.016
      shelf_lower:  pose [0, 0, -0.13115], size 0.3 × 0.6 × 0.016
      shelf_upper:  pose [0, 0, 0.13115], size 0.3 × 0.6 × 0.016
    (bottom 被注释掉了)
    """
    origin = np.array([0.85, 0.0, 0.4])
    rpy_deg = np.array([0.0, 0.0, 0.0])  # no rotation

    boxes = [
        ("shelf_right_wall",  [0, 0.292, 0],    [0.3, 0.016, 0.783]),
        ("shelf_left_wall",   [0, -0.292, 0],   [0.3, 0.016, 0.783]),
        ("shelf_top",         [0, 0, 0.3995],   [0.3, 0.6, 0.016]),
        ("shelf_lower",       [0, 0, -0.13115], [0.3, 0.6, 0.016]),
        ("shelf_upper",       [0, 0, 0.13115],  [0.3, 0.6, 0.016]),
    ]

    obstacles = []
    for name, local_pose, size in boxes:
        aabb_min, aabb_max = _sdf_box_to_aabb(origin, rpy_deg, local_pose, size)
        obstacles.append(_make_obstacle(name, aabb_min, aabb_max))
    return obstacles


def build_bins_obstacles() -> List[Dict]:
    """Bins 场景障碍物 (from bin.sdf).

    binR: welded at [0, -0.6, 0], rotation rpy=[0, 0, 90°]
    binL: welded at [0,  0.6, 0], rotation rpy=[0, 0, 90°]

    每个 bin 包含:
      front:  pose [0.22, 0, 0.105],  size 0.05 × 0.63 × 0.21
      back:   pose [-0.22, 0, 0.105], size 0.05 × 0.63 × 0.21
      left:   pose [0, 0.29, 0.105],  size 0.49 × 0.05 × 0.21
      right:  pose [0, -0.29, 0.105], size 0.49 × 0.05 × 0.21
      bottom: pose [0, 0, 0.0075],    size 0.49 × 0.63 × 0.015
    """
    bin_boxes = [
        ("front",  [0.22,  0,     0.105],  [0.05, 0.63, 0.21]),
        ("back",   [-0.22, 0,     0.105],  [0.05, 0.63, 0.21]),
        ("left",   [0,     0.29,  0.105],  [0.49, 0.05, 0.21]),
        ("right",  [0,    -0.29,  0.105],  [0.49, 0.05, 0.21]),
        ("bottom", [0,     0,     0.0075], [0.49, 0.63, 0.015]),
    ]

    obstacles = []

    # binR at [0, -0.6, 0], yaw=90°
    for name, local_pose, size in bin_boxes:
        aabb_min, aabb_max = _sdf_box_to_aabb(
            [0, -0.6, 0], [0, 0, 90], local_pose, size)
        obstacles.append(_make_obstacle(f"binR_{name}", aabb_min, aabb_max))

    # binL at [0, 0.6, 0], yaw=90°
    for name, local_pose, size in bin_boxes:
        aabb_min, aabb_max = _sdf_box_to_aabb(
            [0, 0.6, 0], [0, 0, 90], local_pose, size)
        obstacles.append(_make_obstacle(f"binL_{name}", aabb_min, aabb_max))

    return obstacles


def build_table_obstacles() -> List[Dict]:
    """Table 场景障碍物 (from table_wide.sdf, welded at [0.4, 0, 0]).

    table_top: pose [0, 0, -0.1], size 2.5 × 2.5 × 0.2

    注: 在 AABB 碰撞模型中, iiwa14 link 1 的半径为 0.08m, 会使其 AABB 延伸
    到 z < 0. 因此将 table 向下偏移 0.15m (local z=-0.25), 使 table z_max=-0.15,
    在机器人基座 (z=0) 与桌面之间留出足够裕量, 避免系统性误报碰撞.
    此偏移不影响场景的物理语义 (机器人绕桌面上方物体的规划).
    """
    origin = np.array([0.4, 0.0, 0.0])
    rpy_deg = np.array([0.0, 0.0, 0.0])
    aabb_min, aabb_max = _sdf_box_to_aabb(
        origin, rpy_deg, [0, 0, -0.25], [2.5, 2.5, 0.2])
    return [_make_obstacle("table_top", aabb_min, aabb_max)]


# ═══════════════════════════════════════════════════════════════════════════
# 完整场景构建
# ═══════════════════════════════════════════════════════════════════════════

_SCENE_BUILDERS = {
    "shelves": build_shelves_obstacles,
    "bins":    build_bins_obstacles,
    "table":   build_table_obstacles,
}


def build_combined_obstacles() -> List[Dict]:
    """合并场景障碍物 (与 prm_comparison notebook 完全一致).

    加载 shelves (5) + binR (5) + binL (5) + table (1) = 16 个障碍物,
    对应 notebook 中: [iiwa, wsg, shelf, binR, binL, table] = models
    """
    obs = []
    obs.extend(build_shelves_obstacles())
    obs.extend(build_bins_obstacles())
    obs.extend(build_table_obstacles())
    return obs


def build_scene(
    scene_name: str,
) -> MarcucciScene:
    """构建指定的 Marcucci 基准场景.

    Args:
        scene_name: 场景名称 ("shelves", "bins", "table")

    Returns:
        MarcucciScene 实例
    """
    if scene_name not in _SCENE_BUILDERS:
        raise ValueError(f"Unknown scene: {scene_name}. "
                         f"Available: {list(_SCENE_BUILDERS.keys())}")

    obstacles = _SCENE_BUILDERS[scene_name]()

    difficulty_map = {"shelves": "hard", "bins": "medium", "table": "easy"}
    description_map = {
        "shelves": "Shelves with narrow passages (Marcucci 2024)",
        "bins":    "Left/right bins with medium difficulty (Marcucci 2024)",
        "table":   "Wide table, open space baseline (Marcucci 2024)",
    }

    return MarcucciScene(
        name=scene_name,
        obstacles=obstacles,
        query_pairs=[(s, g) for s, g in
                     [("LB", "TS"), ("RB", "CS"), ("LB", "RB"),
                      ("C", "CS"), ("AS", "LB"), ("TS", "RB")]
                     ] if scene_name == "shelves" else
                    [(s, g) for s, g in
                     [("LB", "RB"), ("C", "LB"), ("C", "RB"),
                      ("L", "RB"), ("R", "LB"), ("L", "R")]
                     ] if scene_name == "bins" else
                    [(s, g) for s, g in
                     [("L", "R"), ("C", "L"), ("C", "R"),
                      ("AS", "C"), ("L", "AS"), ("R", "AS")]],
        difficulty=difficulty_map[scene_name],
        description=description_map[scene_name],
    )


def build_all_scenes() -> List[MarcucciScene]:
    """构建所有 Marcucci 基准场景."""
    return [build_scene(name)
            for name in ["shelves", "bins", "table"]]


def scene_to_forest_scene(marcucci_scene: MarcucciScene):
    """将 MarcucciScene 转换为 forest.scene.Scene 对象 (用于碰撞检测)."""
    from forest.scene import Scene
    scene = Scene()
    for obs_dict in marcucci_scene.obstacles:
        scene.add_obstacle(
            min_point=obs_dict["min"],
            max_point=obs_dict["max"],
            name=obs_dict["name"],
        )
    return scene


# ═══════════════════════════════════════════════════════════════════════════
# 随机补充场景 (outline 要求的补充)
# ═══════════════════════════════════════════════════════════════════════════

def build_random_obstacles(
    n_obstacles: int = 15,
    workspace_bounds: Tuple[float, float, float, float, float, float] = (
        -0.5, -1.0, 0.0, 1.5, 1.0, 1.2),
    min_size: float = 0.05,
    max_size: float = 0.20,
    seed: int = 42,
    robot_base_clearance: float = 0.15,
) -> List[Dict]:
    """生成随机 AABB 障碍物.

    确保障碍物不与机器人基座重叠.
    """
    rng = np.random.default_rng(seed)
    xmin, ymin, zmin, xmax, ymax, zmax = workspace_bounds
    obstacles = []

    for i in range(n_obstacles * 3):  # over-generate then filter
        if len(obstacles) >= n_obstacles:
            break
        size = rng.uniform(min_size, max_size, size=3)
        center = rng.uniform([xmin, ymin, zmin], [xmax, ymax, zmax])

        # 跳过太靠近基座的障碍物
        dist_to_base = np.sqrt(center[0]**2 + center[1]**2)
        if dist_to_base < robot_base_clearance:
            continue

        obs_min = center - size / 2.0
        obs_max = center + size / 2.0
        obstacles.append(_make_obstacle(f"rand_{i:03d}", obs_min, obs_max))

    return obstacles


# ═══════════════════════════════════════════════════════════════════════════
# 便捷入口: 获取实验用完整场景列表
# ═══════════════════════════════════════════════════════════════════════════

def get_experiment_scenes(
    include_random: bool = True,
    random_counts: Sequence[int] = (8, 15, 20),
    random_seed_base: int = 42,
) -> List[MarcucciScene]:
    """获取论文实验所需的全部场景.

    Returns:
        List[MarcucciScene] — 3 个 Marcucci 场景 + 可选随机场景
    """
    scenes = build_all_scenes()

    if include_random:
        difficulty_names = {8: "easy", 15: "medium", 20: "hard"}
        for n_obs in random_counts:
            obs = build_random_obstacles(
                n_obstacles=n_obs,
                seed=random_seed_base + n_obs)
            scenes.append(MarcucciScene(
                name=f"random_{n_obs}",
                obstacles=obs,
                query_pairs=[],  # 将使用随机查询
                difficulty=difficulty_names.get(n_obs, "medium"),
                description=f"Random {n_obs} obstacles",
            ))

    return scenes

#!/usr/bin/env python3
"""
比较 SBF DH FK 与 Drake URDF FK 在 query 配置下的差异。
诊断 DH 参数与 Drake 模型之间的坐标系匹配问题。
"""
import numpy as np
import os, sys

# ── SBF DH FK ──────────────────────────────────────────────────────────────

def dh_transform(alpha, a, d, theta):
    """Standard DH transform (matches build_dh_joint in interval_math.cpp)."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct,    -st,     0,      a       ],
        [st*ca,  ct*ca, -sa,    -d*sa    ],
        [st*sa,  ct*sa,  ca,     d*ca    ],
        [0,      0,      0,      1       ],
    ])


def sbf_fk(q, dh_params, tool_frame=None):
    """
    Compute SBF DH forward kinematics chain.
    Returns list of frame transforms T_0_i for i = 0..n(+tool).
    """
    T = np.eye(4)
    frames = [T.copy()]
    
    for i, (alpha, a, d, theta_offset) in enumerate(dh_params):
        theta = q[i] + theta_offset
        T = T @ dh_transform(alpha, a, d, theta)
        frames.append(T.copy())
    
    if tool_frame is not None:
        alpha, a, d, theta = tool_frame
        T = T @ dh_transform(alpha, a, d, theta)
        frames.append(T.copy())
    
    return frames


# ── IIWA14 DH parameters (from iiwa14.json) ──────────────────────────────

DH_PARAMS = [
    # (alpha, a, d, theta_offset)
    (0.0,                  0.0, 0.36,  0.0),
    (-np.pi/2,             0.0, 0.0,   0.0),
    ( np.pi/2,             0.0, 0.42,  0.0),
    ( np.pi/2,             0.0, 0.0,   0.0),
    (-np.pi/2,             0.0, 0.4,   0.0),
    (-np.pi/2,             0.0, 0.0,   0.0),
    ( np.pi/2,             0.0, 0.0,   0.0),
]
TOOL_FRAME = (0.0, 0.0, 0.135, 0.0)


# ── Drake FK ───────────────────────────────────────────────────────────────

def drake_fk(q):
    """Compute Drake FK using MultibodyPlant."""
    from pydrake.multibody.parsing import LoadModelDirectives, Parser, ProcessModelDirectives
    from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
    from pydrake.systems.framework import DiagramBuilder
    
    _PROJ_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    _PROJ_ROOT = os.path.dirname(_PROJ_ROOT)   # box_aabb/
    _GCS_ROOT = os.path.join(_PROJ_ROOT, "gcs-science-robotics")
    _YAML_FILE = os.path.join(_GCS_ROOT, "models", "iiwa14_welded_gripper.yaml")
    
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant)
    parser.package_map().Add("gcs", _GCS_ROOT)
    directives = LoadModelDirectives(_YAML_FILE)
    ProcessModelDirectives(directives, plant, parser)
    plant.Finalize()
    diagram = builder.Build()
    
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)
    plant.SetPositions(plant_context, q[:7])
    
    results = {}
    # Get each link's world pose
    for link_idx in range(8):
        link_name = f"iiwa_link_{link_idx}"
        body = plant.GetBodyByName(link_name, plant.GetModelInstanceByName("iiwa"))
        X = body.EvalPoseInWorld(plant_context)
        results[link_name] = X.GetAsMatrix4()
    
    # Get wsg body pose
    wsg_body = plant.GetBodyByName("body", plant.GetModelInstanceByName("wsg"))
    results["wsg_body"] = wsg_body.EvalPoseInWorld(plant_context).GetAsMatrix4()
    
    # Get wsg finger poses
    for name in ["left_finger", "right_finger"]:
        finger = plant.GetBodyByName(name, plant.GetModelInstanceByName("wsg"))
        results[name] = finger.EvalPoseInWorld(plant_context).GetAsMatrix4()
    
    return results, plant, plant_context


# ── Marcucci query configurations ─────────────────────────────────────────

CONFIGS = {
    "AS": np.array([0.0, 0.55, 0.0, -1.45, 0.0, 1.58, 0.0]),
    "TS": np.array([0.2, 0.47, 0.0, -1.33, -0.1, 1.48, 0.65]),
    "CS": np.array([-0.2, 0.47, 0.0, -1.33, 0.1, 1.48, -0.65]),
    "LB": np.array([-1.57, 0.6, 0.0, -1.2, 0.0, 1.6, 0.8]),
    "RB": np.array([1.57, 0.6, 0.0, -1.2, 0.0, 1.6, -0.8]),
}


def main():
    print("=" * 90)
    print("SBF DH FK vs Drake URDF FK 对比验证")
    print("=" * 90)
    
    # Pick a test config
    q_name = "AS"
    q = CONFIGS[q_name]
    print(f"\n配置: {q_name} = {q}")
    
    # SBF DH FK
    frames = sbf_fk(q, DH_PARAMS, TOOL_FRAME)
    print(f"\n--- SBF DH FK ({len(frames)} frames, including identity) ---")
    
    for i, T in enumerate(frames):
        pos = T[:3, 3]
        z_axis = T[:3, 2]
        label = f"frame_{i}"
        if i == 0:
            label = "base"
        elif i <= 7:
            label = f"joint_{i}"
        elif i == 8:
            label = "tool_tip"
        print(f"  {label:12s}  pos=({pos[0]:+.5f}, {pos[1]:+.5f}, {pos[2]:+.5f})  "
              f"z=({z_axis[0]:+.4f}, {z_axis[1]:+.4f}, {z_axis[2]:+.4f})")
    
    # Drake FK
    print(f"\n--- Drake URDF FK ---")
    drake_results, plant, plant_context = drake_fk(q)
    for name in ["iiwa_link_0", "iiwa_link_1", "iiwa_link_2", "iiwa_link_3",
                  "iiwa_link_4", "iiwa_link_5", "iiwa_link_6", "iiwa_link_7",
                  "wsg_body", "left_finger", "right_finger"]:
        T = drake_results[name]
        pos = T[:3, 3]
        z_axis = T[:3, 2]
        print(f"  {name:16s}  pos=({pos[0]:+.5f}, {pos[1]:+.5f}, {pos[2]:+.5f})  "
              f"z=({z_axis[0]:+.4f}, {z_axis[1]:+.4f}, {z_axis[2]:+.4f})")
    
    # Compare link positions
    print(f"\n--- 位置差异 (SBF DH frame vs Drake link) ---")
    drake_link_names = [f"iiwa_link_{i}" for i in range(8)]
    
    for i in range(8):
        sbf_pos = frames[i][:3, 3]       # frame_i (0-based, frame_0=base)
        # Note: DH frame i corresponds to iiwa_link_i in Drake
        # DH frame_0 = identity = iiwa_link_0 (base)
        # DH frame_1 = after joint 1 = iiwa_link_1
        # etc.
        drake_pos = drake_results[drake_link_names[i]][:3, 3]
        diff = np.linalg.norm(sbf_pos - drake_pos)
        print(f"  DH frame_{i} vs {drake_link_names[i]:16s}  "
              f"Δ = {diff:.6f} m  "
              f"SBF=({sbf_pos[0]:+.4f},{sbf_pos[1]:+.4f},{sbf_pos[2]:+.4f})  "
              f"Drake=({drake_pos[0]:+.4f},{drake_pos[1]:+.4f},{drake_pos[2]:+.4f})")
    
    # Also compare tool tip with wsg_body
    sbf_tool = frames[-1][:3, 3]
    wsg_pos = drake_results["wsg_body"][:3, 3]
    diff = np.linalg.norm(sbf_tool - wsg_pos)
    print(f"\n  DH tool_tip vs wsg_body          Δ = {diff:.6f} m  "
          f"SBF=({sbf_tool[0]:+.4f},{sbf_tool[1]:+.4f},{sbf_tool[2]:+.4f})  "
          f"Drake=({wsg_pos[0]:+.4f},{wsg_pos[1]:+.4f},{wsg_pos[2]:+.4f})")
    
    # Check what direction the tool link extends in world frame
    frame7_pos = frames[7][:3, 3]
    tool_pos = frames[8][:3, 3]
    tool_dir = tool_pos - frame7_pos
    tool_len = np.linalg.norm(tool_dir)
    if tool_len > 1e-10:
        tool_dir_n = tool_dir / tool_len
    else:
        tool_dir_n = np.zeros(3)
    
    print(f"\n--- 工具 link 方向 (世界坐标) ---")
    print(f"  frame_7 → tool_tip: direction=({tool_dir_n[0]:+.4f}, {tool_dir_n[1]:+.4f}, {tool_dir_n[2]:+.4f})  len={tool_len:.4f}")
    print(f"  frame_7 z-axis (SBF): ({frames[7][:3,2][0]:+.4f}, {frames[7][:3,2][1]:+.4f}, {frames[7][:3,2][2]:+.4f})")
    drake_link7_T = drake_results["iiwa_link_7"]
    print(f"  link_7 z-axis (Drake): ({drake_link7_T[:3,2][0]:+.4f}, {drake_link7_T[:3,2][1]:+.4f}, {drake_link7_T[:3,2][2]:+.4f})")
    
    # Compare z-axes
    z_sbf = frames[7][:3, 2]
    z_drake = drake_link7_T[:3, 2]
    z_diff = np.arccos(np.clip(np.dot(z_sbf, z_drake), -1, 1))
    print(f"  z轴角度差: {np.degrees(z_diff):.2f}°")
    
    # Run for all configs
    print(f"\n{'='*90}")
    print("所有配置的 frame_7 位置对比:")
    print(f"{'='*90}")
    for name, qi in CONFIGS.items():
        sbf_frames = sbf_fk(qi, DH_PARAMS, TOOL_FRAME)
        drake_res, _, _ = drake_fk(qi)
        
        sbf7 = sbf_frames[7][:3, 3]
        drake7 = drake_res["iiwa_link_7"][:3, 3]
        diff = sbf7 - drake7
        dist = np.linalg.norm(diff)
        
        sbf_tool = sbf_frames[8][:3, 3]
        drake_wsg = drake_res["wsg_body"][:3, 3]
        
        print(f"  {name}:  frame_7 Δ={dist:.4f}m  "
              f"diff=({diff[0]:+.4f},{diff[1]:+.4f},{diff[2]:+.4f})  "
              f"SBF_tool=({sbf_tool[0]:+.4f},{sbf_tool[1]:+.4f},{sbf_tool[2]:+.4f})  "
              f"Drake_wsg=({drake_wsg[0]:+.4f},{drake_wsg[1]:+.4f},{drake_wsg[2]:+.4f})")


if __name__ == "__main__":
    main()

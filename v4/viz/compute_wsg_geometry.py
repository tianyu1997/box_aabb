#!/usr/bin/env python3
"""计算 WSG 所有碰撞球在 iiwa_link_7 坐标系下的位置和包围盒."""
import numpy as np

# wsg_attach frame: translation [0, 0, 0.114] from iiwa_link_7, rotation Rpy [90deg, 0, 0]
# Rx(90deg): rotates Y→Z, Z→-Y
Rx90 = np.array([[1,0,0], [0,0,-1], [0,1,0]], dtype=float)

# Body link pose in model frame: [0, -0.049133, 0]
body_offset = np.array([0, -0.049133, 0])

# Left finger: pose [-0.06, 0.028, 0], rotation Ry(pi)
lf_offset = np.array([-0.06, 0.028, 0])
Ry_pi = np.array([[-1,0,0],[0,1,0],[0,0,-1]], dtype=float)

# Right finger: pose [0.06, 0.028, 0], no rotation
rf_offset = np.array([0.06, 0.028, 0])

body_spheres = [
    ('body_s1', [0.02, 0, 0], 0.05),
    ('body_s2', [-0.02, 0, 0], 0.05),
    ('body_s3', [-0.045, 0.005, 0], 0.05),
    ('body_s4', [0.045, 0.005, 0], 0.05),
]

finger_spheres = [
    ('finger_s1', [0.005, -0.02, 0], 0.018),
    ('finger_s2', [0.005, -0.01, 0], 0.017),
    ('finger_s3', [0.002, 0.005, 0], 0.014),
    ('finger_s4', [0.00, 0.02, 0], 0.01),
    ('finger_s5', [0.00, 0.032, 0], 0.011),
]

all_pts = []  # (model_pos, radius, name)

for name, spos, r in body_spheres:
    model_pos = body_offset + np.array(spos)
    all_pts.append((model_pos, r, name))

for name, spos, r in finger_spheres:
    local = Ry_pi @ np.array(spos)
    model_pos = lf_offset + local
    all_pts.append((model_pos, r, "L_" + name))

for name, spos, r in finger_spheres:
    model_pos = rf_offset + np.array(spos)
    all_pts.append((model_pos, r, "R_" + name))

# Transform to link7 frame: p_link7 = Rx90 @ p_model + [0, 0, 0.114]
attach_t = np.array([0, 0, 0.114])
tool_tip = attach_t.copy()

print("=== All spheres in iiwa_link_7 frame ===")
print(f"{'Name':20s}  {'x':>8s} {'y':>8s} {'z':>8s}   r")
print("-" * 60)
centers_l7 = []
radii_arr = []
for p_model, r, name in all_pts:
    p7 = Rx90 @ p_model + attach_t
    centers_l7.append(p7)
    radii_arr.append(r)
    print(f"{name:20s}  {p7[0]:8.4f} {p7[1]:8.4f} {p7[2]:8.4f}   {r}")

centers_l7 = np.array(centers_l7)
radii_arr = np.array(radii_arr)

print("\n=== Bounding box in link7 frame (center +/- radius) ===")
lo7 = np.min(centers_l7 - radii_arr[:, None], axis=0)
hi7 = np.max(centers_l7 + radii_arr[:, None], axis=0)
print(f"lo = [{lo7[0]:.4f}, {lo7[1]:.4f}, {lo7[2]:.4f}]")
print(f"hi = [{hi7[0]:.4f}, {hi7[1]:.4f}, {hi7[2]:.4f}]")
print(f"extent = [{hi7[0]-lo7[0]:.4f}, {hi7[1]-lo7[1]:.4f}, {hi7[2]-lo7[2]:.4f}]")


def dist_to_segment(p, a, b):
    ab = b - a
    ap = p - a
    t = np.dot(ap, ab) / np.dot(ab, ab)
    t = np.clip(t, 0, 1)
    return np.linalg.norm(p - (a + t * ab))

print("\n=== Distance analysis from tool segment (link7_origin → tool_tip) ===")
print("The tool segment is from [0,0,0] to [0,0,0.114] in link7 frame")
print(f"Current tool_frame link_radii[7] = 0.03")

max_perp = 0
for p7, r, name in zip(centers_l7, radii_arr, [n for _,_,n in all_pts]):
    d = dist_to_segment(p7, np.zeros(3), tool_tip)
    total = d + r
    if total > max_perp:
        max_perp = total
    print(f"  {name:20s}  d_seg={d:.4f}  r={r}  total={total:.4f}")

print(f"\nMax perpendicular distance (center + radius) from tool segment: {max_perp:.4f}")
print(f"Current link_radii[7] = 0.03 → gap = {max_perp - 0.03:.4f}")

print("\n=== Axial extent beyond tool segment endpoints ===")
for p7, r, name in zip(centers_l7, radii_arr, [n for _,_,n in all_pts]):
    z = p7[2]
    if z + r > 0.114:
        print(f"  BEYOND tool_tip: {name:20s} z={z:.4f} r={r} → z_max={z+r:.4f}")
    if z - r < 0:
        print(f"  BEFORE link7_origin: {name:20s} z={z:.4f} r={r} → z_min={z-r:.4f}")

# Compute what we'd need for "multi-segment" WSG abstraction
print("\n=== Potential multi-segment abstraction ===")
# Strategy: add 2-3 extra fixed segments extending from tool_tip
# covering the WSG body and fingers with appropriate radii

# The WSG has 3 main components in link7 frame:
# 1. Body: centered around tool_tip + Rx90 @ body_offset
body_center_l7 = Rx90 @ body_offset + attach_t
print(f"Body center in link7: [{body_center_l7[0]:.4f}, {body_center_l7[1]:.4f}, {body_center_l7[2]:.4f}]")

# 2. Left finger base in link7
lf_base_l7 = Rx90 @ lf_offset + attach_t
print(f"Left finger base in link7: [{lf_base_l7[0]:.4f}, {lf_base_l7[1]:.4f}, {lf_base_l7[2]:.4f}]")

# 3. Right finger base in link7
rf_base_l7 = Rx90 @ rf_offset + attach_t
print(f"Right finger base in link7: [{rf_base_l7[0]:.4f}, {rf_base_l7[1]:.4f}, {rf_base_l7[2]:.4f}]")

# Single bounding sphere approach
all_center = np.mean(centers_l7, axis=0)
all_enclosing_r = np.max(np.linalg.norm(centers_l7 - all_center, axis=1) + radii_arr)
print(f"\nSingle bounding sphere: center={all_center}, r={all_enclosing_r:.4f}")

# Key insight: in the SBF framework, we need AABB of the WSG at each configuration.
# Since WSG is rigidly attached to link7, its AABB in world frame = 
# rotation of the link7-frame bounding box by the link7 orientation.
# For interval FK: prefix[7] * T_attach * {each sphere offset}
# All WSG offsets are FIXED, so in interval FK they just propagate the existing interval uncertainty.

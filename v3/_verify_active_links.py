"""Verify active link optimization works correctly."""
import sys; sys.path.insert(0, 'src')
import numpy as np
from aabb.robot import load_robot
from forest.hier_aabb_tree import HierAABBTree
from forest.scene import Scene

robot = load_robot('panda')
tree = HierAABBTree(robot)

print(f'n_joints         = {robot.n_joints}')
print(f'has_tool         = {robot.tool_frame is not None}')
print(f'zero_length      = {robot.zero_length_links}')
print(f'n_links_total    = {tree._n_links_total}')
print(f'n_links (active) = {tree._n_links}')
print(f'active_indices   = {tree._active_link_indices.tolist()}')

# Verify AABB shape
tree._ensure_aabb_at(0)
aabb = tree._store.get_aabb(0)
print(f'AABB shape       = {aabb.shape}   (expected ({tree._n_links}, 6))')
assert aabb.shape == (tree._n_links, 6), f"AABB shape mismatch!"

# Verify collision packing
scene = Scene()
scene.add_obstacle([0.4, -0.1, 0.4], [0.6, 0.1, 0.6], name="test_box")
obs = scene.get_obstacles()
packed = tree._prepack_obstacles_c(obs)
n_active = tree._n_links
print(f'packed obs       = {len(packed)} entries  ({n_active} active links x {len(obs)} obs)')
assert len(packed) == n_active * len(obs)
for ci, entry in enumerate(packed):
    assert entry[0] == ci, f"Expected compact_idx={ci}, got {entry[0]}"
print(f'packed indices   = {[e[0] for e in packed]}  (all compact)')

# Verify find_free_box
result = tree.find_free_box(np.zeros(7), obs)
status = "OK" if result else "None"
print(f'find_free_box    = {status}')

# Verify storage savings
from forest._hier_layout import compute_stride
stride_old = compute_stride(tree._n_links_total)
stride_new = compute_stride(tree._n_links)
print(f'\nStorage savings:')
print(f'  stride (old, all {tree._n_links_total} links) = {stride_old} B')
print(f'  stride (new, {tree._n_links} active) = {stride_new} B')
print(f'  savings per node = {stride_old - stride_new} B ({100*(stride_old-stride_new)/stride_old:.0f}%)')

print('\nAll checks passed!')

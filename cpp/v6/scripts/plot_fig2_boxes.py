#!/usr/bin/env python3
"""Fig 2: C-space box forest visualization (conceptual 2-DOF).

Generates a 2D illustration of boxes covering free C-space,
with collision regions shown in gray, boxes colored by tree ID,
and an example A* path overlaid.
"""
import sys, os
sys.path.insert(0, os.path.dirname(__file__))
from plot_common import *

setup_ieee_style()

np.random.seed(42)

# ── Define a 2-DOF C-space [-π, π] × [-π, π] ──
LO, HI = -np.pi, np.pi

# Define circular "collision" obstacles in C-space
obstacles = [
    (0.0,  0.0, 1.2),   # center_x, center_y, radius
    (-1.8, 1.5, 0.8),
    (1.5, -1.0, 0.9),
    (-0.5, -2.0, 0.7),
    (2.2,  2.0, 0.6),
]

def in_collision(x, y):
    for ox, oy, r in obstacles:
        if (x - ox)**2 + (y - oy)**2 < r**2:
            return True
    return False

def box_collides(lo_x, lo_y, hi_x, hi_y, n=8):
    """Check if a box overlaps with obstacles (sampling-based check)."""
    for ox, oy, r in obstacles:
        # Check if obstacle circle intersects box
        cx = max(lo_x, min(ox, hi_x))
        cy = max(lo_y, min(oy, hi_y))
        if (cx - ox)**2 + (cy - oy)**2 < r**2:
            return True
    return False

# ── Generate boxes via recursive splitting ──
def split_box(lo_x, lo_y, hi_x, hi_y, depth=0, max_depth=5, min_size=0.25):
    """Recursively split boxes, keeping only collision-free ones."""
    if box_collides(lo_x, lo_y, hi_x, hi_y):
        if depth >= max_depth or (hi_x - lo_x) < min_size:
            return []  # Too small, discard
        # Split along wider dimension
        if (hi_x - lo_x) >= (hi_y - lo_y):
            mid = (lo_x + hi_x) / 2
            left = split_box(lo_x, lo_y, mid, hi_y, depth+1, max_depth, min_size)
            right = split_box(mid, lo_y, hi_x, hi_y, depth+1, max_depth, min_size)
            return left + right
        else:
            mid = (lo_y + hi_y) / 2
            bottom = split_box(lo_x, lo_y, hi_x, mid, depth+1, max_depth, min_size)
            top = split_box(lo_x, mid, hi_x, hi_y, depth+1, max_depth, min_size)
            return bottom + top
    else:
        return [(lo_x, lo_y, hi_x, hi_y, depth)]

# Generate initial grid, then recursively split
boxes = []
grid_n = 4
step = (HI - LO) / grid_n
for i in range(grid_n):
    for j in range(grid_n):
        lo_x = LO + i * step
        lo_y = LO + j * step
        hi_x = lo_x + step
        hi_y = lo_y + step
        boxes.extend(split_box(lo_x, lo_y, hi_x, hi_y, max_depth=5, min_size=0.2))

# Assign tree IDs (simple: based on spatial region)
tree_colors = []
for bx in boxes:
    cx = (bx[0] + bx[2]) / 2
    cy = (bx[1] + bx[3]) / 2
    if cx < 0 and cy > 0:
        tree_colors.append(0)
    elif cx > 0 and cy > 0:
        tree_colors.append(1)
    elif cx < 0 and cy < 0:
        tree_colors.append(2)
    else:
        tree_colors.append(3)

# ── Find a path through boxes (simple BFS) ──
def box_center(b):
    return ((b[0]+b[2])/2, (b[1]+b[3])/2)

def boxes_adjacent(b1, b2, eps=0.01):
    # Boxes are adjacent if they share a face (touch along one dimension)
    overlap_x = max(0, min(b1[2], b2[2]) - max(b1[0], b2[0]))
    overlap_y = max(0, min(b1[3], b2[3]) - max(b1[1], b2[1]))
    touch_x = abs(b1[2] - b2[0]) < eps or abs(b2[2] - b1[0]) < eps
    touch_y = abs(b1[3] - b2[1]) < eps or abs(b2[3] - b1[1]) < eps
    if touch_x and overlap_y > eps:
        return True
    if touch_y and overlap_x > eps:
        return True
    return False

# Build adjacency and BFS for a start→goal path
start_pt = (-2.5, -2.5)
goal_pt = (2.5, 2.5)

def find_box(pt):
    for i, b in enumerate(boxes):
        if b[0] <= pt[0] <= b[2] and b[1] <= pt[1] <= b[3]:
            return i
    # Find closest
    dists = [((box_center(b)[0]-pt[0])**2 + (box_center(b)[1]-pt[1])**2) for b in boxes]
    return int(np.argmin(dists))

si = find_box(start_pt)
gi = find_box(goal_pt)

# BFS
from collections import deque
adj = {i: [] for i in range(len(boxes))}
for i in range(len(boxes)):
    for j in range(i+1, len(boxes)):
        if boxes_adjacent(boxes[i], boxes[j]):
            adj[i].append(j)
            adj[j].append(i)

visited = {si: None}
queue = deque([si])
while queue:
    node = queue.popleft()
    if node == gi:
        break
    for nb in adj[node]:
        if nb not in visited:
            visited[nb] = node
            queue.append(nb)

path_indices = []
if gi in visited:
    node = gi
    while node is not None:
        path_indices.append(node)
        node = visited[node]
    path_indices.reverse()

# ── Plot ──
fig, ax = plt.subplots(figsize=(SINGLE_COL, 3.2))

# Draw collision regions
theta = np.linspace(0, 2*np.pi, 100)
for ox, oy, r in obstacles:
    ax.fill(ox + r*np.cos(theta), oy + r*np.sin(theta),
            color='gray', alpha=0.35, zorder=1)
    ax.plot(ox + r*np.cos(theta), oy + r*np.sin(theta),
            color='gray', linewidth=0.5, zorder=2)

# Draw boxes
tree_pals = [PAL[0], PAL[1], PAL[2], PAL[4]]
for i, (b, tc) in enumerate(zip(boxes, tree_colors)):
    w = b[2] - b[0]
    h = b[3] - b[1]
    alpha = 0.3 if i not in path_indices else 0.6
    lw = 0.3 if i not in path_indices else 1.0
    rect = plt.Rectangle((b[0], b[1]), w, h,
                          facecolor=tree_pals[tc], alpha=alpha,
                          edgecolor='black', linewidth=lw, zorder=3)
    ax.add_patch(rect)

# Draw path
if path_indices:
    path_pts = [box_center(boxes[i]) for i in path_indices]
    px = [p[0] for p in path_pts]
    py = [p[1] for p in path_pts]
    ax.plot(px, py, 'r-', linewidth=1.5, zorder=5, label='A* path')
    ax.plot(px[0], py[0], 'g^', markersize=8, zorder=6, label='Start')
    ax.plot(px[-1], py[-1], 'rv', markersize=8, zorder=6, label='Goal')

ax.set_xlim(LO, HI)
ax.set_ylim(LO, HI)
ax.set_xlabel(r'$\theta_1$ (rad)')
ax.set_ylabel(r'$\theta_2$ (rad)')
ax.set_aspect('equal')
ax.legend(loc='upper left', fontsize=6, framealpha=0.9)
ax.set_title(f'2-DOF C-space Box Forest ({len(boxes)} boxes, 4 trees)',
             fontsize=8, fontweight='bold')

fig.tight_layout()
savefig(fig, 'fig2_cspace_boxes')
print(f'Done: Fig 2 ({len(boxes)} boxes, path length {len(path_indices)} boxes)')

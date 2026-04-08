# SafeBoxForest v2 — User Guide

## Architecture Overview

SafeBoxForest (SBF) is a motion planning library that decomposes C-space into
**axis-aligned safe boxes** — collision-free hyperrectangles that cover the
free configuration space. Planning reduces to searching a graph of adjacent boxes.

```
┌─────────┐    ┌─────────┐    ┌──────────┐    ┌──────────┐    ┌─────────┐
│  Robot   │───▶│  Scene  │───▶│ Envelope │───▶│  Forest  │───▶│ Planner │
│  Model   │    │ Checker │    │ Computer │    │  Grower  │    │ Search  │
└─────────┘    └─────────┘    └──────────┘    └──────────┘    └─────────┘
     DH            AABB           Interval         FFB           Dijkstra
   params        collision          FK           bisection       shortcut
```

### Module Map

| Module | Purpose | Key Classes |
|--------|---------|-------------|
| **common** | Types & configuration | `Interval`, `BoxNode`, `SBFConfig` |
| **robot** | Kinematics | `Robot`, `DHParam`, FK functions |
| **scene** | Collision checking | `AabbCollisionChecker`, `Scene` |
| **envelope** | AABB enclosures | `IntervalFKEnvelopeComputer` *(legacy)*, `FrameStore` + `collision_policy.h` *(new)* |
| **forest** | Box growth | `ForestGrower`, `SafeBoxForest`, `HierAABBTree` |
| **bridge** | Post-processing | `coarsen_forest()`, `find_islands()` |
| **planner** | Path search | `SBFPlanner`, `PathSmoother` |
| **viz** | Export | `export_forest_json()` |
| **io** | Persistence | `load_obstacles_json()`, `HCacheFile` |

---

## Core Concepts

### 1. Safe Boxes

A **safe box** (`BoxNode`) is an axis-aligned hyperrectangle in C-space
(joint space) that is guaranteed to be collision-free. Each box is defined
by a list of `Interval` values — one per joint.

```python
# A 7-DOF box: each joint has an interval
intervals = [
    pysbf2.Interval(-0.5, 0.3),   # joint 0
    pysbf2.Interval(-1.0, -0.2),  # joint 1
    # ... 5 more
]
box = pysbf2.BoxNode(id=0, intervals=intervals, seed_config=q)
```

### 2. Find-Free-Box (FFB)

The FFB algorithm takes a seed configuration and binary-searches the
`HierAABBTree` to find the largest collision-free box containing that seed.
At each level, it computes AABB envelopes via interval FK and checks overlap
with obstacles.

### 3. Forest Growth

`ForestGrower` orchestrates box creation:
1. **Anchor phase**: Create boxes around start and goal
2. **BFS expansion**: Grow adjacent boxes from existing box boundaries
3. **Random fill**: Sample random seeds and run FFB
4. **Phase relaxation**: Progressively relax `min_edge` to allow smaller boxes

### 4. Graph Search

After growing, the forest has an adjacency graph. `SBFPlanner`:
1. Attaches start/goal to the graph via `TreeConnector`
2. Runs Dijkstra to find a box sequence
3. Extracts waypoints at shared face centers
4. Smooths the path via `PathSmoother`

### 5. Incremental Updates

When obstacles change, SBF supports incremental updates:
- `add_obstacle()`: invalidate colliding boxes
- `remove_obstacle()`: (boxes may be regrown)
- `regrow()`: fill depleted regions

---

## Workflows

### Workflow A: One-Shot Planning

The simplest usage — plan once from start to goal:

```python
import numpy as np
import pysbf2

robot = pysbf2.Robot.from_json("configs/panda.json")
obstacles = pysbf2.io.load_obstacles_json("scene.json")

start = np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785])
goal  = np.array([2.0, 0.5, -1.0, -1.5, 0.5, 1.0, -0.5])

result = pysbf2.plan_once(robot, obstacles, start, goal)
print(f"Success: {result.success}, Cost: {result.cost:.3f}")
```

### Workflow B: Build-Once, Query-Many

For multiple queries in the same environment:

```python
config = pysbf2.make_panda_config(seed=42)
planner = pysbf2.SBFPlanner(robot, obstacles, config)

# Build forest for representative start/goal
planner.build(start, goal, timeout=60.0)
print(f"Forest: {planner.forest().n_boxes()} boxes")

# Query multiple times (fast - no rebuilding)
for goal_i in goals:
    result = planner.query(start, goal_i, timeout=5.0)
    if result.success:
        print(f"  Goal reached, cost={result.cost:.3f}")
```

### Workflow C: Incremental

Dynamic environments with changing obstacles:

```python
planner = pysbf2.SBFPlanner(robot, initial_obstacles, config)
planner.build(start, goal, timeout=30.0)
result = planner.query(start, goal)

# New obstacle appears
new_obs = pysbf2.Obstacle(
    np.array([0.4, 0.0, 0.6]),
    np.array([0.1, 0.1, 0.1]),
    "moving_box"
)
planner.add_obstacle(new_obs)

# Regrow depleted regions
planner.regrow(500, timeout=10.0)

# Re-plan
result2 = planner.query(start, goal)
```

### Workflow D: Multi-Pair Forest

Build a forest covering multiple start-goal pairs:

```python
pairs = [(start1, goal1), (start2, goal2), (start3, goal3)]
planner.build_multi(pairs, n_random_boxes=5000, timeout=120.0)

# All pairs can be queried
for s, g in pairs:
    result = planner.query(s, g)
```

### Workflow E: Low-Level Control

For full control over the pipeline:

```python
# 1. Setup
robot = pysbf2.Robot.from_json("configs/panda.json")
checker = pysbf2.AabbCollisionChecker(robot, obstacles)
config = pysbf2.SBFConfig()
config.max_boxes = 500

# 2. Tree & Forest
tree = pysbf2.HierAABBTree(robot, initial_cap=128)
forest = pysbf2.SafeBoxForest(robot.n_joints(), robot.joint_limits())

# 3. Grow
grower = pysbf2.ForestGrower(robot, checker, tree, forest, config)
grower.grow(start, goal)

# 4. Coarsen
cr = pysbf2.coarsen_greedy(forest, checker, target_boxes=200)
print(f"Coarsened: {cr.boxes_before} → {cr.boxes_after}")

# 5. Graph search
forest.rebuild_adjacency()
adj = forest.adjacency()
boxes = forest.boxes()
start_ids = {b.id for b in boxes.values() if b.contains(start)}
goal_ids  = {b.id for b in boxes.values() if b.contains(goal)}

dr = pysbf2.dijkstra_center_distance(adj, boxes, start_ids, goal_ids)
if dr.found:
    waypoints = pysbf2.extract_waypoints(dr.path, boxes, start, goal)

    # 6. Smooth
    smoother = pysbf2.PathSmoother(checker, 0.05)
    smoothed = smoother.shortcut(waypoints, max_iters=200)
    print(f"Path length: {pysbf2.PathSmoother.path_length(smoothed):.3f}")
```

---

## Tuning Guide

### Speed vs. Coverage

| To increase... | Set |
|---|---|
| More boxes | `max_boxes` ↑ |
| Faster growth | `max_consecutive_miss` ↓ |
| Better coverage | `guided_sample_ratio` ↓ (more random) |
| Smaller boxes | `ffb_min_edge` ↓ |
| Fewer FK calls | `ffb_max_depth` ↓ |

### Path Quality

| To improve... | Set |
|---|---|
| Smoother path | `shortcut_max_iters` ↑ |
| Finer segments | `segment_resolution` ↓ |
| Tighter boxes | `ffb_min_edge` ↓ |

### Memory & Cache

| For... | Use |
|---|---|
| Persist tree | `tree.save("cache.hcache")` |
| Fast reload | `HierAABBTree.load_mmap(path, robot)` |
| Auto-cache | `config.use_cache = True; config.cache_path = "..."` |

---

## Robot JSON Format

```json
{
  "name": "panda",
  "dh_params": [
    {"alpha": 0, "a": 0, "d": 0.333, "theta": 0, "joint_type": 0},
    {"alpha": -1.5708, "a": 0, "d": 0, "theta": 0, "joint_type": 0}
  ],
  "joint_limits": [
    {"lo": -2.8973, "hi": 2.8973},
    {"lo": -1.7628, "hi": 1.7628}
  ],
  "tool_frame": {"alpha": 0, "a": 0, "d": 0.107, "theta": -0.7854},
  "link_radii": [0.08, 0.08, 0.07, 0.07, 0.06, 0.06, 0.05, 0.04],
  "ee_spheres": [
    {"center": [0, 0, 0], "radius": 0.04}
  ],
  "ee_spheres_frame": 8
}
```

---

## Error Handling

All C++ exceptions are propagated to Python as `RuntimeError`.
Common issues:

| Error | Cause | Fix |
|---|---|---|
| "No containing box" | Start/goal outside limits | Clamp with `joint_limits.clamp(q)` |
| "Forest empty" | Growth failed | Increase `max_boxes`, relax `min_edge` |
| "Timeout" | Planning timed out | Increase `timeout` or reduce `max_boxes` |
| "File not found" | JSON/cache path wrong | Check path argument |

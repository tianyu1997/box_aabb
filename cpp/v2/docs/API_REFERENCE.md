# SafeBoxForest v2 — Python API Reference

> **Module:** `pysbf2`
> **Backend:** C++17 via pybind11
> **Dependencies:** NumPy, Eigen 3.3+

---

## Table of Contents

1. [Quick Start](#quick-start)
2. [Common Types](#1-common-types)
3. [Configuration](#2-configuration)
4. [Robot](#3-robot)
5. [Scene & Collision](#4-scene--collision)
6. [Envelope](#5-envelope)
7. [Forest](#6-forest)
8. [Bridge / Coarsening](#7-bridge--coarsening)
9. [Planner](#8-planner)
10. [Visualization](#9-visualization)
11. [I/O](#10-io)
12. [Feature Flags](#11-feature-flags)

---

## Quick Start

```python
import numpy as np
import pysbf2

# Load robot and create obstacles
robot = pysbf2.Robot.from_json("configs/panda.json")
obstacles = [
    pysbf2.Obstacle(np.array([0.5, 0.0, 0.5]),
                     np.array([0.05, 0.3, 0.05]), "shelf"),
]

# Plan
config = pysbf2.make_panda_config(seed=42)
result = pysbf2.plan_once(robot, obstacles, start, goal, config)

if result.success:
    print(f"Path: {result.n_waypoints()} waypoints, cost={result.cost:.3f}")
```

---

## 1. Common Types

### `Interval(lo=0.0, hi=0.0)`

A closed interval \[lo, hi\] on the real line.

| Property / Method | Returns | Description |
|---|---|---|
| `lo` | `float` | Lower bound (read/write) |
| `hi` | `float` | Upper bound (read/write) |
| `width()` | `float` | `hi - lo` |
| `mid()` | `float` | `(lo + hi) / 2` |
| `center()` | `float` | Alias for `mid()` |
| `contains(v, tol=1e-10)` | `bool` | `lo - tol <= v <= hi + tol` |
| `overlaps(other, tol=0.0)` | `bool` | Two intervals share a point |
| `hull(other)` | `Interval` | Smallest interval containing both |
| `intersect(other)` | `Interval` | Intersection |
| `+`, `-`, `*` | `Interval` | Interval arithmetic operators |

```python
iv = pysbf2.Interval(-1.0, 1.0)
print(iv.width())        # 2.0
print(iv.contains(0.5))  # True
print(iv * pysbf2.Interval(2.0, 3.0))  # [-3.0, 3.0]
```

---

### `Obstacle(center, half_sizes, name="")`

Axis-aligned box obstacle in 3D workspace.

| Property | Type | Description |
|---|---|---|
| `center` | `np.ndarray (3,)` | Center position |
| `half_sizes` | `np.ndarray (3,)` | Half extents |
| `name` | `str` | Display name |
| `lo()` | `np.ndarray (3,)` | Lower corner |
| `hi()` | `np.ndarray (3,)` | Upper corner |

---

### `JointLimits()`

Collection of per-joint `Interval` bounds.

| Property / Method | Returns | Description |
|---|---|---|
| `limits` | `list[Interval]` | Per-joint bounds |
| `n_dims()` | `int` | Number of joints |
| `contains(q, tol=1e-10)` | `bool` | All joints within limits |
| `clamp(q)` | `np.ndarray` | Clamp q to limits |

---

### `BoxNode(id, intervals, seed_config)`

Axis-aligned box in C-space.

| Property / Method | Returns | Description |
|---|---|---|
| `id` | `int` | Unique identifier |
| `joint_intervals` | `list[Interval]` | Per-joint ranges |
| `seed_config` | `np.ndarray` | Configuration that generated this box |
| `volume` | `float` | Hypervolume |
| `parent_id` | `int` | Parent box ID (-1 if root) |
| `tree_id` | `int` | Tree index (0=start, 1=goal) |
| `children_ids` | `list[int]` | Child box IDs |
| `n_dims()` | `int` | Number of joint dimensions |
| `center()` | `np.ndarray` | Center configuration |
| `contains(q, tol)` | `bool` | Does box contain q? |
| `distance_to_config(q)` | `float` | L2 distance to nearest point |
| `nearest_point_to(q)` | `np.ndarray` | Project q onto box |
| `overlaps_with(other, tol)` | `bool` | Axis-aligned overlap check |
| `is_adjacent_to(other, tol)` | `bool` | Face-touching check |
| `shared_face_center(other)` | `np.ndarray` | Center of shared face |

---

### `Edge(from_id, to_id, weight=1.0)`

Weighted edge between two box IDs.

| Field | Type |
|---|---|
| `from_id` | `int` |
| `to_id` | `int` |
| `weight` | `float` |

---

### `PlanningResult()`

| Field | Type | Description |
|---|---|---|
| `success` | `bool` | Path found? |
| `path` | `np.ndarray (N, J)` | Waypoint matrix |
| `cost` | `float` | Total C-space path length |
| `planning_time` | `float` | Wall-clock seconds |
| `first_solution_time` | `float` | Time to first solution |
| `collision_checks` | `int` | Total collision checks |
| `nodes_explored` | `int` | Graph nodes expanded |
| `phase_times` | `dict[str, float]` | Per-phase timing |
| `metadata` | `dict[str, float]` | Arbitrary metadata |
| `n_waypoints()` | `int` | Number of waypoints |
| `PlanningResult.failure(time)` | static | Create failure result |

---

### `FFBResult()`

Result of the Find-Free-Box algorithm.

| Field | Type | Description |
|---|---|---|
| `node_idx` | `int` | Tree node index (-1 on fail) |
| `path` | `list[int]` | Bisection path |
| `fail_code` | `int` | 0 = success |
| `n_new_nodes` | `int` | Nodes created |
| `n_fk_calls` | `int` | FK evaluations |
| `success()` | `bool` | `fail_code == 0` |

---

## 2. Configuration

### `EnvelopeConfig()`

| Field | Default | Description |
|---|---|---|
| `max_depth` | `1000` | Maximum bisection depth |
| `min_edge` | `0.01` | Minimum interval width |
| `min_edge_anchor` | `0.001` | Min edge for anchors |
| `min_edge_relaxed` | `0.05` | Relaxed min edge |
| `promotion_depth` | `2` | Promotion walk depth |

### `ForestConfig()`

| Field | Default | Description |
|---|---|---|
| `max_boxes` | `500` | Maximum boxes to grow |
| `max_consecutive_miss` | `20` | Stop after N misses |
| `bfs_phase_k` | `[5, 2, 1]` | Phase multipliers |
| `bfs_phase_budget` | `[100, 200, 200]` | Per-phase budgets |
| `guided_sample_ratio` | `0.6` | Goal bias probability |
| `boundary_expand_epsilon` | `0.01` | Boundary expansion |
| `n_edge_samples` | `3` | Samples per box edge |
| `adjacency_tol` | `1e-10` | Adjacency tolerance |
| `min_boxes_per_pair` | `500` | Min boxes multi-pair |
| `max_boxes_per_pair` | `5000` | Max boxes multi-pair |

### `BridgeConfig()`

| Field | Default | Description |
|---|---|---|
| `coarsen_max_rounds` | `20` | Max coarsening rounds |
| `coarsen_target_boxes` | `0` | Target box count (0=auto) |
| `coarsen_greedy_rounds` | `200` | Greedy rounds |
| `coarsen_grid_check` | `False` | Grid-based check |
| `coarsen_split_depth` | `3` | Hull split depth |
| `coarsen_max_tree_fk` | `2000` | FK budget per round |
| `corridor_hops` | `2` | GCS corridor hops |
| `use_gcs` | `False` | Use Drake GCS |

### `PlannerConfig()`

| Field | Default | Description |
|---|---|---|
| `shortcut_max_iters` | `100` | Shortcut iterations |
| `segment_resolution` | `0.05` | Collision check step |
| `parallel_grow` | `False` | Parallel growth |
| `n_partitions_depth` | `3` | Partition depth |
| `parallel_workers` | `4` | Worker count |
| `seed` | `0` | Random seed |

### `SBFConfig()`

Aggregate of all sub-configs. Key properties:

```python
cfg = pysbf2.SBFConfig()
cfg.max_boxes = 1000
cfg.shortcut_max_iters = 200
cfg.seed = 42

# Decompose
env_cfg = cfg.envelope_config()
forest_cfg = cfg.forest_config()
bridge_cfg = cfg.bridge_config()
planner_cfg = cfg.planner_config()
```

| Field | Description |
|---|---|
| `ffb_max_depth`, `ffb_min_edge`, … | Envelope params |
| `max_boxes`, `guided_sample_ratio`, … | Forest params |
| `coarsen_max_rounds`, … | Bridge params |
| `shortcut_max_iters`, `segment_resolution` | Planner params |
| `use_cache`, `cache_path` | Cache settings |
| `seed` | Global random seed |

---

## 3. Robot

### `Robot()`

Kinematic model using DH convention.

**Factory:**
```python
robot = pysbf2.Robot.from_json("configs/panda.json")
```

| Method | Returns | Description |
|---|---|---|
| `from_json(path)` | `Robot` | Static: load from JSON |
| `name()` | `str` | Robot name |
| `n_joints()` | `int` | Number of actuated joints |
| `n_links()` | `int` | Number of links |
| `n_transforms()` | `int` | Number of transforms |
| `joint_limits()` | `JointLimits&` | Reference to joint limits |
| `dh_params()` | `list[DHParam]` | DH parameter list |
| `has_tool()` | `bool` | Has tool frame? |
| `has_ee_spheres()` | `bool` | Has end-effector spheres? |
| `n_ee_spheres()` | `int` | Number of EE spheres |
| `fingerprint()` | `str` | Geometry hash |
| `fk_link_positions(q)` | `np.ndarray (N,3)` | Link positions |
| `fk_transforms(q)` | `list[np.ndarray(4,4)]` | Full transforms |

### `DHParam()`

| Field | Type | Description |
|---|---|---|
| `alpha` | `float` | Link twist (rad) |
| `a` | `float` | Link length |
| `d` | `float` | Link offset |
| `theta` | `float` | Joint angle offset |
| `joint_type` | `int` | 0=revolute, 1=prismatic |

### FK Free Functions

```python
# Scalar FK
positions = pysbf2.fk_link_positions(robot, q)
transforms = pysbf2.fk_transforms(robot, q)
T = pysbf2.dh_transform(alpha, a, d, theta)

# Interval arithmetic
sin_iv = pysbf2.I_sin(-0.5, 0.5)  # guaranteed enclosure
cos_iv = pysbf2.I_cos(-0.5, 0.5)

# Interval FK (for envelope computation)
fk_state = pysbf2.compute_fk_full(robot, intervals)
fk_inc   = pysbf2.compute_fk_incremental(parent, robot, intervals, changed_dim)
```

---

## 4. Scene & Collision

### `ICollisionChecker` (abstract)

| Method | Returns | Description |
|---|---|---|
| `check_config(q)` | `bool` | Is q collision-free? |
| `check_box(intervals)` | `bool` | Is box collision-free? |
| `check_segment(q1, q2, step=0.05)` | `bool` | Is segment free? |
| `n_checks()` | `int` | Total checks |
| `reset_counter()` | | Reset counter |

### `AabbCollisionChecker(robot, obstacles)`

Concrete AABB-based collision checker. Inherits `ICollisionChecker`.

```python
checker = pysbf2.AabbCollisionChecker(robot, obstacles)
assert checker.check_config(q_free) == True
assert checker.check_config(q_collision) == False
print(f"Collision checks: {checker.n_checks()}")
```

Additional methods: `n_obs()`, `n_aabb_slots()`.

### `Scene(obstacles=[])`

Dynamic obstacle collection manager.

```python
scene = pysbf2.Scene()
scene.add_obstacle(pysbf2.Obstacle(center, half_sizes, "box1"))
scene.add_obstacle(pysbf2.Obstacle(center2, half_sizes2, "box2"))
scene.remove_obstacle("box1")
print(scene.n_obstacles())  # 1
```

| Method | Description |
|---|---|
| `add_obstacle(obs)` | Add obstacle |
| `remove_obstacle(name)` | Remove by name |
| `clear()` | Remove all |
| `obstacles()` | Get list |
| `n_obstacles()` | Count |
| `repack()` | Rebuild compact array |

---

## 5. Envelope

> **DEPRECATED:** The Python-facing `EnvelopeResult` / `IntervalFKEnvelopeComputer`
> APIs are retained for backward compatibility only.  New C++ code should use
> `FrameStore` + `collision_policy.h` (see HCACHE03).

### `EnvelopeResult()`

| Field | Description |
|---|---|
| `n_link_slots` | Number of link AABBs |
| `n_ee_slots` | Number of EE AABBs |
| `valid` | Computation succeeded? |
| `fk_state` | `FKState` for incremental updates |

### `IntervalFKEnvelopeComputer(robot)`

Computes AABB envelopes from joint intervals using interval FK.

```python
env_comp = pysbf2.IntervalFKEnvelopeComputer(robot)
intervals = [pysbf2.Interval(q[i]-0.1, q[i]+0.1) for i in range(7)]
result = env_comp.compute_envelope(intervals)

# Incremental (after splitting dim 3)
result2 = env_comp.compute_envelope_incremental(
    result.fk_state, new_intervals, changed_dim=3)
```

| Method | Description |
|---|---|
| `compute_envelope(intervals)` | Full computation |
| `compute_envelope_incremental(fk, ivs, dim)` | Incremental update |
| `n_total_aabb_slots()` | Total AABB slots |

---

## 6. Forest

### `SafeBoxForest(n_dims, limits)`

Core data structure: a collection of safe boxes with adjacency graph.

```python
forest = pysbf2.SafeBoxForest(7, robot.joint_limits())

# Batch add
for box in boxes:
    forest.add_box_no_adjacency(box)
forest.rebuild_adjacency(tol=1e-8)

# Query
box = forest.find_containing(q)
print(f"Config q is in box {box.id}" if box else "Not covered")

# Adjacency
adj = forest.adjacency()  # dict[int, list[int]]
```

| Method | Description |
|---|---|
| `add_box_direct(box)` | Add + update adjacency |
| `add_box_no_adjacency(box)` | Add without adjacency |
| `rebuild_adjacency(tol)` | Rebuild adjacency graph |
| `find_containing(q)` | Box containing q (or None) |
| `find_nearest(q)` | Nearest box to q |
| `validate_boxes(checker)` | Remove invalid → set of IDs |
| `invalidate_against_obstacle(obs, robot, margin)` | Remove colliding |
| `n_boxes()` | Box count |
| `total_volume()` | Sum of box volumes |
| `boxes()` | `dict[int, BoxNode]` |
| `adjacency()` | `dict[int, list[int]]` |
| `allocate_id()` | Get new unique ID |

### `HierAABBTree(robot, initial_cap=64)`

Hierarchical AABB tree for FFB.

```python
tree = pysbf2.HierAABBTree(robot, initial_cap=128)

# Find Free Box
result = tree.find_free_box(seed_q, obs_compact, n_obs,
                             max_depth=200, min_edge=0.01)
if result.success():
    intervals = tree.get_node_intervals(result.node_idx)

# Persistence
tree.save("cache.hcache")
tree2 = pysbf2.HierAABBTree.load("cache.hcache", robot)
tree3 = pysbf2.HierAABBTree.load_mmap("cache.hcache", robot)  # lazy
```

### `RootSampler(limits, seed=0)`

```python
sampler = pysbf2.RootSampler(robot.joint_limits(), seed=42)
q_random = sampler.sample_uniform()
q_biased = sampler.sample_guided(goal, guided_ratio=0.6)
```

### `FFBEngine(tree, forest_config)`

FFB with phase management.

```python
from pysbf2 import FFBEngine, ForestConfig
engine = FFBEngine(tree, ForestConfig())
result = engine.find_free_box(seed, obs_compact, n_obs)
engine.advance_phase()  # relax min_edge for next phase
```

### `ForestGrower(robot, checker, tree, forest, config)`

Orchestrates the full growth algorithm.

```python
grower = pysbf2.ForestGrower(robot, checker, tree, forest, config)
success = grower.grow(start, goal)
print(f"Grew {forest.n_boxes()} boxes")
```

| Method | Description |
|---|---|
| `grow(start, goal)` | Single-pair growth |
| `grow_multi(pairs, n_random, timeout)` | Multi-pair growth |
| `regrow(n_target, timeout)` | Refill depleted regions |

---

## 7. Bridge / Coarsening

### Free Functions

```python
# Merge adjacent boxes
result = pysbf2.coarsen_forest(forest, checker, max_rounds=20)
print(f"Merged: {result.merges_performed}")

# Greedy coarsening
result = pysbf2.coarsen_greedy(forest, checker, target_boxes=100)

# Island detection
adj = forest.adjacency()
all_ids = list(forest.boxes().keys())
islands = pysbf2.find_islands(adj, all_ids)
print(f"Found {len(islands)} connected components")

# Bridge disconnected islands
bridge_result = pysbf2.bridge_islands(islands, forest.boxes())
```

### `CoarsenResult`

| Field | Description |
|---|---|
| `merges_performed` | Merge count |
| `rounds` | Rounds executed |
| `boxes_before` / `boxes_after` | Box counts |

### `UnionFind(n)`

| Method | Description |
|---|---|
| `find(x)` | Find representative |
| `unite(x, y)` | Merge sets |
| `connected(x, y)` | Same set? |
| `n_components()` | Component count |
| `components()` | `dict[int, list[int]]` |

---

## 8. Planner

### `SBFPlanner(robot, obstacles, config=SBFConfig())`

End-to-end motion planner.

```python
planner = pysbf2.SBFPlanner(robot, obstacles, config)

# One-shot planning
result = planner.plan(start, goal, timeout=30.0)

# Or: build once, query many times
planner.build(start, goal, timeout=30.0)
result1 = planner.query(start, goal)
result2 = planner.query(start2, goal2)

# Incremental updates
planner.add_obstacle(new_obs)
planner.regrow(500, timeout=10.0)
result3 = planner.query(start, goal)
```

| Method | Returns | Description |
|---|---|---|
| `plan(start, goal, timeout)` | `PlanningResult` | Build + query |
| `build(start, goal, timeout)` | | Build forest only |
| `build_multi(pairs, n_random, timeout)` | | Multi-pair build |
| `query(start, goal, timeout)` | `PlanningResult` | Query pre-built |
| `add_obstacle(obs)` | | Add + invalidate |
| `remove_obstacle(name)` | | Remove by name |
| `regrow(n_target, timeout)` | `int` | Refill boxes |
| `clear_forest()` | | Clear, keep cache |
| `forest` | `SafeBoxForest` | Access forest |
| `forest_built` | `bool` | Is forest ready? |
| `config` | `SBFConfig` | Configuration |
| `tree` | `HierAABBTree` | Access tree |

### `PathSmoother(checker, segment_resolution=0.05)`

```python
smoother = pysbf2.PathSmoother(checker, 0.05)
smoothed = smoother.shortcut(raw_path, max_iters=200)
smoothed2 = smoother.box_aware_shortcut(smoothed, forest, max_iters=100)
smoothed3 = smoother.smooth_moving_average(smoothed2, forest, window=5)
resampled = smoother.resample(smoothed3, resolution=0.02)
length = pysbf2.PathSmoother.path_length(resampled)
```

### `TreeConnector(forest, checker, radius=2.0, resolution=0.05, max_attempts=50)`

Attach start/goal to the forest graph.

### Graph Search

```python
# Dijkstra on box adjacency
result = pysbf2.dijkstra_center_distance(adj, boxes, start_ids, goal_ids)
if result.found:
    waypoints = pysbf2.extract_waypoints(result.path, boxes, start, goal)
```

### Pipeline Helpers

```python
# Panda defaults
config = pysbf2.make_panda_config(seed=42)

# One-shot convenience
result = pysbf2.plan_once(robot, obstacles, start, goal, config)

# Random scene generation
scene_cfg = pysbf2.SceneConfig()
scene_cfg.n_obstacles = 8
# obstacles = pysbf2.build_random_scene(robot, scene_cfg, start, goal, rng)
```

---

## 9. Visualization

Accessible via submodule `pysbf2.viz`:

```python
import pysbf2

# Export forest + obstacles + path
pysbf2.viz.export_forest_json(forest, obstacles, path_matrix, "forest.json")

# Export forest only
pysbf2.viz.export_forest_json(forest, "forest.json")

# Export FK frames along path
pysbf2.viz.export_path_frames(robot, path_matrix, "frames.json")

# Export box intervals as CSV
pysbf2.viz.export_boxes_csv(forest, "boxes.csv")
```

Then visualize with `python forest_viz.py forest.json`.

---

## 10. I/O

Accessible via submodule `pysbf2.io`:

```python
import pysbf2

# Obstacles
obstacles = pysbf2.io.load_obstacles_json("scene.json")
pysbf2.io.save_obstacles_json("scene.json", obstacles)

# Planning results
pysbf2.io.save_result_json("result.json", planning_result)
loaded_result = pysbf2.io.load_result_json("result.json")

# HCache persistence
cache = pysbf2.io.HCacheFile.create("tree.hcache", n_dims=7, n_links=8,
                                      limits=robot.joint_limits())
cache.flush()
cache.close()

cache2 = pysbf2.io.HCacheFile.open("tree.hcache")
print(f"Nodes: {cache2.n_nodes()}, FK calls: {cache2.n_fk_calls()}")
```

---

## 11. Feature Flags

```python
print(pysbf2.drake_available())  # True if compiled with Drake
print(pysbf2.ompl_available())   # True if compiled with OMPL
```

### `GCSOptimizer()` (requires Drake)

```python
if pysbf2.drake_available():
    gcs = pysbf2.GCSOptimizer()
    result = gcs.optimize(forest, start, goal, corridor_hops=2)
    if result.success:
        print(f"GCS cost: {result.cost:.3f}")
```

---

## Build Instructions

```bash
# From v2/ directory
mkdir build && cd build

# Without Python bindings
cmake .. -DCMAKE_BUILD_TYPE=Release

# With Python bindings
cmake .. -DCMAKE_BUILD_TYPE=Release -DSBF_WITH_PYTHON=ON

# With Drake / OMPL
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DSBF_WITH_PYTHON=ON \
         -DSBF_WITH_DRAKE=ON \
         -DSBF_WITH_OMPL=ON

cmake --build . --config Release
```

After building, add the build directory to `PYTHONPATH` or install:

```bash
# Direct import
export PYTHONPATH=/path/to/v2/build:$PYTHONPATH
python -c "import pysbf2; print(pysbf2.__doc__)"
```

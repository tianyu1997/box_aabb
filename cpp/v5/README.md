# SafeBoxForest v5

Axis-aligned bounding-box decomposition for collision-free motion planning of
serial manipulators.  SafeBoxForest (SBF) tiles configuration space with
provably collision-free AABB boxes and builds a forest (graph) that supports
efficient path planning via Dijkstra search, GCS convex optimisation, or OMPL
integration.

## Key Features

| Feature | Description |
|---------|-------------|
| **Interval Arithmetic Envelope** | Over-approximate link swept volumes with IAABB, Grid, or Hull-16 methods |
| **LECT** | Link-Envelope Configuration Tree — space-partitioning tree that maps C-space boxes to link envelopes |
| **Find-Free-Box (FFB)** | Fast descent through LECT to locate the largest collision-free box at a query point |
| **Forest Grower** | RRT or Wavefront mode, with multi-threaded parallel growth (`n_threads`) |
| **Coarsening** | Dimension-sweep, greedy merge, overlap filter — reduce forest size post-growth |
| **Path Planning** | Dijkstra shortest-path, GCS (Drake), two-pass smoothing |
| **Python Bindings** | pybind11 module with full access to planner, grower, robot, envelopes |

## Directory Structure

```
v5/
├── include/sbf/          C++ public headers
│   ├── core/             Robot, Interval, FK, joint symmetry
│   ├── envelope/         Envelope sources (IFK, Crit, Analytical, GCPC)
│   ├── lect/             LECT tree and serialisation
│   ├── ffb/              Find-Free-Box algorithm
│   ├── forest/           Grower, adjacency, connectivity, coarsening, ThreadPool
│   ├── planner/          Dijkstra, GCS, path extraction and smoothing
│   ├── scene/            SAT-based AABB collision checking
│   ├── voxel/            BitBrick, VoxelGrid, Hull rasteriser
│   ├── viz/              JSON visualisation export
│   └── adapters/         OMPL adapter
├── src/                  C++ implementation files (mirrors include/ layout)
├── test/                 C++ unit tests (doctest) and benchmarks
├── python/
│   ├── sbf5_bindings.cpp pybind11 binding source
│   ├── sbf5/             Python package (wraps _sbf5_cpp)
│   ├── sbf5_bench/       Benchmark harness
│   ├── sbf5_viz/         Visualisation utilities
│   ├── scripts/          Helper scripts
│   └── tests/            Python unit tests (pytest)
├── cmake/                FetchDeps.cmake (Eigen, JSON, doctest, pybind11)
├── data/                 Robot JSON definitions and obstacle sets
├── experiments/          Experiment configurations
├── docs/                 Additional documentation
└── CMakeLists.txt        Top-level build
```

## Prerequisites

* **CMake** ≥ 3.20
* **C++17** compiler (MSVC 2022, GCC 11+, Clang 14+)
* **Python** 3.10+ (for Python bindings)
* **Git** (for FetchContent dependency download)

Third-party libraries are fetched automatically on first configure:

| Library | Version | Purpose |
|---------|---------|---------|
| Eigen | 3.4.0 | Linear algebra |
| nlohmann/json | 3.11.3 | JSON serialisation |
| doctest | 2.4.11 | Unit tests |
| pybind11 | 2.12.0 | Python bindings |

Optional: **Drake** (detected automatically; enables the GCS convex planner).

## Build

### Configure and build (MSVC)

```bash
cmake -B build_x64 -A x64
cmake --build build_x64 --config Release
```

### Run C++ tests

```bash
cd build_x64
ctest -C Release --output-on-failure
```

Or run individual tests:

```bash
build_x64\Release\test_core.exe
build_x64\Release\test_grower.exe
build_x64\Release\test_planner.exe
```

### Install Python extension

Copy (or symlink) the built `_sbf5_cpp.pyd` (Windows) / `_sbf5_cpp.so` (Linux)
into `python/sbf5/`:

```bash
copy build_x64\Release\_sbf5_cpp.pyd python\sbf5\
```

Then add `python/` to your `PYTHONPATH`:

```bash
set PYTHONPATH=python;build_x64\Release
```

### Run Python tests

```bash
python -m pytest python/tests/ -v
```

## Quick Start (Python)

```python
from sbf5 import Robot, SBFPlanner, SBFPlannerConfig, Obstacle
import numpy as np

# Load robot
robot = Robot.from_json("data/2dof.json")

# Define obstacles
obs = [Obstacle([1.0, 1.0, 0.0], [2.0, 2.0, 1.0])]

# Build forest and plan
config = SBFPlannerConfig()
config.grower.n_threads = 4       # parallel growth
planner = SBFPlanner(robot, obs, config)
planner.build()

start = np.array([0.1, 0.1])
goal  = np.array([2.5, 2.5])
result = planner.plan(start, goal)

print(f"Path length: {len(result.waypoints)}")
print(f"Build time:  {result.build_time_ms:.1f} ms")
print(f"Plan  time:  {result.plan_time_ms:.1f} ms")
```

## Quick Start (C++)

```cpp
#include <sbf/core/robot.h>
#include <sbf/planner/sbf_planner.h>
#include <sbf/scene/collision_checker.h>

int main() {
    auto robot = sbf::Robot::from_json("data/2dof.json");

    std::vector<sbf::AABB> obstacles = { /* ... */ };
    sbf::CollisionChecker checker(obstacles);

    sbf::SBFPlannerConfig config;
    config.grower.n_threads = 4;

    sbf::SBFPlanner planner(robot, checker, config);
    planner.build();

    Eigen::VectorXd start(2), goal(2);
    start << 0.1, 0.1;
    goal  << 2.5, 2.5;

    auto result = planner.plan(start, goal);
    // result.waypoints, result.build_time_ms, ...
}
```

## CMake Options

| Option | Default | Description |
|--------|---------|-------------|
| `SBF_BUILD_TESTS` | `ON` | Build C++ unit tests |
| `SBF_BUILD_EXPERIMENTS` | `OFF` | Build experiment suite |
| `SBF_ENABLE_ASAN` | `OFF` | Enable AddressSanitizer (`/fsanitize=address` on MSVC) |

## Module Overview

### Core (`sbf/core/`)
Robot definition (DH parameters), interval arithmetic, forward kinematics state,
joint symmetry detection.

### Envelope (`sbf/envelope/`)
Compute conservative link swept-volume bounding boxes for a C-space interval.
Multiple source algorithms: IFK (interval FK), critical-point sampling, analytical
Jacobian zeros, GCPC (gradient-checked propagation of criticality).

### LECT (`sbf/lect/`)
Link-Envelope Configuration Tree — a quad/oct-tree that caches per-link envelope
AABB at each node. Supports snapshot/transplant for parallel forest growth.
Binary serialisation via `lect_io`.

### FFB (`sbf/ffb/`)
Find-Free-Box: given a query configuration, descend the LECT to find the largest
axis-aligned box guaranteed collision-free.

### Forest (`sbf/forest/`)
Forest grower (RRT and Wavefront modes), face-adjacency graph, Union-Find
connectivity and island bridging, box coarsening.  Parallel growth with
`ThreadPool` and LECT snapshots.

### Planner (`sbf/planner/`)
Dijkstra search over the adjacency graph, GCS convex optimisation (requires Drake),
waypoint extraction, two-pass path smoothing, top-level `SBFPlanner` pipeline.

### Scene (`sbf/scene/`)
SAT-based AABB-vs-obstacle collision checking.

### Voxel (`sbf/voxel/`)
512-bit BitBrick occupancy masks, flat brick-map voxel grid, Hull-16 turbo
scanline rasterisation.

### Viz (`sbf/viz/`)
JSON export for box-forest visualisation.

## Testing

C++ tests (doctest):

| Test | Module |
|------|--------|
| `test_core` | Robot, intervals, FK, joint symmetry |
| `test_endpoint_iaabb` | Endpoint envelope sources |
| `test_link_iaabb` | Link IAABB computation |
| `test_lect` | LECT build, split, query |
| `test_lect_io` | LECT serialisation round-trip |
| `test_ffb` | Find-Free-Box |
| `test_grower` | Forest growth (single & parallel) |
| `test_coarsen` | Box coarsening algorithms |
| `test_planner` | Full plan pipeline |

Python tests (pytest):

| Test | Description |
|------|-------------|
| `test_sbf5.py` | End-to-end planning through Python API |
| `test_bench.py` | Benchmark harness validation |
| `test_viz.py` | Visualisation export |

## License

See repository root for license information.

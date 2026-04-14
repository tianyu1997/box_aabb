# SafeBoxForest v6 — API Reference (English)

> **Version**: v6 (42 headers, 10 modules)
> **Language**: C++17, Eigen 3.x
> **Build**: CMake 3.16+, `-O3 -march=native`
> **Namespace**: `sbf`

---

## Architecture Overview

SBF v6 is a two-phase motion planning system for serial manipulators:

1. **Offline phase** (`build_coverage`): Construct a forest of certified safe boxes covering C-space
2. **Online phase** (`query`): Plan paths through the box forest with multi-layer optimisation

```
                    ┌─────────────────────────────────────┐
                    │         SBFPlanner (planner/)        │
                    │  build_coverage() → query(s, g)      │
                    └────┬──────────┬──────────┬──────────┘
                         │          │          │
              ┌──────────▼──┐  ┌───▼───┐  ┌──▼──────────┐
              │ ForestGrower │  │ LECT  │  │ PathSmoother│
              │ (forest/)    │  │(lect/)│  │ (planner/)  │
              └──────┬───────┘  └───┬───┘  └─────────────┘
                     │              │
           ┌─────────▼───┐   ┌─────▼─────────┐
           │ FFB (ffb/)   │   │ Envelope       │
           │ find_free_box│   │ (envelope/)    │
           └──────────────┘   └───────────────┘
```

---

## Table of Contents

- [core/types](#coretypes)
- [core/robot](#corerobot)
- [core/fk_state](#corefk_state)
- [core/log](#corelog)
- [core/constants](#coreconstants)
- [core/interval_math](#coreinterval_math)
- [core/union_find](#coreunion_find)
- [core/joint_symmetry](#corejoint_symmetry)
- [core/ray_aabb](#coreray_aabb)
- [scene/collision_checker](#scenecollision_checker)
- [envelope/endpoint_source](#envelopeendpoint_source)
- [envelope/ifk_source](#envelopeifk_source)
- [envelope/crit_source](#envelopecrit_source)
- [envelope/analytical_source](#envelopeanalytical_source)
- [envelope/gcpc_source](#envelopegcpc_source)
- [envelope/dh_enumerate](#envelopedh_enumerate)
- [envelope/link_iaabb](#envelopelink_iaabb)
- [envelope/link_grid](#envelopelink_grid)
- [envelope/envelope_type](#envelopeenvelope_type)
- [lect/lect](#lectlect)
- [lect/lect_io](#lectlect_io)
- [lect/lect_mmap](#lectlect_mmap)
- [lect/lect_cache_manager](#lectlect_cache_manager)
- [lect/z4_ep_cache](#lectz4_ep_cache)
- [lect/z4_grid_cache](#lectz4_grid_cache)
- [ffb/ffb](#ffbffb)
- [forest/grower](#forestgrower)
- [forest/adjacency](#forestadjacency)
- [forest/coarsen](#forestcoarsen)
- [forest/connectivity](#forestconnectivity)
- [forest/thread_pool](#forestthread_pool)
- [planner/i_planner](#planneri_planner)
- [planner/sbf_planner](#plannersbf_planner)
- [planner/dijkstra](#plannerdijkstra)
- [planner/path_extract](#plannerpath_extract)
- [planner/path_smoother](#plannerpath_smoother)
- [planner/gcs_planner](#plannergcs_planner)
- [voxel/bit_brick](#voxelbit_brick)
- [voxel/voxel_grid](#voxelvoxel_grid)
- [voxel/hull_rasteriser](#voxelhull_rasteriser)
- [viz/viz_exporter](#vizviz_exporter)
- [adapters/ompl_adapter](#adaptersompl_adapter)
- [Python Bindings (sbf6)](#python-bindings-sbf6)

---

## core/types

> Header: `include/sbf/core/types.h`

### `Interval` — Closed interval [lo, hi]

```cpp
struct Interval {
    double lo, hi;

    double width() const;          // hi - lo
    double mid() const;            // (lo + hi) / 2
    bool contains(double v) const;
    Interval hull(const Interval& o) const;
    Interval intersect(const Interval& o) const;
    bool overlaps(const Interval& o) const;

    // Interval arithmetic
    Interval operator+(const Interval& o) const;
    Interval operator-(const Interval& o) const;
    Interval operator*(const Interval& o) const;
};
```

### `Obstacle` — Workspace AABB

```cpp
struct Obstacle {
    float bounds[6]; // [lx, ly, lz, hx, hy, hz]

    Obstacle() = default;
    Obstacle(float lx, float ly, float lz, float hx, float hy, float hz);
};
```

### `BoxNode` — C-space safe box

```cpp
struct BoxNode {
    int id;
    std::vector<Interval> joint_intervals;  // Per-joint [lo, hi]
    Eigen::VectorXd seed_config;            // Seed configuration
    double volume;                          // Product of dimension widths
    int tree_id;                            // LECT leaf node index
    int parent_box_id;                      // Parent box ID (-1 = root)
    int root_id;                            // Subtree root ID

    int n_dims() const;
    Eigen::VectorXd center() const;
    void compute_volume();
    bool contains(const Eigen::VectorXd& q, double tol) const;
};
```

---

## core/robot

> Header: `include/sbf/core/robot.h`

### `Robot` — DH-parameterised serial robot

```cpp
class Robot {
public:
    static Robot from_json(const std::string& path);

    const std::string& name() const;
    int n_joints() const;
    const std::vector<DHParam>& dh_params() const;
    const JointLimits& joint_limits() const;
    int n_active_links() const;
    int n_active_endpoints() const;
    const int* active_link_map() const;
    bool has_link_radii() const;
    uint64_t fingerprint() const;  // FNV-1a 64-bit fingerprint (identifies LECT cache)
};
```

---

## core/fk_state

> Header: `include/sbf/core/fk_state.h`

### `FKState` — Forward kinematics state

Stores prefix matrices and endpoint coordinates for incremental FK computation. Used by LECT and envelope modules for the IFK fast path.

---

## core/log

> Header: `include/sbf/core/log.h` (v6 new)

Centralised logging macros extracted from scattered source files.

```cpp
#define SBF_LOG_INFO(fmt, ...)  // Info level
#define SBF_LOG_WARN(fmt, ...)  // Warning level
#define SBF_LOG_ERR(fmt, ...)   // Error level
#define SBF_LOG_DBG(fmt, ...)   // Debug level (compile-time gated)
```

---

## core/constants

> Header: `include/sbf/core/constants.h` (v6 new)

Named constants consolidated from magic numbers across the codebase.

```cpp
namespace sbf::constants {
    constexpr double kDefaultMinEdge = 1e-4;
    constexpr int    kDefaultMaxDepth = 30;
    constexpr int    kRootMaxDepth = 60;
    constexpr double kAdjacencyTol = 1e-10;
    constexpr double kBoundaryEpsilon = 1e-6;
    // ... etc.
}
```

---

## core/interval_math

> Header: `include/sbf/core/interval_math.h`

Interval arithmetic utilities: `sin_iv`, `cos_iv`, `mul_iv`, `add_iv`, `hull_iv`, `intersect_iv` for `Interval` types.

---

## core/union_find

> Header: `include/sbf/core/union_find.h` (v6: moved from forest/)

### `UnionFind` — Weighted union-find with path compression

```cpp
class UnionFind {
public:
    explicit UnionFind(int n = 0);
    int find(int x);               // Path compression
    void unite(int x, int y);      // Union by rank
    bool connected(int x, int y);  // O(α(n))
    int size() const;
    void resize(int new_size);     // Incremental expansion
};
```

---

## core/joint_symmetry

> Header: `include/sbf/core/joint_symmetry.h`

Z4 rotational symmetry detection and transformation for joint 0.

---

## core/ray_aabb

> Header: `include/sbf/core/ray_aabb.h`

Ray–AABB intersection test (slab method).

---

## scene/collision_checker

> Header: `include/sbf/scene/collision_checker.h`

### `CollisionChecker` — Thread-safe collision detector

```cpp
class CollisionChecker {
public:
    CollisionChecker();
    CollisionChecker(const Robot& robot, const std::vector<Obstacle>& obstacles);
    void set_obstacles(const Obstacle* obs, int n_obs);

    bool check_config(const Eigen::VectorXd& q) const;       // Single-config collision
    bool check_box(const std::vector<Interval>& ivs) const;  // Box collision
    bool check_segment(const Eigen::VectorXd& a,              // Segment collision
                       const Eigen::VectorXd& b,
                       int resolution = 10) const;

    const Robot& robot() const;
    int n_obs() const;
};
```

> **Thread safety**: All const methods; safe to share across threads.

---

## envelope/endpoint_source

> Header: `include/sbf/envelope/endpoint_source.h`

### `EndpointSource` enum

| Value | ID | Quality | Safety | Description |
|-------|-----|---------|--------|-------------|
| `IFK` | 0 | 0 | SAFE | Interval FK (fastest, ⊇ true reachable set) |
| `CritSample` | 1 | 1 | UNSAFE | Boundary k·π/2 sampling |
| `Analytical` | 2 | 2 | SAFE | Analytical gradient zero enumeration |
| `GCPC` | 3 | 3 | SAFE | Geometric critical-point cache |

### `EndpointSourceConfig`

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `source` | `EndpointSource` | `IFK` | Source selection |
| `n_samples_crit` | `int` | `1000` | CritSample sample count |
| `max_phase_analytical` | `int` | `3` | Analytical max phase (0..3) |
| `gcpc_cache` | `const GcpcCache*` | `nullptr` | GCPC cache pointer |

### Source Substitution Matrix

| Cached \ Requested | IFK | CritSample | Analytical | GCPC |
|--------------------|-----|------------|------------|------|
| **IFK** | ✓ | ✓ | ✗ | ✗ |
| **CritSample** | ✗ | ✓ | ✗ | ✗ |
| **Analytical** | ✓ | ✓ | ✓ | ✗ |
| **GCPC** | ✓ | ✓ | ✓ | ✓ |

### Key Functions

```cpp
EndpointIAABBResult compute_endpoint_iaabb(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    const EndpointSourceConfig& config,
    FKState* fk = nullptr,
    int changed_dim = -1);

bool source_can_serve(EndpointSource cached, EndpointSource requested);
int source_channel(EndpointSource s);  // IFK → CH_SAFE(0), others → CH_UNSAFE(1)
```

---

## lect/lect

> Header: `include/sbf/lect/lect.h`

### `LECT` — Lifelong Envelope Cache Tree

Lazy KD-tree over C-space with dual-channel link envelope caching.

#### Construction

```cpp
LECT(const Robot& robot,
     const std::vector<Interval>& root_intervals,
     const EndpointSourceConfig& ep_config,
     const EnvelopeTypeConfig& env_config,
     int initial_cap = 1023);
```

#### Metadata

```cpp
int n_nodes() const;
int n_dims() const;
int n_active_links() const;
int capacity() const;
const Robot& robot() const;
```

#### Runtime Source Override

```cpp
void set_ep_config(const EndpointSourceConfig& cfg);
```

Override endpoint source. Call after `lect_load_binary()`.

#### Dual-Channel Data Access

```cpp
bool has_data(int i) const;
bool has_safe_data(int i) const;
bool has_unsafe_data(int i) const;

const float* get_endpoint_iaabbs(int i, int channel = CH_SAFE) const;
const float* get_link_iaabbs(int i) const;  // Lazy hull of both channels

EndpointSource source_quality(int i, int channel = CH_SAFE) const;
```

#### Tree Structure

```cpp
int left(int i) const;
int right(int i) const;
int parent(int i) const;
int depth(int i) const;
bool is_leaf(int i) const;
int get_split_dim(int i) const;
double split_val(int i) const;
std::vector<Interval> node_intervals(int node_idx) const;
```

#### Core Operations

```cpp
int expand_leaf(int node_idx);
int expand_leaf(int node_idx, const FKState& fk,
                const std::vector<Interval>& intervals);

void compute_envelope(int node_idx, const FKState& fk,
                      const std::vector<Interval>& intervals,
                      int changed_dim = -1, int parent_idx = -1);
```

Three execution paths:
1. **IFK fast path** (IFK source + valid FK): 40–45% FK savings
2. **Z4 cache lookup** (non-canonical sector): transform from cache
3. **General fallback**: call `compute_endpoint_iaabb()`

#### Collision Queries

```cpp
bool collides_scene(int node_idx, const Obstacle* obs, int n_obs) const;
bool intervals_collide_scene(const std::vector<Interval>& intervals,
                             const Obstacle* obs, int n_obs) const;
```

#### Occupancy Management

```cpp
void mark_occupied(int node_idx, int box_id);
void unmark_occupied(int node_idx);
bool is_occupied(int node_idx) const;
int subtree_occ(int node_idx) const;
void clear_all_occupation();
bool is_point_occupied(const Eigen::VectorXd& q) const;
```

#### Split Strategy

```cpp
void set_split_order(SplitOrder so);
// ROUND_ROBIN | WIDEST_FIRST | BEST_TIGHTEN (default)
```

#### Snapshot & Parallelism

```cpp
LECT snapshot() const;   // Deep copy
int warmup(int max_depth, int n_paths, int seed = 42);
int transplant_subtree(const LECT& worker, int snapshot_base,
                       const std::unordered_map<int, int>& id_map);
```

---

## lect/lect_io

> Header: `include/sbf/lect/lect_io.h`

```cpp
bool lect_save_binary(const LECT& lect, const std::string& path);
bool lect_save_incremental(const LECT& lect, const std::string& path, int old_n_nodes);
bool lect_load_binary(LECT& lect, const Robot& robot, const std::string& path);
```

V4 binary format supports dual-channel and grid data. V1–V3 backward compatible.

---

## lect/lect_mmap

> Header: `include/sbf/lect/lect_mmap.h`

Memory-mapped LECT file access for zero-copy loading of large cache files.

---

## lect/lect_cache_manager

> Header: `include/sbf/lect/lect_cache_manager.h`

Cache path resolution, fingerprint matching, and cache file lifecycle management.

---

## lect/z4_ep_cache & z4_grid_cache

> Headers: `include/sbf/lect/z4_ep_cache.h`, `include/sbf/lect/z4_grid_cache.h`

Z4 rotational symmetry caches for endpoint-iAABBs and voxel grids.

---

## ffb/ffb

> Header: `include/sbf/ffb/ffb.h`

### `FFBResult`

| Field | Type | Description |
|-------|------|-------------|
| `node_idx` | `int` | Target LECT leaf (-1 = failed) |
| `fail_code` | `int` | 0=success, 1=occupied, 2=depth, 3=min\_edge, 4=timeout |
| `n_new_nodes` | `int` | Newly expanded nodes |
| `total_ms` | `double` | Total FFB time |

### `FFBConfig`

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `min_edge` | `double` | `1e-4` | Minimum interval width |
| `max_depth` | `int` | `30` | Maximum tree depth |
| `deadline_ms` | `double` | `0.0` | Timeout (0 = unlimited) |

### Main Function

```cpp
FFBResult find_free_box(
    LECT& lect,
    const Eigen::VectorXd& seed,
    const Obstacle* obs, int n_obs,
    const FFBConfig& config = FFBConfig());
```

---

## forest/grower

> Header: `include/sbf/forest/grower.h`

### `GrowerConfig`

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `mode` | `Mode` | `WAVEFRONT` | `RRT` or `WAVEFRONT` |
| `max_boxes` | `int` | `500` | Per-tree box limit |
| `max_consecutive_miss` | `int` | `50` | Consecutive FFB failure limit |
| `rrt_goal_bias` | `double` | `0.5` | RRT goal bias |
| `connect_mode` | `bool` | `false` | Coordinated multi-tree growth |
| `n_threads` | `int` | hw\_concurrency | Thread count |

**`connect_mode = true` recommended values**: `rrt_goal_bias=0.8`, `rrt_step_ratio=0.05`, `max_consecutive_miss=2000`

### `ForestGrower`

```cpp
class ForestGrower {
public:
    ForestGrower(const Robot& robot, LECT& lect, const GrowerConfig& config);
    ForestGrower(const Robot& robot, LECT&& lect, const GrowerConfig& config);

    void set_multi_goals(const std::vector<VectorXd>& goals);
    GrowerResult grow(const Obstacle* obs, int n_obs);

    const std::vector<BoxNode>& boxes() const;
    LECT&& take_lect();
};
```

---

## forest/adjacency

> Header: `include/sbf/forest/adjacency.h`

```cpp
using AdjacencyGraph = std::unordered_map<int, std::vector<int>>;

AdjacencyGraph compute_adjacency(const std::vector<BoxNode>& boxes, double tol = 1e-10);
std::optional<SharedFace> shared_face(const BoxNode& a, const BoxNode& b, double tol = 1e-10);
std::unordered_set<int> find_articulation_points(const AdjacencyGraph& adj);
```

---

## forest/coarsen

> Header: `include/sbf/forest/coarsen.h`

Five-level coarsening pipeline:

```cpp
// 1. Dimension-scan merge (safe by construction)
CoarsenResult coarsen_forest(std::vector<BoxNode>& boxes, const CollisionChecker& checker,
                              int max_rounds = 20);

// 2. Relaxed dimension-scan merge
CoarsenResult coarsen_sweep_relaxed(std::vector<BoxNode>& boxes,
                                     const CollisionChecker& checker, LECT* lect = nullptr);

// 3. Greedy adjacency merge (hull scoring + hull_region_safe)
GreedyCoarsenResult coarsen_greedy(std::vector<BoxNode>& boxes,
                                     const CollisionChecker& checker,
                                     const GreedyCoarsenConfig& config, LECT* lect = nullptr);

// 4. Multi-box cluster merge
ClusterCoarsenResult coarsen_cluster(std::vector<BoxNode>& boxes,
                                      const CollisionChecker& checker);

// 5. Overlap filtering
int filter_coarsen_overlaps(std::vector<BoxNode>& boxes);
```

Pipeline: Grow → sweep → relaxed → greedy → cluster → Bridge → sweep → relaxed → greedy → cluster → filter → adjacency

---

## forest/connectivity

> Header: `include/sbf/forest/connectivity.h`

```cpp
std::vector<std::vector<int>> find_islands(const AdjacencyGraph& adj);

std::vector<VectorXd> rrt_connect(
    const VectorXd& q_a, const VectorXd& q_b,
    const CollisionChecker& checker, const Robot& robot,
    const RRTConnectConfig& cfg = {}, int seed = 42);

int chain_pave_along_path(const std::vector<VectorXd>& rrt_path,
                           int anchor_box_id, std::vector<BoxNode>& boxes, LECT& lect,
                           const Obstacle* obs, int n_obs, const FFBConfig& ffb_config,
                           AdjacencyGraph& adj, int& next_box_id, const Robot& robot);

int bridge_all_islands(std::vector<BoxNode>& boxes, LECT& lect,
                        const Obstacle* obs, int n_obs, AdjacencyGraph& adj,
                        const FFBConfig& ffb_config, int& next_box_id,
                        const Robot& robot, const CollisionChecker& checker);
```

---

## forest/thread_pool

> Header: `include/sbf/forest/thread_pool.h`

```cpp
class ThreadPool {
public:
    explicit ThreadPool(int n_threads);
    ~ThreadPool();

    template<typename F, typename... Args>
    auto submit(F&& f, Args&&... args) -> std::future<result_of_t<F(Args...)>>;

    int size() const;
};
```

---

## planner/i_planner

> Header: `include/sbf/planner/i_planner.h`

```cpp
class IPlanner {
public:
    virtual ~IPlanner() = default;
    virtual PlanResult plan(const VectorXd& start, const VectorXd& goal,
                            const Obstacle* obs, int n_obs,
                            double timeout_ms = 30000.0) = 0;
};
```

---

## planner/sbf_planner

> Header: `include/sbf/planner/sbf_planner.h`

### `SBFPlannerConfig`

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `grower` | `GrowerConfig` | — | Forest growth config |
| `coarsen` | `GreedyCoarsenConfig` | — | Coarsening config |
| `smoother` | `SmootherConfig` | — | Path smoothing config |
| `proxy` | `ProxySearchConfig` | — | Proxy search config |
| `use_gcs` | `bool` | `false` | Enable GCS mode |
| `endpoint_source` | `EndpointSourceConfig` | IFK | Endpoint source |
| `split_order` | `SplitOrder` | `BEST_TIGHTEN` | Split strategy |
| `z4_enabled` | `bool` | `true` | Z4 symmetry |
| `lect_no_cache` | `bool` | `false` | Disable persistence |
| `lect_cache_dir` | `string` | `~/.sbf_cache` | Cache directory |

### `SBFPlanner`

```cpp
class SBFPlanner : public IPlanner {
public:
    SBFPlanner(const Robot& robot, const SBFPlannerConfig& config = {});

    // All-in-one
    PlanResult plan(const VectorXd& start, const VectorXd& goal,
                    const Obstacle* obs, int n_obs,
                    double timeout_ms = 30000.0) override;

    // Separated build / query
    void build(const VectorXd& start, const VectorXd& goal,
               const Obstacle* obs, int n_obs, double timeout_ms = 30000.0);

    void build_coverage(const Obstacle* obs, int n_obs,
                        double timeout_ms = 30000.0,
                        const std::vector<VectorXd>& seed_points = {});

    PlanResult query(const VectorXd& start, const VectorXd& goal,
                     const Obstacle* obs = nullptr, int n_obs = 0);

    const std::vector<BoxNode>& boxes() const;
    const AdjacencyGraph& adjacency() const;
    int n_boxes() const;
    void clear_forest();
    int warmup_lect(int max_depth, int n_paths, int seed = 42);
};
```

### `PlanResult`

| Field | Type | Description |
|-------|------|-------------|
| `success` | `bool` | Path found |
| `path` | `vector<VectorXd>` | Smoothed C-space waypoints |
| `box_sequence` | `vector<int>` | Box ID sequence |
| `path_length` | `double` | Joint-space Euclidean path length |
| `planning_time_ms` | `double` | Total wall-clock time |
| `n_boxes` | `int` | Post-coarsen box count |
| `build_time_ms` | `double` | Forest build time |
| `lect_time_ms` | `double` | LECT build/load time |

---

## planner/dijkstra

> Header: `include/sbf/planner/dijkstra.h`

```cpp
DijkstraResult dijkstra_search(
    const AdjacencyGraph& adj, const std::vector<BoxNode>& boxes,
    int start_box_id, int goal_box_id,
    const Eigen::VectorXd& goal_point = Eigen::VectorXd());
```

A* search with Euclidean heuristic h(n) = ||c_n - q_g||₂.

---

## planner/path_extract

> Header: `include/sbf/planner/path_extract.h`

```cpp
std::vector<Eigen::VectorXd> extract_waypoints(
    const std::vector<int>& box_sequence,
    const std::vector<BoxNode>& boxes,
    const Eigen::VectorXd& start, const Eigen::VectorXd& goal);
```

---

## planner/path_smoother

> Header: `include/sbf/planner/path_smoother.h`

```cpp
std::vector<VectorXd> shortcut(const std::vector<VectorXd>& path,
                                const CollisionChecker& checker,
                                const SmootherConfig& config, uint64_t seed = 42);

double path_length(const std::vector<VectorXd>& path);
```

### Five-step optimisation pipeline (in sbf_planner)

| Step | Operation | Safety |
|------|-----------|--------|
| 1 | Greedy forward simplification | Per-segment |
| 2 | Random shortcut | Direct check |
| 2.5 | Densify (>0.3 rad) | Skip colliding |
| 3 | Elastic Band (60 iter, multi-α) | Per-seg revert |
| 4 | Final shortcut | Revert |
| 5 | Pass 2 mini EB (30 iter) | Full revert |

---

## planner/gcs_planner

> Header: `include/sbf/planner/gcs_planner.h`
> Conditional: `SBF_HAS_DRAKE`

```cpp
GCSResult gcs_plan(const AdjacencyGraph& adj,
                    const std::vector<BoxNode>& boxes,
                    const VectorXd& start, const VectorXd& goal,
                    const GCSConfig& config = {},
                    const CollisionChecker* checker = nullptr,
                    LECT* lect = nullptr);
```

Corridor coarsening: when Dijkstra corridor exceeds `max_gcs_verts`, group consecutive boxes and verify with `hull_region_safe(depth=6)`.

---

## voxel/bit_brick

> Header: `include/sbf/voxel/bit_brick.h`

`BitBrick`: 8³ = 512 voxels stored as 8 × uint64 (64 bytes, one cache line).

---

## voxel/voxel_grid

> Header: `include/sbf/voxel/voxel_grid.h`

Sparse voxel grid: hash map of `(ix, iy, iz) → BitBrick`. FNV-1a spatial hashing.

---

## voxel/hull_rasteriser

> Header: `include/sbf/voxel/hull_rasteriser.h`

16-point convex hull rasterisation with turbo scanline algorithm. O(ny × nz) complexity, O(1) per scanline.

---

## viz/viz_exporter

> Header: `include/sbf/viz/viz_exporter.h`

Export box forest and paths to JSON/HTML for visualisation.

---

## adapters/ompl_adapter

> Header: `include/sbf/adapters/ompl_adapter.h`

OMPL `StateValidityChecker` and `MotionValidator` adapters for interoperability.

---

## Python Bindings (sbf6)

> Binding: `python/sbf6_bindings.cpp`

### Typical Usage

```python
import numpy as np
from sbf6 import (Robot, SBFPlanner, SBFPlannerConfig,
                   EndpointSource, SplitOrder, Obstacle)

# Configure
config = SBFPlannerConfig()
config.endpoint_source.source = EndpointSource.IFK
config.lect_cache_dir = "./lect_cache"

# Build
robot = Robot.from_json("iiwa14.json")
planner = SBFPlanner(robot, config)

obs = [Obstacle(0.3, -0.3, 0.0, 0.5, 0.3, 0.8)]
planner.build_coverage(obs, len(obs), timeout_ms=60000.0)

# Query
result = planner.query(
    start=np.array([0, 0.5, 0, -1.5, 0, 0.5, 1.57]),
    goal=np.array([1.57, 0.3, 0, -0.8, 0, 1.0, 0]))

print(f"Success={result.success}, len={result.path_length:.3f}, "
      f"boxes={result.n_boxes}, time={result.planning_time_ms:.0f}ms")
```

### `PlanResult` (Python)

| Attribute | Type | Description |
|-----------|------|-------------|
| `success` | `bool` | Planning succeeded |
| `path` | `ndarray` | Waypoints (N×D) |
| `box_sequence` | `list[int]` | Box IDs |
| `path_length` | `float` | Path length (rad) |
| `planning_time_ms` | `float` | Total time |
| `n_boxes` | `int` | Box count |
| `build_time_ms` | `float` | Build time |
| `lect_time_ms` | `float` | LECT time |

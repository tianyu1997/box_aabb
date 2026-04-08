# SafeBoxForest v3  API Reference

> Migrated from v2. Covers currently migrated modules: common, robot, envelope, scene.

---

## 1. Common Types (`include/sbf/common/types.h`)

### `Interval`
| Member | Type | Description |
|---|---|---|
| `lo`, `hi` | `double` | Lower/upper bound |
| `width()` | `double` | `hi - lo` |
| `center()` / `mid()` | `double` | `(lo + hi) / 2` |
| `contains(v, tol)` | `bool` | Point containment |
| `overlaps(other, tol)` | `bool` | Interval overlap |
| `+`, `-`, `*` | `Interval` | Interval arithmetic |
| `hull(other)` | `Interval` | Convex hull of two intervals |

### `AABB3D`
| Member | Type | Description |
|---|---|---|
| `lo[3]`, `hi[3]` | `float` | Axis-aligned bounding box |
| `overlaps(o, eps)` | `bool` | 3D overlap test |

### `Obstacle`
| Member | Type | Description |
|---|---|---|
| `center`, `half_sizes` | `Vector3d` | Box center and half extents |
| `lo()`, `hi()` | `Vector3d` | Corner accessors |

### `BoxNode`
C-space box with `id`, `joint_intervals`, `seed_config`, `volume`, adjacency info.

| Field | Type | Default | Description |
|---|---|---|---|
| `id` | `int` | `-1` | Unique box identifier |
| `joint_intervals` | `vector<Interval>` | — | C-space box bounds per dimension |
| `seed_config` | `VectorXd` | — | Seed configuration that generated this box |
| `volume` | `double` | `0.0` | C-space volume (product of interval widths) |
| `parent_id` | `int` | `-1` | Parent box ID (tree structure) |
| `tree_id` | `int` | `-1` | LECT tree node index |
| `children_ids` | `vector<int>` | — | Child box IDs |
| `parent_box_id` | `int` | `-1` | Expansion source box (`-1` = root) |
| `expand_face_dim` | `int` | `-1` | Dimension of the face used for expansion |
| `expand_face_side` | `int` | `-1` | `0` = lo face, `1` = hi face |
| `root_id` | `int` | `-1` | Which root tree this box belongs to |

Key methods: `contains()`, `is_adjacent_to()`, `shared_face_center()`, `overlaps_with()`, `distance_to_config()`, `nearest_point_to()`.

### `JointLimits`
Vector of `Interval` with `contains()` and `clamp()`.

### Other: `PlanningResult`, `FFBResult`, `Edge`

### `EnvelopeConfig` (`include/sbf/common/config.h`)
Module-level envelope configuration.

| Field | Type | Default | Description |
|---|---|---|---|
| `max_depth` | `int` | `1000` | Maximum tree depth |
| `min_edge` | `double` | `0.01` | Minimum box edge for splitting |
| `min_edge_anchor` | `double` | `0.001` | Min edge for anchor nodes |
| `min_edge_relaxed` | `double` | `0.05` | Relaxed min edge |
| `store_format` | `StoreFormat` | `FRAMES` | `AABB_LEGACY` or `FRAMES` |
| `collision_policy` | `CollisionPolicy` | `AABB` | `AABB`, `AABB_SUBDIV`, or `GRID` |
| `aa_crossover_width` | `double` | `0.0` | **Hybrid IA/AA**: when max interval width ≤ threshold, use AA for AABB. 0 = disabled. Recommended: 0.05–0.10 |

### `PipelineConfig` (`include/sbf/envelope/envelope_type.h`)
Combined source + envelope configuration with factory methods.

| Field | Type | Default | Description |
|---|---|---|---|
| `source` | `EndpointSourceConfig` | — | Endpoint source method config |
| `envelope` | `EnvelopeTypeConfig` | — | Envelope type config |
| `aa_crossover_width` | `double` | `0.0` | **Hybrid IA/AA** crossover width (same semantics as `EnvelopeConfig`) |

| Factory | Source | Envelope | Description |
|---|---|---|---|
| `recommended()` | CritSample | Hull16_Grid | Best cost/quality tradeoff |
| `fast()` | IFK | SubAABB(n_sub=1) | Fastest, most conservative |
| `tightest()` | AnalyticalCrit | Hull16_Grid_Analytical | Slowest, tightest |
| `production()` | IFK | Hull16_Grid | Default (fast + tight) |

---

## 2. Robot (`include/sbf/robot/`)

### `Robot` class
| Method | Returns | Description |
|---|---|---|
| `from_json(path)` | `Robot` | Load from JSON config |
| `n_joints()` | `int` | Number of DOF |
| `n_active_links()` | `int` | Collision-relevant links |
| `dh_params()` | `vector<DHParam>` | DH parameter chain |
| `joint_limits()` | `JointLimits` | Joint range limits |
| `link_radii()` | `vector<double>` | Capsule radii per link |
| `active_link_map()` | `const int*` | Maps active link index to frame |
| `fk_link_positions(q)` | `MatrixXd` | Scalar FK for all links |
| `fk_transforms(q)` | `vector<Matrix4d>` | Full FK transforms |

### Interval FK (`interval_fk.h`)
| Function | Description |
|---|---|
| `compute_fk_full(robot, intervals)` | Full interval-FK from scratch |
| `compute_fk_incremental(parent, robot, intervals, dim)` | Incremental from parent state |
| `extract_link_aabbs(state, ...)` | Derive link AABBs from FK state |
| `compute_link_aabbs(robot, intervals, out)` | One-shot: FK + extract |
| `compute_all_aabbs(robot, intervals, out_link, out_ee)` | Links + EE spheres |

### Interval Math (`interval_math.h`)
| Function | Description |
|---|---|
| `I_sin(lo, hi)` / `I_cos(lo, hi)` | Interval sin/cos |
| `build_dh_joint(...)` | Build interval DH matrix |
| `imat_mul_dh(...)` | Interval 4x4 matrix multiply |

### Affine-Arithmetic FK (`affine_fk.h`)

Reduced Affine Arithmetic (AA) FK for DH chains. Each scalar is represented as $\hat{x} = x_0 + \sum_i x_i \varepsilon_i + \delta \varepsilon^*$ where $\varepsilon_i \in [-1,1]$ are noise symbols (one per joint) and $\delta \geq 0$ is accumulated nonlinear error. Key advantage over IA: $\cos(q_j)$ and $\sin(q_j)$ share noise symbol $\varepsilon_j$, preserving $\cos^2+\sin^2=1$ correlation through the chain.

#### Constants

| Constant | Value | Description |
|---|---|---|
| `AA_NSYM` | `8` | Number of noise symbols (one per joint, 8 suffices for 7-DOF arms) |

#### `AffineForm`
| Member / Method | Type | Description |
|---|---|---|
| `x0` | `double` | Central value |
| `xi[AA_NSYM]` | `double[8]` | Partial deviations (noise coefficients) |
| `delta` | `double` | Accumulated nonlinear error bound (≥ 0) |
| `radius()` | `double` | Total deviation: $\delta + \sum |x_i|$ |
| `lo()`, `hi()` | `double` | Interval bounds: $x_0 \mp \text{radius}$ |
| `constant(v)` | `AffineForm` | (static) Create from scalar constant |
| `from_interval(lo, hi, sym_id)` | `AffineForm` | (static) Create from interval with noise symbol `sym_id` |

#### Arithmetic

| Function | Description |
|---|---|
| `aa_add(a, b)` | Affine addition |
| `aa_sub(a, b)` | Affine subtraction |
| `aa_neg(a)` | Affine negation |
| `aa_scale(a, s)` | Scale by constant |
| `aa_mul(a, b)` | Reduced AA multiplication with $R_x \cdot R_y$ nonlinear remainder |

#### Trigonometric (Taylor linearisation)

| Function | Returns | Description |
|---|---|---|
| `aa_cos_sin(theta)` | `AACosSin` | Min-range 1st-order Taylor: $\cos(\hat\theta) \approx \cos\theta_0 - \sin\theta_0 \cdot \Delta$, remainder $\leq R^2/2$. Both cos and sin share the same noise symbols. |
| `aa_cos_sin_interval(lo, hi, sym_id)` | `AACosSin` | Convenience: directly from interval bounds |

#### `AAMatrix4`
3×4 homogeneous matrix of `AffineForm` (4th row implicitly `[0,0,0,1]`).

| Method | Description |
|---|---|
| `identity()` | (static) AA identity matrix |

| Function | Description |
|---|---|
| `aa_mat_mul(A, B)` | 4×4 AA matrix multiplication |

#### `AAFKResult`
| Member | Type | Description |
|---|---|---|
| `prefix` | `vector<AAMatrix4>` | Prefix transforms (base, T₀, T₀T₁, ...) |
| `n_tf` | `int` | Number of transforms |
| `valid` | `bool` | Computation succeeded |

#### Core API

| Function | Description |
|---|---|
| `aa_compute_fk(robot, intervals)` | Full AA FK chain → `AAFKResult` |
| `aa_position_bounds(T, lo, hi)` | Extract xyz position interval from `AAMatrix4` |
| `aa_extract_link_aabbs(state, link_map, n, out, radii)` | Per-link AABB from AA prefix transforms (analogous to IA `extract_link_aabbs`) |
| `aa_compute_link_aabbs(robot, intervals, out)` | Convenience: AA FK + extract in one call |

**Performance characteristics** (vs IA, on 7-DOF Panda/IIWA14):
- **~6.5× faster** (~3.1 µs vs ~20.5 µs per call)
- **Tighter at narrow intervals** (width ≤ 0.05): AA/MC ≈ 1.37 vs IA/MC ≈ 1.87
- **Explodes at wide intervals** (width > 0.2): Taylor remainder $R^2/2$ compounds through chain
- Both methods are 100% sound (always contain Monte Carlo ground truth)

### Hybrid IA/AA AABB (`hybrid_aabb.h`)

Automatic dispatcher that selects IA or AA based on maximum interval width.

| Function | Returns | Description |
|---|---|---|
| `max_interval_width(intervals)` | `double` | Maximum `hi - lo` across all joints |
| `extract_link_aabbs_hybrid(fk, intervals, robot, out, aa_crossover)` | `bool` | If `aa_crossover > 0` and `max_width ≤ aa_crossover`: uses AA (returns `true`). Otherwise: uses IA from pre-computed FKState (returns `false`). |

**Integration**: Wired into `LECT::compute_envelope()` and `AabbCollisionChecker::check_box()` — both accept `aa_crossover_width` via config.

---

## 3. Envelope (`include/sbf/envelope/`)

### Two-Stage Pipeline Architecture

The envelope pipeline uses a clean **two-stage architecture**:

```
Stage 1 (EndpointSource): C-space intervals → endpoint_aabbs [n_endpoints × 6]
Stage 2 (EnvelopeType):   endpoint_aabbs → LinkEnvelope (SubAABB / Grid / VoxelGrid)
```

**4 × 3 = 12 pipeline combinations:**

| | SubAABB | SubAABB_Grid | Hull16_Grid |
|---|---|---|---|
| **IFK** | ✓ | ✓ | ✓ |
| **CritSample** | ✓ | ✓ | ✓ |
| **Analytical** | ✓ | ✓ | ✓ |
| **GCPC** | ✓ | ✓ | ✓ |

### Stage 1: Endpoint Source (`frame_source.h`)

#### `EndpointSource` (enum)
| Value | Name | Description |
|---|---|---|
| `0` | `IFK` | Interval FK (fast, conservative) |
| `1` | `CritSample` | Critical-point sampling (medium, tighter) |
| `2` | `Analytical` | Analytical Jacobian solver (slow, tightest) |
| `3` | `GCPC` | Global Critical Point Cache + AA pruning |

Backward alias: `FrameSourceMethod = EndpointSource`

#### `EndpointSourceConfig`
| Factory | Source | Notes |
|---|---|---|
| `ifk()` | IFK | Optionally set `ifk_config.aa_crossover` |
| `crit_sampling()` | CritSample | Uses `CriticalSamplingConfig::full_fast()` |
| `analytical()` | Analytical | Uses default `AnalyticalCriticalConfig` |
| `gcpc(cache)` | GCPC | Requires `GcpcCache*` |

Backward alias: `FrameSourceConfig = EndpointSourceConfig`, `analytical_critical() = analytical()`

#### `EndpointAABBResult`
| Member | Type | Description |
|---|---|---|
| `endpoint_aabbs` | `vector<float>` | `[n_endpoints × 6]` — per-endpoint position interval AABBs (geometry only, no radius) |
| `n_endpoints` | `int` | Number of endpoints (= n_joints + has_tool) |
| `n_active` | `int` | Number of active links |
| `fk_state` | `FKState` | Interval FK state (always populated) |
| `n_evaluations` | `int` | Total FK evaluations |
| `total_boxes()` | `int` | `n_endpoints` |
| `has_fk_state()` | `bool` | `fk_state.valid` |

Backward alias: `FrameSourceResult = EndpointAABBResult`

#### Core API
| Function | Description |
|---|---|
| `compute_endpoint_aabb(config, robot, intervals)` | Stage 1: compute endpoint AABBs `[n_endpoints × 6]` |
| `compute_endpoint_aabb_incremental(config, parent_fk, robot, intervals, dim)` | Incremental (prefix reuse) |
| `extract_link_aabbs_from_endpoint(result, robot, out)` | Derive per-link AABB from endpoints `[n_active × 6]` |
| `fk_to_endpoints(fk, robot, out)` | Extract position intervals from FKState |

Backward aliases: `compute_frame_source()`, `compute_frame_source_incremental()`, `extract_link_aabbs_from_result()`

### Stage 2: Envelope Type (`envelope_type.h`)

#### `EnvelopeType` (enum)
| Value | Name | Description |
|---|---|---|
| `0` | `SubAABB` | Per-link subdivided AABBs (n_sub=1 → single AABB per link) |
| `1` | `SubAABB_Grid` | Sub-AABBs rasterised into byte grid |
| `2` | `Hull16_Grid` | Sparse VoxelGrid via `fill_hull16()` per link (Conv(B₁∪B₂)⊕Ball) |

#### `EnvelopeTypeConfig`
| Factory | Type | Key params |
|---|---|---|
| `sub_aabb()` | SubAABB | `n_sub` (default 1) |
| `aabb()` | SubAABB | Backward-compat alias for `sub_aabb()` |
| `sub_aabb_grid(R)` | SubAABB_Grid | `n_sub`, `grid_R` |
| `hull16_grid(delta)` | Hull16_Grid | `delta` (`n_sub` ignored) |

#### `LinkEnvelopeResult`
| Member | Type | Description |
|---|---|---|
| `valid` | `bool` | Computation succeeded |
| `volume` | `double` | Occupied volume (m³) |
| `n_voxels` | `int` | Count of occupied voxels/sub-AABBs |
| `n_bricks` | `int` | Brick count (Hull16_Grid only) |
| `sub_aabbs` | `vector<float>` | Per-link AABBs (SubAABB) |
| `grid` | `vector<uint8_t>` | Byte grid (SubAABB_Grid) |
| `voxel_grid` | `VoxelGrid` | Sparse voxel grid (Hull16_Grid) |

#### Core API
| Function | Description |
|---|---|
| `compute_link_envelope(config, robot, ep_result)` | Stage 2: endpoint_aabbs → link envelope |
| `default_envelope_config(source, env_type)` | Get default config for source × envelope combination |

Backward alias: `compute_envelope_repr()`

### SBFConfig Factories (`forest/sbf.h`)

12 static factory methods for all source × envelope combinations
(+ 4 backward-compatible `*_aabb()` aliases → SubAABB with n_sub=1):

| IFK | CritSample | Analytical | GCPC |
|---|---|---|---|
| `ifk_sub_aabb()` | `crit_sub_aabb()` | `analytical_sub_aabb()` | `gcpc_sub_aabb(cache)` |
| `ifk_sub_aabb_grid()` | `crit_sub_aabb_grid()` | `analytical_sub_aabb_grid()` | `gcpc_sub_aabb_grid(cache)` |
| `ifk_hull16_grid()` | `crit_hull16_grid()` | `analytical_hull16_grid()` | `gcpc_hull16_grid(cache)` |

---

### AABB Derivation (`envelope_derive.h`)
| Function | Description |
|---|---|
| `derive_aabb(frames, ..., out)` | AABB from cached joint-origin frames |
| `derive_aabb_subdivided(frames, ..., n_sub, ..., out)` | Link-subdivided AABBs |
| `derive_grid(frames, ..., R, ..., out)` | Voxel grid envelope from frames |

### Critical-Point Envelope (`envelope_derive_critical.h`)
| Function | Description |
|---|---|
| `derive_crit_endpoints(robot, intervals, out)` | Critical-point endpoint sampling |
| `derive_aabb_critical(robot, intervals, n_sub, out)` | AABB via critical points |
| `derive_aabb_critical_enhanced(robot, intervals, n_sub, config, out)` | Enhanced with manifold+LBFGS |
| `derive_aabb_critical_analytical(robot, intervals, n_sub, config, out)` | Analytical Jacobian approach |

#### `AnalyticalCriticalConfig`
Configuration for the 5-phase analytical critical-point solver.

| Field | Type | Default | Description |
|---|---|---|---|
| `keep_kpi2_baseline` | `bool` | `true` | Phase 0: enumerate kπ/2 vertex configs |
| `enable_edge_solve` | `bool` | `true` | Phase 1: 1-DOF edge critical points |
| `enable_face_solve` | `bool` | `true` | Phase 2: 2-DOF face critical points (symbolic poly + interval solver) |
| `face_all_pairs` | `bool` | `false` | Phase 2: use all joint pairs (vs coupled+adjacent) |
| `enable_pair_1d` | `bool` | `true` | Phase 2.5a: pair-constrained 1D solver |
| `enable_pair_2d` | `bool` | `true` | Phase 2.5b: pair-constrained 2D solver |
| `pair_max_bg` | `int` | `25` | Phase 2.5: max background configs per pair |
| `pair_kpi2_backgrounds` | `bool` | `true` | Phase 2.5: use kπ/2 lattice backgrounds |
| `pair_all_pairs` | `bool` | `false` | Phase 2.5: use all joint pairs |
| `enable_interior_solve` | `bool` | `true` | Phase 3: coordinate-descent interior sweep |
| `interior_max_free` | `int` | `2` | Phase 3: max simultaneously free joints |
| `interior_max_sweeps` | `int` | `3` | Phase 3: max full sweep iterations |
| `improved_interior` | `bool` | `true` | Phase 3: use enhanced multi-start sweeps |
| `interior_n_restarts` | `int` | `4` | Phase 3: extra starting configs |
| `interior_pair_coupling` | `bool` | `true` | Phase 3: couple paired joints during sweep |
| `dual_phase3` | `bool` | `false` | Fused dual-Phase-3 mode (see below) |
| `certified` | `bool` | `true` | Interval-FK certification mode (see below) |

**`dual_phase3`**: When enabled, Phase 3 runs twice within a single analytical call:
1. From the full Phase 0+1+2+2.5 accumulated state (standard path).
2. From a Phase 0+1 checkpoint (bypasses Phase 2 "basin shift").

Results are merged via per-dimension min(lo)/max(hi). This eliminates the need for separate no-Phase-2 passes, removing 2× redundant Phase 0/1 computation. Achieves zero-miss AABB at ~43 ms avg (15× speedup over multi-pass approach). See Exp-28.

**`certified`**: When enabled, after the analytical solver writes the AABB, an interval-FK outer bound is computed and each face is clamped to be at least as extreme as the interval-FK result. This guarantees the output AABB ⊇ true swept volume unconditionally (Theorem 3, Certified Soundness). The overhead is one `compute_fk_full()` call (~5 μs) on top of ~43 ms analytical — less than 1% relative cost.

#### `AnalyticalCriticalStats`

| Field | Type | Description |
|---|---|---|
| `n_phase0_vertices` | `int` | Phase 0 boundary vertex evaluations |
| `n_phase1_edges` | `int` | Phase 1: 1D edge critical points found |
| `n_phase2_faces` | `int` | Phase 2: 2D face critical points found |
| `n_phase25a_pair1d` | `int` | Phase 2.5a pair-constrained 1D points |
| `n_phase25b_pair2d` | `int` | Phase 2.5b pair-constrained 2D points |
| `n_phase3_interior` | `int` | Phase 3+: interior critical points found |
| `n_phase1_fk_calls` | `int` | FK calls during Phase 1 |
| `n_phase2_fk_calls` | `int` | FK calls during Phase 2 |
| `n_phase25_fk_calls` | `int` | FK calls during Phase 2.5 |
| `n_phase3_fk_calls` | `int` | FK calls during Phase 3+ |
| `certified_max_gap` | `float` | Max per-face gap \|ifk − analytical\| (certified mode only) |
| `certified_gap_faces` | `int` | Number of faces where ifk ≠ analytical (certified mode only) |

### GCPC Cache (`envelope/gcpc_cache.h` / `src/envelope/gcpc_cache.cpp`)

**Global Critical Point Cache (GCPC)** — Pre-computed interior critical points of FK position stored in per-link KD-trees. At query time a 7-phase "cache-first + AA prune" pipeline produces tight, sound AABBs that match or exceed the Analytical solver's quality while enabling aggressive early-exit via Affine-Arithmetic bounds.

#### Symmetry Reductions

| Reduction | Effect |
|---|---|
| q₀ elimination | q₀ reconstructed via `atan2(B, A)` at query time; never stored |
| q₁ period-π | Only q₁ ∈ [0, π] stored; reflected copy tested at query time |
| q₆ skip | When d₆ = a₆ = 0 and no tool, q₆ is omitted |

#### `GcpcPoint`
One cached interior critical point.

| Member | Type | Description |
|---|---|---|
| `q_eff[7]` | `double[7]` | Joint configuration (q₁ … qₖ, q₁ ∈ [0,π]). q₀ NOT stored. |
| `n_eff` | `int` | Number of effective joints used |
| `link_id` | `int` | 0-based frame index |
| `direction` | `int` | 0 = xy (radial R²), 1 = z |
| `A`, `B` | `double` | Local FK position x, y (post-T₀ frame) |
| `C` | `double` | Local FK position z |
| `R` | `double` | Precomputed radial distance √(A² + B²) |

#### `GcpcLinkSection`
Per-link cache section with KD-tree for range queries.

| Member | Type | Description |
|---|---|---|
| `link_id` | `int` | 0-based frame index |
| `n_eff_joints` | `int` | Effective DOF (after q₀ elimination) |
| `n_points` | `int` | Number of cached critical points |
| `q6_skipped` | `bool` | Whether q₆ was skipped |
| `points` | `vector<GcpcPoint>` | All cached points for this link |
| `kd_tree` | `vector<KdNode>` | KD-tree nodes |
| `kd_root` | `int` | Root node index |

#### `GcpcQueryResult`
A single critical point matched by a range query.

| Member | Type | Description |
|---|---|---|
| `point` | `const GcpcPoint*` | Pointer to cached point |
| `q0_optimal` | `double` | Reconstructed q₀ |
| `q1_reflected` | `bool` | Whether q₁ was reflected (q₁ − π) |
| `q1_actual` | `double` | Actual q₁ used (after reflection) |

#### `GcpcQueryStats`
Full per-phase telemetry including AA pruning counters.

| Field | Type | Description |
|---|---|---|
| `n_cache_matches` | `int` | Phase A: points found in cache |
| `n_q1_reflected` | `int` | Points from q₁ reflection |
| `n_q0_valid` | `int` | Points with valid q₀ in range |
| `n_boundary_kpi2` | `int` | Phase B: kπ/2 vertex evaluations |
| `n_boundary_atan2` | `int` | Phase C: 1D atan2 solutions |
| `n_fk_calls` | `int` | Total FK evaluations |
| `n_aa_prune_checks` | `int` | AA faces checked (Level 2) |
| `n_aa_pruned` | `int` | Faces pruned by AA bound |
| `n_aa_not_pruned` | `int` | Faces surviving AA pruning |
| `n_phase_d_faces` | `int` | Phase D: 2D face solutions |
| `n_phase_d_fk` | `int` | Phase D: FK calls |
| `n_phase_d_pruned` | `int` | Phase D: combos pruned by AA |
| `n_phase_e_pair1d` | `int` | Phase E: pair-1D solutions |
| `n_phase_e_fk` | `int` | Phase E: FK calls |
| `n_phase_e_pruned` | `int` | Phase E: pruned by AA |
| `n_phase_f_pair2d` | `int` | Phase F: pair-2D solutions |
| `n_phase_f_fk` | `int` | Phase F: FK calls |
| `n_phase_f_pruned` | `int` | Phase F: pruned by AA |
| `n_phase_g_interior` | `int` | Phase G: interior candidate evaluations |
| `n_phase_g_fk` | `int` | Phase G: FK calls |
| `n_phase_g_pruned` | `int` | Phase G: links pruned by AA |
| `n_interior_added` | `int` | Enrichment: new points added |

#### `GcpcCache` Class

| Method | Returns | Description |
|---|---|---|
| `load_json(path, robot)` | `bool` | Load from JSON (nlohmann/json), build KD-trees |
| `load(path)` | `bool` | Load from binary GCPC cache file |
| `save(path)` | `bool` | Save to binary GCPC cache file |
| `build(robot, points)` | `void` | Build cache from a set of critical points |
| `enrich_with_interior_search(robot, n_random=500, max_sweeps=5)` | `int` | Multi-start coordinate descent enrichment (see below). Returns count of new unique points added. |
| `query_link(link_id, intervals, results)` | `void` | KD-tree range query → matching `GcpcQueryResult` vector |
| `derive_aabb_with_gcpc(robot, intervals, n_sub, out, stats)` | `void` | Full Phase A–G pipeline → tight AABB (see below) |
| `is_loaded()` | `bool` | Cache non-empty |
| `n_links()` | `int` | Number of link sections |
| `n_total_points()` | `int` | Total cached critical points |
| `find_section(link_id)` | `const GcpcLinkSection*` | Section for given link (or `nullptr`) |
| `d0()` | `double` | Robot base d₀ (for z reconstruction) |

#### Phase A–G Pipeline (`derive_aabb_with_gcpc`)

The pipeline evaluates critical-point candidates in order of increasing cost, using AA bounds to prune expensive later phases:

| Phase | Name | Description |
|---|---|---|
| **A** | Cache lookup | KD-tree range query + q₀ reconstruction + q₁ reflection. Establishes strong pre-bounds from cached interior critical points. |
| **B** | kπ/2 vertices | Enumerate all boundary vertices where each joint is at kπ/2 within the query interval. |
| **C** | 1D edge atan2 | Solve ∂R/∂qⱼ = 0 via 3-point trig fit + atan2. **Two-level AA pruning**: Level 1 (full box) skips entire links, Level 2 (per face) skips individual AABB faces. |
| **D** | 2D face polynomial | Solve ∂R/∂qⱼ = ∂R/∂qₖ = 0 via symbolic degree-8 polynomial + interval-restricted bisection. AA-pruned at link level. |
| **E** | Pair-constrained 1D | Fix qᵢ + qⱼ = kπ/2, solve 1D atan2. AA-pruned. |
| **F** | Pair-constrained 2D | Fix qᵢ + qⱼ = kπ/2, sweep over extra free joint. AA-pruned. |
| **G** | Dual interior descent | Coordinate descent seeded from `best_face_config` (best config per AABB face). Run twice: (1) from Phase A–F final state, (2) from Phase A+B checkpoint (avoids basin shift from Phases C–F). Results merged per-dimension min/max. |

**Dual Phase G** mirrors the Analytical solver's `dual_phase3` mechanism: running from two different starting points eliminates the "basin shift" caused by Phase C–F accumulation, ensuring GCPC ≥ Analytical at all interval widths.

#### Cache Enrichment (`enrich_with_interior_search`)

Pre-computes interior critical points of R(q₁,…,qₖ) and pz(q₁,…,qₖ) across the full joint range, adding them to the KD-tree so Phase A gives tight pre-bounds.

**Seeds:**
- kπ/2 grid vertices over the full joint range
- `n_random_seeds` (default 500) uniform random configurations
- Pair-constrained seeds (qᵢ + qⱼ = kπ/2)

**Algorithm:** Multi-start coordinate descent (`max_sweeps` iterations) with deduplication (tolerance 1e-6).

**Typical result:** 80–150 unique interior critical points per link (7-DOF IIWA14), enabling Phase A to recover ≥99.9% of the final AABB volume before any expensive boundary solve.

### Collision Policy (`collision_policy.h`)
| Function | Description |
|---|---|
| `check_collision(policy, frames, ...)` | Unified dispatch by `CollisionPolicy` enum |
| `collide_aabb(frames, ..., obs, n_obs)` | Early-exit AABB collision |
| `collide_aabb_subdiv(frames, ..., n_sub, ...)` | Subdivided AABB collision |
| `collide_grid(frames, ..., R, ...)` | Grid voxel collision |

### FrameStore (`frame_store.h`)
Persistent store for joint-origin interval vectors (the LECT data).
| Method | Description |
|---|---|
| `store_from_fk(idx, fk)` | Store FK state for node |
| `get_frames(idx)` | Retrieve cached frames |
| `union_frames(dst, a, b)` | Merge two nodes' frames |
| `save(path)` / `load(path)` | File persistence |
| `create_mmap(path, cap)` / `load_mmap(path)` | Memory-mapped persistence |

### GridStore (`grid_store.h`)
Bitfield grid store (32x32x32 = 32768 voxels, 512 uint64 words per node).
| Method | Description |
|---|---|
| `derive_from_frames(idx, frames, n_sub)` | Build grid from cached frames |
| `check_collision(idx, obs, n_obs)` | Grid-based collision check |
| `union_grids(dst, a, b)` | Bitwise OR merge |
| `save()` / `load()` / mmap variants | Persistence |

### GridEnvelope / GridComputer (`grid_envelope.h`)
`IEnvelope` implementation using voxel grids. `GridComputer` is the corresponding `IEnvelopeComputer`.

### EnvelopeComputer (`envelope_computer.h`) *(legacy)*
`IntervalFKEnvelopeComputer`  wraps Robot + interval-FK for AABB envelope computation.

---

## 4. Scene (`include/sbf/scene/`)

### `ICollisionChecker` (interface)
| Method | Description |
|---|---|
| `check_config(q)` | Point collision check |
| `check_box(intervals)` | Box collision check |
| `check_segment(q1, q2, step)` | Segment collision check |

### `AabbCollisionChecker`
Implements `ICollisionChecker` using AABB envelopes + obstacle overlap.
Alias: `CollisionChecker = AabbCollisionChecker`.

| Method | Returns | Description |
|---|---|---|
| `set_aa_crossover(w)` | `void` | Set hybrid IA/AA crossover width for `check_box()` (0 = disabled) |
| `aa_crossover()` | `double` | Current crossover width |

---

## 5. Voxel (`include/sbf/voxel/`)

### BitBrick (`bit_brick.h`)
8×8×8 = 512-voxel tile packed into 8 × `uint64_t` (64 bytes = 1 cache line).
Layout: `word[z]`, bit = `y * 8 + x`.

| Method | Returns | Description |
|---|---|---|
| `set(x, y, z)` | `void` | Set voxel at local coord |
| `test(x, y, z)` | `bool` | Test voxel at local coord |
| `clear_voxel(x, y, z)` | `void` | Clear single voxel |
| `clear()` | `void` | Zero all words |
| `is_empty()` | `bool` | All words zero? |
| `popcount()` | `int` | Count set voxels (`__popcnt64` on MSVC) |
| `operator\|` / `operator\|=` | `BitBrick` | Bitwise OR (merge) |
| `operator&` | `BitBrick` | Bitwise AND |
| `intersects(other)` | `bool` | Any overlapping voxel? (early-exit AND) |

### BrickCoord / BrickCoordHash (`bit_brick.h`)
| Member | Type | Description |
|---|---|---|
| `bx`, `by`, `bz` | `int` | Integer tile address |
| `BrickCoordHash` | functor | FNV-1a style hash for use in `unordered_map` |

### VoxelGrid (`voxel_grid.h`)
Sparse voxel grid backed by `unordered_map<BrickCoord, BitBrick>`.

**Constructor:**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `delta` | `double` | — | Voxel edge length in metres (e.g. 0.02) |
| `ox`, `oy`, `oz` | `double` | `0.0` | World-space origin |

Safety padding: `√3 · Δ / 2` (half-diagonal of voxel cube), computed automatically.

**Coordinate Helpers:**

| Method | Returns | Description |
|---|---|---|
| `to_cell(w, axis)` | `int` | World position → integer cell index |
| `cell_center(c, axis)` | `double` | Cell index → world position of cell centre |

**Rasterisation:**

| Method | Description |
|---|---|
| `fill_aabb(float[6])` | Fill all voxels overlapping an axis-aligned box |
| `fill_hull16(prox_iv, dist_iv, radius)` | Turbo scanline rasterisation of Conv(B₁∪B₂)⊕Ball(r). O(ny·nz), no per-voxel distance evals. |
| `find_t_range_yz(qy, qz, r², ...)` | (static) Solve piecewise-quadratic for t-range where dist²_yz ≤ r² |

**Volume Queries:**

| Method | Returns | Description |
|---|---|---|
| `count_occupied()` | `int` | Total set voxels (popcount across all bricks) |
| `occupied_volume()` | `double` | Volume in m³ (`count_occupied * Δ³`) |
| `num_bricks()` | `int` | Number of allocated bricks |

**Merge / Collision:**

| Method | Returns | Description |
|---|---|---|
| `merge(other)` | `void` | Bitwise OR of all bricks |
| `collides(other)` | `bool` | Early-exit AND over smaller map |
| `count_colliding(other)` | `int` | Number of overlapping voxels |
| `clear()` | `void` | Remove all bricks |

**Accessors:** `delta()`, `safety_pad()`, `bricks()`.

### Hull Rasteriser (`hull_rasteriser.h` / `src/voxel/hull_rasteriser.cpp`)
High-level API bridging Robot/FKState with VoxelGrid.

| Type/Function | Description |
|---|---|
| `PosInterval` | `float lo[3], hi[3]` — position interval from prefix TF |
| `frame_pos(fk, frame_idx)` | Extract (x,y,z) position interval from FKState columns [3],[7],[11] |
| `rasterise_robot_hull16(robot, fk, grid, n_sub=8)` | Rasterise all active-link envelopes via Hull-16 turbo scanline |
| `rasterise_robot_sub_aabbs(robot, fk, grid, n_sub=8)` | Rasterise via per-link sub-AABB decomposition |
| `rasterise_robot_aabbs(robot, fk, grid)` | Rasterise full per-link AABBs (no subdivision) |
| `rasterise_box_obstacle(cx, cy, cz, hx, hy, hz, grid)` | Rasterise a box obstacle |

---

## 6. Forest (`include/sbf/forest/`)

### Overview

The forest module builds collision-free C-space box forests. Two builder classes:
- **`SafeBoxForest`**: Random sampling → FFB → promotion (existing)
- **`ForestGrower`**: Multi-root directed expansion with wavefront / RRT modes (new)

### `GrowerConfig` (`grower_config.h`)

| Field | Type | Default | Description |
|---|---|---|---|
| `n_roots` | `int` | `2` | Number of root boxes (FPS selection) |
| `mode` | `Mode` | `Wavefront` | `Wavefront` (BFS boundary) or `RRT` (random extend) |
| `max_boxes` | `int` | `500` | Maximum number of boxes to grow |
| `min_edge` | `double` | `0.01` | FFB minimum edge length |
| `max_depth` | `int` | `30` | FFB maximum KD-tree depth |
| `max_consecutive_miss` | `int` | `200` | Stop after N consecutive FFB failures |
| `timeout` | `double` | `60.0` | Wall-clock timeout (seconds) |
| `adjacency_tol` | `double` | `1e-10` | Tolerance for face-adjacency test |
| `rng_seed` | `uint64_t` | `42` | Random number generator seed |
| `n_boundary_samples` | `int` | `6` | Boundary seeds per box face (wavefront) |
| `boundary_epsilon` | `double` | `0.01` | Offset distance beyond box face (wavefront) |
| `goal_face_bias` | `double` | `0.6` | Probability of choosing goal-facing face (wavefront) |
| `rrt_step_ratio` | `double` | `0.3` | Step = ratio × max joint range width (RRT) |
| `rrt_goal_bias` | `double` | `0.1` | Probability of sampling bias target (per-tree: start→goal, goal→start, rand→50/50) |
| `n_threads` | `int` | `1` | Worker thread count (`1` = serial, `>1` = thread-pool parallel) |
| `adaptive_min_edge` | `bool` | `false` | Enable two-phase coarse→fine min_edge strategy |
| `coarse_min_edge` | `double` | `0.1` | Min edge in coarse phase (when `adaptive_min_edge` enabled) |
| `coarse_fraction` | `double` | `0.6` | Fraction of budget to use coarse min_edge (0.0–1.0) |
| `bridge_samples` | `int` | `0` | Seeds per shared face for boundary bridging (`0` = disabled) |
| `coarsen_enabled` | `bool` | `false` | Enable greedy coarsening after expansion (hull-merge adjacent pairs) |
| `coarsen_target_boxes` | `int` | `0` | Stop coarsening when n_boxes ≤ target (`0` = reduce as much as possible) |
| `max_coarsen_rounds` | `int` | `50` | Maximum greedy coarsening rounds per grow() call |
| `coarsen_score_threshold` | `double` | `50.0` | Skip merge candidates with hull_vol/sum_vol > threshold |
| `warm_start_depth` | `int` | `0` | Pre-expand LECT skeleton depth before spawning workers. `0` = auto (depth 3), `-1` = disabled |
| `pipeline` | `PipelineConfig` | `recommended()` | LECT envelope pipeline |

### `AdjacencyGraph` (`adjacency_graph.h` / `adjacency_graph.cpp`)

Sweep-and-prune accelerated box adjacency graph. Ported from v2 `deoverlap.cpp`.

**Acceleration techniques:**
1. Dimension ranking by fill ratio → best sweep/filter dims
2. Sweep-and-prune with sorted break on sweep dimension
3. Dual-dimension pre-filter before full-dim check
4. Row-major flat layout for cache-friendly access
5. Narrow→wide dimension ordering for early exit

| Method | Returns | Description |
|---|---|---|
| `AdjacencyGraph(limits, tol)` | — | Constructor with joint limits |
| `add_box(box)` | `void` | Incremental insert: O(log N + K) |
| `remove_box(id)` | `void` | Remove box and all its edges |
| `clear()` | `void` | Remove all boxes |
| `rebuild(boxes)` | `void` | Full batch rebuild: O(N·K) |
| `neighbors(id)` | `const vector<int>&` | Adjacent box IDs |
| `connected(a, b)` | `bool` | BFS reachability test |
| `n_components()` | `int` | Number of connected components |
| `components()` | `vector<vector<int>>` | All connected components |
| `find_nearest_box(q)` | `int` | Box with center nearest to config. Uses internal KD-tree with subtree AABB pruning — O(log N) average, lazy rebuild on mutation |
| `n_boxes()` | `int` | Number of boxes in graph |
| `n_edges()` | `int` | Number of adjacency edges |
| `sweep_dim()` | `int` | Current sweep dimension |
| `filter_dim()` | `int` | Current filter dimension |

### `ThreadPool` (`thread_pool.h`, header-only)

Lightweight C++17 thread pool with `std::future`-based task submission.

| Method | Returns | Description |
|---|---|---|
| `ThreadPool(n_threads)` | — | Create pool with `n_threads` workers |
| `submit(f, args...)` | `std::future<R>` | Submit callable for async execution; returns future |
| `size()` | `int` | Number of worker threads |
| `~ThreadPool()` | — | Joins all workers; drains remaining tasks |

Non-copyable, non-moveable. Used internally by `ForestGrower::grow_parallel()`.

### `ForestGrower` (`forest_grower.h` / `forest_grower.cpp`)

Multi-root forest growing engine with two expansion modes and optional thread-pool parallelism.

**Serial pipeline:** root selection → subtree partitioning → expansion (wavefront/RRT) → promotion → final adjacency rebuild.

**Parallel pipeline** (`n_threads > 1`): root selection → subtree partitioning → **spawn per-subtree workers** (each with independent LECT + RNG) → **merge boxes** → optional boundary bridging → final adjacency rebuild.

| Method | Returns | Description |
|---|---|---|
| `ForestGrower(robot, config)` | — | Constructor |
| `ForestGrower(robot, config, warm_lect)` | — | Warm-start constructor: moves in a pre-built LECT skeleton (used by parallel workers) |
| `set_endpoints(start, goal)` | `void` | Force start/goal as root 0 and root 1 |
| `get_bias_target(root_id)` | `const VectorXd*` | Per-tree bias target: root 0→goal, root 1→start, root≥2→random 50/50. Returns `nullptr` if no endpoints. |
| `sample_near_existing_boundary(root_id)` | `VectorXd` | Sample a seed on a random face of a random existing box (optionally filtered by root_id). Returns empty on failure. |
| `rrt_snap_to_face(nearest, direction, step)` | `SnapResult` | Find best face of `nearest` box aligned with `direction`, place seed there (70% directional + 30% random). Returns `{seed, face_dim, face_side}`. |
| `grow(obstacles, n_obs)` | `GrowerResult` | Main entry: dispatches serial or parallel based on `n_threads` |
| `grow_subtree(seed, root_id, limits, obs, n_obs, shared_box_count)` | `GrowerResult` | Grow a single subtree in the given C-space sub-region. Optional `shared_box_count` enables work-stealing budget. |
| `lect()` | `const LECT&` | Access underlying LECT tree |
| `lect_mut()` | `LECT&` | Mutable access to LECT tree |
| `graph()` | `const AdjacencyGraph&` | Access adjacency graph |
| `boxes()` | `const vector<BoxNode>&` | All boxes in the forest |
| `config()` | `const GrowerConfig&` | Configuration |
| `n_boxes()` | `int` | Number of boxes |
| `is_grid_envelope()` | `bool` | Whether envelope type uses voxel grids |

**Root selection:**
- Without endpoints: Farthest Point Sampling (FPS) with K=50 candidates
- With endpoints: start/goal forced as root 0/1, remaining roots via FPS along start→goal line

**Subtree partitioning:**
After root selection, the C-space is recursively bisected along cycling dimensions until each cell contains at most one root. Per-root sub-regions are stored and used to constrain random sampling in wavefront fallback and RRT target selection.

**Wavefront expansion:**
BFS queue-based boundary expansion. For each box, generates boundary seeds on valid faces (excluding the incoming face). Face selection uses **per-tree bias** via `get_bias_target(root_id)`. When queue is empty, uses **boundary-aware fallback** via `sample_near_existing_boundary()` to keep new boxes adjacent to the tree (avoids isolated fragments). In adaptive coarse phase, tries up to 30 boundary refill samples before force-transitioning to fine phase.

**RRT expansion:**
Samples random targets (subtree-constrained), finds nearest box, then uses **boundary-snap** via `rrt_snap_to_face()`: places the seed on the best-aligned face of the nearest box (70% directional + 30% random jitter). This ensures new boxes are always adjacent to the tree, eliminating isolated small boxes. With probability `rrt_goal_bias`, a random box's tree determines the per-tree bias target. Step size = `rrt_step_ratio × max_joint_range_width`.

**Promotion:**
Iterative bottom-up leaf merging (same as `SafeBoxForest`). Dispatches AABB or hull-grid collision check based on envelope type.

**Parallel expansion** (`n_threads > 1`):
When `config.n_threads > 1` and there are ≥ 2 subtrees, `grow()` dispatches to `grow_parallel()`. Each subtree is assigned to a worker thread with:
- **Independent ForestGrower instance** (own LECT, own RNG, own boxes vector) — zero shared mutable state
- **Work-stealing budget**: A shared `std::atomic<int>` counter tracks global box count. Workers call `fetch_add(1)` and stop when `total >= max_boxes` — dynamic load balancing instead of static per-worker budget
- **O(1) box lookup**: `box_id_to_idx_` hashmap replaces linear search in expansion loops
- **Deterministic RNG**: worker `i` gets seed `rng_seed + i * 12345 + 1`

After all workers complete, the main thread:
1. Merges all worker boxes with globally-reassigned IDs
2. Remaps `parent_box_id` references
3. Optionally runs **boundary bridging** (`bridge_samples > 0`): samples seeds near shared faces between subtree regions and creates bridge boxes
4. Identifies start/goal boxes by containment check
5. Rebuilds the final adjacency graph over all merged boxes

**LECT warm-start** (`warm_start_depth >= 0`):
Before spawning parallel workers, the coordinator pre-expands the LECT tree to the specified depth (default auto = 3). Each worker receives a snapshot (deep copy with cleared occupation). Avoids redundant FK + envelope computation in the first few tree levels. Measured 7→15 pre-built nodes for depth 3.

**Boundary bridging** (`bridge_samples > 0`):
After parallel merge, samples seeds near shared faces between subtree partition regions. Creates bridge boxes with alternating `root_id` to establish cross-subtree adjacency. Includes `sync_lect_occupation()` to prevent overlapping boxes.

**Adaptive min_edge** (`adaptive_min_edge = true`):
Two-phase expansion strategy. In phase 1 (first `coarse_fraction` of budget), uses `coarse_min_edge` (default 0.1) for rapid coverage with larger boxes. At the transition point:
- `miss_count` is reset to zero (fine min_edge opens many new FFB sites)
- Wavefront queue is re-populated with all existing boxes (re-explore boundaries with fine min_edge)
- RRT continues seamlessly with smaller step tolerance

Measured 2.3× total volume improvement (80 boxes, same seed). Both serial and parallel modes supported; parallel workers aggregate coarse/fine counts.

**Benchmark** (7-DOF iiwa14, 4 roots, 200 boxes, 4 threads):
- Phase 4: serial 169 ms → parallel 60 ms = 2.83× speedup
- **Phase 5: serial 165 ms → parallel 48 ms = 3.41× speedup** (work-stealing + O(1) lookup + adaptive min_edge)

### `GrowerResult`

| Field | Type | Description |
|---|---|---|
| `boxes` | `vector<BoxNode>` | All boxes in the forest |
| `n_roots` | `int` | Number of root boxes |
| `n_boxes_total` | `int` | Total box count |
| `n_ffb_success` | `int` | Successful FFB calls |
| `n_ffb_fail` | `int` | Failed FFB calls |
| `n_promotions` | `int` | Number of leaf promotions |
| `n_bridge_boxes` | `int` | Bridge boxes created at subtree boundaries |
| `n_coarse_boxes` | `int` | Boxes created during coarse phase (adaptive min_edge) |
| `n_fine_boxes` | `int` | Boxes created during fine phase (adaptive min_edge) |
| `n_coarsen_merges` | `int` | Greedy coarsening merges performed |
| `n_components` | `int` | Connected components in adjacency graph |
| `start_goal_connected` | `bool` | Whether start and goal boxes are in same component |
| `total_volume` | `double` | Sum of all box volumes |
| `build_time_ms` | `double` | Wall-clock build time |
| `phase_times` | `map<string,double>` | Per-phase timing (root_select_ms, expand_ms, adj_rebuild_ms, warm_start_ms, coarsen_ms) |

---

## 7. Visualization (`include/sbf/viz/` + `python/sbf_viz/`)

Architecture: **C++ JSON export → Python Plotly interactive HTML**.

### C++ JSON Exporter (`viz_exporter.h` / `src/viz/viz_exporter.cpp`)

| Function | Description |
|---|---|
| `export_robot_json(robot, configs, path)` | Export robot FK link positions for multiple configurations |
| `export_robot_json(robot, config, path)` | Single-configuration convenience overload |
| `export_envelope_json(robot, store, node_indices, n_sub, path)` | Export per-node AABB link envelopes from FrameStore |
| `export_envelope_from_boxes_json(robot, boxes, n_sub, path)` | Export envelopes for C-space boxes (computes FK internally) |
| `export_voxel_json(grid, path)` | Export VoxelGrid as brick-level hex data |
| `export_voxel_centres_json(grid, path)` | Export VoxelGrid with explicit occupied cell centres |
| `export_scene_json(scene, path)` | Export scene obstacles |
| `export_snapshot_json(robot, config, box, robot_grid, obs_grid, scene, n_sub, path)` | Combined snapshot with all layers |

### Python Visualization Package (`python/sbf_viz/`)

| Module | Key Function | Description |
|---|---|---|
| `load_data` | `load_robot_json()`, etc. | JSON loaders returning typed dataclasses |
| `robot_viz` | `plot_robot_3d(data)` | 3D arm traces + joint markers for multi-config overlay |
| `envelope_viz` | `plot_envelope_3d(data)` | AABB mesh + wireframe per-link coloring |
| `voxel_viz` | `plot_voxel_3d(data, mode)` | Scatter or cube-mesh voxel rendering |
| `scene_viz` | `plot_scene_3d(data)` | Obstacle mesh + wireframe |
| `combined_viz` | `plot_snapshot_3d(data)` | Unified multi-layer viewer with toggle buttons |
| `run_demo` | CLI | Reads 6 JSON files → generates 6 HTML files |

**Usage (CLI):**
```bash
# 1. C++ exports JSON
viz_demo <robot.json> <output_dir>

# 2. Python renders HTML
cd python && python -m sbf_viz.run_demo <output_dir>
```

---

## 8. 2D ForestGrower Visualization (`src/viz/`)

Pure Python 2D visualization package that simulates and animates ForestGrower algorithms on a 2D C-space with random box obstacles. Useful for algorithm understanding, debugging, and paper figures.

**Architecture:** `GrowVizConfig` → `ForestGrower2D` → snapshots → `render` → PNG frames → GIF

**Key features (2026-03-17):**
- RRT boundary-snap: new boxes snap to nearest box's boundary face, avoiding scattered small boxes
- Full-space FPS root selection: roots distributed uniformly across entire C-space
- Adaptive two-phase (coarse→fine): large boxes fill space first, then fine boxes fill gaps
- `PatchCollection` + `LineCollection` accelerated rendering (~2× speedup)
- Default `snapshot_every=1` (every box recorded), CLI `--snap-every 2`

### `GrowVizConfig` (`src/viz/core.py`)

Configuration dataclass aligning with C++ `GrowerConfig` parameters.

| Field | Type | Default | Description |
|---|---|---|---|
| `seed` | `int` | `42` | Random seed |
| `q_lo`, `q_hi` | `(float, float)` | `(-π, -π)`, `(π, π)` | C-space bounds |
| `q_start`, `q_goal` | `list[float]` or `None` | set | Start/goal configs |
| `n_obstacles` | `int` | `10` | Random obstacles count |
| `mode` | `str` | `"wavefront"` | `"wavefront"` or `"rrt"` |
| `n_roots` | `int` | `3` | Number of root trees |
| `max_boxes` | `int` | `300` | Total box budget |
| `max_consecutive_miss` | `int` | `150` | Miss count before stop |
| `min_edge` | `float` | `0.03` | FFB fine min edge |
| `max_depth` | `int` | `20` | FFB max KD-tree depth |
| `n_boundary_samples` | `int` | `6` | Seeds per box face |
| `boundary_epsilon` | `float` | `0.02` | Face offset distance |
| `goal_face_bias` | `float` | `0.6` | Goal-directed face priority |
| `rrt_step_ratio` | `float` | `0.15` | RRT step = ratio × range |
| `rrt_goal_bias` | `float` | `0.1` | RRT goal sampling probability |
| `adaptive_min_edge` | `bool` | `False` | Enable coarse→fine two-phase |
| `coarse_min_edge` | `float` | `0.25` | Phase 1 min edge (8× coarser) |
| `coarse_fraction` | `float` | `0.5` | Fraction of budget for coarse |
| `n_threads` | `int` | `1` | Simulated round-robin threads |
| `snapshot_every` | `int` | `1` | Frames per box (1 = every box) |
| `collision_resolution` | `float` | `0.025` | Collision map pixel size |
| `dpi` | `int` | `150` | PNG DPI |
| `gif_frame_ms` | `int` | `200` | GIF frame duration (ms) |
| `fig_size` | `(float, float)` | `(10, 8)` | Figure size (inches) |

### `BoxInfo` (`src/viz/core.py`)

| Field | Type | Description |
|---|---|---|
| `box_id` | `int` | Unique ID |
| `lo`, `hi` | `ndarray` | AABB bounds |
| `seed` | `ndarray` | Seed config |
| `parent_box_id` | `int` | Expansion parent (-1 = root) |
| `expand_face_dim` | `int` | Face dimension used |
| `expand_face_side` | `int` | 0 = lo, 1 = hi |
| `root_id` | `int` | Root tree membership |
| `is_coarse` | `bool` | Created during coarse phase |
| `volume` | `float` | (property) Box volume |
| `center()` | `ndarray` | Box center |
| `is_adjacent(other)` | `bool` | Face-adjacency test |

### `FFBEngine` (`src/viz/ffb_engine.py`)

2D Find-Free-Box engine using KD-tree with adaptive `_effective_min_edge`.

| Method | Returns | Description |
|---|---|---|
| `FFBEngine(cspace, min_edge, max_depth)` | — | Constructor |
| `find_free_box(seed)` | `(lo, hi)` or `None` | Descend & split to find collision-free leaf |
| `mark_box_id(lo, hi, bid)` | `None` | Mark leaf as occupied |
| `set_effective_min_edge(me)` | `None` | Change min edge threshold (adaptive two-phase) |

### `ForestGrower2D` (`src/viz/forest_grower_2d.py`)

2D multi-root forest grower simulating the C++ `ForestGrower` algorithm.

| Method | Returns | Description |
|---|---|---|
| `ForestGrower2D(cspace, cfg)` | — | Constructor |
| `grow(snapshot_every=1)` | `list[dict]` | Run growth, return snapshots |

**Expansion modes:**

| Mode | Config | Description |
|---|---|---|
| Wavefront | `mode="wavefront"` | BFS boundary expansion with goal-face bias |
| RRT | `mode="rrt"` | Random target + nearest-box boundary-snap extension |
| Multi-thread | `n_threads>1` | Round-robin per-subtree simulation |
| Adaptive | `adaptive_min_edge=True` | Coarse phase → fine phase two-stage |

**Algorithm details:**

- **Root selection**: With endpoints → start/goal forced as root 0/1, remaining via full-space FPS (K=80 candidates). Without → pure FPS.
- **Subtree partitioning**: Recursive median bisection along cycling dimensions.
- **Wavefront BFS**: Queue-based boundary expansion. **Per-tree bias** via `_get_bias_target(root_id)` sorts faces by dot product with the tree-specific target. When queue empties: **boundary-aware fallback** via `_sample_near_existing_boundary()` keeps new boxes adjacent to tree. Coarse phase: 30 boundary refill attempts before force-transition.
- **RRT boundary-snap**: Via `_rrt_snap_to_face()`: finds best-aligned face of nearest box in extend direction. Places seed on face (70% directional + 30% random). With probability `rrt_goal_bias`, uses per-tree bias target. Falls back to pure extend when no valid face exists.
- **Adaptive two-phase**: Roots use fine `min_edge` (guaranteed). Then `coarse_min_edge=0.25` for first 50% of budget. When coarse queue empties, tries 30 random boundary fill attempts before force-transition. Fine phase re-queues all boxes for full boundary re-exploration.
- **Multi-thread round-robin**: Each subtree has independent BFS queue. Per round, each subtree expands 1 box. Global miss counter for termination.

**Snapshot format** (dict):

| Key | Type | Description |
|---|---|---|
| `n_boxes` | `int` | Box count at this frame |
| `boxes` | `dict[int, BoxInfo]` | Deep copy of all boxes |
| `adjacency` | `dict[int, set[int]]` | Deep copy of adjacency graph |
| `new_box_id` | `int` | ID of newest box (red highlight) |
| `in_coarse_phase` | `bool` | Currently in coarse phase |
| `n_coarse_boxes` | `int` | Coarse box count |
| `n_fine_boxes` | `int` | Fine box count |

### `build_random_scene(cfg, rng)` (`src/viz/forest_grower_2d.py`)

Generate random 2D C-space with obstacles. Guarantees: start/goal collision-free, direct line blocked.

### Render (`src/viz/render.py`)

| Function | Description |
|---|---|
| `_draw_boxes_on_ax(ax, snap, ...)` | Draw boxes + adjacency edges on matplotlib axis. Uses `PatchCollection` + `LineCollection` for performance. |
| `_build_title(snap, cfg, frame_idx)` | Build title string with mode/thread/adaptive phase info. |
| `plot_snapshot(snap, cmap, extent, cfg, idx, subtrees)` | Render single-frame snapshot → `Figure`. |
| `compose_gif(frames_dir, gif_path, duration_ms)` | Assemble PNG frames → animated GIF. Last frame holds 2s. |

**Colour scheme:**
- Per-root fixed colours from `ROOT_COLORS` (10 colours)
- Coarse boxes: dashed edge, 20% alpha
- Fine boxes: solid edge, 35% alpha
- Newest box: red (#ff0000), 55% alpha

### `plot_4panel(...)` (`src/viz/compare_expansion.py`)

| Parameter | Type | Description |
|---|---|---|
| `snaps` | `list[dict]` | 4 snapshot dicts (one per mode) |
| `cfgs` | `list[GrowVizConfig]` | 4 configs |
| `subtrees_list` | `list[list]` | Per-mode subtree partitions |
| `labels` | `list[str]` | Panel labels |
| `cmap_data` | `ndarray` | Collision map |
| `extent` | `list` | Plot extent |
| `frame_idx` | `int` | Frame index |
| `seed` | `int` | Random seed (for title) |

Returns `Figure` with 2×2 subplots comparing all 4 expansion modes.

### CLI Entry Points

```bash
# Single-mode growth animation
python -m src.viz.forest_grower_2d \
    --mode wavefront --seed 42 --max-boxes 300 --snap-every 2 \
    [--adaptive] [--n-threads 3] [--no-endpoints]

# 4-mode comparison animation (Wavefront / RRT / Multi-thread / Adaptive)
python -m src.viz.compare_expansion \
    --seed 42 --max-boxes 200 --snap-every 2 --dpi 80

# Multi-process visualization
python -m src.viz.multiprocess \
    --seed 42 --n-roots 3 --max-boxes 300

# Single wavefront step detail
python -m src.viz.wavefront_step \
    --seed 42 --pre-grow 20
```

**Output structure** (`results/viz_compare_<timestamp>/`):
```
├── compare_4panel.gif     — 2×2 animated comparison
├── final_compare.png      — Final frame static image
├── scene.json             — Obstacle geometry
├── summary.json           — Per-mode timing & box counts
├── README.md              — Auto-generated summary
└── frames/                — Individual PNG frames
    ├── frame_0000.png
    ├── frame_0001.png
    └── ...
```

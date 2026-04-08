# Paper <-> Code Traceability Map

## Section Mapping

| Paper Section | Subsections | Code Module | Headers |
|---|---|---|---|
| Sec III: Certified Envelope Computation | III-A Interval-FK, III-B Subdivision, III-C Critical-Point Solver (III-C-1 DH Trig Structure, III-C-2 Five-Phase Solver, III-C-3 Completeness Guarantees, III-C-4 Fused Dual-Phase-3, III-C-5 Interval-FK Certification), III-D LECT | `src/robot/`, `src/envelope/` | `robot/interval_fk.h`, `envelope/envelope_derive.h`, `envelope/envelope_derive_critical.h`, `envelope/frame_store.h` |
| Sec III-E: Affine-Arithmetic FK (Hybrid IA/AA) | III-E-1 Reduced AA FK, III-E-2 Taylor Linearisation, III-E-3 Hybrid Crossover Strategy | `src/robot/` | `robot/affine_fk.h`, `robot/hybrid_aabb.h` |
| Sec III-F: Global Critical Point Cache (GCPC) | III-F-1 Symmetry Reductions (q₀ elimination, q₁ period-π), III-F-2 KD-Tree Range Query, III-F-3 Cache-First Pipeline (Phases A–G), III-F-4 AA Two-Level Pruning, III-F-5 Dual Phase G Interior Descent, III-F-6 Cache Enrichment | `src/envelope/` | `envelope/gcpc_cache.h`, `envelope/gcpc_cache.cpp` |
| Sec IV: Sparse Voxel BitMask | IV-A BitBrick Layout, IV-B Hull-16 Rasterisation, IV-C Turbo Scanline, IV-D Collision & Merge | `src/voxel/` | `voxel/bit_brick.h`, `voxel/voxel_grid.h`, `voxel/hull_rasteriser.h` |
| Sec V: SafeBoxForest Architecture | V-A LECT, V-B Forest Growth, V-C FFB, V-D Coarsening, V-E GCS Graph, V-F Incremental | `src/envelope/`, `src/forest/` | `envelope/frame_source.h` (Stage 1: EndpointSource → endpoint_aabb), `envelope/envelope_type.h` (Stage 2: endpoint_aabb → LinkEnvelope), `forest/lect.h`, `forest/sbf.h` |
| Sec V-B: ForestGrower (Multi-Root Expansion) | V-B-1 Root Selection (FPS), V-B-2 Subtree Partitioning, V-B-3 Wavefront/RRT Expansion, V-B-4 Thread-Pool Parallelism, V-B-5 Adaptive min_edge, V-B-6 Boundary Bridging, V-B-7 LECT Warm-Start | `src/forest/` | `forest/forest_grower.h`, `forest/grower_config.h`, `forest/adjacency_graph.h`, `forest/kd_tree.h`, `forest/thread_pool.h` |
| Sec VI: GCS Path Planning | VI-A GCS Integration, VI-B Post-Processing | `src/planner/` (not yet migrated) |  |

## Key Algorithm -> Function Mapping

| Algorithm / Concept | Function / Class | File |
|---|---|---|
| Interval-FK (Eq. 3) | `compute_fk_full()` | `robot/interval_fk.h` |
| Incremental FK (prefix reuse) | `compute_fk_incremental()` | `robot/interval_fk.h` |
| Link-AABB envelope (Eq. 2) | `extract_link_aabbs()`, `extract_link_aabbs_from_endpoint()` | `robot/interval_fk.h`, `envelope/frame_source.h` |
| Link subdivision | `derive_aabb_subdivided()` | `envelope/envelope_derive.h` |
| Affine-Arithmetic FK (Eq. aa_fk) | `aa_compute_fk()` | `robot/affine_fk.h` |
| AA Link-AABB envelope | `aa_extract_link_aabbs()`, `aa_compute_link_aabbs()` | `robot/affine_fk.h` |
| AA Taylor trig linearisation (cos²+sin²≈1 correlation) | `aa_cos_sin()`, `aa_cos_sin_interval()` | `robot/affine_fk.h` |
| Hybrid IA/AA dispatcher (crossover strategy) | `extract_link_aabbs_hybrid()`, `max_interval_width()` | `robot/hybrid_aabb.h` |
| AA crossover config | `EnvelopeConfig::aa_crossover_width`, `PipelineConfig::aa_crossover_width` | `common/config.h`, `envelope/envelope_type.h` |
| Critical-point sampling | `derive_aabb_critical_analytical()` | `envelope/envelope_derive_critical.h` |
| Prop. 1 (sinusoidal structure) | `solve_edges()` 3-point fit + atan2 | `envelope/envelope_derive_critical.cpp` |
| Prop. 2 (bilinear trig structure) | `solve_faces()` 9-point fit | `envelope/envelope_derive_critical.cpp` |
| Prop. 3 (degree-≤8 resultant) | `solve_faces()` symbolic poly + interval solver | `envelope/envelope_derive_critical.cpp` |
| Prop. 4 (pair-manifold reduction) | `solve_pair_constrained_1d()` | `envelope/envelope_derive_critical.cpp` |
| Thm. 1 (face-k completeness) | Phase 0+1+2 boundary enumeration | `envelope/envelope_derive_critical.cpp` |
| Thm. 2 (pair-manifold completeness) | Phase 2.5a | `envelope/envelope_derive_critical.cpp` |
| Prop. 5 (merge dominance) | dual_phase3 checkpoint + merge | `envelope/envelope_derive_critical.cpp` |
| Fused dual-Phase-3 solver | `AnalyticalCriticalConfig::dual_phase3` | `envelope/envelope_derive_critical.h` |
| Thm. 3 (Certified Soundness) | `AnalyticalCriticalConfig::certified` + `compute_fk_full()` clamp | `envelope/envelope_derive_critical.cpp` |
| Sandwich inequality (Eq. sandwich) | analytical inner-bound vs interval-FK outer-bound | `envelope/envelope_derive_critical.cpp` |
| **Two-Stage Pipeline (4×4)** | | |
| Stage 1: Endpoint AABB computation | `compute_endpoint_aabb()` | `envelope/frame_source.h` |
| Stage 1: IFK source → endpoint_aabb | `compute_endpoint_aabb(ifk, ...)` | `envelope/frame_source.cpp` |
| Stage 1: CritSample → endpoint_aabb | `compute_endpoint_aabb(crit, ...)` | `envelope/frame_source.cpp` |
| Stage 1: AnalyticalCrit → endpoint_aabb | `compute_endpoint_aabb(analytical, ...)` | `envelope/frame_source.cpp` |
| Stage 1: GCPC → endpoint_aabb | `compute_endpoint_aabb(gcpc, ...)` | `envelope/frame_source.cpp` |
| Stage 2: Link envelope from endpoint_aabb | `compute_link_envelope()` | `envelope/envelope_type.h` |
| Stage 2: AABB (union of sub-AABBs) | `compute_link_envelope(aabb, ...)` | `envelope/envelope_type.cpp` |
| Stage 2: SubAABB (per-subdivision) | `compute_link_envelope(sub_aabb, ...)` | `envelope/envelope_type.cpp` |
| Stage 2: SubAABB_Grid (byte grid) | `compute_link_envelope(sub_aabb_grid, ...)` | `envelope/envelope_type.cpp` |
| Stage 2: Hull16_Grid (sparse voxel) | `compute_link_envelope(hull16_grid, ...)` | `envelope/envelope_type.cpp` |
| LECT frame cache (Sec V-A) | `FrameStore` | `envelope/frame_store.h` |
| Grid envelope | `GridStore`, `derive_grid()` | `envelope/grid_store.h`, `envelope/envelope_derive.h` |
| AABB collision (early-exit) | `collide_aabb()` | `envelope/collision_policy.h` |
| Grid collision | `collide_grid()`, `GridStore::check_collision()` | `envelope/collision_policy.h`, `envelope/grid_store.h` |
| BitBrick 8³ tile (Sec IV-A) | `BitBrick` struct | `voxel/bit_brick.h` |
| Sparse voxel grid (Sec IV-A) | `VoxelGrid` class | `voxel/voxel_grid.h` |
| Hull-16 convex hull (Sec IV-B, Eq. hull16) | `VoxelGrid::fill_hull16()` | `voxel/voxel_grid.h` |
| Turbo scanline / t-range solver (Sec IV-C, Eq. turbo_x) | `VoxelGrid::find_t_range_yz()`, `fill_hull16()` | `voxel/voxel_grid.h` |
| Voxel collision & merge (Sec IV-D) | `VoxelGrid::collides()`, `VoxelGrid::merge()` | `voxel/voxel_grid.h` |
| Robot envelope → VoxelGrid | `rasterise_robot_hull16()` | `voxel/hull_rasteriser.h` |
| LECT hull grid derivation (unified fill_hull16) | `LECT::derive_hull_grid()` — uses `fill_hull16` for all pipelines (no sub-AABB intermediate) | `forest/lect.cpp` |
| LECT pre-compute all hull grids | `LECT::compute_all_hull_grids()` | `forest/lect.h` |
| Sub-AABB → VoxelGrid | `rasterise_robot_sub_aabbs()` | `voxel/hull_rasteriser.h` |
| Box obstacle → VoxelGrid | `rasterise_box_obstacle()` | `voxel/hull_rasteriser.h` |
| Position interval extraction | `frame_pos()`, `PosInterval` | `voxel/hull_rasteriser.h` |
| **GCPC — Global Critical Point Cache** | | |
| GCPC cache build + KD-tree organisation | `GcpcCache::build()`, `build_kdtree()` | `envelope/gcpc_cache.h` |
| GCPC cache enrichment (multi-start coordinate descent) | `GcpcCache::enrich_with_interior_search()` | `envelope/gcpc_cache.cpp` |
| GCPC Phase A: interior KD-tree range query + q₀/q₁ reconstruction | `GcpcCache::query_link()`, `reconstruct_q0_xy()` | `envelope/gcpc_cache.cpp` |
| GCPC Phase B: boundary kπ/2 vertex enumeration | `derive_aabb_with_gcpc()` Phase B loop | `envelope/gcpc_cache.cpp` |
| GCPC Phase C: 1D edge atan2 (AA-pruned) | `derive_aabb_with_gcpc()` Phase C | `envelope/gcpc_cache.cpp` |
| GCPC Phase D: 2D face polynomial (AA-pruned, symbolic + interval solver) | `derive_aabb_with_gcpc()` Phase D | `envelope/gcpc_cache.cpp` |
| GCPC Phase E: pair-constrained 1D (AA-pruned) | `derive_aabb_with_gcpc()` Phase E | `envelope/gcpc_cache.cpp` |
| GCPC Phase F: pair-constrained 2D (AA-pruned, symbolic + interval solver) | `derive_aabb_with_gcpc()` Phase F | `envelope/gcpc_cache.cpp` |
| GCPC Phase G: dual interior coordinate descent | `run_phase_g_round()` lambda × 2 (from A–F state + A+B checkpoint) | `envelope/gcpc_cache.cpp` |
| GCPC two-level AA pruning (full-box + per-face) | `aa_link_can_improve()` at Level 1 (full box) & Level 2 (per face) | `envelope/gcpc_cache.cpp` |
| GCPC best-face-config tracking | `eval_config` lambda → `best_face_config[ci*6+face]` | `envelope/gcpc_cache.cpp` |
| q₀ elimination (atan2 reconstruction, Sec III-F-1) | `reconstruct_q0_xy()`, `q0_in_range()` | `envelope/gcpc_cache.cpp` |
| q₁ period-π symmetry (half-range storage + reflection) | `GcpcPoint::q_eff[0]` ∈ [0,π]; reflected at query | `envelope/gcpc_cache.cpp` |
| **ForestGrower — Multi-Root Forest Expansion** | | |
| Multi-root expansion (Sec V-B) | `ForestGrower::grow()` | `forest/forest_grower.h` |
| Wavefront BFS boundary expansion | `ForestGrower::grow_wavefront()` | `forest/forest_grower.cpp` |
| RRT-like nearest-box extension | `ForestGrower::grow_rrt()` | `forest/forest_grower.cpp` |
| Root selection (FPS / endpoint-forced) | `ForestGrower::select_roots()` | `forest/forest_grower.cpp` |
| Subtree C-space partitioning | `ForestGrower::partition_subtrees()` | `forest/forest_grower.cpp` |
| Thread-pool parallel expansion | `ForestGrower::grow_parallel()`, `ThreadPool` | `forest/forest_grower.cpp`, `forest/thread_pool.h` |
| Work-stealing budget (shared atomic counter) | `shared_box_count_` atomic fetch_add | `forest/forest_grower.cpp` |
| O(1) box ID lookup | `box_id_to_idx_` unordered_map | `forest/forest_grower.h` |
| KD-tree nearest-box query O(log N) | `KDTree`, `AdjacencyGraph::find_nearest_box()` | `forest/kd_tree.h`, `forest/adjacency_graph.cpp` |
| SIMD adjacency kernel (AVX2 + scalar fallback) | `check_adjacent_flat()` | `forest/adjacency_graph.cpp` |
| LECT warm-start (snapshot + pre-expand) | `LECT::snapshot()`, `LECT::pre_expand()` | `envelope/lect.h`, `envelope/lect.cpp` |
| Cross-subtree boundary bridging | `ForestGrower::bridge_subtree_boundaries()` | `forest/forest_grower.cpp` |
| LECT occupation sync after merge | `ForestGrower::sync_lect_occupation()` | `forest/forest_grower.cpp` |
| Adaptive min_edge two-phase (coarse→fine) | `in_coarse_phase` + miss_count reset + re-queue | `forest/forest_grower.cpp` |
| Per-tree goal bias (start→goal, goal→start, random→50/50) | `ForestGrower::get_bias_target()` | `forest/forest_grower.cpp` |
| RRT boundary-snap (face-aligned seed placement) | `ForestGrower::rrt_snap_to_face()` | `forest/forest_grower.cpp` |
| Boundary-aware fallback (reduce isolated fragments) | `ForestGrower::sample_near_existing_boundary()` | `forest/forest_grower.cpp` |
| Greedy coarsening (hull-merge adjacent boxes) | `ForestGrower::coarsen_greedy()` | `forest/forest_grower.cpp` |
| Arbitrary-interval collision check | `LECT::intervals_collide_scene()` | `envelope/lect.cpp` |
| Box removal (swap-with-last + LECT unmark) | `ForestGrower::remove_box_by_id()` | `forest/forest_grower.cpp` |
| Sweep-and-prune adjacency graph | `AdjacencyGraph::rebuild()` | `forest/adjacency_graph.h` |
| **2D ForestGrower Visualization (Python)** | | |
| 2D growth simulation (Wavefront/RRT/Multi-thread/Adaptive) | `ForestGrower2D.grow()` | `src/viz/forest_grower_2d.py` |
| Full-space FPS root selection (K=80 uniform candidates) | `ForestGrower2D._select_roots_with_endpoints()` | `src/viz/forest_grower_2d.py` |
| RRT boundary-snap extension (face-aligned seed placement) | `ForestGrower2D._rrt_snap_to_face()` | `src/viz/forest_grower_2d.py` |
| Per-tree goal bias (start→goal, goal→start, rand→50/50) | `ForestGrower2D._get_bias_target()` | `src/viz/forest_grower_2d.py` |
| Adaptive two-phase coarse→fine (coarse_min_edge=0.25) | `ForestGrower2D._check_phase_transition()` | `src/viz/forest_grower_2d.py` |
| Multi-thread round-robin simulation | `ForestGrower2D._grow_parallel()` | `src/viz/forest_grower_2d.py` |
| Find-Free-Box KD-tree engine (2D) | `FFBEngine.find_free_box()` | `src/viz/ffb_engine.py` |
| 4-mode comparison animation (2×2 panel) | `plot_4panel()` | `src/viz/compare_expansion.py` |
| PatchCollection/LineCollection accelerated rendering | `_draw_boxes_on_ax()` | `src/viz/render.py` |
| Random 2D C-space scene generation | `build_random_scene()` | `src/viz/forest_grower_2d.py` |

## Figures & Tables (TODO)

| Paper Element | Experiment Source | Results Path |
|---|---|---|
| Fig: 4-mode expansion comparison (Wavefront/RRT/Multi-thread/Adaptive) | `python -m src.viz.compare_expansion` | `v3/results/viz_compare_*/` |
| Fig: Wavefront step detail (single BFS expansion) | `python -m src.viz.wavefront_step` | `v3/results/viz_wavefront_step_*/` |
| Fig: Multi-process ForestGrower | `python -m src.viz.multiprocess` | `v3/results/viz_multiprocess_*/` |
| Table I: Full Pipeline Benchmark (4×4 combos, IIWA14) | `experiments/exp_pipeline_benchmark.cpp` | `exp_pipeline_out/{best_config.json, benchmark_results.csv, warm_disk_size.csv}` |
| Sec III-C dual-Phase-3 timing (43 ms, 15.3× speedup) | `v2/experiments/28_gcpc_validation.cpp` | `v2/results/exp28_gcpc_*/` |
| Sec III-C-5 certified soundness validation (0 violations, <0.3 ms overhead) | `experiments/exp_certified_envelope.cpp` | console output |
| Sec IV voxel benchmarks (turbo 230µs, 9.4× speedup) | `experiments/voxel_proto/main.cpp` | console output |
| Table/Fig: IA vs AA AABB tightness & timing (Sec III-E) | `experiments/exp_ia_vs_aa.cpp` | console (MC-validated vol ratios, µs timing) |
| Table/Fig: Hybrid IA/AA LECT integration (Sec III-E-3) | `experiments/exp_hybrid_lect.cpp` | console (nodes, fk_calls, time vs crossover) |
| Table/Fig: GCPC vs Analytical vs MC validation (Sec III-F) | `experiments/exp_gcpc_validation.cpp` | console (sum-vol per width, contains_MC, Phase A–G stats) |
| Table/Fig: 4×4 Pipeline Benchmark incl. GCPC (Sec III-F) | `experiments/exp_pipeline_benchmark.cpp` | `exp_pipeline_out/` |

> Every figure and table MUST have an entry here.

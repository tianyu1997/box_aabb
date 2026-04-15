# v6 Adjacency Optimization Plan

## Background

In current v6 `build_coverage()` runs, the reported adjacency share is very high (paper shows ~55%).
Code inspection shows two root causes:

1. Timing scope mixes pure adjacency with seed-bridge logic.
2. Repeated full adjacency recomputation across planner phases and coarsen rounds.

This plan separates measurement from optimization, then applies low-risk wins first.

## Goals

1. Make timing attribution accurate and reproducible.
2. Reduce build wall time without harming connectivity/query quality.
3. Keep behavior deterministic and reversible per phase.

## Execution Status (2026-04-15)

1. Phase 1 - Completed.
2. Phase 2 - Completed (removed unused articulation computations and redundant final adjacency rebuild).
3. Phase 3 - Not started.
4. Phase 4 - Not started.

## Non-goals

1. No algorithmic rewrite of `compute_adjacency()` in Phase 1-2.
2. No change to final graph semantics in early phases.

---

## Phase 0 - Baseline Freeze

### Actions

1. Keep current experiment command and seed count fixed:
   - `exp6_build_timing --seeds 5 --scene combined`
2. Record baseline metrics:
   - `total_ms`, `grow_ms`, `coarsen1_ms`, `bridge_ms`, `coarsen2_ms`, `adjacency_ms`
   - final `boxes`, `islands`, `edges`, query success.

### Acceptance

1. Baseline JSON/log archived under results directory.

### Rollback

1. N/A (no code change).

---

## Phase 1 - Timing Attribution Fix (Measurement Hygiene)

### Problem

`adjacency_ms` currently includes non-adjacency work in the tail section (seed-point bridge and related logic), causing inflated ratio.

### Actions

1. Split timing into:
   - `adjacency_pre_seed_ms`
   - `seed_bridge_ms`
   - `adjacency_final_ms`
2. Keep `adjacency_ms` as strict sum of only adjacency calls.
3. Keep old fields for compatibility if needed, but document interpretation.

### Files

1. `include/sbf/planner/sbf_planner.h`
2. `src/planner/sbf_planner_build.cpp`
3. `experiments/exp6_build_timing.cpp` (print new fields)

### Acceptance

1. `seed_bridge_ms` is non-zero when seed bridge runs.
2. `adjacency_ms` no longer changes when seed bridge code path is disabled but adjacency calls stay fixed.
3. Build/query outputs unchanged (same success rate and similar boxes/islands).

### Rollback

1. Revert timing-field edits only.

---

## Phase 2 - Low-Risk Redundant Compute Removal

### Problem

There are guaranteed redundant adjacency-related computations:

1. Unused articulation computation before cluster pass.
2. Extra full adjacency rebuilds where one final rebuild suffices.

### Actions

1. Remove unused `bridge_ids_cl1` / `bridge_ids_cl2` generation.
2. Remove/merge redundant adjacency rebuilds in tail path while preserving correctness.
3. Add concise comments explaining why removed call is safe.

### Files

1. `src/planner/sbf_planner_build.cpp`

### Acceptance

1. Same final islands/connectivity and query success.
2. Reduced build wall time and reduced adjacency call count.

### Rollback

1. Re-enable removed calls behind compile-time toggle if regressions appear.

---

## Phase 3 - Coarsen Internal Adjacency Cost Reduction

### Problem

`coarsen_greedy()` and `coarsen_cluster()` rebuild full adjacency each round.

### Actions

1. Add cheap early-stop guards before adjacency rebuild (e.g., no candidate space left by box-count delta thresholds).
2. Optional: cap rounds adaptively after no-merge streak.
3. Keep exact semantics for accepted merges.

### Files

1. `src/forest/coarsen.cpp`
2. `include/sbf/forest/coarsen.h` (if config knobs added)

### Acceptance

1. Coarsen runtime decreases.
2. Box count reduction remains within acceptable delta (no major regression).

### Rollback

1. Disable adaptive guards via config default.

---

## Phase 4 - Optional Incremental Adjacency (Higher Risk)

### Actions

1. Introduce incremental neighbor update for merged nodes in coarsen loops.
2. Periodic full rebuild for correctness sanity (`every N rounds`).

### Acceptance

1. Significant further speedup on large box sets.
2. Final graph consistency validated by spot full-recompute checks.

### Rollback

1. Runtime flag to force full rebuild every round.

---

## Validation Matrix (Every Phase)

1. Build performance:
   - `exp6_build_timing --seeds 5 --scene combined`
2. Query correctness:
   - 5 benchmark pairs success + collision-free validation.
3. Structural consistency:
   - final islands, edges, box counts, connected ratio.

## Reporting Template

For each phase report:

1. Code changes (file + brief).
2. Metrics delta vs baseline.
3. Risks observed.
4. Decision: keep / rollback.

## Execution Order

1. Phase 1
2. Phase 2
3. Re-benchmark
4. Phase 3
5. Re-benchmark
6. Decide whether to enter Phase 4

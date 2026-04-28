# Connectivity Standardization Plan (v5/v6)

## Goal
Unify connectivity judgment across grower, planner, and experiments using one canonical standard:
- Canonical: UnionFind (UF) tree-connectivity from grower.
- Diagnostic only: adjacency-graph islands from compute_adjacency + find_islands.

## Why
Current pipeline has two non-equivalent signals:
- Grower internal union-find connectivity (tree-level).
- Post-hoc adjacency islands (box-graph geometric connectivity).
This can produce contradictory results in logs and papers.

## Standard
- Canonical connected: UF all_connected == true.
- Canonical connect time: UF connect_time_ms.
- Adjacency islands are retained for diagnostics/audit only, never for branch truth.

## Phase Plan

### Phase 0: Instrumentation (v6 first)
Status: Completed
1. Extend GrowerResult with adjacency-based fields:
   - adjacency_all_connected
   - adjacency_islands
   - adjacency_largest_island
   - adjacency_check_ms
2. At end of ForestGrower::grow(), run one adjacency+islands check and fill fields.
3. Keep existing all_connected/connect_time_ms/connect_n_boxes untouched.
4. Log both statuses in planner build_coverage for side-by-side audit.

Acceptance:
- Build passes.
- exp6_build_timing prints both UF and adjacency connectivity states.

### Phase 1: Canonical switch in v6 planner/experiments
Status: In progress
1. Use UF all_connected as canonical condition in build_coverage branch decisions.
2. Keep adjacency status in output as secondary diagnostic.
3. Update timing/JSON output labels to avoid ambiguity.

Progress:
- build_coverage now uses UF all_connected as canonical branch condition.
- adjacency islands are retained in logs as diagnostic signal.

Acceptance:
- No contradictory connected/islands outputs in v6 experiments.

### Phase 2: Backport to v5 exp_box_connect
Status: Completed
1. Add same adjacency-based fields in v5 GrowerResult.
2. Print both UF and adjacency status in exp_box_connect summary.
3. Keep summary Connected metric on UF canonical.

Progress:
- Added adjacency diagnostic fields in v5 GrowerResult and populated in grow().
- exp_box_connect per-seed and summary now explicitly label
   Connected=UF canonical and Islands=adjacency diagnostic.

Acceptance:
- v5 logs explicitly show two metrics and canonical metric is unambiguous.

### Phase 3: Cleanup and contract
Status: Completed
1. Add code comments documenting contract:
- UF = final connectivity truth.
- adjacency islands = diagnostic audit signal.
2. Add regression smoke command snippets in docs.

Progress:
- Added planner-level contract comments in v5/v6 planner headers and source files.
- Added experiment-level contract notes in v5/v6 experiment source headers.
- Added contract section in v5/v6 experiments/doc/README.md.

Smoke commands:
- v6: `cd cpp/v6/build && ./experiments/exp6_build_timing --seeds 1 --scene combined | grep -E 'canonical connectivity source|grow connectivity'`
- v5: `cd cpp/v5/build && ./experiments/exp_box_connect --seeds 1 --mc 1000 --threads 5 --timeout 20000 --stop-on-connect --no-coarsen | grep -E 'Standard:|Connected \(UF canonical\)|connected\(UF-canonical\)|islands\(diagnostic\)'`

Acceptance:
- Team-visible contract is explicit in code/docs.

## Risks
- Small extra cost for one final adjacency check per grow call (ms-level).
- Historical metric drift after canonical switch.

## Rollback
- Phase 0 is additive; rollback by ignoring new fields.
- Later phases gated by small targeted commits.

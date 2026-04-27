# TRO Major-Revision Full Plan (v6 Paper)

Target: bring [cpp/v6/doc/box_aabb_v6_paper_en.tex](../box_aabb_v6_paper_en.tex)
and its Chinese mirror [cpp/v6/doc/box_aabb_v6_paper_zh.tex](../box_aabb_v6_paper_zh.tex)
from current Major-Revision state to a re-submittable version, addressing the
reviewer concerns on (i) numerical-claim consistency, (ii) theory rigor,
(iii) baseline fairness, (iv) experimental breadth, (v) lifelong/multi-query
evidence.

User decisions locked-in for this plan:
- **D1**: Compute budget allows IRIS-NP at 130 s precompute on all scenes.
- **D2**: Introduce and verify capsule + DH for Franka **Panda** and **UR10**
  (Panda JSON exists but has no `link_radii`; UR10 is missing entirely).
- **D3**: Execute the **full** five-phase plan (no minimum-viable cut).

This document is the master plan. Per-phase progress will be tracked in
session memory and per-experiment scripts under
[cpp/v6/experiments/scripts/](../../experiments/scripts).

---

## Phase 0 — Robot Asset Hardening (BLOCKING for Phase C)

Findings from current repo:
- [cpp/v6/data/iiwa14.json](../../data/iiwa14.json) has `link_radii`
  `[0.10, 0.0, 0.075, 0.0, 0.065, 0.0, 0.06, 0.05]` — capsule is correct.
- [cpp/v6/data/panda.json](../../data/panda.json) has DH + joint limits but
  **no `link_radii`** field.
  [src/core/robot.cpp](../../src/core/robot.cpp) line 187 silently fills the
  active-link radii with `0.0` when missing — **this is a soundness bug for
  Panda experiments**: every capsule degenerates to a zero-radius segment, so
  Theorem 2 still holds in form but the safety margin is lost.
- No UR10 / UR5 / UR5e configuration exists under [cpp/v6/data/](../../data/).

### 0.1 Panda capsule audit
- Source MuJoCo / Drake Panda URDF link mesh radii (Franka Emika spec, also
  used by Drake `franka_panda` model). Reference values (m):
  `[0.075, 0.075, 0.07, 0.07, 0.06, 0.06, 0.05, 0.04]` (8 entries to
  match 7 active links + tool, mirroring iiwa14 layout).
- Validate: render capsule envelope vs URDF mesh in
  [cpp/v6/viz/](../../viz) and visually confirm full enclosure on a 200-pose
  Halton sample.
- Add `link_radii` to [cpp/v6/data/panda.json](../../data/panda.json) and
  re-run [cpp/v6/test/bench_endpoint_iaabb.cpp](../../test/bench_endpoint_iaabb.cpp)
  to confirm IFK/CritSample/Analytical/GCPC still pass and the Krawczyk
  verifier reports `false_accept == 0`.
- Re-build the persistent GCPC cache `data/panda_5000.gcpc` because the DH
  hash changes once `link_radii` is added (radii are part of the kinematics
  fingerprint at [src/core/robot.cpp](../../src/core/robot.cpp) line 130).

### 0.2 UR10 introduction
- Add `cpp/v6/data/ur10.json` with Modified-DH parameters from Universal
  Robots official kinematics (alpha, a, d, theta_offset; 6 revolute joints).
- Active links: 6, no tool segment by default.
- Capsule radii: from UR10 mesh bounding cylinders, e.g.
  `[0.090, 0.090, 0.075, 0.060, 0.050, 0.045]` (verify against URDF).
- Add joint limits (UR10 has $\pm 2\pi$ on most joints — be careful with
  Z4 cache: the rotational symmetry assumption used for IIWA14 does NOT
  apply directly; document this in Sec. 4.4 of the paper).
- Generate `data/ur10_5000.gcpc` via the same offline pre-compute path used
  for IIWA14/Panda.
- Add UR10 unit tests:
  - [cpp/v6/test/bench_endpoint_iaabb.cpp](../../test/bench_endpoint_iaabb.cpp)
    new block (mirror Panda block at line 375).
  - [cpp/v6/test/bench_pipeline.cpp](../../test/bench_pipeline.cpp)
    new block (mirror Panda block at line 396, with appropriate half-widths).

### 0.3 (Optional) UR5e for sanity check
- Same template as UR10, smaller workspace; useful for cross-checking the
  6-DOF baseline numbers without confounding "DOF" and "robot family".

### 0.4 Acceptance gate for Phase 0
- All three robots (IIWA14 / Panda / UR10) build without error in
  `cpp/build` and pass `bench_endpoint_iaabb` with `false_accept == 0` on
  $\geq 5{,}000$ random boxes per width bin.
- GCPC caches are deterministic across two runs (same hash).
- Visualization check signed off.

---

## Phase A — Paper Consistency & Theory Hardening (no new experiments)

Targets are the existing tex files. Each item lists the file and the
approximate location to edit.

### A1. Reconcile path-length / speed-up numbers (single source of truth)
Conflict points to fix:
- Abstract claims `matches IRIS-NP+GCS path quality within 17%` and
  `83x pre-computation speed-up`
  ([box_aabb_v6_paper_en.tex L82-L84](../box_aabb_v6_paper_en.tex#L82)).
- [Sec. 7 baselines tab:baselines](../box_aabb_v6_paper_en.tex#L2113) gives
  `2.87 vs 2.44 rad` (=17.6 % gap).
- [Sec. 7.5 SBF→GCS corridor tab:gcs_corridor](../box_aabb_v6_paper_en.tex#L2042)
  gives mean SBF=2.863 rad, SBF→GCS=2.063 rad.
- [Sec. 8.2 discussion](../box_aabb_v6_paper_en.tex#L2280) gives
  `2.87 vs 3.31 rad` — contradicts the prior table.
**Action**: produce one canonical CSV `cpp/v6/doc/generated/canonical_numbers.csv`
with `{quantity, value, source_table, source_seed_count}`; rewrite each
in-text mention to `\input` or hard-cite that CSV. Add a unit test
`scripts/check_paper_consistency.py` that greps the tex for floats and
diffs them against the CSV.

### A2. Disambiguate the 100% success-rate scope
- Abstract says `100% success rate`. [Tab. tab:scale](../box_aabb_v6_paper_en.tex#L2078)
  shows 4-obstacle case → 0% success.
- Rewrite abstract: `100% success rate on the Marcucci combined scene with
  16 obstacles and 5 query pairs; the dense 4-obstacle stress in
  Sec. 7.6(b) maps a different operating point and is reported separately.`
- In Sec. 7.6 add an explicit "Failure mode" paragraph naming the corridor
  width threshold below which SBF fails (measure during Phase B).

### A3. Strengthen proofs
- [Theorem 1 erosion duality](../box_aabb_v6_paper_en.tex#L370): expand the
  one-line proof into both ⊆ directions explicitly using the defining
  property of Minkowski sums; cite a textbook step
  ([ericson2004real](../box_aabb_v6_paper_en.tex#L2599) Ch. 4 has it).
- [Prop. iaabb_contains](../box_aabb_v6_paper_en.tex#L382): insert the missing
  step linking $\ell^\infty$ pad to $\ell^2$ ball
  ($B_3(r) \subseteq B_\infty(r)$ since $\|x\|_2 \le r \Rightarrow
  \max_d|x_d|\le r$). State this as a one-line lemma.
- [Theorem 4 face-k completeness](../box_aabb_v6_paper_en.tex#L515): make the
  $|F| \le 2$ restriction loud; cross-reference Sec. 3.6 BnB verifier
  immediately so the reader sees Phase 3 is closed by Krawczyk.
- Add a new **Theorem (Sound Coarsening)**: for any candidate hull H,
  if [HullSafe](../box_aabb_v6_paper_en.tex#L1326) returns true then
  $H \subseteq \Cfree$, where the safety check uses the **Safe** channel
  envelope only (not Tight). Provide one-paragraph proof.

### A4. Tight→Safe verification chain in coarsening
- Edit [Alg. 2 multi-level coarsening](../box_aabb_v6_paper_en.tex#L1326):
  in the `HullSafe` step, explicitly write the channel: `envelope from
  channel=Safe (IFK)`. Add a footnote: "Tight-channel envelopes are used
  only to score candidate hulls and to guide growth; every retained hull
  is re-certified against the Safe channel."
- In [Sec. 4.2 dual-channel](../box_aabb_v6_paper_en.tex#L1006) add a
  forward pointer: "see Alg. 2 line X for the Tight→Safe verification step."

### A5. Trim contributions 6 → 4
- Merge contributions (1) "erosion-dilation duality" + (2) "unified
  comparison" → "Unified envelope theory and Pareto characterization."
- Merge (3) "LECT" + (4) "coordinated grower" → "Lifelong Envelope Cache
  Tree with concurrent grower."
- Keep (5) GCS front-end and (6) IIWA14 validation as-is.

### A6. Move negative results to appendix
- [Sec. 8.4 P1/P3 negative results](../box_aabb_v6_paper_en.tex#L2298)
  → new `Appendix C: Negative-Result Notes`.

### A7. Hero figure (IIWA14 + Marcucci scene)
- Render a 7-DOF IIWA14 in the Marcucci combined scene
  (shelves + bins + table) using existing
  [cpp/v6/viz/](../../viz) Drake-Meshcat exporter, save 4 panes
  (start, mid, goal, box-corridor overlay) → `figures/fig_iiwa_hero.png`.
- Promote to `\begin{figure*}` immediately after the abstract; demote the
  current planar 2-link `fig:overview` to Sec. 2 or appendix.

### A8. Misc minor fixes (collected)
- Rename `\Cref{tab:m2_pareto}` Analytical column header to
  "Analytical (gate-disabled)" so the 8170 µs number is not read as
  default-pipeline cost.
- Unify pseudocode style (algorithmic vs algpseudocode) in Algorithms 1–3.
- Add 95% CI to all medians in Sec. 7.4–7.6 (post-Phase B re-runs).
- Cross-link `\Cref{rem:safety}` from the abstract's `100% SR` claim.
- Add capsule-radius footnote pointing to [data/iiwa14.json](../../data/iiwa14.json),
  [data/panda.json](../../data/panda.json), [data/ur10.json](../../data/ur10.json).

### A9. Acceptance gate
- `scripts/check_paper_consistency.py` exits 0.
- `latexmk -pdf` produces no `LaTeX Warning: Reference ... undefined`.
- Cold reading by author: every abstract number maps to a single,
  unambiguous table cell.

---

## Phase B — Baselines & Statistics (new experiments)

All experiments below run on **all three robots × 3 representative scenes**
(see Phase C for scene list). Each cell is medianed over **10 seeds** with
`{p5, p50, p95, mean ± std}` reported. Query set per scene: **50** randomly
sampled `(q_start, q_goal)` pairs that pass a feasibility pre-screen
(reachable by Drake RRT-Connect within 30 s).

### B1. IRIS-NP precompute-budget Pareto sweep
- Budgets: `{1.5, 10, 30, 60, 130}` s. For each budget, run IRIS-NP via
  Drake (`pydrake.planning.IrisInConfigurationSpace`) until the cumulative
  per-region wall-clock equals the budget; record `n_regions, total_time,
  query_path_length(seed=0..9)`.
- Output: `cpp/v6/experiments/results_new/B1_irisnp_pareto.json` and a
  Pareto plot `figures/fig_irisnp_pareto.pdf` overlaid with the SBF point.

### B2. PRM node-count sweep
- Nodes: `{1k, 3k, 10k, 30k}`. Use Drake parallel scene-graph collision
  checker to match the Marcucci 2024 setup.
- Output: `B2_prm_sweep.json`, included as a scaling row in the new
  `tab:baselines_v2`.

### B3. Add Werner 2024 "Fast path planning through large collections of
safe boxes" baseline
- This is the closest competitor (also large box collections). Use the
  authors' open code if available
  ([github.com/RobotLocomotion/drake](https://github.com/RobotLocomotion/drake) — has
  `GcsTrajectoryOptimization`); otherwise reproduce minimum subset:
  IRIS regions → GCS with Bezier edges, on the same scenes.
- Output: `B3_werner2024.json`.

### B4. Add IRIS-ZO baseline
- Use Drake's `IrisZo` (PR merged 2024) at default settings. Same 50 queries.
- Output: `B4_iris_zo.json`.

### B5. Add BIT* and AIT* via OMPL
- Wrap OMPL `ompl::geometric::BITstar` and `AITstar` against the existing
  Drake collision checker. Time budgets: `{0.5, 1, 3, 10}` s.
- Output: `B5_ompl_bit_ait.json`.

### B6. Query-set expansion
- Replace the hand-picked 5-query AS/TS/CS/LB/RB set with 50 sampled pairs;
  retain the original 5 as a labeled subset for backwards compatibility
  (table `tab:per_query_legacy` in appendix).
- Tooling: `cpp/v6/experiments/scripts/sample_queries.py` (new).

### B7. Statistics
- Update [cpp/v6/doc/generated/tab4_baselines.tex](../generated/) and
  `tab5_per_query.tex` to the new schema:
  `method | precompute_s | query_s_p50 | query_s_p95 | path_rad_p50 | SR%`.
- Replace medians-only with `p50 (p5–p95)` cells.

### B8. Acceptance gate
- All five JSON outputs exist, parse, and are referenced from the new
  `tab:baselines_v2`.
- The `precompute_s vs path_rad` plot shows IRIS-NP across 5 budget points;
  the SBF cluster lies on the cheap-precompute / mid-quality knee.

---

## Phase C — Multi-Robot / Multi-Scene Coverage

### C1. Robots
- IIWA14 (existing).
- Panda (after Phase 0.1).
- UR10 (after Phase 0.2).

### C2. Scenes (each rendered for all 3 robots where reachable)
- `marcucci_combined` (existing 16 obstacles).
- `bins_only` (3 bins, narrow drop corridor).
- `dense_clutter` (24 random AABBs, density tuned so that nominal
  RRT-Connect finds a path in <5 s).
- `tabletop_pickplace` (table + 6 graspable cylinders bounded by AABBs).
- Each scene shipped as `cpp/v6/data/scenes/<name>.json`.

### C3. DOF scaling table fill-in
- Add 3-DOF (planar 3R), 4-DOF (planar 4R), 6-DOF (UR10) rows.
- Existing 2-DOF and 7-DOF rows kept.

### C4. Output
- `cpp/v6/experiments/results_new/C_multirobot_multiscene/`, one JSON per
  `(robot, scene, method, seed)` tuple. Aggregator script
  `scripts/aggregate_C.py` produces `tab:multirobot.tex`.

### C5. Acceptance gate
- 3 robots × 4 scenes × 6 methods × 10 seeds × 50 queries (=36 000 runs);
  budget total wall-clock estimate ≤ 72 h on the lab cluster (queue this).
- For every (robot, scene), at least one method achieves ≥ 90% SR;
  otherwise mark the cell `infeasible` and discuss in Sec. 7.

---

## Phase D — Lifelong / Multi-Query Evidence

### D1. Cold→warm cache scaling curve
- Pipeline: clear `~/.cache/sbf` → run build → measure (build_s, query_s_total)
  → re-run on same scene/robot → repeat 30 times.
- Plot `figures/fig_warm_cache.pdf` with build-time on log y, run index on x;
  expected: order-of-magnitude drop after run 1 due to mmap'd LECT.

### D2. Scene-change stress
- (a) Move one obstacle by `{0.01, 0.05, 0.1, 0.3, 1.0}` m; measure
  invalidated-box count, repair-time, query SR after repair.
- (b) Add or remove `{1, 2, 3}` obstacles; same metrics.
- (c) Same robot, completely different scene category (e.g.
  `marcucci_combined → bins_only`); measure cache reuse % (LECT nodes
  successfully reused vs. re-materialized).
- Output: `D2_scene_change.json` and `figures/fig_scene_change.pdf`
  (3 panels, one per sub-experiment).

### D3. Cross-scene cache reuse
- Build LECT on scene A; without rebuild, plan on scene B (same robot).
  Report SR, query latency, and the LECT-hit-vs-miss histogram.

### D4. Acceptance gate
- Cold→warm curve shows ≥ 5× build-time speed-up at run 30.
- Scene-change repair time at 0.1 m perturbation ≤ 30% of cold build.
- Cross-scene SR ≥ 80% for all scene-pair combinations of the same robot.

---

## Phase E — Bilingual Synchronization

### E1. After every accepted Phase A/B/C/D edit to
[box_aabb_v6_paper_en.tex](../box_aabb_v6_paper_en.tex), mirror the edit
into [box_aabb_v6_paper_zh.tex](../box_aabb_v6_paper_zh.tex). User
preference (saved): keep ZH and EN structurally identical.

### E2. Tooling
- `scripts/diff_en_zh.py`: parses both tex files, extracts section
  headers and table labels, reports any structural drift. Run pre-commit.

### E3. Acceptance gate
- `diff_en_zh.py` reports zero structural diff before final submission.

---

## Cross-Phase Dependencies

```
Phase 0 (robots)         ─┬→ Phase B (uses 3 robots)
                          └→ Phase C (uses 3 robots × 4 scenes)
Phase A (paper-only)     ─→ Phase E
Phase B                  ─┬→ Phase C (shares query/seed infra)
                          └→ Tab. tab:baselines_v2 in paper
Phase C                  ─→ Phase E (translation)
Phase D                  ─→ Phase E (translation)
```

Suggested execution order (parallelizable where bandwidth allows):
1. Phase 0 → unblock everything
2. Phase A (paper-only, can start immediately, parallel with Phase 0)
3. Phase B (after Phase 0)
4. Phase C and Phase D in parallel (after Phase B finishes B1/B2 infra)
5. Phase E continuously, gated only by content stability per section.

---

## Risk & Mitigation

- **R1 — IRIS-NP at 130 s × multiple scenes is expensive.**
  Mitigation: run 130 s only on `marcucci_combined`; run 30 s on the
  other three scenes; clearly footnote the asymmetry in Sec. 7.
- **R2 — UR10 capsule radii not formally validated.**
  Mitigation: Phase 0.2 viz check + `false_accept == 0` gate; if any
  bin shows false-accept > 0, increase radii by 5% and re-test.
- **R3 — Drake API drift.**
  Pin Drake version to the same release used by Marcucci 2024
  (Drake 1.31, already documented in Sec. 7.1). Record commit hash.
- **R4 — Wall-clock for 36 000 runs (Phase C).**
  Pre-shard by `(robot, scene)`; resumable via per-cell sentinel files.
- **R5 — Translator drift between EN and ZH after re-runs.**
  Tooling E2 enforces structural parity; numerical cells generated from
  shared CSV.

---

## Tracking

Per-phase progress notes will be appended to
`/memories/session/plan.md`. Final paper-ready artefacts land under
[cpp/v6/doc/generated/](../generated/) (tables) and
[cpp/v6/doc/figures/](../figures/) (figures). Raw experiment outputs
land under [cpp/v6/experiments/results_new/](../../experiments/results_new/).

# TRO Reviewer-Pass Revision Plan (2026-04-24)

Scope: synchronized edits to
- `cpp/v6/doc/box_aabb_v6_paper_en.tex`
- `cpp/v6/doc/box_aabb_v6_paper_zh.tex`

Goal: address the Major-Revision-class issues raised in the internal
TRO-style review without requiring new long-running experiments. Items
that need a full rerun (IRIS-NP/IRIS-ZO, |F|≥3 completeness table,
≥20-query benchmark) are explicitly deferred.

---

## Phase A — Internal consistency & low-risk cleanup (no rerun)

| ID | Item | Files |
|----|------|-------|
| A1 | Unify box-count: 3,468 vs 3,533 → single canonical value from `generated/canonical_numbers.csv` (use the 10-seed median consistently in abstract, contribution #4, build §). | EN+ZH |
| A2 | Fix PRM 100%-SR contradiction in §IX-G (10k claim vs 30k claim). | EN+ZH |
| A3 | Reconcile Analytical timing: Table I "7–10 ms" vs Table II wide-bin "~8 ms" — split into typical / worst-case columns or add footnote. | EN+ZH |
| A4 | Reconcile "12× peak-memory drop" (§VI-H) with M3 65.3% RSS — add baseline definition. | EN+ZH |
| A5 | Definition 1 half-width vs full-width usage; add note in width-bin tables which one is used. | EN+ZH |
| A6 | Move "98.9% monotonicity probability" out of Theorem 6 proof into a Remark. | EN+ZH |
| A7 | Theorem 1 proof: state convexity assumption on $A,B$ explicitly; cite Ericson §4. | EN+ZH |
| A8 | Algorithm 1: define `occupied` semantics in §VI-A before its first use. | EN+ZH |
| A9 | Theorem 8: clarify that Tarjan articulation guard is a heuristic, not part of the safety claim. | EN+ZH |
| A10 | Add prominent caption-level note that Table X(a)(b)(c) uses synthetic narrow-corridor scenes (not Marcucci-16). | EN+ZH |
| A11 | Abstract trim: drop IQR, merge raw vs post-opt sentences. | EN+ZH |
| A12 | Add an operational definition of "lifelong" near the top of §I. | EN+ZH |

## Phase B — Citation / comparison honesty (no rerun)

| ID | Item | Files |
|----|------|-------|
| B1 | Rewrite IRIS-NP "two orders of magnitude" claim: explicitly attribute the 8 regions / 130 s figure to [marcucci2024motion]'s reported budget on different hardware; under our hardware a single region took ~210 s. Replace blanket "100×" with the qualified version. | EN+ZH |
| B2 | Mention IRIS-ZO [werner2024irisze] in §IX-G discussion as a faster IRIS variant whose head-to-head comparison is left to future work. | EN+ZH |
| B3 | Add an explicit differentiator paragraph for [werner2024fast] in §II-A (lifelong cache + dual channel + sub-second build). | EN+ZH |
| B4 | Add at least one modern approximate-cell-decomposition reference in §II-A (e.g., lazy-CD / incremental cell decomposition). Not required to add a new bibitem if a fitting one already exists. | EN+ZH |

## Phase C — Notation & writing polish

| ID | Item | Files |
|----|------|-------|
| C1 | Unify epiAABB notation: keep $E_k(\mathcal{B})$ and remove $E_e$ / $E_{e_\ell}$ aliases, or document the alias once in the notation table. | EN+ZH |
| C2 | Pre-announce the snapshot / concurrency model briefly at end of §VI so §VII does not surprise. | EN+ZH |
| C3 | Slim Fig `envelope_unified` caption (one sentence per subplot). | EN+ZH |

## Phase D — Compile + validation

| ID | Item |
|----|------|
| D1 | `pdflatex` EN twice + `xelatex` ZH twice; ensure no new warnings. |
| D2 | Page-count delta ≤ ±1 vs current PDFs. |
| D3 | Stale-number grep: `3,468`, `12×`, `100\%`, `2.44`, `0.279` cross-checked. |

## Deferred (require rerun, separate round)
- IRIS-NP / IRIS-ZO budget sweep on our i5-12600KF hardware.
- |F|≥3 analytical-vs-Krawczyk completeness table (M1 extension).
- ≥20-query benchmark to bolster small-sample claims.
- 8/16/32 obstacle non-adversarial scaling rows.

## Execution order
A1 → A2 → A3 → A4 → A5 → A6 → A7 → A8 → A9 → A10 → A11 → A12 →
B1 → B2 → B3 → B4 → C1 → C2 → C3 → D1 → D2 → D3.

Each EN edit is mirrored in ZH immediately.

---

# Round-2 Reviewer Plan (added 2026-04-24, T-RO Major-Revision pass)

This round consolidates the issues raised in the second internal T-RO
review pass (M1–M7, W1–W10, E1–E7) into executable phases.
Phases P1 are writing-only (no rerun), P2–P6 require limited reruns,
P7 is deferred.

## P1 — Writing-only fixes (no experiment rerun)

### P1.1 Abstract / contributions trimming (W1, W2, M1, M6)
- Cut abstract ≤ 200 words; remove all `\Cref` references inside abstract.
- Replace "two orders of magnitude" → "an order of magnitude on the same
  hardware (single IRIS-NP region 210 s vs 2.23 s SBF build)" with
  region/box normalization caveat.
- Replace "lifelong" with "persistent kinematics-keyed cache" everywhere
  in headline sentences (keep `LECT` acronym; clarify "lifelong" is a
  property name only).
- Downgrade "15% shorter than IRIS-NP+GCS" claim to "comparable to
  IRIS-NP+GCS after equivalent post-optimisation; full like-for-like
  comparison left to future work."

### P1.2 Theorem / proposition pruning (W3)
- Demote Proposition 4 (radius_lift) to a one-line corollary inline
  with Definition 3.
- Demote Proposition 6 (env_order) to a Lemma stub inside the
  surrounding paragraph (keep one-line proof).
- Re-number remaining theorems to keep ≤ 5 numbered top-level results.

### P1.3 Numerical / unit consistency (E7, M3, A3 enforcement)
- Lock all box counts to a single canonical value (median of
  `exp6_10seed_16thr.json`); update abstract, contributions §I, and
  every headline mention; current drift: 3,547 / 3,533 / 3,468 / 3,459
  / 3,431 must collapse to one number with optional ± stdev.
- M2 table: add a second time column "deployment (gate≥0.15 rad)" so
  the 8 ms diagnostic figure cannot be misread as deployment cost.
- E5: rewrite "matches RRT-Connect after 5 queries" with explicit
  break-even arithmetic: $2.23 + n\cdot 0.279 \le n\cdot 0.157$ has no
  positive solution; correct to $\ge 18$ queries when comparing to PRM.

### P1.4 Failure-mode honesty (M5)
- 4-obstacle row: explicitly state that 4 obstacles is *not* harder
  than 16; failure is geometric (diagonal corridor narrower than the
  $10^{-4}$-rad LECT min-edge cap), not combinatorial.
- Add a one-paragraph "future mitigation" pointer: lower min-edge,
  raise box budget, or admit non-axis-aligned bridge boxes (planA in
  `/memories/repo/plan_a_non_box_bridge.md`).

### P1.5 Algorithm + figure cleanup (W4, W7, W8, W5)
- Split `Algorithm 2` into `Algorithm 2a (master scheduler)` and
  `Algorithm 2b (bridge sub-procedure)`; reduce inline comments.
- Renumber §VI grower opts from `A/U/B/C-1/C-2/D1` → `1)–5)` with
  explicit semantic labels.
- Either delete Appendix A (negative results) or quantify "longer
  final path" as `+8.4%` (verify from `path_optimize_v3.cpp` log) — if
  not measurable in 1 day, delete.
- Fig `overview` (2-link planar) → keep but add a 7-DOF IIWA14 forest
  cross-section figure as Fig 1b (use existing `fig5_scalability.png`
  asset directory if a slice render exists).

### P1.6 Editorial nitpicks (E1–E7)
| Tag | Action |
|-----|--------|
| E1 | Add citation `[lozanoperez1983spatial]` after "bottleneck shifts upstream". |
| E2 | Theorem 1 proof: insert one sentence "since $B_3(r)$ is centrally symmetric, $-u\in B_3(r)$." |
| E3 | Replace "Tight proxy" → "tight inner approximation" in `tab:channels`. |
| E4 | Sweep `~` → exact figures or `[a, b]` ranges in §III, §VI. |
| E6 | Hoist `tab:per_query` numbers into the §IX-G narrative or remove the prose claim. |

## P2 — 30-seed headline rerun

- Run `exp6_build_timing --seeds 30 --threads 16` overnight; produce
  `exp6_30seed_16thr.json`.
- Update `regen_paper_tables.py` to consume the new JSON and emit
  `tab_build.tex`, `tab_baselines_v2.tex` with mean ± 95% CI.
- Update abstract numbers from median(10) → median(30) and add CI.

## P3 — Forest box-width histogram

- Add `experiments/exp7_forest_widths` (single-binary, 30 seeds) that
  dumps a per-leaf width vector to JSON.
- Add Fig 8 "forest width distribution" (single-column, 4 panels per
  active link) to §IX-D.
- Insert one sentence in §IX-D: "Median leaf width is X rad, 90th
  percentile Y rad, confirming wrapping-effect over-approximation does
  not collapse the forest into ε-leaves."

## P4 — IRIS-NP+GCS post-opt fairness (M6)

- Wrap IRIS-NP+GCS output through the same 5-step path optimiser used
  by SBF (`path_optimize.cpp`).
- Add a new column in `tab:gcs_corridor` and a footnote in
  `tab:baselines_v2` reporting the post-opt IRIS-NP+GCS path length;
  either confirm or retract the "15% shorter" claim accordingly.

## P5 — 4-obstacle scaling sweep (M5)

- Re-run `exp_scale_obstacles` with axes (#obstacles ∈ {1,2,4,8,16},
  min-edge ∈ {1e-4, 1e-3, 5e-3}, box-budget ∈ {3k, 6k, 12k}); 5 seeds
  each.
- Replace `tab:scale` row (b) with the full sweep matrix; demonstrate
  that 4-obstacle SR>0 at smaller min-edge.

## P6 — Strengthen M1 sanity check (M4)

- Increase per-box sample count from 200 → $10^4$ and rerun
  `experiments/m1_erosion`. Expect false-accept = 0 to remain (sound
  by Theorem 2); only the FR% gap tightens slightly.
- Reframe M1 caption: "empirical sanity check (necessary condition);
  soundness is established analytically by Theorem 2."

## P7 — IRIS-ZO comparison (deferred)

- Pull `werner2024irisze` reference C++ implementation; build under
  Drake 1.31 + MOSEK.
- Run on the same Marcucci 16-obstacle scene, 10 seeds.
- Add one row to `tab:baselines_v2` and one paragraph to §IX-G.
- Estimated effort: ≥ 1 week (build system integration is the bulk).

---

## Round-2 execution order

P1.1 → P1.2 → P1.3 → P1.4 → P1.5 → P1.6 → compile EN/ZH → commit.
P2 (overnight) → update tables → recompile.
P3 → P4 → P5 → P6 in parallel where possible.
P7 deferred to second submission round.

EN edits land first; ZH mirror follows in the same commit.

---

# Round-3 Reviewer Plan (added 2026-04-24, second TRO simulated review)

Triggered by an additional reviewer-style pass that surfaced new
issues beyond Round-1/2. Items below are *additive*; if they overlap
with an earlier Round item, the Round-3 entry supersedes it.

Files (synchronised, every EN edit mirrored in ZH):
- `cpp/v6/doc/box_aabb_v6_paper_en.tex`
- `cpp/v6/doc/box_aabb_v6_paper_zh.tex`

Severity tagging: **P0** = blocking, **P1** = strongly recommended,
**P2** = polish.

## R3-P0 (no rerun unless noted)

| ID | Item | Action |
|----|------|--------|
| R3-1 | Abstract overstates SBF vs IRIS-NP comparison: "two orders of magnitude faster" without normalising 3 500 boxes vs 8 polytopes. | Rewrite the relevant abstract sentence to "for producing a comparably-connected GCS-ready cover on the same hardware (8-region IRIS-NP cover ≈602 s vs 2.23 s for the SBF backbone)". Mirror in §I contributions and §VIII.E discussion. |
| R3-2 | "Lifelong" semantics drift across abstract/§I/contribution (ii). | Insert a single bold-face one-liner near §I para 3: *Lifelong = persistent across processes and obstacle scenes, invalidated only by a DH-hash change.* Refer back to it from abstract. |
| R3-3 | §VIII.E PRM main-table SR (100 %) vs PRM sweep (93 % at 30 k) self-inconsistent. | Add an explicit footnote to `tab:baselines_v2` explaining the SR=100 % entry comes from the 5-seed × 5-query draw, while the sweep table uses 3-seed × 5-query; do not blend. |
| R3-4 | 4-obstacle failure case mentioned in abstract but absent from main text. | Either (a) move the abstract sentence into a new §X.B paragraph "Known failure mode: diagonal-corridor stress (4-obstacle scene)" with one short paragraph + cite `app:negative_results`, or (b) delete from abstract. Choose (a). |
| R3-5 | IRIS-ZO row shows Path=`--`, SR=40 % at default params; reads as cherry-picked baseline. | Add a paragraph in §VIII.E note explicitly: "We did not sweep `(num_particles, ε)` for IRIS-ZO; under default Drake settings the regions are smaller than IRIS-NP and 8 regions do not cover the corridor. A sweep to recover SR=100 % is left to future work." Mirror in `tab:baselines_v2` caption. |

## R3-P1 (no rerun)

| ID | Item | Action |
|----|------|--------|
| R3-6 | Z4 "2–2.5×" claim has no number in `tab:ablation`. | Demote claim from "key contributor" to "qualitative observation"; remove "2–2.5×" specifically and replace with "modest cold-cache speedup; warm-cache regime saturates the effect". |
| R3-7 | Notation inconsistency: $E_k(\mathcal{B})$ vs $E_e$. | Pick $E_k(\mathcal{B})$ everywhere outside Eq. (\ref{eq:pipeline}) where pipeline-index $e$ is needed; remove the apologetic footnote at first occurrence. |
| R3-8 | Definition 2 already includes $r_\ell$ dilation but Prop 3 then $\oplus B_\infty(r_\ell)$ again — double-dilation hazard. | Insert a clarifying remark after Def 2: "We write $\iAABB_\ell^{\circ}$ for the inner (radius-free) endpoint hull; $\iAABB_\ell = \iAABB_\ell^\circ \oplus B_\infty(r_\ell)$. Prop 3 operates on $\iAABB_\ell^\circ$." |
| R3-9 | "Materialisation-capacity invariant" is heap-buffer-overflow critical but not stated as a Lemma. | Promote the existing prose to `Lemma~\ref{lem:lect_invariant}` with a 4-line proof. |
| R3-10 | Tab. m2_pareto Analytical column 8170 µs vs 7559 µs (wider bin faster) is counter-intuitive and undocumented. | Add a footnote: "Median; Phase-3 BnB cost depends on # of unresolved cells, which can decrease for smoother wide boxes once monotonicity dominates." |
| R3-11 | Tab. query / Tab. gcs_corridor are single-seed | Add caption note "single representative seed; full 10-seed median consistent within ±5 % (see `exp6_10seed_16thr.json`)." |
| R3-12 | RRT-Connect "break-even ≈42 queries" arithmetic ignores path-quality gap. | Rephrase: "SBF amortises its 2.23 s build relative to PRM (12.2 s build) once ≥42 queries are issued; against RRT-Connect, the trade-off is path quality (2.87 vs 5.23 rad, deterministic) rather than wall-clock break-even." |

## R3-P2 (polish, optional)

| ID | Item |
|----|------|
| R3-13 | De-colloquialise: "hot path", "cold path", "halves" → formal phrasing. |
| R3-14 | Tighten Sec V.D Analytical 5-phase solver: collapse subsubsections, reference Algorithm-style listing. |
| R3-15 | Add an "online query state machine" figure in §IX. |
| R3-16 | Notation table: add $L_\text{act}, E_k, \nu(v), K_\text{sym}, s_{\max}, p_g, p_u$. |

## R3 deferred (requires rerun, separate round)

| ID | Item |
|----|------|
| R3-D1 | IRIS-ZO parameter sweep (≥3 (num_particles, ε) combos) until SR≥90 %. |
| R3-D2 | Apply identical 5-step + GCS post-optimiser to IRIS-NP+GCS output for like-for-like path length. |
| R3-D3 | Obstacle-density sweep N∈{8,16,32,64} with build-time scaling for SBF / IRIS-NP / IRIS-ZO. |
| R3-D4 | Panda baseline (RRT-Connect via OMPL+Drake) so §VII.G is no longer SBF-only. |
| R3-D5 | Z4 cold-cache on/off ablation (3 seeds) to back the speedup claim; or delete claim entirely (R3-6). |

## R3 execution order

R3-1 → R3-2 → R3-3 → R3-4 → R3-5  (P0, abstract + headline tables)
R3-6 → R3-7 → R3-8 → R3-9 → R3-10 → R3-11 → R3-12  (P1, theorem + tables)
R3-13 → R3-14 → R3-15 → R3-16  (P2, polish, only if time permits)
Compile EN → mirror ZH → compile ZH → grep stale numbers.

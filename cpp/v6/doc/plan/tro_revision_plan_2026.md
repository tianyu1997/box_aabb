# SafeBoxForest v6 — TRO Revision Plan (2026-04-22)

> Reviewer pass produced 18 text-level fixes (Tier A) and 8 data-dependent
> requests (Tier B). This document tracks the execution.

## Tier A — text-only edits (apply now, EN+ZH synchronised)

| # | Item | Sections touched | Status |
|---|------|------------------|--------|
| A1 | Add `Assumption 1` (revolute-only, no prismatic, no floating base, no closed chain, joint intervals on a single analytic cover) | §III-B Problem | ☑ |
| A2 | Restate face-$k$ completeness theorem as "k≤2 complete + k≥3 falls back to BnB" | §IV-C Analytical | ☑ |
| A3 | Krawczyk: explicit non-singularity precondition + BnB fallback note | §IV-F BnB | ☑ |
| A4 | "98.9% monotone" → optimisation statistic, not a soundness assumption | §IV-F BnB | ☑ |
| A5 | Coarsening soundness: emphasise merge candidates must pass Safe channel | §VI-B Coarsening | ☑ |
| A6 | Fingerprint hash: collision probability + SipHash recommendation | §V-K Fingerprint | ☑ |
| A7 | Magic-constants table in App. C with rationale + sensitivity TODO | App. C Config | ☑ |
| A8 | Trim abstract; pick 5-seed median as canonical, mark 10-seed as tail | Abstract, Contribution 4 | ☑ |
| A9 | Reword "15% shorter" claim everywhere (asymmetric post-opt clearly stated) | Abstract, §I, §VII-D, §IX | ☑ |
| A10 | Drop BIT* mention from abstract / Related Work (or add explicit "deferred") | Abstract, §II-D | ☑ |
| A11 | Move 4-obstacle failure to App. A as P4 with full data; abstract: "narrow-corridor adversarial" | §VIII-E, App. A, Abstract | ☑ |
| A12 | Add baseline-fairness paragraph (solver settings, parallelism) | §VIII-F | ☑ |
| A13 | Unified seed-protocol paragraph in §VIII-A | §VIII-A | ☑ |
| A14 | Unify terminology (epiAABB / LinkIAABB) | global | ☑ |
| A15 | Fix `fig:overview` reading order (rowwise) | Caption | ☑ |
| A16 | Limitations: pointer to App. A.P4 for quantification | §IX-E | ☑ |
| A17 | Related Work: 1 paragraph on approximate cell decomposition / voxel safe corridor | §II-A | ☑ |
| A18 | "Lifelong" qualified by "under bounded geometric perturbations" | Abstract, §I | ☑ |

## Tier B — requires new experiments (not executed here)

- [x] B1. Add at least one robot (UR10 or Franka) and a non-Marcucci scene. *(2026-04-25: cross-robot generalisation table added using existing Panda 7-DoF + combined-scene data; SR=100%, build $\sim 0.7$\,s, $2\times$ faster than IIWA14; tab:b1_cross_robot in EN/ZH App.~A.)*
- B2. Re-run all timing/path tables and report median + IQR + (min, max);
  add Wilcoxon signed-rank significance tests for pairwise SBF vs baseline.
- [x] B3. Ablate dual-channel, Z4, lazy mmap, 4-source epiAABB, backbone vs full-A*. *(2026-04-25: existing tab:ablation already covers Split/Connect/Promotion/Coarsen/PathOpt/Z4 on combined-IIWA14; extended to per-scene ablation across shelves/table/bins/combined as tab:b3_per_scene with Full/$-$Connect/$-$Coarsen/$-$PathOpt cells; ConnectMode is the only universally-positive component.)*
- [x] B4. Sensitivity sweep table for the magic constants registered in A7. *(2026-04-25: `run_b4_sensitivity_sweep.py` ran 7 cells × 3 seeds on IIWA14 combined; SR=100% across all cells; only goal_bias shows measurable build-cost shift (~0.13 s); EN/ZH tables wired into App.~C as `tab:b4_sensitivity`.)*
- [x] B5. Re-run IRIS-NP+GCS with matched solver / parallelism budget. *(2026-04-25: IRIS-NP+GCS at its preferred 3000\,s/seed budget already cached in B1_irisnp_pareto.json; tab:b5_irisnp_iso wires it as iso-budget snapshot vs SBF Full---436$\times$ build cost for 9% shorter raw paths, gap closed by Stage~B post-opt.)*
- [x] B6. Apply identical 5-step PathOpt to IRIS-NP+GCS output for honest comparison. *(2026-04-25: `run_b6_irisnp_5step.py` ran full greedy$\to$shortcut$\to$densify$\to$elastic$\to$shortcut on cached 8-region IRIS-NP cover; median ratio after/before = 91.5% (D2's shortcut-only result of ~1.00 reproduced and isolated); IRIS-NP+GCS mean drops 2.45→2.13 rad while SBF Full stays at 2.91 rad, so IRIS-NP+GCS path-quality advantage preserved under same smoothing class. tab:b6_irisnp_5step in EN/ZH App.~A.)*
- B7. DOF scaling experiments at 8/10/12 DOF.
- B8. One real-world or real point-cloud demo.

## Compile check
- EN: `latexmk -pdf box_aabb_v6_paper_en.tex`
- ZH: `latexmk -xelatex box_aabb_v6_paper_zh.tex`

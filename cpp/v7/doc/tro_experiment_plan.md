# TRO Experiment Plan for v7 SBF Paper

## Objective

This document defines the experiment framework for turning the current v7 SBF
paper into a TRO-level submission. The immediate goal is not to maximize the
number of experiments, but to make the evidence structure match the paper's
actual claims:

1. the certified envelope chain is sound;
2. the certificate pipeline is computationally justified;
3. the certified forest is useful on a demanding multi-query workload;
4. the method remains robust across scale, parameters, and scene difficulty.

The current paper already has substantial evidence. The main remaining gap is
not more baseline variety, but clearer scaling and robustness evidence.

## Evidence Functions

The experiment chapter should answer six questions, in this order:

1. Is the certificate sound in the implemented pipeline?
2. Why are the chosen endpoint and link-envelope pipelines reasonable?
3. Does the method win on the main target workload?
4. How does it compare with strong baselines on that workload?
5. How does performance change as problem scale changes?
6. What fails when the method stops working, and what role do fallback and
   post-processing actually play?

If a table or figure does not answer one of these questions, it should be
demoted to appendix material or removed.

## Recommended Section Structure

The experiment chapter should be organized around five main experiment blocks,
preceded by a short setup-and-soundness preamble. This keeps the chapter easy
to read while matching the paper's real claim structure.

### 7.1 Experimental Setup and Soundness Preamble

This opening part should be short. It is not one of the five main experiments.
Its job is only to define the reporting protocol and to clear the theoretical
entry condition before the paper moves to comparative evidence.

It should contain:

1. platform, seed policy, timeout policy, and reported metrics;
2. the rule that Marcucci is one combined 16-obstacle workload, not five
   separate scenes;
3. the rule that point-bridge paths are reported separately from certified
   corridors;
4. one compact empirical soundness check showing zero observed violations.

### 7.2 epiAABB Pipeline Comparison

This is the first main experiment block.

It should answer one question: why is the chosen endpoint-source pipeline
reasonable?

Recommended contents:

1. width-stratified epiAABB comparison;
2. IFK as the certified hot path;
3. CritSample and Analytical/GCPC as tighter advisory or profiling sources;
4. one compact conclusion on the tightness-versus-cost frontier.

Primary artifacts to reuse:

1. epiaabb pipeline outputs.

### 7.3 Link-Envelope Pipeline Comparison

This is the second main experiment block.

It should answer one question: why is the selected link-envelope representation
the right default for the planner?

Recommended contents:

1. LinkIAABB subdivision sweep;
2. LinkIAABB-Grid and Hull16-Grid comparison;
3. one compact statement tying envelope tightness, runtime, and the chosen
   default representation.

Primary artifacts to reuse:

1. link envelope pipeline outputs.

### 7.4 End-to-End Baseline Comparison

This is the third and most important experiment block. It should carry the main
performance claim of the paper.

It should be anchored on the Marcucci combined workload and should contain:

1. offline build cost and online query cost;
2. success rate and path-quality reporting across the canonical query pairs;
3. SBF versus OMPL baselines already available;
4. IRIS-NP fairness tables when discussing path quality;
5. fallback usage frequency as a reporting boundary, not as the main mechanism
   of success.

Primary artifacts to reuse:

1. generated combined baseline tables;
2. Marcucci combined JSON outputs;
3. IRIS fairness tables.

### 7.5 Generality Experiments

This is the fourth main experiment block.

It should answer one question: does the method remain useful when the planning
context changes?

Recommended contents:

1. stress-scene rows such as iiwa14_clutter and iiwa14_narrow;
2. cross-robot transfer rows such as Panda, if the data remain credible;
3. DoF scaling as the highest-priority new run.

This block should not become a zoo of disconnected examples. Every row should
support the same generality claim: robustness across scene structure, robot
identity, or configuration-space dimensionality.

### 7.6 Scatter / Sensitivity Experiments

This is the fifth main experiment block.

Here, "scatter" should be understood as parameter-scatter or sensitivity-style
evidence rather than as another end-to-end benchmark. Its role is to show how
performance moves when the core knobs are perturbed.

Recommended contents:

1. sensitivity to depth cap;
2. sensitivity to goal bias;
3. sensitivity to subdivision level $S$;
4. sensitivity to bridge budget;
5. one compact component ablation table if needed;
6. a short failure-mode paragraph tying bad cells to disconnected certified
   corridors, conservative envelopes, or budget exhaustion.

This final block should stay compact. It exists to show robustness and failure
boundaries, not to open a second benchmark chapter.

## Detailed Chapter Outline and Asset Mapping

The following outline is the recommended direct rewrite target for the
experiment chapter. Each subsection is mapped to the current available tables
or figures, so that the first rewrite pass can proceed without waiting for new
runs.

### 7.1 Experimental Setup and Soundness Preamble

#### 7.1.1 Setup and reporting rules

Purpose:

1. define scenes, seeds, metrics, timeout policy, and the meaning of
   `used_point_bridge`;
2. state that Marcucci is one combined workload.

Existing assets:

1. no dedicated table required;
2. optional sanity figure: the currently used 2-DoF forest figure from the
   main paper.

#### 7.1.2 Empirical soundness check

Purpose:

1. show that the implemented certificate chain remains consistent with the
   theorem.

Existing assets:

1. tab_soundness.tex.

Recommended use:

1. one short paragraph;
2. one compact table;
3. no extra figure.

### 7.2 epiAABB Pipeline Comparison

#### 7.2.1 Width-stratified endpoint-source comparison

Purpose:

1. compare IFK, CritSample, Analytical, GCPC, and MC under the same width-bin
   protocol;
2. justify IFK as the certified default and the others as advisory/profiling
   references.

Existing assets:

1. tab_epiaabb_pipeline.tex.

Recommended use:

1. keep this as one table;
2. do not split into multiple subfigures unless later needed.

### 7.3 Link-Envelope Pipeline Comparison

#### 7.3.1 LinkIAABB subdivision sweep

Purpose:

1. show the effect of subdivision on envelope tightness and cost.

Existing assets:

1. tab_link_envelope_pipeline.tex.

#### 7.3.2 Grid and hull variants

Purpose:

1. compare LinkIAABB-Grid against Hull16-Grid;
2. support the final default used in the main workload.

Existing assets:

1. tab_link_envelope_pipeline.tex.

Recommended use:

1. keep both 7.3.1 and 7.3.2 under the same table if page budget is tight;
2. the text should extract one conclusion only: why `link_iaabb_grid` with the
   chosen `S` is the paper default.

### 7.4 End-to-End Baseline Comparison

#### 7.4.1 Anchor workload sanity check

Purpose:

1. briefly show that the implementation behaves cleanly on easy anchor scenes;
2. keep this subordinate to the main Marcucci story.

Existing assets:

1. tab_main.tex;
2. optional 2-DoF forest figure already referenced in the main paper.

#### 7.4.2 Build-side throughput and parallelism

Purpose:

1. report offline build performance and thread scaling.

Existing assets:

1. tab_threads.tex;
2. tab_main.tex for anchor build/query timing.

#### 7.4.3 Marcucci online query performance

Purpose:

1. report per-query-pair online performance under the combined-scene protocol.

Existing assets:

1. tab_query.tex.

#### 7.4.4 Main baseline table on the combined workload

Purpose:

1. compare SBF with OMPL and IRIS-style competitors on the main target task.

Existing assets:

1. tab_combined_baselines.tex.

#### 7.4.5 Stress-scene baseline rows

Purpose:

1. show whether the comparative story survives harder non-Marcucci scenes.

Existing assets:

1. tab_newscenes.tex;
2. tab_prm_sweep.tex if node-count sensitivity is kept in the main paper.

#### 7.4.6 Implicit-set fairness comparison

Purpose:

1. separate path-quality fairness from raw runtime comparison.

Existing assets:

1. tab_irisnp_iso.tex;
2. tab_irisnp_5step.tex.

Recommended use:

1. if page budget tightens, 7.4.5 and 7.4.6 can be compressed into one
   subsection titled "Additional baseline evidence".

### 7.5 Generality Experiments

#### 7.5.1 Cross-scene robustness

Purpose:

1. show transfer to scenes with different connectivity structure.

Existing assets:

1. tab_newscenes.tex.

#### 7.5.2 Cross-robot transfer

Purpose:

1. show that the method is not tied to IIWA14 only.

Existing assets:

1. tab_panda.tex.

#### 7.5.3 DoF scaling

Purpose:

1. provide the strongest missing generality evidence for TRO;
2. show how build cost, query cost, and structural size grow with active DoF.

Existing assets:

1. none yet;
2. this should be implemented as a new figure and, if needed, one companion
   summary table.

Recommended new outputs:

1. fig_dof_scaling_build_query.pdf;
2. tab_dof_scaling.tex.

### 7.6 Scatter / Sensitivity Experiments

#### 7.6.1 Parameter sensitivity

Purpose:

1. show how the core hyperparameters move latency, success, and path quality.

Existing assets:

1. tab_b4_sensitivity.tex.

Recommended new output if presentation is too dense:

1. fig_hparam_scatter.pdf or fig_hparam_heatmap.pdf.

#### 7.6.2 PathOpt contribution

Purpose:

1. show that PathOpt is useful but not the main source of feasibility.

Existing assets:

1. tab_pathopt.tex.

#### 7.6.3 Component ablation and failure modes

Purpose:

1. isolate the role of connect/coarsen/PathOpt;
2. explain what fails when success drops.

Existing assets:

1. tab_ablation.tex.

Recommended use:

1. keep one short failure-mode paragraph in prose;
2. do not create a separate failure table unless current results show a real
   pattern that cannot be described compactly.

## Reuse-First Policy

Before planning new experiments, absorb the current generated artifacts into the
new structure.

Existing assets that should be reused first:

1. tab_combined_baselines.tex
2. tab_irisnp_iso.tex
3. tab_irisnp_5step.tex
4. tab_b4_sensitivity.tex
5. soundness outputs
6. epiAABB pipeline outputs
7. link-envelope pipeline outputs
8. Marcucci combined and ablation JSON outputs

The first rewrite pass of the experiment chapter should use these materials
without waiting for any additional runs.

## New Experiments Needed

Only experiments that close a real evidence gap should be added.

### Priority A: DoF Scaling

This is the most valuable new experiment.

Goal:

1. show how offline build cost, online query cost, and box count evolve with
   active DoF;
2. support a scaling discussion in a way that baseline-only tables cannot.

Recommended design:

1. fixed obstacle workload;
2. active DoF in {2, 4, 6, 7, 14} if the robot model and scene tooling allow;
3. report build time, query time, box count, and success rate together;
4. use the same envelope default as the main Marcucci setting when possible.

Minimal acceptable variant:

1. use the iiwa14_far family or a fixed IIWA scene with progressively activated
   joints;
2. 5 seeds per point are enough for an initial figure;
3. plot medians, not only means.

### Concrete DoF-Scaling Protocol

The DoF-scaling experiment should be treated as the execution core of the
"Generality experiments" block. It must be simple enough to run soon, but rich
enough to support one clear scaling figure.

#### Scene

Recommended default scene:

1. iiwa14_far.

Reason:

1. the scene already exists;
2. it is easier to keep fixed while varying the active-joint set;
3. it is hard enough to avoid trivial straight-line behaviour, but cheap enough
   to support multiple seeds and DoF points.

Optional second scene after the first figure succeeds:

1. iiwa14_narrow.

This second scene should only be added if the first figure is stable and page
budget allows a companion robustness sentence.

#### DoF Points

Recommended active-DoF points:

1. 2
2. 4
3. 6
4. 7
5. 14

Interpretation:

1. for points below 14, freeze the inactive joints at the nominal scene start
   posture or another fixed reference posture;
2. run the same planner pipeline on the reduced active subspace;
3. keep obstacle geometry unchanged so the scaling reflects configuration-space
   dimensionality rather than a different scene family.

Fallback if tooling cannot support the full set immediately:

1. 2
2. 4
3. 7
4. 14

#### Seeds

Recommended seed count:

1. 5 seeds per DoF point for the first paper-quality figure.

If the variance is visibly high:

1. raise only the 7-DoF and 14-DoF points to 10 seeds;
2. keep 2-DoF and 4-DoF at 5 seeds.

#### Planner configuration

Use the same main default as the paper workload whenever possible:

1. env = link_iaabb_grid
2. n-sub = 4
3. FFB depth = 55
4. max boxes = 2500
5. point bridge policy explicitly reported

The point of the figure is not to retune for every DoF, but to show how the
same design behaves as dimensionality changes.

#### Output fields

Each trial JSON row should contain at least the following fields:

1. scene
2. active_dof
3. seed
4. success
5. used_point_bridge
6. n_boxes
7. n_islands
8. grow_time_ms
9. path_find_time_ms
10. opt_time_ms
11. total_time_ms
12. raw_length
13. opt_length

The summary JSON for each DoF point should report:

1. success_rate
2. point_bridge_rate
3. median_n_boxes
4. median_grow_time_ms
5. median_total_time_ms
6. median_opt_length
7. median_n_islands

#### Final figure and table

The target deliverables should be:

1. one figure with three aligned panels:
   build time versus DoF;
   total query time versus DoF;
   box count versus DoF.
2. one compact table with success rate and point-bridge rate.

Suggested names:

1. fig_dof_scaling_build_query.pdf
2. tab_dof_scaling.tex

#### Interpretation target

This experiment should support exactly three sentences in the paper:

1. how the structural size grows with DoF;
2. how build/query cost changes with DoF;
3. whether feasibility increasingly depends on fallback at high DoF.

If the figure cannot support all three, the protocol should be simplified and
rerun rather than padded with extra tables.

### Priority B: Sensitivity Visualization

If the current sensitivity table already contains enough structure, it should be
converted into a compact figure rather than rerun.

Primary knobs to surface:

1. depth cap;
2. goal bias;
3. subdivision level S;
4. bridge budget.

Only rerun this experiment if the existing table is too sparse to support a
clear visual summary.

### Priority C: Failure-Mode Summary

This may not require new runs.

Required deliverable:

1. a short paragraph classifying failure into one of the following buckets:
   disconnected certified corridor, envelope over-conservatism, or budget/time
   exhaustion.

If all relevant main-workload cells already succeed, say so explicitly and move
the failure discussion to stress-scene rows.

## Optional Stretch Experiments

These are useful only after the paper already has a coherent TRO evidence arc.

### Obstacle Density Scaling

Use one scene family with varying obstacle counts. This strengthens the claim
that the method's regime of advantage is understood, but it is secondary to DoF
scaling.

### Multi-region IRIS or GCS Baseline

This is a strong comparison if implemented well, but it requires additional
development and should not block the paper rewrite.

## What to Demote or Cut

To stay within a TRO page budget, the following material should not dominate the
main chapter.

1. 2-DoF results beyond sanity checking.
2. Multiple near-duplicate easy-scene tables.
3. Long descriptions of quick/full runner mechanics.
4. Baseline permutations that do not change the main conclusion.

If space becomes tight, cut narrative duplication before cutting core evidence.

## Mapping From Evidence to Current Assets

The following mapping should guide the rewrite.

1. Soundness -> existing soundness outputs.
2. Endpoint-source justification -> epiaabb pipeline.
3. Link-envelope justification -> link envelope pipeline.
4. Main workload superiority -> Marcucci combined + combined baselines.
5. Path-quality fairness -> IRIS iso-budget + IRIS 5-step tables.
6. Component importance -> current ablation and PathOpt-step outputs.
7. Parameter robustness -> tab_b4_sensitivity.tex.
8. Scaling credibility -> new DoF-scaling experiment.

## Execution Order

Recommended order of work:

1. Rewrite the experiment chapter around the new six-block structure using only
   current artifacts.
2. Identify exactly where the narrative still lacks evidence.
3. Run DoF scaling and integrate it as the primary new figure.
4. Reuse or lightly rerun sensitivity if the chapter still lacks robustness
   evidence.
5. Add a short failure-mode paragraph.
6. Only then decide whether density scaling or multi-region GCS is worth the
   remaining time budget.

## TRO Readiness Criterion

The experiment package should be considered TRO-ready when all of the following
are true:

1. the main workload is clearly Marcucci combined and is supported by strong
   baseline evidence;
2. soundness is shown once, early, and cleanly;
3. the geometric pipeline choices are justified by compact profiling evidence;
4. robustness is supported by at least one genuine scaling result and one
   parameter or failure analysis result;
5. the experiment chapter reads as one argument rather than as a bag of runs.

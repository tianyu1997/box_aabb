Experiments
===========

This file is an English prose skeleton for the experiment chapter. It follows
the agreed five-block structure and is written so that each subsection can be
ported directly into the paper draft with minimal restructuring.


7.1 Experimental Setup and Soundness Preamble
---------------------------------------------

### 7.1.1 Setup and reporting discipline

First cite: none. Optional opening visual anchor: Fig. \ref{fig:2dof_forest}.

Paragraph skeleton:

We evaluate SBF in the same order as the method itself develops. We first
verify that the implemented certificate remains sound, then justify the chosen
endpoint and link-envelope pipelines, and finally evaluate the full planning
system on the target multi-query workload together with generality and
sensitivity evidence. Unless otherwise noted, we report success rate, wall
time, and path length over repeated random seeds. For the Marcucci study, the
five shelf-to-shelf queries are treated as one combined 16-obstacle workload
rather than as independent scenes. Whenever a final feasible path requires the
point-level bridge fallback, that event is reported explicitly and is not
counted as part of the reusable certified corridor.

### 7.1.2 Empirical soundness check

First cite: \Cref{tab:soundness}.

Paragraph skeleton:

Before turning to comparative performance, we verify that the implemented
certificate chain remains consistent with the theory. For each tested scene, we
build an SBF forest, sample configurations uniformly from the accepted
certified boxes, and recheck every sample with the same per-configuration
collision predicate used by the baselines. \Cref{tab:soundness} reports zero
observed violations over all tested scenes. This experiment does not replace
the proof of soundness; instead, it confirms that the concrete implementation
choices, including radius inflation, lazy materialisation, and packed AABB
storage, do not break the intended certificate semantics.


7.2 epiAABB Pipeline Comparison
-------------------------------

### 7.2.1 Width-stratified endpoint-source comparison

First cite: \Cref{tab:epiaabb_pipeline}.

Paragraph skeleton:

The first main experiment block studies the endpoint-source pipeline in
isolation. For each of four width bins, we draw 400 random IIWA14 joint boxes
and evaluate IFK, CritSample, Analytical, GCPC, and a width-proportional Monte
Carlo reference calibrated at $2\times 10^6$ samples for geometric-mean width
$\bar{w}=0.35$ rad. \Cref{tab:epiaabb_pipeline} shows that this restored MC
protocol materially changes the baseline cost while preserving the qualitative
frontier: IFK remains the certified hot path at about 2.1--2.5~$\mu$s mean,
CritSample stays at 4.0--10.7~$\mu$s as a cheap advisory source, and
Analytical/GCPC deliver the tightest endpoint boxes at a much higher offline
cost of roughly 4.8--7.1~ms and 4.1--6.2~ms respectively.

### 7.2.2 Interpretation of the endpoint-source frontier

First cite: \Cref{tab:epiaabb_pipeline}.

Paragraph skeleton:

This table should be read as an upstream design justification rather than as a
planner leaderboard. The main conclusion is now sharper with the canonical MC
rerun: GCPC matches Analytical tightness almost exactly while cutting mean
runtime from roughly 4.8--7.1~ms to 4.1--6.2~ms, whereas the dense MC
reference itself grows from 31.6~ms to 511.5~ms as the boxes widen. That leaves
the design roles well separated: IFK is the only practical certified online
default, CritSample is a cheap non-certified probe, and the
Analytical/GCPC family is best reserved for offline profiling, cache-backed
diagnostics, or reference-quality tightness checks.


7.3 Link-Envelope Pipeline Comparison
-------------------------------------

### 7.3.1 LinkIAABB subdivision sweep

First cite: \Cref{tab:link_envelope_pipeline}.

Paragraph skeleton:

The second experiment block moves one layer downstream and studies the link
envelope itself. Keeping the endpoint source fixed to CritSample and reusing
the same four width-stratified IIWA14 intervals as Exp.~1 (400 boxes per bin,
1600 total), we first sweep the LinkIAABB subdivision level and measure the
resulting end-to-end quality-cost trade-off. \Cref{tab:link_envelope_pipeline}
shows a clear knee at $S=4$: the volume ratio drops from 100.0\% to 81.7\%,
whereas moving to $S=8$ buys only another 2.7 percentage points of tightening
at essentially the same total runtime (4.7~$\mu$s versus 4.9~$\mu$s for
$S=1$). This experiment explains why the paper does not treat the AABB
representation as a single fixed object: the subdivision parameter $S$ is a
genuine geometric knob that changes the certified corridor quality available to
the planner.

### 7.3.2 Grid and hull envelope variants

First cite: \Cref{tab:link_envelope_pipeline}.

Paragraph skeleton:

We then compare the subdivided AABB family with the grid- and hull-based
variants used by the final planner. The same table supports the second design
decision of this block: how much extra slack a raster cache introduces, and
which raster variant dominates once that cost is paid. At $\delta=0.04$ m,
LinkIAABB-Grid relaxes from the exact $S=4$ ratio of 81.7\% to 92.0\% while
running at 19.7~$\mu$s with about 4.0k occupied voxels, whereas Hull16-Grid
tightens further to 67.4\% at 25.3~$\mu$s with about 3.0k voxels. The finer
$\delta=0.02$ m sweep is tighter still, but it costs 66.9--76.3~$\mu$s and is
no longer competitive for the online planner. The stable design takeaway is to
keep $S=4$ as the non-grid default and prefer Hull16-Grid at moderate
resolution whenever a raster-cache link envelope is required.

### 7.3.3 Pipeline takeaway

First cite: \Cref{tab:link_envelope_pipeline}.

Paragraph skeleton:

Taken together, the endpoint and link-envelope experiments justify the
geometric defaults used in the remainder of the paper. The endpoint layer is
chosen to preserve certification and hot-path throughput, while the link layer
keeps $S=4$ as its non-grid subdivision default and prefers Hull16-Grid over
LinkIAABB-Grid whenever a voxelised cache is required. The end-to-end
experiments that follow should therefore be interpreted as operating on a
pipeline whose main geometric trade-offs have already been exposed explicitly.


7.4 End-to-End Baseline Comparison
----------------------------------

### 7.4.1 Anchor build and thread scaling

First cite: \Cref{tab:main}. Secondary cite: \Cref{tab:threads}.

Paragraph skeleton:

We begin the end-to-end evaluation with two anchor workloads that establish the
basic operating regime of the implementation. \Cref{tab:main} reports the
complete build-query pipeline on the 2-DoF and IIWA anchor scenes, while
\Cref{tab:threads} isolates the build-side thread scaling of the forest
construction stage. These results are not the main evidence of superiority;
their role is to show that the implementation is stable, that build and query
cost are already moderate in simple regimes, and that the snapshot/transplant
design still exposes useful parallelism in the current code path.

### 7.4.2 Marcucci online query breakdown

First cite: \Cref{tab:query}.

Paragraph skeleton:

The main online-query study follows the Marcucci cabinet protocol in its
combined-scene form: one 16-obstacle environment, one reusable build, and five
canonical shelf-to-shelf query pairs. \Cref{tab:query} reports the per-query
breakdown under this protocol. This table should be read as the core evidence
that the certified forest is useful as a multi-query planning object: once the
forest has been built, later queries are solved by corridor retrieval and local
refinement rather than by reconstructing free-space structure from scratch.

### 7.4.3 Combined-scene baseline comparison

First cite: \Cref{tab:baselines_v2}.

Paragraph skeleton:

The main baseline comparison is performed on the same combined Marcucci
workload. \Cref{tab:baselines_v2} uses the current v7 SBF cached-query row and
reuses the frozen v6 comparison rows for IRIS-NP+GCS, IRIS-ZO+GCS,
RRT-Connect, and PRM. The historical SBF+GCS post row stays out of the table
because the current v7 post-processing path is intentionally excluded from the
paper comparison set.


7.5 Generality Experiments
--------------------------

### 7.5.1 Cross-scene robustness

First cite: \Cref{tab:newscenes}.

Paragraph skeleton:

The next block asks whether the method remains useful when the scene structure
changes. \Cref{tab:newscenes} reports harder stress scenes rather than the easy
anchor cases used only for sanity checking. The purpose of this table is not to
open another baseline chapter, but to show that the end-to-end story survives
outside the single canonical cabinet workload and remains meaningful when the
connectivity structure of the scene changes.

### 7.5.2 Cross-robot transfer

First cite: \Cref{tab:panda}.

Paragraph skeleton:

We then examine whether the same implementation remains credible across robot
models. \Cref{tab:panda} provides the current cross-robot transfer evidence.
This table should be discussed briefly and conservatively: its purpose is to
show that SBF is not tied to IIWA14 alone, not to claim full robot-agnosticity
from a single additional platform.

### 7.5.3 DoF scaling

First cite: none among current paper tables. New outputs required:
`fig_dof_scaling_build_query.pdf` and `tab_dof_scaling.tex`.

Paragraph skeleton:

The strongest missing generality evidence is explicit dimensional scaling. In
this experiment we keep the obstacle workload fixed and vary the active
configuration-space dimensionality, reporting build cost, total query cost,
forest size, and success rate together. The point of this figure is not to
retune the planner for each dimensionality, but to expose how the same certified
forest construction behaves as the configuration space expands. This subsection
should become the main generality result of the paper because it provides a
clearer TRO-style scaling argument than any additional collection of ad hoc
scene cases.


7.6 Scatter / Sensitivity Experiments
-------------------------------------

### 7.6.1 Parameter sensitivity

First cite: \Cref{tab:b4_sensitivity}.

Paragraph skeleton:

The final experiment block studies how the main performance metrics scatter as
the core planner parameters are perturbed. \Cref{tab:b4_sensitivity} should be
used to summarise the sensitivity to depth cap, goal bias, subdivision level,
and bridge budget. This subsection should remain compact. Its function is to
show that the reported performance is not an artifact of one fragile parameter
setting, not to introduce another benchmark competition.

### 7.6.2 PathOpt contribution

First cite: \Cref{tab:pathopt}.

Paragraph skeleton:

We next isolate the contribution of the local corridor-refinement stage.
\Cref{tab:pathopt} shows how much path quality is gained by the successive
cleanup steps applied after certified corridor retrieval. The intended reading
is narrow: PathOpt improves path quality, but it is not the main source of
feasibility. This matches the methodological stance of the paper, where query
success is primarily inherited from the certified corridor rather than created
by a heavy post-optimiser.

### 7.6.3 Component ablation and failure boundaries

First cite: \Cref{tab:ablation}.

Paragraph skeleton:

Finally, \Cref{tab:ablation} isolates the role of the main system components.
This table should support a short failure-boundary discussion rather than a long
catalogue of implementation cases. The main question is where performance is
lost when a component is removed: in corridor connectivity, in cover
compactness, or only in path quality. If the hardest cells still succeed, the
text should say so directly; if failure appears, it should be classified into a
small number of causes, such as disconnected certified corridors, envelope
over-conservatism, or budget exhaustion.


DoF-Scaling Protocol
--------------------

This section records the concrete executable design for the new DoF-scaling
experiment so that the future runner and the future paragraph in Section 7.5.3
use the same protocol.

### Scene

Default scene: `iiwa14_far`.

Reason:

1. it already exists;
2. it is stable enough for repeated seeds;
3. it avoids the extra variance of the narrow and clutter cases for the first
   scaling figure.

Optional second scene after the first figure succeeds: `iiwa14_narrow`.

### Active DoF points

Recommended points:

1. 2
2. 4
3. 6
4. 7
5. 14

Fallback set if tooling is easier initially:

1. 2
2. 4
3. 7
4. 14

Inactive joints should be frozen at a fixed reference posture so that only the
configuration-space dimensionality changes.

### Seeds

Recommended default: 5 seeds per DoF point.

If the 7-DoF or 14-DoF points show large variance, raise only those points to
10 seeds rather than expanding the entire sweep.

### Planner configuration

Use the main paper defaults whenever possible:

1. `env=link_iaabb_grid`
2. `n-sub=4`
3. `ffb-depth=55`
4. `max-boxes=2500`
5. point-bridge usage recorded explicitly

### Trial-level output fields

Each JSON trial row should contain at least:

1. `scene`
2. `active_dof`
3. `seed`
4. `success`
5. `used_point_bridge`
6. `n_boxes`
7. `n_islands`
8. `grow_time_ms`
9. `path_find_time_ms`
10. `opt_time_ms`
11. `total_time_ms`
12. `raw_length`
13. `opt_length`

### Summary fields per DoF point

Each aggregated DoF-level summary should contain:

1. `success_rate`
2. `point_bridge_rate`
3. `median_n_boxes`
4. `median_grow_time_ms`
5. `median_total_time_ms`
6. `median_opt_length`
7. `median_n_islands`

### Final outputs

Required outputs:

1. `fig_dof_scaling_build_query.pdf`
2. `tab_dof_scaling.tex`

The figure should contain three aligned panels:

1. build time versus active DoF;
2. total query time versus active DoF;
3. box count versus active DoF.

The table should report:

1. success rate;
2. point-bridge rate.

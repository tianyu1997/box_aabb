# v7 Paper Experiments — five-block layout

The experiment section of the v7 paper is organised into five blocks. The
first four are stable retained runs; the fifth is the new manifest-backed DoF
scaling study. Each numbered script here corresponds to one paper experiment
block. Shared code lives in `common.py` and is not a paper experiment.

## Mapping from paper section to script

| Paper part | Script | Environment | Main output |
|---|---|---|---|
| 1. epiAABB 管线对比 | `01_epiaabb_pipeline.py` | Random IIWA14 joint intervals, width-stratified like v6 | `results_paper/epiaabb_pipeline.json` |
| 2. link envelope 管线对比 | `02_link_envelope_pipeline.py` | Same width-stratified IIWA14 intervals | `results_paper/link_envelope_pipeline.json` |
| 2.5 Marcucci build 端包络管线对比 | `02_5_marcucci_envelope_build.py` | **Marcucci combined scene only**; `IFK/CritSample × {AABB S=4, AABB-grid S=4 d=0.04, Hull16-grid d=0.04}` with cold/warm LECT cache passes | `results_paper/marcucci_envelope_build.json`, `results_paper/marcucci_envelope_build/raw/*.json` |
| 3. E2E 规划与 baseline 对比 | `03_e2e_baselines_combined.py` | **Marcucci combined 16-obstacle scene only**; five canonical query pairs are one workload | `results_paper/marcucci_combined.json` |
| 4. 消融/仿射实验 | `04_ablation_combined.py` | **Marcucci combined scene only**; one parameter varied per row | `results_paper/ablation/*.json` |
| 5. Generality / DoF scaling | `05_generality_dof_scaling.py` | Manifest-backed per-DoF scene configs | `results_paper/dof_scaling.json`, `doc/generated/tab_dof_scaling.tex`, `doc/figures/fig_dof_scaling_build_query.pdf` |

## Important policy

- Exp. 1 follows the v6 paper protocol for WIDTH-STRATIFIED ENDPOINT-SOURCE COMPARISON: four width bins, five endpoint sources, and a width-proportional MC baseline calibrated at geometric-mean width `0.35 rad` rather than a fixed per-box MC sample count.
- In v7, the GCPC row is emitted from a v7-local cache asset. The default runner now auto-discovers `cpp/v7/data/iiwa14_5000.gcpc` when present and never reaches back into `cpp/v6/`.
- Marcucci is never reported as shelves/bins/table split scenes in the main
  paper.  The five query JSONs under `configs/marcucci/` are treated as one
  combined-scene workload.
- Exp. 2.5 uses the same main Marcucci build configuration defaults as Exp. 3
  (`threads=1`, `ffb-depth=55`, `max-boxes=2500`, `bridge-boxes=2000`,
  quick/full seeds and timeout) and changes only endpoint source, link envelope
  type, and LECT cache state.
- The E2E runner enables the point-level bridge fallback by default and records
  `used_point_bridge` per query.  The `no_point_bridge` ablation row disables it
  to measure pure certified box-corridor connectivity.
- The current Exp. 3 main table retains only the native SBF row.  The
  historical SBF+GCS post row is excluded pending collision-safe validation,
  and OMPL rows are not produced by the paper runner.
- The current C++ experiment binaries do not yet expose a native `active_dof`
  or joint-mask CLI. The DoF-scaling runner therefore consumes a manifest of
  prepared scene JSONs; see `dof_scaling_manifest.json`.
- Low-value toy clutter/narrow scenes are excluded from the paper experiments.
- v6 microbenchmarks not covered by these retained parts are removed to keep the
  manuscript under 20 pages.

## Usage

Build the C++ experiment binaries first:

```bash
cd cpp/v7
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release
cmake --build build-release --target exp_epiaabb_pipeline exp_link_envelope_pipeline \
  exp_marcucci_cached exp_marcucci_combined baseline_ompl -j
```

Paper experiment wrappers default to `build-release` and refuse Debug CMake
trees unless `--allow-debug-build` is passed explicitly. Use Debug only for
gdb/stack traces/logic debugging, never for timing numbers.

Run a smoke test:

```bash
python3 experiments/paper/run_all.py --quick
```

Run only Exp. 1 with the paper MC protocol:

```bash
python3 experiments/paper/01_epiaabb_pipeline.py --full
```

Override the MC calibration explicitly:

```bash
python3 experiments/paper/01_epiaabb_pipeline.py \
  --full \
  --ref-samples 2000000 \
  --min-samples 1000 \
  --max-samples 10000000
```

Run paper mode:

```bash
python3 experiments/paper/run_all.py --full
```

Opt into the manifest-backed DoF scaling block:

```bash
python3 experiments/paper/run_all.py --full --include-dof-scaling
```

Write or refresh the default DoF-scaling manifest template:

```bash
python3 experiments/paper/05_generality_dof_scaling.py \
  --write-template experiments/paper/dof_scaling_manifest.json
```

The legacy anytime-baseline flag is now a compatibility no-op:

```bash
python3 experiments/paper/03_e2e_baselines_combined.py --full --include-anytime
```

## Lifelong Cache Workflow

The paper-level experiment blocks stay focused on the retained evidence tables,
but the same directory now also exposes a reproducible scene-reuse entry path
for the lifelong LECT cache claim.

Run the default same-robot scene sequence:

```bash
python3 experiments/paper/lifelong_cache_reuse.py --quick
```

Specify your own cache path and scene order:

```bash
python3 experiments/paper/lifelong_cache_reuse.py \
  --full \
  --cache-path experiments/results_paper/cache_reuse/iiwa14_shared.bin \
  --scenes iiwa14_far iiwa14_narrow
```

This wrapper calls `exp_main` once per scene, keeps one persisted
`--lect-cache` path across processes, and writes an aggregate
`results_paper/cache_reuse.json` summary together with one JSON per scene.

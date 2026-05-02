# Reproducibility Guide

This guide defines the expected open-release workflow for reproducing the TRO
submission experiments from the SafeBoxForest v6 tree.

## Scope

The source repository contains the implementation, experiment wrappers, frozen
small references, paper source, and generated table/figure infrastructure. Large
regenerated outputs under `experiments/results_paper/` are not part of the light
source tree by default. Publish curated paper result JSONs as a release artifact
or archival deposit and document their checksum in `RELEASE_MANIFEST.md`.

## Reference environment

The paper-facing runs were designed around one workstation running one heavy
experiment at a time.

| Component | Reference or minimum |
| --- | --- |
| OS | Ubuntu 20.04 primary; newer Linux distributions may work |
| CPU | 8 logical paper threads, affinity `0-7` |
| RAM | 32 GB recommended for full runs |
| CMake | 3.20 or newer |
| C++ compiler | C++20, GCC 11+, Clang 14+, or MSVC 2022 |
| Python | 3.10+ for bindings and experiment wrappers |
| Optional | Drake for IRIS/GCS paths, OMPL for PRM/BIT* baselines |

Use the same machine and avoid parallel experiment groups when comparing timing
numbers. Different CPUs can reproduce success/failure and table structure but
will not match wall-clock timings exactly.

## Build configurations

Core build:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j "$(nproc)"
ctest --test-dir build --output-on-failure
```

Python-enabled build:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DSBF_WITH_PYTHON=ON
cmake --build build -j "$(nproc)"
export PYTHONPATH="$PWD/python:$PWD/build/python:${PYTHONPATH:-}"
```

Optional baseline build variants:

```bash
cmake -S . -B build-drake -DCMAKE_BUILD_TYPE=Release -DSBF_WITH_DRAKE=ON -DSBF_WITH_PYTHON=ON
cmake -S . -B build-ompl  -DCMAKE_BUILD_TYPE=Release -DSBF_WITH_OMPL=ON  -DSBF_WITH_PYTHON=ON
```

Drake and OMPL are optional for the core library. Without them, SBF-only tests
and many smoke paths remain available, but IRIS/GCS and OMPL baseline rows
cannot be regenerated locally.

## Paper experiment entrypoints

Paper-facing wrappers live in `experiments/paper/` and should be run from the
repository root unless a script says otherwise.

| Experiment | Entrypoint | Main output |
| --- | --- | --- |
| Exp.1 endpoint pipeline | `01_epiaabb_pipeline.py` | `experiments/results_paper/epiaabb_pipeline.json` |
| Exp.2 link envelopes | `02_link_envelope_pipeline.py` | `experiments/results_paper/link_envelope_pipeline.json` |
| Exp.3 cache replay | `03_marcucci_envelope_build.py` | `experiments/results_paper/marcucci_envelope_build.json` |
| Exp.4 SBF row | `04_e2e_baselines_combined.py` | `experiments/results_paper/marcucci_combined.json` |
| Exp.4 baselines | `04_baselines_marcucci.py` | baseline JSONs under `experiments/results_paper/` |
| Exp.5 random robots | `05_random_robot_scenes.py` | `experiments/results_paper/exp5_random_robot_scenes.json` |
| Exp.6 obstacle updates | `06_sbf_obstacle_rebuild.py` | `experiments/results_paper/exp6_sbf_obstacle_rebuild.json` |

The section-order orchestrator is:

```bash
python experiments/paper/run_all.py --quick --update-paper-results
```

Use `--full` only after confirming the optional dependencies and runtime budget.

## Result regeneration

Generated paper snippets are refreshed with:

```bash
python experiments/paper/07_update_paper_results.py
python experiments/paper/09_generate_paper_figures.py
```

The generated LaTeX files live under `doc/paper/SBF/generated/`. The current
main paper source is `doc/paper/SBF/main.tex`.

## Timing and success metrics

- Use wall-clock time from monotonic timers for reported build/query timings.
- Count timeout, invalid path, internal exception, and no-corridor outcomes as
  failures unless a table explicitly reports a diagnostic subcategory.
- Report path length only for successful validated runs.
- Record seeds, robot, scene, method, build options, cache state, thread budget,
  and optional dependency availability with each result JSON.
- Keep SBF build time, SBF cached query time, local connector time, and final
  validation time separate when the script exposes them.

## Cache and output policy

- LECT and runtime cache files are local artifacts and are ignored by the source
  tree.
- `experiments/results_paper/` is ignored in the lightweight source release.
- Curated paper result JSONs should be shipped as a separate release artifact,
  for example `safeboxforest-v6.0.0-tro-results.zip`.
- Remove smoke directories, scratch logs, `.bak` files, and transient raw dumps
  before publishing the result artifact.

## Paper compile check

After regenerating tables and figures, compile the main TRO source from its
directory:

```bash
cd doc/paper/SBF
xelatex -interaction=nonstopmode main.tex
bibtex main
xelatex -interaction=nonstopmode main.tex
xelatex -interaction=nonstopmode main.tex
```

Resolve undefined citations/references before release. Overfull boxes should be
checked manually because LaTeX can finish successfully while still producing a
visually poor line.

## Expected variance

Exact timing depends on CPU, thermal state, compiler, optional dependency
versions, and system load. A clean reproduction should prioritize:

1. Same experiment row semantics.
2. Same success/failure classification.
3. Same generated table structure and units.
4. Timing within a reasonable hardware-dependent band.

For reviewer support, include the source commit or exported archive checksum,
the result artifact checksum, and a short hardware/software summary.

# SafeBoxForest Quick Start

This file is the shortest path from a clean checkout to a working
SafeBoxForest build. For full paper reproduction details, see
`REPRODUCIBILITY.md`.

## 1. Configure and build the core project

From the repository root:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j "$(nproc)"
ctest --test-dir build --output-on-failure
```

This build uses the default lightweight configuration. Drake and OMPL are
optional and are only needed for the corresponding baseline experiments.

## 2. Build Python bindings when running paper scripts

Most paper-facing wrappers use the Python interface and helper scripts:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DSBF_WITH_PYTHON=ON
cmake --build build -j "$(nproc)"
export PYTHONPATH="$PWD/python:$PWD/build/python:${PYTHONPATH:-}"
python - <<'PY'
import sbf5
print("sbf5 import ok")
PY
```

Optional Python plotting and testing dependencies:

```bash
python -m pip install -r requirements-dev.txt
```

## 3. Run a small paper smoke test

The full TRO experiments are intentionally heavier. Start with a quick endpoint
pipeline run and regenerate paper tables from whatever result JSONs are
available locally:

```bash
python experiments/paper/01_epiaabb_pipeline.py --quick
python experiments/paper/07_update_paper_results.py
```

Generated LaTeX snippets are written under `doc/paper/SBF/generated/`.

## 4. Use the retained orchestrator

For a section-order smoke run:

```bash
python experiments/paper/run_all.py --quick --update-paper-results
```

Run one heavy experiment at a time. The paper scripts assume 8 logical threads
and CPU affinity `0-7` for comparable timings.

## 5. Create a public standalone tree

If this directory is nested inside a larger research workspace, export only this
project before publishing:

```bash
./scripts/export_standalone_repo.sh /tmp/SafeBoxForest-public
cd /tmp/SafeBoxForest-public
git init
git add .
```

The export intentionally drops local build directories, logs, runtime outputs,
and bulk regenerated paper results. See `RELEASE_MANIFEST.md` for the intended
source tree and artifact layout.

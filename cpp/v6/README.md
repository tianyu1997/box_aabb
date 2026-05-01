# SafeBoxForest v6

SafeBoxForest (SBF) tiles configuration space with certified collision-free boxes,
builds a forest graph, and plans multi-query motions with Dijkstra search,
optional Drake GCS optimisation, and an experiment bundle suitable for benchmarking
against baselines.

**Use this directory as the root of a standalone public Git repository**  
When publishing to GitHub (for example alongside an *IEEE Transactions on
Robotics* submission), clone or copy **only this tree** — not a parent mono-repo —
so paths and tooling match reviewer expectations.

**Splitting out of a mono-repo**

- Scripted copy (drop build artefacts locally):[`scripts/export_standalone_repo.sh`](scripts/export_standalone_repo.sh).
- Or preserve Git history only for this subtree:

  ```bash
  git subtree split -P cpp/v6 -b safeboxforest-v6-standalone   # prefix must match mono-repo layout
  ```

**Note:** If you keep `cpp/v6` nested under a legacy parent repo, `.gitignore` rules at the repo root might still suppress files beneath `cmake/` unless you prune broad patterns there. Prefer a standalone clone for reproducible open release.

---

## Companion publication / citation

[CITATION.cff](CITATION.cff) provides software metadata plus a **`preferred-citation`**
placeholder entry for your TRO paper (fill authors, volumes, pages, DOI after
assignment). Typical BibTeX (replace placeholders):

```bibtex
@article{REPLACE_KEY,
  title   = "{REPLACE_TITLE}",
  journal = "{IEEE Transactions on Robotics}",
  year    = {2026},
  volume  = {VOL},
  number  = {ISS},
  pages   = {START--END},
  doi     = {DOI}
}
```

---

## Key features

| Feature | Description |
|---------|-------------|
| **Interval arithmetic envelope** | Conservative link swept volumes (IAABB, grid, Hull-16) |
| **LECT** | Link–envelope configuration tree mapping C-space boxes to link envelopes |
| **Find-Free-Box (FFB)** | Descend the LECT to obtain a maximal certified collision-free box |
| **Forest grower** | RRT or wavefront; parallel growth via `n_threads` |
| **Coarsening** | Dimension sweep, greedy merge, overlap filter |
| **Planning** | Dijkstra, optional Drake GCS, two-pass smoothing |
| **Python** | pybind11 module (`sbf5` package name for compatibility) |

---

## Directory layout (repository root)

```
include/sbf/          C++ public headers
src/                  C++ implementation
test/                 C++ unit tests and benchmarks
experiments/          Binaries, configs, paper runners
python/               Bindings and tools (*sbf5*, *sbf5_bench*, *sbf5_viz*)
tools/                C++ utilities
scripts/              Analysis and reproducibility helpers
data/                 Robots and scenes (URDF, JSON)
doc/
  paper/              Manuscript workspace (English + Chinese drafts)
  plan/               Design notes / history
  reference/          API-style notes
  generated/          Paper support artefacts where applicable
result/               Compiled-in runtime result convention (`SBF_RESULT_DIR`)
output/               Ad hoc caches and scratch dumps (gitignored)
log/                  Verbose logs (often gitignored)
cmake/                CMake modules and FetchDeps
CMakeLists.txt
```

---

## Prerequisites

| Tool | Requirement |
|------|--------------|
| **CMake** | ≥ 3.20 |
| **C++ compiler** | C++ **20** — GCC 11+, Clang 14+, MSVC 2022 |
| **Python** | Optional 3.10+ (`SBF_WITH_PYTHON=ON`) |
| **Git** | Required for CMake FetchContent |

Dependencies download automatically into `_sbf6_deps/` (ignored by `.gitignore`):

| Package | Pin | Licence | Role |
|---------|-----|---------|------|
| Eigen | 3.4.0 | MPL 2.0 | Algebra |
| nlohmann/json | 3.11.3 | MIT | Serialisation |
| doctest | 2.4.11 | MIT | Testing |
| pybind11 | 2.12.0 | BSD-3-Clause | Python bindings |

**Optional linkage**

| CMake flag | Enables | Remark |
|-----------|---------|--------|
| `SBF_WITH_DRAKE=ON` | Drake adapters + GCS code paths | Build without AVX clashes per `CMakeLists.txt` |
| `SBF_WITH_OMPL=ON` | OMPL adapter | Baseline comparisons |
| `SBF_WITH_PYTHON=ON` | pybind11 extension | Required for Python quick-starts |

Without Drake / OMPL, **IRIS–GCS, Drake validation, OMPL-centric baselines, and Drake-specific tables or figures cannot be regenerated** locally; SafeBoxForest’s own C++/Python tests excluding those adapters remain available.

Third-party licences and upstream robot assets — including Apache-2.0 Panda meshes — are summarized in **[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)**.

---

## Configure and build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j "$(nproc)"
```

### CMake options (selected)

| Option | Default | Description |
|--------|---------|----------------|
| `SBF_BUILD_TESTS` | ON | doctest targets + `ctest` |
| `SBF_BUILD_EXPERIMENTS` | ON | Experimental binaries (`experiments`, `tools`) |
| `SBF_WITH_PYTHON` | OFF | Build `_sbf5_cpp` |
| `SBF_WITH_DRAKE` | OFF | Drake linkage |
| `SBF_WITH_OMPL` | OFF | OMPL linkage |
| `SBF_ENABLE_ASAN` | OFF | AddressSanitizer (GCC/Clang) |

Trim CI/runtime:

```bash
cmake -S . -B build -DSBF_BUILD_EXPERIMENTS=OFF
```

### C++ tests

```bash
ctest --test-dir build --output-on-failure
```

---

## Python bindings

```bash
cmake -S . -B build -DSBF_WITH_PYTHON=ON
cmake --build build -j "$(nproc)"
cp build/python/_sbf5_cpp*.so python/sbf5/
export PYTHONPATH="$PWD/python:$PWD/build/python:${PYTHONPATH:-}"
```

Dev dependencies (pytest, numpy, matplotlib, plotly for figures / viz paths):

```bash
pip install -r requirements-dev.txt
python -m pytest python/tests/ -v
```

---

## TRO / supplementary experiment reproduction

Authoritative paper runners live under **[`experiments/paper/`](experiments/paper/)**

- Read **[`experiments/paper/README.md`](experiments/paper/README.md)** for script mapping and metrics definitions.
- Orchestrator: [`experiments/paper/run_all.py`](experiments/paper/run_all.py).

**Resource fairness (do not relax casually)**  
Paper scripts target **8 logical threads** with CPU affinity **0–7**. Run **one** heavy experiment at a time to avoid resource contention; do not launch multiple full sweeps in parallel on the same machine.

**Regenerating numbers vs shipping snapshots**

- Small frozen JSON references for baselines sit in [`experiments/paper/results_frozen/`](experiments/paper/results_frozen/) and are meant to be versioned.
- Large regenerated outputs go to `experiments/results_paper/` (ignored by `.gitignore`). A full `--full` sweep can take substantial wall time and may require Drake/OMPL for complete baseline parity — see the paper README for which rows need which stack.
- After runs, [`experiments/paper/07_update_paper_results.py`](experiments/paper/07_update_paper_results.py) refreshes LaTeX tables under `doc/paper/en/generated/` when provided the corresponding JSON inputs.

---

## Quick start (Python)

```python
from sbf5 import Robot, SBFPlanner, SBFPlannerConfig, Obstacle
import numpy as np

robot = Robot.from_json("data/2dof.json")
obs = [Obstacle([1.0, 1.0, 0.0], [2.0, 2.0, 1.0])]

config = SBFPlannerConfig()
config.grower.n_threads = 4  # paper scripts default to 8 + affinity; see experiments/paper/README.md

planner = SBFPlanner(robot, obs, config)
planner.build()

start = np.array([0.1, 0.1])
goal  = np.array([2.5, 2.5])
result = planner.plan(start, goal)

print(f"Path length: {len(result.waypoints)}")
print(f"Build time:  {result.build_time_ms:.1f} ms")
print(f"Plan  time:  {result.plan_time_ms:.1f} ms")
```

---

## Quick start (C++)

```cpp
#include <sbf/core/robot.h>
#include <sbf/planner/sbf_planner.h>
#include <sbf/scene/collision_checker.h>

int main() {
    auto robot = sbf::Robot::from_json("data/2dof.json");
    std::vector<sbf::AABB> obstacles = { /* ... */ };
    sbf::CollisionChecker checker(obstacles);

    sbf::SBFPlannerConfig config;
    config.grower.n_threads = 4;

    sbf::SBFPlanner planner(robot, checker, config);
    planner.build();

    Eigen::VectorXd start(2), goal(2);
    start << 0.1, 0.1;
    goal  << 2.5, 2.5;

    auto result = planner.plan(start, goal);
}
```

---

## Module overview

- **Core** — Robot JSON, intervals, FK state, joint symmetry.
- **Envelope** — IAABB / grid / hull pipelines, GCPC, sampling sources.
- **LECT** — Tree build, snapshots, IO, caches.
- **FFB** — Find-free-box queries.
- **Forest** — Grower (serial/parallel), adjacency, coarsening, bridges.
- **Planner** — Dijkstra, optional GCS, smoothing.
- **Scene** — AABB collision checking.
- **Voxel** — Hull-16 rasterisation.
- **Viz** — JSON export hooks.

---

## Testing reference

| C++ target | Focus |
|------------|--------|
| `test_core` | Robot & intervals |
| `test_endpoint_iaabb` / `test_link_iaabb` | Envelopes |
| `test_lect` / `test_lect_io` | LECT |
| `test_ffb` | FFB |
| `test_grower` / `test_coarsen` | Forest |
| `test_planner` / `test_full_pipeline` | Planning |
| `test_gcs_drake` | Present only with `SBF_WITH_DRAKE=ON` |

| Python | Role |
|--------|------|
| `test_sbf5.py` | End-to-end bindings |
| `test_bench.py` | Benchmark harness |
| `test_viz.py` | Visualisation export |

---

## Governance

- **Licence:** [MIT](LICENSE)
- **Third-party / upstream notices:** [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)
- **Contributing:** [CONTRIBUTING.md](CONTRIBUTING.md)
- **Security contact:** [SECURITY.md](SECURITY.md)

GitHub Actions workflow [`.github/workflows/ci.yml`](.github/workflows/ci.yml) builds with tests on Ubuntu (no Drake, experiments off) to guard the default developer path.

# SafeBoxForest v7 — Box Forest Planner

A certified box-cover and multi-query planning framework for serial
manipulators. v7 is a refactor of the v6 release that targets
per-box construction throughput and a smaller in-memory cache
footprint while preserving the certified-envelope contract.

For the full method description and theoretical results, see the v6
paper. This release contains the v7 system update note documenting
the architectural changes (D1, D2, GCS adjacency) and reports the
v7-vs-v6 throughput audit.

## Requirements

- CMake ≥ 3.18
- Eigen3 ≥ 3.4
- C++17 compiler (gcc 11+ or clang 14+)
- Optional: OMPL ≥ 1.5 (for the OMPL baseline; auto-detected)
- Optional: Drake / pydrake (for the GCS baseline; auto-detected)
- Python ≥ 3.9 (for experiment drivers and paper tables)

## Quick Build

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8
ctest --output-on-failure
```

All `smoke_P0` … `smoke_P7` should pass in under one second wall time.

## Running Experiments

```bash
# Quick smoke (3 seeds, ~1 minute):
bash scripts/run_quick_all.sh

# Full nightly (20 seeds, ~5 minutes including baselines):
bash scripts/run_nightly_all.sh

# Regenerate paper tables and macros from latest results:
python3 scripts/build_paper_tables.py
```

Result JSONs land in `experiments/results_nightly/{quick,full}/`.

## Paper

Compile both language versions:

```bash
cd doc
pdflatex box_aabb_v7_paper_en.tex   # English (3 pages)
xelatex  box_aabb_v7_paper_zh.tex   # Chinese (3 pages)
```

All numerical content is derived from `doc/generated/*.tex`, which is
produced by `scripts/build_paper_tables.py`. Do not hand-edit the
generated files; rerun the experiments and regenerate.

## Repository Layout

```
cpp/v7/
├── CMakeLists.txt
├── data/                    robot kinematics JSON
├── doc/                     paper sources (en/zh) + generated tables
├── experiments/
│   ├── configs/             scene JSONs (incl. deferred Marcucci)
│   ├── results_nightly/     measured JSONs
│   ├── exp_main.cpp         main success-rate / timing experiment
│   ├── exp_threads.cpp      strong-scaling sweep
│   ├── exp_pathopt_steps.cpp post-optimisation ablation
│   ├── baseline_collision.h shared QFreeChecker
│   └── baseline_ompl.cpp    OMPL RRT-Connect baseline
├── include/sbf/             public headers
├── scripts/                 Python drivers and paper-table generators
├── src/                     planner / forest / lect / envelope / scene
└── test/smoke/              P0..P7 smoke tests
```

## Known Limitation

The current `LinkIAABB` envelope is too conservative for the
Marcucci 16-obstacle scene used in the v6 paper. v7 cannot yet
replicate the v6 Marcucci experiments. The five Marcucci scene JSONs
are pre-populated under `experiments/configs/marcucci/` and will
become runnable once the `LinkIAABB_Grid` voxel envelope is ported
in a follow-up release.

## License

See [LICENSE](LICENSE).

## Citation

If you use this code, please cite the v6 paper (the v7 update note
will be linked once finalised).

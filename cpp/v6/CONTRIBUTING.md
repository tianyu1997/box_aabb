# Contributing

This repository accompanies **SafeBoxForest** research code. Contributions are welcome as issues or pull requests.

## Code style

- Match the prevailing style of the subdirectory you edit (CMake, C++20, Python 3 typing where used).
- Keep changes narrowly scoped so reviewers can correlate diffs with the paper and experiments.

## Build and test locally

See the root [README](README.md). Before opening a PR, run at least:

```bash
cmake -S . -B build -DSBF_BUILD_EXPERIMENTS=OFF
cmake --build build --parallel "$(nproc)"
ctest --test-dir build --output-on-failure
```

## Experimental fairness

Paper-facing scripts enforce fixed thread counts and CPU affinity (`experiments/paper/README.md`). Avoid changing those defaults casually; unfair resource settings invalidate benchmark comparisons.

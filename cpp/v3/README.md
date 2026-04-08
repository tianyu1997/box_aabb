# SafeBoxForest v3

T-RO journal extension of the IROS 2026 paper.

## Build

```bash
mkdir build && cd build
cmake .. -DSBF_BUILD_EXPERIMENTS=ON
cmake --build . --config Release
```

## Structure

```
include/sbf/    Public headers
src/            Implementation
experiments/    Benchmark experiments
tests/          Unit tests
paper/          T-RO paper source
docs/           API reference & maps
configs/        Robot configuration files
```

## Paper Sections

1. **Sec III**  Interval Envelope Computation & Caching (LECT)
2. **Sec IV**  SBF Generation & Update Maintenance
3. **Sec V**  SBF Planner

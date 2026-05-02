# Release Manifest

This manifest describes the intended standalone public release layout for
SafeBoxForest v6. It is a release checklist, not an automatically generated
inventory.

## Source repository root

The public repository root should be the exported contents of `cpp/v6`, not the
parent research workspace. Use:

```bash
./scripts/export_standalone_repo.sh /tmp/SafeBoxForest-public
```

Then inspect `/tmp/SafeBoxForest-public` before initializing a public Git
repository.

## Include in the source release

- `include/`, `src/`, `cmake/`, `CMakeLists.txt`: C++ implementation and build.
- `python/`: Python package and pybind11-facing code.
- `test/`, `tools/`, `viz/`: tests, utilities, and visualization helpers.
- `experiments/paper/`: retained TRO experiment wrappers and small frozen references.
- `experiments/configs/`: experiment configurations referenced by paper scripts.
- `scripts/`: reproducibility, plotting, and analysis helpers.
- `data/`: robot JSON, `.gcpc` inputs, and URDF assets with notices.
- `doc/paper/SBF/`: current TRO paper source and generated snippets.
- `doc/plan/`, `doc/reference/`: design notes and reference documentation.
- `README.md`, `QUICKSTART.md`, `REPRODUCIBILITY.md`, `CONTRIBUTING.md`,
  `SECURITY.md`, `LICENSE`, `THIRD_PARTY_NOTICES.md`, `CITATION.cff`.

## Exclude from the source release

- Build directories: `build/`, `build_*`, `build-*`, `cmake-build-*`.
- CMake local files: `CMakeCache.txt`, `compile_commands.json`, `Testing/`.
- FetchContent staging: `_sbf6_deps/`.
- Runtime outputs: `result/`, `output/`, `log/`.
- Local caches: `.cache/`, `.pytest_cache/`, `__pycache__/`, `.venv/`, `venv/`.
- IDE files: `.idea/`, `.vscode/`.
- Local archive and archaeology: `archive/`.
- Bulk regenerated paper results: `experiments/results_paper/`.
- Generated heavy or transient blobs: `.lect`, `.hcache`, `.cache`, `.bin`,
  `.frames`, `.hull`, `.hulls`, `.sdf`, `.ipch`, `.bak`.
- LaTeX build products: `.aux`, `.bbl`, `.blg`, `.fdb_latexmk`, `.fls`, `.pdf`,
  `.synctex.gz`, `.toc`, `.out`, `.bcf`, `.run.xml`, `.xdv`.

The export script encodes this policy. Update the script when this manifest
changes.

## Result artifacts

Paper result JSONs should be published separately from the lightweight source
tree. Recommended artifact name:

```text
safeboxforest-v6.0.0-tro-results.zip
```

The artifact should contain curated files from `experiments/results_paper/` that
are needed to regenerate the paper tables and figures. Remove smoke directories,
scratch logs, `.bak` files, and temporary raw dumps before publishing.

Record checksums next to the artifact:

```bash
sha256sum safeboxforest-v6.0.0-tro-results.zip > safeboxforest-v6.0.0-tro-results.sha256
```

## Release validation

Run these checks from the exported public tree:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j "$(nproc)"
ctest --test-dir build --output-on-failure
```

For paper-facing smoke validation:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DSBF_WITH_PYTHON=ON
cmake --build build -j "$(nproc)"
export PYTHONPATH="$PWD/python:$PWD/build/python:${PYTHONPATH:-}"
python experiments/paper/01_epiaabb_pipeline.py --quick
python experiments/paper/07_update_paper_results.py
```

Optional Drake and OMPL baseline validation should be recorded in the release
notes because those stacks depend on the host environment.

## Tagging policy

Suggested source tag for the TRO submission package:

```text
v6.0.0-tro-submission
```

After acceptance, create an archival tag or release with the assigned DOI,
volume, pages, and final citation metadata in `CITATION.cff`.

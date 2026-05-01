# Third-party notices

The **SafeBoxForest** code in this repository is released under the **MIT License** ([LICENSE](LICENSE)). The following notices apply to bundled or fetched third-party components and upstream robot assets.

## Build-time fetched dependencies (CMake FetchContent)

These are downloaded on first CMake configure into `_sbf6_deps/` (not committed when using the provided `.gitignore`). Source references: [cmake/FetchDeps.cmake](cmake/FetchDeps.cmake).

| Project | Version (pinned) | License | URL |
|---------|-------------------|---------|-----|
| Eigen | 3.4.0 | MPL 2.0 | https://eigen.tuxfamily.org/ |
| nlohmann/json | 3.11.3 | MIT | https://github.com/nlohmann/json |
| doctest | 2.4.11 | MIT | https://github.com/doctest/doctest |
| pybind11 | 2.12.0 | BSD-3-Clause | https://github.com/pybind/pybind11 |

## Optional integrations (developer environment)

- **Robot Operating Stack / Drake** — If you configure with `-DSBF_WITH_DRAKE=ON`, you link against Drake. Drake is governed by its own BSD-style license terms; see the Drake project documentation for details.
- **OMPL** — If you configure with `-DSBF_WITH_OMPL=ON`, you link against OMPL under its respective license terms.

## Vendored / upstream robot description assets

- **Franka Emika Panda (MoveIt resources)** — Files under [`data/urdf/upstream/moveit_resources_panda_description/`](data/urdf/upstream/moveit_resources_panda_description/) are upstream MoveIt Resources content. Refer to [`data/urdf/upstream/moveit_resources_panda_description/LICENSE`](data/urdf/upstream/moveit_resources_panda_description/LICENSE) (Apache License, Version 2.0).

Other robot models or URDF trees under [`data/urdf/`](data/urdf/) may carry their own notices in subdirectories (*CHANGELOG*, *LICENSE*, upstream README).

## Disclaimer

This file is informational and **not legal advice**. If you redistribute binaries or bundles that include FetchContent vendors or upstream meshes, satisfy the attribution and license terms of those projects yourself.

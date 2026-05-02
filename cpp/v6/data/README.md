# Data Directory

This directory contains the robot and scene assets used by SafeBoxForest v6 and
its TRO experiment wrappers.

## Robot model JSON files

| File | Purpose |
| --- | --- |
| `2dof_planar.json` | Small planar system for debugging and smoke tests |
| `iiwa14.json` | IIWA14 kinematic model used by the shelf-scene experiments |
| `panda.json` | Panda model used by cross-robot random-scene experiments |
| `ur5.json` | UR5 model used by cross-robot random-scene experiments |
| `ur10.json` | UR10 model retained for additional robot tests |

These JSON files are part of the SafeBoxForest source distribution.

## Scene and cache data

| File | Purpose |
| --- | --- |
| `2dof_500.gcpc` | Collision/profile data for the small planar setup |
| `iiwa14_5000.gcpc` | IIWA14 scene/profile data used by paper experiments |
| `panda_5000.gcpc` | Panda scene/profile data used by paper experiments |

The `.gcpc` files are repository assets consumed by the experiment scripts and
C++ tools. Treat them as versioned inputs, not regenerated runtime caches.
Runtime outputs such as `.lect`, `.hcache`, and temporary `.bin` files are
ignored by the release boundary.

## URDF assets

The `urdf/` subtree contains robot description assets used by optional Drake,
OMPL, visualization, and validation paths. Upstream assets must keep their
original notices. In particular, Panda assets under `urdf/upstream/` are covered
by the upstream license files there and are also summarized in
`../THIRD_PARTY_NOTICES.md`.

## Release checklist for data

Before publishing a source release:

1. Keep robot JSON and `.gcpc` inputs that are referenced by examples or paper
   scripts.
2. Keep upstream license files inside `urdf/upstream/`.
3. Do not include generated caches, local logs, or temporary benchmark dumps.
4. If a new external robot asset is added, update this file and
   `../THIRD_PARTY_NOTICES.md` in the same change.

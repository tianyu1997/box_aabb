# Upstream Panda And UR5 Descriptions

This directory stores mesh-based robot description packages downloaded from
public upstream sources so Drake experiments can use complete Panda and UR5
models instead of the lightweight Exp.5 stand-ins.

Contents:

- `moveit_resources_panda_description/`: copied from the `panda_description`
  package in `moveit/moveit_resources` on branch `ros2`. The package name in
  `package.xml` is `moveit_resources_panda_description`, which matches the
  `package://` URIs used by `urdf/panda.urdf`.
- `ur_description/`: mirrored from `ros-industrial/universal_robot` on branch
  `melodic-devel`, but trimmed to the UR5-specific subset needed here:
  `package.xml`, `config/ur5/`, `meshes/ur5/`, `urdf/ur5.xacro`, and the
  supporting files under `urdf/inc/`. The upstream package does not ship a
  plain `ur5.urdf`, so `urdf/ur5.urdf` was added from the generated artifact in
  `Daniella1/urdf_files_dataset/urdf_files/ros-industrial/xacro_generated/...`
  to provide a directly loadable URDF alongside the official UR5 mesh package.

Validation:

- Drake can parse both `moveit_resources_panda_description/urdf/panda.urdf` and
  `ur_description/urdf/ur5.urdf` when the parser package map is populated from
  this directory.
- Panda emits expected warnings for unsupported `safety_controller` tags and a
  mimic finger joint when imported into Drake.
- If you register full geometry in Drake, Panda collision STL meshes may hit a
  Drake convex-hull limitation. Kinematic parsing succeeds.
#!/usr/bin/env python3
"""Generate Drake-compatible URDF collision meshes for Exp.5 robots.

Drake's SceneGraphCollisionChecker convex-hull path accepts OBJ/VTP/GLTF but
not STL meshes in this environment. The upstream UR5/Panda URDFs use STL
collision meshes, so this script converts those STL files to OBJ and writes
URDF copies that reference the OBJ collision meshes.
"""
from __future__ import annotations

import argparse
import re
import struct
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
UPSTREAM = ROOT / "data" / "urdf" / "upstream"
OUT = ROOT / "data" / "urdf" / "generated" / "drake_compatible"

ROBOTS = {
    "ur5": {
        "package": "ur_description",
        "package_dir": UPSTREAM / "ur_description",
        "urdf": UPSTREAM / "ur_description" / "urdf" / "ur5.urdf",
    },
    "panda": {
        "package": "moveit_resources_panda_description",
        "package_dir": UPSTREAM / "moveit_resources_panda_description",
        "urdf": UPSTREAM / "moveit_resources_panda_description" / "urdf" / "panda.urdf",
    },
}


def read_stl_triangles(path: Path) -> list[tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]]:
    data = path.read_bytes()
    triangles: list[tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]] = []
    if len(data) >= 84:
        n_tri = struct.unpack_from("<I", data, 80)[0]
        expected = 84 + 50 * n_tri
        if expected == len(data):
            offset = 84
            for _ in range(n_tri):
                # normal (3 floats), then 3 vertices, then attribute count.
                values = struct.unpack_from("<12f", data, offset)
                triangles.append((
                    (values[3], values[4], values[5]),
                    (values[6], values[7], values[8]),
                    (values[9], values[10], values[11]),
                ))
                offset += 50
            return triangles

    vertices: list[tuple[float, float, float]] = []
    for line in path.read_text(errors="ignore").splitlines():
        stripped = line.strip()
        if not stripped.startswith("vertex "):
            continue
        _, xs, ys, zs = stripped.split()[:4]
        vertices.append((float(xs), float(ys), float(zs)))
        if len(vertices) == 3:
            triangles.append((vertices[0], vertices[1], vertices[2]))
            vertices = []
    return triangles


def write_obj(stl_path: Path, obj_path: Path) -> None:
    triangles = read_stl_triangles(stl_path)
    obj_path.parent.mkdir(parents=True, exist_ok=True)
    with obj_path.open("w") as out:
        out.write(f"# Converted from {stl_path.name}\n")
        vertex_index = 1
        for tri in triangles:
            for vertex in tri:
                out.write(f"v {vertex[0]:.9g} {vertex[1]:.9g} {vertex[2]:.9g}\n")
            out.write(f"f {vertex_index} {vertex_index + 1} {vertex_index + 2}\n")
            vertex_index += 3


def convert_robot(robot: str) -> Path:
    spec = ROBOTS[robot]
    package = spec["package"]
    package_dir = spec["package_dir"]
    urdf_path = spec["urdf"]
    text = urdf_path.read_text()
    pattern = re.compile(rf"package://{re.escape(package)}/([^\"']+\.stl)")

    def replace(match: re.Match[str]) -> str:
        rel_stl = Path(match.group(1))
        stl_path = package_dir / rel_stl
        rel_obj = Path("meshes") / "collision_obj" / rel_stl.name.replace(".stl", ".obj")
        obj_path = package_dir / rel_obj
        write_obj(stl_path, obj_path)
        return f"package://{package}/{rel_obj.as_posix()}"

    converted = pattern.sub(replace, text)
    if robot == "panda":
        # Keep the hand geometry but make the two finger joints fixed so the
        # Drake checker exposes the same 7-DOF arm configuration as SBF.
        converted = re.sub(
            r'<joint name="panda_finger_joint1" type="prismatic">.*?</joint>',
            '<joint name="panda_finger_joint1" type="fixed">\n'
            '        <parent link="panda_hand" />\n'
            '        <child link="panda_leftfinger" />\n'
            '        <origin rpy="0 0 0" xyz="0 0 0.0584" />\n'
            '    </joint>',
            converted,
            flags=re.S,
        )
        converted = re.sub(
            r'<joint name="panda_finger_joint2" type="prismatic">.*?</joint>',
            '<joint name="panda_finger_joint2" type="fixed">\n'
            '        <parent link="panda_hand" />\n'
            '        <child link="panda_rightfinger" />\n'
            '        <origin rpy="0 0 3.141592653589793" xyz="0 0 0.0584" />\n'
            '    </joint>',
            converted,
            flags=re.S,
        )
    out_path = OUT / f"{robot}_drake.urdf"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(converted)
    return out_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robots", default="ur5,panda")
    args = parser.parse_args()
    for robot in [item.strip() for item in args.robots.split(",") if item.strip()]:
        out_path = convert_robot(robot)
        print(f"[drake-urdf] {robot}: {out_path}")


if __name__ == "__main__":
    main()

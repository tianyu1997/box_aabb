#!/usr/bin/env python3
"""Visualize Exp. 5 randomized scenes with Drake-loaded URDF robots."""
from __future__ import annotations

import argparse
import json
import re
import sys
import tempfile
from pathlib import Path

import numpy as np
from pydrake.geometry import Box, MeshcatVisualizer, MeshcatVisualizerParams, Rgba, Role, SceneGraph, StartMeshcat
from pydrake.math import RigidTransform
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.perception import PointCloud
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import TrajectorySource
from pydrake.systems.rendering import MultibodyPositionToGeometryPose
from pydrake.trajectories import PiecewisePolynomial

V6_ROOT = Path(__file__).resolve().parents[1]
PAPER_DIR = V6_ROOT / "experiments" / "paper"
if str(PAPER_DIR) not in sys.path:
    sys.path.insert(0, str(PAPER_DIR))

from exp5_scene_utils import fk_prefixes, frame_points, load_robot_doc  # noqa: E402

OBSTACLE_COLOR = Rgba(0.82, 0.22, 0.18, 0.74)
BLOCKER_COLOR = Rgba(0.95, 0.55, 0.16, 0.86)
WORKSPACE_COLOR = Rgba(0.35, 0.45, 0.60, 0.07)
PATH_COLOR = Rgba(0.05, 0.08, 0.10, 1.0)
UPSTREAM_URDF_ROOT = V6_ROOT / "data" / "urdf" / "upstream"
DEFAULT_URDFS = {
    "ur5": V6_ROOT / "data" / "urdf" / "upstream" / "ur_description" / "urdf" / "ur5.urdf",
    "panda": V6_ROOT / "data" / "urdf" / "upstream" / "moveit_resources_panda_description" / "urdf" / "panda.urdf",
}


def resolve_from_v6(path_text: str) -> Path:
    path = Path(path_text)
    return path if path.is_absolute() else V6_ROOT / path


def inferred_urdf(scene: dict) -> Path:
    if scene.get("robot_urdf"):
        return resolve_from_v6(scene["robot_urdf"])
    return DEFAULT_URDFS[scene["robot"]]


def load_visualization_model(parser: Parser, urdf_path: Path) -> list[int]:
    if urdf_path.suffix.lower() != ".urdf":
        return parser.AddModels(str(urdf_path))
    text = urdf_path.read_text(encoding="utf-8")
    visual_only = re.sub(r"\s*<collision>.*?</collision>", "", text, flags=re.DOTALL)
    if visual_only == text:
        return parser.AddModels(str(urdf_path))
    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False, encoding="utf-8") as handle:
        handle.write(visual_only)
        temp_path = Path(handle.name)
    try:
        return parser.AddModels(str(temp_path))
    finally:
        temp_path.unlink(missing_ok=True)


def meshcat_box(meshcat, path: str, lo: np.ndarray, hi: np.ndarray, color: Rgba) -> None:
    size = hi - lo
    center = 0.5 * (lo + hi)
    meshcat.SetObject(path, Box(float(size[0]), float(size[1]), float(size[2])), rgba=color)
    meshcat.SetTransform(path, RigidTransform(center))


def draw_scene_overlays(meshcat, scene: dict) -> None:
    workspace = scene["workspace_bounds"]
    meshcat_box(
        meshcat,
        "exp5/workspace/sample_bounds",
        np.asarray(workspace["lo"], dtype=float),
        np.asarray(workspace["hi"], dtype=float),
        WORKSPACE_COLOR,
    )
    for obstacle in scene["obstacles"]:
        lo = np.asarray(obstacle["lo"], dtype=float)
        hi = np.asarray(obstacle["hi"], dtype=float)
        color = BLOCKER_COLOR if obstacle.get("role") == "direct_path_blocker" else OBSTACLE_COLOR
        meshcat_box(meshcat, f"exp5/obstacles/{obstacle['name']}", lo, hi, color)


def load_paths(paths_file: Path | None, method: str | None) -> list[dict]:
    if paths_file is None:
        return []
    data = json.loads(paths_file.read_text())
    paths = [entry for entry in data.get("paths", []) if entry.get("waypoints")]
    if method is not None:
        paths = [entry for entry in paths if entry.get("method") == method]
    return paths


def fallback_path(scene: dict) -> dict:
    return {
        "method": "start_goal",
        "label": "Start-goal reference",
        "waypoints": [scene["start"], scene["goal"]],
        "path_length": float(np.linalg.norm(np.asarray(scene["goal"]) - np.asarray(scene["start"]))),
    }


def select_path(scene: dict, paths_file: Path | None, method: str | None) -> dict:
    paths = load_paths(paths_file, method)
    if not paths:
        return fallback_path(scene)
    if method is not None:
        return paths[0]
    preferred_order = ["sbf", "sbf_ifk", "iris_np_gcs", "ompl_bitstar", "ompl_prm"]
    for preferred in preferred_order:
        for entry in paths:
            if entry.get("method") == preferred:
                return entry
    return paths[0]


def waypoints_to_trajectory(waypoints: np.ndarray, speed: float) -> PiecewisePolynomial:
    if waypoints.shape[0] < 2:
        raise ValueError("need at least two waypoints")
    distances = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
    distances = np.maximum(distances, 1e-6)
    breaks = np.concatenate([[0.0], np.cumsum(distances) / max(speed, 1e-6)])
    return PiecewisePolynomial.FirstOrderHold(breaks, waypoints.T)


def build_drake_animation(meshcat, scene: dict, waypoints: np.ndarray, speed: float) -> float:
    urdf_path = inferred_urdf(scene)
    builder = DiagramBuilder()
    scene_graph = builder.AddSystem(SceneGraph())
    plant = MultibodyPlant(time_step=0.0)
    plant.RegisterAsSourceForSceneGraph(scene_graph)
    parser = Parser(plant)
    parser.package_map().PopulateFromFolder(str(UPSTREAM_URDF_ROOT))
    model_instances = load_visualization_model(parser, urdf_path)
    model_instance = model_instances[0]
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link", model_instance))
    plant.Finalize()

    trajectory = waypoints_to_trajectory(waypoints, speed)
    trajectory_source = builder.AddSystem(TrajectorySource(trajectory))
    to_pose = builder.AddSystem(MultibodyPositionToGeometryPose(plant))
    builder.Connect(trajectory_source.get_output_port(), to_pose.get_input_port())
    builder.Connect(to_pose.get_output_port(), scene_graph.get_source_pose_port(plant.get_source_id()))

    params = MeshcatVisualizerParams()
    params.delete_on_initialization_event = False
    params.role = Role.kIllustration
    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat, params)
    diagram = builder.Build()
    simulator = Simulator(diagram)

    plant_context = plant.CreateDefaultContext()
    tool_body = plant.GetBodyByName("tool0", model_instance)
    sample_count = max(int(trajectory.end_time() * 160), 80)
    times = np.linspace(trajectory.start_time(), trajectory.end_time(), sample_count)
    tool_points = []
    for sample_time in times:
        plant.SetPositions(plant_context, model_instance, trajectory.value(sample_time).flatten())
        tool_points.append(plant.EvalBodyPoseInWorld(plant_context, tool_body).translation())
    cloud = PointCloud(len(tool_points))
    cloud.mutable_xyzs()[:] = np.asarray(tool_points, dtype=float).T
    meshcat.SetObject("exp5/path/tool_trace", cloud, 0.009, rgba=PATH_COLOR)

    visualizer.StartRecording()
    simulator.AdvanceTo(trajectory.end_time())
    visualizer.PublishRecording()
    return float(trajectory.end_time())


def interpolate_path(waypoints: np.ndarray, frame_count: int) -> np.ndarray:
    distances = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
    cumulative = np.concatenate([[0.0], np.cumsum(np.maximum(distances, 1e-9))])
    total = cumulative[-1]
    if total <= 1e-12:
        return np.repeat(waypoints[:1], frame_count, axis=0)
    samples = np.linspace(0.0, total, frame_count)
    frames = []
    for sample in samples:
        segment_index = int(np.searchsorted(cumulative, sample, side="right") - 1)
        segment_index = min(segment_index, len(waypoints) - 2)
        segment_start = cumulative[segment_index]
        segment_span = cumulative[segment_index + 1] - segment_start
        fraction = 0.0 if segment_span <= 1e-12 else (sample - segment_start) / segment_span
        frames.append((1.0 - fraction) * waypoints[segment_index] + fraction * waypoints[segment_index + 1])
    return np.asarray(frames, dtype=float)


def draw_box_axes(ax, lo: np.ndarray, hi: np.ndarray, color: str, alpha: float) -> None:
    corners = np.array(
        [
            [lo[0], lo[1], lo[2]],
            [hi[0], lo[1], lo[2]],
            [hi[0], hi[1], lo[2]],
            [lo[0], hi[1], lo[2]],
            [lo[0], lo[1], hi[2]],
            [hi[0], lo[1], hi[2]],
            [hi[0], hi[1], hi[2]],
            [lo[0], hi[1], hi[2]],
        ]
    )
    faces = [
        [0, 1, 2, 3],
        [4, 5, 6, 7],
        [0, 1, 5, 4],
        [2, 3, 7, 6],
        [1, 2, 6, 5],
        [0, 3, 7, 4],
    ]
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    collection = Poly3DCollection([[corners[index] for index in face] for face in faces])
    collection.set_facecolor(color)
    collection.set_alpha(alpha)
    collection.set_edgecolor("#2b2b2b")
    collection.set_linewidth(0.45)
    ax.add_collection3d(collection)


def set_axes_equal(ax, lo: np.ndarray, hi: np.ndarray) -> None:
    center = 0.5 * (lo + hi)
    radius = 0.5 * float(np.max(hi - lo))
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(max(0.0, center[2] - radius), center[2] + radius)


def render_mp4(scene: dict, robot_doc: dict, waypoints: np.ndarray, output_path: Path, fps: int, duration: float) -> None:
    import imageio.v2 as imageio
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    frame_count = max(2, int(fps * duration))
    configs = interpolate_path(waypoints, frame_count)
    workspace_lo = np.asarray(scene["workspace_bounds"]["lo"], dtype=float)
    workspace_hi = np.asarray(scene["workspace_bounds"]["hi"], dtype=float)
    all_points = [workspace_lo, workspace_hi]
    for config in configs[:: max(1, frame_count // 20)]:
        all_points.extend(frame_points(robot_doc, config))
    point_array = np.asarray(all_points, dtype=float)
    bounds_lo = np.min(point_array, axis=0) - 0.10
    bounds_hi = np.max(point_array, axis=0) + 0.10

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with imageio.get_writer(str(output_path), fps=fps, codec="libx264", quality=8) as writer:
        for config in configs:
            fig = plt.figure(figsize=(8.8, 6.2), dpi=130)
            ax = fig.add_subplot(111, projection="3d")
            ax.view_init(elev=24, azim=-58)
            ax.set_facecolor("#f6f7f8")
            ax.grid(False)
            ax.set_xlabel("x [m]")
            ax.set_ylabel("y [m]")
            ax.set_zlabel("z [m]")
            ax.set_title(f"{scene['scene_id']} path replay", pad=12)
            for obstacle in scene["obstacles"]:
                lo = np.asarray(obstacle["lo"], dtype=float)
                hi = np.asarray(obstacle["hi"], dtype=float)
                color = "#e28b2d" if obstacle.get("role") == "direct_path_blocker" else "#ce3f35"
                draw_box_axes(ax, lo, hi, color, 0.42)
            prefixes = fk_prefixes(robot_doc, config)
            points = np.asarray([prefix[:3, 3] for prefix in prefixes], dtype=float)
            ax.plot(points[:, 0], points[:, 1], points[:, 2], color="#143b73", linewidth=5.2, solid_capstyle="round")
            ax.scatter(points[:, 0], points[:, 1], points[:, 2], color="#d9dee7", edgecolor="#143b73", s=34, linewidth=0.8)
            start_points = frame_points(robot_doc, waypoints[0])
            goal_points = frame_points(robot_doc, waypoints[-1])
            ax.scatter(start_points[-1, 0], start_points[-1, 1], start_points[-1, 2], color="#18804b", s=58)
            ax.scatter(goal_points[-1, 0], goal_points[-1, 1], goal_points[-1, 2], color="#1746b0", s=58)
            set_axes_equal(ax, bounds_lo, bounds_hi)
            fig.tight_layout()
            fig.canvas.draw()
            rgba = np.asarray(fig.canvas.buffer_rgba())
            writer.append_data(rgba[:, :, :3])
            plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scene", type=Path, help="saved Exp. 5 scene JSON")
    parser.add_argument("--paths", type=Path, default=None, help="typical path JSON emitted by 05_random_robot_scenes.py")
    parser.add_argument("--method", type=str, default=None, help="method key to visualize, e.g. sbf")
    parser.add_argument("--save-html", type=Path, default=None)
    parser.add_argument("--save-video", type=Path, default=None, help="write an MP4 replay of the selected path")
    parser.add_argument("--speed", type=float, default=1.2, help="joint-space speed used for Meshcat playback")
    parser.add_argument("--video-fps", type=int, default=20)
    parser.add_argument("--video-duration", type=float, default=5.0)
    parser.add_argument("--serve", action="store_true", help="keep the Meshcat server alive until Enter is pressed")
    args = parser.parse_args()

    scene_path = args.scene if args.scene.is_absolute() else Path.cwd() / args.scene
    scene = json.loads(scene_path.read_text())
    selected = select_path(scene, args.paths, args.method)
    waypoints = np.asarray(selected["waypoints"], dtype=float)
    robot_doc = load_robot_doc(resolve_from_v6(scene["robot_json"]))

    meshcat = StartMeshcat()
    meshcat.Delete()
    draw_scene_overlays(meshcat, scene)
    duration = build_drake_animation(meshcat, scene, waypoints, args.speed)
    print(f"Meshcat URL: {meshcat.web_url()}")
    print(f"selected path: {selected.get('label', selected.get('method'))}, duration={duration:.2f}s")

    if args.save_html is not None:
        args.save_html.parent.mkdir(parents=True, exist_ok=True)
        args.save_html.write_text(meshcat.StaticHtml())
        print(f"wrote {args.save_html}")
    if args.save_video is not None:
        render_mp4(scene, robot_doc, waypoints, args.save_video, args.video_fps, args.video_duration)
        print(f"wrote {args.save_video}")
    if args.serve:
        input("Press Enter to stop Meshcat...")


if __name__ == "__main__":
    main()
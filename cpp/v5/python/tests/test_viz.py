"""Tests for sbf5_viz — data loading, figure creation, CLI."""

import json
import os
import tempfile

import numpy as np
import pytest

# ─── Fixtures: sample JSON data matching v5 viz_exporter schema ──────────────

SAMPLE_ROBOT = {
    "type": "robot",
    "name": "test_2dof",
    "n_joints": 2,
    "link_radii": [0.05, 0.05],
    "configs": [
        {
            "q": [0.1, 0.2],
            "link_positions": [[0.0, 0.0, 0.0], [0.5, 0.1, 0.0], [0.9, 0.3, 0.0]],
        },
        {
            "q": [1.0, 1.5],
            "link_positions": [[0.0, 0.0, 0.0], [0.27, 0.42, 0.0], [0.05, 0.86, 0.0]],
        },
    ],
}

SAMPLE_ENVELOPE = {
    "type": "envelope",
    "method": "link_iaabb",
    "boxes": [
        {
            "box_id": 0,
            "intervals": [[0.0, 0.5], [0.0, 0.5]],
            "links": [
                {"link_idx": 0, "aabb": [[-0.1, 0.6], [-0.1, 0.2], [-0.05, 0.05]]},
                {"link_idx": 1, "aabb": [[0.3, 1.0], [-0.1, 0.5], [-0.05, 0.05]]},
            ],
        },
        {
            "box_id": 1,
            "intervals": [[0.5, 1.0], [0.0, 0.5]],
            "links": [
                {"link_idx": 0, "aabb": [[-0.2, 0.5], [-0.2, 0.1], [-0.05, 0.05]]},
                {"link_idx": 1, "aabb": [[0.1, 0.8], [-0.2, 0.4], [-0.05, 0.05]]},
            ],
        },
    ],
}

SAMPLE_SCENE = {
    "type": "scene",
    "obstacles": [
        {"center": [0.5, 0.5, 0.0], "half_sizes": [0.1, 0.1, 0.1]},
        {"center": [-0.3, 0.2, 0.0], "half_sizes": [0.05, 0.15, 0.05]},
    ],
}

SAMPLE_FOREST = {
    "type": "forest",
    "n_boxes": 3,
    "total_volume": 0.375,
    "boxes": [
        {"id": 0, "intervals": [[0.0, 0.5], [0.0, 0.5]], "volume": 0.25},
        {"id": 1, "intervals": [[0.5, 1.0], [0.0, 0.5]], "volume": 0.25},
        {"id": 2, "intervals": [[0.0, 0.5], [0.5, 1.0]], "volume": 0.25},
    ],
}

SAMPLE_SNAPSHOT = {
    "type": "snapshot",
    "robot": SAMPLE_ROBOT,
    "envelope": SAMPLE_ENVELOPE,
    "scene": SAMPLE_SCENE,
    "forest": SAMPLE_FOREST,
}


@pytest.fixture
def tmp_json(tmp_path):
    """Write a dict to a temp JSON file and return the path."""
    def _write(data, name="test.json"):
        p = tmp_path / name
        p.write_text(json.dumps(data))
        return str(p)
    return _write


# ─── K1: Data Loading ────────────────────────────────────────────────────────

class TestLoadData:
    def test_load_robot_data(self, tmp_json):
        from sbf5_viz.load_data import load_robot_data
        path = tmp_json(SAMPLE_ROBOT, "robot.json")
        rd = load_robot_data(path)
        assert rd.name == "test_2dof"
        assert rd.n_joints == 2
        assert len(rd.configs) == 2
        assert rd.configs[0].q.shape == (2,)
        assert rd.configs[0].link_positions.shape == (3, 3)

    def test_load_envelope_data(self, tmp_json):
        from sbf5_viz.load_data import load_envelope_data
        path = tmp_json(SAMPLE_ENVELOPE, "envelope.json")
        ed = load_envelope_data(path)
        assert ed.method == "link_iaabb"
        assert len(ed.boxes) == 2
        assert len(ed.boxes[0].links) == 2
        assert ed.boxes[0].links[0].lo.shape == (3,)

    def test_load_scene_data(self, tmp_json):
        from sbf5_viz.load_data import load_scene_data
        path = tmp_json(SAMPLE_SCENE, "scene.json")
        sd = load_scene_data(path)
        assert len(sd.obstacles) == 2
        np.testing.assert_allclose(sd.obstacles[0].center, [0.5, 0.5, 0.0])

    def test_load_forest_data(self, tmp_json):
        from sbf5_viz.load_data import load_forest_data
        path = tmp_json(SAMPLE_FOREST, "forest.json")
        fd = load_forest_data(path)
        assert fd.n_boxes == 3
        assert len(fd.boxes) == 3
        assert fd.boxes[0].intervals.shape == (2, 2)

    def test_load_snapshot(self, tmp_json):
        from sbf5_viz.load_data import load_snapshot
        path = tmp_json(SAMPLE_SNAPSHOT, "snapshot.json")
        snap = load_snapshot(path)
        assert snap.robot is not None
        assert snap.robot.name == "test_2dof"
        assert snap.envelope is not None
        assert len(snap.envelope.boxes) == 2
        assert snap.scene is not None
        assert len(snap.scene.obstacles) == 2
        assert snap.forest is not None
        assert snap.forest.n_boxes == 3


# ─── K2: Robot Viz ────────────────────────────────────────────────────────────

class TestRobotViz:
    def test_plot_robot_3d_returns_figure(self, tmp_json):
        from sbf5_viz.load_data import load_robot_data
        from sbf5_viz.robot_viz import plot_robot_3d
        path = tmp_json(SAMPLE_ROBOT, "robot.json")
        rd = load_robot_data(path)
        fig = plot_robot_3d(rd, config_idx=0)
        assert hasattr(fig, "data")
        assert len(fig.data) >= 1

    def test_plot_robot_multi_configs(self, tmp_json):
        from sbf5_viz.load_data import load_robot_data
        from sbf5_viz.robot_viz import plot_robot_multi_configs
        path = tmp_json(SAMPLE_ROBOT, "robot.json")
        rd = load_robot_data(path)
        fig = plot_robot_multi_configs(rd)
        assert len(fig.data) >= 2  # at least 2 configs × 2 traces


# ─── K3: Envelope Viz ─────────────────────────────────────────────────────────

class TestEnvelopeViz:
    def test_plot_envelope_wireframe(self, tmp_json):
        from sbf5_viz.load_data import load_envelope_data
        from sbf5_viz.envelope_viz import plot_envelope_wireframe
        path = tmp_json(SAMPLE_ENVELOPE, "envelope.json")
        ed = load_envelope_data(path)
        fig = plot_envelope_wireframe(ed)
        assert len(fig.data) >= 2

    def test_plot_envelope_filled(self, tmp_json):
        from sbf5_viz.load_data import load_envelope_data
        from sbf5_viz.envelope_viz import plot_envelope_filled
        path = tmp_json(SAMPLE_ENVELOPE, "envelope.json")
        ed = load_envelope_data(path)
        fig = plot_envelope_filled(ed, opacity=0.2)
        assert len(fig.data) >= 2


# ─── K4: Forest Viz ──────────────────────────────────────────────────────────

class TestForestViz:
    def test_plot_forest_2d(self, tmp_json):
        from sbf5_viz.load_data import load_forest_data
        from sbf5_viz.forest_viz import plot_forest_2d
        path = tmp_json(SAMPLE_FOREST, "forest.json")
        fd = load_forest_data(path)
        fig = plot_forest_2d(fd, dim_x=0, dim_y=1)
        assert len(fig.data) == 3  # 3 boxes

    def test_plot_path_on_forest(self, tmp_json):
        from sbf5_viz.load_data import load_forest_data
        from sbf5_viz.forest_viz import plot_path_on_forest
        path_json = tmp_json(SAMPLE_FOREST, "forest.json")
        fd = load_forest_data(path_json)
        waypoints = [np.array([0.1, 0.1]), np.array([0.7, 0.3])]
        fig = plot_path_on_forest(fd, waypoints)
        assert len(fig.data) >= 5  # 3 boxes + path + start + goal


# ─── K5: Scene Viz ───────────────────────────────────────────────────────────

class TestSceneViz:
    def test_plot_scene_3d(self, tmp_json):
        from sbf5_viz.load_data import load_scene_data
        from sbf5_viz.scene_viz import plot_scene_3d
        path = tmp_json(SAMPLE_SCENE, "scene.json")
        sd = load_scene_data(path)
        fig = plot_scene_3d(sd)
        assert len(fig.data) >= 2  # 2 obstacles × (mesh + wire)


# ─── K6: Combined Viz ────────────────────────────────────────────────────────

class TestCombinedViz:
    def test_plot_combined_has_traces(self, tmp_json):
        from sbf5_viz.load_data import load_snapshot
        from sbf5_viz.combined_viz import plot_combined
        path = tmp_json(SAMPLE_SNAPSHOT, "snapshot.json")
        snap = load_snapshot(path)
        fig = plot_combined(snap)
        assert len(fig.data) >= 2  # robot + envelope + scene


# ─── K8: CLI ─────────────────────────────────────────────────────────────────

class TestCLI:
    def test_cli_offline_html(self, tmp_json, tmp_path):
        from sbf5_viz.load_data import load_snapshot
        from sbf5_viz.combined_viz import plot_combined
        json_path = tmp_json(SAMPLE_SNAPSHOT, "snapshot.json")
        html_path = str(tmp_path / "out.html")

        snap = load_snapshot(json_path)
        fig = plot_combined(snap)
        fig.write_html(html_path)
        assert os.path.exists(html_path)
        assert os.path.getsize(html_path) > 100

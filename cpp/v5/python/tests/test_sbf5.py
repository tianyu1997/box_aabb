"""Tests for SafeBoxForest v5 Python bindings."""
import os
import sys
import numpy as np
import pytest

# Allow running from build tree: add build dir to path
# (user must set SBF5_BUILD_DIR or the .pyd must be findable)
_build_dir = os.environ.get("SBF5_BUILD_DIR", "")
if _build_dir:
    sys.path.insert(0, _build_dir)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from sbf5 import (
    Interval, Obstacle, JointLimits, BoxNode,
    Robot, SBFPlanner, SBFPlannerConfig,
    GrowerConfig, GrowerMode, PlanResult,
    EndpointSource, EnvelopeType,
    EndpointSourceConfig, EnvelopeTypeConfig,
    GcpcCache,
)

DATA_DIR = os.path.join(os.path.dirname(__file__), "..", "..", "data")


class TestInterval:
    def test_basic(self):
        iv = Interval(0.0, 1.0)
        assert iv.lo == 0.0
        assert iv.hi == 1.0
        assert iv.width() == pytest.approx(1.0)
        assert iv.center() == pytest.approx(0.5)

    def test_empty(self):
        iv = Interval(1.0, 0.0)
        assert iv.empty()

    def test_repr(self):
        iv = Interval(0.0, 1.0)
        r = repr(iv)
        assert "0.0" in r
        assert "1.0" in r


class TestObstacle:
    def test_basic(self):
        obs = Obstacle(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0)
        mn = obs.min_point()
        mx = obs.max_point()
        np.testing.assert_allclose(mn, [-1, -1, -1])
        np.testing.assert_allclose(mx, [1, 1, 1])

    def test_bounds_property(self):
        obs = Obstacle()
        obs.bounds = [0.0, 0.1, 0.2, 1.0, 1.1, 1.2]
        b = obs.bounds
        assert len(b) == 6
        assert b[0] == pytest.approx(0.0)
        assert b[5] == pytest.approx(1.2)


class TestRobot:
    def test_load_2dof(self):
        path = os.path.join(DATA_DIR, "2dof_planar.json")
        robot = Robot.from_json(path)
        assert robot.n_joints() == 2
        assert robot.name() == "2dof_planar"

    def test_joint_limits(self):
        path = os.path.join(DATA_DIR, "2dof_planar.json")
        robot = Robot.from_json(path)
        lim = robot.joint_limits()
        assert lim.n_dims() == 2


class TestPlan2DOF:
    def test_plan_success(self):
        path = os.path.join(DATA_DIR, "2dof_planar.json")
        robot = Robot.from_json(path)

        config = SBFPlannerConfig()
        config.grower.max_boxes = 300
        config.grower.mode = GrowerMode.WAVEFRONT

        planner = SBFPlanner(robot, config)

        start = np.array([0.5, 0.5])
        goal = np.array([2.0, 1.0])
        # Small obstacle far from the robot workspace
        obstacles = [Obstacle(5.0, 5.0, 0.0, 6.0, 6.0, 1.0)]

        result = planner.plan(start, goal, obstacles, timeout_ms=30000.0)
        assert isinstance(result, PlanResult)
        assert result.success
        assert len(result.path) > 0
        assert result.path_length > 0.0
        assert result.n_boxes > 0

    def test_boxes_access(self):
        path = os.path.join(DATA_DIR, "2dof_planar.json")
        robot = Robot.from_json(path)
        planner = SBFPlanner(robot)

        start = np.array([0.5, 0.5])
        goal = np.array([2.0, 1.0])
        result = planner.plan(start, goal, [], timeout_ms=10000.0)

        boxes = planner.boxes()
        assert len(boxes) > 0
        b0 = boxes[0]
        assert b0.n_dims() == 2
        c = b0.center()
        assert len(c) == 2


class TestEndpointSourceEnum:
    def test_values_accessible(self):
        assert EndpointSource.IFK is not None
        assert EndpointSource.CritSample is not None
        assert EndpointSource.Analytical is not None
        assert EndpointSource.GCPC is not None


class TestEnvelopeTypeEnum:
    def test_values_accessible(self):
        assert EnvelopeType.LinkIAABB is not None
        assert EnvelopeType.LinkIAABB_Grid is not None
        assert EnvelopeType.Hull16_Grid is not None


class TestEndpointSourceConfig:
    def test_defaults(self):
        cfg = EndpointSourceConfig()
        assert cfg.source == EndpointSource.IFK
        assert cfg.n_samples_crit == 1000

    def test_set_source(self):
        cfg = EndpointSourceConfig()
        cfg.source = EndpointSource.Analytical
        assert cfg.source == EndpointSource.Analytical


class TestEnvelopeTypeConfig:
    def test_defaults(self):
        cfg = EnvelopeTypeConfig()
        assert cfg.type == EnvelopeType.LinkIAABB
        assert cfg.n_subdivisions == 1


class TestConfigPipeline:
    def test_12_configs_no_crash(self):
        sources = [
            EndpointSource.IFK,
            EndpointSource.CritSample,
            EndpointSource.Analytical,
            EndpointSource.GCPC,
        ]
        types = [
            EnvelopeType.LinkIAABB,
            EnvelopeType.LinkIAABB_Grid,
            EnvelopeType.Hull16_Grid,
        ]
        for src in sources:
            for env in types:
                cfg = SBFPlannerConfig()
                cfg.endpoint_source.source = src
                cfg.envelope_type.type = env
                # Just verify construction doesn't crash
                assert cfg.endpoint_source.source == src
                assert cfg.envelope_type.type == env


class TestPlanResultMetrics:
    def test_new_fields_exist(self):
        path = os.path.join(DATA_DIR, "2dof_planar.json")
        robot = Robot.from_json(path)
        config = SBFPlannerConfig()
        config.grower.max_boxes = 200
        config.grower.mode = GrowerMode.WAVEFRONT
        planner = SBFPlanner(robot, config)
        start = np.array([0.5, 0.5])
        goal = np.array([2.0, 1.0])
        obstacles = [Obstacle(5.0, 5.0, 0.0, 6.0, 6.0, 1.0)]
        result = planner.plan(start, goal, obstacles, timeout_ms=30000.0)
        assert hasattr(result, "build_time_ms")
        assert hasattr(result, "lect_time_ms")
        assert hasattr(result, "envelope_volume_total")
        if result.success:
            assert result.build_time_ms >= 0.0
            assert result.lect_time_ms >= 0.0

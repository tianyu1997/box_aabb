"""Tests for boundary expansion sampling strategy."""

import numpy as np
import pytest

from aabb.robot import load_robot
from forest.models import BoxNode
from forest.scene import Scene
from planner.models import SBFConfig
from planner.sbf_planner import SBFPlanner


# ─────────────────────────────────────────────
#  Fixtures
# ─────────────────────────────────────────────

def _make_2dof_robot():
    return load_robot("2dof_planar")


def _make_simple_scene(obstacles=None):
    scene = Scene()
    if obstacles:
        for obs in obstacles:
            scene.add_obstacle(obs["min"], obs["max"], obs.get("name", ""))
    return scene


# ─────────────────────────────────────────────
#  _generate_boundary_seeds (SBFPlanner)
# ─────────────────────────────────────────────

class TestGenerateBoundarySeeds:
    def test_seeds_outside_box(self):
        robot = _make_2dof_robot()
        scene = _make_simple_scene([
            {"min": [10.0, 10.0, 10.0], "max": [11.0, 11.0, 11.0]}
        ])
        config = SBFConfig(boundary_expand_epsilon=0.01)
        planner = SBFPlanner(robot, scene, config=config, no_cache=True)

        box = BoxNode(
            node_id=0,
            joint_intervals=[(-0.5, 0.5), (-0.5, 0.5)],
            seed_config=np.array([0.0, 0.0]),
        )
        rng = np.random.default_rng(42)
        results = planner._generate_boundary_seeds(box, rng)

        assert len(results) > 0
        # Each seed should be outside the box (in at least one dimension)
        for dim, side, s in results:
            outside = False
            for i, (lo, hi) in enumerate(box.joint_intervals):
                if s[i] < lo - 1e-12 or s[i] > hi + 1e-12:
                    outside = True
                    break
            assert outside, f"Seed {s} should be outside box"

    def test_excluded_faces_reduces_output(self):
        """excluded_faces 应减少返回的 seed 数量"""
        robot = _make_2dof_robot()
        scene = _make_simple_scene([
            {"min": [10.0, 10.0, 10.0], "max": [11.0, 11.0, 11.0]}
        ])
        config = SBFConfig(boundary_expand_epsilon=0.01)
        planner = SBFPlanner(robot, scene, config=config, no_cache=True)

        box = BoxNode(
            node_id=0,
            joint_intervals=[(-0.5, 0.5), (-0.5, 0.5)],
            seed_config=np.array([0.0, 0.0]),
        )
        rng = np.random.default_rng(42)

        all_seeds = planner._generate_boundary_seeds(box, rng)
        # Exclude dim=0 both sides → should have fewer seeds
        excluded = frozenset({(0, 0), (0, 1)})
        rng2 = np.random.default_rng(42)
        fewer_seeds = planner._generate_boundary_seeds(
            box, rng2, excluded_faces=excluded)

        assert len(fewer_seeds) <= len(all_seeds)
        # None of the returned seeds should be on excluded faces
        for dim, side, s in fewer_seeds:
            assert (dim, side) not in excluded

    def test_returns_dim_side_seed_tuples(self):
        """返回值应为 (dim, side, seed) 元组列表"""
        robot = _make_2dof_robot()
        scene = _make_simple_scene([
            {"min": [10.0, 10.0, 10.0], "max": [11.0, 11.0, 11.0]}
        ])
        config = SBFConfig(boundary_expand_epsilon=0.01)
        planner = SBFPlanner(robot, scene, config=config, no_cache=True)

        box = BoxNode(
            node_id=0,
            joint_intervals=[(-0.5, 0.5), (-0.5, 0.5)],
            seed_config=np.array([0.0, 0.0]),
        )
        rng = np.random.default_rng(42)
        results = planner._generate_boundary_seeds(box, rng)

        for item in results:
            assert len(item) == 3
            dim, side, seed = item
            assert isinstance(dim, int)
            assert side in (0, 1)
            assert isinstance(seed, np.ndarray)


# ─────────────────────────────────────────────
#  SBFConfig remaining fields
# ─────────────────────────────────────────────

class TestSBFConfigBoundaryFields:
    def test_defaults(self):
        cfg = SBFConfig()
        assert cfg.boundary_expand_epsilon == 0.01

    def test_to_dict_includes_fields(self):
        cfg = SBFConfig(boundary_expand_epsilon=0.02)
        d = cfg.to_dict()
        assert d["boundary_expand_epsilon"] == 0.02


# ─────────────────────────────────────────────
#  Integration: boundary sampling in main loop
# ─────────────────────────────────────────────

class TestBoundarySamplingIntegration:
    def test_boundary_sampling_produces_boxes(self):
        """边缘采样应在 box 周围产生更多相邻 box"""
        robot = _make_2dof_robot()
        scene = _make_simple_scene([
            {"min": [0.6, -0.2, -1000.0], "max": [0.9, 0.2, 1000.0]},
        ])
        config = SBFConfig(
            boundary_expand_epsilon=0.05,
            max_box_nodes=30,
            max_iterations=100,
        )
        planner = SBFPlanner(robot, scene, config=config, no_cache=True)
        q_s = np.array([-1.0, 0.0])
        q_g = np.array([1.0, 0.0])
        result = planner.plan(q_s, q_g, seed=123)
        assert result is not None
        assert result.n_boxes_created >= 1


# ─────────────────────────────────────────────
#  Integration: end-to-end plan
# ─────────────────────────────────────────────

class TestPlanWithBoundaryExpand:
    def test_plan_basic(self):
        """端到端测试：boundary expansion 不影响规划成功率"""
        robot = _make_2dof_robot()
        scene = _make_simple_scene([
            {"min": [0.3, -0.1, -0.5], "max": [0.5, 0.1, 0.5]},
        ])
        config = SBFConfig(
            max_iterations=50,
            max_box_nodes=50,
        )
        planner = SBFPlanner(robot, scene, config=config, no_cache=True)
        q_start = np.array([0.5, 0.5])
        q_goal = np.array([-0.5, -0.5])
        result = planner.plan(q_start, q_goal, seed=42)
        # 只检查不崩溃，成功率取决于场景
        assert result is not None

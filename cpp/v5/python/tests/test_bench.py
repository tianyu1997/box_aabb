"""Tests for sbf5_bench — baselines, metrics, experiment framework."""

import os
import sys
import json
import tempfile

import numpy as np
import pytest

# Ensure sbf5_bench is importable
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from sbf5_bench.base import BasePlanner, PlanningResult
from sbf5_bench.metrics import (
    PathMetrics,
    compute_path_length,
    compute_smoothness,
    compute_joint_range_usage,
    evaluate_result,
    format_comparison_table,
)
from sbf5_bench.scenes import get_scene, list_scenes, BenchmarkScene
from sbf5_bench.runner import ExperimentConfig, ExperimentResults, run_experiment
from sbf5_bench.report import summary_table, latex_table


# ── helpers ──────────────────────────────────────────────────

class DummyPlanner(BasePlanner):
    """A trivial planner that returns a straight line."""

    def __init__(self, name_str: str = "Dummy"):
        self._name = name_str

    @property
    def name(self) -> str:
        return self._name

    def setup(self, robot, scene, config=None):
        pass

    def plan(self, start, goal, timeout=30.0):
        start = np.asarray(start, dtype=np.float64)
        goal = np.asarray(goal, dtype=np.float64)
        n = 10
        path = np.array([
            start + t * (goal - start) for t in np.linspace(0, 1, n)
        ])
        cost = float(np.linalg.norm(goal - start))
        return PlanningResult(
            success=True, path=path, cost=cost, planning_time_s=0.001)


# ── PlanningResult tests ────────────────────────────────────

class TestPlanningResult:
    def test_to_dict_roundtrip(self):
        path = np.array([[0.0, 0.0], [1.0, 1.0], [2.0, 2.0]])
        pr = PlanningResult(
            success=True, path=path, cost=2.828,
            planning_time_s=0.5, collision_checks=100,
            nodes_explored=50, metadata={"algo": "test"})
        d = pr.to_dict()
        assert d["success"] is True
        assert d["n_waypoints"] == 3
        assert d["algo"] == "test"

    def test_failure_factory(self):
        pr = PlanningResult.failure("timeout")
        assert not pr.success
        assert pr.cost == float("inf")
        assert pr.metadata["reason"] == "timeout"

    def test_n_waypoints_none(self):
        pr = PlanningResult()
        assert pr.n_waypoints == 0


# ── Metrics tests ────────────────────────────────────────────

class TestPathMetrics:
    def test_straight_line_efficiency(self):
        """A straight-line path should have efficiency == 1.0."""
        pts = [np.array([0.0, 0.0]), np.array([3.0, 4.0])]
        length = compute_path_length(pts)
        direct = np.linalg.norm(pts[-1] - pts[0])
        assert length == pytest.approx(5.0)
        assert direct == pytest.approx(5.0)

    def test_path_length_multi(self):
        pts = [np.array([0.0]), np.array([1.0]), np.array([3.0])]
        assert compute_path_length(pts) == pytest.approx(3.0)

    def test_smoothness_straight(self):
        """Straight path has zero smoothness."""
        pts = [np.array([0.0, 0.0]),
               np.array([1.0, 0.0]),
               np.array([2.0, 0.0])]
        mean_s, max_s = compute_smoothness(pts)
        assert mean_s == pytest.approx(0.0, abs=1e-10)
        assert max_s == pytest.approx(0.0, abs=1e-10)

    def test_smoothness_right_angle(self):
        """90-degree turn."""
        pts = [np.array([0.0, 0.0]),
               np.array([1.0, 0.0]),
               np.array([1.0, 1.0])]
        mean_s, max_s = compute_smoothness(pts)
        assert mean_s == pytest.approx(np.pi / 2, abs=1e-6)

    def test_joint_range_usage(self):
        pts = [np.array([0.0, 1.0]),
               np.array([1.0, 1.5]),
               np.array([0.5, 2.0])]
        limits = [(-np.pi, np.pi), (-np.pi, np.pi)]
        usage = compute_joint_range_usage(pts, limits)
        assert len(usage) == 2
        # joint 0: range 1.0 / (2*pi)
        assert usage[0] == pytest.approx(1.0 / (2 * np.pi), abs=1e-4)

    def test_evaluate_result_success(self):
        path = np.array([
            [0.0, 0.0],
            [0.5, 0.5],
            [1.0, 1.0],
        ])
        pr = PlanningResult(success=True, path=path, cost=1.414,
                            planning_time_s=0.1)
        m = evaluate_result(pr)
        assert m.n_waypoints == 3
        assert m.path_length > 0
        assert m.efficiency > 0

    def test_evaluate_result_failure(self):
        pr = PlanningResult.failure("test")
        m = evaluate_result(pr)
        assert m.n_waypoints == 0
        assert m.path_length == 0.0


# ── Scenes tests ─────────────────────────────────────────────

class TestScenes:
    def test_list_scenes(self):
        names = list_scenes()
        assert "2dof_simple" in names
        assert "panda_tabletop" in names

    def test_get_scene(self):
        s = get_scene("2dof_simple")
        assert s.name == "2dof_simple"
        assert len(s.obstacles) >= 1
        assert len(s.start) == 2

    def test_unknown_scene(self):
        with pytest.raises(KeyError):
            get_scene("nonexistent_scene")


# ── Report tests ─────────────────────────────────────────────

class TestReport:
    def _make_results(self):
        trials = []
        from sbf5_bench.runner import TrialResult
        for i in range(3):
            path = np.array([[0.0, 0.0], [1.0, 1.0]])
            pr = PlanningResult(success=True, path=path, cost=1.414,
                                planning_time_s=0.01 * (i + 1))
            m = evaluate_result(pr)
            trials.append(TrialResult(
                scene="2dof_simple", planner="Dummy",
                seed=i, trial_idx=i,
                planning_result=pr, metrics=m))
        return ExperimentResults(trials=trials, timestamp="test")

    def test_summary_table_format(self):
        r = self._make_results()
        table = summary_table(r)
        assert "Planner" in table
        assert "Scene" in table
        assert "Dummy" in table

    def test_latex_table_format(self):
        r = self._make_results()
        tex = latex_table(r)
        assert r"\\toprule" in tex or "\\toprule" in tex
        assert "Dummy" in tex

    def test_format_comparison_table(self):
        path = np.array([[0.0, 0.0], [1.0, 1.0], [2.0, 2.0]])
        pr1 = PlanningResult(success=True, path=path, cost=2.0,
                             planning_time_s=0.1)
        pr2 = PlanningResult(success=True, path=path, cost=3.0,
                             planning_time_s=0.2)
        m1 = evaluate_result(pr1)
        m2 = evaluate_result(pr2)
        table = format_comparison_table({"A": m1, "B": m2})
        assert "| A |" in table or "A" in table
        assert "Path Length" in table


# ── Runner tests (uses DummyPlanner, no C++ needed) ──────────

class TestRunner:
    def test_experiment_results_save_load(self):
        from sbf5_bench.runner import TrialResult
        path = np.array([[0.0, 0.0], [1.0, 1.0]])
        pr = PlanningResult(success=True, path=path, cost=1.0,
                            planning_time_s=0.01)
        m = evaluate_result(pr)
        trial = TrialResult(scene="test", planner="Dummy",
                            seed=0, trial_idx=0,
                            planning_result=pr, metrics=m)
        results = ExperimentResults(
            trials=[trial], config={"test": True}, timestamp="now")

        with tempfile.NamedTemporaryFile(
            mode='w', suffix='.json', delete=False
        ) as f:
            tmp = f.name

        try:
            results.save(tmp)
            loaded = ExperimentResults.load(tmp)
            assert len(loaded.trials) == 1
            assert loaded.trials[0].planner == "Dummy"
            assert loaded.trials[0].planning_result.success is True
        finally:
            os.unlink(tmp)


# ── SBF Adapter integration test (requires C++ .pyd) ────────

class TestSBFAdapter:
    @pytest.fixture
    def sbf5_available(self):
        try:
            import sbf5
            return True
        except ImportError:
            pytest.skip("sbf5 C++ bindings not available")

    def test_sbf_adapter_plan(self, sbf5_available):
        from sbf5_bench.sbf_adapter import SBFPlannerAdapter
        from sbf5_bench.scenes import get_scene

        scene = get_scene("2dof_simple")
        robot = scene.make_robot()
        obstacles = scene.make_obstacles()

        adapter = SBFPlannerAdapter()
        adapter.setup(robot, obstacles, {"max_boxes": 200})

        result = adapter.plan(scene.start, scene.goal, timeout=10.0)
        # May or may not succeed depending on scene difficulty
        assert isinstance(result, PlanningResult)
        if result.success:
            assert result.path is not None


# ── Pipeline config tests (Phase R) ──────────────────────────

class TestPipelineConfig:
    def test_all_pipeline_configs(self):
        from sbf5_bench.runner import PipelineConfig, ALL_PIPELINE_CONFIGS
        assert len(ALL_PIPELINE_CONFIGS) == 12
        for pc in ALL_PIPELINE_CONFIGS:
            assert isinstance(pc, PipelineConfig)
            assert pc.label == f"{pc.endpoint_source}-{pc.envelope_type}"

    def test_experiment_config_pipeline_field(self):
        from sbf5_bench.runner import ExperimentConfig, PipelineConfig
        cfg = ExperimentConfig(
            scenes=["2dof_simple"],
            planners=[],
            pipeline_configs=[PipelineConfig("IFK", "LinkIAABB")],
            n_trials=1,
        )
        assert cfg.pipeline_configs is not None
        assert len(cfg.pipeline_configs) == 1


class TestSBFAdapterPipeline:
    @pytest.fixture
    def sbf5_available(self):
        try:
            import sbf5
            return True
        except ImportError:
            pytest.skip("sbf5 C++ bindings not available")

    def test_adapter_with_pipeline(self, sbf5_available):
        from sbf5_bench.sbf_adapter import SBFPlannerAdapter
        from sbf5_bench.scenes import get_scene

        scene = get_scene("2dof_simple")
        robot = scene.make_robot()
        obstacles = scene.make_obstacles()

        adapter = SBFPlannerAdapter(
            endpoint_source="Analytical",
            envelope_type="LinkIAABB",
        )
        adapter.setup(robot, obstacles, {"max_boxes": 200})
        assert adapter.name == "SBF-Analytical-LinkIAABB"

        result = adapter.plan(scene.start, scene.goal, timeout=10.0)
        assert isinstance(result, PlanningResult)
        if result.success:
            assert "build_time_ms" in result.metadata
            assert "lect_time_ms" in result.metadata
            assert "envelope_volume_total" in result.metadata


# ══════════════════════════════════════════════════════════════
# Phase T: Paper output tests
# ══════════════════════════════════════════════════════════════

from sbf5_bench.report import (
    envelope_volume_table,
    timing_table,
    e2e_table,
    baseline_table,
)


class TestPhaseT_Tables:
    """Test the 4 LaTeX table generators with mock data."""

    def _mock_s1(self):
        return {"rows": [
            {"endpoint": "IFK", "envelope": "LinkIAABB",
             "volume_7dof": 12.5, "ratio_pct": 100.0, "safe": True},
            {"endpoint": "GCPC", "envelope": "Hull16_Grid",
             "volume_7dof": 7.1, "ratio_pct": 56.8, "safe": True},
        ]}

    def _mock_s2(self):
        return {"rows": [
            {"endpoint": "IFK", "envelope": "LinkIAABB",
             "ep_us": 15, "env_us": 2, "total_us": 17, "speedup": 35.0},
            {"endpoint": "Analytical", "envelope": "LinkIAABB",
             "ep_us": 580, "env_us": 2, "total_us": 582, "speedup": 1.0},
        ]}

    def _mock_e2e_results(self):
        from sbf5_bench.runner import TrialResult
        trials = []
        for pl in ["SBF-GCPC-LinkIAABB_Grid", "SBF-IFK-LinkIAABB"]:
            for sc in ["2dof_simple", "panda_tabletop"]:
                for seed in range(5):
                    path = np.array([[0.0, 0.0], [1.0, 1.0]])
                    pr = PlanningResult(
                        success=True, path=path, cost=1.414,
                        planning_time_s=0.01 + 0.002 * seed)
                    m = evaluate_result(pr)
                    trials.append(TrialResult(
                        scene=sc, planner=pl, seed=seed,
                        trial_idx=seed, planning_result=pr, metrics=m))
        return ExperimentResults(trials=trials, timestamp="test")

    def _mock_baseline_results(self):
        from sbf5_bench.runner import TrialResult
        trials = []
        for pl, t, l in [("SBF-GCPC-Grid", 1.2, 5.8),
                          ("RRTConnect", 0.8, 8.2),
                          ("RRT*", 5.2, 4.9)]:
            for seed in range(5):
                path = np.array([[0.0, 0.0], [l / 2, l / 2], [l, l]])
                pr = PlanningResult(
                    success=True, path=path, cost=l,
                    planning_time_s=t + 0.01 * seed)
                m = evaluate_result(pr)
                m.smoothness_mean = 0.15 + 0.01 * seed
                trials.append(TrialResult(
                    scene="mixed", planner=pl, seed=seed,
                    trial_idx=seed, planning_result=pr, metrics=m))
        return ExperimentResults(trials=trials, timestamp="test")

    def test_envelope_volume_table(self):
        tex = envelope_volume_table(self._mock_s1())
        assert r"\begin{table}" in tex
        assert r"\toprule" in tex
        assert r"\bottomrule" in tex
        assert "IFK" in tex
        assert "GCPC" in tex
        # Best value should be bold
        assert r"\textbf{7.1}" in tex

    def test_timing_table(self):
        tex = timing_table(self._mock_s2())
        assert r"\begin{table}" in tex
        assert r"\toprule" in tex
        assert "IFK" in tex
        assert "Analytical" in tex
        # Best total (17) should be bold
        assert r"\textbf{17}" in tex

    def test_e2e_table(self):
        results = self._mock_e2e_results()
        tex = e2e_table(results)
        assert r"\begin{table*}" in tex
        assert "Success Rate" in tex
        assert "Planning Time" in tex
        assert r"\bottomrule" in tex

    def test_baseline_table(self):
        results = self._mock_baseline_results()
        tex = baseline_table(results)
        assert r"\begin{table}" in tex
        assert "RRTConnect" in tex
        assert r"\textbf{" in tex
        assert r"\bottomrule" in tex

    def test_empty_results(self):
        empty = ExperimentResults(trials=[], timestamp="test")
        assert "No trials" in e2e_table(empty)
        assert "No trials" in baseline_table(empty)


class TestPhaseT_Figures:
    """Test figure generators produce valid Plotly figures."""

    def test_fig_envelope_timing(self):
        try:
            import plotly
        except ImportError:
            pytest.skip("plotly not installed")
        from scripts.gen_figures import fig_envelope_timing

        s2 = {"rows": [
            {"endpoint": "IFK", "envelope": "LinkIAABB",
             "ep_us": 15, "env_us": 2, "total_us": 17, "speedup": 35.0},
            {"endpoint": "IFK", "envelope": "LinkIAABB_Grid",
             "ep_us": 15, "env_us": 25, "total_us": 40, "speedup": 14.6},
        ]}
        fig = fig_envelope_timing(s2)
        assert fig is not None
        assert len(fig.data) == 2  # 2 envelope types

    def test_fig_scalability(self):
        try:
            import plotly
        except ImportError:
            pytest.skip("plotly not installed")
        from scripts.gen_figures import fig_scalability

        s5 = {
            "time_vs_dof": {
                "SBF": {"dofs": [2, 7], "times_mean": [0.2, 0.7],
                         "times_std": [0.04, 0.14]}
            },
            "success_vs_obstacles": {
                "SBF": {"n_obstacles": [1, 5], "success_rate": [100, 80]}
            },
            "success_vs_max_boxes": {
                "SBF": {"max_boxes": [100, 1000], "success_rate": [40, 90]}
            },
        }
        fig = fig_scalability(s5)
        assert fig is not None
        assert len(fig.data) >= 3  # at least one per subplot


class TestPhaseT_Stats:
    """Test statistical significance module."""

    def _mock_paired_results(self):
        """Two planners with different distributions on same scenes/seeds."""
        from sbf5_bench.runner import TrialResult

        trials = []
        rng = np.random.RandomState(42)
        for seed in range(20):
            for pl, mean_t in [("PlannerA", 1.0), ("PlannerB", 2.0)]:
                t = max(0.01, mean_t + rng.randn() * 0.3)
                path = np.array([[0.0, 0.0], [1.0, 1.0]])
                pr = PlanningResult(
                    success=True, path=path, cost=1.414,
                    planning_time_s=t)
                m = evaluate_result(pr)
                trials.append(TrialResult(
                    scene="test_scene", planner=pl, seed=seed,
                    trial_idx=seed, planning_result=pr, metrics=m))
        return ExperimentResults(trials=trials, timestamp="test")

    def test_pairwise_significance(self):
        try:
            from scipy.stats import wilcoxon
        except ImportError:
            pytest.skip("scipy not installed")
        from sbf5_bench.stats import pairwise_significance

        results = self._mock_paired_results()
        sig = pairwise_significance(results, metric="planning_time_s")

        assert len(sig) == 1
        key = ("PlannerA", "PlannerB")
        assert key in sig
        info = sig[key]
        assert info["p_value"] < 0.05  # clearly different distributions
        assert info["significant"] is True
        assert info["effect_direction"] == "a < b"

    def test_annotate_table(self):
        try:
            from scipy.stats import wilcoxon
        except ImportError:
            pytest.skip("scipy not installed")
        from sbf5_bench.stats import annotate_table_significance

        latex = r"PlannerA & 90 & 1.0 \\"
        sig = {("RefPlanner", "PlannerA"): {
            "significant": True, "effect_direction": "a > b"}}
        result = annotate_table_significance(
            latex, sig, reference_planner="RefPlanner")
        assert "$^*$" in result

    def test_save_load_significance(self):
        try:
            from scipy.stats import wilcoxon
        except ImportError:
            pytest.skip("scipy not installed")
        from sbf5_bench.stats import save_significance

        sig = {("A", "B"): {"statistic": 10.0, "p_value": 0.03,
                             "significant": True, "effect_direction": "a < b",
                             "n_pairs": 20}}
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".json", delete=False
        ) as f:
            tmp = f.name

        try:
            save_significance(sig, tmp)
            with open(tmp, "r") as f:
                data = json.load(f)
            assert "A vs B" in data
            assert data["A vs B"]["significant"] is True
        finally:
            os.unlink(tmp)

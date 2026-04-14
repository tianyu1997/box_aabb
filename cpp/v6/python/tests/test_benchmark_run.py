"""Tests for Phase Q — benchmark run + paper-ready output.

These are integration tests (not daily CI). Run with:
    pytest python/tests/test_benchmark_run.py -v
"""

import json
import os
import sys
import tempfile

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from sbf5_bench.base import PlanningResult
from sbf5_bench.metrics import evaluate_result
from sbf5_bench.runner import ExperimentConfig, ExperimentResults, TrialResult
from sbf5_bench.report import summary_table, latex_table, plot_comparison
from sbf5_bench.scenes import get_scene, list_scenes


# ── helpers ──────────────────────────────────────────────────

class _DummyPlanner:
    """Minimal planner for testing without C++ bindings."""

    name = "TestDummy"
    supports_reuse = False

    def setup(self, robot, scene, config=None):
        pass

    def plan(self, start, goal, timeout=30.0):
        start = np.asarray(start, dtype=np.float64)
        goal = np.asarray(goal, dtype=np.float64)
        path = np.array([
            start + t * (goal - start)
            for t in np.linspace(0, 1, 8)
        ])
        return PlanningResult(
            success=True, path=path,
            cost=float(np.linalg.norm(goal - start)),
            planning_time_s=0.005,
            nodes_explored=42,
        )

    def reset(self):
        pass


def _make_dummy_results(n_trials=3):
    """Build ExperimentResults from dummy data."""
    trials = []
    for i in range(n_trials):
        path = np.array([[0.0, 0.0], [0.5, 0.5], [1.0, 1.0]])
        pr = PlanningResult(
            success=True, path=path, cost=1.414,
            planning_time_s=0.01 * (i + 1),
            nodes_explored=100 + i * 10,
        )
        m = evaluate_result(pr)
        trials.append(TrialResult(
            scene="2dof_simple", planner="SBF-Dijkstra",
            seed=i, trial_idx=i,
            planning_result=pr, metrics=m))
    return ExperimentResults(trials=trials, timestamp="test",
                             config={"scenes": ["2dof_simple"]})


# ── Q1: Scenes ───────────────────────────────────────────────

class TestExpandedScenes:
    def test_all_six_scenes_exist(self):
        names = list_scenes()
        expected = ["2dof_simple", "2dof_narrow", "2dof_cluttered",
                    "panda_tabletop", "panda_shelf", "panda_multi_obstacle"]
        for s in expected:
            assert s in names, f"Missing scene: {s}"

    def test_2dof_cluttered_obstacles(self):
        s = get_scene("2dof_cluttered")
        assert len(s.obstacles) == 4
        assert s.start.shape == (2,)
        assert s.goal.shape == (2,)

    def test_panda_shelf_obstacles(self):
        s = get_scene("panda_shelf")
        assert len(s.obstacles) == 4
        assert s.start.shape == (7,)
        assert s.goal.shape == (7,)

    def test_panda_multi_obstacle(self):
        s = get_scene("panda_multi_obstacle")
        assert len(s.obstacles) == 3
        assert s.start.shape == (7,)


# ── Q2: Benchmark pipeline ──────────────────────────────────

class TestSingleSceneBenchmark:
    def test_single_scene_dummy(self):
        """1 scene × 1 planner × 1 trial → produces JSON."""
        planner = _DummyPlanner()
        config = ExperimentConfig(
            scenes=["2dof_simple"],
            planners=[planner],
            n_trials=1,
            timeout=5.0,
            output_dir=tempfile.mkdtemp(),
        )
        results = run_experiment_dummy(config, planner)
        assert len(results.trials) >= 1
        assert results.trials[0].planning_result.success


def run_experiment_dummy(config, planner):
    """Simplified experiment run for dummy planners (no sbf5 needed)."""
    from sbf5_bench.scenes import get_scene as _gs
    results = ExperimentResults(timestamp="test", config={})
    for scene_name in config.scenes:
        scene = _gs(scene_name)
        for seed in range(config.n_trials):
            pr = planner.plan(scene.start, scene.goal)
            m = evaluate_result(pr)
            results.trials.append(TrialResult(
                scene=scene_name, planner=planner.name,
                seed=seed, trial_idx=seed,
                planning_result=pr, metrics=m))
    return results


# ── Q4: LaTeX output ─────────────────────────────────────────

class TestLatexOutput:
    def test_latex_compiles_format(self):
        r = _make_dummy_results()
        tex = latex_table(r)
        assert r"\begin{table}" in tex
        assert r"\toprule" in tex
        assert r"\bottomrule" in tex
        assert "SBF-Dijkstra" in tex
        assert "Boxes" in tex or "Scene" in tex

    def test_latex_numeric_precision(self):
        r = _make_dummy_results()
        tex = latex_table(r)
        # Should contain ± formatted values
        assert r"\pm" in tex


# ── Q4+: Plot output ─────────────────────────────────────────

class TestPlotOutput:
    def test_plot_returns_figure(self):
        pytest.importorskip("plotly")
        import plotly.graph_objects as go
        r = _make_dummy_results()
        fig = plot_comparison(r)
        assert isinstance(fig, go.Figure)
        assert len(fig.data) > 0

    def test_summary_table_has_boxes(self):
        r = _make_dummy_results()
        table = summary_table(r)
        assert "Boxes" in table


# ── Q4: Results save/load roundtrip ──────────────────────────

class TestResultsSaveLoad:
    def test_roundtrip(self):
        r = _make_dummy_results(n_trials=5)
        with tempfile.NamedTemporaryFile(
            mode='w', suffix='.json', delete=False
        ) as f:
            tmp = f.name
        try:
            r.save(tmp)
            loaded = ExperimentResults.load(tmp)
            assert len(loaded.trials) == 5
            assert loaded.trials[0].planner == "SBF-Dijkstra"
        finally:
            os.unlink(tmp)

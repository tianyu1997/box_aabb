"""Test a single trial to isolate segfault."""
import sys, os
os.chdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, "python")
os.environ["PYTHONPATH"] = "build_x64/Release;python"

from sbf5_bench.runner import PipelineConfig, ExperimentConfig, run_experiment

config = ExperimentConfig(
    scenes=["2dof_narrow"],
    planners=[],
    pipeline_configs=[
        PipelineConfig("IFK", "LinkIAABB"),
        PipelineConfig("IFK", "LinkIAABB_Grid"),
        PipelineConfig("IFK", "Hull16_Grid"),
        PipelineConfig("CritSample", "LinkIAABB"),
        PipelineConfig("CritSample", "LinkIAABB_Grid"),
        PipelineConfig("CritSample", "Hull16_Grid"),
    ],
    n_trials=30,
    timeout=60.0,
    output_dir="experiments/results/_test_crash",
)
print(f"Running {1*6*30}=180 trials (2dof_narrow, cache disabled)")
import faulthandler; faulthandler.enable()
results = run_experiment(config)
with open("_test_crash_result.txt", "w") as f:
    f.write(f"Done! {len(results.trials)} trials\n")
print(f"Done! {len(results.trials)} trials")

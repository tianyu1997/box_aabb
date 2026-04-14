#!/usr/bin/env python3
"""Run S3 + S4 with subprocess isolation per scene to avoid cumulative C++ memory bugs.
Skips Analytical/GCPC, runs only IFK + CritSample."""
import sys, os, json, time, subprocess, logging

os.chdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, "python")
os.environ["PYTHONPATH"] = "build_x64/Release;python"
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
log = logging.getLogger(__name__)

PYTHON = sys.executable
EP_FAST = ["IFK", "CritSample"]
ENV_TYPES = ["LinkIAABB", "LinkIAABB_Grid", "Hull16_Grid"]

S3_SCENES = ["2dof_simple", "2dof_narrow", "2dof_cluttered",
             "panda_tabletop", "panda_shelf", "panda_multi_obstacle"]
S3_DIR = "experiments/results/s3_e2e"
S4_DIR = "experiments/results/s4_baselines"
N_TRIALS = 30


def run_s3_chunk_subprocess(scene: str, ep_source: str, env_types: list,
                            n_trials: int, out_dir: str) -> bool:
    """Run a single scene × endpoint_source chunk in a subprocess."""
    chunk_key = f"{scene}__{ep_source}"
    script = f'''
import sys, os, json, logging, glob
sys.path.insert(0, "python")
os.environ["PYTHONPATH"] = "build_x64/Release;python"
os.chdir(r"{os.getcwd()}")
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")

# Clear LECT cache to avoid cross-scene corruption
cache_dir = os.path.join(os.path.expanduser("~"), ".sbf_cache")
for f in glob.glob(os.path.join(cache_dir, "*.lect")):
    os.remove(f)

from sbf5_bench.runner import PipelineConfig, ExperimentConfig, run_experiment

pipeline_configs = [
    PipelineConfig({ep_source!r}, env)
    for env in {env_types!r}
]

gcpc_path = os.path.join("data", "panda_5000.gcpc")
gcpc = gcpc_path if os.path.exists(gcpc_path) else None
if not gcpc:
    pipeline_configs = [p for p in pipeline_configs if p.endpoint_source != "GCPC"]

config = ExperimentConfig(
    scenes=[{scene!r}],
    planners=[],
    pipeline_configs=pipeline_configs,
    gcpc_cache_path=gcpc,
    n_trials={n_trials},
    timeout=60.0,
    output_dir=r"{out_dir}",
)

results = run_experiment(config)
out_path = os.path.join(r"{out_dir}", f"chunk_{chunk_key!r}.json")
results.save(out_path)
print(f"CHUNK_DONE: {chunk_key!r} trials={{len(results.trials)}}")
'''
    log.info("  subprocess: %s × %s (%d env × %d seeds = %d trials)",
             scene, ep_source, len(env_types), n_trials, len(env_types) * n_trials)
    t0 = time.time()
    result = subprocess.run(
        [PYTHON, "-c", script],
        capture_output=True, text=True, timeout=3600,
    )
    dt = time.time() - t0

    if result.returncode != 0:
        log.error("  CRASHED %s × %s (exit=%d) after %.1fs",
                  scene, ep_source, result.returncode, dt)
        stderr_lines = result.stderr.strip().split('\n')
        for line in stderr_lines[-10:]:
            log.error("    %s", line)
        return False

    log.info("  done %s × %s in %.1fs", scene, ep_source, dt)
    return True


def merge_chunk_results(out_dir: str, chunk_keys: list, final_path: str):
    """Merge chunk result files into a single results.json."""
    from sbf5_bench.runner import ExperimentResults
    merged = None
    for ck in chunk_keys:
        path = os.path.join(out_dir, f"chunk_{ck!r}.json")
        if not os.path.exists(path):
            log.warning("Missing chunk %s", ck)
            continue
        r = ExperimentResults.load(path)
        if merged is None:
            merged = r
        else:
            merged.trials.extend(r.trials)
    if merged:
        merged.save(final_path)
        log.info("Merged %d trials → %s", len(merged.trials), final_path)
    return merged


def run_s3():
    os.makedirs(S3_DIR, exist_ok=True)
    final_path = os.path.join(S3_DIR, "results.json")
    if os.path.exists(final_path):
        log.info("S3 results already exist, skipping")
        return

    chunk_keys = []
    for scene in S3_SCENES:
        for ep in EP_FAST:
            ck = f"{scene}__{ep}"
            chunk_keys.append(ck)
            chunk_path = os.path.join(S3_DIR, f"chunk_{ck!r}.json")
            if os.path.exists(chunk_path):
                log.info("Chunk %s already done, skipping", ck)
                continue
            run_s3_chunk_subprocess(scene, ep, ENV_TYPES, N_TRIALS, S3_DIR)

    merge_chunk_results(S3_DIR, chunk_keys, final_path)


def run_s4():
    os.makedirs(S4_DIR, exist_ok=True)
    final_path = os.path.join(S4_DIR, "results.json")
    if os.path.exists(final_path):
        log.info("S4 results already exist, skipping")
        return

    scenes = ["2dof_simple", "2dof_narrow", "panda_tabletop", "panda_shelf"]
    chunk_keys = []
    for scene in scenes:
        for ep in EP_FAST:
            ck = f"{scene}__{ep}"
            chunk_keys.append(ck)
            chunk_path = os.path.join(S4_DIR, f"chunk_{ck!r}.json")
            if os.path.exists(chunk_path):
                log.info("S4 chunk %s already done", ck)
                continue
            run_s3_chunk_subprocess(scene, ep, ["LinkIAABB"], N_TRIALS, S4_DIR)

    merge_chunk_results(S4_DIR, chunk_keys, final_path)


if __name__ == "__main__":
    t0 = time.time()
    log.info("=== S3: E2E Benchmark (IFK + CritSample, subprocess isolation) ===")
    run_s3()
    log.info("=== S4: Baselines (IFK + CritSample, skip OMPL) ===")
    run_s4()
    log.info("=== ALL DONE in %.1fs ===", time.time() - t0)

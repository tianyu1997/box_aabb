#!/usr/bin/env python3
"""Run all experiments sequentially: S1, S2, S5, then S3."""
import sys, os, time, logging

os.chdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, "python")
os.environ["PYTHONPATH"] = "build_x64/Release;python"

# Redirect stderr + logging to TEMP (avoids OneDrive file-lock issues)
import tempfile
_log_dir = os.path.join(tempfile.gettempdir(), "sbf5_experiments")
os.makedirs(_log_dir, exist_ok=True)
_stderr_path = os.path.join(_log_dir, "run_stderr.txt")
_stderr_file = open(_stderr_path, "w", encoding="utf-8", buffering=1)
sys.stderr = _stderr_file
logging.basicConfig(stream=_stderr_file, level=logging.INFO, force=True)
print(f"stderr → {_stderr_path}", flush=True)

from scripts.run_all_experiments import run_s1, run_s2, run_s3, run_s4, run_s5

log = open(os.path.join(_log_dir, "experiment_log.txt"), "w", buffering=1)
print(f"log → {os.path.join(_log_dir, 'experiment_log.txt')}", flush=True)

def run_phase(name, fn, **kw):
    t0 = time.time()
    log.write(f"[{name}] START at {time.strftime('%H:%M:%S')}\n")
    log.flush()
    try:
        fn(**kw)
        dt = time.time() - t0
        log.write(f"[{name}] DONE in {dt:.1f}s\n")
    except Exception as e:
        dt = time.time() - t0
        log.write(f"[{name}] FAILED after {dt:.1f}s: {e}\n")
    log.flush()

run_phase("S1", run_s1, quick=False, lite=False)
run_phase("S2", run_s2, quick=False, lite=False)
run_phase("S5", run_s5, quick=False, lite=False)
run_phase("S3", run_s3, quick=False, lite=False)
run_phase("S4", run_s4, quick=False, lite=False, skip_ompl=True)

# ── Post-experiment: generate tables, figures ──
log.write(f"[GEN] START at {time.strftime('%H:%M:%S')}\n"); log.flush()
try:
    from scripts.gen_paper_tables import main as gen_tables_main
    gen_tables_main()
    log.write("[GEN] Tables done\n")
except Exception as e:
    log.write(f"[GEN] Tables FAILED: {e}\n")
log.flush()

try:
    from scripts.gen_figures import main as gen_figures_main
    gen_figures_main()
    log.write("[GEN] Figures done\n")
except Exception as e:
    log.write(f"[GEN] Figures FAILED: {e}\n")
log.flush()

log.write("ALL_DONE\n")
log.close()
print("ALL_EXPERIMENTS_DONE")

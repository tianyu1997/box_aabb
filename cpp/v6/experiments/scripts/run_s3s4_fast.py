#!/usr/bin/env python3
"""Run S3 + S4 with IFK/CritSample only (skip Analytical/GCPC)."""
import sys, os, time, logging

os.chdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, "python")
os.environ["PYTHONPATH"] = "build_x64/Release;python"

import tempfile
_log_dir = os.path.join(tempfile.gettempdir(), "sbf5_experiments")
os.makedirs(_log_dir, exist_ok=True)
_stderr_path = os.path.join(_log_dir, "run_s3s4_stderr.txt")
_stderr_file = open(_stderr_path, "w", encoding="utf-8", buffering=1)
sys.stderr = _stderr_file
logging.basicConfig(stream=_stderr_file, level=logging.INFO, force=True)
print(f"stderr → {_stderr_path}", flush=True)

from scripts.run_all_experiments import run_s3, run_s4

_log_path = os.path.join(_log_dir, "s3s4_log.txt")
log = open(_log_path, "w", buffering=1)
print(f"log → {_log_path}", flush=True)

EP_FAST = ["IFK", "CritSample"]

def run_phase(name, fn, **kw):
    t0 = time.time()
    msg = f"[{name}] START at {time.strftime('%H:%M:%S')}"
    log.write(msg + "\n"); log.flush()
    print(msg, flush=True)
    try:
        fn(**kw)
        dt = time.time() - t0
        msg = f"[{name}] DONE in {dt:.1f}s"
    except Exception as e:
        dt = time.time() - t0
        msg = f"[{name}] FAILED after {dt:.1f}s: {e}"
    log.write(msg + "\n"); log.flush()
    print(msg, flush=True)

run_phase("S3", run_s3, quick=False, lite=False, ep_sources=EP_FAST)
run_phase("S4", run_s4, quick=False, lite=False, skip_ompl=True, ep_sources=EP_FAST)

log.write("ALL_DONE\n"); log.close()
print("ALL_DONE", flush=True)

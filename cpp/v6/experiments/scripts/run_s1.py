#!/usr/bin/env python3
"""Run S1 full experiment."""
import sys, os, time
os.chdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, "python")
os.environ.setdefault("PYTHONPATH", "build_x64/Release;python")
from scripts.run_all_experiments import run_s1
t0 = time.time()
run_s1(quick=False, lite=False)
print(f"S1_FULL_DONE in {time.time()-t0:.1f}s")

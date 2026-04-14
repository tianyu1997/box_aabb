#!/usr/bin/env python3
"""Wrapper: run run_all.py with stderr -> file, so Start-Process won't deadlock."""
import sys, os, logging

os.chdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

# Redirect stderr to a file so Start-Process doesn't deadlock
log_path = os.path.join("experiments", "results", "run_stderr2.txt")
os.makedirs(os.path.dirname(log_path), exist_ok=True)
sys.stderr = open(log_path, "w", encoding="utf-8", buffering=1)

# Also redirect logging to file
logging.basicConfig(stream=sys.stderr, level=logging.INFO)

# Now run the actual experiment
exec(open(os.path.join("experiments", "scripts", "run_all.py")).read())

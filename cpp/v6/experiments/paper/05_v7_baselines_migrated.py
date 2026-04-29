#!/usr/bin/env python3
"""Compatibility entry for the old v7 baseline wrapper name.

The paper-facing implementation is now ``04_baselines_marcucci.py`` and all
baseline scripts/binaries live under ``cpp/v6``.
"""
from __future__ import annotations

import runpy
import sys
from pathlib import Path


if __name__ == "__main__":
    sys.stderr.write(
        "[note] 05_v7_baselines_migrated.py is deprecated; forwarding to "
        "04_baselines_marcucci.py.\n"
    )
    runpy.run_path(str(Path(__file__).resolve().parent / "04_baselines_marcucci.py"), run_name="__main__")

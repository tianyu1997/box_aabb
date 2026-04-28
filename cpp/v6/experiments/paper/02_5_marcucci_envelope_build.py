#!/usr/bin/env python3
"""Legacy compatibility shim for the Marcucci envelope build replay.

Current paper numbering uses ``03_marcucci_envelope_build.py`` as Exp.3.
This file remains only for older commands that still reference the interim
``02_5`` name.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from marcucci_envelope_build_replay import main


if __name__ == "__main__":
    main()
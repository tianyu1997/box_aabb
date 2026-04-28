#!/usr/bin/env python3
"""Legacy compatibility shim for the Marcucci SBF cached-query workload.

Current paper numbering uses ``04_e2e_baselines_combined.py`` as Exp.4.
This file delegates there so older commands cannot accidentally run the stale
Exp.3-era defaults that used to live here.
"""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from typing import Any


CURRENT_ENTRYPOINT = Path(__file__).resolve().with_name("04_e2e_baselines_combined.py")


def load_current_entrypoint() -> Any:
    spec = importlib.util.spec_from_file_location("v6_paper_exp4_e2e_baselines", CURRENT_ENTRYPOINT)
    if spec is None or spec.loader is None:
        raise ImportError(f"Unable to load current Exp.4 entrypoint at {CURRENT_ENTRYPOINT}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> None:
    print(
        "[compat] 03_e2e_baselines_combined.py is deprecated; "
        "delegating to 04_e2e_baselines_combined.py (current Exp.4).",
        file=sys.stderr,
    )
    module = load_current_entrypoint()
    module.main()


if __name__ == "__main__":
    main()

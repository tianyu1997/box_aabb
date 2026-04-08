#!/usr/bin/env python3
"""Generate paper-ready LaTeX tables from experiment JSON data.

Usage (from v5/):
    python python/scripts/gen_tables.py [--data-dir experiments/results]
"""

import argparse
import json
import logging
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from sbf5_bench.runner import ExperimentResults
from sbf5_bench.report import (
    envelope_volume_table,
    timing_table,
    e2e_table,
    baseline_table,
    freeze_table,
)

logger = logging.getLogger(__name__)


def load_json(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def write_tex(tex_str: str, path: str) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(tex_str)
    logger.info("Wrote %s", path)


def main():
    parser = argparse.ArgumentParser(description="Generate LaTeX tables")
    parser.add_argument("--data-dir", default="experiments/results",
                        help="Root directory containing s1-s5 experiment data")
    parser.add_argument("--output-dir", default=None,
                        help="Output directory for .tex files (default: paper/generated)")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")

    data_dir = args.data_dir
    out_dir = args.output_dir or os.path.join(
        os.path.dirname(__file__), "..", "..", "paper", "generated")

    # Table 1: Envelope Volume (S1)
    s1_path = os.path.join(data_dir, "s1_envelope_tightness", "results.json")
    if os.path.exists(s1_path):
        s1_data = load_json(s1_path)
        tex = envelope_volume_table(s1_data)
        write_tex(tex, os.path.join(out_dir, "tab1_envelope_volume.tex"))
    else:
        logger.warning("S1 data not found: %s", s1_path)

    # Table 2: Computation Timing (S2)
    s2_path = os.path.join(data_dir, "s2_envelope_timing", "results.json")
    if os.path.exists(s2_path):
        s2_data = load_json(s2_path)
        tex = timing_table(s2_data)
        write_tex(tex, os.path.join(out_dir, "tab2_computation_timing.tex"))
    else:
        logger.warning("S2 data not found: %s", s2_path)

    # Table 3: End-to-End Planning (S3)
    s3_path = os.path.join(data_dir, "s3_e2e", "results.json")
    if os.path.exists(s3_path):
        s3_results = ExperimentResults.load(s3_path)
        tex = e2e_table(s3_results)
        write_tex(tex, os.path.join(out_dir, "tab3_e2e_planning.tex"))
    else:
        logger.warning("S3 data not found: %s", s3_path)

    # Table 4: Baseline Comparison (S4)
    s4_path = os.path.join(data_dir, "s4_baselines", "results.json")
    if os.path.exists(s4_path):
        s4_results = ExperimentResults.load(s4_path)
        tex = baseline_table(s4_results)
        write_tex(tex, os.path.join(out_dir, "tab4_baselines.tex"))
    else:
        logger.warning("S4 data not found: %s", s4_path)

    # Table 5: Scalability (S5) — freeze_table expects different format,
    # so S5 scalability data is reported via figures only for now.
    s5_path = os.path.join(data_dir, "s5_scalability", "results.json")
    if os.path.exists(s5_path):
        logger.info("S5 scalability data found; see Fig 4 for visualization.")
    else:
        logger.warning("S5 data not found: %s", s5_path)

    logger.info("Done. Tables → %s", out_dir)


if __name__ == "__main__":
    main()

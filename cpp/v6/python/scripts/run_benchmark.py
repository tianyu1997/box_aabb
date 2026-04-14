#!/usr/bin/env python3
"""Run SBF v5 benchmark suite.

Usage (from v5/):
    $env:PYTHONPATH = "build_x64/Release;python"
    python python/scripts/run_benchmark.py [--scenes SCENE ...] [--trials N]
"""

import argparse
import logging
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from sbf5_bench.runner import run_experiment, ExperimentConfig
from sbf5_bench.sbf_adapter import SBFPlannerAdapter
from sbf5_bench.report import summary_table, latex_table, plot_comparison

logger = logging.getLogger(__name__)


def main():
    parser = argparse.ArgumentParser(description="SBF v5 Benchmark")
    parser.add_argument(
        "--scenes", nargs="+",
        default=["2dof_simple", "2dof_narrow", "2dof_cluttered",
                 "panda_tabletop"],
        help="Scene names to benchmark",
    )
    parser.add_argument("--trials", type=int, default=10,
                        help="Number of trials per scene")
    parser.add_argument("--timeout", type=float, default=60.0,
                        help="Per-query timeout in seconds")
    parser.add_argument("--output-dir", type=str,
                        default="results/benchmark_v5",
                        help="Output directory")
    parser.add_argument("--use-gcs", action="store_true",
                        help="Use GCS planner (requires Drake)")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
    )

    planners = [
        SBFPlannerAdapter(use_gcs=args.use_gcs),
    ]

    config = ExperimentConfig(
        scenes=args.scenes,
        planners=planners,
        n_trials=args.trials,
        timeout=args.timeout,
        output_dir=args.output_dir,
    )

    logger.info("Starting benchmark: %d scenes × %d trials",
                len(args.scenes), args.trials)
    results = run_experiment(config)

    # Save canonical results
    results_path = os.path.join(args.output_dir, "results.json")
    results.save(results_path)
    logger.info("Saved results → %s", results_path)

    # Print Markdown table
    print("\n" + summary_table(results))

    # Export LaTeX
    tex_path = os.path.join(args.output_dir, "table.tex")
    os.makedirs(args.output_dir, exist_ok=True)
    with open(tex_path, "w", encoding="utf-8") as f:
        f.write(latex_table(results))
    logger.info("Saved LaTeX → %s", tex_path)

    # Generate comparison chart
    try:
        fig = plot_comparison(results)
        html_path = os.path.join(args.output_dir, "comparison.html")
        fig.write_html(html_path)
        logger.info("Saved comparison chart → %s", html_path)

        # Static export if kaleido available
        try:
            fig.write_image(
                os.path.join(args.output_dir, "comparison.pdf"),
                width=800, height=400)
            fig.write_image(
                os.path.join(args.output_dir, "comparison.png"),
                width=1600, height=800, scale=2)
            logger.info("Saved static images (PDF + PNG)")
        except (ImportError, ValueError) as e:
            logger.warning("Static export skipped (install kaleido): %s", e)

    except ImportError:
        logger.warning("Plotly not available, skipping chart generation")

    logger.info("Benchmark complete.")


if __name__ == "__main__":
    main()

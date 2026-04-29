#!/usr/bin/env python3
"""Generate paper figures from the v6 paper result JSON files."""
from __future__ import annotations

import json
import math
from pathlib import Path
from statistics import median
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
RESULTS = ROOT / "experiments" / "results_paper"
PAPER = ROOT / "doc" / "paper"
GENERATED_DIRS = [PAPER / "en" / "generated", PAPER / "zh" / "generated"]

COLORS = {
    "blue": "#3B6EA8",
    "green": "#4C9F70",
    "orange": "#D9853B",
    "red": "#C44E52",
    "purple": "#8172B2",
    "gray": "#6E6E6E",
}


def load_json(name: str) -> dict[str, Any]:
    path = RESULTS / name
    if not path.is_file():
        raise FileNotFoundError(path)
    return json.loads(path.read_text())


def save_all(fig: plt.Figure, filename: str) -> None:
    for out_dir in GENERATED_DIRS:
        out_dir.mkdir(parents=True, exist_ok=True)
        fig.savefig(out_dir / filename, bbox_inches="tight")
    plt.close(fig)


def setup_axes(ax: plt.Axes, *, ylabel: str, xlabel: str | None = None) -> None:
    ax.set_ylabel(ylabel)
    if xlabel:
        ax.set_xlabel(xlabel)
    ax.grid(True, axis="y", alpha=0.25, linewidth=0.7)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


def exp1_pipeline() -> None:
    payload = load_json("epiaabb_pipeline.json")
    rows = payload.get("rows", [])
    width_bins = list(dict.fromkeys(row["width_bin"] for row in rows))
    sources = ["IFK", "CritSample", "Analytical", "MC"]
    source_colors = [COLORS["gray"], COLORS["blue"], COLORS["green"], COLORS["orange"]]

    def value(width_bin: str, source: str, key: str) -> float:
        for row in rows:
            if row["width_bin"] == width_bin and row["source"] == source:
                return float(row[key])
        return math.nan

    x = np.arange(len(width_bins))
    bar_width = 0.18
    fig, axes = plt.subplots(1, 2, figsize=(7.1, 2.55), constrained_layout=True)

    for idx, (source, color) in enumerate(zip(sources, source_colors)):
        offset = (idx - 1.5) * bar_width
        volumes = [value(width_bin, source, "volume_mean") for width_bin in width_bins]
        times = [value(width_bin, source, "time_us_mean") for width_bin in width_bins]
        axes[0].bar(x + offset, volumes, width=bar_width, label=source, color=color)
        axes[1].bar(x + offset, times, width=bar_width, label=source, color=color)

    axes[0].set_yscale("log")
    axes[1].set_yscale("log")
    for ax in axes:
        ax.set_xticks(x)
        ax.set_xticklabels(width_bins, rotation=18, ha="right")
    setup_axes(axes[0], ylabel="Mean volume")
    setup_axes(axes[1], ylabel="Mean time (us)")
    axes[0].legend(frameon=False, fontsize=7, ncol=2)
    fig.suptitle("Endpoint source envelope quality and cost", fontsize=10)
    save_all(fig, "fig_exp1_epiaabb_pipeline.pdf")


def exp2_link_envelope() -> None:
    payload = load_json("link_envelope_pipeline.json")
    rows = payload.get("rows", [])
    link_rows = sorted(
        [row for row in rows if row.get("type") == "LinkIAABB"],
        key=lambda row: int(row.get("n_subdivisions") or 0),
    )
    hull_rows = sorted(
        [row for row in rows if "Hull" in str(row.get("type", ""))],
        key=lambda row: float(row.get("voxel_delta") or 0.0),
        reverse=True,
    )
    entries = link_rows + hull_rows
    labels = []
    colors = []
    for row in entries:
        if row.get("type") == "LinkIAABB":
            labels.append(f"S={int(row.get('n_subdivisions') or 1)}")
            colors.append(COLORS["blue"])
        else:
            labels.append(f"d={float(row.get('voxel_delta') or 0.0):.2f}")
            colors.append(COLORS["green"])

    volume = [float(row["volume_mean"]) for row in entries]
    time_us = [float(row["time_us_mean"]) for row in entries]
    storage = [float(row.get("storage_bytes_optimized_mean") or 0.0) for row in entries]
    x = np.arange(len(entries))

    fig, axes = plt.subplots(1, 2, figsize=(7.1, 2.55), constrained_layout=True)
    axes[0].scatter(time_us, volume, s=46, color=colors, edgecolor="white", linewidth=0.5)
    for label, xv, yv in zip(labels, time_us, volume):
        axes[0].annotate(label, (xv, yv), textcoords="offset points", xytext=(4, 3), fontsize=6)
    axes[0].set_xscale("log")
    axes[0].set_yscale("log")

    axes[1].bar(x, storage, color=colors)
    axes[1].set_yscale("symlog", linthresh=1.0)
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(labels, rotation=18, ha="right")
    axes[1].axvline(len(link_rows) - 0.5, color="#9A9A9A", linewidth=0.8)
    setup_axes(axes[0], ylabel="Mean volume", xlabel="Mean time (us)")
    setup_axes(axes[1], ylabel="Optimized cache bytes")
    axes[0].scatter([], [], color=COLORS["blue"], label="LinkIAABB")
    axes[0].scatter([], [], color=COLORS["green"], label="Hull16-Grid")
    axes[0].legend(frameon=False, fontsize=7)
    fig.suptitle("Link-envelope tightness and cache payload", fontsize=10)
    save_all(fig, "fig_exp2_link_envelope_pipeline.pdf")


def exp3_cache_replay() -> None:
    payload = load_json("marcucci_envelope_build.json")
    comparisons = [row for row in payload.get("comparisons", []) if row.get("paper_compare", True)]
    if not comparisons:
        comparisons = payload.get("comparisons", [])
    labels = [f"{row.get('endpoint_label', row.get('endpoint_source'))}\n{row.get('envelope_label', row.get('envelope_key'))}" for row in comparisons]
    no_cache = [float(row.get("no_cache_median_build_s") or 0.0) for row in comparisons]
    cache_hit = [float(row.get("cache_hit_median_build_s") or 0.0) for row in comparisons]
    speedup = [float(row.get("total_build_speedup") or row.get("speedup") or 0.0) for row in comparisons]

    x = np.arange(len(labels))
    fig, ax = plt.subplots(figsize=(7.1, 2.65), constrained_layout=True)
    ax.bar(x - 0.18, no_cache, width=0.36, label="No LECT cache", color=COLORS["red"])
    ax.bar(x + 0.18, cache_hit, width=0.36, label="LECT cache hit", color=COLORS["green"])
    for idx, factor in enumerate(speedup):
        if factor > 0:
            ax.annotate(f"{factor:.1f}x", (idx + 0.18, cache_hit[idx]), textcoords="offset points", xytext=(0, 4), ha="center", fontsize=7)
    ax.set_yscale("log")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=7)
    setup_axes(ax, ylabel="Median full build time (s)")
    ax.legend(frameon=False, fontsize=7, ncol=2)
    fig.suptitle("Same-route cache replay reduces full build time", fontsize=10)
    save_all(fig, "fig_exp3_cache_replay.pdf")


def successful_query_values(seed_trials: list[dict[str, Any]], key: str) -> list[float]:
    values: list[float] = []
    for trial in seed_trials:
        for query in trial.get("queries", []):
            if query.get("success") and query.get(key) is not None:
                values.append(float(query[key]))
    return values


def exp4_baselines() -> None:
    sbf = load_json("marcucci_combined.json")
    methods: list[dict[str, Any]] = []
    sbf_query_times = [float(row["t_med_s"]) for row in sbf.get("queries", []) if row.get("t_med_s") is not None]
    sbf_lengths = [float(row["len_med"]) for row in sbf.get("queries", []) if row.get("len_med") is not None]
    methods.append({
        "name": "SBF",
        "build": float(sbf.get("build", {}).get("median_s") or 0.0),
        "query": median(sbf_query_times) if sbf_query_times else math.nan,
        "length": median(sbf_lengths) if sbf_lengths else math.nan,
        "sr": float(np.mean([row.get("sr", 0.0) for row in sbf.get("queries", [])]) * 100.0) if sbf.get("queries") else math.nan,
    })

    for filename, label in [
        ("marcucci_iris_np_gcs.json", "IRIS-NP"),
        ("marcucci_iris_zo_gcs.json", "IRIS-ZO"),
        ("marcucci_ompl_prm.json", "PRM"),
        ("marcucci_ompl_bitstar_budget.json", "BIT*"),
    ]:
        path = RESULTS / filename
        if not path.exists():
            continue
        payload = load_json(filename)
        summary = payload.get("summary", {})
        methods.append({
            "name": label,
            "build": float(summary.get("build_s_median") or 0.0),
            "query": float(summary.get("query_time_s_median") or math.nan),
            "length": float(summary.get("query_path_rad_mean") or math.nan),
            "sr": float(summary.get("sr") or 0.0),
        })

    labels = [method["name"] for method in methods]
    build = [method["build"] for method in methods]
    query = [method["query"] for method in methods]
    length = [method["length"] for method in methods]
    sr = [method["sr"] for method in methods]
    x = np.arange(len(labels))

    fig, axes = plt.subplots(1, 3, figsize=(7.2, 2.65), constrained_layout=True)
    axes[0].bar(x, build, color=[COLORS["green"], COLORS["blue"], COLORS["orange"], COLORS["purple"], COLORS["red"]][:len(x)])
    axes[0].set_yscale("symlog", linthresh=0.05)
    axes[1].bar(x, query, color=[COLORS["green"], COLORS["blue"], COLORS["orange"], COLORS["purple"], COLORS["red"]][:len(x)])
    axes[1].set_yscale("log")
    axes[2].bar(x - 0.18, length, width=0.36, color=COLORS["gray"], label="Path")
    axes[2].bar(x + 0.18, sr, width=0.36, color=COLORS["blue"], label="SR")
    for ax in axes:
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=20, ha="right", fontsize=7)
    setup_axes(axes[0], ylabel="Build median (s)")
    setup_axes(axes[1], ylabel="Query median (s)")
    setup_axes(axes[2], ylabel="Path rad / SR %")
    axes[2].legend(frameon=False, fontsize=7)
    fig.suptitle("Marcucci same-workload build, query, and path metrics", fontsize=10)
    save_all(fig, "fig_exp4_marcucci_baselines.pdf")


def main() -> None:
    plt.rcParams.update({
        "font.size": 8,
        "axes.titlesize": 9,
        "axes.labelsize": 8,
        "legend.fontsize": 7,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
    })
    exp1_pipeline()
    exp2_link_envelope()
    exp3_cache_replay()
    exp4_baselines()
    for out_dir in GENERATED_DIRS:
        print(f"[figures] wrote PDFs under {out_dir}")


if __name__ == "__main__":
    main()

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
    "indigo": "#1F3A5F",
    "teal": "#1D7874",
    "gold": "#C98C2B",
    "brick": "#B24A3A",
    "slate": "#566573",
    "sand": "#DCC7A1",
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
    ax.grid(True, axis="y", alpha=0.22, linewidth=0.7, color="#8A8A8A")
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


def method_color(name: str) -> str:
    return {
        "SBF": COLORS["indigo"],
        "SBF (C+AABB)": COLORS["indigo"],
        "SBF (IFK+AABB)": COLORS["slate"],
        "IFK": COLORS["slate"],
        "CritSample": COLORS["teal"],
        "Analytical": COLORS["gold"],
        "MC": COLORS["brick"],
        "Drake IRIS-NP+GCS": COLORS["teal"],
        "OMPL PRM": COLORS["sand"],
        "OMPL BIT*": COLORS["brick"],
    }.get(name, COLORS["slate"])


def exp1_pipeline() -> None:
    payload = load_json("epiaabb_pipeline.json")
    rows = payload.get("rows", [])
    width_bins = list(dict.fromkeys(row["width_bin"] for row in rows))
    sources = ["IFK", "CritSample", "Analytical", "MC"]

    def value(width_bin: str, source: str, key: str) -> float:
        for row in rows:
            if row["width_bin"] == width_bin and row["source"] == source:
                return float(row[key])
        return math.nan

    x = np.arange(len(width_bins))
    fig, axes = plt.subplots(1, 3, figsize=(7.1, 2.7), constrained_layout=True)

    ifk_vol = [value(wb, "IFK", "volume_mean") for wb in width_bins]
    axes[0].plot(x, ifk_vol, marker="o", linewidth=1.8, color=method_color("IFK"), label="IFK")
    axes[0].set_yscale("log")
    axes[0].annotate("lower is better", (0.03, 0.88), xycoords="axes fraction", fontsize=7, color=COLORS["slate"])

    for source in ["CritSample", "Analytical", "MC"]:
        ratio = [value(wb, source, "volume_mean") / value(wb, "IFK", "volume_mean") for wb in width_bins]
        axes[1].plot(x, ratio, marker="o", linewidth=1.8, label=source, color=method_color(source))
    axes[1].axhline(1.0, linestyle="--", linewidth=0.9, color="#666666")
    axes[1].set_yscale("log")
    axes[1].annotate("higher is better\n(coverage margin vs IFK)", (0.03, 0.80), xycoords="axes fraction", fontsize=7)

    bar_width = 0.18
    for idx, source in enumerate(sources):
        offset = (idx - 1.5) * bar_width
        times = [value(width_bin, source, "time_us_mean") for width_bin in width_bins]
        axes[2].bar(x + offset, times, width=bar_width, label=source, color=method_color(source))
    axes[2].set_yscale("log")

    for ax in axes:
        ax.set_xticks(x)
        ax.set_xticklabels(width_bins, rotation=18, ha="right")
    setup_axes(axes[0], ylabel="IFK volume mean", xlabel="Width bin")
    setup_axes(axes[1], ylabel="Volume ratio to IFK", xlabel="Width bin")
    setup_axes(axes[2], ylabel="Mean time (us)", xlabel="Width bin")
    axes[1].legend(frameon=False, fontsize=7, ncol=1, loc="upper left")
    axes[2].legend(frameon=False, fontsize=6, ncol=2, loc="upper left")
    fig.suptitle("Exp.1 endpoint-source tradeoff (direction-aware)", fontsize=10)
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
            colors.append(COLORS["indigo"])
        else:
            labels.append(f"d={float(row.get('voxel_delta') or 0.0):.2f}")
            colors.append(COLORS["teal"])

    volume = [float(row["volume_mean"]) for row in entries]
    time_us = [float(row["time_us_mean"]) for row in entries]
    storage = [float(row.get("storage_bytes_optimized_mean") or 0.0) for row in entries]
    x = np.arange(len(entries))

    fig, axes = plt.subplots(1, 2, figsize=(7.1, 2.7), constrained_layout=True)
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
    axes[0].scatter([], [], color=COLORS["indigo"], label="LinkIAABB")
    axes[0].scatter([], [], color=COLORS["teal"], label="Hull16-Grid")
    axes[0].legend(frameon=False, fontsize=7)
    fig.suptitle("Exp.2 link-envelope tightness vs cache payload", fontsize=10)
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
    fig, ax = plt.subplots(figsize=(7.1, 2.7), constrained_layout=True)
    ax.bar(x - 0.18, no_cache, width=0.36, label="No LECT cache", color=COLORS["brick"])
    ax.bar(x + 0.18, cache_hit, width=0.36, label="LECT cache hit", color=COLORS["teal"])
    for idx, factor in enumerate(speedup):
        if factor > 0:
            ax.annotate(f"{factor:.1f}x", (idx + 0.18, cache_hit[idx]), textcoords="offset points", xytext=(0, 4), ha="center", fontsize=7)
    ax.set_yscale("log")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=7)
    setup_axes(ax, ylabel="Median full build time (s)")
    ax.legend(frameon=False, fontsize=7, ncol=2)
    fig.suptitle("Exp.3 same-route cache replay reduces full build time", fontsize=10)
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
        "name": "SBF (C+AABB)",
        "build": float(sbf.get("build", {}).get("median_s") or 0.0),
        "query": median(sbf_query_times) if sbf_query_times else math.nan,
        "length": median(sbf_lengths) if sbf_lengths else math.nan,
        "sr": float(np.mean([row.get("sr", 0.0) for row in sbf.get("queries", [])]) * 100.0) if sbf.get("queries") else math.nan,
    })

    for filename, label in [
        ("marcucci_iris_np_gcs.json", "IRIS-NP"),
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

    colors = [method_color(name) for name in labels]
    fig, axes = plt.subplots(1, 3, figsize=(7.2, 2.7), constrained_layout=True)
    axes[0].bar(x, build, color=colors)
    axes[0].set_yscale("symlog", linthresh=0.05)
    axes[1].bar(x, query, color=colors)
    axes[1].set_yscale("log")
    axes[2].bar(x - 0.18, length, width=0.36, color=COLORS["slate"], label="Path")
    axes[2].bar(x + 0.18, sr, width=0.36, color=COLORS["teal"], label="SR")
    for ax in axes:
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=20, ha="right", fontsize=7)
    setup_axes(axes[0], ylabel="Build median (s)")
    setup_axes(axes[1], ylabel="Query median (s)")
    setup_axes(axes[2], ylabel="Path rad / SR %")
    axes[2].legend(frameon=False, fontsize=7)
    fig.suptitle("Exp.4 Marcucci combined workload baselines", fontsize=10)
    save_all(fig, "fig_exp4_marcucci_baselines.pdf")


def exp5_cross_robot() -> None:
    payload = load_json("exp5_random_robot_scenes.json")
    methods = ["SBF (C+AABB)", "SBF (IFK+AABB)", "Drake IRIS-NP+GCS", "OMPL PRM", "OMPL BIT*"]
    method_keys = ["sbf", "sbf_ifk", "iris_np_gcs", "ompl_prm", "ompl_bitstar"]
    groups = list((payload.get("aggregation") or {}).get("groups", []))
    if groups:
        difficulty_order = {"easy": 0, "medium": 1, "hard": 2}
        robot_order = {"ur5": 0, "panda": 1}
        groups = sorted(
            groups,
            key=lambda row: (
                robot_order.get(str(row.get("robot", "")).lower(), 99),
                difficulty_order.get(str(row.get("difficulty", "")).lower(), 99),
            ),
        )
        labels = [
            f"{str(group.get('robot')).upper()}-{str(group.get('difficulty')).capitalize()}"
            for group in groups
        ]
        query_matrix = []
        path_matrix = []
        for method in method_keys:
            query_matrix.append([
                float((group.get("methods", {}).get(method, {}).get("query_time_s") or {}).get("mean") or math.nan)
                for group in groups
            ])
            path_matrix.append([
                float((group.get("methods", {}).get(method, {}).get("path_length") or {}).get("mean") or math.nan)
                for group in groups
            ])

        if not any(math.isfinite(value) and value > 0.0 for row in query_matrix for value in row):
            return

        x = np.arange(len(labels))
        width = 0.14
        fig, axes = plt.subplots(1, 2, figsize=(7.4, 2.8), constrained_layout=True)
        n_methods = len(methods)
        center = 0.5 * float(n_methods - 1)
        for idx, method in enumerate(methods):
            offset = (float(idx) - center) * width
            axes[0].bar(x + offset, query_matrix[idx], width=width, label=method, color=method_color(method))
            axes[1].bar(x + offset, path_matrix[idx], width=width, label=method, color=method_color(method))
        axes[0].set_yscale("log")
        for ax in axes:
            ax.set_xticks(x)
            ax.set_xticklabels(labels, rotation=22, ha="right", fontsize=6)
        setup_axes(axes[0], ylabel="Query time (s)")
        setup_axes(axes[1], ylabel="Path length")
        axes[0].legend(frameon=False, fontsize=5, ncol=1)
    else:
        scenes = payload.get("scenes", [])
        robot_data: dict[str, dict[str, dict[str, float]]] = {}
        for scene in scenes:
            robot = str(scene.get("robot", "unknown")).upper()
            stats: dict[str, dict[str, float]] = {}
            for row in scene.get("baseline_results", []):
                method = str(row.get("method", ""))
                label = dict(zip(method_keys, methods)).get(method, method)
                stats[label] = {
                    "query": float(row.get("query_time_s_mean") or math.nan),
                    "path": float(row.get("path_length_mean") or math.nan),
                }
            robot_data[robot] = stats

        if not any(stats for stats in robot_data.values()):
            return

        robots = sorted(robot_data.keys())
        if not any(
            math.isfinite(robot_data[robot].get(method, {}).get("query", math.nan))
            and robot_data[robot].get(method, {}).get("query", math.nan) > 0.0
            for robot in robots
            for method in methods
        ):
            return

        x = np.arange(len(methods))
        width = 0.36
        fig, axes = plt.subplots(1, 2, figsize=(7.1, 2.7), constrained_layout=True)
        for ridx, robot in enumerate(robots[:2]):
            query = [robot_data[robot].get(m, {}).get("query", math.nan) for m in methods]
            path = [robot_data[robot].get(m, {}).get("path", math.nan) for m in methods]
            ax = axes[ridx]
            ax.bar(x - width / 2, query, width=width, color=COLORS["teal"], label="Query (s)")
            ax2 = ax.twinx()
            ax2.plot(x + width / 2, path, color=COLORS["brick"], marker="o", linewidth=1.8, label="Path")
            ax.set_yscale("log")
            ax.set_xticks(x)
            ax.set_xticklabels(methods, rotation=20, ha="right", fontsize=7)
            setup_axes(ax, ylabel="Query time (s)")
            ax2.set_ylabel("Path length")
            ax2.spines["top"].set_visible(False)
            ax.set_title(robot)
            h1, l1 = ax.get_legend_handles_labels()
            h2, l2 = ax2.get_legend_handles_labels()
            ax.legend(h1 + h2, l1 + l2, frameon=False, fontsize=6, loc="upper left")

    fig.suptitle("Exp.5 cross-robot difficulty profile", fontsize=10)
    save_all(fig, "fig_exp5_cross_robot_baselines.pdf")


def main() -> None:
    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["TeX Gyre Termes", "Times New Roman", "DejaVu Serif"],
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
    exp5_cross_robot()
    for out_dir in GENERATED_DIRS:
        print(f"[figures] wrote PDFs under {out_dir}")


if __name__ == "__main__":
    main()

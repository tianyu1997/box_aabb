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
from matplotlib.lines import Line2D


ROOT = Path(__file__).resolve().parents[2]
RESULTS = ROOT / "experiments" / "results_paper"
PAPER = ROOT / "doc" / "paper"
GENERATED_DIRS = [PAPER / "en" / "generated", PAPER / "zh" / "generated"]

COLORS = {
    "indigo": "#0072B2",
    "sky": "#56B4E9",
    "teal": "#009E73",
    "gold": "#E69F00",
    "brick": "#D55E00",
    "slate": "#6F4EAF",
    "sand": "#CC79A7",
    "dark": "#222222",
}

MARKERS = ["o", "s", "^", "D", "P", "X", "v", "<", ">"]


def load_json(name: str) -> dict[str, Any]:
    path = RESULTS / name
    if not path.is_file():
        raise FileNotFoundError(path)
    return json.loads(path.read_text())


def save_all(fig: plt.Figure, filename: str) -> None:
    for out_dir in GENERATED_DIRS:
        out_dir.mkdir(parents=True, exist_ok=True)
        fig.savefig(out_dir / filename, bbox_inches="tight", pad_inches=0.08)
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
        "SBF (Crit)": COLORS["indigo"],
        "SBF (IFK)": COLORS["sand"],
        "IFK": COLORS["sand"],
        "CritSample": COLORS["teal"],
        "Analytical": COLORS["gold"],
        "MC": COLORS["slate"],
        "IRIS": COLORS["teal"],
        "PRM": COLORS["gold"],
        "BIT*": COLORS["brick"],
    }.get(name, COLORS["slate"])


def marker_for(index: int) -> str:
    return MARKERS[index % len(MARKERS)]


def place_side_legends(
    fig: plt.Figure,
    ax: plt.Axes,
    *,
    method_handles: list[Line2D],
    marker_handles: list[Line2D],
    marker_title: str,
    right_margin: float = 0.53,
    legend_fontsize: float = 6,
    title_fontsize: float = 6,
    anchor_x: float = 1.00,
) -> None:
    """Keep dense scatter legends outside the data region."""
    fig.subplots_adjust(right=right_margin)
    leg1 = ax.legend(
        handles=method_handles,
        frameon=False,
        fontsize=legend_fontsize,
        loc="upper left",
        bbox_to_anchor=(anchor_x, 1.00),
        borderaxespad=0.0,
        title="Method",
        title_fontsize=title_fontsize,
    )
    ax.add_artist(leg1)
    ax.legend(
        handles=marker_handles,
        frameon=False,
        fontsize=legend_fontsize,
        loc="upper left",
        bbox_to_anchor=(anchor_x, 0.53),
        borderaxespad=0.0,
        title=marker_title,
        title_fontsize=title_fontsize,
    )


def finite_positive(value: Any) -> bool:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return False
    return math.isfinite(number) and number > 0.0


def set_log_xlim_with_padding(ax: plt.Axes, x_values: list[float], *, pad_decades: float = 0.10) -> None:
    """Expand log-x limits on both sides so edge markers are not clipped."""
    positive = [float(v) for v in x_values if finite_positive(v)]
    if not positive:
        return
    if len(positive) == 1:
        center = math.log10(positive[0])
        ax.set_xlim(10 ** (center - 0.5), 10 ** (center + 0.5))
        return
    lo = min(positive)
    hi = max(positive)
    lo_log = math.log10(lo)
    hi_log = math.log10(hi)
    if hi_log - lo_log < 1e-9:
        lo_log -= 0.25
        hi_log += 0.25
    ax.set_xlim(10 ** (lo_log - pad_decades), 10 ** (hi_log + pad_decades))


def median_or_nan(values: list[float]) -> float:
    return float(median(values)) if values else math.nan


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
    axes[0].scatter([], [], color=COLORS["teal"], label=r"HullGrid$_{0.02,0.04,0.06,0.08}$")
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


def live_query_points(payload: dict[str, Any], *, fixed_budget_s: float | None = None) -> dict[str, tuple[float, float]]:
    buckets: dict[str, dict[str, list[float]]] = {}
    for query in payload.get("queries", []):
        name = str(query.get("name") or query.get("query") or "")
        if name:
            buckets.setdefault(name, {"time": [], "path": []})
    for trial in payload.get("seed_trials", []):
        for query in trial.get("queries", []):
            if not query.get("success"):
                continue
            name = str(query.get("query") or query.get("name") or "")
            if not name:
                continue
            bucket = buckets.setdefault(name, {"time": [], "path": []})
            if fixed_budget_s is not None:
                bucket["time"].append(float(fixed_budget_s))
            elif query.get("time_s") is not None:
                bucket["time"].append(float(query["time_s"]))
            if query.get("path_length") is not None:
                bucket["path"].append(float(query["path_length"]))
    return {
        name: (median_or_nan(values["time"]), median_or_nan(values["path"]))
        for name, values in buckets.items()
        if values["time"] and values["path"]
    }


def envelope_query_points(
    payload: dict[str, Any],
    *,
    endpoint_source: str,
    envelope_key: str,
    cache_mode: str,
) -> dict[str, tuple[float, float]]:
    buckets: dict[str, dict[str, list[float]]] = {}
    rows = [
        row for row in payload.get("rows", [])
        if str(row.get("endpoint_source")) == endpoint_source
        and str(row.get("envelope_key")) == envelope_key
        and str(row.get("cache_mode")) == cache_mode
    ]
    for row in rows:
        for query in row.get("queries", []):
            if not query.get("ok"):
                continue
            name = str(query.get("name") or "")
            bucket = buckets.setdefault(name, {"time": [], "path": []})
            if query.get("planning_time_ms") is not None:
                bucket["time"].append(float(query["planning_time_ms"]) / 1000.0)
            if query.get("path_length") is not None:
                bucket["path"].append(float(query["path_length"]))
    return {
        name: (median_or_nan(values["time"]), median_or_nan(values["path"]))
        for name, values in buckets.items()
        if values["time"] and values["path"]
    }


def exp4_baselines() -> None:
    sbf = load_json("marcucci_combined.json")
    query_names = [str(row.get("name")) for row in sbf.get("queries", []) if row.get("name")]
    query_markers = {name: marker_for(index) for index, name in enumerate(query_names)}
    method_points: list[tuple[str, str, float, float]] = []

    for query in sbf.get("queries", []):
        if query.get("t_med_s") is not None and query.get("len_med") is not None:
            method_points.append((
                "SBF (Crit)",
                str(query["name"]),
                float(query["t_med_s"]),
                float(query["len_med"]),
            ))

    envelope_path = RESULTS / "marcucci_envelope_build.json"
    if envelope_path.exists():
        ifk_points = envelope_query_points(
            load_json("marcucci_envelope_build.json"),
            endpoint_source="ifk",
            envelope_key="aabb_s4",
            cache_mode="cache_hit",
        )
        for query_name, (query_s, path_len) in ifk_points.items():
            method_points.append(("SBF (IFK)", query_name, query_s, path_len))

    for filename, label, fixed_budget_s in [
        ("marcucci_iris_np_gcs.json", "IRIS", None),
        ("marcucci_ompl_prm.json", "PRM", None),
        ("marcucci_ompl_bitstar_budget.json", "BIT*", 10.0),
    ]:
        path = RESULTS / filename
        if not path.exists():
            continue
        payload = load_json(filename)
        for query_name, (query_s, path_len) in live_query_points(payload, fixed_budget_s=fixed_budget_s).items():
            method_points.append((label, query_name, query_s, path_len))

    fig, ax = plt.subplots(figsize=(8.2, 3.3))
    x_values: list[float] = []
    for method, query_name, query_s, path_len in method_points:
        if not (finite_positive(query_s) and finite_positive(path_len)):
            continue
        x_values.append(float(query_s))
        ax.scatter(
            query_s,
            path_len,
            marker=query_markers.get(query_name, "o"),
            s=42,
            color=method_color(method),
            edgecolor=COLORS["dark"],
            linewidth=0.45,
            alpha=0.96,
        )
    ax.set_xscale("log")
    set_log_xlim_with_padding(ax, x_values, pad_decades=0.12)
    setup_axes(ax, xlabel="Successful query time (s)", ylabel="Path length (rad)")
    method_handles = [
        Line2D([0], [0], marker="o", linestyle="", color=method_color(method), label=method, markersize=5)
        for method in ["SBF (Crit)", "SBF (IFK)", "IRIS", "PRM", "BIT*"]
    ]
    query_handles = [
        Line2D([0], [0], marker=query_markers[name], linestyle="", color="#444444", label=name.replace("->", "→"), markersize=5)
        for name in query_names
    ]
    place_side_legends(
        fig,
        ax,
        method_handles=method_handles,
        marker_handles=query_handles,
        marker_title="Query",
        right_margin=0.42,
        legend_fontsize=5.2,
        title_fontsize=5.2,
        anchor_x=1.00,
    )
    save_all(fig, "fig_exp4_marcucci_baselines.pdf")


def exp5_cross_robot() -> None:
    payload = load_json("exp5_random_robot_scenes.json")
    methods = ["SBF (Crit)", "SBF (IFK)", "IRIS", "PRM", "BIT*"]
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
        group_labels = [
            f"{str(group.get('robot')).upper()}-{str(group.get('difficulty')).capitalize()}"
            for group in groups
        ]
        group_markers = {label: marker_for(index) for index, label in enumerate(group_labels)}
        fig, ax = plt.subplots(figsize=(7.2, 3.3))
        has_points = False
        for group, group_label in zip(groups, group_labels):
            summaries = group.get("methods", {})
            for method_key, method in zip(method_keys, methods):
                summary = summaries.get(method_key, {})
                path_len = (summary.get("path_length") or {}).get("mean")
                if method_key == "ompl_bitstar":
                    query_s = 10.0
                else:
                    query_s = (summary.get("query_time_s") or {}).get("mean")
                if not (finite_positive(query_s) and finite_positive(path_len)):
                    continue
                has_points = True
                ax.scatter(
                    float(query_s),
                    float(path_len),
                    marker=group_markers[group_label],
                    s=46,
                    color=method_color(method),
                    edgecolor=COLORS["dark"],
                    linewidth=0.45,
                    alpha=0.96,
                )
        if not has_points:
            return
        ax.set_xscale("log")
        setup_axes(ax, xlabel="Successful query time (s)", ylabel="Path length (rad)")
        method_handles = [
            Line2D([0], [0], marker="o", linestyle="", color=method_color(method), label=method, markersize=5)
            for method in methods
        ]
        group_handles = [
            Line2D([0], [0], marker=group_markers[label], linestyle="", color="#444444", label=label, markersize=5)
            for label in group_labels
        ]
        place_side_legends(
            fig,
            ax,
            method_handles=method_handles,
            marker_handles=group_handles,
            marker_title="Group",
        )
    else:
        scenes = payload.get("scenes", [])
        scene_points: list[tuple[str, str, float, float]] = []
        for scene in scenes:
            group_label = str(scene.get("robot", "unknown")).upper()
            for row in scene.get("baseline_results", []):
                method = str(row.get("method", ""))
                label = dict(zip(method_keys, methods)).get(method, method)
                query_s = 10.0 if method == "ompl_bitstar" else row.get("query_time_s_mean")
                path_len = row.get("path_length_mean")
                if finite_positive(query_s) and finite_positive(path_len):
                    scene_points.append((label, group_label, float(query_s), float(path_len)))
        if not scene_points:
            return
        group_names = sorted({point[1] for point in scene_points})
        group_markers = {label: marker_for(index) for index, label in enumerate(group_names)}
        fig, ax = plt.subplots(figsize=(7.2, 3.3))
        for method, group_label, query_s, path_len in scene_points:
            ax.scatter(query_s, path_len, marker=group_markers[group_label], s=46,
                       color=method_color(method), edgecolor=COLORS["dark"], linewidth=0.45, alpha=0.96)
        ax.set_xscale("log")
        setup_axes(ax, xlabel="Successful query time (s)", ylabel="Path length (rad)")
        method_handles = [
            Line2D([0], [0], marker="o", linestyle="", color=method_color(method), label=method, markersize=5)
            for method in methods
        ]
        group_handles = [
            Line2D([0], [0], marker=group_markers[label], linestyle="", color="#444444", label=label, markersize=5)
            for label in group_names
        ]
        place_side_legends(
            fig,
            ax,
            method_handles=method_handles,
            marker_handles=group_handles,
            marker_title="Group",
        )
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

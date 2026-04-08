"""
viz_ep_iaabb_bench.py — Endpoint iAABB Stage 1 方法耗时 & 体积可视化

读取 exp_ep_iaabb_bench 产生的输出目录，生成:
  1. timing_boxplot.html      — 各方法 Stage 1 耗时箱线图（按宽度类别分列）
  2. volume_boxplot.html      — 各方法体积箱线图（按宽度类别分列）
  3. timing_vs_volume.html    — 耗时 vs. 体积散点图（方法着色，形状区分宽度）
  4. summary_bar.html         — 均值汇总条形图（耗时 & 体积双轴）
  5. ep_iaabb_3d.html         — 代表性配置的 endpoint iAABB 4 方法 3D Plotly 包络对比视图

用法:
    python scripts/viz_ep_iaabb_bench.py <results_dir>

依赖:
    pip install plotly pandas numpy
    pip install -e v4/python/   (或将 v4/python/ 加入 PYTHONPATH)
"""

import argparse
import os
import sys

import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# ── 确保能 import sbf4_viz ─────────────────────────────────────────────────
_script_dir = os.path.dirname(os.path.abspath(__file__))
_v4_python  = os.path.normpath(os.path.join(_script_dir, "..", "python"))
if _v4_python not in sys.path:
    sys.path.insert(0, _v4_python)

try:
    from sbf4_viz.envelope_comparison_viz import load_envelope_comparison, plot_envelope_comparison
    from sbf4_viz.ep_iaabb_viz import load_ep_iaabb_comparison, plot_ep_iaabb_comparison
    _HAS_SBF4_VIZ = True
except ImportError:
    _HAS_SBF4_VIZ = False


# ── 方法元信息 ─────────────────────────────────────────────────────────────

METHOD_META = {
    "IFK":        {"label": "IFK",        "color": "#e83030"},
    "CritSample": {"label": "CritSample", "color": "#4a90d9"},
    "Analytical": {"label": "Analytical", "color": "#f5a623"},
    "GCPC":       {"label": "GCPC",       "color": "#7ed321"},
}

# 注：Analytical 与 GCPC 均使用 derive_crit_endpoints+cache，结果应高度一致；
#     IFK 最快、体积最大；CritSample 无需缓存、中等质量；
#     Analytical/GCPC 需缓存、端点 iAABBs 最紧密。

WIDTH_BANDS = {
    "narrow": (0.0,  0.19),
    "medium": (0.19, 0.45),
    "wide":   (0.45, 1.01),
}

BAND_COLORS = {
    "narrow": "#3498db",
    "medium": "#2ecc71",
    "wide":   "#e74c3c",
}


def _band(frac: float) -> str:
    for name, (lo, hi) in WIDTH_BANDS.items():
        if lo <= frac < hi:
            return name
    return "wide"


def _save(fig: go.Figure, path: str, title: str) -> None:
    fig.write_html(path)
    print(f"  [{title:30s}]  {os.path.abspath(path)}")


# ─── 1. Stage 1 耗时箱线图 ────────────────────────────────────────────────

def plot_timing_boxplot(df: pd.DataFrame, out: str) -> None:
    """按宽度类别分列，展示各方法 Stage 1 耗时分布（中位数×5次重复）。"""
    methods = list(METHOD_META.keys())
    bands   = ["narrow", "medium", "wide"]

    fig = make_subplots(
        rows=1, cols=3,
        subplot_titles=[f"Width: {b}" for b in bands],
        shared_yaxes=True,
    )

    for ci, band in enumerate(bands, 1):
        sub = df[df["band"] == band]
        for mname in methods:
            md   = METHOD_META[mname]
            vals = sub[sub["source"] == mname]["time_ms"]
            fig.add_trace(
                go.Box(
                    y=vals, name=md["label"],
                    marker_color=md["color"],
                    showlegend=(ci == 1),
                    legendgroup=mname,
                    boxmean="sd",
                ),
                row=1, col=ci,
            )

    fig.update_layout(
        title="Stage 1 Endpoint iAABB 耗时对比 (median over 5 repeats)",
        yaxis_title="time_ms  [ms]",
        legend=dict(x=1.01, y=0.99),
        height=500,
        boxmode="group",
    )
    _save(fig, out, "Timing Boxplot")


# ─── 2. 体积箱线图 ────────────────────────────────────────────────────────

def plot_volume_boxplot(df: pd.DataFrame, out: str) -> None:
    """按宽度类别分列，展示各方法 extract_link_iaabbs 后的体积分布。"""
    methods = list(METHOD_META.keys())
    bands   = ["narrow", "medium", "wide"]

    fig = make_subplots(
        rows=1, cols=3,
        subplot_titles=[f"Width: {b}" for b in bands],
        shared_yaxes=True,
    )

    for ci, band in enumerate(bands, 1):
        sub = df[df["band"] == band]
        for mname in methods:
            md   = METHOD_META[mname]
            vals = sub[sub["source"] == mname]["volume"]
            fig.add_trace(
                go.Box(
                    y=vals, name=md["label"],
                    marker_color=md["color"],
                    showlegend=(ci == 1),
                    legendgroup=mname,
                    boxmean="sd",
                ),
                row=1, col=ci,
            )

    fig.update_layout(
        title="per-link body AABB 体积对比 (m³) — extract_link_iaabbs 输出",
        yaxis_title="volume  [m³]",
        legend=dict(x=1.01, y=0.99),
        height=500,
        boxmode="group",
    )
    _save(fig, out, "Volume Boxplot")


# ─── 3. 耗时 vs 体积散点图 ────────────────────────────────────────────────

def plot_timing_vs_volume(df: pd.DataFrame, out: str) -> None:
    """每 trial 的 time_ms vs volume，方法着色，形状区分宽度类别。"""
    fig = go.Figure()

    symbols_map = {"narrow": "circle", "medium": "diamond", "wide": "square"}

    for mname, md in METHOD_META.items():
        sub = df[df["source"] == mname]
        for band, sym in symbols_map.items():
            bs = sub[sub["band"] == band]
            if len(bs) == 0:
                continue
            fig.add_trace(go.Scatter(
                x=bs["time_ms"],
                y=bs["volume"],
                mode="markers",
                name=f"{md['label']} / {band}",
                legendgroup=mname,
                marker=dict(
                    color=md["color"],
                    symbol=sym,
                    size=7,
                    opacity=0.70,
                    line=dict(width=0.5, color="white"),
                ),
            ))

    fig.update_layout(
        title="耗时 vs 体积  (●=narrow  ◆=medium  ■=wide)",
        xaxis_title="Stage 1 time_ms  [ms]",
        yaxis_title="volume  [m³]",
        legend=dict(x=1.01, y=0.99),
        height=550,
    )
    _save(fig, out, "Timing vs Volume")


# ─── 4. 均值汇总条形图 ────────────────────────────────────────────────────

def plot_summary_bar(df: pd.DataFrame, out: str) -> None:
    """
    各方法全部 trial 均值：
      左 Y 轴 — 耗时 (ms)
      右 Y 轴 — 体积 (m³)
    按宽度类别（narrow / medium / wide）分组，加总均值。
    """
    methods = list(METHOD_META.keys())
    labels  = [METHOD_META[m]["label"] for m in methods]
    bands   = ["narrow", "medium", "wide"]

    fig = make_subplots(
        rows=1, cols=3,
        subplot_titles=[f"Width: {b}" for b in bands],
        specs=[[{"secondary_y": True}] * 3],
        shared_yaxes=False,
    )

    for ci, band in enumerate(bands, 1):
        sub = df[df["band"] == band]
        time_means   = [sub[sub["source"] == m]["time_ms"].mean()  for m in methods]
        volume_means = [sub[sub["source"] == m]["volume"].mean()   for m in methods]

        fig.add_trace(
            go.Bar(
                name="耗时 (ms)",
                x=labels,
                y=time_means,
                marker_color=[METHOD_META[m]["color"] for m in methods],
                opacity=0.85,
                showlegend=(ci == 1),
                legendgroup="time",
            ),
            row=1, col=ci, secondary_y=False,
        )
        fig.add_trace(
            go.Scatter(
                name="体积 (m³)",
                x=labels,
                y=volume_means,
                mode="markers+lines",
                marker=dict(
                    symbol="diamond",
                    size=10,
                    color=[METHOD_META[m]["color"] for m in methods],
                    line=dict(width=1, color="black"),
                ),
                line=dict(color="gray", dash="dot"),
                showlegend=(ci == 1),
                legendgroup="vol",
            ),
            row=1, col=ci, secondary_y=True,
        )

    fig.update_yaxes(title_text="time_ms (均值)", secondary_y=False, row=1, col=1)
    fig.update_yaxes(title_text="volume m³ (均值)", secondary_y=True,  row=1, col=1)
    fig.update_layout(
        title="Stage 1 端点 iAABB 方法汇总：耗时（柱）& 体积（菱形线）均值",
        height=500,
        barmode="group",
        legend=dict(x=1.01, y=0.99),
    )
    _save(fig, out, "Summary Bar")


# ─── 5. 3D 对比视图 ───────────────────────────────────────────────────────

def plot_ep_iaabb_3d(json_path: str, out: str) -> None:
    """使用 sbf4_viz.ep_iaabb_viz 渲染 endpoint iAABB 包络 3D 对比 HTML。"""
    if not os.path.exists(json_path):
        print(f"  [SKIP] ep_iaabb_comparison.json not found: {json_path}")
        return
    if not _HAS_SBF4_VIZ:
        print("  [SKIP] sbf4_viz not importable — skipping endpoint iAABB 3D viz.")
        return
    data = load_ep_iaabb_comparison(json_path)
    plot_ep_iaabb_comparison(
        data,
        title="Endpoint iAABB 包络对比（IFK / CritSample / Analytical / GCPC）",
        show=False,
        save_html=out,
    )
    print(f"  [{'Endpoint iAABB 3D':30s}]  {os.path.abspath(out)}")


# ─── 汇总统计表 ───────────────────────────────────────────────────────────

def print_summary(df: pd.DataFrame) -> None:
    print("\n  ─── 汇总统计（全部 trial 均值）─────────────────────────────────────")
    print(f"  {'方法':14s}  {'time_ms':>9s}  {'volume (m³)':>14s}  {'n_eval':>7s}")
    print("  " + "─" * 52)
    for mname, md in METHOD_META.items():
        sub = df[df["source"] == mname]
        if sub.empty:
            continue
        print(f"  {md['label']:14s}  "
              f"{sub['time_ms'].mean():9.4f}  "
              f"{sub['volume'].mean():14.6e}  "
              f"{sub['n_eval'].mean():7.1f}")
    print()
    # Per-width breakdown
    print("  ─── 按宽度类别细分 ──────────────────────────────────────────────────")
    print(f"  {'方法':14s}  {'宽度':8s}  {'time_ms':>9s}  {'volume (m³)':>14s}")
    print("  " + "─" * 52)
    for mname, md in METHOD_META.items():
        for band in ["narrow", "medium", "wide"]:
            sub = df[(df["source"] == mname) & (df["band"] == band)]
            if sub.empty:
                continue
            print(f"  {md['label']:14s}  {band:8s}  "
                  f"{sub['time_ms'].mean():9.4f}  "
                  f"{sub['volume'].mean():14.6e}")
    print()


# ─── Entry point ──────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Visualise exp_ep_iaabb_bench results")
    parser.add_argument("results_dir",
                        help="Directory containing results.csv (and optionally comparison.json)")
    args = parser.parse_args()

    rd = args.results_dir
    csv_path  = os.path.join(rd, "results.csv")
    json_path = os.path.join(rd, "ep_iaabb_comparison.json")

    if not os.path.exists(csv_path):
        print(f"ERROR: {csv_path} not found", file=sys.stderr)
        sys.exit(1)

    df = pd.read_csv(csv_path)
    df["band"] = df["width_frac"].apply(_band)

    print(f"\n  Loaded {len(df)} rows from {csv_path}")
    present = sorted(df["source"].unique())
    print(f"  Trials: {df['trial'].nunique()}  Sources: {present}")

    print("\n  Generating visualizations...")

    plot_timing_boxplot  (df, os.path.join(rd, "timing_boxplot.html"))
    plot_volume_boxplot  (df, os.path.join(rd, "volume_boxplot.html"))
    plot_timing_vs_volume(df, os.path.join(rd, "timing_vs_volume.html"))
    plot_summary_bar     (df, os.path.join(rd, "summary_bar.html"))
    plot_ep_iaabb_3d     (json_path, os.path.join(rd, "ep_iaabb_3d.html"))

    print_summary(df)


if __name__ == "__main__":
    main()

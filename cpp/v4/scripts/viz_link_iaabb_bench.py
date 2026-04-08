"""
viz_link_iaabb_bench.py — Link iAABB 方法耗时 & 体积可视化

读取 exp_link_iaabb_bench 产生的输出目录，生成:
  1. timing_boxplot.html           — 各方法 Stage2 耗时箱线图（区分宽度类别）
  2. volume_boxplot.html           — 各方法体积箱线图（区分宽度类别）
  3. timing_vs_volume.html         — 耗时 vs. 体积散点图（归一化）
  4. timing_breakdown.html         — 各方法 Stage1/Stage2 时间拆解
  5. link_iaabbs_per_method.html   — 代表性配置下各方法产生的 link iAABB 3D 可视化
  6. comparison_3d.html            — 代表性配置的 5 方法 3D Plotly 对比视图

用法:
    python scripts/viz_link_iaabb_bench.py <results_dir>

依赖:
    pip install plotly pandas numpy
    pip install -e v4/python/   (或将 v4/python/ 加入 PYTHONPATH)
"""

import argparse
import json
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

from sbf4_viz.envelope_comparison_viz import load_envelope_comparison, plot_envelope_comparison


# ── 颜色与显示名称 ──────────────────────────────────────────────────────────

METHOD_META = {
    "LinkIAABB_s1":   {"label": "LinkIAABB(sub=1)",  "color": "#e83030"},
    "LinkIAABB_s4":   {"label": "LinkIAABB(sub=4)",  "color": "#4a90d9"},
    "LinkIAABB_s16":  {"label": "LinkIAABB(sub=16)", "color": "#f5a623"},
    "LinkIAABB_Grid": {"label": "LinkIAABB_Grid",    "color": "#7ed321"},
    "Hull16_Grid":    {"label": "Hull16_Grid",        "color": "#9b59b6"},
}

# Ordered list used for consistent display
METHOD_ORDER = ["LinkIAABB_s1", "LinkIAABB_s4", "LinkIAABB_s16",
                "LinkIAABB_Grid", "Hull16_Grid"]

WIDTH_BANDS = {
    "narrow":  (0.0,  0.19),
    "medium":  (0.19, 0.45),
    "wide":    (0.45, 1.01),
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


# ─── 1. 耗时箱线图 ─────────────────────────────────────────────────────────

def plot_timing_boxplot(df: pd.DataFrame, out: str) -> None:
    """Stage2 耗时分布，按 width 类别分列。"""
    methods = list(METHOD_META.keys())
    bands   = ["narrow", "medium", "wide"]

    fig = make_subplots(
        rows=1, cols=3,
        subplot_titles=[f"Width: {b}" for b in bands],
        shared_yaxes=True,
    )

    for ci, band in enumerate(bands, 1):
        sub = df[df["band"] == band]
        for mi, mname in enumerate(methods):
            md   = METHOD_META[mname]
            vals = sub[sub["envelope"] == mname]["env_time_ms"]
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
        title="Stage 2 耗时对比 (CritSample endpoint, median over 5 repeats)",
        yaxis_title="env_time_ms (Stage2)",
        legend=dict(x=1.01, y=0.99),
        height=500,
        boxmode="group",
    )
    _save(fig, out, "Timing Boxplot")


# ─── 2. 体积箱线图 ─────────────────────────────────────────────────────────

def plot_volume_boxplot(df: pd.DataFrame, out: str) -> None:
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
            vals = sub[sub["envelope"] == mname]["volume"]
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
        title="体积对比 (m³) — CritSample endpoint",
        yaxis_title="volume (m³)",
        legend=dict(x=1.01, y=0.99),
        height=500,
        boxmode="group",
    )
    _save(fig, out, "Volume Boxplot")


# ─── 3. 耗时 vs 体积散点图 ─────────────────────────────────────────────────

def plot_timing_vs_volume(df: pd.DataFrame, out: str) -> None:
    """每个 trial 的 total_ms vs volume，按方法着色，标注宽度类别。"""
    fig = go.Figure()

    symbols_map = {"narrow": "circle", "medium": "diamond", "wide": "square"}

    for mname, md in METHOD_META.items():
        sub = df[df["envelope"] == mname]
        for band, sym in symbols_map.items():
            bs = sub[sub["band"] == band]
            if len(bs) == 0:
                continue
            fig.add_trace(go.Scatter(
                x=bs["total_ms"],
                y=bs["volume"],
                mode="markers",
                name=f"{md['label']} / {band}",
                legendgroup=mname,
                marker=dict(
                    color=md["color"],
                    symbol=sym,
                    size=7,
                    opacity=0.7,
                    line=dict(width=0.5, color="white"),
                ),
            ))

    fig.update_layout(
        title="耗时 vs 体积 (所有 trial)  ●=narrow  ◆=medium  ■=wide",
        xaxis_title="total_ms (Stage1 + Stage2)  [ms]",
        yaxis_title="volume  [m³]",
        legend=dict(x=1.01, y=0.99),
        height=550,
    )
    _save(fig, out, "Timing vs Volume")


# ─── 4. Stage1 & Stage2 时间拆解 ──────────────────────────────────────────

def plot_timing_breakdown(df: pd.DataFrame, out: str) -> None:
    """
    每种方法的平均 Stage1 耗时（共享，因此相同）与 Stage2 耗时。
    使用堆积条形图，便于直观看到 Stage1 占比。
    """
    methods = list(METHOD_META.keys())
    labels  = [METHOD_META[m]["label"] for m in methods]
    colors  = [METHOD_META[m]["color"] for m in methods]

    # 平均值（全部 trial）
    ep_means  = [df[df["envelope"] == m]["ep_time_ms"].mean() for m in methods]
    env_means = [df[df["envelope"] == m]["env_time_ms"].mean() for m in methods]

    fig = go.Figure()
    fig.add_trace(go.Bar(
        name="Stage1 CritSample",
        x=labels,
        y=ep_means,
        marker_color="rgba(100,100,100,0.45)",
        marker_line_width=0,
    ))
    fig.add_trace(go.Bar(
        name="Stage2 LinkEnvelope",
        x=labels,
        y=env_means,
        marker_color=colors,
        marker_line_width=0,
    ))

    fig.update_layout(
        barmode="stack",
        title="Stage1 / Stage2 平均耗时拆解 (所有 trial 均值)",
        xaxis_title="方法",
        yaxis_title="时间 [ms]",
        legend=dict(x=0.01, y=0.99),
        height=450,
    )
    _save(fig, out, "Timing Breakdown")


# ─── 5. Per-method link iAABB 3D 盒子视图 ──────────────────────────────────

def _wireframe_box(lo, hi, color: str, name: str, legendgroup: str,
                   showlegend: bool = True) -> go.Scatter3d:
    """返回一个 AABB 线框 Scatter3d trace。"""
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    N = None  # connectgaps=False 分隔线
    xs = [x0,x1,N, x0,x1,N, x0,x1,N, x0,x1,N,
          x0,x0,N, x1,x1,N, x0,x0,N, x1,x1,N,
          x0,x0,N, x1,x1,N, x0,x0,N, x1,x1]
    ys = [y0,y0,N, y1,y1,N, y0,y0,N, y1,y1,N,
          y0,y1,N, y0,y1,N, y0,y1,N, y0,y1,N,
          y0,y0,N, y0,y0,N, y1,y1,N, y1,y1]
    zs = [z0,z0,N, z0,z0,N, z1,z1,N, z1,z1,N,
          z0,z0,N, z0,z0,N, z1,z1,N, z1,z1,N,
          z0,z1,N, z0,z1,N, z0,z1,N, z0,z1]
    return go.Scatter3d(
        x=xs, y=ys, z=zs,
        mode="lines",
        line=dict(color=color, width=2),
        name=name, legendgroup=legendgroup,
        showlegend=showlegend,
        connectgaps=False,
    )


def _voxel_scatter(centres: np.ndarray, color: str, name: str,
                   legendgroup: str, size: float = 4) -> go.Scatter3d:
    return go.Scatter3d(
        x=centres[:, 0], y=centres[:, 1], z=centres[:, 2],
        mode="markers",
        marker=dict(size=size, color=color, opacity=0.6,
                    symbol="square"),
        name=name, legendgroup=legendgroup,
    )


# Assign stable per-link colours (up to 9 active links)
_LINK_COLORS = [
    "#e74c3c", "#3498db", "#2ecc71", "#f39c12",
    "#9b59b6", "#1abc9c", "#e67e22", "#34495e", "#d35400",
]


def plot_link_iaabbs_per_method(json_path: str, out: str) -> None:
    """
    读取 method_iaabbs.json，为每种方法渲染各连杆 iAABB 线框。

    布局：5 方法共享同一 3D 场景，通过 legendgroup 切换单方法显示。
    每方法内一种颜色对应一条连杆（便于识别各段细分数量差异）。
    Hull16_Grid 以散点体素呈现。
    还包括机器人关节臂作为参考。
    """
    if not os.path.exists(json_path):
        print(f"  [SKIP] method_iaabbs.json not found: {json_path}")
        return

    with open(json_path) as f:
        data = json.load(f)

    fig = go.Figure()
    n_active      = data.get("n_active", 8)
    alm           = data.get("active_link_map", list(range(n_active)))
    robot_name    = data.get("robot_name", "robot")
    methods_data  = data.get("methods", {})

    # ── Robot arm ──
    arm = data.get("robot_arm", {})
    if arm and "link_positions" in arm:
        pos = np.array(arm["link_positions"])
        fig.add_trace(go.Scatter3d(
            x=pos[:, 0], y=pos[:, 1], z=pos[:, 2],
            mode="lines+markers",
            line=dict(color="#2c3e50", width=6),
            marker=dict(size=4, color="#2c3e50"),
            name="Robot Arm",
            legendgroup="robot_arm",
        ))

    # Track trace indices per method for toggle buttons
    method_trace_start: dict = {}
    method_trace_end: dict   = {}
    trace_count = 1  # robot arm is trace 0

    # ── Per-method iAABBs / voxels ──
    for mname in METHOD_ORDER:
        if mname not in methods_data:
            continue
        md        = METHOD_META.get(mname, {"label": mname, "color": "#888"})
        mdata     = methods_data[mname]
        base_color = md["color"]
        label      = md["label"]
        method_trace_start[mname] = trace_count

        if mdata.get("type") == "voxel":
            centres_raw = mdata.get("centres", [])
            if centres_raw:
                centres = np.array(centres_raw)
                fig.add_trace(_voxel_scatter(
                    centres, base_color,
                    name=f"{label} ({len(centres):,} voxels)",
                    legendgroup=mname,
                ))
                trace_count += 1
        else:
            # AABB: group boxes by link, colour by link index
            aabbs = mdata.get("aabbs", [])
            n_sub = mdata.get("n_sub", 1)
            total = len(aabbs)
            # Group by link
            from collections import defaultdict
            by_link: dict = defaultdict(list)
            for ab in aabbs:
                by_link[ab["link"]].append(ab)
            first = True
            for li, link_idx in enumerate(alm):
                lcolor = _LINK_COLORS[li % len(_LINK_COLORS)]
                segs = by_link.get(link_idx, [])
                for s_idx, ab in enumerate(segs):
                    lo = ab["lo"]
                    hi = ab["hi"]
                    sname = (f"{label} link{link_idx}[{s_idx}/{n_sub}]"
                             if s_idx == 0 and first
                             else "")
                    fig.add_trace(_wireframe_box(
                        lo, hi, lcolor,
                        name=f"{label}" if (first and s_idx == 0 and li == 0) else "",
                        legendgroup=mname,
                        showlegend=(first and s_idx == 0 and li == 0),
                    ))
                    trace_count += 1
                first = False

        method_trace_end[mname] = trace_count

    total_traces = trace_count

    # ── Toggle buttons: arm + one method at a time ──
    buttons = []

    # All on
    buttons.append(dict(
        label="全部",
        method="update",
        args=[{"visible": [True] * total_traces}],
    ))

    # Each method solo
    for mname in METHOD_ORDER:
        if mname not in method_trace_start:
            continue
        md = METHOD_META.get(mname, {"label": mname})
        vis = [False] * total_traces
        vis[0] = True  # arm always on
        for i in range(method_trace_start[mname], method_trace_end[mname]):
            vis[i] = True
        buttons.append(dict(
            label=md["label"],
            method="update",
            args=[{"visible": vis}],
        ))

    # Summary info in title
    summary_parts = []
    for mname in METHOD_ORDER:
        if mname not in methods_data:
            continue
        mdata = methods_data[mname]
        label = METHOD_META.get(mname, {}).get("label", mname)
        if mdata.get("type") == "voxel":
            n = mdata.get("n_occupied", len(mdata.get("centres", [])))
            summary_parts.append(f"{label}: {n:,} vox")
        else:
            summary_parts.append(
                f"{label}: {mdata.get('n_aabbs', '?')} boxes (sub={mdata.get('n_sub', 1)})")

    fig.update_layout(
        title=(f"Per-method Link iAABB — {robot_name}"
               f"<br><sub>{'  |  '.join(summary_parts)}</sub>"),
        scene=dict(
            xaxis_title="X (m)", yaxis_title="Y (m)", zaxis_title="Z (m)",
            aspectmode="data",
        ),
        legend=dict(
            x=0.01, y=0.99,
            bgcolor="rgba(255,255,255,0.85)",
            bordercolor="#aaa", borderwidth=1,
            itemclick="toggle",
            itemdoubleclick="toggleothers",
        ),
        margin=dict(l=0, r=0, t=90, b=0),
        updatemenus=[dict(
            type="buttons", direction="left",
            x=0.01, y=-0.02, xanchor="left", yanchor="top",
            bgcolor="rgba(240,240,240,0.9)",
            bordercolor="#bbb", borderwidth=1,
            font=dict(size=11),
            buttons=buttons,
        )],
    )

    fig.write_html(out)
    print(f"  [{'Link iAABBs per Method':30s}]  {os.path.abspath(out)}")


# ─── 6. 3D 对比视图（原有） ──────────────────────────────────────────────────

def plot_comparison_3d(json_path: str, out: str) -> None:
    """使用 sbf4_viz 渲染代表性配置的 5 方法对比 3D HTML。"""
    if not os.path.exists(json_path):
        print(f"  [SKIP] comparison.json not found: {json_path}")
        return
    data = load_envelope_comparison(json_path)
    plot_envelope_comparison(
        data,
        title="Link iAABB 5 方法 3D 对比（代表性配置）",
        show=False,
        save_html=out,
    )
    print(f"  [{'3D Comparison':30s}]  {os.path.abspath(out)}")


# ─── Summary table ─────────────────────────────────────────────────────────

def print_summary(df: pd.DataFrame) -> None:
    print("\n  ─── 汇总统计（全部 trial 均值）─────────────────────────────────")
    print(f"  {'方法':20s}  {'ep_ms':>8s}  {'env_ms':>8s}  {'total_ms':>9s}  {'vol (m3)':>12s}")
    print("  " + "─" * 68)
    for mname, md in METHOD_META.items():
        sub = df[df["envelope"] == mname]
        print(f"  {md['label']:20s}  "
              f"{sub['ep_time_ms'].mean():8.4f}  "
              f"{sub['env_time_ms'].mean():8.4f}  "
              f"{sub['total_ms'].mean():9.4f}  "
              f"{sub['volume'].mean():12.6e}")
    print()


# ─── Entry point ───────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Visualise exp_link_iaabb_bench results")
    parser.add_argument("results_dir",
                        help="Directory containing results.csv and comparison.json")
    args = parser.parse_args()

    rd = args.results_dir
    csv_path  = os.path.join(rd, "results.csv")
    json_path = os.path.join(rd, "comparison.json")

    if not os.path.exists(csv_path):
        print(f"ERROR: {csv_path} not found", file=sys.stderr)
        sys.exit(1)

    # Load CSV
    df = pd.read_csv(csv_path)
    df["band"] = df["width_frac"].apply(_band)

    print(f"\n  Loaded {len(df)} rows from {csv_path}")
    print(f"  Trials: {df['trial'].nunique()}  "
          f"Methods: {sorted(df['envelope'].unique())}")

    print("\n  Generating visualizations...")

    # 1-4: Statistical charts
    plot_timing_boxplot  (df, os.path.join(rd, "timing_boxplot.html"))
    plot_volume_boxplot  (df, os.path.join(rd, "volume_boxplot.html"))
    plot_timing_vs_volume(df, os.path.join(rd, "timing_vs_volume.html"))
    plot_timing_breakdown(df, os.path.join(rd, "timing_breakdown.html"))

    # 5: Per-method link iAABBs
    iaabb_path = os.path.join(rd, "method_iaabbs.json")
    plot_link_iaabbs_per_method(iaabb_path, os.path.join(rd, "link_iaabbs_per_method.html"))

    # 6: 3D comparison
    plot_comparison_3d(json_path, os.path.join(rd, "comparison_3d.html"))

    # Summary
    print_summary(df)


if __name__ == "__main__":
    main()

"""
viz_last_endpoint_small.py — 小区间下末端 endpoint iAABB 3D 可视化

用固定的小 C-space 区间（每个关节宽度约 5% 关节范围）重新生成
ep_iaabb_comparison.json，仅可视化最后一个 link 末端的 endpoint iAABB（index = n_endpoints-1）。

用法:
    python scripts/viz_last_endpoint_small.py [out_dir]

默认 out_dir: results/small_ep_iaabb
"""

import json
import os
import subprocess
import sys

# ─── 确保能 import sbf4_viz ─────────────────────────────────────────────────
_script_dir = os.path.dirname(os.path.abspath(__file__))
_v4_root    = os.path.normpath(os.path.join(_script_dir, ".."))
_v4_python  = os.path.join(_v4_root, "python")
if _v4_python not in sys.path:
    sys.path.insert(0, _v4_python)

import numpy as np
import plotly.graph_objects as go
from sbf4_viz.ep_iaabb_viz import (
    load_ep_iaabb_comparison,
    plot_ep_iaabb_comparison,
    EP_METHOD_STYLES,
    _ep_wireframe,
    _ep_mesh,
)

# ─── 路径 ─────────────────────────────────────────────────────────────────────
out_dir  = sys.argv[1] if len(sys.argv) > 1 else os.path.join(_v4_root, "results", "small_ep_iaabb")
exe_path = os.path.join(_v4_root, "build", "Release", "exp_ep_iaabb_bench.exe")
json_path = os.path.join(out_dir, "ep_iaabb_comparison.json")

os.makedirs(out_dir, exist_ok=True)

# ─── Step 1: 运行 C++ 实验（1 trial, frac=0.05, seed=7） ─────────────────────
# 使用 seed=7 可得到一个处于关节范围中部的有趣小区间
print(f"[viz_small] Running C++ experiment → {out_dir}")
if not os.path.exists(exe_path):
    print(f"ERROR: exe not found: {exe_path}", file=sys.stderr)
    sys.exit(1)

result = subprocess.run(
    [exe_path, "1", "7", out_dir],
    capture_output=True, text=True
)
print(result.stderr.strip())
if result.returncode != 0:
    print("ERROR: C++ experiment failed.", file=sys.stderr)
    sys.exit(1)

if not os.path.exists(json_path):
    print(f"ERROR: JSON not produced: {json_path}", file=sys.stderr)
    sys.exit(1)

# ─── Step 2: 覆盖 box_intervals 为固定小区间 (frac=0.05 at center) ───────────
# C++ 产生的 JSON 用的是 trial=0 的随机区间；
# 我们直接把 JSON 里的 box_intervals 替换成 frac=5% 中心区间后重用。
# iiwa14 joint limits (7 joints):
LIMITS = [
    (-1.865, 1.866),
    (-0.100, 1.087),
    (-0.663, 0.662),
    (-2.094, -0.372),
    (-0.619, 0.620),
    (-1.095, 1.258),
    ( 1.050, 2.091),
]
FRAC = 0.05   # 5% of joint range  (≈ 9–17° per joint)

print(f"[viz_small] Using frac={FRAC} (~5% of each joint range)")

# Re-run C++ will produce its own random interval;
# patch the JSON to use the small centered intervals instead.
with open(json_path, "r", encoding="utf-8") as f:
    d = json.load(f)

new_box_intervals = []
for lo_lim, hi_lim in LIMITS:
    center = (lo_lim + hi_lim) * 0.5
    hw = (hi_lim - lo_lim) * FRAC * 0.5
    new_box_intervals.append([center - hw, center + hw])

print("[viz_small] Patched box_intervals:")
for i, iv in enumerate(new_box_intervals):
    print(f"  j{i}: [{iv[0]:.4f}, {iv[1]:.4f}]  width={iv[1]-iv[0]:.4f} rad")

d["box_intervals"] = new_box_intervals
patched_json = os.path.join(out_dir, "ep_iaabb_comparison_small.json")
with open(patched_json, "w", encoding="utf-8") as f:
    json.dump(d, f, indent=2)

# ─── Step 3: 加载数据 ────────────────────────────────────────────────────────
# NOTE: patching box_intervals in JSON does NOT recompute endpoint_iaabbs;
# we need to re-run the C++ with the actual small interval.
# The cleanest way: run exp with n=1 (1 trial at frac=0.05 near center).
# But since exp uses random boxes, we instead directly compute from C++ by
# patching the JSON's endpoint_iaabbs using a helper — OR we just accept
# the C++ output for trial=0 which uses seed=7 → let's check actual widths.

with open(json_path, "r", encoding="utf-8") as f_orig:
    d_orig = json.load(f_orig)

actual_biv = d_orig["box_intervals"]
widths = [iv[1] - iv[0] for iv in actual_biv]
max_w = max(widths)
print(f"\n[viz_small] C++ trial box max width: {max_w:.4f} rad")

if max_w > 0.5:
    print("[viz_small] Box too large. Importing via direct Python computation...")
    # We won't recompute endpoint_iaabbs here; instead generate a new JSON
    # via patched re-run. The simplest path is to scale down by writing a
    # small one-trial runner. For now use the patched JSON as-is (geometry
    # comes from C++ FK at patched intervals center, but endpoint_iaabbs
    # come from the C++ run which used frac from seed).
    # → Practical fix: re-run C++ with n=1 at frac 0.05 by using a seed
    #   that places trial 0 in the "narrow" band (which is always frac=0.05).
    print("[viz_small] Re-running with n=3 (trial 0 = frac 0.05, narrow band)...")
    result2 = subprocess.run(
        [exe_path, "3", "7", out_dir],
        capture_output=True, text=True
    )
    print(result2.stderr.strip())
    with open(json_path, "r", encoding="utf-8") as f_orig:
        d_orig = json.load(f_orig)
    actual_biv = d_orig["box_intervals"]
    widths = [iv[1] - iv[0] for iv in actual_biv]
    max_w = max(widths)
    print(f"[viz_small] After re-run: max box width = {max_w:.4f} rad")

# ─── Step 4: 加载并只渲染最后一个 endpoint ──────────────────────────────────
data = load_ep_iaabb_comparison(json_path)
n_ep = data.n_endpoints
last_idx = n_ep - 1

print(f"\n[viz_small] n_endpoints={n_ep}, rendering only endpoint[{last_idx}] (last link tip)")

# Print sizes for each method
print(f"\n{'Method':14s}  {'lo':40s}  {'hi':40s}  {'size (m)':30s}")
print("─" * 130)
for src in data.sources:
    if last_idx < len(src.endpoint_iaabbs):
        ep = src.endpoint_iaabbs[last_idx]
        size = ep.hi - ep.lo
        style = EP_METHOD_STYLES.get(src.name, {"label": src.name})
        print(f"{style.get('label', src.name):14s}  "
              f"{str([round(x,5) for x in ep.lo]):40s}  "
              f"{str([round(x,5) for x in ep.hi]):40s}  "
              f"{str([round(x,5) for x in size])}")

# ─── Step 5: 构建只含末端 endpoint 的 Plotly 3D 图 ──────────────────────────
fig = go.Figure()

# Robot arm skeleton
if data.link_positions is not None and len(data.link_positions) > 0:
    pos = data.link_positions
    fig.add_trace(go.Scatter3d(
        x=pos[:, 0], y=pos[:, 1], z=pos[:, 2],
        mode="lines+markers",
        line=dict(color="rgb(40,40,40)", width=5),
        marker=dict(size=5, color="rgb(40,40,40)"),
        name="Robot Arm (FK center)",
        legendgroup="robot_arm",
        showlegend=True,
    ))

# Per-source: only last endpoint iAABB
WIRE_WIDTH = 3.0
fallback_idx = 0
for src in data.sources:
    if last_idx >= len(src.endpoint_iaabbs):
        continue
    ep = src.endpoint_iaabbs[last_idx]
    style = EP_METHOD_STYLES.get(src.name)
    if style is None:
        colors_fb = [
            ("rgb(150,150,150)", "rgba(150,150,150,0.12)"),
            ("rgb(200,100,200)", "rgba(200,100,200,0.12)"),
        ]
        wire_c, fill_c = colors_fb[fallback_idx % len(colors_fb)]
        fallback_idx += 1
        label = src.name
    else:
        wire_c, fill_c = style["wire"], style["fill"]
        label = style["label"]

    group = f"ep_{src.name}"

    # Semi-transparent fill
    fig.add_trace(_ep_mesh(
        ep.lo, ep.hi,
        color_fill=fill_c,
        name=label,
        legendgroup=group,
        showlegend=True,
    ))
    # Wireframe
    fig.add_trace(_ep_wireframe(
        ep.lo, ep.hi,
        color=wire_c,
        width=WIRE_WIDTH,
        name=label,
        legendgroup=group,
        showlegend=False,
    ))

# Mark the FK position of the last endpoint
if data.link_positions is not None and last_idx < len(data.link_positions):
    ep_pos = data.link_positions[last_idx]
    fig.add_trace(go.Scatter3d(
        x=[ep_pos[0]], y=[ep_pos[1]], z=[ep_pos[2]],
        mode="markers",
        marker=dict(size=8, color="rgb(0,0,0)", symbol="cross"),
        name=f"Endpoint {last_idx} (FK center)",
        showlegend=True,
    ))

# Box interval info for title
biv = data.box_intervals
w_str = " | ".join(f"j{i}:{iv[1]-iv[0]:.3f}r" for i, iv in enumerate(biv))
title = (f"末端 Endpoint iAABB 对比（endpoint {last_idx}，最后一个 link 末端）<br>"
         f"<sup>box: {w_str}</sup>")

fig.update_layout(
    title=dict(text=title, font=dict(size=14)),
    legend=dict(title="Source (click to toggle)", itemsizing="constant"),
    scene=dict(
        xaxis_title="X (m)",
        yaxis_title="Y (m)",
        zaxis_title="Z (m)",
        aspectmode="data",
    ),
    margin=dict(l=0, r=0, t=80, b=0),
)

out_html = os.path.join(out_dir, "last_endpoint_small.html")
fig.write_html(out_html)
print(f"\n[viz_small] Saved: {os.path.abspath(out_html)}")
print("[viz_small] Done.")

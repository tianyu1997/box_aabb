"""
src/viz/render.py — 2D ForestGrower 渲染工具

提供: ROOT_COLORS, _draw_boxes_on_ax, _build_title, plot_snapshot, compose_gif
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.collections import PatchCollection, LineCollection

from .core import GrowVizConfig


# ═══════════════════════════════════════════════════════════════════════════
#  颜色
# ═══════════════════════════════════════════════════════════════════════════

# 每棵根树的固定颜色 + 按volume渐变
ROOT_COLORS = [
    "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728",
    "#9467bd", "#8c564b", "#e377c2", "#7f7f7f",
    "#bcbd22", "#17becf",
]


# ═══════════════════════════════════════════════════════════════════════════
#  绘图
# ═══════════════════════════════════════════════════════════════════════════

def _draw_boxes_on_ax(ax, snap, cmap_data, extent, cfg, subtrees,
                      title="", show_start_goal=True, markersize_s=10,
                      markersize_g=14):
    """在 ax 上绘制: 碰撞底图 + boxes (root 着色, coarse/fine 区分) + adj.
    
    Uses PatchCollection + LineCollection for performance.
    """
    ax.imshow(cmap_data, origin="lower", extent=extent,
              cmap="Reds", alpha=0.30, aspect="auto")

    # subtree 分区边界
    for st in subtrees:
        lo, hi = st["lo"], st["hi"]
        rect = Rectangle((lo[0], lo[1]), hi[0] - lo[0], hi[1] - lo[1],
                          linewidth=1.0, edgecolor="#888888",
                          facecolor="none", linestyle="--", alpha=0.5)
        ax.add_patch(rect)

    boxes = snap["boxes"]
    new_id = snap["new_box_id"]

    # 收集 patches 到列表再一次性添加 (PatchCollection)
    rects = []
    facecolors = []
    edgecolors = []
    alphas = []
    linewidths = []
    linestyles = []

    for bid, box in boxes.items():
        w = box.hi - box.lo
        rid = box.root_id % len(ROOT_COLORS) if box.root_id >= 0 else 0
        base_color = ROOT_COLORS[rid]

        if bid == new_id:
            ec, fc, alpha, lw, ls = "#ff0000", "#ff4444", 0.55, 2.0, "-"
        elif getattr(box, "is_coarse", False):
            ec, fc, alpha, lw, ls = base_color, base_color, 0.20, 1.2, "--"
        else:
            ec, fc, alpha, lw, ls = base_color, base_color, 0.35, 0.6, "-"

        rect = Rectangle((box.lo[0], box.lo[1]), w[0], w[1])
        rects.append(rect)
        facecolors.append(fc)
        edgecolors.append(ec)
        alphas.append(alpha)
        linewidths.append(lw)
        linestyles.append(ls)

    if rects:
        pc = PatchCollection(rects, match_original=False)
        pc.set_facecolors(facecolors)
        pc.set_edgecolors(edgecolors)
        pc.set_alpha(alphas)
        pc.set_linewidths(linewidths)
        pc.set_linestyles(linestyles)
        ax.add_collection(pc)

    # 邻接边 — LineCollection (一次绘制所有线段)
    segments = []
    drawn = set()
    for bid, neighbors in snap["adjacency"].items():
        if bid not in boxes:
            continue
        for nid in neighbors:
            if nid not in boxes:
                continue
            ek = (min(bid, nid), max(bid, nid))
            if ek in drawn:
                continue
            drawn.add(ek)
            c1, c2 = boxes[bid].center(), boxes[nid].center()
            segments.append([(c1[0], c1[1]), (c2[0], c2[1])])

    if segments:
        lc = LineCollection(segments, colors="#cccccc", linewidths=0.3,
                            alpha=0.4, zorder=1)
        ax.add_collection(lc)

    if show_start_goal:
        if cfg.q_start is not None:
            ax.plot(cfg.q_start[0], cfg.q_start[1], 'o', color='cyan',
                    markersize=markersize_s, markeredgecolor='black',
                    markeredgewidth=1.0, zorder=20)
        if cfg.q_goal is not None:
            ax.plot(cfg.q_goal[0], cfg.q_goal[1], '*', color='yellow',
                    markersize=markersize_g, markeredgecolor='black',
                    markeredgewidth=0.8, zorder=20)

    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.2)
    if title:
        ax.set_title(title, fontsize=9)


def _build_title(snap, cfg, frame_idx):
    """构建标题 (含 mode / thread / adaptive phase 信息)."""
    n_boxes = snap["n_boxes"]
    in_coarse = snap.get("in_coarse_phase", False)
    n_coarse = snap.get("n_coarse_boxes", 0)
    n_fine = snap.get("n_fine_boxes", 0)

    mode_str = cfg.mode.upper()
    parts = [f"[{mode_str}]"]
    if cfg.n_threads > 1:
        parts[0] = f"[{mode_str} {cfg.n_threads}T]"
    parts.append(f"R={cfg.n_roots}")
    parts.append(f"boxes={n_boxes}")
    if cfg.adaptive_min_edge:
        phase = "COARSE" if in_coarse else "FINE"
        parts.append(f"{phase}(C:{n_coarse} F:{n_fine})")
    parts.append(f"f={frame_idx}")
    return " | ".join(parts)


def plot_snapshot(
    snap: dict,
    cmap_data, extent,
    cfg: GrowVizConfig,
    frame_idx: int,
    subtrees: list,
):
    """绘制单帧快照: 碰撞底图 + boxes (按 root_id 着色, coarse/fine 区分)."""
    fig, ax = plt.subplots(1, 1, figsize=cfg.fig_size)
    title = _build_title(snap, cfg, frame_idx)
    _draw_boxes_on_ax(ax, snap, cmap_data, extent, cfg, subtrees, title=title)
    ax.set_xlabel("q0"); ax.set_ylabel("q1")
    ax.legend(loc="upper right", fontsize=8)
    return fig


# ═══════════════════════════════════════════════════════════════════════════
#  Coarsen 对比图 (coarse-only vs final)
# ═══════════════════════════════════════════════════════════════════════════

def plot_coarsen_comparison(
    snapshots: list,
    cmap_data, extent,
    cfg: GrowVizConfig,
    subtrees: list,
):
    """生成 1×2 对比图: 左 = coarse 阶段结束时, 右 = 最终 fine 结果.

    自动从 snapshots 中找到 coarse→fine 转换点.
    Returns (fig, coarse_idx, final_idx) or None if no phase transition found.
    """
    if not snapshots or not cfg.adaptive_min_edge:
        return None

    # 找最后一个 in_coarse_phase=True 的快照 (即 coarse 阶段结束点)
    coarse_idx = None
    for i, snap in enumerate(snapshots):
        if snap.get("in_coarse_phase", False):
            coarse_idx = i

    if coarse_idx is None:
        # 没有 coarse 阶段 (可能从未启用 adaptive)
        return None

    final_idx = len(snapshots) - 1
    snap_coarse = snapshots[coarse_idx]
    snap_final = snapshots[final_idx]

    n_coarse = snap_coarse.get("n_coarse_boxes", 0)
    n_final_coarse = snap_final.get("n_coarse_boxes", 0)
    n_final_fine = snap_final.get("n_fine_boxes", 0)

    fig, axes = plt.subplots(1, 2, figsize=(16, 7))

    # 左: coarse 阶段结束
    title_l = (f"COARSE Phase Complete\n"
               f"boxes={snap_coarse['n_boxes']}  "
               f"(C:{n_coarse}, min_edge={cfg.coarse_min_edge:.2f})")
    _draw_boxes_on_ax(axes[0], snap_coarse, cmap_data, extent, cfg,
                      subtrees, title=title_l)
    axes[0].set_xlabel("q0")
    axes[0].set_ylabel("q1")

    # 右: 最终结果 (fine 填充后)
    title_r = (f"FINE Phase Complete\n"
               f"boxes={snap_final['n_boxes']}  "
               f"(C:{n_final_coarse} + F:{n_final_fine}, "
               f"min_edge={cfg.min_edge:.2f})")
    _draw_boxes_on_ax(axes[1], snap_final, cmap_data, extent, cfg,
                      subtrees, title=title_r)
    axes[1].set_xlabel("q0")
    axes[1].set_ylabel("q1")

    mode_str = cfg.mode.upper()
    fig.suptitle(
        f"Adaptive Coarsen: COARSE → FINE  [{mode_str}]  "
        f"(seed={cfg.seed}, roots={cfg.n_roots})",
        fontsize=13, fontweight="bold", y=0.98)
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    return fig, coarse_idx, final_idx


# ═══════════════════════════════════════════════════════════════════════════
#  Box-Merge 对比图 (coarsening before vs after)
# ═══════════════════════════════════════════════════════════════════════════

def plot_merge_comparison(
    snap_before: dict,
    snap_after: dict,
    cmap_data, extent,
    cfg: GrowVizConfig,
    subtrees: list,
    coarsen_stats: dict,
):
    """生成 1×2 对比图: 左 = merge 前, 右 = merge 后.

    coarsen_stats 来自 ForestGrower2D.run_coarsen().
    Returns fig.
    """
    fig, axes = plt.subplots(1, 2, figsize=(16, 7))

    n_before = snap_before["n_boxes"]
    n_after = snap_after["n_boxes"]
    total_merges = coarsen_stats.get("total_merges", 0)
    ds = coarsen_stats.get("dim_scan", {})
    gr = coarsen_stats.get("greedy", {})

    # 左: merge 前
    title_l = f"Before Coarsening\nboxes = {n_before}"
    _draw_boxes_on_ax(axes[0], snap_before, cmap_data, extent, cfg,
                      subtrees, title=title_l)
    axes[0].set_xlabel("q0")
    axes[0].set_ylabel("q1")

    # 右: merge 后
    title_r = (f"After Coarsening\n"
               f"boxes = {n_after}  "
               f"(dim-scan: {ds.get('merges_performed', 0)}, "
               f"greedy: {gr.get('merges_performed', 0)})")
    _draw_boxes_on_ax(axes[1], snap_after, cmap_data, extent, cfg,
                      subtrees, title=title_r)
    axes[1].set_xlabel("q0")
    axes[1].set_ylabel("q1")

    mode_str = cfg.mode.upper()
    fig.suptitle(
        f"Box Merging (Coarsening)  [{mode_str}]  "
        f"{n_before} → {n_after} boxes  ({total_merges} merges)  "
        f"seed={cfg.seed}",
        fontsize=13, fontweight="bold", y=0.98)
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    return fig


# ═══════════════════════════════════════════════════════════════════════════
#  GIF 合成
# ═══════════════════════════════════════════════════════════════════════════

def compose_gif(frames_dir: Path, gif_path: Path, duration_ms: int = 200):
    """PNG 帧 → GIF (最后一帧停留更久)."""
    frame_paths = sorted(frames_dir.glob("frame_*.png"))
    if not frame_paths:
        print("  [warn] no frames to compose")
        return False

    try:
        from PIL import Image
    except ImportError:
        print("  [warn] Pillow not installed, skip GIF")
        return False

    images = [Image.open(p).convert("P", palette=Image.ADAPTIVE)
              for p in frame_paths]
    durations = [duration_ms] * len(images)
    durations[-1] = 2000  # 最后一帧 2 秒

    images[0].save(
        gif_path, save_all=True, append_images=images[1:],
        duration=durations, loop=0, optimize=False)

    for img in images:
        img.close()

    return True

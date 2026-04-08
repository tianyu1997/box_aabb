"""
src/viz/wavefront_step.py — 可视化 *一次* 波前扩展 (Wavefront BFS Step)

针对一个 parent box, 展示完整的波前扩展步骤:
  Step 0: 已有 forest + 选中 parent box (粗边框高亮)
  Step 1: parent box 的有效面 + 面上采样的 seed 点
  Step 2-N: 每个 seed 导出的 child box 逐个出现 (橙色高亮)
  Final:  所有 child box 就位, 邻接边画出

输出: 多帧 PNG + GIF, 存于 results/viz_wavefront_step_<timestamp>/

用法:
    cd safeboxforest/v3
    python -m src.viz.wavefront_step [--seed 42] [--pre-grow 20]
"""
from __future__ import annotations

import argparse
import json
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrowPatch, FancyArrow
import matplotlib.patches as mpatches
import matplotlib.colors as mcolors
import numpy as np

from .core import GrowVizConfig, Obstacle2D, CSpace2D, BoxInfo
from .ffb_engine import FFBEngine
from .forest_grower_2d import ForestGrower2D, build_random_scene
from .render import ROOT_COLORS

# ── 中文字体设置 ──
for _fname in ["Microsoft YaHei", "SimHei", "SimSun", "STSong"]:
    try:
        matplotlib.font_manager.FontProperties(family=_fname).get_name()
        plt.rcParams["font.sans-serif"] = [_fname, "DejaVu Sans"]
        plt.rcParams["axes.unicode_minus"] = False
        break
    except Exception:
        continue


# ═══════════════════════════════════════════════════════════════════════════
#  工具函数
# ═══════════════════════════════════════════════════════════════════════════

FACE_LABELS = {
    (0, 0): "x- (左)",
    (0, 1): "x+ (右)",
    (1, 0): "y- (下)",
    (1, 1): "y+ (上)",
}

SEED_MARKERS = ["D", "s", "^", "v", "o", "P"]  # 每个 seed 不同形状

# 面的箭头方向 (法线方向)
FACE_ARROW_DIR = {
    (0, 0): np.array([-1,  0]),
    (0, 1): np.array([ 1,  0]),
    (1, 0): np.array([ 0, -1]),
    (1, 1): np.array([ 0,  1]),
}


def _face_midpoint(box: BoxInfo, dim: int, side: int) -> np.ndarray:
    """面中点."""
    mid = box.center().copy()
    mid[dim] = box.lo[dim] if side == 0 else box.hi[dim]
    return mid


def _pad_lim(lo, hi, margin_ratio=0.15):
    """给 xlim/ylim 加 padding."""
    span = hi - lo
    margin = span * margin_ratio
    return lo - margin, hi + margin


# ═══════════════════════════════════════════════════════════════════════════
#  绘图核心
# ═══════════════════════════════════════════════════════════════════════════

def _draw_base(ax, boxes: Dict[int, BoxInfo], adjacency: Dict[int, Set[int]],
               cmap_data, extent, cfg: GrowVizConfig,
               subtrees: list,
               highlight_ids: Set[int] = None,
               parent_id: int = -1):
    """绘制基础 forest: 碰撞底图 + boxes + 邻接边 + start/goal."""
    if highlight_ids is None:
        highlight_ids = set()

    # 碰撞底图
    ax.imshow(cmap_data, origin="lower", extent=extent,
              cmap="Reds", alpha=0.25, aspect="auto")

    # subtree 虚线
    for st in subtrees:
        lo, hi = st["lo"], st["hi"]
        rect = Rectangle((lo[0], lo[1]), hi[0] - lo[0], hi[1] - lo[1],
                          linewidth=0.8, edgecolor="#888888",
                          facecolor="none", linestyle="--", alpha=0.4)
        ax.add_patch(rect)

    # boxes
    for bid, box in boxes.items():
        w = box.hi - box.lo
        rid = box.root_id % len(ROOT_COLORS) if box.root_id >= 0 else 0
        base_color = ROOT_COLORS[rid]

        if bid == parent_id:
            # parent box: 粗黑 + 半透明蓝色
            ec, fc, alpha, lw = "#000000", "#4488ff", 0.45, 2.8
        elif bid in highlight_ids:
            # 新增 child: 橙色高亮
            ec, fc, alpha, lw = "#ff6600", "#ffaa33", 0.55, 2.2
        else:
            ec, fc, alpha, lw = base_color, base_color, 0.25, 0.5

        rect = Rectangle(
            (box.lo[0], box.lo[1]), w[0], w[1],
            linewidth=lw, edgecolor=ec, facecolor=fc, alpha=alpha)
        ax.add_patch(rect)

    # 邻接边 (中心连线)
    drawn = set()
    for bid, neighbors in adjacency.items():
        if bid not in boxes:
            continue
        for nid in neighbors:
            if nid not in boxes:
                continue
            key = (min(bid, nid), max(bid, nid))
            if key in drawn:
                continue
            drawn.add(key)
            c1 = boxes[bid].center()
            c2 = boxes[nid].center()
            ax.plot([c1[0], c2[0]], [c1[1], c2[1]],
                    color="#cccccc", linewidth=0.3, alpha=0.4, zorder=1)

    # start / goal
    if cfg.q_start is not None:
        ax.plot(cfg.q_start[0], cfg.q_start[1], 'o', color='cyan',
                markersize=9, markeredgecolor='black',
                markeredgewidth=1.0, zorder=20)
    if cfg.q_goal is not None:
        ax.plot(cfg.q_goal[0], cfg.q_goal[1], '*', color='yellow',
                markersize=12, markeredgecolor='black',
                markeredgewidth=0.8, zorder=20)


def render_step0(boxes, adjacency, cmap_data, extent, cfg, subtrees,
                 parent_box: BoxInfo, figsize=(10, 8)):
    """Step 0: 已有 forest + parent box 高亮."""
    fig, ax = plt.subplots(1, 1, figsize=figsize)
    _draw_base(ax, boxes, adjacency, cmap_data, extent, cfg,
               subtrees, parent_id=parent_box.box_id)

    # parent label
    pc = parent_box.center()
    ax.annotate(f"Parent #{parent_box.box_id}",
                xy=(pc[0], pc[1]),
                xytext=(pc[0] + 0.25, pc[1] + 0.35),
                fontsize=9, fontweight="bold", color="#0044aa",
                arrowprops=dict(arrowstyle="->", color="#0044aa", lw=1.2),
                bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="#0044aa",
                          alpha=0.85),
                zorder=30)

    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_xlabel("q0"); ax.set_ylabel("q1")
    ax.set_title("Step 0: 选中 Parent Box (蓝色高亮)", fontsize=11)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.2)
    return fig


def render_step1(boxes, adjacency, cmap_data, extent, cfg, subtrees,
                 parent_box: BoxInfo, faces, seeds_by_face, figsize=(10, 8)):
    """Step 1: parent box 的有效面标注 + seed 点."""
    fig, ax = plt.subplots(1, 1, figsize=figsize)
    _draw_base(ax, boxes, adjacency, cmap_data, extent, cfg,
               subtrees, parent_id=parent_box.box_id)

    # ── 画有效面 (彩色粗线段) ──
    face_colors = ["#e6194b", "#3cb44b", "#4363d8", "#f58231"]
    for fi, (dim, side) in enumerate(faces):
        color = face_colors[fi % len(face_colors)]
        if dim == 0:
            # 竖线
            x = parent_box.lo[0] if side == 0 else parent_box.hi[0]
            y0, y1 = parent_box.lo[1], parent_box.hi[1]
            ax.plot([x, x], [y0, y1], color=color, linewidth=3.5,
                    solid_capstyle="round", zorder=15, alpha=0.85)
        else:
            # 横线
            y = parent_box.lo[1] if side == 0 else parent_box.hi[1]
            x0, x1 = parent_box.lo[0], parent_box.hi[0]
            ax.plot([x0, x1], [y, y], color=color, linewidth=3.5,
                    solid_capstyle="round", zorder=15, alpha=0.85)

        # 面方向箭头 (法线)
        mid = _face_midpoint(parent_box, dim, side)
        arrow_dir = FACE_ARROW_DIR[(dim, side)]
        arrow_len = 0.2
        ax.annotate("", xy=(mid[0] + arrow_dir[0] * arrow_len,
                            mid[1] + arrow_dir[1] * arrow_len),
                     xytext=(mid[0], mid[1]),
                     arrowprops=dict(arrowstyle="->,head_width=0.15",
                                     color=color, lw=2.0),
                     zorder=16)

        # 面标签
        label_offset = arrow_dir * 0.35
        ax.text(mid[0] + label_offset[0], mid[1] + label_offset[1],
                FACE_LABELS.get((dim, side), f"d{dim}s{side}"),
                fontsize=8, ha="center", va="center",
                color=color, fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.2", fc="white", ec=color,
                          alpha=0.85),
                zorder=17)

    # ── 画 seed 点 ──
    for si, (dim, side, q_seed) in enumerate(seeds_by_face):
        fi = faces.index((dim, side)) if (dim, side) in faces else 0
        color = face_colors[fi % len(face_colors)]
        marker = SEED_MARKERS[si % len(SEED_MARKERS)]
        ax.plot(q_seed[0], q_seed[1], marker=marker, color=color,
                markersize=10, markeredgecolor="black", markeredgewidth=0.8,
                zorder=18)
        # 标注 seed 编号
        ax.text(q_seed[0] + 0.08, q_seed[1] + 0.08, f"s{si}",
                fontsize=7, color=color, fontweight="bold", zorder=19)

    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_xlabel("q0"); ax.set_ylabel("q1")
    ax.set_title(f"Step 1: 边界面标注 ({len(faces)} 面) + "
                 f"采样 {len(seeds_by_face)} 个 Seed", fontsize=11)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.2)
    return fig


def render_step_child(boxes, adjacency, cmap_data, extent, cfg, subtrees,
                      parent_box: BoxInfo, faces, seeds_by_face,
                      children_so_far: List[BoxInfo],
                      new_child: BoxInfo, seed_idx: int,
                      figsize=(10, 8)):
    """Step 2+: 逐个 child box 出现."""
    fig, ax = plt.subplots(1, 1, figsize=figsize)

    # 把 children 加入 boxes 用于绘制
    all_boxes = dict(boxes)
    child_ids = set()
    for ch in children_so_far:
        all_boxes[ch.box_id] = ch
        child_ids.add(ch.box_id)

    _draw_base(ax, all_boxes, adjacency, cmap_data, extent, cfg,
               subtrees, highlight_ids=child_ids, parent_id=parent_box.box_id)

    # 仍然画面和 seed 点 (淡化)
    face_colors = ["#e6194b", "#3cb44b", "#4363d8", "#f58231"]
    for fi, (dim, side) in enumerate(faces):
        color = face_colors[fi % len(face_colors)]
        if dim == 0:
            x = parent_box.lo[0] if side == 0 else parent_box.hi[0]
            y0, y1 = parent_box.lo[1], parent_box.hi[1]
            ax.plot([x, x], [y0, y1], color=color, linewidth=2.5,
                    solid_capstyle="round", zorder=15, alpha=0.45)
        else:
            y = parent_box.lo[1] if side == 0 else parent_box.hi[1]
            x0, x1 = parent_box.lo[0], parent_box.hi[0]
            ax.plot([x0, x1], [y, y], color=color, linewidth=2.5,
                    solid_capstyle="round", zorder=15, alpha=0.45)

    for si, (dim, side, q_seed) in enumerate(seeds_by_face):
        fi = faces.index((dim, side)) if (dim, side) in faces else 0
        color = face_colors[fi % len(face_colors)]
        marker = SEED_MARKERS[si % len(SEED_MARKERS)]
        ax.plot(q_seed[0], q_seed[1], marker=marker, color=color,
                markersize=8, markeredgecolor="black", markeredgewidth=0.6,
                zorder=18, alpha=0.6)

    # 新 child 标注: 箭头 from seed → child center
    nc = new_child.center()
    dim_s, side_s, q_s = seeds_by_face[seed_idx]
    ax.annotate(f"Child #{new_child.box_id}\n"
                f"(from seed s{seed_idx})",
                xy=(nc[0], nc[1]),
                xytext=(q_s[0], q_s[1]),
                fontsize=8, color="#cc4400", fontweight="bold",
                arrowprops=dict(arrowstyle="->,head_width=0.12",
                                color="#cc4400", lw=1.5,
                                connectionstyle="arc3,rad=0.15"),
                bbox=dict(boxstyle="round,pad=0.2", fc="#fff8ee",
                          ec="#cc4400", alpha=0.9),
                zorder=25)

    # 新的邻接边 (红色)
    for nbr_id in adjacency.get(new_child.box_id, set()):
        if nbr_id in all_boxes:
            c1 = new_child.center()
            c2 = all_boxes[nbr_id].center()
            ax.plot([c1[0], c2[0]], [c1[1], c2[1]],
                    color="#ff6600", linewidth=1.2, alpha=0.6, zorder=10,
                    linestyle="--")

    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_xlabel("q0"); ax.set_ylabel("q1")
    ax.set_title(f"Step {seed_idx + 2}: Child #{new_child.box_id} 从 seed s{seed_idx} 生长",
                 fontsize=11)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.2)
    return fig


def render_final(boxes, adjacency, cmap_data, extent, cfg, subtrees,
                 parent_box: BoxInfo, children: List[BoxInfo],
                 figsize=(10, 8)):
    """最终帧: 所有 child 就位, 邻接边完整."""
    fig, ax = plt.subplots(1, 1, figsize=figsize)
    all_boxes = dict(boxes)
    child_ids = set()
    for ch in children:
        all_boxes[ch.box_id] = ch
        child_ids.add(ch.box_id)

    _draw_base(ax, all_boxes, adjacency, cmap_data, extent, cfg,
               subtrees, highlight_ids=child_ids, parent_id=parent_box.box_id)

    # parent label
    pc = parent_box.center()
    ax.annotate(f"Parent #{parent_box.box_id}",
                xy=(pc[0], pc[1]),
                xytext=(pc[0] + 0.2, pc[1] + 0.3),
                fontsize=9, fontweight="bold", color="#0044aa",
                arrowprops=dict(arrowstyle="->", color="#0044aa", lw=1.0),
                bbox=dict(boxstyle="round,pad=0.3", fc="white",
                          ec="#0044aa", alpha=0.85),
                zorder=30)

    # children labels
    for ch in children:
        cc = ch.center()
        ax.text(cc[0], cc[1], f"#{ch.box_id}", fontsize=7,
                ha="center", va="center", color="#883300",
                fontweight="bold", zorder=25,
                bbox=dict(boxstyle="round,pad=0.15", fc="white",
                          ec="#ff8800", alpha=0.7))

    # 所有新邻接边 (红色虚线)
    for ch in children:
        for nbr_id in adjacency.get(ch.box_id, set()):
            if nbr_id in all_boxes:
                c1 = ch.center()
                c2 = all_boxes[nbr_id].center()
                ax.plot([c1[0], c2[0]], [c1[1], c2[1]],
                        color="#ff6600", linewidth=1.0, alpha=0.5,
                        zorder=10, linestyle="--")

    n_adj_new = sum(len(adjacency.get(ch.box_id, set())) for ch in children)
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_xlabel("q0"); ax.set_ylabel("q1")
    ax.set_title(f"Final: {len(children)} 个 Child 生成, "
                 f"{n_adj_new} 条新邻接边", fontsize=11)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.2)

    # 图例
    legend_elems = [
        mpatches.Patch(facecolor="#4488ff", edgecolor="black",
                       linewidth=2.0, alpha=0.5, label="Parent Box"),
        mpatches.Patch(facecolor="#ffaa33", edgecolor="#ff6600",
                       linewidth=1.5, alpha=0.55, label="Child Box"),
        plt.Line2D([0], [0], color="#ff6600", linewidth=1.0,
                   linestyle="--", alpha=0.6, label="新邻接边"),
    ]
    ax.legend(handles=legend_elems, loc="upper right", fontsize=8)
    return fig


# ═══════════════════════════════════════════════════════════════════════════
#  GIF 合成 (变速: step0 慢, final 最慢)
# ═══════════════════════════════════════════════════════════════════════════

def compose_gif_wavefront(frames_dir: Path, gif_path: Path,
                          duration_ms: int = 800):
    """PNG 帧 → GIF (变速: step0=1.5s, step1=2s, child=duration_ms, final=3s)."""
    from PIL import Image
    paths = sorted(frames_dir.glob("step_*.png"))
    if not paths:
        print("  [warn] no frames found")
        return False
    imgs = [Image.open(p).convert("P", palette=Image.ADAPTIVE) for p in paths]
    durations = []
    for i in range(len(imgs)):
        if i == 0:
            durations.append(1500)
        elif i == 1:
            durations.append(2000)
        elif i == len(imgs) - 1:
            durations.append(3000)
        else:
            durations.append(duration_ms)
    imgs[0].save(gif_path, save_all=True, append_images=imgs[1:],
                 duration=durations, loop=0, optimize=False)
    for img in imgs:
        img.close()
    return True


# ═══════════════════════════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="可视化一次波前扩展 (Wavefront BFS Step)")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--pre-grow", type=int, default=15,
                        help="先用 wavefront 生长到多少个 box (再选 parent)")
    parser.add_argument("--n-roots", type=int, default=3)
    parser.add_argument("--obstacles", type=int, default=8)
    parser.add_argument("--parent-idx", type=int, default=-1,
                        help="选择第 N 个 box 作为 parent (-1=自动选最大 box)")
    args = parser.parse_args()

    # ── 配置 ──────────────────────────────────────────────────────────
    cfg = GrowVizConfig(
        seed=args.seed,
        mode="wavefront",
        n_roots=args.n_roots,
        max_boxes=args.pre_grow,
        n_obstacles=args.obstacles,
        n_boundary_samples=6,
        boundary_epsilon=0.02,
        goal_face_bias=0.6,
    )
    rng = np.random.default_rng(cfg.seed)

    # ── 输出目录 ──────────────────────────────────────────────────────
    ts = time.strftime("%Y%m%d_%H%M%S")
    viz_dir = Path(__file__).resolve().parent       # src/viz
    v3_root = viz_dir.parent.parent                 # v3/
    out_dir = v3_root / "results" / f"viz_wavefront_step_{ts}"
    frames_dir = out_dir / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)

    # ── 场景 ──────────────────────────────────────────────────────────
    print(f"[viz-step] 生成随机场景 (seed={cfg.seed}, obs={cfg.n_obstacles}) ...")
    cspace = build_random_scene(cfg, rng)

    print("[viz-step] 扫描碰撞底图 ...")
    cmap_data, extent = cspace.scan_collision_map(cfg.collision_resolution)

    # ── Pre-grow: 用 wavefront 生长到 pre_grow 个 box ────────────────
    print(f"[viz-step] 预生长 {args.pre_grow} 个 box ...")
    grower = ForestGrower2D(cspace, cfg)
    grower._select_roots()
    grower._partition_subtrees()

    # 手动跑 wavefront 到 pre_grow 个 box
    bfs_queue = deque()
    for bid in list(grower.boxes.keys()):
        bfs_queue.append((bid, -1, -1))

    miss = 0
    while len(grower.boxes) < cfg.max_boxes and miss < cfg.max_consecutive_miss:
        if bfs_queue:
            box_id, _, _ = bfs_queue.popleft()
            if box_id not in grower.boxes:
                continue
            box = grower.boxes[box_id]
            seeds = grower._sample_boundary(box)
            for dim, side, q_seed in seeds:
                if len(grower.boxes) >= cfg.max_boxes:
                    break
                bid = grower._try_create_box(
                    q_seed, parent_box_id=box.box_id,
                    face_dim=dim, face_side=side,
                    root_id=box.root_id)
                if bid >= 0:
                    grower._update_adjacency(bid)
                    bfs_queue.append((bid, dim, 1 - side))
                    miss = 0
                else:
                    miss += 1
        else:
            break

    print(f"  预生长完成: {len(grower.boxes)} boxes")

    # ── 选择 parent box ──────────────────────────────────────────────
    if args.parent_idx >= 0:
        parent_id = list(grower.boxes.keys())[
            min(args.parent_idx, len(grower.boxes) - 1)]
    else:
        # 按 id 从大到小 (后加入 = 更靠前沿) 试探
        candidates = sorted(grower.boxes.keys(), reverse=True)
        parent_id = candidates[0]  # fallback
        for cand_id in candidates:
            cand_box = grower.boxes[cand_id]
            eps = cfg.boundary_epsilon
            q_lo_c, q_hi_c = cspace.q_lo, cspace.q_hi
            faces_c = []
            for d in range(2):
                if cand_box.lo[d] - eps >= q_lo_c[d]:
                    faces_c.append((d, 0))
                if cand_box.hi[d] + eps <= q_hi_c[d]:
                    faces_c.append((d, 1))
            if cand_box.expand_face_dim >= 0:
                faces_c = [(d, s) for d, s in faces_c
                           if not (d == cand_box.expand_face_dim
                                   and s == cand_box.expand_face_side)]
            if not faces_c:
                continue
            # 试探: 采样 seeds 看能否建出 child
            test_seeds = grower._sample_boundary(cand_box)
            can_grow = 0
            for _, _, qs in test_seeds:
                if not cspace.is_collision(qs) and not grower.ffb.is_occupied(qs):
                    can_grow += 1
            if can_grow >= 1:
                parent_id = cand_id
                break

    parent_box = grower.boxes[parent_id]
    print(f"  选中 parent box #{parent_id} "
          f"(vol={parent_box.volume:.4f}, root={parent_box.root_id})")

    # ── 拍一个快照: 当前 forest 状态 (不含即将生成的 children) ────────
    pre_boxes = {}
    for bid, b in grower.boxes.items():
        pre_boxes[bid] = BoxInfo(
            box_id=b.box_id, lo=b.lo.copy(), hi=b.hi.copy(),
            seed=b.seed.copy(), parent_box_id=b.parent_box_id,
            expand_face_dim=b.expand_face_dim,
            expand_face_side=b.expand_face_side,
            root_id=b.root_id)
    pre_adj = {k: set(v) for k, v in grower.adjacency.items()}

    # ── 计算有效面和 seeds ────────────────────────────────────────────
    faces = []
    eps = cfg.boundary_epsilon
    q_lo, q_hi = cspace.q_lo, cspace.q_hi
    for d in range(2):
        if parent_box.lo[d] - eps >= q_lo[d]:
            faces.append((d, 0))
        if parent_box.hi[d] + eps <= q_hi[d]:
            faces.append((d, 1))
    if parent_box.expand_face_dim >= 0:
        faces = [(d, s) for d, s in faces
                 if not (d == parent_box.expand_face_dim
                         and s == parent_box.expand_face_side)]

    seeds_list = grower._sample_boundary(parent_box)
    print(f"  有效面: {[FACE_LABELS.get(f, f) for f in faces]}")
    print(f"  采样 {len(seeds_list)} 个 seed")

    # ── 逐个 seed → child box ────────────────────────────────────────
    children: List[BoxInfo] = []
    seed_to_child: List[Optional[BoxInfo]] = []

    for si, (dim, side, q_seed) in enumerate(seeds_list):
        bid = grower._try_create_box(
            q_seed,
            parent_box_id=parent_box.box_id,
            face_dim=dim, face_side=side,
            root_id=parent_box.root_id)
        if bid >= 0:
            grower._update_adjacency(bid)
            child = grower.boxes[bid]
            children.append(child)
            seed_to_child.append(child)
            print(f"    seed s{si} → child #{bid} "
                  f"(vol={child.volume:.4f})")
        else:
            seed_to_child.append(None)
            print(f"    seed s{si} → FAIL (碰撞或已占用)")

    print(f"  共生成 {len(children)} / {len(seeds_list)} 个 child box")

    # ── 渲染帧 ────────────────────────────────────────────────────────
    dpi = cfg.dpi
    figsize = cfg.fig_size
    frame_idx = 0

    # Step 0: 已有 forest + parent 高亮
    print("[viz-step] 渲染 Step 0: 已有 forest + parent 高亮 ...")
    fig = render_step0(pre_boxes, pre_adj, cmap_data, extent, cfg,
                       grower.subtrees, parent_box, figsize)
    fig.savefig(frames_dir / f"step_{frame_idx:02d}.png",
                dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    frame_idx += 1

    # Step 1: 面 + seed 标注
    print("[viz-step] 渲染 Step 1: 边界面 + seed 点 ...")
    fig = render_step1(pre_boxes, pre_adj, cmap_data, extent, cfg,
                       grower.subtrees, parent_box, faces,
                       seeds_list, figsize)
    fig.savefig(frames_dir / f"step_{frame_idx:02d}.png",
                dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    frame_idx += 1

    # Step 2..N: 逐个 child 出现
    children_so_far = []
    for si, (dim, side, q_seed) in enumerate(seeds_list):
        child = seed_to_child[si]
        if child is None:
            continue
        children_so_far.append(child)
        print(f"[viz-step] 渲染 Step {frame_idx}: child #{child.box_id} ...")
        fig = render_step_child(
            pre_boxes, grower.adjacency, cmap_data, extent, cfg,
            grower.subtrees, parent_box, faces, seeds_list,
            children_so_far, child, si, figsize)
        fig.savefig(frames_dir / f"step_{frame_idx:02d}.png",
                    dpi=dpi, bbox_inches="tight")
        plt.close(fig)
        frame_idx += 1

    # Final: 总览
    print("[viz-step] 渲染 Final ...")
    fig = render_final(pre_boxes, grower.adjacency, cmap_data, extent, cfg,
                       grower.subtrees, parent_box, children, figsize)
    fig.savefig(frames_dir / f"step_{frame_idx:02d}.png",
                dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    frame_idx += 1

    # ── GIF ────────────────────────────────────────────────────────────
    gif_path = out_dir / "wavefront_step.gif"
    print("[viz-step] 合成 GIF ...")
    try:
        ok = compose_gif_wavefront(frames_dir, gif_path)
        if ok:
            print(f"  GIF: {gif_path}")
    except ImportError:
        print("  [warn] Pillow 未安装, 跳过 GIF")

    # ── 保存说明 ──────────────────────────────────────────────────────
    info = {
        "seed": cfg.seed,
        "pre_grow": args.pre_grow,
        "parent_box_id": parent_id,
        "parent_volume": round(parent_box.volume, 6),
        "parent_root_id": parent_box.root_id,
        "n_faces": len(faces),
        "faces": [FACE_LABELS.get(f, str(f)) for f in faces],
        "n_seeds": len(seeds_list),
        "n_children_created": len(children),
        "children": [
            {"box_id": ch.box_id, "volume": round(ch.volume, 6)}
            for ch in children
        ],
        "n_frames": frame_idx,
    }
    (out_dir / "info.json").write_text(
        json.dumps(info, indent=2, ensure_ascii=False), encoding="utf-8")

    md = [
        "# 波前扩展单步可视化",
        "",
        f"- seed: {cfg.seed}",
        f"- 预生长: {args.pre_grow} boxes",
        f"- Parent box: #{parent_id} (vol={parent_box.volume:.4f})",
        f"- 有效面: {len(faces)}",
        f"- 采样 seed: {len(seeds_list)}",
        f"- 成功生成 child: {len(children)}",
        "",
        "![wavefront_step](wavefront_step.gif)",
        "",
        "## 步骤",
        "| 帧 | 描述 |",
        "|-----|------|",
        "| Step 0 | 已有 forest, parent box 蓝色高亮 |",
        "| Step 1 | 标注有效面 (彩色线段+法线箭头) + seed 点 |",
    ]
    for si, child in enumerate(children):
        md.append(f"| Step {si+2} | Child #{child.box_id} "
                  f"从 seed 生长 (橙色) |")
    md.append(f"| Final | 所有 {len(children)} 个 child 就位, "
              f"邻接边画出 |")
    md.append("")
    (out_dir / "README.md").write_text("\n".join(md) + "\n", encoding="utf-8")

    # ── 输出汇总 ──────────────────────────────────────────────────────
    print(f"\n{'='*60}")
    print(f"输出目录: {out_dir}")
    print(f"  帧数: {frame_idx}")
    print(f"  parent: #{parent_id} (vol={parent_box.volume:.4f})")
    print(f"  children: {len(children)} / {len(seeds_list)} seeds")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()

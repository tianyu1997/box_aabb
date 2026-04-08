"""
src/viz/compare_expansion.py — 4 模式扩展对比可视化

在同一 2D 随机场景上分别运行:
  1. Wavefront (单线程)
  2. RRT (单线程)
  3. Wavefront + 多线程 round-robin
  4. Wavefront + adaptive min_edge (coarse→fine)

生成 2×2 并排 GIF 动画 → results/viz_compare_*/.

用法:
    cd safeboxforest/v3
    python -m src.viz.compare_expansion [--seed 42] [--max-boxes 200]
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .core import GrowVizConfig, CSpace2D
from .forest_grower_2d import ForestGrower2D, build_random_scene
from .render import ROOT_COLORS, compose_gif, _draw_boxes_on_ax, _build_title


# ═══════════════════════════════════════════════════════════════════════════
#  2×2 绘图
# ═══════════════════════════════════════════════════════════════════════════

def plot_4panel(snaps, cfgs, subtrees_list, labels,
                cmap_data, extent, frame_idx, seed):
    """绘制 2×2 对比帧."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    axf = axes.flat

    for i in range(4):
        snap = snaps[i]
        cfg = cfgs[i]
        st = subtrees_list[i]
        title = _build_title(snap, cfg, frame_idx)
        _draw_boxes_on_ax(axf[i], snap, cmap_data, extent, cfg, st,
                          title=f"{labels[i]}\n{title}",
                          markersize_s=7, markersize_g=10)

    axes[1, 0].set_xlabel("q0"); axes[1, 1].set_xlabel("q0")
    axes[0, 0].set_ylabel("q1"); axes[1, 0].set_ylabel("q1")

    fig.suptitle(f"ForestGrower Expansion Comparison  (seed={seed})",
                 fontsize=13, fontweight="bold", y=0.98)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    return fig


# ═══════════════════════════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="4 模式扩展对比 (Wavefront / RRT / 多线程 / Adaptive)")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--n-roots", type=int, default=3)
    parser.add_argument("--max-boxes", type=int, default=200)
    parser.add_argument("--snap-every", type=int, default=2)
    parser.add_argument("--obstacles", type=int, default=10)
    parser.add_argument("--dpi", type=int, default=80)
    parser.add_argument("--gif-ms", type=int, default=250)
    parser.add_argument("--no-endpoints", action="store_true")
    args = parser.parse_args()

    # ── 公共基础配置 ──────────────────────────────────────────────────
    def make_cfg(**overrides) -> GrowVizConfig:
        kw = dict(
            seed=args.seed,
            n_roots=args.n_roots,
            max_boxes=args.max_boxes,
            snapshot_every=args.snap_every,
            n_obstacles=args.obstacles,
            dpi=args.dpi,
            gif_frame_ms=args.gif_ms,
        )
        kw.update(overrides)
        c = GrowVizConfig(**kw)
        if args.no_endpoints:
            c.q_start = None
            c.q_goal = None
        return c

    # 4 种配置
    cfgs = [
        make_cfg(mode="wavefront"),                                         # ① 波前
        make_cfg(mode="rrt"),                                               # ② RRT
        make_cfg(mode="wavefront", n_threads=3),                            # ③ 多线程
        make_cfg(mode="wavefront", adaptive_min_edge=True,
                 coarse_min_edge=0.25, coarse_fraction=0.5),                # ④ 自适应
    ]
    labels = [
        "Wavefront (1T)",
        "RRT (1T)",
        "Wavefront (3T round-robin)",
        "Wavefront + Adaptive min_edge",
    ]

    rng = np.random.default_rng(args.seed)

    # ── 输出目录 ──────────────────────────────────────────────────────
    ts = time.strftime("%Y%m%d_%H%M%S")
    # 定位到 v3/results/
    viz_dir = Path(__file__).resolve().parent       # src/viz
    v3_root = viz_dir.parent.parent                 # v3/
    out_dir = v3_root / "results" / f"viz_compare_{ts}"
    frames_dir = out_dir / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)

    # ── 场景 (所有模式共享) ───────────────────────────────────────────
    print(f"[viz] building scene (seed={args.seed}, obs={args.obstacles}) ...")
    cspace = build_random_scene(cfgs[0], rng)

    (out_dir / "scene.json").write_text(json.dumps({
        "q_lo": list(cfgs[0].q_lo), "q_hi": list(cfgs[0].q_hi),
        "n_obstacles": len(cspace.obstacles),
        "obstacles": [{"lo": o.lo.tolist(), "hi": o.hi.tolist()}
                      for o in cspace.obstacles],
    }, indent=2), encoding="utf-8")

    # ── 碰撞底图 ─────────────────────────────────────────────────────
    print("[viz] scanning collision map ...")
    cmap_data, extent = cspace.scan_collision_map(cfgs[0].collision_resolution)

    # ── 4 种模式分别生长 ──────────────────────────────────────────────
    growers = []
    all_snaps = []
    timings = []
    for i, (cfg, label) in enumerate(zip(cfgs, labels)):
        print(f"[viz] growing {label} ...")
        t0 = time.time()
        g = ForestGrower2D(cspace, cfg)
        snaps = g.grow(snapshot_every=cfg.snapshot_every)
        dt = time.time() - t0
        growers.append(g)
        all_snaps.append(snaps)
        timings.append(dt)
        print(f"  -> {len(snaps)} snaps, {len(g.boxes)} boxes, {dt:.2f}s")

    # ── 对齐帧数 ─────────────────────────────────────────────────────
    n_frames = max(len(s) for s in all_snaps)
    for snaps in all_snaps:
        while len(snaps) < n_frames:
            snaps.append(snaps[-1])

    # ── 渲染帧 ────────────────────────────────────────────────────────
    print(f"[viz] rendering {n_frames} x 2x2 frames ...")
    subtrees_list = [g.subtrees for g in growers]
    for idx in range(n_frames):
        frame_snaps = [all_snaps[i][idx] for i in range(4)]
        fig = plot_4panel(frame_snaps, cfgs, subtrees_list, labels,
                          cmap_data, extent, idx, args.seed)
        fig.savefig(frames_dir / f"frame_{idx:04d}.png", dpi=args.dpi)
        plt.close(fig)
        if (idx + 1) % 5 == 0 or idx == n_frames - 1:
            print(f"  rendered {idx + 1}/{n_frames}")

    # ── GIF ───────────────────────────────────────────────────────────
    print("[viz] composing GIF ...")
    gif_path = out_dir / "compare_4panel.gif"
    ok = compose_gif(frames_dir, gif_path, args.gif_ms)

    # ── 最终图 ────────────────────────────────────────────────────────
    final_snaps = [all_snaps[i][-1] for i in range(4)]
    fig_final = plot_4panel(final_snaps, cfgs, subtrees_list, labels,
                            cmap_data, extent, n_frames - 1, args.seed)
    fig_final.savefig(out_dir / "final_compare.png", dpi=args.dpi)
    plt.close(fig_final)

    # ── Summary ───────────────────────────────────────────────────────
    summary = {
        "seed": args.seed,
        "n_roots": args.n_roots,
        "max_boxes": args.max_boxes,
        "n_obstacles": args.obstacles,
        "n_frames": n_frames,
    }
    for i, label in enumerate(labels):
        summary[f"mode_{i}"] = {
            "label": label,
            "n_boxes": len(growers[i].boxes),
            "elapsed_s": round(timings[i], 3),
            "n_coarse": growers[i].n_coarse_boxes,
            "n_fine": growers[i].n_fine_boxes,
        }
    (out_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")

    # ── README ────────────────────────────────────────────────────────
    md = [
        "# 4 模式扩展对比",
        "",
        f"- seed: {args.seed}",
        f"- n_roots: {args.n_roots}",
        f"- max_boxes: {args.max_boxes}",
        f"- n_obstacles: {args.obstacles}",
        "",
    ]
    for i, label in enumerate(labels):
        g = growers[i]
        md.append(f"## {label}")
        md.append(f"- boxes: {len(g.boxes)}, elapsed: {timings[i]:.2f}s")
        if g.n_coarse_boxes > 0:
            md.append(f"- coarse: {g.n_coarse_boxes}, fine: {g.n_fine_boxes}")
        md.append("")
    md += [
        "## 2x2 Comparison",
        "![compare](compare_4panel.gif)",
        "",
        "![final](final_compare.png)",
        "",
        "## Files",
        "- `compare_4panel.gif` — 2x2 对比动画",
        "- `final_compare.png` — 最终对比图",
        "- `scene.json` / `summary.json`",
        "- `frames/` — 逐帧 PNG",
    ]
    (out_dir / "README.md").write_text("\n".join(md) + "\n", encoding="utf-8")

    # ── 输出 ──────────────────────────────────────────────────────────
    print(f"\n{'='*60}")
    print(f"Output: {out_dir}")
    for i, label in enumerate(labels):
        g = growers[i]
        extra = ""
        if g.n_coarse_boxes > 0:
            extra = f" (C:{g.n_coarse_boxes} F:{g.n_fine_boxes})"
        print(f"  {label}: {len(g.boxes)} boxes, {timings[i]:.2f}s{extra}")
    print(f"  frames: {n_frames}")
    print(f"  gif:    {gif_path.name if ok else 'NOT GENERATED'}")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()

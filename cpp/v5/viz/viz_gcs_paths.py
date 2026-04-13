#!/usr/bin/env python3
"""
viz_gcs_paths.py — Drake Meshcat 可视化 GCS 优化路径

读取 gcs_results_v2.json, 在 Meshcat 中播放 GCS 优化后的路径动画。

Usage:
    conda activate sbf
    python viz/viz_gcs_paths.py result/gcs_results_v2.json
    python viz/viz_gcs_paths.py result/gcs_results_v2.json --pair 0
    python viz/viz_gcs_paths.py result/gcs_results_v2.json --static
    python viz/viz_gcs_paths.py result/gcs_results_v2.json --save gcs_viz.html
"""

import argparse
import json
import os
import sys
import time

import numpy as np

# Reuse infrastructure from viz_drake_paths
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _THIS_DIR)
from viz_drake_paths import (
    PAIR_NAMES,
    PAIR_COLORS,
    build_scene,
    visualize_trajectory,
    visualize_static,
)

from pydrake.geometry import StartMeshcat


def load_gcs_results(json_path, pair_filter=None):
    """加载 gcs_results_v2.json 中的 waypoints。"""
    with open(json_path, "r") as f:
        data = json.load(f)

    results = []
    for entry in data.get("results", []):
        if not entry.get("success", False):
            continue
        wp = entry.get("waypoints", [])
        if len(wp) < 2:
            continue
        pi = entry["pair_idx"]
        if pair_filter is not None and pi not in pair_filter:
            continue

        waypoints = np.array(wp, dtype=float)
        gcs_len = entry.get("gcs_path_length", 0.0)
        total_len = entry.get("total_path_length", gcs_len)
        proxy_s = entry.get("proxy_start_dist", 0.0)
        proxy_g = entry.get("proxy_goal_dist", 0.0)
        ratio = entry.get("improvement_ratio", 0.0)

        results.append({
            "pair_idx": pi,
            "label": entry.get("label", f"pair_{pi}"),
            "waypoints": waypoints,
            "path_length": total_len,
            "gcs_path_length": gcs_len,
            "proxy_start_dist": proxy_s,
            "proxy_goal_dist": proxy_g,
            "improvement_ratio": ratio,
            "corridor_hops_used": entry.get("corridor_hops_used", 0),
        })

    results.sort(key=lambda x: x["pair_idx"])
    return results


def main():
    parser = argparse.ArgumentParser(
        description="Drake Meshcat 可视化 GCS 优化路径")
    parser.add_argument("json_file", help="gcs_results_v2.json")
    parser.add_argument("--pair", type=int, default=None,
                        help="只显示指定 pair index (0-4)")
    parser.add_argument("--static", action="store_true",
                        help="静态模式 (末端轨迹 + 终点姿态)")
    parser.add_argument("--save", type=str, default=None,
                        help="导出交互式 HTML 文件")
    parser.add_argument("--speed", type=float, default=1.5,
                        help="动画播放速度 (默认: 1.5)")
    args = parser.parse_args()

    if not os.path.exists(args.json_file):
        print(f"错误: 找不到 {args.json_file}")
        sys.exit(1)

    pair_filter = [args.pair] if args.pair is not None else None
    paths = load_gcs_results(args.json_file, pair_filter=pair_filter)

    if not paths:
        print(f"没有找到成功的 GCS 路径 (pair={args.pair})")
        sys.exit(1)

    print(f"加载 {len(paths)} 条 GCS 路径:")
    for p in paths:
        wp = p["waypoints"]
        name = PAIR_NAMES[p["pair_idx"]] if p["pair_idx"] < len(PAIR_NAMES) else p["label"]
        print(f"  {name:8s}  {wp.shape[0]:3d} waypoints  "
              f"gcs_len={p['gcs_path_length']:.3f}  "
              f"total={p['path_length']:.3f}  "
              f"ratio={p['improvement_ratio']:.3f}  "
              f"hops={p['corridor_hops_used']}")

    # 启动 Meshcat
    meshcat = StartMeshcat()
    print(f"\nMeshcat URL: {meshcat.web_url()}")

    waypoints_list = [p["waypoints"] for p in paths]
    pair_indices = [p["pair_idx"] for p in paths]

    if args.static:
        build_scene(meshcat)
        time.sleep(0.5)
        visualize_static(meshcat, waypoints_list, pair_indices)
        print("\n静态可视化就绪 (GCS paths)")
    else:
        total_time = visualize_trajectory(
            meshcat, waypoints_list, pair_indices, speed=args.speed)
        print(f"\n动画就绪, 总时长 {total_time:.1f}s")

    if args.save:
        from datetime import datetime
        ts = datetime.now().strftime("_%Y%m%d_%H%M%S")
        base, ext = os.path.splitext(args.save)
        save_path = f"{base}{ts}{ext}" if ext else f"{args.save}{ts}"
        html = meshcat.StaticHtml()
        with open(save_path, "w") as f:
            f.write(html)
        print(f"已导出 HTML → {save_path}")

    print("\n按 Ctrl+C 退出")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n退出")


if __name__ == "__main__":
    main()

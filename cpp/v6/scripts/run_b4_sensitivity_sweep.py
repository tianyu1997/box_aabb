#!/usr/bin/env python3
"""B4: Sensitivity sweep for the magic constants registered in App.~C (A7).

Varies one parameter at a time around the default; reports SR, median build
time, median query time and median path length on the IIWA14 combined scene
(5 queries x N seeds).

Memory-safe protocol (see /memories/repo/heavy_experiment_crash_protocol.md):
  lect_no_cache=True, n_threads=2, max_boxes capped per-cell.
"""
from __future__ import annotations
import json, os, statistics, sys, time
from pathlib import Path

import numpy as np

from run_online_query_comparison import (
    DEFAULT_GOAL_BIAS,
    IIWA_CONFIGS,
    QUERY_PAIRS,
    SBF_BUILD_DIR,
    SBF_DATA_DIR,
    make_combined_obstacles,
)

ROOT = Path(__file__).resolve().parents[1]


def med(xs): return float(statistics.median(xs)) if xs else float("nan")


def _load_sbf5():
    if SBF_BUILD_DIR not in sys.path:
        sys.path.insert(0, SBF_BUILD_DIR)
    import _sbf5_cpp as sbf5
    return sbf5


def run_one(seeds: int, override: dict, timeout_ms: float = 30000.0):
    sbf5 = _load_sbf5()
    robot = sbf5.Robot.from_json(os.path.join(SBF_DATA_DIR, "iiwa14.json"))
    obstacles = make_combined_obstacles()
    seed_points = [IIWA_CONFIGS[k] for k in ["AS", "TS", "CS", "LB", "RB"]]

    builds, boxes, qtimes, plens = [], [], [], []
    succ = total = 0

    for s in range(seeds):
        cfg = sbf5.SBFPlannerConfig()
        # memory-safe defaults
        cfg.lect_no_cache = True
        cfg.grower.n_threads = 2
        cfg.grower.bridge_n_threads = 2
        cfg.grower.timeout_ms = timeout_ms
        cfg.grower.max_boxes = 5000
        cfg.grower.post_connect_extra_boxes = 1000
        cfg.grower.max_consecutive_miss = 2000
        cfg.grower.rrt_goal_bias = DEFAULT_GOAL_BIAS
        cfg.grower.rrt_step_ratio = 0.05
        cfg.grower.ffb_config.max_depth = 300
        cfg.coarsen.target_boxes = 300
        cfg.coarsen.score_threshold = 200
        cfg.grower.rng_seed = s * 1000 + 42
        # apply overrides
        for path, val in override.items():
            obj = cfg
            parts = path.split(".")
            for p in parts[:-1]:
                obj = getattr(obj, p)
            setattr(obj, parts[-1], val)

        planner = sbf5.SBFPlanner(robot, cfg)
        t0 = time.perf_counter()
        planner.build_coverage(obstacles, timeout_ms, seed_points)
        bt = time.perf_counter() - t0
        builds.append(bt); boxes.append(planner.n_boxes())

        for label, sn, gn in QUERY_PAIRS:
            total += 1
            t1 = time.perf_counter()
            r = planner.query(IIWA_CONFIGS[sn], IIWA_CONFIGS[gn])
            qt = time.perf_counter() - t1
            if bool(r.success):
                succ += 1
                qtimes.append(qt); plens.append(float(r.path_length))

    return {
        "override": override,
        "n_seeds": seeds,
        "sr": succ / max(total, 1),
        "build_med_s": med(builds),
        "boxes_med": med(boxes),
        "query_med_s": med(qtimes),
        "path_med_rad": med(plens),
        "n_success": succ, "n_total": total,
    }


def main():
    # Sensitivity cells: name -> (override dict, label for the table)
    cells = [
        ("default",         {},                                              "(default)"),
        ("max_boxes=2500",  {"grower.max_boxes": 2500},                      "max\\_boxes/tree=2500"),
        ("max_boxes=10000", {"grower.max_boxes": 10000},                     "max\\_boxes/tree=10000"),
        ("goal_bias=0.05",  {"grower.rrt_goal_bias": 0.05},                  "goal\\_bias=0.05"),
        ("goal_bias=0.30",  {"grower.rrt_goal_bias": 0.30},                  "goal\\_bias=0.30"),
        ("score_thr=50",    {"coarsen.score_threshold": 50.0},               "merge $s_{\\max}$=50"),
        ("score_thr=500",   {"coarsen.score_threshold": 500.0},              "merge $s_{\\max}$=500"),
    ]

    seeds = int(os.environ.get("B4_SEEDS", "3"))
    out_json = ROOT / "experiments" / "results_new" / f"B4_sensitivity_{time.strftime('%Y%m%d_%H%M%S')}.json"
    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_tex_en = ROOT / "doc" / "generated" / "tab_b4_sensitivity.tex"
    out_tex_zh = ROOT / "doc" / "generated" / "tab_b4_sensitivity_zh.tex"

    rows = []
    for name, override, label in cells:
        print(f"[{name}] running seeds={seeds} ...", flush=True)
        r = run_one(seeds, override)
        r["cell_name"] = name; r["label"] = label
        print(f"  sr={r['sr']:.0%} build_med={r['build_med_s']:.2f}s "
              f"query_med={r['query_med_s']:.3f}s path={r['path_med_rad']:.2f}", flush=True)
        rows.append(r)

    out_json.write_text(json.dumps({"meta": {"scene": "iiwa14_combined",
                                              "seeds": seeds},
                                     "rows": rows}, indent=2))
    print(f"\nwrote {out_json}")

    def emit(path, lang):
        if lang == "en":
            cap = ("Sensitivity sweep around the App.~\\ref{app:config} defaults on the "
                   "IIWA14 combined scene ($N=" + str(seeds) + "$ seeds, 5 queries). "
                   "Each row varies a single parameter; the (default) row is the canonical "
                   "memory-safe configuration.")
            head = "Cell & SR & build med.\\,(s) & boxes med. & query med.\\,(s) & path med.\\,(rad)"
        else:
            cap = ("围绕附录~\\ref{app:config} 默认值的灵敏度扫描（IIWA14 组合场景，"
                   f"$N={seeds}$ 种子，5 查询）。每行只改动单个参数；(default) 行为内存"
                   "安全协议下的规范配置。")
            head = "单元 & SR & 构建中位\\,(s) & 盒数中位 & 查询中位\\,(s) & 路径中位\\,(rad)"
        with path.open("w") as f:
            f.write("% Auto-generated by scripts/run_b4_sensitivity_sweep.py -- do not edit by hand.\n")
            f.write("\\begin{table}[t]\n\\centering\n")
            f.write(f"\\caption{{{cap}}}\n")
            f.write("\\label{tab:b4_sensitivity}\n")
            f.write("\\begin{tabular}{lcrrrr}\n\\toprule\n")
            f.write(head + "\\\\\n\\midrule\n")
            for r in rows:
                qm = r["query_med_s"]; pm = r["path_med_rad"]
                qs = "n/a" if (qm != qm) else f"{qm:.3f}"
                ps = "n/a" if (pm != pm) else f"{pm:.2f}"
                f.write(f"{r['label']} & {r['sr']*100:.0f}\\% & "
                        f"{r['build_med_s']:.2f} & {r['boxes_med']:.0f} & "
                        f"{qs} & {ps}\\\\\n")
            f.write("\\bottomrule\n\\end{tabular}\n\\end{table}\n")

    emit(out_tex_en, "en"); emit(out_tex_zh, "zh")
    print(f"wrote {out_tex_en}\nwrote {out_tex_zh}")


if __name__ == "__main__":
    main()

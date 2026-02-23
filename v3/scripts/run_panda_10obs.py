"""
Panda 10-obs comparison: SBF vs RRTConnect vs BIT*

Separately reports:
  - SBF forest build time (first query includes grow+coarsen+cache)
  - SBF query-only time   (solve step: adj + bridge + dijkstra/SOCP)
  - Additional reuse queries (2nd, 3rd query with same forest)
  - OMPL-RRTConnect / BIT* for comparison
"""
from __future__ import annotations
import json, sys, time, logging
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
_SRC = _ROOT / "src"
for p in (_ROOT, _SRC):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s %(levelname)s %(message)s")

from experiments.runner import (ExperimentRunner, load_scene_from_config,
                                create_planner, _sanitize_for_json,
                                _json_default)
from experiments.scenes import load_scenes, load_planners


# Additional query pairs for reuse measurement
_EXTRA_QUERIES = [
    {"start": [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.785],
     "goal":  [-1.5, 0.8, -1.0, -0.8, -1.5, 2.5, -1.0]},
    {"start": [1.0, -0.5, 0.3, -2.0, 0.8, 1.2, 0.5],
     "goal":  [-1.0, 0.5, -1.5, -1.0, -1.0, 2.0, -0.5]},
]


def main():
    scenes = load_scenes(["panda_10obs_moderate"])
    scene_cfg = scenes[0]

    # ── Standard comparison (SBF + OMPL) ──
    all_planners = load_planners(
        ["sbf_dijkstra", "ompl_rrt_connect", "ompl_bitstar"])
    config = {
        "name": "panda_10obs_comparison",
        "scenes": scenes,
        "planners": all_planners,
        "seeds": list(range(5)),
        "n_trials": 1,
        "timeout": 30.0,
    }
    runner = ExperimentRunner(config)
    total = len(scenes) * len(all_planners) * 5
    print("=== Panda 10-obs Comparison ===")
    print(f"  Scenes: {len(scenes)}, Planners: {len(all_planners)}, "
          f"Seeds: 5, Total: {total}")
    print()

    results = runner.run()

    # ── SBF reuse queries (separate forest build vs query) ──
    print("\n=== SBF Forest Reuse Queries ===")
    robot, scene, default_qps = load_scene_from_config(scene_cfg)
    extra_qps = [(np.array(q["start"], dtype=np.float64),
                  np.array(q["goal"], dtype=np.float64))
                 for q in _EXTRA_QUERIES]

    sbf_cfg = load_planners(["sbf_dijkstra"])[0]
    reuse_results = []

    for seed in range(5):
        planner = create_planner(sbf_cfg)
        params = {k: v for k, v in sbf_cfg.items() if k != "type"}
        params["seed"] = seed
        planner.setup(robot, scene, params)

        queries = list(default_qps) + list(extra_qps)
        for qi, (qs, qg) in enumerate(queries):
            t0 = time.perf_counter()
            result = planner.plan(qs, qg, timeout=30.0)
            wall = time.perf_counter() - t0
            rd = result.to_dict()
            rd["seed"] = seed
            rd["query_index"] = qi
            rd["is_first_query"] = (qi == 0)
            rd["wall_clock"] = wall
            reuse_results.append(rd)

            pt = rd.get("phase_times", {})
            grow_s = pt.get("grow", 0)
            solve_s = pt.get("solve", 0)
            ok = "OK" if rd["success"] else "FAIL"
            cost_str = f"{rd['cost']:.3f}" if rd["success"] else "N/A"
            print(f"  seed={seed} q{qi} {ok}  "
                  f"grow={grow_s:.3f}s  solve={solve_s:.3f}s  "
                  f"total={wall:.3f}s  cost={cost_str}")

    # ── Save combined results ──
    out_path = (_ROOT / "experiments" / "output" / "raw"
                / "panda_10obs_comparison.json")
    out_path.parent.mkdir(parents=True, exist_ok=True)

    save_data = {
        "experiment": "panda_10obs_comparison",
        "metadata": {
            "timestamp": time.strftime("%Y%m%d_%H%M%S"),
            "n_seeds": 5,
            "n_extra_queries": len(_EXTRA_QUERIES),
        },
        "n_trials": len(results.trials),
        "results": [t.to_dict() for t in results.trials],
        "reuse_results": _sanitize_for_json(reuse_results),
    }
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(save_data, f, indent=2, ensure_ascii=False,
                  default=_json_default)
    print(f"\nResults saved: {out_path}")

    # ── Summary table ──
    print("\n" + "=" * 72)
    print("SUMMARY")
    print("=" * 72)

    # SBF first-build vs query-only
    sbf_first = [r for r in reuse_results if r["is_first_query"]]
    sbf_reuse = [r for r in reuse_results if not r["is_first_query"]]

    if sbf_first:
        grow_times = [r["phase_times"]["grow"] for r in sbf_first
                      if r["success"]]
        solve_times = [r["phase_times"]["solve"] for r in sbf_first
                       if r["success"]]
        total_times = [r["wall_clock"] for r in sbf_first if r["success"]]
        print(f"\nSBF-Dijkstra (first query, includes forest build):")
        print(f"  Forest build : {np.mean(grow_times):.3f}s "
              f"(median {np.median(grow_times):.3f}s)")
        print(f"  Query (solve): {np.mean(solve_times):.3f}s "
              f"(median {np.median(solve_times):.3f}s)")
        print(f"  Total        : {np.mean(total_times):.3f}s "
              f"(median {np.median(total_times):.3f}s)")

    if sbf_reuse:
        ok_reuse = [r for r in sbf_reuse if r["success"]]
        if ok_reuse:
            solve_times = [r["phase_times"]["solve"] for r in ok_reuse]
            total_times = [r["wall_clock"] for r in ok_reuse]
            print(f"\nSBF-Dijkstra (reuse queries, forest already built):")
            print(f"  Query (solve): {np.mean(solve_times):.3f}s "
                  f"(median {np.median(solve_times):.3f}s)")
            print(f"  Success: {len(ok_reuse)}/{len(sbf_reuse)}")
        else:
            print(f"\nSBF-Dijkstra (reuse): all {len(sbf_reuse)} failed")

    # OMPL results
    for planner_name in ["OMPL-RRTConnect", "OMPL-BITstar"]:
        trials = [t for t in results.trials
                  if t.planner_name == planner_name]
        ok = [t for t in trials if t.result.get("success")]
        if ok:
            times = [t.result["planning_time"] for t in ok]
            fst = [t.result.get("first_solution_time", t.result["planning_time"])
                   for t in ok]
            costs = [t.result["cost"] for t in ok]
            print(f"\n{planner_name}:")
            print(f"  Plan time    : {np.mean(times):.3f}s "
                  f"(median {np.median(times):.3f}s)")
            print(f"  1st sol time : {np.mean(fst):.3f}s "
                  f"(median {np.median(fst):.3f}s)")
            print(f"  Path cost    : {np.mean(costs):.3f} "
                  f"(median {np.median(costs):.3f})")
            print(f"  Success: {len(ok)}/{len(trials)}")


if __name__ == "__main__":
    main()

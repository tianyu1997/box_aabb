#!/usr/bin/env python
"""快速端到端测试: 3 场景 × 4 方法 × 3 seeds × 1 trial + 摊还."""
import sys, time, logging, json, numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

logging.basicConfig(level=logging.WARNING,
                    format="%(asctime)s [%(levelname)s] %(message)s")
# 抑制 forest 详细输出
for m in ['forest', 'forest.hier_aabb_tree', 'forest.gcs', 'forest.builder']:
    logging.getLogger(m).setLevel(logging.ERROR)

from runner import create_planner, load_scene_from_config
from scenes import load_marcucci_scene

SCENES = ["shelves", "bins", "table"]
PLANNERS = [
    {"type": "SBF", "method": "gcs", "name": "SBF-GCS"},
    # IRIS-NP/C-IRIS 在此 Drake 环境下极慢 (~分钟级/seed), 仅在完整实验中启用
    # {"type": "IRIS-NP-GCS", "n_iris_seeds": 10, "name": "IRIS-NP-GCS"},
    # {"type": "C-IRIS-GCS", "n_regions": 10, "name": "C-IRIS-GCS"},
    {"type": "PRM", "n_samples": 2000, "k_neighbors": 15, "name": "PRM"},
]
N_SEEDS = 3
K_AMORT = 5  # 摊还查询次数

results = []

for scene_name in SCENES:
    scene_cfg = load_marcucci_scene(scene_name, robot="iiwa14")
    robot, scene, qps = load_scene_from_config(scene_cfg)
    n_obs = len(scene.get_obstacles())
    
    for pcfg in PLANNERS:
        pname = pcfg["name"]
        for seed in range(N_SEEDS):
            qi = seed % len(qps)
            q_s, q_g = qps[qi]
            
            planner = create_planner(pcfg)
            planner.setup(robot, scene, {"seed": seed})
            
            # 首次查询
            t0 = time.perf_counter()
            r = planner.plan(q_s, q_g, timeout=60)
            dt = time.perf_counter() - t0
            
            plen = float("inf")
            if r.path is not None and len(r.path) > 1:
                plen = float(np.sum(np.linalg.norm(
                    np.diff(r.path, axis=0), axis=1)))
            
            row = {
                "scene": scene_name, "planner": pname, "seed": seed,
                "success": r.success, "first_time": round(dt, 4),
                "path_L2": round(plen, 4) if plen < 1e6 else None,
                "n_obs": n_obs,
            }
            
            # 摊还查询 (仅 SBF, 复用 forest)
            if pcfg["type"] == "SBF" and r.success:
                t_amor = 0.0
                n_ok = 0
                for k in range(K_AMORT):
                    qk = (seed + k + 1) % len(qps)
                    qs2, qg2 = qps[qk]
                    t1 = time.perf_counter()
                    r2 = planner.plan(qs2, qg2, timeout=30)
                    t_amor += time.perf_counter() - t1
                    if r2.success:
                        n_ok += 1
                row["amort_time"] = round(t_amor / K_AMORT, 4)
                row["amort_sr"] = round(n_ok / K_AMORT, 2)
            
            results.append(row)
            status = "OK" if r.success else "FAIL"
            print(f"  {scene_name:8s} | {pname:14s} | seed={seed} | "
                  f"{status} | {dt:.3f}s | L2={plen:.3f}")

# 汇总
print("\n" + "=" * 70)
print("SUMMARY")
print("=" * 70)
print(f"{'Scene':10s} {'Planner':14s} {'SR':>5s} {'t_med':>7s} "
      f"{'L2_med':>7s} {'amort':>7s}")
print("-" * 70)

from collections import defaultdict
groups = defaultdict(list)
for r in results:
    groups[(r["scene"], r["planner"])].append(r)

for (sc, pl), rows in sorted(groups.items()):
    n = len(rows)
    sr = sum(1 for r in rows if r["success"]) / n * 100
    times = [r["first_time"] for r in rows if r["success"]]
    t_med = np.median(times) if times else float("nan")
    lens = [r["path_L2"] for r in rows if r["path_L2"] is not None]
    l_med = np.median(lens) if lens else float("nan")
    amorts = [r.get("amort_time") for r in rows if r.get("amort_time") is not None]
    a_med = np.median(amorts) if amorts else float("nan")
    print(f"{sc:10s} {pl:14s} {sr:4.0f}% {t_med:7.3f} "
          f"{l_med:7.3f} {a_med:7.3f}")

# 保存原始数据
out_path = Path("output/raw/quick_test.json")
out_path.parent.mkdir(parents=True, exist_ok=True)
with open(out_path, "w") as f:
    json.dump(results, f, indent=2, default=str)
print(f"\nSaved to {out_path}")

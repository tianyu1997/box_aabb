#!/usr/bin/env python3
"""快速查看 iiwa 碰撞的具体连杆"""
import os, sys, pickle
import numpy as np

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_V4_ROOT = os.path.dirname(_THIS_DIR)
_PROJ_ROOT = os.path.dirname(_V4_ROOT)
_GCS_ROOT = os.path.join(_PROJ_ROOT, "gcs-science-robotics")

sys.path.insert(0, os.path.join(_V4_ROOT, "experiments"))
sys.path.insert(0, _THIS_DIR)

from verify_collision_cause import load_data, build_drake_checker, classify_collision, point_in_any_box

seed = 0
paths, forest = load_data(seed)
lo, hi = forest["intervals_lo"], forest["intervals_hi"]
drake = build_drake_checker()

PAIR_NAMES = ["AS→TS", "TS→CS", "CS→LB", "LB→RB", "RB→AS"]

# 收集所有 iiwa 碰撞
iiwa_contacts = {}
for entry in paths:
    pi = entry["pair_idx"]
    wp = entry.get("path_waypoints")
    if wp is None:
        continue
    
    for i in range(len(wp)):
        in_box = point_in_any_box(wp[i], lo, hi)
        col = classify_collision(*drake, wp[i])
        for c in col["iiwa_collisions"]:
            key = f"{c['nameA']} ↔ {c['nameB']}"
            if key not in iiwa_contacts:
                iiwa_contacts[key] = []
            iiwa_contacts[key].append({
                "pair": PAIR_NAMES[pi], "wp": i, "dist": c["distance"],
                "in_box": in_box
            })

print("IIWA 碰撞明细 (waypoints only):")
for key, entries in sorted(iiwa_contacts.items()):
    in_box_cnt = sum(1 for e in entries if e["in_box"])
    print(f"  {key}:")
    for e in entries:
        box_str = "✓box" if e["in_box"] else "✗out"
        print(f"    [{e['pair']}] wp{e['wp']}  dist={e['dist']:.4f}  {box_str}")

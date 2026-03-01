import pickle
import numpy as np
from experiments.marcucci_scenes import get_iiwa14_seed_points

reg_path = "/home/tian/桌面/box_aabb/gcs-science-robotics/data/prm_comparison/IRIS.reg"
with open(reg_path, "rb") as f:
    orig_regions_dict = pickle.load(f)

pts = get_iiwa14_seed_points()

print("Checking which points are in which notebook regions:")
for name, pt in pts.items():
    in_regs = []
    for reg_name, reg in orig_regions_dict.items():
        if reg.PointInSet(pt):
            in_regs.append(reg_name)
    print(f"Point {name} is in: {in_regs}")

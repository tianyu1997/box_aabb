import sys
import numpy as np
import pickle
sys.path.insert(0, "/home/tian/桌面/box_aabb/gcs-science-robotics")
from reproduction.prm_comparison.helpers import InverseKinematics

milestones = {
    "AS": [[0.75, 0, 0.9], [0, -np.pi, -np.pi / 2]],
    "TS": [[0.75, 0, 0.67], [0, -np.pi, -np.pi / 2]],
    "CS": [[0.75, 0, 0.41], [0, -np.pi, -np.pi / 2]],
    "LB": [[0.0, 0.6, 0.22], [np.pi / 2, np.pi, 0]],
    "RB": [[0.0, -0.6, 0.22], [np.pi / 2, np.pi, np.pi]]
}
additional_seed_points = {
    "C": np.array([0, 0.2, 0, -2.09, 0, -0.3, np.pi / 2]),
    "L": np.array([0.8, 0.7, 0, -1.6, 0, 0, np.pi / 2]),
    "R": np.array([-0.8, 0.7, 0, -1.6, 0, 0, np.pi / 2])
}
q0 = [0, 0.3, 0, -1.8, 0, 1, 1.57]
milestone_configurations = {
    name: InverseKinematics(q0, trans, rot)
    for name, (trans, rot) in milestones.items()
}
seed_points = {**milestone_configurations, **additional_seed_points}

with open("/home/tian/桌面/box_aabb/gcs-science-robotics/data/prm_comparison/IRIS.reg", "rb") as f:
    orig_regions_dict = pickle.load(f)

for name, pt in seed_points.items():
    in_regs = []
    for reg_name, reg in orig_regions_dict.items():
        if reg.PointInSet(pt):
            in_regs.append(reg_name)
    print(f"Point {name} is in: {in_regs}")

with open("nb_seed_points.pkl", "wb") as f:
    pickle.dump(seed_points, f)

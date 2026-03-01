import json

with open("/home/tian/桌面/box_aabb/gcs-science-robotics/reproduction/prm_comparison/prm_comparison.ipynb", "r") as f:
    nb = json.load(f)

for cell in nb["cells"]:
    if cell["cell_type"] == "code":
        src = "".join(cell["source"])
        if "offset =" in src and "points = {" in src:
            print("FOUND IT:")
            print(src)
            break

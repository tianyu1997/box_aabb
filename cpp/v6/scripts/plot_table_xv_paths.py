#!/usr/bin/env python3
"""TABLE XV visualisation: SBF vs SBF→GCS end-effector paths
on the IIWA14 combined scene (shelf + bin + table).

Reads:
  /tmp/exp2_paths.json            (SBF cpp paths + obstacles)
  /tmp/gcs_corridor_v2/gcs_pipeline_results.json  (GCS waypoints)

Writes:
  cpp/v6/doc/figures/fig_table_xv_paths.png
"""
import json, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

DH = [
    (0.0,                0.0, 0.36),
    (-1.5707963267948966, 0.0, 0.0),
    (1.5707963267948966,  0.0, 0.42),
    (1.5707963267948966,  0.0, 0.0),
    (-1.5707963267948966, 0.0, 0.4),
    (-1.5707963267948966, 0.0, 0.0),
    (1.5707963267948966,  0.0, 0.081),
]


def fk_ee(q):
    T = np.eye(4)
    for (alpha, a, d), th in zip(DH, q):
        ct, st, ca, sa = np.cos(th), np.sin(th), np.cos(alpha), np.sin(alpha)
        A = np.array([
            [ct,    -st * ca,  st * sa, a * ct],
            [st,     ct * ca, -ct * sa, a * st],
            [0.0,    sa,       ca,      d],
            [0.0,    0.0,      0.0,     1.0],
        ])
        T = T @ A
    return T[:3, 3]


def densify(qs, n=80):
    qs = np.asarray(qs)
    if len(qs) < 2:
        return qs
    seg = np.linalg.norm(np.diff(qs, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] == 0:
        return qs
    u = np.linspace(0, s[-1], n)
    out = np.empty((n, qs.shape[1]))
    for j in range(qs.shape[1]):
        out[:, j] = np.interp(u, s, qs[:, j])
    return out


def ee_curve(qs, n=80):
    return np.array([fk_ee(q) for q in densify(qs, n)])


def draw_box(ax, lo, hi, color, alpha=0.18):
    x = [lo[0], hi[0]]; y = [lo[1], hi[1]]; z = [lo[2], hi[2]]
    verts = [
        [(x[0], y[0], z[0]), (x[1], y[0], z[0]), (x[1], y[1], z[0]), (x[0], y[1], z[0])],
        [(x[0], y[0], z[1]), (x[1], y[0], z[1]), (x[1], y[1], z[1]), (x[0], y[1], z[1])],
        [(x[0], y[0], z[0]), (x[1], y[0], z[0]), (x[1], y[0], z[1]), (x[0], y[0], z[1])],
        [(x[0], y[1], z[0]), (x[1], y[1], z[0]), (x[1], y[1], z[1]), (x[0], y[1], z[1])],
        [(x[0], y[0], z[0]), (x[0], y[1], z[0]), (x[0], y[1], z[1]), (x[0], y[0], z[1])],
        [(x[1], y[0], z[0]), (x[1], y[1], z[0]), (x[1], y[1], z[1]), (x[1], y[0], z[1])],
    ]
    pc = Poly3DCollection(verts, facecolor=color, edgecolor="0.35",
                          linewidths=0.4, alpha=alpha)
    ax.add_collection3d(pc)


PAIRS = ["AS->TS", "TS->CS", "CS->LB", "LB->RB", "RB->AS"]
SEED_USED = 4  # gcs_pipeline keeps the last seed per pair


def main():
    exp2 = json.load(open("/tmp/exp2_paths.json"))
    gcs = json.load(open("/tmp/gcs_corridor_v2/gcs_pipeline_results.json"))

    sbf_paths = {}
    for p in exp2["paths"]:
        if p.get("seed") == SEED_USED and p.get("success"):
            sbf_paths[p["label"]] = p["waypoints"]
    gcs_paths = {r["label"]: r.get("waypoints") for r in gcs["results"]
                 if r.get("success") and r.get("waypoints")}

    obs = exp2["obstacles"]

    fig = plt.figure(figsize=(7.0, 3.2), dpi=180)
    gs = fig.add_gridspec(1, 2, wspace=0.05)
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd"]

    panels = [
        ("(a) SBF Dijkstra + PathOpt", sbf_paths,
         {"AS->TS": 2.053, "TS->CS": 2.910, "CS->LB": 3.567,
          "LB->RB": 3.892, "RB->AS": 1.891}),
        (r"(b) SBF$\to$GCS corridor", gcs_paths,
         {"AS->TS": 1.592, "TS->CS": 1.208, "CS->LB": 2.532,
          "LB->RB": 2.950, "RB->AS": 2.032}),
    ]

    for k, (title, paths, lengths) in enumerate(panels):
        ax = fig.add_subplot(gs[0, k], projection="3d")
        for o in obs:
            draw_box(ax, o["lo"], o["hi"], "0.55", alpha=0.18)
        for i, lab in enumerate(PAIRS):
            wps = paths.get(lab)
            if not wps:
                continue
            ee = ee_curve(wps, n=120)
            ax.plot(ee[:, 0], ee[:, 1], ee[:, 2],
                    color=colors[i], lw=1.6, label=f"{lab} ({lengths[lab]:.2f})")
            ax.scatter(*ee[0], color=colors[i], marker="o", s=18, zorder=5)
            ax.scatter(*ee[-1], color=colors[i], marker="^", s=22, zorder=5)
        ax.set_title(title, fontsize=8.5)
        ax.set_xlabel("x [m]", fontsize=7, labelpad=-6)
        ax.set_ylabel("y [m]", fontsize=7, labelpad=-6)
        ax.set_zlabel("z [m]", fontsize=7, labelpad=-6)
        ax.tick_params(labelsize=6, pad=-2)
        ax.set_box_aspect((1, 1, 0.85))
        ax.view_init(elev=22, azim=-58)
        ax.set_xlim(-0.6, 1.0); ax.set_ylim(-0.7, 0.7); ax.set_zlim(0.0, 1.2)
        ax.legend(loc="upper left", bbox_to_anchor=(0.0, 1.0),
                  fontsize=5.8, frameon=False,
                  title="path (rad)", title_fontsize=6.3,
                  handlelength=1.4, labelspacing=0.25, borderaxespad=0.2)

    fig.suptitle("TABLE XV: end-effector trajectories "
                 "(IIWA14 combined scene, seed=4)", fontsize=8.8, y=0.98)

    out = os.path.join(os.path.dirname(__file__),
                       "..", "doc", "figures", "fig_table_xv_paths.png")
    out = os.path.abspath(out)
    fig.tight_layout(rect=[0, 0, 1.0, 0.94])
    fig.savefig(out, bbox_inches="tight", dpi=220)
    print("wrote", out)


if __name__ == "__main__":
    main()

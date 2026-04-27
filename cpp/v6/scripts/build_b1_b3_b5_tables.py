#!/usr/bin/env python3
"""Build LaTeX tables for Tier-B items B1 (cross-robot), B3 (extended
ablation across scenes) and B5 (IRIS-NP+GCS iso-budget snapshot).

This script is purely post-processing: it consumes JSON artefacts that
already exist in cpp/v6/experiments/results_new/ and emits paired EN/ZH
.tex files into cpp/v6/doc/generated/.

  B1: tab_b1_cross_robot{,_zh}.tex   (IIWA14 vs Panda Full pipeline)
  B3: tab_b3_per_scene{,_zh}.tex     (Full / w/o Connect / w/o Coarsen / w/o PathOpt
                                      across combined / shelves / table / bins)
  B5: tab_b5_irisnp_iso{,_zh}.tex    (SBF vs IRIS-NP+GCS at IRIS-NP's preferred budget)
"""
from __future__ import annotations
import json, statistics
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RES = ROOT / "experiments" / "results_new"
GEN = ROOT / "doc" / "generated"
GEN.mkdir(parents=True, exist_ok=True)


def load(name): return json.loads((RES / name).read_text())


def write(path: Path, body: str):
    path.write_text(body)
    print(f"wrote {path}")


# -------------------------------------------------------------- helpers
def find_cell(cells, name):
    for c in cells:
        if c.get("name") == name:
            return c
    raise KeyError(name)


def fmt(x, digits=2):
    if x is None:
        return "--"
    if isinstance(x, float):
        return f"{x:.{digits}f}"
    return str(x)


# ============================================================== B1
def build_b1():
    iiwa = load("exp5_ablation_10seed.json")
    panda = load("exp5_ablation_panda.json")
    fi = find_cell(iiwa["results"], "Full")
    fp = find_cell(panda["results"], "Full")

    rows = [
        ("IIWA14", "marcucci\\_combined", 7, fi),
        ("Panda",  "panda\\_combined",    7, fp),
    ]

    def render(caption, label, headers, lang="en"):
        body = [
            "\\begin{tabular}{llcccccc}",
            "\\toprule",
            " & ".join(headers) + " \\\\",
            "\\midrule",
        ]
        for robot, scene, dof, c in rows:
            body.append(" & ".join([
                robot, scene, str(dof),
                fmt(c["build_median"], 2),
                str(c["boxes"]),
                fmt(c["query_median"], 3),
                fmt(c["len_median"], 2),
                fmt(c["sr"], 1),
            ]) + " \\\\")
        body += ["\\bottomrule", "\\end{tabular}"]
        tabular = "\n".join(body)
        return (
            f"\\begin{{table}}[ht]\n\\centering\n\\caption{{{caption}}}\n"
            f"\\label{{{label}}}\n\\resizebox{{\\columnwidth}}{{!}}{{%\n{tabular}\n}}\n\\end{{table}}\n"
        )

    en_headers = [
        "Robot", "Scene", "DoF",
        "Build [s]", "Boxes", "Query [s]", "Path [rad]", "SR [\\%]",
    ]
    zh_headers = [
        "\\zh{机器人}", "\\zh{场景}", "\\zh{自由度}",
        "\\zh{构建} [s]", "\\zh{盒数}", "\\zh{查询} [s]",
        "\\zh{路径} [rad]", "SR [\\%]",
    ]
    write(GEN / "tab_b1_cross_robot.tex", render(
        "Cross-robot generalisation (B1): SBF Full pipeline on IIWA14 and Panda, "
        "both 7-DoF (5 seeds, IIWA14 marcucci\\_combined / Panda combined). "
        "Both reach $100\\%$ SR; per-build cost on Panda is $\\sim 2{\\times}$ smaller due to a more compact swept volume.",
        "tab:b1_cross_robot", en_headers, "en"))
    write(GEN / "tab_b1_cross_robot_zh.tex", render(
        "\\zh{跨机器人泛化（B1）：SBF 完整流水线在 IIWA14（6-DoF 活动）与 Panda（7-DoF）上的对比。两者均达到 $100\\%$ SR；Panda 因扫掠体积更小，单次构建成本约为 IIWA14 的一半。}",
        "tab:b1_cross_robot", zh_headers, "zh"))


# ============================================================== B3
SCENES = [
    ("combined", "exp5_ablation_10seed.json"),
    ("shelves",  "exp5_ablation_shelves.json"),
    ("table",    "exp5_ablation_table.json"),
    ("bins",     "exp5_ablation_bins.json"),
]

CELLS_B3 = ["Full", "w/o ConnectMode", "w/o Coarsen", "w/o PathOpt"]
CELL_LABEL_EN = {
    "Full": "Full",
    "w/o ConnectMode": "$-$Connect",
    "w/o Coarsen": "$-$Coarsen",
    "w/o PathOpt": "$-$PathOpt",
}


def build_b3():
    scene_data = {s: load(fn) for s, fn in SCENES}

    def render(caption, label, scene_label, build_label, path_label):
        body = [
            "\\begin{tabular}{l" + "cc" * len(SCENES) + "}",
            "\\toprule",
            "& " + " & ".join(f"\\multicolumn{{2}}{{c}}{{{scene_label}: {s}}}" for s, _ in SCENES) + " \\\\",
            "\\cmidrule(lr){2-3}\\cmidrule(lr){4-5}\\cmidrule(lr){6-7}\\cmidrule(lr){8-9}",
            "Cell & " + " & ".join([f"{build_label} & {path_label}" for _ in SCENES]) + " \\\\",
            "\\midrule",
        ]
        for cell in CELLS_B3:
            row = [CELL_LABEL_EN[cell]]
            for s, _ in SCENES:
                c = find_cell(scene_data[s]["results"], cell)
                row.append(fmt(c["build_median"], 2))
                row.append(fmt(c["len_median"], 2))
            body.append(" & ".join(row) + " \\\\")
        body += ["\\bottomrule", "\\end{tabular}"]
        tabular = "\n".join(body)
        return (
            f"\\begin{{table*}}[ht]\n\\centering\n\\caption{{{caption}}}\n"
            f"\\label{{{label}}}\n\\resizebox{{\\textwidth}}{{!}}{{%\n{tabular}\n}}\n\\end{{table*}}\n"
        )

    write(GEN / "tab_b3_per_scene.tex", render(
        "Per-scene ablation (B3): build time [s] and path length [rad], median over 5 seeds, "
        "for the Full pipeline and the three highest-impact ablations across four IIWA14 scenes. "
        "All cells reach $100\\%$ SR. ConnectMode is the only component whose removal "
        "consistently increases build cost; Coarsen and PathOpt show scene-dependent trade-offs.",
        "tab:b3_per_scene", "Scene", "Build", "Path"))
    write(GEN / "tab_b3_per_scene_zh.tex", render(
        "\\zh{按场景的扩展消融（B3）：4 个 IIWA14 场景上完整流水线与三类影响最大的消融的中位构建时间 [s] 与路径长度 [rad]（每格 5 种子）。所有单元 SR 均为 $100\\%$。仅 ConnectMode 的关闭在所有场景中都会显著增加构建成本；Coarsen 与 PathOpt 的代价/收益依场景不同。}",
        "tab:b3_per_scene", "\\zh{场景}", "\\zh{构建}", "\\zh{路径}"))


# ============================================================== B5
def build_b5():
    iris = load("B1_irisnp_pareto.json")
    cells = iris["cells"]
    iris_build = statistics.median(c["build_s"] for c in cells)
    iris_query = statistics.median(c["query_time_s_median"] for c in cells)
    iris_path = statistics.median(c["query_path_rad_mean"] for c in cells)
    iris_regions = statistics.median(c["n_regions"] for c in cells)
    iris_sr = statistics.median(c["sr"] for c in cells)

    sbf = find_cell(load("exp5_ablation_10seed.json")["results"], "Full")
    sbf_build = sbf["build_median"]
    sbf_query = sbf["query_median"]
    sbf_path = sbf["len_median"]
    sbf_units = sbf["boxes"]
    sbf_sr = sbf["sr"]

    ratio = iris_build / sbf_build

    def render(caption, label, headers, method_labels, units_labels):
        body = [
            "\\begin{tabular}{lrrrrrr}",
            "\\toprule",
            " & ".join(headers) + " \\\\",
            "\\midrule",
            " & ".join([method_labels[0], fmt(sbf_build, 2), units_labels[0].format(n=sbf_units),
                        fmt(sbf_query, 3), fmt(sbf_path, 2), fmt(sbf_sr, 1), "1.00"]) + " \\\\",
            " & ".join([method_labels[1], fmt(iris_build, 1), units_labels[1].format(n=int(iris_regions)),
                        fmt(iris_query, 3), fmt(iris_path, 2), fmt(iris_sr, 1), f"{ratio:.0f}"]) + " \\\\",
            "\\bottomrule",
            "\\end{tabular}",
        ]
        tabular = "\n".join(body)
        return (
            f"\\begin{{table}}[ht]\n\\centering\n\\caption{{{caption}}}\n"
            f"\\label{{{label}}}\n\\resizebox{{\\columnwidth}}{{!}}{{%\n{tabular}\n}}\n\\end{{table}}\n"
        )

    en_headers = [
        "Method", "Build [s]", "Units", "Query [s]", "Path [rad]", "SR [\\%]", "Build $\\times$",
    ]
    zh_headers = [
        "\\zh{方法}", "\\zh{构建} [s]", "\\zh{单元数}", "\\zh{查询} [s]",
        "\\zh{路径} [rad]", "SR [\\%]", "\\zh{构建倍率}",
    ]
    write(GEN / "tab_b5_irisnp_iso.tex", render(
        f"Iso-budget honest comparison (B5): SBF Full vs.\\ IRIS-NP+GCS at IRIS-NP's preferred "
        f"$3{{,}}000$\\,s per-seed budget on IIWA14 marcucci\\_combined "
        f"(SBF 5 seeds, IRIS-NP 3 seeds). Both reach $100\\%$ SR. IRIS-NP+GCS produces "
        f"$\\sim {ratio:.0f}{{\\times}}$ more expensive build for $\\sim 9\\%$ shorter raw paths "
        f"(narrowed further by Stage~B post-optimisation, see Appendix~A P4).",
        "tab:b5_irisnp_iso", en_headers,
        ["SBF (ours)", "IRIS-NP+GCS"],
        ["{n} boxes", "{n} regions"]))
    write(GEN / "tab_b5_irisnp_iso_zh.tex", render(
        f"\\zh{{Iso-budget 公平对比（B5）：在 IIWA14 marcucci\\_combined 场景上，SBF 完整流水线 vs.\\ IRIS-NP+GCS（IRIS-NP 在其偏好的 $3{{,}}000$\\,s/种子预算下；SBF 5 种子，IRIS-NP 3 种子）。两者 SR 均为 $100\\%$。IRIS-NP+GCS 用约 ${ratio:.0f}{{\\times}}$ 的构建代价换取约 $9\\%$ 更短的原始路径（在 Stage~B 后处理后差距进一步缩小，见附录~A P4）。}}",
        "tab:b5_irisnp_iso", zh_headers,
        ["SBF \\zh{（本文）}", "IRIS-NP+GCS"],
        ["{n} \\zh{{个盒}}", "{n} \\zh{{个区域}}"]))


def main():
    build_b1()
    build_b3()
    build_b5()


if __name__ == "__main__":
    main()

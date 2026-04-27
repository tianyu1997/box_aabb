#!/usr/bin/env python3
"""build_paper_tables.py — emit booktabs LaTeX tables for the paper.

Reads the nightly JSON outputs in ``cpp/v7/experiments/results_nightly/full/``
and writes the following files into ``cpp/v7/doc/generated/``:

  * ``tab_main.tex``     — exp_main success-rate / timing per scene
  * ``tab_threads.tex``  — exp_threads strong-scaling table
  * ``tab_pathopt.tex``  — pathopt_steps ablation
  * ``tab_compare.tex``  — SBF vs OMPL vs Drake-GCS
    * ``tab_marcucci.tex`` — Marcucci IIWA14 cabinet (build + per-query)
        * ``tab_query.tex`` — v6-compatible cached online Marcucci queries
        * ``tab_dof_scaling.tex`` — DoF-scaling summary table
  * ``macros.tex``       — \\newcommand{...} numerical macros

All numbers are pulled from the JSON; nothing is hand-typed.
"""
from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CPP_ROOT = ROOT.parent
V6_ROOT = CPP_ROOT / "v6"
SRC  = ROOT / "experiments" / "results_nightly" / "full"
PAPER = ROOT / "experiments" / "results_paper"
OUT  = ROOT / "doc" / "generated"
V6_EXPERIMENTS = V6_ROOT / "experiments"
V6_OUT = V6_ROOT / "doc" / "paper" / "generated"
OUT.mkdir(parents=True, exist_ok=True)
V6_OUT.mkdir(parents=True, exist_ok=True)


def load(name: str) -> dict:
    return json.loads((SRC / name).read_text())


def load_paper(name: str) -> dict:
    return json.loads((PAPER / name).read_text())


def result_path(
    base_dir: Path,
    name: str,
    recursive: bool = False,
    required_params: dict[str, object] | None = None,
) -> Path | None:
    direct = base_dir / name
    if direct.exists():
        try:
            direct_payload = json.loads(direct.read_text())
        except Exception:
            direct_payload = None
        if direct_payload is not None and _payload_matches_required_params(direct_payload, required_params):
            return direct
    if not recursive:
        return None
    matches = list(base_dir.rglob(name))
    if not matches:
        return None

    def rank(path: Path) -> tuple[int, int, int, int, int, int, float, str]:
        try:
            payload = json.loads(path.read_text())
        except Exception:
            return (0, 0, 0, 0, 0, 0, path.stat().st_mtime, str(path))
        summary = payload.get("summary") if isinstance(payload.get("summary"), dict) else {}
        params = payload.get("params") if isinstance(payload.get("params"), dict) else {}
        logical_threads = int(params.get("logical_threads", 0) or 0)
        cpu_affinity = params.get("cpu_affinity") if isinstance(params.get("cpu_affinity"), list) else []
        return (
            1 if not payload.get("quick", False) else 0,
            int(payload.get("seeds", 0) or 0),
            logical_threads,
            len(cpu_affinity),
            int(summary.get("n_queries", 0) or 0),
            int(summary.get("n_success", 0) or 0),
            path.stat().st_mtime,
            str(path),
        )

    filtered_matches = []
    for path in matches:
        try:
            payload = json.loads(path.read_text())
        except Exception:
            payload = None
        if payload is not None and _payload_matches_required_params(payload, required_params):
            filtered_matches.append(path)

    if filtered_matches:
        matches = filtered_matches
    return max(matches, key=rank)


def _param_matches(actual: object, expected: object) -> bool:
    if isinstance(actual, (int, float)) and isinstance(expected, (int, float)):
        return abs(float(actual) - float(expected)) <= 1e-9
    return actual == expected


def _payload_matches_required_params(payload: dict, required_params: dict[str, object] | None) -> bool:
    if not required_params:
        return True
    params = payload.get("params")
    if not isinstance(params, dict):
        return False
    return all(_param_matches(params.get(key), value) for key, value in required_params.items())


def paper_result_path(
    name: str,
    recursive: bool = False,
    required_params: dict[str, object] | None = None,
) -> Path | None:
    return result_path(PAPER, name, recursive=recursive, required_params=required_params)


def v6_result_path(
    name: str,
    recursive: bool = False,
    required_params: dict[str, object] | None = None,
) -> Path | None:
    return result_path(V6_EXPERIMENTS, name, recursive=recursive, required_params=required_params)


def load_paper_if_exists(name: str) -> dict | None:
    path = paper_result_path(name)
    if path is None:
        return None
    return json.loads(path.read_text())


def load_best_paper_if_exists(name: str, required_params: dict[str, object] | None = None) -> dict | None:
    path = paper_result_path(name, recursive=True, required_params=required_params)
    if path is None:
        return None
    return json.loads(path.read_text())


def load_best_v6_result_if_exists(name: str, required_params: dict[str, object] | None = None) -> dict | None:
    path = v6_result_path(name, recursive=True, required_params=required_params)
    if path is None:
        return None
    return json.loads(path.read_text())


def per_box_ms(j: dict) -> float:
    boxes = [t.get("n_boxes", 0) for t in j.get("trials", []) if t.get("success")]
    if not boxes:
        return 0.0
    avg_b = sum(boxes) / len(boxes)
    return j["summary"]["avg_total_time_ms"] / avg_b


def mean(xs: list[float]) -> float:
    return sum(xs) / len(xs) if xs else 0.0


def median(xs: list[float]) -> float | None:
    return percentile(xs, 0.5) if xs else None


def percentile(xs: list[float], q: float) -> float:
    if not xs:
        return 0.0
    ys = sorted(xs)
    if len(ys) == 1:
        return ys[0]
    idx = (len(ys) - 1) * q
    lo = int(idx)
    hi = min(lo + 1, len(ys) - 1)
    frac = idx - lo
    return ys[lo] * (1.0 - frac) + ys[hi] * frac


def fmt_sci(x: float, digits: int = 3) -> str:
    if abs(x) < 1e-300:
        return "0"
    import math
    exp = int(math.floor(math.log10(abs(x))))
    mant = x / (10 ** exp)
    return f"${mant:.{digits}f}\\times10^{{{exp}}}$"


def fmt_gap(x: float) -> str:
    return "0" if abs(x) < 5e-5 else f"{x:.4f}"


def fmt_us_value(x: float | None) -> str:
    if x is None:
        return "--"
    value = float(x)
    if value < 0.1:
        return f"{value:.3f}"
    if value < 10.0:
        return f"{value:.2f}"
    return f"{value:.1f}"


def fmt_bytes_human(x: float | None) -> str:
    if x is None:
        return "--"
    value = float(x)
    units = ["B", "KB", "MB", "GB", "TB", "PB", "EB", "ZB", "YB"]
    unit_idx = 0
    while value >= 1024.0 and unit_idx < len(units) - 1:
        value /= 1024.0
        unit_idx += 1
    if unit_idx == 0:
        return f"{value:.0f} {units[unit_idx]}"
    if value < 10.0:
        return f"{value:.1f} {units[unit_idx]}"
    return f"{value:.0f} {units[unit_idx]}"


def fmt_duration_human(seconds: float | None) -> str:
    if seconds is None:
        return "--"
    value = float(seconds)
    if value < 1e-3:
        return f"{value * 1e6:.1f} $\\mu$s"
    if value < 1.0:
        return f"{value * 1e3:.1f} ms"
    if value < 60.0:
        return f"{value:.1f} s"
    if value < 3600.0:
        return f"{value / 60.0:.1f} min"
    if value < 86400.0:
        return f"{value / 3600.0:.1f} h"
    days = value / 86400.0
    if days < 365.25:
        return f"{days:.1f} d"
    years = days / 365.25
    if years < 1e3:
        return f"{years:.1f} y"
    if years < 1e6:
        return f"{years / 1e3:.1f} kyr"
    if years < 1e9:
        return f"{years / 1e6:.1f} Myr"
    return f"{years / 1e9:.1f} Gyr"


def live_summary(payload: dict | None) -> dict:
    if not payload:
        return {}
    summary = payload.get("summary")
    if isinstance(summary, dict):
        return summary

    seed_trials = payload.get("seed_trials", [])
    build_samples = [float(trial["build_s"]) for trial in seed_trials
                     if trial.get("build_s") is not None]
    total_queries = 0
    total_successes = 0
    for trial in seed_trials:
        for query in trial.get("queries", []):
            total_queries += 1
            if query.get("success"):
                total_successes += 1
    return {
        "build_s_median": median(build_samples),
        "sr": (100.0 * total_successes / total_queries) if total_queries else None,
    }


def live_query_stats(payload: dict | None) -> dict[str, dict[str, float | None]]:
    if not payload:
        return {}

    query_order: list[str] = []
    for query in payload.get("queries", []):
        name = str(query.get("name") or query.get("query") or "")
        if name and name not in query_order:
            query_order.append(name)

    accum: dict[str, dict[str, object]] = {
        name: {"total": 0, "success": 0, "times": [], "paths": []}
        for name in query_order
    }
    for trial in payload.get("seed_trials", []):
        for query in trial.get("queries", []):
            name = str(query.get("query") or query.get("name") or "")
            if not name:
                continue
            bucket = accum.setdefault(name, {"total": 0, "success": 0, "times": [], "paths": []})
            bucket["total"] = int(bucket["total"]) + 1
            if not query.get("success"):
                continue
            bucket["success"] = int(bucket["success"]) + 1
            cast_times = bucket["times"]
            cast_paths = bucket["paths"]
            assert isinstance(cast_times, list)
            assert isinstance(cast_paths, list)
            if query.get("time_s") is not None:
                cast_times.append(float(query["time_s"]))
            if query.get("path_length") is not None:
                cast_paths.append(float(query["path_length"]))

    stats: dict[str, dict[str, float | None]] = {}
    for name, bucket in accum.items():
        total = int(bucket["total"])
        success = int(bucket["success"])
        times = bucket["times"]
        paths = bucket["paths"]
        assert isinstance(times, list)
        assert isinstance(paths, list)
        stats[name] = {
            "sr": (100.0 * success / total) if total else None,
            "query_time_s_median": median(times),
            "query_path_rad_median": median(paths),
        }
    return stats


def tex_source(name: str) -> str:
    return {
        "CritSample": "CritSample",
        "Analytical": "Analytical",
        "GCPC": "GCPC",
        "IFK": "IFK",
        "MC": "MC",
    }.get(name, name.replace("_", "\\_"))


def write_tab_main():
    rows = []
    for sc, robot_pretty in [("2dof_box",   "2-DoF planar"),
                             ("iiwa14_far", "IIWA14")]:
        j = load(f"main_{sc}.json")
        s = j["summary"]
        rows.append((sc.replace("_", "\\_"), robot_pretty, len(j["trials"]),
                     s["success_rate"], s["avg_total_time_ms"],
                     s["avg_opt_length"], s["avg_raw_length"]))
    body = "\n".join(
        f"  {sc} & {rob} & {tr} & {sr*100:.1f}\\% & {t:.1f} & "
        f"{ol:.3f} & {rl:.3f} \\\\"
        for sc, rob, tr, sr, t, ol, rl in rows)
    tex = (
        "\\begin{tabular}{llrrrrr}\n"
        "  \\toprule\n"
        "  Scene & Robot & Trials & SR & $t$ (ms) & $\\ell_{\\mathrm{opt}}$ & "
        "$\\ell_{\\mathrm{raw}}$ \\\\\n"
        "  \\midrule\n"
        f"{body}\n"
        "  \\bottomrule\n"
        "\\end{tabular}\n")
    (OUT / "tab_main.tex").write_text(tex)


def write_tab_threads():
    j = load("threads_iiwa14_far.json")
    body = "\n".join(
        f"  {r['n_threads']} & {r['n_success']} & "
        f"{r['avg_total_time_ms']:.1f} & {r['speedup_vs_1t']:.2f} \\\\"
        for r in j["results"])
    tex = (
        "\\begin{tabular}{rrrr}\n"
        "  \\toprule\n"
        "  $n_\\mathrm{threads}$ & $n_\\mathrm{succ}$ & avg $t$ (ms) & "
        "speedup \\\\\n"
        "  \\midrule\n"
        f"{body}\n"
        "  \\bottomrule\n"
        "\\end{tabular}\n")
    (OUT / "tab_threads.tex").write_text(tex)


def write_tab_pathopt():
    j = load("pathopt_iiwa14_far.json")
    lines = []
    for r in j["results"]:
        combo = r["combo"].replace("_", "\\_")
        lines.append(
            f"  {combo} & {r['n_success']} & "
            f"{r['avg_opt_length']:.3f} & {r['avg_raw_length']:.3f} & "
            f"{r['length_ratio']:.3f} & {r['avg_opt_time_ms']:.2f} \\\\")
    body = "\n".join(lines)
    tex = (
        "\\begin{tabular}{lrrrrr}\n"
        "  \\toprule\n"
        "  Stage & $n_\\mathrm{succ}$ & $\\ell_{\\mathrm{opt}}$ & "
        "$\\ell_{\\mathrm{raw}}$ & ratio & opt $t$ (ms) \\\\\n"
        "  \\midrule\n"
        f"{body}\n"
        "  \\bottomrule\n"
        "\\end{tabular}\n")
    (OUT / "tab_pathopt.tex").write_text(tex)


def write_tab_epiaabb_pipeline():
    """v6-style endpoint-source width-stratified profile."""
    j = load_paper("epiaabb_pipeline.json")
    mc_mode = str(j.get("mc_sampling_mode", "fixed"))
    mc_samples = int(j.get("mc_samples", 0))
    mc_ref_samples = int(j.get("mc_reference_samples", mc_samples))
    mc_ref_width = float(j.get("mc_reference_width", 0.35))
    bin_order = ["0.001-0.05", "0.05-0.10", "0.10-0.20", "0.20-0.50"]
    src_order = ["IFK", "CritSample", "Analytical", "GCPC", "MC"]
    rows = sorted(j["rows"], key=lambda r: (
        bin_order.index(r["width_bin"]) if r["width_bin"] in bin_order else 99,
        src_order.index(r["source"]) if r["source"] in src_order else 99,
    ))
    if mc_mode == "width_proportional":
        caption = (
            "Endpoint-source profiling (MC: width-proportional density, "
            f"ref. {mc_ref_samples} samples at $\\bar{{w}}={mc_ref_width:.2f}$ rad)."
        )
    else:
        caption = f"Endpoint-source profiling (MC: {mc_samples} samples/box)."
    body_lines = []
    prev_bin = None
    for r in rows:
        if prev_bin is not None and r["width_bin"] != prev_bin:
            body_lines.append("\\addlinespace")
        prev_bin = r["width_bin"]
        body_lines.append(
            f"  {r['width_bin']} & {tex_source(r['source'])} & "
            f"{fmt_sci(r['volume_mean'])} & {r['time_us_mean']:.1f} & "
            f"{fmt_gap(r.get('max_negative_gap_vs_mc', 0.0))} \\\\")
    body = "\n".join(body_lines)
    tex = (
        "% Auto-generated from experiments/results_paper/epiaabb_pipeline.json.\n"
        "\\begin{table*}[t]\n"
        "\\centering\n"
        f"\\caption{{{caption}}}\n"
        "\\label{tab:epiaabb_pipeline}\n"
        "\\scriptsize\n"
        "\\setlength{\\tabcolsep}{4pt}\n"
        "\\begin{tabular}{@{}llrrr@{}}\n"
        "\\toprule\n"
        "Width bin & Source & $V$ & Mean $t$ ($\\mu$s) & Gap vs. MC \\\\\n"
        "\\midrule\n"
        f"{body}\n"
        "\\bottomrule\n"
        "\\end{tabular}\n"
        "\\end{table*}\n")
    (OUT / "tab_epiaabb_pipeline.tex").write_text(tex)


def write_tab_link_envelope_pipeline():
    """v6-style subdivision and grid-resolution envelope sweeps."""
    j = load_best_v6_result_if_exists("link_envelope_pipeline.json") or load_paper("link_envelope_pipeline.json")
    marcucci = (
        load_best_v6_result_if_exists("marcucci_envelope_build.json")
        or load_best_paper_if_exists("marcucci_envelope_build.json")
    )
    rows = j["rows"]
    width_sampling_mode = str(j.get("width_sampling_mode", ""))
    depth64_nodes = (1 << 65) - 1
    depth64_capacity = depth64_nodes + max(depth64_nodes // 4, 4096)
    depth64_capacity += depth64_capacity & 1

    marcucci_summary_rows = marcucci.get("summary", []) if isinstance(marcucci, dict) else []
    summary_by_key = {
        (str(row.get("envelope_key")), str(row.get("cache_mode"))): row
        for row in marcucci_summary_rows
        if row.get("endpoint_source") == "critsample"
    }

    def get_summary(envelope_key: str, cache_mode: str = "warm") -> dict:
        row = summary_by_key.get((envelope_key, cache_mode))
        if row is None:
            raise FileNotFoundError(
                f"Missing critsample/{cache_mode} Marcucci summary row for envelope_key={envelope_key}."
            )
        return row

    aabb_probe = get_summary("aabb_s4")
    linkgrid_probe = get_summary("aabb_grid_s4_d004")
    hull_probe = get_summary("hull16_grid_d004")

    def first_row(row_type: str, subdivisions: int, voxel_delta: float) -> dict:
        for row in rows:
            if str(row.get("type")) != row_type:
                continue
            if int(row.get("n_subdivisions", 0)) != subdivisions:
                continue
            if abs(float(row.get("voxel_delta", 0.0)) - voxel_delta) > 1e-9:
                continue
            return row
        raise KeyError(f"Missing row for type={row_type}, subdivisions={subdivisions}, voxel_delta={voxel_delta}")

    base_file_slot_bytes = float(aabb_probe.get("mean_cache_file_bytes_per_capacity_slot") or 0.0)
    base_node_bytes = float(aabb_probe.get("mean_cache_file_bytes_per_node") or 0.0)

    linkgrid_base_row = first_row("LinkIAABB_Grid", 4, 0.04)
    hull_base_row = first_row("Hull16_Grid", 1, 0.04)
    linkgrid_base_payload = float(linkgrid_base_row.get("cache_payload_bytes_mean", 0.0) or 0.0)
    hull_base_payload = float(hull_base_row.get("cache_payload_bytes_mean", 0.0) or 0.0)
    linkgrid_base_node_bytes = float(linkgrid_probe.get("mean_cache_file_bytes_per_node") or 0.0)
    hull_base_node_bytes = float(hull_probe.get("mean_cache_file_bytes_per_node") or 0.0)
    linkgrid_extra_node_bytes = max(0.0, linkgrid_base_node_bytes - base_node_bytes)
    hull_extra_node_bytes = max(0.0, hull_base_node_bytes - base_node_bytes)

    def scaled_extra_node_bytes(row: dict) -> float:
        payload = float(row.get("cache_payload_bytes_mean", 0.0) or 0.0)
        row_type = str(row.get("type"))
        if row_type == "LinkIAABB_Grid":
            if linkgrid_base_payload <= 0.0:
                return 0.0
            return linkgrid_extra_node_bytes * payload / linkgrid_base_payload
        if row_type == "Hull16_Grid":
            if hull_base_payload <= 0.0:
                return 0.0
            return hull_extra_node_bytes * payload / hull_base_payload
        return 0.0

    def node_cache_bytes(row: dict) -> float:
        return base_node_bytes + scaled_extra_node_bytes(row)

    def lect_read_us(row: dict) -> float | None:
        row_type = str(row.get("type"))
        if row_type == "LinkIAABB":
            return float(aabb_probe.get("mean_lect_read_us_per_node_probe") or 0.0)
        if row_type == "LinkIAABB_Grid":
            base_read = float(linkgrid_probe.get("mean_lect_read_us_per_node_probe") or 0.0)
            if linkgrid_base_node_bytes <= 0.0:
                return None
            return base_read * node_cache_bytes(row) / linkgrid_base_node_bytes
        if row_type == "Hull16_Grid":
            base_read = float(hull_probe.get("mean_lect_read_us_per_node_probe") or 0.0)
            if hull_base_node_bytes <= 0.0:
                return None
            return base_read * node_cache_bytes(row) / hull_base_node_bytes
        return None

    def depth64_build_seconds(row: dict) -> float:
        return float(row.get("time_us_mean", 0.0) or 0.0) * depth64_nodes / 1e6

    def depth64_disk_bytes(row: dict) -> float:
        return base_file_slot_bytes * depth64_capacity + scaled_extra_node_bytes(row) * depth64_nodes

    def env_label(r: dict) -> str:
        name = r["envelope"]
        if name.startswith("LinkIAABB_S"):
            return f"LinkIAABB$_{{{r['n_subdivisions']}}}$"
        if name == "LinkGrid_S4":
            return "LinkIAABB-Grid"
        if name == "Hull16Grid":
            return "Hull16-Grid"
        return name.replace("_", "\\_")

    body_lines = []
    for stage in ["subdivision", "grid"]:
        stage_rows = [r for r in rows if r.get("stage") == stage]
        if stage == "grid" and body_lines:
            body_lines.append("\\midrule")
        for r in stage_rows:
            delta = "--" if stage == "subdivision" else f"{r['voxel_delta']:.2f}"
            vox = "--" if not r.get("voxel_count_mean") else f"{r['voxel_count_mean']:.0f}"
            read_us = fmt_us_value(lect_read_us(r))
            node_bytes = fmt_bytes_human(node_cache_bytes(r))
            d64_build = fmt_duration_human(depth64_build_seconds(r))
            d64_disk = fmt_bytes_human(depth64_disk_bytes(r))
            ratio = 100.0 * float(r.get("ratio_to_linkiaabb", 0.0) or 0.0)
            body_lines.append(
                f"  {stage} & {env_label(r)} & {r['n_subdivisions']} & {delta} & "
                f"{r['volume_mean']:.3f} & {r['time_us_mean']:.1f} & {read_us} & "
                f"{node_bytes} & {d64_build} & {d64_disk} & {vox} & "
                f"{ratio:.1f}\\% " + "\\\\")
    body = "\n".join(body_lines)
    if width_sampling_mode == "exp1_stratified_aggregate":
        caption = (
            "Link-envelope subdivision and grid sweep, aggregated over the same "
            "width-stratified IIWA14 intervals used by Exp.~1. Warm LECT-read, "
            "per-node cache, and depth-64 disk estimates are anchored by the "
            "CritSample Marcucci warm-cache probe at $\\delta=0.04$ for each "
            "representation, with grid rows scaled by the measured per-node payload size."
        )
    elif width_sampling_mode == "uniform_joint_width":
        caption = (
            "Link-envelope subdivision and grid sweep under the v6-compatible "
            "IIWA14 CritSample box protocol (per-joint widths sampled uniformly "
            "from $[0.10, 0.50]$ rad)."
        )
    else:
        caption = "Link-envelope subdivision and grid sweep."
    tex = (
        "% Auto-generated from the best available v6 link_envelope_pipeline.json and marcucci_envelope_build.json.\n"
        "\\begin{table*}[t]\n"
        "\\centering\n"
        f"\\caption{{{caption}}}\n"
        "\\label{tab:link_envelope_pipeline}\n"
        "\\scriptsize\n"
        "\\setlength{\\tabcolsep}{4pt}\n"
        "\\resizebox{\\textwidth}{!}{%\n"
        "\\begin{tabular}{@{}llrrrrrrrrrr@{}}\n"
        "\\toprule\n"
        "Stage & Envelope & $S$ & $\\delta$(m) & Vol. (m$^3$) & Mean time ($\\mu$s) & Warm LECT read ($\\mu$s) & Node cache & Depth-64 build & Depth-64 disk & Voxels & Ratio "
        + "\\\\\n"
        "\\midrule\n"
        f"{body}\n"
        "\\bottomrule\n"
        "\\end{tabular}%\n"
        "}\n"
        "\\end{table*}\n")
    (OUT / "tab_link_envelope_pipeline.tex").write_text(tex)
    (V6_OUT / "tab_link_envelope_pipeline.tex").write_text(tex)


def write_tab_compare():
    rows = []
    anytime_planners = [
        ("rrt_star",          "OMPL RRT*"),
        ("informed_rrt_star", "OMPL Informed-RRT*"),
        ("bit_star",          "OMPL BIT*"),
    ]
    for sc in ["2dof_box", "iiwa14_far"]:
        sbf = load(f"main_{sc}.json")
        try:    ompl = load(f"ompl_{sc}.json")
        except FileNotFoundError: ompl = None
        try:    drake = load(f"drake_{sc}.json")
        except FileNotFoundError: drake = None
        s = sbf["summary"]
        rows.append((sc.replace("_", "\\_"), "SBF (ours)",
                     s["success_rate"], s["avg_total_time_ms"],
                     s["avg_opt_length"]))
        if ompl:
            so = ompl["summary"]
            rows.append(("", "OMPL RRT-Connect",
                         so["success_rate"], so["avg_total_time_ms"],
                         so.get("avg_path_length", 0.0)))
        for tag, label in anytime_planners:
            try:
                jp = load(f"ompl_{tag}_{sc}.json")
            except FileNotFoundError:
                continue
            sp = jp["summary"]
            rows.append(("", label,
                         sp["success_rate"], sp["avg_total_time_ms"],
                         sp.get("avg_path_length", 0.0)))
        if drake:
            sd = drake["summary"]
            rows.append(("", "Drake GCS (1-region)",
                         sd["success_rate"], sd["avg_total_time_ms"],
                         sd.get("avg_path_length", 0.0)))
    body = "\n".join(
        f"  {sc} & {p} & {sr*100:.1f}\\% & {t:.1f} & {pl:.3f} \\\\"
        for sc, p, sr, t, pl in rows)
    tex = (
        "\\begin{tabular}{llrrr}\n"
        "  \\toprule\n"
        "  Scene & Planner & SR & avg $t$ (ms) & avg $\\ell$ \\\\\n"
        "  \\midrule\n"
        f"{body}\n"
        "  \\bottomrule\n"
        "\\end{tabular}\n")
    (OUT / "tab_compare.tex").write_text(tex)


def write_tab_newscenes():
    """SBF vs OMPL on the two new scenes (clutter, narrow)."""
    rows = []
    anytime_planners = [
        ("rrt_star",          "OMPL RRT*"),
        ("informed_rrt_star", "OMPL Informed-RRT*"),
        ("bit_star",          "OMPL BIT*"),
    ]
    for sc, pretty in [("iiwa14_clutter", "iiwa14\\_clutter"),
                       ("iiwa14_narrow",  "iiwa14\\_narrow")]:
        try:    sbf = load(f"main_{sc}.json")
        except FileNotFoundError: continue
        s = sbf["summary"]
        rows.append((pretty, "SBF (ours)",
                     s["success_rate"], s["avg_total_time_ms"],
                     s["avg_opt_length"]))
        try:
            ompl = load(f"ompl_{sc}.json")
            so = ompl["summary"]
            rows.append(("", "OMPL RRT-Connect",
                         so["success_rate"], so["avg_total_time_ms"],
                         so.get("avg_path_length", 0.0)))
        except FileNotFoundError:
            pass
        for tag, label in anytime_planners:
            try:
                jp = load(f"ompl_{tag}_{sc}.json")
            except FileNotFoundError:
                continue
            sp = jp["summary"]
            rows.append(("", label,
                         sp["success_rate"], sp["avg_total_time_ms"],
                         sp.get("avg_path_length", 0.0)))
    body = "\n".join(
        f"  {sc} & {p} & {sr*100:.1f}\\% & {t:.1f} & {pl:.3f} \\\\"
        for sc, p, sr, t, pl in rows)
    tex = (
        "\\begin{table}[t]\n"
        "\\centering\n"
        "\\caption{IIWA14 stress-scene baselines.}\n"
        "\\label{tab:newscenes}\n"
        "\\scriptsize\n"
        "\\setlength{\\tabcolsep}{3pt}\n"
        "\\begin{tabular}{llrrr}\n"
        "  \\toprule\n"
        "  Scene & Planner & SR & avg $t$ (ms) & avg $\\ell$ \\\\\n"
        "  \\midrule\n"
        f"{body}\n"
        "  \\bottomrule\n"
        "\\end{tabular}\n"
        "\\end{table}\n")
    (OUT / "tab_newscenes.tex").write_text(tex)


def write_tab_marcucci():
    j = load("marcucci.json")
    rows = []
    for q in j["queries"]:
        name = q["name"].replace("->", "$\\rightarrow$")
        rows.append(
            f"  {name} & {q['sr']*100:.1f}\\% & "
            f"{q['t_med_s']*1000.0:.1f} & {q['len_med']:.3f} \\\\")
    body = "\n".join(rows)
    build_med = j["build"]["median_s"] * 1000.0
    seeds     = j["seeds"]
    tex = (
        "\\begin{tabular}{lrrr}\n"
        "  \\toprule\n"
        "  Query & SR & $t_{\\mathrm{med}}$ (ms) & "
        "$\\ell_{\\mathrm{med}}$ \\\\\n"
        "  \\midrule\n"
        f"{body}\n"
        "  \\midrule\n"
        f"  \\multicolumn{{4}}{{l}}{{Forest build (median over {seeds} "
        f"seeds): {build_med:.0f}\\,ms}} \\\\\n"
        "  \\bottomrule\n"
        "\\end{tabular}\n")
    (OUT / "tab_marcucci.tex").write_text(tex)


def write_tab_query():
    """Write the Marcucci per-query comparison with live IRIS and OMPL baselines."""
    j = load("marcucci.json")
    iris_np_live = load_best_paper_if_exists("marcucci_iris_np_gcs.json")
    iris_zo_live = load_best_paper_if_exists("marcucci_iris_zo_gcs.json")
    ompl_prm_live = load_best_paper_if_exists(
        "marcucci_ompl_prm.json",
        required_params={
            "build_metric": "mean_per_seed_roadmap_build_time_across_query_runs",
            "query_metric": "query_only_solve_time_after_roadmap_build",
        },
    )
    ompl_bitstar_budget_live = load_best_paper_if_exists(
        "marcucci_ompl_bitstar_budget.json",
        required_params={"bitstar_budget_s": 1.0},
    )
    bitstar_budget_s = 1.0
    if ompl_bitstar_budget_live is not None:
        params = ompl_bitstar_budget_live.get("params")
        if isinstance(params, dict) and params.get("bitstar_budget_s") is not None:
            bitstar_budget_s = float(params["bitstar_budget_s"])

    def fmt_stat(value: float | None, digits: int = 3) -> str:
        if value is None:
            return "--"
        return f"{value:.{digits}f}"

    def fmt_build(value: float | None) -> str:
        if value is None:
            return "--"
        if abs(value) < 5e-4:
            return "0"
        return f"{value:.3f}"

    def build_header(label: str, build_s: float | None, *, extra: str | None = None) -> str:
        build_label = "build=--" if build_s is None else f"build={fmt_build(build_s)}\\,s"
        suffix = build_label if extra is None else f"{build_label}, {extra}"
        return rf"\shortstack{{{label}\\{suffix}}}"

    sbf_by_query = {
        q["name"]: {
            "sr": 100.0 * float(q["sr"]),
            "query_time_s_median": float(q["t_med_s"]),
            "query_path_rad_median": float(q["len_med"]),
        }
        for q in j["queries"]
    }
    iris_np_by_query = live_query_stats(iris_np_live)
    iris_zo_by_query = live_query_stats(iris_zo_live)
    ompl_prm_by_query = live_query_stats(ompl_prm_live)
    ompl_bitstar_budget_by_query = live_query_stats(ompl_bitstar_budget_live)
    iris_np_summary = live_summary(iris_np_live)
    iris_zo_summary = live_summary(iris_zo_live)
    ompl_prm_summary = live_summary(ompl_prm_live)

    method_specs = [
        {
            "label": build_header(r"SBF (ours)", float(j["build"]["median_s"])),
            "stats": sbf_by_query,
            "columns": [r"SR (\%)", "Time", "Path"],
            "keys": ["sr", "query_time_s_median", "query_path_rad_median"],
        },
        {
            "label": build_header(r"IRIS-NP~+~GCS", iris_np_summary.get("build_s_median")),
            "stats": iris_np_by_query,
            "columns": [r"SR (\%)", "Time", "Path"],
            "keys": ["sr", "query_time_s_median", "query_path_rad_median"],
        },
        {
            "label": build_header(r"IRIS-ZO~+~GCS", iris_zo_summary.get("build_s_median")),
            "stats": iris_zo_by_query,
            "columns": [r"SR (\%)", "Time", "Path"],
            "keys": ["sr", "query_time_s_median", "query_path_rad_median"],
        },
        {
            "label": build_header(r"OMPL PRM", ompl_prm_summary.get("build_s_median")),
            "stats": ompl_prm_by_query,
            "columns": [r"SR (\%)", "Time", "Path"],
            "keys": ["sr", "query_time_s_median", "query_path_rad_median"],
        },
        {
            "label": build_header(r"OMPL BIT*", 0.0, extra=rf"budget={bitstar_budget_s:g}\,s"),
            "stats": ompl_bitstar_budget_by_query,
            "columns": [r"SR (\%)", "Path"],
            "keys": ["sr", "query_path_rad_median"],
        },
    ]

    group_specs = ["r" * len(spec["columns"]) for spec in method_specs]
    colspec = "@{}l" + "|".join(group_specs) + "@{}"
    group_header = " & ".join(
        [" "] + [f"\\multicolumn{{{len(spec['columns'])}}}{{c}}{{\\textbf{{{spec['label']}}}}}" for spec in method_specs]
    )
    column_header = " & ".join(["Query"] + [" & ".join(spec["columns"]) for spec in method_specs])
    cmidr_parts = []
    current_col = 2
    for spec in method_specs:
        width = len(spec["columns"])
        cmidr_parts.append(f"\\cmidrule(lr){{{current_col}-{current_col + width - 1}}}")
        current_col += width
    cmidr = "".join(cmidr_parts)

    rows = []
    for q in j["queries"]:
        name = q["name"].replace("->", "$\\rightarrow$")
        row_values = [name]
        for spec in method_specs:
            stats = spec["stats"].get(q["name"], {})
            row_values.extend(fmt_stat(stats.get(key), 1 if key == "sr" else 3) for key in spec["keys"])
        rows.append(" & ".join(row_values) + r" \\")
    body = "\n".join(rows)
    tex = (
        "% Generated from experiments/results_nightly/full/marcucci.json.\n"
        "% SBF uses the cached-query protocol of the already built forest; IRIS and OMPL rows use per-query statistics from the best available live reruns.\n"
        "\\begin{table*}[t]\n"
        "\\centering\n"
        "\\caption{Marcucci per-query comparison across reusable-region planners and OMPL baselines.}\n"
        "\\label{tab:query}\n"
        "\\scriptsize\n"
        "\\setlength{\\tabcolsep}{2.2pt}\n"
        "\\resizebox{\\textwidth}{!}{%\n"
        f"\\begin{{tabular}}{{{colspec}}}\n"
        "\\toprule\n"
        f"{group_header} \\\\" + "\n"
        f"{cmidr}\n"
        f"{column_header} \\\\" + "\n"
        "\\midrule\n"
        f"{body}\n"
        "\\bottomrule\n"
        "\\end{tabular}\n"
        "}\n"
        "{\\footnotesize Scene-build medians are shown directly in the method headers. SBF rows use the cached-query protocol of the already built forest. IRIS rows report per-query successful-solve medians; failed solves contribute only to the SR column. OMPL PRM uses the same per-query SR/Path statistics, but its Time column reports only the second solve on a fixed roadmap, while its header build time is the median over seeds of the per-seed mean initial roadmap-building wall-clock across the five query-specific runs. OMPL BIT* has no reusable scene-build phase, so its header shows 0\\,s build; SR and Path summarize the feasible solutions found within the same fixed wall-clock budget of "
        f"{bitstar_budget_s:g}"
        "\\,s, and the Time column is omitted. All live baseline rows in this table use the same 16-thread resource envelope on cores 0--15.}\n"
        "\\end{table*}\n")
    (OUT / "tab_query.tex").write_text(tex)


def remove_stale_generated_tables() -> None:
    stale_tables = [OUT / "tab_combined_baselines.tex"]
    for path in stale_tables:
        if path.exists():
            path.unlink()


def write_tab_dof_scaling():
    j = load_paper_if_exists("dof_scaling.json")
    if j is None:
        return
    cells = sorted(
        j.get("cells", []),
        key=lambda cell: (cell.get("scene_family", ""), cell.get("active_dof", 0)),
    )
    if not cells:
        return

    def scene_label(cell: dict) -> str:
        family = cell.get("scene_family") or cell.get("scene_name", "")
        return family.replace("iiwa14_", "")

    body_lines = []
    prev_scene = None
    for cell in cells:
        cur_scene = scene_label(cell)
        if prev_scene is not None and cur_scene != prev_scene:
            body_lines.append("\\addlinespace")
        prev_scene = cur_scene
        body_lines.append(
            "  {scene} & {dof} & {sr:.1f}\\% & {bridge:.1f}\\% & {boxes:.0f} & {grow:.1f} & {total:.1f} & {opt} \\\\".format(
                scene=cur_scene,
                dof=cell["active_dof"],
                sr=100.0 * cell["summary"]["success_rate"],
                bridge=100.0 * cell["summary"]["point_bridge_rate"],
                boxes=cell["summary"]["median_n_boxes"],
                grow=cell["summary"]["median_grow_time_ms"],
                total=cell["summary"]["median_total_time_ms"],
                opt=("--" if cell["summary"].get("n_success", 0) == 0
                     else f"{cell['summary']['median_opt_length']:.3f}"),
            )
        )
    body = "\n".join(body_lines)
    tex = (
        "% Auto-generated from experiments/results_paper/dof_scaling.json.\n"
        "\\begin{table}[t]\n"
        "\\centering\n"
        "\\caption{True active-DoF scaling on IIWA far and narrow scenes with inactive joints frozen at the scene start posture.}\n"
        "\\label{tab:dof_scaling}\n"
        "\\scriptsize\n"
        "\\setlength{\\tabcolsep}{3pt}\n"
        "\\begin{tabular}{@{}lrrrrrrr@{}}\n"
        "\\toprule\n"
        "Scene & DoF & SR & PB & Boxes & Grow (ms) & Total (ms) & Path (rad) " "\\\\" "\n"
        "\\midrule\n"
        f"{body}\n"
        "\\bottomrule\n"
        "\\end{tabular}\n"
        "\\end{table}\n")
    (OUT / "tab_dof_scaling.tex").write_text(tex)


def write_macros():
    m = {}

    j_iiwa = load("main_iiwa14_far.json")
    j_2dof = load("main_2dof_box.json")
    j_thr  = load("threads_iiwa14_far.json")
    j_po   = load("pathopt_iiwa14_far.json")
    j_marc = load("marcucci.json")

    # IIWA14 shelf scene
    m["sbfIiwaSR"]     = f"{j_iiwa['summary']['success_rate']*100:.1f}"
    m["sbfIiwaTime"]   = f"{j_iiwa['summary']['avg_total_time_ms']:.1f}"
    m["sbfIiwaLen"]    = f"{j_iiwa['summary']['avg_opt_length']:.3f}"
    m["sbfIiwaPerBox"] = f"{per_box_ms(j_iiwa):.4f}"
    m["sbfIiwaSeeds"]  = f"{len(j_iiwa['trials'])}"

    # 2-DoF
    m["sbfTwoDofSR"]     = f"{j_2dof['summary']['success_rate']*100:.1f}"
    m["sbfTwoDofTime"]   = f"{j_2dof['summary']['avg_total_time_ms']:.1f}"
    m["sbfTwoDofPerBox"] = f"{per_box_ms(j_2dof):.4f}"

    # Threads
    m["sbfMaxSpeedup"] = (
        f"{max(r['speedup_vs_1t'] for r in j_thr['results']):.2f}")
    m["sbfMaxThreads"] = (
        f"{max(r['n_threads'] for r in j_thr['results'])}")

    # Path optimisation
    full = next(r for r in j_po["results"] if r["combo"] == "full")
    m["sbfPathOptRatio"] = f"{full['length_ratio']:.3f}"
    m["sbfPathOptTime"]  = f"{full['avg_opt_time_ms']:.2f}"

    # Marcucci IIWA14 cabinet
    pairs_ms = [q["t_med_s"] * 1000.0 for q in j_marc["queries"]]
    total_trials = int(j_marc["seeds"] * len(j_marc["queries"]))
    total_successes = int(round(sum(q["sr"] * j_marc["seeds"]
                                    for q in j_marc["queries"])))
    m["sbfMarcucciSeeds"]      = f"{j_marc['seeds']}"
    m["sbfMarcucciBuildMs"]    = f"{j_marc['build']['median_s']*1000.0:.0f}"
    m["sbfMarcucciAvgQueryMs"] = f"{sum(pairs_ms)/len(pairs_ms):.1f}"
    m["sbfMarcucciNumQueries"] = f"{len(j_marc['queries'])}"
    m["sbfMarcucciSR"] = (
        f"{100.0 * sum(q['sr'] for q in j_marc['queries']) / len(j_marc['queries']):.1f}")
    m["sbfMarcucciHardestMs"]  = f"{max(pairs_ms):.1f}"
    m["sbfMarcucciTotalTrials"] = f"{total_trials}"
    m["sbfMarcucciSuccesses"]   = f"{total_successes}"

    text = "% Auto-generated by scripts/build_paper_tables.py — do not edit.\n"
    for k, v in m.items():
        text += f"\\newcommand{{\\{k}}}{{{v}}}\n"
    (OUT / "macros.tex").write_text(text)


if __name__ == "__main__":
    write_tab_epiaabb_pipeline()
    write_tab_link_envelope_pipeline()
    write_tab_main()
    write_tab_threads()
    write_tab_pathopt()
    write_tab_compare()
    write_tab_newscenes()
    write_tab_marcucci()
    write_tab_query()
    write_tab_dof_scaling()
    write_macros()
    remove_stale_generated_tables()
    print(f"Wrote: {sorted(p.name for p in OUT.glob('*.tex'))}")

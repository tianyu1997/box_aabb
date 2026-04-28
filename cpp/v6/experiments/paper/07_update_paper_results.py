#!/usr/bin/env python3
"""Generate v6 paper LaTeX fragments from the current paper JSON results."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from statistics import median
from typing import Any


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
RESULTS = ROOT / "experiments" / "results_paper"
DOC = ROOT / "doc" / "paper"
GENERATED = DOC / "generated"


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text())


def fmt_percent(value: float) -> str:
    return f"{100.0 * float(value):.1f}\\%"


def query_label(name: str) -> str:
    return name.replace("->", "$\\rightarrow$")


def write_marcucci_table(payload: dict[str, Any], out_path: Path) -> None:
    rows = []
    for query in payload.get("queries", []):
        rows.append(
            "  {name} & {sr} & {time:.1f} & {length:.3f} \\\\".format(
                name=query_label(str(query["name"])),
                sr=fmt_percent(float(query["sr"])),
                time=1000.0 * float(query["t_med_s"]),
                length=float(query["len_med"]),
            )
        )
    build_ms = 1000.0 * float(payload["build"]["median_s"])
    seeds = int(payload.get("seeds", 0))
    text = (
        "% Auto-generated from experiments/results_paper/marcucci_combined.json.\n"
        "\\begin{tabular}{lrrr}\n"
        "  \\toprule\n"
        "  Query & SR & $t_{\\mathrm{med}}$ (ms) & $\\ell_{\\mathrm{med}}$ \\\\\n"
        "  \\midrule\n"
        f"{chr(10).join(rows)}\n"
        "  \\midrule\n"
        f"  \\multicolumn{{4}}{{l}}{{Forest build (median over {seeds} seeds): {build_ms:.0f}\\,ms}} \\\\\n"
        "  \\bottomrule\n"
        "\\end{tabular}\n"
    )
    out_path.write_text(text)


def sorted_scan_summary(payload: dict[str, Any]) -> list[dict[str, Any]]:
    return sorted(
        payload.get("summary", []),
        key=lambda item: (
            -float(item.get("success_rate", 0.0)),
            float(item.get("query_s_sum_mean") or 1e99),
            float(item.get("build_s_mean") or 1e99),
            float(item.get("n_boxes_mean") or 1e99),
        ),
    )


def write_scan_table(payload: dict[str, Any], out_path: Path, *, top_k: int) -> None:
    rows = []
    for item in sorted_scan_summary(payload)[:top_k]:
        rows.append(
            "  {gb:.2f} & {un:.2f} & {miss} & {sr:.1f} & {query:.3f} & {build:.2f} & {boxes:.0f} \\\\".format(
                gb=float(item["goal_bias"]),
                un=float(item["unexplored_sample_prob"]),
                miss=int(item["max_consecutive_miss"]),
                sr=100.0 * float(item["success_rate"]),
                query=float(item.get("query_s_sum_mean") or 0.0),
                build=float(item.get("build_s_mean") or 0.0),
                boxes=float(item.get("n_boxes_mean") or 0.0),
            )
        )
    text = (
        "% Auto-generated from experiments/results_paper/sbf_parameter_scan.json.\n"
        "\\begin{tabular}{rrrrrrr}\n"
        "  \\toprule\n"
        "$p_g$ & $p_u$ & Miss & SR & $\\sum t_q$ (s) & Build (s) & Boxes \\\\\n"
        "  \\midrule\n"
        f"{chr(10).join(rows)}\n"
        "  \\bottomrule\n"
        "\\end{tabular}\n"
    )
    out_path.write_text(text)


def fmt_optional(value: Any, *, digits: int = 2) -> str:
    if value is None:
        return "--"
    return f"{float(value):.{digits}f}"


def percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    xs = sorted(values)
    if len(xs) == 1:
        return xs[0]
    idx = (len(xs) - 1) * q
    lo = int(idx)
    hi = min(lo + 1, len(xs) - 1)
    frac = idx - lo
    return xs[lo] * (1.0 - frac) + xs[hi] * frac


def stat_median(values: list[float]) -> float | None:
    return percentile(values, 0.5)


def result_path(base_dir: Path, name: str, *, required_params: dict[str, object] | None = None) -> Path | None:
    def matches(path: Path) -> bool:
        if not required_params:
            return True
        try:
            payload = load_json(path)
        except Exception:
            return False
        params = payload.get("params")
        if not isinstance(params, dict):
            return False
        for key, expected in required_params.items():
            actual = params.get(key)
            if isinstance(actual, (int, float)) and isinstance(expected, (int, float)):
                if abs(float(actual) - float(expected)) > 1e-9:
                    return False
            elif actual != expected:
                return False
        return True

    candidates = [base_dir / name]
    candidates.extend(base_dir.rglob(name))
    valid = [path for path in candidates if path.exists() and matches(path)]
    if not valid:
        return None

    def rank(path: Path) -> tuple[int, int, int, float, str]:
        try:
            payload = load_json(path)
        except Exception:
            return (0, 0, 0, path.stat().st_mtime, str(path))
        summary = payload.get("summary") if isinstance(payload.get("summary"), dict) else {}
        return (
            1 if not payload.get("quick", False) else 0,
            int(payload.get("seeds", 0) or 0),
            int(summary.get("n_success", 0) or 0),
            path.stat().st_mtime,
            str(path),
        )

    return max(valid, key=rank)


def load_result_if_exists(base_dir: Path, name: str, *, required_params: dict[str, object] | None = None) -> dict[str, Any] | None:
    path = result_path(base_dir, name, required_params=required_params)
    if path is None:
        return None
    return load_json(path)


def live_summary(payload: dict[str, Any] | None) -> dict[str, Any]:
    if not payload:
        return {}
    summary = payload.get("summary")
    if isinstance(summary, dict):
        return summary
    seed_trials = payload.get("seed_trials", [])
    build_samples = [float(trial["build_s"]) for trial in seed_trials if trial.get("build_s") is not None]
    total_queries = 0
    total_successes = 0
    for trial in seed_trials:
        for query in trial.get("queries", []):
            total_queries += 1
            if query.get("success"):
                total_successes += 1
    return {
        "build_s_median": stat_median(build_samples),
        "sr": (100.0 * total_successes / total_queries) if total_queries else None,
    }


def live_query_stats(payload: dict[str, Any] | None) -> dict[str, dict[str, float | None]]:
    if not payload:
        return {}
    accum: dict[str, dict[str, Any]] = {}
    for query in payload.get("queries", []):
        name = str(query.get("name") or query.get("query") or "")
        if name:
            accum.setdefault(name, {"total": 0, "success": 0, "times": [], "paths": []})
    for trial in payload.get("seed_trials", []):
        for query in trial.get("queries", []):
            name = str(query.get("query") or query.get("name") or "")
            if not name:
                continue
            bucket = accum.setdefault(name, {"total": 0, "success": 0, "times": [], "paths": []})
            bucket["total"] += 1
            if not query.get("success"):
                continue
            bucket["success"] += 1
            if query.get("time_s") is not None:
                bucket["times"].append(float(query["time_s"]))
            if query.get("path_length") is not None:
                bucket["paths"].append(float(query["path_length"]))
    return {
        name: {
            "sr": (100.0 * bucket["success"] / bucket["total"]) if bucket["total"] else None,
            "query_time_s_median": stat_median(bucket["times"]),
            "query_path_rad_median": stat_median(bucket["paths"]),
        }
        for name, bucket in accum.items()
    }


def write_query_comparison_table(sbf_payload: dict[str, Any], results_dir: Path, out_path: Path) -> None:
    iris_np = load_result_if_exists(results_dir, "marcucci_iris_np_gcs.json")
    iris_zo = load_result_if_exists(results_dir, "marcucci_iris_zo_gcs.json")
    ompl_prm = load_result_if_exists(
        results_dir,
        "marcucci_ompl_prm.json",
        required_params={
            "build_metric": "mean_per_seed_roadmap_build_time_across_query_runs",
            "query_metric": "query_only_solve_time_after_roadmap_build",
        },
    )
    ompl_bitstar = load_result_if_exists(
        results_dir,
        "marcucci_ompl_bitstar_budget.json",
        required_params={"bitstar_budget_s": 1.0},
    )
    bitstar_budget_s = 1.0
    if ompl_bitstar and isinstance(ompl_bitstar.get("params"), dict):
        bitstar_budget_s = float(ompl_bitstar["params"].get("bitstar_budget_s", 1.0))

    def fmt_stat(value: float | None, digits: int = 3) -> str:
        if value is None:
            return "--"
        return f"{float(value):.{digits}f}"

    def fmt_build(value: float | None) -> str:
        if value is None:
            return "--"
        if abs(float(value)) < 5e-4:
            return "0"
        return f"{float(value):.3f}"

    def build_header(label: str, build_s: float | None, *, extra: str | None = None) -> str:
        build_label = "build=--" if build_s is None else f"build={fmt_build(build_s)}\\,s"
        suffix = build_label if extra is None else f"{build_label}, {extra}"
        return rf"\shortstack{{{label}\\{suffix}}}"

    sbf_by_query = {
        str(query["name"]): {
            "sr": 100.0 * float(query["sr"]),
            "query_time_s_median": float(query["t_med_s"]),
            "query_path_rad_median": float(query["len_med"]),
        }
        for query in sbf_payload.get("queries", [])
    }
    method_specs = [
        {
            "label": build_header(r"SBF (ours)", float(sbf_payload["build"]["median_s"])),
            "stats": sbf_by_query,
            "columns": [r"SR (\%)", "Time", "Path"],
            "keys": ["sr", "query_time_s_median", "query_path_rad_median"],
        },
        {
            "label": build_header(r"IRIS-NP~+~GCS", live_summary(iris_np).get("build_s_median")),
            "stats": live_query_stats(iris_np),
            "columns": [r"SR (\%)", "Time", "Path"],
            "keys": ["sr", "query_time_s_median", "query_path_rad_median"],
        },
        {
            "label": build_header(r"IRIS-ZO~+~GCS", live_summary(iris_zo).get("build_s_median")),
            "stats": live_query_stats(iris_zo),
            "columns": [r"SR (\%)", "Time", "Path"],
            "keys": ["sr", "query_time_s_median", "query_path_rad_median"],
        },
        {
            "label": build_header(r"OMPL PRM", live_summary(ompl_prm).get("build_s_median")),
            "stats": live_query_stats(ompl_prm),
            "columns": [r"SR (\%)", "Time", "Path"],
            "keys": ["sr", "query_time_s_median", "query_path_rad_median"],
        },
        {
            "label": build_header(r"OMPL BIT*", 0.0, extra=rf"budget={bitstar_budget_s:g}\\,s"),
            "stats": live_query_stats(ompl_bitstar),
            "columns": [r"SR (\%)", "Path"],
            "keys": ["sr", "query_path_rad_median"],
        },
    ]

    colspec = "@{}l" + "|".join("r" * len(spec["columns"]) for spec in method_specs) + "@{}"
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

    rows = []
    for query in sbf_payload.get("queries", []):
        name = query_label(str(query["name"]))
        values = [name]
        for spec in method_specs:
            stats = spec["stats"].get(str(query["name"]), {})
            values.extend(fmt_stat(stats.get(key), 1 if key == "sr" else 3) for key in spec["keys"])
        rows.append(" & ".join(values) + r" \\")

    text = (
        "% Auto-generated from v6 paper SBF results and v6 baseline reruns.\n"
        "% SBF uses cached queries on the built forest; IRIS and OMPL rows use live baseline JSONs when present.\n"
        "\\begin{table*}[t]\n"
        "\\centering\n"
        "\\caption{Marcucci per-query comparison across reusable-region planners and OMPL baselines.}\n"
        "\\label{tab:query}\n"
        "\\scriptsize\n"
        "\\setlength{\\tabcolsep}{2.2pt}\n"
        "\\resizebox{\\textwidth}{!}{%\n"
        f"\\begin{{tabular}}{{{colspec}}}\n"
        "\\toprule\n"
        f"{group_header} \\\\\n"
        f"{''.join(cmidr_parts)}\n"
        f"{column_header} \\\\\n"
        "\\midrule\n"
        f"{chr(10).join(rows)}\n"
        "\\bottomrule\n"
        "\\end{tabular}\n"
        "}\n"
        "{\\footnotesize Scene-build medians are shown directly in the method headers. SBF rows use the cached-query protocol of the already built forest. IRIS rows report per-query successful-solve medians; failed solves contribute only to the SR column. OMPL PRM uses the same per-query SR/Path statistics, but its Time column reports only the second solve on a fixed roadmap, while its header build time is the median over seeds of the per-seed mean initial roadmap-building wall-clock across the five query-specific runs. OMPL BIT* has no reusable scene-build phase, so its header shows 0\\,s build; SR and Path summarize the feasible solutions found within the same fixed wall-clock budget of "
        f"{bitstar_budget_s:g}"
        "\\,s, and the Time column is omitted. All live baseline rows in this table use the same 16-thread resource envelope.}\n"
        "\\end{table*}\n"
    )
    out_path.write_text(text)


def fmt_us_value(value: float | None) -> str:
    if value is None:
        return "--"
    value = float(value)
    if value < 0.1:
        return f"{value:.3f}"
    if value < 10.0:
        return f"{value:.2f}"
    return f"{value:.1f}"


def fmt_bytes_human(value: float | None) -> str:
    if value is None:
        return "--"
    value = float(value)
    units = ["B", "KB", "MB", "GB", "TB", "PB", "EB", "ZB", "YB"]
    unit_index = 0
    while value >= 1024.0 and unit_index < len(units) - 1:
        value /= 1024.0
        unit_index += 1
    if unit_index == 0:
        return f"{value:.0f} {units[unit_index]}"
    if value < 10.0:
        return f"{value:.1f} {units[unit_index]}"
    return f"{value:.0f} {units[unit_index]}"


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


def write_link_envelope_pipeline_table(
    payload: dict[str, Any],
    marcucci: dict[str, Any],
    out_path: Path,
    *,
    extrapolation_depth: int,
) -> None:
    rows = payload["rows"]
    extrapolated_nodes = (1 << (extrapolation_depth + 1)) - 1
    extrapolated_capacity = extrapolated_nodes + max(extrapolated_nodes // 4, 4096)
    extrapolated_capacity += extrapolated_capacity & 1

    summary_by_key = {
        (str(row.get("envelope_key")), str(row.get("cache_mode"))): row
        for row in marcucci.get("summary", [])
        if row.get("endpoint_source") == "critsample"
    }

    def get_summary(envelope_key: str, cache_mode: str = "warm") -> dict[str, Any]:
        row = summary_by_key.get((envelope_key, cache_mode))
        if row is None:
            raise FileNotFoundError(
                f"Missing critsample/{cache_mode} Marcucci summary row for envelope_key={envelope_key}."
            )
        return row

    aabb_probe = get_summary("aabb_s4")
    hull_probe = get_summary("hull16_grid_d004")

    def first_row(row_type: str, subdivisions: int, voxel_delta: float) -> dict[str, Any]:
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
    hull_base_row = first_row("Hull16_Grid", 1, 0.04)
    hull_base_payload = float(hull_base_row.get("cache_payload_bytes_mean", 0.0) or 0.0)
    hull_base_node_bytes = float(hull_probe.get("mean_cache_file_bytes_per_node") or 0.0)
    hull_extra_node_bytes = max(0.0, hull_base_node_bytes - base_node_bytes)

    def scaled_extra_node_bytes(row: dict[str, Any]) -> float:
        row_type = str(row.get("type"))
        payload_bytes = float(row.get("cache_payload_bytes_mean", 0.0) or 0.0)
        if row_type == "Hull16_Grid":
            return 0.0 if hull_base_payload <= 0.0 else hull_extra_node_bytes * payload_bytes / hull_base_payload
        return 0.0

    def node_cache_bytes(row: dict[str, Any]) -> float:
        return base_node_bytes + scaled_extra_node_bytes(row)

    def lect_read_us(row: dict[str, Any]) -> float | None:
        row_type = str(row.get("type"))
        if row_type == "LinkIAABB":
            return float(aabb_probe.get("mean_lect_read_us_per_node_probe") or 0.0)
        if row_type == "Hull16_Grid":
            base_read = float(hull_probe.get("mean_lect_read_us_per_node_probe") or 0.0)
            return None if hull_base_node_bytes <= 0.0 else base_read * node_cache_bytes(row) / hull_base_node_bytes
        return None

    def extrapolated_build_seconds(row: dict[str, Any]) -> float:
        return float(row.get("time_us_mean", 0.0) or 0.0) * extrapolated_nodes / 1e6

    def extrapolated_disk_bytes(row: dict[str, Any]) -> float:
        return base_file_slot_bytes * extrapolated_capacity + scaled_extra_node_bytes(row) * extrapolated_nodes

    def envelope_label(row: dict[str, Any]) -> str:
        name = str(row["envelope"])
        if name.startswith("LinkIAABB_S"):
            return f"LinkIAABB$_{{{row['n_subdivisions']}}}$"
        if name == "Hull16Grid":
            return "Hull16-Grid"
        return name.replace("_", "\\_")

    body_lines: list[str] = []
    for stage in ["subdivision", "grid"]:
        stage_rows = [row for row in rows if row.get("stage") == stage]
        if stage == "grid" and body_lines:
            body_lines.append("\\midrule")
        for row in stage_rows:
            delta = "--" if stage == "subdivision" else f"{float(row['voxel_delta']):.2f}"
            voxels = "--" if not row.get("voxel_count_mean") else f"{float(row['voxel_count_mean']):.0f}"
            ratio = 100.0 * float(row.get("ratio_to_linkiaabb", 0.0) or 0.0)
            body_lines.append(
                f"  {stage} & {envelope_label(row)} & {int(row['n_subdivisions'])} & {delta} & "
                f"{float(row['volume_mean']):.3f} & {float(row['time_us_mean']):.1f} & "
                f"{fmt_us_value(lect_read_us(row))} & {fmt_bytes_human(node_cache_bytes(row))} & "
                f"{fmt_duration_human(extrapolated_build_seconds(row))} & "
                f"{fmt_bytes_human(extrapolated_disk_bytes(row))} & {voxels} & {ratio:.1f}\\% \\\\"
            )

    depth_text = f"depth-{extrapolation_depth}"
    caption = (
        "Link-envelope subdivision and grid sweep, aggregated over the same "
        "width-stratified IIWA14 intervals used by Exp.~1. Warm LECT-read, "
        f"per-node cache, and {depth_text} disk estimates are anchored by the "
        "CritSample Marcucci warm-cache probe at $\\delta=0.04$ for the retained "
        "AABB and Hull16-grid representations, with grid rows scaled by the measured per-node payload size."
    )
    text = (
        "% Auto-generated from experiments/results_paper/link_envelope_pipeline.json and marcucci_envelope_build.json.\n"
        "\\begin{table*}[t]\n"
        "\\centering\n"
        f"\\caption{{{caption}}}\n"
        "\\label{tab:link_envelope_pipeline}\n"
        "\\scriptsize\n"
        "\\setlength{\\tabcolsep}{4pt}\n"
        "\\resizebox{\\textwidth}{!}{%\n"
        "\\begin{tabular}{@{}llrrrrrrrrrr@{}}\n"
        "\\toprule\n"
        "Stage & Envelope & $S$ & $\\delta$(m) & Vol. (m$^3$) & Mean time ($\\mu$s) & Warm LECT read ($\\mu$s) & Node cache & "
        f"Depth-{extrapolation_depth} build & Depth-{extrapolation_depth} disk & Voxels & Ratio "
        "\\\\\n"
        "\\midrule\n"
        f"{chr(10).join(body_lines)}\n"
        "\\bottomrule\n"
        "\\end{tabular}%\n"
        "}\n"
        "\\end{table*}\n"
    )
    out_path.write_text(text)


def write_envelope_build_table(payload: dict[str, Any], out_path: Path) -> None:
    by_key: dict[tuple[str, str], dict[str, dict[str, Any]]] = {}
    for row in payload.get("summary", []):
        key = (str(row["endpoint_source"]), str(row["envelope_key"]))
        by_key.setdefault(key, {})[str(row["cache_mode"])] = row

    rows = []
    for key in sorted(by_key):
        modes = by_key[key]
        cold = modes.get("cold", {})
        warm = modes.get("warm", {})
        reference = warm or cold
        rows.append(
            "  {endpoint} & {envelope} & {cold} & {warm} & {boxes:.0f} & {sr:.1f} & {read} \\\\".format(
                endpoint=str(reference.get("endpoint_label", key[0])),
                envelope=str(reference.get("envelope_label", key[1])),
                cold=fmt_optional(cold.get("median_build_s"), digits=2),
                warm=fmt_optional(warm.get("median_build_s"), digits=2),
                boxes=float(reference.get("median_n_boxes") or 0.0),
                sr=100.0 * float(reference.get("query_success_rate_mean") or 0.0),
                read=fmt_optional(warm.get("mean_lect_read_ms_probe"), digits=2),
            )
        )

    text = (
        "% Auto-generated from experiments/results_paper/marcucci_envelope_build.json.\n"
        "\\begin{tabular}{llrrrrr}\n"
        "  \\toprule\n"
        "Endpoint & Envelope & Cold (s) & Warm (s) & Boxes & SR & LECT read (ms) \\\\\n"
        "  \\midrule\n"
        f"{chr(10).join(rows)}\n"
        "  \\bottomrule\n"
        "\\end{tabular}\n"
    )
    out_path.write_text(text)


def macro_line(name: str, value: str) -> str:
    return f"\\providecommand{{\\{name}}}{{}}\\renewcommand{{\\{name}}}{{{value}}}\n"


def write_macros(scan: dict[str, Any], marcucci: dict[str, Any], out_path: Path) -> None:
    scan_best = scan.get("best_config") or sorted_scan_summary(scan)[0]
    paper_params = marcucci.get("params", {})
    query_medians_ms = [1000.0 * float(query["t_med_s"]) for query in marcucci.get("queries", [])]
    total_queries = int(marcucci.get("seeds", 0)) * len(marcucci.get("queries", []))
    total_successes = int(round(sum(float(query["sr"]) * int(marcucci.get("seeds", 0)) for query in marcucci.get("queries", []))))
    text = "% Auto-generated by experiments/paper/07_update_paper_results.py.\n"
    text += macro_line("sbfScanGoalBias", f"{float(paper_params.get('goal_bias', scan_best['goal_bias'])):.2f}")
    text += macro_line("sbfScanUnexplored", f"{float(paper_params.get('unexplored_sample_prob', scan_best['unexplored_sample_prob'])):.2f}")
    text += macro_line("sbfScanMaxMiss", f"{int(paper_params.get('max_consecutive_miss', scan_best['max_consecutive_miss']))}")
    text += macro_line("sbfScanSuccessRate", f"{100.0 * float(scan_best['success_rate']):.1f}")
    text += macro_line("sbfScanBuildS", f"{float(scan_best.get('build_s_mean') or 0.0):.2f}")
    text += macro_line("sbfScanQuerySumS", f"{float(scan_best.get('query_s_sum_mean') or 0.0):.3f}")
    text += macro_line("sbfScanBoxes", f"{float(scan_best.get('n_boxes_mean') or 0.0):.0f}")
    text += macro_line("sbfMarcucciVCurrentSeeds", f"{int(marcucci.get('seeds', 0))}")
    text += macro_line("sbfMarcucciVCurrentBuildMs", f"{1000.0 * float(marcucci['build']['median_s']):.0f}")
    text += macro_line("sbfMarcucciVCurrentAvgQueryMs", f"{median(query_medians_ms):.1f}")
    text += macro_line("sbfMarcucciVCurrentHardestMs", f"{max(query_medians_ms):.1f}")
    text += macro_line("sbfMarcucciVCurrentSR", f"{100.0 * total_successes / total_queries if total_queries else 0.0:.1f}")
    text += macro_line("sbfMarcucciVCurrentTotalTrials", f"{total_queries}")
    text += macro_line("sbfMarcucciVCurrentSuccesses", f"{total_successes}")
    text += macro_line("sbfMarcucciSeeds", f"{int(marcucci.get('seeds', 0))}")
    text += macro_line("sbfMarcucciBuildMs", f"{1000.0 * float(marcucci['build']['median_s']):.0f}")
    text += macro_line("sbfMarcucciAvgQueryMs", f"{median(query_medians_ms):.1f}")
    text += macro_line("sbfMarcucciNumQueries", f"{len(marcucci.get('queries', []))}")
    text += macro_line("sbfMarcucciSR", f"{100.0 * total_successes / total_queries if total_queries else 0.0:.1f}")
    text += macro_line("sbfMarcucciHardestMs", f"{max(query_medians_ms):.1f}")
    text += macro_line("sbfMarcucciTotalTrials", f"{total_queries}")
    text += macro_line("sbfMarcucciSuccesses", f"{total_successes}")
    out_path.write_text(text)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results-dir", type=Path, default=RESULTS)
    parser.add_argument("--generated-dir", type=Path, default=GENERATED)
    parser.add_argument("--scan-top-k", type=int, default=8)
    parser.add_argument("--table3-depth", type=int, default=32)
    args = parser.parse_args()

    args.generated_dir.mkdir(parents=True, exist_ok=True)
    scan = load_json(args.results_dir / "sbf_parameter_scan.json")
    marcucci = load_json(args.results_dir / "marcucci_combined.json")
    envelope_path = args.results_dir / "marcucci_envelope_build.json"
    link_envelope_path = args.results_dir / "link_envelope_pipeline.json"
    write_marcucci_table(marcucci, args.generated_dir / "tab_marcucci.tex")
    write_query_comparison_table(marcucci, args.results_dir, args.generated_dir / "tab_query.tex")
    write_scan_table(scan, args.generated_dir / "tab_sbf_parameter_scan.tex", top_k=args.scan_top_k)
    if link_envelope_path.exists() and envelope_path.exists():
        write_link_envelope_pipeline_table(
            load_json(link_envelope_path),
            load_json(envelope_path),
            args.generated_dir / "tab_link_envelope_pipeline.tex",
            extrapolation_depth=int(args.table3_depth),
        )
    if envelope_path.exists():
        write_envelope_build_table(
            load_json(envelope_path),
            args.generated_dir / "tab_marcucci_envelope_build.tex",
        )
    write_macros(scan, marcucci, args.generated_dir / "macros_sbf_v6.tex")
    print(f"[write] {args.generated_dir / 'tab_marcucci.tex'}")
    print(f"[write] {args.generated_dir / 'tab_query.tex'}")
    print(f"[write] {args.generated_dir / 'tab_sbf_parameter_scan.tex'}")
    if link_envelope_path.exists() and envelope_path.exists():
        print(f"[write] {args.generated_dir / 'tab_link_envelope_pipeline.tex'}")
    if envelope_path.exists():
        print(f"[write] {args.generated_dir / 'tab_marcucci_envelope_build.tex'}")
    print(f"[write] {args.generated_dir / 'macros_sbf_v6.tex'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

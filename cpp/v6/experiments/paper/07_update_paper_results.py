#!/usr/bin/env python3
"""Generate v6 paper LaTeX fragments from the current paper JSON results."""
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from statistics import mean, median
from typing import Any


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
RESULTS = ROOT / "experiments" / "results_paper"
DOC = ROOT / "doc" / "paper"
GENERATED = DOC / "SBF" / "generated"
LEGACY_GENERATED_LANGS = ("en", "zh")

# ---------------------------------------------------------------------------
# Data-only update helpers
# ---------------------------------------------------------------------------
_DATA_BEGIN = "% --- DATA BEGIN ---"
_DATA_END = "% --- DATA END ---"


def _write_table(out_path: Path, text: str) -> None:
    """Write *text* to *out_path*.

    If the file already exists and contains DATA BEGIN/END markers, only the
    section between those markers is replaced; the surrounding preamble
    (caption, label, column headers) and footer are left untouched.
    Otherwise the full text is written (first-time or fallback).
    """
    if out_path.exists():
        existing = out_path.read_text()
        b_idx = existing.find(_DATA_BEGIN)
        e_idx = existing.find(_DATA_END)
        if b_idx != -1 and e_idx != -1 and e_idx > b_idx:
            n_b = text.find(_DATA_BEGIN)
            n_e = text.find(_DATA_END)
            if n_b != -1 and n_e != -1:
                existing_prefix = existing[:b_idx]
                existing_suffix = existing[e_idx + len(_DATA_END):]
                new_prefix = text[:n_b]
                new_suffix = text[n_e + len(_DATA_END):]
                if existing_prefix == new_prefix and existing_suffix == new_suffix:
                    new_data = text[n_b : n_e + len(_DATA_END)]
                    out_path.write_text(
                        existing_prefix + new_data + existing_suffix
                    )
                    return
    out_path.write_text(text)


def generated_dirs(default_dir: Path) -> list[Path]:
    dirs = [default_dir]
    for lang in LEGACY_GENERATED_LANGS:
        candidate = DOC / lang / "generated"
        if candidate.exists():
            dirs.append(candidate)
    return dirs


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
        f"{_DATA_BEGIN}\n"
        f"{chr(10).join(rows)}\n"
        "  \\midrule\n"
        f"  \\multicolumn{{4}}{{l}}{{Forest build (median over {seeds} seeds): {build_ms:.0f}\\,ms}} \\\\\n"
        f"{_DATA_END}\n"
        "  \\bottomrule\n"
        "\\end{tabular}\n"
    )
    _write_table(out_path, text)


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
        f"{_DATA_BEGIN}\n"
        f"{chr(10).join(rows)}\n"
        f"{_DATA_END}\n"
        "  \\bottomrule\n"
        "\\end{tabular}\n"
    )
    _write_table(out_path, text)


def fmt_tex_sci(value: float) -> str:
    if value == 0.0:
        return "0"
    mantissa, exponent = f"{float(value):.3e}".split("e")
    mantissa = mantissa.rstrip("0").rstrip(".")
    exponent_i = int(exponent)
    return rf"{mantissa}\times10^{{{exponent_i}}}"


def fmt_gap_value(value: float) -> str:
    if abs(float(value)) < 5e-7:
        return "0"
    text = f"{float(value):.4f}".rstrip("0").rstrip(".")
    return "0" if text in {"-0", "+0"} else text


def write_epiaabb_pipeline_table(payload: dict[str, Any], out_path: Path) -> None:
    width_order = [str(item.get("width_bin", "")) for item in payload.get("width_bins", [])]
    source_order = ["IFK", "CritSample", "Analytical", "MC"]
    rows_by_key = {
        (str(row.get("width_bin", "")), str(row.get("source", ""))): row
        for row in payload.get("rows", [])
    }

    body: list[str] = []
    for width_index, width_bin in enumerate(width_order):
        for source in source_order:
            row = rows_by_key.get((width_bin, source))
            if row is None:
                continue
            body.append(
                "  {width_bin} & {source} & ${volume}$ & {time_us:.1f} & {gap} \\\\".format(
                    width_bin=width_bin,
                    source=source,
                    volume=fmt_tex_sci(float(row.get("volume_mean", 0.0))),
                    time_us=float(row.get("time_us_mean", 0.0)),
                    gap=fmt_gap_value(float(row.get("max_negative_gap", 0.0))),
                )
            )
        if width_index != len(width_order) - 1:
            body.append("\\addlinespace")

    text = (
        "% Auto-generated from experiments/results_paper/epiaabb_pipeline.json.\n"
        "\\begin{table}[t]\n"
        "\\centering\n"
        "\\caption{Endpoint-interval AABB comparison.}\n"
        "\\label{tab:epiaabb_pipeline}\n"
        "\\footnotesize\n"
        "\\setlength{\\tabcolsep}{4pt}\n"
        "\\begin{tabular}{@{}llrrr@{}}\n"
        "\\toprule\n"
        "Width bin & Source & $V$ & Mean ($\\mu$s) & Gap \\\\\n"
        "\\midrule\n"
        f"{_DATA_BEGIN}\n"
        f"{chr(10).join(body)}\n"
        f"{_DATA_END}\n"
        "\\bottomrule\n"
        "\\end{tabular}\n"
        "\\end{table}\n"
    )
    _write_table(out_path, text)


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


def sbf_payload_from_envelope_build(
    payload: dict[str, Any] | None,
    *,
    architecture_payload: dict[str, Any] | None = None,
    endpoint_source: str = "critsample",
    envelope_key: str = "aabb_s4",
    cache_mode: str = "cache_hit",
) -> dict[str, Any] | None:
    if not payload:
        return None
    rows = [
        row for row in payload.get("rows", [])
        if str(row.get("endpoint_source")) == endpoint_source
        and str(row.get("envelope_key")) == envelope_key
        and str(row.get("cache_mode")) == cache_mode
    ]
    if not rows:
        return None

    build_samples = [float(row["build_s"]) for row in rows if row.get("build_s") is not None]
    query_names = [str(query["name"]) for query in rows[0].get("queries", [])]
    query_summaries: list[dict[str, Any]] = []
    trials: list[dict[str, Any]] = []
    for row in rows:
        trials.append(
            {
                "seed": int(row.get("seed", len(trials))),
                "seed_index": int(row.get("seed", len(trials))),
                "build_s": float(row.get("build_s") or 0.0),
                "n_boxes": int(row.get("raw_box_count", row.get("n_boxes", 0)) or 0),
                "queries": [
                    {
                        "from": str(query.get("from", "")),
                        "to": str(query.get("to", "")),
                        "t_s": float(query.get("planning_time_ms", 0.0)) / 1000.0,
                        "ok": bool(query.get("ok")),
                        "length": float(query.get("path_length", 0.0)),
                        "planning_time_ms": float(query.get("planning_time_ms", 0.0)),
                    }
                    for query in row.get("queries", [])
                ],
            }
        )

    for name in query_names:
        query_rows = []
        for row in rows:
            query_rows.extend([query for query in row.get("queries", []) if str(query.get("name")) == name])
        successes = [query for query in query_rows if query.get("ok")]
        query_summaries.append(
            {
                "name": name,
                "sr": (len(successes) / len(query_rows)) if query_rows else 0.0,
                "t_med_s": stat_median([float(query.get("planning_time_ms", 0.0)) / 1000.0 for query in successes]),
                "len_med": stat_median([float(query.get("path_length", 0.0)) for query in successes]),
            }
        )

    endpoint_label_map = {"ifk": "IFK", "critsample": "CritSample"}
    envelope_label_map = {"aabb_s4": "AABB S=4", "hull16_grid_d004": r"HullGrid$_{0.04}$"}
    params = dict((architecture_payload or {}).get("params", {}))
    params.update(
        {
            "sbf_table_source": "marcucci_envelope_build",
            "endpoint_source": endpoint_label_map.get(endpoint_source.lower(), endpoint_source),
            "envelope": envelope_label_map.get(envelope_key, envelope_key),
            "cache_mode": cache_mode,
        }
    )
    return {
        "experiment": "marcucci",
        "robot": "iiwa14",
        "scene": "marcucci_combined",
        "source_protocol": "table3_cache_replay",
        "seeds": len(rows),
        "params": params,
        "build": {
            "median_s": stat_median(build_samples),
            "mean_s": (sum(build_samples) / len(build_samples)) if build_samples else None,
        },
        "queries": query_summaries,
        "trials": trials,
    }


def write_query_comparison_table(
    sbf_payload: dict[str, Any],
    sbf_ifk_aabb_payload: dict[str, Any] | None,
    results_dir: Path,
    out_path: Path,
) -> None:
    def sbf_build_without_prebridge(payload: dict[str, Any]) -> float:
        trials = payload.get("trials", [])
        if not isinstance(trials, list) or not trials:
            return float(payload["build"]["median_s"])
        adjusted = []
        for trial in trials:
            build_s = trial.get("build_s")
            if build_s is None:
                continue
            prebridge_s = float(trial.get("prebridge_time_s") or 0.0)
            adjusted.append(max(0.0, float(build_s) - prebridge_s))
        if not adjusted:
            return float(payload["build"]["median_s"])
        return float(stat_median(adjusted))

    iris_np = load_result_if_exists(results_dir, "marcucci_iris_np_gcs.json")
    ompl_prm = load_result_if_exists(
        results_dir,
        "marcucci_ompl_prm.json",
        required_params={
            "build_metric": "mean_per_seed_roadmap_build_time_across_query_runs",
            "query_metric": "second_solve_plus_ompl_simplify_after_roadmap_build",
            "prm_build_budget_s": 10.0,
            "prm_query_budget_s": 2.0,
        },
    )
    ompl_bitstar = load_result_if_exists(
        results_dir,
        "marcucci_ompl_bitstar_budget.json",
        required_params={"bitstar_budget_s": 10.0},
    )

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
    sbf_ifk_aabb_by_query = {}
    if sbf_ifk_aabb_payload:
        sbf_ifk_aabb_by_query = {
            str(query["name"]): {
                "sr": 100.0 * float(query["sr"]),
                "query_time_s_median": float(query["t_med_s"]),
                "query_path_rad_median": float(query["len_med"]),
            }
            for query in sbf_ifk_aabb_payload.get("queries", [])
        }

    sbf_ifk_build = (
        float((sbf_ifk_aabb_payload or {}).get("build", {}).get("median_s"))
        if sbf_ifk_aabb_payload and (sbf_ifk_aabb_payload.get("build") or {}).get("median_s") is not None
        else None
    )
    method_specs = [
        {
            "label": build_header(r"SBF (Crit+AABB$_{4}$)", sbf_build_without_prebridge(sbf_payload)),
            "stats": sbf_by_query,
            "columns": ["SR", "Time (s)", "Path"],
            "keys": ["sr", "query_time_s_median", "query_path_rad_median"],
        },
        {
            "label": build_header(r"SBF (IFK+AABB$_{4}$)", sbf_ifk_build),
            "stats": sbf_ifk_aabb_by_query,
            "columns": ["SR", "Time (s)", "Path"],
            "keys": ["sr", "query_time_s_median", "query_path_rad_median"],
        },
        {
            "label": build_header(r"Drake IRIS-NP+GCS", live_summary(iris_np).get("build_s_median")),
            "stats": live_query_stats(iris_np),
            "columns": ["SR", "Time (s)", "Path"],
            "keys": ["sr", "query_time_s_median", "query_path_rad_median"],
        },
        {
            "label": build_header(r"OMPL PRM", live_summary(ompl_prm).get("build_s_median")),
            "stats": live_query_stats(ompl_prm),
            "columns": ["SR", "Time (s)", "Path"],
            "keys": ["sr", "query_time_s_median", "query_path_rad_median"],
        },
        {
            "label": r"\shortstack{OMPL BIT*\\query=10\,s}",
            "stats": live_query_stats(ompl_bitstar),
            "columns": ["SR", "Path"],
            "keys": ["sr", "query_path_rad_median"],
        },
    ]

    colspec = "@{}l" + "|".join("r" * len(spec["columns"]) for spec in method_specs) + "@{}"
    group_header = " & ".join(
        [" "] + [f"\\multicolumn{{{len(spec['columns'])}}}{{c}}{{\\textbf{{\\scriptsize {spec['label']}}}}}" for spec in method_specs]
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
        "% SBF uses cached queries on the built forest; IRIS rows may use archived validated JSONs when live reruns fail.\n"
        "\\begin{table*}[t]\n"
        "\\centering\n"
        "\\caption{Shelf+IIWA baseline comparison~\\cite{marcucci2023motion}: one reusable build, five query pairs. BIT*: fixed $10$~s query budget only.}\n"
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
        f"{_DATA_BEGIN}\n"
        f"{chr(10).join(rows)}\n"
        f"{_DATA_END}\n"
        "\\bottomrule\n"
        "\\end{tabular}}\n"
        "\\end{table*}\n"
    )
    _write_table(out_path, text)


def write_exp5_cross_robot_table(payload: dict[str, Any], out_path: Path, *, caption: str) -> None:
    method_specs = [
        {"key": "sbf", "label": r"SBF (Crit+AABB$_{4}$)", "include_build": False, "build_in_header": True},
        {"key": "sbf_ifk", "label": r"SBF (IFK+AABB$_{4}$)", "include_build": False, "build_in_header": True},
        {"key": "iris_np_gcs", "label": r"Drake IRIS-NP+GCS", "include_build": False, "build_in_header": True},
        {"key": "ompl_prm", "label": r"OMPL PRM", "include_build": False, "build_in_header": True},
        {"key": "ompl_bitstar", "label": r"OMPL BIT*", "include_build": False, "hide_query": True},
    ]

    def fmt_value(value: float | None, digits: int = 3) -> str:
        if value is None:
            return "--"
        return f"{float(value):.{digits}f}"

    def fmt_sr(value: float | None) -> str:
        if value is None:
            return "--"
        return f"{100.0 * float(value):.1f}"

    aggregation_groups = list((payload.get("aggregation") or {}).get("groups", []))
    build_values_by_method: dict[str, list[float]] = {str(spec["key"]): [] for spec in method_specs}
    if aggregation_groups:
        for group in aggregation_groups:
            methods = group.get("methods") or {}
            for spec in method_specs:
                method = str(spec["key"])
                summary = methods.get(method, {})
                build = (summary.get("build_time_s") or {}).get("mean")
                if build is not None:
                    build_values_by_method[method].append(float(build))
    else:
        for scene in payload.get("scenes", []):
            summaries = {str(item.get("method")): item for item in scene.get("baseline_results", [])}
            for spec in method_specs:
                method = str(spec["key"])
                summary = summaries.get(method, {})
                build = summary.get("build_time_s_mean", summary.get("build_time_s_median"))
                if build is not None:
                    build_values_by_method[method].append(float(build))

    def build_header_value(method: str) -> str:
        values = build_values_by_method.get(str(method), [])
        if not values:
            return "--"
        return f"{sum(values) / len(values):.3f}"

    method_columns = {
        spec["key"]: (
            ["Build (s)", "Query (s)", "Path", "SR"]
            if spec["include_build"]
            else (["Path", "SR"] if spec.get("hide_query") else ["Query (s)", "Path", "SR"])
        )
        for spec in method_specs
    }
    method_labels = {
        "sbf": rf"\shortstack{{SBF (Crit+AABB$_{{4}}$)\\avg build={build_header_value('sbf')}\,s}}",
        "sbf_ifk": rf"\shortstack{{SBF (IFK+AABB$_{{4}}$)\\avg build={build_header_value('sbf_ifk')}\,s}}",
        "iris_np_gcs": rf"\shortstack{{Drake IRIS-NP+GCS\\avg build={build_header_value('iris_np_gcs')}\,s}}",
        "ompl_prm": rf"\shortstack{{OMPL PRM\\build={build_header_value('ompl_prm')}\,s}}",
        "ompl_bitstar": r"\shortstack{OMPL BIT*\\query=10\,s}",
    }

    rows = []
    robot_order = {"ur5": 0, "panda": 1}
    difficulty_order = {"easy": 0, "medium": 1, "hard": 2}
    if aggregation_groups:
        ordered_groups = sorted(
            aggregation_groups,
            key=lambda item: (
                robot_order.get(str(item.get("robot", "")).lower(), 99),
                difficulty_order.get(str(item.get("difficulty", "")).lower(), 99),
            ),
        )
        for group in ordered_groups:
            robot = str(group.get("robot", "scene")).upper()
            if robot == "PANDA":
                robot = "Panda"
            label = f"{robot}-{str(group.get('difficulty', 'medium')).capitalize()}"
            values = [label]
            methods = group.get("methods", {})
            for spec in method_specs:
                method = str(spec["key"])
                summary = methods.get(method, {})
                cells = []
                if spec["include_build"]:
                    cells.append(fmt_value((summary.get("build_time_s") or {}).get("mean")))
                if not spec.get("hide_query"):
                    cells.append(fmt_value((summary.get("query_time_s") or {}).get("mean")))
                cells.extend([
                    fmt_value((summary.get("path_length") or {}).get("mean")),
                    fmt_sr(summary.get("success_rate")),
                ])
                values.extend(cells)
            rows.append(" & ".join(values) + r" \\")
    else:
        ordered_scenes = sorted(
            payload.get("scenes", []),
            key=lambda item: (robot_order.get(str(item.get("robot", "")).lower(), 99), str(item.get("scene_id", ""))),
        )
        for scene in ordered_scenes:
            summaries = {str(item.get("method")): item for item in scene.get("baseline_results", [])}
            if not summaries:
                continue
            label = str(scene.get("robot", scene.get("scene_id", "scene"))).upper()
            if label == "PANDA":
                label = "Panda"
            difficulty = str(scene.get("difficulty", ""))
            if difficulty and difficulty != "medium":
                label = f"{label}-{difficulty.capitalize()}"
            values = [label]
            for spec in method_specs:
                method = str(spec["key"])
                summary = summaries.get(method, {})
                cells = []
                if spec["include_build"]:
                    cells.append(fmt_value(summary.get("build_time_s_mean", summary.get("build_time_s_median"))))
                if not spec.get("hide_query"):
                    cells.append(fmt_value(summary.get("query_time_s_mean", summary.get("query_time_s_median"))))
                cells.extend([
                    fmt_value(summary.get("path_length_mean", summary.get("path_length_median"))),
                    fmt_sr(summary.get("success_rate")),
                ])
                values.extend(cells)
            rows.append(" & ".join(values) + r" \\")

    row_header = "Group"
    top_row_header = " "

    group_header = " & ".join(
        [
            top_row_header,
            *[
                rf"\multicolumn{{{len(method_columns[spec['key']])}}}{{c}}{{\textbf{{\scriptsize {method_labels.get(spec['key'], spec['label'])}}}}}"
                for spec in method_specs
            ],
        ]
    )
    column_header = " & ".join([row_header] + [" & ".join(method_columns[spec["key"]]) for spec in method_specs])
    cmidrules = []
    current_col = 2
    for spec in method_specs:
        width = len(method_columns[spec["key"]])
        cmidrules.append(f"\\cmidrule(lr){{{current_col}-{current_col + width - 1}}}")
        current_col += width
    colspec = "@{}l" + "|".join("r" * len(method_columns[spec["key"]]) for spec in method_specs) + "@{}"

    text = (
        "% Auto-generated from experiments/results_paper/exp5_random_robot_scenes.json.\n"
        "\\begin{table*}[t]\n"
        "\\centering\n"
        f"\\caption{{{caption}}}\n"
        "\\label{tab:panda}\n"
        "\\scriptsize\n"
        "\\setlength{\\tabcolsep}{2.4pt}\n"
        "\\resizebox{\\textwidth}{!}{%\n"
        f"\\begin{{tabular}}{{{colspec}}}\n"
        "\\toprule\n"
        f"{group_header} \\\\\n"
        f"{''.join(cmidrules)}\n"
        f"{column_header} \\\\\n"
        "\\midrule\n"
        f"{_DATA_BEGIN}\n"
        f"{chr(10).join(rows)}\n"
        f"{_DATA_END}\n"
        "\\bottomrule\n"
        "\\end{tabular}}\n"
        "\\end{table*}\n"
    )
    _write_table(out_path, text)


def write_exp5_stats_table(payload: dict[str, Any], out_path: Path) -> None:
    is_zh = out_path.parent.parent.name == "zh"
    tests = list((payload.get("aggregation") or {}).get("paired_tests_vs_sbf", []))
    method_order = {"sbf_ifk": 0, "iris_np_gcs": 1, "ompl_prm": 2, "ompl_bitstar": 3}
    robot_order = {"ur5": 0, "panda": 1}
    difficulty_order = {"easy": 0, "medium": 1, "hard": 2}

    def fmt_p(value: Any) -> str:
        if value is None:
            return "--"
        value = float(value)
        if value < 1e-3:
            return "$<10^{-3}$"
        return f"{value:.3f}"

    rows = []
    for row in sorted(
        tests,
        key=lambda item: (
            robot_order.get(str(item.get("robot", "")).lower(), 99),
            difficulty_order.get(str(item.get("difficulty", "")).lower(), 99),
            method_order.get(str(item.get("method", "")), 99),
        ),
    ):
        robot = str(row.get("robot", "scene")).upper()
        if robot == "PANDA":
            robot = "Panda"
        group = f"{robot}-{str(row.get('difficulty', 'medium')).capitalize()}"
        query = row.get("query_time_method_minus_sbf") or {}
        path = row.get("path_length_method_minus_sbf") or {}
        success = row.get("success_method_vs_sbf") or {}
        rows.append(
            " & ".join(
                [
                    group,
                    str(row.get("label", row.get("method", ""))),
                    f"{float(query.get('mean_delta') or 0.0):.3f}",
                    fmt_p(query.get("p_two_sided")),
                    f"{float(path.get('mean_delta') or 0.0):.3f}",
                    fmt_p(path.get("p_two_sided")),
                    fmt_p(success.get("p_two_sided")),
                ]
            )
            + r" \\"
        )

    caption = "Paired baselines vs.\\ SBF (matched scene and seed cohorts): $\\Delta t_q$ and $\\Delta$Path are paired mean differences versus SBF query time/path; $p$ columns are paired two-sided permutation $p$-values ($p_q$, $p_{path}$, success-rate $p_{SR}$)."
    if is_zh:
        caption = "与相同场景/seed 对齐的 baseline 配对检验：数值列为相对 SBF 的查询时间误差、路径长度误差及配对双侧 $p$ 值。"
    text = (
        "% Auto-generated from experiments/results_paper/exp5_random_robot_scenes.json.\n"
        "\\begin{table*}[t]\n"
        "\\centering\n"
        f"\\caption{{{caption}}}\n"
        "\\label{tab:exp5_stats}\n"
        "\\scriptsize\n"
        "\\setlength{\\tabcolsep}{4pt}\n"
        "\\resizebox{\\textwidth}{!}{%\n"
        "\\begin{tabular}{@{}llrrrrr@{}}\n"
        "\\toprule\n"
        "Group & Baseline & $\\Delta t_q$ (s) & $p_q$ & $\\Delta$Path & $p_{path}$ & $p_{SR}$ \\\\\n"
        "\\midrule\n"
        f"{_DATA_BEGIN}\n"
        f"{chr(10).join(rows)}\n"
        f"{_DATA_END}\n"
        "\\bottomrule\n"
        "\\end{tabular}}\n"
        "\\end{table*}\n"
    )
    _write_table(out_path, text)


def write_exp6_rebuild_table(payload: dict[str, Any], out_path: Path, *, caption: str) -> None:
    robot_order = {"ur5": 0, "panda": 1}
    difficulty_order = {"easy": 0, "medium": 1, "hard": 2}

    def fmt_s(summary: dict[str, Any], key: str = "median") -> str:
        value = summary.get(key)
        if value is None:
            return "--"
        return f"{float(value):.3f}"

    groups = sorted(
        list((payload.get("aggregation") or {}).get("groups", [])),
        key=lambda item: (
            robot_order.get(str(item.get("robot", "")).lower(), 99),
            difficulty_order.get(str(item.get("difficulty", "")).lower(), 99),
        ),
    )
    rows = []
    for group in groups:
        robot = str(group.get("robot", "scene")).upper()
        if robot == "PANDA":
            robot = "Panda"
        label = f"{robot}-{str(group.get('difficulty', 'medium')).capitalize()}"
        build = group.get("build_time_s") or {}
        rebuild = group.get("rebuild_time_s") or {}
        build_median = build.get("median")
        rebuild_median = rebuild.get("median")
        speedup = "--"
        if build_median is not None and rebuild_median is not None and float(rebuild_median) > 0.0:
            speedup = f"{float(build_median) / float(rebuild_median):.1f}$\\times$"
        rows.append("  {} & {} & {} {}".format(label, fmt_s(rebuild), speedup, r"\\"))
    if not rows:
        rows.append("  -- & -- & -- " + r"\\")

    text = (
        "% Auto-generated from experiments/results_paper/exp6_sbf_obstacle_rebuild.json.\n"
        "\\begin{table}[t]\n"
        "\\centering\n"
        f"\\caption{{{caption}}}\n"
        "\\label{tab:exp6_rebuild}\n"
        "\\scriptsize\n"
        "\\begin{tabular}{lrr}\n"
        "  \\toprule\n"
        "  Group & Rebuild (s) & Speedup \\\\\n"
        "  \\midrule\n"
        f"{_DATA_BEGIN}\n"
        f"{chr(10).join(rows)}\n"
        f"{_DATA_END}\n"
        "  \\bottomrule\n"
        "\\end{tabular}\n"
        "\\end{table}\n"
    )
    _write_table(out_path, text)


def fmt_us_value(value: float | None) -> str:
    if value is None:
        return "--"
    value = float(value)
    if value < 0.1:
        return f"{value:.3f}"
    if value < 100.0:
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
    replay_read: dict[str, Any],
    out_path: Path,
    *,
    extrapolation_depth: int,
    calibration: dict[str, Any],
) -> None:
    rows = [
        row for row in payload["rows"]
        if str(row.get("type")) != "LinkIAABB_Grid"
    ]
    extrapolated_nodes = (1 << (extrapolation_depth + 1)) - 1
    extrapolated_capacity = extrapolated_nodes + max(extrapolated_nodes // 4, 4096)
    extrapolated_capacity += extrapolated_capacity & 1

    compact_base_node_bytes = float(payload.get("storage_model", {}).get("optimized_base_node_bytes", 64.0) or 64.0)

    def normalized_endpoint(value: Any) -> str:
        text = str(value or "").strip()
        if not text:
            return ""
        t = text.lower()
        if t in {"ifk", "endpointsource.ifk"}:
            return "IFK"
        if t in {"critsample", "crit", "endpointsource.critsample"}:
            return "Crit"
        return text

    calibration_threads = int(calibration.get("threads", 0) or 0)
    if calibration_threads != 8:
        raise ValueError(f"Exp.2 D32 calibration must use 8 threads, got {calibration_threads}.")

    calibration_by_key = {}
    for item in calibration.get("summaries", []):
        key = (
            normalized_endpoint(item.get("endpoint_source")),
            str(item.get("type", "")),
            int(item.get("n_subdivisions", 0) or 0),
            round(float(item.get("voxel_delta", 0.0) or 0.0), 6),
        )
        calibration_by_key[key] = item

    def row_calibration(row: dict[str, Any]) -> dict[str, Any]:
        key = (
            normalized_endpoint(row.get("endpoint_source")),
            str(row.get("type", "")),
            int(row.get("n_subdivisions", 0) or 0),
            round(float(row.get("voxel_delta", 0.0) or 0.0), 6),
        )
        hit = calibration_by_key.get(key)
        if hit is None:
            raise FileNotFoundError(
                f"Missing 8-thread LECT envelope fill calibration at key={key}."
            )
        return hit

    def node_cache_bytes(row: dict[str, Any]) -> float:
        row_type = str(row.get("type"))
        if row_type == "Hull16_Grid":
            optimized_payload = row.get("storage_bytes_optimized_mean")
            if optimized_payload is not None:
                return compact_base_node_bytes + max(0.0, float(optimized_payload))
            return compact_base_node_bytes
        return compact_base_node_bytes

    replay_read_by_key: dict[tuple[str, int, float], list[float]] = {}
    for item in replay_read.get("rows", []):
        value = item.get("cache_hit_read_us_per_call")
        if value is None:
            continue
        key = (
            str(item.get("type", "")),
            int(item.get("n_subdivisions", 0) or 0),
            round(float(item.get("voxel_delta", 0.0) or 0.0), 6),
        )
        replay_read_by_key.setdefault(key, []).append(float(value))

    def cache_hit_replay_us_per_expand(row: dict[str, Any]) -> float | None:
        key = (
            str(row.get("type", "")),
            int(row.get("n_subdivisions", 0) or 0),
            round(float(row.get("voxel_delta", 0.0) or 0.0), 6),
        )
        values = replay_read_by_key.get(key, [])
        if not values:
            raise FileNotFoundError(
                f"Missing envelope-only replay experiment value at key={key}; Replay must come from LECT replay experiments."
            )
        return float(median(values))

    def extrapolated_build_seconds(row: dict[str, Any]) -> float:
        calib = row_calibration(row)
        estimate = calib.get("d32_fill_time_s", calib.get("depth_build_s_estimate"))
        if estimate is None:
            fill_nodes_per_second = calib.get("fill_nodes_per_second")
            if fill_nodes_per_second:
                estimate = float(extrapolated_nodes) / float(fill_nodes_per_second)
        if estimate is None or float(estimate) <= 0.0:
            raise ValueError(
                "Invalid LECT envelope fill calibration; expected positive d32_fill_time_s."
            )
        return float(estimate)

    def extrapolated_disk_bytes(row: dict[str, Any]) -> float:
        return node_cache_bytes(row) * extrapolated_capacity

    def envelope_label(row: dict[str, Any]) -> str:
        endpoint = normalized_endpoint(row.get("endpoint_source"))
        row_type = str(row.get("type"))
        subdivisions = int(row.get("n_subdivisions", 0) or 0)
        endpoint_prefix = f"{endpoint}+" if endpoint else ""
        if row_type == "LinkIAABB":
            base = "LinkIAABB" if subdivisions <= 1 else rf"LinkIAABB$_{{{subdivisions}}}$"
            return f"{endpoint_prefix}{base}"
        if row_type == "Hull16_Grid":
            delta = float(row.get("voxel_delta", 0.0) or 0.0)
            return f"{endpoint_prefix}HullGrid$_{{{delta:.2f}}}$"
        return f"{endpoint_prefix}{row.get('envelope', row_type)}"

    def delta_label(row: dict[str, Any]) -> str:
        if str(row.get("type")) == "LinkIAABB":
            return "--"
        return f"{float(row.get('voxel_delta', 0.0)):.2f}"

    def voxels_label(row: dict[str, Any]) -> str:
        voxels = float(row.get("voxel_count_mean", 0.0) or 0.0)
        return "--" if voxels <= 0.0 else f"{voxels:.0f}"

    body: list[str] = []
    previous_stage: str | None = None
    for row in rows:
        stage = str(row.get("stage", ""))
        if previous_stage is not None and stage != previous_stage:
            body.append("\\midrule")
        previous_stage = stage
        grow_us = cache_hit_replay_us_per_expand(row)
        body.append(
            "  {envelope} & {subdivisions} & {delta} & {volume:.3f} & {time} & {grow} & {node_cache} & {depth_build} & {depth_disk} & {voxels} & {ratio} \\\\".format(
                envelope=envelope_label(row),
                subdivisions=int(row.get("n_subdivisions", 0) or 0),
                delta=delta_label(row),
                volume=float(row.get("volume_mean", 0.0) or 0.0),
                time=fmt_us_value(float(row.get("time_us_mean", 0.0) or 0.0)),
                grow=fmt_us_value(grow_us),
                node_cache=fmt_bytes_human(node_cache_bytes(row)),
                depth_build=fmt_duration_human(extrapolated_build_seconds(row)),
                depth_disk=fmt_bytes_human(extrapolated_disk_bytes(row)),
                voxels=voxels_label(row),
                ratio=fmt_percent(float(row.get("ratio_to_linkiaabb", 0.0) or 0.0)),
            )
        )

    text = (
        "% Auto-generated from experiments/results_paper/link_envelope_pipeline.json and marcucci_envelope_build.json.\n"
        "\\begin{table*}[t]\n"
        "\\centering\n"
        "\\caption{Link-interval envelope comparison and LECT parameter evaluation.}\n"
        "\\label{tab:link_envelope_pipeline}\n"
        "\\scriptsize\n"
        "\\setlength{\\tabcolsep}{4pt}\n"
        "\\resizebox{\\textwidth}{!}{%\n"
        "\\begin{tabular}{@{}lrrrrrrrrrr@{}}\n"
        "\\toprule\n"
        "Group & $S$ & $\\delta$ (m) & Vol. & $t_{\\mathrm{eval}}$ ($\\mu$s) & $t_{\\mathrm{replay}}$ ($\\mu$s) & Cache/node & D32 time & D32 disk & Vox. & Ratio \\\\"
        "\n"
        "\\midrule\n"
        f"{_DATA_BEGIN}\n"
        f"{chr(10).join(body)}\n"
        f"{_DATA_END}\n"
        "\\bottomrule\n"
        "\\end{tabular}%\n"
        "}\n"
        "\\end{table*}\n"
    )
    _write_table(out_path, text)


def write_envelope_build_table(payload: dict[str, Any], out_path: Path) -> None:
    latex_newline = "\\\\"
    if payload.get("schema_version") == 2 and payload.get("comparisons"):
        # Marcucci scene uses IIWA14 (7 DoF); report per-box average edge length
        # by converting mean box volume to geometric edge length in 7D.
        config_dim = 7

        def normalized_endpoint(value: Any) -> str:
            text = str(value or "").strip()
            t = text.lower()
            if t in {"critsample", "crit", "endpointsource.critsample"}:
                return "Crit"
            if t in {"ifk", "endpointsource.ifk"}:
                return "IFK"
            return text

        def normalized_group(item: dict[str, Any]) -> str:
            endpoint = normalized_endpoint(item.get("endpoint_label", item.get("endpoint_source")))
            envelope_key = str(item.get("envelope_key", "")).lower()
            if envelope_key == "aabb_s4":
                envelope = r"LinkIAABB$_{4}$"
            elif "hull16_grid" in envelope_key:
                match = re.search(r"d(\d+)", envelope_key)
                if match:
                    token = match.group(1)
                    if len(token) >= 2:
                        delta_value = f"{token[0]}.{token[1:]}"
                    else:
                        delta_value = token
                    envelope = rf"HullGrid$_{{{delta_value}}}$"
                else:
                    envelope = r"HullGrid$_{0.04}$"
            else:
                envelope = str(item.get("envelope_label", item.get("envelope_key", "")))
                envelope = envelope.replace("Hull16-grid", r"HullGrid$_{0.04}$")
            return f"{endpoint}+{envelope}" if endpoint else envelope

        summary_by_key = {
            (str(row.get("endpoint_source")), str(row.get("envelope_key")), str(row.get("cache_mode"))): row
            for row in payload.get("summary", [])
        }

        def box_volume_sum(item: dict[str, Any]) -> float:
            direct = item.get("median_box_volume_sum")
            if direct is not None:
                return float(direct)
            key = (str(item.get("endpoint_source")), str(item.get("envelope_key")), "cache_hit")
            summary = summary_by_key.get(key, {})
            return float(summary.get("median_box_volume_sum", summary.get("median_dedup_box_volume_sum", 0.0)) or 0.0)

        rows = []
        for item in sorted(payload.get("comparisons", []), key=lambda row: (str(row["endpoint_source"]), str(row["envelope_key"]))):
            speedup = item.get("total_build_speedup")
            volume_sum = box_volume_sum(item)
            n_boxes = float(item.get("median_n_boxes") or item.get("median_raw_box_count") or 0.0)
            avg_edge = (volume_sum / n_boxes) ** (1.0 / float(config_dim)) if n_boxes > 0.0 and volume_sum > 0.0 else None
            rows.append(
                "  {group} & {no_cache:.2f} & {cache_hit:.2f} & {speedup} & {boxes:.0f} & {avg_edge} {nl}".format(
                    group=normalized_group(item),
                    no_cache=float(item.get("no_cache_median_build_s") or 0.0),
                    cache_hit=float(item.get("cache_hit_median_build_s") or 0.0),
                    speedup="--" if speedup is None else f"{float(speedup):.2f}$\\times$",
                    boxes=n_boxes,
                    avg_edge="--" if avg_edge is None else f"{avg_edge:.4f}",
                    nl=latex_newline,
                )
            )

        lines = [
            "% Auto-generated from experiments/results_paper/marcucci_envelope_build.json.",
            "\\begin{tabular}{lrrrrr}",
            "  \\toprule",
            f"Group & Cold (s) & Warm (s) & Speedup & Boxes & $\\bar{{\\ell}}_{{\\mathrm{{box}}}}$ {latex_newline}",
            "  \\midrule",
            _DATA_BEGIN,
            *rows,
            _DATA_END,
            "  \\bottomrule",
            "\\end{tabular}",
        ]
        _write_table(out_path, "\n".join(lines) + "\n")
        return

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
            "  {endpoint} & {envelope} & {cold} & {warm} & {boxes:.0f} & {sr:.1f} & {read} {nl}".format(
                endpoint=str(reference.get("endpoint_label", key[0])),
                envelope=str(reference.get("envelope_label", key[1])),
                cold=fmt_optional(cold.get("median_build_s"), digits=2),
                warm=fmt_optional(warm.get("median_build_s"), digits=2),
                boxes=float(reference.get("median_n_boxes") or 0.0),
                sr=100.0 * float(reference.get("query_success_rate_mean") or 0.0),
                read=fmt_optional(warm.get("mean_lect_read_ms_probe"), digits=2),
                nl=latex_newline,
            )
        )
    lines = [
        "% Auto-generated from experiments/results_paper/marcucci_envelope_build.json.",
        "\\begin{tabular}{llrrrrr}",
        "  \\toprule",
        f"Endpoint & Envelope & Cold (s) & Warm (s) & Boxes & SR & LECT read (ms) {latex_newline}",
        "  \\midrule",
        _DATA_BEGIN,
        *rows,
        _DATA_END,
        "  \\bottomrule",
        "\\end{tabular}",
    ]
    _write_table(out_path, "\n".join(lines) + "\n")


def write_build_ablation_table(payload: dict[str, Any], out_path: Path) -> None:
    latex_newline = "\\\\"
    labels = {
        "baseline": "Baseline",
        "no_coarsen": "No consolidation",
        "no_unexplored": "No unexplored sampling",
        "no_seed_bridge": "No seed connector",
        "no_rescue_bridge": "No rescue connector",
        "force_bridge": "Connector stress",
        "ffb_depth_80": "Lower depth cap",
        "ffb_depth_200": "Higher depth cap",
        "parallel_partitioned": "Partitioned grower",
        "parallel_legacy": "Legacy independent grower",
    }
    by_key = {str(item.get("case", {}).get("key")): item for item in payload.get("results", [])}
    rows = []
    for key, label in labels.items():
        item = by_key.get(key)
        if not item:
            continue
        summary = item.get("summary", {})
        bridge_plus = float(summary.get("bridge_ms_median", 0.0) or 0.0) + float(summary.get("coarsen2_ms_median", 0.0) or 0.0)
        rows.append(
            "  {label} & {build:.0f} & {grow:.0f} & {seed:.0f} & {bridge:.0f} & {boxes:.0f} & {sr:.0f} & {path:.3f} {nl}".format(
                label=label,
                build=float(summary.get("total_ms_median", 0.0) or 0.0),
                grow=float(summary.get("grow_ms_median", 0.0) or 0.0),
                seed=float(summary.get("seed_bridge_ms_median", 0.0) or 0.0),
                bridge=bridge_plus,
                boxes=float(summary.get("boxes_final_median", 0.0) or 0.0),
                sr=100.0 * float(summary.get("query_success_rate", 0.0) or 0.0),
                path=float(summary.get("query_mean_length_median", 0.0) or 0.0),
                nl=latex_newline,
            )
        )
    lines = [
        "% Auto-generated from experiments/results_paper/build_ablation_sweep.json.",
        "\\begin{tabular}{lrrrrrrr}",
        "  \\toprule",
        f"Setting & Build (ms) & Grow & Seed conn. & Bridge+merge & Boxes & SR (\\%) & Path {latex_newline}",
        "  \\midrule",
        _DATA_BEGIN,
        *rows,
        _DATA_END,
        "  \\bottomrule",
        "\\end{tabular}",
    ]
    _write_table(out_path, "\n".join(lines) + "\n")


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
    epiaabb_path = args.results_dir / "epiaabb_pipeline.json"
    envelope_path = args.results_dir / "marcucci_envelope_build.json"
    link_envelope_path = args.results_dir / "link_envelope_pipeline.json"
    link_growth_path = args.results_dir / "link_envelope_growth_calibration.json"
    link_replay_read_path = args.results_dir / "link_envelope_replay_read.json"
    exp5_path = args.results_dir / "exp5_random_robot_scenes.json"
    exp6_rebuild_path = args.results_dir / "exp6_sbf_obstacle_rebuild.json"

    envelope_payload = load_json(envelope_path) if envelope_path.exists() else None
    sbf_for_query_table = marcucci
    sbf_ifk_aabb_for_query_table = sbf_payload_from_envelope_build(
        envelope_payload,
        architecture_payload=marcucci,
        endpoint_source="ifk",
        envelope_key="aabb_s4",
        cache_mode="cache_hit",
    )

    write_marcucci_table(marcucci, args.generated_dir / "tab_marcucci.tex")
    write_query_comparison_table(
        sbf_for_query_table,
        sbf_ifk_aabb_for_query_table,
        args.results_dir,
        args.generated_dir / "tab_query.tex",
    )
    if epiaabb_path.exists():
        write_epiaabb_pipeline_table(
            load_json(epiaabb_path),
            args.generated_dir / "tab_epiaabb_pipeline.tex",
        )
    if link_envelope_path.exists() and envelope_path.exists():
        write_link_envelope_pipeline_table(
            load_json(link_envelope_path),
            load_json(envelope_path),
            load_json(link_replay_read_path),
            args.generated_dir / "tab_link_envelope_pipeline.tex",
            extrapolation_depth=int(args.table3_depth),
            calibration=load_json(link_growth_path),
        )
    if envelope_path.exists():
        write_envelope_build_table(
            load_json(envelope_path),
            args.generated_dir / "tab_marcucci_envelope_build.tex",
        )
    if exp5_path.exists():
        caption = "UR5/Panda randomized-scene baseline comparison."
        if args.generated_dir.parent.name == "zh":
            caption = "与货架表格相同的缓存查询口径：各行汇总随机场景与规划器种子下某机器人$\times$难度组的中位数。列为可复用构建/查询耗时（s）、成功路径均值及成功率（\\%）；``--'' 表示 IRIS+GCS 无解。BIT* 仅 $10$~s query 且无构建列。"
        write_exp5_cross_robot_table(
            load_json(exp5_path),
            args.generated_dir / "tab_panda.tex",
            caption=caption,
        )
    write_macros(scan, marcucci, args.generated_dir / "macros_sbf_v6.tex")
    print(f"[write] {args.generated_dir / 'tab_marcucci.tex'}")
    print(f"[write] {args.generated_dir / 'tab_query.tex'}")
    if epiaabb_path.exists():
        print(f"[write] {args.generated_dir / 'tab_epiaabb_pipeline.tex'}")
    if link_envelope_path.exists() and envelope_path.exists():
        print(f"[write] {args.generated_dir / 'tab_link_envelope_pipeline.tex'}")
    if envelope_path.exists():
        print(f"[write] {args.generated_dir / 'tab_marcucci_envelope_build.tex'}")
    if exp5_path.exists():
        print(f"[write] {args.generated_dir / 'tab_panda.tex'}")
    if exp6_rebuild_path.exists():
        _payload_exp6 = load_json(exp6_rebuild_path)
        for gdir in generated_dirs(args.generated_dir):
            gdir.mkdir(parents=True, exist_ok=True)
            caption = "SBF reconstruction results under obstacle addition."
            if gdir.parent.name == "zh":
                caption = "单障碍插入后的增量重建（CritSample 端点、LinkIAABB $S{=}4$）：剔除与障碍物相交的包络盒并重连邻接。与 \\cref{tab:panda} 同分布随机场景组的中位数结果；Rebuild（s）、Speedup（相对全流程 regrowth）。"
            write_exp6_rebuild_table(_payload_exp6, gdir / "tab_exp6_rebuild.tex", caption=caption)
            print(f"[write] {gdir / 'tab_exp6_rebuild.tex'}")
    build_ablation_path = args.results_dir / "build_ablation_sweep.json"
    if build_ablation_path.exists():
        _payload_ba = load_json(build_ablation_path)
        for gdir in generated_dirs(args.generated_dir):
            gdir.mkdir(parents=True, exist_ok=True)
            write_build_ablation_table(_payload_ba, gdir / "tab_build_ablation.tex")
            print(f"[write] {gdir / 'tab_build_ablation.tex'}")
    print(f"[write] {args.generated_dir / 'macros_sbf_v6.tex'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

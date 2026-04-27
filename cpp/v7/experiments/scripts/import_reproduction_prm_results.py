#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import pickle
import sys
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from common import MARCUCCI_QUERY_FILES, RESULTS_PAPER, aggregate_method_trials, write_json


LABEL_ALIASES = {
    "BS->LB": "CS->LB",
    "RB->TS": "RB->AS",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--field", default=None,
                        help="optional top-level field to extract, e.g. prm_data or sprm_data")
    parser.add_argument("--method", default="prm_reproduction")
    parser.add_argument("--build-s", type=float, default=None,
                        help="optional roadmap/precomputation time in seconds")
    parser.add_argument("--out", type=Path, default=RESULTS_PAPER / "marcucci_prm_gcs.json")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def load_payload(path: Path) -> dict[str, Any]:
    if path.suffix.lower() in {".pkl", ".pickle"}:
        with path.open("rb") as handle:
            return pickle.load(handle)
    return json.loads(path.read_text())


def canonical_label(label: str) -> str:
    return LABEL_ALIASES.get(label, label)


def pick_result_dict(payload: dict[str, Any], field: str | None) -> dict[str, Any]:
    data = payload[field] if field else payload
    if not isinstance(data, dict):
        raise TypeError("selected payload must be a dict")
    return data


def validate_and_order(data: dict[str, Any]) -> list[tuple[str, str, dict[str, Any]]]:
    by_label = {canonical_label(label): value for label, value in data.items()}
    ordered: list[tuple[str, str, dict[str, Any]]] = []
    missing: list[str] = []
    for label, filename in MARCUCCI_QUERY_FILES:
        item = by_label.get(label)
        if item is None:
            missing.append(label)
            continue
        ordered.append((label, filename, item))
    if missing:
        raise KeyError(f"missing canonical reproduction tasks: {missing}")
    return ordered


def as_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(out) or math.isinf(out):
        return None
    return out


def build_seed_trials(
    ordered: list[tuple[str, str, dict[str, Any]]],
    *,
    build_s: float | None,
) -> list[dict[str, Any]]:
    lengths = []
    for label, _, item in ordered:
        paths = item.get("Path Length (rad)")
        times = item.get("Time (ms)")
        if not isinstance(paths, list) or not isinstance(times, list):
            raise TypeError(f"task {label} must contain list-valued Path Length (rad) and Time (ms)")
        if len(paths) != len(times):
            raise ValueError(f"task {label} has mismatched sample counts")
        lengths.append(len(paths))
    if not lengths:
        return []
    if len(set(lengths)) != 1:
        raise ValueError(f"task sample counts differ: {lengths}")

    seed_trials: list[dict[str, Any]] = []
    n_samples = lengths[0]
    for sample_idx in range(n_samples):
        trial = {
            "seed": sample_idx,
            "queries": [],
        }
        if build_s is not None:
            trial["build_s"] = float(build_s)
        for label, filename, item in ordered:
            path_length = as_float(item["Path Length (rad)"][sample_idx])
            time_ms = as_float(item["Time (ms)"][sample_idx])
            if path_length is None or time_ms is None:
                trial["queries"].append(
                    {
                        "query": label,
                        "file": filename,
                        "seed": sample_idx,
                        "success": False,
                        "time_s": None,
                        "path_length": None,
                        "note": "missing reproduction sample",
                    }
                )
                continue
            trial["queries"].append(
                {
                    "query": label,
                    "file": filename,
                    "seed": sample_idx,
                    "success": True,
                    "time_s": time_ms / 1000.0,
                    "path_length": path_length,
                }
            )
        seed_trials.append(trial)
    return seed_trials


def main() -> int:
    args = parse_args()
    payload = load_payload(args.input)
    data = pick_result_dict(payload, args.field)
    ordered = validate_and_order(data)
    seed_trials = build_seed_trials(ordered, build_s=args.build_s)
    imported = aggregate_method_trials(
        method=args.method,
        scene="iiwa14_marcucci_combined",
        quick=False,
        seeds=len(seed_trials),
        params={
            "source": "gcs-science-robotics/reproduction/prm_comparison",
            "input": str(args.input),
            "field": args.field,
            "imported_from_reproduction": True,
            "build_s": args.build_s,
        },
        seed_trials=seed_trials,
    )

    if args.dry_run:
        print(json.dumps(imported, indent=2))
        return 0

    write_json(args.out, imported)
    print(f"[import_reproduction_prm_results] wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
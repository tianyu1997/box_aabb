from __future__ import annotations

import argparse
import json
import math
from collections import Counter, defaultdict
from pathlib import Path
from typing import Dict, Iterable, List, Tuple


def _load_boxes(path: Path) -> List[dict]:
    return json.loads(path.read_text(encoding="utf-8"))


def _root_widths(boxes: Iterable[dict]) -> List[float]:
    box_list = list(boxes)
    widths = []
    for dim in range(2):
        lo = min(box["joint_intervals"][dim]["lo"] for box in box_list)
        hi = max(box["joint_intervals"][dim]["hi"] for box in box_list)
        widths.append(float(hi - lo))
    return widths


def _level(root_width: float, width: float) -> int:
    if width <= 0.0:
        raise ValueError("Interval width must be positive")
    return int(round(math.log(root_width / width, 2)))


def _analyze_boxes(boxes: List[dict], label: str, path: Path) -> Dict[str, object]:
    root_widths = _root_widths(boxes)
    dims: List[Dict[str, object]] = []
    joint_level_matrix: Counter[Tuple[int, int]] = Counter()

    for dim in range(2):
        level_counts: Counter[int] = Counter()
        level_unique_intervals: defaultdict[int, set[Tuple[float, float]]] = defaultdict(set)
        level_widths: defaultdict[int, set[float]] = defaultdict(set)

        for box in boxes:
            interval = box["joint_intervals"][dim]
            width = float(interval["hi"] - interval["lo"])
            level = _level(root_widths[dim], width)
            level_counts[level] += 1
            level_unique_intervals[level].add((round(float(interval["lo"]), 9), round(float(interval["hi"]), 9)))
            level_widths[level].add(round(width, 9))

        dims.append(
            {
                "dim": f"q{dim}",
                "root_width": root_widths[dim],
                "distinct_intervals": len(
                    {
                        (
                            round(float(box["joint_intervals"][dim]["lo"]), 9),
                            round(float(box["joint_intervals"][dim]["hi"]), 9),
                        )
                        for box in boxes
                    }
                ),
                "boxes_by_level": {str(level): count for level, count in sorted(level_counts.items())},
                "unique_intervals_by_level": {
                    str(level): len(intervals) for level, intervals in sorted(level_unique_intervals.items())
                },
                "widths_by_level": {
                    str(level): sorted(widths) for level, widths in sorted(level_widths.items())
                },
            }
        )

    for box in boxes:
        levels = []
        for dim in range(2):
            interval = box["joint_intervals"][dim]
            width = float(interval["hi"] - interval["lo"])
            levels.append(_level(root_widths[dim], width))
        joint_level_matrix[(levels[0], levels[1])] += 1

    return {
        "label": label,
        "path": str(path).replace("\\", "/"),
        "n_boxes": len(boxes),
        "dims": dims,
        "joint_level_matrix": {
            f"{level_q0},{level_q1}": count
            for (level_q0, level_q1), count in sorted(joint_level_matrix.items())
        },
    }


def _format_report(report: Dict[str, object]) -> str:
    lines: List[str] = []
    for run in report["runs"]:
        lines.append(f"=== {run['label']} ===")
        lines.append(f"path={run['path']}")
        lines.append(f"n_boxes={run['n_boxes']}")
        for dim in run["dims"]:
            lines.append(
                f"{dim['dim']}: distinct_intervals={dim['distinct_intervals']} root_width={dim['root_width']:.6f}"
            )
            lines.append(f"  boxes_by_level={dim['boxes_by_level']}")
            lines.append(f"  unique_intervals_by_level={dim['unique_intervals_by_level']}")
        lines.append(f"joint_level_matrix={run['joint_level_matrix']}")
        lines.append("")
    return "\n".join(lines).rstrip() + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compare 2DOF boxes.json files by q0/q1 split level distributions."
    )
    parser.add_argument("pre", type=Path, help="Path to the baseline boxes.json")
    parser.add_argument("post", type=Path, help="Path to the comparison boxes.json")
    parser.add_argument("--output-json", type=Path, default=None, help="Optional JSON report path")
    parser.add_argument("--output-txt", type=Path, default=None, help="Optional text summary path")
    parser.add_argument("--pre-label", type=str, default="pre_fix", help="Label for the baseline run")
    parser.add_argument("--post-label", type=str, default="post_fix", help="Label for the comparison run")
    args = parser.parse_args()

    runs = [
        _analyze_boxes(_load_boxes(args.pre), args.pre_label, args.pre.resolve()),
        _analyze_boxes(_load_boxes(args.post), args.post_label, args.post.resolve()),
    ]
    report = {"runs": runs}
    text = _format_report(report)

    if args.output_json is not None:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(report, indent=2), encoding="utf-8")
    if args.output_txt is not None:
        args.output_txt.parent.mkdir(parents=True, exist_ok=True)
        args.output_txt.write_text(text, encoding="utf-8")

    print(text, end="")


if __name__ == "__main__":
    main()
#!/usr/bin/env bash
# run_compare_all.sh — Run all planners on the v7 quick scenes and emit
# the v6-vs-v7 comparison report.
#
# Output: cpp/v7/experiments/results_nightly/compare/compare_report.md
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD="${BUILD:-$ROOT/build}"
OUT="${OUT:-$ROOT/experiments/results_nightly/compare}"
CFG="$ROOT/experiments/configs"
SCRIPTS="$ROOT/scripts"
PYBIN="${PYBIN:-/home/tian/miniconda3/envs/sbf/bin/python}"

mkdir -p "$OUT"
SCENES=(2dof_box iiwa14_far)

for s in "${SCENES[@]}"; do
    echo "── v7 SBF: $s ──"
    "$BUILD/experiments/exp_main" --scene="$CFG/$s.json" \
        --out="$OUT/main_$s.json" --quick

    echo "── v7 OMPL RRT-Connect: $s ──"
    "$BUILD/experiments/baseline_ompl" --scene="$CFG/$s.json" \
        --out="$OUT/ompl_$s.json" --quick

    echo "── Drake GCS: $s ──"
    "$PYBIN" "$SCRIPTS/baseline_drake_gcs.py" --scene "$CFG/$s.json" \
        --out "$OUT/drake_$s.json" --quick
done

echo
echo "── compare_planners.py ──"
"$PYBIN" "$SCRIPTS/compare_planners.py" \
    --results-dir "$OUT" \
    --scenes "${SCENES[@]}" \
    --v6-build-ms 444 \
    --v6-per-box-ms 0.32 \
    --out "$OUT/compare_report.md"

echo
echo "Wrote $OUT/compare_report.md"
cat "$OUT/compare_report.md"

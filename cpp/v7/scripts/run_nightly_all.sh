#!/usr/bin/env bash
# run_nightly_all.sh — Auxiliary planner-regression nightly driver.
#
# This script keeps the legacy planner/baseline nightly checks separate from the
# paper experiment drivers under `experiments/paper/`. Endpoint-source and
# link-envelope profiling are part of the paper suite and are not launched here.
#
# Usage:  scripts/run_nightly_all.sh [<build_dir>] [<out_dir>]
# Defaults: build_dir=cpp/v7/build, out_dir=cpp/v7/experiments/results_nightly/full
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD="${1:-$ROOT/build}"
OUT="${2:-$ROOT/experiments/results_nightly/full}"
CFG="$ROOT/experiments/configs"
SCRIPTS="$ROOT/scripts"

mkdir -p "$OUT"

echo "[aux] planner regression nightly only; paper profiling runs live under experiments/paper/"

run() {
    local bin="$1"; shift
    local scene="$1"; shift
    local out="$1"; shift
    echo "── $bin --scene=$(basename "$scene") --full ──"
    "$BUILD/experiments/$bin" --scene="$scene" --out="$out" --full "$@"
}

# Full main on every scene the v7 planner is expected to solve.
for s in 2dof_box iiwa14_far; do
    run exp_main "$CFG/$s.json" "$OUT/main_$s.json"
done

# threads + pathopt sweeps on iiwa14_far at full seeds
run exp_threads        "$CFG/iiwa14_far.json" "$OUT/threads_iiwa14_far.json"
run exp_pathopt_steps  "$CFG/iiwa14_far.json" "$OUT/pathopt_iiwa14_far.json"

# OMPL baseline at full seeds (multi-planner)
if [[ -x "$BUILD/experiments/baseline_ompl" ]]; then
    for s in 2dof_box iiwa14_far; do
        for planner in rrt_connect rrt_star informed_rrt_star bit_star; do
            echo "── baseline_ompl --planner=$planner --scene=$s --full ──"
            "$BUILD/experiments/baseline_ompl" --scene="$CFG/$s.json" \
                --out="$OUT/ompl_${planner}_${s}.json" --full \
                --planner="$planner" || echo "  (ompl $planner skipped: $?)"
        done
    done
fi

# Drake GCS baseline at full seeds
if command -v python3 >/dev/null; then
    for s in 2dof_box iiwa14_far; do
        echo "── baseline_drake_gcs --scene=$s --full ──"
        python3 "$SCRIPTS/baseline_drake_gcs.py" --scene="$CFG/$s.json" \
            --out="$OUT/drake_$s.json" --seeds 20 || \
            echo "  (drake skipped: $?)"
    done
fi

# Build per-experiment tables (main/threads/pathopt)
echo
echo "── tables ──"
python3 "$SCRIPTS/build_tables.py" \
    "$OUT/main_2dof_box.json" \
    "$OUT/main_iiwa14_far.json" \
    "$OUT/threads_iiwa14_far.json" \
    "$OUT/pathopt_iiwa14_far.json" \
    --out "$OUT/tables.md"
echo "Wrote $OUT/tables.md"

# Build cross-planner comparison report
python3 "$SCRIPTS/compare_planners.py" --results-dir "$OUT" \
    --scenes 2dof_box iiwa14_far \
    --v6-build-ms 444 --v6-per-box-ms 0.32 \
    --out "$OUT/compare_report.md" || true
echo "Wrote $OUT/compare_report.md"

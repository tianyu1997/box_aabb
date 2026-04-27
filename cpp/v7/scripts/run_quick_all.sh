#!/usr/bin/env bash
# run_quick_all.sh — Auxiliary planner-regression smoke driver.
#
# This script is intentionally narrower than the paper experiment suite under
# `experiments/paper/`: it only exercises the planner/regression binaries used
# for quick local checks and does not run the paper-side profiling blocks such
# as endpoint-source or link-envelope pipeline comparisons.
#
# Usage:  scripts/run_quick_all.sh [<build_dir>] [<out_dir>]
# Defaults: build_dir=cpp/v7/build, out_dir=cpp/v7/experiments/results_nightly
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD="${1:-$ROOT/build}"
OUT="${2:-$ROOT/experiments/results_nightly/quick}"
CFG="$ROOT/experiments/configs"
SCRIPTS="$ROOT/scripts"

mkdir -p "$OUT"

echo "[aux] planner regression smoke only; paper profiling runs live under experiments/paper/"

run() {
    local bin="$1"; shift
    local scene="$1"; shift
    local out="$1"; shift
    echo "── $bin --scene=$(basename "$scene") --quick ──"
    "$BUILD/experiments/$bin" --scene="$scene" --out="$out" --quick "$@"
}

# main on every scene that the v7 planner is expected to solve in --quick
# mode. (iiwa14_block is intentionally deferred — it requires the full
# pipeline / Drake GCS baseline planned for a later phase.)
for s in 2dof_box iiwa14_far; do
    run exp_main "$CFG/$s.json" "$OUT/main_$s.json"
done

# threads + pathopt sweeps on iiwa14_far (the one most likely to be SR=1)
run exp_threads        "$CFG/iiwa14_far.json" "$OUT/threads_iiwa14_far.json"
run exp_pathopt_steps  "$CFG/iiwa14_far.json" "$OUT/pathopt_iiwa14_far.json"

# Verify acceptance & build markdown tables.
echo
echo "── verify ──"
python3 "$SCRIPTS/verify_quick_results.py" "$OUT"/*.json

echo
echo "── tables ──"
python3 "$SCRIPTS/build_tables.py" "$OUT"/*.json --out "$OUT/tables.md"
echo "Wrote $OUT/tables.md"

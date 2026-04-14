#!/bin/bash
# v5 vs v6 Experiment Comparison
# Runs exp2 (e2e), exp4 (envelope), exp5 (ablation), exp6 (build timing) on both versions
# Usage: bash run_v5v6_compare.sh [--quick]

set -e

QUICK=""
if [[ "$1" == "--quick" ]]; then
    QUICK="--quick"
    echo "=== QUICK MODE ==="
fi

V5_BUILD="/home/tian/桌面/box_aabb/cpp/v5/build"
V6_BUILD="/home/tian/桌面/box_aabb/cpp/v6/build"
OUT_DIR="/home/tian/桌面/box_aabb/output/v5v6_compare"
mkdir -p "$OUT_DIR"

TIMESTAMP=$(date +%Y%m%d_%H%M%S)

echo "=========================================="
echo " v5 vs v6 Experiment Comparison"
echo " Timestamp: $TIMESTAMP"
echo "=========================================="
echo ""

# ── Exp 4: Envelope Pipeline Benchmark ──
echo ">>> Exp4: Envelope Pipeline Benchmark"
echo "  [v5] Running..."
timeout 300 "$V5_BUILD/experiments/exp4_envelope_benchmark" $QUICK 2>&1 | tee "$OUT_DIR/v5_exp4_${TIMESTAMP}.txt"
echo ""
echo "  [v6] Running..."
timeout 300 "$V6_BUILD/experiments/exp4_envelope_benchmark" $QUICK 2>&1 | tee "$OUT_DIR/v6_exp4_${TIMESTAMP}.txt"
echo ""

# ── Exp 2: End-to-End Planning ──
echo ">>> Exp2: End-to-End Planning"
echo "  [v5] Running..."
timeout 600 "$V5_BUILD/experiments/exp2_e2e_planning" $QUICK 2>&1 | tee "$OUT_DIR/v5_exp2_${TIMESTAMP}.txt"
echo ""
echo "  [v6] Running..."
timeout 600 "$V6_BUILD/experiments/exp2_e2e_planning" $QUICK 2>&1 | tee "$OUT_DIR/v6_exp2_${TIMESTAMP}.txt"
echo ""

# ── Exp 6: Build Timing ──
echo ">>> Exp6: Build Timing"
echo "  [v5] Running..."
timeout 600 "$V5_BUILD/experiments/exp6_build_timing" $QUICK 2>&1 | tee "$OUT_DIR/v5_exp6_${TIMESTAMP}.txt"
echo ""
echo "  [v6] Running..."
timeout 600 "$V6_BUILD/experiments/exp6_build_timing" $QUICK 2>&1 | tee "$OUT_DIR/v6_exp6_${TIMESTAMP}.txt"
echo ""

# ── Exp 5: Ablation Study (longest — run last) ──
echo ">>> Exp5: Ablation Study"
echo "  [v5] Running..."
timeout 1800 "$V5_BUILD/experiments/exp5_ablation" $QUICK 2>&1 | tee "$OUT_DIR/v5_exp5_${TIMESTAMP}.txt"
echo ""
echo "  [v6] Running..."
timeout 1800 "$V6_BUILD/experiments/exp5_ablation" $QUICK 2>&1 | tee "$OUT_DIR/v6_exp5_${TIMESTAMP}.txt"
echo ""

echo "=========================================="
echo " All experiments complete!"
echo " Results saved to: $OUT_DIR"
echo "=========================================="

#!/bin/bash
# run_exp1.sh — 运行实验1: 端到端 GCS 规划性能
# 自动处理: Cython 构建 + 路径设置

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# ── 1. 构建 Cython 扩展 (如 .so 不存在) ──
if ! ls src/forest/_hier_core*.so &>/dev/null; then
    echo "[setup] Building Cython extensions..."
    python3 setup_cython.py build_ext --inplace -q
    echo "[setup] Done."
else
    echo "[setup] Cython extensions already built."
fi

# ── 2. 设置 PYTHONPATH ──
export PYTHONPATH="$SCRIPT_DIR:$SCRIPT_DIR/src${PYTHONPATH:+:$PYTHONPATH}"

# ── 3. 运行实验 ──
python3 -m experiments.paper_exp1_e2e_gcs "$@"

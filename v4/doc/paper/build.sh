#!/usr/bin/env bash
# Build script for IROS paper  (Linux / macOS)
# Usage:
#   ./build.sh          # full build (pdflatex → bibtex → pdflatex × 2)
#   ./build.sh --once   # single pdflatex pass
#   ./build.sh --clean  # remove build artifacts
#
# Requires: pdflatex, bibtex  (TeX Live recommended)
#           IEEEtran.cls in this directory or TeX search path

set -euo pipefail
cd "$(dirname "$0")"

if [[ "${1:-}" == "--clean" ]]; then
    rm -f main.{aux,bbl,blg,log,out,pdf,synctex.gz,fls,fdb_latexmk}
    echo "Cleaned."
    exit 0
fi

# Check IEEEtran.cls
if ! kpsewhich IEEEtran.cls >/dev/null 2>&1 && [[ ! -f IEEEtran.cls ]]; then
    echo "ERROR: IEEEtran.cls not found."
    echo "Install texlive-publishers or download from:"
    echo "  https://ctan.org/pkg/ieeetran"
    exit 1
fi

if [[ "${1:-}" == "--once" ]]; then
    pdflatex -interaction=nonstopmode main
    exit 0
fi

echo "=== Pass 1: pdflatex ==="
pdflatex -interaction=nonstopmode main
echo "=== Pass 2: bibtex ==="
bibtex main || true
echo "=== Pass 3: pdflatex ==="
pdflatex -interaction=nonstopmode main
echo "=== Pass 4: pdflatex ==="
pdflatex -interaction=nonstopmode main
echo "=== Done: main.pdf ==="

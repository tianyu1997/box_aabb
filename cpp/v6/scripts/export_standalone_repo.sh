#!/usr/bin/env bash
# Copy this project's sources into DESTINATION for use as the root of a new
# standalone Git repo (omit local build artefacts and caches). Does not run
# git init; run that in DESTINATION after verifying the tree.
#
# Usage (from repo root = parent of scripts/):
#   ./scripts/export_standalone_repo.sh /path/to/SafeBoxForest-public
#
set -euo pipefail
DEST="${1:-}"
if [[ -z "$DEST" ]]; then
  echo "usage: $(basename "$0") <DESTINATION_DIRECTORY>" >&2
  exit 1
fi
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="$(mkdir -p "$DEST" && cd "$DEST" && pwd)"
mkdir -p "$DEST"
exec rsync -a "$ROOT/" "$DEST/" \
  --exclude '.git/' \
  --exclude 'build/' \
  --exclude 'build_*/' \
  --exclude 'build-*/' \
  --exclude 'cmake-build-*/' \
  --exclude '**/CMakeFiles/' \
  --exclude 'CMakeCache.txt' \
  --exclude 'cmake_install.cmake' \
  --exclude 'CTestTestfile.cmake' \
  --exclude 'compile_commands.json' \
  --exclude 'Testing/' \
  --exclude '_sbf6_deps/' \
  --exclude 'result/' \
  --exclude 'output/' \
  --exclude 'log/' \
  --exclude 'archive/' \
  --exclude '.cache/' \
  --exclude '.pytest_cache/' \
  --exclude '__pycache__/' \
  --exclude '*.pyc' \
  --exclude '*.egg-info/' \
  --exclude '.venv/' \
  --exclude 'venv/' \
  --exclude '.idea/' \
  --exclude '.vscode/' \
  --exclude '*.log' \
  --exclude '*.aux' \
  --exclude '*.bbl' \
  --exclude '*.blg' \
  --exclude '*.fdb_latexmk' \
  --exclude '*.fls' \
  --exclude '*.pdf' \
  --exclude '*.synctex.gz' \
  --exclude '*.toc' \
  --exclude '*.out' \
  --exclude '*.bcf' \
  --exclude '*.run.xml' \
  --exclude '*.xdv' \
  --exclude '*.lect' \
  --exclude '*.bin' \
  --exclude '*.cache' \
  --exclude '*.hull' \
  --exclude '*.hulls' \
  --exclude '*.frames' \
  --exclude '*.hcache' \
  --exclude '*.sdf' \
  --exclude '*.ipch' \
  --exclude '*.bak' \
  --exclude 'viz_output/' \
  --exclude 'experiments/results_paper/'

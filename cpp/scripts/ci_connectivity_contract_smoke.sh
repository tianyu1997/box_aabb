#!/usr/bin/env bash
set -euo pipefail

# Smoke test for connectivity metric contract:
# - Canonical: UF connected
# - Diagnostic: adjacency islands

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CPP_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
V5_BUILD="${CPP_ROOT}/v5/build"
V6_BUILD="${CPP_ROOT}/v6/build"

if [[ ! -d "${V5_BUILD}" || ! -d "${V6_BUILD}" ]]; then
  echo "[FAIL] Build directories not found:" >&2
  echo "  ${V5_BUILD}" >&2
  echo "  ${V6_BUILD}" >&2
  exit 1
fi

NPROC="$(command -v nproc >/dev/null 2>&1 && nproc || echo 4)"

echo "[1/4] Build v5 exp_box_connect"
make -C "${V5_BUILD}" -j"${NPROC}" exp_box_connect >/dev/null

echo "[2/4] Run v5 smoke and assert UF-canonical labels"
V5_OUT="$(${V5_BUILD}/experiments/exp_box_connect \
  --seeds 1 --mc 1000 --threads 5 --timeout 20000 --stop-on-connect --no-coarsen 2>&1)"

echo "${V5_OUT}" | grep -q "Standard: Connected = UF canonical, Islands = adjacency diagnostic" || {
  echo "[FAIL] v5 missing summary contract line" >&2
  echo "${V5_OUT}" | tail -n 80 >&2
  exit 1
}

echo "${V5_OUT}" | grep -q "connected(UF-canonical)=" || {
  echo "[FAIL] v5 missing UF-canonical per-seed field" >&2
  echo "${V5_OUT}" | tail -n 80 >&2
  exit 1
}

echo "${V5_OUT}" | grep -q "islands(diagnostic)=" || {
  echo "[FAIL] v5 missing diagnostic islands field" >&2
  echo "${V5_OUT}" | tail -n 80 >&2
  exit 1
}

echo "[3/4] Build v6 exp6_build_timing"
make -C "${V6_BUILD}" -j"${NPROC}" exp6_build_timing >/dev/null

echo "[4/4] Run v6 smoke and assert UF-canonical source line"
V6_OUT="$(${V6_BUILD}/experiments/exp6_build_timing --seeds 1 --scene combined 2>&1)"

echo "${V6_OUT}" | grep -q "canonical connectivity source: UF (adjacency islands are diagnostic only)" || {
  echo "[FAIL] v6 missing canonical UF source line" >&2
  echo "${V6_OUT}" | tail -n 80 >&2
  exit 1
}

echo "${V6_OUT}" | grep -q "grow connectivity: uf_connected=" || {
  echo "[FAIL] v6 missing grow connectivity audit line" >&2
  echo "${V6_OUT}" | tail -n 80 >&2
  exit 1
}

echo "[PASS] Connectivity contract smoke checks passed (v5 + v6)."
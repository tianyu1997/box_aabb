#!/usr/bin/env bash
set -euo pipefail

export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:${PATH:-}"
export HOME="${HOME:-/home/tian}"

CRON_TAG="copilot-reboot-baseline-once"
WORKSPACE="${WORKSPACE:-/home/tian/桌面/box_aabb}"
WORKDIR="/workspace/box_aabb"
PRM_IMAGE="gcs-science-robotics-prm-drake:latest"
LOCAL_IMAGE="gcs-science-robotics-local:latest"
LOGICAL_THREADS="${LOGICAL_THREADS:-4}"
DOCKER_CPUS="${DOCKER_CPUS:-4}"
DOCKER_MEMORY="${DOCKER_MEMORY:-20g}"
TASKSET_CPUS="${TASKSET_CPUS:-}"
HOST_DRAKE_PACKAGE_DIR="${HOST_DRAKE_PACKAGE_DIR:-${HOME}/drake_prm_gcs/src/drake}"
CONTAINER_DRAKE_PACKAGE_DIR="${CONTAINER_DRAKE_PACKAGE_DIR:-/opt/drake-src-fallback}"
RESULTS_ROOT="${RESULTS_ROOT:-${WORKSPACE}/cpp/v7/experiments/results_paper/post_reboot}"
LOG_ROOT="${LOG_ROOT:-${WORKSPACE}/output/reboot_baseline_logs}"
STAMP="$(date +%F_%H%M%S)"
OUT_DIR="${RESULTS_ROOT}/${STAMP}"
LOG_FILE="${LOG_ROOT}/run_${STAMP}.log"
MOSEK_LICENSE="${MOSEK_LICENSE:-${HOME}/mosek/mosek.lic}"
CONTAINER_MOSEK_LICENSE="${CONTAINER_MOSEK_LICENSE:-/tmp/mosek.lic}"

host_to_container_path() {
  local host_path="$1"
  case "${host_path}" in
    "${WORKSPACE}")
      printf '%s\n' "${WORKDIR}"
      ;;
    "${WORKSPACE}"/*)
      printf '%s/%s\n' "${WORKDIR}" "${host_path#${WORKSPACE}/}"
      ;;
    *)
      printf '%s\n' "${host_path}"
      ;;
  esac
}

CONTAINER_OUT_DIR="$(host_to_container_path "${OUT_DIR}")"

mkdir -p "${OUT_DIR}" "${LOG_ROOT}"
exec > >(tee -a "${LOG_FILE}") 2>&1

remove_self_from_crontab() {
  local tmp
  tmp="$(mktemp)"
  crontab -l 2>/dev/null | grep -Fv "${CRON_TAG}" > "${tmp}" || true
  if [[ -s "${tmp}" ]]; then
    crontab "${tmp}"
  else
    crontab -r || true
  fi
  rm -f "${tmp}"
}

wait_for_docker() {
  local attempt
  for attempt in $(seq 1 60); do
    if docker info >/dev/null 2>&1; then
      return 0
    fi
    echo "[$(date +%T)] waiting for docker daemon (${attempt}/60)"
    sleep 5
  done
  return 1
}

run_container() {
  local image="$1"
  shift
  local runner=()
  local docker_args=(
    docker run --rm
    --cpus "${DOCKER_CPUS}"
    --memory "${DOCKER_MEMORY}"
    -w "${WORKDIR}"
    -e "PYTHONPATH=${WORKDIR}/gcs-science-robotics:/opt/drake/lib/python3.10/site-packages"
    -v "${WORKSPACE}:${WORKDIR}"
  )
  if [[ -f "${MOSEK_LICENSE}" ]]; then
    docker_args+=(
      -e "MOSEKLM_LICENSE_FILE=${CONTAINER_MOSEK_LICENSE}"
      -v "${MOSEK_LICENSE}:${CONTAINER_MOSEK_LICENSE}:ro"
    )
  fi
  if [[ -d "${HOST_DRAKE_PACKAGE_DIR}" ]]; then
    docker_args+=(
      -e "DRAKE_PACKAGE_DIR=${CONTAINER_DRAKE_PACKAGE_DIR}"
      -v "${HOST_DRAKE_PACKAGE_DIR}:${CONTAINER_DRAKE_PACKAGE_DIR}:ro"
    )
  fi
  if command -v nice >/dev/null 2>&1; then
    runner+=( nice -n 15 )
  fi
  if [[ -n "${TASKSET_CPUS}" ]] && command -v taskset >/dev/null 2>&1; then
    runner+=( taskset -c "${TASKSET_CPUS}" )
  fi
  "${runner[@]}" "${docker_args[@]}" "${image}" "$@"
}

prm_probe() {
  local image="$1"
  run_container "${image}" python3 -c "import pydrake.all as dr; req=['PRMPlanner','PRMPlannerCreationParameters','PRMPlannerQueryParameters','PathProcessor','PathProcessorParameters','HolonomicKinematicPlanningSpace','VoxelizedEnvironmentCollisionChecker']; missing=[n for n in req if not hasattr(dr,n)]; print('missing=', missing); raise SystemExit(0 if not missing else 1)"
}

write_skip_note() {
  cat > "${OUT_DIR}/PRM_SKIPPED.txt" <<EOF
PRM baseline was skipped after reboot because no PRM-capable Drake image was available.
Checked images:
- ${PRM_IMAGE}
- ${LOCAL_IMAGE}

The available image did not provide the required Drake bindings:
- PRMPlanner
- PRMPlannerCreationParameters
- PRMPlannerQueryParameters
- PathProcessor
- PathProcessorParameters
- HolonomicKinematicPlanningSpace
- VoxelizedEnvironmentCollisionChecker
EOF
}

echo "[$(date +%F' '%T)] post-reboot baseline runner starting"
remove_self_from_crontab

if ! wait_for_docker; then
  echo "Docker daemon did not become ready within 5 minutes"
  exit 1
fi

IMAGE=""
if docker image inspect "${PRM_IMAGE}" >/dev/null 2>&1; then
  IMAGE="${PRM_IMAGE}"
elif docker image inspect "${LOCAL_IMAGE}" >/dev/null 2>&1; then
  IMAGE="${LOCAL_IMAGE}"
else
  echo "No usable Docker image found"
  exit 1
fi

echo "Using image: ${IMAGE}"
echo "Output dir: ${OUT_DIR}"
echo "Log file  : ${LOG_FILE}"

if prm_probe "${IMAGE}"; then
  echo "PRM bindings available; running full Marcucci baseline"
  run_container "${IMAGE}" \
    python3 cpp/v7/experiments/scripts/run_marcucci_baselines.py \
    --logical-threads "${LOGICAL_THREADS}" \
    --out-dir "${CONTAINER_OUT_DIR}"
else
  echo "PRM bindings unavailable in ${IMAGE}; running IRIS-only fallback"
  run_container "${IMAGE}" \
    python3 cpp/v7/experiments/scripts/marcucci_iris_np_gcs.py \
    --logical-threads "${LOGICAL_THREADS}" \
    --out "${CONTAINER_OUT_DIR}/marcucci_iris_np_gcs.json"
  run_container "${IMAGE}" \
    python3 cpp/v7/experiments/scripts/marcucci_iris_zo_gcs.py \
    --logical-threads "${LOGICAL_THREADS}" \
    --out "${CONTAINER_OUT_DIR}/marcucci_iris_zo_gcs.json"
  write_skip_note
fi

echo "[$(date +%F' '%T)] baseline runner finished"
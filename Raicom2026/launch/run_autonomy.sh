#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RAICOM_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="$(cd "${RAICOM_DIR}/.." && pwd)"
SIM_SCRIPT="${RAICOM_DIR}/sim_mujoco/bin/start_sim.sh"
MC_SCRIPT="${RAICOM_DIR}/mc/bin/em_run.sh"
AUTONOMY_DIR="${WORKSPACE_DIR}/raicom2026_x2_autonomy"
AUTONOMY_BUILD_DIR="${AUTONOMY_DIR}/build"
AUTONOMY_BIN="${AUTONOMY_BUILD_DIR}/raicom2026_x2_autonomy"

if [[ ! -x "${SIM_SCRIPT}" ]]; then
  echo "Missing simulator script: ${SIM_SCRIPT}" >&2
  exit 1
fi

if [[ ! -x "${MC_SCRIPT}" ]]; then
  echo "Missing MC script: ${MC_SCRIPT}" >&2
  exit 1
fi

cmake -S "${AUTONOMY_DIR}" -B "${AUTONOMY_BUILD_DIR}"
cmake --build "${AUTONOMY_BUILD_DIR}"

export ROS_LOG_DIR="${AUTONOMY_DIR}/logs"
export LD_LIBRARY_PATH="${RAICOM_DIR}/mc/lib:${RAICOM_DIR}/sim_mujoco/lib:${LD_LIBRARY_PATH:-}"

(
  cd "$(dirname "${SIM_SCRIPT}")"
  source "${SIM_SCRIPT}" "$@"
) &
SIM_PID=$!

(
  cd "$(dirname "${MC_SCRIPT}")"
  source "${MC_SCRIPT}" "$@"
) &
MC_PID=$!

cleanup() {
  if kill -0 "${SIM_PID}" 2>/dev/null; then
    kill "${SIM_PID}" 2>/dev/null || true
  fi
  if kill -0 "${MC_PID}" 2>/dev/null; then
    kill "${MC_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

sleep 5
"${AUTONOMY_BIN}"

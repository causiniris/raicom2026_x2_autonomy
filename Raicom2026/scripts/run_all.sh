#!/usr/bin/env bash
set -euo pipefail

export ROS_DOMAIN_ID=20
export ROS_LOCALHOST_ONLY=0

WORKSPACE_DIR="${X2_DEPLOY_WORKSPACE:-/home/agi/x2_deploy_workspace}"
export X2_DEPLOY_WORKSPACE="${WORKSPACE_DIR}"
SIM_DIR="${WORKSPACE_DIR}/sim_mujoco/bin"
MC_DIR="${WORKSPACE_DIR}/mc/bin"
AUTONOMY_DIR="${WORKSPACE_DIR}/raicom2026_x2_autonomy"
AUTONOMY_BUILD_DIR="${AUTONOMY_DIR}/build_docker"
AUTONOMY_BIN="${AUTONOMY_BUILD_DIR}/raicom2026_x2_autonomy"

if [[ ! -f "/.dockerenv" && "${X2_ALLOW_NON_DOCKER:-0}" != "1" ]]; then
  echo "[ERROR] run_all.sh must run inside the x2_deploy Docker container." >&2
  echo "[ERROR] Use: docker start x2_deploy && docker exec -it x2_deploy /home/agi/x2_deploy_workspace/scripts/run_all.sh" >&2
  exit 1
fi

if [[ ! -x "${SIM_DIR}/start_sim.sh" ]]; then
  echo "[ERROR] Missing simulator script: ${SIM_DIR}/start_sim.sh" >&2
  exit 1
fi

if [[ ! -x "${MC_DIR}/em_run.sh" ]]; then
  echo "[ERROR] Missing MC script: ${MC_DIR}/em_run.sh" >&2
  exit 1
fi

if [[ ! -d "${AUTONOMY_DIR}" ]]; then
  echo "[ERROR] Missing autonomy source inside Docker workspace: ${AUTONOMY_DIR}" >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${WORKSPACE_DIR}/aimdk_msgs/share/aimdk_msgs/local_setup.bash" ]]; then
  source "${WORKSPACE_DIR}/aimdk_msgs/share/aimdk_msgs/local_setup.bash"
fi
if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  source "${WORKSPACE_DIR}/install/setup.bash"
fi
set -u

export ROS_LOG_DIR="${AUTONOMY_DIR}/logs"
export LD_LIBRARY_PATH="${WORKSPACE_DIR}/mc/lib:${WORKSPACE_DIR}/sim_mujoco/lib:${LD_LIBRARY_PATH:-}"
mkdir -p "${ROS_LOG_DIR}"

echo "[INFO] Runtime environment: docker"
echo "[INFO] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[INFO] ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
echo "[INFO] Building autonomy in Docker workspace: ${AUTONOMY_BUILD_DIR}"
cmake -S "${AUTONOMY_DIR}" -B "${AUTONOMY_BUILD_DIR}"
cmake --build "${AUTONOMY_BUILD_DIR}"

cleanup() {
  if [[ -n "${SIM_PID:-}" ]] && kill -0 "${SIM_PID}" 2>/dev/null; then
    kill "${SIM_PID}" 2>/dev/null || true
  fi
  if [[ -n "${MC_PID:-}" ]] && kill -0 "${MC_PID}" 2>/dev/null; then
    kill "${MC_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[INFO] Starting sim_mujoco"
(
  cd "${SIM_DIR}"
  ./start_sim.sh "$@"
) &
SIM_PID=$!

sleep 3

if pgrep -f "[m]c_app_main" >/dev/null; then
  echo "[INFO] mc_app_main already running"
else
  echo "[INFO] Starting MC"
  (
    cd "${MC_DIR}"
    ./em_run.sh "$@"
  ) &
  MC_PID=$!
fi

for _ in $(seq 1 20); do
  if pgrep -f "[m]c_app_main" >/dev/null; then
    break
  fi
  sleep 1
done

if ! pgrep -f "[m]c_app_main" >/dev/null; then
  echo "[ERROR] mc_app_main is not running after startup wait." >&2
  exit 1
fi

echo "[INFO] Starting autonomy node"
cd "${AUTONOMY_DIR}"
"${AUTONOMY_BIN}"

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RAICOM_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [[ ! -f "/.dockerenv" && "${X2_ALLOW_NON_DOCKER:-0}" != "1" ]]; then
  echo "[ERROR] Host-side autonomy execution is forbidden." >&2
  echo "[ERROR] Run inside Docker instead:" >&2
  echo "        docker start x2_deploy && docker exec -it x2_deploy /home/agi/x2_deploy_workspace/scripts/run_all.sh" >&2
  exit 1
fi

exec "${RAICOM_DIR}/scripts/run_all.sh" "$@"

#!/usr/bin/env bash
set -o pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec bash "${SCRIPT_DIR}/run_full_mission.sh" --drop-only "$@"

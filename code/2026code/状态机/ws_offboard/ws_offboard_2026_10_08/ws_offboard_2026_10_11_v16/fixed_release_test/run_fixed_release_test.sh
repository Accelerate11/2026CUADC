#!/usr/bin/env bash
set -o pipefail

APP_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
SUITE_ROOT=$(cd -- "${APP_ROOT}/.." && pwd)
MISSION_ROOT="${SUITE_ROOT}"
export CUADC_APP_ROOT="${APP_ROOT}"
export CUADC_PARAMS_SOURCE="${SUITE_ROOT}/full_mission/mission.yaml"
export CUADC_LOG_ROOT="${APP_ROOT}/logs"
export CUADC_POSTFLIGHT_REPORT=false

exec bash "${MISSION_ROOT}/scripts/run_full_mission.sh" --fixed-release-test "$@"

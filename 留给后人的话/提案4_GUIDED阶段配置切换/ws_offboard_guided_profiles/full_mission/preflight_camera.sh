#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INSTALL_BASE="${CUADC_INSTALL_BASE:-${ROOT}/install}"
source /opt/ros/humble/setup.bash
source "${INSTALL_BASE}/setup.bash"
exec ros2 run cuadc_full_mission_20260803 preflight_camera_view.py "$@"

#!/usr/bin/env bash
set -euo pipefail
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$script_dir/_common.sh"
if (( $# > 0 )); then
  printf 'Usage: bash scripts/build_onboard.sh\n' >&2
  [[ "$1" == --help || "$1" == -h ]] && exit 0
  exit 2
fi
cuadc_source_ros
command -v colcon >/dev/null
cd -- "$CUADC_WORKSPACE"
if command -v rosdep >/dev/null 2>&1 && [[ "${CUADC_SKIP_ROSDEP_CHECK:-0}" != 1 ]]; then
  rosdep check --from-paths src --ignore-src --rosdistro humble
fi
colcon build --symlink-install --event-handlers console_direct+
cuadc_source "$CUADC_WORKSPACE/install/setup.bash"
for package in cuadc_mission cuadc_perception cuadc_tools cuadc_bringup; do
  ros2 pkg prefix "$package"
done

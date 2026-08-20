#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=_common.sh
source "$script_dir/_common.sh"

usage() {
  printf '%s\n' \
    'Usage: build_onboard.sh' \
    'Builds the real-aircraft workspace only. It does not install OS/Python dependencies.'
}

if (( $# > 0 )); then
  case "$1" in
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
fi

cuadc_require_command colcon
cuadc_source_ros
cd -- "$CUADC_WORKSPACE"

if command -v rosdep >/dev/null 2>&1; then
  rosdep check --from-paths "$CUADC_WORKSPACE/src" --ignore-src --rosdistro humble
else
  printf 'WARNING: rosdep is unavailable; dependency check was skipped.\n' >&2
fi

colcon build --symlink-install --event-handlers console_direct+
cuadc_source_nounset_safe "$CUADC_WORKSPACE/install/setup.bash"

for package in cuadc_bringup cuadc_mission cuadc_payload cuadc_perception cuadc_safety; do
  ros2 pkg prefix "$package" >/dev/null
done
printf 'Onboard build complete: %s\n' "$CUADC_WORKSPACE"
printf 'Use only: ros2 launch cuadc_bringup flight.launch.py ...\n'

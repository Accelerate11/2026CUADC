#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=_common.sh
source "$script_dir/_common.sh"

timeout_s=${CUADC_ROS_SERVICE_TIMEOUT_S:-15}

usage() {
  printf '%s\n' \
    'Usage: emergency_land.sh' \
    'Requires an interactive exact confirmation, then calls only /mavros/cmd/land.'
}

if (( $# > 0 )); then
  case "$1" in
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
fi

cuadc_validate_timeout "$timeout_s"
cuadc_require_command timeout
cuadc_source_workspace

service_type=$(ros2 service type /mavros/cmd/land 2>/dev/null || true)
[[ "$service_type" == mavros_msgs/srv/CommandTOL ]] || \
  cuadc_die "Expected /mavros/cmd/land type mavros_msgs/srv/CommandTOL, got: ${service_type:-unavailable}"
timeout --foreground "$timeout_s" ros2 topic echo --once /mavros/state || \
  cuadc_die 'No fresh MAVROS state; LAND was not sent.'

cuadc_prompt_exact \
  'DANGER: verify the connected aircraft identity and clear its landing area.' \
  'LAND CONNECTED AIRCRAFT NOW'

response=$(timeout --foreground "$timeout_s" ros2 service call \
  /mavros/cmd/land mavros_msgs/srv/CommandTOL \
  '{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 0.0}')
printf '%s\n' "$response"
printf '%s\n' "$response" | grep -Eq 'success:[[:space:]]*true' || \
  cuadc_die 'FCU did not acknowledge the LAND service call.'
printf 'LAND acknowledged; keep supervising. This script does not disarm.\n'

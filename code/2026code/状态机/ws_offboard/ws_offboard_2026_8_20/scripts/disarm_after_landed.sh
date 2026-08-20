#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=_common.sh
source "$script_dir/_common.sh"

timeout_s=${CUADC_ROS_SERVICE_TIMEOUT_S:-15}

usage() {
  printf '%s\n' \
    'Usage: disarm_after_landed.sh' \
    'Requires fresh landed_state=1 and interactive confirmation, then calls only MAVROS arming=false.'
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

service_type=$(ros2 service type /mavros/cmd/arming 2>/dev/null || true)
[[ "$service_type" == mavros_msgs/srv/CommandBool ]] || \
  cuadc_die "Expected /mavros/cmd/arming type mavros_msgs/srv/CommandBool, got: ${service_type:-unavailable}"

extended_state=$(timeout --foreground "$timeout_s" ros2 topic echo --once /mavros/extended_state) || \
  cuadc_die 'No fresh extended state; DISARM was not sent.'
printf '%s\n' "$extended_state"
printf '%s\n' "$extended_state" | grep -Eq 'landed_state:[[:space:]]*1([[:space:]]|$)' || \
  cuadc_die 'FCU does not report MAV_LANDED_STATE_ON_GROUND (1); DISARM was refused.'
timeout --foreground "$timeout_s" ros2 topic echo --once /mavros/state || \
  cuadc_die 'No fresh MAVROS state; DISARM was not sent.'

cuadc_prompt_exact \
  'DANGER: physically verify the connected aircraft is on ground and motionless.' \
  'DISARM LANDED CONNECTED AIRCRAFT'

response=$(timeout --foreground "$timeout_s" ros2 service call \
  /mavros/cmd/arming mavros_msgs/srv/CommandBool '{value: false}')
printf '%s\n' "$response"
printf '%s\n' "$response" | grep -Eq 'success:[[:space:]]*true' || \
  cuadc_die 'FCU did not acknowledge the DISARM service call.'
printf 'DISARM acknowledged; visually verify propellers are stopped.\n'

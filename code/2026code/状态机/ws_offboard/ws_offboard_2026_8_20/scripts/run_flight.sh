#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=_common.sh
source "$script_dir/_common.sh"

aircraft=${CUADC_AIRCRAFT:-}
config_root=${CUADC_CONFIG_ROOT:-}
mission=${CUADC_MISSION:-competition_2026}

usage() {
  printf '%s\n' \
    'Usage: run_flight.sh --aircraft v6x_3E0032|v5nano_410035 --config-root APPROVED_DIR [--mission competition_2026]' \
    '' \
    'Required operator environment (this wrapper never sets these values):' \
    '  CUADC_FLIGHT_AUTHORIZED=YES_I_COMPLETED_PREFLIGHT' \
    '  CUADC_APPROVED_CONFIG_BUNDLE_SHA256=<signed 64-char lowercase hash>'
}

while (( $# > 0 )); do
  case "$1" in
    --aircraft) aircraft=${2:?missing value for --aircraft}; shift 2 ;;
    --config-root) config_root=${2:?missing value for --config-root}; shift 2 ;;
    --mission) mission=${2:?missing value for --mission}; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -n "$aircraft" ]] || cuadc_die '--aircraft is required.'
cuadc_validate_aircraft "$aircraft"
cuadc_source_workspace
config_root=$(cuadc_require_external_config_root "$config_root")
cuadc_require_operator_authorization

printf 'Requesting gated real flight through the single supported launch.\n'
printf 'aircraft=%s config_root=%s mission=%s\n' "$aircraft" "$config_root" "$mission"
exec ros2 launch cuadc_bringup flight.launch.py \
  aircraft:="$aircraft" \
  mission:="$mission" \
  config_root:="$config_root" \
  preflight_only:=false

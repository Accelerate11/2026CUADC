#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=_common.sh
source "$script_dir/_common.sh"

destination=${CUADC_PROFILE_DESTINATION:-${HOME:?HOME is required}/cuadc_flight_profiles/field_candidate/config}

usage() {
  printf '%s\n' \
    'Usage: prepare_flight_profile.sh [--destination ABSOLUTE_CONFIG_DIR]' \
    '' \
    'Copies all five candidate layers to a new user-writable external directory.' \
    'It never enables flight, approves calibration, or creates a signed hash.'
}

while (( $# > 0 )); do
  case "$1" in
    --destination) destination=${2:?missing value for --destination}; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

cuadc_source_workspace
source_config=$(cuadc_installed_config_root)
destination=$(readlink -m -- "$destination")
[[ "$destination" == /* ]] || cuadc_die 'Destination must resolve to an absolute path.'
cuadc_require_outside_workspace "$destination"
[[ ! -e "$destination" ]] || cuadc_die "Refusing to overwrite existing destination: $destination"

install -d -m 0700 -- "$destination"
for layer in aircraft calibration mission field perception; do
  [[ -d "$source_config/$layer" ]] || cuadc_die "Installed candidate layer is missing: $layer"
  install -d -m 0700 -- "$destination/$layer"
  cp -a -- "$source_config/$layer/." "$destination/$layer/"
done
find "$destination" -type d -exec chmod 0700 -- {} +
find "$destination" -type f -exec chmod 0600 -- {} +

printf 'Five-layer candidate copy created: %s\n' "$destination"
printf '%s\n' \
  'It remains flight_enable=false and calibration_approved=false.' \
  'Before approval, measure and verify the selected FCU by-id link/baud, exact D435i serial,' \
  'camera extrinsics, payload offsets, both servo channels/PWMs/durations, and field geometry.' \
  'Record one approval_reference in both aircraft and calibration layers only after sign-off.' \
  'Then run read-only preflight, record its CONFIG_BUNDLE_SHA256 on the signed checklist,' \
  'and only an authorized operator may export that exact hash for run_flight.sh.'

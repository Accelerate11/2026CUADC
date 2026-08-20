#!/usr/bin/env bash
set -euo pipefail

CUADC_SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
CUADC_WORKSPACE=$(cd -- "$CUADC_SCRIPT_DIR/.." && pwd -P)
CUADC_SOURCE_CONFIG="$CUADC_WORKSPACE/src/cuadc_bringup/config"

export CUADC_WORKSPACE

cuadc_die() {
  printf 'ERROR: %s\n' "$*" >&2
  return 2
}

cuadc_require_command() {
  command -v -- "$1" >/dev/null 2>&1 || cuadc_die "Required command is unavailable: $1"
}

cuadc_source_nounset_safe() {
  local setup_file=$1
  local nounset_was_enabled=false

  [[ -r "$setup_file" ]] || cuadc_die "Setup file is missing or unreadable: $setup_file"
  if [[ $- == *u* ]]; then
    nounset_was_enabled=true
    set +u
  fi
  # shellcheck disable=SC1090
  source "$setup_file"
  if [[ "$nounset_was_enabled" == true ]]; then
    set -u
  fi
}

cuadc_source_ros() {
  cuadc_source_nounset_safe /opt/ros/humble/setup.bash
}

cuadc_source_workspace() {
  cuadc_source_ros
  cuadc_source_nounset_safe "$CUADC_WORKSPACE/install/setup.bash"
}

cuadc_validate_aircraft() {
  case "$1" in
    v6x_3E0032|v5nano_410035) ;;
    *) cuadc_die "Aircraft must be exactly v6x_3E0032 or v5nano_410035: $1" ;;
  esac
}

cuadc_resolve_config_root() {
  local configured=$1
  local resolved

  [[ -n "$configured" ]] || cuadc_die 'An external five-layer config root is required.'
  resolved=$(readlink -e -- "$configured") || cuadc_die "Config root does not exist: $configured"
  [[ -d "$resolved" ]] || cuadc_die "Config root is not a directory: $resolved"
  local layer
  for layer in aircraft calibration mission field perception; do
    [[ -d "$resolved/$layer" ]] || cuadc_die "Config root is missing layer directory: $layer"
  done
  printf '%s\n' "$resolved"
}

cuadc_installed_config_root() {
  local prefix
  prefix=$(ros2 pkg prefix cuadc_bringup) || cuadc_die 'cuadc_bringup is not installed.'
  readlink -e -- "$prefix/share/cuadc_bringup/config" || \
    cuadc_die 'Installed cuadc_bringup config directory is missing.'
}

cuadc_require_outside_workspace() {
  local candidate=$1
  case "$candidate" in
    "$CUADC_WORKSPACE"|"$CUADC_WORKSPACE"/*)
      cuadc_die "External flight configuration must be outside the shipped workspace: $candidate"
      ;;
  esac
}

cuadc_require_external_config_root() {
  local configured=$1
  local resolved
  local installed
  local source_config=''

  resolved=$(cuadc_resolve_config_root "$configured")
  cuadc_require_outside_workspace "$resolved"
  installed=$(cuadc_installed_config_root)
  if [[ -d "$CUADC_SOURCE_CONFIG" ]]; then
    source_config=$(readlink -e -- "$CUADC_SOURCE_CONFIG")
  fi
  [[ "$resolved" != "$installed" ]] || \
    cuadc_die 'Packaged candidate profiles cannot be used for real flight; prepare an external copy.'
  if [[ -n "$source_config" ]]; then
    [[ "$resolved" != "$source_config" ]] || \
      cuadc_die 'Source-tree candidate profiles cannot be used for real flight; prepare an external copy.'
  fi
  printf '%s\n' "$resolved"
}

cuadc_require_operator_authorization() {
  local approved=${CUADC_APPROVED_CONFIG_BUNDLE_SHA256:-}
  [[ ${CUADC_FLIGHT_AUTHORIZED:-} == YES_I_COMPLETED_PREFLIGHT ]] || \
    cuadc_die 'CUADC_FLIGHT_AUTHORIZED must exactly equal YES_I_COMPLETED_PREFLIGHT after signed preflight.'
  [[ "$approved" =~ ^[0-9a-f]{64}$ ]] || \
    cuadc_die 'CUADC_APPROVED_CONFIG_BUNDLE_SHA256 must be the signed 64-character lowercase bundle hash.'
}

cuadc_prompt_exact() {
  local prompt=$1
  local expected=$2
  local answer

  [[ -r /dev/tty && -w /dev/tty ]] || \
    cuadc_die 'An interactive operator terminal is required; piped confirmation is refused.'
  printf '%s\nType exactly: %s\n> ' "$prompt" "$expected" >/dev/tty
  IFS= read -r answer </dev/tty || cuadc_die 'Operator confirmation was not received.'
  [[ "$answer" == "$expected" ]] || cuadc_die 'Confirmation did not match exactly; no command was sent.'
}

cuadc_validate_timeout() {
  [[ "$1" =~ ^[1-9][0-9]*$ ]] || cuadc_die "Timeout must be a positive integer: $1"
}

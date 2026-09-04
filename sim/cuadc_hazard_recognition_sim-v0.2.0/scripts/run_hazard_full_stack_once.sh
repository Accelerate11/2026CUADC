#!/usr/bin/env bash
set -e
exec "$(dirname "$0")/run_hazard_coverage_once.sh" "$@"

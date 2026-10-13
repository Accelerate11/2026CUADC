#!/usr/bin/env bash
# Detached post-flight watcher. It is started in a separate session before the
# mission begins and therefore does not depend on wrapper EXIT traps or MAVROS
# cleanup. The atomic claim directory prevents duplicate reports.
set -o pipefail
set +u

MISSION_PID=$1
REPORT_SCRIPT=$2
FLIGHT_LOG=$3
HOOK_LOG="${FLIGHT_LOG}/postflight_report_hook.log"
CLAIM_DIR="${FLIGHT_LOG}/.postflight_report_claimed"

while kill -0 "${MISSION_PID}" 2>/dev/null; do
  sleep 0.5
done

# Allow ROS and video writers a brief window to flush after launch exits.
sleep 2
if ! mkdir "${CLAIM_DIR}" 2>/dev/null; then
  exit 0
fi

printf 'postflight watcher started at %s mission_pid=%s\n' \
  "$(date --iso-8601=seconds)" "${MISSION_PID}" >>"${HOOK_LOG}"
python3 -u "${REPORT_SCRIPT}" "${FLIGHT_LOG}" --no-email >>"${HOOK_LOG}" 2>&1
status=$?
printf 'postflight watcher finished at %s status=%s\n' \
  "$(date --iso-8601=seconds)" "${status}" >>"${HOOK_LOG}"
exit "${status}"

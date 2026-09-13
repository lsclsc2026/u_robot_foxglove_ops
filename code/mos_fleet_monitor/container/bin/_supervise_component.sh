#!/usr/bin/env bash

# Lightweight lifecycle supervisor. Components remain independent: this
# process records one component's exit and never stops or restarts its peers.
set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"

component="${1:-}"
case "${component}" in
  vision|navigation|image_relay|status_logger|teleop|bridge) ;;
  *)
    echo "未知组件：${component}" >&2
    exit 64
    ;;
esac

log_event "START component=${component} supervisor_pid=$$"

component_log="${MOS_MONITOR_RUNTIME}/${component}.log"
: > "${component_log}"

# The vendor RealSense driver can occasionally abort on an uncaught
# "Frame didn't arrive within 15000" exception. Keep this recovery in the
# additive monitor layer: wait for udev to settle and recreate only Vision.
# Three short-lived failures still stop the supervisor to avoid a restart loop.
if [[ "${component}" == vision ]]; then
  shutdown_requested=0
  consecutive_failures=0
  trap 'shutdown_requested=1' INT TERM

  while (( shutdown_requested == 0 )); do
    started_at="$(date +%s)"
    set +e
    "${COMPONENT_RUNNER}" vision >>"${component_log}" 2>&1
    exit_code=$?
    set -e
    ended_at="$(date +%s)"

    (( shutdown_requested == 1 )) && exit "${exit_code}"
    if (( ended_at - started_at >= 60 )); then
      consecutive_failures=1
    else
      consecutive_failures=$((consecutive_failures + 1))
    fi
    log_event \
      "EXIT component=vision supervisor_pid=$$ exit_code=${exit_code} recovery_attempt=${consecutive_failures}"

    if (( consecutive_failures >= 3 )); then
      log_event \
        "RESTART_GIVEUP component=vision supervisor_pid=$$ consecutive_failures=${consecutive_failures}"
      exit "${exit_code}"
    fi
    log_event \
      "RESTART component=vision supervisor_pid=$$ delay_seconds=10"
    sleep 10
  done
  exit 0
fi

set +e
"${COMPONENT_RUNNER}" "${component}" >>"${component_log}" 2>&1
exit_code=$?
set -e

log_event \
  "EXIT component=${component} supervisor_pid=$$ exit_code=${exit_code} action=peers_kept_running"
exit "${exit_code}"

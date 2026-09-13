#!/usr/bin/env bash

# Lightweight lifecycle supervisor. Components remain independent: this
# process records one component's exit and never stops or restarts its peers.
set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"

component="${1:-}"
case "${component}" in
  vision|navigation|image_relay|status_logger|bridge) ;;
  *)
    echo "未知组件：${component}" >&2
    exit 64
    ;;
esac

log_event "START component=${component} supervisor_pid=$$"

set +e
"${COMPONENT_RUNNER}" "${component}" >/dev/null 2>&1
exit_code=$?
set -e

log_event \
  "EXIT component=${component} supervisor_pid=$$ exit_code=${exit_code} action=peers_kept_running"
exit "${exit_code}"

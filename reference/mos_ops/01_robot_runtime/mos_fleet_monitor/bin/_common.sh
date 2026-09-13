#!/usr/bin/env bash

set -Eeuo pipefail

MOS_MONITOR_ROOT="${MOS_MONITOR_ROOT:-/root/work_space/mos_fleet_monitor}"
MOS_MONITOR_RUNTIME="${MOS_MONITOR_RUNTIME:-/run/mos_fleet_monitor}"
MOS_MONITOR_LOCK="${MOS_MONITOR_LOCK:-/run/lock/mos_fleet_monitor.lock}"
COMPONENT_RUNNER="${MOS_MONITOR_ROOT}/bin/_run_component.zsh"
COMPONENT_SUPERVISOR="${MOS_MONITOR_ROOT}/bin/_supervise_component.sh"
MOS_MONITOR_EVENT_LOG="${MOS_MONITOR_RUNTIME}/events.log"

if [[ -f "${MOS_MONITOR_ROOT}/config/monitor.env" ]]; then
  # The configuration file only contains parameter defaults and exports and is
  # intentionally compatible with both bash and zsh.
  source "${MOS_MONITOR_ROOT}/config/monitor.env"
fi

COMPONENTS=(vision navigation image_relay status_logger bridge)
STOP_COMPONENTS=(bridge status_logger image_relay navigation vision)

declare -A OWNED_MARKERS=(
  [vision]="${COMPONENT_SUPERVISOR} vision"
  [navigation]="${COMPONENT_SUPERVISOR} navigation"
  [image_relay]="${COMPONENT_SUPERVISOR} image_relay"
  [status_logger]="${COMPONENT_SUPERVISOR} status_logger"
  [bridge]="${COMPONENT_SUPERVISOR} bridge"
)

# Processes started by releases before the per-component supervisor are still
# considered owned when their recorded PID and original command both match.
# This keeps stop/status compatibility across an in-place upgrade.
declare -A LEGACY_OWNED_MARKERS=(
  [vision]="ros2 launch mos_3d_object vision.launch.py"
  [navigation]="ros2 launch lumos_nav nav2_fastlio_bringup.launch.py"
  [image_relay]="${MOS_MONITOR_ROOT}/assets/foxglove_image_relay.py"
  [status_logger]="${MOS_MONITOR_ROOT}/assets/mos_status_logger.py"
  [bridge]="ros2 launch foxglove_bridge foxglove_bridge_launch.xml"
)

declare -A CONFLICT_PATTERNS=(
  [vision]="ros2 launch mos_3d_object vision.launch.py|mos_3d_object_node"
  [navigation]="ros2 launch lumos_nav nav2_fastlio_bringup.launch.py|gicp_localization|fastlio_mapping|nav2_controller/controller_server|nav2_planner/planner_server|nav2_bt_navigator/bt_navigator"
  [image_relay]="foxglove_image_relay.py"
  [status_logger]="mos_status_logger.py"
  [bridge]="foxglove_bridge_launch.xml|/foxglove_bridge/foxglove_bridge"
)

ensure_runtime() {
  mkdir -p -- "${MOS_MONITOR_RUNTIME}" "$(dirname -- "${MOS_MONITOR_LOCK}")"
  touch -- "${MOS_MONITOR_LOCK}"
}

log_event() {
  local message="$*" max_bytes="${MOS_EVENT_LOG_MAX_BYTES:-1048576}"
  ensure_runtime
  printf '%s | %s\n' "$(date '+%Y-%m-%d %H:%M:%S %Z')" "${message}" \
    >> "${MOS_MONITOR_EVENT_LOG}"

  # The event log contains only lifecycle events. Keep it bounded even when a
  # component is restarted repeatedly over a long deployment.
  local size=0 keep_bytes=$((max_bytes / 2)) temporary=""
  size="$(stat -c '%s' "${MOS_MONITOR_EVENT_LOG}" 2>/dev/null || printf '0')"
  if (( size > max_bytes )); then
    temporary="$(mktemp "${MOS_MONITOR_EVENT_LOG}.XXXXXX")"
    tail -c "${keep_bytes}" "${MOS_MONITOR_EVENT_LOG}" > "${temporary}"
    mv -f -- "${temporary}" "${MOS_MONITOR_EVENT_LOG}"
  fi
}

pid_file() {
  printf '%s/%s.pid\n' "${MOS_MONITOR_RUNTIME}" "$1"
}

read_component_pid() {
  local file
  file="$(pid_file "$1")"
  [[ -s "${file}" ]] || return 1
  local pid
  read -r pid < "${file}"
  [[ "${pid}" =~ ^[0-9]+$ ]] || return 1
  printf '%s\n' "${pid}"
}

pid_is_live() {
  local pid="$1"
  [[ -d "/proc/${pid}" ]] || return 1
  [[ "$(awk '{print $3}' "/proc/${pid}/stat" 2>/dev/null || true)" != "Z" ]]
}

pid_cmdline() {
  tr '\0' ' ' < "/proc/$1/cmdline" 2>/dev/null || true
}

owned_component_is_live() {
  local component="$1" pid cmdline
  pid="$(read_component_pid "${component}")" || return 1
  pid_is_live "${pid}" || return 1
  cmdline="$(pid_cmdline "${pid}")"
  [[ "${cmdline}" == *"${OWNED_MARKERS[${component}]}"* ]] || \
    [[ "${cmdline}" == *"${LEGACY_OWNED_MARKERS[${component}]}"* ]]
}

remove_stale_pid_file() {
  local component="$1" file pid
  file="$(pid_file "${component}")"
  [[ -s "${file}" ]] || return 0

  pid="$(read_component_pid "${component}" 2>/dev/null || true)"
  if [[ -z "${pid}" ]] || ! owned_component_is_live "${component}"; then
    rm -f -- "${file}"
    if [[ -n "${pid}" ]]; then
      log_event "STALE component=${component} pid=${pid} action=pid_file_removed"
    fi
  fi
  return 0
}

find_conflicts() {
  local component="$1"
  pgrep -af -- "${CONFLICT_PATTERNS[${component}]}" 2>/dev/null || true
}

all_owned_components_live() {
  local component
  for component in "${COMPONENTS[@]}"; do
    owned_component_is_live "${component}" || return 1
  done
}

bridge_port_is_listening() {
  ss -ltn 2>/dev/null | awk '{print $4}' | \
    grep -Eq "(^|:)(${MOS_BRIDGE_PORT})$"
}

#!/usr/bin/env bash

set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}/mos-fleet-monitor-${UID}"
CONFIG_FILE="${SCRIPT_DIR}/robots.conf"
stop_remote=0

for argument in "$@"; do
  case "${argument}" in
    --remote) stop_remote=1 ;;
    --config=*) CONFIG_FILE="${argument#*=}" ;;
    *)
      echo "用法：$0 [--remote] [--config=/path/robots.conf]" >&2
      exit 64
      ;;
  esac
done

if [[ -d "${RUNTIME_DIR}" ]]; then
  for pid_file in "${RUNTIME_DIR}"/*.pid; do
    [[ -e "${pid_file}" ]] || continue
    read -r launcher_pid < "${pid_file}" || launcher_pid=""
    name="$(basename -- "${pid_file}" .pid)"
    if [[ "${launcher_pid}" =~ ^[0-9]+$ ]] && kill -0 "${launcher_pid}" 2>/dev/null; then
      echo "[关闭本地隧道] ${name} pid=${launcher_pid}"
      kill -INT "${launcher_pid}" 2>/dev/null || true
    fi
    rm -f -- "${pid_file}"
  done
fi

if (( stop_remote )); then
  [[ -r "${CONFIG_FILE}" ]] || {
    echo "读不到机器人配置：${CONFIG_FILE}" >&2
    exit 1
  }
  while read -r name host local_port extra; do
    [[ -n "${name:-}" && "${name}" != \#* ]] || continue
    echo "[停止远端监控栈] ${name} ${host}"
    SSH_BATCH_MODE=1 "${SCRIPT_DIR}/stop-robot-monitor.sh" "${host}" || true
  done < "${CONFIG_FILE}"
fi

echo "多机停止操作完成。"


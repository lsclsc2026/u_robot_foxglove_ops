#!/usr/bin/env bash

set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${1:-${SCRIPT_DIR}/robots.conf}"
RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}/mos-fleet-monitor-${UID}"

[[ -r "${CONFIG_FILE}" ]] || {
  echo "读不到机器人配置：${CONFIG_FILE}" >&2
  exit 1
}
mkdir -p -m 0700 -- "${RUNTIME_DIR}"

started=0
failed=0
declare -A seen_names=()
declare -A seen_ports=()

while read -r name host local_port extra; do
  [[ -n "${name:-}" ]] || continue
  [[ "${name}" == \#* ]] && continue
  if [[ -n "${extra:-}" || -z "${host:-}" || ! "${local_port:-}" =~ ^[0-9]+$ ]]; then
    echo "[配置错误] ${name} ${host:-} ${local_port:-} ${extra:-}" >&2
    failed=1
    continue
  fi
  if [[ -n "${seen_names[${name}]:-}" || -n "${seen_ports[${local_port}]:-}" ]]; then
    echo "[配置冲突] 名称或本地端口重复：${name} ${local_port}" >&2
    failed=1
    continue
  fi
  seen_names[${name}]=1
  seen_ports[${local_port}]=1

  pid_file="${RUNTIME_DIR}/${name}.pid"
  log_file="${RUNTIME_DIR}/${name}.log"
  if [[ -s "${pid_file}" ]]; then
    read -r existing_pid < "${pid_file}"
    if [[ "${existing_pid}" =~ ^[0-9]+$ ]] && kill -0 "${existing_pid}" 2>/dev/null; then
      echo "[已运行] ${name} pid=${existing_pid} ws://localhost:${local_port}"
      continue
    fi
    rm -f -- "${pid_file}"
  fi

  echo "[启动] ${name} ${host} -> ws://localhost:${local_port}"
  nohup env SSH_BATCH_MODE=1 \
    "${SCRIPT_DIR}/start-robot-monitor.sh" "${host}" "${local_port}" \
    >"${log_file}" 2>&1 &
  launcher_pid=$!
  printf '%s\n' "${launcher_pid}" > "${pid_file}"
  started=$((started + 1))
done < "${CONFIG_FILE}"

echo
echo "已提交 ${started} 个启动任务。日志目录：${RUNTIME_DIR}"
echo "多机后台启动要求 SSH 公钥已配置；本脚本不保存密码。"
echo "Foxglove 入口：${SCRIPT_DIR}/list-foxglove-links.sh"
exit "${failed}"


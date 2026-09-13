#!/usr/bin/env bash

set -Eeuo pipefail

ROBOT_HOST="${1:-${ROBOT_HOST:-100.127.82.99}}"
LOCAL_PORT="${2:-${LOCAL_PORT:-8765}}"
ROBOT_USER="${ROBOT_USER:-lumos}"
REMOTE_PORT="${REMOTE_PORT:-8765}"
REMOTE_START="${REMOTE_START:-/home/lumos/mos_fleet_monitor/bin/mos-monitor-start}"
SSH_BATCH_MODE="${SSH_BATCH_MODE:-0}"

ssh_initial_options=()
if [[ "${SSH_BATCH_MODE}" == "1" ]]; then
  ssh_initial_options+=(-o BatchMode=yes)
fi

if ss -ltn 2>/dev/null | awk '{print $4}' | grep -Eq "(^|:)${LOCAL_PORT}$"; then
  echo "本机端口 ${LOCAL_PORT} 已被占用。请先关闭旧隧道，或指定另一个端口：" >&2
  echo "  $0 ${ROBOT_HOST} 8766" >&2
  exit 2
fi

target="${ROBOT_USER}@${ROBOT_HOST}"
control_dir="${XDG_RUNTIME_DIR:-/tmp}/mos-monitor-ssh-${UID}"
safe_host="${ROBOT_HOST//[^A-Za-z0-9_.-]/_}"
control_path="${control_dir}/ctl-${ROBOT_USER}-${safe_host}-${LOCAL_PORT}"
tunnel_active=0

mkdir -p -m 0700 -- "${control_dir}"

cleanup() {
  trap - EXIT INT TERM
  if (( tunnel_active )); then
    ssh -S "${control_path}" -O cancel \
      -L "${LOCAL_PORT}:127.0.0.1:${REMOTE_PORT}" \
      "${target}" >/dev/null 2>&1 || true
  fi
  ssh -S "${control_path}" -O exit "${target}" >/dev/null 2>&1 || true
  rm -f -- "${control_path}"
}
trap cleanup EXIT INT TERM

echo "连接 ${target}；远端将复用已运行组件，并补启动缺失组件。"

# Create one authenticated master connection. All following SSH operations use
# this socket, so password authentication is requested only once.
ssh "${ssh_initial_options[@]}" -M -S "${control_path}" -fNT \
  -o ControlPersist=yes \
  -o ServerAliveInterval=15 \
  -o ServerAliveCountMax=3 \
  "${target}"

if ! ssh -S "${control_path}" -T "${target}" "${REMOTE_START}"; then
  echo "远端总控命令本身执行失败；没有建立 Foxglove 本地转发。" >&2
  exit 1
fi

echo "远端按需启动处理完成，建立 localhost:${LOCAL_PORT} -> ${ROBOT_HOST}:${REMOTE_PORT}。"
echo "Foxglove 连接：ws://localhost:${LOCAL_PORT}"
echo "按 Ctrl+C 只关闭本机隧道，机器人监控栈继续运行。"

ssh -S "${control_path}" -O forward \
  -L "${LOCAL_PORT}:127.0.0.1:${REMOTE_PORT}" \
  "${target}"
tunnel_active=1

while ssh -S "${control_path}" -O check "${target}" >/dev/null 2>&1; do
  sleep 5
done
echo "SSH Master 连接已断开，本地 Foxglove 隧道随之关闭。" >&2
exit 1

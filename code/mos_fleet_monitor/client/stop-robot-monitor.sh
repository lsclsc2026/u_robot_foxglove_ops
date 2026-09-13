#!/usr/bin/env bash

set -Eeuo pipefail
ROBOT_HOST="${1:-${ROBOT_HOST:-100.127.82.99}}"
ROBOT_USER="${ROBOT_USER:-lumos}"
REMOTE_STOP="${REMOTE_STOP:-/home/lumos/mos_fleet_monitor/bin/mos-monitor-stop}"
SSH_BATCH_MODE="${SSH_BATCH_MODE:-0}"

ssh_options=()
if [[ "${SSH_BATCH_MODE}" == "1" ]]; then
  ssh_options+=(-o BatchMode=yes)
fi

echo "这会停止机器人 ${ROBOT_HOST} 上由总控脚本启动的导航、视觉、图像中继、状态和 Bridge。"
echo "停止过程最长约 11 秒；看到最终完成提示前请不要按 Ctrl+C。"
exec ssh "${ssh_options[@]}" -T "${ROBOT_USER}@${ROBOT_HOST}" "${REMOTE_STOP}"

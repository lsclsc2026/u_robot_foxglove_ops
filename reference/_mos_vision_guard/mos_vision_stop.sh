#!/usr/bin/env bash

set -u

pid_file="/run/mos_vision.pid"
lock_dir="/run/lock/mos_vision.instance"
status_file="/run/mos_vision.status"

if [[ ! -s "${pid_file}" ]]; then
    echo "vision.launch.py 当前未运行。"
    rm -f -- "${status_file}" "${lock_dir}/pid"
    rmdir -- "${lock_dir}" 2>/dev/null || true
    exit 0
fi

read -r vision_pid < "${pid_file}"

if [[ ! "${vision_pid}" =~ ^[0-9]+$ ]] || [[ ! -d "/proc/${vision_pid}" ]]; then
    echo "检测到过期状态文件；vision.launch.py 当前未运行。"
    rm -f -- "${pid_file}" "${status_file}" "${lock_dir}/pid"
    rmdir -- "${lock_dir}" 2>/dev/null || true
    exit 0
fi

cmdline="$(tr '\0' ' ' < "/proc/${vision_pid}/cmdline" 2>/dev/null || true)"
if [[ "${cmdline}" != *"ros2 launch mos_3d_object vision.launch.py"* ]]; then
    echo "拒绝停止：PID ${vision_pid} 不是受管的 vision.launch.py 进程。"
    echo "实际命令：${cmdline}"
    exit 1
fi

echo "正在停止 vision.launch.py（PID ${vision_pid}）..."
kill -INT "${vision_pid}" 2>/dev/null || true

for _ in {1..10}; do
    if [[ ! -d "/proc/${vision_pid}" ]]; then
        rm -f -- "${pid_file}" "${status_file}" "${lock_dir}/pid"
        rmdir -- "${lock_dir}" 2>/dev/null || true
        echo "vision.launch.py 已正常停止。"
        exit 0
    fi
    sleep 1
done

echo "正常停止超时，发送 SIGTERM..."
kill -TERM "${vision_pid}" 2>/dev/null || true

for _ in {1..5}; do
    if [[ ! -d "/proc/${vision_pid}" ]]; then
        rm -f -- "${pid_file}" "${status_file}" "${lock_dir}/pid"
        rmdir -- "${lock_dir}" 2>/dev/null || true
        echo "vision.launch.py 已停止。"
        exit 0
    fi
    sleep 1
done

echo "强制停止 vision.launch.py..."
kill -KILL "${vision_pid}" 2>/dev/null || true
rm -f -- "${pid_file}" "${status_file}" "${lock_dir}/pid"
rmdir -- "${lock_dir}" 2>/dev/null || true

echo "vision.launch.py 已强制停止。"

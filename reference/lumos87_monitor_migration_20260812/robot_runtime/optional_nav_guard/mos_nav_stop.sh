#!/usr/bin/env bash

set -u

pid_file="/run/mos_navigation.pid"
status_file="/run/mos_navigation.status"
lock_dir="/run/lock/mos_navigation.instance"

cleanup_state() {
    rm -f -- "${pid_file}" "${status_file}" "${lock_dir}/pid"
    rmdir -- "${lock_dir}" 2>/dev/null || true
}

if [[ ! -s "${pid_file}" ]]; then
    echo "导航 launch 当前未运行。"
    cleanup_state
    exit 0
fi
read -r nav_pid < "${pid_file}"
if [[ ! "${nav_pid}" =~ ^[0-9]+$ ]] || [[ ! -d "/proc/${nav_pid}" ]]; then
    echo "检测到过期状态文件；导航 launch 当前未运行。"
    cleanup_state
    exit 0
fi

cmdline="$(tr '\0' ' ' < "/proc/${nav_pid}/cmdline" 2>/dev/null || true)"
pgid="$(ps -o pgid= -p "${nav_pid}" 2>/dev/null | tr -d ' ' || true)"
if [[ "${cmdline}" != *"ros2 launch lumos_nav nav2_fastlio_bringup.launch.py"* ]] || [[ "${pgid}" != "${nav_pid}" ]]; then
    echo "拒绝停止：PID ${nav_pid} 不是受管的独立导航进程组。" >&2
    echo "实际命令：${cmdline}" >&2
    exit 1
fi

echo "正在停止导航 launch（进程组 ${nav_pid}）..."
kill -INT -- "-${nav_pid}" 2>/dev/null || true
for _ in {1..6}; do
    kill -0 -- "-${nav_pid}" 2>/dev/null || {
        cleanup_state
        echo "导航 launch 已正常停止。"
        exit 0
    }
    sleep 1
done

echo "导航在 6 秒内未完全退出，发送 SIGTERM..."
kill -TERM -- "-${nav_pid}" 2>/dev/null || true
for _ in {1..4}; do
    kill -0 -- "-${nav_pid}" 2>/dev/null || {
        cleanup_state
        echo "导航 launch 已停止。"
        exit 0
    }
    sleep 1
done

echo "导航仍未退出，强制停止受管进程组。"
kill -KILL -- "-${nav_pid}" 2>/dev/null || true
sleep 1
cleanup_state
if kill -0 -- "-${nav_pid}" 2>/dev/null; then
    echo "导航停止复核失败：进程组 ${nav_pid} 仍存在。" >&2
    exit 1
fi
echo "导航 launch 已强制停止并确认无进程组残留。"


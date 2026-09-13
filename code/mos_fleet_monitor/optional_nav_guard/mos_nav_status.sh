#!/usr/bin/env bash

set -u

pid_file="/run/mos_navigation.pid"
status_file="/run/mos_navigation.status"
lock_dir="/run/lock/mos_navigation.instance"

if [[ -s "${pid_file}" ]]; then
    read -r nav_pid < "${pid_file}"
    if [[ "${nav_pid}" =~ ^[0-9]+$ ]] && [[ -d "/proc/${nav_pid}" ]]; then
        cmdline="$(tr '\0' ' ' < "/proc/${nav_pid}/cmdline" 2>/dev/null || true)"
        state="$(awk '{print $3}' "/proc/${nav_pid}/stat" 2>/dev/null || true)"
        if [[ "${state}" != "Z" ]] && [[ "${cmdline}" == *"ros2 launch lumos_nav nav2_fastlio_bringup.launch.py"* ]]; then
            if [[ -r "${status_file}" ]]; then
                echo "正在运行：$(<"${status_file}")"
            else
                echo "正在运行：pid=${nav_pid}"
            fi
            exit 0
        fi
    fi
fi

rm -f -- "${pid_file}" "${status_file}"
if [[ -d "${lock_dir}" ]]; then
    lock_pid="$(<"${lock_dir}/pid" 2>/dev/null || true)"
    if [[ -z "${lock_pid}" ]] || [[ ! -d "/proc/${lock_pid}" ]]; then
        rm -f -- "${lock_dir}/pid"
        rmdir -- "${lock_dir}" 2>/dev/null || true
    fi
fi
echo "未运行：导航 launch 当前可以启动。"
exit 3


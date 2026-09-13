#!/usr/bin/env bash

# Source-time guard for the vendor navigation environment script.
#
# The original setup script actively kills leftover navigation processes. A
# launch-file-only lock is therefore too late: a second operator could damage
# the running navigation stack before ros2 launch reaches its guard. This
# wrapper refuses first, then sources the preserved vendor implementation only
# when navigation is not running.

_mos_nav_lock_pid="/run/lock/mos_navigation.instance/pid"
_mos_nav_pid_file="/run/mos_navigation.pid"
_mos_nav_setup_impl="/tmp/mos_all_test_ws/setup_nav2_fastlio_impl.sh"

_mos_nav_setup_live_pid=""
for _mos_nav_setup_pid_path in \
    "${_mos_nav_lock_pid}" \
    "${_mos_nav_pid_file}"
do
    if [[ -s "${_mos_nav_setup_pid_path}" ]]; then
        IFS= read -r _mos_nav_setup_candidate < "${_mos_nav_setup_pid_path}"
        if [[ "${_mos_nav_setup_candidate}" =~ ^[0-9]+$ ]] \
            && [[ -d "/proc/${_mos_nav_setup_candidate}" ]]; then
            _mos_nav_setup_cmdline="$(
                tr '\0' ' ' < "/proc/${_mos_nav_setup_candidate}/cmdline" \
                    2>/dev/null || true
            )"
            if [[ "${_mos_nav_setup_cmdline}" == *"ros2 launch lumos_nav"* ]] \
                && [[ "${_mos_nav_setup_cmdline}" == *"nav2_fastlio_bringup.launch.py"* ]]; then
                _mos_nav_setup_live_pid="${_mos_nav_setup_candidate}"
                break
            fi
        fi
    fi
done

if [[ -n "${_mos_nav_setup_live_pid}" ]]; then
    echo >&2
    echo "[禁止] 导航已经运行（PID ${_mos_nav_setup_live_pid}）。" >&2
    echo "禁止再次执行 setup_nav2_fastlio.sh，以免其清理正在运行的导航子进程。" >&2
    echo >&2
    unset _mos_nav_lock_pid _mos_nav_pid_file _mos_nav_setup_impl
    unset _mos_nav_setup_pid_path _mos_nav_setup_candidate
    unset _mos_nav_setup_cmdline _mos_nav_setup_live_pid
    return 73 2>/dev/null || exit 73
fi

if [[ ! -f "${_mos_nav_setup_impl}" ]]; then
    echo "找不到原始导航环境脚本：${_mos_nav_setup_impl}" >&2
    return 1 2>/dev/null || exit 1
fi

unset _mos_nav_lock_pid _mos_nav_pid_file
unset _mos_nav_setup_pid_path _mos_nav_setup_candidate
unset _mos_nav_setup_cmdline _mos_nav_setup_live_pid

# shellcheck disable=SC1090
source "${_mos_nav_setup_impl}"
_mos_nav_setup_result=$?
unset _mos_nav_setup_impl
return "${_mos_nav_setup_result}" 2>/dev/null || exit "${_mos_nav_setup_result}"

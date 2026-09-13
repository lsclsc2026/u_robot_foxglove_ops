#!/usr/bin/env bash

set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LAUNCH_DIR="/tmp/mos_all_test_ws/install/lumos_nav/share/lumos_nav/launch"
TARGET="${LAUNCH_DIR}/nav2_fastlio_bringup.launch.py"
IMPL="${LAUNCH_DIR}/nav2_fastlio_bringup_impl.launch.py"
SETUP_TARGET="/tmp/mos_all_test_ws/setup_nav2_fastlio.sh"
SETUP_IMPL="/tmp/mos_all_test_ws/setup_nav2_fastlio_impl.sh"

# Preflight both targets before changing either one, so a setup-script conflict
# cannot leave only half of the navigation protection installed.
if [[ ! -f "${SETUP_TARGET}" ]]; then
  echo "找不到 ${SETUP_TARGET}" >&2
  exit 1
fi
if grep -q "Source-time guard for the vendor navigation environment script" \
  "${SETUP_TARGET}"; then
  if [[ ! -f "${SETUP_IMPL}" ]]; then
    echo "导航 setup 保护存在，但原始实现缺失：${SETUP_IMPL}" >&2
    exit 2
  fi
elif [[ -e "${SETUP_IMPL}" ]]; then
  echo "${SETUP_IMPL} 已存在但当前 setup 不是保护包装器，拒绝覆盖。" >&2
  exit 2
fi

if [[ ! -f "${TARGET}" ]]; then
  echo "找不到 ${TARGET}" >&2
  exit 1
fi
launch_guard_installed=0
if grep -q 'LOCK_DIR = Path("/run/lock/mos_navigation.instance")' "${TARGET}"; then
  if [[ ! -f "${IMPL}" ]]; then
    echo "导航 launch 保护存在，但原始实现缺失：${IMPL}" >&2
    exit 2
  fi
  launch_guard_installed=1
else
  if grep -q "Single-instance guard" "${TARGET}"; then
    if [[ ! -f "${IMPL}" ]]; then
      echo "检测到旧保护，但找不到原始实现 ${IMPL}；拒绝更新。" >&2
      exit 2
    fi
    backup="${TARGET}.before_directory_lock_$(date +%Y%m%d_%H%M%S)"
    cp -a -- "${TARGET}" "${backup}"
    echo "旧导航保护已备份：${backup}"
  else
    if [[ -e "${IMPL}" ]]; then
      echo "${IMPL} 已存在，拒绝覆盖；请人工核对。" >&2
      exit 2
    fi
    cp -a -- "${TARGET}" "${IMPL}"
  fi
fi

if (( launch_guard_installed == 0 )); then
  install -m 0644 "${SCRIPT_DIR}/nav2_fastlio_bringup.launch.py" "${TARGET}"
else
  echo "导航 launch 原子目录锁已经安装。"
fi

if grep -q "Source-time guard for the vendor navigation environment script" \
  "${SETUP_TARGET}"; then
  :
else
  cp -a -- "${SETUP_TARGET}" "${SETUP_IMPL}"
  install -m 0644 "${SCRIPT_DIR}/setup_nav2_fastlio.sh" "${SETUP_TARGET}"
fi

install -m 0755 "${SCRIPT_DIR}/mos_nav_status.sh" \
  /root/work_space/mos_nav_status.sh
install -m 0755 "${SCRIPT_DIR}/mos_nav_stop.sh" \
  /root/work_space/mos_nav_stop.sh

echo "导航 launch 锁和 setup 前置保护均已安装。"
echo "重新 colcon build lumos_nav 后可能需要重新安装该保护。"

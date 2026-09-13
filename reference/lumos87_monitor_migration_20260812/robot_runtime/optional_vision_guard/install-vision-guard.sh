#!/usr/bin/env bash

set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LAUNCH_DIR="/tmp/mos_all_test_ws/install/mos_3d_object/share/mos_3d_object/launch"
TARGET="${LAUNCH_DIR}/vision.launch.py"
IMPL="${LAUNCH_DIR}/vision_impl.launch.py"

if [[ ! -f "${TARGET}" ]]; then
  echo "找不到 ${TARGET}" >&2
  exit 1
fi
if grep -q 'LOCK_DIR = Path("/run/lock/mos_vision.instance")' "${TARGET}"; then
  echo "Vision 原子目录锁单实例保护已经安装。"
  exit 0
fi

if grep -q "Single-instance guard" "${TARGET}"; then
  if [[ ! -f "${IMPL}" ]]; then
    echo "检测到旧保护，但找不到原始实现 ${IMPL}；拒绝更新。" >&2
    exit 2
  fi
  backup="${TARGET}.before_directory_lock_$(date +%Y%m%d_%H%M%S)"
  cp -a -- "${TARGET}" "${backup}"
  echo "旧文件锁保护已备份：${backup}"
else
  if [[ -e "${IMPL}" ]]; then
    echo "${IMPL} 已存在，拒绝覆盖；请人工核对。" >&2
    exit 2
  fi
  cp -a -- "${TARGET}" "${IMPL}"
fi

install -m 0644 "${SCRIPT_DIR}/vision.launch.py" "${TARGET}"
install -m 0755 "${SCRIPT_DIR}/mos_vision_stop.sh" \
  /root/work_space/mos_vision_stop.sh
install -m 0755 "${SCRIPT_DIR}/mos_vision_status.sh" \
  /root/work_space/mos_vision_status.sh

echo "Vision 单实例保护已安装到 install 空间。"
echo "注意：重新 colcon build mos_3d_object 后可能需要重新安装该保护。"

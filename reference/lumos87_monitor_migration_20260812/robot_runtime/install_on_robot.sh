#!/usr/bin/env bash

set -Eeuo pipefail

PACKAGE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER_NAME="${MOS_CONTAINER_NAME:-lumos_dev}"
HOST_TARGET="${MOS_MONITOR_HOST_TARGET:-${HOME}/mos_fleet_monitor}"
CONTAINER_TARGET="/root/work_space/mos_fleet_monitor"

check_only=0
install_guards=0
guards_only=0
for argument in "$@"; do
  case "${argument}" in
    --check-only) check_only=1 ;;
    --with-guards) install_guards=1 ;;
    --guards-only) guards_only=1; install_guards=1 ;;
    *)
      echo "未知参数：${argument}" >&2
      echo "用法：$0 [--check-only] [--with-guards|--guards-only]" >&2
      exit 64
      ;;
  esac
done

docker inspect "${CONTAINER_NAME}" >/dev/null 2>&1 || {
  echo "找不到容器 ${CONTAINER_NAME}；请先执行 lumos run。" >&2
  exit 1
}

if [[ "$(docker inspect -f '{{.State.Running}}' "${CONTAINER_NAME}")" != "true" ]]; then
  docker start "${CONTAINER_NAME}" >/dev/null
fi

ensure_guard_targets_idle() {
  local conflicts
  conflicts="$(
    docker exec "${CONTAINER_NAME}" pgrep -af \
      'ros2 launch mos_3d_object vision.launch.py|mos_3d_object_node|ros2 launch lumos_nav nav2_fastlio_bringup.launch.py|gicp_localization|fastlio_mapping|controller_server|planner_server|bt_navigator' \
      2>/dev/null || true
  )"
  if [[ -n "${conflicts}" ]]; then
    echo "检测到 Vision/导航仍在运行，拒绝安装启动锁：" >&2
    printf '%s\n' "${conflicts}" >&2
    echo "请先正常停止相关 launch，再重新执行安装。" >&2
    return 1
  fi
}

required=(
  /opt/ros/humble/setup.zsh
  /tmp/mos_all_test_ws/install/setup.zsh
  /tmp/mos_all_test_ws/setup_nav2_fastlio.sh
  /tmp/mos_all_test_ws/unsetup_nav2.sh
  /root/work_space/mos_fleet_monitor/assets/foxglove_image_relay.py
  /root/work_space/mos_fleet_monitor/assets/mos_status_logger.py
)

base_required=(
  /opt/ros/humble/setup.zsh
  /tmp/mos_all_test_ws/install/setup.zsh
  /tmp/mos_all_test_ws/setup_nav2_fastlio.sh
  /tmp/mos_all_test_ws/unsetup_nav2.sh
)

missing=0
for path in "${base_required[@]}"; do
  if ! docker exec "${CONTAINER_NAME}" test -e "${path}"; then
    echo "[缺失] 容器内 ${path}" >&2
    missing=1
  fi
done
if (( missing )); then
  echo "目标机 ROS/导航基础环境不完整，未部署。" >&2
  exit 1
fi

if (( check_only )); then
  echo "基础路径检查通过。继续检查 ROS/Python 依赖："
  check_failed=0
  for ros_package in mos_3d_object lumos_nav foxglove_bridge; do
    if prefix="$(docker exec "${CONTAINER_NAME}" zsh -lc "
      source /opt/ros/humble/setup.zsh
      source /tmp/mos_all_test_ws/install/setup.zsh
      ros2 pkg prefix ${ros_package}
    " 2>/dev/null)"; then
      echo "[通过] ROS 包 ${ros_package}：${prefix}"
    else
      echo "[缺失] ROS 包 ${ros_package}" >&2
      check_failed=1
    fi
  done
  if docker exec "${CONTAINER_NAME}" zsh -lc '
    source /opt/ros/humble/setup.zsh
    source /tmp/mos_all_test_ws/install/setup.zsh
    python3 -c "import cv2, cv_bridge, diagnostic_msgs, rclpy"
  ' >/dev/null 2>&1; then
    echo "[通过] Python 模块 cv2、cv_bridge、diagnostic_msgs、rclpy"
  else
    echo "[缺失] Python 模块 cv2/cv_bridge/diagnostic_msgs/rclpy 中至少一项不可用" >&2
    check_failed=1
  fi
  if (( check_failed )); then
    echo "依赖检查失败，尚未执行部署。" >&2
    exit 1
  fi
  echo "全部部署前检查通过。"
  exit 0
fi

if (( guards_only )); then
  if ! docker exec "${CONTAINER_NAME}" test -d "${CONTAINER_TARGET}"; then
    echo "容器内尚未部署监控包：${CONTAINER_TARGET}" >&2
    exit 2
  fi
  ensure_guard_targets_idle
  docker exec "${CONTAINER_NAME}" mkdir -p \
    "${CONTAINER_TARGET}/guards/vision" \
    "${CONTAINER_TARGET}/guards/navigation"
  docker cp "${PACKAGE_ROOT}/optional_vision_guard/." \
    "${CONTAINER_NAME}:${CONTAINER_TARGET}/guards/vision/"
  docker cp "${PACKAGE_ROOT}/optional_nav_guard/." \
    "${CONTAINER_NAME}:${CONTAINER_TARGET}/guards/navigation/"
  docker exec "${CONTAINER_NAME}" chmod 0755 \
    "${CONTAINER_TARGET}/guards/vision/install-vision-guard.sh" \
    "${CONTAINER_TARGET}/guards/vision/mos_vision_status.sh" \
    "${CONTAINER_TARGET}/guards/vision/mos_vision_stop.sh" \
    "${CONTAINER_TARGET}/guards/navigation/install-nav-guard.sh" \
    "${CONTAINER_TARGET}/guards/navigation/mos_nav_status.sh" \
    "${CONTAINER_TARGET}/guards/navigation/mos_nav_stop.sh"
  docker exec "${CONTAINER_NAME}" \
    "${CONTAINER_TARGET}/guards/vision/install-vision-guard.sh"
  docker exec "${CONTAINER_NAME}" \
    "${CONTAINER_TARGET}/guards/navigation/install-nav-guard.sh"
  echo "Vision 与导航单实例保护安装完成；未启动 ROS 节点。"
  exit 0
fi

if [[ -e "${HOST_TARGET}" ]]; then
  echo "目标目录已存在：${HOST_TARGET}" >&2
  echo "为防止覆盖现有部署，本安装器已退出；请先人工比较版本。" >&2
  exit 2
fi

if docker exec "${CONTAINER_NAME}" test -e "${CONTAINER_TARGET}"; then
  echo "容器目标目录已存在：${CONTAINER_TARGET}" >&2
  echo "为防止覆盖现有部署，本安装器已退出；请先人工比较版本。" >&2
  exit 2
fi

mkdir -p -- "${HOST_TARGET}/bin"
install -m 0755 "${PACKAGE_ROOT}/host/bin/mos-monitor-start" \
  "${HOST_TARGET}/bin/mos-monitor-start"
install -m 0755 "${PACKAGE_ROOT}/host/bin/mos-monitor-stop" \
  "${HOST_TARGET}/bin/mos-monitor-stop"
install -m 0755 "${PACKAGE_ROOT}/host/bin/mos-monitor-status" \
  "${HOST_TARGET}/bin/mos-monitor-status"

docker exec "${CONTAINER_NAME}" mkdir -p -- "${CONTAINER_TARGET}"
docker cp "${PACKAGE_ROOT}/container/." \
  "${CONTAINER_NAME}:${CONTAINER_TARGET}/"
docker exec "${CONTAINER_NAME}" chmod 0755 \
  "${CONTAINER_TARGET}/bin/_common.sh" \
  "${CONTAINER_TARGET}/bin/_run_component.zsh" \
  "${CONTAINER_TARGET}/bin/_supervise_component.sh" \
  "${CONTAINER_TARGET}/bin/mos-monitor-start" \
  "${CONTAINER_TARGET}/bin/mos-monitor-stop" \
  "${CONTAINER_TARGET}/bin/mos-monitor-status"

docker exec "${CONTAINER_NAME}" mkdir -p \
  "${CONTAINER_TARGET}/guards/vision" \
  "${CONTAINER_TARGET}/guards/navigation"
docker cp "${PACKAGE_ROOT}/optional_vision_guard/." \
  "${CONTAINER_NAME}:${CONTAINER_TARGET}/guards/vision/"
docker cp "${PACKAGE_ROOT}/optional_nav_guard/." \
  "${CONTAINER_NAME}:${CONTAINER_TARGET}/guards/navigation/"
docker exec "${CONTAINER_NAME}" chmod 0755 \
  "${CONTAINER_TARGET}/guards/vision/install-vision-guard.sh" \
  "${CONTAINER_TARGET}/guards/vision/mos_vision_status.sh" \
  "${CONTAINER_TARGET}/guards/vision/mos_vision_stop.sh" \
  "${CONTAINER_TARGET}/guards/navigation/install-nav-guard.sh" \
  "${CONTAINER_TARGET}/guards/navigation/mos_nav_status.sh" \
  "${CONTAINER_TARGET}/guards/navigation/mos_nav_stop.sh"

if (( install_guards )); then
  echo "安装 Vision 与导航单实例保护..."
  ensure_guard_targets_idle
  docker exec "${CONTAINER_NAME}" \
    "${CONTAINER_TARGET}/guards/vision/install-vision-guard.sh"
  docker exec "${CONTAINER_NAME}" \
    "${CONTAINER_TARGET}/guards/navigation/install-nav-guard.sh"
fi

echo "部署完成："
echo "  宿主机入口：${HOST_TARGET}/bin"
echo "  容器内程序：${CONTAINER_TARGET}"
echo "注意：本安装器没有启动任何 ROS 节点，也没有修改地图或导航参数。"
if (( install_guards )); then
  echo "已备份原始实现并安装 Vision、导航 launch 锁及导航 setup 前置保护。"
else
  echo "单实例保护尚未安装；可在部署目录执行：$0 --guards-only"
fi

#!/usr/bin/env zsh

# Colcon-generated setup.zsh files intentionally probe unset trace variables,
# so nounset (-u) cannot be enabled while sourcing the ROS environment.
set -e
setopt pipefail

ROOT="${MOS_MONITOR_ROOT:-/root/work_space/mos_fleet_monitor}"
source "${ROOT}/config/monitor.env"

# `lumos run` normally injects these through interactive /root/.zshrc. A
# docker-exec based launcher is non-interactive, so reproduce only the SDK
# library paths instead of sourcing Oh My Zsh, tmux and completion hooks.
export PKG_CONFIG_PATH="/opt/lumos/third_party/lib/pkgconfig:/opt/lumos/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
export LD_LIBRARY_PATH="/opt/lumos/third_party/lib:/opt/lumos/lib:${LD_LIBRARY_PATH:-}"

# Never allow these managed processes to produce legacy core.PID files.
ulimit -c 0
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}"

component="${1:-}"

case "${component}" in
  vision)
    source /opt/ros/humble/setup.zsh
    source /tmp/mos_all_test_ws/install/setup.zsh
    exec ros2 launch mos_3d_object vision.launch.py
    ;;

  navigation)
    source /tmp/mos_all_test_ws/unsetup_nav2.sh
    cd /tmp/mos_all_test_ws
    # setup_nav2_fastlio.sh resolves install/setup.zsh from $PWD.
    source /tmp/mos_all_test_ws/setup_nav2_fastlio.sh
    source install/setup.zsh
    exec ros2 launch lumos_nav nav2_fastlio_bringup.launch.py \
      "rviz:=${MOS_NAV_RVIZ}"
    ;;

  image_relay)
    source /opt/ros/humble/setup.zsh
    source /tmp/mos_all_test_ws/install/setup.zsh
    exec python3 "${ROOT}/assets/foxglove_image_relay.py" \
      --ros-args --disable-external-lib-logs \
      -p "jpeg_quality:=${MOS_IMAGE_JPEG_QUALITY}" \
      -p "max_fps:=${MOS_IMAGE_MAX_FPS}" \
      -p "max_frame_age:=${MOS_IMAGE_MAX_FRAME_AGE}" \
      -p "head_width:=${MOS_IMAGE_HEAD_WIDTH}" \
      -p "head_height:=${MOS_IMAGE_HEAD_HEIGHT}" \
      -p "arm_width:=${MOS_IMAGE_ARM_WIDTH}" \
      -p "arm_height:=${MOS_IMAGE_ARM_HEIGHT}"
    ;;

  status_logger)
    source /opt/ros/humble/setup.zsh
    source /tmp/mos_all_test_ws/install/setup.zsh
    exec python3 "${ROOT}/assets/mos_status_logger.py" \
      --ros-args --disable-external-lib-logs \
      -p "interval_seconds:=${MOS_STATUS_INTERVAL}" \
      -p "graph_interval_seconds:=${MOS_STATUS_GRAPH_INTERVAL}"
    ;;

  bridge)
    source /opt/ros/humble/setup.zsh
    source /tmp/mos_all_test_ws/install/setup.zsh
    whitelist="['^/rosout$', '^/mos/monitor/(status_log|diagnostics)$', '^/foxglove/.*$', '^/(tf|tf_static|map|scan|cloud_registered|cloud_map|localization|localization_confidence|map_to_odom|odom|Odometry|plan|local_plan|global_plan|goal_pose|waypoints)$', '^/(global_costmap|local_costmap)/.*$']"
    exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
      "port:=${MOS_BRIDGE_PORT}" \
      "address:=${MOS_BRIDGE_ADDRESS}" \
      min_qos_depth:=1 \
      max_qos_depth:=1 \
      "topic_whitelist:=${whitelist}"
    ;;

  *)
    print -u2 -- "未知组件：${component}"
    exit 64
    ;;
esac

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
    head_camera=/dev/video0
    left_camera=/dev/video6
    right_camera=/dev/video12
    for camera_path in "${head_camera}" "${left_camera}" "${right_camera}"; do
      [[ "${camera_path}" == /dev/video<-> && -c "${camera_path}" ]] || {
        print -u2 -- "固定相机设备不存在：${camera_path}"
        exit 1
      }
    done
    runtime_sensor_config=/run/mos_fleet_monitor/mos_sensor_runtime.yaml
    {
      print -- 'bind_cpu: 9,10,11'
      print -- ''
      print -- 'camera:'
      print -- '  -'
      print -- '    name: "HEAD_CAMERA"'
      print -- '    device_path: "/dev/video0"'
      print -- '    frequency_hz: 30'
      print -- '    have_color: true'
      print -- '    have_depth: true'
      print -- '    stream: {width: 1280, height: 720, stream_fps: 30, rtmp_url: "rtmp://localhost:41935/cam1"}'
      print -- '  -'
      print -- '    name: "LEFT_ARM_CAMERA"'
      print -- '    device_path: "/dev/video6"'
      print -- '    frequency_hz: 30'
      print -- '    have_color: true'
      print -- '    have_depth: true'
      print -- '    stream: {width: 640, height: 480, stream_fps: 30, rtmp_url: "rtmp://localhost:41935/cam2"}'
      print -- '  -'
      print -- '    name: "RIGHT_ARM_CAMERA"'
      print -- '    device_path: "/dev/video12"'
      print -- '    frequency_hz: 30'
      print -- '    have_color: true'
      print -- '    have_depth: true'
      print -- '    stream: {width: 640, height: 480, stream_fps: 30, rtmp_url: "rtmp://localhost:41935/cam0"}'
    } > "${runtime_sensor_config}"
    source /opt/ros/humble/setup.zsh
    source /tmp/mos_all_test_ws/install/setup.zsh
    vision_share=/tmp/mos_all_test_ws/install/mos_3d_object/share/mos_3d_object
    exec ros2 run mos_3d_object mos_3d_object_node --ros-args \
      -r __node:=mos_3d_object \
      --params-file "${vision_share}/config/yolo_params.yaml" \
      -p "config:=${runtime_sensor_config}" \
      -p "weights:=${vision_share}/weights/r1g2_a06_0.pt" \
      -p "static_extrinsic_file:=${vision_share}/config/static_extrinsic.yaml"
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

  teleop)
    source /opt/ros/humble/setup.zsh
    source /tmp/mos_all_test_ws/install/setup.zsh
    export AMENT_PREFIX_PATH="/tmp/mos_all_test_ws/install/mos_ops_control:${AMENT_PREFIX_PATH:-}"
    export PYTHONPATH="/tmp/mos_all_test_ws/build/mos_ops_control:${PYTHONPATH:-}"
    exec ros2 run mos_ops_control ops_control \
      --ros-args --disable-external-lib-logs \
      -p "max_linear_x:=${MOS_TELEOP_MAX_LINEAR}" \
      -p "max_angular_z:=${MOS_TELEOP_MAX_ANGULAR}" \
      -p "command_timeout:=${MOS_TELEOP_COMMAND_TIMEOUT}"
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

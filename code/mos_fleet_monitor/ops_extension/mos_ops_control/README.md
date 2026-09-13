# MOS Foxglove operations control

This package adds guarded Foxglove controls without changing native Lumos,
Nav2, SLAM, arm-controller, or hardware-controller source code.

## Foxglove panels

- Teleop: `/foxglove/teleop/cmd_vel` (`geometry_msgs/msg/Twist`), 10 Hz,
  stop on release enabled.
- Enable teleop: `/foxglove/ops/teleop_enable` (`std_msgs/msg/Bool`), message
  `{"data":true}`.
- Disable teleop: `/foxglove/ops/teleop_disable` (`std_msgs/msg/Bool`), message
  `{"data":true}`.
- Status: `/foxglove/ops/status` (`std_msgs/msg/String`).

Enabling teleop scans for existing Nav2, SLAM, and keyboard teleop process
groups. Any detected motion sources are temporarily stopped and zero velocity
is published before Foxglove commands are accepted. If no navigation or other
motion source exists, Foxglove teleop is enabled directly. Disabling teleop
gates commands, publishes zero, and resumes only the process groups stopped by
this node. The monitor never launches a second navigation stack merely to
support teleop. A 0.5 second command watchdog remains active and there is no
30 second authorization lease.

## Existing workspace environment

This workspace uses an older isolated-install aggregate setup which may omit a
newly selected package from `AMENT_PREFIX_PATH`. After building, use:

```zsh
source /opt/ros/humble/setup.zsh
source /tmp/mos_all_test_ws/install/setup.zsh
export AMENT_PREFIX_PATH=/tmp/mos_all_test_ws/install/mos_ops_control:${AMENT_PREFIX_PATH:-}
export PYTHONPATH=/tmp/mos_all_test_ws/build/mos_ops_control:${PYTHONPATH:-}
ros2 pkg prefix mos_ops_control
```

The monitor component runner exports these paths automatically.

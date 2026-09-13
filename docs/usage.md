# 使用说明

以下客户端命令在 `code/mos_fleet_monitor` 目录执行。`robot.example` 均需替换为可达设备地址；操作前按安装与配置文档准备目标环境。

## 仅连接已有 Bridge

这条命令只建 SSH 隧道，不调用远端组件启动器：

```bash
ssh -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=15 \
  -L 8765:127.0.0.1:8765 lumos@robot.example
```

Foxglove 新建 WebSocket 连接，地址 `ws://localhost:8765`。若当前 Bridge 未运行，应由现场既定启动流程处理。

## 启动、状态与停止

允许启动视觉和导航时：

```bash
./client/start-robot-monitor.sh robot.example 8765
```

启动器复用已运行组件并补启动缺失组件，然后建立隧道。输出成功或退出码 0 不能单独证明所有组件就绪，应查看状态与数据。

```bash
ssh lumos@robot.example /home/lumos/mos_fleet_monitor/bin/mos-monitor-status
./client/stop-robot-monitor.sh robot.example
```

Ctrl+C 只关闭客户端连接。`stop-robot-monitor.sh` 会停止受管 ROS 组件，应等停止输出结束后再重新启动；外部进程可能仍存在。

## 多机器人

编辑 `client/robots.conf`，准备每台机器 SSH 公钥认证后：

```bash
./client/start-fleet-monitor.sh
./client/list-foxglove-links.sh
./client/stop-fleet-monitor.sh
```

每台机器人可使用相同远端端口 8765，但客户端本地端口必须唯一。后台“已提交”表示启动任务已提交，逐台结果查看 `${XDG_RUNTIME_DIR:-/tmp}/mos-fleet-monitor-${UID}` 中日志和远端状态。停止 fleet 会调用远端停止入口，不只是关闭浏览器数据源。

## Foxglove 面板和话题

| 用途 | 话题/设置 |
|---|---|
| 头部图像 | `/foxglove/head_camera/image/compressed` |
| 左臂图像 | `/foxglove/left_arm_camera/image/compressed` |
| 右臂图像 | `/foxglove/right_arm_camera/image/compressed` |
| 简短摘要 | `/mos/monitor/status_log`，`std_msgs/msg/String` |
| 详细状态 | `/mos/monitor/diagnostics`，`diagnostic_msgs/msg/DiagnosticArray` |
| ROS 日志 | `/rosout` |
| 地图 | `/map`、`/cloud_map`，以实际发布为准 |
| 实时环境 | `/cloud_registered`、`/scan` |
| TF 与位姿 | `/tf`、`/tf_static`、`/localization`、`/Odometry`、`/odom` |
| 导航 | `/plan`、其他实际路径话题、`/global_costmap/*`、`/local_costmap/*` |

3D 面板先以 `map` 为参考系，启用实际地图话题，再添加点云、TF 与轨迹；需要有真实消息和连通 TF。没有导出的布局 JSON，以上面板需自己创建并保存。图像输入是 `/{head,left_arm,right_arm}_camera/color/image_raw`；当前中继不处理原始深度。

## 显式遥控模式

切换模式前按现场流程停好受管栈，再启动：

```bash
./client/stop-robot-monitor.sh robot.example
./client/start-robot-teleop.sh robot.example 8765
```

当前 teleop 组件集合也包含 vision 和 navigation，仍会补启动缺失的这些组件。遥控 ROS 节点初始锁定，但启动和切换过程中会发布零速度；这是机器人控制入口。

在 Foxglove 配置：

| 面板行为 | 话题 | 消息 |
|---|---|---|
| 使能遥控 | `/foxglove/ops/teleop_enable` | `std_msgs/msg/Bool`，`{"data":true}` |
| 禁用遥控 | `/foxglove/ops/teleop_disable` | `std_msgs/msg/Bool`，`{"data":true}` |
| Teleop | `/foxglove/teleop/cmd_vel` | `geometry_msgs/msg/Twist`，建议 10 Hz，启用松手停止 |
| 状态 | `/foxglove/ops/status` | `std_msgs/msg/String` |

使能会按固定命令前缀扫描 Nav2、SLAM、键盘遥控等进程组，SIGSTOP 暂停匹配组并发送零速度；禁用关闭命令通道、发送零速度并 SIGCONT 恢复它记录的组。当前回调按“消息到达”触发，没有检查 Bool.data；不能向 enable 发 `false` 作为禁用，必须使用独立 disable 话题。

只转发限幅后的 `linear.x` 与 `angular.z` 到 `/cmd_vel`。默认上限 0.10 m/s 与 0.30 rad/s，0.5 秒无新命令触发停车，但不会因此取消使能；没有旧文档描述的 30 秒授权租约。节点还检查 `/cmd_vel` 订阅者是否有 `mos_hardware_controller`，缺失时退出使能。

进程前缀匹配不能覆盖所有潜在运动源，ROS 图检查也不等同硬件急停。退出 Foxglove 或关闭 SSH 隧道不等同明确禁用遥控；结束时先用 disable，再按需要停止受管栈，并确认原运动源恢复行为符合现场任务。

> 历史来源文档：保留原文供追溯，其中旧部署状态、遥控租约、相机与日志说明可能与当前代码不同。当前使用入口见[仓库 README](../../../../README.md)；不要将本页直接当成现行部署手册。

# Foxglove 安全远程操纵

## 目标与边界

本功能只控制机器人差速底盘，不控制双臂。导航、地图、点云、Vision、图像中继、状态和
Foxglove Bridge 仍可继续运行。

浏览器不会直接向 `/cmd_vel` 下发速度，而是向专用话题发送命令：

```text
/foxglove/teleop/cmd_vel
```

机器人端 `foxglove_teleop_guard.py` 负责：

- 默认不解锁，启动时主动发送零速度；
- 线速度限制为 `0.10 m/s`，角速度限制为 `0.30 rad/s`；
- 连续 `0.50 s` 收不到命令立即发送零速度；
- 每次解锁只持续 `30 s`，到期后停车并重新锁定；
- Foxglove 松开方向按钮时立即停车；
- 检测到另一个 `/cmd_vel` 底盘订阅者时拒绝解锁；
- 退出时重复发送零速度，再关闭 MosHal。

这些参数可在 `container/config/monitor.env` 中调整。首次实车测试应架空驱动轮或清空机器人
周围区域，并安排现场人员掌握实体急停。

## WSL 启动方式

先停止旧的监控总控栈，再启动遥控模式：

```bash
cd ~/learning/mos_fleet_monitor/client
./stop-robot-monitor.sh 100.127.82.99
./start-robot-teleop.sh 100.127.82.99 8765
```

连接地址仍然是：

```text
ws://localhost:8765
```

普通监控入口 `start-robot-monitor.sh` 不启动遥控组件。遥控入口
`start-robot-teleop.sh` 会启动普通监控的全部组件，再额外启动安全遥控组件。

退出遥控模式时，先在 Foxglove 点击“锁定/停车”，再执行：

```bash
./stop-robot-monitor.sh 100.127.82.99
```

确认停止完成后，才能重新运行普通监控入口。

## Foxglove 面板设置

### 1. 添加 Teleop 面板

在“添加面板”中选择 `Teleop`，设置：

```text
Topic: /foxglove/teleop/cmd_vel
Publish rate: 10 Hz
Stop on release: On
Up:    linear.x  =  0.08
Down:  linear.x  = -0.06
Left:  angular.z =  0.25
Right: angular.z = -0.25
Stop:  linear.x  =  0.00
```

面板发布的是 `geometry_msgs/msg/Twist`。即便界面数值被误设得更大，机器人端仍会按配置
二次限速。

### 2. 添加“解锁 30 秒”按钮

添加 `Publish` 面板：

```text
Topic: /foxglove/teleop/enable
Schema: std_msgs/msg/Bool
Button title: 解锁30秒
Message:
{"data":true}
```

点击只会进入等待命令状态，不会自行运动。30 秒后必须重新点击才能继续操作。

### 3. 添加“锁定/停车”按钮

再添加一个 `Publish` 面板：

```text
Topic: /foxglove/teleop/enable
Schema: std_msgs/msg/Bool
Button title: 锁定/停车
Message:
{"data":false}
```

### 4. 添加状态面板

添加 `Raw Messages` 面板并选择：

```text
/foxglove/teleop/status
```

只有看到 `"armed":true` 才表示当前授权有效。常见状态包括：

```text
DISARMED
ARMED_WAITING_FOR_COMMAND
ACTIVE
ARMED_STOPPED
COMMAND_TIMEOUT
LEASE_EXPIRED
CONTROL_CONFLICT
SHUTDOWN
```

`CONTROL_CONFLICT` 表示系统发现另一个底盘控制器。此时不要反复点击解锁，应先查清
`mos_arm_controller` 或 `cmd_vel_to_base.py` 是否在运行。

## 无运动验证

部署后先只检查节点和状态，不点击解锁按钮：

```bash
ssh lumos@100.127.82.99
docker exec lumos_dev zsh -lc '
source /opt/ros/humble/setup.zsh
source /tmp/mos_all_test_ws/install/setup.zsh
export ROS_DOMAIN_ID=111 ROS_LOCALHOST_ONLY=1
ros2 node list | grep foxglove_teleop_guard
ros2 topic echo /foxglove/teleop/status --once
ros2 topic info /foxglove/teleop/cmd_vel --verbose
'
```

预期状态为 `DISARMED`、`armed:false`、`moving:false`。

## 重要限制

- Tailscale/SSH 断开会让 Teleop 消息停止，0.50 秒 watchdog 随即停车；但远程安全措施不能
  替代实体急停和现场看护。
- 遥控期间不要启动 `mos_arm_controller`、`cmd_vel_to_base.py` 或其他直接底盘控制程序。
- 本功能不取消正在执行的 Nav2 目标；它绕开 Nav2 的 `/cmd_vel` 输出直接控制底盘。结束遥控
  前应确认导航任务没有处于待执行状态。
- 不要把机器人上的 Foxglove Bridge 端口直接暴露到公网；继续使用当前 SSH 隧道。


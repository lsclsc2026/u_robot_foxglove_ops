# 配置说明

仓库保留来源中的业务默认值。`100.127.82.99`、`172.18.21.87`、`172.18.21.210`、`lumos-87`、`mos-87` 以及 `/home/shuochen/learning` 均是原环境地址、机器名或路径示例，不承诺在新环境可用；使用者需按现场修改。它们不是认证凭据。

## 客户端与多机

单机脚本的第一参数是机器人地址，第二参数是本地端口，默认分别为 `100.127.82.99` 和 `8765`。客户端默认 `ROBOT_USER=lumos`、`REMOTE_PORT=8765`，普通启动路径为 `/home/lumos/mos_fleet_monitor/bin/mos-monitor-start`。

```bash
ROBOT_USER=lumos REMOTE_PORT=8765 \
  ./client/start-robot-monitor.sh robot.example 8766
```

多机配置 `client/robots.conf` 每行三个空白分隔字段：名称、可达地址、本地端口。名称和本地端口不能重复，`#` 开头为注释。例如：

```text
robot-a robot-a.example 8765
robot-b robot-b.example 8766
```

`client/fleet.env` 的 `FOXGLOVE_LAYOUT_ID` 可填写自己保存布局的 ID，留空只生成数据源链接。仓库没有布局 JSON，无法直接恢复演示布局。单机可以交互认证，多机后台脚本要求已配置 SSH 公钥认证。

## 容器运行参数

默认配置文件是 `code/mos_fleet_monitor/container/config/monitor.env`；部署后位于容器 `/root/work_space/mos_fleet_monitor/config/monitor.env`。它用 shell 默认赋值，容器调用环境里已有值时优先使用已有值。客户端的普通环境变量不会自动穿过 SSH 和 Docker；需要确认实际传递链，或修改部署配置。

| 参数 | 默认 | 含义 |
|---|---|---|
| `ROS_DOMAIN_ID` | `111` | 与目标 ROS 节点一致 |
| `ROS_LOCALHOST_ONLY` | `1` | ROS 本机通信设置 |
| `MOS_NAV_RVIZ` | `false` | 集成导航不启动 RViz |
| `MOS_BRIDGE_PORT` / `MOS_BRIDGE_ADDRESS` | `8765` / `127.0.0.1` | Bridge 监听 |
| `MOS_IMAGE_JPEG_QUALITY` | `58` | JPEG 质量 |
| `MOS_IMAGE_MAX_FPS` | `15.0` | 每路输出限频 |
| `MOS_IMAGE_MAX_FRAME_AGE` | `0.25` | 超龄输入帧丢弃阈值（秒） |
| `MOS_IMAGE_HEAD_WIDTH` / `HEIGHT` | `576` / `324` | 头部预览分辨率 |
| `MOS_IMAGE_ARM_WIDTH` / `HEIGHT` | `432` / `243` | 双臂预览分辨率 |
| `MOS_STATUS_INTERVAL` | `2.0` | 摘要/诊断刷新周期（秒） |
| `MOS_STATUS_GRAPH_INTERVAL` | `10.0` | ROS 图信息刷新周期（秒） |
| `MOS_EVENT_LOG_MAX_BYTES` | `1048576` | 事件日志截短阈值 |
| `MOS_MONITOR_MODE` | `normal` | 普通/teleop 组件集合 |
| `MOS_TELEOP_MAX_LINEAR` | `0.10` | `linear.x` 速度上限，m/s |
| `MOS_TELEOP_MAX_ANGULAR` | `0.30` | `angular.z` 速度上限，rad/s |
| `MOS_TELEOP_COMMAND_TIMEOUT` | `0.50` | 无新命令停车阈值（秒） |
| `MOS_TELEOP_ARM_TIMEOUT` | `0.0` | 保留参数；当前 ops_control runner 未传入，不提供授权租约 |
| `MOS_TELEOP_HAL_CONFIG` | `/opt/lumos/config/mos_hal/mos_p15.yaml` | 旧 HAL 遥控资产相关参数，当前 ops_control 不使用 |

## 相机和 Bridge 白名单

当前 `_run_component.zsh` 的 Vision 分支写死头部 `/dev/video0`、左臂 `/dev/video6`、右臂 `/dev/video12`，检查字符设备后生成 `/run/mos_fleet_monitor/mos_sensor_runtime.yaml`。头部配置 1280×720，双臂 640×480，三路均 30 Hz；这里是采集配置，和预览缩放参数不同。该配置只列相机，未列 lidar。迁移时必须核对设备映射和 SDK。

Bridge 白名单在 `_run_component.zsh` 中，允许 `/foxglove/*`、监控状态、ROS 日志、选定地图/导航/TF 与 costmap 话题；不含 `/livox/lidar` 及原始深度图像。白名单只改变对 Foxglove 的可见性，不修复上游同名不同类型或重复驱动。

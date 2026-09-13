> 历史来源文档：保留原文供追溯，其中旧部署状态、遥控租约、相机与日志说明可能与当前代码不同。当前使用入口见[仓库 README](../../README.md)；不要将本页直接当成现行部署手册。

# MOS 远程运维与 Foxglove 监控

本仓库保存 MOS 机器人远程监控所需的最小代码资产，用于在基础环境一致的机器人上恢复：

- Vision、导航、压缩图像、状态监控和 Foxglove Bridge 的统一启停；
- 已运行组件复用、缺失组件补启动、组件独立存活；
- 三路相机低延迟 JPEG 话题；
- CPU、内存、磁盘、温度、ROS 2、传感器和导航状态；
- Vision 单实例保护及 MOS 原生高速图像发布。

本仓库不包含地图、导航参数、模型权重、MOS SDK、ROS 2、Docker 镜像或 Foxglove Layout。

## 代码结构

```text
mos_ops/
├── 01_robot_runtime/
│   ├── host_entry/bin/                 # 机器人 Ubuntu 宿主机入口
│   └── mos_fleet_monitor/              # lumos_dev 容器内运行程序
│       ├── assets/                     # 图像中继与状态监控节点
│       ├── bin/                        # 五组件启停、状态及监督器
│       └── config/monitor.env          # ROS Domain、端口与图像参数
├── 02_wsl_tools/client/                # WSL SSH 隧道和多机入口
├── 03_mos_all_test_overrides/          # mos_3d_object 源码覆盖文件
├── FILE_INDEX.txt                      # 运维资产索引
└── ACTIVE_FILES.sha256                 # 运维资产 SHA-256
```

### 五个受管组件

| 组件 | 启动内容 | 主要输出 |
|---|---|---|
| `vision` | `mos_3d_object vision.launch.py` | RGB、深度、雷达和 IMU 原始话题 |
| `navigation` | `lumos_nav nav2_fastlio_bringup.launch.py` | 地图、定位、点云、TF 和导航话题 |
| `image_relay` | `foxglove_image_relay.py` | `/foxglove/*/image/compressed` |
| `status_logger` | `mos_status_logger.py` | `/mos/monitor/status_log`、`diagnostics` |
| `bridge` | `foxglove_bridge` | 本机 TCP 8765 WebSocket 服务 |

启动器采用幂等策略：已运行组件不重复启动，只补启动缺失组件。某个组件退出时写入有限大小的事件日志，不停止其他组件。

## 目标机器前提

部署前，目标机器必须已经具备与 lumos-87 同版本的基础环境：

```text
Ubuntu 宿主机用户：lumos
Docker 容器：lumos_dev
ROS 2：/opt/ros/humble
MOS 工作空间：/tmp/mos_all_test_ws
MOS 运行库：/opt/lumos
SLAM 环境：/opt/slam/apps/install
ROS 包：mos_3d_object、lumos_nav、foxglove_bridge
```

目标机还必须保留其自己的以下资产：

```text
/tmp/mos_all_test_ws/setup_nav2_fastlio.sh
/tmp/mos_all_test_ws/unsetup_nav2.sh
/tmp/mos_all_test_ws/src/lumos_nav/
相机序列号与标定文件
模型权重
地图、PCD 和导航参数
```

不要从 lumos-87 复制整套 `build/`、`install/`、地图或导航参数覆盖新机器。

## 一、在部署用 WSL 中获取仓库

推荐只在 WSL 保存 GitLab 凭据。机器人宿主机和 Docker 容器不需要保存 Git 私钥。

```bash
cd /home/shuochen/learning
git clone git@mos-gitlab:star_brain/mos_ops.git
cd /home/shuochen/learning/mos_ops
```

验证资产完整性：

```bash
sha256sum -c ACTIVE_FILES.sha256
```

所有项目都应显示 `OK`。

## 二、部署前检查目标机器人

以下示例使用新机器人 `172.18.21.210`：

```bash
export MOS_TARGET_IP=172.18.21.210
ssh lumos@${MOS_TARGET_IP} 'docker inspect lumos_dev >/dev/null'
```

检查基础路径：

```bash
ssh lumos@${MOS_TARGET_IP} "docker exec lumos_dev bash -lc '
  test -f /opt/ros/humble/setup.zsh &&
  test -f /tmp/mos_all_test_ws/install/setup.zsh &&
  test -f /tmp/mos_all_test_ws/setup_nav2_fastlio.sh &&
  test -f /tmp/mos_all_test_ws/unsetup_nav2.sh &&
  test -d /tmp/mos_all_test_ws/src/mos_3d_object &&
  test -d /tmp/mos_all_test_ws/src/lumos_nav
'"
```

检查 ROS 包：

```bash
ssh lumos@${MOS_TARGET_IP} "docker exec lumos_dev zsh -lc '
  source /opt/ros/humble/setup.zsh
  source /tmp/mos_all_test_ws/install/setup.zsh
  ros2 pkg prefix mos_3d_object
  ros2 pkg prefix lumos_nav
  ros2 pkg prefix foxglove_bridge
'"
```

## 三、停止目标机相关进程

覆盖 ROS 源码和重新编译前，Vision 与导航必须停止：

```bash
ssh lumos@${MOS_TARGET_IP} "docker exec lumos_dev pgrep -af \
  'vision.launch.py|mos_3d_object_node|nav2_fastlio_bringup.launch.py|gicp_localization|fastlio_mapping' || true"
```

如果已经部署过本监控包，使用：

```bash
ssh lumos@${MOS_TARGET_IP} \
  /home/lumos/mos_fleet_monitor/bin/mos-monitor-stop
```

如果是其他终端手工启动的进程，应回到原终端正常按 `Ctrl+C` 停止。不要在进程运行时覆盖文件或重新编译。

## 四、备份目标机原文件

创建带时间戳的备份，不删除目标机原内容：

```bash
ssh lumos@${MOS_TARGET_IP} "docker exec lumos_dev bash -lc '
  stamp=\$(date +%Y%m%d_%H%M%S)
  backup=/tmp/mos_all_test_ws/monitor_backups/\${stamp}
  mkdir -p \${backup}
  cd /tmp/mos_all_test_ws
  tar -czf \${backup}/mos_3d_object_before_monitor.tar.gz \
    src/mos_3d_object/mos_3d_object/vision_node.py \
    src/mos_3d_object/config/yolo_params.yaml \
    src/mos_3d_object/launch/vision.launch.py \
    2>/dev/null || true
  echo \${backup}
'"
```

记下输出的备份目录，回滚时使用。

## 五、部署宿主机入口

在仓库根目录执行：

```bash
ssh lumos@${MOS_TARGET_IP} 'mkdir -p /home/lumos/mos_fleet_monitor/bin'

scp 01_robot_runtime/host_entry/bin/* \
  lumos@${MOS_TARGET_IP}:/home/lumos/mos_fleet_monitor/bin/

ssh lumos@${MOS_TARGET_IP} \
  'chmod 0755 /home/lumos/mos_fleet_monitor/bin/*'
```

宿主机入口只负责启动 `lumos_dev` 并调用容器内程序。

## 六、部署容器运行文件

```bash
tar -C 01_robot_runtime -czf - mos_fleet_monitor | \
ssh lumos@${MOS_TARGET_IP} \
  'docker exec -i lumos_dev tar -C /root/work_space -xzf -'
```

设置权限：

```bash
ssh lumos@${MOS_TARGET_IP} "docker exec lumos_dev chmod 0755 \
  /root/work_space/mos_fleet_monitor/bin/_common.sh \
  /root/work_space/mos_fleet_monitor/bin/_run_component.zsh \
  /root/work_space/mos_fleet_monitor/bin/_supervise_component.sh \
  /root/work_space/mos_fleet_monitor/bin/mos-monitor-start \
  /root/work_space/mos_fleet_monitor/bin/mos-monitor-status \
  /root/work_space/mos_fleet_monitor/bin/mos-monitor-stop"
```

## 七、部署 Vision 覆盖文件

该目录只覆盖四个 `mos_3d_object` 文件，不包含目标机地图或导航参数：

```bash
tar -C 03_mos_all_test_overrides -czf - src | \
ssh lumos@${MOS_TARGET_IP} \
  'docker exec -i lumos_dev tar -C /tmp/mos_all_test_ws -xzf -'
```

重新编译 `mos_3d_object`：

```bash
ssh lumos@${MOS_TARGET_IP} "docker exec lumos_dev zsh -lc '
  source /opt/ros/humble/setup.zsh
  cd /tmp/mos_all_test_ws
  ulimit -c 0
  colcon build --packages-select mos_3d_object \
    --event-handlers console_direct+
'"
```

不要重新编译或覆盖 `lumos_nav`，除非目标机开发人员明确要求。

## 八、部署后检查

确认关键文件存在：

```bash
ssh lumos@${MOS_TARGET_IP} "docker exec lumos_dev bash -lc '
  test -x /root/work_space/mos_fleet_monitor/bin/mos-monitor-start &&
  test -f /root/work_space/mos_fleet_monitor/assets/foxglove_image_relay.py &&
  test -f /root/work_space/mos_fleet_monitor/assets/mos_status_logger.py &&
  test -f /tmp/mos_all_test_ws/install/mos_3d_object/share/mos_3d_object/launch/vision.launch.py
'"
```

检查 Vision 发布参数：

```bash
ssh lumos@${MOS_TARGET_IP} "docker exec lumos_dev grep -nE \
  'enable_sensor_ros_publisher|enable_python_rgb_publisher' \
  /tmp/mos_all_test_ws/install/mos_3d_object/share/mos_3d_object/config/yolo_params.yaml"
```

期望值：

```text
enable_sensor_ros_publisher: true
enable_python_rgb_publisher: false
```

## 九、配置新 WSL 与启动

新 WSL 需要自行配置到机器人宿主机的 SSH 公钥。私钥不在本仓库中。

设置客户端权限：

```bash
chmod 0755 02_wsl_tools/client/*.sh
```

单机启动并建立 Foxglove 隧道：

```bash
cd 02_wsl_tools/client
./start-robot-monitor.sh 172.18.21.210 8765
```

Foxglove 连接地址：

```text
ws://localhost:8765
```

按 `Ctrl+C` 只关闭本机 SSH 隧道，不停止机器人上的组件。

停止由总控登记的机器人组件：

```bash
./stop-robot-monitor.sh 172.18.21.210
```

查看机器人状态：

```bash
ssh lumos@172.18.21.210 \
  /home/lumos/mos_fleet_monitor/bin/mos-monitor-status
```

## 十、Foxglove 主要话题

```text
/foxglove/head_camera/image/compressed
/foxglove/left_arm_camera/image/compressed
/foxglove/right_arm_camera/image/compressed
/mos/monitor/status_log
/mos/monitor/diagnostics
/rosout
/map
/cloud_registered
/localization
/tf
/tf_static
```

Foxglove Layout 尚未纳入本仓库，需要后续单独导出和版本管理。

## 十一、配置参数

容器配置文件：

```text
/root/work_space/mos_fleet_monitor/config/monitor.env
```

主要默认值：

```text
ROS_DOMAIN_ID=111
ROS_LOCALHOST_ONLY=1
MOS_BRIDGE_PORT=8765
MOS_BRIDGE_ADDRESS=127.0.0.1
MOS_NAV_RVIZ=false
MOS_IMAGE_JPEG_QUALITY=58
MOS_IMAGE_MAX_FPS=15.0
MOS_IMAGE_MAX_FRAME_AGE=0.25
MOS_STATUS_INTERVAL=2.0
MOS_EVENT_LOG_MAX_BYTES=1048576
```

Bridge 只监听机器人回环地址，应通过 WSL SSH 隧道访问。

## 十二、运行日志与磁盘边界

生命周期事件日志：

```text
/run/mos_fleet_monitor/events.log
```

该日志默认限制为 1 MiB。`status_log` 只发布 ROS 2 最新状态，不保存历史数据。组件启动前执行 `ulimit -c 0`，但目标系统的其他进程及 Apport 配置仍需单独管理。

不要将以下内容提交 Git：

```text
SSH 私钥、密码、Token
患者或医院图像
地图与定位数据
core dump
ROS 运行日志
build/、install/、log/
/run 下的 PID 和状态文件
```

## 十三、更新与补启动

再次执行启动入口时：

- 已运行组件直接复用；
- 缺失组件单独启动；
- 单个组件启动失败不会停止其他组件；
- 单个组件退出后会记录日志，其他组件继续运行。

查看最近事件：

```bash
ssh lumos@${MOS_TARGET_IP} \
  "docker exec lumos_dev tail -n 50 /run/mos_fleet_monitor/events.log"
```

## 十四、回滚 Vision 文件

找到第四步生成的备份包，例如：

```text
/tmp/mos_all_test_ws/monitor_backups/20260812_120000/mos_3d_object_before_monitor.tar.gz
```

确保 Vision 和导航已经停止，然后在容器中恢复并重新编译：

```bash
ssh lumos@${MOS_TARGET_IP} "docker exec lumos_dev zsh -lc '
  cd /tmp/mos_all_test_ws
  tar -xzf /tmp/mos_all_test_ws/monitor_backups/时间戳/mos_3d_object_before_monitor.tar.gz
  source /opt/ros/humble/setup.zsh
  ulimit -c 0
  colcon build --packages-select mos_3d_object \
    --event-handlers console_direct+
'"
```

`01_robot_runtime` 与 `02_wsl_tools` 均为脚本文件；需要回滚时可从 Git 历史版本重新部署。

## 当前边界

- 当前没有统一管理 Foxglove Layout；
- 当前没有安装独立的导航 launch 锁，总控负责避免重复启动；
- 当前不自动重启崩溃组件，防止 Vision 重复执行相机硬件复位；
- 当前不自动删除 ROS 日志、core dump 或其他系统文件；
- 当前不携带目标机地图、定位、导航参数和模型权重。

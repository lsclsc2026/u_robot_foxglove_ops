> 历史来源文档：保留原文供追溯，其中旧部署状态、遥控租约、相机与日志说明可能与当前代码不同。当前使用入口见[仓库 README](../../README.md)；不要将本页直接当成现行部署手册。

# MOS 多机 Foxglove 监控启动包

本包完成当前阶段的两项工作：

1. 将导航、3D Vision、低延迟压缩图像、`status_log` 和 Foxglove Bridge
   封装为一套统一启停的监控栈；
2. 将本次新增脚本和部署入口整理成可迁移资产包。

另提供显式启用的 Foxglove 底盘遥控模式。它使用短时授权、速度上限、0.5 秒命令 watchdog
和控制器冲突检测；普通 `start-robot-monitor.sh` 不会启动遥控。完整配置见
`docs/2026-08-24/01-Foxglove安全远程操纵.md`。

它不修改导航、地图、定位参数、相机参数或原始业务节点。默认只会停止由本总控脚本
登记并启动的进程组，不会用 `pkill python3` 一类宽泛命令处理同事进程。

## 目录分层

```text
client/                       WSL 端一键连接脚本
host/bin/                     机器人 Ubuntu 宿主机入口
container/bin/                lumos_dev 容器内统一启停与状态
container/assets/             图像压缩与状态话题脚本
container/config/monitor.env  ROS Domain、Bridge 端口等参数
optional_vision_guard/        可选的 Vision 原命令单实例保护
optional_nav_guard/           导航 launch 锁与 setup 前置保护
layouts/                      Foxglove 标准布局 JSON 存放位置
install_on_robot.sh           新机器人部署器
build-package.sh              版本化迁移包生成器
```

新机器部署、`172.18.21.87`/`172.18.21.210` 多机启停、SSH 公钥、布局导出和
Foxglove 数据源切换说明见 `DEPLOYMENT_AND_MULTI_ROBOT.md`。

## 当前机器人上的使用方式

### WSL 一键启动并连接

先把 `client/` 保存在 WSL。赋予权限后执行：

```bash
chmod +x client/*.sh
./client/start-robot-monitor.sh 172.18.21.87 8765
```

它会在一条 SSH 连接中完成：

- 建立一个 SSH Master，并复用该认证连接调用机器人宿主机入口；
- 在 `lumos_dev` 容器中复用已运行组件，仅补启动缺失组件；
- 某个组件启动失败或后续退出时只记录事件，不停止其他组件；
- 建立 `localhost:8765 -> 机器人:8765` 转发；
- 保持隧道在线。

随后在 Foxglove 中连接：

```text
ws://localhost:8765
```

脚本先通过复用连接执行远端按需启动，再由 SSH Master 持有 `-L` 转发。密码只输入
一次。即使 Bridge 暂时退出，SSH 隧道和其他组件也会保持；重新执行启动入口补起 Bridge
后，无需重启其他健康组件。

按 `Ctrl+C` 只会关闭 WSL 隧道，机器人里的监控栈继续运行。要停止整个受管栈：

```bash
./client/stop-robot-monitor.sh 172.18.21.87
```

需要显式进入安全底盘遥控模式时使用：

```bash
./client/start-robot-teleop.sh 100.127.82.99 8765
```

遥控模式不是普通监控模式的默认能力；切换前后都应通过 `stop-robot-monitor.sh` 完整停止一次。

停止按受管进程组执行 `SIGINT → SIGTERM → SIGKILL`，最长约 11 秒。应等待看到
“MOS 受管监控栈已停止”后再重新启动。

### 机器人宿主机直接操作

```bash
/home/lumos/mos_fleet_monitor/bin/mos-monitor-start
/home/lumos/mos_fleet_monitor/bin/mos-monitor-status
/home/lumos/mos_fleet_monitor/bin/mos-monitor-stop
```

### 容器内直接操作

```bash
/root/work_space/mos_fleet_monitor/bin/mos-monitor-start
/root/work_space/mos_fleet_monitor/bin/mos-monitor-status
/root/work_space/mos_fleet_monitor/bin/mos-monitor-stop
```

## 幂等启动和独立存活策略

- 已经运行的受管组件：再次执行 `start` 时直接复用，不重复启动；
- 手工启动的 Vision、导航、图像中继、状态脚本或 Bridge：标记为外部运行并复用，
  继续补启动其他缺失组件；
- 上次受管栈只剩部分组件：删除失效 PID 记录，只补启动已经退出的组件；
- 任一组件启动失败或运行后退出：写入 `/run/mos_fleet_monitor/events.log`，其他组件
  和 WSL SSH 隧道继续运行；
- `stop` 根据 PID、命令标记和独立进程组共同校验，只停止本脚本登记的进程。

Vision 自身的单实例保护是第二道防线。它使用“原子目录 + 活跃 PID 校验”，避免普通
文件锁在 ROS launch 加载文件后提前释放。它放在 `optional_vision_guard/`，新机器可先
核对其 ROS 包版本再单独安装；总安装器不会自动改动原始 launch。

在目标机确认 Vision 当前未运行后，可显式安装：

```bash
docker cp optional_vision_guard lumos_dev:/tmp/optional_vision_guard
docker exec lumos_dev \
  /tmp/optional_vision_guard/install-vision-guard.sh
```

安装器会先保留原始实现为 `vision_impl.launch.py`；如果检测到旧版保护，会先创建带
时间戳的备份再升级。重新编译 `mos_3d_object` 可能覆盖 install 空间，届时需重新核对。

完整新机器部署建议直接使用：

```bash
./install_on_robot.sh --check-only
./install_on_robot.sh --with-guards
```

导航保护除 launch 原子锁外，还会备份并包装
`/tmp/mos_all_test_ws/setup_nav2_fastlio.sh`，使重复操作在原脚本清理导航子进程前被拒绝。

## 运行参数

默认值位于 `container/config/monitor.env`：

```text
ROS_DOMAIN_ID=111
ROS_LOCALHOST_ONLY=1
MOS_NAV_RVIZ=false
MOS_BRIDGE_PORT=8765
MOS_BRIDGE_ADDRESS=127.0.0.1
MOS_IMAGE_JPEG_QUALITY=58
MOS_IMAGE_MAX_FPS=15.0
MOS_IMAGE_MAX_FRAME_AGE=0.25
MOS_IMAGE_HEAD_WIDTH=576
MOS_IMAGE_HEAD_HEIGHT=324
MOS_IMAGE_ARM_WIDTH=432
MOS_IMAGE_ARM_HEIGHT=243
MOS_STATUS_INTERVAL=2.0
MOS_STATUS_GRAPH_INTERVAL=10.0
MOS_EVENT_LOG_MAX_BYTES=1048576
MOS_MONITOR_MODE=normal
MOS_TELEOP_MAX_LINEAR=0.10
MOS_TELEOP_MAX_ANGULAR=0.30
MOS_TELEOP_COMMAND_TIMEOUT=0.50
MOS_TELEOP_ARM_TIMEOUT=30.0
```

远程 Foxglove 使用时默认不启动导航自带 RViz，避免图形渲染和 NoMachine 额外占用。
Bridge 只监听机器人本机回环地址，必须通过 SSH 转发访问。

图像中继在未被 Foxglove 订阅时不做 JPEG 编码；运行时每路只保留最新一帧，
帧龄超过 0.25 秒直接丢弃。默认分辨率与质量是面向三路同时预览的平衡值。

`status_log` 每 2 秒刷新一行核心摘要；`/mos/monitor/diagnostics` 使用标准
`diagnostic_msgs/msg/DiagnosticArray` 按资源、导航、传感器、图像、组件和 ROS 分组展示详情。
DDS 两个话题都只保留最新 1 条；ROS 图谱每 10 秒采集一次。脚本不写状态历史文件。

每个组件启动前执行 `ulimit -c 0`；OpenBLAS/OMP 默认各限制为一个线程。总控脚本的
标准输出丢弃，不创建新的无限增长业务日志；只保存启动、退出等生命周期事件，事件日志
上限默认 1 MiB。ROS 2 自身的既有日志机制保持不变。

## 迁移到另一台机器人

将整个压缩包复制到目标机器人宿主机并解压，在宿主机执行：

```bash
cd mos_fleet_monitor
./install_on_robot.sh --check-only
./install_on_robot.sh
```

安装器先验证 ROS、导航工作空间和容器基础路径；若目标目录已存在则拒绝覆盖。它只部署
本包文件，不复制地图、厂商库或整套 `/tmp/mos_all_test_ws`。这些机器相关基础资产必须
由同版本 MOS 安装包提供，不能用本监控包覆盖。

部署后先执行 `mos-monitor-status`，确认没有同事手工进程，再执行启动。

## 当前不在本工作范围

- Foxglove 多机器人数据源切换；
- 团队共享/固定 Foxglove Layout；
- 自动清理 ROS 日志、core 或历史文件；
- 修改相机原生分辨率、帧率；
- 修改导航地图、定位或 GICP/ScanContext 参数。

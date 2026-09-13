# MOS Foxglove 交接说明

整理日期：2026-09-13。此包是本地可找回资产的完整交接快照，不是已经在实机验证过的统一发布版。

## 1. 当前状态和来源

本次 SSH 只读连接 `lumos-87`，解析到 `172.18.21.87`，返回 `No route to host`。没有进入宿主机或容器，也没有启动、停止或修改实机程序。当前部署版本、地图、驱动、容器挂载和运行状态均待恢复连接后核对。

已经保存：上位机 WSL 客户端、宿主机 Docker 入口、容器运行脚本、图像中继、状态采集、Vision/Nav 锁、遥控扩展、ROS 工作空间源码及权重，以及历史版本用于比对。代码原样复制，没有替换业务实现。

| 本地来源 | 交接包位置 | 如何使用 |
|---|---|---|
| `/home/shuochen/learning/mos_fleet_monitor` | `code/mos_fleet_monitor` | 当前本地工作副本，优先阅读，但不能等同当前实机 |
| `/home/shuochen/learning/mos_all_test_ws` | `code/mos_all_test_ws` | ROS 源码和权重快照；部署后必须编译 |
| `/home/shuochen/learning/mos_ops` | `reference/mos_ops` | Git 整理版的历史副本，不覆盖当前工作副本 |
| `lumos87_monitor_migration_20260812` | `reference/lumos87_monitor_migration_20260812` | 8月12日迁移快照，保留日期语义 |
| `_mos_vision_guard`、`mos_ops_guard` | `reference/` 对应目录 | 旧单实例保护、运维保护代码，不能认定已部署 |
| `mos_fleet_monitor/backups`、`dist` | `reference/mos_fleet_monitor/` | 历史压缩包与部署记录，供追溯 |

未带入 `.git`、Python 缓存、ROS build/install/log、core.PID 和旧运行态 PID 目录。保留源码内的 config、yaml、模型权重、测试和文档。没有复制 SSH 私钥、账号密码或整台 Docker 镜像。

## 2. 系统调用链和目录

```text
WSL client/start-robot-monitor.sh
  → SSH lumos@机器人地址
  → 宿主机 /home/lumos/mos_fleet_monitor/bin/mos-monitor-start
  → docker exec lumos_dev
  → 容器 /root/work_space/mos_fleet_monitor/bin/mos-monitor-start
  → _supervise_component.sh → _run_component.zsh
  → vision / navigation / image_relay / status_logger / bridge
  → SSH 本地端口转发 → Foxglove ws://localhost:8765
```

`/root/work_space/mos_fleet_monitor` 是容器路径，不是宿主机部署路径。`/tmp/mos_all_test_ws` 是容器中的 ROS 工作空间。只有源码不足以重建厂商环境，还依赖 `/opt/ros/humble`、`/opt/lumos`、`/opt/slam/apps`、相机/雷达设备与现有地图。

## 3. 必须交接的版本差异

1. **地址已经变化。** 当前客户端默认地址和 robots.conf 是 Tailscale `100.127.82.99`；旧内网地址是 `172.18.21.87`。本次仅确认旧内网地址不可达，没有测试 Tailscale。启动时建议显式传入现场确认的地址。
2. **版本标记不能作为唯一依据。** 当前 VERSION 是 `1.7.0`，但历史 dist 中有 `1.8.0`；旧 SHA256SUMS 也不应假定代表当前文件。以本包新生成的 SHA256SUMS 和代码内容为准。
3. **当前集成 Vision 已绕过 vision.launch.py。** `_run_component.zsh` 检查 `/dev/video0`、`/dev/video6`、`/dev/video12`，生成 `/run/mos_fleet_monitor/mos_sensor_runtime.yaml`，然后 `ros2 run mos_3d_object mos_3d_object_node`。生成配置包含三路相机，未列出 lidar。相机路径、分辨率和 30Hz 都写死在这个分支，迁移前必须核对设备映射与厂商 SDK。
4. **防重复启动并未形成统一入口锁。** 总控用 flock 串行化总控启停，并用进程匹配复用已有组件；optional Vision/Nav launch 使用目录锁。然而当前直接 ros2 run 的 Vision 不经过 launch 锁。手动 launch 与集成启动的并发互斥不能宣称已彻底解决。
5. **ROS 源码和可选锁不是同一版。** 当前 ROS 源码 vision.launch.py 使用 `/run/lock/mos_vision.lock` 的文件锁；optional_vision_guard 使用 `/run/lock/mos_vision.instance` 目录锁。重新编译可能覆盖 install 中的保护，需要人工确认后重新安装保护。
6. **组件退出策略有所变化。** 非 Vision 组件退出记录 EXIT，其他组件继续；Vision 延迟10秒重试，短时连续失败3次放弃，运行超过60秒会重置失败计数。普通退出也可能进入重试。用户想停 Vision 时应走受管停止入口，不能假定杀掉一个子进程就永久关闭。
7. **日志并非完全不写入。** status_log 本身只发布 ROS 消息；总控会写 events.log 和各组件 stdout/stderr。events.log 约1MiB后截短，组件 `.log` 本次启动清空后持续追加，没有同样的容量限制。ROS launch 也可能写 `/root/.ros/log`。`/run` 在容器中是否 tmpfs 必须实机确认。
8. **布局仍缺资产。** layouts 只有 README，没有导出的布局 JSON；不能自动还原截图布局。浏览器主题、账号设置也不能仅靠部署机器人代码保证一致。

## 4. 核心代码职责

以下路径相对 `code/mos_fleet_monitor/`。所有文件的逐项索引见 FILE_INDEX.txt。

| 文件/目录 | 职责与关键行为 |
|---|---|
| `client/start-robot-monitor.sh` | 复用 SSH master；远端按需启动后建立本地转发；Ctrl+C 只关本机连接 |
| `client/stop-robot-monitor.sh` | 调用宿主机停止入口，会停止受管 ROS 组件 |
| `client/start-fleet-monitor.sh`、`stop-fleet-monitor.sh` | 按 robots.conf 多机后台启动/停止；后台启动要求 SSH 公钥认证 |
| `client/robots.conf`、`fleet.env`、`list-foxglove-links.sh` | 名称/地址/本地端口、布局 ID 与网页连接链接 |
| `host/bin/` | 检查/启动 lumos_dev，通过 docker exec 调用容器入口 |
| `container/bin/_common.sh` | 组件列表、进程识别、PID归属、启停锁、事件日志 |
| `container/bin/mos-monitor-start` | 复用已有进程，补启动缺失组件；未就绪仍可能返回0，需看状态输出 |
| `container/bin/_supervise_component.sh` | 单组件进程生命周期和日志；Vision有单独重试策略 |
| `container/bin/_run_component.zsh` | ROS/SDK环境、相机运行配置、每个组件的实际命令、Bridge白名单 |
| `container/bin/mos-monitor-stop` | 受管进程组 INT→6秒→TERM→4秒→KILL，再复核；保留未登记外部进程 |
| `container/bin/mos-monitor-status` | 受管/外部进程与端口状态；进程存在不等于业务健康 |
| `container/assets/foxglove_image_relay.py` | 每路仅保存最新待处理图像；缩放/JPEG编码、限频、过期帧丢弃 |
| `container/assets/mos_status_logger.py` | 资源与ROS图信息、图像接收统计；发布摘要和结构化诊断 |
| `container/config/monitor.env` | 参数默认值；调用环境有值时优先使用已有值 |
| `optional_vision_guard/` | Vision launch包装器、原始实现备份、状态和停止工具 |
| `optional_nav_guard/` | Nav launch目录锁及source前置检查；备份原始launch和setup |
| `ops_extension/mos_ops_control/` | 新遥控ROS包；当前teleop分支调用它，涉及控制与进程暂停，非纯监视 |
| `container/assets/foxglove_teleop_guard.py` | 旧HAL遥控实现仍保留；不能与新ops_control混当一个入口 |
| `install_on_robot.sh` | 宿主机安装器；复制容器脚本，并将mos_ops_control复制到ROS源码后单独编译 |

## 5. 话题、显示与观测边界

| 数据 | 话题 | 说明 |
|---|---|---|
| 三路RGB输入 | `/{head,left_arm,right_arm}_camera/color/image_raw` | 原始 Image；实际发布由视觉节点决定 |
| 三路压缩输出 | `/foxglove/{head,left_arm,right_arm}_camera/image/compressed` | CompressedImage，给图像面板使用 |
| 深度输入 | `/{head,left_arm,right_arm}_camera/depth/image_raw` | 压缩中继目前仅处理RGB；Bridge当前白名单不包含原始深度话题 |
| 摘要 | `/mos/monitor/status_log` | String，每2秒更新，QoS深度1、可靠、Transient Local |
| 详细资源 | `/mos/monitor/diagnostics` | DiagnosticArray，包含组件资源等，详细展示优先用它 |
| 节点日志 | `/rosout` | ROS日志，与status_log不同 |
| 地图 | `/map`、`/cloud_map` | 栅格/点云；需上游实际发布、Bridge可见、面板启用 |
| 实时环境 | `/cloud_registered`、`/scan` | 点云/激光扫描 |
| 位姿与TF | `/localization`、`/Odometry`、`/odom`、`/tf`、`/tf_static` | 正确TF连接是叠加显示前提 |
| 导航轨迹/代价地图 | `/plan`等、`/global_costmap/*`、`/local_costmap/*` | 具体实际话题依目标Nav版本 |

Bridge主动过滤 `/livox/lidar`，避免历史同名不同类型的冲突；这不修复上游重复驱动。3D先以map为参考系，启用实际地图话题，再添加点云、TF和路径。只有网格通常要检查订阅、消息、QoS和TF，不应仅更改相机视角。

状态里的导航“运行”主要依据节点/话题注册，不证明定位成功。过去GICP匹配失败时状态仍可能显示定位运行；异常状态不能直接等同安全许可。图像FPS/帧龄是在采集节点侧计算，不是浏览器最终显示延迟。CPU组件值以单核100%计算，可超过100%；磁盘百分比是容量，不是I/O吞吐量；当前没有完整的网卡吞吐与磁盘读写速率监视。

默认图像：头部576×324，双臂432×243，JPEG质量58，上限15FPS，最大输入帧龄0.25秒。ROS侧丢旧帧不能消除WebSocket/TCP、SSH或浏览器端积压；网络不足时仍需减少订阅负载。

## 6. 现有设备的使用命令

以下在WSL执行；先按现场网络选对地址。它会补启动视觉和导航，因此要在可以启动机器业务组件时执行。

```bash
cd /home/shuochen/learning/mos_fleet_monitor
./client/start-robot-monitor.sh 172.18.21.87 8765
```

Foxglove连接 `ws://localhost:8765`。终端Ctrl+C只关闭隧道。

状态与停止：

```bash
ssh lumos@172.18.21.87 /home/lumos/mos_fleet_monitor/bin/mos-monitor-status
./client/stop-robot-monitor.sh 172.18.21.87
```

只连接已启动Bridge，不启动业务组件：

```bash
ssh -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=15 \
  -L 8765:127.0.0.1:8765 lumos@172.18.21.87
```

多机通过client/robots.conf每行配置名称、可达地址、唯一的本地端口，然后：

```bash
./client/start-fleet-monitor.sh
./client/list-foxglove-links.sh
```

不同机器人容器内可都用8765；WSL分别映射8765/8766等。多机启动输出“已提交”不是全部就绪，需查看每台状态。

## 7. 新设备部署

首先准备兼容厂商镜像、Humble与已编译的 `/tmp/mos_all_test_ws/install`。本包保存了工作空间源码供迁移，但不含厂商SDK、外部地图、驱动二进制或整个镜像；不能用一句全量colcon命令保证这些依赖一致。地图必须对应当前现场，不能沿用医院/办公室错误地图。

将交接包传到宿主机并解压；以下假设解压到 `/home/lumos/foxglove_handoff_20260913`：

```bash
# WSL：传包
scp /home/shuochen/learning/foxglove_handoff_20260913.tar.gz lumos@172.18.21.87:/home/lumos/

# 机器人宿主机：解压并进入安装目录
cd /home/lumos
tar -xzf foxglove_handoff_20260913.tar.gz
cd foxglove_handoff_20260913/code/mos_fleet_monitor
./install_on_robot.sh --check-only
```

检查相机路径和安装器依赖后，设备业务处于停止状态时安装：

```bash
./install_on_robot.sh --with-guards
```

安装器默认宿主机目标 `/home/lumos/mos_fleet_monitor`，容器目标 `/root/work_space/mos_fleet_monitor`。已有目标目录时拒绝覆盖，这是新安装命令，不是升级命令。`--check-only`也可能启动原本停止的容器，并执行ROS/Python环境检查；不能当作完全无副作用。安装器还会新增/编译mos_ops_control，并不是仅复制几个监控脚本。

锁安装生成的关键原始实现：

```text
/tmp/mos_all_test_ws/install/mos_3d_object/share/mos_3d_object/launch/vision_impl.launch.py
/tmp/mos_all_test_ws/install/lumos_nav/share/lumos_nav/launch/nav2_fastlio_bringup_impl.launch.py
/tmp/mos_all_test_ws/setup_nav2_fastlio_impl.sh
/root/work_space/mos_vision_status.sh
/root/work_space/mos_vision_stop.sh
/root/work_space/mos_nav_status.sh
/root/work_space/mos_nav_stop.sh
```

重新编译后，在无视觉/导航进程时可执行安装器 `--guards-only` 重新核对安装；若实现文件冲突应比较内容，不删除文件强行绕过。

## 8. 恢复连接后的只读核验

在宿主机运行，先核对真实部署，不进行启动或停止：

```bash
hostname
docker ps -a --format '{{.Names}} {{.Status}}'
docker inspect -f '{{.HostConfig.NetworkMode}} {{json .Mounts}}' lumos_dev
ls -l /home/lumos/mos_fleet_monitor/bin
docker exec lumos_dev sha256sum \
  /root/work_space/mos_fleet_monitor/bin/_run_component.zsh \
  /root/work_space/mos_fleet_monitor/bin/_supervise_component.sh \
  /root/work_space/mos_fleet_monitor/assets/foxglove_image_relay.py \
  /root/work_space/mos_fleet_monitor/assets/mos_status_logger.py
docker exec lumos_dev cat /root/work_space/mos_fleet_monitor/config/monitor.env
docker exec lumos_dev sh -c 'ls -l /dev/video*; tail -n 40 /run/mos_fleet_monitor/events.log'
```

比对本包对应文件SHA256后，再核查install中的launch/impl、setup和源代码是否一致。Bridge绑容器127.0.0.1时，宿主机隧道可达依赖容器网络模式；若不是host网络，需要另行明确连通方案。

## 9. 已知问题和下一位开发者优先项

- 先确认实机当前版本，尤其8月28日相机固定映射和直接ros2 run是否仍适用。
- 统一Vision单独/集成启动的锁；现有进程扫描存在并发窗口，不能当原子互斥。
- 对组件日志增加明确容量管理；检查容器传统core与宿主机Apport策略，ulimit不能单独证明全系统不生成dump。
- 统一版本号、打包校验和、部署文档；保留各历史快照，不拼成未经验证的新版本。
- 补导出的Foxglove布局JSON；状态详细展示使用diagnostics，避免String对比模式混排新旧内容。
- 增加消息新鲜度/定位质量判断后，才能将“已注册”升级为“功能健康”。

历史Wi-Fi事故的证据是：机器人连2.4GHz时到网关严重丢包，省电关闭仍未恢复；当时CPU尚有空闲、无明显I/O等待。单次rx bitrate=1Mbps及累计RX dropped不能证明实际可用带宽或独立根因。切换5GHz是应验证的措施，不能把过去“根因已确认”的措辞当作完成闭环。

## 10. 交接包核验

```bash
cd foxglove_handoff_20260913
sha256sum -c SHA256SUMS > /tmp/foxglove_handoff_check.txt 2>&1
tail -n 20 /tmp/foxglove_handoff_check.txt
```

FILE_INDEX.txt列出所有保留文件；SOURCE_MAP.tsv列出每个代码文件的原始本地来源；SHA256SUMS是本次重新生成的交接校验，不使用历史包里的旧校验文件来断言完整性。

> 历史来源文档：保留原文供追溯，其中旧部署状态、遥控租约、相机与日志说明可能与当前代码不同。当前使用入口见[仓库 README](../../README.md)；不要将本页直接当成现行部署手册。

# MOS Foxglove 运维包迁移与多机器人使用说明

## 1. 目标与边界

本包在每台 MOS 上提供一致的 ROS 2 话题、统一启停、三路低延迟图像、结构化状态、
Foxglove Bridge，以及 Vision/导航单实例保护。它不复制地图、ScanContext 数据、厂商
SLAM 二进制、相机标定或机器人业务参数。

WSL 端为每台机器人分配不同本地端口：

| 机器人 | IP | WSL 端口 | Foxglove 数据源 |
|---|---:|---:|---|
| `mos-87` | `172.18.21.87` | `8765` | `ws://localhost:8765` |
| `mos-210` | `172.18.21.210` | `8766` | `ws://localhost:8766` |

密码不进入配置文件、脚本、布局 JSON 或压缩包。

## 2. 本包涉及的文件

### 2.1 WSL 端

| 路径 | 功能 |
|---|---|
| `client/start-robot-monitor.sh` | 启动一台机器人并建立 SSH 隧道 |
| `client/stop-robot-monitor.sh` | 停止一台机器人的受管监控栈 |
| `client/robots.conf` | 机器人名称、IP、本地端口清单 |
| `client/start-fleet-monitor.sh` | 后台启动清单中的全部机器人 |
| `client/stop-fleet-monitor.sh` | 关闭全部隧道，可选停止远端监控栈 |
| `client/list-foxglove-links.sh` | 生成每台机器的 Foxglove 深链接 |
| `client/fleet.env` | 统一 Foxglove `layoutId` |

### 2.2 机器人宿主机

部署到：

```text
/home/lumos/mos_fleet_monitor/bin/
```

包含 `mos-monitor-start`、`mos-monitor-stop`、`mos-monitor-status` 三个容器入口。

### 2.3 `lumos_dev` 容器

部署到：

```text
/root/work_space/mos_fleet_monitor/
```

主要资产：

| 路径 | 功能 |
|---|---|
| `assets/foxglove_image_relay.py` | 最新帧、过期帧丢弃、三路按需 JPEG 图像 |
| `assets/mos_status_logger.py` | `/mos/monitor/status_log` 与 `/mos/monitor/diagnostics` |
| `bin/_run_component.zsh` | 五个组件的实际启动入口 |
| `bin/_common.sh` | PID、进程归属和冲突检查 |
| `bin/mos-monitor-start` | 依次启动 Vision、导航、图像、状态、Bridge |
| `bin/mos-monitor-stop` | 精确停止受管进程组并复核 |
| `bin/mos-monitor-status` | 查看五组件与 8765 端口状态 |
| `config/monitor.env` | ROS Domain、图像、状态刷新和 Bridge 参数 |

### 2.4 单实例保护

Vision 安装器会：

```text
vision.launch.py       -> 单实例包装器
vision_impl.launch.py  -> 原始实现备份
/run/lock/mos_vision.instance
```

导航安装器会同时保护两个入口：

```text
nav2_fastlio_bringup.launch.py       -> launch 单实例包装器
nav2_fastlio_bringup_impl.launch.py  -> 原始 launch 备份
setup_nav2_fastlio.sh                -> setup 前置保护
setup_nav2_fastlio_impl.sh           -> 原始 setup 备份
/run/lock/mos_navigation.instance
```

必须保护 `setup_nav2_fastlio.sh`，因为厂商原脚本会先清理导航进程。仅在 launch 层加锁，
第二位操作者仍可能在到达 launch 锁之前杀掉正在运行的导航子进程。

### 2.5 不进入本次迁移包的旧文件

下列旧试验文件不是当前统一启动链路的依赖，不应复制到新机器：

```text
/root/work_space/foxglove_nav_viz.py
/root/work_space/fastlio_mos_pointcloud2.yaml
```

当前 3D 地图、定位和点云由导航的官方 ROS 2 话题经 Bridge 直接发布，不再依赖
`foxglove_nav_viz.py`。`mos_3d_object/vision_node.py` 的历史备份也不在本包内；新机器应使用其
自身与 MOS 版本匹配的原始 ROS 包。

## 3. 生成迁移包

在 WSL 执行：

```bash
cd /home/shuochen/learning/mos_fleet_monitor
chmod +x build-package.sh client/*.sh
./build-package.sh
```

输出：

```text
dist/mos_fleet_monitor-<版本>.tar.gz
```

脚本同时打印 SHA256。压缩包排除 `dist/`、`__pycache__` 和 `*.pyc`。

## 4. 将包部署到新机器人 `172.18.21.210`

### 4.1 前置条件

新机器人必须已有：

- `lumos_dev` 容器；
- `/opt/ros/humble/setup.zsh`；
- `/tmp/mos_all_test_ws/install/setup.zsh`；
- 正确版本的 `mos_3d_object`、`lumos_nav`、`foxglove_bridge`；
- 与该机器人和现场匹配的地图及定位数据。

部署前必须停止 Vision 和导航。

### 4.2 复制并解压

在 WSL 执行：

```bash
scp \
  /home/shuochen/learning/mos_fleet_monitor/dist/mos_fleet_monitor-1.2.1.tar.gz \
  lumos@172.18.21.210:/home/lumos/

ssh lumos@172.18.21.210
```

在机器人宿主机执行：

```bash
mkdir -p /home/lumos/mos_monitor_packages/v1.2.1
tar -xzf /home/lumos/mos_fleet_monitor-1.2.1.tar.gz \
  -C /home/lumos/mos_monitor_packages/v1.2.1
cd /home/lumos/mos_monitor_packages/v1.2.1/mos_fleet_monitor
```

### 4.3 只读检查

```bash
./install_on_robot.sh --check-only
```

检查通过后再部署，并同时安装 Vision/Nav 锁：

```bash
./install_on_robot.sh --with-guards
```

安装器不启动 ROS 节点，不复制地图，不修改 GICP/ScanContext 参数。安装锁之前会检查
Vision 和导航进程；发现仍在运行就拒绝修改。

如果首次部署时没有带 `--with-guards`，可补装：

```bash
./install_on_robot.sh --guards-only
```

## 5. 配置多机 SSH 公钥

多机后台启动不能交互式保存密码。首次在 WSL 执行：

```bash
test -f ~/.ssh/id_ed25519.pub || ssh-keygen -t ed25519
ssh-copy-id lumos@172.18.21.87
ssh-copy-id lumos@172.18.21.210
```

每台设备各输入一次密码。之后确认：

```bash
ssh -o BatchMode=yes lumos@172.18.21.87 true
ssh -o BatchMode=yes lumos@172.18.21.210 true
```

## 6. 多机器人启动和停止

配置文件已经包含：

```text
mos-87       172.18.21.87     8765
mos-210      172.18.21.210    8766
```

一键启动：

```bash
cd /home/shuochen/learning/mos_fleet_monitor
./client/start-fleet-monitor.sh
```

后台日志位于：

```text
${XDG_RUNTIME_DIR:-/tmp}/mos-fleet-monitor-$UID/
```

只关闭 WSL 隧道，机器人继续运行：

```bash
./client/stop-fleet-monitor.sh
```

同时停止所有机器人上的受管监控栈：

```bash
./client/stop-fleet-monitor.sh --remote
```

## 7. 保存和复用 Foxglove 布局

1. 在布局菜单中将当前布局重命名为 `MOS运维标准布局`。
2. 选择“保存更改”。
3. 在布局菜单中选择“导出”，保存为 `layouts/MOS运维标准布局.json`。
4. 其他账号或浏览器通过“从文件导入”载入该 JSON。
5. 同一 Foxglove 账号可直接使用已保存的个人布局；团队账号可以共享为组织布局。

布局只保存面板位置、面板设置和话题名。因为每台 MOS 都由本包发布完全相同的话题，
同一布局切换数据源后无需重新配置面板。

当前统一话题至少包括：

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

## 8. 使用同一布局切换机器人

先打开已保存的布局，然后从浏览器 URL 中取得 `layoutId`，写入：

```text
client/fleet.env
```

例如：

```bash
FOXGLOVE_LAYOUT_ID="xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx"
```

生成入口：

```bash
./client/list-foxglove-links.sh
```

两个链接使用同一个 Foxglove 网站和同一个布局，只是数据源分别指向本地 `8765`、`8766`。

网页版一个标签页同一时刻只连接一台实时机器人。可以在不同浏览器标签页打开两个链接，
或者在同一标签页通过对应链接切换。Foxglove Desktop 支持在同一窗口建立多个标签页，
每个标签页连接一台机器人。

如果未来要求在一个自研页面内提供“机器人下拉框”并无刷新切换，需要额外开发基于
`@foxglove/embed` 的外层网页；这不属于当前开源 Web 页面本身的配置能力，并涉及相应
Foxglove 许可。

## 9. 注意事项

- 不要把机器人密码写进 `robots.conf`。
- 两台机器必须使用不同 WSL 本地端口。
- 每台机器内部 Bridge 仍可统一使用 8765。
- 重新 `colcon build mos_3d_object` 或 `lumos_nav` 可能覆盖 install 空间的 launch 锁；
  构建后重新执行 `./install_on_robot.sh --guards-only`。
- 不要把一台机器的地图、ScanContext 数据或定位 YAML 直接覆盖到另一台机器。
- Foxglove 布局相同不代表地图相同；地图内容仍由各机器人自己的 `/map` 等话题提供。

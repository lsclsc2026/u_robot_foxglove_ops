> 历史来源文档：保留原文供追溯，其中旧部署状态、遥控租约、相机与日志说明可能与当前代码不同。当前使用入口见[仓库 README](../../README.md)；不要将本页直接当成现行部署手册。

# 迁移资产清单

## 本包携带

| 资产 | 目标位置 | 用途 |
|---|---|---|
| `container/bin/*` | `/root/work_space/mos_fleet_monitor/bin/` | 监控及可选遥控组件的统一启停、状态和进程组管理 |
| `container/config/monitor.env` | `/root/work_space/mos_fleet_monitor/config/` | ROS Domain、端口、RViz 开关 |
| `foxglove_image_relay.py` | `/root/work_space/mos_fleet_monitor/assets/` | 三路 RGB 最新帧、过期帧丢弃、按需 JPEG 编码 |
| `mos_status_logger.py` | `/root/work_space/mos_fleet_monitor/assets/` | 每 2 秒发布简要 `status_log` 和分组诊断状态，不写文件 |
| `foxglove_teleop_guard.py` | `/root/work_space/mos_fleet_monitor/assets/` | Foxglove 底盘遥控的限速、短时授权、失联停车和控制器冲突保护 |
| `host/bin/*` | `/home/lumos/mos_fleet_monitor/bin/` | 宿主机到 `lumos_dev` 容器入口 |
| `client/*` | WSL 任意固定目录 | SSH 远端启动与本地端口转发 |
| `optional_vision_guard/*` | 按说明可选安装 | 防止原始 Vision 命令重复启动 |
| `optional_nav_guard/*` | 按说明可选安装 | 防止导航 launch 重复启，并在厂商 setup 清理前拦截 |
| `layouts/*` | WSL 包内 | 存放从 Foxglove 导出的标准布局 JSON |
| `build-package.sh` | WSL 包根目录 | 生成排除缓存和旧产物的版本化 tar.gz |
| `DEPLOYMENT_AND_MULTI_ROBOT.md` | WSL 包根目录 | 迁移、启停、布局和多机使用说明 |
| `docs/2026-08-24/01-Foxglove安全远程操纵.md` | WSL 包内 | 遥控部署、Foxglove 面板配置和安全限制 |

## 目标机必须已有，不随包复制

| 依赖 | 预期路径/包 |
|---|---|
| Docker 容器 | `lumos_dev`（由 `lumos run` 创建） |
| ROS 2 Humble | `/opt/ros/humble/setup.zsh` |
| MOS 工作空间 | `/tmp/mos_all_test_ws/install/setup.zsh` |
| 导航环境切换 | `setup_nav2_fastlio.sh`、`unsetup_nav2.sh` |
| ROS 包 | `mos_3d_object`、`lumos_nav`、`foxglove_bridge` |
| Python 模块 | `rclpy`、`cv2`、`cv_bridge` |
| 机器专属地图 | `/opt/slam/map_datas/maps/`，必须由现场确认版本 |
| 厂商 SLAM 安装 | `/opt/slam/apps/install/` |

地图、ScanContext 数据、FAST-LIO/GICP 参数和厂商二进制与具体机器人/现场绑定，本包不
迁移这些内容，防止再次出现“程序正常但加载错地图”的问题。

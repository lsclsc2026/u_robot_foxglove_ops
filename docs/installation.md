# 安装与依赖

## 适用环境

客户端面向带 Bash、OpenSSH、`ss` 的 WSL/Linux。机器人宿主机需要可使用 Docker 的账号，已有兼容的 `lumos_dev` 容器；容器脚本使用 Bash、zsh、ROS 2 Humble、Python 3 和厂商 Lumos 环境。

安装器要求以下容器路径已经可用：

```text
/opt/ros/humble/setup.zsh
/tmp/mos_all_test_ws/install/setup.zsh
/tmp/mos_all_test_ws/setup_nav2_fastlio.sh
/tmp/mos_all_test_ws/unsetup_nav2.sh
```

实际运行还依赖 `/opt/lumos`、`/opt/slam/apps`、ROS 包 `mos_3d_object`、`lumos_nav`、`foxglove_bridge`，以及 `cv2`、`cv_bridge`、`diagnostic_msgs`、`geometry_msgs`、`mos_hal`、`rclpy`、`std_msgs` 等模块。设备、地图、SDK、驱动和依赖版本需要与目标机器人匹配。仓库保留 ROS 工作空间源码和权重，但不包含编译后的工作空间或完整厂商环境；因此没有可在任意 Ubuntu 上一键全量复现的安装命令。

## 获取源码与准备

将仓库克隆或复制到客户端和机器人宿主机。示例假设机器人宿主机目录是 `/home/lumos/u_robot_foxglove_ops`：

```bash
git clone git@github.com:lsclsc2026/u_robot_foxglove_ops.git
cd u_robot_foxglove_ops/code/mos_fleet_monitor
```

私有仓库使用自己的 GitHub 身份认证。机器人 SSH 公钥、私钥和账号密码由部署环境配置，不放入仓库。

新设备先按照厂商环境准备 `/tmp/mos_all_test_ws` 的依赖和编译结果，再进行以下安装。这里的步骤是部署指导，本次发布整理没有执行这些命令。

## 宿主机安装

确认业务允许容器启动，且安装保护时视觉和导航均已正常停止：

```bash
cd /home/lumos/u_robot_foxglove_ops/code/mos_fleet_monitor
./install_on_robot.sh --check-only
./install_on_robot.sh --with-guards
```

`--check-only` 会检查路径与 ROS/Python 依赖，也可能启动原本停止的容器，不能视为完全只读。正常安装会：

1. 将宿主机入口安装到当前用户的 `~/mos_fleet_monitor/bin`，默认用户 `lumos` 时为 `/home/lumos/mos_fleet_monitor/bin`。
2. 将容器脚本、配置和 assets 复制到 `/root/work_space/mos_fleet_monitor`。
3. 复制 `mos_ops_control` 到 `/tmp/mos_all_test_ws/src/` 并执行 `colcon build --packages-select mos_ops_control --symlink-install`。
4. 使用 `--with-guards` 时备份并包装 Vision/Nav launch 与导航 setup。

安装器拒绝覆盖已存在的宿主机或容器目标目录，属于新安装入口。已有部署应逐文件比较后安排更新，不能删目录强行绕过检查。安装器不复制地图，不全量安装 `code/mos_all_test_ws`，也不提供事务式回滚；失败时可能已有部分文件落地，应根据输出确认位置。

可通过 `MOS_CONTAINER_NAME` 和 `MOS_MONITOR_HOST_TARGET` 调整安装器容器名与宿主机目标；客户端远端命令路径和宿主机运行环境必须同步配置。容器目标与 ROS 路径多处固定，不能只改一个变量就认为完成移植。

## 可选保护与后续编译

不带 `--with-guards` 时正常部署不安装 launch 保护。已部署后可在相关组件停止时执行：

```bash
./install_on_robot.sh --guards-only
```

保护涉及 install 空间中的 `vision_impl.launch.py`、`nav2_fastlio_bringup_impl.launch.py`，工作空间的 `setup_nav2_fastlio_impl.sh`，以及 `/root/work_space/mos_vision_{status,stop}.sh`、`mos_nav_{status,stop}.sh`。重新编译可能覆盖 install 中的包装器；应重新核对原实现与保护，不删除冲突文件绕过保护。

## 网络

Bridge 默认只监听容器回环 `127.0.0.1:8765`，客户端 SSH 转发指向宿主机回环。二者直接连通通常依赖容器使用 host 网络；采用其他网络模式时必须明确容器到宿主机的端口连通方案。不要仅将 Bridge 地址改成公网监听来掩盖网络配置问题。

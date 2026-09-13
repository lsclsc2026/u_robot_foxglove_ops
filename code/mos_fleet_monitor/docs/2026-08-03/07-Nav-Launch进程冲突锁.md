> 历史来源文档：保留原文供追溯，其中旧部署状态、遥控租约、相机与日志说明可能与当前代码不同。当前使用入口见[仓库 README](../../../../README.md)；不要将本页直接当成现行部署手册。

# Nav Launch 进程冲突锁

## 核心说明

导航重复启动时，厂商 `setup_nav2_fastlio.sh` 会在 launch 锁执行前先清理导航子进程。因此当前
使用两层保护：

```text
nav2_fastlio_bringup.launch.py  launch原子目录锁
setup_nav2_fastlio.sh           清理动作前置拦截
```

原始文件备份为：

```text
nav2_fastlio_bringup_impl.launch.py
setup_nav2_fastlio_impl.sh
```

## 全部操作代码

新机器安装：

```bash
./install_on_robot.sh --check-only
./install_on_robot.sh --with-guards
```

已部署机器补装：

```bash
./install_on_robot.sh --guards-only
```

查看状态：

```bash
docker exec lumos_dev /root/work_space/mos_nav_status.sh
```

单独停止导航：

```bash
docker exec lumos_dev /root/work_space/mos_nav_stop.sh
```

总控启停：

```bash
/home/lumos/mos_fleet_monitor/bin/mos-monitor-start
/home/lumos/mos_fleet_monitor/bin/mos-monitor-stop
```

重新编译 `lumos_nav` 后再次安装锁：

```bash
colcon build --packages-select lumos_nav
./install_on_robot.sh --guards-only
```

## 任务描述

防止手工启动和总控启动重复进入导航，并防止第二个 setup 脚本清理正在运行的导航子进程。

## 进展说明

launch 锁、setup 前置锁、状态/停止脚本和部署入口已完成；每次重新编译 `lumos_nav` 后需重新执行 `--guards-only`。


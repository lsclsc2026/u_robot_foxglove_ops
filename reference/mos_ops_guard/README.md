> 历史来源文档：保留原文供追溯，其中旧部署状态、遥控租约、相机与日志说明可能与当前代码不同。当前使用入口见[仓库 README](../../README.md)；不要将本页直接当成现行部署手册。

# MOS 独立运维库

本目录按《下位机独立运维库开发清单 V2.0》实现，目标部署路径为：

```text
/root/work_space/mos_ops_guard
```

它不修改导航、机械臂、Vision、协调器或 `/tmp/mos_all_test_ws` 中的业务代码。
Watchdog 和一键维护默认只读；只有人工调用 `mos-protect` / `mos-recover`
时，才可能执行 `config/protection.yaml` 中显式启用的白名单命令。

## 当前安全边界

- 厂家的硬件急停、急停状态读取、导航取消、机械臂停止和任务入口锁定接口尚未确认，
  因而默认全部禁用。
- 禁用动作会明确记录在 `missing_actions`，不会把“写入了 P0/P1/P2 状态”
  表述成“硬件急停已经成功”。
- 当前已知的 Vision 停止脚本已加入白名单；启动动作仍保持禁用，防止恢复时产生
  重复 Vision 实例。
- `auto_protect.enabled` 固定默认为 `false`。Watchdog 不执行控制命令。
- 恢复只回到 `NORMAL` 待命，不恢复故障前任务。

在真机启用任何控制动作前，先用厂家接口文档确认命令、返回码、超时和幂等性，
然后编辑 `config/protection.yaml`。动作必须用参数数组，例如：

```yaml
cancel_navigation:
  enabled: true
  argv: ["ros2", "action", "cancel", "..."]
  description: "已在指定 MOS 版本验证的官方取消接口"
```

实现不使用 shell，也不使用 `killall python3`、`pkill -f ros2` 等宽泛命令。

## 部署

将整个目录复制到目标位置，并保留脚本执行权限：

```bash
cp -a /home/shuochen/learning/mos_ops_guard /root/work_space/
chmod 0755 /root/work_space/mos_ops_guard/bin/*
```

本项目只依赖 MOS 已有的 Python 3、PyYAML 和 ROS 2 Humble `rclpy`。

所有 ROS 进程使用与现有系统一致的环境：

```bash
source /opt/ros/humble/setup.bash
source /tmp/mos_all_test_ws/install/setup.bash
export ROS_DOMAIN_ID=111
export ROS_LOCALHOST_ONLY=1
```

先启动状态仲裁器：

```bash
/root/work_space/mos_ops_guard/bin/mos-status-mux \
  --ros-args --disable-external-lib-logs
```

原有状态脚本只调整启动参数，不修改代码：

```bash
python3 /root/work_space/mos_status_logger.py \
  --ros-args \
  --disable-external-lib-logs \
  -r /mos/monitor/status_log:=/mos/monitor/status_normal
```

再启动只读 Watchdog：

```bash
/root/work_space/mos_ops_guard/bin/mos-watchdog \
  --ros-args --disable-external-lib-logs
```

`systemd/` 中提供了独立服务样例。安装或启用服务属于目标机部署动作，本开发目录
不会自动改动系统服务。

## 人工保护与恢复

P2 取消任务：

```bash
bin/mos-protect p2 --reason "导航目标错误" --operator "zhangsan"
```

P1 必须指定一个或多个隔离范围：

```bash
bin/mos-protect p1 \
  --scope vision \
  --reason "Vision 重复启动" \
  --operator "zhangsan"
```

可选范围为 `navigation`、`base`、`arm`、`vision`、`scheduler`。

P0 软件保护请求：

```bash
bin/mos-protect p0 --reason "失控运动" --operator "zhangsan"
```

在 `hardware_estop` 动作仍禁用时，该命令不会宣称已触发硬件急停。紧急现场仍应
优先使用物理急停/断电。

查看状态及恢复：

```bash
bin/mos-ops-status
bin/mos-recover --operator "zhangsan" --onsite-confirmed
```

重复提交完全相同或被当前范围包含的保护请求是幂等的：不重复执行、不增加事件计数。
从高等级降到低等级必须先恢复，不能直接覆盖。

`--dry-run` 会解析配置并列出拟执行动作，但不执行命令、不写状态、不发告警。

## 告警和状态仲裁

输入：

- `/mos/monitor/status_normal`：原 60 秒正常摘要；
- `/mos/ops/alarm_input`：Watchdog 或保护模块发出的 JSON 告警事件。

输出：

- `/mos/monitor/status_log`：Foxglove 最终状态；
- `/mos/monitor/alarm`：告警/恢复消息。

仲裁器按 `source + alarm_id` 分别锁存多个告警。异常存在时不转发普通摘要；
所有活动告警解除后先发一条恢复消息，再恢复正常摘要。

Watchdog 当前检测：

- 必需话题是否注册，以及是否实际持续收到消息；
- 关键节点、根分区使用率、新 core/crash 文件；
- 默认网关及可选远程地址；
- `/cmd_vel` 持续非零但 `/Odometry` 位移不足；
- 当前 P0/P1/P2 保护状态。

话题类型、必需性、阈值和启动宽限期均在 `config/watchdog.yaml` 中配置。
现场 `/localization_confidence` 为 `std_msgs/msg/Float32`，当前只检查其新鲜度；
数值含义和厂家阈值未确认前，不根据数值高低触发保护或告警。

## 一键维护

```bash
bin/mos-maintain
```

命令输出中文摘要并覆盖：

```text
runtime/latest_maintenance.json
runtime/latest_maintenance.txt
runtime/maintenance_baseline.json
```

检查包括主机资源、温度、网络、SSH/Foxglove 端口、受限目录新增文件、ROS 2
节点/话题/频率、重复 Vision、当前保护和告警。多个话题频率并行采样，默认目标是
60 秒内退出。

默认即使发现问题也返回 0，便于人工查看完整报告；自动化需要按等级返回码时使用：

```bash
bin/mos-maintain --strict
```

返回码为 0（正常）、1（警告）、2（异常）。

## 运行时文件

- `latest_protection.json`：最新保护状态；
- `protection_events.jsonl`：保护/恢复事件，单文件 5 MiB、最多 3 个备份；
- `latest_alarm.json`：仲裁器当前锁存告警；
- `watchdog_state.json`：Watchdog 最新检测状态；
- `latest_maintenance.json` / `.txt`：最新一次维护报告；
- `maintenance_baseline.json`：固定文件元数据基线。

除保护事件的有限轮转日志外，其余状态全部原子覆盖，不持续创建历史文件。

## 开发机验证

```bash
cd /home/shuochen/learning/mos_ops_guard
python3 -m unittest discover -s tests -v
python3 -m compileall -q mos_ops_guard
```

# 排障

先确认目标 IP、SSH 用户、宿主机与容器路径。下面检查命令不会主动启动 ROS 节点，但 `mos-monitor-status` 可能清理失效 PID 记录；安装器 `--check-only` 会按需启动容器，不能用作严格只读探测。

| 现象 | 检查方向 |
|---|---|
| `No route to host` / SSH 超时 | 当前网络路由、机器人在线状态和实际地址。旧内网/Tailscale 地址不是新部署保证 |
| 本地 8765 已占用 | 查看已有隧道或使用本地 8766；连接地址相应改成 `ws://localhost:8766` |
| SSH 正常但 WebSocket 不通 | Bridge 是否监听、远端端口、容器网络模式、宿主机回环到容器回环的关系 |
| 启动显示完成但缺数据 | 单个组件可能未就绪；查看每个组件日志和 ROS 消息，不能只看脚本退出码 |
| `固定相机设备不存在` | 对照 `/dev/video0`、`6`、`12` 与实际设备映射，核对 SDK 和容器设备挂载 |
| 图像卡顿 | 实际输入消息、帧龄、订阅负载、Wi-Fi/SSH 带宽；减少面板订阅或预览分辨率/FPS |
| 3D 只有网格 | 实际地图/点云消息、白名单、面板启用项、参考系、TF 与 QoS |
| 看不到深度或 `/livox/lidar` | 当前 Bridge 白名单有意不包含，不表示机器人上不存在消息 |
| 遥控包找不到 | 安装器是否编译成功；runner 显式补入 ops 包 AMENT/PYTHONPATH；旧隔离 install 可能遗漏新包 |
| 重编译后锁失效 | 比对 install 中包装器与 impl，相关进程停止后重新核对可选保护 |
| 停止后进程仍在 | 确认是外部进程还是受管进程；停止策略保留未登记进程，不要用宽泛 pkill 替代归属检查 |
| 磁盘或日志增长 | 组件 stdout/stderr 与 ROS 日志不受事件日志上限约束，检查具体目录 |

宿主机检查示例：

```bash
docker ps -a --format '{{.Names}} {{.Status}}'
docker inspect -f '{{.HostConfig.NetworkMode}} {{json .Mounts}}' lumos_dev
docker exec lumos_dev sh -c 'ls -l /dev/video*; tail -n 40 /run/mos_fleet_monitor/events.log'
docker exec lumos_dev cat /root/work_space/mos_fleet_monitor/config/monitor.env
```

容器内在正确 ROS 环境下可用 `ros2 topic list -t`、`ros2 topic info <topic>` 确认类型和端点；是否继续采样消息按现场资源条件决定。本次整理未运行这些检查。

## 理解监控数值

- 导航或定位“运行”通常是节点/话题已注册，不证明 GICP 匹配成功、定位质量足够或导航可执行。
- 图像 FPS/帧龄来自采集节点，不是浏览器最终端到端延迟；ROS 丢旧帧无法消除 WebSocket、TCP、SSH 和浏览器队列。
- 组件 CPU 以单核 100% 表示，可能超过 100%；磁盘百分比表示容量占用，不表示 I/O 吞吐。
- 当前不提供完整网卡吞吐或磁盘读写速率；摘要和 diagnostics 只保留最新 ROS 消息，没有状态历史数据库。

## 日志与历史网络问题

`events.log` 约 1 MiB 后截短；各组件 `.log` 在本次启动清空后持续追加，没有相同容量管理。ROS 还可能写 `/root/.ros/log`，`/run` 是否为 tmpfs 取决于容器。`ulimit -c 0` 仅作用于这些受管进程，不证明系统其他机制不会生成转储。

历史材料记录过 2.4 GHz Wi-Fi 到网关严重丢包。单次 bitrate 或累计 RX dropped 不能独自证明根因；应结合当前链路证据排查，不能将旧记录视为当前机器诊断结论。

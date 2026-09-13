# lumos_nav 联动视觉说明

当前导航工作流可以和 `ros2_ws` 中的 `mos_vision` 无侵入联动。

## 方案

`lumos_nav` 新增了一个被动监视节点：

```text
goal_arrival_monitor
```

它只做三件事：

1. 订阅 `/goal_pose`
2. 订阅 `/localization`
3. 到达目标后发布 `/mosnav/arrival`

它不控制导航器，不修改 Nav2 行为树，不改 `purepursuit_follower` 二进制。

## 到点判定

默认参数：

```yaml
goal_topic: "/goal_pose"
odom_topic: "/localization"
arrival_topic: "/mosnav/arrival"
xy_tolerance: 0.25
yaw_tolerance: 0.25
hold_time_sec: 0.5
max_linear_speed: 0.05
max_angular_speed: 0.10
```

含义：

- 机器人距离目标点小于 `0.25 m`
- 朝向误差小于 `0.25 rad`
- 线速度和角速度都足够小
- 持续满足 `0.5 s`

才会认为真正“到点”，然后只发布一次到点事件。

## 发布内容

发布话题：

```text
/mosnav/arrival
```

消息类型：

```text
std_msgs/msg/String
```

消息内容是 JSON，例如：

```json
{
  "goal_id": "1710000000.123:1.200:0.800:1.570",
  "status": "reached",
  "goal_frame": "map",
  "odom_frame": "map",
  "goal_pose": {"x": 1.2, "y": 0.8, "yaw": 1.57},
  "current_pose": {"x": 1.18, "y": 0.79, "yaw": 1.56},
  "distance_xy": 0.022,
  "yaw_error": 0.010,
  "timestamp_ns": 1710000000123456789
}
```

这正好能被 `ros2_ws` 里的 `nav_arrival_vision_bridge` 直接订阅。

## 启动方式

如果你们平时使用：

```bash
ros2 launch lumos_nav relocalization_and_navigation.launch.py
```

现在会自动带上 `goal_arrival_monitor`。

如果使用：

```bash
ros2 launch lumos_nav nav2_fastlio_bringup.launch.py
```

现在也会自动带上 `goal_arrival_monitor`。

单独调试可运行：

```bash
ros2 launch lumos_nav goal_arrival_monitor.launch.py
```

## 与视觉串联

另一侧启动：

```bash
ros2 launch mos_vision vision_with_grasp.launch.py
ros2 launch mos_vision nav_arrival_vision_bridge.launch.py
```

完整链路就是：

```text
导航发 /goal_pose
  -> 机器人到点
  -> lumos_nav 发布 /mosnav/arrival
  -> mos_vision 桥接节点收到到点事件
  -> 自动 reset grasp selector
  -> 自动 enable vision
  -> 自动 enable grasp selector
  -> /vision/grasp_tasks 输出稳定任务
```

## 不影响现有导航的原因

- 不修改 `navigation2` 源码
- 不修改 `purepursuit_follower` 二进制
- 不拦截 `/cmd_vel`
- 不更改导航目标接口
- 只是额外增加只读订阅和一个事件发布

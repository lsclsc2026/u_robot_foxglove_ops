# mos_grasp_selector

`mos_grasp_selector` 是 3D 抓取任务选择包。它不读取相机、不运行 YOLO，也不计算深度；它订阅 `mos_3d_object` 发布的 `DetectedObject3DArray`，从有效的 3D 目标中选择稳定抓取点，并发布 `GraspTaskArray`。

本包负责：

- 根据任务规则筛选目标类别；
- 检查目标是否有有效深度和目标坐标；
- 在同一类别内选择距离最近或置信度最高的抓取点；
- 按坐标 Y 方向分配左右抓取角色；
- 使用多帧历史判断抓取点是否稳定；
- 将稳定的点封装成下游控制节点可以直接消费的抓取任务。

本包不负责相机采集、YOLO 推理、3D 坐标计算、治疗车满载判断、导航、底盘运动或机械臂控制。

## 1. 工作流程

```text
mos_3d_object
  /vision/detections_3d
          │
          ▼
  mos_grasp_selector
  ├── 检查目标类别、深度和坐标系
  ├── 按类别组成抓取点候选
  ├── 距离/置信度排序
  ├── 分配 left/right 角色
  ├── 多帧中值滤波和稳定性检查
  └── 发布 /vision/grasp_tasks
```

默认任务为 `auto_treatment_cart_push`。它把类别 `6`、`7`、`8` 作为同等候选，但每个类别必须单独形成一对有效把手点，不会把不同类别的把手混成一对。

## 2. 输入和输出话题

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 输入 | `/vision/detections_3d` | `mos_3d_object/msg/DetectedObject3DArray` | 3D 目标检测结果 |
| 输出 | `/vision/grasp_tasks` | `mos_grasp_selector/msg/GraspTaskArray` | 抓取任务结果 |
| 输出 | `/vision/grasp_tasks_json` | `std_msgs/msg/String` | 选择状态和任务的 JSON 调试信息 |

输入检测必须满足以下条件才会进入候选：

- `class_id` 在当前任务的 `target_class_ids` 中；
- `score` 不低于任务配置的最小置信度；
- `depth_m` 是有限正数；
- `has_target_position == true`；
- `position_target` 坐标是有限值，且坐标系与任务的 `target_frame` 一致。

### 2.1 `GraspTaskArray` 和 `GraspTask`

消息定义位于 `msg/GraspTaskArray.msg` 和 `msg/GraspTask.msg`。默认每次最多发布一个 `GraspTask`：

| 字段 | 含义 |
| --- | --- |
| `task_name` | 任务名，例如 `auto_treatment_cart_push` |
| `grasp_mode` | 抓取模式，默认 `dual_arm_sync` |
| `target_frame` | 抓取点坐标系，默认 `body_link` |
| `ready` | 是否已经达到发布条件 |
| `quality` | 当前质量，例如 `ok`、`collecting`、`unstable` |
| `reason` | 当前选择/稳定性原因 |
| `target_class_id` | 实际选中的目标类别 |
| `target_class_name` | 实际选中的目标类别名称 |
| `required_points` | 需要的抓取点数 |
| `points[]` | `GraspPoint` 数组 |

每个 `GraspPoint` 包含：

| 字段 | 含义 |
| --- | --- |
| `role` | 抓取角色，默认 `left_handle` 或 `right_handle` |
| `rank` | 候选排序名次 |
| `class_id` / `class_name` | 实际选中的类别 |
| `source_object_id` | 对应 3D 检测目标 ID |
| `source_score` | 原始检测置信度 |
| `point` | `geometry_msgs/msg/PointStamped` 抓取坐标 |
| `distance_m` | 该点到坐标原点的距离 |

如果当前还没有稳定任务，默认 `publish_empty_tasks=true`，`/vision/grasp_tasks` 会发布 `tasks=[]`；更详细的未就绪原因可查看 `/vision/grasp_tasks_json`。

## 3. 默认任务和选择规则

默认配置文件 `config/grasp_selector_params.yaml` 使用：

```yaml
active_task: auto_treatment_cart_push
target_class_ids: [6, 7, 8]
target_frame: body_link
enabled: false
```

默认规则位于 `config/grasp_selector_rules.yaml`：

| 规则 | 默认值 | 说明 |
| --- | --- | --- |
| `target_class_ids` | `[6, 7, 8]` | A/B/C 型治疗车把手候选 |
| `required_points` | `2` | 需要两个把手点 |
| `grasp_mode` | `dual_arm_sync` | 双臂同步抓取模式标识 |
| `selection.mode` | `nearest` | 默认选择距离最近的一对 |
| `role.mode` | `split_by_y` | 按 Y 坐标分配左右角色 |
| `stability.window` | `5` | 历史窗口大小 |
| `stability.min_frames` | `3` | 至少需要的稳定帧数 |
| `stability.filter` | `median` | 位置中值滤波 |
| `stability.max_std_m` | `0.03` | 最大位置标准差 |
| `stability.max_range_m` | `0.06` | 最大位置范围 |

当同一帧同时出现多个类别的有效把手对时：

- `selection.mode=nearest`：选择两点距离总和最小的一对；
- `selection.mode=highest_score`：选择两点置信度总和最高的一对。

稳定性历史只会在同一目标类别内累积。如果候选类别发生切换，旧历史会被清空，避免把不同类别的点混合平均。

## 4. 安装和启动

### 4.1 运行前提

启动本包前需要：

- 已构建 `mos_3d_object` 和 `mos_grasp_selector`；
- `mos_3d_object` 正在发布 `/vision/detections_3d`；
- 两个包使用的消息接口已经通过 `colcon build` 生成；
- 下游控制节点理解 `GraspTaskArray` 中的坐标系和抓取角色。

构建示例：

```bash
cd /home/jmt/mos_real_final
colcon build --base-paths 3dvision1 \
  --packages-select mos_3d_object mos_grasp_selector
source install/setup.bash
```

### 4.2 使用 launch 启动

```bash
ros2 launch mos_grasp_selector grasp_selector.launch.py
```

启动文件加载 `config/grasp_selector_params.yaml`。该文件默认 `enabled=false`，所以启动后需要调用使能服务：

```bash
ros2 service call /mos_grasp_selector/set_enabled std_srvs/srv/SetBool \
  "{data: true}"
```

关闭选择器：

```bash
ros2 service call /mos_grasp_selector/set_enabled std_srvs/srv/SetBool \
  "{data: false}"
```

节点服务：

```text
/mos_grasp_selector/set_enabled  std_srvs/srv/SetBool
/mos_grasp_selector/reset        std_srvs/srv/Trigger
```

重置多帧历史、清除已发布锁存任务：

```bash
ros2 service call /mos_grasp_selector/reset std_srvs/srv/Trigger "{}"
```

## 5. 如何订阅

命令行检查：

```bash
ros2 topic info /vision/detections_3d
ros2 topic info /vision/grasp_tasks
ros2 topic echo /vision/grasp_tasks
ros2 topic echo /vision/grasp_tasks_json
```

Python 订阅抓取任务：

```python
import rclpy
from rclpy.node import Node
from mos_grasp_selector.msg import GraspTaskArray


class GraspConsumer(Node):
    def __init__(self):
        super().__init__("grasp_consumer")
        self.create_subscription(
            GraspTaskArray,
            "/vision/grasp_tasks",
            self.on_tasks,
            10,
        )

    def on_tasks(self, msg):
        for task in msg.tasks:
            if not task.ready:
                continue
            self.get_logger().info(
                f"task={task.task_name} "
                f"target_class={task.target_class_id} "
                f"points={len(task.points)}"
            )
            for point in task.points:
                xyz = point.point.point
                self.get_logger().info(
                    f"role={point.role} "
                    f"frame={point.point.header.frame_id} "
                    f"xyz=({xyz.x:.3f}, {xyz.y:.3f}, {xyz.z:.3f})"
                )


rclpy.init()
rclpy.spin(GraspConsumer())
rclpy.shutdown()
```

下游控制节点使用 `PointStamped.header.frame_id` 确认坐标系，不要假设所有部署的坐标系名称都相同。当前默认是 `body_link`，但可通过 `target_frame` 参数调整。

## 6. 配置和扩展任务

```text
config/grasp_selector_params.yaml   # ROS 参数和当前任务
config/grasp_selector_rules.yaml    # 任务规则
config/class_map.yaml               # 类别 ID 到名称的映射
launch/grasp_selector.launch.py
```

推荐通过 `grasp_selector_rules.yaml` 增加任务。例如，一个单点垃圾车顶部横杆任务可以设置：

```yaml
tasks:
  trash_cart_top_bar_push:
    enabled: true
    task_name: "trash_cart_top_bar_push"
    target_class_id: 18
    target_class_name: "垃圾车顶部横杆"
    required_points: 1
    grasp_mode: "single_arm_or_base_push"
    trigger:
      mode: "none"
    selection:
      mode: "nearest"
    role:
      mode: "single"
      name: "top_bar"
    position:
      mode: "detected"
    stability:
      enabled: true
      window: 5
      min_frames: 3
```

然后在 `grasp_selector_params.yaml` 中选择：

```yaml
active_task: "trash_cart_top_bar_push"
```

修改规则后需要重启节点；当前节点没有动态重新加载 YAML 的服务。

## 7. 常见问题

### `/vision/grasp_tasks` 一直是空数组

先确认 `/vision/detections_3d` 有数据，并检查目标是否满足 `has_target_position=true`、深度有效、坐标系一致、类别在当前任务候选列表中。然后确认稳定窗口是否已经收集到至少 3 个有效帧。

### 任务已发布但下游没有收到

检查下游订阅的消息类型是否为 `mos_grasp_selector/msg/GraspTaskArray`，以及话题是否仍为 `/vision/grasp_tasks`。可以使用 `ros2 topic info /vision/grasp_tasks` 查看发布者和订阅者。

### 选择到了错误的左右点

默认规则按目标坐标的 Y 值分配 `negative_y=right_handle`、`positive_y=left_handle`。如果机器人坐标系的 Y 方向定义不同，应调整 `grasp_selector_rules.yaml` 的角色规则或统一上游坐标系。

### 目标类别切换后任务迟迟不稳定

这是保护逻辑：类别切换会清除历史，必须重新收集 `min_frames` 个同类别候选点，避免把不同类别的抓取点混合稳定化。


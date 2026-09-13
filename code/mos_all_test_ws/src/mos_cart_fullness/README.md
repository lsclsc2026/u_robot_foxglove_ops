# mos_cart_fullness

`mos_cart_fullness` 是治疗车满载判断包。它不打开相机，也不创建 `MosSensorAdapter`，而是订阅 `mos_3d_object` 发布的头部、左腕和右腕 RGB 图像，使用独立的 YOLO 模型进行 2D 检测，再根据治疗车装载区域和物品数量输出 `full`/`not_full` 判断。

本包负责：

- 管理三路相机的最新 RGB 帧；
- 检测治疗车、绿色包裹和腔镜盒；
- 根据装载区域占用率和关联物品数量判断当前帧是否满载；
- 通过多帧窗口过滤单帧误检；
- 支持腕部单视角、腕部+头部双视角巡检和抓取点二次确认；
- 发布给导航或上层任务流程使用的 JSON 结果。

本包不负责相机采集、3D 坐标计算、导航、底盘运动、机械臂控制或抓取动作执行。

## 1. 工作流程

```text
mos_3d_object
   ├── /left_arm_camera/color/image_raw
   ├── /right_arm_camera/color/image_raw
   └── /head_camera/color/image_raw
                │
                ▼
       mos_cart_fullness
       ├── 保存最新且未过期的图像
       ├── YOLO 2D 检测治疗车和装载物品
       ├── 计算每辆车的装载 ROI/占用率/物品数
       ├── 多帧稳定判断
       └── 发布 JSON 结果
```

每次推理只使用当前帧的治疗车框和物品框，不跨帧锁定某辆车，也不使用 IOU 做目标跟踪。如果当前帧没有有效治疗车，视图结果会变为 `unknown`/`no_cart`，并重置该视图的稳定窗口。

## 2. 输入和输出话题

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 输入 | `/left_arm_camera/color/image_raw` | `sensor_msgs/msg/Image` | 左腕图像 |
| 输入 | `/right_arm_camera/color/image_raw` | `sensor_msgs/msg/Image` | 右腕图像 |
| 输入 | `/head_camera/color/image_raw` | `sensor_msgs/msg/Image` | 头部图像 |
| 输入 | `/cart_fullness/set_waypoint` | `std_msgs/msg/String` | 开始检查、停止或重置 |
| 输出 | `/cart_fullness/route_signal` | `std_msgs/msg/String` | 检查完成后发布一次结果 JSON |
| 输出 | `/cart_fullness/status` | `std_msgs/msg/String` | 检查过程和最终状态 JSON |

图像订阅使用传感器数据 QoS。发布图像的节点和本节点的 QoS 必须兼容，否则会出现节点已启动但收不到图像的情况。

### 2.1 满载判断规则

严格的治疗车视图会执行以下处理：

1. 在当前图像中检测治疗车类别 `0..5` 和物品类别 `9,10`。
2. 根据当前治疗车框计算装载 ROI，并计算物品框与关联区域的重叠。
3. 当装载区域占用率达到 `full_threshold`，或关联物品数量达到配置的数量条件时，当前帧记为 `full`。
4. 最近 `decision_window=5` 帧中至少有 `min_full_frames=3` 个 `full`，才输出稳定的 `full`；至少有 3 个 `not_full` 才输出稳定的 `not_full`。帧不要求连续。

`empty_cart` 和 `no_cart` 使用一次性采样规则：每次收到检查命令后仅连续检测 10 帧，检测期间不发布中间状态。10 帧中至少 5 帧为 `empty_cart` 时，在第 10 帧发布一次 `decision=empty_cart`；至少 6 帧为 `no_cart` 时，在第 10 帧发布一次 `decision=no_cart`。两个条件同时满足时 `empty_cart` 优先。第 10 帧后本次检测停止，直到收到新的检查命令、`reset` 或 `off`。

当前 `config/cart_fullness.yaml` 的默认值是：

| 视图 | 占用率条件 | 数量条件 |
| --- | --- | --- |
| 腕部巡检 `scan_wrist` | `occupancy_ratio >= 0.60` | `used_item_count > 1` |
| 头部视图 | `occupancy_ratio >= 0.60` | `used_item_count > 5` |
| `verify_dual` 腕部 | 不要求完整治疗车和 ROI | 当前腕部帧至少 1 个有效类别 `9/10` 物品 |

关联区域会向治疗车框上方扩展，因此位于上层货架、中心点不在传统 `LOAD_ROI` 内的物品仍可能被关联；占用率计算仍使用装载 ROI。具体阈值以 `config/cart_fullness.yaml` 为准。

## 3. 检查模式

| 模式 | 使用相机 | 决策方式 |
| --- | --- | --- |
| `scan_wrist` | 指定左腕或右腕 | 只使用指定腕部视图的稳定结果 |
| `scan_dual` | 指定腕部 + 头部 | 两个视图独立评估；任一视图产生有效结果即可决策，`not_full` 优先 |
| `verify_dual` | 指定腕部 + 头部 | 用于抓取点二次确认；任一视图为 `not_full` 或超时则输出 `not_full` |

双视角模式不要求两个相机的时间戳完全匹配。每个相机使用自己的最新有效帧；any available fresh view 都可以提供证据，一个相机没有新帧时，不会阻塞另一个相机给出决定。

每次检查最多发布一个 `/cart_fullness/route_signal`。默认 `disable_after_decision=true`，做出决定后节点会停止处理当前检查，直到收到下一次检查命令或重新使能。

## 4. 启动

### 4.1 运行前提

启动本包前需要：

- 已构建并运行 `mos_3d_object`，且三路 RGB 话题有数据；
- ROS 2、`cv_bridge`、OpenCV 和 Python 推理环境；
- Ultralytics YOLO 依赖；
- 与满载类别约定匹配的权重文件。

构建示例：

```bash
cd /home/jmt/mos_real_final
colcon build --base-paths 3dvision1 --packages-select mos_cart_fullness
source install/setup.bash
```

### 4.2 使用 launch 启动

```bash
ros2 launch mos_cart_fullness cart_fullness.launch.py
```

当前启动文件默认加载 `weights/best3.pt`，并将 `enabled` 设置为 `true`。启动节点后，还需要通过 `/cart_fullness/set_waypoint` 发起一次具体检查。

打开 OpenCV 预览：

```bash
ros2 launch mos_cart_fullness cart_fullness.launch.py preview:=true
```

预览会显示检测框、治疗车装载区域、每个视图的判断和双视角融合结果。预览只读图像和发布结果，不会调用底盘、机械臂或导航接口。

服务接口：

```text
/mos_cart_fullness/set_enabled  std_srvs/srv/SetBool
/mos_cart_fullness/reset        std_srvs/srv/Trigger
```

## 5. 发起一次检查

### 5.1 兼容旧命令

以下字符串会启动腕部单视角检查：

```bash
ros2 topic pub --once /cart_fullness/set_waypoint std_msgs/msg/String \
  '{data: "left_start"}'
```

```text
left_start  → 左腕相机
right_start → 右腕相机
```

映射由参数 `waypoint_camera_map` 和 `waypoint_cart_map` 控制。

### 5.2 推荐 JSON 命令

```bash
ros2 topic pub --once /cart_fullness/set_waypoint std_msgs/msg/String \
  '{data: "{\"check_id\":\"cart_a_scan_001\",\"mode\":\"scan_dual\",\"waypoint\":\"cart_a_scan\",\"zone_id\":\"cart_a\",\"cart_id\":\"cart_a\",\"wrist_camera\":\"left\",\"timeout_sec\":5.0}"}'
```

JSON 字段：

| 字段 | 是否必需 | 说明 |
| --- | --- | --- |
| `check_id` | 是 | 本次检查的唯一 ID；已完成的重复 ID 会被忽略 |
| `mode` | 是 | `scan_wrist`、`scan_dual` 或 `verify_dual` |
| `wrist_camera` | 是 | `left` 或 `right` |
| `waypoint` | 否 | 路点名称；缺省时使用 `data` 或空值 |
| `zone_id` | 否 | 检测区域 ID，缺省使用 `waypoint` |
| `cart_id` | 否 | 治疗车 ID，缺省使用 `waypoint` |
| `timeout_sec` | 否 | 超时时间；`verify_dual` 默认使用 5 秒 |

停止当前检查或重置状态：

```bash
ros2 topic pub --once /cart_fullness/set_waypoint std_msgs/msg/String \
  '{data: "off"}'

ros2 topic pub --once /cart_fullness/set_waypoint std_msgs/msg/String \
  '{data: "reset"}'
```

`verify_dual` 超时后发布 `skip_next_waypoint`；`scan_dual` 超时不会自动发布路线信号，应该由上层重新设置检查上下文或发送 `reset`/`off`。

## 6. 输出结果和订阅

查看过程状态和最终结果：

```bash
ros2 topic echo /cart_fullness/status
ros2 topic echo /cart_fullness/route_signal
```

`/cart_fullness/route_signal` 的 `std_msgs/msg/String.data` 是 JSON，常见字段如下：

```json
{
  "waypoint": "cart_a_scan",
  "camera": "left",
  "cart_id": "cart_a",
  "signal": "continue_to_next_waypoint_for_grasp",
  "decision": "full",
  "route_step_delta": 1,
  "reason": "occupancy_ratio>=0.6",
  "occupancy_ratio": 0.72,
  "used_item_count": 2,
  "full_frames": 3,
  "not_full_frames": 0,
  "check_id": "cart_a_scan_001",
  "mode": "scan_dual",
  "wrist_camera": "left",
  "wrist_decision": "full",
  "head_decision": "unknown"
}
```

其中：

```text
decision=full      → signal=continue_to_next_waypoint_for_grasp
decision=not_full  → signal=skip_next_waypoint
```

Python 订阅示例：

```python
import json
from std_msgs.msg import String


def on_route_signal(msg: String):
    result = json.loads(msg.data)
    if result.get("decision") == "full":
        # 继续当前治疗车的抓取流程
        pass
    elif result.get("decision") == "not_full":
        # 跳过当前目标
        pass


# 在 ROS 2 Node 初始化中：
# self.create_subscription(
#     String, "/cart_fullness/route_signal", on_route_signal, 10
# )
```

状态消息会持续报告 `state=checking`，并包含 `decision`、`reason`、`full_frames`、`not_full_frames`、`wrist_decision`、`head_decision` 等调试字段。

## 7. 配置文件

```text
config/cart_fullness.yaml
launch/cart_fullness.launch.py
weights/best3.pt
```

常用参数：

| 参数 | 当前值 | 作用 |
| --- | --- | --- |
| `camera_poll_seconds` | `0.10` | 推理轮询周期 |
| `max_frame_age_sec` | `2.0` | 允许使用的最大图像年龄 |
| `decision_window` | `5` | 稳定判断窗口 |
| `min_full_frames` / `min_not_full_frames` | `3` / `3` | 窗口内所需稳定帧数 |
| `cart_class_ids` | `0,1,2,3,4,5` | 治疗车类别 |
| `item_class_ids` | `9,10` | 装载物品类别 |
| `full_threshold` | `0.60` | 装载区域占用率阈值 |
| `min_item_association_overlap` | `0.10` | 物品与车关联区域的最小重叠比例 |
| `route_signal_topic` | `/cart_fullness/route_signal` | 路线结果话题 |
| `status_topic` | `/cart_fullness/status` | 状态话题 |

修改输入图像话题时，必须保证 `mos_3d_object` 的 RGB 输出和本包的三个输入参数一致。

## 8. 常见问题

### 节点启动但一直没有结果

本节点启动后不会自行开始检查。先确认三个 RGB 话题有数据，再向 `/cart_fullness/set_waypoint` 发布命令。

### `status` 一直是 `checking`

检查是否有足够的新鲜图像、是否检测到有效治疗车/物品，以及稳定窗口是否达到 `min_full_frames` 或 `min_not_full_frames`。也可以打开 `preview:=true` 观察检测框。

### 只收到腕部图像，头部视图为 `no_head_frame`

确认 `mos_3d_object` 正在发布 `/head_camera/color/image_raw`，并检查两边的 QoS 是否兼容。

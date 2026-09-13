# mos_3d_object

`mos_3d_object` 是 MOS 视觉链路的相机和 3D 目标检测包。它是三个视觉包中唯一直接创建 `MosSensorAdapter`、读取 MOS 相机数据的包。

本包负责：

- 读取头部相机 RGB-D 数据，使用 YOLO 检测目标；
- 根据深度计算目标在相机坐标系下的 3D 位置；
- 通过 TF 或静态外参将位置转换到目标坐标系；
- 发布头部、左腕、右腕三路 RGB 图像，供其他 ROS 2 节点使用。

本包不负责治疗车满载判断、抓取点筛选、导航、底盘控制或机械臂控制。

## 1. 工作流程

```text
MosSensorAdapter
      │
      ├── 头部 RGB-D
      │      ├── RGB 图像 → YOLO 目标检测
      │      ├── 深度图 → 目标深度/相机坐标
      │      └── TF/静态外参 → 目标坐标系
      │                         │
      │                         ▼
      │                /vision/detections_3d
      │
      ├── 头部 RGB → /head_camera/color/image_raw
      ├── 左腕 RGB → /left_arm_camera/color/image_raw
      └── 右腕 RGB → /right_arm_camera/color/image_raw
```

只有头部相机的 RGB-D 数据进入 3D 检测流程。腕部相机只发布 RGB 图像，不在本包中进行深度定位；例如 `mos_cart_fullness` 会订阅这些腕部图像进行 2D 满载判断。

每个检测周期会依次检查 RGB-D 是否存在、RGB 和深度尺寸是否对齐、时间差是否超过阈值，然后执行 YOLO、深度转换和坐标变换。若目标坐标变换失败，仍可能发布检测框，但该目标的 `has_target_position` 会为 `false`，下游抓取选择器不会使用它。

## 2. 输入和输出话题

本节点直接从 MOS SDK 读取相机数据，因此没有 ROS 图像输入话题。

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/vision/detections_3d` | `mos_3d_object/msg/DetectedObject3DArray` | 头部 RGB-D YOLO 3D 检测结果 |
| `/head_camera/color/image_raw` | `sensor_msgs/msg/Image` | 头部 RGB 图像 |
| `/left_arm_camera/color/image_raw` | `sensor_msgs/msg/Image` | 左腕 RGB 图像 |
| `/right_arm_camera/color/image_raw` | `sensor_msgs/msg/Image` | 右腕 RGB 图像 |

RGB 话题使用传感器数据 QoS。下游图像订阅者建议使用 `qos_profile_sensor_data` 或等价的兼容 QoS。

### 2.1 `DetectedObject3DArray`

消息定义位于 `msg/DetectedObject3DArray.msg`：

| 字段 | 含义 |
| --- | --- |
| `header` | 当前结果时间戳和默认目标坐标系 |
| `camera_name` | 检测相机名称，默认 `head` |
| `camera_frame` | 相机坐标系 |
| `target_frame` | 输出目标坐标系，默认 `body_link` |
| `detections[]` | 当前帧的 `DetectedObject3D` 数组 |

每个 `DetectedObject3D` 的主要字段如下：

| 字段 | 含义 |
| --- | --- |
| `class_id` / `class_name` | YOLO 类别 ID 和名称 |
| `score` | 检测置信度 |
| `object_id` | 检测目标 ID；未提供时为 `-1` |
| `bbox_xyxy` | RGB 图像中的检测框 `[x1, y1, x2, y2]` |
| `position_camera` | 相机坐标系下的 3D 点 |
| `position_target` | 目标坐标系下的 3D 点 |
| `has_target_position` | `position_target` 是否有效 |
| `target_frame` | 该目标点使用的目标坐标系 |
| `depth_m` | 目标深度，单位米 |

抓取类下游节点应优先检查：

```python
if detection.has_target_position:
    point = detection.position_target.point
```

常用类别 ID 由模型和下游配置共同约定，当前抓取/满载流程常用：`0..5` 为 A/B/C 型空车和载物车，`6..8` 为 A/B/C 型车把手，`9` 为绿色包裹，`10` 为腔镜盒，`18` 为垃圾车顶部横杆。

## 3. 安装和启动

### 3.1 运行前提

启动本包前需要：

- ROS 2 环境和已经构建的 `mos_3d_object` 包；
- MOS/Lumos SDK Python 环境，使 `mos_sdk` 可导入；
- 可用的 MOS 相机和传感器配置文件；
- Ultralytics YOLO、OpenCV、NumPy 等推理依赖；
- 与模型类别匹配的 `.pt` 权重。

构建示例：

```bash
cd /home/jmt/mos_real_final
colcon build --base-paths 3dvision1 --packages-select mos_3d_object
source install/setup.bash
```

### 3.2 使用 launch 启动

启动文件为 `launch/vision.launch.py`。它会加载 `config/yolo_params.yaml`，并将权重和静态外参解析为安装后的包内路径；当前默认权重为 `weights/best_1.pt`。

```bash
ros2 launch mos_3d_object vision.launch.py
```

配置文件中的 `enabled` 默认是 `false`。启动节点后需要显式开启检测：

```bash
ros2 service call /mos_3d_object/set_enabled std_srvs/srv/SetBool \
  "{data: true}"
```

关闭 3D 推理，但保留节点和 RGB 图像发布：

```bash
ros2 service call /mos_3d_object/set_enabled std_srvs/srv/SetBool \
  "{data: false}"
```

服务类型为 `std_srvs/srv/SetBool`，服务名称中的 `/mos_3d_object` 来自节点名。

## 4. 配置

主要配置文件：

```text
config/yolo_params.yaml
config/static_extrinsic.yaml
launch/vision.launch.py
```

常用参数如下：

| 参数 | 当前配置 | 作用 |
| --- | --- | --- |
| `camera` | `head` | 用于 RGB-D 检测的相机 |
| `weights` | launch 中为 `weights/best_1.pt` | YOLO 权重路径 |
| `conf` / `iou` | `0.15` / `0.45` | 检测置信度和 NMS 阈值 |
| `device` | `cuda` | 推理设备 |
| `min_depth_m` / `max_depth_m` | `0.15` / `12.0` | 有效深度范围，单位米 |
| `target_frame` | `body_link` | 输出 3D 点的目标坐标系 |
| `use_tf` | `false` | 是否使用 TF 查询外参 |
| `static_extrinsic_file` | `config/static_extrinsic.yaml` | 静态相机外参 |
| `detections_3d_topic` | `/vision/detections_3d` | 3D 检测输出话题 |
| `rgb_publish_seconds` | `0.05` | RGB 发布周期 |

当 `use_tf=false` 时，需要确认 `static_extrinsic.yaml` 与当前相机安装姿态一致；当 `use_tf=true` 时，需要保证 TF 树中存在相机坐标系到 `target_frame` 的变换。

修改输出话题或 RGB 话题后，所有下游包的配置也需要同步修改。

## 5. 如何订阅

查看话题类型和实时消息：

```bash
ros2 topic info /vision/detections_3d
ros2 topic echo /vision/detections_3d

ros2 topic echo /head_camera/color/image_raw
```

Python 订阅 3D 检测：

```python
import rclpy
from rclpy.node import Node
from mos_3d_object.msg import DetectedObject3DArray


class DetectionConsumer(Node):
    def __init__(self):
        super().__init__("detection_consumer")
        self.create_subscription(
            DetectedObject3DArray,
            "/vision/detections_3d",
            self.on_detections,
            10,
        )

    def on_detections(self, msg):
        for detection in msg.detections:
            if not detection.has_target_position:
                continue
            point = detection.position_target.point
            self.get_logger().info(
                f"class={detection.class_id} "
                f"xyz=({point.x:.3f}, {point.y:.3f}, {point.z:.3f})"
            )


rclpy.init()
rclpy.spin(DetectionConsumer())
rclpy.shutdown()
```

图像订阅使用 `sensor_msgs/msg/Image`，通常配合 `cv_bridge` 转换为 OpenCV 图像。该包不会发布检测可视化图；如需可视化，可以订阅 RGB 图像并在下游自行绘制。

## 6. 常见问题

### 启动时报 `mos_sdk is unavailable`

当前终端没有进入 MOS/Lumos SDK 环境，或者 SDK 没有安装在当前 Python 环境中。需要在正确的 SDK 环境中 source 工作空间后再启动。

### 有 RGB 图像但没有 `/vision/detections_3d`

检查节点是否已调用 `/mos_3d_object/set_enabled`，RGB-D 是否同时返回有效帧，以及 RGB 和深度分辨率、时间差是否满足配置要求。

### 检测有框但抓取选择器不产生抓取点

检查 `has_target_position`、`position_target.header.frame_id` 和 `target_frame`。没有有效目标坐标的检测只能用于 2D 调试，不能用于 3D 抓取。


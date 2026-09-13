# mos_grasp_pipeline 配置说明

`mos_grasp_pipeline` 负责把导航后的视觉抓取流程串起来。它不做 YOLO 推理，也不实现机械臂底层控制，只负责流程编排和参数化调用。

## 1. 启动

手动测试时先启动新视觉链路：

```bash
ros2 launch mos_3d_object vision.launch.py
ros2 launch mos_grasp_selector grasp_selector.launch.py
ros2 launch mos_arm_controller arm_controller.launch.py
```

注意：`mos_arm_controller arm_controller.launch.py` 现在启动的是统一硬件控制节点。这个节点只创建一次 `MosController()`，同时提供：

```text
/mos_arm_controller/run_task
/mos_chassis_controller/run_chassis
```

真机 pipeline 测试时不要再单独启动 `mos_chassis_controller chassis_controller.launch.py`，否则会再次创建第二个 `MosController()` 连接。

再启动 pipeline：

```bash
ros2 launch mos_grasp_pipeline grasp_pipeline.launch.py
```

启动后默认不执行，需要调用：

```bash
ros2 service call /mos_grasp_pipeline/start std_srvs/srv/Trigger "{}"
```

## 2. 流程

```text
start
  ↓
等待 arm/chassis 控制服务
  ↓
打开 mos_3d_object 和 mos_grasp_selector
  ↓
等待第一阶段 /vision/grasp_tasks
  ↓
关闭视觉和选择器
  ↓
根据第一阶段左右把手 x 均值计算底盘前进距离
  ↓
调用 /mos_chassis_controller/run_chassis 前进并等待稳定
  ↓
再次打开视觉和选择器
  ↓
等待第二阶段 /vision/grasp_tasks
  ↓
根据第二阶段左右抓取点下发左右臂目标位姿
  ↓
调用 /mos_arm_controller/run_task 执行 grasp
  ↓
可选调用底盘服务回退
```

## 3. 配置文件

```text
config/grasp_pipeline_params.yaml
```

## 4. 基础参数

```yaml
auto_start: false

grasp_tasks_topic: "/vision/grasp_tasks"
vision_set_enabled_service: "/mos_3d_object/set_enabled"
grasp_selector_set_enabled_service: "/mos_grasp_selector/set_enabled"
grasp_selector_reset_service: "/mos_grasp_selector/reset"
arm_run_task_service: "/mos_arm_controller/run_task"
chassis_run_chassis_service: "/mos_chassis_controller/run_chassis"

target_frame: "body_link"
first_grasp_timeout_sec: 30.0
second_grasp_timeout_sec: 30.0
service_timeout_sec: 30.0
post_chassis_settle_sec: 0.8
return_after_grasp: true
task_name: "grasp"
```

说明：

- `auto_start`：节点启动后是否自动执行流程。建议保持 `false`，由导航脚本或人工触发。
- `grasp_tasks_topic`：pipeline 等待的抓取任务话题。
- `vision_set_enabled_service`：打开/关闭视觉节点的服务。
- `grasp_selector_set_enabled_service`：打开/关闭抓取选择器的服务。
- `grasp_selector_reset_service`：重置抓取选择器历史窗口的服务。
- `arm_run_task_service`：机械臂任务服务，当前调用 `grasp`。
- `chassis_run_chassis_service`：底盘靠近/回退服务。
- `target_frame`：接受的抓取点坐标系。当前为 `body_link`。
- `first_grasp_timeout_sec`：第一阶段等待稳定抓取任务的超时时间。
- `second_grasp_timeout_sec`：底盘靠近后二次等待稳定抓取任务的超时时间。
- `service_timeout_sec`：等待和调用 ROS 服务的超时时间。
- `post_chassis_settle_sec`：底盘停止后等待相机和车体稳定的时间。
- `return_after_grasp`：抓取后是否调用底盘服务回退。
- `task_name`：下发给机械臂服务的任务名，当前为 `grasp`。

## 5. 控制服务边界

pipeline 不再直接 import `mos_sdk`，也不直接控制机械臂或底盘。

它只做流程编排：

- 底盘靠近：把第一阶段左右把手 x 均值作为 `MoveChassis.target_x_m` 发给 `/mos_chassis_controller/run_chassis`。
- 机械臂抓取：把第二阶段左右把手点封装为 `RunTask.left_point/right_point`，发给 `/mos_arm_controller/run_task`。
- 抓取偏移、夹爪开合、机械臂等待、底盘靠近和回退距离记录都由统一硬件控制节点维护。
- `mos_arm_controller` 包负责启动统一硬件控制节点，并保留原来的两个服务名给 pipeline 调用。

## 9. pipeline 接收的抓取任务格式

pipeline 当前只处理：

```text
grasp_mode = "dual_arm_sync"
target_frame = "body_link"
points 中同时包含：
  role = "left_handle"
  role = "right_handle"
```

因此，`mos_grasp_selector` 的规则需要保证输出这两个 role。

## 10. 后续扩展建议

### 检测物体增多，但仍是双臂抓两个点

优先只改 `mos_grasp_selector/config/grasp_selector_rules.yaml`：

- 修改 `target_class_id`。
- 修改 `required_points`。
- 修改 `selection.mode`。
- 修改 `role` 分配方式。
- 修改稳定滤波参数。

pipeline 不需要改，只要最后仍输出：

```text
grasp_mode = dual_arm_sync
left_handle
right_handle
```

### 新增单臂抓取

如果后续任务只需要一个抓取点，可以在 `mos_grasp_selector` 中新增规则：

```yaml
grasp_mode: "single_arm"
required_points: 1
role:
  mode: "single"
  name: "pick_point"
```

但当前 pipeline 的 `_extract_ready_grasp_pair()` 和 `_execute_grasp()` 只处理左右双臂。因此需要在 pipeline 中新增：

- 单点任务解析。
- 选择左臂或右臂。
- 单臂目标位姿生成。
- 单臂夹爪动作。

### 新增多个候选点再决策

推荐保持分层：

```text
mos_3d_object 发布所有目标 /vision/detections_3d
mos_grasp_selector 根据规则输出任务 /vision/grasp_tasks
mos_grasp_pipeline 只执行 ready 任务
```

不要让 pipeline 直接处理原始 YOLO 检测结果。这样以后换目标、换规则时，优先改 `grasp_selector_rules.yaml` 或选择器节点，pipeline 尽量保持稳定。

### 新增复杂决策

例如：

- 最近两个目标。
- 最左/最右目标。
- 离图像中心最近目标。
- 先检测车体，再检测把手。
- 不同垃圾车使用不同抓取部件。

推荐扩展位置：

```text
mos_grasp_selector/mos_grasp_selector/grasp_selector_node.py
```

优先扩展：

```text
_rank_targets()
_assign_roles()
_apply_active_task()
```

并在 `grasp_selector_rules.yaml` 中用配置打开新策略。

## 11. 常用检查命令

查看 pipeline 状态：

```bash
ros2 topic echo /mos_grasp_pipeline/status
```

查看抓取任务：

```bash
ros2 topic echo /vision/grasp_tasks
ros2 topic echo /vision/grasp_tasks_json
```

手动启动 pipeline：

```bash
ros2 service call /mos_grasp_pipeline/start std_srvs/srv/Trigger "{}"
```

查看日志：

```bash
tail -f /tmp/mos_grasp_pipeline.log
tail -f /tmp/mos_3d_object.log
```

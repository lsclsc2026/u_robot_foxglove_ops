# mos_grasp_selector 抓取选择节点

`mos_grasp_selector` 只负责从视觉检测结果中选择抓取任务，不负责机械臂 SDK 调用。

## 输入

```text
/vision/detections_3d
```

消息类型：

```text
mos_3d_object/msg/DetectedObject3DArray
```

## 输出

正式抓取任务：

```text
/vision/grasp_tasks
```

消息类型：

```text
mos_grasp_selector/msg/GraspTaskArray
```

调试状态：

```text
/vision/grasp_tasks_json
```

运行时控制：

```text
/mos_grasp_selector/set_enabled   # std_srvs/srv/SetBool
/mos_grasp_selector/reset         # std_srvs/srv/Trigger
```

默认 `enabled=false` 时不处理检测结果，也不发布抓取任务。导航完成后先调用 `reset`，再调用 `set_enabled true`。

## 当前任务规则

自动治疗车把手任务：

```text
active_task = auto_treatment_cart_push
target_class_ids = [6, 7, 8]
required_points = 2
```

节点会独立检查三个类别：

```text
6: cart_handle_A
7: cart_handle_B
8: cart_handle_C
```

每个类别都必须单独检测到两个有效 3D 把手点。`target_class_ids` 是允许
使用的类别集合，不是类别优先级。只要 6、7 或 8 中任意一个类别形成有效
把手对，就可以生成任务。如果同一帧有多个类别都形成把手对，默认按
`selection.mode=nearest` 选择距离最近的一对；使用 `highest_score` 时选择
总置信度最高的一对。

最终只发布一个 `GraspTask`，并将实际选中的类别写入
`GraspTask.target_class_id` 和两个 `GraspPoint.class_id`。不会把不同类别
的把手配成一对。稳定帧收集期间如果类别发生变化，会清空历史并重新收集，
避免跨类别融合。

治疗车 A/B/C 把手双臂同步抓取：

```text
require_trigger = false     # 默认不要求检测治疗车本体
trigger_class_ids = [0, 1, 2, 3, 4, 5]  # A/B/C 空车和载物车
target_class_id = 6         # 默认 A 型车把手
required_points = 2
grasp_mode = dual_arm_sync
```

单独任务 `treatment_cart_A_push`：只要检测到不少于 2 个
`target_class_id=6` 的有效 A 型把手，就进入稳定滤波并生成抓取任务。
B/C 型分别使用任务 `treatment_cart_B_push`/`treatment_cart_C_push`，目标
类别为 7/8；垃圾车顶部横杆使用类别 18。

## 有效性验证

节点只使用满足以下条件的检测目标：

```text
class_id 匹配任务规则
score >= 对应置信度阈值
has_target_position = true
position_target.header.frame_id = target_frame
position_target.point.x/y/z 都是有限数值
```

## 选择和稳定

第一版选择方法：

```text
1. 默认直接筛选当前任务的把手类别 6/7/8/18；require_trigger=true 时才要求先检测到 class_id=0~5 的治疗车。
2. 按 body_link 原点距离排序。
3. 选择最近两个把手。
4. y 较大分配为 left_handle，y 较小分配为 right_handle。
5. 缓存最近 stability_window 帧。
6. 至少 min_stable_frames 帧后，以中位数点为中心剔除离群点。
7. 对剩余点计算左右把手 xyz 标准差和极差。
8. 最大标准差 <= max_position_std_m 且最大极差 <= max_position_range_m 时，使用中位数 x/y/z 发布抓取任务。
9. publish_once_when_ready=true 时，稳定任务只发布一次，目标丢失后才重新允许发布。
```

推荐参数：

```yaml
publish_once_when_ready: true
position_filter: "median"
outlier_distance_m: 0.08
max_position_std_m: 0.03
max_position_range_m: 0.06
```

## 对接语义

机械臂控制方只需要订阅 `/vision/grasp_tasks`。

当 `tasks` 为空时，表示当前没有可执行抓取任务。

当 `tasks[0].ready = true` 且 `grasp_mode = dual_arm_sync` 时：

```text
left_handle  给左手
right_handle 给右手
point.header.frame_id 是坐标系
point.point.x/y/z 单位是 m
```

## 部署触发流程

```text
1. 导航结束。
2. 调用 /mos_grasp_selector/reset。
3. 调用 /mos_3d_object/set_enabled true。
4. 调用 /mos_grasp_selector/set_enabled true。
5. 等待 /vision/grasp_tasks 发布一次 ready=true 任务。
6. 机械臂控制方锁定任务坐标并执行。
7. 调用 /mos_grasp_selector/set_enabled false。
8. 调用 /mos_3d_object/set_enabled false。
```

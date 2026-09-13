# 第四步实现总结 - 把手检测、判断距离与抓取

## ✅ 完成时间
2026-07-17

## 📋 实现内容

### 目标
在POSITIONING或ALIGNING状态后，调用vision判断把手距离，距离合适/对准后直接抓取，抓取失败时自动重试。

### 新增状态

```python
class TaskState(Enum):
    GRASPING = "grasping"    # 抓取中
```

### 核心实现

#### 1. 把手检测回调处理
**文件**: `coordinator_node.py`

```python
def on_handle_positions_received(self, left_handle, right_handle, task_name='grasp'):
    """接收把手位置信息：判断是否需要对准，然后抓取"""
    if self.state != TaskState.POSITIONING:
        return
    
    # 保存把手位置（用于重试）
    self.last_handle_left = left_handle
    self.last_handle_right = right_handle
    
    # 判断把手距离
    handle_distance = left_handle.point.x
    
    if handle_distance > self.alignment_threshold:  # 0.85m
        # 路线A：把手太远，需要底盘对准
        self.transition_to(TaskState.ALIGNING)
        self.navigation.request_alignment(target_x=handle_distance)
    else:
        # 路线B：把手距离合适，直接抓取
        self.execute_grasp(left_handle, right_handle)
```

#### 2. 抓取执行
```python
def execute_grasp(self, left_handle, right_handle):
    """执行抓取操作"""
    self.transition_to(TaskState.GRASPING)
    self.arm.grasp_at_point(left_handle, right_handle)
```

#### 3. 底盘对准完成处理
```python
def on_alignment_complete(self):
    """底盘对准完成事件"""
    if self.state == TaskState.ALIGNING:
        # 对准完成，重新检测把手位置
        time.sleep(0.8)  # 等待系统稳定
        
        self.vision.reset_selector()
        self.vision.set_vision_enabled(True)
        self.vision.set_selector_enabled(True)
        
        self.transition_to(TaskState.POSITIONING)
        self.vision.request_handle_detection()
```

#### 4. 抓取结果处理与重试
```python
def on_grasp_result(self, success: bool):
    """接收机械臂抓取结果"""
    if self.state != TaskState.GRASPING:
        return
    
    if success:
        # 抓取成功
        self.grasp_retry_count = 0
        self.is_holding_cart = True
        # TODO: 下一步实现运输逻辑
    else:
        # 抓取失败，重试
        self.grasp_retry_count += 1
        
        if self.grasp_retry_count < self.max_grasp_retries:
            # 使用上次把手位置重试
            if self.last_handle_left and self.last_handle_right:
                self.execute_grasp(self.last_handle_left, self.last_handle_right)
            else:
                # 重新检测把手
                self.transition_to(TaskState.POSITIONING)
                self.vision.request_handle_detection()
        else:
            # 超过重试次数
            self.transition_to(TaskState.ERROR, "抓取失败次数过多")
```

## 🎯 决策逻辑

### 路线判断流程

```
POSITIONING状态
  ↓
收到把手位置 (on_handle_positions_received)
  ↓
计算把手距离 (handle_distance = left_handle.point.x)
  ↓
        ┌─────────────────────┐
        │ 距离判断            │
        └─────────────────────┘
         /                    \
        /                      \
距离 > 0.85m                距离 ≤ 0.85m
    ↓                           ↓
【路线A：需要对准】          【路线B：直接抓取】
    ↓                           ↓
转ALIGNING状态               转GRASPING状态
    ↓                           ↓
request_alignment()          execute_grasp()
    ↓                           ↓
等待对准完成                  等待抓取结果
    ↓                           ↓
on_alignment_complete()      on_grasp_result()
    ↓                           ↓
重新检测把手                  成功/失败处理
    ↓
回到POSITIONING
```

### 重试机制

```
抓取失败
  ↓
grasp_retry_count++
  ↓
        ┌─────────────────────┐
        │ 重试次数判断         │
        └─────────────────────┘
         /                    \
        /                      \
  < 3次                      ≥ 3次
    ↓                           ↓
重试抓取                     转ERROR状态
    ├─ 有保存位置 → 直接重试
    └─ 无保存位置 → 重新检测把手
```

## 📊 关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `alignment_threshold` | 0.85m | 对准阈值，超过需要对准 |
| `max_grasp_retries` | 3次 | 最大抓取重试次数 |
| `stabilize_delay` | 0.8s | 对准后系统稳定等待时间 |

## 🧪 测试脚本

```
test/scripts/
└── test_step4_grasp.sh    # 第四步完整测试
```

## 📝 实现特点

### ✅ 智能决策
1. **距离判断** - 自动判断是否需要对准
2. **路径选择** - 根据距离选择最优路径
3. **自动重试** - 失败时智能重试
4. **位置缓存** - 保存把手位置用于快速重试

### ✅ 鲁棒性
1. **状态检查** - 每个回调都检查当前状态
2. **视觉控制** - 适时开启/关闭视觉系统
3. **错误处理** - 超过重试次数转ERROR状态
4. **系统稳定** - 对准后等待系统稳定

### ✅ 可配置性
1. **对准阈值** - 可调整alignment_threshold
2. **重试次数** - 可配置max_grasp_retries
3. **稳定时间** - 可调整stabilize_delay

## 🔍 数据流

```
【完整第四步流程】

APPROACHING → 到达小车
  ↓
POSITIONING
  ↓
vision.request_handle_detection()
  ↓
/vision/grasp_tasks 发布把手位置
  ↓
on_handle_positions_received()
  ├─ 保存: last_handle_left, last_handle_right
  └─ 判断距离
      ├─ > 0.85m → ALIGNING
      │      ↓
      │   request_alignment()
      │      ↓
      │   on_alignment_complete()
      │      ↓
      │   重新检测把手 → 回到POSITIONING
      │
      └─ ≤ 0.85m → GRASPING
             ↓
          execute_grasp()
             ↓
          arm.grasp_at_point()
             ↓
          on_grasp_result(success)
             ├─ True → 抓取成功 ✅
             │    └─ is_holding_cart = True
             │
             └─ False → 重试机制
                  ├─ < 3次 → 重试
                  └─ ≥ 3次 → ERROR
```

## 🎯 验证要点

### ✅ 功能实现
1. ✅ **GRASPING状态** - 已添加
2. ✅ **把手检测回调** - on_handle_positions_received实现
3. ✅ **距离判断** - 基于alignment_threshold
4. ✅ **路线分流** - 对准路线和直接抓取路线
5. ✅ **对准处理** - on_alignment_complete实现
6. ✅ **抓取执行** - execute_grasp实现
7. ✅ **结果处理** - on_grasp_result实现
8. ✅ **重试机制** - 最多3次重试

### ✅ 状态转换
```
POSITIONING → (距离>0.85m) → ALIGNING → POSITIONING → GRASPING
POSITIONING → (距离≤0.85m) → GRASPING
GRASPING → (失败) → GRASPING (重试)
GRASPING → (成功) → [下一步]
```

## ⚠️ 当前限制

### 测试环境
1. **模拟把手检测** - 使用dummy_grasp_selector_node
2. **无真实机械臂** - 无法验证实际抓取
3. **无真实底盘对准** - chassis服务不可用

### 待完善
1. **把手检测超时** - 需要添加超时保护
2. **对准失败处理** - request_alignment失败时的处理
3. **真实硬件集成** - 需要连接实际的机械臂和底盘

## 🔄 下一步计划

### 第五步：运输到B点
- [ ] 抓取成功后导航到B点
- [ ] 寻找空位放置空车
- [ ] 放置小车
- [ ] 转换到找满阶段

---

**状态**: ✅ 第四步完成  
**实现**: ✅ 把手检测、距离判断、对准、抓取  
**重试**: ✅ 抓取失败自动重试（最多3次）  
**文档**: ✅ 完善  
**日期**: 2026-07-17

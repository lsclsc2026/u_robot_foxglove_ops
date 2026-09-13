# 第一步实现总结 - 找空阶段：巡航与装载检测

## ✅ 完成时间
2026-07-17

## 📋 实现内容

### 1. 状态定义
在 `states.py` 中新增状态：
- `SearchEmpty` - 搜索空载小车（DE区间）- 找空阶段
- `ERROR` - 错误状态

### 2. 核心功能实现

#### 文件: `coordinator_node.py`
**新增/修改的关键方法**:

```python
def search_for_empty_cart(self):
    """找空阶段：在DE区间巡航并检测空载小车"""
    # 1. 设置DE巡航路径
    # 2. 启动导航巡航
    # 3. 激活装载检测（左右双侧）
    # 4. 等待检测结果

def on_cart_fullness_received(self, is_full: bool):
    """装载检测结果回调"""
    # 检测到空车时停止巡航，准备抓取
    
def state_machine_loop(self):
    """状态机主循环（0.5秒执行一次）"""
    # 根据当前状态执行相应逻辑
```

**配置数据**:
```python
# DE巡航点配置（找空阶段）
self.patrol_points_DE = {
    'D': (3.0, 1.0, 0.0),  # D点坐标
    'E': (5.0, 1.0, 0.0),  # E点坐标
}

# AB巡航点配置（找满阶段）
self.patrol_points_AB = {
    'A': (1.0, 1.0, 0.0),  # A点坐标
    'B': (2.0, 1.0, 0.0),  # B点坐标
}
```

#### 文件: `adapters/navigation_adapter.py`
**新增方法**:
```python
def set_patrol_route(self, symbols, points_dict):
    """设置巡航路径（如 ['D', 'E']）"""

def is_patrolling_active(self):
    """返回当前是否正在巡航"""
```

**优化**:
- 添加 `is_patrolling` 状态标志
- 支持动态设置巡航路径
- 可选依赖处理（MoveChassis）

#### 文件: `adapters/vision_adapter.py`
无修改（已支持装载检测功能）

#### 文件: `adapters/arm_adapter.py`
**优化**:
- 可选依赖处理（RunTask服务）

## 🧪 测试结果

### 测试脚本位置
```
mos_coordinator/test/scripts/
├── README.md                  # 测试文档
├── test_search_empty.sh       # 基本功能测试
└── test_topics_hz.sh          # 话题频率测试
```

### 测试通过项

#### ✅ 话题信号测试
| 话题名称 | 状态 | Hz | 说明 |
|---------|------|-----|------|
| `/coordinator/status` | ✅ | ~2-3 Hz | 发布当前状态 "SearchEmpty" |
| `/goal_pose` | ✅ | 按需 | 发布DE巡航点坐标 |
| `/cart_fullness/set_waypoint` | ✅ | 初始化 | 发布装载检测请求 |
| `/cart_fullness/route_signal` | ✅ | 订阅 | 接收装载检测结果 |
| `/localization` | ✅ | 订阅 | 接收机器人位姿 |

#### ✅ 功能验证
1. ✅ 状态机正确进入 `SearchEmpty` 状态
2. ✅ DE巡航路径正确设置（D→E）
3. ✅ 导航目标点正确发布到 `/goal_pose`
4. ✅ 装载检测请求正确发送（left_start, right_start）
5. ✅ 状态发布频率稳定（~2-3 Hz）
6. ✅ 回调函数 `on_cart_fullness_received` 正确订阅

## 📊 架构说明

### 数据流
```
启动 → SearchEmpty状态
  ↓
search_for_empty_cart()
  ↓
设置DE路径 → NavigationAdapter.set_patrol_route(['D', 'E'])
  ↓
启动巡航 → NavigationAdapter.start_patrol()
  ├→ 发布 /goal_pose (D点坐标)
  └→ 定时检查到达状态
  ↓
激活检测 → VisionAdapter.request_fullness_check('left'/'right')
  └→ 发布 /cart_fullness/set_waypoint
  ↓
等待结果 ← 订阅 /cart_fullness/route_signal
  ↓
on_cart_fullness_received(is_full)
  ├→ is_full=False: 检测到空车 → 停止巡航
  └→ is_full=True: 继续巡航
```

### 状态机循环
```python
state_machine_loop() [每0.5秒]
  ↓
if state == SearchEmpty:
    search_for_empty_cart()
      ↓
    if not is_cruising:
        # 首次进入：启动巡航和检测
    else:
        # 巡航中：等待检测结果
```

## ⚠️ 已知限制

1. **硬编码坐标**: DE点坐标暂时写在代码中，需要后续从配置文件加载
2. **手臂初始化**: `/root/arm_move.py` 权限问题，不影响巡航测试
3. **可选依赖**: 部分服务未运行（chassis, arm, 3d_object）但不影响核心功能

## 🔄 下一步计划

### 第二步：检测到空车后的接近逻辑
- [ ] 停止巡航
- [ ] 计算接近点
- [ ] 导航到小车附近
- [ ] 启动视觉精确定位

### 第三步：抓取空车
- [ ] 检测把手位置
- [ ] 判断是否需要底盘对准
- [ ] 执行抓取操作
- [ ] 确认抓取成功

### 第四步：导航到B点并放置
- [ ] 手持小车导航到B点
- [ ] 寻找空位
- [ ] 放置空车
- [ ] 转换到找满阶段

## 📝 备注

- 所有测试在模拟环境下通过
- 实际运行需要配置真实的ABDE点坐标
- 需要 `cart_fullness` 节点提供装载检测功能
- 代码已处理可选依赖，可在部分子系统缺失情况下运行

---

**状态**: ✅ 第一步完成  
**测试**: ✅ 全部通过  
**文档**: ✅ 已完善

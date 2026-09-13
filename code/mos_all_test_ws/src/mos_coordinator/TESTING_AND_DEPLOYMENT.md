# 完整流程测试与真机适配指南

## 📚 文档索引

1. **[完整流程测试脚本](test/scripts/test_full_process.sh)** - 一键测试完整找空阶段
2. **[真机适配指南](REAL_MACHINE_GUIDE.md)** - 从模拟到真机的完整指南

---

## 🧪 如何进行完整流程测试

### 快速开始

```bash
cd /home/jeff/mos
source install/setup.bash

# 运行完整流程测试（60秒自动测试）
./mos_coordinator/test/scripts/test_full_process.sh
```

### 测试内容

完整流程测试会自动验证以下6个步骤：

1. ✅ **SearchEmpty** - 巡航DE区间并检测空车
2. ✅ **APPROACHING** - 接近空车
3. ✅ **POSITIONING** - 检测把手位置
4. ✅ **GRASPING** - 抓取空车
5. ✅ **TRANSPORTING** - 运输到B点
6. ✅ **RELEASING** - 释放小车

### 测试通过标准

#### ✅ 状态转换完整
```
idle → SearchEmpty → APPROACHING → POSITIONING → 
GRASPING → TRANSPORTING → RELEASING → SearchFull ✅
```

#### ✅ 最终状态正确
```bash
# 测试结束后检查
ros2 topic echo /coordinator/status --once

# 应该显示：
data: SearchFull
```

#### ✅ 无错误日志
```bash
# 检查日志
grep "ERROR\|❌" /tmp/full_test_coordinator.log

# 应该没有严重错误
```

### 查看测试结果

```bash
# 查看完整日志
cat /tmp/full_test_coordinator.log

# 查看状态转换历史
grep "状态转换:" /tmp/full_test_coordinator.log

# 查看关键事件
grep -E "(检测到|成功|到达)" /tmp/full_test_coordinator.log
```

---

## 🔧 如何适配真机

### 第一步：确认硬件准备

#### 必需的硬件系统

| 系统 | 节点名称 | 功能 | 检查命令 |
|------|---------|------|---------|
| 导航系统 | lumos_nav | 机器人导航 | `ros2 node list \| grep lumos` |
| 装载检测 | cart_fullness_node | 检测小车满/空 | `ros2 node list \| grep cart_fullness` |
| 把手检测 | grasp_selector_node | 检测把手位置 | `ros2 node list \| grep grasp` |
| 机械臂控制 | arm_controller_node | 抓取/释放 | `ros2 service list \| grep arm` |
| 底盘对准 | chassis_controller_node | 精确对准 | `ros2 service list \| grep chassis` |

#### 检查所有系统是否运行

```bash
# 一键检查脚本
#!/bin/bash
echo "检查硬件系统..."

# 检查导航
ros2 node list | grep -q lumos && echo "✅ 导航系统" || echo "❌ 导航系统"

# 检查装载检测
ros2 node list | grep -q cart_fullness && echo "✅ 装载检测" || echo "❌ 装载检测"

# 检查把手检测
ros2 node list | grep -q grasp && echo "✅ 把手检测" || echo "❌ 把手检测"

# 检查机械臂
ros2 service list | grep -q arm && echo "✅ 机械臂控制" || echo "❌ 机械臂控制"

# 检查底盘
ros2 service list | grep -q chassis && echo "✅ 底盘对准" || echo "❌ 底盘对准"
```

---

### 第二步：配置真实坐标

#### 2.1 测量实际坐标点

在真实环境中测量以下点的坐标（在map坐标系下）：

- **A点** - AB区间起点（找满阶段）
- **B点** - AB区间终点，空车放置点
- **D点** - DE区间起点（找空阶段）
- **E点** - DE区间终点

#### 2.2 修改配置

**方法1：直接修改代码**

编辑 `coordinator_node.py`，找到坐标配置：

```python
# 修改为实际坐标
self.patrol_points_AB = {
    'A': self._create_pose(1.0, 1.0, 0.0),  # ← 修改这里
    'B': self._create_pose(2.0, 1.0, 0.0),  # ← 修改这里
}

self.patrol_points_DE = {
    'D': self._create_pose(3.0, 1.0, 0.0),  # ← 修改这里
    'E': self._create_pose(5.0, 1.0, 0.0),  # ← 修改这里
}
```

**方法2：使用配置文件（推荐）**

创建 `config/waypoints.yaml`：

```yaml
waypoints:
  A: {x: 1.5, y: 2.0, yaw: 0.0}
  B: {x: 3.5, y: 2.0, yaw: 0.0}
  D: {x: 5.0, y: 1.5, yaw: 0.0}
  E: {x: 7.0, y: 1.5, yaw: 0.0}
```

---

### 第三步：调整关键参数

#### 3.1 对准阈值

```python
# 根据机械臂抓取范围调整
self.alignment_threshold = 0.85  # 默认值

# 如果机械臂抓取范围大（推荐）
self.alignment_threshold = 1.0   # 1米内可直接抓取

# 如果需要更精确（不推荐，会增加对准次数）
self.alignment_threshold = 0.6   # 0.6米内才直接抓取
```

#### 3.2 重试次数

```python
# 根据实际可靠性调整
self.max_grasp_retries = 3      # 抓取失败重试次数
self.max_release_retries = 3    # 释放失败重试次数
self.max_approach_retries = 3   # 接近失败重试次数
```

---

### 第四步：停用模拟节点

#### ❌ 不要启动这些模拟节点

```bash
# 模拟测试时使用，真机时不要启动
# ros2 run mos_coordinator dummy_cart_fullness_node  ← 不要启动
# ros2 run mos_coordinator dummy_grasp_selector_node ← 不要启动
```

#### ✅ 启动真实节点

```bash
# 启动真实的装载检测
ros2 run mos_cart_fullness cart_fullness_node

# 启动真实的把手检测
ros2 run mos_grasp_selector grasp_selector_node
```

---

### 第五步：逐步验证

#### 5.1 静态验证（不移动）

```bash
# 只启动coordinator，检查初始化
ros2 run mos_coordinator coordinator_node

# 查看是否所有服务都连接成功
# 应该看到：
# [INFO] NavigationAdapter initialized ✅
# [INFO] VisionAdapter initialized ✅
# [INFO] ArmAdapter initialized ✅
# [WARN] MoveChassis 服务不可用  ← 这个应该消失
```

#### 5.2 单步测试

**测试1：巡航** ✅
```bash
# 启动后观察机器人是否移动到D点
# 检查状态
ros2 topic echo /coordinator/status
# 应该显示: SearchEmpty
```

**测试2：装载检测** ✅
```bash
# 将小车放在相机视野内
# 观察是否检测到空车
ros2 topic echo /cart_fullness/route_signal
```

**测试3：接近** ✅
```bash
# 检测到空车后，观察是否接近小车
ros2 topic echo /goal_pose
```

**测试4：把手检测** ✅
```bash
# 到达小车后，观察是否检测到把手
ros2 topic echo /vision/grasp_tasks
```

**测试5：抓取** ✅
```bash
# 观察机械臂是否执行抓取
# 检查状态应变为: grasping
```

**测试6：运输** ✅
```bash
# 观察机器人是否移动到B点
# 状态应为: transporting
```

**测试7：释放** ✅
```bash
# 到达B点后观察是否释放
# 最终状态应为: SearchFull
```

---

## 📊 真机测试检查清单

### 启动前 ☑️

- [ ] 所有硬件系统已上电并初始化
- [ ] 所有ROS节点已启动
- [ ] 坐标点配置已修改为真实值
- [ ] 参数已根据实际情况调整
- [ ] 模拟节点已停用
- [ ] 安全区域已确认

### 运行中 ☑️

- [ ] 状态转换正常
- [ ] 机器人移动正确
- [ ] 检测结果准确
- [ ] 机械臂动作正确
- [ ] 无碰撞
- [ ] 无卡顿

### 完成后 ☑️

- [ ] 空车成功放置到B点
- [ ] 最终状态为SearchFull
- [ ] 无错误日志
- [ ] 系统可以循环执行

---

## 🔍 常见问题排查

### Q1: 导航不响应怎么办？

**症状**: 发布/goal_pose后机器人不移动

**解决**:
```bash
# 1. 检查lumos_nav是否运行
ros2 node list | grep lumos

# 2. 手动测试导航
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0}, orientation: {w: 1.0}}}"

# 3. 检查坐标系是否正确
# frame_id应该是'map'
```

### Q2: 装载检测不准确怎么办？

**症状**: 检测结果不稳定或错误

**解决**:
- 调整相机角度和高度
- 改善光照条件
- 检查检测模型是否正确加载
- 调整检测阈值

### Q3: 把手检测失败怎么办？

**症状**: POSITIONING状态下长时间等待

**解决**:
- 确认小车在相机视野内
- 检查把手是否被遮挡
- 调整机器人与小车的相对位置
- 检查grasp_selector_node日志

### Q4: 抓取失败怎么办？

**症状**: 机械臂无法抓住小车

**解决**:
- 检查把手位置是否准确
- 调整alignment_threshold增大对准频率
- 检查机械臂标定是否正确
- 增加抓取重试次数

---

## 🚀 真机快速启动模板

创建 `launch_real_robot.sh`:

```bash
#!/bin/bash
echo "启动真实机器人系统..."

# 1. 启动导航
ros2 launch lumos_nav navigation.launch.py &
sleep 3

# 2. 启动装载检测
ros2 run mos_cart_fullness cart_fullness_node &
sleep 2

# 3. 启动把手检测
ros2 run mos_grasp_selector grasp_selector_node &
sleep 2

# 4. 启动机械臂控制
ros2 run mos_arm_controller arm_controller_node &
sleep 2

# 5. 启动底盘控制
ros2 run mos_chassis_controller chassis_controller_node &
sleep 2

echo "所有子系统已启动，等待5秒后启动协调器..."
sleep 5

# 6. 启动协调器（找空阶段）
ros2 run mos_coordinator coordinator_node
```

使用方法：
```bash
chmod +x launch_real_robot.sh
./launch_real_robot.sh
```

---

## 📋 总结

### 模拟测试 → 真机的关键步骤

1. ✅ **运行完整流程测试** - `test_full_process.sh`
2. ✅ **确认硬件系统运行** - 所有节点和服务可用
3. ✅ **修改真实坐标** - ABDE点坐标
4. ✅ **调整参数** - 对准阈值、重试次数
5. ✅ **停用模拟节点** - 使用真实硬件
6. ✅ **逐步验证** - 从静态到动态，从单步到完整流程

### 相关文档

- **[完整流程测试脚本](test/scripts/test_full_process.sh)**
- **[真机适配详细指南](REAL_MACHINE_GUIDE.md)**
- **[测试文档索引](test/scripts/INDEX.md)**

---

**最后更新**: 2026-07-17  
**版本**: v1.0
